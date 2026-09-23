// SPDX-License-Identifier: GPL-3.0-only
// Copyright (c) 2026 NearMidnightNow (NMN).

#include "PCH.h"
#include "Profiler.h"

#include "ObjectStamps.h"
#include "ObjectStampFilter.h"

#include "ActorShapes.h"
#include "ContactSampler.h"
#include "Globals.h"
#include "HeatSources.h"
#include "Settings.h"
#include "SnowSurface.h"
#include "SurfaceProfiles.h"
#include "SurfaceTypes.h"
#include "Weather.h"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

namespace ObjectStamps
{
	namespace
	{
		std::mutex g_lock;

		struct Candidate
		{
			RE::ObjectRefHandle handle;

			bool heat{ false };
		};

		std::vector<Candidate> g_candidates;
		float                  g_timer{ 0.0f };
		bool                   g_haveSet{ false };
		uint64_t               g_frame{ 0 };

		constexpr size_t kMaxCandidates = 256;

		constexpr float kMaxMotion = 48.0f;

		// A projectile's contact, as reported by the engine through the hit
		// hook.  Keyed by the projectile reference and dropped after a second,
		// so a projectile that landed and was recycled cannot hand its landing
		// point to whatever reference reuses its id next.
		struct ContactRecord
		{
			RE::FormID     form{};
			RE::NiPoint3   position{};
			std::chrono::steady_clock::time_point time{};
			bool           live{ false };
		};

		std::array<ContactRecord, 64> g_contacts{};
		size_t                        g_nextContact{};

		// The engine's "first thing along this ray", wired to the same call
		// the engine's own projectiles use.  Used from the main thread only,
		// under the same lock that guards the candidate list.
		//
		// The output carries no hit point - only a normal and a fraction along
		// the ray - so the point is interpolated, which is exactly what the
		// shelter ray already does (Shelter.cpp:113).  Both the from/to and
		// the answer are in world units: the physics world's own scale is
		// applied *inside* Pick, and the fraction is unitless either way.
		// True when a ray that hit this collidable has hit something that is
		// not the world: a projectile, an actor, or a piece of gear in flight.
		// A shaft's own bound sits around its origin, so a ray aimed down from
		// the origin hits the arrow first every time.  That is exactly what
		// made the landing point sit slightly *above* the origin with a miss
		// of zero: the ray never reached the ground at all.
		bool IsNotGround(const RE::hkpCollidable* a_collidable)
		{
			if (!a_collidable) {
				return false;
			}

			switch (a_collidable->GetCollisionLayer()) {
			case RE::COL_LAYER::kProjectile:
			case RE::COL_LAYER::kProjectileZone:
			case RE::COL_LAYER::kConeProjectile:
			case RE::COL_LAYER::kSpell:
			case RE::COL_LAYER::kSpellExplosion:
			case RE::COL_LAYER::kBiped:
			case RE::COL_LAYER::kBipedNoCC:
			case RE::COL_LAYER::kCharController:
			case RE::COL_LAYER::kDeadBip:
			case RE::COL_LAYER::kWeapon:
			case RE::COL_LAYER::kClutter:
			case RE::COL_LAYER::kProps:
				return true;
			default:
				return false;
			}
		}

		bool PickRay(float a_x, float a_y, float a_z, float a_toX, float a_toY, float a_toZ,
			float& a_outX, float& a_outY, float& a_outZ)
		{
			auto* tes = RE::TES::GetSingleton();
			if (!tes) {
				return false;
			}

			const float scale = RE::bhkWorld::GetWorldScale();

			// Walking the ray forward past anything that is not the world.
			// A single pick would otherwise be spent on the arrow's own bound
			// and the caller would be told the shaft landed on itself.
			float fromX = a_x;
			float fromY = a_y;
			float fromZ = a_z;

			for (int attempt = 0; attempt < 4; ++attempt) {
				RE::bhkPickData pick;
				pick.rayInput.from =
					RE::hkVector4(fromX * scale, fromY * scale, fromZ * scale, 0.0f);
				pick.rayInput.to =
					RE::hkVector4(a_toX * scale, a_toY * scale, a_toZ * scale, 0.0f);
				// Static and terrain collision only.  The projectile's own
				// layer is not in this set, so the arrow cannot answer its
				// own question, and neither can an actor standing over it.
				pick.rayInput.filterInfo.SetCollisionLayer(RE::COL_LAYER::kGround);

				tes->Pick(pick);
				if (!pick.rayOutput.HasHit()) {
					return false;
				}

				const float t = std::clamp(pick.rayOutput.hitFraction, 0.0f, 1.0f);

				// What the ray actually answered with.  A mark in the wrong
				// place is decided here, and the layer is the one number that
				// says whether the ground answered or the shaft's own bound
				// did.  Cheap, and it beats inferring from the result.
				if (Settings::logContactSamples) {
					const auto* hit = pick.rayOutput.rootCollidable;
					logger::info(
						"Probe attempt {}: layer={} t={:.4f} from=({:.1f},{:.1f},{:.1f}) "
						"to=({:.1f},{:.1f},{:.1f})",
						attempt, hit ? static_cast<int>(hit->GetCollisionLayer()) : -1, t, fromX,
						fromY, fromZ, a_toX, a_toY, a_toZ);
				}

				if (!IsNotGround(pick.rayOutput.rootCollidable)) {
					a_outX = a_x + (a_toX - a_x) * t;
					a_outY = a_y + (a_toY - a_y) * t;
					a_outZ = a_z + (a_toZ - a_z) * t;
					return true;
				}

				// Something in flight answered.  Resume from just past it so
				// the next attempt can find the ground behind it, and give up
				// rather than loop if the ray has no room left.
				const float stepX = fromX + (a_toX - fromX) * t;
				const float stepY = fromY + (a_toY - fromY) * t;
				const float stepZ = fromZ + (a_toZ - fromZ) * t;
				const float pushX = (a_toX - fromX) * 0.01f;
				const float pushY = (a_toY - fromY) * 0.01f;
				const float pushZ = (a_toZ - fromZ) * 0.01f;

				fromX = stepX + pushX;
				fromY = stepY + pushY;
				fromZ = stepZ + pushZ;

				// Past the destination: nothing left to search.
				if ((a_toX - fromX) * (a_toX - a_x) < 0.0f ||
					(a_toY - fromY) * (a_toY - a_y) < 0.0f ||
					(a_toZ - fromZ) * (a_toZ - a_z) < 0.0f) {
					return false;
				}
			}

			return false;
		}

		ContactSampler::Inputs PickInputs()
		{
			return ContactSampler::Inputs{ &PickRay };
		}

		struct float2
		{
			float x{ 0.0f }, y{ 0.0f };
		};

		struct Tracked
		{
			float    x{ 0.0f }, y{ 0.0f };
			uint64_t frame{ 0 };

			// Where this object was when it was last given a stamp, and
			// whether it has had one at all.
			//
			// A spent arrow keeps its projectile component and stays in the
			// candidate set for as long as it lies there, so it was re-stamped
			// every objectStampInterval - 0.20 s, five times a second -
			// indefinitely.  The log showed three arrows stamped 13, 13 and 14
			// times each at identical coordinates, 19 ms apart: not three
			// holes, one hole pressed into itself until the rims of the run
			// read as a trench.  The field was there to be "a hole an arrow
			// made" and it was the one thing the repeated stamping could not
			// be.
			//
			// A mark is laid once at a place.  It is laid again only when the
			// object has moved far enough that the new place is a new place -
			// a second arrow, or a projectile still in flight - so a shaft
			// that has come to rest stops marking instead of grinding.
			float    markX{ 0.0f }, markY{ 0.0f };
			bool     marked{ false };
		};

		// How far an object must move from where it was last marked before it
		// earns another.  A still arrow drifts by fractions of a unit as the
		// physics settles and must not count; an arrow still travelling moves
		// tens of units per interval and must.  8 units is well clear of the
		// settling and well under a flight step, and it is deliberately larger
		// than the 4-unit pin-prick radius so a re-mark cannot land on top of
		// the mark it is replacing.
		constexpr float kRemarkDistance = 8.0f;

		std::unordered_map<RE::FormID, Tracked> g_motion;
		std::unordered_set<RE::FormID> g_visualBloodReported;

		// Newest first, so a reference that somehow collected two records
		// resolves to the one it most recently earned.
		const ContactRecord* FindContact(RE::FormID a_form)
		{
			for (size_t i = 0; i < g_contacts.size(); ++i) {
				const auto& record = g_contacts[(g_nextContact + g_contacts.size() - 1 - i) %
					g_contacts.size()];
				if (record.live && record.form == a_form) {
					return &record;
				}
			}
			return nullptr;
		}

		bool LeavesAMark(RE::TESObjectREFR* a_ref)
		{

			if (auto* projectile = a_ref->As<RE::Projectile>()) {
				const auto* base = projectile->GetProjectileBase();
				const char* model = base ? base->GetModel() : nullptr;
				if (model && ObjectStampFilter::IsVisualBloodProjectile(model)) {
					if (g_visualBloodReported.size() < 16 && g_visualBloodReported.insert(base->GetFormID()).second) {
						logger::info("Object stamps B5: visual blood projectile excluded base={:08X} model={}",
							base->GetFormID(), model);
					}
					return false;
				}
				return true;
			}

			const auto* base = a_ref->GetBaseObject();
			if (!base) {
				return false;
			}

			switch (base->GetFormType()) {
			case RE::FormType::Weapon:
			case RE::FormType::Armor:
			case RE::FormType::Ammo:
			case RE::FormType::Misc:
			case RE::FormType::Ingredient:
			case RE::FormType::AlchemyItem:
			case RE::FormType::Book:
			case RE::FormType::Scroll:
			case RE::FormType::SoulGem:
			case RE::FormType::KeyMaster:
			case RE::FormType::Apparatus:
			case RE::FormType::Light:
				return true;
			default:
				return false;
			}
		}

		// A spent arrow sticks in the ground and goes on registering as a
		// projectile.  Its collision bound, though, spans the whole shaft, so
		// the generic object path below presses a crater as long as the arrow
		// and rings it with a rim - which reads as heaps of snow thrown up all
		// around the player, not as a hole an arrow made.  A shaft gets one
		// pin-prick instead: fixed small radius, shallow, and no rim.
		//
		// Every projectile counts, not only ammo: a spent projectile's origin
		// is not its tip whatever it was fired from, which is the reason the
		// mark and the object disagreed in the first place.
		bool IsShaft(RE::TESObjectREFR* a_ref)
		{
			if (a_ref->As<RE::Projectile>()) {
				return true;
			}
			const auto* base = a_ref->GetBaseObject();
			return base && base->GetFormType() == RE::FormType::Ammo;
		}

		std::unordered_set<RE::FormID> g_reported;
		// Lines already written this session; reset with the rest of the state.
		size_t g_contactLines{ 0 };
		// Lines reporting a shaft that was held back; same treatment.
		size_t g_heldLines{ 0 };

		void LogObject(RE::TESObjectREFR* a_ref, Surfaces::Type a_surface,
			const Clipmap::Stamp& a_stamp, float a_drop)
		{
			if (!Settings::logObjectStamps || g_reported.size() >= 24) {
				return;
			}

			const auto* base = a_ref->GetBaseObject();
			if (!base || !g_reported.insert(base->GetFormID()).second) {
				return;
			}

			logger::info(
				"Object stamp: {} on {:<7} radius={:.1f} depth={:.1f} drop={:.1f}",
				a_ref->GetName(), Surfaces::Name(a_surface), a_stamp.radius, a_stamp.depth,
				a_drop);
		}

		// A shaft that was asked for a mark and denied one, because it already
		// has one where it still is.
		//
		// Bounded by its own count rather than by object type: the whole
		// question this line answers is how many times the same arrow came
		// back, so deduplicating by object would hide exactly the number it
		// exists to report.  Before the gate, three arrows produced 41 contact
		// lines; after it, the same three should produce a small handful of
		// these and no second marks.
		void LogShaftHeld(RE::TESObjectREFR* a_ref, float a_markX, float a_markY)
		{
			constexpr size_t kMaxHeldLines = 40;

			if (!Settings::logObjectStamps || g_heldLines >= kMaxHeldLines) {
				return;
			}
			++g_heldLines;

			const auto position = a_ref->GetPosition();
			logger::info(
				"Shaft held: {} already marked at ({:.1f}, {:.1f}), now at ({:.1f}, {:.1f}), "
				"moved {:.2f} units - no second mark",
				a_ref->GetName(), a_markX, a_markY, position.x, position.y,
				std::hypot(position.x - a_markX, position.y - a_markY));
		}

		// One line per shaft, carrying the fields a wrong answer shows up in:
		// which answer was used, where it landed relative to the object, how
		// far the object's origin was from the ground, and whether the mark
		// was kept at all.  A mark that is clearly on the snow and clearly
		// not where the arrow is has to be visible from the log alone.
		void LogContact(RE::TESObjectREFR* a_ref, const ContactSampler::Output& a_contact,
			const ContactSampler::Query& a_query, float a_miss, bool a_kept)
		{
			// Bounded by a plain count, not by arrow type.  Deduplicating on
			// the base form meant only the first arrow of each kind was ever
			// reported, so firing five arrows produced one line and looked
			// exactly like four arrows never being sampled at all.  A cap is
			// needed; a cap per type is a cap that hides the evidence.
			constexpr size_t kMaxContactLines = 40;

			if (!Settings::logContactSamples || g_contactLines >= kMaxContactLines) {
				return;
			}
			++g_contactLines;

			const float above = a_query.hasReference ?
				a_query.referenceZ - a_contact.z : 0.0f;

			logger::info(
				"Contact {}: {} origin=({:.1f},{:.1f},{:.1f}) landed=({:.1f},{:.1f},{:.1f}) "
				"lane={} above={:.1f} miss={:.1f} len={:.1f} kept={}",
				ContactSampler::SourceName(a_query.source), a_ref->GetName(),
				a_query.referenceX, a_query.referenceY, a_query.referenceZ,
				a_contact.x, a_contact.y, a_contact.z,
				ContactSampler::OriginName(a_contact.origin), above, a_miss,
				a_query.length, a_kept);
		}

		constexpr size_t kMaxShapesPerObject = 3;

		struct ShapeBound
		{
			RE::NiPoint3 centre;
			float        radius{ 0.0f };
			// The long and short half axes, kept from the shape itself.  A
			// bound radius alone cannot say "long and thin", and an arrow is
			// the one object where that distinction is the whole point.
			float        length{ 0.0f };
			float        thickness{ 0.0f };
			// Which way the long axis points, in world space.  Without it the
			// only ray a shaft can be probed with is a vertical one, which
			// lands under the shaft's own origin by construction - the very
			// error these axes were added to expose.
			RE::NiPoint3 axis{ 0.0f, 0.0f, 1.0f };
			bool         hasAxis{ false };
			// Diagnostic mirror of the same two fields on the extent, so the
			// log can say which route measured the axes.
			bool         fromChildren{ false };
			int          childCount{ 0 };
			// The raw projections, un-scaled, straight from the engine.  A
			// zero length beside a non-zero radius cannot be explained from
			// the scaled numbers alone.
			bool         measured{ false };
			float        rawPX{ 0.0f }, rawMX{ 0.0f };
			float        rawPY{ 0.0f }, rawMY{ 0.0f };
			float        rawPZ{ 0.0f }, rawMZ{ 0.0f };
			// Whether GetExtent reported success at all.  A failed call
			// leaves length and thickness at their initial zeros, which is
			// indistinguishable in the log from a successful call that
			// measured a genuinely thin shape.
			bool         extentOk{ false };
			// Which step declined, when it did.
			const char*  extentStep{ "none" };
			// Set only for the zero-shape fallback, where the radius comes
			// from the scene graph's world bound rather than from any
			// collidable.  A reader that treats the two alike will take the
			// bound radius for a measured one.
			bool         fromWorldBound{ false };
		};

		size_t GatherShapes(RE::NiAVObject* a_root, ShapeBound (&a_out)[kMaxShapesPerObject])
		{
			std::vector<ShapeBound> found;
			// Counted rather than assumed.  "No shapes collected" and "one
			// shape collected with a zero length" produce the same log line
			// unless the visit is counted, and they are opposite problems:
			// the first means the walk never reached a collidable, the second
			// means it reached one and the measurement failed.
			size_t visited = 0;
			size_t boundFailed = 0;
			// Why a collidable was rejected, counted per cause.  "Rejected"
			// on its own does not say whether the node had no body, had a
			// body that is not a rigid body, or had one that made no sense -
			// and an arrow is exactly the case where the body may well be a
			// phantom rather than a rigid body, which is a different fix
			// from a shape whose measurement failed.
			size_t noBody = 0;
			size_t notRigid = 0;
			size_t shapeFailed = 0;
			RE::BSVisit::TraverseScenegraphCollision(
				a_root, [&](RE::bhkNiCollisionObject* a_object) -> RE::BSVisit::BSVisitControl {
					++visited;
					ShapeBound bound{};
					if (ActorShapes::GetBound(a_object, bound.centre, bound.radius) &&
						bound.radius > 0.1f) {
						// A second, cheap walk of the same node for the two
						// axes; a projection per axis is all it costs, and
						// only for nodes that already produced a bound.
						ActorShapes::Extent extent{};
						if (ActorShapes::GetExtent(a_object, bound.centre, extent)) {
							bound.length = extent.length;
							bound.thickness = extent.thickness;
							bound.fromChildren = extent.fromChildren;
							bound.childCount = extent.childCount;
							bound.measured = extent.measured;
							bound.rawPX = extent.rawPX;
							bound.rawMX = extent.rawMX;
							bound.rawPY = extent.rawPY;
							bound.rawMY = extent.rawMY;
							bound.rawPZ = extent.rawPZ;
							bound.rawMZ = extent.rawMZ;
							bound.extentOk = true;
						}
						// Recorded even on failure.  The step name is the whole
						// point of the last round's probe: "length is zero" was
						// consistent with four different causes and the log had
						// no way to pick one.
						bound.extentStep = ActorShapes::LastExtentStep();
						// And the direction those axes run in.  A length is
						// not enough to aim with.
						RE::NiPoint3 axis{};
						if (ActorShapes::GetLongAxis(a_object, axis)) {
							bound.axis = axis;
							bound.hasAxis = true;
						}
						found.push_back(bound);
					} else {
						// The branch that used to be invisible.  A collidable
						// that produced no usable bound simply vanished, and
						// the world-bound fallback below then filled in a
						// radius that looked like a successful measurement -
						// which is exactly how "radius is 151 but length is 0"
						// was read as a contradiction for a whole round.
						if (!a_object || !a_object->body.get()) {
							++noBody;
						} else if (!a_object->body.get()->AsBhkRigidBody()) {
							++notRigid;
						} else {
							++shapeFailed;
						}
						++boundFailed;
					}
					return RE::BSVisit::BSVisitControl::kContinue;
				});

			if (Settings::logContactSamples) {
				logger::info(
					"Shape walk: visited={} accepted={} rejected={} (noBody={} notRigid={} shapeFailed={})",
					visited, found.size(), boundFailed, noBody, notRigid, shapeFailed);
				// What the extent walk last saw, whether or not it succeeded.
				// Reported after the walk rather than inside it so one line
				// covers the whole attempt: the shape type says which branch
				// of the extent switch ran, and the six raw projections say
				// whether the engine gave back anything at all.
				const auto last = ActorShapes::LastExtent();
				logger::info(
					"  last extent: step={} shapeType={} ok={} length={:.1f} thickness={:.1f} radius={:.1f} raw(+X={:.2f} -X={:.2f} +Y={:.2f} -Y={:.2f} +Z={:.2f} -Z={:.2f})",
					ActorShapes::LastExtentStep(), ActorShapes::LastShapeType(), last.measured,
					last.length, last.thickness, last.radius, last.rawPX, last.rawMX, last.rawPY,
					last.rawMY, last.rawPZ, last.rawMZ);
			}

			if (found.empty()) {
				const auto& bound = a_root->worldBound;
				if (bound.radius <= 0.1f) {
					return 0;
				}
				// The fallback, marked as such.  A ShapeBound built here has a
				// radius but no half axes, and every downstream reader that
				// cannot tell the two apart will read the bound radius as a
				// shaft half length - which is where 302.7 came from.
				a_out[0] = ShapeBound{ bound.center, bound.radius };
				a_out[0].fromWorldBound = true;
				return 1;
			}

			std::sort(found.begin(), found.end(),
				[](const ShapeBound& a_lhs, const ShapeBound& a_rhs) {
					return a_lhs.radius > a_rhs.radius;
				});

			const size_t take = std::min(found.size(), kMaxShapesPerObject);
			for (size_t i = 0; i < take; ++i) {
				a_out[i] = found[i];
			}
			return take;
		}

		size_t StampsFor(RE::TESObjectREFR* a_ref, float a_motionX, float a_motionY,
			size_t a_budget, std::vector<Clipmap::Stamp>& a_out)
		{
			if (a_budget == 0) {
				return 0;
			}

			auto* root = a_ref->Get3D();
			if (!root) {
				return 0;
			}

			ShapeBound   shapes[kMaxShapesPerObject]{};
			const size_t shapeCount = GatherShapes(root, shapes);
			if (shapeCount == 0) {
				return 0;
			}

		const bool shaft = IsShaft(a_ref);

		// A shaft that has already been marked and has not moved does not
		// get a second mark.  See Tracked::marked for what the repeated
		// stamping looked like in the log; the short version is that the
		// component keeps answering the candidate query forever and the
		// interval kept re-emitting it, so one arrow became a trench.
		//
		// The check is on distance from the last mark, not on speed: an
		// arrow that has stopped is still "moving" by tiny amounts as the
		// physics settles, and a rule that asked whether it was moving would
		// keep re-marking it.  Asking how far it has come from where the
		// mark was laid is a question with a stable answer.
		if (shaft) {
			const auto mark = g_motion.find(a_ref->GetFormID());
			if (mark != g_motion.end() && mark->second.marked) {
				const float dx = a_ref->GetPosition().x - mark->second.markX;
				const float dy = a_ref->GetPosition().y - mark->second.markY;
				if (dx * dx + dy * dy < kRemarkDistance * kRemarkDistance) {
					// Logged, because a gate that silently returns 0 cannot be
					// told apart in the log from an arrow that was simply
					// never re-scanned - and then a run where nothing changed
					// would read as a working suppression.  This line is the
					// only evidence the mark was found and used, so it names
					// what was suppressed and how still it was.
					LogShaftHeld(a_ref, mark->second.markX, mark->second.markY);
					return 0;
				}
			}
		}

		// The ground a shaft is judged against is not the ground under its
			// bound - a shaft's bound bottom is a whole half length below
			// wherever it actually met something.  So a shaft is resolved to a
			// real contact point first (the engine's reported hit if we have
			// it, otherwise a short ray down the shaft) and everything below
			// uses that point.  Anything else keeps the bound it already had.
			ContactSampler::Output contact{};
			float                 groundZ = shapes[0].centre.z - shapes[0].radius;
			const RE::NiPoint3    boundCentre{ shapes[0].centre.x, shapes[0].centre.y,
				   shapes[0].centre.z };
			RE::NiPoint3 contactPoint = boundCentre;

			if (shaft) {
				ContactSampler::Query query{};
				query.mustBeContact = true;
				query.hasShape = true;

				// The shaft's own axes, not its bound diameter.  Feeding the
				// diameter as the length and half of it as the thickness made
				// the ratio exactly 2:1 for every projectile, which is below
				// the 4:1 a shaft needs - so the shaft lane never ran, no
				// ray was ever spent, and the fix could not take effect.
				//
				// Both axes are taken from *one* shape - the longest - rather
				// than each being the largest across all shapes.  Mixing them
				// describes a shape that does not exist: an arrow's longest
				// part is its shaft and its thickest may be a fletching slab,
				// and inheriting the slab's thickness while keeping the
				// shaft's length shrinks the ratio until the object stops
				// reading as a shaft at all.
				float longest = -1.0f;
				for (size_t i = 0; i < shapeCount; ++i) {
					if (shapes[i].length > longest) {
						longest = shapes[i].length;
						query.length = shapes[i].length;
						query.thickness = shapes[i].thickness;
					}
				}

				// Fall back to the bound only if the shape walk gave nothing:
				// a long thin ratio is then assumed rather than measured, and
				// the caller's own shaft verdict decides.
				if (!(query.length > 0.0f)) {
					for (size_t i = 0; i < shapeCount; ++i) {
						query.length = std::max(query.length, shapes[i].radius * 2.0f);
					}
					query.thickness = query.length * 0.25f;
				} else if (!(query.thickness > 0.0f)) {
					query.thickness = query.length * 0.25f;
				}

				// The axis the shaft will be probed along.  The widest shape
				// wins, to match how length and thickness were taken: the
				// shaft's own capsule, not a fletching slab.
				//
				// Chosen by comparing the shapes against *each other*, never
				// against query.length: that field may already have been
				// rewritten by the bound fallback above into a bound diameter
				// several times any real half axis, and a comparison against
				// it would then never hold - leaving hasAxis false and the
				// axis ray unspent, which is the silent no-op this whole
				// change exists to remove.
				float bestAxisLength = -1.0f;
				for (size_t i = 0; i < shapeCount; ++i) {
					if (shapes[i].hasAxis && shapes[i].length > bestAxisLength) {
						bestAxisLength = shapes[i].length;
						query.hasAxis = true;
						query.axisX = shapes[i].axis.x;
						query.axisY = shapes[i].axis.y;
						query.axisZ = shapes[i].axis.z;
					}
				}

				// The axes the shaft branch will act on.  Printed because a
				// length far larger than the arrow is long was once the only
				// visible symptom of the bound walk picking up the wrong
				// collidable, and there was no way to tell that from the
				// landing point alone.  The axis is printed for the same
				// reason: a ray fired down a wrong axis looks identical to
				// one fired down the right axis when only the landing point
				// is recorded, but not when the direction is.
				if (Settings::logContactSamples) {
					logger::info(
						"Shaft axes: shape(s)={} length={:.1f} thickness={:.1f} axis=({:.3f},{:.3f},{:.3f}) hasAxis={}",
						shapeCount, query.length, query.thickness, query.axisX, query.axisY,
						query.axisZ, query.hasAxis);
					for (size_t i = 0; i < shapeCount; ++i) {
						logger::info(
							"  shape {}: radius={:.1f} length={:.1f} thickness={:.1f} axis=({:.3f},{:.3f},{:.3f})",
							i, shapes[i].radius, shapes[i].length, shapes[i].thickness,
							shapes[i].axis.x, shapes[i].axis.y, shapes[i].axis.z);
					}
					// Which route produced the two axes above.  A shape that
					// reports no children was measured directly; one that
					// reports children was a collection and the numbers came
					// from the widest child.  Printed because the two routes
					// share an output format and a wrong length looks the
					// same either way.
					for (size_t i = 0; i < shapeCount; ++i) {
						logger::info(
							"  shape {}: children={} fromChildren={} extentOk={} measured={} step={} worldBound={}",
							i, shapes[i].childCount, shapes[i].fromChildren, shapes[i].extentOk,
							shapes[i].measured, shapes[i].extentStep, shapes[i].fromWorldBound);
					}
					// The engine's own projections, un-scaled.  These are the
					// numbers everything above is derived from, so a zero
					// length is decided here: six zeros mean the shape's
					// projection really is empty, while six non-zeros with a
					// zero length mean the arithmetic after it is wrong.
					for (size_t i = 0; i < shapeCount; ++i) {
						logger::info(
							"  shape {}: raw +X={:.2f} -X={:.2f} +Y={:.2f} -Y={:.2f} +Z={:.2f} -Z={:.2f}",
							i, shapes[i].rawPX, shapes[i].rawMX, shapes[i].rawPY, shapes[i].rawMY,
							shapes[i].rawPZ, shapes[i].rawMZ);
					}
				}

				query.source = ContactSampler::Source::kProjectile;

				if (a_ref->As<RE::Projectile>()) {
					// Matched on the projectile base, the same way the object
					// scan matches it, so a visual-only blood spray is
					// excluded here too and not just there.
					const auto* base = a_ref->GetBaseObject();
					const auto* model = base ? base->As<RE::TESModel>() : nullptr;
					const char* path = model ? model->GetModel() : nullptr;
					if (path && ObjectStampFilter::IsVisualBloodProjectile(path)) {
						return 0;
					}
					if (const auto* record = FindContact(a_ref->GetFormID())) {
						query.hasContact = true;
						query.contactX = record->position.x;
						query.contactY = record->position.y;
						query.contactZ = record->position.z;
					}
				}

				if (!query.hasContact && !Settings::contactProbeShafts) {
					// Ray probing off, and the engine never told us where this
					// one met something.  There is no honest landing point to
					// stamp, and inventing one is what put the hole in the
					// wrong place, so nothing is stamped.  The engine's own
					// contact, when it exists, is still used - that is free.
					LogContact(a_ref, contact, query, 0.0f, false);
					return 0;
				}

				const auto origin = a_ref->GetPosition();
				query.hasReference = true;
				query.referenceX = origin.x;
				query.referenceY = origin.y;
				query.referenceZ = origin.z;

				const auto inputs = PickInputs();
				contact = ContactSampler::Resolve(query, inputs);

				const auto miss = query.length > 0.0f ? contact.horizontalMiss : 0.0f;
				if (!contact.reachedGround ||
					(Settings::contactProbeMaxMiss > 0.0f &&
						miss > Settings::contactProbeMaxMiss)) {
					LogContact(a_ref, contact, query, miss, false);
					return 0;
				}

				contactPoint.x = contact.x;
				contactPoint.y = contact.y;
				contactPoint.z = contact.z;
				groundZ = contact.z;
				LogContact(a_ref, contact, query, miss, true);
			} else {
				for (size_t i = 1; i < shapeCount; ++i) {
					groundZ = std::min(groundZ, shapes[i].centre.z - shapes[i].radius);
				}
			}

			const RE::NiPoint3 probe{ contactPoint.x, contactPoint.y, contactPoint.z };

			auto* tes = RE::TES::GetSingleton();
			float landZ = 0.0f;
			if (!tes || !tes->GetLandHeight(probe, landZ)) {
				return 0;
			}

			const float surfaceZ = landZ + SnowSurface::LiftAt(probe.x, probe.y);
			const float drop = groundZ - surfaceZ;
			if (drop > Settings::objectContactTolerance || drop < -Settings::objectSinkLimit) {
				return 0;
			}

			const auto  ground = Surfaces::GroundAt(probe);
			const auto  surface = ground.type;
			const auto& response = ground.response;

			if (response.depthScale <= 0.0f &&
				Surfaces::RimHeight(0.0f, response.rimScale) <= 0.0f) {
				return 0;
			}

			const size_t take = shaft ? 1 : std::min(shapeCount, a_budget);
			for (size_t i = 0; i < take; ++i) {
				const auto& shape = shapes[i];

				Clipmap::Stamp stamp{};
				stamp.snow = surface == Surfaces::Type::kSnow;
				if (shaft) {
					// The mark goes where the shaft met something, not where
					// the shaft currently is - the whole point of resolving a
					// contact.  Taking the bound centre here is what put the
					// hole behind the player while the arrow was in front.
					stamp.x = contactPoint.x;
					stamp.y = contactPoint.y;
				} else {
					stamp.x = shape.centre.x;
					stamp.y = shape.centre.y;
				}

				stamp.motionX = a_motionX;
				stamp.motionY = a_motionY;

				stamp.shoulder = std::clamp(response.shoulder, 0.0f, 0.95f);
				stamp.decay = std::clamp(
					Settings::stampDecayPerSecond * response.decayScale * Weather::DecayScale(),
					0.0f, 0.9999f);

				if (shaft) {
					// Pin-prick: a fixed, small radius instead of the shaft's
					// own bound, and no rim at all, so nothing is thrown up
					// around it.  This is the "small hole, no snow" case.
					stamp.radius = Settings::arrowStampRadius *
						std::max(response.radiusScale, 0.0f);
					stamp.depth = std::clamp(
						Settings::stampDepth * Settings::arrowStampDepthScale *
							response.depthScale * Weather::DepthScale(),
						0.0f, 64.0f);
					stamp.rim = 0.0f;
				} else {
					stamp.radius =
						shape.radius * Settings::objectStampRadiusScale * response.radiusScale;

					const float bulk = std::clamp(
						shape.radius / std::max(Settings::objectFullSizeRadius, 1.0f), 0.0f, 1.0f);

					const float ordinary = Settings::stampDepth * Settings::objectStampDepthScale *
						bulk * response.depthScale * Weather::DepthScale();

					stamp.depth = Surfaces::MarkDepth(surface, ordinary, bulk, stamp.x, stamp.y);
					stamp.rim = Surfaces::RimHeight(ordinary, response.rimScale);
				}

				if (i == 0) {
					LogObject(a_ref, surface, stamp, drop);
				}
				a_out.push_back(stamp);

				// Record that this shaft has now been marked, and where.
				// The position taken is the object's own, not the contact
				// point: the gate in this function compares against
				// GetPosition(), so a mark stored as a contact point would
				// compare two different frames and never match.  The gate
				// asks "has this object come somewhere new", and that is a
				// question about where the object is.
				if (shaft) {
					auto& entry = g_motion[a_ref->GetFormID()];
					entry.markX = a_ref->GetPosition().x;
					entry.markY = a_ref->GetPosition().y;
					entry.marked = true;
				}
			}
			return take;
		}

		void Refresh(const RE::NiPoint3& a_anchor, size_t a_budget)
		{
			g_candidates.clear();
			if (a_budget == 0) {
				return;
			}

			auto* tes = RE::TES::GetSingleton();
			auto* player = globals::game::player;
			if (!tes || !player) {
				return;
			}

			const float radius = Clipmap::kWorldSize * 0.375f;

			tes->ForEachReferenceInRange(player, radius,
				[&](RE::TESObjectREFR* a_ref) -> RE::BSContainer::ForEachResult {
					if (g_candidates.size() >= kMaxCandidates) {
						return RE::BSContainer::ForEachResult::kStop;
					}

					if (!a_ref || !a_ref->Is3DLoaded() || a_ref->IsDisabled() ||
						a_ref->IsMarkedForDeletion()) {
						return RE::BSContainer::ForEachResult::kContinue;
					}

					if (a_ref->As<RE::Actor>()) {
						return RE::BSContainer::ForEachResult::kContinue;
					}

					Clipmap::Stamp probe{};
					if (HeatSources::StampFor(a_ref, probe)) {
						g_candidates.push_back({ a_ref->CreateRefHandle(), true });
					} else if (LeavesAMark(a_ref)) {
						g_candidates.push_back({ a_ref->CreateRefHandle(), false });
					}

					return RE::BSContainer::ForEachResult::kContinue;
				});
		}

		float2 TrackMotion(RE::FormID a_form, const RE::NiPoint3& a_position)
		{
			float2 motion{ 0.0f, 0.0f };

			const auto previous = g_motion.find(a_form);
			if (previous != g_motion.end()) {
				const float dx = a_position.x - previous->second.x;
				const float dy = a_position.y - previous->second.y;
				if (dx * dx + dy * dy <= kMaxMotion * kMaxMotion) {
					motion = { dx, dy };
				}
			}

			// Only the three fields this function owns are written.  Assigning
			// a whole Tracked{} here would read the same and be wrong: this
			// runs every frame, before StampsFor is asked, and a fresh entry
			// carries marked = false - so the mark StampsFor laid on the
			// previous pass would be erased before the gate that reads it ran,
			// and the gate would never fire.
			auto& entry = g_motion[a_form];
			entry.x = a_position.x;
			entry.y = a_position.y;
			entry.frame = g_frame;
			return motion;
		}

	}

	void Append(float a_deltaSeconds, const RE::NiPoint3& a_anchor,
		std::vector<Clipmap::Stamp>& a_out)
	{
		if (!Settings::enableObjectStamps || a_out.size() >= Clipmap::kMaxStamps) {
			return;
		}

		const std::scoped_lock lock(g_lock);
		++g_frame;

		g_timer -= a_deltaSeconds;
		if (!g_haveSet || g_timer <= 0.0f) {
			g_timer = std::max(Settings::objectStampInterval, 0.0f);
			g_haveSet = true;
			const int64_t scanStarted = Profiler::Ticks();
			Refresh(a_anchor, Clipmap::kMaxStamps);
			Profiler::AddCpuTicks(Profiler::CpuScope::kObjectScan, Profiler::Ticks() - scanStarted);
		}

		const size_t room = Clipmap::kMaxStamps - a_out.size();
		if (room == 0) {
			return;
		}

		static std::vector<Clipmap::Stamp> built;
		built.clear();

		for (const auto& candidate : g_candidates) {
			if (built.size() >= room) {
				break;
			}

			auto  refPtr = candidate.handle.get();
			auto* ref = refPtr.get();
			if (!ref || !ref->Is3DLoaded() || ref->IsDisabled() || ref->IsMarkedForDeletion()) {
				continue;
			}

			if (candidate.heat) {

				Clipmap::Stamp stamp{};
				if (HeatSources::StampFor(ref, stamp)) {
					built.push_back(stamp);
				}
				continue;
			}

			const auto motion = TrackMotion(ref->GetFormID(), ref->GetPosition());
			StampsFor(ref, motion.x, motion.y, room - built.size(), built);
		}

		std::sort(built.begin(), built.end(),
			[&a_anchor](const Clipmap::Stamp& a_lhs, const Clipmap::Stamp& a_rhs) {
				const float lx = a_lhs.x - a_anchor.x;
				const float ly = a_lhs.y - a_anchor.y;
				const float rx = a_rhs.x - a_anchor.x;
				const float ry = a_rhs.y - a_anchor.y;
				return (lx * lx + ly * ly) < (rx * rx + ry * ry);
			});

		const size_t take = std::min(room, built.size());
		a_out.insert(a_out.end(), built.begin(), built.begin() + take);

		// Entries are dropped once they stop being updated - but a marked
		// shaft that has come to rest is exactly that: its frame stops
		// advancing because TrackMotion only runs for live candidates, and
		// evicting it would forget the mark, so the next pass would lay a
		// second one on top of the first.  A spent arrow lies in the snow for
		// the rest of the session, so the whole point is that its entry
		// outlives its motion.
		//
		// Only unmarked entries expire.  A marked one is kept until the
		// object itself goes away and the reference handle stops resolving.
		for (auto it = g_motion.begin(); it != g_motion.end();) {
			it = (!it->second.marked && it->second.frame + 120 < g_frame) ?
				g_motion.erase(it) :
				std::next(it);
		}
	}

	void NoteContact(RE::Projectile* a_projectile, const RE::NiPoint3& a_position)
	{
		if (!a_projectile || !std::isfinite(a_position.x) || !std::isfinite(a_position.y) ||
			!std::isfinite(a_position.z)) {
			return;
		}

		const std::scoped_lock lock(g_lock);
		g_contacts[g_nextContact++ % g_contacts.size()] = {
			a_projectile->GetFormID(), a_position, std::chrono::steady_clock::now(), true
		};
	}

	void PruneContacts()
	{
		const auto now = std::chrono::steady_clock::now();
		const std::scoped_lock lock(g_lock);
		for (auto& record : g_contacts) {
			if (record.live && now - record.time > std::chrono::seconds(1)) {
				record.live = false;
			}
		}
	}

	size_t ContactCount()
	{
		const std::scoped_lock lock(g_lock);
		size_t live = 0;
		for (const auto& record : g_contacts) {
			live += record.live ? 1 : 0;
		}
		return live;
	}

	void Reset()
	{
		HeatSources::Reset();

		const std::scoped_lock lock(g_lock);
		g_candidates.clear();
		g_motion.clear();
		g_reported.clear();
		g_contactLines = 0;
		g_heldLines = 0;
		g_contacts = {};
		g_nextContact = 0;
		g_timer = 0.0f;
		g_haveSet = false;
	}
}
