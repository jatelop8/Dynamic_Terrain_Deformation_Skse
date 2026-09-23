// SPDX-License-Identifier: GPL-3.0-only
// Copyright (c) 2026 NearMidnightNow (NMN).

#pragma once

namespace ActorShapes
{

	// The three half extents of a shape along its own axes, in game units.
	// A single radius cannot describe an arrow: it is long and thin, and
	// collapsing it to one number is what made every projectile read as a
	// sphere and lose its long axis.  The largest of the three is the length
	// half; the smallest is the thickness half.
	struct Extent
	{
		float length{ 0.0f };     // half extent along the long axis
		float thickness{ 0.0f };  // half extent along the short axis
		float radius{ 0.0f };     // the single-number bound, as before

		// Diagnostic only.  Says whether the numbers above came from walking
		// a container's children or from projecting the shape itself, and how
		// many children there were.  An arrow whose collidable turns out to
		// be a single capsule reports zero children; one wrapped in a
		// collection reports two or three.  Without this the log cannot tell
		// a correct reading from a container's undocumented self-projection.
		bool fromChildren{ false };
		int  childCount{ 0 };

		// Diagnostic only, and the one that matters most: the raw numbers
		// GetMaximumProjection handed back per axis, before any scaling or
		// halving.  A zero length with a non-zero radius is otherwise
		// ambiguous - it could mean the projection returned zero, or that the
		// caller never got as far as measuring - and those two want opposite
		// fixes.
		float rawPX{ 0.0f };
		float rawMX{ 0.0f };
		float rawPY{ 0.0f };
		float rawMY{ 0.0f };
		float rawPZ{ 0.0f };
		float rawMZ{ 0.0f };
		bool  measured{ false };
	};

	bool GetBound(RE::bhkNiCollisionObject* a_object, RE::NiPoint3& a_centre, float& a_radius);

	// Which step of the last GetExtent call declined, as a short label.  All
	// the failure paths return the same false, and the fix differs per step,
	// so the name is reported rather than inferred.
	const char* LastExtentStep();

	// The last shape the extent walk measured, and what it measured.  The
	// raw hkpShapeType value is reported because the recognised types are a
	// short list and anything outside it takes a different branch.
	int    LastShapeType();
	Extent LastExtent();

	// The extent form: same walk, but it keeps the long and short axes
	// apart instead of discarding them.
	bool GetExtent(RE::bhkNiCollisionObject* a_object, RE::NiPoint3& a_centre, Extent& a_extent);

	// The unit direction, in world space, of the long axis that GetExtent
	// measured.  A length alone cannot aim a ray: an arrow stuck in at an
	// angle has its long axis pointing somewhere between horizontal and
	// vertical, and a ray fired straight down from its origin can only ever
	// land under the origin.
	bool GetLongAxis(RE::bhkNiCollisionObject* a_object, RE::NiPoint3& a_axis);

	bool ExtractRadius(const RE::hkpShape* a_shape, float& a_radius);

	bool ExtractExtent(const RE::hkpShape* a_shape, Extent& a_extent);
}
