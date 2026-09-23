# SPDX-License-Identifier: GPL-3.0-only
# Copyright (c) 2026 NearMidnightNow (NMN).
#
# Byte scan of the *deployed* dll for the strings a build with these changes
# must carry and the ones it must not.  It answers one question - is the file
# in the game folder the file that was just built - and it has to be read that
# way: a green scan says nothing about whether the change is correct, only that
# the build reached disk.  `dtd_build.sh` swallowing an exit code, or a copy
# that silently did nothing, would both show up here and nowhere else.
#
# What this build changed, and therefore what has to be pinned:
#
#   * the build tag;
#   * the width knob actually reaches the mark.  An earlier build wired the
#     width to `LineWidth(thickness, 1.0f, 0.0f, 0.0f)`, whose last line is
#     `return scaled < a_max ? scaled : a_max;` - so a_max = 0 means "return
#     zero", not "no ceiling", and the footprint's half width came out as 0.0.
#     The ceiling then resolved to the bare ShaftLineMaxWidth 6.0 and every
#     mark was drawn 24 x 6.  `ContactPoint::FootprintHalfWidth` carries the
#     same arithmetic by name, and `ContactPoint::LengthDerivedWidth` is what
#     lets ShaftMarkFootAspect move the width at all - with the footprint's
#     half width left in as a floor, every value below 0.506 resolved back to
#     12.15 and the knob was dead.  Both calls are pinned, because the whole
#     point of that failure was arithmetic that no test and no pin could see;
#   * the rim edge of a *line* mark is jittered in world space.  The gate is
#     pinned rather than the arithmetic, because a footprint's mark must keep
#     its smooth edge - see the note below about why the switch itself has
#     to be pinned and not just the body.
#
# Pins that must not be used:
#
#   * anything naming an `inline` function (`ContactReach`, `HeightAboveSnow`,
#     `DropAt`, `HalfLengthFromStretch`, `Allow`, `GateIsBinding`,
#     `WidestLimit`, `TightestLimit`).  Inline definitions are either expanded
#     or dropped, so their names are not reliably in the image at all - and
#     `ContactReach` and `HeightAboveSnow` both measure **0** in this build,
#     which was checked rather than assumed.  A pin on one is a claim that
#     cannot fail, which is worse than no pin.  The arithmetic is asserted in
#     tools/offline/StampSurfaceTest.cpp (CheckContactReach) instead, where it
#     can actually be called.
#   * `groupshared` and `const float span = max(` as equality-1 pins.  The
#     update shader's body is embedded **twice**, and has been in every build
#     measured - across forty-odd archives, not assumed.
#
# Run:  python tools/offline/r54_scan.py
#
# A pin has to be a needle that can only match the thing it names.  A bare
# `t {:+.2f}` reads **2**, and the second match is `highes{t {:+.2f}} world
# units` in the shape census - a different literal that happens to end in the
# same characters.  Appending the NUL terminator pins the literal itself and
# the false match disappears.  A count that is right for the wrong reason is
# the same failure as a probe that cannot say no.
#
# The surface pins are shader text and are kept as they were: the changes since
# did not touch the outline measurement they name, so their counts are still
# the evidence for half of the image; the two shader pins below are the other
# half of the same argument, for the shader text that did change.

import datetime
import hashlib
import os
import sys

DLL = ("E:/EJ/mod/mods/Dynamic Terrain Deformation Skse EJ"
       "/SKSE/plugins/NMN_DeformableTerrain.dll")

# The source the walk's surface and the mark's shape are built in, scanned as
# TEXT rather than as bytes of the image.
#
# This is a second kind of pin, and it exists because the first kind cannot do
# this job.  The dll pins above can only see strings, and no string records an
# addition: a build where the walk's `land + lift` was flipped back to the bare
# `land` carried every string the image was supposed to carry, and the offline
# test - which does not link Clipmap.cpp - stayed green.  The sabotage survived
# both gates, which makes it exactly the kind of half-change these rules exist
# to catch.
#
# The arithmetic itself is assertable (ContactPoint::SurfaceAt, pinned by
# CheckContactReach in StampSurfaceTest.cpp).  What cannot be asserted is that
# the caller uses it, so that is what this checks: the gate and the walk in
# Clipmap.cpp must reach the surface through the named function, and the
# source must not carry the old bare-land expression anywhere on this path.
#
# A source pin is weaker than a test - it cannot know the value is right - so
# it is written to be exact rather than suggestive: the call must appear, and
# the pattern it replaced must not.
SRC = "D:/Modding/_dtd_src/src/Clipmap.cpp"

# (substring, expected count, what it is evidence of)
SRC_PINS = [
    ("ContactPoint::SurfaceAt(land,", 1,
     "the axis walk's land sampler reaches the surface by name"),
    ("SnowSurface::LiftAt(", 5,
     "every place the weapon path needs the blanket's height has one, in "
     "code: the gate, the mesh gap, the walk, the crossing, and the "
     "end-fallback.  Five, not seven - two more matches in the raw file are "
     "the comments that name this call while explaining it, which is why "
     "comments are stripped above"),
    ("a_z = land;", 0,
     "the bare-land sampler is not back"),
    ("= land + SnowSurface::LiftAt", 0,
     "and neither is the inline expression the name replaced"),
    ("GetLandHeight(RE::NiPoint3{ hit.x, hit.y, hit.z }, landAtHit)",
     1, "the crossing still samples the terrain and adds the blanket through "
        "HeightAboveSnow"),
    ("ContactPoint::FootprintHalfWidth(", 1,
     "the footprint's half width is computed by name, and this is the call "
     "that would have shown the bug: it used to be "
     "`LineWidth(thickness, 1.0f, 0.0f, 0.0f)`, whose `a_max = 0` means "
     "`return 0`, not `no ceiling` - so the half width was 0.0 and the "
     "ceiling resolved to the bare 6.0"),
    ("ContactPoint::LengthDerivedWidth(", 1,
     "and the mark's own width is derived from its length and its aspect, "
     "without the footprint's half width left in as a floor - that floor is "
     "what made ShaftMarkFootAspect a dead knob for every value below 0.506"),
    ("ContactPoint::AspectCappedLength(", 1,
     "the length is capped against that width, which is what stops a long "
     "thin mark from stacking into the rake of parallel teeth"),
    ("ContactPoint::AlignedWidthCeiling(", 1,
     "the ceiling is resolved rather than borrowed: ShaftLineMaxWidth is 6.0 "
     "and a footprint's own half width is 12.15, so the old constant would cap "
     "every aligned mark at half the shape it is aligning to"),
    # Both arms of the alignment branch, and the switch that guards them.
    #
    # A sabotage replaced the switch's condition with a literal `false` and
    # nothing caught it: the calls above are still in the file, so the pins
    # stayed green, and `Clipmap.cpp` is not linked into the offline test so no
    # assertion could see it either.  Pinning the switch itself and the
    # else-arm is what closes that - a branch that is never taken on the built
    # configuration is not covered by pinning its body.
    ("if (Settings::shaftMarkAlignToFoot) {", 1,
     "the alignment is actually branched on, not switched off with a literal"),
    ("if (false) {", 0,
     "and the switch was not replaced by something that never runs"),
    ("stamp.halfWidth = thicknessWidth;", 1,
     "the thickness width survives as the escape arm, so turning the "
     "alignment off really does restore the thickness shape"),

]

# (needle, expected count, what it is evidence of)
PINS = [
    ("0.1.0.2", 1, "the build tag, matching the version declared in plugin.cpp"),
    ("r54-width-knob-rim-jitter", 0, "the round-scoped tag it replaced is gone"),
    ("r53-mark-align-foot", 0, "and the one before that"),
    ("r52-shaft-snow-surface", 0, "and the one before that"),
    ("r50-rim-band-world", 0, "and the one before that"),
    ("r49-one-surface", 0, "and the one before that"),
    ("r48-shaft-log-rate", 0, "and the one before that"),
    ("r47-", 0, "no round-scoped tag survives in the image"),

    # --- the surface -------------------------------------------------------
    # The two shape-dependent formats, one per route.  Two is the count
    # because LogShaftStamp carries a line form and a disc form, exactly as
    # `rim {:.2f}` does below.
    ("lowest {:.2f} above snow (limit {:.2f})", 2,
     "the mark line reports the height above the SNOW on both routes"),
    ("end {:.2f} above snow | ", 1,
     "and the end field names the snow, on the route that has one"),
    ("end {:.2f} above land | ", 0,
     "the old spelling is gone - it named a surface the gate no longer uses"),
    # Exactly one `above land` must survive, and it is the foot path's own
    # format in LogCollidable.  Zero would mean the foot diagnostic was
    # renamed along with the weapon one, which would be a silent change to a
    # line this round had no business touching; two or more means a weapon
    # format was missed.
    ("above land", 1,
     "exactly the foot path's own field keeps the old spelling - see the note"),
    # NOTE (why there is no bare `above snow` pin here): counting that phrase
    # by hand gives 3, not 2, and the extra one is real - the collision probe
    # format reads `{} lowest corner {:.2f} above it`, whose "it" is the snow.
    # A bare-phrase pin would therefore either read 3 (and look, wrongly, like
    # the foot format had been renamed) or, pinned at 3, would silently absorb
    # a genuine fourth site.  The phrase cannot distinguish sites, which is
    # what rule V is about; the formats above name each site instead.

    # --- the mark alignment, width knob, and rim jitter --------------------
    ("MarkAlignToFoot=", 1,
     "the startup line reports the alignment switch"),
    ("FootAspect={:.2f} MaxAspect={:.2f} RimJitter={:.2f} RimJitterBand={:.2f}"
     " ", 1,
     "and the four numbers that decide the aligned shape and the wobble of "
     "its edge - FootAspect is the width knob, so a build that lost it would "
     "silently fall back to the checkbox's hard-coded default"),

    # --- the rim jitter, shader half ---------------------------------------
    #
    # The gate is pinned here rather than in the source list because the gate
    # is *shader text*, and shader text is what this half of the file scans.
    # It is worth pinning because it is a caller rule: the arithmetic lives in
    # RimEdgeJitter, the decision to apply it to a line and not to a footprint
    # lives in this one `if`, and flipping the `if` to always-on would leave
    # every other string in the image intact.  Both halves of the condition
    # are in the pin, so neither the shape test nor the amount test can be
    # dropped without it going red.
    #
    # Counts are the measured ones, not the header counts.  `RimEdgeJitter`
    # reads 3 in ClipmapUpdateCS.h - a definition, its call, and the comment
    # above the definition - and 6 in the image, because the shader body is
    # embedded twice and the comment survives in the emitted text.  Writing
    # the header count here is what the first run of this script caught: it
    # went red on both, which is the scan doing its job.
    ("if (StampShape[i].z > 0.0f && MarkRimJitter.x > 0.0f) {", 2,
     "the rim jitter is gated on the mark being a line, so a footprint keeps "
     "the smooth edge it has always had - twice, once per embedded copy"),
    ("float jitter = 0.0f;", 2,
     "and a mark that is not a line leaves both terms at their identity, so "
     "the footprint's rim is byte-for-byte what it was"),
    ("RimEdgeJitter", 6,
     "the rim jitter's definition, its comment and its call, in both copies "
     "of the shader"),
    ("MarkRimJitter", 10,
     "the constant buffer field it reads: the declaration, and each use, in "
     "both copies of the shader"),
    ("0.5f + 0.5f * n) - 0.25f * a_amount", 2,
     "the jitter is biased outward, so the edge wobbles without leaving gaps "
     "inside the band - an unbiased wobble eats the band it is drawn on"),

    # --- the gate and the window -------------------------------------------
    ("Shaft gate: carried clear", 1, "the refusal line is kept"),
    ("Shaft tally:", 1, "the tally window line is kept"),
    # Three, not two: the tally's own column plus the two shape-dependent
    # forms in LogShaftStamp.  Written as 2 first and measured as 3 - the
    # count is easy to get wrong here precisely because the stamp's two are
    # already inside the `lowest ...` pin above, so they are easy to forget
    # while reading the list.  Measured on the built image, then written down.
    ("(limit {:.2f})", 3, "the tally and both mark routes still print a limit"),

    # --- carried forward, untouched ----------------------------------------
    ("t {:+.2f}\0", 1, "the crossing parameter, formatted only where there is one"),
    ("t -\0", 1, "and the spelling for a mark that solved no crossing"),
    ("place {} {} | ", 2, "the column it goes in, in the line and the disc format strings"),
    ("place {} t {:+.2f}", 0, "the old inline form is gone from both routes"),
    ("rim {:.2f}", 2, "the rim is printed on the line route and the disc route"),
    ("ShaftStampMinDepth", 1, "the depth floor is kept"),
    ("StampMinDepth={:.2f}", 1, "and is still reported at startup"),
    ("Shaft stamp: LINE half length", 1, "the mark line"),
    ("drop {:.2f} of radial {:.2f}, axis z {:+.2f}", 1, "the thickness probe"),
    ("(drop {:.2f})", 0, "the old length-shaped drop format is gone"),
    ("SpanLengthScale={:.2f}", 1, "the startup line still reports the scale"),

    # --- the shader half, untouched ----------------------------------------
    ("StampOutlineOvershoot", 4, "the rim measures the mark's outline in world units"),
    ("(len - 1.0f) / invGrad", 2, "with the first-order distance to that outline"),
    ("max(bandRef * rim.x, 2.0f * Window.z)", 2, "and the band is held to two cells"),
    ("d - s.z", 0, "the mark's own units are gone from the rim"),
    ("groupshared", 2, "the shader body is embedded twice - see the note above"),
]


def main():
    if not os.path.exists(DLL):
        print("MISSING:", DLL)
        return 2
    with open(DLL, "rb") as f:
        blob = f.read()

    print("dll   :", DLL)
    print("size  :", len(blob))
    print("md5   :", hashlib.md5(blob).hexdigest())
    print("mtime :", datetime.datetime.fromtimestamp(os.path.getmtime(DLL)))
    print()

    bad = 0
    for needle, want, why in PINS:
        got = blob.count(needle.encode("utf-8"))
        ok = want is None or got == want
        if not ok:
            bad += 1
        tag = "OK " if ok else "!! "
        shown = "?" if want is None else str(want)
        print(f"{tag}{needle!r:46s} got {got:3d}  want {shown:>3s}   {why}")

    print()
    print("--- source pins (Clipmap.cpp) ---")
    if not os.path.exists(SRC):
        print("MISSING:", SRC)
        return 3
    with open(SRC, encoding="utf-8") as f:
        raw = f.read()

    # Comments are stripped before counting, and that is not tidiness.
    #
    # The first version counted the raw text and read 7 where 5 was meant:
    # two of the matches were the explanatory comments that name the same
    # call while describing it.  A pin that counts prose is a pin that goes
    # red when a comment is reworded and green when a line is deleted with a
    # comment added in its place - wrong in both directions, and it would
    # have been read as evidence about the code either way.
    #
    # This is a line-based strip rather than a parser: it removes `//` to end
    # of line outside string literals, which is all this file's comments are.
    # It is deliberately conservative - a `//` inside a literal is left alone
    # - because a stripper that eats code would silently shorten the haystack
    # and turn every pin into a weaker claim.
    code_lines = []
    for line in raw.splitlines():
        in_literal = False
        quote = ""
        cut = None
        i = 0
        while i < len(line):
            ch = line[i]
            if in_literal:
                if ch == "\\":
                    i += 2
                    continue
                if ch == quote:
                    in_literal = False
            elif ch in "\"'":
                in_literal = True
                quote = ch
            elif ch == "/" and i + 1 < len(line) and line[i + 1] == "/":
                cut = i
                break
            i += 1
        code_lines.append(line if cut is None else line[:cut])
    text = "\n".join(code_lines)

    for needle, want, why in SRC_PINS:
        got = text.count(needle)
        ok = want is None or got == want
        if not ok:
            bad += 1
        tag = "OK " if ok else "!! "
        shown = "?" if want is None else str(want)
        print(f"{tag}{needle!r:52s} got {got:3d}  want {shown:>3s}   {why}")

    print()
    print("MISMATCH" if bad else "ALL OK", f"({bad} mismatched)" if bad else "")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
