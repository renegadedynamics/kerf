# Chamfers past tight corners

*8 November 2026, unit 12W; decisions 1988 on. The owner's decision of that day: add an option to chamfer past tight corners.
OCCT and build123d have nothing like it, and a user chamfering a rounded polygon by more than its corner radius today needs a
hull or a loft. Unit 12V makes such a chamfer refuse, before building, with the limit; this is the option that builds instead.*

**In short.** An equal-distance chamfer of length d steps the face's outline in by d. Along a convex arc of radius r < d that
can't be done: the stepped arc would need a radius of r - d. **Instead the two straight insets beside the arc are carried on
until they meet, at a point P on the corner's bisector, and the chamfer over the arc is the cone from P to the arc's contact
on the side face, at depth d: an oblique circular cone, tangent to the planes beside it along the two lines where it meets
them.** With d = r it's the ordinary chamfer's right cone, its apex on the face; so as d passes r the shape doesn't jump. It's
built where ChFi3d isn't asked, by one Boolean with a tool made exactly of those faces, and checked against a volume and area
worked out from the outline alone. With the option off nothing changes.

## 1. The geometry

### The ordinary chamfer, as a ruled surface

Take the face F to be chamfered as the plane z = 0, its material below, and a loop of its edges that's a whole boundary of
F: lines and circular arcs, γ(s), with n(s) the unit normal in F pointing into F's material. Along the loop the other face S
is square to F: a plane for a line, a cylinder about an axis along z for an arc. A chamfer measured a on F and b on S
(a = b for an equal-distance one) has its contact on S at σ(s) = γ(s) - b z, and on F at e(s) = γ(s) + a n(s), and is ruled
between them: the segment from σ(s) to e(s), in the plane square to the edge. Over a line that's a plane; over an arc, a
right circular cone about the arc's axis. At height z = -b (1 - λ), 0 ≤ λ ≤ 1, its section is the curve

    C_λ(s) = (1 - λ) σ(s) + λ e(s), seen from above: γ(s) + λ a n(s),

the outline stepped in by λ a. Over a convex arc of radius r (its centre c on the material's side) that's an arc about c of
radius r - λ a. **For a > r the arc of radius r - λ a turns inside out above λ = r / a:** the cone passes through its apex,
inside the solid, and comes back out as the other nappe. That's unit 12V's proof that a > r can't be built so, and what ChFi3d
makes there is invalid (unit 12V's measurements).

### What is built instead

Let the arc run from T1 to T2, tangent there to lines L1 and L2 (a rounded corner of a polygon). Step L1 and L2 into F by a:
the lines L1' and L2'. They meet at a point P on the bisector of the angle between them, which is the bisector of the
corner, since the corner's arc is tangent to both. With θ the corner's interior angle and V where L1 and L2 meet, the arc's
centre c is at r / sin(θ / 2) from V along the bisector, and P at a / sin(θ / 2); so P is past c, by (a - r) / sin(θ / 2).

**The contact on F is the polygon E of the stepped lines,** P at the corner. The contact on S is unchanged: the whole edge,
at depth b, arc and all. **The chamfer is the ruled surface between these two contacts, each ruling joining points where
the outline's normal is the same.** Over the arc every normal from L1's to L2's is the arc's, and E has them all only at P,
so the arc's rulings all run to P: **the chamfer over the arc is the cone from P to the arc at depth b.** Over L1 the rulings
join L1 at depth b to L1', the segment of L1 from T1 on included, so the plane through them is the chamfer, cut off where the
ruling from T1 runs to P. That ruling isn't square to the edge, as it is below the limit: the plane's face is a trapezium
from the line at depth b to the segment of E.

**The cone is oblique.** Its base is a circle of radius r about c, its apex P at height b above it, and P isn't over c unless
a = r. A cone over a circle with its apex off the axis is a quadric cone with elliptical sections square to its own axis, and
not a surface of revolution, so it isn't OCCT's `Geom_ConicalSurface`. It's exactly a rational B-spline of degree 2 by 1:
the arc's rational poles and weights for one row, and P with the same weights for the other, since
(1 - v) C(u) + v P has the weights of C(u) whatever v is.

**It's tangent to the planes beside it.** Along the ruling from T1 to P, the cone's tangent plane holds the ruling and the
arc's tangent at T1, which is L1's direction, as the arc is tangent to L1 there. The trapezium's plane holds the same ruling
and the same direction (L1 at depth b). So the planes are one, all along the ruling: the chamfer is tangent-continuous, as
the ordinary one is, and it has no edge of its own but its two contacts and the lines it meets its neighbours along.

**Below the limit and at it, it's the ordinary chamfer.** With a < r, E keeps an arc of radius r - a and the rulings are the
ordinary ones; with a = r, P = c, and the cone is the ordinary right cone with its apex on F. So the shape is continuous in
a through the limit.

**For a convex outline it's the convex hull** of the loop at depth b and E at z = 0, which is the workaround users reach
for. The hull of two convex sets in parallel planes is ruled by joining their points of equal outward normal (their
support points), which is the rule above. It follows that its section at height λ is the Minkowski combination
(1 - λ) R + λ E of the outline's region R and E. **For the check (section 4) only the ruled form matters,** which holds for
outlines that aren't convex too.

### The alternative, and why not

The other natural reading takes each section to be the outline eroded by λ a: the region of points at least λ a inside it.
Over a tight corner that's the ordinary right cone up to its apex, at height b (r / a - 1) below F, and then the two planes
beside it carried on until they meet, along a ridge from the apex up to P. Every point of that has the chamfer's slope, but
it adds a sharp edge running up the middle of the corner, and its face over the arc isn't ruled between the contacts. A
chamfer is the ruled face between its two contacts, and the hull is what a user who works round it today builds, so this
isn't built.

### The cases

| Case | What happens | Built |
|---|---|---|
| Straight neighbours, tangent | As above | **Yes** |
| A sharp corner between two lines, convex or re-entrant | The two planes meet along the ruling from the corner at depth b to where the stepped lines cross: the ordinary chamfer's mitre | **Yes**, in a loop that has a tight corner elsewhere |
| A concave arc (its centre outside the material) | Its stepped arc has radius r + a, which never collapses: the ordinary right cone | **Yes**, beside the rest |
| A convex arc with r > a | The ordinary right cone | **Yes**, beside the rest |
| Two tight corners with a line between | Each its own P on the line's inset. If the two Ps meet or cross, the line's inset is used up and E loses a side; the corners would merge into one | **Yes** while the line's inset is longer than the shape's tolerance; **refused** after |
| Two tight corners with no line between (arc to arc) | The neighbours aren't straight: below | **Refused** |
| A tight arc beside an arc (not a line) | The stepped neighbours meet at a P that isn't on the bisector. Over the stretch of the neighbour from its foot under P to the joint, the neighbour's normals are E's only at P too, so the cone from P runs on over that stretch, as a second oblique cone (over the neighbour's circle), and the neighbour's own chamfer stops at the ruling to P. For a convex neighbour that's still the hull, and tangent throughout; a concave one isn't locally convex, and the rule there isn't proved | **Refused**: not built yet |
| The face too narrow for the insets | E is empty or turns inside out (a side of zero length or less), or its sides cross | **Refused**, saying so |
| Two-distance chamfers | All of the above with a on F and b on S: only a, the distance on the face whose outline has the arcs, decides what collapses. b > r with a < r collapses nothing | **Yes** |
| Distance and angle | Two distances: with S square to F, the angle α from the reference face gives the other distance as the reference's times tan α | **Yes** |
| Constant throat, with or without penetration | The throat isn't measured on the faces | **Not read:** the option leaves it to ChFi3d, as before |
| The side face not square to F (a draft), or not a plane or such a cylinder | Not read | **Refused** |
| Edges that aren't whole loops of one planar face | Not read: a chain that stops has ends of ChFi3d's kind | **Refused** |
| The chamfer as deep as the side face, or deeper | Uses up the side | **Refused** |
| Arcs in the other face of the edge | With S square to F, S is straight going down, so nothing in it collapses. A curved S isn't read | Nothing to do |

"Refused" here is only where a corner is tight. A chamfer below every corner's radius goes to ChFi3d as it always did, with
the option on or off.

## 2. How it's built, and why there

**Alongside ChFi3d, as units 12L and 12M built fillets:** a tool solid of exact faces, one Boolean, a check against the
geometry. ChFi3d isn't asked first where a corner is tight, as it can't make the shape and what it makes there is invalid.

The other way the brief offered, the ordinary chamfer on a modified outline, doesn't give this shape. Rounding the corner to
a radius of a or more changes the side's contact; making it sharp changes the side itself. Either needs the side put back
afterwards with Booleans of its own, which is the same tool by a longer road.

**The tool** is the region between the chamfer and the outside of the solid: for each edge of the loop, its chamfer face
(a plane, a right cone, or the oblique cone), carried below the side's contact by m, a wall standing straight up from the
chamfer's lower edge to F's plane, and its top in F's plane. The faces are made one by one, sewn, and made a solid. The
chamfer faces are carried on below the contact so that they cross the side faces rather than touch them along a line, and
the walls stand outside the solid, as the rulings run outwards below the contact. The top lies in F's plane, the only
coincidence, of the kind Booleans handle every day: it can't stand above F, as the oblique cone's apex is on F and the cone
can't be carried past it. m is half the smaller of b and what the side faces have left below the contact (read from their
vertices), so the tool stays above their feet; anything it cuts that it shouldn't, such as a hole in the face too near the
loop, misses the check and the chamfer isn't built.

Every corner of the loop is built so, also those that don't collapse (as right cones, OCCT's, about the arc's axis), since
the one tool has to run all round: so a loop with any tight corner is built here, all of it. The walls over arcs are
cylinders (unit 13B).

**Built from its faces, where nothing else is near (unit 13B).** The Boolean was 30 to 80 times a plain chamfer: on the user's
box, 89% of the time was the Boolean, most of it intersecting the oblique cones, which are B-splines, with the side cylinders
they cross and with F and the side planes they only touch, at P and at the arc's ends. Where it's known in advance what the
Boolean would find, it isn't asked: F loses the band outside the contact, each side face loses its top down to depth b,
and the chamfer's faces fill the gap, so those faces are made, F and the sides rebuilt with the new edges in place of the
old, and the solid put together from them and the shape's other faces, unchanged. That's known when:

- F has no loops but those chamfered, and no face of the shape but F and the loops' sides comes within depth b of F (by its
  box, which is larger than it);
- at each corner of a loop one straight edge runs square to F, down between the two sides, past depth b, and each side's
  other edges lie deeper than b;
- seen from above, the band each piece's chamfer lies over (its edge, its stretch of the contact, the rulings between) meets
  no other piece's band, its neighbours' only along the ruling they share, and the two beside a collapsed corner only at P.
  Each band is taken as the convex hull of its corners and its arcs' control points, which holds it, so this can turn down
  what's sound but never pass what isn't.

Then nothing of the shape lies in what's removed but material, and no face of the chamfer crosses another or the shape's,
which the volume, area and validity checks couldn't see on their own: a chamfer running through a hollow below F would
measure the same, as the hollow's own faces count against it either way. Anything else goes to the Boolean, as before. The
edges are exact: the side's arc at depth b is the side's own circle moved down, its curve on the side moved with it, and on
the oblique cone fitted by SameParameter (to 1e-7 on the user's box); a right cone is OCCT's, in the arc's own frame, so its
curves are lines; the rulings are lines. The Boolean's were B-splines, to 5e-6. The faces, edges, vertices and history are
the Boolean's, for the shape's own parts, and the volumes agree to 1e-9 (`ChamferTightCornersTests`).

## 3. The option

- `Kerf.Kernel`: `ChamferOptions.TightCorners`, of the enum `TightCorners`: `Refuse` (the default) or `Collapse`.
- `Kerf.Modeling`: `Operations.ChamferOptions { TightCorners = TightCorners.Collapse }`, an optional last argument of
  `Chamfer` (the operation, and the `Chamfer` member of a 3D shape).

`Collapse` builds what's above where a corner is tight and the case is one it builds; where a corner is tight and the case
isn't, it refuses, saying which part isn't built yet. Where no corner is tight it does nothing at all. **Tight** means a
convex arc of the loop whose radius is no more than the distance on its face plus the shape's tolerance there, so a length
within tolerance of the radius is built here too, exactly, as the right cone it is.

`Refuse` is everything as it was: ChFi3d, and unit 12V's refusal (decision 1985). With `Collapse`, the collapse is asked
first, inside the same operation, so 12V's refusal is reached only where no corner is tight or the option is off.

**`MaxChamfer` and `ChamferLimit`** (unit 12V, decision 1987) take no options and give the default's limit: the tightest
convex arc's radius, or a face's width. With `Collapse` a tight arc sets no limit; the most that fits is then where a
straight edge's inset is used up or the face is too narrow, which they don't compute. They're left as they are
(decision 1992).

## 4. The check

Independent of how the tool is made: from the outline's lines and arcs, and the two distances.

**Volume.** At height z = -b (1 - λ) the section is the curve C_λ above (a line from (1 - λ) T + λ P to the next, or an arc
about (1 - λ) c + λ P of radius (1 - λ) r over a tight corner, or about c of radius r ∓ λ a over the rest). Its area A(λ), by
Green's theorem, is a quadratic in λ, as C_λ is linear in λ for each s. So Simpson's rule is exact:

    removed = b A(0) - b / 6 (A(0) + 4 A(1/2) + A(1)) = b / 6 (4 (A(0) - A(1/2)) + (A(0) - A(1))),

the signed areas taken with the material on the left, so a loop round a hole counts the right way. **Area:** F loses
A(0) - A(1), each side a band b deep along the edge, and the chamfer adds its faces: trapezia from their corners, right cones
as θ s (r_0 + r_1) / 2 with s the slant, and an oblique cone as ½ r ∫ √(b² + ((c - P)·e(u) + r)²) du over the arc's angle,
e(u) the arc's radial direction there, by Gauss-Legendre: a smooth integrand, to well past the check's one part in a
million. Then validity. Anything that misses refuses: "couldn't be built".

**Measured once (unit 13B).** Built from its faces, the shape's volume and area are those of the faces it kept plus those it
lost, and the answer's the faces kept plus the new, each face as it adds to the whole about one point: the faces kept are
integrated once, not twice. Planar faces bounded by lines and circles are taken in closed form (`PlanarFaceMass`), which
fixed Gauss points are exact on too, only slower. The operation doesn't check validity again after this check has.

**In the tests,** the volumes are worked out again a third way, by the closed form for a rounded rectangle:
A(1/2) is the area of the rectangle (W - r - a) by (H - r - a) grown by r / 2, as the mean of the region and E.

## 5. What it doesn't do

The cases refused above; outlines of B-splines or ellipses; the side faces drafted; open chains; chamfers of several faces'
loops at once (several loops of one face, an outline and its holes, are built, each with its own tool, in one Boolean). A tight arc beside a convex arc is the next to build, if a user brings one: its geometry is above.

## Sources

None outside the code: OCCT has no such option (`BRepFilletAPI_MakeChamfer`, `ChFi3d_ChBuilder`), nor has build123d's
`chamfer`.
