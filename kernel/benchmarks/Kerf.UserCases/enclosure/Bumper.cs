// Blade sensor, bumper design (Kerf): JP's spec of 2026-11-07, for the first 10-20 sets. A port of blade_bumper.py.
//
// A one-piece ABS body clamps round a bonded rubber strip, closed by a cap under the shaft. A door on its back face
// carries most of the boards. The arm runs from the body to the coil, inside a TPU boot that never touches it, and
// ends in a TPU housing clamped to the shaft round a rigid saddle and the aluminium plate. TPU bumpers cover the body
// and door, leaving the raised flanges either side of the O-ring bare. See README-bumper.md.
//
// Frame used everywhere:
//     origin  on the shaft axis, at the body's back face (where the door meets it)
//     +X      along the shaft towards the handle; the arm points +X, the door is at -X
//     +Z      out of the stern face with the blade square: the arm side
//     +Y      completes the right-handed frame
//
// On each side, the boards lie on a flat facet tilted 60 deg from +Z. n is the distance out from the axis along the
// facet's normal; u is up the facet, towards the arm. y = s(n S60 - u C60), z = n C60 + u S60.
//
// Sections are sketches in the XY plane holding (y, z); Ex() stands them on the YZ plane and extrudes them along X.

using Kerf;
using Kerf.Geometry;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Renegade.BladeBumper;

public static partial class Bumper
{
    static readonly double S60 = Math.Sin(Math.PI / 3), C60 = Math.Cos(Math.PI / 3);
    static readonly Alignment MIN = new(Align.Center, Align.Center, Align.Min);
    static readonly Alignment MAX = new(Align.Center, Align.Center, Align.Max);

    /// <summary>The value of an operation that must succeed: a failure stops the model, with Kerf's reason.</summary>
    static T Ok<T>(this Result<T> r, string what) =>
        r.TryGetValue(out var v) ? v : throw new InvalidOperationException($"{what}: {r.Error}");

    // ---------------------------------------------------------------- basic geometry

    public static double ZArm() => P.SHAFT_D / 2 + P.ARM_GAP + P.ARM_D / 2;

    public static double BodyBoreR() => P.SHAFT_D / 2 + P.RUBBER_T;

    public static double BodyTopZ() => P.Z_ROOF + P.WALL;

    public static (double Y, double Z) Yz(double n, double u, int s = 1) => (s * (n * S60 - u * C60), n * C60 + u * S60);

    /// <summary>(n, u) of a point on the +y side.</summary>
    public static (double N, double U) Nu((double Y, double Z) p) => (p.Y * S60 + p.Z * C60, -p.Y * C60 + p.Z * S60);

    /// <summary>Where the facet's floor meets the top floor.</summary>
    public static (double Y, double Z) PtA() => ((P.N_FLOOR - C60 * P.Z_TOP_FLOOR) / S60, P.Z_TOP_FLOOR);

    static (double Y, double Z) PtB() => Yz(P.N_FLOOR, P.U_LOW);

    static (double Y, double Z) PtC() => Yz(P.N_ROOF, P.U_LOW);

    static (double Y, double Z) PtD() => ((P.N_ROOF - C60 * P.Z_ROOF) / S60, P.Z_ROOF);

    static IEnumerable<(double Y, double Z)> Mirror(IEnumerable<(double Y, double Z)> pts) => pts.Reverse().Select(p => (-p.Y, p.Z));

    static Point3D Pt((double Y, double Z) p) => new(p.Y, p.Z, 0);

    public static Sketch Poly(IEnumerable<(double Y, double Z)> pts) => new Polygon(pts.Select(Pt), align: Align.None);

    static readonly ExtrudeOptions AlongX = new() { Direction = new Vector3D(1, 0, 0) };
    static readonly ExtrudeOptions AlongZ = new() { Direction = new Vector3D(0, 0, 1) };

    /// <summary>
    /// Extrude a (y, z) sketch from x0 to x1. The direction is given: Kerf, like build123d, extrudes along each face's
    /// normal by default, and a polygon whose points run clockwise faces -X.
    /// </summary>
    public static Part Ex(Sketch sketch, double x0, double x1) =>
        new Pos(x0, 0, 0) * Extrude(new Location(Plane.YZ) * sketch, x1 - x0, AlongX).Ok("extrude");

    /// <summary>Extrude an (x, y) sketch up +Z by h.</summary>
    static Part ExZ(Sketch sketch, double h) => Extrude(sketch, h, AlongZ).Ok("extrude up");

    static Sketch Off(Sketch sketch, double d) => Offset(sketch, d, new OffsetOptions { Kind = Kind.Arc }).Ok($"offset {d}");

    public static Sketch Circle2(double r, double y = 0, double z = 0) => new Pos(y, z) * new Circle(r);

    static Sketch Rr(double w, double z0, double z1, double r) => new Pos(0, (z0 + z1) / 2) * new RectangleRounded(w, z1 - z0, r);

    /// <summary>(y, z) of the four door screws: the top corners, and below the pockets on each side.</summary>
    public static List<(double Y, double Z)> DoorScrews()
    {
        var (ty, tz) = P.TOP_SCREW;
        var (ly, lz) = P.LOW_SCREW;
        return [(ty, tz), (-ty, tz), (ly, lz), (-ly, lz)];
    }

    /// <summary>The convex hull of edges in the (y, z) plane, with true arcs where it follows a circle.</summary>
    static Sketch Hull(IEnumerable<Edge> edges) => new(Face.Make(Wire.MakeConvexHull(edges)));

    static IEnumerable<Edge> Lines(params (double Y, double Z)[] pts) =>
        pts.Zip(pts.Skip(1), (a, b) => Edge.MakeLine(Pt(a), Pt(b)));

    static IEnumerable<Edge> CircleEdges(double r, double y, double z) => Circle2(r, y, z).Edges();

    static Sketch BoreChamfers(double r, double z)
    {
        var c = P.BORE_CHAMFER;
        Sketch Side(int s) => Poly([(s * r, z + c), (s * (r + c), z), (s * (r + c), z - 1), (s * r, z - 1)]);
        return Side(1) + Side(-1);
    }

    /// <summary>A bore that is straight-sided below the axis, so a part can lift off the shaft.</summary>
    static Sketch UBore(double r) => Circle2(r) + Poly([(-r, 0), (r, 0), (r, -50), (-r, -50)]);

    /// <summary>The arch's outline, before its wall: the cavity with its floors filled in, and room round the screws.</summary>
    static IEnumerable<Edge> ArchEdges()
    {
        var edges = Lines(PtB(), PtC(), PtD()).Concat(Lines(Mirror([PtB(), PtC(), PtD()]).ToArray()));
        foreach (var (y, z) in DoorScrews())
            edges = edges.Concat(CircleEdges(P.HULL_PAD, y, z));
        return edges;
    }

    static readonly Lazy<Sketch> hull = new(() => Hull(ArchEdges()));

    /// <summary>The arch's convex outline: the cavity with its floors filled in, and room round the screws.</summary>
    static Sketch ArchHull() => hull.Value;

    static readonly Lazy<Sketch> bodySection = new(() =>
    {
        var r = P.CLAMP_BOTTOM_R;
        double yc = P.CLAMP_HALF - r, zc = -(P.CLAMP_BOTTOM - r);
        var edges = ArchEdges().Concat(CircleEdges(r - P.WALL, yc, zc)).Concat(CircleEdges(r - P.WALL, -yc, zc));
        return Off(Hull(edges), P.WALL);
    });

    /// <summary>The body and its clamp together: one convex outline, split at the body's underside, the cap tapered.</summary>
    public static Sketch BodySection() => bodySection.Value;

    static readonly Lazy<Sketch> housingSection = new(() =>
    {
        var (y0, z0, r) = P.HOUSING_TOP;
        var trim = Off(Poly([(-y0, z0), (y0, z0), (60, -10), (60, -60), (-60, -60), (-60, -10)]), r);
        return Off(Off(BodySection() & trim, -1.5), 1.5);
    });

    /// <summary>The body's section with its top trimmed to the housing's width: it fits inside the body's.</summary>
    public static Sketch HousingSection() => housingSection.Value;

    static Part Above(double z) => new Pos(0, 0, z) * new Box(1000, 400, 400, align: MIN);

    static Part Below(double z) => new Pos(0, 0, z) * new Box(1000, 400, 400, align: MAX);

    /// <summary>The door's cavity in section, an arch.</summary>
    static Sketch SealedFace()
    {
        (double, double)[] half = [PtA(), PtB(), PtC(), PtD()];
        return Poly(half.Concat(Mirror(half)));
    }

    static readonly Lazy<Sketch> mouthFace = new(() =>
    {
        var r = P.SEAL_ROUND;
        var m = SealBase();
        foreach (var (y, z) in DoorScrews())
            m -= Circle2(P.SCREW_CLEAR, y, z);
        var near = DoorScrews().Take(2).Select(s => Circle2(P.SEAL_ROUND_NEAR, s.Y, s.Z)).Aggregate((a, b) => a + b);
        m = Off(Off(m, -r), r) + (m - near);
        if (m.Faces().Count != 1)
            throw new InvalidOperationException($"the mouth has {m.Faces().Count} faces");
        return m;
    });

    /// <summary>
    /// The sealed face where the door meets the body: the O-ring runs round it, and round the top screws. Where it
    /// loops round a top screw its corners are opened (rounded off) so the ring has no hairpins; elsewhere the convex
    /// corners stay, because the pockets reach into them. Concave corners are closed (filled) everywhere.
    /// </summary>
    public static Sketch MouthFace() => mouthFace.Value;

    /// <summary>The door's cavity, but between the facets' tangent points its floor is an arc round the shaft (JP).</summary>
    static Sketch SealBase()
    {
        (double, double)[] half = [PtB(), PtC(), PtD()];
        var f = Poly(half.Concat(Mirror(half))) - Circle2(P.N_FLOOR);
        foreach (var s in new[] { 1, -1 })   // below each facet's floor, down-slope of its tangent point
            f -= Poly([Yz(P.N_FLOOR, 0, s), Yz(P.N_FLOOR, -40, s), Yz(0, -40, s), Yz(0, 0, s)]);
        return f;
    }

    public static Wire OringCentreline() => Off(MouthFace(), P.SEAL_LAND + P.GROOVE_W / 2).Faces()[0].OuterWire();

    static Sketch GrooveFace() => Off(MouthFace(), P.SEAL_LAND + P.GROOVE_W) - Off(MouthFace(), P.SEAL_LAND);

    public static double OringLength() => OringCentreline().Length;

    static Sketch DoorOutline() => Off(ArchHull(), P.WALL);

    static Sketch FlangeOutline() => Off(ArchHull(), P.WALL + P.FLANGE_RAISE);

    /// <summary>A box on a facet: s = +1 or -1 for the +y or -y side.</summary>
    static Part FBox(int s, double x0, double x1, double u0, double u1, double n0, double n1)
    {
        var plane = new Plane(Point3D.Origin, new Vector3D(1, 0, 0), new Vector3D(0, s * S60, C60));
        return new Location(plane) * new Pos((x0 + x1) / 2, -s * (u0 + u1) / 2, (n0 + n1) / 2) * new Box(x1 - x0, Math.Abs(u1 - u0), n1 - n0);
    }

    public static Sketch PocketFace(int s)
    {
        var (u0, u1) = P.POCKET_U;
        var c = P.POCKET_CHAMFER;
        return Poly([Yz(P.N_FLOOR, u0, s), Yz(P.POCKET_N1, u0, s), Yz(P.POCKET_N1, u1 - c, s), Yz(P.POCKET_N1 - c, u1, s), Yz(P.N_FLOOR, u1, s)]);
    }

    // --- along the door, each side from the mouth

    public sealed record Sled(int Side, double Front, double Back, List<(double X0, double X1)> Stops,
        Dictionary<string, (double X0, double X1)> Boards);

    public sealed record SledLayout(double BatX0, Sled Xiao, Sled Bmm)
    {
        public Sled this[string name] => name == "sled_xiao" ? Xiao : name == "sled_bmm" ? Bmm : throw new ArgumentException(name);
    }

    /// <summary>Along each sled, from the door's mouth: x ranges of its boards, stops and ends.</summary>
    public static SledLayout Sleds()
    {
        var w = P.SLED_STOP_W;
        var batX0 = (P.BODY_L - P.FRONT_WALL) - P.BAT_L;
        var xiaoX1 = P.USB_TIP_X - P.USB_OUT;
        var xiaoX0 = xiaoX1 - P.XIAO_L;
        var bmiX1 = xiaoX0 - w;
        var bmiX0 = bmiX1 - P.SHUTTLE_L;
        var flashX1 = batX0 - 0.5 - w;
        var flashX0 = flashX1 - P.FLASH_L;
        var bmmX1 = flashX0 - w;
        var bmmX0 = bmmX1 - P.SHUTTLE_L;
        return new SledLayout(batX0,
            new Sled(1, P.USB_TIP_X - 0.2, bmiX0 - w, [(xiaoX0 - w, xiaoX0)],
                new() { ["xiao"] = (xiaoX0, xiaoX1), ["bmi330"] = (bmiX0, bmiX1) }),
            new Sled(-1, batX0 - 0.5, bmmX0 - w, [(flashX1, flashX1 + w), (flashX0 - w, flashX0)],
                new() { ["flash"] = (flashX0, flashX1), ["bmm350"] = (bmmX0, bmmX1) }));
    }

    public static double SledDetentX(string name)
    {
        var sl = Sleds()[name];
        return (sl.Front + sl.Back) / 2;
    }

    public static double DoorLen()
    {
        var lay = Sleds();
        return -Math.Min(lay.Xiao.Back, lay.Bmm.Back) + P.DOOR_END_GAP + P.WALL;
    }

    public static double XSleeve() => -(DoorLen() + P.BUMPER_T + P.SLEEVE_GAP);

    public static double ShaftR(double x) => P.SHAFT_D / 2 + P.TAPER * (x - XSleeve());

    static Part Cone(double x0, double x1, double extra = 0.0) =>
        new Pos(x0, 0, 0) * new Rotation(0, 90, 0) * new Kerf.Modeling.Cone(ShaftR(x0) + extra, ShaftR(x1) + extra, x1 - x0, align: MIN);

    public static double XTip() => P.BODY_L + P.ARM_OUT;

    public static double XCoil() => XTip() + P.COIL_PAST_TIP;

    static double ZCoilBottom() => ZArm() - P.COIL_T / 2;

    static double ZPlateTop() => ZCoilBottom() - P.COIL_AIR - P.COIL_WALL;

    static double ZChamberFloor() => ZPlateTop() + P.COIL_WALL;

    static double ZChamberTop() => ZCoilBottom() + P.COIL_T + P.CHAMBER_ABOVE;

    static double XChamberEnd() => XCoil() + P.COIL_HEAD_D / 2 + P.CHAMBER_END;

    public static (double X0, double X1) XHousing() => (XCoil() - P.HOUSING_L / 2, XCoil() + P.HOUSING_L / 2);

    // --- the boot's section: the arm with air round it

    public static (double W, double Z0, double Z1) BootInner()
    {
        var w = P.ARM_D + 2 * P.AIR_SIDE;
        return (w, ZArm() - P.ARM_D / 2 - P.AIR_BOTTOM, ZArm() + P.ARM_D / 2 + P.AIR_TOP);
    }

    static Sketch BootInFace()
    {
        var (w, z0, z1) = BootInner();
        return Rr(w, z0, z1, 3.6);
    }

    /// <summary>The boot's and sockets' undersides: an arc round the bare shaft, at its widest under them.</summary>
    public static double BootArcR() => ShaftR(XHousing().X0) + P.BOOT_SHAFT_GAP;

    /// <summary>
    /// The sockets' undersides: under their floor, under the boot's thin floor. Both sockets sit by a clamp, where the
    /// shaft hardly moves relative to them, so they can be closer to it than the boot's middle.
    /// </summary>
    static double SocketArcR() => BootInner().Z0 - P.BOOT_END_FLOOR - P.SOCKET_CLR - P.SOCKET_FLOOR;

    /// <summary>The arm's section grown by t: rounded on top, vertical sides down to the shaft, an arc underneath.</summary>
    static Sketch Hugging(double t, double cornerR, double chamfer = 0.0, double? arcR = null)
    {
        var (w, _, z1) = BootInner();
        var half = w / 2 + t;
        var rg = arcR ?? BootArcR();
        var f = Rr(2 * half, 2.0, z1 + t, cornerR) - Circle2(rg);
        if (chamfer > 0)
        {
            var zc = Math.Sqrt(rg * rg - half * half);
            var za = Math.Sqrt(rg * rg - (half - chamfer) * (half - chamfer));
            foreach (var s in new[] { 1, -1 })
                f -= Poly([(s * half, zc + chamfer), (s * (half - chamfer), za), (s * (half + 2), zc - 2), (s * (half + 2), zc + chamfer)]);
        }
        return f;
    }

    /// <summary>The boot's ends, and (with the glue gap) the sockets' holes: a convex rounded rectangle.</summary>
    public static Sketch BootEndFace(double extra = 0.0)
    {
        var (w, z0, z1) = BootInner();
        var half = w / 2 + P.BOOT_WALL + extra;
        return Rr(2 * half, z0 - P.BOOT_END_FLOOR - extra, z1 + P.BOOT_WALL + extra, P.BOOT_END_R + extra);
    }

    static double OuterT() => P.BOOT_WALL + P.SOCKET_CLR + P.SOCKET_WALL;

    static Sketch SocketOutFace() => Hugging(OuterT(), 6.0, P.BOOT_CHAMFER, SocketArcR());

    static Sketch BootThickFace() => Hugging(OuterT(), 6.0, P.BOOT_CHAMFER);

    /// <summary>A ring round the boot's section, on its top and sides only.</summary>
    static Part Rib(double extra, double x0, double x1) =>
        Ex(BootEndFace(extra), x0, x1) & (new Pos((x0 + x1) / 2, 0, P.RIB_LOW_Z) * new Box(x1 - x0 + 1, 60, 60, align: MIN));

    /// <summary>
    /// The solids to add and cut for a ribbed socket running from its mouth, away from the boot's free length.
    /// sign = -1: the socket runs towards -x from its mouth (on the body bumper); +1: towards +x (on the housing).
    /// </summary>
    static (Part Outer, Part Inner) Socket(double xMouth, int sign)
    {
        double x0 = Math.Min(xMouth, xMouth + sign * P.SOCKET_L), x1 = Math.Max(xMouth, xMouth + sign * P.SOCKET_L);
        var outer = Ex(SocketOutFace(), x0, x1);
        var inner = Ex(BootEndFace(P.SOCKET_CLR), x0 - 0.01, x1 + 0.01);
        foreach (var at in P.RIB_AT)
        {
            double a = xMouth + sign * at, b = xMouth + sign * (at + P.RIB_L);
            inner += Rib(P.GROOVE_H, Math.Min(a, b) - 0.2, Math.Max(a, b) + 0.2);
        }
        return (outer, inner);
    }

    static (double X0, double X1) BootX() => (P.BODY_L + P.BUMPER_T + 0.2, P.BODY_L + P.ARM_OUT - 1.0 - 0.2);

    // ---------------------------------------------------------------- parts

    static Part BodyPart()
    {
        var r = BodyBoreR();
        var b = Ex(BodySection(), 0, P.BODY_L) + Ex(FlangeOutline(), 0, P.BODY_FLANGE_L);
        b &= Above(-P.SPLIT_Z);
        b -= Ex(UBore(r), -1, P.BODY_L + 1);
        b -= Ex(BoreChamfers(r, -P.SPLIT_Z), -1, P.BODY_L + 1);
        foreach (var s in new[] { 1, -1 })
            b -= Ex(PocketFace(s), -1, P.BODY_L - P.FRONT_WALL);
        var za = ZArm();
        b -= Ex(Circle2(P.SOCKET_D / 2, 0, za), P.RIM_T, P.BODY_L + 1);
        b -= Ex(Circle2(P.RIM_HOLE / 2, 0, za), -1, P.RIM_T + 1);
        b -= Ex(GrooveFace(), -1, P.GROOVE_DEPTH);
        foreach (var (y, z) in DoorScrews())
            b -= Ex(Circle2(P.M3_HOLE / 2, y, z), -1, P.BODY_L + 1);
        foreach (var (x, y) in ClampBolts())
            b -= new Pos(x, y, -P.SPLIT_Z - 0.1) * new Cylinder(P.M3_INSERT_D / 2, P.M3_INSERT_HOLE + 0.1, align: MIN);
        return b;
    }

    public static List<(double X, double Y)> ClampBolts()
    {
        double[] xs = [P.BODY_L / 2 - P.BOLT_PITCH / 2, P.BODY_L / 2 + P.BOLT_PITCH / 2];
        return [.. new[] { 1, -1 }.SelectMany(s => xs.Select(x => (x, s * P.CLAMP_Y)))];
    }

    /// <summary>The cap under the shaft.</summary>
    static Part ClampPart()
    {
        var c = Ex(BodySection(), 0, P.BODY_L) & Below(-P.SPLIT_Z - P.SPLIT_GAP);
        c -= Ex(Circle2(BodyBoreR()), -1, P.BODY_L + 1);
        foreach (var (x, y) in ClampBolts())
        {
            c -= new Pos(x, y, -P.CLAMP_BOTTOM - 1) * new Cylinder(P.M3_HOLE / 2, 30.0, align: MIN);
            c -= new Pos(x, y, -P.CLAMP_BOTTOM - 10) * new Cylinder(P.M3_CBORE_D / 2, P.CLAMP_SEAT_Z + P.CLAMP_BOTTOM + 10, align: MIN);
        }
        return c;
    }

    static Part DoorPart()
    {
        var bore = UBore(P.DOOR_BORE_R);
        var L = DoorLen();
        var d = Ex(DoorOutline() - bore, -L, 0) + Ex(FlangeOutline() - bore, -P.DOOR_FLANGE_L, 0);
        d -= Ex(SealedFace(), -L + P.WALL, -P.DOOR_FLANGE_L);
        d -= Ex(MouthFace(), -P.DOOR_FLANGE_L - 0.01, 1);
        foreach (var (y, z) in DoorScrews())
        {
            d += Ex(Circle2(P.DOOR_BOSS_R, y, z), -P.DOOR_INSERT_DEPTH - 0.5, 0);
            d -= Ex(Circle2(P.M3_INSERT_D / 2, y, z), -P.DOOR_INSERT_DEPTH, 1);
        }
        // each sled runs under a lip on the lower wall and a lip on a rib along its upper side, back from the flange
        var (u0, u1) = P.SLED_U;
        var nRail = P.N_FLOOR + P.SLED_CLR + P.SLED_T;
        var nLip = nRail + P.SLED_RAIL_H + P.LIP_GAP;
        double xa = -L + P.WALL - 0.01, xb = -P.DOOR_FLANGE_L;
        foreach (var name in new[] { "sled_xiao", "sled_bmm" })
        {
            var s = Sleds()[name].Side;
            d += FBox(s, xa, xb, P.U_LOW - 0.5, u0 + P.LIP_OVER, nLip, nLip + P.LIP_T);
            var ur = u1 + P.SLED_SIDE_CLR;
            d += FBox(s, xa, xb, ur, ur + P.RIB_W, P.N_FLOOR - 5.0, nLip + P.LIP_T);   // rooted well into the floor
            var ua = Nu(PtA()).U;
            d += FBox(s, xa, xb, ua - 0.5, ur + 0.01, P.N_FLOOR - 5.0, P.N_FLOOR);       // the floor runs on to the rib
            d += FBox(s, xa, xb, u1 - P.LIP_OVER, ur, nLip, nLip + P.LIP_T);
            var xd = SledDetentX(name);
            d += FBox(s, xd - P.DETENT_L / 2, xd + P.DETENT_L / 2, ur - P.DETENT_BUMP, ur, nRail, nRail + P.SLED_RAIL_H);
        }
        return d;
    }

    /// <summary>A tray on one facet: a floor, side rails, fences across, pads under the shuttles' ends, a detent notch.</summary>
    static Part SledPart(string name)
    {
        var sl = Sleds()[name];
        var (s, xf, xb) = (sl.Side, sl.Front, sl.Back);
        var (u0, u1) = P.SLED_U;
        var n0 = P.N_FLOOR + P.SLED_CLR;
        var n1 = n0 + P.SLED_T;
        var t = FBox(s, xb, xf, u0, u1, n0, n1);
        var rail = n1 + P.SLED_RAIL_H;
        t += FBox(s, xb, xf, u0, u0 + P.SLED_RAIL_W, n1, rail);
        t += FBox(s, xb, xf, u1 - P.SLED_RAIL_W, u1, n1, rail);
        t += FBox(s, xb, xb + P.SLED_STOP_W, u0, u1, n1, rail);
        foreach (var (a, b) in sl.Stops)
            t += FBox(s, a, b, u0, u1, n1, rail);
        var (c0, c1) = Channel();
        foreach (var (board, (a, b)) in sl.Boards)
            if (board is "bmi330" or "bmm350")
                foreach (var (c, e) in new[] { (a, a + P.SLED_PAD_L), (b - P.SLED_PAD_L, b) })
                    t += FBox(s, c, e, c0, c1, n1, n1 + P.SHUTTLE_LIFT);   // between the rails, under the lips
        var xd = SledDetentX(name);
        t -= FBox(s, xd - P.DETENT_L / 2 - 0.25, xd + P.DETENT_L / 2 + 0.25, u1 - P.DETENT_BUMP + 0.1, u1 + 0.1, n1 - 0.01, rail + 0.01);
        return t;
    }

    static Part BodyBumperPart()
    {
        double x0 = P.BODY_FLANGE_L + P.BUMPER_CLR, x1 = P.BODY_L + P.BUMPER_T;
        Part Shape(double extra, double a0, double a1) => Ex(Off(BodySection(), extra), a0, a1);

        var b = Shape(P.BUMPER_CLR + P.BUMPER_T, x0, x1);
        b -= Shape(P.BUMPER_CLR, x0 - 1, P.BODY_L + P.BUMPER_CLR);
        b -= Ex(Circle2(P.BUMPER_BORE_R), x0 - 1, x1 + 1);
        b -= Ex(BootInFace(), P.BODY_L - 1, x1 + 1);
        b -= Below(-P.SPLIT_Z - P.BUMPER_SKIRT);   // a little over the cap's top, round the lower door screws' holes
        foreach (var (y, z) in DoorScrews())
            b -= Ex(Circle2(P.BUMPER_HOLE_R, y, z), P.BODY_L - 1, x1 + 1);
        var (outer, inner) = Socket(x1 + P.SOCKET_L, -1);
        return (b + outer) - inner;
    }

    /// <summary>The bumpers' outline, along the body and door: nothing hard should stick out of it.</summary>
    public static Part BumperEnvelope()
    {
        var t = P.WALL + P.BUMPER_CLR + P.BUMPER_T;
        return Ex(Off(ArchHull(), t), -DoorLen() - t - 1, P.BODY_L + P.BUMPER_T + 1);
    }

    static Part DoorBumperPart()
    {
        var L = DoorLen();
        double x0 = -L - P.BUMPER_CLR - P.BUMPER_T, x1 = -P.DOOR_FLANGE_L - P.BUMPER_CLR;
        var innerFace = Off(DoorOutline(), P.BUMPER_CLR);
        var b = Ex(Off(innerFace, P.BUMPER_T), x0, x1);
        b -= Ex(innerFace, -L - P.BUMPER_CLR, x1 + 1);
        b -= Ex(UBore(P.BUMPER_DOOR_BORE_R), x0 - 1, x1 + 1);
        return b;
    }

    static Part BootPart()
    {
        var (x0, x1) = BootX();
        var b = Ex(BootEndFace(), x0, x1);
        var mouth0 = P.BODY_L + P.BUMPER_T + P.SOCKET_L;
        var mouth1 = XHousing().X0 - P.SOCKET_L;
        // between the sockets, as thick as their outsides: flush, and more TPU over the arm (JP)
        b += Ex(BootThickFace(), mouth0 + 0.1, mouth1 - 0.1);
        foreach (var at in P.RIB_AT)
        {
            b += Rib(P.RIB_H, mouth0 - at - P.RIB_L, mouth0 - at);
            b += Rib(P.RIB_H, mouth1 + at, mouth1 + at + P.RIB_L);
        }
        b -= Ex(BootInFace(), x0 - 1, x1 + 1);
        return b;
    }

    static readonly Lazy<Part> bootTower = new(() =>
    {
        var x1 = BootX().X1;
        var xl = XHousing().X0 - P.SOCKET_L - 0.1;   // the ledge: the thick middle's end over the bottom socket end
        var mid = BootThickFace();
        var outer = Off(mid, P.TOWER_GAP + P.TOWER_WALL);
        var wall = Ex(outer - Off(mid, P.TOWER_GAP), xl - P.TOWER_COLLAR, xl + P.TOWER_ZGAP);
        var shelf = Ex(outer - Off(BootEndFace(P.RIB_H), P.TOWER_GAP), xl + P.TOWER_ZGAP, x1);
        return wall + shelf;
    });

    /// <summary>
    /// Printed with the boot, on end, then cut away: a collar TOWER_GAP off its thick middle for TOWER_COLLAR above the
    /// ledge, and under the ledge where that middle overhangs the bottom socket end (printed on end, the housing end is down) a shelf up
    /// to TOWER_ZGAP below it, clear of the socket end and its ribs by TOWER_GAP.
    /// </summary>
    public static Part BootTower() => bootTower.Value;

    static readonly Lazy<IReadOnlyDictionary<string, Part>> printAids = new(() => new Dictionary<string, Part> { ["boot_tower"] = BootTower() });

    /// <summary>Printed with a part and thrown away; not in the assembly.</summary>
    public static IReadOnlyDictionary<string, Part> PrintAids => printAids.Value;

    static Part SaddleBox(double clr = 0.0)
    {
        var x0 = XCoil() - P.PLATE_L / 2;
        var x1 = XCoil() + P.PLATE_L / 2 + P.SADDLE_END;
        var z1 = ZPlateTop();
        return new Pos((x0 + x1) / 2, 0, (P.SADDLE_LOW_Z + z1) / 2) * new Box(x1 - x0 + 2 * clr, P.SADDLE_W + 2 * clr, z1 - P.SADDLE_LOW_Z);
    }

    static Part SaddlePart() => SaddleBox() - Cone(XCoil() - 20, XCoil() + 20, P.SEAT_CLR) - Plate(extra: 0.1);

    static Part Plate(double extra = 0.0)
    {
        var x0 = XCoil() - P.PLATE_L / 2 - extra;
        return new Pos(x0, 0, ZPlateTop() - P.PLATE_T - extra) *
               new Box(P.PLATE_L + 2 * extra, P.PLATE_L + 2 * extra, P.PLATE_T + 2 * extra, align: new Alignment(Align.Min, Align.Center, Align.Min));
    }

    // --- the housing's outside: its section, the end chamfered, and a hull up to the boot's socket

    /// <summary>
    /// Points round a sketch's outline in (y, z), in order, as a polygon or a hull's station: each line's ends, and
    /// each curve at least every 0.25 mm. An edge's start is the last one's end, so it's left out.
    /// </summary>
    static List<(double Y, double Z)> Ring(Sketch s)
    {
        var pts = new List<(double Y, double Z)>();
        double Dist((double Y, double Z) a, (double Y, double Z) b) => Math.Max(Math.Abs(a.Y - b.Y), Math.Abs(a.Z - b.Z));
        foreach (var e in s.Faces()[0].OuterWire().Edges())
        {
            int n = e.GeomType == GeomType.Line ? 1 : Math.Max(4, (int)Math.Ceiling(e.Length / 0.25));
            var along = Enumerable.Range(0, n + 1).Select(i => e.PositionAt(i / (double)n)).Select(p => (p.X, p.Y)).ToList();
            if (pts.Count > 0 && Dist(pts[^1], along[^1]) < Dist(pts[^1], along[0]))
                along.Reverse();   // the edge runs against the wire
            pts.AddRange(pts.Count == 0 ? along : along.Skip(1));
        }
        pts.RemoveAt(pts.Count - 1);   // back at the start
        return pts;
    }

    static IEnumerable<Point3D> At(double x, IEnumerable<(double Y, double Z)> ring) => ring.Select(p => new Point3D(x, p.Y, p.Z));

    static Part HullOf(params IEnumerable<Point3D>[] stations) => new ConvexPolyhedron(stations.SelectMany(s => s));

    static readonly Lazy<Part> housingPrism = new(() =>
    {
        // The whole section (cap included) along the housing, its far end chamfered 45 deg all round. The chamfer is
        // longer than the section's 1.5 mm corner arcs, so they collapse to points (a Kerf addition: OCCT refuses it,
        // and blade_bumper.py builds this end as a hull of points round the section and its inset).
        var (x0, x1) = XHousing();
        var prism = Ex(HousingSection(), x0, x1);
        var end = prism.Edges().GroupBy(Axis.X).Last();
        return Chamfer(prism, end, P.HOUSING_END_C, options: new ChamferOptions { TightCorners = TightCorners.Collapse }).Ok("housing end chamfer");
    });

    static readonly Lazy<Part> housingEnvelope = new(() =>
    {
        var x0 = XHousing().X0;
        var sock = Ring(SocketOutFace());
        var upper = Ring(HousingSection() & Poly([(-60, -P.HOUSING_SPLIT_Z), (60, -P.HOUSING_SPLIT_Z), (60, 60), (-60, 60)]));
        var ramp = HullOf(At(x0, sock), At(x0, upper), At(x0 + P.HOUSING_RAMP, upper));
        var whole = Ex(Poly(sock), x0 - P.SOCKET_L, x0) + ramp + housingPrism.Value;   // the socket: the ramp's start, exactly
        return whole & Above(-P.HOUSING_SPLIT_Z);
    });

    /// <summary>
    /// The housing's outside, above its split: the boot's socket, then the convex hull of the socket and the housing's
    /// section over HOUSING_RAMP, then that section, its far end chamfered. Every cross-section is convex.
    /// </summary>
    public static Part HousingEnvelope() => housingEnvelope.Value;

    static Part HousingPart()
    {
        var (x0, x1) = XHousing();
        var h = HousingEnvelope();
        var (_, inner) = Socket(x0 - P.SOCKET_L, 1);
        // the shaft, the saddle's pocket, the chamber and the boot's inner section running into it
        h -= inner;
        h -= Cone(x0 - P.SOCKET_L - 1, x1 + 1, P.SEAT_CLR);
        var rw = ShaftR(x1) + P.SEAT_CLR;   // straight below the axis, at the widest the taper gets here
        h -= Ex(Poly([(-rw, 0), (rw, 0), (rw, -50), (-rw, -50)]), x0 - 1, x1 + 1);
        h -= Ex(BoreChamfers(rw, -P.HOUSING_SPLIT_Z), x0 - 1, x1 + 1);
        h -= SaddleBox(clr: P.SEAT_CLR);
        h -= Ex(BootInFace(), x0 - 1, P.STADIUM_END);
        double y0 = 7.0, y1 = P.CHAMBER_Y, xe = XChamberEnd();
        var plan = Poly([(x0, y0), (x0 + 4.8, y1), (xe, y1), (xe, -y1), (x0 + 4.8, -y1), (x0, -y0)]);   // (x, y) here
        h -= new Pos(0, 0, ZChamberFloor()) * ExZ(plan, ZChamberTop() - ZChamberFloor());
        // nuts drop down shafts from the top onto a ledge; the bolts come up through it
        var zs = -P.HOUSING_SPLIT_Z;
        foreach (var (x, y) in HousingBolts())
        {
            h -= new Pos(x, y, P.HOUSING_NUT_Z) * ExZ(Hexagon(P.NUT_SHAFT_AF), 80.0);
            h -= new Pos(x, y, zs - 0.1) * new Cylinder(P.M3_HOLE / 2, P.HOUSING_NUT_Z - zs + 0.2, align: MIN);
        }
        return h;
    }

    /// <summary>The TPU plug glued into a nut's shaft, trimmed flush with the housing's outside. Four alike.</summary>
    static Part HousingPlug(int i = 0)
    {
        var (x, y) = HousingBolts()[i];
        var z0 = P.HOUSING_NUT_Z + P.M3_NUT_H + P.PLUG_GAP;
        return (new Pos(x, y, z0) * ExZ(Hexagon(P.PLUG_AF), 80.0)) & HousingEnvelope();
    }

    static Sketch Hexagon(double af) => new RegularPolygon(af / 2 / Math.Cos(Math.PI / 6), 6);

    public static List<(double X, double Y)> HousingBolts()
    {
        var xc = XCoil();
        double[] xs = [xc - P.BOLT_PITCH / 2, xc + P.BOLT_PITCH / 2];
        return [.. new[] { 1, -1 }.SelectMany(s => xs.Select(x => (x, s * P.HOUSING_BOLT_Y)))];
    }

    /// <summary>The cap under the shaft at the coil end.</summary>
    static Part HousingClampPart()
    {
        var (x0, x1) = XHousing();
        var c = housingPrism.Value & Below(-P.HOUSING_SPLIT_Z - P.HOUSING_GAP);
        c -= Cone(x0 - 1, x1 + 1, P.SEAT_CLR);
        foreach (var (x, y) in HousingBolts())
        {
            c -= new Pos(x, y, -P.HOUSING_BOTTOM - 1) * new Cylinder(P.M3_HOLE / 2, 30.0, align: MIN);
            c -= new Pos(x, y, -P.HOUSING_BOTTOM - 10) * new Cylinder(P.M3_CBORE_D / 2, P.HOUSING_SEAT_Z + P.HOUSING_BOTTOM + 10, align: MIN);
        }
        return c;
    }

    /// <summary>Bonded in the arm's end; the coil board's tail is bonded in its slot.</summary>
    static Part AdapterPart()
    {
        double xt = XTip(), za = ZArm();
        var a = Ex(Circle2(P.SPIGOT_D / 2, 0, za), xt - P.SPIGOT_L, xt) + Ex(Circle2(P.ARM_D / 2, 0, za), xt, xt + P.COLLAR_L);
        var xs0 = XCoil() - P.COIL_TAIL - 0.1;
        a -= new Pos((xs0 + xt + P.COLLAR_L + 1) / 2, 0, za) * new Box(xt + P.COLLAR_L + 1 - xs0, P.SLOT_W, P.SLOT_T);
        a -= Ex(Circle2(1.8, 0, za), xt - P.SPIGOT_L - 1, xs0 + 0.5);
        return a;
    }

    static readonly Lazy<IReadOnlyDictionary<string, Part>> parts = new(() => new Dictionary<string, Part>
    {
        ["body"] = BodyPart(), ["clamp"] = ClampPart(), ["door"] = DoorPart(),
        ["sled_xiao"] = SledPart("sled_xiao"), ["sled_bmm"] = SledPart("sled_bmm"),
        ["body_bumper"] = BodyBumperPart(), ["door_bumper"] = DoorBumperPart(), ["boot"] = BootPart(),
        ["housing"] = HousingPart(), ["housing_clamp"] = HousingClampPart(), ["housing_plug"] = HousingPlug(),
        ["saddle"] = SaddlePart(), ["adapter"] = AdapterPart(),
    });

    /// <summary>The printed parts, in assembly position.</summary>
    public static IReadOnlyDictionary<string, Part> Parts => parts.Value;
}
