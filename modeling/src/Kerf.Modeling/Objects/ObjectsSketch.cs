namespace Kerf.Modeling;

/// <summary>
/// The base of the sketch objects (<see cref="Rectangle"/>, <see cref="Circle"/>, …): a sketch of faces, aligned in X
/// and Y by their bounding box and then rotated about Z.
/// </summary>
public abstract class BaseSketchObject : Sketch, IBuilderObject
{
    /// <summary>A sketch object from a face or compound of faces.</summary>
    /// <param name="obj">The faces.</param>
    /// <param name="rotation">An angle in degrees to rotate them about Z.</param>
    /// <param name="align">How to align their bounding box in X and Y; null for no alignment.</param>
    /// <param name="mode">How the object combines with a builder's sketch.</param>
    protected BaseSketchObject(Shape obj, double rotation = 0, Alignment? align = null, Mode mode = Mode.Add)
        : base(Prepare(obj, rotation, align).Select(f => (Shape)f))
    {
        Rotation = rotation;
        Mode = mode;
    }

    private static ShapeList<Face> Prepare(Shape obj, double rotation, Alignment? align)
    {
        if (align is { } a)
            obj = obj.Moved(new Location(obj.BoundingBox().ToAlignOffset(TwoAxes(a))));
        return rotation != 0 ? obj.Moved(new Rotation(0, 0, rotation)).Faces() : obj.Faces();
    }

    /// <summary>An alignment on X and Y only, as build123d's sketches take it.</summary>
    internal static Alignment TwoAxes(Alignment a) => a.Count == 1 ? (a[0]!.Value, a[0]!.Value) : a.Count == 3 ? (a[0]!.Value, a[1]!.Value) : a;

    /// <summary>The rotation applied, in degrees about Z.</summary>
    public double Rotation { get; }

    /// <inheritdoc/>
    public Mode Mode { get; }

    /// <summary>A centred alignment in X and Y, the default of most sketch objects.</summary>
    internal static readonly Alignment Centered = (Align.Center, Align.Center);
}

/// <summary>A circle, or a sector of one; centred by default.</summary>
public class Circle : BaseSketchObject
{
    /// <summary>A circle of a radius; <paramref name="arcSize"/> below 360 makes a sector.</summary>
    public Circle(double radius, double arcSize = 360.0, Alignment? align = null, Mode mode = Mode.Add)
        : base(Make(radius, arcSize), 0, align ?? Centered, mode)
    {
        (Radius, ArcSize, Align) = (radius, arcSize, TwoAxes(align ?? Centered));
    }

    private static Face Make(double radius, double arcSize) =>
        arcSize == 360.0 ? new Face(Wire.MakeCircle(radius)) : Face.Revolve(Edge.MakeLine((radius, 0), (0, 0)), arcSize, Axis.Z);

    /// <summary>The radius.</summary>
    public double Radius { get; }

    /// <summary>The angular size in degrees.</summary>
    public double ArcSize { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>An ellipse of an x and y radius; centred by default.</summary>
public class Ellipse : BaseSketchObject
{
    /// <summary>An ellipse, rotated by <paramref name="rotation"/> degrees.</summary>
    public Ellipse(double xRadius, double yRadius, double rotation = 0, Alignment? align = null, Mode mode = Mode.Add)
        : base(new Face(Wire.MakeEllipse(xRadius, yRadius)), rotation, align ?? Centered, mode)
    {
        (XRadius, YRadius, Align) = (xRadius, yRadius, TwoAxes(align ?? Centered));
    }

    /// <summary>The radius along X.</summary>
    public double XRadius { get; }

    /// <summary>The radius along Y.</summary>
    public double YRadius { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>
/// A polygon through points; not aligned by default. In algebra mode the points' order sets the face's normal:
/// counter-clockwise points face up.
/// </summary>
public class Polygon : BaseSketchObject
{
    /// <summary>A polygon through the points.</summary>
    public Polygon(IEnumerable<Vector> pts, double rotation = 0, Alignment? align = null, Mode mode = Mode.Add)
        : this(pts.ToList(), rotation, align, mode) { }

    /// <summary>A polygon through the points.</summary>
    public Polygon(params Vector[] pts) : this((IEnumerable<Vector>)pts) { }

    private Polygon(List<Vector> pts, double rotation, Alignment? align, Mode mode)
        : base(new Face(Wire.MakePolygon(pts)), rotation, align ?? (Kerf.Modeling.Align.None, Kerf.Modeling.Align.None), mode)
    {
        (Pts, Align) = (pts, TwoAxes(align ?? (Kerf.Modeling.Align.None, Kerf.Modeling.Align.None)));
    }

    /// <summary>The points.</summary>
    public IReadOnlyList<Vector> Pts { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>A rectangle of a width (along X) and height (along Y); centred by default.</summary>
public class Rectangle : BaseSketchObject
{
    /// <summary>A rectangle, rotated by <paramref name="rotation"/> degrees.</summary>
    public Rectangle(double width, double height, double rotation = 0, Alignment? align = null, Mode mode = Mode.Add)
        : base(Face.MakeRect(width, height), rotation, align ?? Centered, mode)
    {
        (Width, RectangleHeight, Align) = (width, height, TwoAxes(align ?? Centered));
    }

    /// <summary>The size along X.</summary>
    public double Width { get; }

    /// <summary>The size along Y (build123d's <c>rectangle_height</c>).</summary>
    public double RectangleHeight { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>A rectangle with its corners filleted to a radius; centred by default.</summary>
public class RectangleRounded : BaseSketchObject
{
    /// <summary>A rounded rectangle.</summary>
    /// <exception cref="ArgumentException">The width or height is no more than twice the radius.</exception>
    public RectangleRounded(double width, double height, double radius, double rotation = 0, Alignment? align = null, Mode mode = Mode.Add)
        : base(Make(width, height, radius), rotation, align ?? Centered, mode)
    {
        (Width, RectangleHeight, Radius, Align) = (width, height, radius, TwoAxes(align ?? Centered));
    }

    private static Face Make(double width, double height, double radius)
    {
        if (width <= 2 * radius || height <= 2 * radius)
            throw new ArgumentException("width and height must be > 2*radius");
        var face = Face.MakeRect(width, height);
        return face.Fillet2D(radius, face.Vertices());
    }

    /// <summary>The size along X.</summary>
    public double Width { get; }

    /// <summary>The size along Y.</summary>
    public double RectangleHeight { get; }

    /// <summary>The corner radius.</summary>
    public double Radius { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>
/// A regular polygon of a radius and side count: <c>majorRadius</c> makes the radius that of the circumscribed
/// circle (through the vertices), else of the inscribed one (touching the sides). Centred by default.
/// </summary>
public class RegularPolygon : BaseSketchObject
{
    /// <summary>A regular polygon, its first vertex at <paramref name="rotation"/> degrees.</summary>
    /// <exception cref="ArgumentException">Fewer than three sides.</exception>
    public RegularPolygon(double radius, int sideCount, bool majorRadius = true, double rotation = 0, Alignment? align = null,
                          Mode mode = Mode.Add)
        : base(Make(radius, sideCount, majorRadius, rotation, TwoAxes(align ?? Centered)), 0, null, mode)
    {
        Radius = majorRadius ? radius : radius / Math.Cos(Math.PI / sideCount);
        Apothem = Radius * Math.Cos(Math.PI / sideCount);
        SideCount = sideCount;
        Align = TwoAxes(align ?? Centered);
    }

    private static Face Make(double radius, int sideCount, bool majorRadius, double rotation, Alignment align)
    {
        if (sideCount < 3)
            throw new ArgumentException($"RegularPolygon must have at least three sides, not {sideCount}");
        var rad = majorRadius ? radius : radius / Math.Cos(Math.PI / sideCount);
        var pts = Enumerable.Range(0, sideCount + 1)
            .Select(i => new Vector(rad * Math.Cos(i * 2 * Math.PI / sideCount + rotation * Constants.DegToRad),
                                    rad * Math.Sin(i * 2 * Math.PI / sideCount + rotation * Constants.DegToRad))).ToList();
        var byX = pts.OrderBy(p => p.X).ToList();
        var byY = pts.OrderBy(p => p.Y).ToList();
        var offset = align.Offset(new Vector(byX[0].X, byY[0].Y), new Vector(byX[^1].X, byY[^1].Y), new Vector(0, 0));
        return new Face(Wire.MakePolygon(pts.Select(p => p + offset)));
    }

    /// <summary>The radius of the circumscribed circle (the major radius).</summary>
    public double Radius { get; }

    /// <summary>The radius of the inscribed circle (the minor radius).</summary>
    public double Apothem { get; }

    /// <summary>The number of sides.</summary>
    public int SideCount { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>A slot along a centre line (any edge or wire: an arc, a line, a spline, …) of a height (its width across).</summary>
public class SlotCurve : BaseSketchObject
{
    /// <summary>A slot round a curve.</summary>
    public SlotCurve(Shape1D curve, double height, double rotation = 0, Mode mode = Mode.Add)
        : base(new Face((Wire)(curve as Wire ?? new Wire((Edge)curve)).Offset2D(height / 2)), rotation, null, mode)
    {
        (Curve, SlotHeight) = (curve, height);
    }

    /// <summary>The centre line.</summary>
    public Shape1D Curve { get; }

    /// <summary>The width across the slot.</summary>
    public double SlotHeight { get; }
}

/// <summary>A slot along an arc or other curve: use <see cref="SlotCurve"/>.</summary>
[Obsolete("SlotArc is deprecated; use SlotCurve instead.")]
public class SlotArc : SlotCurve
{
    /// <summary>A slot round a curve.</summary>
    public SlotArc(Shape1D arc, double height, double rotation = 0, Mode mode = Mode.Add) : base(arc, height, rotation, mode) => Arc = arc;

    /// <summary>The centre line.</summary>
    public Shape1D Arc { get; }
}

/// <summary>A slot from its centre and the centre of one end arc, symmetric about the centre.</summary>
public class SlotCenterPoint : BaseSketchObject
{
    /// <summary>A slot of a height (its width across).</summary>
    /// <exception cref="ArgumentException">The centre and point coincide.</exception>
    public SlotCenterPoint(Vector center, Vector point, double height, double rotation = 0, Mode mode = Mode.Add)
        : base(Make(center, point, height), rotation, null, mode)
    {
        (SlotCenter, Point, SlotHeight) = (center, point, height);
    }

    private static Face Make(Vector center, Vector point, double height)
    {
        var halfLine = point - center;
        if (halfLine.Length <= 0)
            throw new ArgumentException(
                $"Distance between center and point must be greater than 0 Got: distance = {Formatting.Number(halfLine.Length)} (computed)");
        var wires = Wire.Combine([Edge.MakeLine(point, center), Edge.MakeLine(center, center - halfLine)]);
        return new Face((Wire)wires[0].Offset2D(height / 2));
    }

    /// <summary>The centre of the slot.</summary>
    public Vector SlotCenter { get; }

    /// <summary>The centre of one end arc.</summary>
    public Vector Point { get; }

    /// <summary>The width across the slot.</summary>
    public double SlotHeight { get; }
}

/// <summary>A slot from the distance between the centres of its end arcs; a circle when that is zero.</summary>
public class SlotCenterToCenter : BaseSketchObject
{
    /// <summary>A slot of a height (its width across).</summary>
    /// <exception cref="ArgumentException">The separation is negative.</exception>
    public SlotCenterToCenter(double centerSeparation, double height, double rotation = 0, Mode mode = Mode.Add)
        : base(Make(centerSeparation, height, mode), rotation, null, mode)
    {
        (CenterSeparation, SlotHeight) = (centerSeparation, height);
    }

    private static Face Make(double separation, double height, Mode mode)
    {
        if (separation < 0)
            throw new ArgumentException($"The centre separation must be positive, not {Formatting.Number(separation)}.");
        if (separation > 0)
            return new Face((Wire)new Wire([Edge.MakeLine(new Vector(-separation / 2, 0, 0), new Vector(0, 0, 0)),
                                            Edge.MakeLine(new Vector(0, 0, 0), new Vector(separation / 2, 0, 0))]).Offset2D(height / 2));
        return new Circle(height / 2, mode: mode).SingleFace();
    }

    /// <summary>The distance between the centres of the end arcs.</summary>
    public double CenterSeparation { get; }

    /// <summary>The width across the slot.</summary>
    public double SlotHeight { get; }
}

/// <summary>A slot of an overall width and height; a circle when they are equal. Centred by default.</summary>
public class SlotOverall : BaseSketchObject
{
    /// <summary>A slot.</summary>
    /// <exception cref="ArgumentException">The width is less than the height.</exception>
    public SlotOverall(double width, double height, double rotation = 0, Alignment? align = null, Mode mode = Mode.Add)
        : base(Make(width, height, mode), rotation, align ?? Centered, mode)
    {
        (Width, SlotHeight) = (width, height);
    }

    private static Face Make(double width, double height, Mode mode)
    {
        if (width < height)
            throw new ArgumentException($"Slot requires that width > height. Got: width={Formatting.Number(width)}, height={Formatting.Number(height)}");
        if (width > height)
            return new Face((Wire)new Wire([Edge.MakeLine(new Vector(-width / 2 + height / 2, 0, 0), new Vector(0, 0, 0)),
                                            Edge.MakeLine(new Vector(0, 0, 0), new Vector(width / 2 - height / 2, 0, 0))]).Offset2D(height / 2));
        return new Circle(width / 2, mode: mode).SingleFace();
    }

    /// <summary>The overall width.</summary>
    public double Width { get; }

    /// <summary>The width across the slot.</summary>
    public double SlotHeight { get; }
}

/// <summary>
/// A superellipse ("squircle") of a width, height and order: order 1 is a diamond, 2 an ellipse, and higher orders
/// approach a rectangle. Centred by default.
/// </summary>
public class Superellipse : BaseSketchObject
{
    /// <summary>A superellipse; above order 2 each quadrant is a spline through <paramref name="pointCount"/> points.</summary>
    /// <exception cref="ArgumentException">The order isn't positive.</exception>
    public Superellipse(double width, double height, double order = 4, int pointCount = 16, double rotation = 0, Alignment? align = null,
                        Mode mode = Mode.Add)
        : base(Make(width, height, order, pointCount), rotation, align ?? Centered, mode)
    {
        (Width, Height, SuperOrder, PointCount, Align) = (width, height, order, pointCount, TwoAxes(align ?? Centered));
    }

    private static Face Make(double width, double height, double order, int pointCount)
    {
        if (order <= 0)
            throw new ArgumentException("Order must be greater than zero.");
        Edge topRight;
        if (order == 1)
            topRight = Edge.MakeLine((width / 2, 0), (0, height / 2));
        else if (order == 2)
            topRight = Edge.MakeEllipse(width / 2, height / 2, startAngle: 0, endAngle: 90);
        else
        {
            var points = new List<Vector> { new(width / 2, 0.0) };
            for (int i = 1; i < pointCount - 1; i++)
            {
                var t = i * Math.PI / pointCount / 2;
                double c = Math.Cos(t), s = Math.Sin(t);
                var x = Math.Pow(Math.Abs(c), 2 / order) * width * Math.CopySign(1, c) / 2;
                var y = Math.Pow(Math.Abs(s), 2 / order) * height * Math.CopySign(1, s) / 2;
                points.Add(new Vector(x, y));
            }
            points.Add(new Vector(0.0, height / 2));
            topRight = Edge.MakeSpline(points);
        }
        var topLeft = topRight.Mirror(Plane.YZ).Reversed();
        return new Face(new Wire([topRight, topLeft, topLeft.Mirror(Plane.XZ).Reversed(), topRight.Mirror(Plane.XZ).Reversed()]));
    }

    /// <summary>The width.</summary>
    public double Width { get; }

    /// <summary>The height (build123d's <c>height_</c>).</summary>
    public double Height { get; }

    /// <summary>The order (build123d's <c>order</c>).</summary>
    public double SuperOrder { get; }

    /// <summary>The number of points per quadrant.</summary>
    public int PointCount { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>
/// A trapezoid of a major (bottom) width, a height and its bottom interior angles (symmetric if only the left is given).
/// Centred by default.
/// </summary>
public class Trapezoid : BaseSketchObject
{
    /// <summary>A trapezoid.</summary>
    /// <exception cref="ArgumentException">The angles make an invalid trapezoid.</exception>
    public Trapezoid(double width, double height, double leftSideAngle, double? rightSideAngle = null, double rotation = 0,
                     Alignment? align = null, Mode mode = Mode.Add)
        : base(Make(width, height, leftSideAngle, rightSideAngle is { } r && r != 0 ? r : leftSideAngle), rotation, align ?? Centered, mode)
    {
        (Width, TrapezoidHeight, LeftSideAngle, RightSideAngle, Align) =
            (width, height, leftSideAngle, rightSideAngle is { } r2 && r2 != 0 ? r2 : leftSideAngle, TwoAxes(align ?? Centered));
    }

    private static Face Make(double width, double height, double left, double right)
    {
        var reductionLeft = left == 90 ? 0 : height / Math.Tan(left * Constants.DegToRad);
        var reductionRight = right == 90 ? 0 : height / Math.Tan(right * Constants.DegToRad);
        double topLeft = width / 2, topRight = width / 2, botLeft = width / 2, botRight = width / 2;
        if (reductionLeft > 0)
            topLeft -= reductionLeft;
        else
            botLeft += reductionLeft;
        if (reductionRight > 0)
            topRight -= reductionRight;
        else
            botRight += reductionRight;
        if (botLeft + botRight < 0)
            throw new ArgumentException("Trapezoid bottom invalid - change angles");
        if (topLeft + topRight < 0)
            throw new ArgumentException("Trapezoid top invalid - change angles");
        Vector[] pts = [new(-botLeft, -height / 2), new(botRight, -height / 2), new(topRight, height / 2), new(-topLeft, height / 2), new(-botLeft, -height / 2)];
        return new Face(Wire.MakePolygon(pts));
    }

    /// <summary>The major (bottom) width.</summary>
    public double Width { get; }

    /// <summary>The height.</summary>
    public double TrapezoidHeight { get; }

    /// <summary>The bottom left interior angle.</summary>
    public double LeftSideAngle { get; }

    /// <summary>The bottom right interior angle.</summary>
    public double RightSideAngle { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>
/// A triangle from one side length and any two other sides or interior angles: each angle is opposite the side of the same
/// letter, side a along the bottom and b on the right, counter-clockwise. Centred on its vertices' average; not aligned by
/// default.
/// </summary>
public class Triangle : BaseSketchObject
{
    /// <summary>A triangle; give one side and two other values (angles in degrees).</summary>
    /// <exception cref="ArgumentException">One length and two other values weren't given, or no such triangle exists.</exception>
    public Triangle(double? a = null, double? b = null, double? c = null, double? A = null, double? B = null, double? C = null,
                    Alignment? align = null, double rotation = 0, Mode mode = Mode.Add)
        : this(Solve(a, b, c, A, B, C), align, rotation, mode) { }

    private Triangle((double a, double b, double c, double A, double B, double C) t, Alignment? align, double rotation, Mode mode)
        : base(Make(t), rotation, align, mode)
    {
        (SideA, SideB, SideC) = (t.a, t.b, t.c);
        (AngleA, AngleB, AngleC) = (t.A * Constants.RadToDeg, t.B * Constants.RadToDeg, t.C * Constants.RadToDeg);
        var edges = Edges();
        EdgeA = edges.First(e => Math.Abs(e.Length - t.a) < Constants.Tolerance);
        EdgeB = edges.First(e => Math.Abs(e.Length - t.b) < Constants.Tolerance && e != EdgeA);
        EdgeC = edges.First(e => e != EdgeA && e != EdgeB);
        VertexA = (Vertex)Vertex.CommonVertex(EdgeB, EdgeC).WithTopoPath([this]);
        VertexB = (Vertex)Vertex.CommonVertex(EdgeA, EdgeC).WithTopoPath([this]);
        VertexC = (Vertex)Vertex.CommonVertex(EdgeA, EdgeB).WithTopoPath([this]);
    }

    private static (double, double, double, double, double, double) Solve(double? a, double? b, double? c, double? A, double? B, double? C)
    {
        double?[] all = [a, b, c, A, B, C];
        if (a is null && b is null && c is null || all.Count(v => v is null) != 3)
            throw new ArgumentException("Give one length and two other values.");
        double? Rad(double? deg) => deg * Constants.DegToRad;
        return TriangleSolver.Solve(a, b, c, Rad(A), Rad(B), Rad(C));
    }

    private static Face Make((double a, double b, double c, double A, double B, double C) t)
    {
        var triangle = new Face(Wire.MakePolygon([new Vector(0, 0), new Vector(t.a, 0), new Vector(t.c, 0).Rotate(Axis.Z, t.B * Constants.RadToDeg)]));
        var center = triangle.Vertices().Aggregate(new Vector(0, 0, 0), (sum, v) => sum + v.ToVector()) / 3;
        return triangle.Moved(new Location(-center));
    }

#pragma warning disable CS1591 // The triangle's sides, angles (in degrees), edges and vertices, by letter
    public double SideA { get; }
    public double SideB { get; }
    public double SideC { get; }
    public double AngleA { get; }
    public double AngleB { get; }
    public double AngleC { get; }
    public Edge EdgeA { get; }
    public Edge EdgeB { get; }
    public Edge EdgeC { get; }
    public Vertex VertexA { get; }
    public Vertex VertexB { get; }
    public Vertex VertexC { get; }
#pragma warning restore CS1591
}

/// <summary>
/// A port of Steven Byrnes' trianglesolver (MIT): the sides and angles (radians) of a triangle from any three, one of them a
/// side, by the laws of sines and cosines. An ambiguous side-side-angle case is an error, as with its "forbid" flag.
/// </summary>
internal static class TriangleSolver
{
    internal static (double a, double b, double c, double A, double B, double C) Solve(double? a, double? b, double? c, double? A, double? B, double? C)
    {
        double?[] sides = [a, b, c];
        double?[] angles = [A, B, C];
        if (sides.Concat(angles).Count(x => x is not null) != 3)
            throw new ArgumentException("Must provide exactly 3 inputs");
        if (sides.All(x => x is null))
            throw new ArgumentException("Must provide at least 1 side length");
        if (sides.Concat(angles).Any(x => x is <= 0) || angles.Any(x => x >= Math.PI))
            throw new ArgumentException("No such triangle");
        int known = sides.Count(x => x is not null);
        if (known == 3)
            return Sss(a!.Value, b!.Value, c!.Value);
        if (known == 2)
        {
            if (a is { } a1 && A is { } A1 && b is { } b1) return Ssa(a1, b1, A1);
            if (a is { } a2 && A is { } A2 && c is { } c2) { var (x, y, z, X, Y, Z) = Ssa(a2, c2, A2); return (x, z, y, X, Z, Y); }
            if (b is { } b3 && B is { } B3 && a is { } a3) { var (x, y, z, X, Y, Z) = Ssa(b3, a3, B3); return (y, x, z, Y, X, Z); }
            if (b is { } b4 && B is { } B4 && c is { } c4) { var (x, y, z, X, Y, Z) = Ssa(b4, c4, B4); return (z, x, y, Z, X, Y); }
            if (c is { } c5 && C is { } C5 && a is { } a5) { var (x, y, z, X, Y, Z) = Ssa(c5, a5, C5); return (y, z, x, Y, Z, X); }
            if (c is { } c6 && C is { } C6 && b is { } b6) { var (x, y, z, X, Y, Z) = Ssa(c6, b6, C6); return (z, y, x, Z, Y, X); }
            if (a is { } a7 && b is { } b7 && C is { } C7) return Sas(a7, b7, C7);
            if (b is { } b8 && c is { } c8 && A is { } A8) { var (x, y, z, X, Y, Z) = Sas(b8, c8, A8); return (z, x, y, Z, X, Y); }
            if (c is { } c9 && a is { } a9 && B is { } B9) { var (x, y, z, X, Y, Z) = Sas(c9, a9, B9); return (y, z, x, Y, Z, X); }
        }
        // One side and two angles
        double aa = A ?? Math.PI - B!.Value - C!.Value;
        double bb = B ?? Math.PI - aa - C!.Value;
        double cc = C ?? Math.PI - aa - bb;
        if (aa <= 0 || bb <= 0 || cc <= 0)
            throw new ArgumentException("No such triangle");
        if (c is { } cs)
            return Aaas(aa, bb, cc, cs);
        if (a is { } asd)
        {
            var (x, y, z, X, Y, Z) = Aaas(bb, cc, aa, asd);
            return (z, x, y, Z, X, Y);
        }
        {
            var (x, y, z, X, Y, Z) = Aaas(cc, aa, bb, b!.Value);
            return (y, z, x, Y, Z, X);
        }
    }

    private static (double, double, double, double, double, double) Aaas(double D, double E, double F, double f) =>
        (f * Math.Sin(D) / Math.Sin(F), f * Math.Sin(E) / Math.Sin(F), f, D, E, F);

    private static (double, double, double, double, double, double) Sss(double d, double e, double f)
    {
        if (!(d + e > f && e + f > d && f + d > e))
            throw new ArgumentException("No such triangle");
        var F = Math.Acos((d * d + e * e - f * f) / (2 * d * e));
        var E = Math.Acos((d * d + f * f - e * e) / (2 * d * f));
        return (d, e, f, Math.PI - F - E, E, F);
    }

    private static (double, double, double, double, double, double) Sas(double d, double e, double F) =>
        Sss(d, e, Math.Sqrt(d * d + e * e - 2 * d * e * Math.Cos(F)));

    private static (double, double, double, double, double, double) Ssa(double d, double e, double D)
    {
        var sinE = Math.Sin(D) * e / d;
        double E;
        if (TopologyUtils.IsCloseB(sinE, 1, absTol: 0))
            E = Math.PI / 2;
        else
        {
            if (!(sinE < 1))
                throw new ArgumentException("No such triangle");
            var acute = Math.Asin(sinE);
            var obtuse = Math.PI - acute;
            bool acuteValid = 0 < Math.PI - D - acute && Math.PI - D - acute < Math.PI;
            bool obtuseValid = 0 < Math.PI - D - obtuse && Math.PI - D - obtuse < Math.PI;
            if (acuteValid && obtuseValid)
                throw new ArgumentException("Two different triangles fit this description");
            if (!acuteValid && !obtuseValid)
                throw new ArgumentException("No such triangle");
            E = acuteValid ? acute : obtuse;
        }
        var F = Math.PI - D - E;
        var (e2, f2, d2, E2, F2, D2) = Aaas(E, F, D, d);
        return (d2, e2, f2, D2, E2, F2);
    }
}
