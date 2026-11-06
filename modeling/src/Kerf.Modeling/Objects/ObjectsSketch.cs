namespace Kerf.Modeling;

/// <summary>
/// The base of the sketch objects (<see cref="Rectangle"/>, <see cref="Circle"/>, …): a sketch of faces, aligned in X
/// and Y by their bounding box and then rotated about Z.
/// </summary>
public abstract class BaseSketchObject : Sketch
{
    /// <summary>A sketch object from a face or compound of faces.</summary>
    /// <param name="obj">The faces.</param>
    /// <param name="rotation">An angle in degrees to rotate them about Z.</param>
    /// <param name="align">How to align their bounding box in X and Y; null for no alignment.</param>
    protected BaseSketchObject(Shape obj, double rotation = 0, Alignment? align = null)
        : base(Prepare(obj, rotation, align).Select(f => (Shape)f))
    {
        Rotation = rotation;
    }

    private static ShapeList<Face> Prepare(Shape obj, double rotation, Alignment? align)
    {
        if (align is { } a)
            obj = obj.Moved(new Location(obj.Bounds().ToAlignOffset(TwoAxes(a))));
        return rotation != 0 ? obj.Moved(new Rotation(0, 0, rotation)).Faces() : obj.Faces();
    }

    /// <summary>An alignment on X and Y only, as build123d's sketches take it.</summary>
    internal static Alignment TwoAxes(Alignment a) => a.Count == 1 ? new Alignment(a[0]!.Value, a[0]!.Value) : a.Count == 3 ? new Alignment(a[0]!.Value, a[1]!.Value) : a;

    /// <summary>The rotation applied, in degrees about Z.</summary>
    public double Rotation { get; }

    /// <summary>A centred alignment in X and Y, the default of most sketch objects.</summary>
    internal static readonly Alignment Centered = new Alignment(Align.Center, Align.Center);
}

/// <summary>A circle, or a sector of one; centred by default.</summary>
public class Circle : BaseSketchObject
{
    /// <summary>A circle of a radius; <paramref name="arcSize"/> below 360 makes a sector.</summary>
    public Circle(double radius, double arcSize = 360.0, Alignment? align = null)
        : base(Make(radius, arcSize), 0, align ?? Centered)
    {
        (Radius, ArcSize, Align) = (radius, arcSize, TwoAxes(align ?? Centered));
    }

    private static Face Make(double radius, double arcSize) =>
        arcSize == 360.0 ? Face.Make(Wire.MakeCircle(radius)) : Face.Revolve(Edge.MakeLine(new Point3D(radius, 0, 0), new Point3D(0, 0, 0)), arcSize, Axis.Z);

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
    public Ellipse(double xRadius, double yRadius, double rotation = 0, Alignment? align = null)
        : base(Face.Make(Wire.MakeEllipse(xRadius, yRadius)), rotation, align ?? Centered)
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
    public Polygon(IEnumerable<Point3D> points, double rotation = 0, Alignment? align = null)
        : this(points.ToList(), rotation, align) { }

    /// <summary>A polygon through the points.</summary>
    public Polygon(params Point3D[] points) : this((IEnumerable<Point3D>)points) { }

    private Polygon(List<Point3D> pts, double rotation, Alignment? align)
        : base(Face.Make(Wire.MakePolygon(pts)), rotation, align ?? new Alignment(Kerf.Modeling.Align.None, Kerf.Modeling.Align.None))
    {
        (Points, Align) = (pts, TwoAxes(align ?? new Alignment(Kerf.Modeling.Align.None, Kerf.Modeling.Align.None)));
    }

    /// <summary>The points.</summary>
    public IReadOnlyList<Point3D> Points { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>A rectangle of a width (along X) and height (along Y); centred by default.</summary>
public class Rectangle : BaseSketchObject
{
    /// <summary>A rectangle, rotated by <paramref name="rotation"/> degrees.</summary>
    public Rectangle(double width, double height, double rotation = 0, Alignment? align = null)
        : base(Face.MakeRect(width, height), rotation, align ?? Centered)
    {
        (Width, Height, Align) = (width, height, TwoAxes(align ?? Centered));
    }

    /// <summary>The size along X.</summary>
    public double Width { get; }

    /// <summary>The size along Y (build123d's <c>rectangle_height</c>).</summary>
    public double Height { get; }

    /// <summary>The alignment.</summary>
    public Alignment Align { get; }
}

/// <summary>A rectangle with its corners filleted to a radius; centred by default.</summary>
public class RectangleRounded : BaseSketchObject
{
    /// <summary>A rounded rectangle.</summary>
    /// <exception cref="ArgumentException">The width or height is no more than twice the radius.</exception>
    public RectangleRounded(double width, double height, double radius, double rotation = 0, Alignment? align = null)
        : base(Make(width, height, radius), rotation, align ?? Centered)
    {
        (Width, Height, Radius, Align) = (width, height, radius, TwoAxes(align ?? Centered));
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
    public double Height { get; }

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
    public RegularPolygon(double radius, int sideCount, bool majorRadius = true, double rotation = 0, Alignment? align = null)
        : base(Make(radius, sideCount, majorRadius, rotation, TwoAxes(align ?? Centered)), 0, null)
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
            .Select(i => new Point3D(rad * Math.Cos(i * 2 * Math.PI / sideCount + rotation * Constants.DegToRad),
                                    rad * Math.Sin(i * 2 * Math.PI / sideCount + rotation * Constants.DegToRad), 0)).ToList();
        var byX = pts.OrderBy(p => p.X).ToList();
        var byY = pts.OrderBy(p => p.Y).ToList();
        var offset = align.Offset(new Point3D(byX[0].X, byY[0].Y, 0), new Point3D(byX[^1].X, byY[^1].Y, 0), new Point3D(0, 0, 0));
        return Face.Make(Wire.MakePolygon(pts.Select(p => p + offset)));
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
    public SlotCurve(IShape1D curve, double height, double rotation = 0)
        : base(Face.Make((Wire)TopoUtil.AsWire(curve).Offset2D(height / 2).AsShape()), rotation, null)
    {
        (Curve, Height) = (curve, height);
    }

    /// <summary>The centre line.</summary>
    public IShape1D Curve { get; }

    /// <summary>The width across the slot.</summary>
    public double Height { get; }
}

/// <summary>A slot from its centre and the centre of one end arc, symmetric about the centre.</summary>
public class SlotCenterPoint : BaseSketchObject
{
    /// <summary>A slot of a height (its width across).</summary>
    /// <exception cref="ArgumentException">The centre and point coincide.</exception>
    public SlotCenterPoint(Point3D center, Point3D point, double height, double rotation = 0)
        : base(Make(center, point, height), rotation, null)
    {
        (SlotCenter, Point, Height) = (center, point, height);
    }

    private static Face Make(Point3D center, Point3D point, double height)
    {
        var halfLine = point - center;
        if (halfLine.Magnitude <= 0)
            throw new ArgumentException(
                $"Distance between center and point must be greater than 0 Got: distance = {Formatting.Number(halfLine.Magnitude)} (computed)");
        var wires = Wire.Combine([Edge.MakeLine(point, center), Edge.MakeLine(center, center - halfLine)]);
        return Face.Make((Wire)wires[0].Offset2D(height / 2));
    }

    /// <summary>The centre of the slot.</summary>
    public Point3D SlotCenter { get; }

    /// <summary>The centre of one end arc.</summary>
    public Point3D Point { get; }

    /// <summary>The width across the slot.</summary>
    public double Height { get; }
}

/// <summary>A slot from the distance between the centres of its end arcs; a circle when that is zero.</summary>
public class SlotCenterToCenter : BaseSketchObject
{
    /// <summary>A slot of a height (its width across).</summary>
    /// <exception cref="ArgumentException">The separation is negative.</exception>
    public SlotCenterToCenter(double centerSeparation, double height, double rotation = 0)
        : base(Make(centerSeparation, height), rotation, null)
    {
        (CenterSeparation, Height) = (centerSeparation, height);
    }

    private static Face Make(double separation, double height)
    {
        if (separation < 0)
            throw new ArgumentException($"The centre separation must be positive, not {Formatting.Number(separation)}.");
        if (separation > 0)
            return Face.Make((Wire)Wire.Make([Edge.MakeLine(new Point3D(-separation / 2, 0, 0), new Point3D(0, 0, 0)),
                                            Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(separation / 2, 0, 0))]).Offset2D(height / 2));
        return new Circle(height / 2).SingleFace();
    }

    /// <summary>The distance between the centres of the end arcs.</summary>
    public double CenterSeparation { get; }

    /// <summary>The width across the slot.</summary>
    public double Height { get; }
}

/// <summary>A slot of an overall width and height; a circle when they are equal. Centred by default.</summary>
public class SlotOverall : BaseSketchObject
{
    /// <summary>A slot.</summary>
    /// <exception cref="ArgumentException">The width is less than the height.</exception>
    public SlotOverall(double width, double height, double rotation = 0, Alignment? align = null)
        : base(Make(width, height), rotation, align ?? Centered)
    {
        (Width, Height) = (width, height);
    }

    private static Face Make(double width, double height)
    {
        if (width < height)
            throw new ArgumentException($"Slot requires that width > height. Got: width={Formatting.Number(width)}, height={Formatting.Number(height)}");
        if (width > height)
            return Face.Make((Wire)Wire.Make([Edge.MakeLine(new Point3D(-width / 2 + height / 2, 0, 0), new Point3D(0, 0, 0)),
                                            Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(width / 2 - height / 2, 0, 0))]).Offset2D(height / 2));
        return new Circle(width / 2).SingleFace();
    }

    /// <summary>The overall width.</summary>
    public double Width { get; }

    /// <summary>The width across the slot.</summary>
    public double Height { get; }
}

/// <summary>
/// A superellipse ("squircle") of a width, height and order: order 1 is a diamond, 2 an ellipse, and higher orders
/// approach a rectangle. Centred by default.
/// </summary>
public class Superellipse : BaseSketchObject
{
    /// <summary>A superellipse; above order 2 each quadrant is a spline through <paramref name="pointCount"/> points.</summary>
    /// <exception cref="ArgumentException">The order isn't positive.</exception>
    public Superellipse(double width, double height, double order = 4, int pointCount = 16, double rotation = 0, Alignment? align = null)
        : base(Make(width, height, order, pointCount), rotation, align ?? Centered)
    {
        (Width, Height, SuperOrder, PointCount, Align) = (width, height, order, pointCount, TwoAxes(align ?? Centered));
    }

    private static Face Make(double width, double height, double order, int pointCount)
    {
        if (order <= 0)
            throw new ArgumentException("Order must be greater than zero.");
        Edge topRight;
        if (order == 1)
            topRight = Edge.MakeLine(new Point3D(width / 2, 0, 0), new Point3D(0, height / 2, 0));
        else if (order == 2)
            topRight = Edge.MakeEllipse(width / 2, height / 2, startAngle: 0, endAngle: 90);
        else
        {
            var points = new List<Point3D> { new(width / 2, 0.0, 0) };
            for (int i = 1; i < pointCount - 1; i++)
            {
                var t = i * Math.PI / pointCount / 2;
                double c = Math.Cos(t), s = Math.Sin(t);
                var x = Math.Pow(Math.Abs(c), 2 / order) * width * Math.CopySign(1, c) / 2;
                var y = Math.Pow(Math.Abs(s), 2 / order) * height * Math.CopySign(1, s) / 2;
                points.Add(new Point3D(x, y, 0));
            }
            points.Add(new Point3D(0.0, height / 2, 0));
            topRight = Edge.MakeSpline(points);
        }
        var topLeft = topRight.Mirror(Plane.YZ).Reversed();
        return Face.Make(Wire.Make([topRight, topLeft, topLeft.Mirror(Plane.XZ).Reversed(), topRight.Mirror(Plane.XZ).Reversed()]));
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
                     Alignment? align = null)
        : base(Make(width, height, leftSideAngle, rightSideAngle is { } r && r != 0 ? r : leftSideAngle), rotation, align ?? Centered)
    {
        (Width, Height, LeftSideAngle, RightSideAngle, Align) =
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
        Point3D[] pts = [new(-botLeft, -height / 2, 0), new(botRight, -height / 2, 0), new(topRight, height / 2, 0), new(-topLeft, height / 2, 0), new(-botLeft, -height / 2, 0)];
        return Face.Make(Wire.MakePolygon(pts));
    }

    /// <summary>The major (bottom) width.</summary>
    public double Width { get; }

    /// <summary>The height.</summary>
    public double Height { get; }

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
    public Triangle(double? sideA = null, double? sideB = null, double? sideC = null, double? angleA = null, double? angleB = null, double? angleC = null,
                    Alignment? align = null, double rotation = 0)
        : this(Solve(sideA, sideB, sideC, angleA, angleB, angleC), align, rotation) { }

    private Triangle((double a, double b, double c, double angleA, double angleB, double angleC) t, Alignment? align, double rotation)
        : base(Make(t), rotation, align)
    {
        (SideA, SideB, SideC) = (t.a, t.b, t.c);
        (AngleA, AngleB, AngleC) = (t.angleA * Constants.RadToDeg, t.angleB * Constants.RadToDeg, t.angleC * Constants.RadToDeg);
        var edges = Edges();
        EdgeA = edges.First(e => Math.Abs(e.Length - t.a) < Constants.Tolerance);
        EdgeB = edges.First(e => Math.Abs(e.Length - t.b) < Constants.Tolerance && e != EdgeA);
        EdgeC = edges.First(e => e != EdgeA && e != EdgeB);
        VertexA = (Vertex)Vertex.CommonVertex(EdgeB, EdgeC);
        VertexB = (Vertex)Vertex.CommonVertex(EdgeA, EdgeC);
        VertexC = (Vertex)Vertex.CommonVertex(EdgeA, EdgeB);
    }

    private static (double, double, double, double, double, double) Solve(double? a, double? b, double? c, double? angleA, double? angleB, double? angleC)
    {
        double?[] all = [a, b, c, angleA, angleB, angleC];
        if (a is null && b is null && c is null || all.Count(v => v is null) != 3)
            throw new ArgumentException("Give one length and two other values.");
        double? Rad(double? deg) => deg * Constants.DegToRad;
        return TriangleSolver.Solve(a, b, c, Rad(angleA), Rad(angleB), Rad(angleC));
    }

    private static Face Make((double a, double b, double c, double angleA, double angleB, double angleC) t)
    {
        var triangle = Face.Make(Wire.MakePolygon([new Point3D(0, 0, 0), new Point3D(t.a, 0, 0), new Vector3D(t.c, 0, 0).Rotate(Axis.Z, t.angleB * Constants.RadToDeg).ToPoint()]));
        var center = triangle.Vertices().Aggregate(Vector3D.Zero, (sum, v) => sum + v.Point.ToVector()) * (1.0 / 3);
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
    internal static (double a, double b, double c, double angleA, double angleB, double angleC) Solve(double? a, double? b, double? c, double? angleA, double? angleB, double? angleC)
    {
        double?[] sides = [a, b, c];
        double?[] angles = [angleA, angleB, angleC];
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
            if (a is { } a1 && angleA is { } angleA1 && b is { } b1) return Ssa(a1, b1, angleA1);
            if (a is { } a2 && angleA is { } angleA2 && c is { } c2) { var (x, y, z, angleX, angleY, angleZ) = Ssa(a2, c2, angleA2); return (x, z, y, angleX, angleZ, angleY); }
            if (b is { } b3 && angleB is { } angleB3 && a is { } a3) { var (x, y, z, angleX, angleY, angleZ) = Ssa(b3, a3, angleB3); return (y, x, z, angleY, angleX, angleZ); }
            if (b is { } b4 && angleB is { } angleB4 && c is { } c4) { var (x, y, z, angleX, angleY, angleZ) = Ssa(b4, c4, angleB4); return (z, x, y, angleZ, angleX, angleY); }
            if (c is { } c5 && angleC is { } angleC5 && a is { } a5) { var (x, y, z, angleX, angleY, angleZ) = Ssa(c5, a5, angleC5); return (y, z, x, angleY, angleZ, angleX); }
            if (c is { } c6 && angleC is { } angleC6 && b is { } b6) { var (x, y, z, angleX, angleY, angleZ) = Ssa(c6, b6, angleC6); return (z, y, x, angleZ, angleY, angleX); }
            if (a is { } a7 && b is { } b7 && angleC is { } angleC7) return Sas(a7, b7, angleC7);
            if (b is { } b8 && c is { } c8 && angleA is { } angleA8) { var (x, y, z, angleX, angleY, angleZ) = Sas(b8, c8, angleA8); return (z, x, y, angleZ, angleX, angleY); }
            if (c is { } c9 && a is { } a9 && angleB is { } angleB9) { var (x, y, z, angleX, angleY, angleZ) = Sas(c9, a9, angleB9); return (y, z, x, angleY, angleZ, angleX); }
        }
        // One side and two angles
        double aa = angleA ?? Math.PI - angleB!.Value - angleC!.Value;
        double bb = angleB ?? Math.PI - aa - angleC!.Value;
        double cc = angleC ?? Math.PI - aa - bb;
        if (aa <= 0 || bb <= 0 || cc <= 0)
            throw new ArgumentException("No such triangle");
        if (c is { } cs)
            return Aaas(aa, bb, cc, cs);
        if (a is { } asd)
        {
            var (x, y, z, angleX, angleY, angleZ) = Aaas(bb, cc, aa, asd);
            return (z, x, y, angleZ, angleX, angleY);
        }
        {
            var (x, y, z, angleX, angleY, angleZ) = Aaas(cc, aa, bb, b!.Value);
            return (y, z, x, angleY, angleZ, angleX);
        }
    }

    private static (double, double, double, double, double, double) Aaas(double angleD, double angleE, double angleF, double f) =>
        (f * Math.Sin(angleD) / Math.Sin(angleF), f * Math.Sin(angleE) / Math.Sin(angleF), f, angleD, angleE, angleF);

    private static (double, double, double, double, double, double) Sss(double d, double e, double f)
    {
        if (!(d + e > f && e + f > d && f + d > e))
            throw new ArgumentException("No such triangle");
        var angleF = Math.Acos((d * d + e * e - f * f) / (2 * d * e));
        var angleE = Math.Acos((d * d + f * f - e * e) / (2 * d * f));
        return (d, e, f, Math.PI - angleF - angleE, angleE, angleF);
    }

    private static (double, double, double, double, double, double) Sas(double d, double e, double angleF) =>
        Sss(d, e, Math.Sqrt(d * d + e * e - 2 * d * e * Math.Cos(angleF)));

    private static (double, double, double, double, double, double) Ssa(double d, double e, double angleD)
    {
        var sinE = Math.Sin(angleD) * e / d;
        double angleE;
        if (TopologyUtils.IsCloseB(sinE, 1, absTol: 0))
            angleE = Math.PI / 2;
        else
        {
            if (!(sinE < 1))
                throw new ArgumentException("No such triangle");
            var acute = Math.Asin(sinE);
            var obtuse = Math.PI - acute;
            bool acuteValid = 0 < Math.PI - angleD - acute && Math.PI - angleD - acute < Math.PI;
            bool obtuseValid = 0 < Math.PI - angleD - obtuse && Math.PI - angleD - obtuse < Math.PI;
            if (acuteValid && obtuseValid)
                throw new ArgumentException("Two different triangles fit this description");
            if (!acuteValid && !obtuseValid)
                throw new ArgumentException("No such triangle");
            angleE = acuteValid ? acute : obtuse;
        }
        var angleF = Math.PI - angleD - angleE;
        var (e2, f2, d2, angleE2, angleF2, angleD2) = Aaas(angleE, angleF, angleD, d);
        return (d2, e2, f2, angleD2, angleE2, angleF2);
    }
}
