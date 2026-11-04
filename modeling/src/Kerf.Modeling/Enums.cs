// Ported from build123d/build_enums.py
namespace Kerf.Modeling;

/// <summary>Align an object about an axis.</summary>
public enum Align
{
    /// <summary>Align the minimum extent with the axis.</summary>
    Min,
    /// <summary>Centre the object on the axis.</summary>
    Center,
    /// <summary>Align the maximum extent with the axis.</summary>
    Max,
    /// <summary>Leave the object where it is.</summary>
    None,
}

/// <summary>DXF export spline approximation strategy.</summary>
public enum ApproxOption
{
    /// <summary>Approximate splines with arcs.</summary>
    Arc,
    /// <summary>Don't approximate.</summary>
    None,
    /// <summary>Approximate with simpler splines.</summary>
    Spline,
}

/// <summary>Angular rotation direction.</summary>
public enum AngularDirection
{
    /// <summary>Clockwise.</summary>
    Clockwise,
    /// <summary>Counter-clockwise.</summary>
    CounterClockwise,
}

/// <summary>
/// Where a sheet-metal bend sits relative to its bend line: which feature of the formed bend lands on
/// the line. The bend reaches from there into the moving side.
/// </summary>
public enum BendPosition
{
    /// <summary>The bend straddles the line.</summary>
    Center,
    /// <summary>The inside mould line falls on the line.</summary>
    MaterialInside,
    /// <summary>The outside mould line falls on the line.</summary>
    MaterialOutside,
    /// <summary>The whole bend lies past the line.</summary>
    BendOutside,
}

/// <summary>Which centre of an object to use.</summary>
public enum CenterOf
{
    /// <summary>The geometric centre.</summary>
    Geometry,
    /// <summary>The centre of mass.</summary>
    Mass,
    /// <summary>The centre of the bounding box.</summary>
    BoundingBox,
}

/// <summary>
/// Continuity level for geometric connections, such as shared vertices between edges or shared edges
/// between faces. Compatible with OCCT's GeomAbs_Shape.
/// </summary>
public enum ContinuityLevel
{
    /// <summary>Positional continuity: elements meet but may have sharp angles.</summary>
    C0 = 0,
    /// <summary>Tangent continuity: elements share a tangent direction at the junction.</summary>
    C1 = 1,
    /// <summary>Curvature continuity: elements share curvature at the junction.</summary>
    C2 = 2,
}

/// <summary>
/// How the material of a shape sits around one of its elements. It is a property of the element's
/// relationship to the shape it was selected from, not of the element alone.
/// </summary>
public enum Convexity
{
    /// <summary>The material closes around the element by less than half a turn (the outer edge of a box).</summary>
    Convex,
    /// <summary>By more than half a turn (the inner corner of a pocket, a hole).</summary>
    Concave,
    /// <summary>The boundary passes through without bending (a fillet seam, a planar face).</summary>
    Smooth,
    /// <summary>Both senses are present (a face with principal curvatures of opposite sign).</summary>
    Saddle,
}

/// <summary>Order to apply extrinsic rotations, about the fixed world axes.</summary>
public enum Extrinsic
{
#pragma warning disable CS1591 // Self-describing axis orders
    XYZ, XZY, YZX, YXZ, ZXY, ZYX, XYX, XZX, YZY, YXY, ZXZ, ZYZ,
#pragma warning restore CS1591
}

/// <summary>Order to apply intrinsic rotations, about the rotating object's own axes.</summary>
public enum Intrinsic
{
#pragma warning disable CS1591 // Self-describing axis orders
    XYZ, XZY, YZX, YXZ, ZXY, ZYX, XYX, XZX, YZY, YXY, ZXZ, ZYZ,
#pragma warning restore CS1591
}

/// <summary>Moving frame calculation method.</summary>
public enum FrameMethod
{
    /// <summary>The Frenet frame.</summary>
    Frenet,
    /// <summary>The corrected Frenet frame, which avoids flips.</summary>
    Corrected,
}

/// <summary>CAD geometry object type.</summary>
public enum GeomType
{
#pragma warning disable CS1591 // Self-describing geometry kinds
    Plane, Cylinder, Cone, Sphere, Torus, Bezier, BSpline, Revolution, Extrusion, Offset,
    Line, Circle, Ellipse, Hyperbola, Parabola, Other,
#pragma warning restore CS1591
}

/// <summary>Arrow head types.</summary>
public enum HeadType
{
#pragma warning disable CS1591
    Straight, Curved, Filleted,
#pragma warning restore CS1591
}

/// <summary>Sheet metal hem styles.</summary>
public enum HemType
{
    /// <summary>180° fold flat onto the sheet.</summary>
    Flat,
    /// <summary>180° fold with a gap.</summary>
    Open,
    /// <summary>Teardrop profile fold.</summary>
    Teardrop,
    /// <summary>Open rolled curl, with no flat leg.</summary>
    Rolled,
}

/// <summary>Reference surface represented by a sheet-metal shell.</summary>
public enum SheetSurface
{
#pragma warning disable CS1591
    Inside, Outside, Mid, Neutral,
#pragma warning restore CS1591
}

/// <summary>Split options.</summary>
public enum Keep
{
#pragma warning disable CS1591
    All, Bottom, Both, Inside, Outside, Top,
#pragma warning restore CS1591
}

/// <summary>Offset corner transition.</summary>
public enum Kind
{
#pragma warning disable CS1591
    Arc, Intersection, Tangent,
#pragma warning restore CS1591
}

/// <summary>How a builder combines a new object with what it has built so far.</summary>
public enum Mode
{
    /// <summary>Fuse the new object in.</summary>
    Add,
    /// <summary>Cut the new object away.</summary>
    Subtract,
    /// <summary>Keep only what the two share.</summary>
    Intersect,
    /// <summary>Replace everything built so far.</summary>
    Replace,
    /// <summary>Create the object without combining it.</summary>
    Private,
}

/// <summary>Text font styles.</summary>
public enum FontStyle
{
#pragma warning disable CS1591
    Regular, Bold, Italic, BoldItalic,
#pragma warning restore CS1591
}

/// <summary>Sagitta selection.</summary>
public enum Sagitta
{
    /// <summary>The short arc.</summary>
    Short = 0,
    /// <summary>The long arc.</summary>
    Long = -1,
    /// <summary>Both arcs.</summary>
    Both = 1,
}

/// <summary>Where a sheet-metal flange length is measured from.</summary>
public enum LengthFrom
{
    /// <summary>From the bend tangent line: the flat wall alone.</summary>
    Tangent,
    /// <summary>From the inner virtual sharp.</summary>
    InnerSharp,
    /// <summary>From the outer virtual sharp.</summary>
    OuterSharp,
}

/// <summary>Which side of a right triangle a length is.</summary>
public enum LengthMode
{
    /// <summary>The hypotenuse: the line itself.</summary>
    Diagonal,
    /// <summary>The run along the reference axis.</summary>
    Horizontal,
    /// <summary>The height off the reference axis.</summary>
    Vertical,
}

/// <summary>3MF mesh types, typically for 3D printing.</summary>
public enum MeshType
{
#pragma warning disable CS1591
    Other, Model, Support, SolidSupport,
#pragma warning restore CS1591
}

/// <summary>Methods for displaying numbers.</summary>
public enum NumberDisplay
{
#pragma warning disable CS1591
    Decimal, Fraction,
#pragma warning restore CS1591
}

/// <summary>Page sizes.</summary>
public enum PageSize
{
#pragma warning disable CS1591
    A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, Letter, Legal, Ledger,
#pragma warning restore CS1591
}

/// <summary>Tangency constraint for solvers' edge selection (OCCT's GccEnt_Position).</summary>
public enum Tangency
{
#pragma warning disable CS1591 // Values match GccEnt_Position
    Unqualified = 0, Enclosing = 1, Enclosed = 2, Outside = 3,
#pragma warning restore CS1591
}

/// <summary>Position along a curve mode.</summary>
public enum PositionMode
{
    /// <summary>A fraction of the curve's length.</summary>
    Length,
    /// <summary>A fraction of the curve's parameter range.</summary>
    Parameter,
}

/// <summary>Precision of the geometric data written to a STEP file.</summary>
public enum PrecisionMode
{
#pragma warning disable CS1591 // Values match OCCT's write.precision.mode
    Session = 2, Greatest = 1, Average = 0, Least = -1,
#pragma warning restore CS1591
}

/// <summary>Sheet metal relief shapes. Each names a shape, not a placement.</summary>
public enum ReliefType
{
    /// <summary>A circle.</summary>
    Round,
    /// <summary>Square-cornered, aligned to the fold lines.</summary>
    Square,
    /// <summary>A slot with rounded ends.</summary>
    Obround,
    /// <summary>Continues the flange gap through a corner.</summary>
    ConstantWidth,
}

/// <summary>How much of a shape a selector returns.</summary>
public enum Select
{
    /// <summary>Every feature of the kind.</summary>
    All,
    /// <summary>The features the last operation brought in or created.</summary>
    Last,
    /// <summary>Only the features that existed in no input of the last operation.</summary>
    New,
}

/// <summary>2D offset types.</summary>
public enum Side
{
#pragma warning disable CS1591
    Left, Right, Both,
#pragma warning restore CS1591
}

/// <summary>A measure of a shape to sort, group or filter by: <c>edges.OrderBy(ShapeKey.Length)</c>.</summary>
public enum ShapeKey
{
#pragma warning disable CS1591
    Length, Radius, Area, Volume, Distance,
#pragma warning restore CS1591
}

/// <summary>Text alignment.</summary>
public enum TextAlign
{
#pragma warning disable CS1591
    Bottom, Center, Left, Right, Top, TopFirstLine,
#pragma warning restore CS1591
}

/// <summary>Sweep discontinuity handling option.</summary>
public enum Transition
{
#pragma warning disable CS1591
    Right, Round, Transformed,
#pragma warning restore CS1591
}

/// <summary>Standard units. <see cref="UnitExtensions.Symbol"/> gives the unit's symbol.</summary>
public enum Unit
{
    /// <summary>Micrometre (µm).</summary>
    Micrometer,
    /// <summary>Millimetre (mm).</summary>
    Millimeter,
    /// <summary>Centimetre (cm).</summary>
    Centimeter,
    /// <summary>Metre (m).</summary>
    Meter,
    /// <summary>Inch (in).</summary>
    Inch,
    /// <summary>Foot (ft).</summary>
    Foot,
    /// <summary>Gram (g).</summary>
    Gram,
    /// <summary>Kilogram (kg).</summary>
    Kilogram,
    /// <summary>Pound (lb).</summary>
    Pound,
}

/// <summary>Helpers for <see cref="Unit"/>.</summary>
public static class UnitExtensions
{
    /// <summary>The unit's symbol, e.g. "mm".</summary>
    public static string Symbol(this Unit unit) => unit switch
    {
        Unit.Micrometer => "µm",
        Unit.Millimeter => "mm",
        Unit.Centimeter => "cm",
        Unit.Meter => "m",
        Unit.Inch => "in",
        Unit.Foot => "ft",
        Unit.Gram => "g",
        Unit.Kilogram => "kg",
        Unit.Pound => "lb",
        _ => throw new ArgumentOutOfRangeException(nameof(unit)),
    };
}

/// <summary>Extrude limit.</summary>
public enum Until
{
#pragma warning disable CS1591
    Next, Last, Previous, First,
#pragma warning restore CS1591
}
