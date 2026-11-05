using Kerf.Geometry;
using Kerf.Measurement;

namespace Kerf.Modeling;

/// <summary>
/// An immutable axis-aligned bounding box.
/// </summary>
/// <remarks>
/// <para>
/// Besides its extents, a box offers spatial predicates. <see cref="Covers"/> and <see cref="CoveredBy"/>
/// test inclusive enclosure; <see cref="Contains"/> and <see cref="Within"/> require enclosure with
/// shared interior; <see cref="ContainsProperly"/> also excludes boundary contact.
/// <see cref="Intersects"/> and <see cref="Disjoint"/> test whether the distance between boxes is within
/// the tolerance; <see cref="Touches"/> finds contact without shared interior, and <see cref="Overlaps"/>
/// shared interior where neither box contains the other.
/// </para>
/// <para>
/// The predicates allow for degenerate boxes: an axis that is degenerate and coincident in both boxes is
/// left out of interior comparisons, so coplanar rectangles compare in two dimensions and collinear
/// lines in one. Degenerate axes at different coordinates stay distinct: rectangles on parallel planes
/// share no interior.
/// </para>
/// </remarks>
public sealed class BoundBox
{
    // The box as OCCT's Bnd_Box holds it: its extents, and a gap around them that its corners include
    private readonly BoundingBox _extents;
    private readonly double _gap;

    /// <summary>The minimum corner; the origin for an empty box.</summary>
    public Vector Min { get; }

    /// <summary>The maximum corner; the origin for an empty box.</summary>
    public Vector Max { get; }

    /// <summary>The overall size.</summary>
    public Vector Size { get; }

    /// <summary>The bounding box of nothing.</summary>
    public static BoundBox Empty { get; } = new(BoundingBox.Empty, 0.0);

    BoundBox(BoundingBox extents, double gap)
    {
        if (extents.IsEmpty)
        {
            return;   // all corners at the origin
        }
        _extents = extents;
        _gap = gap;
        Min = new Vector(extents.Min.X - gap, extents.Min.Y - gap, extents.Min.Z - gap);
        Max = new Vector(extents.Max.X + gap, extents.Max.Y + gap, extents.Max.Z + gap);
        Size = Max - Min;
    }

    /// <summary>A bounding box from the kernel's.</summary>
    public static BoundBox FromBoundingBox(BoundingBox box) => new(box, 0.0);

    /// <summary>
    /// A box with the given corners as they are, even where a minimum is above its maximum (a box's corners transformed,
    /// as OCCT's <c>Bnd_Box(min, max)</c> keeps them); its extents are the box around both.
    /// </summary>
    internal static BoundBox FromCorners(Vector min, Vector max) =>
        new(BoundingBox.Of([min.ToPoint3D(), max.ToPoint3D()]), min, max);

    BoundBox(BoundingBox extents, Vector min, Vector max)
    {
        _extents = extents;
        Min = min;
        Max = max;
        Size = max - min;
    }

    /// <summary>The box as the kernel's bounding box.</summary>
    public BoundingBox ToBoundingBox() => IsEmpty ? BoundingBox.Empty : _gap == 0 ? _extents : _extents.Enlarged(_gap);

    /// <summary>
    /// The bounding box of a shape: tight when <paramref name="optimal"/>, else OCCT's quicker, looser one. Any mesh on the
    /// shape is ignored, as it can affect the box, unless the shape has nothing else (faces holding only triangles, as
    /// STL and glTF are read).
    /// </summary>
    public static BoundBox FromShape(Shape shape, double? tolerance = null, bool optimal = true) =>
        shape.IsEmpty ? Empty : FromTopoDS(shape, tolerance, optimal);

    /// <summary>The bounding box of an OCCT shape (see <see cref="FromShape"/>).</summary>
    public static BoundBox FromTopoDS(Shape shape, double? tolerance = null, bool optimal = true)
    {
        _ = tolerance;   // accepted, as build123d accepts it, but unused
        // Ignoring a mesh gives the box build123d gets by removing it first, without changing the shape
        var box = shape.BoundingBox(optimal);
        // A shape with faces holding only triangles (STL, glTF) has no geometry to bound but its mesh: use it then
        if (box.IsEmpty)
            box = shape.BoundingBox(optimal: false, useTriangulation: true);
        return new BoundBox(box, 0.0);
    }

    /// <summary>True for the bounding box of nothing, which has zero size and contains, covers, touches and intersects nothing.</summary>
    public bool IsEmpty => _extents.IsEmpty;

    /// <summary>
    /// The Lebesgue measure: the product of the extents larger than the tolerance, so the length of a line,
    /// the area of a rectangle or the volume of a box.
    /// </summary>
    public double Measure
    {
        get
        {
            double result = 1;
            for (int i = 0; i < 3; i++)
                if (Size[i] > Constants.Tolerance)
                    result *= Size[i];
            return result;
        }
    }

    /// <summary>The length of the body diagonal: the object's maximum size.</summary>
    public double Diagonal => IsEmpty ? 0.0 : Math.Sqrt(Square(_extents.Size.X + _gap + _gap) + Square(_extents.Size.Y + _gap + _gap)
                                                        + Square(_extents.Size.Z + _gap + _gap));

    static double Square(double x) => x * x;

    /// <summary>The centre.</summary>
    public Vector Center() => (Min + Max) / 2;

    /// <summary>A box enlarged to include a point, with a gap of <paramref name="tolerance"/> (default <see cref="Constants.LooseTolerance"/>).</summary>
    public BoundBox Add(Vector point, double? tolerance = null) =>
        // As Bnd_Box: the gap asked for applies only to a box started from nothing; one added to keeps its own
        IsEmpty ? new BoundBox(BoundingBox.Of(point.ToPoint3D()), Math.Abs(tolerance ?? Constants.LooseTolerance))
                : new BoundBox(_extents.Including(point.ToPoint3D()), _gap);

    /// <summary>A box enclosing this and another, with a gap of <paramref name="tolerance"/> (default <see cref="Constants.LooseTolerance"/>).</summary>
    public BoundBox Add(BoundBox other, double? tolerance = null)
    {
        _ = tolerance;   // as Bnd_Box: a box added to, or started from another, keeps the gaps the boxes have
        if (other.IsEmpty)
            return IsEmpty ? Empty : this;
        if (IsEmpty)
            return other;
        return new BoundBox(_extents.Union(other._extents), Math.Max(_gap, other._gap));
    }

    /// <summary>
    /// The axes relevant to an interior comparison as (selfMin, selfMax, otherMin, otherMax), or null if a
    /// degenerate axis at different coordinates rules out any shared interior.
    /// </summary>
    List<(double, double, double, double)>? InteriorAxes(BoundBox other, double tolerance)
    {
        var axes = new List<(double, double, double, double)>(3);
        for (int i = 0; i < 3; i++)
        {
            double sMin = Min[i], sMax = Max[i], oMin = other.Min[i], oMax = other.Max[i];
            bool selfDegenerate = sMax - sMin <= tolerance;
            bool otherDegenerate = oMax - oMin <= tolerance;
            if (selfDegenerate && otherDegenerate)
            {
                if (Math.Abs((sMin + sMax) / 2 - (oMin + oMax) / 2) > tolerance)
                    return null;
            }
            else
            {
                axes.Add((sMin, sMax, oMin, oMax));
            }
        }
        return axes;
    }

    bool HasInteriorOverlap(BoundBox other, double tolerance)
    {
        if (IsEmpty || other.IsEmpty)
            return false;
        var axes = InteriorAxes(other, tolerance);
        return axes is not null && axes.All(a => Math.Max(a.Item1, a.Item3) < Math.Min(a.Item2, a.Item4) - tolerance);
    }

    /// <summary>True if this box includes every point of <paramref name="other"/>, boundary contact included.</summary>
    public bool Covers(BoundBox other, double tolerance = Constants.Tolerance)
    {
        if (IsEmpty || other.IsEmpty)
            return false;
        for (int i = 0; i < 3; i++)
            if (!(Min[i] <= other.Min[i] + tolerance && other.Max[i] <= Max[i] + tolerance))
                return false;
        return true;
    }

    /// <summary>True if every point of this box is in <paramref name="other"/>.</summary>
    public bool CoveredBy(BoundBox other, double tolerance = Constants.Tolerance) => other.Covers(this, tolerance);

    /// <summary>
    /// True if this box contains <paramref name="other"/>: boundary contact is allowed, but the other box
    /// must have an interior point in this one. Use <see cref="Covers"/> for degenerate boxes.
    /// </summary>
    public bool Contains(BoundBox other, double tolerance = Constants.Tolerance) =>
        Covers(other, tolerance) && HasInteriorOverlap(other, tolerance);

    /// <summary>True if <paramref name="other"/> is contained without touching this box's boundary.</summary>
    public bool ContainsProperly(BoundBox other, double tolerance = Constants.Tolerance)
    {
        if (IsEmpty || other.IsEmpty)
            return false;
        var axes = InteriorAxes(other, tolerance);
        if (axes is null || axes.Count == 0)
            return false;
        return axes.All(a => a.Item1 + tolerance < a.Item3 && a.Item4 < a.Item2 - tolerance);
    }

    /// <summary>True if this box is contained in <paramref name="other"/>.</summary>
    public bool Within(BoundBox other, double tolerance = Constants.Tolerance) => other.Contains(this, tolerance);

    /// <summary>True if the distance between the boxes is within the tolerance.</summary>
    public bool Intersects(BoundBox other, double tolerance = Constants.Tolerance) =>
        !IsEmpty && !other.IsEmpty && Distance(other) <= tolerance;

    // Bnd_Box::Distance: the distance between the boxes' corners, axis by axis
    double Distance(BoundBox other)
    {
        double sum = 0;
        for (int i = 0; i < 3; i++)
        {
            double min1 = Min[i], max1 = Max[i], min2 = other.Min[i], max2 = other.Max[i];
            if ((min1 <= min2 && min2 <= max1) || (min2 <= min1 && min1 <= max2))
                continue;
            sum += Math.Min(Square(min1 - max2), Square(max1 - min2));
        }
        return Math.Sqrt(sum);
    }

    /// <summary>True if the boxes don't intersect.</summary>
    public bool Disjoint(BoundBox other, double tolerance = Constants.Tolerance) => !Intersects(other, tolerance);

    /// <summary>True if the boxes meet without sharing interior.</summary>
    public bool Touches(BoundBox other, double tolerance = Constants.Tolerance) =>
        Intersects(other, tolerance) && !HasInteriorOverlap(other, tolerance);

    /// <summary>True if the boxes share interior but neither contains the other.</summary>
    public bool Overlaps(BoundBox other, double tolerance = Constants.Tolerance) =>
        HasInteriorOverlap(other, tolerance) && !Contains(other, tolerance) && !other.Contains(this, tolerance);

    /// <summary>The amount to move the object to align it.</summary>
    public Vector ToAlignOffset(Alignment align) => align.Offset(Min, Max);

    /// <summary>"bbox: 0.0 &lt;= x &lt;= 1.0, 0.0 &lt;= y &lt;= 1.0, 0.0 &lt;= z &lt;= 1.0".</summary>
    public override string ToString()
    {
        string r(double v) => Formatting.Number(v);
        return $"bbox: {r(Min.X)} <= x <= {r(Max.X)}, {r(Min.Y)} <= y <= {r(Max.Y)}, {r(Min.Z)} <= z <= {r(Max.Z)}";
    }
}
