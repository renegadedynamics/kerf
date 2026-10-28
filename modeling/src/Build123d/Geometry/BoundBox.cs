using Build123d.Occt;

namespace Build123d;

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
    /// <summary>The OCCT box, or null for an empty box. Never modified.</summary>
    internal Bnd_Box? Wrapped { get; }

    /// <summary>The minimum corner; the origin for an empty box.</summary>
    public Vector Min { get; }

    /// <summary>The maximum corner; the origin for an empty box.</summary>
    public Vector Max { get; }

    /// <summary>The overall size.</summary>
    public Vector Size { get; }

    /// <summary>The bounding box of nothing.</summary>
    public static BoundBox Empty { get; } = Own(new Bnd_Box());

    BoundBox(Bnd_Box box)
    {
        if (box.IsVoid())
        {
            box.Dispose();
            return;   // all corners at the origin
        }
        Wrapped = box;
        Min = new Vector(box.GetXMin(), box.GetYMin(), box.GetZMin());
        Max = new Vector(box.GetXMax(), box.GetYMax(), box.GetZMax());
        Size = Max - Min;
    }

    /// <summary>Wraps a Bnd_Box this bounding box takes ownership of.</summary>
    internal static BoundBox Own(Bnd_Box box) => new(box);

    /// <summary>A bounding box from an OCCT Bnd_Box (copied).</summary>
    public static BoundBox FromBndBox(Bnd_Box box) => new(new Bnd_Box(box));

    /// <summary>True for the bounding box of nothing, which has zero size and contains, covers, touches and intersects nothing.</summary>
    public bool IsEmpty => Wrapped is null;

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
    public double Diagonal => Wrapped is null ? 0.0 : Math.Sqrt(Wrapped.SquareExtent());

    /// <summary>The centre.</summary>
    public Vector Center() => (Min + Max) / 2;

    /// <summary>A box enlarged to include a point, with a gap of <paramref name="tolerance"/> (default <see cref="Constants.LooseTolerance"/>).</summary>
    public BoundBox Add(Vector point, double? tolerance = null)
    {
        var box = Start(tolerance);
        box.Update(point.X, point.Y, point.Z);
        return new BoundBox(box);
    }

    /// <summary>A box enclosing this and another, with a gap of <paramref name="tolerance"/> (default <see cref="Constants.LooseTolerance"/>).</summary>
    public BoundBox Add(BoundBox other, double? tolerance = null)
    {
        var box = Start(tolerance);
        if (other.Wrapped is not null)
            box.Add(other.Wrapped);
        return new BoundBox(box);
    }

    Bnd_Box Start(double? tolerance)
    {
        var box = new Bnd_Box();
        box.SetGap(tolerance ?? Constants.LooseTolerance);
        if (Wrapped is not null)
            box.Add(Wrapped);
        return box;
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
        Wrapped is not null && other.Wrapped is not null && Wrapped.Distance(other.Wrapped) <= tolerance;

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
        string r(double v) => PyFormat.Repr(v);
        return $"bbox: {r(Min.X)} <= x <= {r(Max.X)}, {r(Min.Y)} <= y <= {r(Max.Y)}, {r(Min.Z)} <= z <= {r(Max.Z)}";
    }
}
