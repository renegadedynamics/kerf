namespace Kerf.Modeling;

/// <summary>
/// build123d's conveniences on the kernel's <see cref="BoundingBox"/>, which took the place of the layer's <c>BoundBox</c>.
/// </summary>
/// <remarks>
/// <para>
/// Besides its extents, a box offers spatial predicates. <c>Covers</c> and <c>CoveredBy</c>
/// test inclusive enclosure; <c>Contains</c> and <c>Within</c> require enclosure with
/// shared interior; <c>ContainsProperly</c> also excludes boundary contact.
/// <c>IntersectsNear</c> and <c>Disjoint</c> test whether the distance between boxes is within
/// the tolerance (the kernel's <see cref="BoundingBox.Intersects"/> is exact); <c>Touches</c> finds contact without
/// shared interior, and <c>Overlaps</c> shared interior where neither box contains the other.
/// </para>
/// <para>
/// The predicates allow for degenerate boxes: an axis that is degenerate and coincident in both boxes is
/// left out of interior comparisons, so coplanar rectangles compare in two dimensions and collinear
/// lines in one. Degenerate axes at different coordinates stay distinct: rectangles on parallel planes
/// share no interior.
/// </para>
/// </remarks>
public static class BoundingBoxExtensions
{
    static double Square(double x) => x * x;

    static double Component(Point3D p, int i) => i == 0 ? p.X : i == 1 ? p.Y : p.Z;

    static double Component(Vector3D v, int i) => i == 0 ? v.X : i == 1 ? v.Y : v.Z;

    extension(BoundingBox box)
    {
        /// <summary>
        /// The Lebesgue measure: the product of the extents larger than the tolerance, so the length of a line,
        /// the area of a rectangle or the volume of a box.
        /// </summary>
        public double Measure
        {
            get
            {
                double result = 1;
                var size = box.Size;
                for (int i = 0; i < 3; i++)
                    if (Component(size, i) > Constants.Tolerance)
                        result *= Component(size, i);
                return result;
            }
        }

        /// <summary>The length of the body diagonal: the object's maximum size; zero for an empty box.</summary>
        public double Diagonal => box.IsEmpty ? 0.0 : Math.Sqrt(Square(box.Size.X) + Square(box.Size.Y) + Square(box.Size.Z));

        /// <summary>The centre; the origin for an empty box, where the kernel's <see cref="BoundingBox.Centre"/> throws.</summary>
        public Point3D Center() => box.IsEmpty ? Point3D.Origin : ((box.Min.ToVector() + box.Max.ToVector()) / 2).ToPoint();

        /// <summary>
        /// The axes relevant to an interior comparison as (selfMin, selfMax, otherMin, otherMax), or null if a
        /// degenerate axis at different coordinates rules out any shared interior.
        /// </summary>
        private List<(double, double, double, double)>? InteriorAxes(BoundingBox other, double tolerance)
        {
            var axes = new List<(double, double, double, double)>(3);
            for (int i = 0; i < 3; i++)
            {
                double sMin = Component(box.Min, i), sMax = Component(box.Max, i), oMin = Component(other.Min, i), oMax = Component(other.Max, i);
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

        private bool HasInteriorOverlap(BoundingBox other, double tolerance)
        {
            if (box.IsEmpty || other.IsEmpty)
                return false;
            var axes = box.InteriorAxes(other, tolerance);
            return axes is not null && axes.All(a => Math.Max(a.Item1, a.Item3) < Math.Min(a.Item2, a.Item4) - tolerance);
        }

        /// <summary>True if this box includes every point of <paramref name="other"/>, boundary contact included.</summary>
        public bool Covers(BoundingBox other, double tolerance = Constants.Tolerance)
        {
            if (box.IsEmpty || other.IsEmpty)
                return false;
            for (int i = 0; i < 3; i++)
                if (!(Component(box.Min, i) <= Component(other.Min, i) + tolerance && Component(other.Max, i) <= Component(box.Max, i) + tolerance))
                    return false;
            return true;
        }

        /// <summary>True if every point of this box is in <paramref name="other"/>.</summary>
        public bool CoveredBy(BoundingBox other, double tolerance = Constants.Tolerance) => other.Covers(box, tolerance);

        /// <summary>
        /// True if this box contains <paramref name="other"/>: boundary contact is allowed, but the other box
        /// must have an interior point in this one. Use <c>Covers</c> for degenerate boxes.
        /// </summary>
        public bool Contains(BoundingBox other, double tolerance = Constants.Tolerance) =>
            box.Covers(other, tolerance) && box.HasInteriorOverlap(other, tolerance);

        /// <summary>True if <paramref name="other"/> is contained without touching this box's boundary.</summary>
        public bool ContainsProperly(BoundingBox other, double tolerance = Constants.Tolerance)
        {
            if (box.IsEmpty || other.IsEmpty)
                return false;
            var axes = box.InteriorAxes(other, tolerance);
            if (axes is null || axes.Count == 0)
                return false;
            return axes.All(a => a.Item1 + tolerance < a.Item3 && a.Item4 < a.Item2 - tolerance);
        }

        /// <summary>True if this box is contained in <paramref name="other"/>.</summary>
        public bool Within(BoundingBox other, double tolerance = Constants.Tolerance) => other.Contains(box, tolerance);

        /// <summary>True if the distance between the boxes is within the tolerance.</summary>
        public bool IntersectsNear(BoundingBox other, double tolerance = Constants.Tolerance) =>
            !box.IsEmpty && !other.IsEmpty && box.Distance(other) <= tolerance;

        // Bnd_Box::Distance: the distance between the boxes' corners, axis by axis
        private double Distance(BoundingBox other)
        {
            double sum = 0;
            for (int i = 0; i < 3; i++)
            {
                double min1 = Component(box.Min, i), max1 = Component(box.Max, i), min2 = Component(other.Min, i), max2 = Component(other.Max, i);
                if ((min1 <= min2 && min2 <= max1) || (min2 <= min1 && min1 <= max2))
                    continue;
                sum += Math.Min(Square(min1 - max2), Square(max1 - min2));
            }
            return Math.Sqrt(sum);
        }

        /// <summary>True if the boxes don't intersect.</summary>
        public bool Disjoint(BoundingBox other, double tolerance = Constants.Tolerance) => !box.IntersectsNear(other, tolerance);

        /// <summary>True if the boxes meet without sharing interior.</summary>
        public bool Touches(BoundingBox other, double tolerance = Constants.Tolerance) =>
            box.IntersectsNear(other, tolerance) && !box.HasInteriorOverlap(other, tolerance);

        /// <summary>True if the boxes share interior but neither contains the other.</summary>
        public bool Overlaps(BoundingBox other, double tolerance = Constants.Tolerance) =>
            box.HasInteriorOverlap(other, tolerance) && !box.Contains(other, tolerance) && !other.Contains(box, tolerance);

        /// <summary>The amount to move the object to align it; nothing for an empty box.</summary>
        public Vector3D ToAlignOffset(Alignment align) =>
            box.IsEmpty ? align.Offset(Point3D.Origin, Point3D.Origin) : align.Offset(box.Min, box.Max);

        /// <summary>"bbox: 0.0 &lt;= x &lt;= 1.0, 0.0 &lt;= y &lt;= 1.0, 0.0 &lt;= z &lt;= 1.0".</summary>
        public string Describe()
        {
            string r(double v) => Formatting.Number(v);
            var (min, max) = box.IsEmpty ? (Point3D.Origin, Point3D.Origin) : (box.Min, box.Max);
            return $"bbox: {r(min.X)} <= x <= {r(max.X)}, {r(min.Y)} <= y <= {r(max.Y)}, {r(min.Z)} <= z <= {r(max.Z)}";
        }
    }
}

/// <summary>
/// A box's two corners, each transformed as it is, without fitting a box around them: <c>Min</c> can be above <c>Max</c>
/// on an axis the transformation reverses, which a <see cref="BoundingBox"/> can't hold (build123d's <c>Bnd_Box(min, max)</c>
/// keeps them so, and its tests pin it).
/// </summary>
/// <param name="Min">Where the box's minimum corner went.</param>
/// <param name="Max">Where the box's maximum corner went.</param>
public readonly record struct BoxCorners(Point3D Min, Point3D Max)
{
    /// <summary>The box around both corners.</summary>
    public BoundingBox ToBoundingBox() => BoundingBox.Of([Min, Max]);
}
