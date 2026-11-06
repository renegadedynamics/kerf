// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The polygons already sampled on 2D curves during work that asks for the same ones over and over, as a 2D offset does: it
/// intersects each bisector with its two edges, and each edge with all its bisectors. A polygon's first sampling depends only on
/// the curve, the range, the number of points and the tolerance, so a later polygon shares its points until it has to refine them.
/// A table belongs to one piece of work on one thread and is current only during <see cref="Use"/>, so nothing crosses threads,
/// nothing outlives the work, and no other intersection sees it.
/// </summary>
internal sealed class SampledPolygons
{
    // A table stops growing here, about 13 MB of points and parameters. What it already has stays usable
    private const int MostPoints = 1 << 18;

    [ThreadStatic]
    private static SampledPolygons? t_current;

    private readonly Dictionary<Key, Sampled> _sampled = [];
    private int _points;

    /// <summary>This thread's current table, if any.</summary>
    public static SampledPolygons? Current => t_current;

    /// <summary>Makes this the thread's table until the result is disposed.</summary>
    public Scope Use()
    {
        t_current = this;
        return default;
    }

    /// <summary>The first sampling of a curve over a range, if this table has it.</summary>
    public bool TryGet(Curve2D curve, double first, double last, int nbPoints, double tolerance, out Sampled sampled) =>
        _sampled.TryGetValue(new Key(curve, first, last, nbPoints, tolerance), out sampled);

    /// <summary>Keeps a first sampling, unless the table is full. If so, its arrays mustn't be written from here on.</summary>
    public bool TryAdd(Curve2D curve, double first, double last, int nbPoints, double tolerance, in Sampled sampled)
    {
        if (_points + nbPoints > MostPoints)
        {
            return false;
        }
        _points += nbPoints;
        _sampled.Add(new Key(curve, first, last, nbPoints, tolerance), sampled);
        return true;
    }

    /// <summary>Ends a table's <see cref="Use"/>.</summary>
    public readonly struct Scope : IDisposable
    {
        /// <summary>Leaves the thread with no table.</summary>
        public void Dispose() => t_current = null;
    }

    /// <summary>A polygon as its sampling left it.</summary>
    public readonly record struct Sampled(NCollection_Array1<Point2D> Points, NCollection_Array1<double> Parameters, Box2d Box, double Deflection);

    // By the curve's identity, as curves have no value equality to compare
    private readonly record struct Key(Curve2D Curve, double First, double Last, int NbPoints, double Tolerance)
    {
        public bool Equals(Key other) =>
            ReferenceEquals(Curve, other.Curve) && First == other.First && Last == other.Last && NbPoints == other.NbPoints
            && Tolerance == other.Tolerance;

        public override int GetHashCode() => HashCode.Combine(RuntimeHelpers.GetHashCode(Curve), First, Last, NbPoints, Tolerance);
    }
}
