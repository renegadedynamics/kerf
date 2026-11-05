// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Sphere.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>A bounding sphere of a geometric entity (triangle, line segment, ...), with the entity's U and V indices.</summary>
internal partial struct BoundingSphere : IEquatable<BoundingSphere>, ISpatialTreeBox<BoundingSphere>
{
    private Xyz _center;
    private double _radius;
    private bool _isValid;
    private int _u;
    private int _v;

    /// <summary>Creates a sphere of radius 0 at the origin, not valid.</summary>
    public BoundingSphere()
    {
        _center = new Xyz(0.0, 0.0, 0.0);
        _radius = 0.0;
        _isValid = false;
        _u = 0;
        _v = 0;
    }

    /// <summary>Creates a definite sphere (not marked valid).</summary>
    public BoundingSphere(in Xyz center, double radius, int u, int v)
    {
        _center = center;
        _radius = radius;
        _isValid = false;
        _u = u;
        _v = v;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValid(bool isValid) => _isValid = isValid;

    // NCollection_UBTree's bounding-type contract (the tree mutates its own nodes' values through it)
    void ISpatialTreeBox<BoundingSphere>.Add(in BoundingSphere other) => Add(other);

    internal void Add(in BoundingSphere other)
    {
        // OCCT tests a negative radius for "not initialised yet", but the default constructor sets the radius to 0,
        // so adding to a default sphere encloses the origin as well.
        if (_radius < 0.0)
        {
            // not initialised yet
            this = other;
            return;
        }

        double dist = (_center - other._center).Modulus();
        if (_radius + dist <= other._radius)
        {
            // the other sphere is larger and encloses this
            this = other;
            return;
        }

        if (other._radius + dist <= _radius)
        {
            return; // this sphere encloses other
        }

        // expansion
        double dfR = (dist + _radius + other._radius) * 0.5;
        double paramOnDiam = (dfR - _radius) / dist;
        _center = _center * (1.0 - paramOnDiam) + other._center * paramOnDiam;
        _radius = dfR;
        _isValid = false;
    }

    // Copies with a mutator applied (the public way to "set")

    public readonly BoundingSphere WithValid(bool isValid)
    {
        BoundingSphere copy = this;
        copy.SetValid(isValid);
        return copy;
    }

    /// <summary>The smallest sphere enclosing this sphere and <paramref name="other"/> (not marked valid).</summary>
    public readonly BoundingSphere Added(in BoundingSphere other)
    {
        BoundingSphere copy = this;
        copy.Add(other);
        return copy;
    }

    // Queries

    /// <summary>The U parameter on the shape.</summary>
    public readonly int U() => _u;

    /// <summary>The V parameter on the shape.</summary>
    public readonly int V() => _v;

    /// <summary>True if this sphere corresponds to a real entity.</summary>
    public readonly bool IsValid => _isValid;

    public readonly Xyz Center() => _center;

    public readonly double Radius() => _radius;

    /// <summary>The minimal and maximal distances from <paramref name="xyz"/> to the sphere.</summary>
    public readonly void Distances(in Xyz xyz, out double min, out double max)
    {
        max = (xyz - _center).Modulus();
        min = (max - _radius < 0 ? 0.0 : max - _radius);
        max += _radius;
    }

    /// <summary>The minimal and maximal squared distances from <paramref name="xyz"/> to the sphere.</summary>
    public readonly void SquareDistances(in Xyz xyz, out double min, out double max)
    {
        max = (xyz - _center).SquareModulus();
        double radSq = _radius * _radius;
        min = (max < radSq ? 0.0 : max - radSq);
        max += radSq;
    }

    /// <summary>Projects a point on the entity (here, its center); always true.</summary>
    public readonly bool Project(in Xyz node, out Xyz projNode, out double dist, out bool inside)
    {
        projNode = _center;
        dist = (node - projNode).Modulus();
        inside = true;
        return true;
    }

    public readonly double Distance(in Xyz node) => (node - _center).Modulus();

    public readonly double SquareDistance(in Xyz node) => (node - _center).SquareModulus();

    public readonly bool IsOut(in BoundingSphere other)
        => (_center - other._center).SquareModulus()
           > (_radius + other._radius) * (_radius + other._radius);

    /// <summary>
    /// True if the point's minimal distance exceeds <paramref name="maxDist"/>; otherwise, for a valid sphere,
    /// lowers <paramref name="maxDist"/> to the maximal distance when that is smaller.
    /// </summary>
    public readonly bool IsOut(in Xyz xyz, ref double maxDist)
    {
        Distances(xyz, out double curMinDist, out double curMaxDist);
        if (curMinDist > maxDist)
        {
            return true;
        }

        if (_isValid && curMaxDist < maxDist)
        {
            maxDist = curMaxDist;
        }

        return false;
    }

    public readonly double SquareExtent() => 4 * _radius * _radius;

    // Exact equality, for use as a value

    public readonly bool Equals(BoundingSphere other)
        => _center.Equals(other._center) && _radius.Equals(other._radius) && _isValid == other._isValid
           && _u == other._u && _v == other._v;

    public override readonly bool Equals(object? obj) => obj is BoundingSphere other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_center, _radius, _isValid, _u, _v);

    public static bool operator ==(in BoundingSphere left, in BoundingSphere right) => left.Equals(right);

    public static bool operator !=(in BoundingSphere left, in BoundingSphere right) => !left.Equals(right);

    /// <summary>The center, radius, validity (1 or 0), U and V.</summary>
    public override readonly string ToString()
        => Constants.Format("Bnd_Sphere", _center.X(), _center.Y(), _center.Z(), _radius, _isValid ? 1 : 0, _u, _v);
}
