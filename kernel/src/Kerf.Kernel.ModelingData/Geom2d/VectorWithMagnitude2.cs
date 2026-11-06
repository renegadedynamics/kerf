// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_VectorWithMagnitude.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_VectorWithMagnitude.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>A vector with a magnitude, possibly zero, in the plane (a shareable <see cref="Vector2D"/>).</summary>
/// <remarks>
/// OCCT's compound operators (<c>+=</c>, <c>-=</c>, <c>*=</c>, <c>/=</c>) mutate in place, so they are left out;
/// ported code calls <c>Add</c>, <c>Subtract</c>, <c>Multiply</c>, <c>Divide</c>. The binary operators return new
/// vectors, as in OCCT.
/// </remarks>
internal class VectorWithMagnitude2 : GeometryVector2D
{
    /// <summary>A vector equal to V.</summary>
    public VectorWithMagnitude2(in Vector2D v) => _gpVec2d = v;

    /// <summary>The vector (X, Y).</summary>
    public VectorWithMagnitude2(double x, double y) => _gpVec2d = new Vector2D(x, y);

    /// <summary>The vector from P1 to P2.</summary>
    public VectorWithMagnitude2(in Point2D p1, in Point2D p2) => _gpVec2d = new Vector2D(p1, p2);

    /// <summary>A new, unfrozen vector equal to this one.</summary>
    public override Geometry2D Copy()
    {
        VectorWithMagnitude2 v;
        v = new VectorWithMagnitude2(_gpVec2d);
        return v;
    }

    internal void SetCoord(double x, double y)
    {
        CheckMutable();
        _gpVec2d = new Vector2D(x, y);
    }

    internal void SetVec2d(in Vector2D v)
    {
        CheckMutable();
        _gpVec2d = v;
    }

    internal void SetX(double x)
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.WithX(x);
    }

    internal void SetY(double y)
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.WithY(y);
    }

    public override double Magnitude() => _gpVec2d.Magnitude;

    public override double SquareMagnitude() => _gpVec2d.SquareMagnitude;

    internal void Add(GeometryVector2D other)
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.Added(other.Vec2d());
    }

    /// <summary>this + Other.</summary>
    public VectorWithMagnitude2 Added(GeometryVector2D other)
    {
        Vector2D temp = other.Vec2d();
        temp = temp.Added(_gpVec2d);
        return new VectorWithMagnitude2(temp);
    }

    public static VectorWithMagnitude2 operator +(VectorWithMagnitude2 left, GeometryVector2D other) =>
        left.Added(other);

    /// <summary>The cross product this ^ Other.</summary>
    public override double Crossed(GeometryVector2D other) => _gpVec2d.Crossed(other.Vec2d());

    public static double operator ^(VectorWithMagnitude2 left, GeometryVector2D other) => left.Crossed(other);

    internal void Divide(double scalar)
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.Divided(scalar);
    }

    /// <summary>this / Scalar.</summary>
    public VectorWithMagnitude2 Divided(double scalar)
    {
        Vector2D v = _gpVec2d;
        v = v.Divided(scalar);
        return new VectorWithMagnitude2(v);
    }

    public static VectorWithMagnitude2 operator /(VectorWithMagnitude2 left, double scalar) =>
        left.Divided(scalar);

    /// <summary>this * Scalar.</summary>
    public VectorWithMagnitude2 Multiplied(double scalar)
    {
        Vector2D v = _gpVec2d;
        v = v.Multiplied(scalar);
        return new VectorWithMagnitude2(v);
    }

    internal void Multiply(double scalar)
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.Multiplied(scalar);
    }

    /// <exception cref="ConstructionErrorException">The magnitude is at most <c>gp::Resolution()</c>.</exception>
    internal void Normalize()
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.Normalized();
    }

    /// <summary>The unit vector of this one.</summary>
    /// <exception cref="ConstructionErrorException">The magnitude is at most <c>gp::Resolution()</c>.</exception>
    public VectorWithMagnitude2 Normalized()
    {
        Vector2D v = _gpVec2d.Normalized();
        return new VectorWithMagnitude2(v);
    }

    internal void Subtract(GeometryVector2D other)
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.Subtracted(other.Vec2d());
    }

    /// <summary>this - Other.</summary>
    public VectorWithMagnitude2 Subtracted(GeometryVector2D other)
    {
        Vector2D v = _gpVec2d;
        v = v.Subtracted(other.Vec2d());
        return new VectorWithMagnitude2(v);
    }

    public static VectorWithMagnitude2 operator -(VectorWithMagnitude2 left, GeometryVector2D other) =>
        left.Subtracted(other);

    /// <summary>Applies T to this vector.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.Transformed(t);
    }

    // Non-mutating twins of the Set methods (Kerf)

    public VectorWithMagnitude2 WithCoord(double x, double y)
    {
        VectorWithMagnitude2 copy = (VectorWithMagnitude2)Copy();
        copy.SetCoord(x, y);
        return copy;
    }

    public VectorWithMagnitude2 WithVec2d(in Vector2D v)
    {
        VectorWithMagnitude2 copy = (VectorWithMagnitude2)Copy();
        copy.SetVec2d(v);
        return copy;
    }

    public VectorWithMagnitude2 WithX(double x)
    {
        VectorWithMagnitude2 copy = (VectorWithMagnitude2)Copy();
        copy.SetX(x);
        return copy;
    }

    public VectorWithMagnitude2 WithY(double y)
    {
        VectorWithMagnitude2 copy = (VectorWithMagnitude2)Copy();
        copy.SetY(y);
        return copy;
    }
}
