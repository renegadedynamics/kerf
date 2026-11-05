// Port of: src/ModelingData/TKG3d/Geom/Geom_VectorWithMagnitude.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_VectorWithMagnitude.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>A vector with a magnitude, possibly zero, in 3D space (a shareable <see cref="Vector3D"/>).</summary>
internal class VectorWithMagnitude : GeometryVector3D
{
    /// <summary>A vector equal to V.</summary>
    public VectorWithMagnitude(in Vector3D v) => _gpVec = v;

    /// <summary>The vector (X, Y, Z).</summary>
    public VectorWithMagnitude(double x, double y, double z) => _gpVec = new Vector3D(x, y, z);

    /// <summary>The vector from P1 to P2.</summary>
    public VectorWithMagnitude(in Point3D p1, in Point3D p2) => _gpVec = new Vector3D(p1, p2);

    /// <summary>A new, unfrozen vector equal to this one.</summary>
    public override Geometry3D Copy()
    {
        VectorWithMagnitude v;
        v = new VectorWithMagnitude(_gpVec);
        return v;
    }

    internal void SetCoord(double x, double y, double z)
    {
        CheckMutable();
        _gpVec = new Vector3D(x, y, z);
    }

    internal void SetVec(in Vector3D v)
    {
        CheckMutable();
        _gpVec = v;
    }

    internal void SetX(double x)
    {
        CheckMutable();
        _gpVec = _gpVec.WithX(x);
    }

    internal void SetY(double y)
    {
        CheckMutable();
        _gpVec = _gpVec.WithY(y);
    }

    internal void SetZ(double z)
    {
        CheckMutable();
        _gpVec = _gpVec.WithZ(z);
    }

    public override double Magnitude() => _gpVec.Magnitude;

    public override double SquareMagnitude() => _gpVec.SquareMagnitude;

    internal void Add(GeometryVector3D other)
    {
        CheckMutable();
        _gpVec = _gpVec.Added(other.Vec());
    }

    /// <summary>this + Other.</summary>
    public VectorWithMagnitude Added(GeometryVector3D other)
    {
        Vector3D v1 = _gpVec;
        v1 = v1.Added(other.Vec());
        return new VectorWithMagnitude(v1);
    }

    protected internal override void Cross(GeometryVector3D other)
    {
        CheckMutable();
        _gpVec = _gpVec.Crossed(other.Vec());
    }

    /// <summary>this ^ Other.</summary>
    public override GeometryVector3D Crossed(GeometryVector3D other)
    {
        Vector3D v = _gpVec;
        v = v.Crossed(other.Vec());
        return new VectorWithMagnitude(v);
    }

    protected internal override void CrossCross(GeometryVector3D v1, GeometryVector3D v2)
    {
        CheckMutable();
        _gpVec = _gpVec.CrossCrossed(v1.Vec(), v2.Vec());
    }

    /// <summary>this ^ (V1 ^ V2).</summary>
    public override GeometryVector3D CrossCrossed(GeometryVector3D v1, GeometryVector3D v2)
    {
        Vector3D v = _gpVec;
        v = v.CrossCrossed(v1.Vec(), v2.Vec());
        return new VectorWithMagnitude(v);
    }

    internal void Divide(double scalar)
    {
        CheckMutable();
        _gpVec = _gpVec.Divided(scalar);
    }

    /// <summary>this / Scalar.</summary>
    public VectorWithMagnitude Divided(double scalar)
    {
        Vector3D v = _gpVec;
        v = v.Divided(scalar);
        return new VectorWithMagnitude(v);
    }

    /// <summary>this * Scalar.</summary>
    public VectorWithMagnitude Multiplied(double scalar)
    {
        Vector3D v = _gpVec;
        v = v.Multiplied(scalar);
        return new VectorWithMagnitude(v);
    }

    internal void Multiply(double scalar)
    {
        CheckMutable();
        _gpVec = _gpVec.Multiplied(scalar);
    }

    /// <exception cref="ConstructionErrorException">The magnitude is at most <c>gp::Resolution()</c>.</exception>
    internal void Normalize()
    {
        CheckMutable();
        _gpVec = _gpVec.Normalized();
    }

    /// <summary>The unit vector of this one.</summary>
    /// <exception cref="ConstructionErrorException">The magnitude is at most <c>gp::Resolution()</c>.</exception>
    public VectorWithMagnitude Normalized()
    {
        Vector3D v = _gpVec;
        v = v.Normalized();
        return new VectorWithMagnitude(v);
    }

    internal void Subtract(GeometryVector3D other)
    {
        CheckMutable();
        _gpVec = _gpVec.Subtracted(other.Vec());
    }

    /// <summary>this - Other.</summary>
    public VectorWithMagnitude Subtracted(GeometryVector3D other)
    {
        Vector3D v = _gpVec;
        v = v.Subtracted(other.Vec());
        return new VectorWithMagnitude(v);
    }

    /// <summary>Applies T to this vector.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        _gpVec = _gpVec.Transformed(t);
    }

    // Non-mutating twins of the Set methods (Kerf)

    public VectorWithMagnitude WithCoord(double x, double y, double z)
    {
        VectorWithMagnitude copy = (VectorWithMagnitude)Copy();
        copy.SetCoord(x, y, z);
        return copy;
    }

    public VectorWithMagnitude WithVec(in Vector3D v)
    {
        VectorWithMagnitude copy = (VectorWithMagnitude)Copy();
        copy.SetVec(v);
        return copy;
    }

    public VectorWithMagnitude WithX(double x)
    {
        VectorWithMagnitude copy = (VectorWithMagnitude)Copy();
        copy.SetX(x);
        return copy;
    }

    public VectorWithMagnitude WithY(double y)
    {
        VectorWithMagnitude copy = (VectorWithMagnitude)Copy();
        copy.SetY(y);
        return copy;
    }

    public VectorWithMagnitude WithZ(double z)
    {
        VectorWithMagnitude copy = (VectorWithMagnitude)Copy();
        copy.SetZ(z);
        return copy;
    }
}
