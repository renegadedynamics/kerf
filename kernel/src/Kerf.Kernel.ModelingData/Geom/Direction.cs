// Port of: src/ModelingData/TKG3d/Geom/Geom_Direction.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Direction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>A unit vector in 3D space (a shareable <see cref="Direction3D"/>); never null.</summary>
internal class Direction : GeometryVector3D
{
    /// <summary>The unit vector of (X, Y, Z).</summary>
    /// <exception cref="ConstructionErrorException">sqrt(X*X + Y*Y + Z*Z) is at most <c>gp::Resolution()</c>.</exception>
    public Direction(double x, double y, double z)
    {
        double d = Math.Sqrt(x * x + y * y + z * z);
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(),
                                            "Geom_Direction() - input vector has zero length");
        _gpVec = new Vector3D(x / d, y / d, z / d);
    }

    /// <summary>A direction equal to V.</summary>
    public Direction(in Direction3D v) => _gpVec = new Vector3D(v);

    /// <summary>A new, unfrozen direction equal to this one.</summary>
    public override Geometry3D Copy()
    {
        Direction d;
        d = new Direction(new Direction3D(_gpVec));
        return d;
    }

    internal void SetDir(in Direction3D v)
    {
        CheckMutable();
        _gpVec = new Vector3D(v);
    }

    /// <summary>The direction as a <see cref="Direction3D"/>.</summary>
    public Direction3D Dir() => new Direction3D(_gpVec);

    /// <summary>1.</summary>
    public override double Magnitude() => 1.0;

    /// <summary>1.</summary>
    public override double SquareMagnitude() => 1.0;

    internal void SetCoord(double x, double y, double z)
    {
        CheckMutable();
        double d = Math.Sqrt(x * x + y * y + z * z);
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(),
                                            "Geom_Direction::SetCoord() - input vector has zero length");
        _gpVec = new Vector3D(x / d, y / d, z / d);
    }

    internal void SetX(double x)
    {
        CheckMutable();
        double d = Math.Sqrt(x * x + _gpVec.Y * _gpVec.Y + _gpVec.Z * _gpVec.Z);
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(),
                                            "Geom_Direction::SetX() - input vector has zero length");
        _gpVec = new Vector3D(x / d, _gpVec.Y / d, _gpVec.Z / d);
    }

    internal void SetY(double y)
    {
        CheckMutable();
        double d = Math.Sqrt(_gpVec.X * _gpVec.X + y * y + _gpVec.Z * _gpVec.Z);
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(),
                                            "Geom_Direction::SetY() - input vector has zero length");
        _gpVec = new Vector3D(_gpVec.X / d, y / d, _gpVec.Z / d);
    }

    internal void SetZ(double z)
    {
        CheckMutable();
        double d = Math.Sqrt(_gpVec.X * _gpVec.X + _gpVec.Y * _gpVec.Y + z * z);
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(),
                                            "Geom_Direction::SetZ() - input vector has zero length");
        _gpVec = new Vector3D(_gpVec.X / d, _gpVec.Y / d, z / d);
    }

    /// <summary>Replaces this direction by this ^ Other.</summary>
    /// <exception cref="ConstructionErrorException">The vectors are parallel.</exception>
    protected internal override void Cross(GeometryVector3D other)
    {
        CheckMutable();
        Direction3D v = new Direction3D(_gpVec.Crossed(other.Vec()));
        _gpVec = new Vector3D(v);
    }

    /// <summary>Replaces this direction by this ^ (V1 ^ V2).</summary>
    /// <exception cref="ConstructionErrorException">V1 and V2, or this and V1 ^ V2, are parallel.</exception>
    protected internal override void CrossCross(GeometryVector3D v1, GeometryVector3D v2)
    {
        CheckMutable();
        Direction3D v = new Direction3D(_gpVec.CrossCrossed(v1.Vec(), v2.Vec()));
        _gpVec = new Vector3D(v);
    }

    /// <summary>The direction of this ^ Other.</summary>
    /// <exception cref="ConstructionErrorException">The vectors are parallel.</exception>
    public override GeometryVector3D Crossed(GeometryVector3D other)
    {
        Direction3D v = new Direction3D(_gpVec.Crossed(other.Vec()));
        return new Direction(v);
    }

    /// <summary>The direction of this ^ (V1 ^ V2).</summary>
    /// <exception cref="ConstructionErrorException">V1 and V2, or this and V1 ^ V2, are parallel.</exception>
    public override GeometryVector3D CrossCrossed(GeometryVector3D v1, GeometryVector3D v2)
    {
        Direction3D v = new Direction3D(_gpVec.CrossCrossed(v1.Vec(), v2.Vec()));
        return new Direction(v);
    }

    /// <summary>Applies T to this direction, then normalizes it.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        Direction3D v = new Direction3D(_gpVec);
        v = v.Transformed(t);
        _gpVec = new Vector3D(v);
    }

    // Non-mutating twins of the Set methods (Kerf)

    public Direction WithCoord(double x, double y, double z)
    {
        Direction copy = (Direction)Copy();
        copy.SetCoord(x, y, z);
        return copy;
    }

    public Direction WithDir(in Direction3D v)
    {
        Direction copy = (Direction)Copy();
        copy.SetDir(v);
        return copy;
    }

    public Direction WithX(double x)
    {
        Direction copy = (Direction)Copy();
        copy.SetX(x);
        return copy;
    }

    public Direction WithY(double y)
    {
        Direction copy = (Direction)Copy();
        copy.SetY(y);
        return copy;
    }

    public Direction WithZ(double z)
    {
        Direction copy = (Direction)Copy();
        copy.SetZ(z);
        return copy;
    }
}
