// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Direction.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Direction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>A unit vector in the plane (a shareable <see cref="Direction2D"/>); never null.</summary>
internal class Direction2 : GeometryVector2D
{
    /// <summary>The unit vector of (X, Y).</summary>
    /// <exception cref="ConstructionErrorException">sqrt(X*X + Y*Y) is at most <c>gp::Resolution()</c>.</exception>
    public Direction2(double x, double y)
    {
        double d = Math.Sqrt(x * x + y * y);
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(),
                                            "Geom2d_Direction() - input vector has zero length");
        _gpVec2d = new Vector2D(x / d, y / d);
    }

    /// <summary>A direction equal to V.</summary>
    public Direction2(in Direction2D v) => _gpVec2d = new Vector2D(v);

    /// <summary>A new, unfrozen direction equal to this one.</summary>
    public override Geometry2D Copy()
    {
        Direction2 d;
        d = new Direction2(new Direction2D(_gpVec2d));
        return d;
    }

    internal void SetCoord(double x, double y)
    {
        CheckMutable();
        double d = Math.Sqrt(x * x + y * y);
        Standard_ConstructionError_Raise_if(
          d <= Constants.Resolution(),
          "Geom2d_Direction::SetCoord() - input vector has zero length");
        _gpVec2d = new Vector2D(x / d, y / d);
    }

    internal void SetDir2d(in Direction2D v)
    {
        CheckMutable();
        _gpVec2d = new Vector2D(v);
    }

    internal void SetX(double x)
    {
        CheckMutable();
        double d = Math.Sqrt(x * x + _gpVec2d.Y * _gpVec2d.Y);
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(),
                                            "Geom2d_Direction::SetX() - input vector has zero length");
        _gpVec2d = new Vector2D(x / d, _gpVec2d.Y / d);
    }

    internal void SetY(double y)
    {
        CheckMutable();
        double d = Math.Sqrt(_gpVec2d.X * _gpVec2d.X + y * y);
        Standard_ConstructionError_Raise_if(d <= Constants.Resolution(),
                                            "Geom2d_Direction::SetY() - input vector has zero length");
        _gpVec2d = new Vector2D(_gpVec2d.X / d, y / d);
    }

    /// <summary>The direction as a <see cref="Direction2D"/>.</summary>
    public Direction2D Dir2d() => new Direction2D(_gpVec2d);

    /// <summary>1.</summary>
    public override double Magnitude() => 1.0;

    /// <summary>1.</summary>
    public override double SquareMagnitude() => 1.0;

    /// <summary>The cross product this ^ Other.</summary>
    public override double Crossed(GeometryVector2D other) => _gpVec2d.Crossed(other.Vec2d());

    /// <summary>The cross product theLeft ^ Other.</summary>
    public static double operator ^(Direction2 left, GeometryVector2D other) => left.Crossed(other);

    /// <summary>Applies T to this direction, then normalizes it.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        Direction2D dir = new Direction2D(_gpVec2d);
        dir = dir.Transformed(t);
        _gpVec2d = new Vector2D(dir);
    }

    // Non-mutating twins of the Set methods (Kerf)

    public Direction2 WithCoord(double x, double y)
    {
        Direction2 copy = (Direction2)Copy();
        copy.SetCoord(x, y);
        return copy;
    }

    public Direction2 WithDir2d(in Direction2D v)
    {
        Direction2 copy = (Direction2)Copy();
        copy.SetDir2d(v);
        return copy;
    }

    public Direction2 WithX(double x)
    {
        Direction2 copy = (Direction2)Copy();
        copy.SetX(x);
        return copy;
    }

    public Direction2 WithY(double y)
    {
        Direction2 copy = (Direction2)Copy();
        copy.SetY(y);
        return copy;
    }
}
