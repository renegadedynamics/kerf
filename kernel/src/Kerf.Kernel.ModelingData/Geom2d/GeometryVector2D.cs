// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Vector.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Vector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>
/// The common behaviour of vectors in the plane: unit vectors (<see cref="Direction2"/>) and vectors with a
/// magnitude (<see cref="VectorWithMagnitude2"/>).
/// </summary>
public abstract class GeometryVector2D : Geometry2D
{
    private protected Vector2D _gpVec2d;

    /// <summary>Reverses the vector in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        _gpVec2d = _gpVec2d.Reversed();
    }

    /// <summary>A reversed copy.</summary>
    public GeometryVector2D Reversed()
    {
        GeometryVector2D v = (Copy() as GeometryVector2D)!;
        v.Reverse();
        return v;
    }

    /// <summary>The angle from this vector to Other, in [-PI, PI].</summary>
    /// <exception cref="NullVectorException">Either vector has a null magnitude.</exception>
    public double Angle(GeometryVector2D other) => _gpVec2d.AngleTo(other.Vec2d());

    /// <summary>The coordinates.</summary>
    public void Coord(out double x, out double y) => _gpVec2d.Coord(out x, out y);

    /// <summary>The magnitude.</summary>
    public abstract double Magnitude();

    /// <summary>The square of the magnitude.</summary>
    public abstract double SquareMagnitude();

    /// <summary>The X coordinate.</summary>
    public double X() => _gpVec2d.X;

    /// <summary>The Y coordinate.</summary>
    public double Y() => _gpVec2d.Y;

    /// <summary>The cross product this ^ Other (a scalar in 2D).</summary>
    public abstract double Crossed(GeometryVector2D other);

    /// <summary>The scalar product with Other.</summary>
    public double Dot(GeometryVector2D other) => _gpVec2d.Dot(other.Vec2d());

    /// <summary>The vector as a <see cref="Vector2D"/>.</summary>
    public Vector2D Vec2d() => _gpVec2d;
}
