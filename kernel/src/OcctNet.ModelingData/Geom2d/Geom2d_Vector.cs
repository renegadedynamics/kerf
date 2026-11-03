// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Vector.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Vector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The common behaviour of vectors in the plane: unit vectors (<see cref="Geom2d_Direction"/>) and vectors with a
/// magnitude (<see cref="Geom2d_VectorWithMagnitude"/>).
/// </summary>
public abstract class Geom2d_Vector : Geom2d_Geometry
{
    protected Vector2D gpVec2d;

    /// <summary>Reverses the vector in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        gpVec2d.Reverse();
    }

    /// <summary>A reversed copy.</summary>
    public Geom2d_Vector Reversed()
    {
        Geom2d_Vector V = (Copy() as Geom2d_Vector)!;
        V.Reverse();
        return V;
    }

    /// <summary>The angle from this vector to Other, in [-PI, PI].</summary>
    /// <exception cref="NullVectorException">Either vector has a null magnitude.</exception>
    public double Angle(Geom2d_Vector Other) => gpVec2d.Angle(Other.Vec2d());

    /// <summary>The coordinates.</summary>
    public void Coord(out double X, out double Y) => gpVec2d.Coord(out X, out Y);

    /// <summary>The magnitude.</summary>
    public abstract double Magnitude();

    /// <summary>The square of the magnitude.</summary>
    public abstract double SquareMagnitude();

    /// <summary>The X coordinate.</summary>
    public double X() => gpVec2d.X();

    /// <summary>The Y coordinate.</summary>
    public double Y() => gpVec2d.Y();

    /// <summary>The cross product this ^ Other (a scalar in 2D).</summary>
    public abstract double Crossed(Geom2d_Vector Other);

    /// <summary>The scalar product with Other.</summary>
    public double Dot(Geom2d_Vector Other) => gpVec2d.Dot(Other.Vec2d());

    /// <summary>The vector as a <see cref="Vector2D"/>.</summary>
    public Vector2D Vec2d() => gpVec2d;
}
