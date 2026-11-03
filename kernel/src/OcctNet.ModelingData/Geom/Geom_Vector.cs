// Port of: src/ModelingData/TKG3d/Geom/Geom_Vector.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Vector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The common behaviour of vectors in 3D space: unit vectors (<see cref="Geom_Direction"/>) and vectors with a
/// magnitude (<see cref="Geom_VectorWithMagnitude"/>).
/// </summary>
public abstract class Geom_Vector : Geom_Geometry
{
    protected Vector3D gpVec;

    /// <summary>Reverses the vector in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        gpVec.Reverse();
    }

    /// <summary>A reversed copy.</summary>
    public Geom_Vector Reversed()
    {
        Geom_Vector V = (Copy() as Geom_Vector)!;
        V.Reverse();
        return V;
    }

    /// <summary>The angle to Other, in [0, PI].</summary>
    /// <exception cref="NullVectorException">Either magnitude is at most <c>gp::Resolution()</c>.</exception>
    public double Angle(Geom_Vector Other) => gpVec.Angle(Other.Vec());

    /// <summary>
    /// The angle to Other, in [-PI, PI], positive if this ^ Other has the orientation of VRef.
    /// </summary>
    /// <exception cref="DomainErrorException">The three vectors are coplanar and this and Other are not parallel.</exception>
    /// <exception cref="NullVectorException">A magnitude is at most <c>gp::Resolution()</c>.</exception>
    public double AngleWithRef(Geom_Vector Other, Geom_Vector VRef) => gpVec.AngleWithRef(Other.Vec(), VRef.Vec());

    /// <summary>The coordinates.</summary>
    public void Coord(out double X, out double Y, out double Z) => gpVec.Coord(out X, out Y, out Z);

    /// <summary>The magnitude.</summary>
    public abstract double Magnitude();

    /// <summary>The square of the magnitude.</summary>
    public abstract double SquareMagnitude();

    /// <summary>The X coordinate.</summary>
    public double X() => gpVec.X();

    /// <summary>The Y coordinate.</summary>
    public double Y() => gpVec.Y();

    /// <summary>The Z coordinate.</summary>
    public double Z() => gpVec.Z();

    /// <summary>Replaces this vector by this ^ Other.</summary>
    /// <exception cref="ConstructionErrorException">This is a direction and the vectors are parallel.</exception>
    protected internal abstract void Cross(Geom_Vector Other);

    /// <summary>this ^ Other, as a new vector of the same kind.</summary>
    /// <exception cref="ConstructionErrorException">This is a direction and the vectors are parallel.</exception>
    public abstract Geom_Vector Crossed(Geom_Vector Other);

    /// <summary>Replaces this vector by this ^ (V1 ^ V2).</summary>
    protected internal abstract void CrossCross(Geom_Vector V1, Geom_Vector V2);

    /// <summary>this ^ (V1 ^ V2), as a new vector of the same kind.</summary>
    public abstract Geom_Vector CrossCrossed(Geom_Vector V1, Geom_Vector V2);

    /// <summary>The scalar product with Other.</summary>
    public double Dot(Geom_Vector Other) => gpVec.Dot(Other.Vec());

    /// <summary>The triple scalar product this . (V1 ^ V2).</summary>
    public double DotCross(Geom_Vector V1, Geom_Vector V2) => gpVec.DotCross(V1.Vec(), V2.Vec());

    /// <summary>The vector as a <see cref="Vector3D"/>.</summary>
    public Vector3D Vec() => gpVec;
}
