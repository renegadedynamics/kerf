// Port of: src/ModelingData/TKG3d/Geom/Geom_Vector.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Vector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>
/// The common behaviour of vectors in 3D space: unit vectors (<see cref="Direction"/>) and vectors with a
/// magnitude (<see cref="VectorWithMagnitude"/>).
/// </summary>
public abstract class GeometryVector3D : Geometry3D
{
    protected Vector3D _gpVec;

    /// <summary>Reverses the vector in place.</summary>
    internal void Reverse()
    {
        CheckMutable();
        _gpVec.Reverse();
    }

    /// <summary>A reversed copy.</summary>
    public GeometryVector3D Reversed()
    {
        GeometryVector3D v = (Copy() as GeometryVector3D)!;
        v.Reverse();
        return v;
    }

    /// <summary>The angle to Other, in [0, PI].</summary>
    /// <exception cref="NullVectorException">Either magnitude is at most <c>gp::Resolution()</c>.</exception>
    public double Angle(GeometryVector3D other) => _gpVec.Angle(other.Vec());

    /// <summary>
    /// The angle to Other, in [-PI, PI], positive if this ^ Other has the orientation of VRef.
    /// </summary>
    /// <exception cref="DomainErrorException">The three vectors are coplanar and this and Other are not parallel.</exception>
    /// <exception cref="NullVectorException">A magnitude is at most <c>gp::Resolution()</c>.</exception>
    public double AngleWithRef(GeometryVector3D other, GeometryVector3D vRef) => _gpVec.AngleWithRef(other.Vec(), vRef.Vec());

    /// <summary>The coordinates.</summary>
    public void Coord(out double x, out double y, out double z) => _gpVec.Coord(out x, out y, out z);

    /// <summary>The magnitude.</summary>
    public abstract double Magnitude();

    /// <summary>The square of the magnitude.</summary>
    public abstract double SquareMagnitude();

    /// <summary>The X coordinate.</summary>
    public double X() => _gpVec.X();

    /// <summary>The Y coordinate.</summary>
    public double Y() => _gpVec.Y();

    /// <summary>The Z coordinate.</summary>
    public double Z() => _gpVec.Z();

    /// <summary>Replaces this vector by this ^ Other.</summary>
    /// <exception cref="ConstructionErrorException">This is a direction and the vectors are parallel.</exception>
    protected internal abstract void Cross(GeometryVector3D other);

    /// <summary>this ^ Other, as a new vector of the same kind.</summary>
    /// <exception cref="ConstructionErrorException">This is a direction and the vectors are parallel.</exception>
    public abstract GeometryVector3D Crossed(GeometryVector3D other);

    /// <summary>Replaces this vector by this ^ (V1 ^ V2).</summary>
    protected internal abstract void CrossCross(GeometryVector3D v1, GeometryVector3D v2);

    /// <summary>this ^ (V1 ^ V2), as a new vector of the same kind.</summary>
    public abstract GeometryVector3D CrossCrossed(GeometryVector3D v1, GeometryVector3D v2);

    /// <summary>The scalar product with Other.</summary>
    public double Dot(GeometryVector3D other) => _gpVec.Dot(other.Vec());

    /// <summary>The triple scalar product this . (V1 ^ V2).</summary>
    public double DotCross(GeometryVector3D v1, GeometryVector3D v2) => _gpVec.DotCross(v1.Vec(), v2.Vec());

    /// <summary>The vector as a <see cref="Vector3D"/>.</summary>
    public Vector3D Vec() => _gpVec;
}
