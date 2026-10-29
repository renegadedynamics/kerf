// Port of: src/ModelingData/TKG3d/Geom/Geom_Direction.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Direction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A unit vector in 3D space (a shareable <see cref="gp_Dir"/>); never null.</summary>
public class Geom_Direction : Geom_Vector
{
    /// <summary>The unit vector of (X, Y, Z).</summary>
    /// <exception cref="Standard_ConstructionError">sqrt(X*X + Y*Y + Z*Z) is at most <c>gp::Resolution()</c>.</exception>
    public Geom_Direction(double X, double Y, double Z)
    {
        double D = Math.Sqrt(X * X + Y * Y + Z * Z);
        Standard_ConstructionError_Raise_if(D <= gp.Resolution(),
                                            "Geom_Direction() - input vector has zero length");
        gpVec = new gp_Vec(X / D, Y / D, Z / D);
    }

    /// <summary>A direction equal to V.</summary>
    public Geom_Direction(in gp_Dir V) => gpVec = new gp_Vec(V);

    /// <summary>A new, unfrozen direction equal to this one.</summary>
    public override Geom_Geometry Copy()
    {
        Geom_Direction D;
        D = new Geom_Direction(new gp_Dir(gpVec));
        return D;
    }

    internal void SetDir(in gp_Dir V)
    {
        CheckMutable();
        gpVec = new gp_Vec(V);
    }

    /// <summary>The direction as a <see cref="gp_Dir"/>.</summary>
    public gp_Dir Dir() => new gp_Dir(gpVec);

    /// <summary>1.</summary>
    public override double Magnitude() => 1.0;

    /// <summary>1.</summary>
    public override double SquareMagnitude() => 1.0;

    internal void SetCoord(double X, double Y, double Z)
    {
        CheckMutable();
        double D = Math.Sqrt(X * X + Y * Y + Z * Z);
        Standard_ConstructionError_Raise_if(D <= gp.Resolution(),
                                            "Geom_Direction::SetCoord() - input vector has zero length");
        gpVec = new gp_Vec(X / D, Y / D, Z / D);
    }

    internal void SetX(double X)
    {
        CheckMutable();
        double D = Math.Sqrt(X * X + gpVec.Y() * gpVec.Y() + gpVec.Z() * gpVec.Z());
        Standard_ConstructionError_Raise_if(D <= gp.Resolution(),
                                            "Geom_Direction::SetX() - input vector has zero length");
        gpVec = new gp_Vec(X / D, gpVec.Y() / D, gpVec.Z() / D);
    }

    internal void SetY(double Y)
    {
        CheckMutable();
        double D = Math.Sqrt(gpVec.X() * gpVec.X() + Y * Y + gpVec.Z() * gpVec.Z());
        Standard_ConstructionError_Raise_if(D <= gp.Resolution(),
                                            "Geom_Direction::SetY() - input vector has zero length");
        gpVec = new gp_Vec(gpVec.X() / D, Y / D, gpVec.Z() / D);
    }

    internal void SetZ(double Z)
    {
        CheckMutable();
        double D = Math.Sqrt(gpVec.X() * gpVec.X() + gpVec.Y() * gpVec.Y() + Z * Z);
        Standard_ConstructionError_Raise_if(D <= gp.Resolution(),
                                            "Geom_Direction::SetZ() - input vector has zero length");
        gpVec = new gp_Vec(gpVec.X() / D, gpVec.Y() / D, Z / D);
    }

    /// <summary>Replaces this direction by this ^ Other.</summary>
    /// <exception cref="Standard_ConstructionError">The vectors are parallel.</exception>
    protected internal override void Cross(Geom_Vector Other)
    {
        CheckMutable();
        gp_Dir V = new gp_Dir(gpVec.Crossed(Other.Vec()));
        gpVec = new gp_Vec(V);
    }

    /// <summary>Replaces this direction by this ^ (V1 ^ V2).</summary>
    /// <exception cref="Standard_ConstructionError">V1 and V2, or this and V1 ^ V2, are parallel.</exception>
    protected internal override void CrossCross(Geom_Vector V1, Geom_Vector V2)
    {
        CheckMutable();
        gp_Dir V = new gp_Dir(gpVec.CrossCrossed(V1.Vec(), V2.Vec()));
        gpVec = new gp_Vec(V);
    }

    /// <summary>The direction of this ^ Other.</summary>
    /// <exception cref="Standard_ConstructionError">The vectors are parallel.</exception>
    public override Geom_Vector Crossed(Geom_Vector Other)
    {
        gp_Dir V = new gp_Dir(gpVec.Crossed(Other.Vec()));
        return new Geom_Direction(V);
    }

    /// <summary>The direction of this ^ (V1 ^ V2).</summary>
    /// <exception cref="Standard_ConstructionError">V1 and V2, or this and V1 ^ V2, are parallel.</exception>
    public override Geom_Vector CrossCrossed(Geom_Vector V1, Geom_Vector V2)
    {
        gp_Dir V = new gp_Dir(gpVec.CrossCrossed(V1.Vec(), V2.Vec()));
        return new Geom_Direction(V);
    }

    /// <summary>Applies T to this direction, then normalizes it.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        gp_Dir V = new gp_Dir(gpVec);
        V.Transform(T);
        gpVec = new gp_Vec(V);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_Direction WithCoord(double X, double Y, double Z)
    {
        Geom_Direction aCopy = (Geom_Direction)Copy();
        aCopy.SetCoord(X, Y, Z);
        return aCopy;
    }

    public Geom_Direction WithDir(in gp_Dir V)
    {
        Geom_Direction aCopy = (Geom_Direction)Copy();
        aCopy.SetDir(V);
        return aCopy;
    }

    public Geom_Direction WithX(double X)
    {
        Geom_Direction aCopy = (Geom_Direction)Copy();
        aCopy.SetX(X);
        return aCopy;
    }

    public Geom_Direction WithY(double Y)
    {
        Geom_Direction aCopy = (Geom_Direction)Copy();
        aCopy.SetY(Y);
        return aCopy;
    }

    public Geom_Direction WithZ(double Z)
    {
        Geom_Direction aCopy = (Geom_Direction)Copy();
        aCopy.SetZ(Z);
        return aCopy;
    }
}
