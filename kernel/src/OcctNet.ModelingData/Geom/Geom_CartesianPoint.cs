// Port of: src/ModelingData/TKG3d/Geom/Geom_CartesianPoint.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_CartesianPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A point in 3D space defined by its Cartesian coordinates (a shareable <see cref="gp_Pnt"/>).</summary>
public class Geom_CartesianPoint : Geom_Point
{
    private gp_Pnt gpPnt;

    /// <summary>A point equal to P.</summary>
    public Geom_CartesianPoint(in gp_Pnt P) => gpPnt = P;

    /// <summary>The point (X, Y, Z).</summary>
    public Geom_CartesianPoint(double X, double Y, double Z) => gpPnt = new gp_Pnt(X, Y, Z);

    /// <summary>A new, unfrozen point equal to this one.</summary>
    public override Geom_Geometry Copy()
    {
        Geom_CartesianPoint P;
        P = new Geom_CartesianPoint(gpPnt);
        return P;
    }

    internal void SetCoord(double X, double Y, double Z)
    {
        CheckMutable();
        gpPnt.SetCoord(X, Y, Z);
    }

    internal void SetPnt(in gp_Pnt P)
    {
        CheckMutable();
        gpPnt = P;
    }

    internal void SetX(double X)
    {
        CheckMutable();
        gpPnt.SetX(X);
    }

    internal void SetY(double Y)
    {
        CheckMutable();
        gpPnt.SetY(Y);
    }

    internal void SetZ(double Z)
    {
        CheckMutable();
        gpPnt.SetZ(Z);
    }

    public override void Coord(out double X, out double Y, out double Z) => gpPnt.Coord(out X, out Y, out Z);

    public override gp_Pnt Pnt() => gpPnt;

    public override double X() => gpPnt.X();

    public override double Y() => gpPnt.Y();

    public override double Z() => gpPnt.Z();

    /// <summary>Applies T to this point.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        gpPnt.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom_CartesianPoint WithCoord(double X, double Y, double Z)
    {
        Geom_CartesianPoint aCopy = (Geom_CartesianPoint)Copy();
        aCopy.SetCoord(X, Y, Z);
        return aCopy;
    }

    public Geom_CartesianPoint WithPnt(in gp_Pnt P)
    {
        Geom_CartesianPoint aCopy = (Geom_CartesianPoint)Copy();
        aCopy.SetPnt(P);
        return aCopy;
    }

    public Geom_CartesianPoint WithX(double X)
    {
        Geom_CartesianPoint aCopy = (Geom_CartesianPoint)Copy();
        aCopy.SetX(X);
        return aCopy;
    }

    public Geom_CartesianPoint WithY(double Y)
    {
        Geom_CartesianPoint aCopy = (Geom_CartesianPoint)Copy();
        aCopy.SetY(Y);
        return aCopy;
    }

    public Geom_CartesianPoint WithZ(double Z)
    {
        Geom_CartesianPoint aCopy = (Geom_CartesianPoint)Copy();
        aCopy.SetZ(Z);
        return aCopy;
    }
}
