// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_CartesianPoint.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_CartesianPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A point in the plane defined by its Cartesian coordinates (a shareable <see cref="gp_Pnt2d"/>).</summary>
public class Geom2d_CartesianPoint : Geom2d_Point
{
    private gp_Pnt2d gpPnt2d;

    /// <summary>A point equal to P.</summary>
    public Geom2d_CartesianPoint(in gp_Pnt2d P) => gpPnt2d = P;

    /// <summary>The point (X, Y).</summary>
    public Geom2d_CartesianPoint(double X, double Y) => gpPnt2d = new gp_Pnt2d(X, Y);

    /// <summary>A new, unfrozen point equal to this one.</summary>
    public override Geom2d_Geometry Copy()
    {
        Geom2d_CartesianPoint P;
        P = new Geom2d_CartesianPoint(gpPnt2d);
        return P;
    }

    internal void SetCoord(double X, double Y)
    {
        CheckMutable();
        gpPnt2d.SetCoord(X, Y);
    }

    public override void Coord(out double X, out double Y) => gpPnt2d.Coord(out X, out Y);

    internal void SetPnt2d(in gp_Pnt2d P)
    {
        CheckMutable();
        gpPnt2d = P;
    }

    internal void SetX(double X)
    {
        CheckMutable();
        gpPnt2d.SetX(X);
    }

    internal void SetY(double Y)
    {
        CheckMutable();
        gpPnt2d.SetY(Y);
    }

    public override gp_Pnt2d Pnt2d() => gpPnt2d;

    public override double X() => gpPnt2d.X();

    public override double Y() => gpPnt2d.Y();

    /// <summary>Applies T to this point.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        gpPnt2d.Transform(T);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    public Geom2d_CartesianPoint WithCoord(double X, double Y)
    {
        Geom2d_CartesianPoint aCopy = (Geom2d_CartesianPoint)Copy();
        aCopy.SetCoord(X, Y);
        return aCopy;
    }

    public Geom2d_CartesianPoint WithPnt2d(in gp_Pnt2d P)
    {
        Geom2d_CartesianPoint aCopy = (Geom2d_CartesianPoint)Copy();
        aCopy.SetPnt2d(P);
        return aCopy;
    }

    public Geom2d_CartesianPoint WithX(double X)
    {
        Geom2d_CartesianPoint aCopy = (Geom2d_CartesianPoint)Copy();
        aCopy.SetX(X);
        return aCopy;
    }

    public Geom2d_CartesianPoint WithY(double Y)
    {
        Geom2d_CartesianPoint aCopy = (Geom2d_CartesianPoint)Copy();
        aCopy.SetY(Y);
        return aCopy;
    }
}
