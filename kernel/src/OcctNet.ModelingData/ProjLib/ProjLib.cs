// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// Projection of elementary 3D points and curves on elementary surfaces, giving their parametric (2D) form.
/// </summary>
public static partial class ProjLib
{
    public static gp_Pnt2d Project(in gp_Pln Pl, in gp_Pnt P)
    {
        double U, V;
        ElSLib.Parameters(Pl, P, out U, out V);
        return new gp_Pnt2d(U, V);
    }

    public static gp_Lin2d Project(in gp_Pln Pl, in gp_Lin L)
    {
        ProjLib_Plane Proj = new(Pl, L);
        return Proj.Line();
    }

    public static gp_Circ2d Project(in gp_Pln Pl, in gp_Circ C)
    {
        ProjLib_Plane Proj = new(Pl, C);
        return Proj.Circle();
    }

    public static gp_Elips2d Project(in gp_Pln Pl, in gp_Elips E)
    {
        ProjLib_Plane Proj = new(Pl, E);
        return Proj.Ellipse();
    }

    public static gp_Parab2d Project(in gp_Pln Pl, in gp_Parab P)
    {
        ProjLib_Plane Proj = new(Pl, P);
        return Proj.Parabola();
    }

    public static gp_Hypr2d Project(in gp_Pln Pl, in gp_Hypr H)
    {
        ProjLib_Plane Proj = new(Pl, H);
        return Proj.Hyperbola();
    }

    public static gp_Pnt2d Project(in gp_Cylinder Cy, in gp_Pnt P)
    {
        double U, V;
        ElSLib.Parameters(Cy, P, out U, out V);
        return new gp_Pnt2d(U, V);
    }

    /// <exception cref="Standard_NoSuchObject">The line is not parallel to the axis.</exception>
    public static gp_Lin2d Project(in gp_Cylinder Cy, in gp_Lin L)
    {
        ProjLib_Cylinder Proj = new(Cy, L);
        return Proj.Line();
    }

    /// <exception cref="Standard_NoSuchObject">The circle is not normal to the axis.</exception>
    public static gp_Lin2d Project(in gp_Cylinder Cy, in gp_Circ Ci)
    {
        ProjLib_Cylinder Proj = new(Cy, Ci);
        return Proj.Line();
    }

    public static gp_Pnt2d Project(in gp_Cone Co, in gp_Pnt P)
    {
        double U, V;
        ElSLib.Parameters(Co, P, out U, out V);
        return new gp_Pnt2d(U, V);
    }

    /// <exception cref="Standard_NoSuchObject">The line is not a generatrix of the cone.</exception>
    public static gp_Lin2d Project(in gp_Cone Co, in gp_Lin L)
    {
        ProjLib_Cone Proj = new(Co, L);
        return Proj.Line();
    }

    /// <remarks>As in OCCT, a circle not normal to the axis gives the default line: ProjLib_Cone sets the type to
    /// Line before it checks.</remarks>
    public static gp_Lin2d Project(in gp_Cone Co, in gp_Circ Ci)
    {
        ProjLib_Cone Proj = new(Co, Ci);
        return Proj.Line();
    }

    public static gp_Pnt2d Project(in gp_Sphere Sp, in gp_Pnt P)
    {
        double U, V;
        ElSLib.Parameters(Sp, P, out U, out V);
        return new gp_Pnt2d(U, V);
    }

    /// <exception cref="Standard_NoSuchObject">The circle is not an isoline.</exception>
    public static gp_Lin2d Project(in gp_Sphere Sp, in gp_Circ Ci)
    {
        ProjLib_Sphere Proj = new(Sp, Ci);
        return Proj.Line();
    }

    public static gp_Pnt2d Project(in gp_Torus To, in gp_Pnt P)
    {
        double U, V;
        ElSLib.Parameters(To, P, out U, out V);
        return new gp_Pnt2d(U, V);
    }

    public static gp_Lin2d Project(in gp_Torus To, in gp_Circ Ci)
    {
        ProjLib_Torus Proj = new(To, Ci);
        return Proj.Line();
    }

    /// <summary>
    /// Makes the 2D curve of a projection's result type: a new line or conic, or the projection's B-spline curve
    /// itself (frozen, shared with <paramref name="PC"/>, as OCCT shares the handle).
    /// </summary>
    /// <exception cref="Standard_NotImplemented">The result is a Bezier curve or another curve.</exception>
    public static void MakePCurveOfType(ProjLib_ProjectedCurve PC, out Geom2d_Curve C2D)
    {
        switch (PC.GetType())
        {
            case GeomAbs_CurveType.GeomAbs_Line:
                C2D = new Geom2d_Line(PC.Line());
                break;
            case GeomAbs_CurveType.GeomAbs_Circle:
                C2D = new Geom2d_Circle(PC.Circle());
                break;
            case GeomAbs_CurveType.GeomAbs_Ellipse:
                C2D = new Geom2d_Ellipse(PC.Ellipse());
                break;
            case GeomAbs_CurveType.GeomAbs_Parabola:
                C2D = new Geom2d_Parabola(PC.Parabola());
                break;
            case GeomAbs_CurveType.GeomAbs_Hyperbola:
                C2D = new Geom2d_Hyperbola(PC.Hyperbola());
                break;
            case GeomAbs_CurveType.GeomAbs_BSplineCurve:
                C2D = PC.BSpline();
                break;
            case GeomAbs_CurveType.GeomAbs_BezierCurve:
            case GeomAbs_CurveType.GeomAbs_OtherCurve:
            default:
                throw new Standard_NotImplemented("ProjLib::MakePCurveOfType");
        }
    }

    /// <summary>
    /// True if the surface is analytical: a plane, cylinder, cone, sphere or torus.
    /// </summary>
    public static bool IsAnaSurf(Adaptor3d_Surface theAS)
    {
        switch (theAS.GetType())
        {
            case GeomAbs_Plane:
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
                return true;
            default:
                return false;
        }
    }
}
