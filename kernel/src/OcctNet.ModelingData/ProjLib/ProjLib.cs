// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// Projection of elementary 3D points and curves on elementary surfaces, giving their parametric (2D) form.
/// </summary>
public static partial class ProjLib
{
    public static Point2D Project(in Plane3D Pl, in Point3D P)
    {
        double U, V;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Pl, P, out U, out V);
        return new Point2D(U, V);
    }

    public static Line2D Project(in Plane3D Pl, in Line3D L)
    {
        ProjLib_Plane Proj = new(Pl, L);
        return Proj.Line();
    }

    public static Circle2D Project(in Plane3D Pl, in Circle3D C)
    {
        ProjLib_Plane Proj = new(Pl, C);
        return Proj.Circle();
    }

    public static Ellipse2D Project(in Plane3D Pl, in Ellipse3D E)
    {
        ProjLib_Plane Proj = new(Pl, E);
        return Proj.Ellipse();
    }

    public static Parabola2D Project(in Plane3D Pl, in Parabola3D P)
    {
        ProjLib_Plane Proj = new(Pl, P);
        return Proj.Parabola();
    }

    public static Hyperbola2D Project(in Plane3D Pl, in Hyperbola3D H)
    {
        ProjLib_Plane Proj = new(Pl, H);
        return Proj.Hyperbola();
    }

    public static Point2D Project(in Cylinder Cy, in Point3D P)
    {
        double U, V;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Cy, P, out U, out V);
        return new Point2D(U, V);
    }

    /// <exception cref="NoSuchObjectException">The line is not parallel to the axis.</exception>
    public static Line2D Project(in Cylinder Cy, in Line3D L)
    {
        ProjLib_Cylinder Proj = new(Cy, L);
        return Proj.Line();
    }

    /// <exception cref="NoSuchObjectException">The circle is not normal to the axis.</exception>
    public static Line2D Project(in Cylinder Cy, in Circle3D Ci)
    {
        ProjLib_Cylinder Proj = new(Cy, Ci);
        return Proj.Line();
    }

    public static Point2D Project(in Cone Co, in Point3D P)
    {
        double U, V;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Co, P, out U, out V);
        return new Point2D(U, V);
    }

    /// <exception cref="NoSuchObjectException">The line is not a generatrix of the cone.</exception>
    public static Line2D Project(in Cone Co, in Line3D L)
    {
        ProjLib_Cone Proj = new(Co, L);
        return Proj.Line();
    }

    /// <remarks>As in OCCT, a circle not normal to the axis gives the default line: ProjLib_Cone sets the type to
    /// Line before it checks.</remarks>
    public static Line2D Project(in Cone Co, in Circle3D Ci)
    {
        ProjLib_Cone Proj = new(Co, Ci);
        return Proj.Line();
    }

    public static Point2D Project(in Sphere Sp, in Point3D P)
    {
        double U, V;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(Sp, P, out U, out V);
        return new Point2D(U, V);
    }

    /// <exception cref="NoSuchObjectException">The circle is not an isoline.</exception>
    public static Line2D Project(in Sphere Sp, in Circle3D Ci)
    {
        ProjLib_Sphere Proj = new(Sp, Ci);
        return Proj.Line();
    }

    public static Point2D Project(in Torus To, in Point3D P)
    {
        double U, V;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(To, P, out U, out V);
        return new Point2D(U, V);
    }

    public static Line2D Project(in Torus To, in Circle3D Ci)
    {
        ProjLib_Torus Proj = new(To, Ci);
        return Proj.Line();
    }

    /// <summary>
    /// Makes the 2D curve of a projection's result type: a new line or conic, or the projection's B-spline curve
    /// itself (frozen, shared with <paramref name="PC"/>, as OCCT shares the handle).
    /// </summary>
    /// <exception cref="MethodNotImplementedException">The result is a Bezier curve or another curve.</exception>
    public static void MakePCurveOfType(ProjLib_ProjectedCurve PC, out Geom2d_Curve C2D)
    {
        switch (PC.GetType())
        {
            case CurveKind.Line:
                C2D = new Geom2d_Line(PC.Line());
                break;
            case CurveKind.Circle:
                C2D = new Geom2d_Circle(PC.Circle());
                break;
            case CurveKind.Ellipse:
                C2D = new Geom2d_Ellipse(PC.Ellipse());
                break;
            case CurveKind.Parabola:
                C2D = new Geom2d_Parabola(PC.Parabola());
                break;
            case CurveKind.Hyperbola:
                C2D = new Geom2d_Hyperbola(PC.Hyperbola());
                break;
            case CurveKind.BSplineCurve:
                C2D = PC.BSpline();
                break;
            case CurveKind.BezierCurve:
            case CurveKind.OtherCurve:
            default:
                throw new MethodNotImplementedException("ProjLib::MakePCurveOfType");
        }
    }

    /// <summary>
    /// True if the surface is analytical: a plane, cylinder, cone, sphere or torus.
    /// </summary>
    public static bool IsAnaSurf(Adaptor3d_Surface theAS)
    {
        switch (theAS.GetType())
        {
            case Plane:
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
                return true;
            default:
                return false;
        }
    }
}
