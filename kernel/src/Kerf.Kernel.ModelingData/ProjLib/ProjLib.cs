// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// Projection of elementary 3D points and curves on elementary surfaces, giving their parametric (2D) form.
/// </summary>
internal static partial class ProjLib
{
    public static Point2D Project(in Plane3D pl, in Point3D p)
    {
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(pl, p, out u, out v);
        return new Point2D(u, v);
    }

    public static Line2D Project(in Plane3D pl, in Line3D l)
    {
        PlaneProjector proj = new(pl, l);
        return proj.Line();
    }

    public static Circle2D Project(in Plane3D pl, in Circle3D c)
    {
        PlaneProjector proj = new(pl, c);
        return proj.Circle();
    }

    public static Ellipse2D Project(in Plane3D pl, in Ellipse3D e)
    {
        PlaneProjector proj = new(pl, e);
        return proj.Ellipse();
    }

    public static Parabola2D Project(in Plane3D pl, in Parabola3D p)
    {
        PlaneProjector proj = new(pl, p);
        return proj.Parabola();
    }

    public static Hyperbola2D Project(in Plane3D pl, in Hyperbola3D h)
    {
        PlaneProjector proj = new(pl, h);
        return proj.Hyperbola();
    }

    public static Point2D Project(in Cylinder cy, in Point3D p)
    {
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(cy, p, out u, out v);
        return new Point2D(u, v);
    }

    /// <exception cref="NoSuchObjectException">The line is not parallel to the axis.</exception>
    public static Line2D Project(in Cylinder cy, in Line3D l)
    {
        CylinderProjector proj = new(cy, l);
        return proj.Line();
    }

    /// <exception cref="NoSuchObjectException">The circle is not normal to the axis.</exception>
    public static Line2D Project(in Cylinder cy, in Circle3D ci)
    {
        CylinderProjector proj = new(cy, ci);
        return proj.Line();
    }

    public static Point2D Project(in Cone co, in Point3D p)
    {
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(co, p, out u, out v);
        return new Point2D(u, v);
    }

    /// <exception cref="NoSuchObjectException">The line is not a generatrix of the cone.</exception>
    public static Line2D Project(in Cone co, in Line3D l)
    {
        ConeProjector proj = new(co, l);
        return proj.Line();
    }

    /// <remarks>As in OCCT, a circle not normal to the axis gives the default line: ProjLib_Cone sets the type to
    /// Line before it checks.</remarks>
    public static Line2D Project(in Cone co, in Circle3D ci)
    {
        ConeProjector proj = new(co, ci);
        return proj.Line();
    }

    public static Point2D Project(in Sphere sp, in Point3D p)
    {
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(sp, p, out u, out v);
        return new Point2D(u, v);
    }

    /// <exception cref="NoSuchObjectException">The circle is not an isoline.</exception>
    public static Line2D Project(in Sphere sp, in Circle3D ci)
    {
        SphereProjector proj = new(sp, ci);
        return proj.Line();
    }

    public static Point2D Project(in Torus to, in Point3D p)
    {
        double u, v;
        global::Kerf.Engine.Geometry.ElSLib.Parameters(to, p, out u, out v);
        return new Point2D(u, v);
    }

    public static Line2D Project(in Torus to, in Circle3D ci)
    {
        TorusProjector proj = new(to, ci);
        return proj.Line();
    }

    /// <summary>
    /// Makes the 2D curve of a projection's result type: a new line or conic, or the projection's B-spline curve
    /// itself (frozen, shared with <paramref name="pc"/>, as OCCT shares the handle).
    /// </summary>
    /// <exception cref="MethodNotImplementedException">The result is a Bezier curve or another curve.</exception>
    public static void MakePCurveOfType(ProjectedCurve pc, out Curve2D c2D)
    {
        switch (pc.GetType())
        {
            case CurveKind.Line:
                c2D = new LineCurve2D(pc.Line());
                break;
            case CurveKind.Circle:
                c2D = new CircleCurve2D(pc.Circle());
                break;
            case CurveKind.Ellipse:
                c2D = new EllipseCurve2D(pc.Ellipse());
                break;
            case CurveKind.Parabola:
                c2D = new ParabolaCurve2D(pc.Parabola());
                break;
            case CurveKind.Hyperbola:
                c2D = new HyperbolaCurve2D(pc.Hyperbola());
                break;
            case CurveKind.BSplineCurve:
                c2D = pc.BSpline();
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
    public static bool IsAnaSurf(SurfaceAdaptorBase theAS)
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
