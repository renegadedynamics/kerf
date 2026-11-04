// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_HContTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_HContTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// The tool of the contour computation for what is not in the adaptors: samples on surfaces and arcs, the bounds of
/// the arcs, projection on an arc, and the vertices' parameters and tolerances on the arcs.
/// </summary>
/// <remarks>
/// A struct with static members (decision 338), so that it can be the boundary search's tool
/// (<see cref="TheSearch"/>, through <see cref="ISobTool"/>). OCCT keeps the sampling bounds that
/// <see cref="NbSamplePoints"/> sets and <see cref="SamplePoint"/> reads in file-scope statics; they are read across
/// calls, so they are <c>[ThreadStatic]</c> here (plan 2.3, decision 1075).
/// </remarks>
internal readonly struct HContTool : ISobTool
{
    // OCCT: static double uinf, vinf, usup, vsup; (decision 1075)
    [ThreadStatic] private static double s_uinf;
    [ThreadStatic] private static double s_vinf;
    [ThreadStatic] private static double s_usup;
    [ThreadStatic] private static double s_vsup;

    /// <summary>The number of samples in V of a surface, by its type.</summary>
    public static int NbSamplesV(SurfaceAdaptorBase s, double v1, double v2)
    {
        int nbs;
        SurfaceKind typS = s.GetType();
        switch (typS)
        {
            case Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + s.VPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                //-- occ::handle<Geom_BSplineSurface>& HBS=S->BSpline();
                nbs = s.VKnotCount;
                nbs *= s.VDegree();
                if (nbs < 2)
                {
                    nbs = 2;
                }
                break;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
                nbs = 15;
                break;
            default:
                nbs = 10;
                break;
        }
        return (nbs);
    }

    /// <summary>The number of samples in U of a surface, by its type.</summary>
    public static int NbSamplesU(SurfaceAdaptorBase s, double u1, double u2)
    {
        int nbs;
        SurfaceKind typS = s.GetType();
        switch (typS)
        {
            case Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + s.UPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                //-- occ::handle<Geom_BSplineSurface>& HBS=S->BSpline();
                nbs = s.UKnotCount;
                nbs *= s.UDegree();
                if (nbs < 2)
                {
                    nbs = 2;
                }
                break;
            case SurfaceKind.Torus:
                nbs = 20;
                break;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
                nbs = 10;
                break;
            default:
                nbs = 10;
                break;
        }
        return (nbs);
    }

    /// <summary>The number of sample points of the surface; sets the sampling bounds <see cref="SamplePoint"/> reads.</summary>
    public static int NbSamplePoints(SurfaceAdaptorBase s)
    {
        s_uinf = s.FirstUParameter();
        s_usup = s.LastUParameter();
        s_vinf = s.FirstVParameter();
        s_vsup = s.LastVParameter();

        if (s_usup < s_uinf)
        {
            double temp = s_uinf;
            s_uinf = s_usup;
            s_usup = temp;
        }
        if (s_vsup < s_vinf)
        {
            double temp = s_vinf;
            s_vinf = s_vsup;
            s_vsup = temp;
        }
        if (s_uinf == RealFirst() && s_usup == RealLast())
        {
            s_uinf = -1.0e5;
            s_usup = 1.0e5;
        }
        else if (s_uinf == RealFirst())
        {
            s_uinf = s_usup - 2.0e5;
        }
        else if (s_usup == RealLast())
        {
            s_usup = s_uinf + 2.0e5;
        }

        if (s_vinf == RealFirst() && s_vsup == RealLast())
        {
            s_vinf = -1.0e5;
            s_vsup = 1.0e5;
        }
        else if (s_vinf == RealFirst())
        {
            s_vinf = s_vsup - 2.0e5;
        }
        else if (s_vsup == RealLast())
        {
            s_vsup = s_vinf + 2.0e5;
        }
        if (s.GetType() == SurfaceKind.BSplineSurface)
        {
            int m = (NbSamplesU(s, s_uinf, s_usup) / 3) * (NbSamplesV(s, s_vinf, s_vsup) / 3);
            if (m > 5)
            {
                return (m);
            }
            else
            {
                return (5);
            }
        }
        else
        {
            return 5;
        }
    }

    /// <summary>The parameters of the sample point <paramref name="index"/>, in the bounds the last <see cref="NbSamplePoints"/> set.</summary>
    public static void SamplePoint(SurfaceAdaptorBase s, int index, out double u, out double v)
    {
        if (s.GetType() == SurfaceKind.BSplineSurface)
        {
            int nbIntU = NbSamplesU(s, s_uinf, s_usup) / 3;
            int nbIntV = NbSamplesV(s, s_vinf, s_vsup) / 3;
            if (nbIntU * nbIntV > 5)
            {
                int indU = (index - 1) / nbIntU;        //----   0 --> nbIntV
                int indV = (index - 1) - indU * nbIntU; //----   0 --> nbIntU

                u = s_uinf + ((s_usup - s_uinf) / ((double)(nbIntU + 1))) * (double)(indU + 1);
                v = s_vinf + ((s_vsup - s_vinf) / ((double)(nbIntV + 2))) * (double)(indV + 1);
                return;
            }
        }

        switch (index)
        {
            case 1:
                u = 0.75 * s_uinf + 0.25 * s_usup; // 0.25;
                v = 0.75 * s_vinf + 0.25 * s_vsup; // 0.25;
                break;
            case 2:
                u = 0.75 * s_uinf + 0.25 * s_usup; // 0.25;
                v = 0.25 * s_vinf + 0.75 * s_vsup; // 0.75;
                break;
            case 3:
                u = 0.25 * s_uinf + 0.75 * s_usup; // 0.75;
                v = 0.75 * s_vinf + 0.25 * s_vsup; // 0.25;
                break;
            case 4:
                u = 0.25 * s_uinf + 0.75 * s_usup; // 0.75;
                v = 0.25 * s_vinf + 0.75 * s_vsup; // 0.75;
                break;
            default:
                u = 0.5 * (s_uinf + s_usup); // 0.5;
                v = 0.5 * (s_vinf + s_vsup); // 0.5;
                break;
        }
    }

    /// <summary>True if all the intersection points and segments on the arc are known (never, here).</summary>
    public static bool HasBeenSeen(Curve2DAdaptorBase c) => false;

    /// <summary>The number of samples on an arc, by its type.</summary>
    public static int NbSamplesOnArc(Curve2DAdaptorBase a)
    {
        CurveKind curveType = a.GetType();

        double nbsOnC = 5;
        switch (curveType)
        {
            case Line:
                nbsOnC = 2;
                break;
            case Circle:
            case Ellipse:
            case Hyperbola:
            case Parabola:
                nbsOnC = 10;
                break;
            case BezierCurve:
                nbsOnC = a.PoleCount;
                break;
            case BSplineCurve:
            {
                //-- occ::handle<Geom2d_BSplineCurve>& BSC=A->BSpline();
                nbsOnC = 2 + a.KnotCount * a.Degree();
                break;
            }
            default:
                nbsOnC = 10;
                break;
        }
        return (int)(nbsOnC);
    }

    /// <summary>The parametric limits of the arc.</summary>
    public static void Bounds(Curve2DAdaptorBase a, out double ufirst, out double ulast)
    {
        ufirst = a.FirstParameter();
        ulast = a.LastParameter();
    }

    /// <summary>
    /// Projects <paramref name="p"/> on the arc: true and the parameter and point of the nearest projection, or false.
    /// </summary>
    public static bool Project(Curve2DAdaptorBase c, in Point2D p, out double paramproj, out Point2D ptproj)
    {
        double epsX = 1.0e-8;
        int nbu = 20;
        double tol = 1.0e-5;
        double dist2;
        paramproj = 0.0;
        ptproj = new Point2D();

        EpcOfExtPC2d extrema = new(p, c, nbu, epsX, tol);
        if (!extrema.IsDone)
        {
            return false;
        }
        int nbext = extrema.ExtCount;
        if (nbext == 0)
        {
            return false;
        }
        int indexmin = 1;
        dist2 = extrema.SquareDistance(1);
        for (int i = 2; i <= nbext; i++)
        {
            if (extrema.SquareDistance(i) < dist2)
            {
                indexmin = i;
                dist2 = extrema.SquareDistance(i);
            }
        }
        paramproj = extrema.Point(indexmin).Parameter();
        ptproj = extrema.Point(indexmin).Value();
        return true;
    }

    /// <summary>The parametric tolerance within which the vertex and another point on the arc meet.</summary>
    public static double Tolerance(HVertex v, Curve2DAdaptorBase c) => v.Resolution(c);

    /// <summary>The parameter of the vertex on the arc.</summary>
    public static double Parameter(HVertex v, Curve2DAdaptorBase c) => v.Parameter(c);

    /// <summary>The number of known intersection points on the arc (none).</summary>
    public static int NbPoints(Curve2DAdaptorBase c) => 0;

    /// <summary>A known intersection point (there is none: throws Standard_OutOfRange).</summary>
    public static void Value(Curve2DAdaptorBase c, int index, out Point3D pt, out double tol, out double u) =>
        throw new OutOfRangeException();

    public static bool IsVertex(Curve2DAdaptorBase c, int index) => false;

    /// <summary>The vertex of a known point (there is none: throws Standard_OutOfRange).</summary>
    public static void Vertex(Curve2DAdaptorBase c, int index, out HVertex v) =>
        throw new OutOfRangeException();

    /// <summary>The number of known solution segments on the arc (none).</summary>
    public static int NbSegments(Curve2DAdaptorBase c) => 0;

    public static bool HasFirstPoint(Curve2DAdaptorBase c, int index, out int indFirst) =>
        throw new OutOfRangeException();

    public static bool HasLastPoint(Curve2DAdaptorBase c, int index, out int indLast) =>
        throw new OutOfRangeException();

    /// <summary>True when the whole arc is a solution (never known, here).</summary>
    public static bool IsAllSolution(Curve2DAdaptorBase c) => false;
}
