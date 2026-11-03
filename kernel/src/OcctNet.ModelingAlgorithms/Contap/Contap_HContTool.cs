// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_HContTool.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_HContTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// The tool of the contour computation for what is not in the adaptors: samples on surfaces and arcs, the bounds of
/// the arcs, projection on an arc, and the vertices' parameters and tolerances on the arcs.
/// </summary>
/// <remarks>
/// A struct with static members (decision 338), so that it can be the boundary search's tool
/// (<see cref="Contap_TheSearch"/>, through <see cref="IIntStart_SOBTool"/>). OCCT keeps the sampling bounds that
/// <see cref="NbSamplePoints"/> sets and <see cref="SamplePoint"/> reads in file-scope statics; they are read across
/// calls, so they are <c>[ThreadStatic]</c> here (plan 2.3, decision 1075).
/// </remarks>
public readonly struct Contap_HContTool : IIntStart_SOBTool
{
    // OCCT: static double uinf, vinf, usup, vsup; (decision 1075)
    [ThreadStatic] private static double uinf;
    [ThreadStatic] private static double vinf;
    [ThreadStatic] private static double usup;
    [ThreadStatic] private static double vsup;

    /// <summary>The number of samples in V of a surface, by its type.</summary>
    public static int NbSamplesV(SurfaceAdaptorBase S, double v1, double v2)
    {
        int nbs;
        SurfaceKind typS = S.GetType();
        switch (typS)
        {
            case Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + S.VPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                //-- occ::handle<Geom_BSplineSurface>& HBS=S->BSpline();
                nbs = S.VKnotCount;
                nbs *= S.VDegree();
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
    public static int NbSamplesU(SurfaceAdaptorBase S, double u1, double u2)
    {
        int nbs;
        SurfaceKind typS = S.GetType();
        switch (typS)
        {
            case Plane:
                nbs = 2;
                break;
            case SurfaceKind.BezierSurface:
                nbs = 3 + S.UPoleCount;
                break;
            case SurfaceKind.BSplineSurface:
                //-- occ::handle<Geom_BSplineSurface>& HBS=S->BSpline();
                nbs = S.UKnotCount;
                nbs *= S.UDegree();
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
    public static int NbSamplePoints(SurfaceAdaptorBase S)
    {
        uinf = S.FirstUParameter();
        usup = S.LastUParameter();
        vinf = S.FirstVParameter();
        vsup = S.LastVParameter();

        if (usup < uinf)
        {
            double temp = uinf;
            uinf = usup;
            usup = temp;
        }
        if (vsup < vinf)
        {
            double temp = vinf;
            vinf = vsup;
            vsup = temp;
        }
        if (uinf == RealFirst() && usup == RealLast())
        {
            uinf = -1.0e5;
            usup = 1.0e5;
        }
        else if (uinf == RealFirst())
        {
            uinf = usup - 2.0e5;
        }
        else if (usup == RealLast())
        {
            usup = uinf + 2.0e5;
        }

        if (vinf == RealFirst() && vsup == RealLast())
        {
            vinf = -1.0e5;
            vsup = 1.0e5;
        }
        else if (vinf == RealFirst())
        {
            vinf = vsup - 2.0e5;
        }
        else if (vsup == RealLast())
        {
            vsup = vinf + 2.0e5;
        }
        if (S.GetType() == SurfaceKind.BSplineSurface)
        {
            int m = (NbSamplesU(S, uinf, usup) / 3) * (NbSamplesV(S, vinf, vsup) / 3);
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

    /// <summary>The parameters of the sample point <paramref name="Index"/>, in the bounds the last <see cref="NbSamplePoints"/> set.</summary>
    public static void SamplePoint(SurfaceAdaptorBase S, int Index, out double U, out double V)
    {
        if (S.GetType() == SurfaceKind.BSplineSurface)
        {
            int nbIntU = NbSamplesU(S, uinf, usup) / 3;
            int nbIntV = NbSamplesV(S, vinf, vsup) / 3;
            if (nbIntU * nbIntV > 5)
            {
                int indU = (Index - 1) / nbIntU;        //----   0 --> nbIntV
                int indV = (Index - 1) - indU * nbIntU; //----   0 --> nbIntU

                U = uinf + ((usup - uinf) / ((double)(nbIntU + 1))) * (double)(indU + 1);
                V = vinf + ((vsup - vinf) / ((double)(nbIntV + 2))) * (double)(indV + 1);
                return;
            }
        }

        switch (Index)
        {
            case 1:
                U = 0.75 * uinf + 0.25 * usup; // 0.25;
                V = 0.75 * vinf + 0.25 * vsup; // 0.25;
                break;
            case 2:
                U = 0.75 * uinf + 0.25 * usup; // 0.25;
                V = 0.25 * vinf + 0.75 * vsup; // 0.75;
                break;
            case 3:
                U = 0.25 * uinf + 0.75 * usup; // 0.75;
                V = 0.75 * vinf + 0.25 * vsup; // 0.25;
                break;
            case 4:
                U = 0.25 * uinf + 0.75 * usup; // 0.75;
                V = 0.25 * vinf + 0.75 * vsup; // 0.75;
                break;
            default:
                U = 0.5 * (uinf + usup); // 0.5;
                V = 0.5 * (vinf + vsup); // 0.5;
                break;
        }
    }

    /// <summary>True if all the intersection points and segments on the arc are known (never, here).</summary>
    public static bool HasBeenSeen(Curve2DAdaptorBase C) => false;

    /// <summary>The number of samples on an arc, by its type.</summary>
    public static int NbSamplesOnArc(Curve2DAdaptorBase A)
    {
        CurveKind CurveType = A.GetType();

        double nbsOnC = 5;
        switch (CurveType)
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
                nbsOnC = A.PoleCount;
                break;
            case BSplineCurve:
            {
                //-- occ::handle<Geom2d_BSplineCurve>& BSC=A->BSpline();
                nbsOnC = 2 + A.KnotCount * A.Degree();
                break;
            }
            default:
                nbsOnC = 10;
                break;
        }
        return (int)(nbsOnC);
    }

    /// <summary>The parametric limits of the arc.</summary>
    public static void Bounds(Curve2DAdaptorBase A, out double Ufirst, out double Ulast)
    {
        Ufirst = A.FirstParameter();
        Ulast = A.LastParameter();
    }

    /// <summary>
    /// Projects <paramref name="P"/> on the arc: true and the parameter and point of the nearest projection, or false.
    /// </summary>
    public static bool Project(Curve2DAdaptorBase C, in Point2D P, out double Paramproj, out Point2D Ptproj)
    {
        double epsX = 1.0e-8;
        int Nbu = 20;
        double Tol = 1.0e-5;
        double Dist2;
        Paramproj = 0.0;
        Ptproj = new Point2D();

        EpcOfExtPC2d extrema = new(P, C, Nbu, epsX, Tol);
        if (!extrema.IsDone)
        {
            return false;
        }
        int Nbext = extrema.ExtCount;
        if (Nbext == 0)
        {
            return false;
        }
        int indexmin = 1;
        Dist2 = extrema.SquareDistance(1);
        for (int i = 2; i <= Nbext; i++)
        {
            if (extrema.SquareDistance(i) < Dist2)
            {
                indexmin = i;
                Dist2 = extrema.SquareDistance(i);
            }
        }
        Paramproj = extrema.Point(indexmin).Parameter();
        Ptproj = extrema.Point(indexmin).Value();
        return true;
    }

    /// <summary>The parametric tolerance within which the vertex and another point on the arc meet.</summary>
    public static double Tolerance(HVertex V, Curve2DAdaptorBase C) => V.Resolution(C);

    /// <summary>The parameter of the vertex on the arc.</summary>
    public static double Parameter(HVertex V, Curve2DAdaptorBase C) => V.Parameter(C);

    /// <summary>The number of known intersection points on the arc (none).</summary>
    public static int NbPoints(Curve2DAdaptorBase C) => 0;

    /// <summary>A known intersection point (there is none: throws Standard_OutOfRange).</summary>
    public static void Value(Curve2DAdaptorBase C, int Index, out Point3D Pt, out double Tol, out double U) =>
        throw new OutOfRangeException();

    public static bool IsVertex(Curve2DAdaptorBase C, int Index) => false;

    /// <summary>The vertex of a known point (there is none: throws Standard_OutOfRange).</summary>
    public static void Vertex(Curve2DAdaptorBase C, int Index, out HVertex V) =>
        throw new OutOfRangeException();

    /// <summary>The number of known solution segments on the arc (none).</summary>
    public static int NbSegments(Curve2DAdaptorBase C) => 0;

    public static bool HasFirstPoint(Curve2DAdaptorBase C, int Index, out int IndFirst) =>
        throw new OutOfRangeException();

    public static bool HasLastPoint(Curve2DAdaptorBase C, int Index, out int IndLast) =>
        throw new OutOfRangeException();

    /// <summary>True when the whole arc is a solution (never known, here).</summary>
    public static bool IsAllSolution(Curve2DAdaptorBase C) => false;
}
