// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HInterTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HInterTool.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HInterTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The tool of the surface/surface intersection for what is not in the adaptors: samples on surfaces and arcs, the
/// bounds of the arcs, projection on an arc, and the vertices' parameters and tolerances on the arcs.
/// </summary>
/// <remarks>
/// A struct: OCCT's class has static members and a sampling state (<see cref="NbSamplePoints"/> sets the bounds that
/// <see cref="SamplePoint"/> reads). It implements <see cref="IIntStart_SOBTool"/>, as the boundary search's tool
/// (IntPatch_TheSOnBounds). The "known points" queries say that no arc has been seen, as OCCT's do.
/// </remarks>
public struct IntPatch_HInterTool : IIntStart_SOBTool
{
    private double uinf;
    private double vinf;
    private double usup;
    private double vsup;

    public IntPatch_HInterTool()
    {
        uinf = 0.0;
        vinf = 0.0;
        usup = 0.0;
        vsup = 0.0;
    }

    public static bool SingularOnUMin(Adaptor3d_Surface S) => false;

    public static bool SingularOnUMax(Adaptor3d_Surface S) => false;

    public static bool SingularOnVMin(Adaptor3d_Surface S) => false;

    public static bool SingularOnVMax(Adaptor3d_Surface S) => false;

    /// <summary>The number of samples in U of a surface, by its type.</summary>
    public static int NbSamplesU(Adaptor3d_Surface S, double u1, double u2)
    {
        switch (S.GetType())
        {
            case GeomAbs_Plane:
                return 2;
            case GeomAbs_BezierSurface:
                return (3 + S.NbUPoles());
            case GeomAbs_BSplineSurface:
            {
                int nbs = S.NbUKnots();
                nbs *= S.UDegree();
                if (!S.IsURational())
                {
                    nbs *= 2;
                }
                if (nbs < 4)
                {
                    nbs = 4;
                }
                return nbs;
            }
            case GeomAbs_Torus:
                return 20;
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_SurfaceOfRevolution:
            case GeomAbs_SurfaceOfExtrusion:
            case GeomAbs_OffsetSurface:
            case GeomAbs_OtherSurface:
                return 10;
        }
        return 10;
    }

    /// <summary>The number of samples in V of a surface, by its type.</summary>
    public static int NbSamplesV(Adaptor3d_Surface S, double v1, double v2)
    {
        switch (S.GetType())
        {
            case GeomAbs_Plane:
                return 2;
            case GeomAbs_BezierSurface:
                return (3 + S.NbVPoles());
            case GeomAbs_BSplineSurface:
            {
                int nbs = S.NbVKnots();
                nbs *= S.VDegree();
                if (!S.IsVRational())
                {
                    nbs *= 2;
                }
                if (nbs < 4)
                {
                    nbs = 4;
                }
                return nbs;
            }
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
            case GeomAbs_Torus:
            case GeomAbs_SurfaceOfRevolution:
            case GeomAbs_SurfaceOfExtrusion:
                return 15;

            case GeomAbs_OffsetSurface:
            case GeomAbs_OtherSurface:
                return 10;
        }
        return 10;
    }

    /// <summary>
    /// The number of sample points of a surface; sets the (finite) bounds that <see cref="SamplePoint"/> uses.
    /// </summary>
    public int NbSamplePoints(Adaptor3d_Surface S)
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
        int m = (1 + NbSamplesU(S, uinf, usup) / 2) * (1 + NbSamplesV(S, vinf, vsup) / 2);
        return (m);
    }

    /// <summary>The parameters of the sample point of range <paramref name="Index"/> (after <see cref="NbSamplePoints"/>).</summary>
    public readonly void SamplePoint(Adaptor3d_Surface S, int Index, out double U, out double V)
    {
        int nbIntU = 1 + NbSamplesU(S, uinf, usup);
        nbIntU >>= 1;
        int nbIntV = 1 + NbSamplesV(S, vinf, vsup);
        nbIntV >>= 1;

        if (nbIntU * nbIntV > 5)
        {
            int NV = (Index - 1) / nbIntU;
            int NU = (Index - 1) - NV * nbIntU;
            double du = ((usup - uinf) / ((double)(nbIntU + 1)));
            double dv = ((vsup - vinf) / ((double)(nbIntV + 1)));

            int perturb = (NU + NV) & 3;

            //-- petite perturbation pou eviter les pbs de symetrie avec Les recherches de points int.

            switch (perturb)
            {
                case 1:
                    dv *= 1.001;
                    dv *= 0.999; // OCCT: scales dv twice (presumably du *= 1.001 was meant): dv * 0.999999
                    break;
                case 2:
                    du *= 1.001;
                    dv *= 1.001;
                    break;
                case 3:
                    du *= 0.999;
                    break;
                default:
                    break;
            }

            U = uinf + du * (double)(NU + 1);
            V = vinf + dv * (double)(NV + 1);
            return;
        }

        switch (Index)
        {
            case 1:
                U = 0.76 * uinf + 0.24 * usup;
                V = 0.74 * vinf + 0.26 * vsup;
                break;
            case 2:
                U = 0.73 * uinf + 0.27 * usup;
                V = 0.24 * vinf + 0.76 * vsup;
                break;
            case 3:
                U = 0.25 * uinf + 0.75 * usup;
                V = 0.76 * vinf + 0.24 * vsup;
                break;
            case 4:
                U = 0.26 * uinf + 0.74 * usup;
                V = 0.25 * vinf + 0.75 * vsup;
                break;
            default:
                U = 0.51 * uinf + 0.49 * usup;
                V = 0.49 * vinf + 0.51 * vsup;
                break;
        }
    }

    /// <summary>True if all the intersection points and segments on the arc are known (never, here).</summary>
    public static bool HasBeenSeen(Adaptor2d_Curve2d C) => false;

    /// <summary>The number of samples on an arc, by its type.</summary>
    public static int NbSamplesOnArc(Adaptor2d_Curve2d A)
    {
        GeomAbs_CurveType CurveType = A.GetType();

        //  double nbsOnC = 5;
        int nbsOnC = 5;
        switch (CurveType)
        {
            case GeomAbs_Line:
                nbsOnC = 2;
                break;
            case GeomAbs_Circle:
            case GeomAbs_Ellipse:
            case GeomAbs_Hyperbola:
            case GeomAbs_Parabola:
                nbsOnC = 10;
                break;
            case GeomAbs_BezierCurve:
                nbsOnC = A.NbPoles();
                break;
            case GeomAbs_BSplineCurve:
            {
                //-- occ::handle<Geom2d_BSplineCurve>& BSC=A->BSpline();
                nbsOnC = 2 + A.NbKnots() * A.Degree();
                break;
            }
            default:
                nbsOnC = 10;
                break;
        }
        return (nbsOnC);
    }

    /// <summary>The parametric limits of the arc.</summary>
    public static void Bounds(Adaptor2d_Curve2d A, out double Ufirst, out double Ulast)
    {
        Ufirst = A.FirstParameter();
        Ulast = A.LastParameter();
    }

    /// <summary>
    /// Projects <paramref name="P"/> on the arc: true and the parameter and point of the nearest projection, or false.
    /// </summary>
    public static bool Project(Adaptor2d_Curve2d C, in gp_Pnt2d P, out double Paramproj, out gp_Pnt2d Ptproj)
    {
        double epsX = 1.0e-8;
        int Nbu = 20;
        double Tol = 1.0e-5;
        double Dist2;
        Paramproj = 0.0;
        Ptproj = new gp_Pnt2d();

        Extrema_EPCOfExtPC2d extrema = new(P, C, Nbu, epsX, Tol);
        if (!extrema.IsDone())
        {
            return false;
        }
        int Nbext = extrema.NbExt();
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
    public static double Tolerance(Adaptor3d_HVertex V, Adaptor2d_Curve2d C) => V.Resolution(C);

    /// <summary>The parameter of the vertex on the arc.</summary>
    public static double Parameter(Adaptor3d_HVertex V, Adaptor2d_Curve2d C) => V.Parameter(C);

    /// <summary>The number of known intersection points on the arc (none).</summary>
    public static int NbPoints(Adaptor2d_Curve2d C) => 0;

    /// <summary>A known intersection point (there is none: throws Standard_OutOfRange).</summary>
    public static void Value(Adaptor2d_Curve2d C, int Index, out gp_Pnt Pt, out double Tol, out double U) =>
        throw new Standard_OutOfRange();

    public static bool IsVertex(Adaptor2d_Curve2d C, int Index) => false;

    /// <summary>The vertex of a known point (there is none: throws Standard_OutOfRange).</summary>
    public static void Vertex(Adaptor2d_Curve2d C, int Index, out Adaptor3d_HVertex V) =>
        throw new Standard_OutOfRange();

    /// <summary>The number of known solution segments on the arc (none).</summary>
    public static int NbSegments(Adaptor2d_Curve2d C) => 0;

    public static bool HasFirstPoint(Adaptor2d_Curve2d C, int Index, out int IndFirst) =>
        throw new Standard_OutOfRange();

    public static bool HasLastPoint(Adaptor2d_Curve2d C, int Index, out int IndLast) =>
        throw new Standard_OutOfRange();

    /// <summary>True when the whole arc is a solution (never known, here).</summary>
    public static bool IsAllSolution(Adaptor2d_Curve2d C) => false;
}
