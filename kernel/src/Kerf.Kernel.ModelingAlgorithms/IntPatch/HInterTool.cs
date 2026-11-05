// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HInterTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HInterTool.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_HInterTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The tool of the surface/surface intersection for what is not in the adaptors: samples on surfaces and arcs, the
/// bounds of the arcs, projection on an arc, and the vertices' parameters and tolerances on the arcs.
/// </summary>
/// <remarks>
/// A struct: OCCT's class has static members and a sampling state (<see cref="NbSamplePoints"/> sets the bounds that
/// <see cref="SamplePoint"/> reads). It implements <see cref="ISobTool"/>, as the boundary search's tool
/// (IntPatch_TheSOnBounds). The "known points" queries say that no arc has been seen, as OCCT's do.
/// </remarks>
internal struct HInterTool : ISobTool
{
    private double _uinf;
    private double _vinf;
    private double _usup;
    private double _vsup;

    public HInterTool()
    {
        _uinf = 0.0;
        _vinf = 0.0;
        _usup = 0.0;
        _vsup = 0.0;
    }

    public static bool SingularOnUMin(SurfaceAdaptorBase s) => false;

    public static bool SingularOnUMax(SurfaceAdaptorBase s) => false;

    public static bool SingularOnVMin(SurfaceAdaptorBase s) => false;

    public static bool SingularOnVMax(SurfaceAdaptorBase s) => false;

    /// <summary>The number of samples in U of a surface, by its type.</summary>
    public static int NbSamplesU(SurfaceAdaptorBase s, double u1, double u2)
    {
        switch (s.GetType())
        {
            case Plane:
                return 2;
            case SurfaceKind.BezierSurface:
                return (3 + s.UPoleCount);
            case SurfaceKind.BSplineSurface:
            {
                int nbs = s.UKnotCount;
                nbs *= s.UDegree();
                if (!s.IsURational)
                {
                    nbs *= 2;
                }
                if (nbs < 4)
                {
                    nbs = 4;
                }
                return nbs;
            }
            case SurfaceKind.Torus:
                return 20;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
            case SurfaceKind.OffsetSurface:
            case OtherSurface:
                return 10;
        }
        return 10;
    }

    /// <summary>The number of samples in V of a surface, by its type.</summary>
    public static int NbSamplesV(SurfaceAdaptorBase s, double v1, double v2)
    {
        switch (s.GetType())
        {
            case Plane:
                return 2;
            case SurfaceKind.BezierSurface:
                return (3 + s.VPoleCount);
            case SurfaceKind.BSplineSurface:
            {
                int nbs = s.VKnotCount;
                nbs *= s.VDegree();
                if (!s.IsVRational)
                {
                    nbs *= 2;
                }
                if (nbs < 4)
                {
                    nbs = 4;
                }
                return nbs;
            }
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            case SurfaceKind.Torus:
            case SurfaceOfRevolution:
            case SurfaceOfExtrusion:
                return 15;

            case SurfaceKind.OffsetSurface:
            case OtherSurface:
                return 10;
        }
        return 10;
    }

    /// <summary>
    /// The number of sample points of a surface; sets the (finite) bounds that <see cref="SamplePoint"/> uses.
    /// </summary>
    public int NbSamplePoints(SurfaceAdaptorBase s)
    {
        _uinf = s.FirstUParameter();
        _usup = s.LastUParameter();
        _vinf = s.FirstVParameter();
        _vsup = s.LastVParameter();

        if (_usup < _uinf)
        {
            double temp = _uinf;
            _uinf = _usup;
            _usup = temp;
        }
        if (_vsup < _vinf)
        {
            double temp = _vinf;
            _vinf = _vsup;
            _vsup = temp;
        }
        if (_uinf == RealFirst() && _usup == RealLast())
        {
            _uinf = -1.0e5;
            _usup = 1.0e5;
        }
        else if (_uinf == RealFirst())
        {
            _uinf = _usup - 2.0e5;
        }
        else if (_usup == RealLast())
        {
            _usup = _uinf + 2.0e5;
        }

        if (_vinf == RealFirst() && _vsup == RealLast())
        {
            _vinf = -1.0e5;
            _vsup = 1.0e5;
        }
        else if (_vinf == RealFirst())
        {
            _vinf = _vsup - 2.0e5;
        }
        else if (_vsup == RealLast())
        {
            _vsup = _vinf + 2.0e5;
        }
        int m = (1 + NbSamplesU(s, _uinf, _usup) / 2) * (1 + NbSamplesV(s, _vinf, _vsup) / 2);
        return (m);
    }

    /// <summary>The parameters of the sample point of range <paramref name="index"/> (after <see cref="NbSamplePoints"/>).</summary>
    public readonly void SamplePoint(SurfaceAdaptorBase s, int index, out double u, out double v)
    {
        int nbIntU = 1 + NbSamplesU(s, _uinf, _usup);
        nbIntU >>= 1;
        int nbIntV = 1 + NbSamplesV(s, _vinf, _vsup);
        nbIntV >>= 1;

        if (nbIntU * nbIntV > 5)
        {
            int nv = (index - 1) / nbIntU;
            int nu = (index - 1) - nv * nbIntU;
            double du = ((_usup - _uinf) / ((double)(nbIntU + 1)));
            double dv = ((_vsup - _vinf) / ((double)(nbIntV + 1)));

            int perturb = (nu + nv) & 3;

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

            u = _uinf + du * (double)(nu + 1);
            v = _vinf + dv * (double)(nv + 1);
            return;
        }

        switch (index)
        {
            case 1:
                u = 0.76 * _uinf + 0.24 * _usup;
                v = 0.74 * _vinf + 0.26 * _vsup;
                break;
            case 2:
                u = 0.73 * _uinf + 0.27 * _usup;
                v = 0.24 * _vinf + 0.76 * _vsup;
                break;
            case 3:
                u = 0.25 * _uinf + 0.75 * _usup;
                v = 0.76 * _vinf + 0.24 * _vsup;
                break;
            case 4:
                u = 0.26 * _uinf + 0.74 * _usup;
                v = 0.25 * _vinf + 0.75 * _vsup;
                break;
            default:
                u = 0.51 * _uinf + 0.49 * _usup;
                v = 0.49 * _vinf + 0.51 * _vsup;
                break;
        }
    }

    /// <summary>True if all the intersection points and segments on the arc are known (never, here).</summary>
    public static bool HasBeenSeen(Curve2DAdaptorBase c) => false;

    /// <summary>The number of samples on an arc, by its type.</summary>
    public static int NbSamplesOnArc(Curve2DAdaptorBase a)
    {
        CurveKind curveType = a.GetType();

        //  double nbsOnC = 5;
        int nbsOnC = 5;
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
        return (nbsOnC);
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
