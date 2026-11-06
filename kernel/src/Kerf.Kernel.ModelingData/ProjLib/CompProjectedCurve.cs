// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_CompProjectedCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_CompProjectedCurve.cxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_HCompProjectedCurve.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1997-09-23
// Created by: Roman BORISOV

using Kerf.Engine.Approximation;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using System.Runtime.InteropServices;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// The orthogonal projection of a 3D curve on a surface, as a 2D curve in the surface's parametric space. The
/// projection may be made of several continuous parts (<see cref="CurveCount"/>), each a sequence of (t, u, v)
/// triples found by walking along the curve; points between them are found by Newton's method. A part can be a
/// single point or an isoparametric line. <see cref="Perform"/> approximates each part by a 2D (and/or 3D)
/// B-spline curve.
/// </summary>
/// <remarks>
/// An adaptor and a working object (decisions 21 and 101): <see cref="Init"/>, <see cref="Perform"/>, the setters
/// and <c>Load</c> are public. It keeps the caller's adaptors, as OCCT keeps its handles: they must outlive the
/// projection. OCCT's <c>ProjLib_HCompProjectedCurve</c> is a typedef of this class. Evaluating one projection from
/// several threads at once is not safe (the intervals are cached).
/// </remarks>
internal class CompProjectedCurve : Curve2DAdaptorBase
{
    private const double FuncTol = 1.0e-10;

    // Null until loaded (the default constructor), as OCCT's handles.
    private SurfaceAdaptorBase _surface = null!;
    private CurveAdaptorBase _curve = null!;
    private int _nbCurves;
    private NCollection_HSequence<NCollection_HSequence<Point3D>> _sequence = null!;
    private NCollection_HArray1<bool>? _uIso;
    private NCollection_HArray1<bool>? _vIso;
    private NCollection_HArray1<bool>? _snglPnts;
    private NCollection_HArray1<double>? _maxDistance;
    private NCollection_HArray1<double>? _tabInt;
    private double _tol3d;
    private Continuity _continuity;
    private int _maxDegree;
    private int _maxSeg;
    private bool _proj2d;
    private bool _proj3d;
    private double _maxDist;
    private double _tolU;
    private double _tolV;

    private NCollection_HArray1<bool>? _resultIsPoint;
    private NCollection_HArray1<double>? _result2dUApproxError;
    private NCollection_HArray1<double>? _result2dVApproxError;
    private NCollection_HArray1<double>? _result3dApproxError;
    private NCollection_HArray1<Point3D>? _result3dPoint;
    private NCollection_HArray1<Point2D>? _result2dPoint;
    private NCollection_HArray1<Curve3D?>? _result3dCurve;
    private NCollection_HArray1<Curve2D?>? _result2dCurve;

    // Structure to perform splits computation.
    // This structure is not thread-safe since operations under Splits should be performed in a
    // critical section. PeriodicDir - 0 for U periodicity and 1 for V periodicity.
    private sealed class SplitDS
    {
        public SplitDS(CurveAdaptorBase curve, SurfaceAdaptorBase surface, List<double> splits)
        {
            myCurve = curve;
            mySurface = surface;
            Splits = splits;
            PerMinParam = 0.0;
            PerMaxParam = 0.0;
            PeriodicDir = 0;
            ExtCCCurve1 = null;
            ExtCCLast2DParam = 0.0;
            myExtPS = null;
        }

        public readonly CurveAdaptorBase myCurve;
        public readonly SurfaceAdaptorBase mySurface;
        public readonly List<double> Splits;

        public double PerMinParam;
        public double PerMaxParam;
        public int PeriodicDir;

        public CurveOnSurface? ExtCCCurve1;
        public double ExtCCLast2DParam;

        public ExtPS? myExtPS;
    }

    private static int Comparator(double a, double b)
    {
        // std::sort with theA < theB.
        return a < b ? -1 : (b < a ? 1 : 0);
    }

    private static void D1Core(double t,
                           double u,
                           double v,
                           out Vector2D V,
                           CurveAdaptorBase curve,
                           SurfaceAdaptorBase surface)
    {
        Point3D s, c;
        Vector3D DS1_u, DS1_v, DS2_u, DS2_uv, DS2_v, DC1_t;
        surface.D2(u, v, out s, out DS1_u, out DS1_v, out DS2_u, out DS2_v, out DS2_uv);
        curve.D1(t, out c, out DC1_t);
        Vector3D ort = new(c, s); // Ort = S - C

        Vector2D dE_dt = new(-DC1_t * DS1_u, -DC1_t * DS1_v);
        Xy dE_du = new(DS1_u * DS1_u + ort * DS2_u, DS1_u * DS1_v + ort * DS2_uv);
        Xy dE_dv = new(DS1_v * DS1_u + ort * DS2_uv, DS1_v * DS1_v + ort * DS2_v);

        double det = dE_du.X() * dE_dv.Y() - dE_du.Y() * dE_dv.X();
        if (Math.Abs(det) < Constants.Resolution())
        {
            throw new ConstructionErrorException();
        }

        Matrix2x2 m = new(new Xy(dE_dv.Y() / det, -dE_du.Y() / det), new Xy(-dE_dv.X() / det, dE_du.X() / det));

        V = -new Vector2D(new Vector2D(m.Row(1)) * dE_dt, new Vector2D(m.Row(2)) * dE_dt);
    }

    private static void D2Core(double t,
                           double u,
                           double v,
                           out Vector2D v1,
                           out Vector2D v2,
                           CurveAdaptorBase curve,
                           SurfaceAdaptorBase surface)
    {
        Point3D s, c;
        Vector3D DS1_u, DS1_v, DS2_u, DS2_uv, DS2_v, DS3_u, DS3_v, DS3_uuv, DS3_uvv, DC1_t, DC2_t;
        surface.D3(u, v, out s, out DS1_u, out DS1_v, out DS2_u, out DS2_v, out DS2_uv, out DS3_u, out DS3_v,
                   out DS3_uuv, out DS3_uvv);
        curve.D2(t, out c, out DC1_t, out DC2_t);
        Vector3D ort = new(c, s);

        Vector2D dE_dt = new(-DC1_t * DS1_u, -DC1_t * DS1_v);
        Xy dE_du = new(DS1_u * DS1_u + ort * DS2_u, DS1_u * DS1_v + ort * DS2_uv);
        Xy dE_dv = new(DS1_v * DS1_u + ort * DS2_uv, DS1_v * DS1_v + ort * DS2_v);

        double det = dE_du.X() * dE_dv.Y() - dE_du.Y() * dE_dv.X();
        if (Math.Abs(det) < Constants.Resolution())
        {
            throw new ConstructionErrorException();
        }

        Matrix2x2 m = new(new Xy(dE_dv.Y() / det, -dE_du.Y() / det), new Xy(-dE_dv.X() / det, dE_du.X() / det));

        // First derivative
        v1 = -new Vector2D(new Vector2D(m.Row(1)) * dE_dt, new Vector2D(m.Row(2)) * dE_dt);

        /* Second derivative */

        // Computation of d2E_dt2 = S1
        Vector2D d2E_dt = new(-DC2_t * DS1_u, -DC2_t * DS1_v);

        // Computation of 2*(d2E/dtdX)(dX/dt) = S2
        Vector2D d2E1_dtdX = new(-DC1_t * DS2_u, -DC1_t * DS2_uv);
        Vector2D d2E2_dtdX = new(-DC1_t * DS2_uv, -DC1_t * DS2_v);
        Vector2D s2 = 2 * new Vector2D(d2E1_dtdX * v1, d2E2_dtdX * v1);

        // Computation of (d2E/dX2)*(dX/dt)2 = S3

        // Row11 = (d2E1/du2, d2E1/dudv)
        double tmp;
        Vector2D row11 = new(3 * DS1_u * DS2_u + ort * DS3_u,
                             tmp = 2 * DS1_u * DS2_uv + DS1_v * DS2_u + ort * DS3_uuv);

        // Row12 = (d2E1/dudv, d2E1/dv2)
        Vector2D row12 = new(tmp, DS2_v * DS1_u + 2 * DS1_v * DS2_uv + ort * DS3_uvv);

        // Row21 = (d2E2/du2, d2E2/dudv)
        Vector2D row21 = new(DS2_u * DS1_v + 2 * DS1_u * DS2_uv + ort * DS3_uuv,
                             tmp = 2 * DS2_uv * DS1_v + DS1_u * DS2_v + ort * DS3_uvv);

        // Row22 = (d2E2/duv, d2E2/dvdv)
        Vector2D row22 = new(tmp, 3 * DS1_v * DS2_v + ort * DS3_v);

        Vector2D s3 = new(v1 * new Vector2D(row11 * v1, row12 * v1), v1 * new Vector2D(row21 * v1, row22 * v1));

        Vector2D sum = d2E_dt + s2 + s3;

        v2 = -new Vector2D(new Vector2D(m.Row(1)) * sum, new Vector2D(m.Row(2)) * sum);
    }

    private static void D2CurvOnSurf(double t,
                                     double u,
                                     double v,
                                     out Vector3D v1,
                                     out Vector3D v2,
                                     CurveAdaptorBase curve,
                                     SurfaceAdaptorBase surface)
    {
        Point3D s, c;
        Vector2D v12d, v22d;
        Vector3D DS1_u, DS1_v, DS2_u, DS2_uv, DS2_v, DS3_u, DS3_v, DS3_uuv, DS3_uvv, DC1_t, DC2_t;
        surface.D3(u, v, out s, out DS1_u, out DS1_v, out DS2_u, out DS2_v, out DS2_uv, out DS3_u, out DS3_v,
                   out DS3_uuv, out DS3_uvv);
        curve.D2(t, out c, out DC1_t, out DC2_t);
        Vector3D ort = new(c, s);

        Vector2D dE_dt = new(-DC1_t * DS1_u, -DC1_t * DS1_v);
        Xy dE_du = new(DS1_u * DS1_u + ort * DS2_u, DS1_u * DS1_v + ort * DS2_uv);
        Xy dE_dv = new(DS1_v * DS1_u + ort * DS2_uv, DS1_v * DS1_v + ort * DS2_v);

        double det = dE_du.X() * dE_dv.Y() - dE_du.Y() * dE_dv.X();
        if (Math.Abs(det) < Constants.Resolution())
        {
            throw new ConstructionErrorException();
        }

        Matrix2x2 m = new(new Xy(dE_dv.Y() / det, -dE_du.Y() / det), new Xy(-dE_dv.X() / det, dE_du.X() / det));

        // First derivative
        v12d = -new Vector2D(new Vector2D(m.Row(1)) * dE_dt, new Vector2D(m.Row(2)) * dE_dt);

        /* Second derivative */

        // Computation of d2E_dt2 = S1
        Vector2D d2E_dt = new(-DC2_t * DS1_u, -DC2_t * DS1_v);

        // Computation of 2*(d2E/dtdX)(dX/dt) = S2
        Vector2D d2E1_dtdX = new(-DC1_t * DS2_u, -DC1_t * DS2_uv);
        Vector2D d2E2_dtdX = new(-DC1_t * DS2_uv, -DC1_t * DS2_v);
        Vector2D s2 = 2 * new Vector2D(d2E1_dtdX * v12d, d2E2_dtdX * v12d);

        // Computation of (d2E/dX2)*(dX/dt)2 = S3

        // Row11 = (d2E1/du2, d2E1/dudv)
        double tmp;
        Vector2D row11 = new(3 * DS1_u * DS2_u + ort * DS3_u,
                             tmp = 2 * DS1_u * DS2_uv + DS1_v * DS2_u + ort * DS3_uuv);

        // Row12 = (d2E1/dudv, d2E1/dv2)
        Vector2D row12 = new(tmp, DS2_v * DS1_u + 2 * DS1_v * DS2_uv + ort * DS3_uvv);

        // Row21 = (d2E2/du2, d2E2/dudv)
        Vector2D row21 = new(DS2_u * DS1_v + 2 * DS1_u * DS2_uv + ort * DS3_uuv,
                             tmp = 2 * DS2_uv * DS1_v + DS1_u * DS2_v + ort * DS3_uvv);

        // Row22 = (d2E2/duv, d2E2/dvdv)
        Vector2D row22 = new(tmp, 3 * DS1_v * DS2_v + ort * DS3_v);

        Vector2D s3 = new(v12d * new Vector2D(row11 * v12d, row12 * v12d),
                          v12d * new Vector2D(row21 * v12d, row22 * v12d));

        Vector2D sum = d2E_dt + s2 + s3;

        v22d = -new Vector2D(new Vector2D(m.Row(1)) * sum, new Vector2D(m.Row(2)) * sum);

        v1 = DS1_u * v12d.X + DS1_v * v12d.Y;
        v2 = DS2_u * v12d.X * v12d.X + DS1_u * v22d.X + 2 * DS2_uv * v12d.X * v12d.Y
             + DS2_v * v12d.Y * v12d.Y + DS1_v * v22d.Y;
    }

    private static bool ExactBound(ref Point3D sol,
                                   double notSol,
                                   double tol,
                                   double tolU,
                                   double tolV,
                                   CurveAdaptorBase curve,
                                   SurfaceAdaptorBase surface)
    {
        double u0, v0, t, t1, t2, firstU, lastU, firstV, lastV;
        Point2D pOnS;
        u0 = sol.Y;
        v0 = sol.Z;
        firstU = surface.FirstUParameter();
        lastU = surface.LastUParameter();
        firstV = surface.FirstVParameter();
        lastV = surface.LastVParameter();
        // Here we have to compute the boundary that projection is going to intersect
        Vector2D d2d;
        // these variables are to estimate which boundary has more opportunity
        // to be intersected
        double ru1, ru2, rv1, rv2;
        D1Core(sol.X, u0, v0, out d2d, curve, surface);
        // Here we assume that D2d != (0, 0)
        if (Math.Abs(d2d.X) < Constants.Resolution())
        {
            ru1 = global::Kerf.Engine.Foundation.Precision.Infinite();
            ru2 = global::Kerf.Engine.Foundation.Precision.Infinite();
            rv1 = v0 - firstV;
            rv2 = lastV - v0;
        }
        else if (Math.Abs(d2d.Y) < Constants.Resolution())
        {
            ru1 = u0 - firstU;
            ru2 = lastU - u0;
            rv1 = global::Kerf.Engine.Foundation.Precision.Infinite();
            rv2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        else
        {
            ru1 = new Point2D(u0, v0).DistanceTo(new Point2D(firstU, v0 + (firstU - u0) * d2d.Y / d2d.X));
            ru2 = new Point2D(u0, v0).DistanceTo(new Point2D(lastU, v0 + (lastU - u0) * d2d.Y / d2d.X));
            rv1 = new Point2D(u0, v0).DistanceTo(new Point2D(u0 + (firstV - v0) * d2d.X / d2d.Y, firstV));
            rv2 = new Point2D(u0, v0).DistanceTo(new Point2D(u0 + (lastV - v0) * d2d.X / d2d.Y, lastV));
        }
        NCollection_Sequence<Point3D> seq = new();
        seq.Append(new Point3D(firstU, ru1, 2));
        seq.Append(new Point3D(lastU, ru2, 2));
        seq.Append(new Point3D(firstV, rv1, 3));
        seq.Append(new Point3D(lastV, rv2, 3));
        int i, j;
        for (i = 1; i <= 3; i++)
        {
            for (j = 1; j <= 4 - i; j++)
            {
                if (seq.Value(j).Y < seq.Value(j + 1).Y)
                {
                    Point3D swp;
                    swp = seq.Value(j + 1);
                    seq.ChangeValue(j + 1) = seq.Value(j);
                    seq.ChangeValue(j) = swp;
                }
            }
        }

        t = sol.X;
        t1 = Min(sol.X, notSol);
        t2 = Max(sol.X, notSol);

        bool isDone = false;
        while (!seq.IsEmpty())
        {
            Point3D p;
            p = seq.Last();
            seq.Remove(seq.Length());
            PrjResolve prjPS = new(curve, surface, (int)p.Z);
            if ((int)p.Z == 2)
            {
                prjPS.Perform(t,
                               p.X,
                               v0,
                               new Point2D(tol, tolV),
                               new Point2D(t1, surface.FirstVParameter()),
                               new Point2D(t2, surface.LastVParameter()),
                               FuncTol);
                if (!prjPS.IsDone)
                {
                    continue;
                }
                pOnS = prjPS.Solution();
                sol = new Point3D(pOnS.X, p.X, pOnS.Y);
                isDone = true;
                break;
            }
            else
            {
                prjPS.Perform(t,
                               u0,
                               p.X,
                               new Point2D(tol, tolU),
                               new Point2D(t1, surface.FirstUParameter()),
                               new Point2D(t2, surface.LastUParameter()),
                               FuncTol);
                if (!prjPS.IsDone)
                {
                    continue;
                }
                pOnS = prjPS.Solution();
                sol = new Point3D(pOnS.X, pOnS.Y, p.X);
                isDone = true;
                break;
            }
        }

        return isDone;
    }

    private static void DichExactBound(ref Point3D sol,
                                       double notSol,
                                       double tol,
                                       double tolU,
                                       double tolV,
                                       CurveAdaptorBase curve,
                                       SurfaceAdaptorBase surface)
    {
        double u0, v0, t;
        Point2D pOnS;
        u0 = sol.Y;
        v0 = sol.Z;
        PrjResolve prjPS = new(curve, surface, 1);

        double aNotSol = notSol;
        while (Math.Abs(sol.X - aNotSol) > tol)
        {
            t = (sol.X + aNotSol) / 2;
            prjPS.Perform(t,
                           u0,
                           v0,
                           new Point2D(tolU, tolV),
                           new Point2D(surface.FirstUParameter(), surface.FirstVParameter()),
                           new Point2D(surface.LastUParameter(), surface.LastVParameter()),
                           FuncTol,
                           true);

            if (prjPS.IsDone)
            {
                pOnS = prjPS.Solution();
                sol = new Point3D(t, pOnS.X, pOnS.Y);
                u0 = sol.Y;
                v0 = sol.Z;
            }
            else
            {
                aNotSol = t;
            }
        }
    }

    /// <remarks><paramref name="u"/> and <paramref name="v"/> are 0 when it returns false, where OCCT leaves them
    /// unchanged; no caller reads them then (decision 32).</remarks>
    private static bool InitialPoint(in Point3D point,
                                     double t,
                                     CurveAdaptorBase c,
                                     SurfaceAdaptorBase s,
                                     double tolU,
                                     double tolV,
                                     out double u,
                                     out double v,
                                     double maxDist)
    {
        PrjResolve prjPS = new(c, s, 1);
        double parU, parV;
        ExtPS extPS = new(point,
                                   s,
                                   s.FirstUParameter(),
                                   s.LastUParameter(),
                                   s.FirstVParameter(),
                                   s.LastVParameter(),
                                   tolU,
                                   tolV,
                                   ExtFlag_MIN);
        int argmin = 0;
        double aMaxDist = maxDist;
        if (aMaxDist > 0.0)
        {
            aMaxDist *= aMaxDist;
        }
        if (extPS.IsDone && extPS.ExtCount != 0)
        {
            int i, nend;
            // Search for the nearest solution which is also a normal projection
            nend = extPS.ExtCount;
            for (i = 1; i <= nend; i++)
            {
                if (aMaxDist > 0.0 && aMaxDist < extPS.SquareDistance(i))
                {
                    continue;
                }
                POnSurf pOnS = extPS.Point(i);
                pOnS.Parameter(out parU, out parV);
                prjPS.Perform(t,
                               parU,
                               parV,
                               new Point2D(tolU, tolV),
                               new Point2D(s.FirstUParameter(), s.FirstVParameter()),
                               new Point2D(s.LastUParameter(), s.LastVParameter()),
                               FuncTol,
                               true);
                if (prjPS.IsDone)
                {
                    if (argmin == 0 || extPS.SquareDistance(i) < extPS.SquareDistance(argmin))
                    {
                        argmin = i;
                    }
                }
            }
        }
        if (argmin == 0)
        {
            u = 0.0;
            v = 0.0;
            return false;
        }
        else
        {
            POnSurf pOnS = extPS.Point(argmin);
            pOnS.Parameter(out u, out v);
            return true;
        }
    }

    /// <summary>An empty projection.</summary>
    /// <remarks>OCCT leaves the approximation parameters (tolerance, continuity, degree, segments, the 2D and 3D
    /// flags) uninitialised here; they are 0, C0 and false in the port.</remarks>
    public CompProjectedCurve()
    {
        _nbCurves = 0;
        _maxDist = 0.0;
        _tolU = 0.0;
        _tolV = 0.0;
    }

    /// <summary>
    /// Projects the curve on the surface (calls <see cref="Init"/>), with the tolerances in U and V.
    /// </summary>
    public CompProjectedCurve(SurfaceAdaptorBase surface,
                                      CurveAdaptorBase curve,
                                      double tolU,
                                      double tolV)
    {
        _surface = surface;
        _curve = curve;
        _nbCurves = 0;
        _sequence = new NCollection_HSequence<NCollection_HSequence<Point3D>>();
        _tol3d = 1.0e-6;
        _continuity = C2;
        _maxDegree = 14;
        _maxSeg = 16;
        _proj2d = true;
        _proj3d = false;
        _maxDist = -1.0;
        _tolU = tolU;
        _tolV = tolV;
        Init();
    }

    /// <summary>
    /// Projects the curve on the surface (calls <see cref="Init"/>); the parts farther than
    /// <paramref name="maxDist"/> from the surface are left out (no limit if negative).
    /// </summary>
    public CompProjectedCurve(SurfaceAdaptorBase surface,
                                      CurveAdaptorBase curve,
                                      double tolU,
                                      double tolV,
                                      double maxDist)
    {
        _surface = surface;
        _curve = curve;
        _nbCurves = 0;
        _sequence = new NCollection_HSequence<NCollection_HSequence<Point3D>>();
        _tol3d = 1.0e-6;
        _continuity = C2;
        _maxDegree = 14;
        _maxSeg = 16;
        _proj2d = true;
        _proj3d = false;
        _maxDist = maxDist;
        _tolU = tolU;
        _tolV = tolV;
        Init();
    }

    /// <summary>
    /// Projects the curve on the surface (calls <see cref="Init"/>), the 2D tolerances being the surface's
    /// resolutions for <paramref name="tol3d"/> (at least Precision::PConfusion).
    /// </summary>
    public CompProjectedCurve(double tol3d,
                                      SurfaceAdaptorBase surface,
                                      CurveAdaptorBase curve,
                                      double maxDist = -1.0)
    {
        _surface = surface;
        _curve = curve;
        _nbCurves = 0;
        _sequence = new NCollection_HSequence<NCollection_HSequence<Point3D>>();
        _tol3d = tol3d;
        _continuity = C2;
        _maxDegree = 14;
        _maxSeg = 16;
        _proj2d = true;
        _proj3d = false;
        _maxDist = maxDist;
        _tolU = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), _surface.UResolution(tol3d));
        _tolV = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), _surface.VResolution(tol3d));

        Init();
    }

    /// <summary>
    /// A copy sharing the projected parts, with shallow copies of the adaptors. As in OCCT, the approximation
    /// parameters and results are not copied.
    /// </summary>
    public override Curve2DAdaptorBase ShallowCopy()
    {
        CompProjectedCurve copy = new();

        if (_surface is not null)
        {
            copy._surface = _surface.ShallowCopy();
        }
        if (_curve is not null)
        {
            copy._curve = _curve.ShallowCopy();
        }
        copy._nbCurves = _nbCurves;
        copy._sequence = _sequence;
        copy._tolU = _tolU;
        copy._tolV = _tolV;
        copy._maxDist = _maxDist;
        copy._uIso = _uIso;
        copy._vIso = _vIso;
        copy._snglPnts = _snglPnts;
        copy._maxDistance = _maxDistance;

        return copy;
    }

    /// <summary>
    /// Computes the projected parts: walks along the curve, projecting its points on the surface. As in OCCT, it
    /// appends to the parts already computed.
    /// </summary>
    public void Init()
    {
        _tabInt = null;
        List<double> splits = new();
        splits.Clear();

        double Tol; // Tolerance for ExactBound
        int i, nend = 0, splitIdx = 0;
        bool fromLastU = false, isSplitsComputed = false;

        double tolExt = global::Kerf.Engine.Foundation.Precision.PConfusion();
        ExtCS cExt = new(_curve, _surface, tolExt, tolExt);
        if (cExt.IsDone && cExt.ExtCount != 0)
        {
            // Search for the minimum solution.
            // Avoid usage of extrema result that can be wrong for extrusion.
            if (_maxDist > 0 &&

                _surface.GetType() != SurfaceOfExtrusion)
            {
                double min_val2;
                min_val2 = cExt.SquareDistance(1);

                nend = cExt.ExtCount;
                for (i = 2; i <= nend; i++)
                {
                    if (cExt.SquareDistance(i) < min_val2)
                    {
                        min_val2 = cExt.SquareDistance(i);
                    }
                }
                if (min_val2 > _maxDist * _maxDist)
                {
                    return; // No near solution -> exit.
                }
            }
        }

        double firstU, lastU, step, searchStep, walkStep, t;

        firstU = _curve.FirstParameter();
        lastU = _curve.LastParameter();
        const double globalMinStep = 1.0e-4;
        //<GlobalMinStep> is sufficiently small to provide solving from initial point
        // and, on the other hand, it is sufficiently large to avoid too close solutions.
        double minStep = 0.01 * (lastU - firstU), maxStep = 0.1 * (lastU - firstU);
        searchStep = 10 * minStep;
        step = searchStep;

        Point2D lowBorder = new(_surface.FirstUParameter(), _surface.FirstVParameter());
        Point2D uppBorder = new(_surface.LastUParameter(), _surface.LastVParameter());
        Point2D tol = new(_tolU, _tolV);
        PrjResolve prjPS = new(_curve, _surface, 1);

        t = firstU;
        bool new_part;
        double prevDeb = 0.0;
        bool sameDeb = false;

        Point3D Triple = new(), prevTriple = new();

        // Basic loop
        while (t <= lastU)
        {
            // Search for the beginning of a new continuous part
            // to avoid infinite computation in some difficult cases.
            new_part = false;
            if (t > firstU && Math.Abs(t - prevDeb) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                sameDeb = true;
            }
            while (t <= lastU && !new_part && !fromLastU && !sameDeb)
            {
                prevDeb = t;
                if (t == lastU)
                {
                    fromLastU = true;
                }
                bool initpoint = false;
                double u = 0.0, v = 0.0;
                Point3D cPoint = new();
                double parT, parU, parV;

                // Search an initial point in the list of Extrema Curve-Surface
                if (nend != 0 && !cExt.IsParallel)
                {
                    for (i = 1; i <= nend; i++)
                    {
                        POnCurv p1;
                        POnSurf p2;
                        cExt.Points(i, out p1, out p2);
                        parT = p1.Parameter();
                        p2.Parameter(out parU, out parV);

                        prjPS.Perform(parT, parU, parV, tol, lowBorder, uppBorder, FuncTol, true);

                        if (prjPS.IsDone
                            && p1.Parameter() > Max(firstU, t - step + global::Kerf.Engine.Foundation.Precision.PConfusion())
                            && p1.Parameter() <= t)
                        {
                            t = parT;
                            u = parU;
                            v = parV;
                            cPoint = p1.Value();
                            initpoint = true;
                            break;
                        }
                    }
                }
                if (!initpoint)
                {
                    _curve.D0(t, out cPoint);
                    // PConfusion - use geometric tolerances in extrema / optimization.
                    initpoint = InitialPoint(cPoint, t, _curve, _surface, _tolU, _tolV, out u, out v, _maxDist);
                }
                if (initpoint)
                {
                    // When U or V lie on surface joint in some cases we cannot use them
                    // as initial point for aPrjPS, so we switch them
                    Vector2D D;

                    if ((_surface.IsUPeriodic
                         && Math.Abs(uppBorder.X - lowBorder.X - _surface.UPeriod())
                              < global::Kerf.Engine.Foundation.Precision.Confusion())
                        || (_surface.IsVPeriodic
                            && Math.Abs(uppBorder.Y - lowBorder.Y - _surface.VPeriod())
                                 < global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        if ((Math.Abs(u - lowBorder.X) < _surface.UResolution(global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            && _surface.IsUPeriodic)
                        {
                            D1Core(t, u, v, out D, _curve, _surface);
                            if (D.X < 0)
                            {
                                u = uppBorder.X;
                            }
                        }
                        else if ((Math.Abs(u - uppBorder.X) < _surface.UResolution(global::Kerf.Engine.Foundation.Precision.PConfusion()))
                                 && _surface.IsUPeriodic)
                        {
                            D1Core(t, u, v, out D, _curve, _surface);
                            if (D.X > 0)
                            {
                                u = lowBorder.X;
                            }
                        }

                        if ((Math.Abs(v - lowBorder.Y) < _surface.VResolution(global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            && _surface.IsVPeriodic)
                        {
                            D1Core(t, u, v, out D, _curve, _surface);
                            if (D.Y < 0)
                            {
                                v = uppBorder.Y;
                            }
                        }
                        else if ((Math.Abs(v - uppBorder.Y) <= _surface.VResolution(global::Kerf.Engine.Foundation.Precision.PConfusion()))
                                 && _surface.IsVPeriodic)
                        {
                            D1Core(t, u, v, out D, _curve, _surface);
                            if (D.Y > 0)
                            {
                                v = lowBorder.Y;
                            }
                        }
                    }

                    if (_maxDist > 0)
                    {
                        // Here we are going to stop if the distance between projection and
                        // corresponding curve point is greater than myMaxDist
                        Point3D pOnS;
                        double d;
                        _surface.D0(u, v, out pOnS);
                        d = cPoint.DistanceTo(pOnS);
                        if (d > _maxDist)
                        {
                            _sequence.Clear();
                            _nbCurves = 0;
                            return;
                        }
                    }
                    Triple = new Point3D(t, u, v);
                    if (t != firstU)
                    {
                        // Search for exact boundary point
                        Tol = Min(_tolU, _tolV);
                        Vector2D aD;
                        D1Core(Triple.X, Triple.Y, Triple.Z, out aD, _curve, _surface);
                        Tol /= Max(Math.Abs(aD.X), Math.Abs(aD.Y));

                        if (!ExactBound(ref Triple, t - step, Tol, _tolU, _tolV, _curve, _surface))
                        {
                            DichExactBound(ref Triple, t - step, Tol, _tolU, _tolV, _curve, _surface);
                        }
                    }
                    new_part = true;
                }
                else
                {
                    if (t == lastU)
                    {
                        break;
                    }
                    t += step;
                    if (t > lastU)
                    {
                        step = step + lastU - t;
                        t = lastU;
                    }
                }
            }
            if (!new_part)
            {
                break;
            }

            // We have found a new continuous part
            NCollection_HSequence<Point3D> hSeq = new();
            _sequence.Append(hSeq);
            _nbCurves++;
            _sequence.Value(_nbCurves).Append(Triple);
            prevTriple = Triple;

            if (Triple.X == lastU)
            {
                break; // return;
            }

            // Computation of WalkStep
            Vector3D d1, d2;
            double magnD1, magnD2;
            D2CurvOnSurf(Triple.X, Triple.Y, Triple.Z, out d1, out d2, _curve, _surface);
            magnD1 = d1.Magnitude;
            magnD2 = d2.Magnitude;
            if (magnD2 < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                walkStep = maxStep;
            }
            else
            {
                walkStep = Min(maxStep, Max(minStep, 0.1 * magnD1 / magnD2));
            }

            step = walkStep;

            t = Triple.X + step;
            if (t > lastU)
            {
                t = lastU;
            }
            double prevStep = step;
            double u0, v0;

            // Here we are trying to prolong continuous part
            while (t <= lastU && new_part)
            {
                u0 = Triple.Y + (step / prevStep) * (Triple.Y - prevTriple.Y);
                v0 = Triple.Z + (step / prevStep) * (Triple.Z - prevTriple.Z);
                // adjust U0 to be in [mySurface->FirstUParameter(),mySurface->LastUParameter()]
                u0 = Min(Max(u0, lowBorder.X), uppBorder.X);
                // adjust V0 to be in [mySurface->FirstVParameter(),mySurface->LastVParameter()]
                v0 = Min(Max(v0, lowBorder.Y), uppBorder.Y);

                prjPS.Perform(t, u0, v0, tol, lowBorder, uppBorder, FuncTol, true);
                if (!prjPS.IsDone)
                {
                    if (step <= globalMinStep)
                    {
                        // Search for exact boundary point
                        Tol = Min(_tolU, _tolV);
                        Vector2D D;
                        D1Core(Triple.X, Triple.Y, Triple.Z, out D, _curve, _surface);
                        Tol /= Max(Math.Abs(D.X), Math.Abs(D.Y));

                        if (!ExactBound(ref Triple, t, Tol, _tolU, _tolV, _curve, _surface))
                        {
                            DichExactBound(ref Triple, t, Tol, _tolU, _tolV, _curve, _surface);
                        }

                        if ((Triple.X
                             - _sequence.Value(_nbCurves).Value(_sequence.Value(_nbCurves).Length()).X)
                            > 1.0e-10)
                        {
                            _sequence.Value(_nbCurves).Append(Triple);
                        }
                        if ((lastU - Triple.X) < Tol)
                        {
                            t = lastU + 1;
                            break;
                        } // return;

                        step = searchStep;
                        t = Triple.X + step;
                        if (t > (lastU - minStep / 2))
                        {
                            step = step + lastU - t;
                            t = lastU;
                        }
                        new_part = false;
                    }
                    else
                    {
                        // decrease step
                        double saveStep = step;
                        step /= 2.0;
                        t = Triple.X + step;
                        if (t > (lastU - minStep / 4))
                        {
                            step = step + lastU - t;
                            if (Math.Abs(step - saveStep) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                step = globalMinStep; // to avoid looping
                            }
                            t = lastU;
                        }
                    }
                }
                // Go further
                else
                {
                    prevTriple = Triple;
                    prevStep = step;
                    Triple = new Point3D(t, prjPS.Solution().X, prjPS.Solution().Y);

                    // Check for possible local traps.
                    UpdateTripleByTrapCriteria(ref Triple);

                    // Protection from case when the whole curve lies on a seam.
                    if (!isSplitsComputed)
                    {
                        bool isUPossible = false;
                        if (_surface.IsUPeriodic
                            && (Math.Abs(Triple.Y - _surface.FirstUParameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                && Math.Abs(Triple.Y - _surface.LastUParameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion()))
                        {
                            isUPossible = true;
                        }

                        bool isVPossible = false;
                        if (_surface.IsVPeriodic
                            && (Math.Abs(Triple.Z - _surface.FirstVParameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                && Math.Abs(Triple.Z - _surface.LastVParameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion()))
                        {
                            isVPossible = true;
                        }

                        if (isUPossible || isVPossible)
                        {
                            // When point is good conditioned.
                            BuildCurveSplits(_curve, _surface, _tolU, _tolV, splits);
                            isSplitsComputed = true;
                        }
                    }

                    if ((Triple.X
                         - _sequence.Value(_nbCurves).Value(_sequence.Value(_nbCurves).Length()).X)
                        > 1.0e-10)
                    {
                        _sequence.Value(_nbCurves).Append(Triple);
                    }
                    if (t == lastU)
                    {
                        t = lastU + 1;
                        break;
                    } // return;
                    // Computation of WalkStep
                    D2CurvOnSurf(Triple.X, Triple.Y, Triple.Z, out d1, out d2, _curve, _surface);
                    magnD1 = d1.Magnitude;
                    magnD2 = d2.Magnitude;
                    if (magnD2 < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        walkStep = maxStep;
                    }
                    else
                    {
                        walkStep = Min(maxStep, Max(minStep, 0.1 * magnD1 / magnD2));
                    }

                    step = walkStep;
                    t += step;
                    if (t > (lastU - minStep / 2))
                    {
                        step = step + lastU - t;
                        t = lastU;
                    }

                    // We assume at least one point of cache inside of a split.
                    int size = splits.Count;
                    for (int idx = splitIdx; idx < size; ++idx)
                    {
                        double param = splits[idx];
                        if (Math.Abs(param - Triple.X) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            // The current point is equal to a split point.
                            new_part = false;

                            // Move split index to avoid check of the whole list.
                            ++splitIdx;
                            break;
                        }
                        else if (param < t + global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            // The next point crosses the split point.
                            t = param;
                            step = t - prevTriple.X;
                        }
                    } // for(int anIdx = aSplitIdx; anIdx < aSize; ++anIdx)
                }
            }
        }

        // Sequence post-proceeding.
        int j;

        // 1. Removing poor parts
        int nbPart = _nbCurves;
        int ipart = 1;
        for (i = 1; i <= nbPart; i++)
        {
            //    int NbPoints = mySequence->Value(i)->Length();
            if (_sequence.Value(ipart).Length() < 2)
            {
                _sequence.Remove(ipart);
                _nbCurves--;
            }
            else
            {
                ipart++;
            }
        }

        if (_nbCurves == 0)
        {
            return;
        }

        // 2. Removing common parts of bounds
        for (i = 1; i < _nbCurves; i++)
        {
            if (_sequence.Value(i).Value(_sequence.Value(i).Length()).X
                >= _sequence.Value(i + 1).Value(1).X)
            {
                _sequence.ChangeValue(i + 1).ChangeValue(1) = _sequence.ChangeValue(i + 1).ChangeValue(1).WithX(
                    _sequence.Value(i).Value(_sequence.Value(i).Length()).X + 1.0e-12);
            }
        }

        // 3. Computation of the maximum distance from each part of curve to surface

        _maxDistance = new NCollection_HArray1<double>(1, _nbCurves);
        _maxDistance.Init(0);
        for (i = 1; i <= _nbCurves; i++)
        {
            for (j = 1; j <= _sequence.Value(i).Length(); j++)
            {
                Point3D pOnC, pOnS, triple;
                double distance;
                triple = _sequence.Value(i).Value(j);
                _curve.D0(triple.X, out pOnC);
                _surface.D0(triple.Y, triple.Z, out pOnS);
                distance = pOnC.DistanceTo(pOnS);
                if (_maxDistance.Value(i) < distance)
                {
                    _maxDistance.ChangeValue(i) = distance;
                }
            }
        }

        // 4. Check the projection to be a single point

        Point2D pmoy = new(), pcurr, p;
        double aveU, aveV;
        _snglPnts = new NCollection_HArray1<bool>(1, _nbCurves);
        _snglPnts.Init(true);

        for (i = 1; i <= _nbCurves; i++)
        {
            // compute an average U and V

            for (j = 1, aveU = 0.0, aveV = 0.0; j <= _sequence.Value(i).Length(); j++)
            {
                aveU += _sequence.Value(i).Value(j).Y;
                aveV += _sequence.Value(i).Value(j).Z;
            }
            aveU /= _sequence.Value(i).Length();
            aveV /= _sequence.Value(i).Length();

            pmoy = pmoy.WithCoord(aveU, aveV);
            for (j = 1; j <= _sequence.Value(i).Length(); j++)
            {
                pcurr = new Point2D(_sequence.Value(i).Value(j).Y, _sequence.Value(i).Value(j).Z);
                if (pcurr.DistanceTo(pmoy) > ((_tolU < _tolV) ? _tolV : _tolU))
                {
                    _snglPnts.SetValue(i, false);
                    break;
                }
            }
        }

        // 5. Check the projection to be an isoparametric curve of the surface

        _uIso = new NCollection_HArray1<bool>(1, _nbCurves);
        _uIso.Init(true);

        _vIso = new NCollection_HArray1<bool>(1, _nbCurves);
        _vIso.Init(true);

        for (i = 1; i <= _nbCurves; i++)
        {
            if (IsSinglePnt(i, out p) || _sequence.Value(i).Length() <= 2)
            {
                _uIso.SetValue(i, false);
                _vIso.SetValue(i, false);
                continue;
            }

            // new test for isoparametrics

            if (_sequence.Value(i).Length() > 2)
            {
                // compute an average U and V

                for (j = 1, aveU = 0.0, aveV = 0.0; j <= _sequence.Value(i).Length(); j++)
                {
                    aveU += _sequence.Value(i).Value(j).Y;
                    aveV += _sequence.Value(i).Value(j).Z;
                }
                aveU /= _sequence.Value(i).Length();
                aveV /= _sequence.Value(i).Length();

                // is i-part U-isoparametric ?
                for (j = 1; j <= _sequence.Value(i).Length(); j++)
                {
                    if (Math.Abs(_sequence.Value(i).Value(j).Y - aveU) > _tolU)
                    {
                        _uIso.SetValue(i, false);
                        break;
                    }
                }

                // is i-part V-isoparametric ?
                for (j = 1; j <= _sequence.Value(i).Length(); j++)
                {
                    if (Math.Abs(_sequence.Value(i).Value(j).Z - aveV) > _tolV)
                    {
                        _vIso.SetValue(i, false);
                        break;
                    }
                }
                //
            }
        }
    }

    /// <summary>
    /// Approximates the projected parts: a point for a punctual part, a line for an isoparametric one, a B-spline
    /// curve (Approx_CurveOnSurface) otherwise; 2D curves if <see cref="SetProj2d"/> (the default), 3D curves if
    /// <see cref="SetProj3d"/>.
    /// </summary>
    public void Perform()
    {
        if (_nbCurves == 0)
        {
            return;
        }

        bool approx2d = _proj2d;
        bool approx3d = _proj3d;
        double udeb, ufin, uIso, vIso;
        Point2D p2d, pdeb, pfin;
        Point3D p;
        Curve2DAdaptorBase? hpCur;
        SurfaceAdaptorBase hs = _surface.ShallowCopy(); // For expand bounds of surface
        Curve2D? pCur2d = null;                    // Only for isoparametric projection
        Curve3D? pCur3d = null;

        if (_proj2d)
        {
            _result2dPoint = new NCollection_HArray1<Point2D>(1, _nbCurves);
            _result2dCurve = new NCollection_HArray1<Curve2D?>(1, _nbCurves);
        }

        if (_proj3d)
        {
            _result3dPoint = new NCollection_HArray1<Point3D>(1, _nbCurves);
            _result3dCurve = new NCollection_HArray1<Curve3D?>(1, _nbCurves);
        }

        _resultIsPoint = new NCollection_HArray1<bool>(1, _nbCurves);
        _resultIsPoint.Init(false);

        _result3dApproxError = new NCollection_HArray1<double>(1, _nbCurves);
        _result3dApproxError.Init(0.0);

        _result2dUApproxError = new NCollection_HArray1<double>(1, _nbCurves);
        _result2dUApproxError.Init(0.0);

        _result2dVApproxError = new NCollection_HArray1<double>(1, _nbCurves);
        _result2dVApproxError.Init(0.0);

        for (int k = 1; k <= _nbCurves; k++)
        {
            if (IsSinglePnt(k, out p2d)) // Part k of the projection is punctual
            {
                GetSurface()!.D0(p2d.X, p2d.Y, out p);
                if (_proj2d)
                {
                    _result2dPoint!.SetValue(k, p2d);
                }
                if (_proj3d)
                {
                    _result3dPoint!.SetValue(k, p);
                }
                _resultIsPoint.SetValue(k, true);
            }
            else
            {
                Bounds(k, out udeb, out ufin);
                Direction2D dir; // Only for isoparametric projection

                // An isoparametric part needs no 2D approximation; each part starts from the requested ones (OCCT
                // cleared approx2d for good, so a later free part got the iso line: decision 1460).
                approx2d = _proj2d;
                if (IsUIso(k, out uIso)) // Part k of the projection is U-isoparametric curve
                {
                    approx2d = false;

                    D0(udeb, out pdeb);
                    D0(ufin, out pfin);
                    udeb = pdeb.Y;
                    ufin = pfin.Y;
                    if (udeb > ufin)
                    {
                        dir = new Direction2D(Direction2D.D.NY);
                        udeb = -udeb;
                        ufin = -ufin;
                    }
                    else
                    {
                        dir = new Direction2D(Direction2D.D.Y);
                    }
                    pCur2d = new TrimmedCurve2D(new LineCurve2D(new Point2D(uIso, 0), dir), udeb, ufin);
                    hpCur = new CurveAdaptor2D(pCur2d);
                }
                else if (IsVIso(k, out vIso)) // Part k of the projection is V-isoparametric curve
                {
                    approx2d = false;

                    D0(udeb, out pdeb);
                    D0(ufin, out pfin);
                    udeb = pdeb.X;
                    ufin = pfin.X;
                    if (udeb > ufin)
                    {
                        dir = new Direction2D(Direction2D.D.NX);
                        udeb = -udeb;
                        ufin = -ufin;
                    }
                    else
                    {
                        dir = new Direction2D(Direction2D.D.X);
                    }
                    pCur2d = new TrimmedCurve2D(new LineCurve2D(new Point2D(0, vIso), dir), udeb, ufin);
                    hpCur = new CurveAdaptor2D(pCur2d);
                }
                else
                {
                    if (!_surface.IsUPeriodic)
                    {
                        double u1, u2;
                        double dU = 10.0 * _tolU;

                        u1 = _surface.FirstUParameter();
                        u2 = _surface.LastUParameter();
                        u1 -= dU;
                        u2 += dU;

                        hs = hs.UTrim(u1, u2, 0.0);
                    }

                    if (!_surface.IsVPeriodic)
                    {
                        double v1, v2;
                        double dV = 10.0 * _tolV;

                        v1 = _surface.FirstVParameter();
                        v2 = _surface.LastVParameter();
                        v1 -= dV;
                        v2 += dV;

                        hs = hs.VTrim(v1, v2, 0.0);
                    }

                    CompProjectedCurve hp = (CompProjectedCurve)ShallowCopy();
                    hp.Load(hs);
                    hpCur = hp;
                }

                if (approx2d || approx3d)
                {
                    bool only2d, only3d;
                    if (approx2d && approx3d)
                    {
                        only2d = !approx2d;
                        only3d = !approx3d;
                    }
                    else
                    {
                        only2d = approx2d;
                        only3d = approx3d;
                    }

                    CurveOnSurfaceApproximation appr = new(hpCur, hs, udeb, ufin, _tol3d);
                    appr.Perform(_maxSeg, _maxDegree, _continuity, only3d, only2d);

                    // The approximation is discarded: its curves are read unfrozen (decision 287).
                    if (approx2d)
                    {
                        pCur2d = appr.InternalCurve2d();
                        _result2dUApproxError.SetValue(k, appr.MaxError2dU());
                        _result2dVApproxError.SetValue(k, appr.MaxError2dV());
                    }

                    if (approx3d)
                    {
                        pCur3d = appr.InternalCurve3d();
                        _result3dApproxError.SetValue(k, appr.MaxError3d());
                    }
                }

                if (_proj2d)
                {
                    _result2dCurve!.SetValue(k, pCur2d);
                }

                if (_proj3d)
                {
                    _result3dCurve!.SetValue(k, pCur3d);
                }
            }
        }
    }

    /// <summary>Sets the 3D tolerance of the approximation (default 1e-6).</summary>
    public void SetTol3d(double tol3d)
    {
        _tol3d = tol3d;
    }

    /// <summary>Sets the continuity of the approximation (default C2).</summary>
    public void SetContinuity(Continuity continuity)
    {
        _continuity = continuity;
    }

    /// <summary>Sets the maximum degree of the approximation (default 14); ignored if less than 1.</summary>
    public void SetMaxDegree(int maxDegree)
    {
        if (maxDegree < 1)
        {
            return;
        }
        _maxDegree = maxDegree;
    }

    /// <summary>Sets the maximum number of segments of the approximation (default 16); ignored if less than 1.</summary>
    public void SetMaxSeg(int maxSeg)
    {
        if (maxSeg < 1)
        {
            return;
        }
        _maxSeg = maxSeg;
    }

    /// <summary>Whether <see cref="Perform"/> builds 3D curves (default false).</summary>
    public void SetProj3d(bool proj3d)
    {
        _proj3d = proj3d;
    }

    /// <summary>Whether <see cref="Perform"/> builds 2D curves (default true).</summary>
    public void SetProj2d(bool proj2d)
    {
        _proj2d = proj2d;
    }

    /// <summary>Changes the surface (kept by reference).</summary>
    public void Load(SurfaceAdaptorBase s)
    {
        _surface = s;
    }

    /// <summary>Changes the curve (kept by reference).</summary>
    public void Load(CurveAdaptorBase c)
    {
        _curve = c;
    }

    /// <summary>The surface (the caller's adaptor); null for an empty projection.</summary>
    public SurfaceAdaptorBase? GetSurface() => _surface;

    /// <summary>The curve (the caller's adaptor); null for an empty projection.</summary>
    public CurveAdaptorBase? GetCurve() => _curve;

    public void GetTolerance(out double tolU, out double tolV)
    {
        tolU = _tolU;
        tolV = _tolV;
    }

    /// <summary>The number of continuous parts of the projection.</summary>
    public int CurveCount => _nbCurves;

    /// <summary>The curve parameters bounding part <paramref name="index"/>.</summary>
    /// <exception cref="NoSuchObjectException">Index is out of range.</exception>
    public void Bounds(int index, out double udeb, out double ufin)
    {
        if (index < 1 || index > _nbCurves)
        {
            throw new NoSuchObjectException();
        }
        udeb = _sequence.Value(index).Value(1).X;
        ufin = _sequence.Value(index).Value(_sequence.Value(index).Length()).X;
    }

    /// <summary>True if part <paramref name="index"/> projects to a single point, <paramref name="p"/>.</summary>
    /// <exception cref="NoSuchObjectException">Index is out of range.</exception>
    public bool IsSinglePnt(int index, out Point2D p)
    {
        if (index < 1 || index > _nbCurves)
        {
            throw new NoSuchObjectException();
        }
        p = new Point2D(_sequence.Value(index).Value(1).Y, _sequence.Value(index).Value(1).Z);
        return _snglPnts!.Value(index);
    }

    /// <summary>True if part <paramref name="index"/> is a U-isoparametric line, at <paramref name="u"/>.</summary>
    /// <exception cref="NoSuchObjectException">Index is out of range.</exception>
    public bool IsUIso(int index, out double u)
    {
        if (index < 1 || index > _nbCurves)
        {
            throw new NoSuchObjectException();
        }
        u = _sequence.Value(index).Value(1).Y;
        return _uIso!.Value(index);
    }

    /// <summary>True if part <paramref name="index"/> is a V-isoparametric line, at <paramref name="v"/>.</summary>
    /// <exception cref="NoSuchObjectException">Index is out of range.</exception>
    public bool IsVIso(int index, out double v)
    {
        if (index < 1 || index > _nbCurves)
        {
            throw new NoSuchObjectException();
        }
        v = _sequence.Value(index).Value(1).Z;
        return _vIso!.Value(index);
    }

    public override Point2D Value(double t)
    {
        Point2D p;
        D0(t, out p);
        return p;
    }

    /// <exception cref="DomainErrorException">U is in none of the parts.</exception>
    public override void D0(double u, out Point2D p)
    {
        int i, j;
        double udeb, ufin;
        bool found = false;

        for (i = 1; i <= _nbCurves; i++)
        {
            Bounds(i, out udeb, out ufin);
            if (u >= udeb && u <= ufin)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            throw new DomainErrorException("ProjLib_CompProjectedCurve::D0");
        }

        double u0, v0;

        int end = _sequence.Value(i).Length();
        for (j = 1; j < end; j++)
        {
            if ((u >= _sequence.Value(i).Value(j).X) && (u <= _sequence.Value(i).Value(j + 1).X))
            {
                break;
            }
        }

        //  U0 = mySequence->Value(i)->Value(j).Y();
        //  V0 = mySequence->Value(i)->Value(j).Z();

        //  Cubic Interpolation
        if (_sequence.Value(i).Length() < 4
            || (Math.Abs(u - _sequence.Value(i).Value(j).X) <= global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            u0 = _sequence.Value(i).Value(j).Y;
            v0 = _sequence.Value(i).Value(j).Z;
        }
        else if (Math.Abs(u - _sequence.Value(i).Value(j + 1).X) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            u0 = _sequence.Value(i).Value(j + 1).Y;
            v0 = _sequence.Value(i).Value(j + 1).Z;
        }
        else
        {
            if (j == 1)
            {
                j = 2;
            }
            if (j > _sequence.Value(i).Length() - 2)
            {
                j = _sequence.Value(i).Length() - 2;
            }

            Vector2D i1, i2, i3, i21, i22, i31, y1, y2, y3, y4, res;
            double x1, x2, x3, x4;

            x1 = _sequence.Value(i).Value(j - 1).X;
            x2 = _sequence.Value(i).Value(j).X;
            x3 = _sequence.Value(i).Value(j + 1).X;
            x4 = _sequence.Value(i).Value(j + 2).X;

            y1 = new Vector2D(_sequence.Value(i).Value(j - 1).Y, _sequence.Value(i).Value(j - 1).Z);
            y2 = new Vector2D(_sequence.Value(i).Value(j).Y, _sequence.Value(i).Value(j).Z);
            y3 = new Vector2D(_sequence.Value(i).Value(j + 1).Y, _sequence.Value(i).Value(j + 1).Z);
            y4 = new Vector2D(_sequence.Value(i).Value(j + 2).Y, _sequence.Value(i).Value(j + 2).Z);

            i1 = (y1 - y2) / (x1 - x2);
            i2 = (y2 - y3) / (x2 - x3);
            i3 = (y3 - y4) / (x3 - x4);

            i21 = (i1 - i2) / (x1 - x3);
            i22 = (i2 - i3) / (x2 - x4);

            i31 = (i21 - i22) / (x1 - x4);

            res = y1 + (u - x1) * (i1 + (u - x2) * (i21 + (u - x3) * i31));

            u0 = res.X;
            v0 = res.Y;

            if (u0 < _surface.FirstUParameter())
            {
                u0 = _surface.FirstUParameter();
            }
            else if (u0 > _surface.LastUParameter())
            {
                u0 = _surface.LastUParameter();
            }

            if (v0 < _surface.FirstVParameter())
            {
                v0 = _surface.FirstVParameter();
            }
            else if (v0 > _surface.LastVParameter())
            {
                v0 = _surface.LastVParameter();
            }
        }
        // End of cubic interpolation

        PrjResolve prjPS = new(_curve, _surface, 1);
        prjPS.Perform(u,
                       u0,
                       v0,
                       new Point2D(_tolU, _tolV),
                       new Point2D(_surface.FirstUParameter(), _surface.FirstVParameter()),
                       new Point2D(_surface.LastUParameter(), _surface.LastVParameter()),
                       FuncTol);
        if (prjPS.IsDone)
        {
            p = prjPS.Solution();
        }
        else
        {
            Point3D thePoint = _curve.Value(u);
            ExtPS extPS = new(thePoint, _surface, _tolU, _tolV, ExtFlag_MIN);
            if (extPS.IsDone && extPS.ExtCount != 0)
            {
                int k, nend, imin = 1;
                // Search for the nearest solution which is also a normal projection
                nend = extPS.ExtCount;
                for (k = 2; k <= nend; k++)
                {
                    if (extPS.SquareDistance(k) < extPS.SquareDistance(imin))
                    {
                        imin = k;
                    }
                }
                POnSurf pOnS = extPS.Point(imin);
                double parU, parV;
                pOnS.Parameter(out parU, out parV);
                p = new Point2D(parU, parV);
            }
            else
            {
                p = new Point2D(u0, v0);
            }
        }
    }

    public override void D1(double t, out Point2D p, out Vector2D V)
    {
        double u, v;
        D0(t, out p);
        u = p.X;
        v = p.Y;
        D1Core(t, u, v, out V, _curve, _surface);
    }

    public override void D2(double t, out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        double u, v;
        D0(t, out p);
        u = p.X;
        v = p.Y;
        D2Core(t, u, v, out v1, out v2, _curve, _surface);
    }

    /// <exception cref="OutOfRangeException">N is less than 1.</exception>
    /// <exception cref="MethodNotImplementedException">N is greater than 2.</exception>
    public override Vector2D DN(double t, int n)
    {
        if (n < 1)
        {
            throw new OutOfRangeException("ProjLib_CompProjectedCurve : N must be greater than 0");
        }
        else if (n == 1)
        {
            Point2D p;
            Vector2D v;
            D1(t, out p, out v);
            return v;
        }
        else if (n == 2)
        {
            Point2D p;
            Vector2D v1, v2;
            D2(t, out p, out v1, out v2);
            return v2;
        }
        else if (n > 2)
        {
            throw new MethodNotImplementedException("ProjLib_CompProjectedCurve::DN");
        }
        return new Vector2D();
    }

    /// <summary>
    /// The (t, u, v) triples of each part. A deep copy in the port, where OCCT returns its own sequences: the
    /// projection keeps (and its shallow copies share) them.
    /// </summary>
    public NCollection_HSequence<NCollection_HSequence<Point3D>> GetSequence()
    {
        NCollection_HSequence<NCollection_HSequence<Point3D>> copy = new();
        if (_sequence is not null)
        {
            for (int i = 1; i <= _sequence.Length(); i++)
            {
                copy.Append(new NCollection_HSequence<Point3D>(_sequence.Value(i)));
            }
        }
        return copy;
    }

    public override double FirstParameter() => _curve.FirstParameter();

    public override double LastParameter() => _curve.LastParameter();

    public override Continuity Continuity()
    {
        Continuity contC = _curve.Continuity();
        Continuity contSu = _surface.UContinuity();
        if (contSu < contC)
        {
            contC = contSu;
        }
        Continuity contSv = _surface.VContinuity();
        if (contSv < contC)
        {
            contC = contSv;
        }

        return contC;
    }

    /// <summary>The maximum distance between the curve and its projection in part <paramref name="index"/>.</summary>
    /// <exception cref="NoSuchObjectException">Index is out of range.</exception>
    public double MaxDistance(int index)
    {
        if (index < 1 || index > _nbCurves)
        {
            throw new NoSuchObjectException();
        }
        return _maxDistance!.Value(index);
    }

    /// <summary>
    /// The number of intervals of continuity <paramref name="s"/>: the parts' bounds, the curve's intervals and
    /// the points where the projection crosses the surface's intervals. Recomputes and caches them.
    /// </summary>
    public override int NbIntervals(Continuity s)
    {
        _tabInt = null;
        BuildIntervals(s);
        return _tabInt!.Length() - 1;
    }

    /// <summary>
    /// The intervals cached by <see cref="NbIntervals"/>, computed for <paramref name="s"/> if none are cached (as
    /// OCCT, a cache computed for another continuity is returned).
    /// </summary>
    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        if (_tabInt is null)
        {
            BuildIntervals(s);
        }
        t.Assign(_tabInt!.Array1());
    }

    private void BuildIntervals(Continuity s)
    {
        Continuity sforS = CN;
        switch (s)
        {
            case C0:
                sforS = C1;
                break;
            case C1:
                sforS = C2;
                break;
            case C2:
                sforS = C3;
                break;
            case C3:
                sforS = CN;
                break;
            case CN:
                sforS = CN;
                break;
            default:
                throw new OutOfRangeException();
        }
        int i, j, k;
        int nbIntCur = _curve.NbIntervals(s);
        int nbIntSurU = _surface.NbUIntervals(sforS);
        int nbIntSurV = _surface.NbVIntervals(sforS);

        NCollection_Array1<double> cutPntsT = new(1, nbIntCur + 1);
        NCollection_Array1<double> cutPntsU = new(1, nbIntSurU + 1);
        NCollection_Array1<double> cutPntsV = new(1, nbIntSurV + 1);

        _curve.Intervals(cutPntsT, s);
        _surface.UIntervals(cutPntsU, sforS);
        _surface.VIntervals(cutPntsV, sforS);

        double tl, tr, ul, ur, vl, vr, tol;

        NCollection_HArray1<double>? bArr = null, cArr = null, uArr = null, vArr = null;

        // processing projection bounds
        bArr = new NCollection_HArray1<double>(1, 2 * _nbCurves);
        for (i = 1; i <= _nbCurves; i++)
        {
            Bounds(i, out bArr[2 * i - 1], out bArr[2 * i]);
        }

        // processing curve discontinuities
        if (nbIntCur > 1)
        {
            cArr = new NCollection_HArray1<double>(1, nbIntCur - 1);
            for (i = 1; i <= cArr.Length(); i++)
            {
                cArr.ChangeValue(i) = cutPntsT[i + 1];
            }
        }

        // processing U-surface discontinuities
        NCollection_Sequence<double> tUdisc = new();

        for (k = 2; k <= nbIntSurU; k++)
        {
            //    std::cout<<"CutPntsU("<<k<<") = "<<CutPntsU(k)<<std::endl;
            for (i = 1; i <= _nbCurves; i++)
            {
                for (j = 1; j < _sequence.Value(i).Length(); j++)
                {
                    ul = _sequence.Value(i).Value(j).Y;
                    ur = _sequence.Value(i).Value(j + 1).Y;

                    if (Math.Abs(ul - cutPntsU[k]) <= _tolU)
                    {
                        tUdisc.Append(_sequence.Value(i).Value(j).X);
                    }
                    else if (Math.Abs(ur - cutPntsU[k]) <= _tolU)
                    {
                        tUdisc.Append(_sequence.Value(i).Value(j + 1).X);
                    }
                    else if ((ul < cutPntsU[k] && cutPntsU[k] < ur) || (ur < cutPntsU[k] && cutPntsU[k] < ul))
                    {
                        double v;
                        v = (_sequence.Value(i).Value(j).Z + _sequence.Value(i).Value(j + 1).Z) / 2;
                        PrjResolve solver = new(_curve, _surface, 2);

                        Vector2D d;
                        Point3D triple;
                        triple = _sequence.Value(i).Value(j);
                        D1Core(triple.X, triple.Y, triple.Z, out d, _curve, _surface);
                        if (Math.Abs(d.X) < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            tol = _tolU;
                        }
                        else
                        {
                            tol = Min(_tolU, _tolU / Math.Abs(d.X));
                        }

                        tl = _sequence.Value(i).Value(j).X;
                        tr = _sequence.Value(i).Value(j + 1).X;

                        solver.Perform((tl + tr) / 2,
                                       cutPntsU[k],
                                       v,
                                       new Point2D(tol, _tolV),
                                       new Point2D(tl, _surface.FirstVParameter()),
                                       new Point2D(tr, _surface.LastVParameter()),
                                       FuncTol);
                        //
                        if (solver.IsDone)
                        {
                            tUdisc.Append(solver.Solution().X);
                        }
                    }
                }
            }
        }
        for (i = 2; i <= tUdisc.Length(); i++)
        {
            if (tUdisc.Value(i) - tUdisc.Value(i - 1) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                tUdisc.Remove(i--);
            }
        }

        if (tUdisc.Length() != 0)
        {
            uArr = new NCollection_HArray1<double>(1, tUdisc.Length());
            for (i = 1; i <= uArr.Length(); i++)
            {
                uArr.ChangeValue(i) = tUdisc.Value(i);
            }
        }
        // processing V-surface discontinuities
        NCollection_Sequence<double> tVdisc = new();

        for (k = 2; k <= nbIntSurV; k++)
        {
            for (i = 1; i <= _nbCurves; i++)
            {
                //      std::cout<<"CutPntsV("<<k<<") = "<<CutPntsV(k)<<std::endl;
                for (j = 1; j < _sequence.Value(i).Length(); j++)
                {
                    vl = _sequence.Value(i).Value(j).Z;
                    vr = _sequence.Value(i).Value(j + 1).Z;

                    if (Math.Abs(vl - cutPntsV[k]) <= _tolV)
                    {
                        tVdisc.Append(_sequence.Value(i).Value(j).X);
                    }
                    else if (Math.Abs(vr - cutPntsV[k]) <= _tolV)
                    {
                        tVdisc.Append(_sequence.Value(i).Value(j + 1).X);
                    }
                    else if ((vl < cutPntsV[k] && cutPntsV[k] < vr) || (vr < cutPntsV[k] && cutPntsV[k] < vl))
                    {
                        double u;
                        u = (_sequence.Value(i).Value(j).Y + _sequence.Value(i).Value(j + 1).Y) / 2;
                        PrjResolve solver = new(_curve, _surface, 3);

                        Vector2D d;
                        Point3D triple;
                        triple = _sequence.Value(i).Value(j);
                        D1Core(triple.X, triple.Y, triple.Z, out d, _curve, _surface);
                        if (Math.Abs(d.Y) < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            tol = _tolV;
                        }
                        else
                        {
                            tol = Min(_tolV, _tolV / Math.Abs(d.Y));
                        }

                        tl = _sequence.Value(i).Value(j).X;
                        tr = _sequence.Value(i).Value(j + 1).X;

                        solver.Perform((tl + tr) / 2,
                                       u,
                                       cutPntsV[k],
                                       new Point2D(tol, _tolV),
                                       new Point2D(tl, _surface.FirstUParameter()),
                                       new Point2D(tr, _surface.LastUParameter()),
                                       FuncTol);
                        //
                        if (solver.IsDone)
                        {
                            tVdisc.Append(solver.Solution().X);
                        }
                    }
                }
            }
        }

        for (i = 2; i <= tVdisc.Length(); i++)
        {
            if (tVdisc.Value(i) - tVdisc.Value(i - 1) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                tVdisc.Remove(i--);
            }
        }

        if (tVdisc.Length() != 0)
        {
            vArr = new NCollection_HArray1<double>(1, tVdisc.Length());
            for (i = 1; i <= vArr.Length(); i++)
            {
                vArr.ChangeValue(i) = tVdisc.Value(i);
            }
        }

        // fusion
        NCollection_Sequence<double> fusion = new();
        if (cArr is not null)
        {
            global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(bArr.ChangeArray1(),
                                  cArr.ChangeArray1(),
                                  fusion,
                                  global::Kerf.Engine.Foundation.Precision.PConfusion());
            bArr = new NCollection_HArray1<double>(1, fusion.Length());
            for (i = 1; i <= bArr.Length(); i++)
            {
                bArr.ChangeValue(i) = fusion.Value(i);
            }
            fusion.Clear();
        }

        if (uArr is not null)
        {
            global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(bArr.ChangeArray1(),
                                  uArr.ChangeArray1(),
                                  fusion,
                                  global::Kerf.Engine.Foundation.Precision.PConfusion());
            bArr = new NCollection_HArray1<double>(1, fusion.Length());
            for (i = 1; i <= bArr.Length(); i++)
            {
                bArr.ChangeValue(i) = fusion.Value(i);
            }
            fusion.Clear();
        }

        if (vArr is not null)
        {
            global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(bArr.ChangeArray1(),
                                  vArr.ChangeArray1(),
                                  fusion,
                                  global::Kerf.Engine.Foundation.Precision.PConfusion());
            bArr = new NCollection_HArray1<double>(1, fusion.Length());
            for (i = 1; i <= bArr.Length(); i++)
            {
                bArr.ChangeValue(i) = fusion.Value(i);
            }
        }

        _tabInt = new NCollection_HArray1<double>(1, bArr.Length());
        for (i = 1; i <= bArr.Length(); i++)
        {
            _tabInt.ChangeValue(i) = bArr.Value(i);
        }
    }

    /// <summary>
    /// A copy of the projection (OCCT's copy constructor: the parts, flags and results are shared) on the curve
    /// trimmed to [First, Last].
    /// </summary>
    public override Curve2DAdaptorBase Trim(double first, double last, double tol)
    {
        CompProjectedCurve hcs = (CompProjectedCurve)MemberwiseClone();
        hcs.Load(_surface);
        hcs.Load(_curve.Trim(first, last, tol));
        return hcs;
    }

    /// <summary>OtherCurve.</summary>
    public override CurveKind GetType() => OtherCurve;

    /// <summary>True if part <paramref name="index"/> of the result of <see cref="Perform"/> is a point.</summary>
    public bool ResultIsPoint(int index) => _resultIsPoint!.Value(index);

    public double GetResult2dUApproxError(int index) => _result2dUApproxError!.Value(index);

    public double GetResult2dVApproxError(int index) => _result2dVApproxError!.Value(index);

    public double GetResult3dApproxError(int index) => _result3dApproxError!.Value(index);

    /// <summary>
    /// The 2D curve of part <paramref name="index"/> (null for a point), frozen: the projection keeps it, and
    /// returns the same object at each call (decision 243).
    /// </summary>
    public Curve2D? GetResult2dC(int index)
    {
        Curve2D? curve = _result2dCurve!.Value(index);
        curve?.Freeze();
        return curve;
    }

    /// <summary>
    /// The 3D curve of part <paramref name="index"/> (null for a point), frozen: the projection keeps it, and
    /// returns the same object at each call (decision 243).
    /// </summary>
    public Curve3D? GetResult3dC(int index)
    {
        Curve3D? curve = _result3dCurve!.Value(index);
        curve?.Freeze();
        return curve;
    }

    /// <exception cref="TypeMismatchException">The part is not a point.</exception>
    public Point2D GetResult2dP(int index)
    {
        Standard_TypeMismatch_Raise_if(!_resultIsPoint!.Value(index),
                                       "ProjLib_CompProjectedCurve : result is not a point 2d");
        return _result2dPoint!.Value(index);
    }

    /// <exception cref="TypeMismatchException">The part is not a point.</exception>
    public Point3D GetResult3dP(int index)
    {
        Standard_TypeMismatch_Raise_if(!_resultIsPoint!.Value(index),
                                       "ProjLib_CompProjectedCurve : result is not a point 3d");
        return _result3dPoint!.Value(index);
    }

    public bool GetProj2d() => _proj2d;

    public bool GetProj3d() => _proj3d;

    private void UpdateTripleByTrapCriteria(ref Point3D point)
    {
        bool isProblemsPossible = false;
        // Check possible traps cases:

        // 25892 bug.
        if (_surface.GetType() == SurfaceOfRevolution)
        {
            // Compute maximal deviation from 3D and choose the biggest one.
            double vRes = _surface.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion());
            double maxTol = Max(global::Kerf.Engine.Foundation.Precision.PConfusion(), vRes);

            if (Math.Abs(point.Z - _surface.FirstVParameter()) < maxTol
                || Math.Abs(point.Z - _surface.LastVParameter()) < maxTol)
            {
                isProblemsPossible = true;
            }
        }

        // 27135 bug. Trap on degenerated edge.
        if (_surface.GetType() == SurfaceKind.Sphere
            && (Math.Abs(point.Z - _surface.FirstVParameter()) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                || Math.Abs(point.Z - _surface.LastVParameter()) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                || Math.Abs(point.Y - _surface.FirstUParameter()) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                || Math.Abs(point.Y - _surface.LastUParameter()) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            isProblemsPossible = true;
        }

        if (!isProblemsPossible)
        {
            return;
        }

        double u, v;
        bool isDone = InitialPoint(_curve.Value(point.X),
                                   point.X,
                                   _curve,
                                   _surface,
                                   global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                   global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                   out u,
                                   out v,
                                   _maxDist);

        if (!isDone)
        {
            return;
        }

        // Restore original position in case of period jump.
        if (_surface.IsUPeriodic
            && Math.Abs(Math.Abs(u - point.Y) - _surface.UPeriod()) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            u = point.Y;
        }
        if (_surface.IsVPeriodic
            && Math.Abs(Math.Abs(v - point.Z) - _surface.VPeriod()) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            v = point.Z;
        }
        point = point.WithY(u);
        point = point.WithZ(v);
    }

    /// <summary>Compute split points in the parameter space of the curve.</summary>
    private static void BuildCurveSplits(CurveAdaptorBase curve,
                                         SurfaceAdaptorBase surface,
                                         double tolU,
                                         double tolV,
                                         List<double> splits)
    {
        SplitDS ds = new(curve, surface, splits);

        ExtPS extPS = new();
        extPS.Initialize(surface,
                           surface.FirstUParameter(),
                           surface.LastUParameter(),
                           surface.FirstVParameter(),
                           surface.LastVParameter(),
                           tolU,
                           tolV);
        extPS.SetFlag(ExtFlag_MIN);
        ds.myExtPS = extPS;

        if (surface.IsUPeriodic)
        {
            ds.PeriodicDir = 0;
            SplitOnDirection(ds);
        }
        if (surface.IsVPeriodic)
        {
            ds.PeriodicDir = 1;
            SplitOnDirection(ds);
        }

        CollectionsMarshal.AsSpan(ds.Splits).Sort(Comparator);
    }

    /// <summary>Perform splitting on a specified direction. Sub-method in BuildCurveSplits.</summary>
    private static void SplitOnDirection(SplitDS splitDS)
    {
        // Algorithm:
        // Create 3D curve which is correspond to the periodic bound in 2d space.
        // Run curve / curve extrema and run extrema point / surface to check that
        // the point will be projected to the periodic bound.
        // In this method assumed that the points cannot be closer to each other that 1% of the parameter
        // space.

        Point2D startPnt = new(splitDS.mySurface.FirstUParameter(),
                                 splitDS.mySurface.FirstVParameter());
        Direction2D dir = new(splitDS.PeriodicDir, splitDS.PeriodicDir == 0 ? 1 : 0);

        splitDS.PerMinParam = splitDS.PeriodicDir == 0 ? splitDS.mySurface.FirstUParameter()
                                                                 : splitDS.mySurface.FirstVParameter();
        splitDS.PerMaxParam = splitDS.PeriodicDir == 0 ? splitDS.mySurface.LastUParameter()
                                                                 : splitDS.mySurface.LastVParameter();
        double last2DParam =
            splitDS.PeriodicDir != 0
                ? splitDS.mySurface.LastUParameter() - splitDS.mySurface.FirstUParameter()
                : splitDS.mySurface.LastVParameter() - splitDS.mySurface.FirstVParameter();

        // Create line which is represent periodic border.
        Curve2D c2GC = new LineCurve2D(startPnt, dir);
        CurveAdaptor2D c = new(c2GC, 0, last2DParam);
        CurveOnSurface cOnS = new(c, splitDS.mySurface);
        splitDS.ExtCCCurve1 = cOnS;
        splitDS.ExtCCLast2DParam = last2DParam;

        FindSplitPoint(splitDS,
                       splitDS.myCurve.FirstParameter(), // Initial curve range.
                       splitDS.myCurve.LastParameter());
    }

    /// <summary>Perform recursive search of the split points.</summary>
    private static void FindSplitPoint(SplitDS splitDS, double minParam, double maxParam)
    {
        // Make extrema copy to avoid dependencies between different levels of the recursion.
        ExtCC extCC = new();
        extCC.SetCurve(1, splitDS.ExtCCCurve1!);
        extCC.SetCurve(2, splitDS.myCurve);
        extCC.SetSingleSolutionFlag(true); // Search only one solution since multiple invocations are needed.
        extCC.SetRange(1, 0, splitDS.ExtCCLast2DParam);
        extCC.SetRange(2, minParam, maxParam);
        extCC.Perform();

        if (extCC.IsDone && !extCC.IsParallel)
        {
            int nbExt = extCC.ExtCount;
            for (int idx = 1; idx <= nbExt; ++idx)
            {
                POnCurv pOnC1, pOnC2;
                extCC.Points(idx, out pOnC1, out pOnC2);

                splitDS.myExtPS!.Perform(pOnC2.Value());
                if (!splitDS.myExtPS.IsDone)
                {
                    return;
                }

                // Find point with the minimal Euclidean distance to avoid
                // false positive points detection.
                int minIdx = -1;
                double minSqDist = RealLast();
                int nbPext = splitDS.myExtPS.ExtCount;
                for (int pIdx = 1; pIdx <= nbPext; ++pIdx)
                {
                    double currSqDist = splitDS.myExtPS.SquareDistance(pIdx);

                    if (currSqDist < minSqDist)
                    {
                        minSqDist = currSqDist;
                        minIdx = pIdx;
                    }
                }

                // Check that is point will be projected to the periodic border.
                // OCCT: with no extremum aMinIdx stays -1 and Point(-1) is out of range (Standard_OutOfRange in
                // the port, as OCCT's check throws in a debug build).
                POnSurf pOnS = splitDS.myExtPS.Point(minIdx);
                double u, v, projParam;
                pOnS.Parameter(out u, out v);
                projParam = splitDS.PeriodicDir != 0 ? v : u;

                if (Math.Abs(projParam - splitDS.PerMinParam) < global::Kerf.Engine.Foundation.Precision.PConfusion()
                    || Math.Abs(projParam - splitDS.PerMaxParam) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    double param = pOnC2.Parameter();
                    double cfParam = splitDS.myCurve.FirstParameter();
                    double clParam = splitDS.myCurve.LastParameter();

                    if (param > cfParam + global::Kerf.Engine.Foundation.Precision.PConfusion()
                        && param < clParam - global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        // Add only inner points.
                        splitDS.Splits.Add(param);
                    }

                    const double deltaCoeff = 0.01;
                    double delta = (maxParam - minParam + clParam - cfParam) * deltaCoeff;

                    if (param - delta > minParam + global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        FindSplitPoint(splitDS, minParam, param - delta); // Curve parameters.
                    }

                    if (param + delta < maxParam - global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        FindSplitPoint(splitDS, param + delta, maxParam); // Curve parameters.
                    }
                }
            } // for (int anIdx = 1; anIdx <= aNbExt; ++anIdx)
        }
    }
}
