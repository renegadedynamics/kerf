// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_CompProjectedCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_CompProjectedCurve.cxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_HCompProjectedCurve.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1997-09-23
// Created by: Roman BORISOV

using static Occt.Extrema_ExtFlag;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The orthogonal projection of a 3D curve on a surface, as a 2D curve in the surface's parametric space. The
/// projection may be made of several continuous parts (<see cref="NbCurves"/>), each a sequence of (t, u, v)
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
public class ProjLib_CompProjectedCurve : Adaptor2d_Curve2d
{
    private const double FuncTol = 1.0e-10;

    // Null until loaded (the default constructor), as OCCT's handles.
    private Adaptor3d_Surface mySurface = null!;
    private Adaptor3d_Curve myCurve = null!;
    private int myNbCurves;
    private NCollection_HSequence<NCollection_HSequence<gp_Pnt>> mySequence = null!;
    private NCollection_HArray1<bool>? myUIso;
    private NCollection_HArray1<bool>? myVIso;
    private NCollection_HArray1<bool>? mySnglPnts;
    private NCollection_HArray1<double>? myMaxDistance;
    private NCollection_HArray1<double>? myTabInt;
    private double myTol3d;
    private GeomAbs_Shape myContinuity;
    private int myMaxDegree;
    private int myMaxSeg;
    private bool myProj2d;
    private bool myProj3d;
    private double myMaxDist;
    private double myTolU;
    private double myTolV;

    private NCollection_HArray1<bool>? myResultIsPoint;
    private NCollection_HArray1<double>? myResult2dUApproxError;
    private NCollection_HArray1<double>? myResult2dVApproxError;
    private NCollection_HArray1<double>? myResult3dApproxError;
    private NCollection_HArray1<gp_Pnt>? myResult3dPoint;
    private NCollection_HArray1<gp_Pnt2d>? myResult2dPoint;
    private NCollection_HArray1<Geom_Curve?>? myResult3dCurve;
    private NCollection_HArray1<Geom2d_Curve?>? myResult2dCurve;

    // Structure to perform splits computation.
    // This structure is not thread-safe since operations under mySplits should be performed in a
    // critical section. myPeriodicDir - 0 for U periodicity and 1 for V periodicity.
    private sealed class SplitDS
    {
        public SplitDS(Adaptor3d_Curve theCurve, Adaptor3d_Surface theSurface, NCollection_DynamicArray<double> theSplits)
        {
            myCurve = theCurve;
            mySurface = theSurface;
            mySplits = theSplits;
            myPerMinParam = 0.0;
            myPerMaxParam = 0.0;
            myPeriodicDir = 0;
            myExtCCCurve1 = null;
            myExtCCLast2DParam = 0.0;
            myExtPS = null;
        }

        public readonly Adaptor3d_Curve myCurve;
        public readonly Adaptor3d_Surface mySurface;
        public readonly NCollection_DynamicArray<double> mySplits;

        public double myPerMinParam;
        public double myPerMaxParam;
        public int myPeriodicDir;

        public Adaptor3d_CurveOnSurface? myExtCCCurve1;
        public double myExtCCLast2DParam;

        public Extrema_ExtPS? myExtPS;
    }

    private static int Comparator(double theA, double theB)
    {
        // std::sort with theA < theB.
        return theA < theB ? -1 : (theB < theA ? 1 : 0);
    }

    private static void d1(double t,
                           double u,
                           double v,
                           out gp_Vec2d V,
                           Adaptor3d_Curve Curve,
                           Adaptor3d_Surface Surface)
    {
        gp_Pnt S, C;
        gp_Vec DS1_u, DS1_v, DS2_u, DS2_uv, DS2_v, DC1_t;
        Surface.D2(u, v, out S, out DS1_u, out DS1_v, out DS2_u, out DS2_v, out DS2_uv);
        Curve.D1(t, out C, out DC1_t);
        gp_Vec Ort = new(C, S); // Ort = S - C

        gp_Vec2d dE_dt = new(-DC1_t * DS1_u, -DC1_t * DS1_v);
        gp_XY dE_du = new(DS1_u * DS1_u + Ort * DS2_u, DS1_u * DS1_v + Ort * DS2_uv);
        gp_XY dE_dv = new(DS1_v * DS1_u + Ort * DS2_uv, DS1_v * DS1_v + Ort * DS2_v);

        double det = dE_du.X() * dE_dv.Y() - dE_du.Y() * dE_dv.X();
        if (Math.Abs(det) < gp.Resolution())
        {
            throw new Standard_ConstructionError();
        }

        gp_Mat2d M = new(new gp_XY(dE_dv.Y() / det, -dE_du.Y() / det), new gp_XY(-dE_dv.X() / det, dE_du.X() / det));

        V = -new gp_Vec2d(new gp_Vec2d(M.Row(1)) * dE_dt, new gp_Vec2d(M.Row(2)) * dE_dt);
    }

    private static void d2(double t,
                           double u,
                           double v,
                           out gp_Vec2d V1,
                           out gp_Vec2d V2,
                           Adaptor3d_Curve Curve,
                           Adaptor3d_Surface Surface)
    {
        gp_Pnt S, C;
        gp_Vec DS1_u, DS1_v, DS2_u, DS2_uv, DS2_v, DS3_u, DS3_v, DS3_uuv, DS3_uvv, DC1_t, DC2_t;
        Surface.D3(u, v, out S, out DS1_u, out DS1_v, out DS2_u, out DS2_v, out DS2_uv, out DS3_u, out DS3_v,
                   out DS3_uuv, out DS3_uvv);
        Curve.D2(t, out C, out DC1_t, out DC2_t);
        gp_Vec Ort = new(C, S);

        gp_Vec2d dE_dt = new(-DC1_t * DS1_u, -DC1_t * DS1_v);
        gp_XY dE_du = new(DS1_u * DS1_u + Ort * DS2_u, DS1_u * DS1_v + Ort * DS2_uv);
        gp_XY dE_dv = new(DS1_v * DS1_u + Ort * DS2_uv, DS1_v * DS1_v + Ort * DS2_v);

        double det = dE_du.X() * dE_dv.Y() - dE_du.Y() * dE_dv.X();
        if (Math.Abs(det) < gp.Resolution())
        {
            throw new Standard_ConstructionError();
        }

        gp_Mat2d M = new(new gp_XY(dE_dv.Y() / det, -dE_du.Y() / det), new gp_XY(-dE_dv.X() / det, dE_du.X() / det));

        // First derivative
        V1 = -new gp_Vec2d(new gp_Vec2d(M.Row(1)) * dE_dt, new gp_Vec2d(M.Row(2)) * dE_dt);

        /* Second derivative */

        // Computation of d2E_dt2 = S1
        gp_Vec2d d2E_dt = new(-DC2_t * DS1_u, -DC2_t * DS1_v);

        // Computation of 2*(d2E/dtdX)(dX/dt) = S2
        gp_Vec2d d2E1_dtdX = new(-DC1_t * DS2_u, -DC1_t * DS2_uv);
        gp_Vec2d d2E2_dtdX = new(-DC1_t * DS2_uv, -DC1_t * DS2_v);
        gp_Vec2d S2 = 2 * new gp_Vec2d(d2E1_dtdX * V1, d2E2_dtdX * V1);

        // Computation of (d2E/dX2)*(dX/dt)2 = S3

        // Row11 = (d2E1/du2, d2E1/dudv)
        double tmp;
        gp_Vec2d Row11 = new(3 * DS1_u * DS2_u + Ort * DS3_u,
                             tmp = 2 * DS1_u * DS2_uv + DS1_v * DS2_u + Ort * DS3_uuv);

        // Row12 = (d2E1/dudv, d2E1/dv2)
        gp_Vec2d Row12 = new(tmp, DS2_v * DS1_u + 2 * DS1_v * DS2_uv + Ort * DS3_uvv);

        // Row21 = (d2E2/du2, d2E2/dudv)
        gp_Vec2d Row21 = new(DS2_u * DS1_v + 2 * DS1_u * DS2_uv + Ort * DS3_uuv,
                             tmp = 2 * DS2_uv * DS1_v + DS1_u * DS2_v + Ort * DS3_uvv);

        // Row22 = (d2E2/duv, d2E2/dvdv)
        gp_Vec2d Row22 = new(tmp, 3 * DS1_v * DS2_v + Ort * DS3_v);

        gp_Vec2d S3 = new(V1 * new gp_Vec2d(Row11 * V1, Row12 * V1), V1 * new gp_Vec2d(Row21 * V1, Row22 * V1));

        gp_Vec2d Sum = d2E_dt + S2 + S3;

        V2 = -new gp_Vec2d(new gp_Vec2d(M.Row(1)) * Sum, new gp_Vec2d(M.Row(2)) * Sum);
    }

    private static void d2CurvOnSurf(double t,
                                     double u,
                                     double v,
                                     out gp_Vec V1,
                                     out gp_Vec V2,
                                     Adaptor3d_Curve Curve,
                                     Adaptor3d_Surface Surface)
    {
        gp_Pnt S, C;
        gp_Vec2d V12d, V22d;
        gp_Vec DS1_u, DS1_v, DS2_u, DS2_uv, DS2_v, DS3_u, DS3_v, DS3_uuv, DS3_uvv, DC1_t, DC2_t;
        Surface.D3(u, v, out S, out DS1_u, out DS1_v, out DS2_u, out DS2_v, out DS2_uv, out DS3_u, out DS3_v,
                   out DS3_uuv, out DS3_uvv);
        Curve.D2(t, out C, out DC1_t, out DC2_t);
        gp_Vec Ort = new(C, S);

        gp_Vec2d dE_dt = new(-DC1_t * DS1_u, -DC1_t * DS1_v);
        gp_XY dE_du = new(DS1_u * DS1_u + Ort * DS2_u, DS1_u * DS1_v + Ort * DS2_uv);
        gp_XY dE_dv = new(DS1_v * DS1_u + Ort * DS2_uv, DS1_v * DS1_v + Ort * DS2_v);

        double det = dE_du.X() * dE_dv.Y() - dE_du.Y() * dE_dv.X();
        if (Math.Abs(det) < gp.Resolution())
        {
            throw new Standard_ConstructionError();
        }

        gp_Mat2d M = new(new gp_XY(dE_dv.Y() / det, -dE_du.Y() / det), new gp_XY(-dE_dv.X() / det, dE_du.X() / det));

        // First derivative
        V12d = -new gp_Vec2d(new gp_Vec2d(M.Row(1)) * dE_dt, new gp_Vec2d(M.Row(2)) * dE_dt);

        /* Second derivative */

        // Computation of d2E_dt2 = S1
        gp_Vec2d d2E_dt = new(-DC2_t * DS1_u, -DC2_t * DS1_v);

        // Computation of 2*(d2E/dtdX)(dX/dt) = S2
        gp_Vec2d d2E1_dtdX = new(-DC1_t * DS2_u, -DC1_t * DS2_uv);
        gp_Vec2d d2E2_dtdX = new(-DC1_t * DS2_uv, -DC1_t * DS2_v);
        gp_Vec2d S2 = 2 * new gp_Vec2d(d2E1_dtdX * V12d, d2E2_dtdX * V12d);

        // Computation of (d2E/dX2)*(dX/dt)2 = S3

        // Row11 = (d2E1/du2, d2E1/dudv)
        double tmp;
        gp_Vec2d Row11 = new(3 * DS1_u * DS2_u + Ort * DS3_u,
                             tmp = 2 * DS1_u * DS2_uv + DS1_v * DS2_u + Ort * DS3_uuv);

        // Row12 = (d2E1/dudv, d2E1/dv2)
        gp_Vec2d Row12 = new(tmp, DS2_v * DS1_u + 2 * DS1_v * DS2_uv + Ort * DS3_uvv);

        // Row21 = (d2E2/du2, d2E2/dudv)
        gp_Vec2d Row21 = new(DS2_u * DS1_v + 2 * DS1_u * DS2_uv + Ort * DS3_uuv,
                             tmp = 2 * DS2_uv * DS1_v + DS1_u * DS2_v + Ort * DS3_uvv);

        // Row22 = (d2E2/duv, d2E2/dvdv)
        gp_Vec2d Row22 = new(tmp, 3 * DS1_v * DS2_v + Ort * DS3_v);

        gp_Vec2d S3 = new(V12d * new gp_Vec2d(Row11 * V12d, Row12 * V12d),
                          V12d * new gp_Vec2d(Row21 * V12d, Row22 * V12d));

        gp_Vec2d Sum = d2E_dt + S2 + S3;

        V22d = -new gp_Vec2d(new gp_Vec2d(M.Row(1)) * Sum, new gp_Vec2d(M.Row(2)) * Sum);

        V1 = DS1_u * V12d.X() + DS1_v * V12d.Y();
        V2 = DS2_u * V12d.X() * V12d.X() + DS1_u * V22d.X() + 2 * DS2_uv * V12d.X() * V12d.Y()
             + DS2_v * V12d.Y() * V12d.Y() + DS1_v * V22d.Y();
    }

    private static bool ExactBound(ref gp_Pnt Sol,
                                   double NotSol,
                                   double Tol,
                                   double TolU,
                                   double TolV,
                                   Adaptor3d_Curve Curve,
                                   Adaptor3d_Surface Surface)
    {
        double U0, V0, t, t1, t2, FirstU, LastU, FirstV, LastV;
        gp_Pnt2d POnS;
        U0 = Sol.Y();
        V0 = Sol.Z();
        FirstU = Surface.FirstUParameter();
        LastU = Surface.LastUParameter();
        FirstV = Surface.FirstVParameter();
        LastV = Surface.LastVParameter();
        // Here we have to compute the boundary that projection is going to intersect
        gp_Vec2d D2d;
        // these variables are to estimate which boundary has more opportunity
        // to be intersected
        double RU1, RU2, RV1, RV2;
        d1(Sol.X(), U0, V0, out D2d, Curve, Surface);
        // Here we assume that D2d != (0, 0)
        if (Math.Abs(D2d.X()) < gp.Resolution())
        {
            RU1 = Precision.Infinite();
            RU2 = Precision.Infinite();
            RV1 = V0 - FirstV;
            RV2 = LastV - V0;
        }
        else if (Math.Abs(D2d.Y()) < gp.Resolution())
        {
            RU1 = U0 - FirstU;
            RU2 = LastU - U0;
            RV1 = Precision.Infinite();
            RV2 = Precision.Infinite();
        }
        else
        {
            RU1 = new gp_Pnt2d(U0, V0).Distance(new gp_Pnt2d(FirstU, V0 + (FirstU - U0) * D2d.Y() / D2d.X()));
            RU2 = new gp_Pnt2d(U0, V0).Distance(new gp_Pnt2d(LastU, V0 + (LastU - U0) * D2d.Y() / D2d.X()));
            RV1 = new gp_Pnt2d(U0, V0).Distance(new gp_Pnt2d(U0 + (FirstV - V0) * D2d.X() / D2d.Y(), FirstV));
            RV2 = new gp_Pnt2d(U0, V0).Distance(new gp_Pnt2d(U0 + (LastV - V0) * D2d.X() / D2d.Y(), LastV));
        }
        NCollection_Sequence<gp_Pnt> Seq = new();
        Seq.Append(new gp_Pnt(FirstU, RU1, 2));
        Seq.Append(new gp_Pnt(LastU, RU2, 2));
        Seq.Append(new gp_Pnt(FirstV, RV1, 3));
        Seq.Append(new gp_Pnt(LastV, RV2, 3));
        int i, j;
        for (i = 1; i <= 3; i++)
        {
            for (j = 1; j <= 4 - i; j++)
            {
                if (Seq.Value(j).Y() < Seq.Value(j + 1).Y())
                {
                    gp_Pnt swp;
                    swp = Seq.Value(j + 1);
                    Seq.ChangeValue(j + 1) = Seq.Value(j);
                    Seq.ChangeValue(j) = swp;
                }
            }
        }

        t = Sol.X();
        t1 = Min(Sol.X(), NotSol);
        t2 = Max(Sol.X(), NotSol);

        bool isDone = false;
        while (!Seq.IsEmpty())
        {
            gp_Pnt P;
            P = Seq.Last();
            Seq.Remove(Seq.Length());
            ProjLib_PrjResolve aPrjPS = new(Curve, Surface, (int)P.Z());
            if ((int)P.Z() == 2)
            {
                aPrjPS.Perform(t,
                               P.X(),
                               V0,
                               new gp_Pnt2d(Tol, TolV),
                               new gp_Pnt2d(t1, Surface.FirstVParameter()),
                               new gp_Pnt2d(t2, Surface.LastVParameter()),
                               FuncTol);
                if (!aPrjPS.IsDone())
                {
                    continue;
                }
                POnS = aPrjPS.Solution();
                Sol = new gp_Pnt(POnS.X(), P.X(), POnS.Y());
                isDone = true;
                break;
            }
            else
            {
                aPrjPS.Perform(t,
                               U0,
                               P.X(),
                               new gp_Pnt2d(Tol, TolU),
                               new gp_Pnt2d(t1, Surface.FirstUParameter()),
                               new gp_Pnt2d(t2, Surface.LastUParameter()),
                               FuncTol);
                if (!aPrjPS.IsDone())
                {
                    continue;
                }
                POnS = aPrjPS.Solution();
                Sol = new gp_Pnt(POnS.X(), POnS.Y(), P.X());
                isDone = true;
                break;
            }
        }

        return isDone;
    }

    private static void DichExactBound(ref gp_Pnt Sol,
                                       double NotSol,
                                       double Tol,
                                       double TolU,
                                       double TolV,
                                       Adaptor3d_Curve Curve,
                                       Adaptor3d_Surface Surface)
    {
        double U0, V0, t;
        gp_Pnt2d POnS;
        U0 = Sol.Y();
        V0 = Sol.Z();
        ProjLib_PrjResolve aPrjPS = new(Curve, Surface, 1);

        double aNotSol = NotSol;
        while (Math.Abs(Sol.X() - aNotSol) > Tol)
        {
            t = (Sol.X() + aNotSol) / 2;
            aPrjPS.Perform(t,
                           U0,
                           V0,
                           new gp_Pnt2d(TolU, TolV),
                           new gp_Pnt2d(Surface.FirstUParameter(), Surface.FirstVParameter()),
                           new gp_Pnt2d(Surface.LastUParameter(), Surface.LastVParameter()),
                           FuncTol,
                           true);

            if (aPrjPS.IsDone())
            {
                POnS = aPrjPS.Solution();
                Sol = new gp_Pnt(t, POnS.X(), POnS.Y());
                U0 = Sol.Y();
                V0 = Sol.Z();
            }
            else
            {
                aNotSol = t;
            }
        }
    }

    /// <remarks><paramref name="U"/> and <paramref name="V"/> are 0 when it returns false, where OCCT leaves them
    /// unchanged; no caller reads them then (decision 32).</remarks>
    private static bool InitialPoint(in gp_Pnt Point,
                                     double t,
                                     Adaptor3d_Curve C,
                                     Adaptor3d_Surface S,
                                     double TolU,
                                     double TolV,
                                     out double U,
                                     out double V,
                                     double theMaxDist)
    {
        ProjLib_PrjResolve aPrjPS = new(C, S, 1);
        double ParU, ParV;
        Extrema_ExtPS aExtPS = new(Point,
                                   S,
                                   S.FirstUParameter(),
                                   S.LastUParameter(),
                                   S.FirstVParameter(),
                                   S.LastVParameter(),
                                   TolU,
                                   TolV,
                                   Extrema_ExtFlag_MIN);
        int argmin = 0;
        double aMaxDist = theMaxDist;
        if (aMaxDist > 0.0)
        {
            aMaxDist *= aMaxDist;
        }
        if (aExtPS.IsDone() && aExtPS.NbExt() != 0)
        {
            int i, Nend;
            // Search for the nearest solution which is also a normal projection
            Nend = aExtPS.NbExt();
            for (i = 1; i <= Nend; i++)
            {
                if (aMaxDist > 0.0 && aMaxDist < aExtPS.SquareDistance(i))
                {
                    continue;
                }
                Extrema_POnSurf POnS = aExtPS.Point(i);
                POnS.Parameter(out ParU, out ParV);
                aPrjPS.Perform(t,
                               ParU,
                               ParV,
                               new gp_Pnt2d(TolU, TolV),
                               new gp_Pnt2d(S.FirstUParameter(), S.FirstVParameter()),
                               new gp_Pnt2d(S.LastUParameter(), S.LastVParameter()),
                               FuncTol,
                               true);
                if (aPrjPS.IsDone())
                {
                    if (argmin == 0 || aExtPS.SquareDistance(i) < aExtPS.SquareDistance(argmin))
                    {
                        argmin = i;
                    }
                }
            }
        }
        if (argmin == 0)
        {
            U = 0.0;
            V = 0.0;
            return false;
        }
        else
        {
            Extrema_POnSurf POnS = aExtPS.Point(argmin);
            POnS.Parameter(out U, out V);
            return true;
        }
    }

    /// <summary>An empty projection.</summary>
    /// <remarks>OCCT leaves the approximation parameters (tolerance, continuity, degree, segments, the 2D and 3D
    /// flags) uninitialised here; they are 0, C0 and false in the port.</remarks>
    public ProjLib_CompProjectedCurve()
    {
        myNbCurves = 0;
        myMaxDist = 0.0;
        myTolU = 0.0;
        myTolV = 0.0;
    }

    /// <summary>
    /// Projects the curve on the surface (calls <see cref="Init"/>), with the tolerances in U and V.
    /// </summary>
    public ProjLib_CompProjectedCurve(Adaptor3d_Surface theSurface,
                                      Adaptor3d_Curve theCurve,
                                      double theTolU,
                                      double theTolV)
    {
        mySurface = theSurface;
        myCurve = theCurve;
        myNbCurves = 0;
        mySequence = new NCollection_HSequence<NCollection_HSequence<gp_Pnt>>();
        myTol3d = 1.0e-6;
        myContinuity = GeomAbs_C2;
        myMaxDegree = 14;
        myMaxSeg = 16;
        myProj2d = true;
        myProj3d = false;
        myMaxDist = -1.0;
        myTolU = theTolU;
        myTolV = theTolV;
        Init();
    }

    /// <summary>
    /// Projects the curve on the surface (calls <see cref="Init"/>); the parts farther than
    /// <paramref name="theMaxDist"/> from the surface are left out (no limit if negative).
    /// </summary>
    public ProjLib_CompProjectedCurve(Adaptor3d_Surface theSurface,
                                      Adaptor3d_Curve theCurve,
                                      double theTolU,
                                      double theTolV,
                                      double theMaxDist)
    {
        mySurface = theSurface;
        myCurve = theCurve;
        myNbCurves = 0;
        mySequence = new NCollection_HSequence<NCollection_HSequence<gp_Pnt>>();
        myTol3d = 1.0e-6;
        myContinuity = GeomAbs_C2;
        myMaxDegree = 14;
        myMaxSeg = 16;
        myProj2d = true;
        myProj3d = false;
        myMaxDist = theMaxDist;
        myTolU = theTolU;
        myTolV = theTolV;
        Init();
    }

    /// <summary>
    /// Projects the curve on the surface (calls <see cref="Init"/>), the 2D tolerances being the surface's
    /// resolutions for <paramref name="theTol3d"/> (at least Precision::PConfusion).
    /// </summary>
    public ProjLib_CompProjectedCurve(double theTol3d,
                                      Adaptor3d_Surface theSurface,
                                      Adaptor3d_Curve theCurve,
                                      double theMaxDist = -1.0)
    {
        mySurface = theSurface;
        myCurve = theCurve;
        myNbCurves = 0;
        mySequence = new NCollection_HSequence<NCollection_HSequence<gp_Pnt>>();
        myTol3d = theTol3d;
        myContinuity = GeomAbs_C2;
        myMaxDegree = 14;
        myMaxSeg = 16;
        myProj2d = true;
        myProj3d = false;
        myMaxDist = theMaxDist;
        myTolU = Max(Precision.PConfusion(), mySurface.UResolution(theTol3d));
        myTolV = Max(Precision.PConfusion(), mySurface.VResolution(theTol3d));

        Init();
    }

    /// <summary>
    /// A copy sharing the projected parts, with shallow copies of the adaptors. As in OCCT, the approximation
    /// parameters and results are not copied.
    /// </summary>
    public override Adaptor2d_Curve2d ShallowCopy()
    {
        ProjLib_CompProjectedCurve aCopy = new();

        if (mySurface is not null)
        {
            aCopy.mySurface = mySurface.ShallowCopy();
        }
        if (myCurve is not null)
        {
            aCopy.myCurve = myCurve.ShallowCopy();
        }
        aCopy.myNbCurves = myNbCurves;
        aCopy.mySequence = mySequence;
        aCopy.myTolU = myTolU;
        aCopy.myTolV = myTolV;
        aCopy.myMaxDist = myMaxDist;
        aCopy.myUIso = myUIso;
        aCopy.myVIso = myVIso;
        aCopy.mySnglPnts = mySnglPnts;
        aCopy.myMaxDistance = myMaxDistance;

        return aCopy;
    }

    /// <summary>
    /// Computes the projected parts: walks along the curve, projecting its points on the surface. As in OCCT, it
    /// appends to the parts already computed.
    /// </summary>
    public void Init()
    {
        myTabInt = null;
        NCollection_DynamicArray<double> aSplits = new();
        aSplits.Clear();

        double Tol; // Tolerance for ExactBound
        int i, Nend = 0, aSplitIdx = 0;
        bool FromLastU = false, isSplitsComputed = false;

        double aTolExt = Precision.PConfusion();
        Extrema_ExtCS CExt = new(myCurve, mySurface, aTolExt, aTolExt);
        if (CExt.IsDone() && CExt.NbExt() != 0)
        {
            // Search for the minimum solution.
            // Avoid usage of extrema result that can be wrong for extrusion.
            if (myMaxDist > 0 &&

                mySurface.GetType() != GeomAbs_SurfaceOfExtrusion)
            {
                double min_val2;
                min_val2 = CExt.SquareDistance(1);

                Nend = CExt.NbExt();
                for (i = 2; i <= Nend; i++)
                {
                    if (CExt.SquareDistance(i) < min_val2)
                    {
                        min_val2 = CExt.SquareDistance(i);
                    }
                }
                if (min_val2 > myMaxDist * myMaxDist)
                {
                    return; // No near solution -> exit.
                }
            }
        }

        double FirstU, LastU, Step, SearchStep, WalkStep, t;

        FirstU = myCurve.FirstParameter();
        LastU = myCurve.LastParameter();
        const double GlobalMinStep = 1.0e-4;
        //<GlobalMinStep> is sufficiently small to provide solving from initial point
        // and, on the other hand, it is sufficiently large to avoid too close solutions.
        double MinStep = 0.01 * (LastU - FirstU), MaxStep = 0.1 * (LastU - FirstU);
        SearchStep = 10 * MinStep;
        Step = SearchStep;

        gp_Pnt2d aLowBorder = new(mySurface.FirstUParameter(), mySurface.FirstVParameter());
        gp_Pnt2d aUppBorder = new(mySurface.LastUParameter(), mySurface.LastVParameter());
        gp_Pnt2d aTol = new(myTolU, myTolV);
        ProjLib_PrjResolve aPrjPS = new(myCurve, mySurface, 1);

        t = FirstU;
        bool new_part;
        double prevDeb = 0.0;
        bool SameDeb = false;

        gp_Pnt Triple = new(), prevTriple = new();

        // Basic loop
        while (t <= LastU)
        {
            // Search for the beginning of a new continuous part
            // to avoid infinite computation in some difficult cases.
            new_part = false;
            if (t > FirstU && Math.Abs(t - prevDeb) <= Precision.PConfusion())
            {
                SameDeb = true;
            }
            while (t <= LastU && !new_part && !FromLastU && !SameDeb)
            {
                prevDeb = t;
                if (t == LastU)
                {
                    FromLastU = true;
                }
                bool initpoint = false;
                double U = 0.0, V = 0.0;
                gp_Pnt CPoint = new();
                double ParT, ParU, ParV;

                // Search an initial point in the list of Extrema Curve-Surface
                if (Nend != 0 && !CExt.IsParallel())
                {
                    for (i = 1; i <= Nend; i++)
                    {
                        Extrema_POnCurv P1;
                        Extrema_POnSurf P2;
                        CExt.Points(i, out P1, out P2);
                        ParT = P1.Parameter();
                        P2.Parameter(out ParU, out ParV);

                        aPrjPS.Perform(ParT, ParU, ParV, aTol, aLowBorder, aUppBorder, FuncTol, true);

                        if (aPrjPS.IsDone()
                            && P1.Parameter() > Max(FirstU, t - Step + Precision.PConfusion())
                            && P1.Parameter() <= t)
                        {
                            t = ParT;
                            U = ParU;
                            V = ParV;
                            CPoint = P1.Value();
                            initpoint = true;
                            break;
                        }
                    }
                }
                if (!initpoint)
                {
                    myCurve.D0(t, out CPoint);
                    // PConfusion - use geometric tolerances in extrema / optimization.
                    initpoint = InitialPoint(CPoint, t, myCurve, mySurface, myTolU, myTolV, out U, out V, myMaxDist);
                }
                if (initpoint)
                {
                    // When U or V lie on surface joint in some cases we cannot use them
                    // as initial point for aPrjPS, so we switch them
                    gp_Vec2d D;

                    if ((mySurface.IsUPeriodic()
                         && Math.Abs(aUppBorder.X() - aLowBorder.X() - mySurface.UPeriod())
                              < Precision.Confusion())
                        || (mySurface.IsVPeriodic()
                            && Math.Abs(aUppBorder.Y() - aLowBorder.Y() - mySurface.VPeriod())
                                 < Precision.Confusion()))
                    {
                        if ((Math.Abs(U - aLowBorder.X()) < mySurface.UResolution(Precision.PConfusion()))
                            && mySurface.IsUPeriodic())
                        {
                            d1(t, U, V, out D, myCurve, mySurface);
                            if (D.X() < 0)
                            {
                                U = aUppBorder.X();
                            }
                        }
                        else if ((Math.Abs(U - aUppBorder.X()) < mySurface.UResolution(Precision.PConfusion()))
                                 && mySurface.IsUPeriodic())
                        {
                            d1(t, U, V, out D, myCurve, mySurface);
                            if (D.X() > 0)
                            {
                                U = aLowBorder.X();
                            }
                        }

                        if ((Math.Abs(V - aLowBorder.Y()) < mySurface.VResolution(Precision.PConfusion()))
                            && mySurface.IsVPeriodic())
                        {
                            d1(t, U, V, out D, myCurve, mySurface);
                            if (D.Y() < 0)
                            {
                                V = aUppBorder.Y();
                            }
                        }
                        else if ((Math.Abs(V - aUppBorder.Y()) <= mySurface.VResolution(Precision.PConfusion()))
                                 && mySurface.IsVPeriodic())
                        {
                            d1(t, U, V, out D, myCurve, mySurface);
                            if (D.Y() > 0)
                            {
                                V = aLowBorder.Y();
                            }
                        }
                    }

                    if (myMaxDist > 0)
                    {
                        // Here we are going to stop if the distance between projection and
                        // corresponding curve point is greater than myMaxDist
                        gp_Pnt POnS;
                        double d;
                        mySurface.D0(U, V, out POnS);
                        d = CPoint.Distance(POnS);
                        if (d > myMaxDist)
                        {
                            mySequence.Clear();
                            myNbCurves = 0;
                            return;
                        }
                    }
                    Triple = new gp_Pnt(t, U, V);
                    if (t != FirstU)
                    {
                        // Search for exact boundary point
                        Tol = Min(myTolU, myTolV);
                        gp_Vec2d aD;
                        d1(Triple.X(), Triple.Y(), Triple.Z(), out aD, myCurve, mySurface);
                        Tol /= Max(Math.Abs(aD.X()), Math.Abs(aD.Y()));

                        if (!ExactBound(ref Triple, t - Step, Tol, myTolU, myTolV, myCurve, mySurface))
                        {
                            DichExactBound(ref Triple, t - Step, Tol, myTolU, myTolV, myCurve, mySurface);
                        }
                    }
                    new_part = true;
                }
                else
                {
                    if (t == LastU)
                    {
                        break;
                    }
                    t += Step;
                    if (t > LastU)
                    {
                        Step = Step + LastU - t;
                        t = LastU;
                    }
                }
            }
            if (!new_part)
            {
                break;
            }

            // We have found a new continuous part
            NCollection_HSequence<gp_Pnt> hSeq = new();
            mySequence.Append(hSeq);
            myNbCurves++;
            mySequence.Value(myNbCurves).Append(Triple);
            prevTriple = Triple;

            if (Triple.X() == LastU)
            {
                break; // return;
            }

            // Computation of WalkStep
            gp_Vec D1, D2;
            double MagnD1, MagnD2;
            d2CurvOnSurf(Triple.X(), Triple.Y(), Triple.Z(), out D1, out D2, myCurve, mySurface);
            MagnD1 = D1.Magnitude();
            MagnD2 = D2.Magnitude();
            if (MagnD2 < Precision.Confusion())
            {
                WalkStep = MaxStep;
            }
            else
            {
                WalkStep = Min(MaxStep, Max(MinStep, 0.1 * MagnD1 / MagnD2));
            }

            Step = WalkStep;

            t = Triple.X() + Step;
            if (t > LastU)
            {
                t = LastU;
            }
            double prevStep = Step;
            double U0, V0;

            // Here we are trying to prolong continuous part
            while (t <= LastU && new_part)
            {
                U0 = Triple.Y() + (Step / prevStep) * (Triple.Y() - prevTriple.Y());
                V0 = Triple.Z() + (Step / prevStep) * (Triple.Z() - prevTriple.Z());
                // adjust U0 to be in [mySurface->FirstUParameter(),mySurface->LastUParameter()]
                U0 = Min(Max(U0, aLowBorder.X()), aUppBorder.X());
                // adjust V0 to be in [mySurface->FirstVParameter(),mySurface->LastVParameter()]
                V0 = Min(Max(V0, aLowBorder.Y()), aUppBorder.Y());

                aPrjPS.Perform(t, U0, V0, aTol, aLowBorder, aUppBorder, FuncTol, true);
                if (!aPrjPS.IsDone())
                {
                    if (Step <= GlobalMinStep)
                    {
                        // Search for exact boundary point
                        Tol = Min(myTolU, myTolV);
                        gp_Vec2d D;
                        d1(Triple.X(), Triple.Y(), Triple.Z(), out D, myCurve, mySurface);
                        Tol /= Max(Math.Abs(D.X()), Math.Abs(D.Y()));

                        if (!ExactBound(ref Triple, t, Tol, myTolU, myTolV, myCurve, mySurface))
                        {
                            DichExactBound(ref Triple, t, Tol, myTolU, myTolV, myCurve, mySurface);
                        }

                        if ((Triple.X()
                             - mySequence.Value(myNbCurves).Value(mySequence.Value(myNbCurves).Length()).X())
                            > 1.0e-10)
                        {
                            mySequence.Value(myNbCurves).Append(Triple);
                        }
                        if ((LastU - Triple.X()) < Tol)
                        {
                            t = LastU + 1;
                            break;
                        } // return;

                        Step = SearchStep;
                        t = Triple.X() + Step;
                        if (t > (LastU - MinStep / 2))
                        {
                            Step = Step + LastU - t;
                            t = LastU;
                        }
                        new_part = false;
                    }
                    else
                    {
                        // decrease step
                        double SaveStep = Step;
                        Step /= 2.0;
                        t = Triple.X() + Step;
                        if (t > (LastU - MinStep / 4))
                        {
                            Step = Step + LastU - t;
                            if (Math.Abs(Step - SaveStep) <= Precision.PConfusion())
                            {
                                Step = GlobalMinStep; // to avoid looping
                            }
                            t = LastU;
                        }
                    }
                }
                // Go further
                else
                {
                    prevTriple = Triple;
                    prevStep = Step;
                    Triple = new gp_Pnt(t, aPrjPS.Solution().X(), aPrjPS.Solution().Y());

                    // Check for possible local traps.
                    UpdateTripleByTrapCriteria(ref Triple);

                    // Protection from case when the whole curve lies on a seam.
                    if (!isSplitsComputed)
                    {
                        bool isUPossible = false;
                        if (mySurface.IsUPeriodic()
                            && (Math.Abs(Triple.Y() - mySurface.FirstUParameter()) > Precision.PConfusion()
                                && Math.Abs(Triple.Y() - mySurface.LastUParameter()) > Precision.PConfusion()))
                        {
                            isUPossible = true;
                        }

                        bool isVPossible = false;
                        if (mySurface.IsVPeriodic()
                            && (Math.Abs(Triple.Z() - mySurface.FirstVParameter()) > Precision.PConfusion()
                                && Math.Abs(Triple.Z() - mySurface.LastVParameter()) > Precision.PConfusion()))
                        {
                            isVPossible = true;
                        }

                        if (isUPossible || isVPossible)
                        {
                            // When point is good conditioned.
                            BuildCurveSplits(myCurve, mySurface, myTolU, myTolV, aSplits);
                            isSplitsComputed = true;
                        }
                    }

                    if ((Triple.X()
                         - mySequence.Value(myNbCurves).Value(mySequence.Value(myNbCurves).Length()).X())
                        > 1.0e-10)
                    {
                        mySequence.Value(myNbCurves).Append(Triple);
                    }
                    if (t == LastU)
                    {
                        t = LastU + 1;
                        break;
                    } // return;
                    // Computation of WalkStep
                    d2CurvOnSurf(Triple.X(), Triple.Y(), Triple.Z(), out D1, out D2, myCurve, mySurface);
                    MagnD1 = D1.Magnitude();
                    MagnD2 = D2.Magnitude();
                    if (MagnD2 < Precision.Confusion())
                    {
                        WalkStep = MaxStep;
                    }
                    else
                    {
                        WalkStep = Min(MaxStep, Max(MinStep, 0.1 * MagnD1 / MagnD2));
                    }

                    Step = WalkStep;
                    t += Step;
                    if (t > (LastU - MinStep / 2))
                    {
                        Step = Step + LastU - t;
                        t = LastU;
                    }

                    // We assume at least one point of cache inside of a split.
                    int aSize = aSplits.Length();
                    for (int anIdx = aSplitIdx; anIdx < aSize; ++anIdx)
                    {
                        double aParam = aSplits.Value(anIdx);
                        if (Math.Abs(aParam - Triple.X()) < Precision.PConfusion())
                        {
                            // The current point is equal to a split point.
                            new_part = false;

                            // Move split index to avoid check of the whole list.
                            ++aSplitIdx;
                            break;
                        }
                        else if (aParam < t + Precision.PConfusion())
                        {
                            // The next point crosses the split point.
                            t = aParam;
                            Step = t - prevTriple.X();
                        }
                    } // for(int anIdx = aSplitIdx; anIdx < aSize; ++anIdx)
                }
            }
        }

        // Sequence post-proceeding.
        int j;

        // 1. Removing poor parts
        int NbPart = myNbCurves;
        int ipart = 1;
        for (i = 1; i <= NbPart; i++)
        {
            //    int NbPoints = mySequence->Value(i)->Length();
            if (mySequence.Value(ipart).Length() < 2)
            {
                mySequence.Remove(ipart);
                myNbCurves--;
            }
            else
            {
                ipart++;
            }
        }

        if (myNbCurves == 0)
        {
            return;
        }

        // 2. Removing common parts of bounds
        for (i = 1; i < myNbCurves; i++)
        {
            if (mySequence.Value(i).Value(mySequence.Value(i).Length()).X()
                >= mySequence.Value(i + 1).Value(1).X())
            {
                mySequence.ChangeValue(i + 1).ChangeValue(1).SetX(
                    mySequence.Value(i).Value(mySequence.Value(i).Length()).X() + 1.0e-12);
            }
        }

        // 3. Computation of the maximum distance from each part of curve to surface

        myMaxDistance = new NCollection_HArray1<double>(1, myNbCurves);
        myMaxDistance.Init(0);
        for (i = 1; i <= myNbCurves; i++)
        {
            for (j = 1; j <= mySequence.Value(i).Length(); j++)
            {
                gp_Pnt POnC, POnS, aTriple;
                double Distance;
                aTriple = mySequence.Value(i).Value(j);
                myCurve.D0(aTriple.X(), out POnC);
                mySurface.D0(aTriple.Y(), aTriple.Z(), out POnS);
                Distance = POnC.Distance(POnS);
                if (myMaxDistance.Value(i) < Distance)
                {
                    myMaxDistance.ChangeValue(i) = Distance;
                }
            }
        }

        // 4. Check the projection to be a single point

        gp_Pnt2d Pmoy = new(), Pcurr, P;
        double AveU, AveV;
        mySnglPnts = new NCollection_HArray1<bool>(1, myNbCurves);
        mySnglPnts.Init(true);

        for (i = 1; i <= myNbCurves; i++)
        {
            // compute an average U and V

            for (j = 1, AveU = 0.0, AveV = 0.0; j <= mySequence.Value(i).Length(); j++)
            {
                AveU += mySequence.Value(i).Value(j).Y();
                AveV += mySequence.Value(i).Value(j).Z();
            }
            AveU /= mySequence.Value(i).Length();
            AveV /= mySequence.Value(i).Length();

            Pmoy.SetCoord(AveU, AveV);
            for (j = 1; j <= mySequence.Value(i).Length(); j++)
            {
                Pcurr = new gp_Pnt2d(mySequence.Value(i).Value(j).Y(), mySequence.Value(i).Value(j).Z());
                if (Pcurr.Distance(Pmoy) > ((myTolU < myTolV) ? myTolV : myTolU))
                {
                    mySnglPnts.SetValue(i, false);
                    break;
                }
            }
        }

        // 5. Check the projection to be an isoparametric curve of the surface

        myUIso = new NCollection_HArray1<bool>(1, myNbCurves);
        myUIso.Init(true);

        myVIso = new NCollection_HArray1<bool>(1, myNbCurves);
        myVIso.Init(true);

        for (i = 1; i <= myNbCurves; i++)
        {
            if (IsSinglePnt(i, out P) || mySequence.Value(i).Length() <= 2)
            {
                myUIso.SetValue(i, false);
                myVIso.SetValue(i, false);
                continue;
            }

            // new test for isoparametrics

            if (mySequence.Value(i).Length() > 2)
            {
                // compute an average U and V

                for (j = 1, AveU = 0.0, AveV = 0.0; j <= mySequence.Value(i).Length(); j++)
                {
                    AveU += mySequence.Value(i).Value(j).Y();
                    AveV += mySequence.Value(i).Value(j).Z();
                }
                AveU /= mySequence.Value(i).Length();
                AveV /= mySequence.Value(i).Length();

                // is i-part U-isoparametric ?
                for (j = 1; j <= mySequence.Value(i).Length(); j++)
                {
                    if (Math.Abs(mySequence.Value(i).Value(j).Y() - AveU) > myTolU)
                    {
                        myUIso.SetValue(i, false);
                        break;
                    }
                }

                // is i-part V-isoparametric ?
                for (j = 1; j <= mySequence.Value(i).Length(); j++)
                {
                    if (Math.Abs(mySequence.Value(i).Value(j).Z() - AveV) > myTolV)
                    {
                        myVIso.SetValue(i, false);
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
        if (myNbCurves == 0)
        {
            return;
        }

        bool approx2d = myProj2d;
        bool approx3d = myProj3d;
        double Udeb, Ufin, UIso, VIso;
        gp_Pnt2d P2d, Pdeb, Pfin;
        gp_Pnt P;
        Adaptor2d_Curve2d? HPCur;
        Adaptor3d_Surface HS = mySurface.ShallowCopy(); // For expand bounds of surface
        Geom2d_Curve? PCur2d = null;                    // Only for isoparametric projection
        Geom_Curve? PCur3d = null;

        if (myProj2d)
        {
            myResult2dPoint = new NCollection_HArray1<gp_Pnt2d>(1, myNbCurves);
            myResult2dCurve = new NCollection_HArray1<Geom2d_Curve?>(1, myNbCurves);
        }

        if (myProj3d)
        {
            myResult3dPoint = new NCollection_HArray1<gp_Pnt>(1, myNbCurves);
            myResult3dCurve = new NCollection_HArray1<Geom_Curve?>(1, myNbCurves);
        }

        myResultIsPoint = new NCollection_HArray1<bool>(1, myNbCurves);
        myResultIsPoint.Init(false);

        myResult3dApproxError = new NCollection_HArray1<double>(1, myNbCurves);
        myResult3dApproxError.Init(0.0);

        myResult2dUApproxError = new NCollection_HArray1<double>(1, myNbCurves);
        myResult2dUApproxError.Init(0.0);

        myResult2dVApproxError = new NCollection_HArray1<double>(1, myNbCurves);
        myResult2dVApproxError.Init(0.0);

        for (int k = 1; k <= myNbCurves; k++)
        {
            if (IsSinglePnt(k, out P2d)) // Part k of the projection is punctual
            {
                GetSurface()!.D0(P2d.X(), P2d.Y(), out P);
                if (myProj2d)
                {
                    myResult2dPoint!.SetValue(k, P2d);
                }
                if (myProj3d)
                {
                    myResult3dPoint!.SetValue(k, P);
                }
                myResultIsPoint.SetValue(k, true);
            }
            else
            {
                Bounds(k, out Udeb, out Ufin);
                gp_Dir2d Dir; // Only for isoparametric projection

                // OCCT: approx2d is cleared for good by an isoparametric part, so a later part that isn't
                // isoparametric is not approximated in 2D and gets the previous part's PCur2d (occt-issues.md;
                // ProjLib_CompProjectedCurveTests.Perform_IsoThenFreePart_ReusesIsoLine_OcctBug).
                if (IsUIso(k, out UIso)) // Part k of the projection is U-isoparametric curve
                {
                    approx2d = false;

                    D0(Udeb, out Pdeb);
                    D0(Ufin, out Pfin);
                    Udeb = Pdeb.Y();
                    Ufin = Pfin.Y();
                    if (Udeb > Ufin)
                    {
                        Dir = new gp_Dir2d(gp_Dir2d.D.NY);
                        Udeb = -Udeb;
                        Ufin = -Ufin;
                    }
                    else
                    {
                        Dir = new gp_Dir2d(gp_Dir2d.D.Y);
                    }
                    PCur2d = new Geom2d_TrimmedCurve(new Geom2d_Line(new gp_Pnt2d(UIso, 0), Dir), Udeb, Ufin);
                    HPCur = new Geom2dAdaptor_Curve(PCur2d);
                }
                else if (IsVIso(k, out VIso)) // Part k of the projection is V-isoparametric curve
                {
                    approx2d = false;

                    D0(Udeb, out Pdeb);
                    D0(Ufin, out Pfin);
                    Udeb = Pdeb.X();
                    Ufin = Pfin.X();
                    if (Udeb > Ufin)
                    {
                        Dir = new gp_Dir2d(gp_Dir2d.D.NX);
                        Udeb = -Udeb;
                        Ufin = -Ufin;
                    }
                    else
                    {
                        Dir = new gp_Dir2d(gp_Dir2d.D.X);
                    }
                    PCur2d = new Geom2d_TrimmedCurve(new Geom2d_Line(new gp_Pnt2d(0, VIso), Dir), Udeb, Ufin);
                    HPCur = new Geom2dAdaptor_Curve(PCur2d);
                }
                else
                {
                    if (!mySurface.IsUPeriodic())
                    {
                        double U1, U2;
                        double dU = 10.0 * myTolU;

                        U1 = mySurface.FirstUParameter();
                        U2 = mySurface.LastUParameter();
                        U1 -= dU;
                        U2 += dU;

                        HS = HS.UTrim(U1, U2, 0.0);
                    }

                    if (!mySurface.IsVPeriodic())
                    {
                        double V1, V2;
                        double dV = 10.0 * myTolV;

                        V1 = mySurface.FirstVParameter();
                        V2 = mySurface.LastVParameter();
                        V1 -= dV;
                        V2 += dV;

                        HS = HS.VTrim(V1, V2, 0.0);
                    }

                    ProjLib_CompProjectedCurve HP = (ProjLib_CompProjectedCurve)ShallowCopy();
                    HP.Load(HS);
                    HPCur = HP;
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

                    Approx_CurveOnSurface appr = new(HPCur, HS, Udeb, Ufin, myTol3d);
                    appr.Perform(myMaxSeg, myMaxDegree, myContinuity, only3d, only2d);

                    // The approximation is discarded: its curves are read unfrozen (decision 287).
                    if (approx2d)
                    {
                        PCur2d = appr.InternalCurve2d();
                        myResult2dUApproxError.SetValue(k, appr.MaxError2dU());
                        myResult2dVApproxError.SetValue(k, appr.MaxError2dV());
                    }

                    if (approx3d)
                    {
                        PCur3d = appr.InternalCurve3d();
                        myResult3dApproxError.SetValue(k, appr.MaxError3d());
                    }
                }

                if (myProj2d)
                {
                    myResult2dCurve!.SetValue(k, PCur2d);
                }

                if (myProj3d)
                {
                    myResult3dCurve!.SetValue(k, PCur3d);
                }
            }
        }
    }

    /// <summary>Sets the 3D tolerance of the approximation (default 1e-6).</summary>
    public void SetTol3d(double theTol3d)
    {
        myTol3d = theTol3d;
    }

    /// <summary>Sets the continuity of the approximation (default C2).</summary>
    public void SetContinuity(GeomAbs_Shape theContinuity)
    {
        myContinuity = theContinuity;
    }

    /// <summary>Sets the maximum degree of the approximation (default 14); ignored if less than 1.</summary>
    public void SetMaxDegree(int theMaxDegree)
    {
        if (theMaxDegree < 1)
        {
            return;
        }
        myMaxDegree = theMaxDegree;
    }

    /// <summary>Sets the maximum number of segments of the approximation (default 16); ignored if less than 1.</summary>
    public void SetMaxSeg(int theMaxSeg)
    {
        if (theMaxSeg < 1)
        {
            return;
        }
        myMaxSeg = theMaxSeg;
    }

    /// <summary>Whether <see cref="Perform"/> builds 3D curves (default false).</summary>
    public void SetProj3d(bool theProj3d)
    {
        myProj3d = theProj3d;
    }

    /// <summary>Whether <see cref="Perform"/> builds 2D curves (default true).</summary>
    public void SetProj2d(bool theProj2d)
    {
        myProj2d = theProj2d;
    }

    /// <summary>Changes the surface (kept by reference).</summary>
    public void Load(Adaptor3d_Surface S)
    {
        mySurface = S;
    }

    /// <summary>Changes the curve (kept by reference).</summary>
    public void Load(Adaptor3d_Curve C)
    {
        myCurve = C;
    }

    /// <summary>The surface (the caller's adaptor); null for an empty projection.</summary>
    public Adaptor3d_Surface? GetSurface() => mySurface;

    /// <summary>The curve (the caller's adaptor); null for an empty projection.</summary>
    public Adaptor3d_Curve? GetCurve() => myCurve;

    public void GetTolerance(out double TolU, out double TolV)
    {
        TolU = myTolU;
        TolV = myTolV;
    }

    /// <summary>The number of continuous parts of the projection.</summary>
    public int NbCurves() => myNbCurves;

    /// <summary>The curve parameters bounding part <paramref name="Index"/>.</summary>
    /// <exception cref="Standard_NoSuchObject">Index is out of range.</exception>
    public void Bounds(int Index, out double Udeb, out double Ufin)
    {
        if (Index < 1 || Index > myNbCurves)
        {
            throw new Standard_NoSuchObject();
        }
        Udeb = mySequence.Value(Index).Value(1).X();
        Ufin = mySequence.Value(Index).Value(mySequence.Value(Index).Length()).X();
    }

    /// <summary>True if part <paramref name="Index"/> projects to a single point, <paramref name="P"/>.</summary>
    /// <exception cref="Standard_NoSuchObject">Index is out of range.</exception>
    public bool IsSinglePnt(int Index, out gp_Pnt2d P)
    {
        if (Index < 1 || Index > myNbCurves)
        {
            throw new Standard_NoSuchObject();
        }
        P = new gp_Pnt2d(mySequence.Value(Index).Value(1).Y(), mySequence.Value(Index).Value(1).Z());
        return mySnglPnts!.Value(Index);
    }

    /// <summary>True if part <paramref name="Index"/> is a U-isoparametric line, at <paramref name="U"/>.</summary>
    /// <exception cref="Standard_NoSuchObject">Index is out of range.</exception>
    public bool IsUIso(int Index, out double U)
    {
        if (Index < 1 || Index > myNbCurves)
        {
            throw new Standard_NoSuchObject();
        }
        U = mySequence.Value(Index).Value(1).Y();
        return myUIso!.Value(Index);
    }

    /// <summary>True if part <paramref name="Index"/> is a V-isoparametric line, at <paramref name="V"/>.</summary>
    /// <exception cref="Standard_NoSuchObject">Index is out of range.</exception>
    public bool IsVIso(int Index, out double V)
    {
        if (Index < 1 || Index > myNbCurves)
        {
            throw new Standard_NoSuchObject();
        }
        V = mySequence.Value(Index).Value(1).Z();
        return myVIso!.Value(Index);
    }

    public override gp_Pnt2d Value(double t)
    {
        gp_Pnt2d P;
        D0(t, out P);
        return P;
    }

    /// <exception cref="Standard_DomainError">U is in none of the parts.</exception>
    public override void D0(double U, out gp_Pnt2d P)
    {
        int i, j;
        double Udeb, Ufin;
        bool found = false;

        for (i = 1; i <= myNbCurves; i++)
        {
            Bounds(i, out Udeb, out Ufin);
            if (U >= Udeb && U <= Ufin)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            throw new Standard_DomainError("ProjLib_CompProjectedCurve::D0");
        }

        double U0, V0;

        int End = mySequence.Value(i).Length();
        for (j = 1; j < End; j++)
        {
            if ((U >= mySequence.Value(i).Value(j).X()) && (U <= mySequence.Value(i).Value(j + 1).X()))
            {
                break;
            }
        }

        //  U0 = mySequence->Value(i)->Value(j).Y();
        //  V0 = mySequence->Value(i)->Value(j).Z();

        //  Cubic Interpolation
        if (mySequence.Value(i).Length() < 4
            || (Math.Abs(U - mySequence.Value(i).Value(j).X()) <= Precision.PConfusion()))
        {
            U0 = mySequence.Value(i).Value(j).Y();
            V0 = mySequence.Value(i).Value(j).Z();
        }
        else if (Math.Abs(U - mySequence.Value(i).Value(j + 1).X()) <= Precision.PConfusion())
        {
            U0 = mySequence.Value(i).Value(j + 1).Y();
            V0 = mySequence.Value(i).Value(j + 1).Z();
        }
        else
        {
            if (j == 1)
            {
                j = 2;
            }
            if (j > mySequence.Value(i).Length() - 2)
            {
                j = mySequence.Value(i).Length() - 2;
            }

            gp_Vec2d I1, I2, I3, I21, I22, I31, Y1, Y2, Y3, Y4, Res;
            double X1, X2, X3, X4;

            X1 = mySequence.Value(i).Value(j - 1).X();
            X2 = mySequence.Value(i).Value(j).X();
            X3 = mySequence.Value(i).Value(j + 1).X();
            X4 = mySequence.Value(i).Value(j + 2).X();

            Y1 = new gp_Vec2d(mySequence.Value(i).Value(j - 1).Y(), mySequence.Value(i).Value(j - 1).Z());
            Y2 = new gp_Vec2d(mySequence.Value(i).Value(j).Y(), mySequence.Value(i).Value(j).Z());
            Y3 = new gp_Vec2d(mySequence.Value(i).Value(j + 1).Y(), mySequence.Value(i).Value(j + 1).Z());
            Y4 = new gp_Vec2d(mySequence.Value(i).Value(j + 2).Y(), mySequence.Value(i).Value(j + 2).Z());

            I1 = (Y1 - Y2) / (X1 - X2);
            I2 = (Y2 - Y3) / (X2 - X3);
            I3 = (Y3 - Y4) / (X3 - X4);

            I21 = (I1 - I2) / (X1 - X3);
            I22 = (I2 - I3) / (X2 - X4);

            I31 = (I21 - I22) / (X1 - X4);

            Res = Y1 + (U - X1) * (I1 + (U - X2) * (I21 + (U - X3) * I31));

            U0 = Res.X();
            V0 = Res.Y();

            if (U0 < mySurface.FirstUParameter())
            {
                U0 = mySurface.FirstUParameter();
            }
            else if (U0 > mySurface.LastUParameter())
            {
                U0 = mySurface.LastUParameter();
            }

            if (V0 < mySurface.FirstVParameter())
            {
                V0 = mySurface.FirstVParameter();
            }
            else if (V0 > mySurface.LastVParameter())
            {
                V0 = mySurface.LastVParameter();
            }
        }
        // End of cubic interpolation

        ProjLib_PrjResolve aPrjPS = new(myCurve, mySurface, 1);
        aPrjPS.Perform(U,
                       U0,
                       V0,
                       new gp_Pnt2d(myTolU, myTolV),
                       new gp_Pnt2d(mySurface.FirstUParameter(), mySurface.FirstVParameter()),
                       new gp_Pnt2d(mySurface.LastUParameter(), mySurface.LastVParameter()),
                       FuncTol);
        if (aPrjPS.IsDone())
        {
            P = aPrjPS.Solution();
        }
        else
        {
            gp_Pnt thePoint = myCurve.Value(U);
            Extrema_ExtPS aExtPS = new(thePoint, mySurface, myTolU, myTolV, Extrema_ExtFlag_MIN);
            if (aExtPS.IsDone() && aExtPS.NbExt() != 0)
            {
                int k, Nend, imin = 1;
                // Search for the nearest solution which is also a normal projection
                Nend = aExtPS.NbExt();
                for (k = 2; k <= Nend; k++)
                {
                    if (aExtPS.SquareDistance(k) < aExtPS.SquareDistance(imin))
                    {
                        imin = k;
                    }
                }
                Extrema_POnSurf POnS = aExtPS.Point(imin);
                double ParU, ParV;
                POnS.Parameter(out ParU, out ParV);
                P = new gp_Pnt2d(ParU, ParV);
            }
            else
            {
                P = new gp_Pnt2d(U0, V0);
            }
        }
    }

    public override void D1(double t, out gp_Pnt2d P, out gp_Vec2d V)
    {
        double u, v;
        D0(t, out P);
        u = P.X();
        v = P.Y();
        d1(t, u, v, out V, myCurve, mySurface);
    }

    public override void D2(double t, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        double u, v;
        D0(t, out P);
        u = P.X();
        v = P.Y();
        d2(t, u, v, out V1, out V2, myCurve, mySurface);
    }

    /// <exception cref="Standard_OutOfRange">N is less than 1.</exception>
    /// <exception cref="Standard_NotImplemented">N is greater than 2.</exception>
    public override gp_Vec2d DN(double t, int N)
    {
        if (N < 1)
        {
            throw new Standard_OutOfRange("ProjLib_CompProjectedCurve : N must be greater than 0");
        }
        else if (N == 1)
        {
            gp_Pnt2d P;
            gp_Vec2d V;
            D1(t, out P, out V);
            return V;
        }
        else if (N == 2)
        {
            gp_Pnt2d P;
            gp_Vec2d V1, V2;
            D2(t, out P, out V1, out V2);
            return V2;
        }
        else if (N > 2)
        {
            throw new Standard_NotImplemented("ProjLib_CompProjectedCurve::DN");
        }
        return new gp_Vec2d();
    }

    /// <summary>
    /// The (t, u, v) triples of each part. A deep copy in the port, where OCCT returns its own sequences: the
    /// projection keeps (and its shallow copies share) them.
    /// </summary>
    public NCollection_HSequence<NCollection_HSequence<gp_Pnt>> GetSequence()
    {
        NCollection_HSequence<NCollection_HSequence<gp_Pnt>> aCopy = new();
        if (mySequence is not null)
        {
            for (int i = 1; i <= mySequence.Length(); i++)
            {
                aCopy.Append(new NCollection_HSequence<gp_Pnt>(mySequence.Value(i)));
            }
        }
        return aCopy;
    }

    public override double FirstParameter() => myCurve.FirstParameter();

    public override double LastParameter() => myCurve.LastParameter();

    public override GeomAbs_Shape Continuity()
    {
        GeomAbs_Shape ContC = myCurve.Continuity();
        GeomAbs_Shape ContSu = mySurface.UContinuity();
        if (ContSu < ContC)
        {
            ContC = ContSu;
        }
        GeomAbs_Shape ContSv = mySurface.VContinuity();
        if (ContSv < ContC)
        {
            ContC = ContSv;
        }

        return ContC;
    }

    /// <summary>The maximum distance between the curve and its projection in part <paramref name="Index"/>.</summary>
    /// <exception cref="Standard_NoSuchObject">Index is out of range.</exception>
    public double MaxDistance(int Index)
    {
        if (Index < 1 || Index > myNbCurves)
        {
            throw new Standard_NoSuchObject();
        }
        return myMaxDistance!.Value(Index);
    }

    /// <summary>
    /// The number of intervals of continuity <paramref name="S"/>: the parts' bounds, the curve's intervals and
    /// the points where the projection crosses the surface's intervals. Recomputes and caches them.
    /// </summary>
    public override int NbIntervals(GeomAbs_Shape S)
    {
        myTabInt = null;
        BuildIntervals(S);
        return myTabInt!.Length() - 1;
    }

    /// <summary>
    /// The intervals cached by <see cref="NbIntervals"/>, computed for <paramref name="S"/> if none are cached (as
    /// OCCT, a cache computed for another continuity is returned).
    /// </summary>
    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        if (myTabInt is null)
        {
            BuildIntervals(S);
        }
        T.Assign(myTabInt!.Array1());
    }

    private void BuildIntervals(GeomAbs_Shape S)
    {
        GeomAbs_Shape SforS = GeomAbs_CN;
        switch (S)
        {
            case GeomAbs_C0:
                SforS = GeomAbs_C1;
                break;
            case GeomAbs_C1:
                SforS = GeomAbs_C2;
                break;
            case GeomAbs_C2:
                SforS = GeomAbs_C3;
                break;
            case GeomAbs_C3:
                SforS = GeomAbs_CN;
                break;
            case GeomAbs_CN:
                SforS = GeomAbs_CN;
                break;
            default:
                throw new Standard_OutOfRange();
        }
        int i, j, k;
        int NbIntCur = myCurve.NbIntervals(S);
        int NbIntSurU = mySurface.NbUIntervals(SforS);
        int NbIntSurV = mySurface.NbVIntervals(SforS);

        NCollection_Array1<double> CutPntsT = new(1, NbIntCur + 1);
        NCollection_Array1<double> CutPntsU = new(1, NbIntSurU + 1);
        NCollection_Array1<double> CutPntsV = new(1, NbIntSurV + 1);

        myCurve.Intervals(CutPntsT, S);
        mySurface.UIntervals(CutPntsU, SforS);
        mySurface.VIntervals(CutPntsV, SforS);

        double Tl, Tr, Ul, Ur, Vl, Vr, Tol;

        NCollection_HArray1<double>? BArr = null, CArr = null, UArr = null, VArr = null;

        // processing projection bounds
        BArr = new NCollection_HArray1<double>(1, 2 * myNbCurves);
        for (i = 1; i <= myNbCurves; i++)
        {
            Bounds(i, out BArr[2 * i - 1], out BArr[2 * i]);
        }

        // processing curve discontinuities
        if (NbIntCur > 1)
        {
            CArr = new NCollection_HArray1<double>(1, NbIntCur - 1);
            for (i = 1; i <= CArr.Length(); i++)
            {
                CArr.ChangeValue(i) = CutPntsT[i + 1];
            }
        }

        // processing U-surface discontinuities
        NCollection_Sequence<double> TUdisc = new();

        for (k = 2; k <= NbIntSurU; k++)
        {
            //    std::cout<<"CutPntsU("<<k<<") = "<<CutPntsU(k)<<std::endl;
            for (i = 1; i <= myNbCurves; i++)
            {
                for (j = 1; j < mySequence.Value(i).Length(); j++)
                {
                    Ul = mySequence.Value(i).Value(j).Y();
                    Ur = mySequence.Value(i).Value(j + 1).Y();

                    if (Math.Abs(Ul - CutPntsU[k]) <= myTolU)
                    {
                        TUdisc.Append(mySequence.Value(i).Value(j).X());
                    }
                    else if (Math.Abs(Ur - CutPntsU[k]) <= myTolU)
                    {
                        TUdisc.Append(mySequence.Value(i).Value(j + 1).X());
                    }
                    else if ((Ul < CutPntsU[k] && CutPntsU[k] < Ur) || (Ur < CutPntsU[k] && CutPntsU[k] < Ul))
                    {
                        double V;
                        V = (mySequence.Value(i).Value(j).Z() + mySequence.Value(i).Value(j + 1).Z()) / 2;
                        ProjLib_PrjResolve Solver = new(myCurve, mySurface, 2);

                        gp_Vec2d D;
                        gp_Pnt Triple;
                        Triple = mySequence.Value(i).Value(j);
                        d1(Triple.X(), Triple.Y(), Triple.Z(), out D, myCurve, mySurface);
                        if (Math.Abs(D.X()) < Precision.Confusion())
                        {
                            Tol = myTolU;
                        }
                        else
                        {
                            Tol = Min(myTolU, myTolU / Math.Abs(D.X()));
                        }

                        Tl = mySequence.Value(i).Value(j).X();
                        Tr = mySequence.Value(i).Value(j + 1).X();

                        Solver.Perform((Tl + Tr) / 2,
                                       CutPntsU[k],
                                       V,
                                       new gp_Pnt2d(Tol, myTolV),
                                       new gp_Pnt2d(Tl, mySurface.FirstVParameter()),
                                       new gp_Pnt2d(Tr, mySurface.LastVParameter()),
                                       FuncTol);
                        //
                        if (Solver.IsDone())
                        {
                            TUdisc.Append(Solver.Solution().X());
                        }
                    }
                }
            }
        }
        for (i = 2; i <= TUdisc.Length(); i++)
        {
            if (TUdisc.Value(i) - TUdisc.Value(i - 1) < Precision.PConfusion())
            {
                TUdisc.Remove(i--);
            }
        }

        if (TUdisc.Length() != 0)
        {
            UArr = new NCollection_HArray1<double>(1, TUdisc.Length());
            for (i = 1; i <= UArr.Length(); i++)
            {
                UArr.ChangeValue(i) = TUdisc.Value(i);
            }
        }
        // processing V-surface discontinuities
        NCollection_Sequence<double> TVdisc = new();

        for (k = 2; k <= NbIntSurV; k++)
        {
            for (i = 1; i <= myNbCurves; i++)
            {
                //      std::cout<<"CutPntsV("<<k<<") = "<<CutPntsV(k)<<std::endl;
                for (j = 1; j < mySequence.Value(i).Length(); j++)
                {
                    Vl = mySequence.Value(i).Value(j).Z();
                    Vr = mySequence.Value(i).Value(j + 1).Z();

                    if (Math.Abs(Vl - CutPntsV[k]) <= myTolV)
                    {
                        TVdisc.Append(mySequence.Value(i).Value(j).X());
                    }
                    else if (Math.Abs(Vr - CutPntsV[k]) <= myTolV)
                    {
                        TVdisc.Append(mySequence.Value(i).Value(j + 1).X());
                    }
                    else if ((Vl < CutPntsV[k] && CutPntsV[k] < Vr) || (Vr < CutPntsV[k] && CutPntsV[k] < Vl))
                    {
                        double U;
                        U = (mySequence.Value(i).Value(j).Y() + mySequence.Value(i).Value(j + 1).Y()) / 2;
                        ProjLib_PrjResolve Solver = new(myCurve, mySurface, 3);

                        gp_Vec2d D;
                        gp_Pnt Triple;
                        Triple = mySequence.Value(i).Value(j);
                        d1(Triple.X(), Triple.Y(), Triple.Z(), out D, myCurve, mySurface);
                        if (Math.Abs(D.Y()) < Precision.Confusion())
                        {
                            Tol = myTolV;
                        }
                        else
                        {
                            Tol = Min(myTolV, myTolV / Math.Abs(D.Y()));
                        }

                        Tl = mySequence.Value(i).Value(j).X();
                        Tr = mySequence.Value(i).Value(j + 1).X();

                        Solver.Perform((Tl + Tr) / 2,
                                       U,
                                       CutPntsV[k],
                                       new gp_Pnt2d(Tol, myTolV),
                                       new gp_Pnt2d(Tl, mySurface.FirstUParameter()),
                                       new gp_Pnt2d(Tr, mySurface.LastUParameter()),
                                       FuncTol);
                        //
                        if (Solver.IsDone())
                        {
                            TVdisc.Append(Solver.Solution().X());
                        }
                    }
                }
            }
        }

        for (i = 2; i <= TVdisc.Length(); i++)
        {
            if (TVdisc.Value(i) - TVdisc.Value(i - 1) < Precision.PConfusion())
            {
                TVdisc.Remove(i--);
            }
        }

        if (TVdisc.Length() != 0)
        {
            VArr = new NCollection_HArray1<double>(1, TVdisc.Length());
            for (i = 1; i <= VArr.Length(); i++)
            {
                VArr.ChangeValue(i) = TVdisc.Value(i);
            }
        }

        // fusion
        NCollection_Sequence<double> Fusion = new();
        if (CArr is not null)
        {
            GeomLib.FuseIntervals(BArr.ChangeArray1(),
                                  CArr.ChangeArray1(),
                                  Fusion,
                                  Precision.PConfusion());
            BArr = new NCollection_HArray1<double>(1, Fusion.Length());
            for (i = 1; i <= BArr.Length(); i++)
            {
                BArr.ChangeValue(i) = Fusion.Value(i);
            }
            Fusion.Clear();
        }

        if (UArr is not null)
        {
            GeomLib.FuseIntervals(BArr.ChangeArray1(),
                                  UArr.ChangeArray1(),
                                  Fusion,
                                  Precision.PConfusion());
            BArr = new NCollection_HArray1<double>(1, Fusion.Length());
            for (i = 1; i <= BArr.Length(); i++)
            {
                BArr.ChangeValue(i) = Fusion.Value(i);
            }
            Fusion.Clear();
        }

        if (VArr is not null)
        {
            GeomLib.FuseIntervals(BArr.ChangeArray1(),
                                  VArr.ChangeArray1(),
                                  Fusion,
                                  Precision.PConfusion());
            BArr = new NCollection_HArray1<double>(1, Fusion.Length());
            for (i = 1; i <= BArr.Length(); i++)
            {
                BArr.ChangeValue(i) = Fusion.Value(i);
            }
        }

        myTabInt = new NCollection_HArray1<double>(1, BArr.Length());
        for (i = 1; i <= BArr.Length(); i++)
        {
            myTabInt.ChangeValue(i) = BArr.Value(i);
        }
    }

    /// <summary>
    /// A copy of the projection (OCCT's copy constructor: the parts, flags and results are shared) on the curve
    /// trimmed to [First, Last].
    /// </summary>
    public override Adaptor2d_Curve2d Trim(double First, double Last, double Tol)
    {
        ProjLib_CompProjectedCurve HCS = (ProjLib_CompProjectedCurve)MemberwiseClone();
        HCS.Load(mySurface);
        HCS.Load(myCurve.Trim(First, Last, Tol));
        return HCS;
    }

    /// <summary>OtherCurve.</summary>
    public override GeomAbs_CurveType GetType() => GeomAbs_OtherCurve;

    /// <summary>True if part <paramref name="theIndex"/> of the result of <see cref="Perform"/> is a point.</summary>
    public bool ResultIsPoint(int theIndex) => myResultIsPoint!.Value(theIndex);

    public double GetResult2dUApproxError(int theIndex) => myResult2dUApproxError!.Value(theIndex);

    public double GetResult2dVApproxError(int theIndex) => myResult2dVApproxError!.Value(theIndex);

    public double GetResult3dApproxError(int theIndex) => myResult3dApproxError!.Value(theIndex);

    /// <summary>
    /// The 2D curve of part <paramref name="theIndex"/> (null for a point), frozen: the projection keeps it, and
    /// returns the same object at each call (decision 243).
    /// </summary>
    public Geom2d_Curve? GetResult2dC(int theIndex)
    {
        Geom2d_Curve? aCurve = myResult2dCurve!.Value(theIndex);
        aCurve?.Freeze();
        return aCurve;
    }

    /// <summary>
    /// The 3D curve of part <paramref name="theIndex"/> (null for a point), frozen: the projection keeps it, and
    /// returns the same object at each call (decision 243).
    /// </summary>
    public Geom_Curve? GetResult3dC(int theIndex)
    {
        Geom_Curve? aCurve = myResult3dCurve!.Value(theIndex);
        aCurve?.Freeze();
        return aCurve;
    }

    /// <exception cref="Standard_TypeMismatch">The part is not a point.</exception>
    public gp_Pnt2d GetResult2dP(int theIndex)
    {
        Standard_TypeMismatch_Raise_if(!myResultIsPoint!.Value(theIndex),
                                       "ProjLib_CompProjectedCurve : result is not a point 2d");
        return myResult2dPoint!.Value(theIndex);
    }

    /// <exception cref="Standard_TypeMismatch">The part is not a point.</exception>
    public gp_Pnt GetResult3dP(int theIndex)
    {
        Standard_TypeMismatch_Raise_if(!myResultIsPoint!.Value(theIndex),
                                       "ProjLib_CompProjectedCurve : result is not a point 3d");
        return myResult3dPoint!.Value(theIndex);
    }

    public bool GetProj2d() => myProj2d;

    public bool GetProj3d() => myProj3d;

    private void UpdateTripleByTrapCriteria(ref gp_Pnt thePoint)
    {
        bool isProblemsPossible = false;
        // Check possible traps cases:

        // 25892 bug.
        if (mySurface.GetType() == GeomAbs_SurfaceOfRevolution)
        {
            // Compute maximal deviation from 3D and choose the biggest one.
            double aVRes = mySurface.VResolution(Precision.Confusion());
            double aMaxTol = Max(Precision.PConfusion(), aVRes);

            if (Math.Abs(thePoint.Z() - mySurface.FirstVParameter()) < aMaxTol
                || Math.Abs(thePoint.Z() - mySurface.LastVParameter()) < aMaxTol)
            {
                isProblemsPossible = true;
            }
        }

        // 27135 bug. Trap on degenerated edge.
        if (mySurface.GetType() == GeomAbs_Sphere
            && (Math.Abs(thePoint.Z() - mySurface.FirstVParameter()) < Precision.PConfusion()
                || Math.Abs(thePoint.Z() - mySurface.LastVParameter()) < Precision.PConfusion()
                || Math.Abs(thePoint.Y() - mySurface.FirstUParameter()) < Precision.PConfusion()
                || Math.Abs(thePoint.Y() - mySurface.LastUParameter()) < Precision.PConfusion()))
        {
            isProblemsPossible = true;
        }

        if (!isProblemsPossible)
        {
            return;
        }

        double U, V;
        bool isDone = InitialPoint(myCurve.Value(thePoint.X()),
                                   thePoint.X(),
                                   myCurve,
                                   mySurface,
                                   Precision.PConfusion(),
                                   Precision.PConfusion(),
                                   out U,
                                   out V,
                                   myMaxDist);

        if (!isDone)
        {
            return;
        }

        // Restore original position in case of period jump.
        if (mySurface.IsUPeriodic()
            && Math.Abs(Math.Abs(U - thePoint.Y()) - mySurface.UPeriod()) < Precision.PConfusion())
        {
            U = thePoint.Y();
        }
        if (mySurface.IsVPeriodic()
            && Math.Abs(Math.Abs(V - thePoint.Z()) - mySurface.VPeriod()) < Precision.PConfusion())
        {
            V = thePoint.Z();
        }
        thePoint.SetY(U);
        thePoint.SetZ(V);
    }

    /// <summary>Compute split points in the parameter space of the curve.</summary>
    private static void BuildCurveSplits(Adaptor3d_Curve theCurve,
                                         Adaptor3d_Surface theSurface,
                                         double theTolU,
                                         double theTolV,
                                         NCollection_DynamicArray<double> theSplits)
    {
        SplitDS aDS = new(theCurve, theSurface, theSplits);

        Extrema_ExtPS anExtPS = new();
        anExtPS.Initialize(theSurface,
                           theSurface.FirstUParameter(),
                           theSurface.LastUParameter(),
                           theSurface.FirstVParameter(),
                           theSurface.LastVParameter(),
                           theTolU,
                           theTolV);
        anExtPS.SetFlag(Extrema_ExtFlag_MIN);
        aDS.myExtPS = anExtPS;

        if (theSurface.IsUPeriodic())
        {
            aDS.myPeriodicDir = 0;
            SplitOnDirection(aDS);
        }
        if (theSurface.IsVPeriodic())
        {
            aDS.myPeriodicDir = 1;
            SplitOnDirection(aDS);
        }

        aDS.mySplits.AsSpan().Sort(Comparator);
    }

    /// <summary>Perform splitting on a specified direction. Sub-method in BuildCurveSplits.</summary>
    private static void SplitOnDirection(SplitDS theSplitDS)
    {
        // Algorithm:
        // Create 3D curve which is correspond to the periodic bound in 2d space.
        // Run curve / curve extrema and run extrema point / surface to check that
        // the point will be projected to the periodic bound.
        // In this method assumed that the points cannot be closer to each other that 1% of the parameter
        // space.

        gp_Pnt2d aStartPnt = new(theSplitDS.mySurface.FirstUParameter(),
                                 theSplitDS.mySurface.FirstVParameter());
        gp_Dir2d aDir = new(theSplitDS.myPeriodicDir, theSplitDS.myPeriodicDir == 0 ? 1 : 0);

        theSplitDS.myPerMinParam = theSplitDS.myPeriodicDir == 0 ? theSplitDS.mySurface.FirstUParameter()
                                                                 : theSplitDS.mySurface.FirstVParameter();
        theSplitDS.myPerMaxParam = theSplitDS.myPeriodicDir == 0 ? theSplitDS.mySurface.LastUParameter()
                                                                 : theSplitDS.mySurface.LastVParameter();
        double aLast2DParam =
            theSplitDS.myPeriodicDir != 0
                ? theSplitDS.mySurface.LastUParameter() - theSplitDS.mySurface.FirstUParameter()
                : theSplitDS.mySurface.LastVParameter() - theSplitDS.mySurface.FirstVParameter();

        // Create line which is represent periodic border.
        Geom2d_Curve aC2GC = new Geom2d_Line(aStartPnt, aDir);
        Geom2dAdaptor_Curve aC = new(aC2GC, 0, aLast2DParam);
        Adaptor3d_CurveOnSurface aCOnS = new(aC, theSplitDS.mySurface);
        theSplitDS.myExtCCCurve1 = aCOnS;
        theSplitDS.myExtCCLast2DParam = aLast2DParam;

        FindSplitPoint(theSplitDS,
                       theSplitDS.myCurve.FirstParameter(), // Initial curve range.
                       theSplitDS.myCurve.LastParameter());
    }

    /// <summary>Perform recursive search of the split points.</summary>
    private static void FindSplitPoint(SplitDS theSplitDS, double theMinParam, double theMaxParam)
    {
        // Make extrema copy to avoid dependencies between different levels of the recursion.
        Extrema_ExtCC anExtCC = new();
        anExtCC.SetCurve(1, theSplitDS.myExtCCCurve1!);
        anExtCC.SetCurve(2, theSplitDS.myCurve);
        anExtCC.SetSingleSolutionFlag(true); // Search only one solution since multiple invocations are needed.
        anExtCC.SetRange(1, 0, theSplitDS.myExtCCLast2DParam);
        anExtCC.SetRange(2, theMinParam, theMaxParam);
        anExtCC.Perform();

        if (anExtCC.IsDone() && !anExtCC.IsParallel())
        {
            int aNbExt = anExtCC.NbExt();
            for (int anIdx = 1; anIdx <= aNbExt; ++anIdx)
            {
                Extrema_POnCurv aPOnC1, aPOnC2;
                anExtCC.Points(anIdx, out aPOnC1, out aPOnC2);

                theSplitDS.myExtPS!.Perform(aPOnC2.Value());
                if (!theSplitDS.myExtPS.IsDone())
                {
                    return;
                }

                // Find point with the minimal Euclidean distance to avoid
                // false positive points detection.
                int aMinIdx = -1;
                double aMinSqDist = RealLast();
                int aNbPext = theSplitDS.myExtPS.NbExt();
                for (int aPIdx = 1; aPIdx <= aNbPext; ++aPIdx)
                {
                    double aCurrSqDist = theSplitDS.myExtPS.SquareDistance(aPIdx);

                    if (aCurrSqDist < aMinSqDist)
                    {
                        aMinSqDist = aCurrSqDist;
                        aMinIdx = aPIdx;
                    }
                }

                // Check that is point will be projected to the periodic border.
                // OCCT: with no extremum aMinIdx stays -1 and Point(-1) is out of range (Standard_OutOfRange in
                // the port, as OCCT's check throws in a debug build).
                Extrema_POnSurf aPOnS = theSplitDS.myExtPS.Point(aMinIdx);
                double U, V, aProjParam;
                aPOnS.Parameter(out U, out V);
                aProjParam = theSplitDS.myPeriodicDir != 0 ? V : U;

                if (Math.Abs(aProjParam - theSplitDS.myPerMinParam) < Precision.PConfusion()
                    || Math.Abs(aProjParam - theSplitDS.myPerMaxParam) < Precision.PConfusion())
                {
                    double aParam = aPOnC2.Parameter();
                    double aCFParam = theSplitDS.myCurve.FirstParameter();
                    double aCLParam = theSplitDS.myCurve.LastParameter();

                    if (aParam > aCFParam + Precision.PConfusion()
                        && aParam < aCLParam - Precision.PConfusion())
                    {
                        // Add only inner points.
                        theSplitDS.mySplits.Append(aParam);
                    }

                    const double aDeltaCoeff = 0.01;
                    double aDelta = (theMaxParam - theMinParam + aCLParam - aCFParam) * aDeltaCoeff;

                    if (aParam - aDelta > theMinParam + Precision.PConfusion())
                    {
                        FindSplitPoint(theSplitDS, theMinParam, aParam - aDelta); // Curve parameters.
                    }

                    if (aParam + aDelta < theMaxParam - Precision.PConfusion())
                    {
                        FindSplitPoint(theSplitDS, aParam + aDelta, theMaxParam); // Curve parameters.
                    }
                }
            } // for (int anIdx = 1; anIdx <= aNbExt; ++anIdx)
        }
    }
}
