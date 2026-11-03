// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Test-only stand-ins for GeomAdaptor_Curve (unit 2A) and Geom2dAdaptor_Curve (unit 2B), written after OCCT V8_0_1's
// src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Curve.cxx and src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor_Curve.cxx
// so that algorithms over adaptors (GCPnts, CPnts, ...) compute what native OCCT computes. This file is compiled into
// both Kerf.Kernel.ModelingData.Tests and Kerf.Kernel.Oracle.Tests. Replace with the real adaptors once they are merged.
//
// Mirrored: trimmed-curve unwrapping (the adaptor keeps the basis curve), lines and conics evaluated with ElCLib,
// Bezier curves through a BSplCLib_Cache over the flat Bezier knots, B-splines through a BSplCLib_Cache with
// LocalD0..LocalDN at exactly First/Last (IsBoundary), LocalContinuity, Continuity, NbIntervals/Intervals
// (BSplCLib.Intervals), Resolution, the type accessors, Degree, IsRational, NbPoles, NbKnots, IsPeriodic, Period,
// IsClosed, Trim, ShallowCopy, NbSamples (2D).
// Simplified (test only): offset curves and curves carrying an evaluation representation are evaluated through the
// Geom curve's own EvalD*, and offset curves report a single interval and the curve's own continuity (OCCT evaluates
// them through Geom_OffsetCurveUtils over a basis adaptor); ShallowCopy shares nothing but the curve and bounds.

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Foundation.Standard;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>A test stand-in for GeomAdaptor_Curve: a Geom_Curve seen through Adaptor3d_Curve (see the file header).</summary>
internal sealed class TestGeomAdaptor_Curve : CurveAdaptorBase
{
    private static readonly double PosTol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 2;

    private Curve3D? myCurve;
    private CurveKind myTypeCurve = OtherCurve;
    private double myFirst;
    private double myLast;

    private Line3D myLin = new();
    private Circle3D myCirc = new();
    private Ellipse3D myElips = new();
    private Hyperbola3D myHypr = new();
    private Parabola3D myParab = new();
    private BezierCurve3D? myBezier;
    private BSplineCurve3D? myBSpline;
    private bool myHasEvalRep;
    private global::Kerf.Engine.Geometry.Splines.Cache? myCache;
    private NCollection_Array1<double>? myBezierFlatKnots;

    public TestGeomAdaptor_Curve()
    {
    }

    public TestGeomAdaptor_Curve(Curve3D theCurve)
    {
        Load(theCurve);
    }

    public TestGeomAdaptor_Curve(Curve3D theCurve, double theUFirst, double theULast)
    {
        Load(theCurve, theUFirst, theULast);
    }

    public void Load(Curve3D theCurve)
    {
        if (theCurve is null)
        {
            throw new global::Kerf.Engine.Foundation.NullObjectException();
        }
        load(theCurve, theCurve.FirstParameter(), theCurve.LastParameter());
    }

    public void Load(Curve3D theCurve, double theUFirst, double theULast)
    {
        if (theCurve is null)
        {
            throw new global::Kerf.Engine.Foundation.NullObjectException();
        }
        if (theUFirst > theULast + global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new global::Kerf.Engine.Foundation.ConstructionErrorException();
        }
        load(theCurve, theUFirst, theULast);
    }

    public Curve3D? Curve() => myCurve;

    private void load(Curve3D C, double UFirst, double ULast)
    {
        myFirst = UFirst;
        myLast = ULast;

        if (!ReferenceEquals(myCurve, C))
        {
            myCurve = C;
            myBezier = null;
            myBSpline = null;
            myCache = null;
            myHasEvalRep = false;

            switch (C)
            {
                case TrimmedCurve3D aTrimmed when C.GetType() == typeof(TrimmedCurve3D):
                    Load(aTrimmed.BasisCurve(), UFirst, ULast);
                    break;
                case CircleCurve3D aCircle when C.GetType() == typeof(CircleCurve3D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Circle;
                    myCirc = aCircle.Circ();
                    break;
                case LineCurve3D aLine when C.GetType() == typeof(LineCurve3D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Line;
                    myLin = aLine.Lin();
                    break;
                case EllipseCurve3D anEllipse when C.GetType() == typeof(EllipseCurve3D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Ellipse;
                    myElips = anEllipse.Elips();
                    break;
                case ParabolaCurve3D aParabola when C.GetType() == typeof(ParabolaCurve3D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Parabola;
                    myParab = aParabola.Parab();
                    break;
                case HyperbolaCurve3D aHyperbola when C.GetType() == typeof(HyperbolaCurve3D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Hyperbola;
                    myHypr = aHyperbola.Hypr();
                    break;
                case BezierCurve3D aBezier when C.GetType() == typeof(BezierCurve3D):
                    myTypeCurve = BezierCurve;
                    myBezier = aBezier;
                    myHasEvalRep = aBezier.HasEvalRepresentation;
                    break;
                case BSplineCurve3D aBSpline when C.GetType() == typeof(BSplineCurve3D):
                    myTypeCurve = BSplineCurve;
                    myBSpline = aBSpline;
                    myHasEvalRep = aBSpline.HasEvalRepresentation;
                    break;
                case OffsetCurve3D when C.GetType() == typeof(OffsetCurve3D):
                    // Test-only: evaluated through the curve itself (see the file header).
                    myTypeCurve = global::Kerf.Geometry.CurveKind.OffsetCurve;
                    myHasEvalRep = true;
                    break;
                default:
                    myTypeCurve = OtherCurve;
                    break;
            }
        }
        else
        {
            // Same curve, but need to invalidate cache if bounds changed
            myCache = null;
        }
    }

    public override CurveAdaptorBase ShallowCopy()
    {
        var aCopy = new TestGeomAdaptor_Curve();
        if (myCurve is not null)
        {
            aCopy.load(myCurve, myFirst, myLast);
        }
        return aCopy;
    }

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    private Continuity LocalContinuity(double U1, double U2)
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != BSplineCurve, " ");
        BSplineCurve3D aBSpl = myBSpline!;
        int Nb = aBSpl.KnotCount;
        int Index1;
        int Index2;
        NCollection_Array1<double> TK = aBSpl.InternalKnots();
        NCollection_Array1<int> TM = aBSpl.InternalMultiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aBSpl.Degree(), TK, TM, U1, aBSpl.IsPeriodic, 1, Nb, out Index1, out double newFirst);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aBSpl.Degree(), TK, TM, U2, aBSpl.IsPeriodic, 1, Nb, out Index2, out double newLast);
        if (Math.Abs(newFirst - TK[Index1 + 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (Index1 < Nb)
            {
                Index1++;
            }
        }
        if (Math.Abs(newLast - TK[Index2]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            Index2--;
        }
        int MultMax;
        // Handle periodic curves.
        if ((aBSpl.IsPeriodic) && (Index1 == Nb))
        {
            Index1 = 1;
        }

        if ((Index2 - Index1 <= 0) && (!aBSpl.IsPeriodic))
        {
            MultMax = 100; // CN between 2 consecutive nodes
        }
        else
        {
            MultMax = TM[Index1 + 1];
            for (int i = Index1 + 1; i <= Index2; i++)
            {
                if (TM[i] > MultMax)
                {
                    MultMax = TM[i];
                }
            }
            MultMax = aBSpl.Degree() - MultMax;
        }
        if (MultMax <= 0)
        {
            return C0;
        }
        else if (MultMax == 1)
        {
            return C1;
        }
        else if (MultMax == 2)
        {
            return C2;
        }
        else if (MultMax == 3)
        {
            return C3;
        }
        else
        {
            return CN;
        }
    }

    public override Continuity Continuity()
    {
        if (myTypeCurve == BSplineCurve)
        {
            return LocalContinuity(myFirst, myLast);
        }
        if (myTypeCurve == global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            return myCurve!.Continuity(); // test-only simplification
        }
        else if (myTypeCurve == OtherCurve)
        {
            throw new global::Kerf.Engine.Foundation.NoSuchObjectException("GeomAdaptor_Curve::Contunuity");
        }
        return CN;
    }

    private int BSplineContinuityOrder(Continuity S, string theWhere)
    {
        switch (S)
        {
            case C1:
                return 1;
            case C2:
                return 2;
            case C3:
                return 3;
            case CN:
                return myBSpline!.Degree();
            default:
                throw new global::Kerf.Engine.Foundation.DomainErrorException(theWhere);
        }
    }

    public override int NbIntervals(Continuity S)
    {
        if (myTypeCurve == BSplineCurve)
        {
            BSplineCurve3D aBSpl = myBSpline!;
            if ((!aBSpl.IsPeriodic && S <= Continuity()) || S == C0)
            {
                return 1;
            }
            int aDegree = aBSpl.Degree();
            int aCont = BSplineContinuityOrder(S, "GeomAdaptor_Curve::NbIntervals()");
            double anEps = Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(aBSpl.InternalKnots(), aBSpl.InternalMultiplicities(), aDegree, aBSpl.IsPeriodic,
                                      aCont, myFirst, myLast, anEps, null);
        }
        return 1;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        if (myTypeCurve == BSplineCurve)
        {
            BSplineCurve3D aBSpl = myBSpline!;
            if ((!aBSpl.IsPeriodic && S <= Continuity()) || S == C0)
            {
                T[T.Lower()] = myFirst;
                T[T.Lower() + 1] = myLast;
                return;
            }
            int aDegree = aBSpl.Degree();
            int aCont = BSplineContinuityOrder(S, "GeomAdaptor_Curve::Intervals()");
            double anEps = Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(aBSpl.InternalKnots(), aBSpl.InternalMultiplicities(), aDegree, aBSpl.IsPeriodic, aCont,
                               myFirst, myLast, anEps, T);
        }
        else
        {
            T[T.Lower()] = myFirst;
            T[T.Lower() + 1] = myLast;
        }
    }

    public override CurveAdaptorBase Trim(double First, double Last, double Tol) =>
        new TestGeomAdaptor_Curve(myCurve!, First, Last);

    public override bool IsClosed
    {
        get
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(myLast) && !global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(myFirst))
            {
                Point3D Pd = Value(myFirst);
                Point3D Pf = Value(myLast);
                return (Pd.Distance(Pf) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            return false;
        }
    }

    public override bool IsPeriodic => myCurve!.IsPeriodic;

    public override double Period() => myCurve!.LastParameter() - myCurve.FirstParameter();

    private void RebuildCache(double theParameter)
    {
        if (myTypeCurve == BezierCurve)
        {
            // Create cache for Bezier
            BezierCurve3D aBezier = myBezier!;
            int aDeg = aBezier.Degree();
            myBezierFlatKnots ??= NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(aDeg));
            myCache ??= new global::Kerf.Engine.Geometry.Splines.Cache(aDeg, aBezier.IsPeriodic, myBezierFlatKnots, aBezier.InternalPoles(),
                                           aBezier.InternalWeights());
            myCache.BuildCache(theParameter, myBezierFlatKnots, aBezier.InternalPoles(), aBezier.InternalWeights());
        }
        else if (myTypeCurve == BSplineCurve)
        {
            // Create cache for B-spline
            BSplineCurve3D aBSpl = myBSpline!;
            myCache ??= new global::Kerf.Engine.Geometry.Splines.Cache(aBSpl.Degree(), aBSpl.IsPeriodic, aBSpl.InternalKnotSequence(),
                                           aBSpl.InternalPoles(), aBSpl.InternalWeights());
            myCache.BuildCache(theParameter, aBSpl.InternalKnotSequence(), aBSpl.InternalPoles(), aBSpl.InternalWeights());
        }
    }

    private bool IsBoundary(double theU, out int theSpanStart, out int theSpanFinish)
    {
        theSpanStart = 0;
        theSpanFinish = 0;
        if (myBSpline is not null && (theU == myFirst || theU == myLast))
        {
            BSplineCurve3D aBSpl = myBSpline;
            if (theU == myFirst)
            {
                aBSpl.LocateU(myFirst, PosTol, out theSpanStart, out theSpanFinish);
                if (theSpanStart < 1)
                {
                    theSpanStart = 1;
                }
                if (theSpanStart >= theSpanFinish)
                {
                    theSpanFinish = theSpanStart + 1;
                }
            }
            else if (theU == myLast)
            {
                aBSpl.LocateU(myLast, PosTol, out theSpanStart, out theSpanFinish);
                if (theSpanFinish > aBSpl.KnotCount)
                {
                    theSpanFinish = aBSpl.KnotCount;
                }
                if (theSpanStart >= theSpanFinish)
                {
                    theSpanStart = theSpanFinish - 1;
                }
            }
            return true;
        }
        return false;
    }

    public override Point3D EvalD0(double theU)
    {
        double U = theU;
        Point3D P;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myLin);
            case global::Kerf.Geometry.CurveKind.Circle:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myCirc);
            case global::Kerf.Geometry.CurveKind.Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myElips);
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myHypr);
            case global::Kerf.Geometry.CurveKind.Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myParab);
            case BezierCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD0(U);
                }
                if (myCache is null)
                {
                    RebuildCache(U);
                }
                myCache!.D0(U, out P);
                return P;
            }
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD0(U);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    myBSpline!.LocalD0(U, aStart, aFinish, out P);
                }
                else
                {
                    if (myCache is null || !myCache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    myCache!.D0(U, out P);
                }
                return P;
            }
            default:
                return myCurve!.EvalD0(U);
        }
    }

    public override Curve3D.ResD1 EvalD1(double theU)
    {
        double U = theU;
        Curve3D.ResD1 aResult = new();
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myCirc, out aResult.Point, out aResult.D1);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myElips, out aResult.Point, out aResult.D1);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myHypr, out aResult.Point, out aResult.D1);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myParab, out aResult.Point, out aResult.D1);
                return aResult;
            case BezierCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD1(U);
                }
                if (myCache is null)
                {
                    RebuildCache(U);
                }
                myCache!.D1(U, out aResult.Point, out aResult.D1);
                return aResult;
            }
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD1(U);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    myBSpline!.LocalD1(U, aStart, aFinish, out aResult.Point, out aResult.D1);
                }
                else
                {
                    if (myCache is null || !myCache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    myCache!.D1(U, out aResult.Point, out aResult.D1);
                }
                return aResult;
            }
            default:
                return myCurve!.EvalD1(U);
        }
    }

    public override Curve3D.ResD2 EvalD2(double theU)
    {
        double U = theU;
        Curve3D.ResD2 aResult = new();
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                aResult.D2.SetCoord(0.0, 0.0, 0.0);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myCirc, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myElips, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myHypr, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myParab, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case BezierCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD2(U);
                }
                if (myCache is null)
                {
                    RebuildCache(U);
                }
                myCache!.D2(U, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            }
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD2(U);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    myBSpline!.LocalD2(U, aStart, aFinish, out aResult.Point, out aResult.D1, out aResult.D2);
                }
                else
                {
                    if (myCache is null || !myCache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    myCache!.D2(U, out aResult.Point, out aResult.D1, out aResult.D2);
                }
                return aResult;
            }
            default:
                return myCurve!.EvalD2(U);
        }
    }

    public override Curve3D.ResD3 EvalD3(double theU)
    {
        double U = theU;
        Curve3D.ResD3 aResult = new();
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                aResult.D2.SetCoord(0.0, 0.0, 0.0);
                aResult.D3.SetCoord(0.0, 0.0, 0.0);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, myCirc, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, myElips, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, myHypr, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myParab, out aResult.Point, out aResult.D1, out aResult.D2);
                aResult.D3.SetCoord(0.0, 0.0, 0.0);
                return aResult;
            case BezierCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD3(U);
                }
                if (myCache is null)
                {
                    RebuildCache(U);
                }
                myCache!.D3(U, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            }
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD3(U);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    myBSpline!.LocalD3(U, aStart, aFinish, out aResult.Point, out aResult.D1, out aResult.D2,
                                       out aResult.D3);
                }
                else
                {
                    if (myCache is null || !myCache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    myCache!.D3(U, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                }
                return aResult;
            }
            default:
                return myCurve!.EvalD3(U);
        }
    }

    public override Vector3D EvalDN(double theU, int theN)
    {
        double U = theU;
        int N = theN;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myLin, N);
            case global::Kerf.Geometry.CurveKind.Circle:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myCirc, N);
            case global::Kerf.Geometry.CurveKind.Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myElips, N);
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myHypr, N);
            case global::Kerf.Geometry.CurveKind.Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myParab, N);
            case BezierCurve:
                return myCurve!.EvalDN(U, N);
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalDN(U, N);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    return myBSpline!.LocalDN(U, aStart, aFinish, N);
                }
                return myCurve!.EvalDN(U, N);
            }
            default:
                return myCurve!.EvalDN(U, N);
        }
    }

    public override double Resolution(double R3D)
    {
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return R3D;
            case global::Kerf.Geometry.CurveKind.Circle:
            {
                double R = myCirc.Radius();
                if (R > R3D / 2.0)
                {
                    return 2 * Math.Asin(R3D / (2 * R));
                }
                return 2 * Math.PI;
            }
            case global::Kerf.Geometry.CurveKind.Ellipse:
                return R3D / myElips.MajorRadius();
            case BezierCurve:
            {
                myBezier!.Resolution(R3D, out double res);
                return res;
            }
            case BSplineCurve:
            {
                myBSpline!.Resolution(R3D, out double res);
                return res;
            }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(R3D);
        }
    }

    public override CurveKind GetType() => myTypeCurve;

    public override Line3D Line()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Line, "GeomAdaptor_Curve::Line() - curve is not a Line");
        return myLin;
    }

    public override Circle3D Circle()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Circle, "GeomAdaptor_Curve::Circle() - curve is not a Circle");
        return myCirc;
    }

    public override Ellipse3D Ellipse()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Ellipse,
                                       "GeomAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return myElips;
    }

    public override Hyperbola3D Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Hyperbola,
                                       "GeomAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return myHypr;
    }

    public override Parabola3D Parabola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Parabola,
                                       "GeomAdaptor_Curve::Parabola() - curve is not a Parabola");
        return myParab;
    }

    public override int Degree()
    {
        if (myTypeCurve == BezierCurve)
        {
            return myBezier!.Degree();
        }
        else if (myTypeCurve == BSplineCurve)
        {
            return myBSpline!.Degree();
        }
        throw new global::Kerf.Engine.Foundation.NoSuchObjectException();
    }

    public override bool IsRational
    {
        get
        {
            switch (myTypeCurve)
            {
                case BSplineCurve:
                    return myBSpline!.IsRational;
                case BezierCurve:
                    return myBezier!.IsRational;
                default:
                    return false;
            }
        }
    }

    public override int PoleCount
    {
        get
        {
            if (myTypeCurve == BezierCurve)
            {
                return myBezier!.PoleCount;
            }
            else if (myTypeCurve == BSplineCurve)
            {
                return myBSpline!.PoleCount;
            }
            throw new global::Kerf.Engine.Foundation.NoSuchObjectException();
        }
    }

    public override int KnotCount
    {
        get
        {
            if (myTypeCurve != BSplineCurve)
            {
                throw new global::Kerf.Engine.Foundation.NoSuchObjectException("GeomAdaptor_Curve::NbKnots");
            }
            return myBSpline!.KnotCount;
        }
    }

    public override BezierCurve3D Bezier()
    {
        if (myTypeCurve != BezierCurve)
        {
            throw new global::Kerf.Engine.Foundation.NoSuchObjectException("GeomAdaptor_Curve::Bezier");
        }
        return myBezier!;
    }

    public override BSplineCurve3D BSpline()
    {
        if (myTypeCurve != BSplineCurve)
        {
            throw new global::Kerf.Engine.Foundation.NoSuchObjectException("GeomAdaptor_Curve::BSpline");
        }
        return myBSpline!;
    }

    public override OffsetCurve3D OffsetCurve()
    {
        if (myTypeCurve != global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            throw new global::Kerf.Engine.Foundation.NoSuchObjectException("GeomAdaptor_Curve::OffsetCurve");
        }
        return (OffsetCurve3D)myCurve!;
    }
}

/// <summary>A test stand-in for Geom2dAdaptor_Curve: a Geom2d_Curve seen through Adaptor2d_Curve2d (see the file header).</summary>
internal sealed class TestGeom2dAdaptor_Curve : Curve2DAdaptorBase
{
    private static readonly double PosTol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 2;

    private Curve2D? myCurve;
    private CurveKind myTypeCurve = OtherCurve;
    private double myFirst;
    private double myLast;

    private Line2D myLin = new();
    private Circle2D myCirc = new();
    private Ellipse2D myElips = new();
    private Hyperbola2D myHypr = new();
    private Parabola2D myParab = new();
    private BezierCurve2D? myBezier;
    private BSplineCurve2D? myBSpline;
    private bool myHasEvalRep;
    private global::Kerf.Engine.Geometry.Splines.Cache? myCache;
    private NCollection_Array1<double>? myBezierFlatKnots;

    public TestGeom2dAdaptor_Curve()
    {
    }

    public TestGeom2dAdaptor_Curve(Curve2D theCrv)
    {
        Load(theCrv);
    }

    public TestGeom2dAdaptor_Curve(Curve2D theCrv, double theUFirst, double theULast)
    {
        myFirst = theUFirst;
        myLast = theULast;
        Load(theCrv, theUFirst, theULast);
    }

    public void Load(Curve2D theCurve)
    {
        if (theCurve is null)
        {
            throw new global::Kerf.Engine.Foundation.NullObjectException();
        }
        load(theCurve, theCurve.FirstParameter(), theCurve.LastParameter());
    }

    public void Load(Curve2D theCurve, double theUFirst, double theULast)
    {
        if (theCurve is null)
        {
            throw new global::Kerf.Engine.Foundation.NullObjectException();
        }
        if (theUFirst > theULast + global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new global::Kerf.Engine.Foundation.ConstructionErrorException();
        }
        load(theCurve, theUFirst, theULast);
    }

    public Curve2D? Curve() => myCurve;

    private void load(Curve2D C, double UFirst, double ULast)
    {
        myFirst = UFirst;
        myLast = ULast;

        if (!ReferenceEquals(myCurve, C))
        {
            myCurve = C;
            myBezier = null;
            myBSpline = null;
            myCache = null;
            myHasEvalRep = false;

            switch (C)
            {
                case TrimmedCurve2D aTrimmed when C.GetType() == typeof(TrimmedCurve2D):
                    Load(aTrimmed.BasisCurve(), UFirst, ULast);
                    break;
                case CircleCurve2D aCircle when C.GetType() == typeof(CircleCurve2D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Circle;
                    myCirc = aCircle.Circ2d();
                    break;
                case LineCurve2D aLine when C.GetType() == typeof(LineCurve2D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Line;
                    myLin = aLine.Lin2d();
                    break;
                case EllipseCurve2D anEllipse when C.GetType() == typeof(EllipseCurve2D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Ellipse;
                    myElips = anEllipse.Elips2d();
                    break;
                case ParabolaCurve2D aParabola when C.GetType() == typeof(ParabolaCurve2D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Parabola;
                    myParab = aParabola.Parab2d();
                    break;
                case HyperbolaCurve2D aHyperbola when C.GetType() == typeof(HyperbolaCurve2D):
                    myTypeCurve = global::Kerf.Geometry.CurveKind.Hyperbola;
                    myHypr = aHyperbola.Hypr2d();
                    break;
                case BezierCurve2D aBezier when C.GetType() == typeof(BezierCurve2D):
                    myTypeCurve = BezierCurve;
                    myBezier = aBezier;
                    myHasEvalRep = aBezier.HasEvalRepresentation;
                    break;
                case BSplineCurve2D aBSpline when C.GetType() == typeof(BSplineCurve2D):
                    myTypeCurve = BSplineCurve;
                    myBSpline = aBSpline;
                    myHasEvalRep = aBSpline.HasEvalRepresentation;
                    break;
                case OffsetCurve2D when C.GetType() == typeof(OffsetCurve2D):
                    // Test-only: evaluated through the curve itself (see the file header).
                    myTypeCurve = OffsetCurve;
                    myHasEvalRep = true;
                    break;
                default:
                    myTypeCurve = OtherCurve;
                    break;
            }
        }
        else
        {
            // Same curve but potentially different parameters - invalidate cache
            myCache = null;
        }
    }

    public override Curve2DAdaptorBase ShallowCopy()
    {
        var aCopy = new TestGeom2dAdaptor_Curve();
        if (myCurve is not null)
        {
            aCopy.load(myCurve, myFirst, myLast);
        }
        return aCopy;
    }

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    private Continuity LocalContinuity(double U1, double U2)
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != BSplineCurve, " ");
        BSplineCurve2D aBSpline = myBSpline!;
        int Nb = aBSpline.KnotCount;
        int Index1;
        int Index2;
        NCollection_Array1<double> TK = aBSpline.InternalKnots();
        NCollection_Array1<int> TM = aBSpline.InternalMultiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aBSpline.Degree(), TK, TM, U1, aBSpline.IsPeriodic, 1, Nb, out Index1,
                                 out double newFirst);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aBSpline.Degree(), TK, TM, U2, aBSpline.IsPeriodic, 1, Nb, out Index2,
                                 out double newLast);
        if (Math.Abs(newFirst - TK[Index1 + 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (Index1 < Nb)
            {
                Index1++;
            }
        }
        if (Math.Abs(newLast - TK[Index2]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            Index2--;
        }
        int MultMax;
        // beware of periodic curves.
        if (aBSpline.IsPeriodic && Index1 == Nb)
        {
            Index1 = 1;
        }

        if ((Index2 - Index1 <= 0) && (!aBSpline.IsPeriodic))
        {
            MultMax = 100; // CN between 2 consecutive nodes
        }
        else
        {
            MultMax = TM[Index1 + 1];
            for (int i = Index1 + 1; i <= Index2; i++)
            {
                if (TM[i] > MultMax)
                {
                    MultMax = TM[i];
                }
            }
            MultMax = aBSpline.Degree() - MultMax;
        }
        if (MultMax <= 0)
        {
            return C0;
        }
        else if (MultMax == 1)
        {
            return C1;
        }
        else if (MultMax == 2)
        {
            return C2;
        }
        else if (MultMax == 3)
        {
            return C3;
        }
        else
        {
            return CN;
        }
    }

    public override Continuity Continuity()
    {
        if (myTypeCurve == BSplineCurve)
        {
            return LocalContinuity(myFirst, myLast);
        }
        else if (myTypeCurve == OffsetCurve)
        {
            return myCurve!.Continuity(); // test-only simplification
        }
        else if (myTypeCurve == OtherCurve)
        {
            throw new global::Kerf.Engine.Foundation.NoSuchObjectException("Geom2dAdaptor_Curve::Continuity");
        }
        else
        {
            return CN;
        }
    }

    private int BSplineContinuityOrder(Continuity S, string theWhere)
    {
        switch (S)
        {
            case C1:
                return 1;
            case C2:
                return 2;
            case C3:
                return 3;
            case CN:
                return myBSpline!.Degree();
            default:
                throw new global::Kerf.Engine.Foundation.DomainErrorException(theWhere);
        }
    }

    public override int NbIntervals(Continuity S)
    {
        if (myTypeCurve == BSplineCurve)
        {
            BSplineCurve2D aBSpline = myBSpline!;
            if ((!aBSpline.IsPeriodic && S <= Continuity()) || S == C0)
            {
                return 1;
            }
            int aDegree = aBSpline.Degree();
            int aCont = BSplineContinuityOrder(S, "Geom2dAdaptor_Curve::NbIntervals()");
            double anEps = Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(aBSpline.InternalKnots(), aBSpline.InternalMultiplicities(), aDegree,
                                      aBSpline.IsPeriodic, aCont, myFirst, myLast, anEps, null);
        }
        return 1;
    }

    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        if (myTypeCurve == BSplineCurve)
        {
            BSplineCurve2D aBSpline = myBSpline!;
            if ((!aBSpline.IsPeriodic && S <= Continuity()) || S == C0)
            {
                T[T.Lower()] = myFirst;
                T[T.Lower() + 1] = myLast;
                return;
            }
            int aDegree = aBSpline.Degree();
            int aCont = BSplineContinuityOrder(S, "Geom2dAdaptor_Curve::Intervals()");
            double anEps = Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(aBSpline.InternalKnots(), aBSpline.InternalMultiplicities(), aDegree, aBSpline.IsPeriodic,
                               aCont, myFirst, myLast, anEps, T);
        }
        else
        {
            T[T.Lower()] = myFirst;
            T[T.Lower() + 1] = myLast;
        }
    }

    public override Curve2DAdaptorBase Trim(double First, double Last, double Tol) =>
        new TestGeom2dAdaptor_Curve(myCurve!, First, Last);

    public override bool IsClosed
    {
        get
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(myLast) && !global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(myFirst))
            {
                Point2D Pd = Value(myFirst);
                Point2D Pf = Value(myLast);
                return (Pd.Distance(Pf) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            return false;
        }
    }

    public override bool IsPeriodic => myCurve!.IsPeriodic;

    public override double Period() => myCurve!.LastParameter() - myCurve.FirstParameter();

    private void RebuildCache(double theParameter)
    {
        if (myTypeCurve == BezierCurve)
        {
            // Create cache for Bezier
            BezierCurve2D aBezier = myBezier!;
            int aDeg = aBezier.Degree();
            myBezierFlatKnots ??= NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(aDeg));
            myCache ??= new global::Kerf.Engine.Geometry.Splines.Cache(aDeg, aBezier.IsPeriodic, myBezierFlatKnots, aBezier.InternalPoles(),
                                           aBezier.InternalWeights());
            myCache.BuildCache(theParameter, myBezierFlatKnots, aBezier.InternalPoles(), aBezier.InternalWeights());
        }
        else if (myTypeCurve == BSplineCurve)
        {
            // Create cache for B-spline
            BSplineCurve2D aBSpline = myBSpline!;
            myCache ??= new global::Kerf.Engine.Geometry.Splines.Cache(aBSpline.Degree(), aBSpline.IsPeriodic, aBSpline.InternalKnotSequence(),
                                           aBSpline.InternalPoles(), aBSpline.InternalWeights());
            myCache.BuildCache(theParameter, aBSpline.InternalKnotSequence(), aBSpline.InternalPoles(),
                               aBSpline.InternalWeights());
        }
    }

    private bool IsBoundary(double theU, out int theSpanStart, out int theSpanFinish)
    {
        theSpanStart = 0;
        theSpanFinish = 0;
        if (myBSpline is not null && (theU == myFirst || theU == myLast))
        {
            BSplineCurve2D aBSpline = myBSpline;
            if (theU == myFirst)
            {
                aBSpline.LocateU(myFirst, PosTol, out theSpanStart, out theSpanFinish);
                if (theSpanStart < 1)
                {
                    theSpanStart = 1;
                }
                if (theSpanStart >= theSpanFinish)
                {
                    theSpanFinish = theSpanStart + 1;
                }
            }
            else if (theU == myLast)
            {
                aBSpline.LocateU(myLast, PosTol, out theSpanStart, out theSpanFinish);
                if (theSpanFinish > aBSpline.KnotCount)
                {
                    theSpanFinish = aBSpline.KnotCount;
                }
                if (theSpanStart >= theSpanFinish)
                {
                    theSpanStart = theSpanFinish - 1;
                }
            }
            return true;
        }
        return false;
    }

    public override Point2D Value(double U)
    {
        D0(U, out Point2D aRes);
        return aRes;
    }

    public override void D0(double U, out Point2D P) => P = EvalD0(U);

    public override void D1(double U, out Point2D P, out Vector2D V)
    {
        Curve2D.ResD1 aRes = EvalD1(U);
        P = aRes.Point;
        V = aRes.D1;
    }

    public override void D2(double U, out Point2D P, out Vector2D V1, out Vector2D V2)
    {
        Curve2D.ResD2 aRes = EvalD2(U);
        P = aRes.Point;
        V1 = aRes.D1;
        V2 = aRes.D2;
    }

    public override void D3(double U, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3)
    {
        Curve2D.ResD3 aRes = EvalD3(U);
        P = aRes.Point;
        V1 = aRes.D1;
        V2 = aRes.D2;
        V3 = aRes.D3;
    }

    public override Vector2D DN(double U, int N) => EvalDN(U, N);

    public override Point2D EvalD0(double theU)
    {
        double U = theU;
        Point2D P;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myLin);
            case global::Kerf.Geometry.CurveKind.Circle:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myCirc);
            case global::Kerf.Geometry.CurveKind.Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myElips);
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myHypr);
            case global::Kerf.Geometry.CurveKind.Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, myParab);
            case BezierCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD0(U);
                }
                if (myCache is null)
                {
                    RebuildCache(U);
                }
                myCache!.D0(U, out P);
                return P;
            }
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD0(U);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    myBSpline!.LocalD0(U, aStart, aFinish, out P);
                }
                else
                {
                    if (myCache is null || !myCache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    myCache!.D0(U, out P);
                }
                return P;
            }
            default:
                return myCurve!.EvalD0(U);
        }
    }

    public override Curve2D.ResD1 EvalD1(double theU)
    {
        double U = theU;
        Curve2D.ResD1 aResult = new();
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myCirc, out aResult.Point, out aResult.D1);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myElips, out aResult.Point, out aResult.D1);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myHypr, out aResult.Point, out aResult.D1);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myParab, out aResult.Point, out aResult.D1);
                return aResult;
            case BezierCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD1(U);
                }
                if (myCache is null)
                {
                    RebuildCache(U);
                }
                myCache!.D1(U, out aResult.Point, out aResult.D1);
                return aResult;
            }
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD1(U);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    myBSpline!.LocalD1(U, aStart, aFinish, out aResult.Point, out aResult.D1);
                }
                else
                {
                    if (myCache is null || !myCache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    myCache!.D1(U, out aResult.Point, out aResult.D1);
                }
                return aResult;
            }
            default:
                return myCurve!.EvalD1(U);
        }
    }

    public override Curve2D.ResD2 EvalD2(double theU)
    {
        double U = theU;
        Curve2D.ResD2 aResult = new();
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                aResult.D2.SetCoord(0.0, 0.0);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myCirc, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myElips, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myHypr, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myParab, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case BezierCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD2(U);
                }
                if (myCache is null)
                {
                    RebuildCache(U);
                }
                myCache!.D2(U, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            }
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD2(U);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    myBSpline!.LocalD2(U, aStart, aFinish, out aResult.Point, out aResult.D1, out aResult.D2);
                }
                else
                {
                    if (myCache is null || !myCache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    myCache!.D2(U, out aResult.Point, out aResult.D1, out aResult.D2);
                }
                return aResult;
            }
            default:
                return myCurve!.EvalD2(U);
        }
    }

    public override Curve2D.ResD3 EvalD3(double theU)
    {
        double U = theU;
        Curve2D.ResD3 aResult = new();
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                aResult.D2.SetCoord(0.0, 0.0);
                aResult.D3.SetCoord(0.0, 0.0);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, myCirc, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, myElips, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, myHypr, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, myParab, out aResult.Point, out aResult.D1, out aResult.D2);
                aResult.D3.SetCoord(0.0, 0.0);
                return aResult;
            case BezierCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD3(U);
                }
                if (myCache is null)
                {
                    RebuildCache(U);
                }
                myCache!.D3(U, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            }
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalD3(U);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    myBSpline!.LocalD3(U, aStart, aFinish, out aResult.Point, out aResult.D1, out aResult.D2,
                                       out aResult.D3);
                }
                else
                {
                    if (myCache is null || !myCache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    myCache!.D3(U, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                }
                return aResult;
            }
            default:
                return myCurve!.EvalD3(U);
        }
    }

    public override Vector2D EvalDN(double theU, int theN)
    {
        double U = theU;
        int N = theN;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myLin, N);
            case global::Kerf.Geometry.CurveKind.Circle:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myCirc, N);
            case global::Kerf.Geometry.CurveKind.Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myElips, N);
            case global::Kerf.Geometry.CurveKind.Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myHypr, N);
            case global::Kerf.Geometry.CurveKind.Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, myParab, N);
            case BezierCurve:
                return myCurve!.EvalDN(U, N);
            case BSplineCurve:
            {
                if (myHasEvalRep)
                {
                    return myCurve!.EvalDN(U, N);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    return myBSpline!.LocalDN(U, aStart, aFinish, N);
                }
                return myCurve!.EvalDN(U, N);
            }
            default:
                return myCurve!.EvalDN(U, N);
        }
    }

    public override double Resolution(double Ruv)
    {
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return Ruv;
            case global::Kerf.Geometry.CurveKind.Circle:
            {
                double R = myCirc.Radius();
                if (R > Ruv / 2.0)
                {
                    return 2 * Math.Asin(Ruv / (2 * R));
                }
                return 2 * Math.PI;
            }
            case global::Kerf.Geometry.CurveKind.Ellipse:
                return Ruv / myElips.MajorRadius();
            case BezierCurve:
            {
                myBezier!.Resolution(Ruv, out double res);
                return res;
            }
            case BSplineCurve:
            {
                myBSpline!.Resolution(Ruv, out double res);
                return res;
            }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(Ruv);
        }
    }

    public override CurveKind GetType() => myTypeCurve;

    public override Line2D Line()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Line, "Geom2dAdaptor_Curve::Line() - curve is not a Line");
        return myLin;
    }

    public override Circle2D Circle()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Circle,
                                       "Geom2dAdaptor_Curve::Circle() - curve is not a Circle");
        return myCirc;
    }

    public override Ellipse2D Ellipse()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Ellipse,
                                       "Geom2dAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return myElips;
    }

    public override Hyperbola2D Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Hyperbola,
                                       "Geom2dAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return myHypr;
    }

    public override Parabola2D Parabola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Parabola,
                                       "Geom2dAdaptor_Curve::Parabola() - curve is not a Parabola");
        return myParab;
    }

    public override int Degree()
    {
        if (myTypeCurve == BezierCurve)
        {
            return myBezier!.Degree();
        }
        else if (myTypeCurve == BSplineCurve)
        {
            return myBSpline!.Degree();
        }
        throw new global::Kerf.Engine.Foundation.NoSuchObjectException();
    }

    public override bool IsRational
    {
        get
        {
            switch (myTypeCurve)
            {
                case BSplineCurve:
                    return myBSpline!.IsRational;
                case BezierCurve:
                    return myBezier!.IsRational;
                default:
                    return false;
            }
        }
    }

    public override int PoleCount
    {
        get
        {
            if (myTypeCurve == BezierCurve)
            {
                return myBezier!.PoleCount;
            }
            else if (myTypeCurve == BSplineCurve)
            {
                return myBSpline!.PoleCount;
            }
            throw new global::Kerf.Engine.Foundation.NoSuchObjectException();
        }
    }

    public override int KnotCount
    {
        get
        {
            if (myTypeCurve != BSplineCurve)
            {
                throw new global::Kerf.Engine.Foundation.NoSuchObjectException("Geom2dAdaptor_Curve::NbKnots");
            }
            return myBSpline!.KnotCount;
        }
    }

    // Geom2dAdaptor_Curve::Bezier() and BSpline() return a null handle (they don't throw) for other curves.
    public override BezierCurve2D Bezier() => (myCurve as BezierCurve2D)!;

    public override BSplineCurve2D BSpline() => myBSpline!;

    private static int nbPoints(Curve2D theCurve)
    {
        int nbs = 20;
        if (theCurve is LineCurve2D)
        {
            nbs = 2;
        }
        else if (theCurve is BezierCurve2D aBezier)
        {
            nbs = 3 + aBezier.PoleCount;
        }
        else if (theCurve is BSplineCurve2D aBSpline)
        {
            nbs = aBSpline.KnotCount;
            nbs *= aBSpline.Degree();
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        else if (theCurve is OffsetCurve2D anOffset)
        {
            return Max(nbs, nbPoints(anOffset.BasisCurve()));
        }
        else if (theCurve is TrimmedCurve2D aTrimmed)
        {
            return Max(nbs, nbPoints(aTrimmed.BasisCurve()));
        }
        if (nbs > 300)
        {
            nbs = 300;
        }
        return nbs;
    }

    public override int SampleCount => nbPoints(myCurve!);
}
