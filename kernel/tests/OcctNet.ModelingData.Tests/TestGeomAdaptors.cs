// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Test-only stand-ins for GeomAdaptor_Curve (unit 2A) and Geom2dAdaptor_Curve (unit 2B), written after OCCT V8_0_1's
// src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Curve.cxx and src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor_Curve.cxx
// so that algorithms over adaptors (GCPnts, CPnts, ...) compute what native OCCT computes. This file is compiled into
// both OcctNet.ModelingData.Tests and OcctNet.Oracle.Tests. Replace with the real adaptors once they are merged.
//
// Mirrored: trimmed-curve unwrapping (the adaptor keeps the basis curve), lines and conics evaluated with ElCLib,
// Bezier curves through a BSplCLib_Cache over the flat Bezier knots, B-splines through a BSplCLib_Cache with
// LocalD0..LocalDN at exactly First/Last (IsBoundary), LocalContinuity, Continuity, NbIntervals/Intervals
// (BSplCLib.Intervals), Resolution, the type accessors, Degree, IsRational, NbPoles, NbKnots, IsPeriodic, Period,
// IsClosed, Trim, ShallowCopy, NbSamples (2D).
// Simplified (test only): offset curves and curves carrying an evaluation representation are evaluated through the
// Geom curve's own EvalD*, and offset curves report a single interval and the curve's own continuity (OCCT evaluates
// them through Geom_OffsetCurveUtils over a basis adaptor); ShallowCopy shares nothing but the curve and bounds.

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.Standard;

namespace OcctNet.ModelingData.Tests;

/// <summary>A test stand-in for GeomAdaptor_Curve: a Geom_Curve seen through Adaptor3d_Curve (see the file header).</summary>
internal sealed class TestGeomAdaptor_Curve : Adaptor3d_Curve
{
    private static readonly double PosTol = Precision.PConfusion() / 2;

    private Geom_Curve? myCurve;
    private GeomAbs_CurveType myTypeCurve = GeomAbs_OtherCurve;
    private double myFirst;
    private double myLast;

    private gp_Lin myLin = new();
    private gp_Circ myCirc = new();
    private gp_Elips myElips = new();
    private gp_Hypr myHypr = new();
    private gp_Parab myParab = new();
    private Geom_BezierCurve? myBezier;
    private Geom_BSplineCurve? myBSpline;
    private bool myHasEvalRep;
    private BSplCLib_Cache? myCache;
    private NCollection_Array1<double>? myBezierFlatKnots;

    public TestGeomAdaptor_Curve()
    {
    }

    public TestGeomAdaptor_Curve(Geom_Curve theCurve)
    {
        Load(theCurve);
    }

    public TestGeomAdaptor_Curve(Geom_Curve theCurve, double theUFirst, double theULast)
    {
        Load(theCurve, theUFirst, theULast);
    }

    public void Load(Geom_Curve theCurve)
    {
        if (theCurve is null)
        {
            throw new Standard_NullObject();
        }
        load(theCurve, theCurve.FirstParameter(), theCurve.LastParameter());
    }

    public void Load(Geom_Curve theCurve, double theUFirst, double theULast)
    {
        if (theCurve is null)
        {
            throw new Standard_NullObject();
        }
        if (theUFirst > theULast + Precision.Confusion())
        {
            throw new Standard_ConstructionError();
        }
        load(theCurve, theUFirst, theULast);
    }

    public Geom_Curve? Curve() => myCurve;

    private void load(Geom_Curve C, double UFirst, double ULast)
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
                case Geom_TrimmedCurve aTrimmed when C.GetType() == typeof(Geom_TrimmedCurve):
                    Load(aTrimmed.BasisCurve(), UFirst, ULast);
                    break;
                case Geom_Circle aCircle when C.GetType() == typeof(Geom_Circle):
                    myTypeCurve = GeomAbs_Circle;
                    myCirc = aCircle.Circ();
                    break;
                case Geom_Line aLine when C.GetType() == typeof(Geom_Line):
                    myTypeCurve = GeomAbs_Line;
                    myLin = aLine.Lin();
                    break;
                case Geom_Ellipse anEllipse when C.GetType() == typeof(Geom_Ellipse):
                    myTypeCurve = GeomAbs_Ellipse;
                    myElips = anEllipse.Elips();
                    break;
                case Geom_Parabola aParabola when C.GetType() == typeof(Geom_Parabola):
                    myTypeCurve = GeomAbs_Parabola;
                    myParab = aParabola.Parab();
                    break;
                case Geom_Hyperbola aHyperbola when C.GetType() == typeof(Geom_Hyperbola):
                    myTypeCurve = GeomAbs_Hyperbola;
                    myHypr = aHyperbola.Hypr();
                    break;
                case Geom_BezierCurve aBezier when C.GetType() == typeof(Geom_BezierCurve):
                    myTypeCurve = GeomAbs_BezierCurve;
                    myBezier = aBezier;
                    myHasEvalRep = aBezier.HasEvalRepresentation();
                    break;
                case Geom_BSplineCurve aBSpline when C.GetType() == typeof(Geom_BSplineCurve):
                    myTypeCurve = GeomAbs_BSplineCurve;
                    myBSpline = aBSpline;
                    myHasEvalRep = aBSpline.HasEvalRepresentation();
                    break;
                case Geom_OffsetCurve when C.GetType() == typeof(Geom_OffsetCurve):
                    // Test-only: evaluated through the curve itself (see the file header).
                    myTypeCurve = GeomAbs_OffsetCurve;
                    myHasEvalRep = true;
                    break;
                default:
                    myTypeCurve = GeomAbs_OtherCurve;
                    break;
            }
        }
        else
        {
            // Same curve, but need to invalidate cache if bounds changed
            myCache = null;
        }
    }

    public override Adaptor3d_Curve ShallowCopy()
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

    private GeomAbs_Shape LocalContinuity(double U1, double U2)
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_BSplineCurve, " ");
        Geom_BSplineCurve aBSpl = myBSpline!;
        int Nb = aBSpl.NbKnots();
        int Index1;
        int Index2;
        NCollection_Array1<double> TK = aBSpl.InternalKnots();
        NCollection_Array1<int> TM = aBSpl.InternalMultiplicities();
        BSplCLib.LocateParameter(aBSpl.Degree(), TK, TM, U1, aBSpl.IsPeriodic(), 1, Nb, out Index1, out double newFirst);
        BSplCLib.LocateParameter(aBSpl.Degree(), TK, TM, U2, aBSpl.IsPeriodic(), 1, Nb, out Index2, out double newLast);
        if (Math.Abs(newFirst - TK[Index1 + 1]) < Precision.PConfusion())
        {
            if (Index1 < Nb)
            {
                Index1++;
            }
        }
        if (Math.Abs(newLast - TK[Index2]) < Precision.PConfusion())
        {
            Index2--;
        }
        int MultMax;
        // Handle periodic curves.
        if ((aBSpl.IsPeriodic()) && (Index1 == Nb))
        {
            Index1 = 1;
        }

        if ((Index2 - Index1 <= 0) && (!aBSpl.IsPeriodic()))
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
            return GeomAbs_C0;
        }
        else if (MultMax == 1)
        {
            return GeomAbs_C1;
        }
        else if (MultMax == 2)
        {
            return GeomAbs_C2;
        }
        else if (MultMax == 3)
        {
            return GeomAbs_C3;
        }
        else
        {
            return GeomAbs_CN;
        }
    }

    public override GeomAbs_Shape Continuity()
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return LocalContinuity(myFirst, myLast);
        }
        if (myTypeCurve == GeomAbs_OffsetCurve)
        {
            return myCurve!.Continuity(); // test-only simplification
        }
        else if (myTypeCurve == GeomAbs_OtherCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::Contunuity");
        }
        return GeomAbs_CN;
    }

    private int BSplineContinuityOrder(GeomAbs_Shape S, string theWhere)
    {
        switch (S)
        {
            case GeomAbs_C1:
                return 1;
            case GeomAbs_C2:
                return 2;
            case GeomAbs_C3:
                return 3;
            case GeomAbs_CN:
                return myBSpline!.Degree();
            default:
                throw new Standard_DomainError(theWhere);
        }
    }

    public override int NbIntervals(GeomAbs_Shape S)
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = myBSpline!;
            if ((!aBSpl.IsPeriodic() && S <= Continuity()) || S == GeomAbs_C0)
            {
                return 1;
            }
            int aDegree = aBSpl.Degree();
            int aCont = BSplineContinuityOrder(S, "GeomAdaptor_Curve::NbIntervals()");
            double anEps = Min(Resolution(Precision.Confusion()), Precision.PConfusion());
            return BSplCLib.Intervals(aBSpl.InternalKnots(), aBSpl.InternalMultiplicities(), aDegree, aBSpl.IsPeriodic(),
                                      aCont, myFirst, myLast, anEps, null);
        }
        return 1;
    }

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = myBSpline!;
            if ((!aBSpl.IsPeriodic() && S <= Continuity()) || S == GeomAbs_C0)
            {
                T[T.Lower()] = myFirst;
                T[T.Lower() + 1] = myLast;
                return;
            }
            int aDegree = aBSpl.Degree();
            int aCont = BSplineContinuityOrder(S, "GeomAdaptor_Curve::Intervals()");
            double anEps = Min(Resolution(Precision.Confusion()), Precision.PConfusion());
            BSplCLib.Intervals(aBSpl.InternalKnots(), aBSpl.InternalMultiplicities(), aDegree, aBSpl.IsPeriodic(), aCont,
                               myFirst, myLast, anEps, T);
        }
        else
        {
            T[T.Lower()] = myFirst;
            T[T.Lower() + 1] = myLast;
        }
    }

    public override Adaptor3d_Curve Trim(double First, double Last, double Tol) =>
        new TestGeomAdaptor_Curve(myCurve!, First, Last);

    public override bool IsClosed()
    {
        if (!Precision.IsPositiveInfinite(myLast) && !Precision.IsNegativeInfinite(myFirst))
        {
            gp_Pnt Pd = Value(myFirst);
            gp_Pnt Pf = Value(myLast);
            return (Pd.Distance(Pf) <= Precision.Confusion());
        }
        return false;
    }

    public override bool IsPeriodic() => myCurve!.IsPeriodic();

    public override double Period() => myCurve!.LastParameter() - myCurve.FirstParameter();

    private void RebuildCache(double theParameter)
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            // Create cache for Bezier
            Geom_BezierCurve aBezier = myBezier!;
            int aDeg = aBezier.Degree();
            myBezierFlatKnots ??= NCollection_Array1<double>.From(BSplCLib.FlatBezierKnots(aDeg));
            myCache ??= new BSplCLib_Cache(aDeg, aBezier.IsPeriodic(), myBezierFlatKnots, aBezier.InternalPoles(),
                                           aBezier.InternalWeights());
            myCache.BuildCache(theParameter, myBezierFlatKnots, aBezier.InternalPoles(), aBezier.InternalWeights());
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            // Create cache for B-spline
            Geom_BSplineCurve aBSpl = myBSpline!;
            myCache ??= new BSplCLib_Cache(aBSpl.Degree(), aBSpl.IsPeriodic(), aBSpl.InternalKnotSequence(),
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
            Geom_BSplineCurve aBSpl = myBSpline;
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
                if (theSpanFinish > aBSpl.NbKnots())
                {
                    theSpanFinish = aBSpl.NbKnots();
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

    public override gp_Pnt EvalD0(double theU)
    {
        double U = theU;
        gp_Pnt P;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                return ElCLib.Value(U, myLin);
            case GeomAbs_Circle:
                return ElCLib.Value(U, myCirc);
            case GeomAbs_Ellipse:
                return ElCLib.Value(U, myElips);
            case GeomAbs_Hyperbola:
                return ElCLib.Value(U, myHypr);
            case GeomAbs_Parabola:
                return ElCLib.Value(U, myParab);
            case GeomAbs_BezierCurve:
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
            case GeomAbs_BSplineCurve:
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

    public override Geom_Curve.ResD1 EvalD1(double theU)
    {
        double U = theU;
        Geom_Curve.ResD1 aResult = new();
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_Circle:
                ElCLib.D1(U, myCirc, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_Ellipse:
                ElCLib.D1(U, myElips, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_Hyperbola:
                ElCLib.D1(U, myHypr, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_Parabola:
                ElCLib.D1(U, myParab, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_BezierCurve:
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
            case GeomAbs_BSplineCurve:
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

    public override Geom_Curve.ResD2 EvalD2(double theU)
    {
        double U = theU;
        Geom_Curve.ResD2 aResult = new();
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                aResult.D2.SetCoord(0.0, 0.0, 0.0);
                return aResult;
            case GeomAbs_Circle:
                ElCLib.D2(U, myCirc, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case GeomAbs_Ellipse:
                ElCLib.D2(U, myElips, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case GeomAbs_Hyperbola:
                ElCLib.D2(U, myHypr, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case GeomAbs_Parabola:
                ElCLib.D2(U, myParab, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case GeomAbs_BezierCurve:
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
            case GeomAbs_BSplineCurve:
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

    public override Geom_Curve.ResD3 EvalD3(double theU)
    {
        double U = theU;
        Geom_Curve.ResD3 aResult = new();
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                aResult.D2.SetCoord(0.0, 0.0, 0.0);
                aResult.D3.SetCoord(0.0, 0.0, 0.0);
                return aResult;
            case GeomAbs_Circle:
                ElCLib.D3(U, myCirc, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case GeomAbs_Ellipse:
                ElCLib.D3(U, myElips, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case GeomAbs_Hyperbola:
                ElCLib.D3(U, myHypr, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case GeomAbs_Parabola:
                ElCLib.D2(U, myParab, out aResult.Point, out aResult.D1, out aResult.D2);
                aResult.D3.SetCoord(0.0, 0.0, 0.0);
                return aResult;
            case GeomAbs_BezierCurve:
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
            case GeomAbs_BSplineCurve:
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

    public override gp_Vec EvalDN(double theU, int theN)
    {
        double U = theU;
        int N = theN;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                return ElCLib.DN(U, myLin, N);
            case GeomAbs_Circle:
                return ElCLib.DN(U, myCirc, N);
            case GeomAbs_Ellipse:
                return ElCLib.DN(U, myElips, N);
            case GeomAbs_Hyperbola:
                return ElCLib.DN(U, myHypr, N);
            case GeomAbs_Parabola:
                return ElCLib.DN(U, myParab, N);
            case GeomAbs_BezierCurve:
                return myCurve!.EvalDN(U, N);
            case GeomAbs_BSplineCurve:
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
            case GeomAbs_Line:
                return R3D;
            case GeomAbs_Circle:
            {
                double R = myCirc.Radius();
                if (R > R3D / 2.0)
                {
                    return 2 * Math.Asin(R3D / (2 * R));
                }
                return 2 * Math.PI;
            }
            case GeomAbs_Ellipse:
                return R3D / myElips.MajorRadius();
            case GeomAbs_BezierCurve:
            {
                myBezier!.Resolution(R3D, out double res);
                return res;
            }
            case GeomAbs_BSplineCurve:
            {
                myBSpline!.Resolution(R3D, out double res);
                return res;
            }
            default:
                return Precision.Parametric(R3D);
        }
    }

    public override GeomAbs_CurveType GetType() => myTypeCurve;

    public override gp_Lin Line()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Line, "GeomAdaptor_Curve::Line() - curve is not a Line");
        return myLin;
    }

    public override gp_Circ Circle()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Circle, "GeomAdaptor_Curve::Circle() - curve is not a Circle");
        return myCirc;
    }

    public override gp_Elips Ellipse()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Ellipse,
                                       "GeomAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return myElips;
    }

    public override gp_Hypr Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Hyperbola,
                                       "GeomAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return myHypr;
    }

    public override gp_Parab Parabola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Parabola,
                                       "GeomAdaptor_Curve::Parabola() - curve is not a Parabola");
        return myParab;
    }

    public override int Degree()
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            return myBezier!.Degree();
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return myBSpline!.Degree();
        }
        throw new Standard_NoSuchObject();
    }

    public override bool IsRational()
    {
        switch (myTypeCurve)
        {
            case GeomAbs_BSplineCurve:
                return myBSpline!.IsRational();
            case GeomAbs_BezierCurve:
                return myBezier!.IsRational();
            default:
                return false;
        }
    }

    public override int NbPoles()
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            return myBezier!.NbPoles();
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return myBSpline!.NbPoles();
        }
        throw new Standard_NoSuchObject();
    }

    public override int NbKnots()
    {
        if (myTypeCurve != GeomAbs_BSplineCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::NbKnots");
        }
        return myBSpline!.NbKnots();
    }

    public override Geom_BezierCurve Bezier()
    {
        if (myTypeCurve != GeomAbs_BezierCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::Bezier");
        }
        return myBezier!;
    }

    public override Geom_BSplineCurve BSpline()
    {
        if (myTypeCurve != GeomAbs_BSplineCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::BSpline");
        }
        return myBSpline!;
    }

    public override Geom_OffsetCurve OffsetCurve()
    {
        if (myTypeCurve != GeomAbs_OffsetCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::OffsetCurve");
        }
        return (Geom_OffsetCurve)myCurve!;
    }
}

/// <summary>A test stand-in for Geom2dAdaptor_Curve: a Geom2d_Curve seen through Adaptor2d_Curve2d (see the file header).</summary>
internal sealed class TestGeom2dAdaptor_Curve : Adaptor2d_Curve2d
{
    private static readonly double PosTol = Precision.PConfusion() / 2;

    private Geom2d_Curve? myCurve;
    private GeomAbs_CurveType myTypeCurve = GeomAbs_OtherCurve;
    private double myFirst;
    private double myLast;

    private gp_Lin2d myLin = new();
    private gp_Circ2d myCirc = new();
    private gp_Elips2d myElips = new();
    private gp_Hypr2d myHypr = new();
    private gp_Parab2d myParab = new();
    private Geom2d_BezierCurve? myBezier;
    private Geom2d_BSplineCurve? myBSpline;
    private bool myHasEvalRep;
    private BSplCLib_Cache? myCache;
    private NCollection_Array1<double>? myBezierFlatKnots;

    public TestGeom2dAdaptor_Curve()
    {
    }

    public TestGeom2dAdaptor_Curve(Geom2d_Curve theCrv)
    {
        Load(theCrv);
    }

    public TestGeom2dAdaptor_Curve(Geom2d_Curve theCrv, double theUFirst, double theULast)
    {
        myFirst = theUFirst;
        myLast = theULast;
        Load(theCrv, theUFirst, theULast);
    }

    public void Load(Geom2d_Curve theCurve)
    {
        if (theCurve is null)
        {
            throw new Standard_NullObject();
        }
        load(theCurve, theCurve.FirstParameter(), theCurve.LastParameter());
    }

    public void Load(Geom2d_Curve theCurve, double theUFirst, double theULast)
    {
        if (theCurve is null)
        {
            throw new Standard_NullObject();
        }
        if (theUFirst > theULast + Precision.Confusion())
        {
            throw new Standard_ConstructionError();
        }
        load(theCurve, theUFirst, theULast);
    }

    public Geom2d_Curve? Curve() => myCurve;

    private void load(Geom2d_Curve C, double UFirst, double ULast)
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
                case Geom2d_TrimmedCurve aTrimmed when C.GetType() == typeof(Geom2d_TrimmedCurve):
                    Load(aTrimmed.BasisCurve(), UFirst, ULast);
                    break;
                case Geom2d_Circle aCircle when C.GetType() == typeof(Geom2d_Circle):
                    myTypeCurve = GeomAbs_Circle;
                    myCirc = aCircle.Circ2d();
                    break;
                case Geom2d_Line aLine when C.GetType() == typeof(Geom2d_Line):
                    myTypeCurve = GeomAbs_Line;
                    myLin = aLine.Lin2d();
                    break;
                case Geom2d_Ellipse anEllipse when C.GetType() == typeof(Geom2d_Ellipse):
                    myTypeCurve = GeomAbs_Ellipse;
                    myElips = anEllipse.Elips2d();
                    break;
                case Geom2d_Parabola aParabola when C.GetType() == typeof(Geom2d_Parabola):
                    myTypeCurve = GeomAbs_Parabola;
                    myParab = aParabola.Parab2d();
                    break;
                case Geom2d_Hyperbola aHyperbola when C.GetType() == typeof(Geom2d_Hyperbola):
                    myTypeCurve = GeomAbs_Hyperbola;
                    myHypr = aHyperbola.Hypr2d();
                    break;
                case Geom2d_BezierCurve aBezier when C.GetType() == typeof(Geom2d_BezierCurve):
                    myTypeCurve = GeomAbs_BezierCurve;
                    myBezier = aBezier;
                    myHasEvalRep = aBezier.HasEvalRepresentation();
                    break;
                case Geom2d_BSplineCurve aBSpline when C.GetType() == typeof(Geom2d_BSplineCurve):
                    myTypeCurve = GeomAbs_BSplineCurve;
                    myBSpline = aBSpline;
                    myHasEvalRep = aBSpline.HasEvalRepresentation();
                    break;
                case Geom2d_OffsetCurve when C.GetType() == typeof(Geom2d_OffsetCurve):
                    // Test-only: evaluated through the curve itself (see the file header).
                    myTypeCurve = GeomAbs_OffsetCurve;
                    myHasEvalRep = true;
                    break;
                default:
                    myTypeCurve = GeomAbs_OtherCurve;
                    break;
            }
        }
        else
        {
            // Same curve but potentially different parameters - invalidate cache
            myCache = null;
        }
    }

    public override Adaptor2d_Curve2d ShallowCopy()
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

    private GeomAbs_Shape LocalContinuity(double U1, double U2)
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_BSplineCurve, " ");
        Geom2d_BSplineCurve aBSpline = myBSpline!;
        int Nb = aBSpline.NbKnots();
        int Index1;
        int Index2;
        NCollection_Array1<double> TK = aBSpline.InternalKnots();
        NCollection_Array1<int> TM = aBSpline.InternalMultiplicities();
        BSplCLib.LocateParameter(aBSpline.Degree(), TK, TM, U1, aBSpline.IsPeriodic(), 1, Nb, out Index1,
                                 out double newFirst);
        BSplCLib.LocateParameter(aBSpline.Degree(), TK, TM, U2, aBSpline.IsPeriodic(), 1, Nb, out Index2,
                                 out double newLast);
        if (Math.Abs(newFirst - TK[Index1 + 1]) < Precision.PConfusion())
        {
            if (Index1 < Nb)
            {
                Index1++;
            }
        }
        if (Math.Abs(newLast - TK[Index2]) < Precision.PConfusion())
        {
            Index2--;
        }
        int MultMax;
        // beware of periodic curves.
        if (aBSpline.IsPeriodic() && Index1 == Nb)
        {
            Index1 = 1;
        }

        if ((Index2 - Index1 <= 0) && (!aBSpline.IsPeriodic()))
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
            return GeomAbs_C0;
        }
        else if (MultMax == 1)
        {
            return GeomAbs_C1;
        }
        else if (MultMax == 2)
        {
            return GeomAbs_C2;
        }
        else if (MultMax == 3)
        {
            return GeomAbs_C3;
        }
        else
        {
            return GeomAbs_CN;
        }
    }

    public override GeomAbs_Shape Continuity()
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return LocalContinuity(myFirst, myLast);
        }
        else if (myTypeCurve == GeomAbs_OffsetCurve)
        {
            return myCurve!.Continuity(); // test-only simplification
        }
        else if (myTypeCurve == GeomAbs_OtherCurve)
        {
            throw new Standard_NoSuchObject("Geom2dAdaptor_Curve::Continuity");
        }
        else
        {
            return GeomAbs_CN;
        }
    }

    private int BSplineContinuityOrder(GeomAbs_Shape S, string theWhere)
    {
        switch (S)
        {
            case GeomAbs_C1:
                return 1;
            case GeomAbs_C2:
                return 2;
            case GeomAbs_C3:
                return 3;
            case GeomAbs_CN:
                return myBSpline!.Degree();
            default:
                throw new Standard_DomainError(theWhere);
        }
    }

    public override int NbIntervals(GeomAbs_Shape S)
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            Geom2d_BSplineCurve aBSpline = myBSpline!;
            if ((!aBSpline.IsPeriodic() && S <= Continuity()) || S == GeomAbs_C0)
            {
                return 1;
            }
            int aDegree = aBSpline.Degree();
            int aCont = BSplineContinuityOrder(S, "Geom2dAdaptor_Curve::NbIntervals()");
            double anEps = Min(Resolution(Precision.Confusion()), Precision.PConfusion());
            return BSplCLib.Intervals(aBSpline.InternalKnots(), aBSpline.InternalMultiplicities(), aDegree,
                                      aBSpline.IsPeriodic(), aCont, myFirst, myLast, anEps, null);
        }
        return 1;
    }

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            Geom2d_BSplineCurve aBSpline = myBSpline!;
            if ((!aBSpline.IsPeriodic() && S <= Continuity()) || S == GeomAbs_C0)
            {
                T[T.Lower()] = myFirst;
                T[T.Lower() + 1] = myLast;
                return;
            }
            int aDegree = aBSpline.Degree();
            int aCont = BSplineContinuityOrder(S, "Geom2dAdaptor_Curve::Intervals()");
            double anEps = Min(Resolution(Precision.Confusion()), Precision.PConfusion());
            BSplCLib.Intervals(aBSpline.InternalKnots(), aBSpline.InternalMultiplicities(), aDegree, aBSpline.IsPeriodic(),
                               aCont, myFirst, myLast, anEps, T);
        }
        else
        {
            T[T.Lower()] = myFirst;
            T[T.Lower() + 1] = myLast;
        }
    }

    public override Adaptor2d_Curve2d Trim(double First, double Last, double Tol) =>
        new TestGeom2dAdaptor_Curve(myCurve!, First, Last);

    public override bool IsClosed()
    {
        if (!Precision.IsPositiveInfinite(myLast) && !Precision.IsNegativeInfinite(myFirst))
        {
            gp_Pnt2d Pd = Value(myFirst);
            gp_Pnt2d Pf = Value(myLast);
            return (Pd.Distance(Pf) <= Precision.Confusion());
        }
        return false;
    }

    public override bool IsPeriodic() => myCurve!.IsPeriodic();

    public override double Period() => myCurve!.LastParameter() - myCurve.FirstParameter();

    private void RebuildCache(double theParameter)
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            // Create cache for Bezier
            Geom2d_BezierCurve aBezier = myBezier!;
            int aDeg = aBezier.Degree();
            myBezierFlatKnots ??= NCollection_Array1<double>.From(BSplCLib.FlatBezierKnots(aDeg));
            myCache ??= new BSplCLib_Cache(aDeg, aBezier.IsPeriodic(), myBezierFlatKnots, aBezier.InternalPoles(),
                                           aBezier.InternalWeights());
            myCache.BuildCache(theParameter, myBezierFlatKnots, aBezier.InternalPoles(), aBezier.InternalWeights());
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            // Create cache for B-spline
            Geom2d_BSplineCurve aBSpline = myBSpline!;
            myCache ??= new BSplCLib_Cache(aBSpline.Degree(), aBSpline.IsPeriodic(), aBSpline.InternalKnotSequence(),
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
            Geom2d_BSplineCurve aBSpline = myBSpline;
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
                if (theSpanFinish > aBSpline.NbKnots())
                {
                    theSpanFinish = aBSpline.NbKnots();
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

    public override gp_Pnt2d Value(double U)
    {
        D0(U, out gp_Pnt2d aRes);
        return aRes;
    }

    public override void D0(double U, out gp_Pnt2d P) => P = EvalD0(U);

    public override void D1(double U, out gp_Pnt2d P, out gp_Vec2d V)
    {
        Geom2d_Curve.ResD1 aRes = EvalD1(U);
        P = aRes.Point;
        V = aRes.D1;
    }

    public override void D2(double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        Geom2d_Curve.ResD2 aRes = EvalD2(U);
        P = aRes.Point;
        V1 = aRes.D1;
        V2 = aRes.D2;
    }

    public override void D3(double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
    {
        Geom2d_Curve.ResD3 aRes = EvalD3(U);
        P = aRes.Point;
        V1 = aRes.D1;
        V2 = aRes.D2;
        V3 = aRes.D3;
    }

    public override gp_Vec2d DN(double U, int N) => EvalDN(U, N);

    public override gp_Pnt2d EvalD0(double theU)
    {
        double U = theU;
        gp_Pnt2d P;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                return ElCLib.Value(U, myLin);
            case GeomAbs_Circle:
                return ElCLib.Value(U, myCirc);
            case GeomAbs_Ellipse:
                return ElCLib.Value(U, myElips);
            case GeomAbs_Hyperbola:
                return ElCLib.Value(U, myHypr);
            case GeomAbs_Parabola:
                return ElCLib.Value(U, myParab);
            case GeomAbs_BezierCurve:
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
            case GeomAbs_BSplineCurve:
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

    public override Geom2d_Curve.ResD1 EvalD1(double theU)
    {
        double U = theU;
        Geom2d_Curve.ResD1 aResult = new();
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_Circle:
                ElCLib.D1(U, myCirc, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_Ellipse:
                ElCLib.D1(U, myElips, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_Hyperbola:
                ElCLib.D1(U, myHypr, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_Parabola:
                ElCLib.D1(U, myParab, out aResult.Point, out aResult.D1);
                return aResult;
            case GeomAbs_BezierCurve:
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
            case GeomAbs_BSplineCurve:
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

    public override Geom2d_Curve.ResD2 EvalD2(double theU)
    {
        double U = theU;
        Geom2d_Curve.ResD2 aResult = new();
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                aResult.D2.SetCoord(0.0, 0.0);
                return aResult;
            case GeomAbs_Circle:
                ElCLib.D2(U, myCirc, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case GeomAbs_Ellipse:
                ElCLib.D2(U, myElips, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case GeomAbs_Hyperbola:
                ElCLib.D2(U, myHypr, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case GeomAbs_Parabola:
                ElCLib.D2(U, myParab, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            case GeomAbs_BezierCurve:
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
            case GeomAbs_BSplineCurve:
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

    public override Geom2d_Curve.ResD3 EvalD3(double theU)
    {
        double U = theU;
        Geom2d_Curve.ResD3 aResult = new();
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, myLin, out aResult.Point, out aResult.D1);
                aResult.D2.SetCoord(0.0, 0.0);
                aResult.D3.SetCoord(0.0, 0.0);
                return aResult;
            case GeomAbs_Circle:
                ElCLib.D3(U, myCirc, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case GeomAbs_Ellipse:
                ElCLib.D3(U, myElips, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case GeomAbs_Hyperbola:
                ElCLib.D3(U, myHypr, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            case GeomAbs_Parabola:
                ElCLib.D2(U, myParab, out aResult.Point, out aResult.D1, out aResult.D2);
                aResult.D3.SetCoord(0.0, 0.0);
                return aResult;
            case GeomAbs_BezierCurve:
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
            case GeomAbs_BSplineCurve:
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

    public override gp_Vec2d EvalDN(double theU, int theN)
    {
        double U = theU;
        int N = theN;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                return ElCLib.DN(U, myLin, N);
            case GeomAbs_Circle:
                return ElCLib.DN(U, myCirc, N);
            case GeomAbs_Ellipse:
                return ElCLib.DN(U, myElips, N);
            case GeomAbs_Hyperbola:
                return ElCLib.DN(U, myHypr, N);
            case GeomAbs_Parabola:
                return ElCLib.DN(U, myParab, N);
            case GeomAbs_BezierCurve:
                return myCurve!.EvalDN(U, N);
            case GeomAbs_BSplineCurve:
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
            case GeomAbs_Line:
                return Ruv;
            case GeomAbs_Circle:
            {
                double R = myCirc.Radius();
                if (R > Ruv / 2.0)
                {
                    return 2 * Math.Asin(Ruv / (2 * R));
                }
                return 2 * Math.PI;
            }
            case GeomAbs_Ellipse:
                return Ruv / myElips.MajorRadius();
            case GeomAbs_BezierCurve:
            {
                myBezier!.Resolution(Ruv, out double res);
                return res;
            }
            case GeomAbs_BSplineCurve:
            {
                myBSpline!.Resolution(Ruv, out double res);
                return res;
            }
            default:
                return Precision.Parametric(Ruv);
        }
    }

    public override GeomAbs_CurveType GetType() => myTypeCurve;

    public override gp_Lin2d Line()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Line, "Geom2dAdaptor_Curve::Line() - curve is not a Line");
        return myLin;
    }

    public override gp_Circ2d Circle()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Circle,
                                       "Geom2dAdaptor_Curve::Circle() - curve is not a Circle");
        return myCirc;
    }

    public override gp_Elips2d Ellipse()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Ellipse,
                                       "Geom2dAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return myElips;
    }

    public override gp_Hypr2d Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Hyperbola,
                                       "Geom2dAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return myHypr;
    }

    public override gp_Parab2d Parabola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Parabola,
                                       "Geom2dAdaptor_Curve::Parabola() - curve is not a Parabola");
        return myParab;
    }

    public override int Degree()
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            return myBezier!.Degree();
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return myBSpline!.Degree();
        }
        throw new Standard_NoSuchObject();
    }

    public override bool IsRational()
    {
        switch (myTypeCurve)
        {
            case GeomAbs_BSplineCurve:
                return myBSpline!.IsRational();
            case GeomAbs_BezierCurve:
                return myBezier!.IsRational();
            default:
                return false;
        }
    }

    public override int NbPoles()
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            return myBezier!.NbPoles();
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return myBSpline!.NbPoles();
        }
        throw new Standard_NoSuchObject();
    }

    public override int NbKnots()
    {
        if (myTypeCurve != GeomAbs_BSplineCurve)
        {
            throw new Standard_NoSuchObject("Geom2dAdaptor_Curve::NbKnots");
        }
        return myBSpline!.NbKnots();
    }

    // Geom2dAdaptor_Curve::Bezier() and BSpline() return a null handle (they don't throw) for other curves.
    public override Geom2d_BezierCurve Bezier() => (myCurve as Geom2d_BezierCurve)!;

    public override Geom2d_BSplineCurve BSpline() => myBSpline!;

    private static int nbPoints(Geom2d_Curve theCurve)
    {
        int nbs = 20;
        if (theCurve is Geom2d_Line)
        {
            nbs = 2;
        }
        else if (theCurve is Geom2d_BezierCurve aBezier)
        {
            nbs = 3 + aBezier.NbPoles();
        }
        else if (theCurve is Geom2d_BSplineCurve aBSpline)
        {
            nbs = aBSpline.NbKnots();
            nbs *= aBSpline.Degree();
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        else if (theCurve is Geom2d_OffsetCurve anOffset)
        {
            return Max(nbs, nbPoints(anOffset.BasisCurve()));
        }
        else if (theCurve is Geom2d_TrimmedCurve aTrimmed)
        {
            return Max(nbs, nbPoints(aTrimmed.BasisCurve()));
        }
        if (nbs > 300)
        {
            nbs = 300;
        }
        return nbs;
    }

    public override int NbSamples() => nbPoints(myCurve!);
}
