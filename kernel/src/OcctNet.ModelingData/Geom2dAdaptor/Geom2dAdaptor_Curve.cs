// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor_Curve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dAdaptor/Geom2dAdaptor_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// An interface between the services provided by any curve from the package Geom2d and those required of the
/// curve by algorithms which use it.
/// </summary>
/// <remarks>
/// An adaptor is a working object (decision 101): <see cref="Load(Geom2d_Curve)"/> is public, and B-spline and
/// Bezier coefficients are cached for evaluation, so evaluating one adaptor from several threads at once is not
/// safe. The adaptor keeps a reference to the curve it adapts, as OCCT does; it doesn't copy it.
/// </remarks>
public class Geom2dAdaptor_Curve : Adaptor2d_Curve2d
{
    // OCCT's std::variant<std::monostate, gp_Lin2d, gp_Circ2d, gp_Elips2d, gp_Hypr2d, gp_Parab2d, OffsetData,
    // BezierData, BSplineData> is an object: null (monostate), a boxed gp value, or one of the classes below.

    /// <summary>Evaluation data of a 2D offset curve.</summary>
    internal sealed class OffsetData
    {
        /// <summary>Adaptor for basis curve.</summary>
        public Geom2dAdaptor_Curve? BasisAdaptor;
        /// <summary>Offset distance.</summary>
        public double Offset = 0.0;
        /// <summary>Eval representation descriptor.</summary>
        public Geom2dEval_RepCurveDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a Bezier curve.</summary>
    internal sealed class BezierData
    {
        /// <summary>Bezier curve to prevent downcasts.</summary>
        public Geom2d_BezierCurve? Curve;
        /// <summary>Cached data for evaluation.</summary>
        public Cache? Cache;
        /// <summary>Eval representation descriptor.</summary>
        public Geom2dEval_RepCurveDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a B-spline curve.</summary>
    internal sealed class BSplineData
    {
        /// <summary>BSpline curve to prevent downcasts.</summary>
        public Geom2d_BSplineCurve? Curve;
        /// <summary>Cached data for evaluation.</summary>
        public Cache? Cache;
        /// <summary>Eval representation descriptor.</summary>
        public Geom2dEval_RepCurveDesc.Base? EvalRep;
    }

    private static readonly double PosTol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 2;

    protected Geom2d_Curve? myCurve;
    protected CurveKind myTypeCurve;
    protected double myFirst;
    protected double myLast;
    // Curve-specific evaluation data (BSpline, Bezier, offset)
    private protected object? myCurveData;

    private static bool hasEvalRep(object? theData)
    {
        if (theData is BezierData aBezierData)
        {
            return aBezierData.EvalRep is not null;
        }
        if (theData is BSplineData aBSplineData)
        {
            return aBSplineData.EvalRep is not null;
        }
        if (theData is OffsetData anOffsetData)
        {
            return anOffsetData.EvalRep is not null;
        }
        return false;
    }

    /// <summary>An empty adaptor; load a curve before use.</summary>
    public Geom2dAdaptor_Curve()
    {
        myTypeCurve = OtherCurve;
        myFirst = 0.0;
        myLast = 0.0;
    }

    /// <summary>Adapts the whole curve.</summary>
    /// <exception cref="NullObjectException">The curve is null.</exception>
    public Geom2dAdaptor_Curve(Geom2d_Curve theCrv)
    {
        myTypeCurve = OtherCurve;
        myFirst = 0.0;
        myLast = 0.0;
        Load(theCrv);
    }

    /// <summary>Adapts the curve between theUFirst and theULast.</summary>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast + Precision::Confusion().</exception>
    public Geom2dAdaptor_Curve(Geom2d_Curve theCrv, double theUFirst, double theULast)
    {
        myTypeCurve = OtherCurve;
        myFirst = theUFirst;
        myLast = theULast;
        Load(theCrv, theUFirst, theULast);
    }

    /// <summary>A shallow copy: the curve is shared, the evaluation caches are not.</summary>
    public override Adaptor2d_Curve2d ShallowCopy()
    {
        Geom2dAdaptor_Curve aCopy = new Geom2dAdaptor_Curve();
        ShallowCopyTo(aCopy);
        return aCopy;
    }

    /// <summary>
    /// Copies the curve, its range and its evaluation data (without caches) into <paramref name="aCopy"/>: the body of
    /// <see cref="ShallowCopy"/>, shared with BRepAdaptor_Curve2d's (OCCT copies the fields there by hand).
    /// </summary>
    private protected void ShallowCopyTo(Geom2dAdaptor_Curve aCopy)
    {
        aCopy.myCurve = myCurve;
        aCopy.myTypeCurve = myTypeCurve;
        aCopy.myFirst = myFirst;
        aCopy.myLast = myLast;

        // Copy curve-specific data based on variant type
        if (myCurveData is OffsetData anOffsetData)
        {
            OffsetData aNewData = new OffsetData();
            if (anOffsetData.BasisAdaptor is not null)
            {
                aNewData.BasisAdaptor = anOffsetData.BasisAdaptor.ShallowCopy() as Geom2dAdaptor_Curve;
            }
            aNewData.Offset = anOffsetData.Offset;
            aNewData.EvalRep = anOffsetData.EvalRep;
            aCopy.myCurveData = aNewData;
        }
        else if (myCurveData is BSplineData aBSplineData)
        {
            BSplineData aNewData = new BSplineData();
            aNewData.Curve = aBSplineData.Curve;
            aNewData.EvalRep = aBSplineData.EvalRep;
            aCopy.myCurveData = aNewData;
        }
        else if (myCurveData is BezierData aBezierData)
        {
            BezierData aNewData = new BezierData();
            aNewData.Curve = aBezierData.Curve;
            aNewData.EvalRep = aBezierData.EvalRep;
            // Cache is not copied - will be rebuilt on demand.
            aCopy.myCurveData = aNewData;
        }
        else
        {
            // Elementary curve types (gp_Lin2d, gp_Circ2d, etc.) are value types (boxed: immutable here)
            aCopy.myCurveData = myCurveData;
        }
    }

    // Computes the Continuity of a BSplineCurve between the parameters U1 and U2
    // The continuity is C(d-m)
    //   with d = degree,
    //        m = max multiplicity of the Knots between U1 and U2
    private Continuity LocalContinuity(double U1, double U2)
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != BSplineCurve, " ");
        Geom2d_BSplineCurve aBSpline = ((BSplineData)myCurveData!).Curve!;
        int Nb = aBSpline.NbKnots();
        int Index1;
        int Index2;
        double newFirst, newLast;
        NCollection_Array1<double> TK = aBSpline.InternalKnots();
        NCollection_Array1<int> TM = aBSpline.InternalMultiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aBSpline.Degree(), TK, TM, U1, aBSpline.IsPeriodic(), 1, Nb, out Index1, out newFirst);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aBSpline.Degree(), TK, TM, U2, aBSpline.IsPeriodic(), 1, Nb, out Index2, out newLast);
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

    /// <summary>Resets the adaptor to its empty state (undoes Load).</summary>
    public void Reset()
    {
        myTypeCurve = OtherCurve;
        myCurve = null;
        myCurveData = null;
        myFirst = myLast = 0.0;
    }

    /// <summary>Loads the whole curve.</summary>
    /// <exception cref="NullObjectException">The curve is null.</exception>
    public void Load(Geom2d_Curve theCurve)
    {
        if (theCurve is null)
        {
            throw new NullObjectException();
        }
        load(theCurve, theCurve.FirstParameter(), theCurve.LastParameter());
    }

    /// <summary>Loads the curve between theUFirst and theULast.</summary>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast + Precision::Confusion().</exception>
    public void Load(Geom2d_Curve theCurve, double theUFirst, double theULast)
    {
        if (theCurve is null)
        {
            throw new NullObjectException();
        }
        if (theUFirst > theULast + global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            throw new ConstructionErrorException();
        }
        load(theCurve, theUFirst, theULast);
    }

    /// <summary>True if a curve has been loaded.</summary>
    public bool IsInitialized() => myCurve is not null;

    /// <summary>The adapted curve (null if none is loaded).</summary>
    public Geom2d_Curve? Curve() => myCurve;

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    private void load(Geom2d_Curve C, double UFirst, double ULast)
    {
        myFirst = UFirst;
        myLast = ULast;

        if (myCurve != C)
        {
            myCurve = C;
            myCurveData = null;

            Type TheType = C.DynamicType();
            if (TheType == typeof(Geom2d_TrimmedCurve))
            {
                Load(((Geom2d_TrimmedCurve)C).BasisCurve(), UFirst, ULast);
            }
            else if (TheType == typeof(Geom2d_Circle))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Circle;
                myCurveData = ((Geom2d_Circle)C).Circ2d();
            }
            else if (TheType == typeof(Geom2d_Line))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Line;
                myCurveData = ((Geom2d_Line)C).Lin2d();
            }
            else if (TheType == typeof(Geom2d_Ellipse))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Ellipse;
                myCurveData = ((Geom2d_Ellipse)C).Elips2d();
            }
            else if (TheType == typeof(Geom2d_Parabola))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Parabola;
                myCurveData = ((Geom2d_Parabola)C).Parab2d();
            }
            else if (TheType == typeof(Geom2d_Hyperbola))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Hyperbola;
                myCurveData = ((Geom2d_Hyperbola)C).Hypr2d();
            }
            else if (TheType == typeof(Geom2d_BezierCurve))
            {
                myTypeCurve = BezierCurve;
                BezierData aBezierData = new BezierData();
                aBezierData.Curve = (Geom2d_BezierCurve)myCurve;
                aBezierData.EvalRep = aBezierData.Curve.EvalRepresentation();
                myCurveData = aBezierData;
            }
            else if (TheType == typeof(Geom2d_BSplineCurve))
            {
                myTypeCurve = BSplineCurve;
                BSplineData aBSplineData = new BSplineData();
                aBSplineData.Curve = (Geom2d_BSplineCurve)myCurve;
                aBSplineData.EvalRep = aBSplineData.Curve.EvalRepresentation();
                myCurveData = aBSplineData;
            }
            else if (TheType == typeof(Geom2d_OffsetCurve))
            {
                myTypeCurve = OffsetCurve;
                Geom2d_OffsetCurve anOffsetCurve = (Geom2d_OffsetCurve)myCurve;
                // Create nested adaptor for base curve and store offset data
                Geom2d_Curve aBaseCurve = anOffsetCurve.BasisCurve();

                OffsetData anOffsetData = new OffsetData();
                anOffsetData.BasisAdaptor = new Geom2dAdaptor_Curve(aBaseCurve, UFirst, ULast);
                anOffsetData.Offset = anOffsetCurve.Offset();
                anOffsetData.EvalRep = anOffsetCurve.EvalRepresentation();
                myCurveData = anOffsetData;
            }
            else
            {
                myTypeCurve = OtherCurve;
            }
        }
        else
        {
            // Same curve but potentially different parameters - invalidate cache
            if (myCurveData is BSplineData aBSplineData)
            {
                aBSplineData.Cache = null;
            }
            else if (myCurveData is BezierData aBezierData)
            {
                aBezierData.Cache = null;
            }
        }
    }

    //    --
    //    --     Global methods - Apply to the whole curve.
    //    --

    public override Continuity Continuity()
    {
        if (myTypeCurve == BSplineCurve)
        {
            return LocalContinuity(myFirst, myLast);
        }
        else if (myTypeCurve == OffsetCurve)
        {
            Continuity S = ((Geom2d_OffsetCurve)myCurve!).GetBasisCurveContinuity();
            switch (S)
            {
                case CN:
                    return CN;
                case C3:
                    return C2;
                case C2:
                    return C1;
                case C1:
                    return C0;
                case G1:
                    return G1;
                case G2:
                    return G2;

                default:
                    throw new NoSuchObjectException("Geom2dAdaptor_Curve::Continuity");
            }
        }

        else if (myTypeCurve == OtherCurve)
        {
            throw new NoSuchObjectException("Geom2dAdaptor_Curve::Continuity");
        }
        else
        {
            return CN;
        }
    }

    /// <summary>
    /// Breaks the curve, if necessary, into intervals of continuity S, and returns their number.
    /// </summary>
    public override int NbIntervals(Continuity S)
    {
        if (myTypeCurve == BSplineCurve)
        {
            Geom2d_BSplineCurve aBSpline = ((BSplineData)myCurveData!).Curve!;
            if ((!aBSpline.IsPeriodic() && S <= Continuity()) || S == C0)
            {
                return 1;
            }

            int aDegree = aBSpline.Degree();
            int aCont;

            switch (S)
            {
                case C1:
                    aCont = 1;
                    break;
                case C2:
                    aCont = 2;
                    break;
                case C3:
                    aCont = 3;
                    break;
                case CN:
                    aCont = aDegree;
                    break;
                default:
                    throw new DomainErrorException("Geom2dAdaptor_Curve::NbIntervals()");
            }

            double anEps = Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());

            return global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(aBSpline.InternalKnots(),
                                      aBSpline.InternalMultiplicities(),
                                      aDegree,
                                      aBSpline.IsPeriodic(),
                                      aCont,
                                      myFirst,
                                      myLast,
                                      anEps,
                                      null);
        }

        else if (myTypeCurve == OffsetCurve)
        {
            int myNbIntervals = 1;
            Continuity BaseS = C0;
            switch (S)
            {
                case G1:
                case G2:
                    throw new DomainErrorException("GeomAdaptor_Curve::NbIntervals");
                case C0:
                    BaseS = C1;
                    break;
                case C1:
                    BaseS = C2;
                    break;
                case C2:
                    BaseS = C3;
                    break;
                default:
                    BaseS = CN;
                    break;
            }
            Geom2dAdaptor_Curve anAdaptor = new Geom2dAdaptor_Curve(((Geom2d_OffsetCurve)myCurve!).BasisCurve(),
                                                                    myFirst,
                                                                    myLast);
            myNbIntervals = anAdaptor.NbIntervals(BaseS);
            return myNbIntervals;
        }

        else
        {
            return 1;
        }
    }

    /// <summary>
    /// Stores in T the parameters bounding the intervals of continuity S. T must have room for
    /// NbIntervals(S) + 1 values.
    /// </summary>
    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        if (myTypeCurve == BSplineCurve)
        {
            Geom2d_BSplineCurve aBSpline = ((BSplineData)myCurveData!).Curve!;
            if ((!aBSpline.IsPeriodic() && S <= Continuity()) || S == C0)
            {
                T[T.Lower()] = myFirst;
                T[T.Lower() + 1] = myLast;
                return;
            }

            int aDegree = aBSpline.Degree();
            int aCont;

            switch (S)
            {
                case C1:
                    aCont = 1;
                    break;
                case C2:
                    aCont = 2;
                    break;
                case C3:
                    aCont = 3;
                    break;
                case CN:
                    aCont = aDegree;
                    break;
                default:
                    throw new DomainErrorException("Geom2dAdaptor_Curve::Intervals()");
            }

            double anEps = Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(aBSpline.InternalKnots(),
                               aBSpline.InternalMultiplicities(),
                               aDegree,
                               aBSpline.IsPeriodic(),
                               aCont,
                               myFirst,
                               myLast,
                               anEps,
                               T);
        }

        else if (myTypeCurve == OffsetCurve)
        {
            int myNbIntervals = 1;
            Continuity BaseS = C0;
            switch (S)
            {
                case G1:
                case G2:
                    throw new DomainErrorException("GeomAdaptor_Curve::NbIntervals");
                case C0:
                    BaseS = C1;
                    break;
                case C1:
                    BaseS = C2;
                    break;
                case C2:
                    BaseS = C3;
                    break;
                default:
                    BaseS = CN;
                    break;
            }

            Geom2dAdaptor_Curve anAdaptor = new Geom2dAdaptor_Curve(((Geom2d_OffsetCurve)myCurve!).BasisCurve(),
                                                                    myFirst,
                                                                    myLast);
            myNbIntervals = anAdaptor.NbIntervals(BaseS);
            anAdaptor.Intervals(T, BaseS);
            T[T.Lower()] = myFirst;
            T[T.Lower() + myNbIntervals] = myLast;
        }

        else
        {
            T[T.Lower()] = myFirst;
            T[T.Lower() + 1] = myLast;
        }
    }

    /// <summary>A new adaptor of the same curve between First and Last (Tol is not used).</summary>
    public override Adaptor2d_Curve2d Trim(double First, double Last, double Tol)
    {
        _ = Tol;
        Geom2dAdaptor_Curve HE = new Geom2dAdaptor_Curve(myCurve!, First, Last);
        return HE;
    }

    public override bool IsClosed()
    {
        if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(myLast) && !global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(myFirst))
        {
            Point2D Pd = Value(myFirst);
            Point2D Pf = Value(myLast);
            return (Pd.Distance(Pf) <= global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            return false;
        }
    }

    public override bool IsPeriodic() => myCurve!.IsPeriodic();

    public override double Period() => myCurve!.LastParameter() - myCurve.FirstParameter();

    // Rebuilds the B-spline cache for the span of theParameter
    private void RebuildCache(double theParameter)
    {
        if (myTypeCurve == BezierCurve)
        {
            // Create cache for Bezier
            BezierData aBezierData = (BezierData)myCurveData!;
            Geom2d_BezierCurve aBezier = (Geom2d_BezierCurve)myCurve!;
            int aDeg = aBezier.Degree();
            // OCCT wraps BSplCLib::FlatBezierKnots(aDeg) in an array; the curve's shared flat knots are the same values.
            NCollection_Array1<double> aFlatKnots = aBezier.InternalKnotSequence();
            if (aBezierData.Cache is null)
            {
                aBezierData.Cache = new Cache(aDeg,
                                                       aBezier.IsPeriodic(),
                                                       aFlatKnots,
                                                       aBezier.InternalPoles(),
                                                       aBezier.InternalWeights());
            }
            aBezierData.Cache.BuildCache(theParameter, aFlatKnots, aBezier.InternalPoles(), aBezier.InternalWeights());
        }
        else if (myTypeCurve == BSplineCurve)
        {
            // Create cache for B-spline
            BSplineData aBSplineData = (BSplineData)myCurveData!;
            Geom2d_BSplineCurve aBSpline = aBSplineData.Curve!;
            if (aBSplineData.Cache is null)
            {
                aBSplineData.Cache = new Cache(aBSpline.Degree(),
                                                        aBSpline.IsPeriodic(),
                                                        aBSpline.InternalKnotSequence(),
                                                        aBSpline.InternalPoles(),
                                                        aBSpline.InternalWeights());
            }
            aBSplineData.Cache.BuildCache(theParameter,
                                          aBSpline.InternalKnotSequence(),
                                          aBSpline.InternalPoles(),
                                          aBSpline.InternalWeights());
        }
    }

    // Checks whether theU is the start or end point of a B-spline curve and returns the indices of its span
    private bool IsBoundary(double theU, out int theSpanStart, out int theSpanFinish)
    {
        theSpanStart = 0;
        theSpanFinish = 0;
        if (myCurveData is BSplineData aBSplineData && aBSplineData.Curve is not null
            && (theU == myFirst || theU == myLast))
        {
            Geom2d_BSplineCurve aBSpline = aBSplineData.Curve;
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

    /// <summary>The point of parameter U.</summary>
    public sealed override Point2D Value(double U)
    {
        Point2D aRes;
        D0(U, out aRes);
        return aRes;
    }

    /// <summary>The point of parameter U.</summary>
    public sealed override void D0(double U, out Point2D P)
    {
        P = EvalD0(U);
    }

    /// <summary>The point of parameter U.</summary>
    public sealed override Point2D EvalD0(double theU)
    {
        double U = theU;
        Point2D P;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Line2D)myCurveData!);
                return P;

            case global::Kerf.Geometry.CurveKind.Circle:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Circle2D)myCurveData!);
                return P;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Ellipse2D)myCurveData!);
                return P;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Hyperbola2D)myCurveData!);
                return P;

            case global::Kerf.Geometry.CurveKind.Parabola:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Parabola2D)myCurveData!);
                return P;

            case BezierCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD0(U);
                    }
                    BezierData aBezierData = (BezierData)myCurveData!;
                    if (aBezierData.Cache is null)
                    {
                        RebuildCache(U);
                    }
                    aBezierData.Cache!.D0(U, out P);
                    return P;
                }

            case BSplineCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD0(U);
                    }
                    BSplineData aBSplineData = (BSplineData)myCurveData!;
                    if (IsBoundary(U, out int aStart, out int aFinish))
                    {
                        aBSplineData.Curve!.LocalD0(U, aStart, aFinish, out P);
                    }
                    else
                    {
                        if (aBSplineData.Cache is null || !aBSplineData.Cache.IsCacheValid(U))
                        {
                            RebuildCache(U);
                        }
                        aBSplineData.Cache!.D0(U, out P);
                    }
                    return P;
                }

            case OffsetCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD0(U);
                    }
                    OffsetData anOffsetData = (OffsetData)myCurveData!;
                    if (!Geom2d_OffsetCurveUtils.EvaluateD0(U, anOffsetData.BasisAdaptor!, anOffsetData.Offset, out P))
                    {
                        throw new Geom2d_UndefinedValue("Geom2dAdaptor_Curve::EvalD0: evaluation failed");
                    }
                    return P;
                }

            default:
                return myCurve!.EvalD0(U);
        }
    }

    /// <summary>The point and first derivative at U.</summary>
    public sealed override void D1(double U, out Point2D P, out Vector2D V)
    {
        Geom2d_Curve.ResD1 aResult = EvalD1(U);
        P = aResult.Point;
        V = aResult.D1;
    }

    /// <summary>The point and first derivative at U.</summary>
    public sealed override Geom2d_Curve.ResD1 EvalD1(double theU)
    {
        double U = theU;
        Geom2d_Curve.ResD1 aResult;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line2D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Circle2D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Ellipse2D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Hyperbola2D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Parabola2D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case BezierCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD1(U);
                    }
                    BezierData aBezierData = (BezierData)myCurveData!;
                    if (aBezierData.Cache is null)
                    {
                        RebuildCache(U);
                    }
                    aBezierData.Cache!.D1(U, out aResult.Point, out aResult.D1);
                    return aResult;
                }

            case BSplineCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD1(U);
                    }
                    BSplineData aBSplineData = (BSplineData)myCurveData!;
                    if (IsBoundary(U, out int aStart, out int aFinish))
                    {
                        aBSplineData.Curve!.LocalD1(U, aStart, aFinish, out aResult.Point, out aResult.D1);
                    }
                    else
                    {
                        if (aBSplineData.Cache is null || !aBSplineData.Cache.IsCacheValid(U))
                        {
                            RebuildCache(U);
                        }
                        aBSplineData.Cache!.D1(U, out aResult.Point, out aResult.D1);
                    }
                    return aResult;
                }

            case OffsetCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD1(U);
                    }
                    OffsetData anOffsetData = (OffsetData)myCurveData!;
                    if (!Geom2d_OffsetCurveUtils.EvaluateD1(U,
                                                            anOffsetData.BasisAdaptor!,
                                                            anOffsetData.Offset,
                                                            out aResult.Point,
                                                            out aResult.D1))
                    {
                        throw new Geom2d_UndefinedDerivative("Geom2dAdaptor_Curve::EvalD1: evaluation failed");
                    }
                    return aResult;
                }

            default:
                return myCurve!.EvalD1(U);
        }
    }

    /// <summary>The point and first two derivatives at U.</summary>
    public sealed override void D2(double U, out Point2D P, out Vector2D V1, out Vector2D V2)
    {
        Geom2d_Curve.ResD2 aResult = EvalD2(U);
        P = aResult.Point;
        V1 = aResult.D1;
        V2 = aResult.D2;
    }

    /// <summary>The point and first two derivatives at U.</summary>
    public sealed override Geom2d_Curve.ResD2 EvalD2(double theU)
    {
        double U = theU;
        Geom2d_Curve.ResD2 aResult;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line2D)myCurveData!, out aResult.Point, out aResult.D1);
                aResult.D2 = new Vector2D();
                aResult.D2.SetCoord(0.0, 0.0);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Circle2D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Ellipse2D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Hyperbola2D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Parabola2D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case BezierCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD2(U);
                    }
                    BezierData aBezierData = (BezierData)myCurveData!;
                    if (aBezierData.Cache is null)
                    {
                        RebuildCache(U);
                    }
                    aBezierData.Cache!.D2(U, out aResult.Point, out aResult.D1, out aResult.D2);
                    return aResult;
                }

            case BSplineCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD2(U);
                    }
                    BSplineData aBSplineData = (BSplineData)myCurveData!;
                    if (IsBoundary(U, out int aStart, out int aFinish))
                    {
                        aBSplineData.Curve!.LocalD2(U, aStart, aFinish, out aResult.Point, out aResult.D1, out aResult.D2);
                    }
                    else
                    {
                        if (aBSplineData.Cache is null || !aBSplineData.Cache.IsCacheValid(U))
                        {
                            RebuildCache(U);
                        }
                        aBSplineData.Cache!.D2(U, out aResult.Point, out aResult.D1, out aResult.D2);
                    }
                    return aResult;
                }

            case OffsetCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD2(U);
                    }
                    OffsetData anOffsetData = (OffsetData)myCurveData!;
                    if (!Geom2d_OffsetCurveUtils.EvaluateD2(U,
                                                            anOffsetData.BasisAdaptor!,
                                                            anOffsetData.Offset,
                                                            out aResult.Point,
                                                            out aResult.D1,
                                                            out aResult.D2))
                    {
                        throw new Geom2d_UndefinedDerivative("Geom2dAdaptor_Curve::EvalD2: evaluation failed");
                    }
                    return aResult;
                }

            default:
                return myCurve!.EvalD2(U);
        }
    }

    /// <summary>The point and first three derivatives at U.</summary>
    public sealed override void D3(double U, out Point2D P, out Vector2D V1, out Vector2D V2, out Vector2D V3)
    {
        Geom2d_Curve.ResD3 aResult = EvalD3(U);
        P = aResult.Point;
        V1 = aResult.D1;
        V2 = aResult.D2;
        V3 = aResult.D3;
    }

    /// <summary>The point and first three derivatives at U.</summary>
    public sealed override Geom2d_Curve.ResD3 EvalD3(double theU)
    {
        double U = theU;
        Geom2d_Curve.ResD3 aResult;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line2D)myCurveData!, out aResult.Point, out aResult.D1);
                aResult.D2 = new Vector2D();
                aResult.D2.SetCoord(0.0, 0.0);
                aResult.D3 = new Vector2D();
                aResult.D3.SetCoord(0.0, 0.0);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Circle2D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Ellipse2D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Hyperbola2D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Parabola2D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                aResult.D3 = new Vector2D();
                aResult.D3.SetCoord(0.0, 0.0);
                return aResult;

            case BezierCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD3(U);
                    }
                    BezierData aBezierData = (BezierData)myCurveData!;
                    if (aBezierData.Cache is null)
                    {
                        RebuildCache(U);
                    }
                    aBezierData.Cache!.D3(U, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                    return aResult;
                }

            case BSplineCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD3(U);
                    }
                    BSplineData aBSplineData = (BSplineData)myCurveData!;
                    if (IsBoundary(U, out int aStart, out int aFinish))
                    {
                        aBSplineData.Curve!
                          .LocalD3(U, aStart, aFinish, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                    }
                    else
                    {
                        if (aBSplineData.Cache is null || !aBSplineData.Cache.IsCacheValid(U))
                        {
                            RebuildCache(U);
                        }
                        aBSplineData.Cache!.D3(U, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                    }
                    return aResult;
                }

            case OffsetCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalD3(U);
                    }
                    OffsetData anOffsetData = (OffsetData)myCurveData!;
                    if (!Geom2d_OffsetCurveUtils.EvaluateD3(U,
                                                            anOffsetData.BasisAdaptor!,
                                                            anOffsetData.Offset,
                                                            out aResult.Point,
                                                            out aResult.D1,
                                                            out aResult.D2,
                                                            out aResult.D3))
                    {
                        throw new Geom2d_UndefinedDerivative("Geom2dAdaptor_Curve::EvalD3: evaluation failed");
                    }
                    return aResult;
                }

            default:
                return myCurve!.EvalD3(U);
        }
    }

    /// <summary>The derivative of order N at U.</summary>
    public sealed override Vector2D DN(double U, int N)
    {
        return EvalDN(U, N);
    }

    /// <summary>The derivative of order N at U.</summary>
    public sealed override Vector2D EvalDN(double theU, int theN)
    {
        double U = theU;
        int N = theN;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Line2D)myCurveData!, N);

            case global::Kerf.Geometry.CurveKind.Circle:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Circle2D)myCurveData!, N);

            case global::Kerf.Geometry.CurveKind.Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Ellipse2D)myCurveData!, N);

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Hyperbola2D)myCurveData!, N);

            case global::Kerf.Geometry.CurveKind.Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Parabola2D)myCurveData!, N);

            case BezierCurve:
                return myCurve!.EvalDN(U, N);

            case BSplineCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalDN(U, N);
                    }
                    if (IsBoundary(U, out int aStart, out int aFinish))
                    {
                        return ((BSplineData)myCurveData!).Curve!.LocalDN(U, aStart, aFinish, N);
                    }
                    else
                    {
                        return myCurve!.EvalDN(U, N);
                    }
                }

            case OffsetCurve:
                {
                    if (hasEvalRep(myCurveData))
                    {
                        return myCurve!.EvalDN(U, N);
                    }
                    OffsetData anOffsetData = (OffsetData)myCurveData!;
                    Vector2D aDN;
                    if (!Geom2d_OffsetCurveUtils.EvaluateDN(U, anOffsetData.BasisAdaptor!, anOffsetData.Offset, N, out aDN))
                    {
                        throw new Geom2d_UndefinedDerivative("Geom2dAdaptor_Curve::EvalDN: evaluation failed");
                    }
                    return aDN;
                }

            default:
                break;
        }
        return myCurve!.EvalDN(U, N);
    }

    /// <summary>The parametric resolution corresponding to the 2D resolution Ruv.</summary>
    public override double Resolution(double Ruv)
    {
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return Ruv;
            case global::Kerf.Geometry.CurveKind.Circle:
                {
                    double R = ((Circle2D)myCurveData!).Radius();
                    if (R > Ruv / 2.0)
                    {
                        return 2 * Math.Asin(Ruv / (2 * R));
                    }
                    else
                    {
                        return 2 * M_PI;
                    }
                }
            case global::Kerf.Geometry.CurveKind.Ellipse:
                {
                    return Ruv / ((Ellipse2D)myCurveData!).MajorRadius();
                }
            case BezierCurve:
                {
                    double res;
                    ((Geom2d_BezierCurve)myCurve!).Resolution(Ruv, out res);
                    return res;
                }
            case BSplineCurve:
                {
                    double res;
                    ((Geom2d_BSplineCurve)myCurve!).Resolution(Ruv, out res);
                    return res;
                }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(Ruv);
        }
    }

    public override CurveKind GetType() => myTypeCurve;

    //    --
    //    --     The following methods must  be called when GetType returned
    //    --     the corresponding type.
    //    --

    public override Line2D Line()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Line,
                                       "Geom2dAdaptor_Curve::Line() - curve is not a Line");
        return (Line2D)myCurveData!;
    }

    public override Circle2D Circle()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Circle,
                                       "Geom2dAdaptor_Curve::Circle() - curve is not a Circle");
        return (Circle2D)myCurveData!;
    }

    public override Ellipse2D Ellipse()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Ellipse,
                                       "Geom2dAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return (Ellipse2D)myCurveData!;
    }

    public override Hyperbola2D Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Hyperbola,
                                       "Geom2dAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return (Hyperbola2D)myCurveData!;
    }

    public override Parabola2D Parabola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Parabola,
                                       "Geom2dAdaptor_Curve::Parabola() - curve is not a Parabola");
        return (Parabola2D)myCurveData!;
    }

    public override int Degree()
    {
        if (myTypeCurve == BezierCurve)
        {
            return ((Geom2d_BezierCurve)myCurve!).Degree();
        }
        else if (myTypeCurve == BSplineCurve)
        {
            return ((BSplineData)myCurveData!).Curve!.Degree();
        }
        else
        {
            throw new NoSuchObjectException();
        }
    }

    public override bool IsRational()
    {
        switch (myTypeCurve)
        {
            case BSplineCurve:
                return ((BSplineData)myCurveData!).Curve!.IsRational();
            case BezierCurve:
                return ((Geom2d_BezierCurve)myCurve!).IsRational();
            default:
                return false;
        }
    }

    public override int NbPoles()
    {
        if (myTypeCurve == BezierCurve)
        {
            return ((Geom2d_BezierCurve)myCurve!).NbPoles();
        }
        else if (myTypeCurve == BSplineCurve)
        {
            return ((BSplineData)myCurveData!).Curve!.NbPoles();
        }
        else
        {
            throw new NoSuchObjectException();
        }
    }

    public override int NbKnots()
    {
        if (myTypeCurve != BSplineCurve)
        {
            throw new NoSuchObjectException("Geom2dAdaptor_Curve::NbKnots");
        }
        return ((BSplineData)myCurveData!).Curve!.NbKnots();
    }

    /// <summary>The adapted curve as a Bezier curve, or null if it isn't one.</summary>
    public override Geom2d_BezierCurve Bezier()
    {
        // OCCT returns a null handle for another curve type; the base's non-nullable signature is kept.
        return (myCurve as Geom2d_BezierCurve)!;
    }

    /// <summary>The adapted curve as a B-spline curve, or null if it isn't one.</summary>
    public override Geom2d_BSplineCurve BSpline()
    {
        if (myCurveData is BSplineData aBSplineData)
        {
            return aBSplineData.Curve!;
        }
        return null!;
    }

    private static int nbPoints(Geom2d_Curve theCurve)
    {

        int nbs = 20;

        if (theCurve.IsKind<Geom2d_Line>())
        {
            nbs = 2;
        }
        else if (theCurve.IsKind<Geom2d_BezierCurve>())
        {
            nbs = 3 + ((Geom2d_BezierCurve)theCurve).NbPoles();
        }
        else if (theCurve.IsKind<Geom2d_BSplineCurve>())
        {
            nbs = ((Geom2d_BSplineCurve)theCurve).NbKnots();
            nbs *= ((Geom2d_BSplineCurve)theCurve).Degree();
            if (nbs < 2.0)
            {
                nbs = 2;
            }
        }
        else if (theCurve.IsKind<Geom2d_OffsetCurve>())
        {
            Geom2d_Curve aCurve = ((Geom2d_OffsetCurve)theCurve).BasisCurve();
            return Max(nbs, nbPoints(aCurve));
        }

        else if (theCurve.IsKind<Geom2d_TrimmedCurve>())
        {
            Geom2d_Curve aCurve = ((Geom2d_TrimmedCurve)theCurve).BasisCurve();
            return Max(nbs, nbPoints(aCurve));
        }
        if (nbs > 300)
        {
            nbs = 300;
        }
        return nbs;
    }

    public override int NbSamples() => nbPoints(myCurve!);
}
