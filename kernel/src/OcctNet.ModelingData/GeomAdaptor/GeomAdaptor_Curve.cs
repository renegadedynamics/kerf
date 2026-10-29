// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Curve.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// An interface between the services provided by any curve from the package Geom and those required of the
/// curve by algorithms which use it. Creation of the loaded curve: the curve is C1 by piece.
/// </summary>
/// <remarks>
/// <para>Polynomial coefficients of B-spline and Bezier curves are cached in the adaptor (a
/// <see cref="BSplCLib_Cache"/>, rebuilt when the evaluated span changes), so evaluating one adaptor from several
/// threads at once is not safe; give each thread its own (<see cref="ShallowCopy"/> doesn't copy the cache).</para>
/// <para>A trimmed curve is replaced by its basis curve (with the given bounds); the curve, its basis curves and
/// the B-spline or Bezier curve returned by <see cref="BSpline"/> or <see cref="Bezier"/> are the shared, frozen
/// geometry: the adaptor never changes it.</para>
/// </remarks>
public class GeomAdaptor_Curve : Adaptor3d_Curve
{
    private static readonly double PosTol = Precision.PConfusion() / 2;

    /// <summary>Evaluation data of an offset curve.</summary>
    internal sealed class OffsetData
    {
        /// <summary>Adaptor for the basis curve.</summary>
        public GeomAdaptor_Curve BasisAdaptor = null!;

        /// <summary>Offset distance.</summary>
        public double Offset;

        /// <summary>Offset direction.</summary>
        public gp_Dir Direction;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public GeomEval_RepCurveDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a Bezier curve.</summary>
    internal sealed class BezierData
    {
        /// <summary>The Bezier curve (to prevent downcasts).</summary>
        public Geom_BezierCurve Curve = null!;

        /// <summary>Cached data for evaluation.</summary>
        public BSplCLib_Cache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public GeomEval_RepCurveDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a B-spline curve.</summary>
    internal sealed class BSplineData
    {
        /// <summary>The B-spline curve (to prevent downcasts).</summary>
        public Geom_BSplineCurve Curve = null!;

        /// <summary>Cached data for evaluation.</summary>
        public BSplCLib_Cache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public GeomEval_RepCurveDesc.Base? EvalRep;
    }

    private Geom_Curve? myCurve;
    private GeomAbs_CurveType myTypeCurve;
    private double myFirst;
    private double myLast;

    // OCCT's std::variant<monostate, gp_Lin, gp_Circ, gp_Elips, gp_Hypr, gp_Parab, OffsetData, BezierData,
    // BSplineData>: null, a boxed gp_* value or one of the data classes.
    private object? myCurveData;

    /// <summary>An adaptor without a curve.</summary>
    public GeomAdaptor_Curve()
    {
        myTypeCurve = GeomAbs_OtherCurve;
        myFirst = 0.0;
        myLast = 0.0;
    }

    /// <summary>An adaptor of theCurve over its whole parameter range.</summary>
    /// <exception cref="Standard_NullObject">theCurve is null.</exception>
    public GeomAdaptor_Curve(Geom_Curve theCurve)
    {
        Load(theCurve);
    }

    /// <summary>An adaptor of theCurve between theUFirst and theULast.</summary>
    /// <exception cref="Standard_ConstructionError">theUFirst &gt; theULast + Precision::Confusion().</exception>
    public GeomAdaptor_Curve(Geom_Curve theCurve, double theUFirst, double theULast)
    {
        Load(theCurve, theUFirst, theULast);
    }

    /// <summary>A shallow copy: the same curve, a new evaluation cache.</summary>
    public override Adaptor3d_Curve ShallowCopy()
    {
        GeomAdaptor_Curve aCopy = new GeomAdaptor_Curve();

        aCopy.myCurve = myCurve;
        aCopy.myTypeCurve = myTypeCurve;
        aCopy.myFirst = myFirst;
        aCopy.myLast = myLast;

        // Copy curve-specific data based on variant type
        if (myCurveData is OffsetData anOffsetData)
        {
            OffsetData aCopyData = new OffsetData();
            aCopyData.BasisAdaptor = (GeomAdaptor_Curve)anOffsetData.BasisAdaptor.ShallowCopy();
            aCopyData.Offset = anOffsetData.Offset;
            aCopyData.Direction = anOffsetData.Direction;
            aCopyData.EvalRep = anOffsetData.EvalRep;
            aCopy.myCurveData = aCopyData;
        }
        else if (myCurveData is BSplineData aBSplineData)
        {
            BSplineData aCopyData = new BSplineData();
            aCopyData.Curve = aBSplineData.Curve;
            aCopyData.EvalRep = aBSplineData.EvalRep;
            aCopy.myCurveData = aCopyData;
        }
        else if (myCurveData is BezierData aBezierData)
        {
            BezierData aCopyData = new BezierData();
            aCopyData.Curve = aBezierData.Curve;
            aCopyData.EvalRep = aBezierData.EvalRep;
            // Cache is not copied - will be rebuilt on demand.
            aCopy.myCurveData = aCopyData;
        }
        else
        {
            // Elementary curve types (gp_Lin, gp_Circ, etc.) are value types - direct copy (boxed values are
            // never changed in place)
            aCopy.myCurveData = myCurveData;
        }

        return aCopy;
    }

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

    // Computes the Continuity of a BSplineCurve between the parameters U1 and U2. The continuity is C(d-m)
    // with d = degree, m = max multiplicity of the Knots between U1 and U2
    private GeomAbs_Shape LocalContinuity(double U1, double U2)
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_BSplineCurve, " ");
        Geom_BSplineCurve aBSpl = ((BSplineData)myCurveData!).Curve;
        int Nb = aBSpl.NbKnots();
        int Index1;
        int Index2;
        double newFirst, newLast;
        NCollection_Array1<double> TK = aBSpl.InternalKnots();
        NCollection_Array1<int> TM = aBSpl.InternalMultiplicities();
        BSplCLib.LocateParameter(aBSpl.Degree(), TK, TM, U1, aBSpl.IsPeriodic(), 1, Nb, out Index1, out newFirst);
        BSplCLib.LocateParameter(aBSpl.Degree(), TK, TM, U2, aBSpl.IsPeriodic(), 1, Nb, out Index2, out newLast);
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

    /// <summary>Removes the curve: the adaptor is as newly constructed.</summary>
    public void Reset()
    {
        myTypeCurve = GeomAbs_OtherCurve;
        myCurve = null;
        myCurveData = null;
        myFirst = myLast = 0.0;
    }

    /// <summary>Loads theCurve over its whole parameter range.</summary>
    /// <exception cref="Standard_NullObject">theCurve is null.</exception>
    public void Load(Geom_Curve theCurve)
    {
        if (theCurve is null)
        {
            throw new Standard_NullObject();
        }
        load(theCurve, theCurve.FirstParameter(), theCurve.LastParameter());
    }

    /// <summary>Loads theCurve between theUFirst and theULast.</summary>
    /// <exception cref="Standard_NullObject">theCurve is null.</exception>
    /// <exception cref="Standard_ConstructionError">theUFirst &gt; theULast + Precision::Confusion().</exception>
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

    /// <summary>The adapted curve (the basis curve for a trimmed curve), shared: null before a Load.</summary>
    public Geom_Curve? Curve() => myCurve;

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    private void load(Geom_Curve C, double UFirst, double ULast)
    {
        myFirst = UFirst;
        myLast = ULast;

        if (!ReferenceEquals(myCurve, C))
        {
            myCurve = C;
            myCurveData = null;

            Type TheType = C.DynamicType();
            if (TheType == typeof(Geom_TrimmedCurve))
            {
                Load(((Geom_TrimmedCurve)C).BasisCurve(), UFirst, ULast);
            }
            else if (TheType == typeof(Geom_Circle))
            {
                myTypeCurve = GeomAbs_Circle;
                myCurveData = ((Geom_Circle)C).Circ();
            }
            else if (TheType == typeof(Geom_Line))
            {
                myTypeCurve = GeomAbs_Line;
                myCurveData = ((Geom_Line)C).Lin();
            }
            else if (TheType == typeof(Geom_Ellipse))
            {
                myTypeCurve = GeomAbs_Ellipse;
                myCurveData = ((Geom_Ellipse)C).Elips();
            }
            else if (TheType == typeof(Geom_Parabola))
            {
                myTypeCurve = GeomAbs_Parabola;
                myCurveData = ((Geom_Parabola)C).Parab();
            }
            else if (TheType == typeof(Geom_Hyperbola))
            {
                myTypeCurve = GeomAbs_Hyperbola;
                myCurveData = ((Geom_Hyperbola)C).Hypr();
            }
            else if (TheType == typeof(Geom_BezierCurve))
            {
                myTypeCurve = GeomAbs_BezierCurve;
                BezierData aBezierData = new BezierData();
                aBezierData.Curve = (Geom_BezierCurve)C;
                aBezierData.EvalRep = aBezierData.Curve.EvalRepresentation();
                myCurveData = aBezierData;
            }
            else if (TheType == typeof(Geom_BSplineCurve))
            {
                myTypeCurve = GeomAbs_BSplineCurve;
                BSplineData aBSplineData = new BSplineData();
                aBSplineData.Curve = (Geom_BSplineCurve)C;
                aBSplineData.EvalRep = aBSplineData.Curve.EvalRepresentation();
                myCurveData = aBSplineData;
            }
            else if (TheType == typeof(Geom_OffsetCurve))
            {
                myTypeCurve = GeomAbs_OffsetCurve;
                Geom_OffsetCurve anOffsetCurve = (Geom_OffsetCurve)C;
                OffsetData anOffsetData = new OffsetData();
                anOffsetData.BasisAdaptor = new GeomAdaptor_Curve(anOffsetCurve.BasisCurve());
                anOffsetData.Offset = anOffsetCurve.Offset();
                anOffsetData.Direction = anOffsetCurve.Direction();
                anOffsetData.EvalRep = anOffsetCurve.EvalRepresentation();
                myCurveData = anOffsetData;
            }
            else
            {
                myTypeCurve = GeomAbs_OtherCurve;
            }
        }
        else
        {
            // Same curve, but need to invalidate cache if bounds changed
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

    /// <exception cref="Standard_NoSuchObject">The curve is not a Geom curve this adaptor knows.</exception>
    public override GeomAbs_Shape Continuity()
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return LocalContinuity(myFirst, myLast);
        }

        if (myTypeCurve == GeomAbs_OffsetCurve)
        {
            GeomAbs_Shape S = ((Geom_OffsetCurve)myCurve!).GetBasisCurveContinuity();
            switch (S)
            {
                case GeomAbs_CN:
                    return GeomAbs_CN;
                case GeomAbs_C3:
                    return GeomAbs_C2;
                case GeomAbs_C2:
                    return GeomAbs_C1;
                case GeomAbs_C1:
                    return GeomAbs_C0;
                case GeomAbs_G1:
                    return GeomAbs_G1;
                case GeomAbs_G2:
                    return GeomAbs_G2;
                default:
                    throw new Standard_NoSuchObject("GeomAdaptor_Curve::Continuity");
            }
        }
        else if (myTypeCurve == GeomAbs_OtherCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::Contunuity");
        }

        return GeomAbs_CN;
    }

    /// <exception cref="Standard_DomainError">S is G1 or G2 on a B-spline or offset curve.</exception>
    public override int NbIntervals(GeomAbs_Shape S)
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = ((BSplineData)myCurveData!).Curve;
            if ((!aBSpl.IsPeriodic() && S <= Continuity()) || S == GeomAbs_C0)
            {
                return 1;
            }

            int aDegree = aBSpl.Degree();
            int aCont;

            switch (S)
            {
                case GeomAbs_C1:
                    aCont = 1;
                    break;
                case GeomAbs_C2:
                    aCont = 2;
                    break;
                case GeomAbs_C3:
                    aCont = 3;
                    break;
                case GeomAbs_CN:
                    aCont = aDegree;
                    break;
                default:
                    throw new Standard_DomainError("GeomAdaptor_Curve::NbIntervals()");
            }

            double anEps = Math.Min(Resolution(Precision.Confusion()), Precision.PConfusion());

            return BSplCLib.Intervals(aBSpl.InternalKnots(),
                                      aBSpl.InternalMultiplicities(),
                                      aDegree,
                                      aBSpl.IsPeriodic(),
                                      aCont,
                                      myFirst,
                                      myLast,
                                      anEps,
                                      null);
        }

        else if (myTypeCurve == GeomAbs_OffsetCurve)
        {
            int myNbIntervals = 1;
            GeomAbs_Shape BaseS;
            switch (S)
            {
                case GeomAbs_G1:
                case GeomAbs_G2:
                    throw new Standard_DomainError("GeomAdaptor_Curve::NbIntervals");
                case GeomAbs_C0:
                    BaseS = GeomAbs_C1;
                    break;
                case GeomAbs_C1:
                    BaseS = GeomAbs_C2;
                    break;
                case GeomAbs_C2:
                    BaseS = GeomAbs_C3;
                    break;
                default:
                    BaseS = GeomAbs_CN;
                    break;
            }
            GeomAdaptor_Curve C = new GeomAdaptor_Curve(((Geom_OffsetCurve)myCurve!).BasisCurve(), myFirst, myLast);
            // akm 05/04/02 (OCC278)  If our curve is trimmed we must recalculate
            //                    the number of intervals obtained from the basis to
            //              vvv   reflect parameter bounds
            int iNbBasisInt = C.NbIntervals(BaseS), iInt;
            if (iNbBasisInt > 1)
            {
                NCollection_Array1<double> rdfInter = new(1, 1 + iNbBasisInt);
                C.Intervals(rdfInter, BaseS);
                for (iInt = 1; iInt <= iNbBasisInt; iInt++)
                {
                    if (rdfInter[iInt] > myFirst && rdfInter[iInt] < myLast)
                    {
                        myNbIntervals++;
                    }
                }
            }
            // akm 05/04/02 ^^^
            return myNbIntervals;
        }

        else
        {
            return 1;
        }
    }

    /// <exception cref="Standard_DomainError">S is G1 or G2 on a B-spline or offset curve.</exception>
    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = ((BSplineData)myCurveData!).Curve;
            if ((!aBSpl.IsPeriodic() && S <= Continuity()) || S == GeomAbs_C0)
            {
                T[T.Lower()] = myFirst;
                T[T.Lower() + 1] = myLast;
                return;
            }

            int aDegree = aBSpl.Degree();
            int aCont;

            switch (S)
            {
                case GeomAbs_C1:
                    aCont = 1;
                    break;
                case GeomAbs_C2:
                    aCont = 2;
                    break;
                case GeomAbs_C3:
                    aCont = 3;
                    break;
                case GeomAbs_CN:
                    aCont = aDegree;
                    break;
                default:
                    throw new Standard_DomainError("GeomAdaptor_Curve::Intervals()");
            }

            double anEps = Math.Min(Resolution(Precision.Confusion()), Precision.PConfusion());

            BSplCLib.Intervals(aBSpl.InternalKnots(),
                               aBSpl.InternalMultiplicities(),
                               aDegree,
                               aBSpl.IsPeriodic(),
                               aCont,
                               myFirst,
                               myLast,
                               anEps,
                               T);
        }

        else if (myTypeCurve == GeomAbs_OffsetCurve)
        {
            int myNbIntervals = 1;
            GeomAbs_Shape BaseS;
            switch (S)
            {
                case GeomAbs_G1:
                case GeomAbs_G2:
                    throw new Standard_DomainError("GeomAdaptor_Curve::NbIntervals");
                case GeomAbs_C0:
                    BaseS = GeomAbs_C1;
                    break;
                case GeomAbs_C1:
                    BaseS = GeomAbs_C2;
                    break;
                case GeomAbs_C2:
                    BaseS = GeomAbs_C3;
                    break;
                default:
                    BaseS = GeomAbs_CN;
                    break;
            }
            GeomAdaptor_Curve C = new GeomAdaptor_Curve(((Geom_OffsetCurve)myCurve!).BasisCurve(), myFirst, myLast);
            // akm 05/04/02 (OCC278)  If our curve is trimmed we must recalculate
            //                    the array of intervals obtained from the basis to
            //              vvv   reflect parameter bounds
            int iNbBasisInt = C.NbIntervals(BaseS), iInt;
            if (iNbBasisInt > 1)
            {
                NCollection_Array1<double> rdfInter = new(1, 1 + iNbBasisInt);
                C.Intervals(rdfInter, BaseS);
                for (iInt = 1; iInt <= iNbBasisInt; iInt++)
                {
                    if (rdfInter[iInt] > myFirst && rdfInter[iInt] < myLast)
                    {
                        T[++myNbIntervals] = rdfInter[iInt];
                    }
                }
            }
            // old - myNbIntervals = C.NbIntervals(BaseS);
            // old - C.Intervals(T, BaseS);
            // akm 05/04/02 ^^^
            T[T.Lower()] = myFirst;
            T[T.Lower() + myNbIntervals] = myLast;
        }

        else
        {
            T[T.Lower()] = myFirst;
            T[T.Lower() + 1] = myLast;
        }
    }

    /// <summary>A new adaptor of the same curve between First and Last.</summary>
    public override Adaptor3d_Curve Trim(double First, double Last, double Tol)
    {
        return new GeomAdaptor_Curve(myCurve!, First, Last);
    }

    /// <summary>True if the bounds are finite and their points are within Precision::Confusion().</summary>
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

    /// <summary>The curve's LastParameter() - FirstParameter() (whether periodic or not, as in OCCT).</summary>
    public override double Period() => myCurve!.LastParameter() - myCurve.FirstParameter();

    private void RebuildCache(double theParameter)
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            // Create cache for Bezier
            BezierData aBezData = (BezierData)myCurveData!;
            Geom_BezierCurve aBezier = (Geom_BezierCurve)myCurve!;
            int aDeg = aBezier.Degree();
            NCollection_Array1<double> aFlatKnots = NCollection_Array1<double>.From(BSplCLib.FlatBezierKnots(aDeg));
            if (aBezData.Cache is null)
            {
                aBezData.Cache = new BSplCLib_Cache(aDeg,
                                                    aBezier.IsPeriodic(),
                                                    aFlatKnots,
                                                    aBezier.InternalPoles(),
                                                    aBezier.InternalWeights());
            }
            aBezData.Cache.BuildCache(theParameter, aFlatKnots, aBezier.InternalPoles(), aBezier.InternalWeights());
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            // Create cache for B-spline
            BSplineData aBSplData = (BSplineData)myCurveData!;
            Geom_BSplineCurve aBSpl = aBSplData.Curve;
            if (aBSplData.Cache is null)
            {
                aBSplData.Cache = new BSplCLib_Cache(aBSpl.Degree(),
                                                     aBSpl.IsPeriodic(),
                                                     aBSpl.InternalKnotSequence(),
                                                     aBSpl.InternalPoles(),
                                                     aBSpl.InternalWeights());
            }
            aBSplData.Cache.BuildCache(theParameter,
                                       aBSpl.InternalKnotSequence(),
                                       aBSpl.InternalPoles(),
                                       aBSpl.InternalWeights());
        }
    }

    private bool IsBoundary(double theU, out int theSpanStart, out int theSpanFinish)
    {
        theSpanStart = 0;
        theSpanFinish = 0;
        if (myCurveData is BSplineData aBSplData && (theU == myFirst || theU == myLast))
        {
            Geom_BSplineCurve aBSpl = aBSplData.Curve;
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

    /// <exception cref="Geom_UndefinedValue">The point of an offset curve is undefined at theU.</exception>
    public sealed override gp_Pnt EvalD0(double theU)
    {
        double U = theU;
        gp_Pnt P;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                P = ElCLib.Value(U, (gp_Lin)myCurveData!);
                return P;

            case GeomAbs_Circle:
                P = ElCLib.Value(U, (gp_Circ)myCurveData!);
                return P;

            case GeomAbs_Ellipse:
                P = ElCLib.Value(U, (gp_Elips)myCurveData!);
                return P;

            case GeomAbs_Hyperbola:
                P = ElCLib.Value(U, (gp_Hypr)myCurveData!);
                return P;

            case GeomAbs_Parabola:
                P = ElCLib.Value(U, (gp_Parab)myCurveData!);
                return P;

            case GeomAbs_BezierCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD0(U);
                }
                BezierData aBezData = (BezierData)myCurveData!;
                if (aBezData.Cache is null)
                {
                    RebuildCache(U);
                }
                aBezData.Cache!.D0(U, out P);
                return P;
            }

            case GeomAbs_BSplineCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD0(U);
                }
                BSplineData aBSplData = (BSplineData)myCurveData!;
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    aBSplData.Curve.LocalD0(U, aStart, aFinish, out P);
                }
                else
                {
                    if (aBSplData.Cache is null || !aBSplData.Cache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    aBSplData.Cache!.D0(U, out P);
                }
                return P;
            }

            case GeomAbs_OffsetCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD0(U);
                }
                OffsetData anOffsetData = (OffsetData)myCurveData!;
                if (!Geom_OffsetCurveUtils.EvaluateD0(U,
                                                      anOffsetData.BasisAdaptor,
                                                      anOffsetData.Direction,
                                                      anOffsetData.Offset,
                                                      out P))
                {
                    throw new Geom_UndefinedValue("GeomAdaptor_Curve::EvalD0: evaluation failed");
                }
                return P;
            }

            default:
                return myCurve!.EvalD0(U);
        }
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivative of an offset curve is undefined at theU.</exception>
    public sealed override Geom_Curve.ResD1 EvalD1(double theU)
    {
        double U = theU;
        Geom_Curve.ResD1 aResult;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, (gp_Lin)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case GeomAbs_Circle:
                ElCLib.D1(U, (gp_Circ)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case GeomAbs_Ellipse:
                ElCLib.D1(U, (gp_Elips)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case GeomAbs_Hyperbola:
                ElCLib.D1(U, (gp_Hypr)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case GeomAbs_Parabola:
                ElCLib.D1(U, (gp_Parab)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case GeomAbs_BezierCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD1(U);
                }
                BezierData aBezData = (BezierData)myCurveData!;
                if (aBezData.Cache is null)
                {
                    RebuildCache(U);
                }
                aBezData.Cache!.D1(U, out aResult.Point, out aResult.D1);
                return aResult;
            }

            case GeomAbs_BSplineCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD1(U);
                }
                BSplineData aBSplData = (BSplineData)myCurveData!;
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    aBSplData.Curve.LocalD1(U, aStart, aFinish, out aResult.Point, out aResult.D1);
                }
                else
                {
                    if (aBSplData.Cache is null || !aBSplData.Cache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    aBSplData.Cache!.D1(U, out aResult.Point, out aResult.D1);
                }
                return aResult;
            }

            case GeomAbs_OffsetCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD1(U);
                }
                OffsetData anOffsetData = (OffsetData)myCurveData!;
                if (!Geom_OffsetCurveUtils.EvaluateD1(U,
                                                      anOffsetData.BasisAdaptor,
                                                      anOffsetData.Direction,
                                                      anOffsetData.Offset,
                                                      out aResult.Point,
                                                      out aResult.D1))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Curve::EvalD1: evaluation failed");
                }
                return aResult;
            }

            default:
                return myCurve!.EvalD1(U);
        }
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives of an offset curve are undefined at theU.</exception>
    public sealed override Geom_Curve.ResD2 EvalD2(double theU)
    {
        double U = theU;
        Geom_Curve.ResD2 aResult;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, (gp_Lin)myCurveData!, out aResult.Point, out aResult.D1);
                aResult.D2 = new gp_Vec(0.0, 0.0, 0.0);
                return aResult;

            case GeomAbs_Circle:
                ElCLib.D2(U, (gp_Circ)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case GeomAbs_Ellipse:
                ElCLib.D2(U, (gp_Elips)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case GeomAbs_Hyperbola:
                ElCLib.D2(U, (gp_Hypr)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case GeomAbs_Parabola:
                ElCLib.D2(U, (gp_Parab)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case GeomAbs_BezierCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD2(U);
                }
                BezierData aBezData = (BezierData)myCurveData!;
                if (aBezData.Cache is null)
                {
                    RebuildCache(U);
                }
                aBezData.Cache!.D2(U, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;
            }

            case GeomAbs_BSplineCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD2(U);
                }
                BSplineData aBSplData = (BSplineData)myCurveData!;
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    aBSplData.Curve.LocalD2(U, aStart, aFinish, out aResult.Point, out aResult.D1, out aResult.D2);
                }
                else
                {
                    if (aBSplData.Cache is null || !aBSplData.Cache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    aBSplData.Cache!.D2(U, out aResult.Point, out aResult.D1, out aResult.D2);
                }
                return aResult;
            }

            case GeomAbs_OffsetCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD2(U);
                }
                OffsetData anOffsetData = (OffsetData)myCurveData!;
                if (!Geom_OffsetCurveUtils.EvaluateD2(U,
                                                      anOffsetData.BasisAdaptor,
                                                      anOffsetData.Direction,
                                                      anOffsetData.Offset,
                                                      out aResult.Point,
                                                      out aResult.D1,
                                                      out aResult.D2))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Curve::EvalD2: evaluation failed");
                }
                return aResult;
            }

            default:
                return myCurve!.EvalD2(U);
        }
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivatives of an offset curve are undefined at theU.</exception>
    public sealed override Geom_Curve.ResD3 EvalD3(double theU)
    {
        double U = theU;
        Geom_Curve.ResD3 aResult;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                ElCLib.D1(U, (gp_Lin)myCurveData!, out aResult.Point, out aResult.D1);
                aResult.D2 = new gp_Vec(0.0, 0.0, 0.0);
                aResult.D3 = new gp_Vec(0.0, 0.0, 0.0);
                return aResult;

            case GeomAbs_Circle:
                ElCLib.D3(U, (gp_Circ)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case GeomAbs_Ellipse:
                ElCLib.D3(U, (gp_Elips)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case GeomAbs_Hyperbola:
                ElCLib.D3(U, (gp_Hypr)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case GeomAbs_Parabola:
                ElCLib.D2(U, (gp_Parab)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                aResult.D3 = new gp_Vec(0.0, 0.0, 0.0);
                return aResult;

            case GeomAbs_BezierCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD3(U);
                }
                BezierData aBezData = (BezierData)myCurveData!;
                if (aBezData.Cache is null)
                {
                    RebuildCache(U);
                }
                aBezData.Cache!.D3(U, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;
            }

            case GeomAbs_BSplineCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD3(U);
                }
                BSplineData aBSplData = (BSplineData)myCurveData!;
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    aBSplData.Curve.LocalD3(U,
                                            aStart,
                                            aFinish,
                                            out aResult.Point,
                                            out aResult.D1,
                                            out aResult.D2,
                                            out aResult.D3);
                }
                else
                {
                    if (aBSplData.Cache is null || !aBSplData.Cache.IsCacheValid(U))
                    {
                        RebuildCache(U);
                    }
                    aBSplData.Cache!.D3(U, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                }
                return aResult;
            }

            case GeomAbs_OffsetCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalD3(U);
                }
                OffsetData anOffsetData = (OffsetData)myCurveData!;
                if (!Geom_OffsetCurveUtils.EvaluateD3(U,
                                                      anOffsetData.BasisAdaptor,
                                                      anOffsetData.Direction,
                                                      anOffsetData.Offset,
                                                      out aResult.Point,
                                                      out aResult.D1,
                                                      out aResult.D2,
                                                      out aResult.D3))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Curve::EvalD3: evaluation failed");
                }
                return aResult;
            }

            default:
                return myCurve!.EvalD3(U);
        }
    }

    /// <exception cref="Geom_UndefinedDerivative">The derivative is undefined at theU, or theN &lt; 1.</exception>
    public sealed override gp_Vec EvalDN(double theU, int theN)
    {
        double U = theU;
        int N = theN;
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                return ElCLib.DN(U, (gp_Lin)myCurveData!, N);

            case GeomAbs_Circle:
                return ElCLib.DN(U, (gp_Circ)myCurveData!, N);

            case GeomAbs_Ellipse:
                return ElCLib.DN(U, (gp_Elips)myCurveData!, N);

            case GeomAbs_Hyperbola:
                return ElCLib.DN(U, (gp_Hypr)myCurveData!, N);

            case GeomAbs_Parabola:
                return ElCLib.DN(U, (gp_Parab)myCurveData!, N);

            case GeomAbs_BezierCurve:
                return myCurve!.EvalDN(U, N);

            case GeomAbs_BSplineCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalDN(U, N);
                }
                if (IsBoundary(U, out int aStart, out int aFinish))
                {
                    return ((BSplineData)myCurveData!).Curve.LocalDN(U, aStart, aFinish, N);
                }
                else
                {
                    return myCurve!.EvalDN(U, N);
                }
            }

            case GeomAbs_OffsetCurve:
            {
                if (hasEvalRep(myCurveData))
                {
                    return myCurve!.EvalDN(U, N);
                }
                OffsetData anOffsetData = (OffsetData)myCurveData!;
                if (!Geom_OffsetCurveUtils.EvaluateDN(U,
                                                      anOffsetData.BasisAdaptor,
                                                      anOffsetData.Direction,
                                                      anOffsetData.Offset,
                                                      N,
                                                      out gp_Vec aDN))
                {
                    throw new Geom_UndefinedDerivative("GeomAdaptor_Curve::EvalDN: evaluation failed");
                }
                return aDN;
            }

            default:
                break;
        }
        return myCurve!.EvalDN(U, N);
    }

    /// <summary>The parametric resolution corresponding to the 3D resolution R3D.</summary>
    public override double Resolution(double R3D)
    {
        switch (myTypeCurve)
        {
            case GeomAbs_Line:
                return R3D;
            case GeomAbs_Circle:
            {
                double R = ((gp_Circ)myCurveData!).Radius();
                if (R > R3D / 2.0)
                {
                    return 2 * Math.Asin(R3D / (2 * R));
                }
                else
                {
                    return 2 * Math.PI;
                }
            }
            case GeomAbs_Ellipse:
            {
                return R3D / ((gp_Elips)myCurveData!).MajorRadius();
            }
            case GeomAbs_BezierCurve:
            {
                ((Geom_BezierCurve)myCurve!).Resolution(R3D, out double res);
                return res;
            }
            case GeomAbs_BSplineCurve:
            {
                ((BSplineData)myCurveData!).Curve.Resolution(R3D, out double res);
                return res;
            }
            default:
                return Precision.Parametric(R3D);
        }
    }

    /// <summary>The type of the curve (GeomAbs_OtherCurve before a Load).</summary>
    public override GeomAbs_CurveType GetType() => myTypeCurve;

    //    --
    //    --     The following methods must  be called when GetType returned
    //    --     the corresponding type.
    //    --

    /// <exception cref="Standard_NoSuchObject">The curve is not a line.</exception>
    public override gp_Lin Line()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Line, "GeomAdaptor_Curve::Line() - curve is not a Line");
        return (gp_Lin)myCurveData!;
    }

    /// <exception cref="Standard_NoSuchObject">The curve is not a circle.</exception>
    public override gp_Circ Circle()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Circle,
                                       "GeomAdaptor_Curve::Circle() - curve is not a Circle");
        return (gp_Circ)myCurveData!;
    }

    /// <exception cref="Standard_NoSuchObject">The curve is not an ellipse.</exception>
    public override gp_Elips Ellipse()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Ellipse,
                                       "GeomAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return (gp_Elips)myCurveData!;
    }

    /// <exception cref="Standard_NoSuchObject">The curve is not a hyperbola.</exception>
    public override gp_Hypr Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Hyperbola,
                                       "GeomAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return (gp_Hypr)myCurveData!;
    }

    /// <exception cref="Standard_NoSuchObject">The curve is not a parabola.</exception>
    public override gp_Parab Parabola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != GeomAbs_Parabola,
                                       "GeomAdaptor_Curve::Parabola() - curve is not a Parabola");
        return (gp_Parab)myCurveData!;
    }

    /// <exception cref="Standard_NoSuchObject">The curve is neither a Bezier nor a B-spline curve.</exception>
    public override int Degree()
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            return ((Geom_BezierCurve)myCurve!).Degree();
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return ((BSplineData)myCurveData!).Curve.Degree();
        }
        else
        {
            throw new Standard_NoSuchObject();
        }
    }

    /// <summary>True for a rational Bezier or B-spline curve; false for any other curve.</summary>
    public override bool IsRational()
    {
        switch (myTypeCurve)
        {
            case GeomAbs_BSplineCurve:
                return ((BSplineData)myCurveData!).Curve.IsRational();
            case GeomAbs_BezierCurve:
                return ((Geom_BezierCurve)myCurve!).IsRational();
            default:
                return false;
        }
    }

    /// <exception cref="Standard_NoSuchObject">The curve is neither a Bezier nor a B-spline curve.</exception>
    public override int NbPoles()
    {
        if (myTypeCurve == GeomAbs_BezierCurve)
        {
            return ((Geom_BezierCurve)myCurve!).NbPoles();
        }
        else if (myTypeCurve == GeomAbs_BSplineCurve)
        {
            return ((BSplineData)myCurveData!).Curve.NbPoles();
        }
        else
        {
            throw new Standard_NoSuchObject();
        }
    }

    /// <exception cref="Standard_NoSuchObject">The curve is not a B-spline curve.</exception>
    public override int NbKnots()
    {
        if (myTypeCurve != GeomAbs_BSplineCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::NbKnots");
        }
        return ((BSplineData)myCurveData!).Curve.NbKnots();
    }

    /// <summary>The Bezier curve (shared, frozen when it was loaded from a trimmed curve).</summary>
    /// <exception cref="Standard_NoSuchObject">The curve is not a Bezier curve.</exception>
    public override Geom_BezierCurve Bezier()
    {
        if (myTypeCurve != GeomAbs_BezierCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::Bezier");
        }
        return (Geom_BezierCurve)myCurve!;
    }

    /// <summary>The B-spline curve (shared, frozen when it was loaded from a trimmed curve).</summary>
    /// <exception cref="Standard_NoSuchObject">The curve is not a B-spline curve.</exception>
    public override Geom_BSplineCurve BSpline()
    {
        if (myTypeCurve != GeomAbs_BSplineCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::BSpline");
        }

        return ((BSplineData)myCurveData!).Curve;
    }

    /// <summary>The offset curve (shared).</summary>
    /// <exception cref="Standard_NoSuchObject">The curve is not an offset curve.</exception>
    public override Geom_OffsetCurve OffsetCurve()
    {
        if (myTypeCurve != GeomAbs_OffsetCurve)
        {
            throw new Standard_NoSuchObject("GeomAdaptor_Curve::OffsetCurve");
        }
        return (Geom_OffsetCurve)myCurve!;
    }
}
