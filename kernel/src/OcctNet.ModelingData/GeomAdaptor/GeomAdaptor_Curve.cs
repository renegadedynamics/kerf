// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Curve.hxx
// Port of: src/ModelingData/TKG3d/GeomAdaptor/GeomAdaptor_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// An interface between the services provided by any curve from the package Geom and those required of the
/// curve by algorithms which use it. Creation of the loaded curve: the curve is C1 by piece.
/// </summary>
/// <remarks>
/// <para>Polynomial coefficients of B-spline and Bezier curves are cached in the adaptor (a
/// <see cref="Cache"/>, rebuilt when the evaluated span changes), so evaluating one adaptor from several
/// threads at once is not safe; give each thread its own (<see cref="ShallowCopy"/> doesn't copy the cache).</para>
/// <para>A trimmed curve is replaced by its basis curve (with the given bounds); the curve, its basis curves and
/// the B-spline or Bezier curve returned by <see cref="BSpline"/> or <see cref="Bezier"/> are the shared, frozen
/// geometry: the adaptor never changes it.</para>
/// </remarks>
public class GeomAdaptor_Curve : Adaptor3d_Curve
{
    private static readonly double PosTol = global::Kerf.Engine.Foundation.Precision.PConfusion() / 2;

    /// <summary>Evaluation data of an offset curve.</summary>
    internal sealed class OffsetData
    {
        /// <summary>Adaptor for the basis curve.</summary>
        public GeomAdaptor_Curve BasisAdaptor = null!;

        /// <summary>Offset distance.</summary>
        public double Offset;

        /// <summary>Offset direction.</summary>
        public Direction3D Direction;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public GeomEval_RepCurveDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a Bezier curve.</summary>
    internal sealed class BezierData
    {
        /// <summary>The Bezier curve (to prevent downcasts).</summary>
        public Geom_BezierCurve Curve = null!;

        /// <summary>Cached data for evaluation.</summary>
        public Cache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public GeomEval_RepCurveDesc.Base? EvalRep;
    }

    /// <summary>Evaluation data of a B-spline curve.</summary>
    internal sealed class BSplineData
    {
        /// <summary>The B-spline curve (to prevent downcasts).</summary>
        public Geom_BSplineCurve Curve = null!;

        /// <summary>Cached data for evaluation.</summary>
        public Cache? Cache;

        /// <summary>Evaluation representation descriptor (shared with the curve: don't modify).</summary>
        public GeomEval_RepCurveDesc.Base? EvalRep;
    }

    private Geom_Curve? myCurve;
    private CurveKind myTypeCurve;
    private double myFirst;
    private double myLast;

    // OCCT's std::variant<monostate, gp_Lin, gp_Circ, gp_Elips, gp_Hypr, gp_Parab, OffsetData, BezierData,
    // BSplineData>: null, a boxed gp_* value or one of the data classes.
    private object? myCurveData;

    /// <summary>An adaptor without a curve.</summary>
    public GeomAdaptor_Curve()
    {
        myTypeCurve = OtherCurve;
        myFirst = 0.0;
        myLast = 0.0;
    }

    /// <summary>An adaptor of theCurve over its whole parameter range.</summary>
    /// <exception cref="NullObjectException">theCurve is null.</exception>
    public GeomAdaptor_Curve(Geom_Curve theCurve)
    {
        Load(theCurve);
    }

    /// <summary>An adaptor of theCurve between theUFirst and theULast.</summary>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast + Precision::Confusion().</exception>
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
    private Continuity LocalContinuity(double U1, double U2)
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != BSplineCurve, " ");
        Geom_BSplineCurve aBSpl = ((BSplineData)myCurveData!).Curve;
        int Nb = aBSpl.NbKnots();
        int Index1;
        int Index2;
        double newFirst, newLast;
        NCollection_Array1<double> TK = aBSpl.InternalKnots();
        NCollection_Array1<int> TM = aBSpl.InternalMultiplicities();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aBSpl.Degree(), TK, TM, U1, aBSpl.IsPeriodic(), 1, Nb, out Index1, out newFirst);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(aBSpl.Degree(), TK, TM, U2, aBSpl.IsPeriodic(), 1, Nb, out Index2, out newLast);
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

    /// <summary>Removes the curve: the adaptor is as newly constructed.</summary>
    public void Reset()
    {
        myTypeCurve = OtherCurve;
        myCurve = null;
        myCurveData = null;
        myFirst = myLast = 0.0;
    }

    /// <summary>Loads theCurve over its whole parameter range.</summary>
    /// <exception cref="NullObjectException">theCurve is null.</exception>
    public void Load(Geom_Curve theCurve)
    {
        if (theCurve is null)
        {
            throw new NullObjectException();
        }
        load(theCurve, theCurve.FirstParameter(), theCurve.LastParameter());
    }

    /// <summary>Loads theCurve between theUFirst and theULast.</summary>
    /// <exception cref="NullObjectException">theCurve is null.</exception>
    /// <exception cref="ConstructionErrorException">theUFirst &gt; theULast + Precision::Confusion().</exception>
    public void Load(Geom_Curve theCurve, double theUFirst, double theULast)
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
                myTypeCurve = global::Kerf.Geometry.CurveKind.Circle;
                myCurveData = ((Geom_Circle)C).Circ();
            }
            else if (TheType == typeof(Geom_Line))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Line;
                myCurveData = ((Geom_Line)C).Lin();
            }
            else if (TheType == typeof(Geom_Ellipse))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Ellipse;
                myCurveData = ((Geom_Ellipse)C).Elips();
            }
            else if (TheType == typeof(Geom_Parabola))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Parabola;
                myCurveData = ((Geom_Parabola)C).Parab();
            }
            else if (TheType == typeof(Geom_Hyperbola))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.Hyperbola;
                myCurveData = ((Geom_Hyperbola)C).Hypr();
            }
            else if (TheType == typeof(Geom_BezierCurve))
            {
                myTypeCurve = BezierCurve;
                BezierData aBezierData = new BezierData();
                aBezierData.Curve = (Geom_BezierCurve)C;
                aBezierData.EvalRep = aBezierData.Curve.EvalRepresentation();
                myCurveData = aBezierData;
            }
            else if (TheType == typeof(Geom_BSplineCurve))
            {
                myTypeCurve = BSplineCurve;
                BSplineData aBSplineData = new BSplineData();
                aBSplineData.Curve = (Geom_BSplineCurve)C;
                aBSplineData.EvalRep = aBSplineData.Curve.EvalRepresentation();
                myCurveData = aBSplineData;
            }
            else if (TheType == typeof(Geom_OffsetCurve))
            {
                myTypeCurve = global::Kerf.Geometry.CurveKind.OffsetCurve;
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
                myTypeCurve = OtherCurve;
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

    /// <exception cref="NoSuchObjectException">The curve is not a Geom curve this adaptor knows.</exception>
    public override Continuity Continuity()
    {
        if (myTypeCurve == BSplineCurve)
        {
            return LocalContinuity(myFirst, myLast);
        }

        if (myTypeCurve == global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            Continuity S = ((Geom_OffsetCurve)myCurve!).GetBasisCurveContinuity();
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
                    throw new NoSuchObjectException("GeomAdaptor_Curve::Continuity");
            }
        }
        else if (myTypeCurve == OtherCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::Contunuity");
        }

        return CN;
    }

    /// <exception cref="DomainErrorException">S is G1 or G2 on a B-spline or offset curve.</exception>
    public override int NbIntervals(Continuity S)
    {
        if (myTypeCurve == BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = ((BSplineData)myCurveData!).Curve;
            if ((!aBSpl.IsPeriodic() && S <= Continuity()) || S == C0)
            {
                return 1;
            }

            int aDegree = aBSpl.Degree();
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
                    throw new DomainErrorException("GeomAdaptor_Curve::NbIntervals()");
            }

            double anEps = Math.Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());

            return global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(aBSpl.InternalKnots(),
                                      aBSpl.InternalMultiplicities(),
                                      aDegree,
                                      aBSpl.IsPeriodic(),
                                      aCont,
                                      myFirst,
                                      myLast,
                                      anEps,
                                      null);
        }

        else if (myTypeCurve == global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            int myNbIntervals = 1;
            Continuity BaseS;
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

    /// <exception cref="DomainErrorException">S is G1 or G2 on a B-spline or offset curve.</exception>
    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        if (myTypeCurve == BSplineCurve)
        {
            Geom_BSplineCurve aBSpl = ((BSplineData)myCurveData!).Curve;
            if ((!aBSpl.IsPeriodic() && S <= Continuity()) || S == C0)
            {
                T[T.Lower()] = myFirst;
                T[T.Lower() + 1] = myLast;
                return;
            }

            int aDegree = aBSpl.Degree();
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
                    throw new DomainErrorException("GeomAdaptor_Curve::Intervals()");
            }

            double anEps = Math.Min(Resolution(global::Kerf.Engine.Foundation.Precision.Confusion()), global::Kerf.Engine.Foundation.Precision.PConfusion());

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Intervals(aBSpl.InternalKnots(),
                               aBSpl.InternalMultiplicities(),
                               aDegree,
                               aBSpl.IsPeriodic(),
                               aCont,
                               myFirst,
                               myLast,
                               anEps,
                               T);
        }

        else if (myTypeCurve == global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            int myNbIntervals = 1;
            Continuity BaseS;
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
        if (!global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(myLast) && !global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(myFirst))
        {
            Point3D Pd = Value(myFirst);
            Point3D Pf = Value(myLast);
            return (Pd.Distance(Pf) <= global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        return false;
    }

    public override bool IsPeriodic() => myCurve!.IsPeriodic();

    /// <summary>The curve's LastParameter() - FirstParameter() (whether periodic or not, as in OCCT).</summary>
    public override double Period() => myCurve!.LastParameter() - myCurve.FirstParameter();

    private void RebuildCache(double theParameter)
    {
        if (myTypeCurve == BezierCurve)
        {
            // Create cache for Bezier
            BezierData aBezData = (BezierData)myCurveData!;
            Geom_BezierCurve aBezier = (Geom_BezierCurve)myCurve!;
            int aDeg = aBezier.Degree();
            NCollection_Array1<double> aFlatKnots = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(aDeg));
            if (aBezData.Cache is null)
            {
                aBezData.Cache = new Cache(aDeg,
                                                    aBezier.IsPeriodic(),
                                                    aFlatKnots,
                                                    aBezier.InternalPoles(),
                                                    aBezier.InternalWeights());
            }
            aBezData.Cache.BuildCache(theParameter, aFlatKnots, aBezier.InternalPoles(), aBezier.InternalWeights());
        }
        else if (myTypeCurve == BSplineCurve)
        {
            // Create cache for B-spline
            BSplineData aBSplData = (BSplineData)myCurveData!;
            Geom_BSplineCurve aBSpl = aBSplData.Curve;
            if (aBSplData.Cache is null)
            {
                aBSplData.Cache = new Cache(aBSpl.Degree(),
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
    public sealed override Point3D EvalD0(double theU)
    {
        double U = theU;
        Point3D P;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Line3D)myCurveData!);
                return P;

            case global::Kerf.Geometry.CurveKind.Circle:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Circle3D)myCurveData!);
                return P;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Ellipse3D)myCurveData!);
                return P;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Hyperbola3D)myCurveData!);
                return P;

            case global::Kerf.Geometry.CurveKind.Parabola:
                P = global::Kerf.Engine.Geometry.ElCLib.Value(U, (Parabola3D)myCurveData!);
                return P;

            case BezierCurve:
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

            case BSplineCurve:
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

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
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
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line3D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Circle3D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Ellipse3D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Hyperbola3D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Parabola3D)myCurveData!, out aResult.Point, out aResult.D1);
                return aResult;

            case BezierCurve:
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

            case BSplineCurve:
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

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
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
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line3D)myCurveData!, out aResult.Point, out aResult.D1);
                aResult.D2 = new Vector3D(0.0, 0.0, 0.0);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Circle3D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Ellipse3D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Hyperbola3D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Parabola3D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                return aResult;

            case BezierCurve:
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

            case BSplineCurve:
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

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
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
            case global::Kerf.Geometry.CurveKind.Line:
                global::Kerf.Engine.Geometry.ElCLib.D1(U, (Line3D)myCurveData!, out aResult.Point, out aResult.D1);
                aResult.D2 = new Vector3D(0.0, 0.0, 0.0);
                aResult.D3 = new Vector3D(0.0, 0.0, 0.0);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Circle:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Circle3D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Ellipse:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Ellipse3D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                global::Kerf.Engine.Geometry.ElCLib.D3(U, (Hyperbola3D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
                return aResult;

            case global::Kerf.Geometry.CurveKind.Parabola:
                global::Kerf.Engine.Geometry.ElCLib.D2(U, (Parabola3D)myCurveData!, out aResult.Point, out aResult.D1, out aResult.D2);
                aResult.D3 = new Vector3D(0.0, 0.0, 0.0);
                return aResult;

            case BezierCurve:
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

            case BSplineCurve:
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

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
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
    public sealed override Vector3D EvalDN(double theU, int theN)
    {
        double U = theU;
        int N = theN;
        switch (myTypeCurve)
        {
            case global::Kerf.Geometry.CurveKind.Line:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Line3D)myCurveData!, N);

            case global::Kerf.Geometry.CurveKind.Circle:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Circle3D)myCurveData!, N);

            case global::Kerf.Geometry.CurveKind.Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Ellipse3D)myCurveData!, N);

            case global::Kerf.Geometry.CurveKind.Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Hyperbola3D)myCurveData!, N);

            case global::Kerf.Geometry.CurveKind.Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.DN(U, (Parabola3D)myCurveData!, N);

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
                    return ((BSplineData)myCurveData!).Curve.LocalDN(U, aStart, aFinish, N);
                }
                else
                {
                    return myCurve!.EvalDN(U, N);
                }
            }

            case global::Kerf.Geometry.CurveKind.OffsetCurve:
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
                                                      out Vector3D aDN))
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
            case global::Kerf.Geometry.CurveKind.Line:
                return R3D;
            case global::Kerf.Geometry.CurveKind.Circle:
            {
                double R = ((Circle3D)myCurveData!).Radius();
                if (R > R3D / 2.0)
                {
                    return 2 * Math.Asin(R3D / (2 * R));
                }
                else
                {
                    return 2 * Math.PI;
                }
            }
            case global::Kerf.Geometry.CurveKind.Ellipse:
            {
                return R3D / ((Ellipse3D)myCurveData!).MajorRadius();
            }
            case BezierCurve:
            {
                ((Geom_BezierCurve)myCurve!).Resolution(R3D, out double res);
                return res;
            }
            case BSplineCurve:
            {
                ((BSplineData)myCurveData!).Curve.Resolution(R3D, out double res);
                return res;
            }
            default:
                return global::Kerf.Engine.Foundation.Precision.Parametric(R3D);
        }
    }

    /// <summary>The type of the curve (GeomAbs_OtherCurve before a Load).</summary>
    public override CurveKind GetType() => myTypeCurve;

    //    --
    //    --     The following methods must  be called when GetType returned
    //    --     the corresponding type.
    //    --

    /// <exception cref="NoSuchObjectException">The curve is not a line.</exception>
    public override Line3D Line()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Line, "GeomAdaptor_Curve::Line() - curve is not a Line");
        return (Line3D)myCurveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is not a circle.</exception>
    public override Circle3D Circle()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Circle,
                                       "GeomAdaptor_Curve::Circle() - curve is not a Circle");
        return (Circle3D)myCurveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is not an ellipse.</exception>
    public override Ellipse3D Ellipse()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Ellipse,
                                       "GeomAdaptor_Curve::Ellipse() - curve is not an Ellipse");
        return (Ellipse3D)myCurveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is not a hyperbola.</exception>
    public override Hyperbola3D Hyperbola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Hyperbola,
                                       "GeomAdaptor_Curve::Hyperbola() - curve is not a Hyperbola");
        return (Hyperbola3D)myCurveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is not a parabola.</exception>
    public override Parabola3D Parabola()
    {
        Standard_NoSuchObject_Raise_if(myTypeCurve != global::Kerf.Geometry.CurveKind.Parabola,
                                       "GeomAdaptor_Curve::Parabola() - curve is not a Parabola");
        return (Parabola3D)myCurveData!;
    }

    /// <exception cref="NoSuchObjectException">The curve is neither a Bezier nor a B-spline curve.</exception>
    public override int Degree()
    {
        if (myTypeCurve == BezierCurve)
        {
            return ((Geom_BezierCurve)myCurve!).Degree();
        }
        else if (myTypeCurve == BSplineCurve)
        {
            return ((BSplineData)myCurveData!).Curve.Degree();
        }
        else
        {
            throw new NoSuchObjectException();
        }
    }

    /// <summary>True for a rational Bezier or B-spline curve; false for any other curve.</summary>
    public override bool IsRational()
    {
        switch (myTypeCurve)
        {
            case BSplineCurve:
                return ((BSplineData)myCurveData!).Curve.IsRational();
            case BezierCurve:
                return ((Geom_BezierCurve)myCurve!).IsRational();
            default:
                return false;
        }
    }

    /// <exception cref="NoSuchObjectException">The curve is neither a Bezier nor a B-spline curve.</exception>
    public override int NbPoles()
    {
        if (myTypeCurve == BezierCurve)
        {
            return ((Geom_BezierCurve)myCurve!).NbPoles();
        }
        else if (myTypeCurve == BSplineCurve)
        {
            return ((BSplineData)myCurveData!).Curve.NbPoles();
        }
        else
        {
            throw new NoSuchObjectException();
        }
    }

    /// <exception cref="NoSuchObjectException">The curve is not a B-spline curve.</exception>
    public override int NbKnots()
    {
        if (myTypeCurve != BSplineCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::NbKnots");
        }
        return ((BSplineData)myCurveData!).Curve.NbKnots();
    }

    /// <summary>The Bezier curve (shared, frozen when it was loaded from a trimmed curve).</summary>
    /// <exception cref="NoSuchObjectException">The curve is not a Bezier curve.</exception>
    public override Geom_BezierCurve Bezier()
    {
        if (myTypeCurve != BezierCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::Bezier");
        }
        return (Geom_BezierCurve)myCurve!;
    }

    /// <summary>The B-spline curve (shared, frozen when it was loaded from a trimmed curve).</summary>
    /// <exception cref="NoSuchObjectException">The curve is not a B-spline curve.</exception>
    public override Geom_BSplineCurve BSpline()
    {
        if (myTypeCurve != BSplineCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::BSpline");
        }

        return ((BSplineData)myCurveData!).Curve;
    }

    /// <summary>The offset curve (shared).</summary>
    /// <exception cref="NoSuchObjectException">The curve is not an offset curve.</exception>
    public override Geom_OffsetCurve OffsetCurve()
    {
        if (myTypeCurve != global::Kerf.Geometry.CurveKind.OffsetCurve)
        {
            throw new NoSuchObjectException("GeomAdaptor_Curve::OffsetCurve");
        }
        return (Geom_OffsetCurve)myCurve!;
    }
}
