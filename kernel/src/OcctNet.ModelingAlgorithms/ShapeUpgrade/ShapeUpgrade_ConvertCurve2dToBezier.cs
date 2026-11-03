// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve2dToBezier.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve2dToBezier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Converts a 2D curve to a sequence of Bezier curves: B-splines and Beziers that are lines within
/// Precision::Approximation become one linear Bezier; lines become a linear Bezier; conics (through an approximation)
/// and other curves are converted to B-splines, simplified and cut at their knots.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public class ShapeUpgrade_ConvertCurve2dToBezier : ShapeUpgrade_SplitCurve2d
{
    private readonly NCollection_HSequence<Geom2d_Curve> mySegments;
    private readonly NCollection_HSequence<double> mySplitParams;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_ConvertCurve2dToBezier()
    {
        mySegments = new NCollection_HSequence<Geom2d_Curve>();
        mySplitParams = new NCollection_HSequence<double>();
    }

    private static Geom2d_BezierCurve MakeBezier2d(Geom2d_Curve theCurve2d, double theFirst, double theLast)
    {
        var poles = new NCollection_Array1<Point2D>(1, 2);
        poles[1] = theCurve2d.Value(theFirst);
        poles[2] = theCurve2d.Value(theLast);
        var bezier = new Geom2d_BezierCurve(poles);
        return bezier;
    }

    /// <summary>Converts the curve to Bezier segments and computes the splitting values.</summary>
    public override void Compute()
    {
        mySegments.Clear();
        mySplitParams.Clear();
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double First = mySplitValues!.Value(1);
        double Last = mySplitValues.Value(mySplitValues.Length());

        // PTV Try to create line2d from myCurve
        if (myCurve is Geom2d_BSplineCurve || myCurve is Geom2d_BezierCurve)
        {
            // static function`s code getted from ShapeConvert
            double tmpF = 0, tmpL = 0, aDeviation = 0;
            Geom2d_Line? aTmpLine2d = ShapeCustom_Curve2d.ConvertToLine2d(myCurve,
                                                                         First,
                                                                         Last,
                                                                         Kerf.Engine.Foundation.Precision.Approximation(),
                                                                         ref tmpF,
                                                                         ref tmpL,
                                                                         ref aDeviation);
            if (aTmpLine2d is not null && (aDeviation <= Kerf.Engine.Foundation.Precision.Approximation()))
            {
                Geom2d_BezierCurve bezier = MakeBezier2d(aTmpLine2d, tmpF, tmpL);
                mySegments.Append(bezier);
                mySplitParams.Append(First);
                mySplitParams.Append(Last);
                myNbCurves = mySplitValues.Length() - 1;
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                return;
            }
        }

        if (myCurve is Geom2d_TrimmedCurve tmp)
        {
            Geom2d_Curve BasCurve = tmp.BasisCurve();
            var converter = new ShapeUpgrade_ConvertCurve2dToBezier();
            // converter.Init(BasCurve,Max(First,BasCurve->FirstParameter()),Min(Last,BasCurve->LastParameter()));
            // //???
            converter.Init(BasCurve, First, Last);
            converter.SetSplitValues(mySplitValues);
            converter.Compute();
            mySplitValues.Clear();
            mySplitValues.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
            myNbCurves = mySplitValues.Length() - 1;
            myStatus |= converter.myStatus;
            mySegments.ChangeSequence().Assign(converter.Segments().Sequence());
            mySplitParams.ChangeSequence().Assign(converter.SplitParams().Sequence());
            return;
        }
        else if (myCurve is Geom2d_BezierCurve bezier)
        {
            myNbCurves = mySplitValues.Length() - 1;
            mySplitParams.Append(First);
            mySplitParams.Append(Last);
            if (First < precision && Last > 1 - precision)
            {
                mySegments.Append(bezier);
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
            }
            else
            {
                var besNew = (Geom2d_BezierCurve)bezier.Copy();
                besNew.Segment(First, Last);
                mySegments.Append(besNew);
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }
            return;
        }
        else if (myCurve is Geom2d_Line aLine2d)
        {
            Geom2d_BezierCurve bezierLine = MakeBezier2d(aLine2d, First, Last);
            mySegments.Append(bezierLine);
            mySplitParams.Append(First);
            mySplitParams.Append(Last);
            myNbCurves = mySplitValues.Length() - 1;
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            return;
        }
        else
        {
            Geom2d_BSplineCurve aBSpline2d;
            double Shift = 0.0;
            if (myCurve is Geom2d_Conic)
            {
                Geom2d_Curve tcurve = new Geom2d_TrimmedCurve(myCurve, First, Last); // protection against parabols ets
                var approx = new Geom2dConvert_ApproxCurve(tcurve, Kerf.Engine.Foundation.Precision.Approximation(), C1, 100, 6);
                if (approx.HasResult())
                {
                    aBSpline2d = approx.Curve()!;
                }
                else
                {
                    aBSpline2d = Geom2dConvert.CurveToBSplineCurve(tcurve, QuasiAngular);
                }

                Shift = First - aBSpline2d.FirstParameter();
                First = aBSpline2d.FirstParameter();
                Last = aBSpline2d.LastParameter();
            }
            else if (myCurve is not Geom2d_BSplineCurve)
            {
                aBSpline2d = Geom2dConvert.CurveToBSplineCurve(myCurve!, QuasiAngular);
            }
            else
            {
                aBSpline2d = (Geom2d_BSplineCurve)myCurve;
            }

            double bf = aBSpline2d.FirstParameter();
            double bl = aBSpline2d.LastParameter();
            if (Math.Abs(First - bf) < precision)
            {
                First = bf;
            }
            if (Math.Abs(Last - bl) < precision)
            {
                Last = bl;
            }
            if (First < bf)
            {
                // OCCT_DEBUG output left out: the range of the edge exceeds the pcurve domain
                First = bf;
                mySplitValues.SetValue(1, First);
            }
            if (Last > bl)
            {
                // OCCT_DEBUG output left out
                Last = bl;
                mySplitValues.SetValue(mySplitValues.Length(), Last);
            }

            // PTV 20.12.2001 Try to simplify BSpline Curve
            // OCCT.NET: OCCT simplifies the curve in place (the tool's own copy here, or the approximation's); a frozen
            // curve is simplified on a copy, given back through the ref (decision 974)
            ShapeCustom_Curve2d.SimplifyBSpline2d(ref aBSpline2d, Kerf.Engine.Foundation.Precision.Approximation());

            var tool = new Geom2dConvert_BSplineCurveToBezierCurve(aBSpline2d, First, Last, precision);
            int nbArcs = tool.NbArcs();
            var knots = new NCollection_Array1<double>(1, nbArcs + 1);
            tool.Knots(knots);
            mySplitParams.Append(First + Shift);
            int j;
            double newFirst = First + Shift;
            double newLast = First + Shift;
            for (j = 1; j <= nbArcs; j++)
            {
                double nextKnot = knots[j + 1] + Shift;
                if (nextKnot - mySplitParams.Value(mySplitParams.Length()) > precision)
                {
                    Geom2d_Curve aCrv2d = tool.Arc(j);
                    if (aCrv2d is Geom2d_BezierCurve)
                    {
                        newFirst = newLast;
                        newLast = nextKnot;
                        double tmpF = 0, tmpL = 0, aDeviation = 0;
                        Geom2d_Line? aTmpLine2d = ShapeCustom_Curve2d.ConvertToLine2d(aCrv2d,
                                                                                     newFirst,
                                                                                     newLast,
                                                                                     Kerf.Engine.Foundation.Precision.Approximation(),
                                                                                     ref tmpF,
                                                                                     ref tmpL,
                                                                                     ref aDeviation);
                        if (aTmpLine2d is not null && (aDeviation <= Kerf.Engine.Foundation.Precision.Approximation()))
                        {
                            Geom2d_BezierCurve bezierArc = MakeBezier2d(aBSpline2d, newFirst, newLast);
                            mySegments.Append(bezierArc);
                            mySplitParams.Append(newLast);
                            continue;
                        }
                    }
                    mySegments.Append(aCrv2d);
                    mySplitParams.Append(nextKnot);
                }
            }

            First = mySplitValues.Value(1);
            for (j = 2; j <= mySplitValues.Length(); j++)
            {
                Last = mySplitValues.Value(j);
                for (int i = 2; i <= nbArcs + 1; i++)
                {
                    double valknot = knots[i] + Shift;
                    if (valknot <= First + precision)
                    {
                        continue;
                    }
                    if (valknot >= Last - precision)
                    {
                        break;
                    }
                    mySplitValues.InsertBefore(j++, valknot);
                }
                First = Last;
            }
            myNbCurves = mySplitValues.Length() - 1;
        }
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
    }

    /// <summary>Builds the resulting curves: the Bezier segments cut at the splitting values.</summary>
    public override void Build(bool Segment)
    {
        double prec = Kerf.Engine.Foundation.Precision.PConfusion();
        int nb = mySplitValues!.Length();
        myResultingCurves = new NCollection_HArray1<Geom2d_Curve>(1, nb - 1);
        double prevPar = 0.0;
        int j = 2;
        for (int i = 2; i <= nb; i++)
        {
            double par = mySplitValues.Value(i);
            for (; j <= mySplitParams.Length(); j++)
            {
                if (mySplitParams.Value(j) + prec > par)
                {
                    break;
                }
                else
                {
                    prevPar = 0.0;
                }
            }

            // OCCT down-casts the segment to a Bezier and dereferences it: every segment is one here
            var bes = (Geom2d_BezierCurve)mySegments.Value(j - 1).Copy();
            double uFact = mySplitParams.Value(j) - mySplitParams.Value(j - 1);
            double pp = mySplitValues.Value(i - 1);
            double length = (par - pp) / uFact;
            bes.Segment(prevPar, prevPar + length);
            prevPar += length;
            myResultingCurves.SetValue(i - 1, bes);
        }
    }

    // OCCT's private Segments(): the Bezier segments.
    private NCollection_HSequence<Geom2d_Curve> Segments() => mySegments;

    /// <summary>Returns the parameters of the Bezier segments' ends.</summary>
    public NCollection_HSequence<double> SplitParams() => mySplitParams;
}
