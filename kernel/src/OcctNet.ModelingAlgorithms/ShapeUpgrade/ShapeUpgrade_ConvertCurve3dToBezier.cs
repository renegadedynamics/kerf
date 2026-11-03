// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve3dToBezier.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve3dToBezier.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve3dToBezier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Converts a 3D curve to a sequence of Bezier curves: lines (in line mode), conics (in conic and circle modes, through
/// an approximation) and other curves are converted to B-splines and cut at their knots.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
public class ShapeUpgrade_ConvertCurve3dToBezier : ShapeUpgrade_SplitCurve3d
{
    private readonly NCollection_HSequence<Curve3D> mySegments;
    private readonly NCollection_HSequence<double> mySplitParams;
    private bool myLineMode;
    private bool myCircleMode;
    private bool myConicMode;

    /// <summary>Empty constructor (all modes on).</summary>
    public ShapeUpgrade_ConvertCurve3dToBezier()
    {
        mySegments = new NCollection_HSequence<Curve3D>();
        mySplitParams = new NCollection_HSequence<double>();
        myLineMode = true;
        myCircleMode = true;
        myConicMode = true;
    }

    /// <summary>Sets the mode for converting Geom_Line to a Bezier.</summary>
    public void SetLineMode(bool mode)
    {
        myLineMode = mode;
    }

    /// <summary>Returns the Geom_Line conversion mode.</summary>
    public bool GetLineMode() => myLineMode;

    /// <summary>Sets the mode for converting Geom_Circle to Beziers.</summary>
    public void SetCircleMode(bool mode)
    {
        myCircleMode = mode;
    }

    /// <summary>Returns the Geom_Circle conversion mode.</summary>
    public bool GetCircleMode() => myCircleMode;

    /// <summary>Sets the mode for converting Geom_Conic to Beziers.</summary>
    public void SetConicMode(bool mode)
    {
        myConicMode = mode;
    }

    /// <summary>Returns the Geom_Conic conversion mode.</summary>
    public bool GetConicMode() => myConicMode;

    /// <summary>Converts the curve to Bezier segments and computes the splitting values.</summary>
    public override void Compute()
    {
        mySegments.Clear();
        mySplitParams.Clear();
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double First = mySplitValues!.Value(1);
        double Last = mySplitValues.Value(mySplitValues.Length());
        if (myCurve is TrimmedCurve3D tmp)
        {
            Curve3D BasCurve = tmp.BasisCurve();
            var converter = new ShapeUpgrade_ConvertCurve3dToBezier();
            converter.Init(BasCurve, First, Last);
            converter.SetSplitValues(mySplitValues);
            converter.Compute();
            mySplitValues.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
            myNbCurves = mySplitValues.Length() - 1;
            myStatus |= converter.myStatus;
            mySegments.ChangeSequence().Assign(converter.Segments().Sequence());
            mySplitParams.ChangeSequence().Assign(converter.SplitParams().Sequence());
            return;
        }
        else if (myCurve is BezierCurve3D bezier)
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
                var besNew = (BezierCurve3D)bezier.Copy();
                besNew.Segment(First, Last);
                mySegments.Append(besNew);
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }
            return;
        }
        else if (myCurve is LineCurve3D aLine)
        {
            myNbCurves = mySplitValues.Length() - 1;
            mySplitParams.Append(First);
            mySplitParams.Append(Last);
            if (!myLineMode)
            {
                mySegments.Append(aLine);
                myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
                return;
            }
            var poles = new NCollection_Array1<Point3D>(1, 2);
            poles[1] = aLine.Value(First);
            poles[2] = aLine.Value(Last);
            var bezierLine = new BezierCurve3D(poles);
            mySegments.Append(bezierLine);
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            return;
        }
        else if ((myCurve is ConicCurve3D && !myConicMode) || (myCurve is CircleCurve3D && !myCircleMode))
        {
            myNbCurves = mySplitValues.Length() - 1;
            mySplitParams.Append(First);
            mySplitParams.Append(Last);
            mySegments.Append(myCurve);
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
            return;
        }
        else
        {
            BSplineCurve3D aBSpline;
            double Shift = 0.0;
            if (myCurve is ConicCurve3D)
            {
                Curve3D tcurve = new TrimmedCurve3D(myCurve, First, Last); // protection against parabols ets
                var approx = new ApproxCurve(tcurve, Kerf.Engine.Foundation.Precision.Approximation(), C1, 100, 6);
                if (approx.HasResult)
                {
                    aBSpline = approx.Curve()!;
                }
                else
                {
                    var t3d = new TrimmedCurve3D(myCurve, First, Last);
                    aBSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(t3d, QuasiAngular);
                }
                Shift = First - aBSpline.FirstParameter();
                First = aBSpline.FirstParameter();
                Last = aBSpline.LastParameter();
            }
            else if (myCurve is not BSplineCurve3D)
            {
                aBSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(myCurve!, QuasiAngular);
            }
            else
            {
                aBSpline = (BSplineCurve3D)myCurve;
            }

            double bf = aBSpline.FirstParameter();
            double bl = aBSpline.LastParameter();
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
                // OCCT_DEBUG output left out: the range of the edge exceeds the curve domain
                First = bf;
                mySplitValues.SetValue(1, First);
            }
            if (Last > bl)
            {
                // OCCT_DEBUG output left out
                Last = bl;
                mySplitValues.SetValue(mySplitValues.Length(), Last);
            }

            var tool = new BSplineCurveToBezierCurve(aBSpline, First, Last, precision);
            int nbArcs = tool.ArcCount;
            var knots = new NCollection_Array1<double>(1, nbArcs + 1);
            tool.Knots(knots);
            mySplitParams.Append(First + Shift);
            int j;
            for (j = 1; j <= nbArcs; j++)
            {
                double nextKnot = knots[j + 1] + Shift;
                if (nextKnot - mySplitParams.Value(mySplitParams.Length()) > precision)
                {
                    mySegments.Append(tool.Arc(j));
                    mySplitParams.Append(knots[j + 1] + Shift);
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
        myResultingCurves = new NCollection_HArray1<Curve3D>(1, nb - 1);
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

            var crv = (Curve3D)mySegments.Value(j - 1).Copy();
            if (crv is BezierCurve3D bes)
            {
                double uFact = mySplitParams.Value(j) - mySplitParams.Value(j - 1);
                double pp = mySplitValues.Value(i - 1);
                double length = (par - pp) / uFact;
                bes.Segment(prevPar, prevPar + length);
                prevPar += length;
                myResultingCurves.SetValue(i - 1, bes);
            }
            else
            {
                myResultingCurves.SetValue(i - 1, crv);
            }
        }
    }

    // OCCT's private Segments(): the Bezier segments.
    private NCollection_HSequence<Curve3D> Segments() => mySegments;

    /// <summary>Returns the parameters of the Bezier segments' ends.</summary>
    public NCollection_HSequence<double> SplitParams() => mySplitParams;
}
