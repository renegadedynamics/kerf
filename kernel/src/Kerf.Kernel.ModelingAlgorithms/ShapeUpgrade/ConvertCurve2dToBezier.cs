// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve2dToBezier.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve2dToBezier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Converts a 2D curve to a sequence of Bezier curves: B-splines and Beziers that are lines within
/// Precision::Approximation become one linear Bezier; lines become a linear Bezier; conics (through an approximation)
/// and other curves are converted to B-splines, simplified and cut at their knots.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
internal class ConvertCurve2dToBezier : SplitCurve2d
{
    private readonly NCollection_HSequence<Curve2D> _segments;
    private readonly NCollection_HSequence<double> _splitParams;

    /// <summary>Empty constructor.</summary>
    public ConvertCurve2dToBezier()
    {
        _segments = new NCollection_HSequence<Curve2D>();
        _splitParams = new NCollection_HSequence<double>();
    }

    private static BezierCurve2D MakeBezier2d(Curve2D curve2d, double first, double last)
    {
        var poles = new NCollection_Array1<Point2D>(1, 2);
        poles[1] = curve2d.Value(first);
        poles[2] = curve2d.Value(last);
        var bezier = new BezierCurve2D(poles);
        return bezier;
    }

    /// <summary>Converts the curve to Bezier segments and computes the splitting values.</summary>
    public override void Compute()
    {
        _segments.Clear();
        _splitParams.Clear();
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double first = _splitValues!.Value(1);
        double last = _splitValues.Value(_splitValues.Length());

        // PTV Try to create line2d from myCurve
        if (_curve is BSplineCurve2D || _curve is BezierCurve2D)
        {
            // static function`s code getted from ShapeConvert
            double tmpF = 0, tmpL = 0, deviation = 0;
            LineCurve2D? tmpLine2d = CustomCurve2d.ConvertToLine2d(_curve,
                                                                         first,
                                                                         last,
                                                                         Kerf.Engine.Foundation.Precision.Approximation(),
                                                                         ref tmpF,
                                                                         ref tmpL,
                                                                         ref deviation);
            if (tmpLine2d is not null && (deviation <= Kerf.Engine.Foundation.Precision.Approximation()))
            {
                BezierCurve2D bezier = MakeBezier2d(tmpLine2d, tmpF, tmpL);
                _segments.Append(bezier);
                _splitParams.Append(first);
                _splitParams.Append(last);
                _nbCurves = _splitValues.Length() - 1;
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                return;
            }
        }

        if (_curve is TrimmedCurve2D tmp)
        {
            Curve2D basCurve = tmp.BasisCurve;
            var converter = new ConvertCurve2dToBezier();
            // converter.Init(BasCurve,Max(First,BasCurve->FirstParameter()),Min(Last,BasCurve->LastParameter()));
            // //???
            converter.Init(basCurve, first, last);
            converter.SetSplitValues(_splitValues);
            converter.Compute();
            _splitValues.Clear();
            _splitValues.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
            _nbCurves = _splitValues.Length() - 1;
            _status |= converter._status;
            _segments.ChangeSequence().Assign(converter.Segments().Sequence());
            _splitParams.ChangeSequence().Assign(converter.SplitParams().Sequence());
            return;
        }
        else if (_curve is BezierCurve2D bezier)
        {
            _nbCurves = _splitValues.Length() - 1;
            _splitParams.Append(first);
            _splitParams.Append(last);
            if (first < precision && last > 1 - precision)
            {
                _segments.Append(bezier);
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
            }
            else
            {
                var besNew = (BezierCurve2D)bezier.Copy();
                besNew.Segment(first, last);
                _segments.Append(besNew);
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }
            return;
        }
        else if (_curve is LineCurve2D line2d)
        {
            BezierCurve2D bezierLine = MakeBezier2d(line2d, first, last);
            _segments.Append(bezierLine);
            _splitParams.Append(first);
            _splitParams.Append(last);
            _nbCurves = _splitValues.Length() - 1;
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            return;
        }
        else
        {
            BSplineCurve2D bSpline2d;
            double shift = 0.0;
            if (_curve is ConicCurve2D)
            {
                Curve2D tcurve = new TrimmedCurve2D(_curve, first, last); // protection against parabols ets
                var approx = new ApproxCurve2D(tcurve, Kerf.Engine.Foundation.Precision.Approximation(), C1, 100, 6);
                if (approx.HasResult)
                {
                    bSpline2d = approx.Curve()!;
                }
                else
                {
                    bSpline2d = Geom2dConvert2D.CurveToBSplineCurve(tcurve, QuasiAngular);
                }

                shift = first - bSpline2d.FirstParameter;
                first = bSpline2d.FirstParameter;
                last = bSpline2d.LastParameter;
            }
            else if (_curve is not BSplineCurve2D)
            {
                bSpline2d = Geom2dConvert2D.CurveToBSplineCurve(_curve!, QuasiAngular);
            }
            else
            {
                bSpline2d = (BSplineCurve2D)_curve;
            }

            double bf = bSpline2d.FirstParameter;
            double bl = bSpline2d.LastParameter;
            if (Math.Abs(first - bf) < precision)
            {
                first = bf;
            }
            if (Math.Abs(last - bl) < precision)
            {
                last = bl;
            }
            if (first < bf)
            {
                // OCCT_DEBUG output left out: the range of the edge exceeds the pcurve domain
                first = bf;
                _splitValues.SetValue(1, first);
            }
            if (last > bl)
            {
                // OCCT_DEBUG output left out
                last = bl;
                _splitValues.SetValue(_splitValues.Length(), last);
            }

            // PTV 20.12.2001 Try to simplify BSpline Curve
            // Kerf: OCCT simplifies the curve in place (the tool's own copy here, or the approximation's); a frozen
            // curve is simplified on a copy, given back through the ref (decision 974)
            CustomCurve2d.SimplifyBSpline2d(ref bSpline2d, Kerf.Engine.Foundation.Precision.Approximation());

            var tool = new BSplineCurveToBezierCurve2D(bSpline2d, first, last, precision);
            int nbArcs = tool.ArcCount;
            var knots = new NCollection_Array1<double>(1, nbArcs + 1);
            tool.Knots(knots);
            _splitParams.Append(first + shift);
            int j;
            double newFirst = first + shift;
            double newLast = first + shift;
            for (j = 1; j <= nbArcs; j++)
            {
                double nextKnot = knots[j + 1] + shift;
                if (nextKnot - _splitParams.Value(_splitParams.Length()) > precision)
                {
                    Curve2D crv2d = tool.Arc(j);
                    if (crv2d is BezierCurve2D)
                    {
                        newFirst = newLast;
                        newLast = nextKnot;
                        double tmpF = 0, tmpL = 0, deviation = 0;
                        LineCurve2D? tmpLine2d = CustomCurve2d.ConvertToLine2d(crv2d,
                                                                                     newFirst,
                                                                                     newLast,
                                                                                     Kerf.Engine.Foundation.Precision.Approximation(),
                                                                                     ref tmpF,
                                                                                     ref tmpL,
                                                                                     ref deviation);
                        if (tmpLine2d is not null && (deviation <= Kerf.Engine.Foundation.Precision.Approximation()))
                        {
                            BezierCurve2D bezierArc = MakeBezier2d(bSpline2d, newFirst, newLast);
                            _segments.Append(bezierArc);
                            _splitParams.Append(newLast);
                            continue;
                        }
                    }
                    _segments.Append(crv2d);
                    _splitParams.Append(nextKnot);
                }
            }

            first = _splitValues.Value(1);
            for (j = 2; j <= _splitValues.Length(); j++)
            {
                last = _splitValues.Value(j);
                for (int i = 2; i <= nbArcs + 1; i++)
                {
                    double valknot = knots[i] + shift;
                    if (valknot <= first + precision)
                    {
                        continue;
                    }
                    if (valknot >= last - precision)
                    {
                        break;
                    }
                    _splitValues.InsertBefore(j++, valknot);
                }
                first = last;
            }
            _nbCurves = _splitValues.Length() - 1;
        }
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
    }

    /// <summary>Builds the resulting curves: the Bezier segments cut at the splitting values.</summary>
    public override void Build(bool segment)
    {
        double prec = Kerf.Engine.Foundation.Precision.PConfusion();
        int nb = _splitValues!.Length();
        _resultingCurves = new NCollection_HArray1<Curve2D>(1, nb - 1);
        double prevPar = 0.0;
        int j = 2;
        for (int i = 2; i <= nb; i++)
        {
            double par = _splitValues.Value(i);
            for (; j <= _splitParams.Length(); j++)
            {
                if (_splitParams.Value(j) + prec > par)
                {
                    break;
                }
                else
                {
                    prevPar = 0.0;
                }
            }

            // OCCT down-casts the segment to a Bezier and dereferences it: every segment is one here
            var bes = (BezierCurve2D)_segments.Value(j - 1).Copy();
            double uFact = _splitParams.Value(j) - _splitParams.Value(j - 1);
            double pp = _splitValues.Value(i - 1);
            double length = (par - pp) / uFact;
            bes.Segment(prevPar, prevPar + length);
            prevPar += length;
            _resultingCurves.SetValue(i - 1, bes);
        }
    }

    // OCCT's private Segments(): the Bezier segments.
    private NCollection_HSequence<Curve2D> Segments() => _segments;

    /// <summary>Returns the parameters of the Bezier segments' ends.</summary>
    public NCollection_HSequence<double> SplitParams() => _splitParams;
}
