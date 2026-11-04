// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve3dToBezier.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve3dToBezier.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertCurve3dToBezier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Converts a 3D curve to a sequence of Bezier curves: lines (in line mode), conics (in conic and circle modes, through
/// an approximation) and other curves are converted to B-splines and cut at their knots.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
internal class ConvertCurve3dToBezier : SplitCurve3d
{
    private readonly NCollection_HSequence<Curve3D> _segments;
    private readonly NCollection_HSequence<double> _splitParams;
    private bool _lineMode;
    private bool _circleMode;
    private bool _conicMode;

    /// <summary>Empty constructor (all modes on).</summary>
    public ConvertCurve3dToBezier()
    {
        _segments = new NCollection_HSequence<Curve3D>();
        _splitParams = new NCollection_HSequence<double>();
        _lineMode = true;
        _circleMode = true;
        _conicMode = true;
    }

    /// <summary>Sets the mode for converting Geom_Line to a Bezier.</summary>
    public void SetLineMode(bool mode)
    {
        _lineMode = mode;
    }

    /// <summary>Returns the Geom_Line conversion mode.</summary>
    public bool GetLineMode() => _lineMode;

    /// <summary>Sets the mode for converting Geom_Circle to Beziers.</summary>
    public void SetCircleMode(bool mode)
    {
        _circleMode = mode;
    }

    /// <summary>Returns the Geom_Circle conversion mode.</summary>
    public bool GetCircleMode() => _circleMode;

    /// <summary>Sets the mode for converting Geom_Conic to Beziers.</summary>
    public void SetConicMode(bool mode)
    {
        _conicMode = mode;
    }

    /// <summary>Returns the Geom_Conic conversion mode.</summary>
    public bool GetConicMode() => _conicMode;

    /// <summary>Converts the curve to Bezier segments and computes the splitting values.</summary>
    public override void Compute()
    {
        _segments.Clear();
        _splitParams.Clear();
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double first = _splitValues!.Value(1);
        double last = _splitValues.Value(_splitValues.Length());
        if (_curve is TrimmedCurve3D tmp)
        {
            Curve3D basCurve = tmp.BasisCurve;
            var converter = new ConvertCurve3dToBezier();
            converter.Init(basCurve, first, last);
            converter.SetSplitValues(_splitValues);
            converter.Compute();
            _splitValues.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
            _nbCurves = _splitValues.Length() - 1;
            _status |= converter._status;
            _segments.ChangeSequence().Assign(converter.Segments().Sequence());
            _splitParams.ChangeSequence().Assign(converter.SplitParams().Sequence());
            return;
        }
        else if (_curve is BezierCurve3D bezier)
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
                var besNew = (BezierCurve3D)bezier.Copy();
                besNew.Segment(first, last);
                _segments.Append(besNew);
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }
            return;
        }
        else if (_curve is LineCurve3D line)
        {
            _nbCurves = _splitValues.Length() - 1;
            _splitParams.Append(first);
            _splitParams.Append(last);
            if (!_lineMode)
            {
                _segments.Append(line);
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
                return;
            }
            var poles = new NCollection_Array1<Point3D>(1, 2);
            poles[1] = line.Value(first);
            poles[2] = line.Value(last);
            var bezierLine = new BezierCurve3D(poles);
            _segments.Append(bezierLine);
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            return;
        }
        else if ((_curve is ConicCurve3D && !_conicMode) || (_curve is CircleCurve3D && !_circleMode))
        {
            _nbCurves = _splitValues.Length() - 1;
            _splitParams.Append(first);
            _splitParams.Append(last);
            _segments.Append(_curve);
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
            return;
        }
        else
        {
            BSplineCurve3D bSpline;
            double shift = 0.0;
            if (_curve is ConicCurve3D)
            {
                Curve3D tcurve = new TrimmedCurve3D(_curve, first, last); // protection against parabols ets
                var approx = new ApproxCurve(tcurve, Kerf.Engine.Foundation.Precision.Approximation(), C1, 100, 6);
                if (approx.HasResult)
                {
                    bSpline = approx.Curve()!;
                }
                else
                {
                    var t3d = new TrimmedCurve3D(_curve, first, last);
                    bSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(t3d, QuasiAngular);
                }
                shift = first - bSpline.FirstParameter;
                first = bSpline.FirstParameter;
                last = bSpline.LastParameter;
            }
            else if (_curve is not BSplineCurve3D)
            {
                bSpline = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(_curve!, QuasiAngular);
            }
            else
            {
                bSpline = (BSplineCurve3D)_curve;
            }

            double bf = bSpline.FirstParameter;
            double bl = bSpline.LastParameter;
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
                // OCCT_DEBUG output left out: the range of the edge exceeds the curve domain
                first = bf;
                _splitValues.SetValue(1, first);
            }
            if (last > bl)
            {
                // OCCT_DEBUG output left out
                last = bl;
                _splitValues.SetValue(_splitValues.Length(), last);
            }

            var tool = new BSplineCurveToBezierCurve(bSpline, first, last, precision);
            int nbArcs = tool.ArcCount;
            var knots = new NCollection_Array1<double>(1, nbArcs + 1);
            tool.Knots(knots);
            _splitParams.Append(first + shift);
            int j;
            for (j = 1; j <= nbArcs; j++)
            {
                double nextKnot = knots[j + 1] + shift;
                if (nextKnot - _splitParams.Value(_splitParams.Length()) > precision)
                {
                    _segments.Append(tool.Arc(j));
                    _splitParams.Append(knots[j + 1] + shift);
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
        _resultingCurves = new NCollection_HArray1<Curve3D>(1, nb - 1);
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

            var crv = (Curve3D)_segments.Value(j - 1).Copy();
            if (crv is BezierCurve3D bes)
            {
                double uFact = _splitParams.Value(j) - _splitParams.Value(j - 1);
                double pp = _splitValues.Value(i - 1);
                double length = (par - pp) / uFact;
                bes.Segment(prevPar, prevPar + length);
                prevPar += length;
                _resultingCurves.SetValue(i - 1, bes);
            }
            else
            {
                _resultingCurves.SetValue(i - 1, crv);
            }
        }
    }

    // OCCT's private Segments(): the Bezier segments.
    private NCollection_HSequence<Curve3D> Segments() => _segments;

    /// <summary>Returns the parameters of the Bezier segments' ends.</summary>
    public NCollection_HSequence<double> SplitParams() => _splitParams;
}
