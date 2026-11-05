// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve2d.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Splits a 2D curve with a criterion (the base class splits at the values given). The curve given is copied at
/// <see cref="Init(Curve2D, double, double)"/>, so it is never modified.
/// </summary>
/// <remarks>A working object (decision 21). The resulting curves are the tool's own, unfrozen.</remarks>
internal class SplitCurve2d : SplitCurve
{
    /// <summary>The tool's copy of the curve.</summary>
    protected Curve2D? _curve;
    /// <summary>The resulting curves.</summary>
    protected NCollection_HArray1<Curve2D>? _resultingCurves;

    /// <summary>Empty constructor.</summary>
    public SplitCurve2d()
    {
    }

    /// <summary>Initializes with the curve over its whole range.</summary>
    public void Init(Curve2D c)
    {
        Init(c, c.FirstParameter, c.LastParameter);
    }

    /// <summary>Initializes with a copy of the curve, bounded by the parameters (clipped to a non-periodic curve's domain).</summary>
    public void Init(Curve2D c, double first, double last)
    {
        Curve2D copyOfC = (Curve2D)c.Copy();
        _curve = copyOfC;

        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double firstPar = first;
        double lastPar = last;
        Curve2D curve = _curve;
        if (curve is TrimmedCurve2D trimmed)
        {
            curve = trimmed.BasisCurve;
        }
        // 15.11.2002 PTV OCC966
        if (!CurveAnalysis.IsPeriodic(c))
        {
            double fP = curve.FirstParameter;
            double lP = curve.LastParameter;
            if (Math.Abs(firstPar - fP) < precision)
            {
                firstPar = fP;
            }
            if (Math.Abs(lastPar - lP) < precision)
            {
                lastPar = lP;
            }
            if (firstPar < fP)
            {
                // OCCT_DEBUG output left out: the range of the edge exceeds the curve domain
                firstPar = fP;
            }
            if (lastPar > lP)
            {
                // OCCT_DEBUG output left out
                lastPar = lP;
            }
            if ((lastPar - firstPar) < precision)
            {
                lastPar = firstPar + 2 * precision;
            }
        }
        base.Init(firstPar, lastPar);

        // first, we make a copy of C to prevent modification:
        _nbCurves = 1;
    }

    /// <summary>
    /// Builds the resulting curves: segments of a B-spline or Bezier curve when <paramref name="segment"/> (trimmed
    /// curves if segmenting fails), trimmed curves otherwise; trimmed and offset curves are split through their basis.
    /// </summary>
    public override void Build(bool segment)
    {
        double first = _splitValues!.Value(1);
        double last = _splitValues.Value(_splitValues.Length());
        // PrepareKnots();
        if (_splitValues.Length() > 2)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        if (_curve is TrimmedCurve2D tmp)
        {
            Curve2D basCurve = tmp.BasisCurve;
            var spc = new SplitCurve2d();
            spc.Init(basCurve, first, last);
            spc.SetSplitValues(_splitValues);
            spc.Build(segment);
            _nbCurves = spc.GetCurves()!.Length();

            _resultingCurves = new NCollection_HArray1<Curve2D>(1, _nbCurves);
            if (_nbCurves == 1)
            {
                var newTrimCurve = new TrimmedCurve2D(spc.GetCurves()!.Value(1), first, last);
                _resultingCurves.SetValue(1, newTrimCurve);
            }
            else
            {
                _resultingCurves = spc.GetCurves();
            }
            _status |= spc._status;
            return;
        }
        else if (_curve is OffsetCurve2D tmpOffset)
        {
            Curve2D basCurve = tmpOffset.BasisCurve;
            double offset = tmpOffset.Offset;
            var spc = new SplitCurve2d();
            spc.Init(basCurve, first, last);
            spc.SetSplitValues(_splitValues);
            spc.Build(segment);
            _nbCurves = spc.GetCurves()!.Length();
            _resultingCurves = new NCollection_HArray1<Curve2D>(1, _nbCurves);
            for (int i = 1; i <= _nbCurves; i++)
            {
                var newOffsetCurve = new OffsetCurve2D(spc.GetCurves()!.Value(i), offset);
                _resultingCurves.SetValue(i, newOffsetCurve);
            }
            _status |= spc._status;
            return;
        }

        _nbCurves = _splitValues.Length() - 1;
        _resultingCurves = new NCollection_HArray1<Curve2D>(1, _nbCurves);
        if (_nbCurves == 1)
        {
            bool filled = true;
            if (Math.Abs(_curve!.FirstParameter - first) < Kerf.Engine.Foundation.Precision.PConfusion()
                && Math.Abs(_curve.LastParameter - last) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                _resultingCurves.SetValue(1, _curve);
            }
            else if (!segment || (_curve is not BSplineCurve2D && _curve is not BezierCurve2D)
                     || !Status(ExtendStatus.DONE2))
            {
                if (_curve is BSplineCurve2D || _curve is BezierCurve2D)
                {
                    var theNewCurve = (Curve2D)_curve.Copy();
                    try
                    {
                        if (_curve is BSplineCurve2D)
                        {
                            ((BSplineCurve2D)theNewCurve).Segment(first, last);
                        }
                        else if (_curve is BezierCurve2D)
                        {
                            ((BezierCurve2D)theNewCurve).Segment(first, last);
                        }
                    }
                    catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                    {
                        // OCCT_DEBUG output left out
                        theNewCurve = new TrimmedCurve2D((Curve2D)_curve.Copy(), first, last);
                    }
                    _resultingCurves.SetValue(1, theNewCurve);
                }
                else
                {
                    var theNewCurve = new TrimmedCurve2D((Curve2D)_curve.Copy(), first, last);
                    _resultingCurves.SetValue(1, theNewCurve);
                }
            }
            else
            {
                filled = false;
            }
            if (filled)
            {
                return;
            }
        }
        if (_curve is BSplineCurve2D)
        {
            var bsCurve = (BSplineCurve2D)_curve.Copy();
            int firstInd = bsCurve.FirstUKnotIndex(), lastInd = bsCurve.LastUKnotIndex();
            int j = firstInd;
            for (int ii = 1; ii <= _splitValues.Length(); ii++)
            {
                double spval = _splitValues.Value(ii);
                for (; j <= lastInd; j++)
                {
                    if (spval > bsCurve.Knot(j) + Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }
                    if (spval < bsCurve.Knot(j) - Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        break;
                    }
                    _splitValues.SetValue(ii, bsCurve.Knot(j));
                }
                if (j == lastInd)
                {
                    break;
                }
            }
        }

        for (int i = 1; i <= _nbCurves; i++)
        {
            // skl : in the next block I change "First","Last" to "Firstt","Lastt"
            double firstt = _splitValues.Value(i), lastt = _splitValues.Value(i + 1);
            Curve2D? theNewCurve = null;
            if (segment)
            {
                // creates a copy of myCurve before to segment:
                if (_curve is BSplineCurve2D || _curve is BezierCurve2D)
                {
                    theNewCurve = (Curve2D)_curve!.Copy();
                    try
                    {
                        if (_curve is BSplineCurve2D)
                        {
                            ((BSplineCurve2D)theNewCurve).Segment(firstt, lastt);
                        }
                        else if (_curve is BezierCurve2D)
                        {
                            ((BezierCurve2D)theNewCurve).Segment(firstt, lastt);
                        }
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                    }
                    catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                    {
                        // OCCT_DEBUG output left out
                        theNewCurve = new TrimmedCurve2D((Curve2D)_curve.Copy(), firstt, lastt);
                    }
                }
                else
                {
                    theNewCurve = new TrimmedCurve2D((Curve2D)_curve!.Copy(), firstt, lastt);
                }
            }
            _resultingCurves.SetValue(i, theNewCurve!);
        }
    }

    /// <summary>Returns the resulting curves (the tool's own).</summary>
    public NCollection_HArray1<Curve2D>? GetCurves() => _resultingCurves;
}
