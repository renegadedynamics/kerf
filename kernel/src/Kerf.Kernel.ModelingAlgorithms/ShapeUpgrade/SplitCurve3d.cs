// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve3d.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Splits a 3D curve with a criterion (the base class splits at the values given). The curve given is copied at
/// <see cref="Init(Curve3D, double, double)"/>, so it is never modified.
/// </summary>
/// <remarks>A working object (decision 21). The resulting curves are the tool's own, unfrozen.</remarks>
public class SplitCurve3d : SplitCurve
{
    /// <summary>The tool's copy of the curve.</summary>
    protected Curve3D? _curve;
    /// <summary>The resulting curves.</summary>
    protected NCollection_HArray1<Curve3D>? _resultingCurves;

    /// <summary>Empty constructor.</summary>
    public SplitCurve3d()
    {
    }

    /// <summary>Initializes with the curve over its whole range.</summary>
    public void Init(Curve3D c)
    {
        Init(c, c.FirstParameter(), c.LastParameter());
    }

    /// <summary>Initializes with a copy of the curve, bounded by the parameters (clipped to a non-periodic curve's domain).</summary>
    public void Init(Curve3D c, double first, double last)
    {
        Curve3D copyOfC = (Curve3D)c.Copy();
        _curve = copyOfC;

        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double firstPar = first;
        double lastPar = last;
        Curve3D curve = _curve;
        if (curve is TrimmedCurve3D trimmed)
        {
            curve = trimmed.BasisCurve();
        }
        // 15.11.2002 PTV OCC966
        if (!CurveAnalysis.IsPeriodic(c))
        {
            double fP = curve.FirstParameter();
            double lP = curve.LastParameter();
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
        if (_splitValues.Length() > 2)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        if (_curve is TrimmedCurve3D tmp)
        {
            Curve3D basCurve = tmp.BasisCurve();
            var spc = new SplitCurve3d();
            spc.Init(basCurve, first, last);
            spc.SetSplitValues(_splitValues);
            spc.Build(segment);
            _nbCurves = spc.GetCurves()!.Length();

            _resultingCurves = new NCollection_HArray1<Curve3D>(1, _nbCurves);
            if (_nbCurves == 1)
            {
                var newTrimCurve = new TrimmedCurve3D(spc.GetCurves()!.Value(1), first, last);
                _resultingCurves.SetValue(1, newTrimCurve);
            }
            else
            {
                _resultingCurves = spc.GetCurves();
            }
            _status |= spc._status;
            return;
        }
        else if (_curve is OffsetCurve3D tmpOffset)
        {
            Curve3D basCurve = tmpOffset.BasisCurve();
            double offset = tmpOffset.Offset();
            Direction3D direct = tmpOffset.Direction();
            var spc = new SplitCurve3d();
            spc.Init(basCurve, first, last);
            spc.SetSplitValues(_splitValues);
            spc.Build(segment);
            _nbCurves = spc.GetCurves()!.Length();
            _resultingCurves = new NCollection_HArray1<Curve3D>(1, _nbCurves);
            for (int i = 1; i <= _nbCurves; i++)
            {
                var newOffsetCurve = new OffsetCurve3D(spc.GetCurves()!.Value(i), offset, direct);
                _resultingCurves.SetValue(i, newOffsetCurve);
            }
            _status |= spc._status;
            return;
        }

        // pdn fix on BuildCurve 3d
        //  15.11.2002 PTV OCC966
        if (!CurveAnalysis.IsPeriodic(_curve!))
        {
            // pdn exceptons only on non periodic curves
            double precision = Kerf.Engine.Foundation.Precision.PConfusion();
            double firstPar = _curve!.FirstParameter();
            double lastPar = _curve.LastParameter();
            if (Math.Abs(first - firstPar) < precision)
            {
                first = firstPar;
            }
            if (Math.Abs(last - lastPar) < precision)
            {
                last = lastPar;
            }
            if (first < firstPar)
            {
                // OCCT_DEBUG output left out
                first = firstPar;
                _splitValues.SetValue(1, first);
            }
            if (last > lastPar)
            {
                // OCCT_DEBUG output left out
                last = lastPar;
                _splitValues.SetValue(_splitValues.Length(), last);
            }
        }

        _nbCurves = _splitValues.Length() - 1;
        _resultingCurves = new NCollection_HArray1<Curve3D>(1, _nbCurves);
        if (_nbCurves == 1)
        {
            bool filled = true;
            if (Math.Abs(_curve!.FirstParameter() - first) < Kerf.Engine.Foundation.Precision.PConfusion()
                && Math.Abs(_curve.LastParameter() - last) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                _resultingCurves.SetValue(1, _curve);
            }
            else if (!segment || (_curve is not BSplineCurve3D && _curve is not BezierCurve3D)
                     || !Status(ExtendStatus.DONE2))
            {
                // OCCT's commented-out segmentation left out
                var theNewCurve = new TrimmedCurve3D((Curve3D)_curve.Copy(), first, last);
                _resultingCurves.SetValue(1, theNewCurve);
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

        if (_curve is BSplineCurve3D)
        {
            var bsCurve = (BSplineCurve3D)_curve.Copy();
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
            Curve3D? theNewCurve = null;
            if (segment)
            {
                if (_curve is BSplineCurve3D || _curve is BezierCurve3D)
                {
                    theNewCurve = (Curve3D)_curve!.Copy();
                    try
                    {
                        if (_curve is BSplineCurve3D)
                        {
                            ((BSplineCurve3D)theNewCurve).Segment(firstt, lastt);
                        }
                        else if (_curve is BezierCurve3D)
                        {
                            ((BezierCurve3D)theNewCurve).Segment(firstt, lastt);
                        }
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                    }
                    catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                    {
                        // OCCT_DEBUG output left out
                        theNewCurve = new TrimmedCurve3D((Curve3D)_curve.Copy(), firstt, lastt);
                    }
                }
                else
                {
                    theNewCurve = new TrimmedCurve3D((Curve3D)_curve!.Copy(), firstt, lastt);
                }
            }
            _resultingCurves.SetValue(i, theNewCurve!);
        }
    }

    /// <summary>Returns the resulting curves (the tool's own).</summary>
    public NCollection_HArray1<Curve3D>? GetCurves() => _resultingCurves;
}
