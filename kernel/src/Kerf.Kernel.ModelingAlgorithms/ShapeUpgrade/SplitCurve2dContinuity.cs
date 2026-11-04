// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve2dContinuity.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitCurve2dContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Corrects or splits a 2D curve with a continuity criterion: at each interior knot of a B-spline below the criterion
/// the knot is removed within the tolerance if possible (the tool's copy of the curve is changed), else it becomes a
/// splitting value. Trimmed and offset curves are processed through their basis.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
internal class SplitCurve2dContinuity : SplitCurve2d
{
    private Continuity _criterion;
    private int _cont;
    private double _tolerance;

    /// <summary>Empty constructor (criterion C1, tolerance PConfusion).</summary>
    public SplitCurve2dContinuity()
    {
        _criterion = C1;
        _tolerance = Kerf.Engine.Foundation.Precision.PConfusion();
        _cont = 1;
    }

    /// <summary>Sets the criterion of continuity (C0, C1, C2, C3 or CN; G1 and G2 count as C1).</summary>
    public void SetCriterion(Continuity criterion)
    {
        _criterion = criterion;
        switch (_criterion)
        {
            case C0:
                _cont = 0;
                break;
            case C1:
                _cont = 1;
                break;
            case C2:
                _cont = 2;
                break;
            case C3:
                _cont = 3;
                break;
            case CN:
                _cont = 4;
                break;
            default:
                _cont = 1;
                break;
        }
    }

    /// <summary>Sets the tolerance for removing knots.</summary>
    public void SetTolerance(double tol)
    {
        _tolerance = tol;
    }

    /// <summary>Computes the splitting values and corrects the curve where it can.</summary>
    public override void Compute()
    {
        if (_curve!.Continuity < _criterion)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        if (_splitValues!.Length() > 2)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double first = _splitValues.Value(1);
        double last = _splitValues.Value(_splitValues.Length());
        if (_curve is TrimmedCurve2D tmp)
        {
            Curve2D basCurve = tmp.BasisCurve;
            var spc = new SplitCurve2dContinuity();
            //    spc.Init(BasCurve,Max(First,tmp->FirstParameter()),Min(Last,tmp->LastParameter()));
            spc.Init(basCurve, first, last);
            spc.SetSplitValues(_splitValues);
            spc.SetTolerance(_tolerance);
            spc.SetCriterion(_criterion);
            spc.Compute();
            _splitValues.Clear();
            _splitValues.ChangeSequence().Assign(spc.SplitValues()!.Sequence());
            // mySplitValues = spc.SplitValues();
            _status |= spc._status;
            return;
        }
        else if (_curve is OffsetCurve2D tmpOffset)
        {
            Continuity basCriterion;
            switch (_criterion)
            {
                default:
                case C1:
                    basCriterion = C2;
                    break;
                case C2:
                    basCriterion = C3;
                    break;
                case C3: // OCCT_DEBUG warning left out: criterion C3 for Offset curve
                case CN:
                    basCriterion = CN;
                    break;
            }
            Curve2D basCurve = tmpOffset.BasisCurve;
            // double Offset = tmp->Offset(); // Offset not used (skl)
            var spc = new SplitCurve2dContinuity();
            //    spc.Init(BasCurve,Max(tmp->FirstParameter(),First),Min(tmp->LastParameter(),Last));
            spc.Init(basCurve, first, last);
            spc.SetSplitValues(_splitValues);
            spc.SetTolerance(_tolerance);
            spc.SetCriterion(basCriterion);
            spc.Compute();
            _splitValues.Clear();
            _splitValues.ChangeSequence().Assign(spc.SplitValues()!.Sequence());
            _status |= spc._status;
            return;
        }

        BSplineCurve2D? myBSpline = _curve as BSplineCurve2D;
        if (myBSpline is null)
        {
            return;
        }

        _nbCurves = 1;
        int deg = myBSpline.Degree;
        int nbKnots = myBSpline.KnotCount;
        if (nbKnots <= 2)
        {
            return;
        }

        int firstInd = myBSpline.FirstUKnotIndex() + 1, lastInd = myBSpline.LastUKnotIndex() - 1;
        int iknot = firstInd;
        for (int j = 2; j <= _splitValues.Length(); j++)
        {
            last = _splitValues.Value(j);
            for (; iknot <= lastInd; iknot++)
            {
                double valknot = myBSpline.Knot(iknot);
                if (valknot <= first + precision)
                {
                    continue;
                }
                if (valknot >= last - precision)
                {
                    break;
                }
                int continuity = deg - myBSpline.Multiplicity(iknot);
                // double tt = MyBSpline->Knot(iknot); // tt not used (skl)
                if (continuity < _cont)
                {
                    // At this knot, the curve is C0; try to remove Knot.
                    bool corrected = false;
                    int newMultiplicity = deg - _cont;
                    if (newMultiplicity < 0)
                    {
                        newMultiplicity = 0;
                    }
                    {
                        try
                        {
                            corrected = myBSpline.RemoveKnot(iknot, newMultiplicity, _tolerance);
                        }
                        catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                        {
                            corrected = false;
                        }
                    }
                    if (corrected && newMultiplicity > 0)
                    {
                        continuity = deg - myBSpline.Multiplicity(iknot);
                        corrected = (continuity >= _cont);
                    }
                    if (corrected)
                    {
                        // at this knot, the continuity is now C1. Nothing else to do.
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                    }
                    else
                    {
                        // impossible to force C1 within the tolerance:
                        // this knot will be a splitting value.
                        _splitValues.InsertBefore(j++, myBSpline.Knot(iknot));
                        _nbCurves++;
                    }
                }
            }

            first = last;
        }

        if (_splitValues.Length() > 2)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
    }
}
