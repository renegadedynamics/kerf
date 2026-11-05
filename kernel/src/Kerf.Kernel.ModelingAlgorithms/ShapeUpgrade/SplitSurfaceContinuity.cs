// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceContinuity.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Corrects or splits a surface with a continuity criterion: at each interior knot of a B-spline below the criterion
/// the knot is removed within the tolerance if possible (a copy of the surface), else it becomes a splitting value.
/// Swept surfaces are processed through their basis curve, trimmed and offset surfaces through their basis surface.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT replaces the basis curve of a surface of linear extrusion in place when it
/// removed knots from it; the port does that on a copy of a frozen surface, which then becomes the tool's surface
/// (decision 998).
/// </remarks>
internal class SplitSurfaceContinuity : SplitSurface
{
    private Continuity _criterion;
    private double _tolerance;
    private int _cont;

    /// <summary>Empty constructor (criterion C1, tolerance Confusion).</summary>
    public SplitSurfaceContinuity()
    {
        _cont = 0;
        _criterion = C1;
        _tolerance = Kerf.Engine.Foundation.Precision.Confusion();
    }

    /// <summary>Sets the criterion for splitting (C1, C2, C3 or CN; any other counts as C1).</summary>
    public void SetCriterion(Continuity criterion)
    {
        _criterion = criterion;
        switch (_criterion)
        {
            default:
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
        }
    }

    /// <summary>Sets the tolerance.</summary>
    public void SetTolerance(double tol)
    {
        _tolerance = tol;
    }

    /// <summary>Computes the splitting values and corrects the surface where it can.</summary>
    public override void Compute(bool segment)
    {
        if (!segment)
        {
            _surface!.ParameterBounds(out double uf, out double ul, out double vf, out double vl);
            if (!Kerf.Engine.Foundation.Precision.IsInfinite(uf))
            {
                _uSplitValues!.SetValue(1, uf);
            }
            if (!Kerf.Engine.Foundation.Precision.IsInfinite(ul))
            {
                _uSplitValues!.SetValue(_uSplitValues.Length(), ul);
            }
            if (!Kerf.Engine.Foundation.Precision.IsInfinite(vf))
            {
                _vSplitValues!.SetValue(1, vf);
            }
            if (!Kerf.Engine.Foundation.Precision.IsInfinite(vl))
            {
                _vSplitValues!.SetValue(_vSplitValues.Length(), vl);
            }
        }

        double uFirst = _uSplitValues!.Value(1);
        double uLast = _uSplitValues.Value(_uSplitValues.Length());
        double vFirst = _vSplitValues!.Value(1);
        double vLast = _vSplitValues.Value(_vSplitValues.Length());
        double precision = Kerf.Engine.Foundation.Precision.Confusion();

        if (_surface!.Continuity < _criterion)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        if (_uSplitValues.Length() > 2 || _vSplitValues.Length() > 2)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        if (_surface is RevolutionSurface surfaceRev)
        {
            if (surfaceRev.Continuity >= _criterion && _uSplitValues.Length() == 2
                && _vSplitValues.Length() == 2)
            {
                return;
            }
            Curve3D basCurve = surfaceRev.BasisCurve;
            var spc = new SplitCurve3dContinuity();
            spc.Init(basCurve, vFirst, vLast);
            spc.SetCriterion(_criterion);
            spc.SetTolerance(_tolerance);
            spc.SetSplitValues(_vSplitValues);
            spc.Compute();
            _vSplitValues.Clear();
            _vSplitValues.ChangeSequence().Assign(spc.SplitValues()!.Sequence());
            if (spc.Status(ExtendStatus.DONE1))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }
            if (spc.Status(ExtendStatus.DONE2))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }
            if (spc.Status(ExtendStatus.DONE3))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            }
            return;
        }
        if (_surface is ExtrusionSurface surfaceExt)
        {
            if (surfaceExt.Continuity >= _criterion && _uSplitValues.Length() == 2
                && _vSplitValues.Length() == 2)
            {
                return;
            }
            Curve3D basCurve = surfaceExt.BasisCurve;
            var spc = new SplitCurve3dContinuity();
            spc.Init(basCurve, uFirst, uLast);
            spc.SetCriterion(_criterion);
            spc.SetTolerance(_tolerance);
            spc.SetSplitValues(_uSplitValues);
            spc.Compute();
            _uSplitValues.Clear();
            _uSplitValues.ChangeSequence().Assign(spc.SplitValues()!.Sequence());
            if (spc.Status(ExtendStatus.DONE1))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }
            if (spc.Status(ExtendStatus.DONE2))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }
            if (spc.Status(ExtendStatus.DONE3))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                Curve3D newBascurve = spc.GetCurve()!;
                // Kerf: OCCT sets the basis curve of the surface it was given (the face's own); a frozen surface
                // is copied first and the copy becomes the tool's surface (decision 998)
                if (surfaceExt.IsFrozen)
                {
                    surfaceExt = (ExtrusionSurface)surfaceExt.Copy();
                    _surface = surfaceExt;
                }
                surfaceExt.SetBasisCurve(newBascurve);
            }
            return;
        }

        if (_surface is TrimmedSurface tmpRect)
        {
            if (tmpRect.Continuity >= _criterion && _uSplitValues.Length() == 2
                && _vSplitValues.Length() == 2)
            {
                return;
            }

            tmpRect.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
            Surface theSurf = tmpRect.BasisSurface;
            var sps = new SplitSurfaceContinuity();
            sps.Init(theSurf,
                     Math.Max(u1, uFirst),
                     Math.Min(u2, uLast),
                     Math.Max(v1, vFirst),
                     Math.Min(v2, vLast));
            sps.SetUSplitValues(_uSplitValues);
            sps.SetVSplitValues(_vSplitValues);
            sps.SetTolerance(_tolerance);
            sps.SetCriterion(_criterion);
            sps.Compute(true);
            _uSplitValues.Clear();
            _uSplitValues.ChangeSequence().Assign(sps.USplitValues()!.Sequence());
            _vSplitValues.Clear();
            _vSplitValues.ChangeSequence().Assign(sps.VSplitValues()!.Sequence());
            _status |= sps._status;
            return;
        }
        else if (_surface is OffsetSurface tmpOffset)
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
                case C3: // OCCT_DEBUG warning left out: criterion C3 for Offset surface
                case CN:
                    basCriterion = CN;
                    break;
            }
            Surface theSurf = tmpOffset.BasisSurface;
            if (theSurf.Continuity >= basCriterion && _uSplitValues.Length() == 2
                && _vSplitValues.Length() == 2)
            {
                return;
            }

            var sps = new SplitSurfaceContinuity();
            sps.Init(theSurf, uFirst, uLast, vFirst, vLast);
            sps.SetUSplitValues(_uSplitValues);
            sps.SetVSplitValues(_vSplitValues);
            sps.SetTolerance(_tolerance);
            sps.SetCriterion(basCriterion);
            sps.Compute(true);
            _uSplitValues.Clear();
            _uSplitValues.ChangeSequence().Assign(sps.USplitValues()!.Sequence());
            _vSplitValues.Clear();
            _vSplitValues.ChangeSequence().Assign(sps.VSplitValues()!.Sequence());
            _status |= sps._status;
            return;
        }

        BSplineSurface? myBSpline = null;
        if (_surface is BSplineSurface)
        {
            myBSpline = (BSplineSurface)_surface.Copy();
        }
        if (myBSpline is null)
        {
            return;
        }
        if (_surface.Continuity >= _criterion)
        {
            return;
        }

        // it is a BSplineSurface
        int uDeg = myBSpline.UDegree;
        int vDeg = myBSpline.VDegree;
        int nbUKnots = myBSpline.UKnotCount;
        int uFirstInd = myBSpline.FirstUKnotIndex() + 1, uLastInd = myBSpline.LastUKnotIndex() - 1,
            vFirstInd = myBSpline.FirstVKnotIndex() + 1, vLastInd = myBSpline.LastVKnotIndex() - 1;
        int nbVKnots = myBSpline.VKnotCount;

        if (nbUKnots > 2)
        {
            // Only the internal knots are checked.
            int iknot = uFirstInd;
            for (int j = 2; j <= _uSplitValues.Length(); j++)
            {
                uLast = _uSplitValues.Value(j);

                for (; iknot <= uLastInd; iknot++)
                {
                    double valknot = myBSpline.UKnot(iknot);
                    if (valknot <= uFirst + precision)
                    {
                        continue;
                    }
                    if (valknot >= uLast - precision)
                    {
                        break;
                    }
                    int continuity = uDeg - myBSpline.UMultiplicity(iknot);
                    if (continuity < _cont)
                    {
                        // At this knot, the Surface is C0; try to remove Knot.
                        int newMultiplicity = uDeg - _cont;
                        bool corrected = false;
                        if (newMultiplicity >= 0)
                        {
                            corrected = myBSpline.RemoveUKnot(iknot, newMultiplicity, _tolerance);
                        }
                        if (corrected && newMultiplicity > 0)
                        {
                            continuity = uDeg - myBSpline.UMultiplicity(iknot);
                            corrected = (continuity >= _cont);
                        }
                        if (corrected)
                        {
                            // at this knot, the continuity is now C1. Nothing else to do.
                            // PTV 15.05.2002 decrease iknot and ULastIndex values if knot removed
                            if (newMultiplicity == 0)
                            {
                                iknot--;
                                uLastInd--;
                            }
                            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                        }
                        else
                        {
                            // impossible to force C1 within the tolerance:
                            // this knot will be a splitting value.
                            double u = myBSpline.UKnot(iknot);
                            _uSplitValues.InsertBefore(j++, u);
                            _nbResultingRow++;
                        }
                    }
                }
                uFirst = uLast;
            }
        }

        if (nbVKnots > 2)
        {
            // Only the internal knots are checked.
            int iknot = vFirstInd;
            for (int j1 = 2; j1 <= _vSplitValues.Length(); j1++)
            {
                vLast = _vSplitValues.Value(j1);
                for (; iknot <= vLastInd; iknot++)
                {
                    double valknot = myBSpline.VKnot(iknot);
                    if (valknot <= vFirst + precision)
                    {
                        continue;
                    }
                    if (valknot >= vLast - precision)
                    {
                        break;
                    }
                    int continuity = vDeg - myBSpline.VMultiplicity(iknot);
                    if (continuity < _cont)
                    {
                        // At this knot, the Surface is C0; try to remove Knot.
                        int newMultiplicity = vDeg - _cont;
                        bool corrected = false;
                        if (newMultiplicity >= 0)
                        {
                            corrected = myBSpline.RemoveVKnot(iknot, newMultiplicity, _tolerance);
                        }
                        if (corrected && newMultiplicity > 0)
                        {
                            continuity = vDeg - myBSpline.VMultiplicity(iknot);
                            corrected = (continuity >= _cont);
                        }
                        if (corrected)
                        {
                            // at this knot, the continuity is now Criterion. Nothing else to do.
                            // PTV 15.05.2002 decrease iknot and ULastIndex values if knot removed
                            if (newMultiplicity == 0)
                            {
                                iknot--;
                                vLastInd--;
                            }
                            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                        }
                        else
                        {
                            // this knot will be a splitting value.
                            double v = myBSpline.VKnot(iknot);
                            _vSplitValues.InsertBefore(j1++, v);
                            _nbResultingCol++;
                        }
                    }
                }
                vFirst = vLast;
            }
        }
        if (Status(ExtendStatus.DONE3))
        {
            _surface = myBSpline;
        }

        if (_uSplitValues.Length() > 2 || _vSplitValues.Length() > 2)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
    }
}
