// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertSurfaceToBezierBasis.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertSurfaceToBezierBasis.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ConvertSurfaceToBezierBasis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Converts a surface into a grid of Bezier-based surfaces: planes and B-splines into Bezier patches, surfaces of
/// revolution and of linear extrusion into surfaces on Bezier curves (each conversion with its mode), trimmed and offset
/// surfaces through their basis; the grid is then cut at the splitting values.
/// </summary>
/// <remarks>A working object (decision 21).</remarks>
internal class ConvertSurfaceToBezierBasis : SplitSurface
{
    private CompositeSurface? _segments;
    private bool _planeMode;
    private bool _revolutionMode;
    private bool _extrusionMode;
    private bool _bSplineMode;

    /// <summary>Empty constructor (all modes on).</summary>
    public ConvertSurfaceToBezierBasis()
    {
        _planeMode = true;
        _revolutionMode = true;
        _extrusionMode = true;
        _bSplineMode = true;
    }

    /// <summary>Converts the surface into a grid of Bezier-based surfaces and stores it.</summary>
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
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();

        if (_surface is TrimmedSurface surface)
        {
            Surface basSurf = surface.BasisSurface;
            var converter = new ConvertSurfaceToBezierBasis();
            converter.Init(basSurf, uFirst, uLast, vFirst, vLast);
            converter.SetUSplitValues(_uSplitValues);
            converter.SetVSplitValues(_vSplitValues);
            converter.Compute(true);
            _uSplitValues.ChangeSequence().Assign(converter.USplitValues()!.Sequence());
            _vSplitValues.ChangeSequence().Assign(converter.VSplitValues()!.Sequence());
            _status |= converter._status;
            _segments = converter.Segments();
            return;
        }
        else if (_surface is OffsetSurface Offset)
        {
            Surface basSurf = Offset.BasisSurface;
            var converter = new ConvertSurfaceToBezierBasis();
            converter.Init(basSurf, uFirst, uLast, vFirst, vLast);
            converter.SetUSplitValues(_uSplitValues);
            converter.SetVSplitValues(_vSplitValues);
            converter.Compute(true);
            _uSplitValues.ChangeSequence().Assign(converter.USplitValues()!.Sequence());
            _vSplitValues.ChangeSequence().Assign(converter.VSplitValues()!.Sequence());
            _status |= converter._status;
            _segments = converter.Segments();
            return;
        }
        else if (_surface is PlaneSurface pln && _planeMode)
        {
            var poles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
            Point3D dp;
            poles[1, 1] = dp = pln.Value(uFirst, vFirst);
            poles[1, 2] = dp = pln.Value(uFirst, vLast);
            poles[2, 1] = dp = pln.Value(uLast, vFirst);
            poles[2, 2] = dp = pln.Value(uLast, vLast);
            var bezierPlane = new BezierSurface(poles);
            var uJoints = new NCollection_Array1<double>(1, 2);
            uJoints[1] = uFirst;
            uJoints[2] = uLast;
            var vJoints = new NCollection_Array1<double>(1, 2);
            vJoints[1] = vFirst;
            vJoints[2] = vLast;
            var surf = new NCollection_HArray2<Surface>(1, 1, 1, 1);
            surf.SetValue(1, 1, bezierPlane);
            _segments = new CompositeSurface(surf, uJoints, vJoints);
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            return;
        }
        else if (_surface is BezierSurface bezierSurf)
        {
            var surf = new NCollection_HArray2<Surface>(1, 1, 1, 1);
            var uJoints = new NCollection_Array1<double>(1, 2);
            uJoints[1] = uFirst;
            uJoints[2] = uLast;
            var vJoints = new NCollection_Array1<double>(1, 2);
            vJoints[1] = vFirst;
            vJoints[2] = vLast;
            if (uFirst < precision && uLast > 1 - precision && vFirst < precision && vLast > 1 - precision)
            {
                surf.SetValue(1, 1, bezierSurf);
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
            }
            else
            {
                var besNew = (BezierSurface)bezierSurf.Copy();
                // pdn K4L+ (work around)
                // rln C30 (direct use)
                double u1 = uFirst;
                double u2 = uLast;
                double v1 = vFirst;
                double v2 = vLast;
                besNew.Segment(u1, u2, v1, v2);
                surf.SetValue(1, 1, besNew);
                _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }
            _segments = new CompositeSurface(surf, uJoints, vJoints);
            return;
        }
        else if (_surface is BSplineSurface bspline && _bSplineMode)
        {
            // pdn
            bspline.ParameterBounds(out _, out _, out _, out _);
            var converter = new BSplineSurfaceToBezierSurface(bspline); //,UFirst,ULast,VFirst,VLast,precision;
            int nbUPatches = converter.UPatchCount;
            int nbVPatches = converter.VPatchCount;
            var uJoints = new NCollection_Array1<double>(1, nbUPatches + 1);
            var uReject = new NCollection_Array1<bool>(1, nbUPatches + 1);
            uReject.Init(false);
            var vJoints = new NCollection_Array1<double>(1, nbVPatches + 1);
            var vReject = new NCollection_Array1<bool>(1, nbVPatches + 1);
            vReject.Init(false);
            int nbUFiltered = 0;
            int nbVFiltered = 0;

            converter.UKnots(uJoints);
            var uFilteredJoints = new NCollection_Sequence<double>();
            uFilteredJoints.Append(uJoints[1]);
            int i;
            for (i = 2; i <= nbUPatches + 1; i++)
            {
                if (uJoints[i] - uJoints[i - 1] < precision)
                {
                    nbUFiltered++;
                    uReject[i - 1] = true;
                }
                else
                {
                    uFilteredJoints.Append(uJoints[i]);
                }
            }

            converter.VKnots(vJoints);
            var vFilteredJoints = new NCollection_Sequence<double>();
            vFilteredJoints.Append(vJoints[1]);
            for (i = 2; i <= nbVPatches + 1; i++)
            {
                if (vJoints[i] - vJoints[i - 1] < precision)
                {
                    nbVFiltered++;
                    vReject[i - 1] = true;
                }
                else
                {
                    vFilteredJoints.Append(vJoints[i]);
                }
            }

            // OCCT_DEBUG output left out: thin patches dropped

            var surfaces = new NCollection_Array2<BezierSurface>(1, nbUPatches, 1, nbVPatches);
            converter.Patches(surfaces);
            var srf = new NCollection_HArray2<Surface>(1, nbUPatches - nbUFiltered, 1, nbVPatches - nbVFiltered);
            int indApp1 = 0;
            for (int ind1 = 1; ind1 <= nbUPatches; ind1++)
            {
                if (uReject[ind1])
                {
                    continue;
                }
                indApp1++;
                int indApp2 = 0;
                for (int ind2 = 1; ind2 <= nbVPatches; ind2++)
                {
                    if (vReject[ind2])
                    {
                        continue;
                    }
                    indApp2++;
                    srf.SetValue(indApp1, indApp2, surfaces[ind1, ind2]);
                }
            }

            var uj = new NCollection_Array1<double>(1, uFilteredJoints.Length());
            for (i = 1; i <= uFilteredJoints.Length(); i++)
            {
                uj[i] = uFilteredJoints.Value(i);
            }

            var vj = new NCollection_Array1<double>(1, vFilteredJoints.Length());
            for (i = 1; i <= vFilteredJoints.Length(); i++)
            {
                vj[i] = vFilteredJoints.Value(i);
            }

            _segments = new CompositeSurface(srf, uj, vj);

            int j;
            for (j = 2; j <= _uSplitValues.Length(); j++)
            {
                uLast = _uSplitValues.Value(j);
                for (int ii = 2; ii <= nbUPatches + 1; ii++)
                {
                    double valknot = uJoints[ii];
                    if (valknot - uFirst <= precision)
                    {
                        continue;
                    }
                    if (uLast - valknot <= precision)
                    {
                        break;
                    }
                    _uSplitValues.InsertBefore(j++, valknot);
                }
                uFirst = uLast;
            }

            for (j = 2; j <= _vSplitValues.Length(); j++)
            {
                vLast = _vSplitValues.Value(j);
                for (int ii = 2; ii <= nbVPatches + 1; ii++)
                {
                    double valknot = vJoints[ii];
                    if (valknot - vFirst <= precision)
                    {
                        continue;
                    }
                    if (vLast - valknot <= precision)
                    {
                        break;
                    }
                    _vSplitValues.InsertBefore(j++, valknot);
                }
                vFirst = vLast;
            }
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            return;
        }
        else if (_surface is RevolutionSurface revol && _revolutionMode)
        {
            Curve3D basis = revol.BasisCurve;
            if (basis is TrimmedCurve3D tc)
            {
                basis = tc.BasisCurve;
            }
            NCollection_HArray1<Curve3D> curves;
            int nbCurves;
            var vPar = new NCollection_HSequence<double>();
            var vSVal = new NCollection_HSequence<double>();
            if (basis is OffsetCurve3D offset)
            {
                double value = offset.Offset;
                Direction3D direction = offset.Direction;
                Curve3D bas = offset.BasisCurve;
                var converter = new ConvertCurve3dToBezier();
                converter.Init(bas, vFirst, vLast);
                converter.Perform(true);
                if (converter.Status(ExtendStatus.Done))
                {
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
                else
                {
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
                }

                vPar.ChangeSequence().Assign(converter.SplitParams().Sequence());
                vSVal.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
                curves = converter.GetCurves()!;
                nbCurves = curves.Length();
                for (int i = 1; i <= nbCurves; i++)
                {
                    var offCur = new OffsetCurve3D(curves.Value(i), value, direction);
                    curves.SetValue(i, offCur);
                }
            }
            else
            {
                var converter = new ConvertCurve3dToBezier();
                converter.Init(basis, vFirst, vLast);
                converter.Perform(true);
                if (converter.Status(ExtendStatus.Done))
                {
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
                else
                {
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
                }

                vPar.ChangeSequence().Assign(converter.SplitParams().Sequence());
                vSVal.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
                curves = converter.GetCurves()!;
                nbCurves = curves.Length();
            }

            Axis3D axis = revol.Axis;
            var surf = new NCollection_HArray2<Surface>(1, 1, 1, nbCurves);
            _surface.ParameterBounds(out double umin, out double umax, out _, out _);
            int k;
            for (k = 1; k <= nbCurves; k++)
            {
                var rev = new RevolutionSurface(curves.Value(k), axis);
                if (uFirst - umin < Kerf.Engine.Foundation.Precision.PConfusion() && umax - uLast < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    surf.SetValue(1, k, rev);
                }
                else
                {
                    var rect = new TrimmedSurface(rev, uFirst, uLast, true);
                    surf.SetValue(1, k, rect);
                }
            }
            var uJoints = new NCollection_Array1<double>(1, 2);
            var vJoints = new NCollection_Array1<double>(1, nbCurves + 1);
            uJoints[1] = uFirst;
            uJoints[2] = uLast;
            for (k = 1; k <= nbCurves + 1; k++)
            {
                vJoints[k] = vPar.Value(k);
            }

            _segments = new CompositeSurface(surf, uJoints, vJoints);

            for (int j = 2; j <= _vSplitValues.Length(); j++)
            {
                vLast = _vSplitValues.Value(j);
                for (int ii = 2; ii <= nbCurves + 1; ii++)
                {
                    double valknot = vSVal.Value(ii);
                    if (valknot - vFirst <= precision)
                    {
                        continue;
                    }
                    if (vLast - valknot <= precision)
                    {
                        break;
                    }
                    _vSplitValues.InsertBefore(j++, valknot);
                }
                vFirst = vLast;
            }
            return;
        }
        else if (_surface is ExtrusionSurface extr && _extrusionMode)
        {
            Curve3D basis = extr.BasisCurve;
            // gp_Dir direction = extr->Direction(); // direction not used (skl)
            NCollection_HArray1<Curve3D> curves;
            int nbCurves;
            var uPar = new NCollection_HSequence<double>();
            var uSVal = new NCollection_HSequence<double>();
            var converter = new ConvertCurve3dToBezier();
            converter.Init(basis, uFirst, uLast);
            converter.Perform(true);
            uPar.ChangeSequence().Assign(converter.SplitParams().Sequence());
            uSVal.ChangeSequence().Assign(converter.SplitValues()!.Sequence());
            curves = converter.GetCurves()!;
            nbCurves = curves.Length();

            Transform3D shiftF = new(), shiftL = new();
            shiftF = shiftF.WithTranslation(extr.Value(uFirst, 0), extr.Value(uFirst, vFirst));
            shiftL = shiftL.WithTranslation(extr.Value(uFirst, 0), extr.Value(uFirst, vLast));
            var surf = new NCollection_HArray2<Surface>(1, nbCurves, 1, 1);

            int k;
            for (k = 1; k <= nbCurves; k++)
            {
                // OCCT down-casts the curve to a Bezier and dereferences it
                var bez = (BezierCurve3D)curves.Value(k);
                int nbPoles = bez.PoleCount;
                NCollection_Array1<Point3D> poles = bez.InternalPoles();
                var resPoles = new NCollection_Array2<Point3D>(1, nbPoles, 1, 2);
                for (int j = 1; j <= nbPoles; j++)
                {
                    resPoles[j, 1] = poles[j].Transformed(shiftF);
                    resPoles[j, 2] = poles[j].Transformed(shiftL);
                }
                var bezier = new BezierSurface(resPoles);
                surf.SetValue(k, 1, bezier);
            }

            var uJoints = new NCollection_Array1<double>(1, nbCurves + 1);
            var vJoints = new NCollection_Array1<double>(1, 2);
            vJoints[1] = vFirst;
            vJoints[2] = vLast;
            for (k = 1; k <= nbCurves + 1; k++)
            {
                uJoints[k] = uPar.Value(k);
            }

            _segments = new CompositeSurface(surf, uJoints, vJoints);

            for (int j = 2; j <= _uSplitValues.Length(); j++)
            {
                uLast = _uSplitValues.Value(j);
                for (int ii = 2; ii <= nbCurves + 1; ii++)
                {
                    double valknot = uSVal.Value(ii);
                    if (valknot + uFirst <= precision)
                    {
                        continue;
                    }
                    if (uLast - valknot <= precision)
                    {
                        break;
                    }
                    _uSplitValues.InsertBefore(j++, valknot);
                }
                uFirst = uLast;
            }
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            return;
        }
        else
        {
            var uJoints = new NCollection_Array1<double>(1, 2);
            uJoints[1] = uFirst;
            uJoints[2] = uLast;
            var vJoints = new NCollection_Array1<double>(1, 2);
            vJoints[1] = vFirst;
            vJoints[2] = vLast;
            var surf = new NCollection_HArray2<Surface>(1, 1, 1, 1);
            _surface!.ParameterBounds(out double U1, out double U2, out _, out double V2);
            Surface s;
            if (U1 - uFirst < precision && uLast - U2 < precision && V2 - vFirst < precision
                && vLast - V2 < precision)
            {
                s = _surface;
            }
            else
            {
                var rts = new TrimmedSurface(_surface, uFirst, uLast, vFirst, vLast);
                s = rts;
            }
            surf.SetValue(1, 1, s);
            _segments = new CompositeSurface(surf, uJoints, vJoints);
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
            return;
        }
    }

    // OCCT's file-static GetSegment: the part of a patch between the parameters.
    private static Surface GetSegment(Surface surf, double U1, double U2, double V1, double V2)
    {
        if (surf is BezierSurface)
        {
            var bezier = (BezierSurface)surf.Copy();
            double prec = Kerf.Engine.Foundation.Precision.PConfusion();
            if (U1 < prec && U2 > 1 - prec && V1 < prec && V2 > 1 - prec)
            {
                return bezier;
            }
            // pdn K4L+ (work around)
            // rln C30 (direct use)
            double u1 = U1;
            double u2 = U2;
            double v1 = V1;
            double v2 = V2;
            bezier.Segment(u1, u2, v1, v2);
            return bezier;
        }

        Surface s;
        if (surf is TrimmedSurface rect)
        {
            s = rect.BasisSurface;
        }
        else
        {
            s = surf;
        }

        if (s is RevolutionSurface)
        {
            var revol = (RevolutionSurface)s.Copy();
            revol.ParameterBounds(out double umin, out double umax, out _, out _);
            Curve3D basis = revol.BasisCurve;
            OffsetCurve3D? offset = null;
            if (basis is OffsetCurve3D anOffset)
            {
                offset = anOffset;
                basis = anOffset.BasisCurve;
            }
            if (basis is BezierCurve3D bezierBasis)
            {
                // Kerf: OCCT segments the copy's basis curve in place; the port's basis curves are shared and
                // frozen once handed out (decision 159), so the segment is made on a copy, rebuilt into the offset
                // curve if there is one and set as the copy's basis (decision 998)
                var segment = (BezierCurve3D)bezierBasis.Copy();
                segment.Segment(V1, V2);
                Curve3D newBasis = offset is null ? segment : new OffsetCurve3D(segment, offset.Offset, offset.Direction);
                revol.SetBasisCurve(newBasis);
            }
            else
            {
                // OCCT_DEBUG output left out: resulting path is not surface of revolution basis on bezier curve
            }
            if (Math.Abs(U1 - umin) < Kerf.Engine.Foundation.Precision.PConfusion() && Math.Abs(U2 - umax) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return revol;
            }

            var res = new TrimmedSurface(revol, U1, U2, true);
            return res;
        }
        else
        {
            surf.ParameterBounds(out double umin, out double umax, out double vmin, out double vmax);
            if (U1 - umin < Kerf.Engine.Foundation.Precision.PConfusion() && umax - U2 < Kerf.Engine.Foundation.Precision.PConfusion()
                && V1 - vmin < Kerf.Engine.Foundation.Precision.PConfusion() && vmax - V2 < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return surf;
            }

            var res = new TrimmedSurface(surf, U1, U2, V1, V2);
            return res;
        }
    }

    /// <summary>Cuts the grid of Bezier-based surfaces made by <see cref="Compute"/> at the splitting values.</summary>
    public override void Build(bool segment)
    {
        bool isOffset = false;
        double offsetValue = 0;
        Surface s;
        if (_surface is TrimmedSurface surface)
        {
            s = surface.BasisSurface;
        }
        else
        {
            s = _surface!;
        }
        if (s is OffsetSurface offSur)
        {
            isOffset = true;
            offsetValue = offSur.Offset;
        }

        double prec = Kerf.Engine.Foundation.Precision.PConfusion();
        NCollection_HArray1<double> myUSplitParams = _segments!.InternalUJointValues()!;
        NCollection_HArray1<double> myVSplitParams = _segments.InternalVJointValues()!;
        int nbU = _uSplitValues!.Length();
        int nbV = _vSplitValues!.Length();

        var resSurfaces = new NCollection_HArray2<Surface>(1, nbU - 1, 1, nbV - 1);
        int j1 = 2;
        for (int i1 = 2; i1 <= nbU; i1++)
        {
            double parU = _uSplitValues.Value(i1);
            for (; j1 <= myUSplitParams.Length(); j1++)
            {
                double param = myUSplitParams.Value(j1);
                if (parU - param < prec)
                {
                    break;
                }
            }

            int j2 = 2;
            for (int i2 = 2; i2 <= nbV; i2++)
            {
                double parV = _vSplitValues.Value(i2);
                for (; j2 <= myVSplitParams.Length(); j2++)
                {
                    if (parV - myVSplitParams.Value(j2) < prec)
                    {
                        break;
                    }
                }

                Surface patch = _segments.Patch(j1 - 1, j2 - 1);
                patch.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
                // linear recomputation of part:
                double uFirst = myUSplitParams.Value(j1 - 1);
                double uLast = myUSplitParams.Value(j1);
                double vFirst = myVSplitParams.Value(j2 - 1);
                double vLast = myVSplitParams.Value(j2);
                double uFact = (u2 - u1) / (uLast - uFirst);
                double vFact = (v2 - v1) / (vLast - vFirst);
                double ppU = _uSplitValues.Value(i1 - 1);
                double ppV = _vSplitValues.Value(i2 - 1);
                // defining a part
                double uL1 = u1 + (ppU - uFirst) * uFact;
                double uL2 = u1 + (parU - uFirst) * uFact;
                double vL1 = v1 + (ppV - vFirst) * vFact;
                double vL2 = v1 + (parV - vFirst) * vFact;
                Surface res = GetSegment(patch, uL1, uL2, vL1, vL2);
                if (isOffset)
                {
                    var resOff = new OffsetSurface(res, offsetValue);
                    res = resOff;
                }
                resSurfaces.SetValue(i1 - 1, i2 - 1, res);
            }
        }

        var uJoints = new NCollection_Array1<double>(1, nbU);
        int i;
        for (i = 1; i <= nbU; i++)
        {
            uJoints[i] = _uSplitValues.Value(i);
        }

        var vJoints = new NCollection_Array1<double>(1, nbV);
        for (i = 1; i <= nbV; i++)
        {
            vJoints[i] = _vSplitValues.Value(i);
        }

        _resSurfaces = new CompositeSurface(resSurfaces, uJoints, vJoints);
    }

    /// <summary>Returns the grid of Bezier-based surfaces made from the surface.</summary>
    public CompositeSurface? Segments() => _segments;

    /// <summary>Sets the mode for converting Geom_Plane to a Bezier.</summary>
    public void SetPlaneMode(bool mode)
    {
        _planeMode = mode;
    }

    /// <summary>Returns the Geom_Plane conversion mode.</summary>
    public bool GetPlaneMode() => _planeMode;

    /// <summary>Sets the mode for converting Geom_SurfaceOfRevolution to Beziers.</summary>
    public void SetRevolutionMode(bool mode)
    {
        _revolutionMode = mode;
    }

    /// <summary>Returns the Geom_SurfaceOfRevolution conversion mode.</summary>
    public bool GetRevolutionMode() => _revolutionMode;

    /// <summary>Sets the mode for converting Geom_SurfaceOfLinearExtrusion to Beziers.</summary>
    public void SetExtrusionMode(bool mode)
    {
        _extrusionMode = mode;
    }

    /// <summary>Returns the Geom_SurfaceOfLinearExtrusion conversion mode.</summary>
    public bool GetExtrusionMode() => _extrusionMode;

    /// <summary>Sets the mode for converting Geom_BSplineSurface to Beziers.</summary>
    public void SetBSplineMode(bool mode)
    {
        _bSplineMode = mode;
    }

    /// <summary>Returns the Geom_BSplineSurface conversion mode.</summary>
    public bool GetBSplineMode() => _bSplineMode;
}
