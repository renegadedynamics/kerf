// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Splits a surface with a criterion (the base class splits at the U and V values given) into a grid of patches,
/// returned as a <see cref="CompositeSurface"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). The surface given is not copied (OCCT's isn't); the patches are segments or
/// trimmed copies of it, or the surface itself when nothing is cut.
/// </remarks>
internal class SplitSurface : Transient
{
    /// <summary>The U splitting values, the first and last included.</summary>
    protected NCollection_HSequence<double>? _uSplitValues;
    /// <summary>The V splitting values, the first and last included.</summary>
    protected NCollection_HSequence<double>? _vSplitValues;
    /// <summary>The number of rows of the resulting grid.</summary>
    protected int _nbResultingRow;
    /// <summary>The number of columns of the resulting grid.</summary>
    protected int _nbResultingCol;
    /// <summary>The surface.</summary>
    protected Surface? _surface;
    /// <summary>The status.</summary>
    protected int _status;
    /// <summary>The resulting grid.</summary>
    protected CompositeSurface? _resSurfaces;
    /// <summary>The area of the face (0 when not given).</summary>
    protected double _area;
    /// <summary>The length of the middle V iso (with an area).</summary>
    protected double _usize;
    /// <summary>The length of the middle U iso (with an area).</summary>
    protected double _vsize;

    /// <summary>Empty constructor.</summary>
    public SplitSurface()
    {
        _nbResultingRow = 0;
        _nbResultingCol = 0;
        _status = 0;
    }

    /// <summary>Initializes with a single supporting surface (its bounds).</summary>
    public void Init(Surface s)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        _uSplitValues = new NCollection_HSequence<double>();
        _vSplitValues = new NCollection_HSequence<double>();
        _surface = s;
        _resSurfaces = new CompositeSurface();
        _nbResultingRow = 1;
        _nbResultingCol = 1;
        _surface.ParameterBounds(out double u1, out double u2, out double v1, out double v2);

        _uSplitValues.Append(u1);
        _uSplitValues.Append(u2);

        _vSplitValues.Append(v1);
        _vSplitValues.Append(v2);
    }

    /// <summary>
    /// Initializes with a single supporting surface and bounding parameters (and, when <paramref name="area"/> isn't
    /// 0, the face's area, measuring the middle isos for splitting by area).
    /// </summary>
    public void Init(Surface s, double uFirst, double uLast, double vFirst, double vLast, double area = 0.0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        _surface = s;
        _resSurfaces = new CompositeSurface();
        _uSplitValues = new NCollection_HSequence<double>();
        _vSplitValues = new NCollection_HSequence<double>();

        _nbResultingRow = 1;
        _nbResultingCol = 1;

        _area = area;

        _surface.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        if (_surface.IsUPeriodic && uLast - uFirst <= u2 - u1 + precision)
        {
            u1 = uFirst;
            u2 = u1 + _surface.UPeriod;
        }
        if (_surface.IsVPeriodic && vLast - vFirst <= v2 - v1 + precision)
        {
            v1 = vFirst;
            v2 = v1 + _surface.VPeriod;
        }
        double uf, ul, vf, vl;
        if (uFirst > u2 - precision || uLast < u1 - precision)
        {
            uf = u1;
            ul = u2;
        }
        else
        {
            uf = Math.Max(u1, uFirst);
            ul = Math.Min(u2, uLast);
        }
        if (vFirst > v2 - precision || vLast < v1 - precision)
        {
            vf = v1;
            vl = v2;
        }
        else
        {
            vf = Math.Max(v1, vFirst);
            vl = Math.Min(v2, vLast);
        }

        if (_area != 0.0)
        {
            //<myArea> is set and will be used with <myUsize> and <myVsize>
            // in further computations
            double umid = (uf + ul) / 2, vmid = (vf + vl) / 2;
            var trSurf = new TrimmedSurface(_surface, uf, ul, vf, vl);
            Curve3D uiso = trSurf.UIso(umid);
            Curve3D viso = trSurf.VIso(vmid);
            Edge edgeUiso = new LibraryMakeEdge(uiso).InternalEdge();
            Edge edgeViso = new LibraryMakeEdge(viso).InternalEdge();
            Properties gprop1 = new(), gprop2 = new();
            MassPropertiesBuilder.LinearProperties(edgeViso, gprop1);
            _usize = gprop1.Mass();
            MassPropertiesBuilder.LinearProperties(edgeUiso, gprop2);
            _vsize = gprop2.Mass();
        }

        if (ul - uf < precision)
        {
            double p2 = precision / 2.0;
            uf -= p2;
            ul += p2;
        }
        if (vl - vf < precision)
        {
            double p2 = precision / 2.0;
            vf -= p2;
            vl += p2;
        }

        _uSplitValues.Append(uf);
        _uSplitValues.Append(ul);
        _vSplitValues.Append(vf);
        _vSplitValues.Append(vl);
    }

    /// <summary>Sets the U parameters where splitting has to be done (those inside the current intervals).</summary>
    public void SetUSplitValues(NCollection_HSequence<double>? uValues)
    {
        if (uValues is null)
        {
            return;
        }
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double uFirst = _uSplitValues!.Value(1), uLast = _uSplitValues.Value(_uSplitValues.Length());
        int i = 1;
        int len = uValues.Length();

        for (int ku = 2; ku <= _uSplitValues.Length(); ku++)
        {
            uLast = _uSplitValues.Value(ku);
            for (; i <= len; i++)
            {
                if ((uFirst + precision) >= uValues.Value(i))
                {
                    continue;
                }
                if ((uLast - precision) <= uValues.Value(i))
                {
                    break;
                }
                _uSplitValues.InsertBefore(ku++, uValues.Value(i));
            }
            uFirst = uLast;
        }
    }

    /// <summary>Sets the V parameters where splitting has to be done (those inside the current intervals).</summary>
    public void SetVSplitValues(NCollection_HSequence<double>? vValues)
    {
        if (vValues is null)
        {
            return;
        }
        double precision = Kerf.Engine.Foundation.Precision.PConfusion();
        double vFirst = _vSplitValues!.Value(1), vLast = _vSplitValues.Value(_vSplitValues.Length());
        int i = 1;
        int len = vValues.Length();
        for (int kv = 2; kv <= _vSplitValues.Length(); kv++)
        {
            vLast = _vSplitValues.Value(kv);
            for (; i <= len; i++)
            {
                if ((vFirst + precision) >= vValues.Value(i))
                {
                    continue;
                }
                if ((vLast - precision) <= vValues.Value(i))
                {
                    break;
                }
                _vSplitValues.InsertBefore(kv++, vValues.Value(i));
            }
            vFirst = vLast;
        }
    }

    /// <summary>
    /// Performs the splitting of the surface: with <paramref name="segment"/> a B-spline or Bezier result is made of
    /// segments of the surface bounded by the splitting values, otherwise of rectangular trimmed surfaces on the
    /// same surface.
    /// </summary>
    public virtual void Build(bool segment)
    {
        double uFirst = _uSplitValues!.Value(1);
        double uLast = _uSplitValues.Value(_uSplitValues.Length());
        double vFirst = _vSplitValues!.Value(1);
        double vLast = _vSplitValues.Value(_vSplitValues.Length());

        if (_uSplitValues.Length() > 2 || _vSplitValues.Length() > 2)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        _surface!.ParameterBounds(out double U1, out double U2, out double V1, out double V2);
        if (_surface is RevolutionSurface surfaceRev)
        {
            Curve3D basCurve = surfaceRev.BasisCurve;
            var spc = new SplitCurve3d();
            spc.Init(basCurve, vFirst, vLast);
            spc.SetSplitValues(_vSplitValues);
            spc.Build(segment);
            NCollection_HArray2<Surface> surfaces;
            _nbResultingCol = spc.GetCurves()!.Length();
            if (_uSplitValues.Length() > 2)
            {
                _nbResultingRow = _uSplitValues.Length() - 1;
                surfaces = new NCollection_HArray2<Surface>(1, _nbResultingRow, 1, _nbResultingCol);
                for (int nc = 1; nc <= _nbResultingCol; nc++)
                {
                    var newSurfaceRev = new RevolutionSurface(spc.GetCurves()!.Value(nc), surfaceRev.Axis);
                    newSurfaceRev.ParameterBounds(out double u1p, out double u2p, out double v1p, out double v2p);
                    for (int nc1 = 1; nc1 <= _nbResultingRow; nc1++)
                    {
                        var newSurf = new TrimmedSurface(newSurfaceRev,
                                                                         _uSplitValues.Value(nc1),
                                                                         _uSplitValues.Value(nc1 + 1),
                                                                         v1p,
                                                                         v2p);
                        surfaces.SetValue(nc1, nc, newSurf);
                    }
                }
            }
            else
            {
                surfaces = new NCollection_HArray2<Surface>(1, 1, 1, _nbResultingCol);

                for (int nc = 1; nc <= spc.GetCurves()!.Length(); nc++)
                {
                    var newSurfaceRev = new RevolutionSurface(spc.GetCurves()!.Value(nc), surfaceRev.Axis);
                    newSurfaceRev.ParameterBounds(out U1, out U2, out V1, out V2);
                    if (uFirst == U1 && uLast == U2)
                    {
                        surfaces.SetValue(1, nc, newSurfaceRev);
                    }
                    else
                    {
                        var newSurf = new TrimmedSurface(newSurfaceRev, uFirst, uLast, V1, V2); // pdn correction for main seq
                        surfaces.SetValue(1, nc, newSurf);
                    }
                }
            }
            _resSurfaces!.Init(surfaces);
            _resSurfaces.SetUFirstValue(_uSplitValues.Sequence().First());
            _resSurfaces.SetVFirstValue(_vSplitValues.Sequence().First());
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
            Curve3D basCurve = surfaceExt.BasisCurve;
            var spc = new SplitCurve3d();
            spc.Init(basCurve, uFirst, uLast);
            spc.SetSplitValues(_uSplitValues);
            spc.Build(segment);
            _nbResultingRow = spc.GetCurves()!.Length();
            NCollection_HArray2<Surface> surfaces;
            if (_vSplitValues.Length() > 2)
            {
                _nbResultingCol = _vSplitValues.Length() - 1;
                surfaces = new NCollection_HArray2<Surface>(1, _nbResultingRow, 1, _nbResultingCol);
                for (int nc1 = 1; nc1 <= _nbResultingRow; nc1++)
                {
                    var newSurfaceEx = new ExtrusionSurface(spc.GetCurves()!.Value(nc1), surfaceExt.Direction);
                    newSurfaceEx.ParameterBounds(out double u1p, out double u2p, out double v1p, out double v2p);
                    for (int nc2 = 1; nc2 <= _nbResultingCol; nc2++)
                    {
                        var newSurf = new TrimmedSurface(newSurfaceEx,
                                                                         u1p,
                                                                         u2p,
                                                                         _vSplitValues.Value(nc2),
                                                                         _vSplitValues.Value(nc2 + 1));
                        surfaces.SetValue(nc1, nc2, newSurf);
                    }
                }
            }
            else
            {
                surfaces = new NCollection_HArray2<Surface>(1, _nbResultingRow, 1, 1);

                for (int nc1 = 1; nc1 <= _nbResultingRow; nc1++)
                {
                    var newSurfaceEx = new ExtrusionSurface(spc.GetCurves()!.Value(nc1), surfaceExt.Direction);
                    newSurfaceEx.ParameterBounds(out U1, out U2, out V1, out V2);
                    if (vFirst == V1 && vLast == V2)
                    {
                        surfaces.SetValue(nc1, 1, newSurfaceEx);
                    }
                    else
                    {
                        var newSurf = new TrimmedSurface(newSurfaceEx,
                                                                         Math.Max(U1, uFirst),
                                                                         Math.Min(uLast, U2),
                                                                         Math.Max(vFirst, V1),
                                                                         Math.Min(vLast, V2));
                        surfaces.SetValue(nc1, 1, newSurf);
                    }
                }
            }
            _resSurfaces!.Init(surfaces);
            _resSurfaces.SetUFirstValue(_uSplitValues.Sequence().First());
            _resSurfaces.SetVFirstValue(_vSplitValues.Sequence().First());
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

        if (_surface is TrimmedSurface tmpRect)
        {
            Surface theSurf = tmpRect.BasisSurface;
            var sps = new SplitSurface();
            sps.Init(theSurf, uFirst, uLast, vFirst, vLast);
            sps.SetUSplitValues(_uSplitValues);
            sps.SetVSplitValues(_vSplitValues);
            sps._status = _status;
            sps.Build(segment);
            _status |= sps._status;
            _resSurfaces = sps._resSurfaces;
            return;
        }
        else if (_surface is OffsetSurface tmpOffset)
        {
            Surface theSurf = tmpOffset.BasisSurface;
            var sps = new SplitSurface();
            sps.Init(theSurf, uFirst, uLast, vFirst, vLast);
            sps.SetUSplitValues(_uSplitValues);
            sps.SetVSplitValues(_vSplitValues);
            sps._status = _status;
            sps.Build(segment);
            var patches = new NCollection_HArray2<Surface>(1,
                                                                 sps.ResSurfaces()!.UPatchCount,
                                                                 1,
                                                                 sps.ResSurfaces()!.VPatchCount);
            for (int i1 = 1; i1 <= sps.ResSurfaces()!.UPatchCount; i1++)
            {
                for (int j1 = 1; j1 <= sps.ResSurfaces()!.VPatchCount; j1++)
                {
                    var newOffSur = new OffsetSurface(sps.ResSurfaces()!.Patch(i1, j1), tmpOffset.Offset);
                    patches.SetValue(i1, j1, newOffSur);
                }
            }
            _resSurfaces!.Init(patches);
            _resSurfaces.SetUFirstValue(_uSplitValues.Sequence().First());
            _resSurfaces.SetVFirstValue(_vSplitValues.Sequence().First());
            _status |= sps._status;
            return;
        }

        // splitting the surfaces:
        _nbResultingRow = _uSplitValues.Length() - 1;
        _nbResultingCol = _vSplitValues.Length() - 1;
        var surfacesGrid = new NCollection_HArray2<Surface>(1, _nbResultingRow, 1, _nbResultingCol);
        bool isBSpline = _surface is BSplineSurface;
        bool isBezier = _surface is BezierSurface;

        if (_nbResultingRow == 1 && _nbResultingCol == 1)
        {
            _surface.ParameterBounds(out U1, out U2, out V1, out V2);
            bool filled = true;
            if (Math.Abs(U1 - uFirst) < Kerf.Engine.Foundation.Precision.PConfusion()
                && Math.Abs(U2 - uLast) < Kerf.Engine.Foundation.Precision.PConfusion()
                && Math.Abs(V1 - vFirst) < Kerf.Engine.Foundation.Precision.PConfusion()
                && Math.Abs(V2 - vLast) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                surfacesGrid.SetValue(1, 1, _surface);
            }
            else if (!segment || _surface is not BSplineSurface || !Status(ExtendStatus.DONE2))
            {
                // pdn copying of surface
                var tmp = (Surface)_surface.Copy();
                var surf = new TrimmedSurface(tmp, uFirst, uLast, vFirst, vLast);
                surfacesGrid.SetValue(1, 1, surf);
            }
            else
            {
                filled = false;
            }
            if (filled)
            {
                _resSurfaces!.Init(surfacesGrid);
                _resSurfaces.SetUFirstValue(_uSplitValues.Sequence().First());
                _resSurfaces.SetVFirstValue(_vSplitValues.Sequence().First());
                return;
            }
        }

        if (_surface is BSplineSurface)
        {
            var bsSurface = (BSplineSurface)_surface.Copy();
            int firstInd = bsSurface.FirstUKnotIndex(), lastInd = bsSurface.LastUKnotIndex();
            int j = firstInd;
            for (int ii = 1; ii <= _uSplitValues.Length(); ii++)
            {
                double spval = _uSplitValues.Value(ii);
                for (; j <= lastInd; j++)
                {
                    if (spval > bsSurface.UKnot(j) + Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }
                    if (spval < bsSurface.UKnot(j) - Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        break;
                    }
                    _uSplitValues.ChangeValue(ii) = bsSurface.UKnot(j);
                }
                if (j == lastInd)
                {
                    break;
                }
            }
            firstInd = bsSurface.FirstVKnotIndex();
            lastInd = bsSurface.LastVKnotIndex();
            j = firstInd;
            for (int ii1 = 1; ii1 <= _vSplitValues.Length(); ii1++)
            {
                double spval = _vSplitValues.Value(ii1);
                for (; j <= lastInd; j++)
                {
                    if (spval > bsSurface.VKnot(j) + Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }
                    if (spval < bsSurface.VKnot(j) - Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        break;
                    }
                    _vSplitValues.ChangeValue(ii1) = bsSurface.VKnot(j);
                }
                if (j == lastInd)
                {
                    break;
                }
            }
        }

        U1 = _uSplitValues.Value(1);
        V1 = _vSplitValues.Value(1);
        for (int irow = 2; irow <= _uSplitValues.Length(); irow++)
        {
            U2 = _uSplitValues.Value(irow);
            for (int icol = 2; icol <= _vSplitValues.Length(); icol++)
            {
                V2 = _vSplitValues.Value(icol);
                // creates a copy of theSurf before to segment:
                var theNew = (Surface)_surface.Copy();
                if (isBSpline || isBezier)
                {
                    try
                    {
                        if (isBSpline)
                        {
                            ((BSplineSurface)theNew).Segment(U1, U2, V1, V2);
                        }
                        else if (isBezier)
                        {
                            // pdn K4L+ (work around)
                            // rln C30 (direct use)
                            double u1 = U1;
                            double u2 = U2;
                            double v1 = V1;
                            double v2 = V2;
                            ((BezierSurface)theNew).Segment(u1, u2, v1, v2);
                        }
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                    }
                    catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                    {
                        // OCCT_DEBUG output left out
                        Surface theNewSurf = theNew;
                        theNew = new TrimmedSurface(theNewSurf, U1, U2, V1, V2);
                    }
                    surfacesGrid.SetValue((irow - 1), (icol - 1), theNew);
                }
                else
                {
                    // not a BSpline: trimming instead of segmentation
                    var splittedSurf = new TrimmedSurface(theNew, U1, U2, V1, V2);
                    surfacesGrid.SetValue((irow - 1), (icol - 1), splittedSurf);
                }

                V1 = V2;
            }
            U1 = U2;
            V1 = _vSplitValues.Value(1);
        }
        int nbU = _uSplitValues.Length();
        var uJoints = new NCollection_Array1<double>(1, nbU);
        int i;
        for (i = 1; i <= nbU; i++)
        {
            uJoints[i] = _uSplitValues.Value(i);
        }

        int nbV = _vSplitValues.Length();
        var vJoints = new NCollection_Array1<double>(1, nbV);
        for (i = 1; i <= nbV; i++)
        {
            vJoints[i] = _vSplitValues.Value(i);
        }
        _resSurfaces!.Init(surfacesGrid, uJoints, vJoints);
    }

    /// <summary>Calculates the splitting values; the base class only resets the status.</summary>
    public virtual void Compute(bool segment = true)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>Computes the splitting values (<see cref="Compute"/>), then splits the surface (<see cref="Build"/>).</summary>
    public void Perform(bool segment = true)
    {
        Compute(segment);
        //  SetUSplitValues(myUSplitValues);
        //  SetVSplitValues(myVSplitValues);
        Build(segment);
    }

    /// <summary>Returns all the U splitting values, the first and last parameters of the surface included.</summary>
    public NCollection_HSequence<double>? USplitValues() => _uSplitValues;

    /// <summary>Returns all the V splitting values, the first and last parameters of the surface included.</summary>
    public NCollection_HSequence<double>? VSplitValues() => _vSplitValues;

    /// <summary>
    /// Returns the status: OK (no splitting needed), DONE1 (splitting gives more than one patch), DONE2 (splitting
    /// required, a single patch), DONE3 (the surface's form or parametrisation is modified).
    /// </summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Returns the surfaces obtained by splitting, as a composite surface.</summary>
    public CompositeSurface? ResSurfaces() => _resSurfaces;
}
