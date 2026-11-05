// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ClosedFaceDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ClosedFaceDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Divides a closed face (one with a seam, or on a geometrically closed surface) into the given number of parts plus
/// one, across the seam's direction.
/// </summary>
/// <remarks>A working object (decision 21). The face given is not modified (see <see cref="FaceDivide"/>).</remarks>
internal class ClosedFaceDivide : FaceDivide
{
    private int _nbSplit;

    /// <summary>Empty constructor (one splitting line).</summary>
    public ClosedFaceDivide()
    {
        _nbSplit = 1;
    }

    /// <summary>Initializes by a face.</summary>
    public ClosedFaceDivide(in Face f) : base(f)
    {
        _nbSplit = 1;
    }

    /// <summary>Splits the surface and computes the shell from the face; true if it was split.</summary>
    public override bool SplitSurface(double area = 0.0)
    {
        SplitSurface? splitSurf = GetSplitSurfaceTool();
        if (splitSurf is null)
        {
            return false;
        }

        if (_result.IsNull || _result.Type != ShapeType.Face)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }
        Face face = ShapeCasts.Face(_result);

        ShapeAnalysisAnalysis.GetFaceUVBounds(_face, out double uf, out double ul, out double vf, out double vl);
        // 01.10.99 pdn Porting on DEC
        if (Kerf.Engine.Foundation.Precision.IsInfinite(uf) || Kerf.Engine.Foundation.Precision.IsInfinite(ul) || Kerf.Engine.Foundation.Precision.IsInfinite(vf)
            || Kerf.Engine.Foundation.Precision.IsInfinite(vl))
        {
            return false;
        }

        Surface surf = Tool.Surface(face, out Placement l)!;

        bool isUSplit = false;
        bool doSplit = false;
        var split = new NCollection_HSequence<double>();

        for (var iter = new Iterator(face); iter.More() && !doSplit; iter.Next())
        {
            if (iter.Value().Type != ShapeType.Wire)
            {
                continue;
            }
            Wire wire = ShapeCasts.Wire(iter.Value());
            var sewd = new ExtendWireData(wire);
            for (int i = 1; i <= sewd.EdgeCount && !doSplit; i++)
            {
                if (sewd.IsSeam(i))
                {
                    doSplit = true;
                    Edge edge = sewd.Edge(i);
                    var sae = new EdgeAnalysis();
                    if (!sae.PCurve(edge, face, out Curve2D? c1, out double f1, out double l1, false))
                    {
                        continue;
                    }
                    Shape tmpE = edge.Reversed();
                    if (!sae.PCurve(ShapeCasts.Edge(tmpE), face, out Curve2D? c2, out double f2, out double l2, false))
                    {
                        continue;
                    }
                    if (ReferenceEquals(c2, c1))
                    {
                        continue;
                    }
                    // splitting
                    var sac = new CurveAnalysis();
                    var b1 = new Box2d();
                    var b2 = new Box2d();
                    sac.FillBndBox(c1!, f1, l1, 20, true, ref b1);
                    sac.FillBndBox(c2!, f2, l2, 20, true, ref b2);
                    b1.Get(out double x1min, out double y1min, out double x1max, out double y1max);
                    b2.Get(out double x2min, out double y2min, out double x2max, out double y2max);
                    double xf, xl, yf, yl;
                    if (x1min < x2min)
                    {
                        xf = x1max;
                        xl = x2min;
                    }
                    else
                    {
                        xf = x2max;
                        xl = x1min;
                    }
                    if (y1min < y2min)
                    {
                        yf = y1max;
                        yl = y2min;
                    }
                    else
                    {
                        yf = y2max;
                        yl = y1min;
                    }

                    double dU = xl - xf;
                    double dV = yl - yf;
                    if (dU > dV)
                    {
                        double step = dU / (_nbSplit + 1);
                        double val = xf + step;
                        for (int j = 1; j <= _nbSplit; j++, val += step)
                        {
                            split.Append(val);
                        }
                        isUSplit = true;
                    }
                    else
                    {
                        double step = dV / (_nbSplit + 1);
                        double val = yf + step;
                        for (int j = 1; j <= _nbSplit; j++, val += step)
                        {
                            split.Append(val);
                        }
                        isUSplit = false;
                    }
                }
            }
        }

        if (!doSplit)
        {
            // pdn try to define geometric closure.
            var sas = new SurfaceAnalysis(surf);
            bool uclosed = sas.IsUClosed(Precision());
            bool vclosed = sas.IsVClosed(Precision());
            double u1, u2, v1, v2;
            if (uclosed)
            {
                surf.ParameterBounds(out u1, out u2, out v1, out v2);
                var gas = new SurfaceAdaptor(surf);
                double toler = gas.UResolution(Precision());
                if ((u2 - u1) - (ul - uf) < toler)
                {
                    var rts = new TrimmedSurface(surf, u1, (u2 + u1) / 2, true);
                    var sast = new SurfaceAnalysis(rts);
                    if (!sast.IsUClosed(Precision()))
                    {
                        doSplit = true;
                        double step = (ul - uf) / (_nbSplit + 1);
                        double val = uf + step;
                        for (int i = 1; i <= _nbSplit; i++, val += step)
                        {
                            split.Append(val);
                        }
                        isUSplit = true;
                    }
                    // OCCT_DEBUG output left out: thin face, not split
                }
            }

            if (vclosed && !doSplit)
            {
                surf.ParameterBounds(out u1, out u2, out v1, out v2);
                var gas = new SurfaceAdaptor(surf);
                double toler = gas.VResolution(Precision());
                if ((v2 - v1) - (vl - vf) < toler)
                {
                    var rts = new TrimmedSurface(surf, v1, (v2 + v1) / 2, false);
                    var sast = new SurfaceAnalysis(rts);
                    if (!sast.IsVClosed(Precision()))
                    {
                        doSplit = true;
                        double step = (vl - vf) / (_nbSplit + 1);
                        double val = vf + step;
                        for (int i = 1; i <= _nbSplit; i++, val += step)
                        {
                            split.Append(val);
                        }
                        isUSplit = false;
                    }
                    // OCCT_DEBUG output left out: thin face, not split
                }
            }
        }

        if (!doSplit)
        {
            return false;
        }

        splitSurf.Init(surf, uf, ul, vf, vl);
        if (isUSplit)
        {
            splitSurf.SetUSplitValues(split);
        }
        else
        {
            splitSurf.SetVSplitValues(split);
        }

        splitSurf.Perform(_segmentMode);
        if (!splitSurf.Status(ExtendStatus.Done))
        {
            return false;
        }

        CompositeSurface grid = splitSurf.ResSurfaces()!;

        var compShell = new ComposeShellFixer();
        compShell.Init(grid, l, face, Precision());
        compShell.SetMaxTolerance(MaxTolerance());
        compShell.SetContext(Context());
        compShell.Perform();
        if (compShell.Status(ExtendStatus.Fail) || !compShell.Status(ExtendStatus.Done))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }

        // Kerf: the result unfrozen, the caller's sub-shapes given back where the face was copied (decision 999)
        Shape res = compShell.InternalFinishedResult();
        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        for (var exp = new Explorer(res, ShapeType.Face); exp.More(); exp.Next())
        {
            Shape tempf = Context()!.InternalApply(exp.Current());
            Face f = ShapeCasts.Face(tempf);
            _result = f;
            if (SplitSurface())
            {
                Context()!.Replace(f, _result);
            }
        }
        _result = Context()!.InternalApply(res);
        return true;
    }

    /// <summary>Sets the number of cutting lines the closed face is split by (the resulting faces are num + 1).</summary>
    public void SetNbSplitPoints(int num)
    {
        if (num > 0)
        {
            _nbSplit = num;
        }
    }

    /// <summary>Returns the number of splitting points.</summary>
    public int GetNbSplitPoints() => _nbSplit;
}
