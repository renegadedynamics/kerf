// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCompositeCurve.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCompositeCurve.lxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCompositeCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Engine.Exchange.Step.Geometry.StepTransitionCode;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>
/// Translates a composite_curve into a TopoDS_Wire. If a surface is given, the curve is assumed to lie on it, and
/// where a segment is a curve on surface, its pcurve is taken.
/// </summary>
/// <remarks>
/// A segment may itself be a composite_curve; only one-level protection against cyclic references is implemented.
/// <see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).
/// </remarks>
internal sealed class TranslateCompositeCurve : ImportRoot
{
    private Wire _wire = new Wire();
    private bool _infiniteSegment;

    public TranslateCompositeCurve()
    {
        _infiniteSegment = false;
    }

    /// <summary>Translates a standalone composite_curve.</summary>
    public TranslateCompositeCurve(StepCompositeCurve cc, TransientProcess tp, in Factors localFactors)
    {
        Init(cc, tp, localFactors);
    }

    public TranslateCompositeCurve(StepCompositeCurve cc, TransientProcess tp)
        : this(cc, tp, new Factors())
    {
    }

    /// <summary>Translates a composite_curve lying on a surface.</summary>
    public TranslateCompositeCurve(StepCompositeCurve cc, TransientProcess tp, StepSurface s, Surface surf,
                                                in Factors localFactors)
    {
        Init(cc, tp, s, surf, localFactors);
    }

    public TranslateCompositeCurve(StepCompositeCurve cc, TransientProcess tp, StepSurface s, Surface surf)
        : this(cc, tp, s, surf, new Factors())
    {
    }

    /// <summary>Translates a standalone composite_curve.</summary>
    public bool Init(StepCompositeCurve cc, TransientProcess tp, in Factors localFactors)
    {
        StepSurface s = null;
        Surface surf = null;
        return Init(cc, tp, s, surf, localFactors);
    }

    public bool Init(StepCompositeCurve cc, TransientProcess tp) => Init(cc, tp, new Factors());

    /// <summary>Translates a composite_curve lying on a surface.</summary>
    public bool Init(StepCompositeCurve CC, TransientProcess tp, StepSurface s, Surface surf, in Factors localFactors)
    {
        _wire.Nullify();
        _infiniteSegment = false;
        if (CC is null)
        {
            return false;
        }

        bool surfMode = (s is not null && surf is not null);
        bool isClosed = false;
        StepModel stepModel = tp.Model() as StepModel;

        if (surfMode)
        {
            int modepcurve = (int)stepModel.InternalParameters.ReadSurfaceCurveMode;
            if (modepcurve == -3)
            {
                surfMode = false;
            }
        }

        var sbwd = new ExtendWireData();
        int nbs = CC.NbSegments();
        for (int i = 1; i <= nbs; i++)
        {
            StepCompositeCurveSegment ccs = CC.SegmentsValue(i);
            if (ccs is null)
            {
                tp.AddFail(CC, "Null segment");
                return false;
            }

            StepCurve crv = ccs.ParentCurve();
            if (crv is null)
            {
                tp.AddFail(CC, "Segment has null parent curve");
                return false;
            }

            isClosed = (ccs.Transition() != StepTransitionCode.TcDiscontinuous);

            // if segment is itself a composite_curve, translate recursively
            if (crv is StepCompositeCurve)
            {
                if (crv == CC)
                { // cyclic reference protection
                    tp.AddFail(ccs, "Cyclic reference; segment dropped");
                    continue;
                }

                StepCompositeCurve cc = crv as StepCompositeCurve;
                if (!Init(cc, tp, s, surf, localFactors) || _wire.IsNull)
                {
                    continue;
                }

                int nb = sbwd.EdgeCount + 1;
                for (var it = new Iterator(_wire); it.More(); it.Next())
                {
                    Edge edge = ShapeCasts.Edge(it.Value());
                    if (ccs.SameSense())
                    {
                        sbwd.Add(edge);
                    }
                    else
                    {
                        edge.Reverse();
                        sbwd.Add(edge, nb > sbwd.EdgeCount ? 0 : nb);
                    }
                }

                _wire.Nullify();
                continue;
            }

            // ordinary segment

            // detect pcurve and 3d curve
            StepPcurve pcurve = crv as StepPcurve;
            if (pcurve is null)
            {
                StepSurfaceCurve sc = crv as StepSurfaceCurve;
                if (sc is not null)
                {
                    crv = sc.Curve3d();
                    if (surfMode)
                    { // find proper pcurve
                        for (int j = 1; j <= sc.AssociatedGeometryCount; j++)
                        {
                            StepPcurveOrSurface pCorS = sc.AssociatedGeometryValue(j);
                            StepPcurve pc = pCorS?.Pcurve(); // a null SELECT: C++ has an empty one (decision 1254)
                            if (pc is null || pc.BasisSurface() != s)
                            {
                                continue;
                            }

                            pcurve = pc;
                            if (ccs.SameSense())
                            {
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                if (!surfMode || pcurve.BasisSurface() != s)
                {
                    pcurve = null;
                }

                crv = null;
            }

            // prepare edge
            Edge edge1 = new Edge();

            // translate 3d curve, if present
            if (crv is not null)
            {
                try
                {
                    Curve3D c3d = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCurve(crv, localFactors);
                    if (c3d is not null)
                    {
                        var mkEdge = new MakeEdge(c3d, c3d.FirstParameter, c3d.LastParameter);
                        if (mkEdge.IsDone)
                        {
                            if (Kerf.Engine.Foundation.Precision.IsNegativeInfinite(c3d.FirstParameter)
                                || Kerf.Engine.Foundation.Precision.IsPositiveInfinite(c3d.LastParameter))
                            {
                                _infiniteSegment = true;
                                tp.AddWarning(CC, "Segment with infinite parameters");
                            }

                            edge1 = mkEdge.InternalEdge();
                        }
                    }
                }
                catch (Exception exception) when (CaughtBySignals(exception))
                {
                    // OCCT_DEBUG output left out
                }
            }

            // translate pcurve, if available
            if (pcurve is not null)
            {
                try
                {
                    var trE = new TranslateEdge();
                    Curve2D c2d = trE.MakePCurve(pcurve, surf, localFactors);
                    if (c2d is not null)
                    {
                        if (edge1.IsNull)
                        {
                            var mkEdge = new MakeEdge(c2d, surf, c2d.FirstParameter, c2d.LastParameter);
                            if (mkEdge.IsDone)
                            {
                                if (Kerf.Engine.Foundation.Precision.IsNegativeInfinite(c2d.FirstParameter)
                                    || Kerf.Engine.Foundation.Precision.IsPositiveInfinite(c2d.LastParameter))
                                {
                                    _infiniteSegment = true;
                                    tp.AddWarning(CC, "Segment with infinite parameters");
                                }

                                edge1 = mkEdge.InternalEdge();
                            }
                        }
                        else
                        {
                            var b = new BrepBuilder();
                            Placement l = new Placement();
                            b.UpdateEdge(edge1, c2d, surf, l, 0.0);
                            b.Range(edge1, surf, l, c2d.FirstParameter, c2d.LastParameter);
                            b.SameRange(edge1, false);
                            b.SameParameter(edge1, false);
                        }
                    }
                }
                catch (Exception exception) when (CaughtBySignals(exception))
                {
                    // OCCT_DEBUG output left out
                }
            }

            if (edge1.IsNull)
            {
                tp.AddFail(crv, "Curve can not be translated");
                continue;
            }

            if (!ccs.SameSense())
            {
                edge1.Reverse();
            }

            sbwd.Add(edge1);
        }

        if (sbwd.EdgeCount <= 0)
        {
            tp.AddFail(CC, "Translation gives no result");
            return false;
        }

        // connect wire; all other fixes are left for caller
        double preci = Precision();
        var sfw = new WireFixer();
        sfw.Load(sbwd);
        sfw.SetPrecision(preci);
        sfw.ClosedWireMode() = isClosed;
        sfw.FixReorder(); //: o3 abv 17 Feb 99: r0301_db.stp #57082
        if (sfw.StatusReorder(ExtendStatus.Done))
        {
            tp.AddWarning(CC, "Segments were disordered; fixed");
        }

        sfw.FixConnected(preci);
        if (sfw.StatusConnected(ExtendStatus.Fail))
        {
            tp.AddWarning(CC, "Segments are not connected");
        }

        _wire = sbwd.InternalWire();
        _done = (sbwd.EdgeCount > 0);
        return true;
    }

    public bool Init(StepCompositeCurve cc, TransientProcess tp, StepSurface s, Surface surf)
        => Init(cc, tp, s, surf, new Factors());

    /// <summary>The result of the last translation, frozen; a null wire if it failed.</summary>
    public Wire Value()
    {
        Wire wire = _wire;
        wire.TShape()?.FreezeGraph();
        return wire;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Wire InternalValue() => _wire;

    /// <summary>True if the composite_curve contains a segment with infinite parameters.</summary>
    public bool IsInfiniteSegment => _infiniteSegment;
}
