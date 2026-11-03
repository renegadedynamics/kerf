// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCompositeCurve.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCompositeCurve.lxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateCompositeCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Occt.StepGeom_TransitionCode;

namespace Occt;

/// <summary>
/// Translates a composite_curve into a TopoDS_Wire. If a surface is given, the curve is assumed to lie on it, and
/// where a segment is a curve on surface, its pcurve is taken.
/// </summary>
/// <remarks>
/// A segment may itself be a composite_curve; only one-level protection against cyclic references is implemented.
/// <see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).
/// </remarks>
public sealed class StepToTopoDS_TranslateCompositeCurve : StepToTopoDS_Root
{
    private Wire myWire = new Wire();
    private bool myInfiniteSegment;

    public StepToTopoDS_TranslateCompositeCurve()
    {
        myInfiniteSegment = false;
    }

    /// <summary>Translates a standalone composite_curve.</summary>
    public StepToTopoDS_TranslateCompositeCurve(StepGeom_CompositeCurve CC, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors)
    {
        Init(CC, TP, theLocalFactors);
    }

    public StepToTopoDS_TranslateCompositeCurve(StepGeom_CompositeCurve CC, Transfer_TransientProcess TP)
        : this(CC, TP, new StepData_Factors())
    {
    }

    /// <summary>Translates a composite_curve lying on a surface.</summary>
    public StepToTopoDS_TranslateCompositeCurve(StepGeom_CompositeCurve CC, Transfer_TransientProcess TP, StepGeom_Surface S, Surface Surf,
                                                in StepData_Factors theLocalFactors)
    {
        Init(CC, TP, S, Surf, theLocalFactors);
    }

    public StepToTopoDS_TranslateCompositeCurve(StepGeom_CompositeCurve CC, Transfer_TransientProcess TP, StepGeom_Surface S, Surface Surf)
        : this(CC, TP, S, Surf, new StepData_Factors())
    {
    }

    /// <summary>Translates a standalone composite_curve.</summary>
    public bool Init(StepGeom_CompositeCurve CC, Transfer_TransientProcess TP, in StepData_Factors theLocalFactors)
    {
        StepGeom_Surface S = null;
        Surface Surf = null;
        return Init(CC, TP, S, Surf, theLocalFactors);
    }

    public bool Init(StepGeom_CompositeCurve CC, Transfer_TransientProcess TP) => Init(CC, TP, new StepData_Factors());

    /// <summary>Translates a composite_curve lying on a surface.</summary>
    public bool Init(StepGeom_CompositeCurve CC, Transfer_TransientProcess TP, StepGeom_Surface S, Surface Surf, in StepData_Factors theLocalFactors)
    {
        myWire.Nullify();
        myInfiniteSegment = false;
        if (CC is null)
        {
            return false;
        }

        bool SurfMode = (S is not null && Surf is not null);
        bool isClosed = false;
        StepData_StepModel aStepModel = TP.Model() as StepData_StepModel;

        if (SurfMode)
        {
            int modepcurve = (int)aStepModel.InternalParameters.ReadSurfaceCurveMode;
            if (modepcurve == -3)
            {
                SurfMode = false;
            }
        }

        var sbwd = new ShapeExtend_WireData();
        int nbs = CC.NbSegments();
        for (int i = 1; i <= nbs; i++)
        {
            StepGeom_CompositeCurveSegment ccs = CC.SegmentsValue(i);
            if (ccs is null)
            {
                TP.AddFail(CC, "Null segment");
                return false;
            }

            StepGeom_Curve crv = ccs.ParentCurve();
            if (crv is null)
            {
                TP.AddFail(CC, "Segment has null parent curve");
                return false;
            }

            isClosed = (ccs.Transition() != StepGeom_tcDiscontinuous);

            // if segment is itself a composite_curve, translate recursively
            if (crv is StepGeom_CompositeCurve)
            {
                if (crv == CC)
                { // cyclic reference protection
                    TP.AddFail(ccs, "Cyclic reference; segment dropped");
                    continue;
                }

                StepGeom_CompositeCurve cc = crv as StepGeom_CompositeCurve;
                if (!Init(cc, TP, S, Surf, theLocalFactors) || myWire.IsNull)
                {
                    continue;
                }

                int nb = sbwd.NbEdges() + 1;
                for (var it = new Iterator(myWire); it.More(); it.Next())
                {
                    Edge edge = ShapeCasts.Edge(it.Value());
                    if (ccs.SameSense())
                    {
                        sbwd.Add(edge);
                    }
                    else
                    {
                        edge.Reverse();
                        sbwd.Add(edge, nb > sbwd.NbEdges() ? 0 : nb);
                    }
                }

                myWire.Nullify();
                continue;
            }

            // ordinary segment

            // detect pcurve and 3d curve
            StepGeom_Pcurve pcurve = crv as StepGeom_Pcurve;
            if (pcurve is null)
            {
                StepGeom_SurfaceCurve sc = crv as StepGeom_SurfaceCurve;
                if (sc is not null)
                {
                    crv = sc.Curve3d();
                    if (SurfMode)
                    { // find proper pcurve
                        for (int j = 1; j <= sc.NbAssociatedGeometry(); j++)
                        {
                            StepGeom_PcurveOrSurface PCorS = sc.AssociatedGeometryValue(j);
                            StepGeom_Pcurve pc = PCorS?.Pcurve(); // a null SELECT: C++ has an empty one (decision 1254)
                            if (pc is null || pc.BasisSurface() != S)
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
                if (!SurfMode || pcurve.BasisSurface() != S)
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
                    Curve3D c3d = StepToGeom.MakeCurve(crv, theLocalFactors);
                    if (c3d is not null)
                    {
                        var MkEdge = new BRepBuilderAPI_MakeEdge(c3d, c3d.FirstParameter(), c3d.LastParameter());
                        if (MkEdge.IsDone())
                        {
                            if (Kerf.Engine.Foundation.Precision.IsNegativeInfinite(c3d.FirstParameter())
                                || Kerf.Engine.Foundation.Precision.IsPositiveInfinite(c3d.LastParameter()))
                            {
                                myInfiniteSegment = true;
                                TP.AddWarning(CC, "Segment with infinite parameters");
                            }

                            edge1 = MkEdge.InternalEdge();
                        }
                    }
                }
                catch (Exception anException) when (CaughtBySignals(anException))
                {
                    // OCCT_DEBUG output left out
                }
            }

            // translate pcurve, if available
            if (pcurve is not null)
            {
                try
                {
                    var TrE = new StepToTopoDS_TranslateEdge();
                    Curve2D c2d = TrE.MakePCurve(pcurve, Surf, theLocalFactors);
                    if (c2d is not null)
                    {
                        if (edge1.IsNull)
                        {
                            var MkEdge = new BRepBuilderAPI_MakeEdge(c2d, Surf, c2d.FirstParameter(), c2d.LastParameter());
                            if (MkEdge.IsDone())
                            {
                                if (Kerf.Engine.Foundation.Precision.IsNegativeInfinite(c2d.FirstParameter())
                                    || Kerf.Engine.Foundation.Precision.IsPositiveInfinite(c2d.LastParameter()))
                                {
                                    myInfiniteSegment = true;
                                    TP.AddWarning(CC, "Segment with infinite parameters");
                                }

                                edge1 = MkEdge.InternalEdge();
                            }
                        }
                        else
                        {
                            var B = new BrepBuilder();
                            Placement L = new Placement();
                            B.UpdateEdge(edge1, c2d, Surf, L, 0.0);
                            B.Range(edge1, Surf, L, c2d.FirstParameter(), c2d.LastParameter());
                            B.SameRange(edge1, false);
                            B.SameParameter(edge1, false);
                        }
                    }
                }
                catch (Exception anException) when (CaughtBySignals(anException))
                {
                    // OCCT_DEBUG output left out
                }
            }

            if (edge1.IsNull)
            {
                TP.AddFail(crv, "Curve can not be translated");
                continue;
            }

            if (!ccs.SameSense())
            {
                edge1.Reverse();
            }

            sbwd.Add(edge1);
        }

        if (sbwd.NbEdges() <= 0)
        {
            TP.AddFail(CC, "Translation gives no result");
            return false;
        }

        // connect wire; all other fixes are left for caller
        double preci = Precision();
        var sfw = new ShapeFix_Wire();
        sfw.Load(sbwd);
        sfw.SetPrecision(preci);
        sfw.ClosedWireMode() = isClosed;
        sfw.FixReorder(); //: o3 abv 17 Feb 99: r0301_db.stp #57082
        if (sfw.StatusReorder(ShapeExtend_DONE))
        {
            TP.AddWarning(CC, "Segments were disordered; fixed");
        }

        sfw.FixConnected(preci);
        if (sfw.StatusConnected(ShapeExtend_FAIL))
        {
            TP.AddWarning(CC, "Segments are not connected");
        }

        myWire = sbwd.InternalWire();
        done = (sbwd.NbEdges() > 0);
        return true;
    }

    public bool Init(StepGeom_CompositeCurve CC, Transfer_TransientProcess TP, StepGeom_Surface S, Surface Surf)
        => Init(CC, TP, S, Surf, new StepData_Factors());

    /// <summary>The result of the last translation, frozen; a null wire if it failed.</summary>
    public Wire Value()
    {
        Wire aWire = myWire;
        aWire.TShape()?.FreezeGraph();
        return aWire;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Wire InternalValue() => myWire;

    /// <summary>True if the composite_curve contains a segment with infinite parameters.</summary>
    public bool IsInfiniteSegment() => myInfiniteSegment;
}
