// OCCT.NET: the functional twins of ShapeBuild_Edge's in-place methods (docs/phase3-design.md, section 4.6), on the
// working overlay since Kerf unit 10H4. Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Healing.Build;

public sealed partial class BuildEdgeBuilder
{
    // The twin of an in-place method (Kerf unit 10H4): the operation on the caller's edge in a working overlay, which
    // takes its writes; the result holds new TShapes for what changed and the edge's own for the rest (the edge itself
    // if nothing changed), frozen. The history is the overlay's, direct.
    private static Edge InOverlay(in Edge edge, Action<Edge> operation, out History history)
    {
        if (edge.IsNull)
        {
            history = new History();
            return edge;
        }

        Edge work = ShapeCasts.Edge(WorkingOverlay.WithUnfrozenCopied(edge, out Dictionary<TShape, TShape>? copies));
        using var overlay = WorkingOverlay.Begin(work);
        operation(work);
        Shape result = WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies);
        history = overlay.History();
        result.TShape()!.FreezeGraph();
        return ShapeCasts.Edge(result);
    }

    /// <summary>
    /// The functional twin of <see cref="CopyRanges"/>: <paramref name="toedge"/> with the ranges of its curves set
    /// from <paramref name="fromedge"/>'s.
    /// </summary>
    public Edge RangesCopied(in Edge toedge, in Edge fromedge, double alpha = 0, double beta = 1)
        => RangesCopied(toedge, fromedge, out _, alpha, beta);

    /// <summary>The functional twin of CopyRanges, with the history from the edge to the result.</summary>
    public Edge RangesCopied(in Edge toedge, in Edge fromedge, out History history, double alpha = 0,
                                    double beta = 1)
    {
        Edge aFrom = fromedge;
        return InOverlay(toedge, anEdge => CopyRanges(anEdge, aFrom, alpha, beta), out history);
    }

    /// <summary>The functional twin of <see cref="SetRange3d"/>: the edge with the range of its 3D curve set.</summary>
    public Edge WithRange3d(in Edge edge, double first, double last)
        => InOverlay(edge, anEdge => SetRange3d(anEdge, first, last), out _);

    /// <summary>
    /// The functional twin of <see cref="CopyPCurves"/>: <paramref name="toedge"/> holding copies of
    /// <paramref name="fromedge"/>'s pcurves.
    /// </summary>
    public Edge PCurvesCopied(in Edge toedge, in Edge fromedge)
    {
        Edge aFrom = fromedge;
        return InOverlay(toedge, anEdge => CopyPCurves(anEdge, aFrom), out _);
    }

    /// <summary>The functional twin of <see cref="RemovePCurve(in Edge, in Face)"/>.</summary>
    public Edge PCurveRemoved(in Edge edge, in Face face)
    {
        Face aFace = face;
        return InOverlay(edge, anEdge => RemovePCurve(anEdge, aFace), out _);
    }

    /// <summary>The functional twin of <see cref="RemovePCurve(in Edge, Surface)"/>.</summary>
    public Edge PCurveRemoved(in Edge edge, Surface? surf)
        => InOverlay(edge, anEdge => RemovePCurve(anEdge, surf), out _);

    /// <summary>The functional twin of <see cref="RemovePCurve(in Edge, Surface, in Placement)"/>.</summary>
    public Edge PCurveRemoved(in Edge edge, Surface? surf, in Placement loc)
    {
        Placement aLoc = loc;
        return InOverlay(edge, anEdge => RemovePCurve(anEdge, surf, aLoc), out _);
    }

    /// <summary>
    /// The functional twin of <see cref="ReplacePCurve"/>: the edge with its pcurve on the face replaced by
    /// <paramref name="pcurve"/> (which is frozen when the edge takes it).
    /// </summary>
    public Edge PCurveReplaced(in Edge edge, Curve2D? pcurve, in Face face)
    {
        Face aFace = face;
        return InOverlay(edge, anEdge => ReplacePCurve(anEdge, pcurve, aFace), out _);
    }

    /// <summary>
    /// The functional twin of <see cref="ReassignPCurve"/>: the edge with its pcurve moved from face
    /// <paramref name="old"/> to face <paramref name="sub"/>; <paramref name="theIsDone"/> is ReassignPCurve's result.
    /// </summary>
    public Edge PCurveReassigned(in Edge edge, in Face old, in Face sub, out bool theIsDone)
    {
        Face anOld = old, aSub = sub;
        bool isDone = false;
        Edge result = InOverlay(edge, anEdge => isDone = ReassignPCurve(anEdge, anOld, aSub), out _);
        theIsDone = isDone;
        return result;
    }

    /// <summary>The functional twin of <see cref="RemoveCurve3d"/>: the edge without its 3D curve.</summary>
    public Edge Curve3dRemoved(in Edge edge) => InOverlay(edge, anEdge => RemoveCurve3d(anEdge), out _);

    /// <summary>
    /// The functional twin of <see cref="BuildCurve3d"/>: the edge with a 3D curve computed from its pcurve;
    /// <paramref name="theIsDone"/> is BuildCurve3d's result.
    /// </summary>
    public Edge Curve3dBuilt(in Edge edge, out bool theIsDone)
    {
        bool isDone = false;
        Edge result = InOverlay(edge, anEdge => isDone = BuildCurve3d(anEdge), out _);
        theIsDone = isDone;
        return result;
    }
}
