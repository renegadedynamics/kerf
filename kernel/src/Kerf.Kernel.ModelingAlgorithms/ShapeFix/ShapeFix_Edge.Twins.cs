// Kerf: the functional twins of ShapeFix_Edge's in-place fixes (docs/phase5-6-plan.md, section 2.7; decision
// 583), on the working overlay since Kerf unit 10H1. Not a port of an OCCT file.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Healing.Fixes;

internal partial class EdgeFixer
{
    // The twin of an in-place fix (Kerf unit 10H1, the working overlay's pilot): the fix runs on the caller's edge in a
    // working overlay, which takes its writes into frozen TShapes; the result holds new TShapes for what the fix changed
    // and the caller's own for the rest, frozen. The context's requests stay keyed by the caller's sub-shapes, and a
    // sub-shape changed without a request is recorded in it. A fix that changes nothing gives back the edge itself.
    private Edge FixedInOverlay(in Edge edge, Func<Edge, bool> fix, out bool isDone)
    {
        if (edge.IsNull)
        {
            isDone = fix(edge);
            return edge;
        }
        // an unfrozen edge (ported code's) is copied, so that the fix doesn't write into the caller's (unit 10H4)
        Edge work = ShapeCasts.Edge(WorkingOverlay.WithUnfrozenCopied(edge, out Dictionary<TShape, TShape>? copies));
        using var overlay = WorkingOverlay.Begin(work, _context);
        isDone = fix(work);
        Shape result = WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies);
        result.TShape()!.FreezeGraph();
        return ShapeCasts.Edge(result);
    }

    /// <summary>
    /// The edge without its pcurve on the face if the pcurve's ends don't lie on the vertices (OCCT's
    /// <c>FixRemovePCurve</c>, whose result is <paramref name="isDone"/>), frozen.
    /// </summary>
    public Edge RemovePCurveFixed(in Edge edge, in Face face, out bool isDone)
    {
        Face aFace = face;
        return FixedInOverlay(edge, e => FixRemovePCurve(e, aFace), out isDone);
    }

    /// <summary>The edge without its pcurve on the surface if inconsistent (<c>FixRemovePCurve</c>), frozen.</summary>
    public Edge RemovePCurveFixed(in Edge edge, Surface surface, in Placement location,
                                         out bool isDone)
    {
        Placement loc = location;
        return FixedInOverlay(edge, e => FixRemovePCurve(e, surface, loc), out isDone);
    }

    /// <summary>The edge without its 3D curve if inconsistent with the vertices (<c>FixRemoveCurve3d</c>), frozen.</summary>
    public Edge RemoveCurve3dFixed(in Edge edge, out bool isDone) =>
        FixedInOverlay(edge, e => FixRemoveCurve3d(e), out isDone);

    /// <summary>
    /// The edge with its missing pcurve(s) on the face added by projection (<c>FixAddPCurve</c>), frozen; the status
    /// is the object's.
    /// </summary>
    public Edge AddPCurveFixed(in Edge edge, in Face face, bool isSeam, out bool isDone,
                                      double prec = 0.0)
    {
        Face aFace = face;
        return FixedInOverlay(edge, e => FixAddPCurve(e, aFace, isSeam, prec), out isDone);
    }

    /// <summary>The edge with its missing pcurve(s) on the surface added (<c>FixAddPCurve</c>), frozen.</summary>
    public Edge AddPCurveFixed(in Edge edge, Surface surface, in Placement location, bool isSeam,
                                      out bool isDone, double prec = 0.0)
    {
        Placement loc = location;
        return FixedInOverlay(edge, e => FixAddPCurve(e, surface, loc, isSeam, prec), out isDone);
    }

    /// <summary>
    /// The edge with its missing pcurve(s) on the face added, with an analyser of the surface (<c>FixAddPCurve</c>),
    /// frozen.
    /// </summary>
    public Edge AddPCurveFixed(in Edge edge, in Face face, bool isSeam, SurfaceAnalysis surfana,
                                      out bool isDone, double prec = 0.0)
    {
        Face aFace = face;
        return FixedInOverlay(edge, e => FixAddPCurve(e, aFace, isSeam, surfana, prec), out isDone);
    }

    /// <summary>
    /// The edge with its missing pcurve(s) on the surface added, with an analyser of the surface
    /// (<c>FixAddPCurve</c>), frozen.
    /// </summary>
    public Edge AddPCurveFixed(in Edge edge, Surface surface, in Placement location, bool isSeam,
                                      SurfaceAnalysis surfana, out bool isDone, double prec = 0.0)
    {
        Placement loc = location;
        return FixedInOverlay(edge, e => FixAddPCurve(e, surface, loc, isSeam, surfana, prec), out isDone);
    }

    /// <summary>The edge with its missing 3D curve built (<c>FixAddCurve3d</c>), frozen.</summary>
    public Edge AddCurve3dFixed(in Edge edge, out bool isDone) =>
        FixedInOverlay(edge, e => FixAddCurve3d(e), out isDone);

    /// <summary>
    /// The edge with its vertices' tolerances raised to cover its curves on the face (<c>FixVertexTolerance</c>),
    /// frozen. With a context, OCCT's fix writes only into new vertices recorded in the context: the edge is then
    /// returned unchanged and the context answers for its vertices.
    /// </summary>
    public Edge VertexToleranceFixed(in Edge edge, in Face face, out bool isDone)
    {
        Face aFace = face;
        return FixedInOverlay(edge, e => FixVertexTolerance(e, aFace), out isDone);
    }

    /// <summary>The edge with its vertices' tolerances raised (<c>FixVertexTolerance</c>), frozen; see the face form.</summary>
    public Edge VertexToleranceFixed(in Edge edge, out bool isDone) =>
        FixedInOverlay(edge, e => FixVertexTolerance(e), out isDone);

    /// <summary>The edge with its pcurve on the face reversed if against the 3D curve (<c>FixReversed2d</c>), frozen.</summary>
    public Edge Reversed2dFixed(in Edge edge, in Face face, out bool isDone)
    {
        Face aFace = face;
        return FixedInOverlay(edge, e => FixReversed2d(e, aFace), out isDone);
    }

    /// <summary>The edge with its pcurve on the surface reversed if needed (<c>FixReversed2d</c>), frozen.</summary>
    public Edge Reversed2dFixed(in Edge edge, Surface surface, in Placement location,
                                       out bool isDone)
    {
        Placement loc = location;
        return FixedInOverlay(edge, e => FixReversed2d(e, surface, loc), out isDone);
    }

    /// <summary>The edge made SameParameter (<c>FixSameParameter</c>), frozen.</summary>
    public Edge SameParameterFixed(in Edge edge, out bool isDone, double tolerance = 0.0) =>
        FixedInOverlay(edge, e => FixSameParameter(e, tolerance), out isDone);

    /// <summary>The edge made SameParameter, checking its pcurve on the face (<c>FixSameParameter</c>), frozen.</summary>
    public Edge SameParameterFixed(in Edge edge, in Face face, out bool isDone,
                                          double tolerance = 0.0)
    {
        Face aFace = face;
        return FixedInOverlay(edge, e => FixSameParameter(e, aFace, tolerance), out isDone);
    }
}
