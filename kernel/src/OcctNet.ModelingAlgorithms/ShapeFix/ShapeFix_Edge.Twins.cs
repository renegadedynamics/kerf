// OCCT.NET: the functional twins of ShapeFix_Edge's in-place fixes (docs/phase5-6-plan.md, section 2.7; decision
// 583). Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Occt;

public partial class ShapeFix_Edge
{
    // The twin of an in-place fix: the fix on a copy of the edge if it holds a frozen shape (the context's requests for
    // the edge's sub-shapes moved onto the copy and back), the copy's unchanged parts the caller's own again, frozen.
    // A fix that changes nothing gives back the edge itself.
    private Edge fixedOnCopy(in Edge edge, Func<Edge, bool> theFix, out bool theIsDone)
    {
        if (edge.IsNull)
        {
            theIsDone = theFix(edge);
            return edge;
        }
        var anEntry = new CopyAtEntry(edge, myContext);
        theIsDone = theFix(ShapeCasts.Edge(anEntry.Shape()));
        Shape aResult = anEntry.Finish(anEntry.Shape());
        aResult.TShape()!.FreezeGraph();
        return ShapeCasts.Edge(aResult);
    }

    /// <summary>
    /// The edge without its pcurve on the face if the pcurve's ends don't lie on the vertices (OCCT's
    /// <c>FixRemovePCurve</c>, whose result is <paramref name="theIsDone"/>), frozen.
    /// </summary>
    public Edge RemovePCurveFixed(in Edge edge, in Face face, out bool theIsDone)
    {
        Face aFace = face;
        return fixedOnCopy(edge, e => FixRemovePCurve(e, aFace), out theIsDone);
    }

    /// <summary>The edge without its pcurve on the surface if inconsistent (<c>FixRemovePCurve</c>), frozen.</summary>
    public Edge RemovePCurveFixed(in Edge edge, Surface surface, in Placement location,
                                         out bool theIsDone)
    {
        Placement aLoc = location;
        return fixedOnCopy(edge, e => FixRemovePCurve(e, surface, aLoc), out theIsDone);
    }

    /// <summary>The edge without its 3D curve if inconsistent with the vertices (<c>FixRemoveCurve3d</c>), frozen.</summary>
    public Edge RemoveCurve3dFixed(in Edge edge, out bool theIsDone) =>
        fixedOnCopy(edge, e => FixRemoveCurve3d(e), out theIsDone);

    /// <summary>
    /// The edge with its missing pcurve(s) on the face added by projection (<c>FixAddPCurve</c>), frozen; the status
    /// is the object's.
    /// </summary>
    public Edge AddPCurveFixed(in Edge edge, in Face face, bool isSeam, out bool theIsDone,
                                      double prec = 0.0)
    {
        Face aFace = face;
        return fixedOnCopy(edge, e => FixAddPCurve(e, aFace, isSeam, prec), out theIsDone);
    }

    /// <summary>The edge with its missing pcurve(s) on the surface added (<c>FixAddPCurve</c>), frozen.</summary>
    public Edge AddPCurveFixed(in Edge edge, Surface surface, in Placement location, bool isSeam,
                                      out bool theIsDone, double prec = 0.0)
    {
        Placement aLoc = location;
        return fixedOnCopy(edge, e => FixAddPCurve(e, surface, aLoc, isSeam, prec), out theIsDone);
    }

    /// <summary>
    /// The edge with its missing pcurve(s) on the face added, with an analyser of the surface (<c>FixAddPCurve</c>),
    /// frozen.
    /// </summary>
    public Edge AddPCurveFixed(in Edge edge, in Face face, bool isSeam, ShapeAnalysis_Surface surfana,
                                      out bool theIsDone, double prec = 0.0)
    {
        Face aFace = face;
        return fixedOnCopy(edge, e => FixAddPCurve(e, aFace, isSeam, surfana, prec), out theIsDone);
    }

    /// <summary>
    /// The edge with its missing pcurve(s) on the surface added, with an analyser of the surface
    /// (<c>FixAddPCurve</c>), frozen.
    /// </summary>
    public Edge AddPCurveFixed(in Edge edge, Surface surface, in Placement location, bool isSeam,
                                      ShapeAnalysis_Surface surfana, out bool theIsDone, double prec = 0.0)
    {
        Placement aLoc = location;
        return fixedOnCopy(edge, e => FixAddPCurve(e, surface, aLoc, isSeam, surfana, prec), out theIsDone);
    }

    /// <summary>The edge with its missing 3D curve built (<c>FixAddCurve3d</c>), frozen.</summary>
    public Edge AddCurve3dFixed(in Edge edge, out bool theIsDone) =>
        fixedOnCopy(edge, e => FixAddCurve3d(e), out theIsDone);

    /// <summary>
    /// The edge with its vertices' tolerances raised to cover its curves on the face (<c>FixVertexTolerance</c>),
    /// frozen. With a context, OCCT's fix writes only into new vertices recorded in the context: the edge is then
    /// returned unchanged and the context answers for its vertices.
    /// </summary>
    public Edge VertexToleranceFixed(in Edge edge, in Face face, out bool theIsDone)
    {
        Face aFace = face;
        return fixedOnCopy(edge, e => FixVertexTolerance(e, aFace), out theIsDone);
    }

    /// <summary>The edge with its vertices' tolerances raised (<c>FixVertexTolerance</c>), frozen; see the face form.</summary>
    public Edge VertexToleranceFixed(in Edge edge, out bool theIsDone) =>
        fixedOnCopy(edge, e => FixVertexTolerance(e), out theIsDone);

    /// <summary>The edge with its pcurve on the face reversed if against the 3D curve (<c>FixReversed2d</c>), frozen.</summary>
    public Edge Reversed2dFixed(in Edge edge, in Face face, out bool theIsDone)
    {
        Face aFace = face;
        return fixedOnCopy(edge, e => FixReversed2d(e, aFace), out theIsDone);
    }

    /// <summary>The edge with its pcurve on the surface reversed if needed (<c>FixReversed2d</c>), frozen.</summary>
    public Edge Reversed2dFixed(in Edge edge, Surface surface, in Placement location,
                                       out bool theIsDone)
    {
        Placement aLoc = location;
        return fixedOnCopy(edge, e => FixReversed2d(e, surface, aLoc), out theIsDone);
    }

    /// <summary>The edge made SameParameter (<c>FixSameParameter</c>), frozen.</summary>
    public Edge SameParameterFixed(in Edge edge, out bool theIsDone, double tolerance = 0.0) =>
        fixedOnCopy(edge, e => FixSameParameter(e, tolerance), out theIsDone);

    /// <summary>The edge made SameParameter, checking its pcurve on the face (<c>FixSameParameter</c>), frozen.</summary>
    public Edge SameParameterFixed(in Edge edge, in Face face, out bool theIsDone,
                                          double tolerance = 0.0)
    {
        Face aFace = face;
        return fixedOnCopy(edge, e => FixSameParameter(e, aFace, tolerance), out theIsDone);
    }
}
