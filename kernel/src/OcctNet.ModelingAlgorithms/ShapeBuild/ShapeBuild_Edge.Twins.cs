// OCCT.NET: the functional twins of ShapeBuild_Edge's in-place methods (docs/phase3-design.md, section 4.6). Not a port
// of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

public sealed partial class ShapeBuild_Edge
{
    // The twin of an in-place method: the operation on an exact copy for write of the edge, whose sub-shapes it left
    // unchanged are the input's own again (the input itself if nothing changed), frozen.
    private static TopoDS_Edge onCopy(in TopoDS_Edge theEdge, Action<TopoDS_Edge> theOperation, out BRepTools_History theHistory)
    {
        if (theEdge.IsNull())
        {
            theHistory = new BRepTools_History();
            return theEdge;
        }

        TopoDS_Edge aCopy = TopoDS.Edge(BRepTools.ExactCopyForWrite(theEdge, out _));
        theOperation(aCopy);
        TopoDS_Shape aResult = BRepTools.KeepUnchanged(theEdge, aCopy, out theHistory);
        aResult.TShape()!.FreezeGraph();
        return TopoDS.Edge(aResult);
    }

    /// <summary>
    /// The functional twin of <see cref="CopyRanges"/>: <paramref name="toedge"/> with the ranges of its curves set
    /// from <paramref name="fromedge"/>'s.
    /// </summary>
    public TopoDS_Edge RangesCopied(in TopoDS_Edge toedge, in TopoDS_Edge fromedge, double alpha = 0, double beta = 1)
        => RangesCopied(toedge, fromedge, out _, alpha, beta);

    /// <summary>The functional twin of CopyRanges, with the history from the edge to the result.</summary>
    public TopoDS_Edge RangesCopied(in TopoDS_Edge toedge, in TopoDS_Edge fromedge, out BRepTools_History theHistory, double alpha = 0,
                                    double beta = 1)
    {
        TopoDS_Edge aFrom = fromedge;
        return onCopy(toedge, aCopy => CopyRanges(aCopy, aFrom, alpha, beta), out theHistory);
    }

    /// <summary>The functional twin of <see cref="SetRange3d"/>: the edge with the range of its 3D curve set.</summary>
    public TopoDS_Edge WithRange3d(in TopoDS_Edge edge, double first, double last)
        => onCopy(edge, aCopy => SetRange3d(aCopy, first, last), out _);

    /// <summary>
    /// The functional twin of <see cref="CopyPCurves"/>: <paramref name="toedge"/> holding copies of
    /// <paramref name="fromedge"/>'s pcurves.
    /// </summary>
    public TopoDS_Edge PCurvesCopied(in TopoDS_Edge toedge, in TopoDS_Edge fromedge)
    {
        TopoDS_Edge aFrom = fromedge;
        return onCopy(toedge, aCopy => CopyPCurves(aCopy, aFrom), out _);
    }

    /// <summary>The functional twin of <see cref="RemovePCurve(in TopoDS_Edge, in TopoDS_Face)"/>.</summary>
    public TopoDS_Edge PCurveRemoved(in TopoDS_Edge edge, in TopoDS_Face face)
    {
        TopoDS_Face aFace = face;
        return onCopy(edge, aCopy => RemovePCurve(aCopy, aFace), out _);
    }

    /// <summary>The functional twin of <see cref="RemovePCurve(in TopoDS_Edge, Geom_Surface)"/>.</summary>
    public TopoDS_Edge PCurveRemoved(in TopoDS_Edge edge, Geom_Surface? surf)
        => onCopy(edge, aCopy => RemovePCurve(aCopy, surf), out _);

    /// <summary>The functional twin of <see cref="RemovePCurve(in TopoDS_Edge, Geom_Surface, in Placement)"/>.</summary>
    public TopoDS_Edge PCurveRemoved(in TopoDS_Edge edge, Geom_Surface? surf, in Placement loc)
    {
        Placement aLoc = loc;
        return onCopy(edge, aCopy => RemovePCurve(aCopy, surf, aLoc), out _);
    }

    /// <summary>
    /// The functional twin of <see cref="ReplacePCurve"/>: the edge with its pcurve on the face replaced by
    /// <paramref name="pcurve"/> (which is frozen when the edge takes it).
    /// </summary>
    public TopoDS_Edge PCurveReplaced(in TopoDS_Edge edge, Geom2d_Curve? pcurve, in TopoDS_Face face)
    {
        TopoDS_Face aFace = face;
        return onCopy(edge, aCopy => ReplacePCurve(aCopy, pcurve, aFace), out _);
    }

    /// <summary>
    /// The functional twin of <see cref="ReassignPCurve"/>: the edge with its pcurve moved from face
    /// <paramref name="old"/> to face <paramref name="sub"/>; <paramref name="theIsDone"/> is ReassignPCurve's result.
    /// </summary>
    public TopoDS_Edge PCurveReassigned(in TopoDS_Edge edge, in TopoDS_Face old, in TopoDS_Face sub, out bool theIsDone)
    {
        TopoDS_Face anOld = old, aSub = sub;
        bool isDone = false;
        TopoDS_Edge aResult = onCopy(edge, aCopy => isDone = ReassignPCurve(aCopy, anOld, aSub), out _);
        theIsDone = isDone;
        return aResult;
    }

    /// <summary>The functional twin of <see cref="RemoveCurve3d"/>: the edge without its 3D curve.</summary>
    public TopoDS_Edge Curve3dRemoved(in TopoDS_Edge edge) => onCopy(edge, aCopy => RemoveCurve3d(aCopy), out _);

    /// <summary>
    /// The functional twin of <see cref="BuildCurve3d"/>: the edge with a 3D curve computed from its pcurve;
    /// <paramref name="theIsDone"/> is BuildCurve3d's result.
    /// </summary>
    public TopoDS_Edge Curve3dBuilt(in TopoDS_Edge edge, out bool theIsDone)
    {
        bool isDone = false;
        TopoDS_Edge aResult = onCopy(edge, aCopy => isDone = BuildCurve3d(aCopy), out _);
        theIsDone = isDone;
        return aResult;
    }
}
