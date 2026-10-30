// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_IntersectionTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// SKELETON: the signatures other classes call; the bodies are being ported.
public sealed class ShapeFix_IntersectionTool
{
    public ShapeFix_IntersectionTool(ShapeBuild_ReShape? context, double preci, double maxtol = 1.0)
    {
        throw new NotImplementedException();
    }

    public ShapeBuild_ReShape? Context() => throw new NotImplementedException();

    internal bool SplitEdge(in TopoDS_Edge edge, double param, in TopoDS_Vertex vert, in TopoDS_Face face,
                            ref TopoDS_Edge newE1, ref TopoDS_Edge newE2, double preci) => throw new NotImplementedException();

    internal bool CutEdge(in TopoDS_Edge edge, double pend, double cut, in TopoDS_Face face, ref bool iscutline) =>
        throw new NotImplementedException();

    internal bool FixSelfIntersectWire(ShapeExtend_WireData sewd, in TopoDS_Face face, ref int NbSplit, ref int NbCut,
                                       ref int NbRemoved) => throw new NotImplementedException();

    internal bool FixIntersectingWires(ref TopoDS_Face face) => throw new NotImplementedException();
}
