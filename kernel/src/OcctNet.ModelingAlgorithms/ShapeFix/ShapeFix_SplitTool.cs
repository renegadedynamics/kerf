// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitTool.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_SplitTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// SKELETON: the signatures other classes call; the bodies are being ported.
public sealed class ShapeFix_SplitTool
{
    public ShapeFix_SplitTool()
    {
    }

    internal bool SplitEdge(in TopoDS_Edge edge, double param, in TopoDS_Vertex vert, in TopoDS_Face face,
                            ref TopoDS_Edge newE1, ref TopoDS_Edge newE2, double tol3d, double tol2d) =>
        throw new NotImplementedException();

    internal bool SplitEdge(in TopoDS_Edge edge, double param1, double param2, in TopoDS_Vertex vert, in TopoDS_Face face,
                            ref TopoDS_Edge newE1, ref TopoDS_Edge newE2, double tol3d, double tol2d) =>
        throw new NotImplementedException();

    internal bool CutEdge(in TopoDS_Edge edge, double pend, double cut, in TopoDS_Face face, ref bool iscutline) =>
        throw new NotImplementedException();

    internal bool SplitEdge(in TopoDS_Edge edge, double fp, in TopoDS_Vertex V1, double lp, in TopoDS_Vertex V2,
                            in TopoDS_Face face, NCollection_Sequence<TopoDS_Shape> SeqE, ref int aNum,
                            ShapeBuild_ReShape context, double tol3d, double tol2d) => throw new NotImplementedException();
}
