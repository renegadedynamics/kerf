// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Edge.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// SKELETON: the signatures other classes call; the bodies are being ported.
public class ShapeFix_Edge : Standard_Transient
{
    protected ShapeBuild_ReShape?                  myContext;
    protected int                                  myStatus;
    protected ShapeConstruct_ProjectCurveOnSurface myProjector;

    public ShapeFix_Edge()
    {
        throw new NotImplementedException();
    }

    public ShapeConstruct_ProjectCurveOnSurface Projector() => throw new NotImplementedException();

    internal bool FixRemovePCurve(in TopoDS_Edge edge, in TopoDS_Face face) => throw new NotImplementedException();

    internal bool FixRemovePCurve(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location) =>
        throw new NotImplementedException();

    internal bool FixRemoveCurve3d(in TopoDS_Edge edge) => throw new NotImplementedException();

    internal bool FixAddPCurve(in TopoDS_Edge edge, in TopoDS_Face face, bool isSeam, double prec = 0.0) =>
        throw new NotImplementedException();

    internal bool FixAddPCurve(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location, bool isSeam,
                               double prec = 0.0) => throw new NotImplementedException();

    internal bool FixAddPCurve(in TopoDS_Edge edge, in TopoDS_Face face, bool isSeam, ShapeAnalysis_Surface surfana,
                               double prec = 0.0) => throw new NotImplementedException();

    internal bool FixAddPCurve(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location, bool isSeam,
                               ShapeAnalysis_Surface surfana, double prec = 0.0) => throw new NotImplementedException();

    internal bool FixAddCurve3d(in TopoDS_Edge edge) => throw new NotImplementedException();

    internal bool FixVertexTolerance(in TopoDS_Edge edge, in TopoDS_Face face) => throw new NotImplementedException();

    internal bool FixVertexTolerance(in TopoDS_Edge edge) => throw new NotImplementedException();

    internal bool FixReversed2d(in TopoDS_Edge edge, in TopoDS_Face face) => throw new NotImplementedException();

    internal bool FixReversed2d(in TopoDS_Edge edge, Geom_Surface surface, in TopLoc_Location location) =>
        throw new NotImplementedException();

    internal bool FixSameParameter(in TopoDS_Edge edge, double tolerance = 0.0) => throw new NotImplementedException();

    internal bool FixSameParameter(in TopoDS_Edge edge, in TopoDS_Face face, double tolerance = 0.0) =>
        throw new NotImplementedException();

    public bool Status(ShapeExtend_Status status) => throw new NotImplementedException();

    public void SetContext(ShapeBuild_ReShape? context) => throw new NotImplementedException();

    public ShapeBuild_ReShape? Context() => throw new NotImplementedException();
}
