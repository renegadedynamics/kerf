// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Fixes a wire: the order of its edges, small edges, the connection of adjacent edges, the edges' curves (pcurves,
/// 3d curves, seams, shifted pcurves, same parameter, vertex tolerances), degenerated edges, self-intersections,
/// lacking edges, notched edges, tails and gaps in 3d and 2d. The wire may be on a face (the 2d fixes need one).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). Load a wire (with <see cref="Init(in TopoDS_Wire, in TopoDS_Face, double)"/> or
/// <see cref="Load(in TopoDS_Wire)"/>), set its face and precision, then call <see cref="Perform"/> or the fixes one by
/// one; each sets its own status (<c>Status...</c>), and the advanced fixes (by edge number) share
/// <see cref="LastFixStatus"/>. The modes (<c>...Mode()</c>) are three-position flags: -1 is the default, 0 off, 1 on.
/// </para>
/// <para>
/// The fixes edit the edges of the loaded wire in place, as OCCT's. A loaded wire holding frozen shapes (any wire a
/// caller holds) is copied at the first fix (<see cref="BRepTools_CopyAtEntry"/>, decision 583), so the caller's wire
/// never changes; <see cref="Wire"/> and <see cref="WireAPIMake"/> give back the caller's own edges and vertices
/// wherever the fixes left them unchanged, record in the context the ones changed in place, and freeze what they
/// return. Set the context before the first fix. After a public result, a further fix copies again. The edges of
/// <see cref="WireData"/> are the working copies between the first fix and the result.
/// </para>
/// </remarks>
public partial class ShapeFix_Wire : ShapeFix_Root
{
    protected ShapeFix_Edge      myFixEdge;
    protected ShapeAnalysis_Wire myAnalyzer;
    protected bool               myGeomMode;
    protected bool               myTopoMode;
    protected bool               myClosedMode;
    protected bool               myPreference2d;
    protected bool               myFixGapsByRanges;
    protected int                myFixReversed2dMode;
    protected int                myFixRemovePCurveMode;
    protected int                myFixAddPCurveMode;
    protected int                myFixRemoveCurve3dMode;
    protected int                myFixAddCurve3dMode;
    protected int                myFixSeamMode;
    protected int                myFixShiftedMode;
    protected int                myFixSameParameterMode;
    protected int                myFixVertexToleranceMode;
    protected int                myFixNotchedEdgesMode;
    protected int                myFixSelfIntersectingEdgeMode;
    protected int                myFixIntersectingEdgesMode;
    protected int                myFixNonAdjacentIntersectingEdgesMode;
    protected int                myFixTailMode;
    protected int                myRemoveLoopMode;
    protected int                myFixReorderMode;
    protected int                myFixSmallMode;
    protected int                myFixConnectedMode;
    protected int                myFixEdgeCurvesMode;
    protected int                myFixDegeneratedMode;
    protected int                myFixSelfIntersectionMode;
    protected int                myFixLackingMode;
    protected int                myFixGaps3dMode;
    protected int                myFixGaps2dMode;
    protected int                myLastFixStatus;
    protected int                myStatusReorder;
    protected int                myStatusSmall;
    protected int                myStatusConnected;
    protected int                myStatusEdgeCurves;
    protected int                myStatusDegenerated;
    protected int                myStatusClosed;
    protected int                myStatusSelfIntersection;
    protected int                myStatusLacking;
    protected int                myStatusGaps3d;
    protected int                myStatusGaps2d;
    protected bool               myStatusRemovedSegment;
    protected int                myStatusNotches;
    protected int                myStatusFixTails;
    protected double             myMaxTailAngleSine;
    protected double             myMaxTailWidth;

    // ---- ShapeFix_Wire.lxx ----

    /// <summary>Sets the face of the wire.</summary>
    public void SetFace(in TopoDS_Face face)
    {
        myAnalyzer.SetFace(face);
    }

    /// <summary>Sets the face of the wire with an analyser of its surface.</summary>
    public void SetFace(in TopoDS_Face theFace, ShapeAnalysis_Surface? theSurfaceAnalysis)
    {
        myAnalyzer.SetFace(theFace, theSurfaceAnalysis);
    }

    /// <summary>Sets the surface analyser (and through it the surface) of the wire.</summary>
    public void SetSurface(ShapeAnalysis_Surface? theSurfaceAnalysis)
    {
        myAnalyzer.SetSurface(theSurfaceAnalysis);
    }

    /// <summary>Sets the surface of the wire.</summary>
    public void SetSurface(Geom_Surface? surf)
    {
        myAnalyzer.SetSurface(surf);
    }

    /// <summary>Sets the surface of the wire with a location.</summary>
    public void SetSurface(Geom_Surface? surf, in TopLoc_Location loc)
    {
        myAnalyzer.SetSurface(surf, loc);
    }

    /// <summary>True if a wire is loaded.</summary>
    public bool IsLoaded() => myAnalyzer.IsLoaded();

    /// <summary>True if a wire and a face are loaded.</summary>
    public bool IsReady() => myAnalyzer.IsReady();

    /// <summary>Returns the analyser of the wire.</summary>
    public ShapeAnalysis_Wire Analyzer() => myAnalyzer;

    /// <summary>
    /// Returns the working wire data: the loaded edges, which the fixes edit (between the first fix and a public
    /// result, copies of a frozen wire's edges).
    /// </summary>
    public ShapeExtend_WireData? WireData() => myAnalyzer.WireData();

    /// <summary>Returns the face of the wire.</summary>
    public TopoDS_Face Face() => myAnalyzer.Face();

    /// <summary>The mode allowing the fixes to modify the topology (default false).</summary>
    public ref bool ModifyTopologyMode() => ref myTopoMode;

    /// <summary>The mode allowing the fixes to modify the geometry (default true).</summary>
    public ref bool ModifyGeometryMode() => ref myGeomMode;

    /// <summary>The mode for removing loops (-1 default, 0 keep, 1 remove).</summary>
    public ref int ModifyRemoveLoopMode() => ref myRemoveLoopMode;

    /// <summary>The mode telling that the wire is to be closed (default true).</summary>
    public ref bool ClosedWireMode() => ref myClosedMode;

    /// <summary>The mode preferring pcurves to 3d curves for gaps (default true).</summary>
    public ref bool PreferencePCurveMode() => ref myPreference2d;

    /// <summary>The mode fixing gaps by ranges (default false).</summary>
    public ref bool FixGapsByRangesMode() => ref myFixGapsByRanges;

    public ref int FixReorderMode() => ref myFixReorderMode;

    public ref int FixSmallMode() => ref myFixSmallMode;

    public ref int FixConnectedMode() => ref myFixConnectedMode;

    public ref int FixEdgeCurvesMode() => ref myFixEdgeCurvesMode;

    public ref int FixDegeneratedMode() => ref myFixDegeneratedMode;

    public ref int FixReversed2dMode() => ref myFixReversed2dMode;

    public ref int FixRemovePCurveMode() => ref myFixRemovePCurveMode;

    public ref int FixRemoveCurve3dMode() => ref myFixRemoveCurve3dMode;

    public ref int FixAddPCurveMode() => ref myFixAddPCurveMode;

    public ref int FixAddCurve3dMode() => ref myFixAddCurve3dMode;

    public ref int FixSeamMode() => ref myFixSeamMode;

    public ref int FixShiftedMode() => ref myFixShiftedMode;

    public ref int FixSameParameterMode() => ref myFixSameParameterMode;

    public ref int FixVertexToleranceMode() => ref myFixVertexToleranceMode;

    public ref int FixLackingMode() => ref myFixLackingMode;

    public ref int FixSelfIntersectionMode() => ref myFixSelfIntersectionMode;

    public ref int FixGaps3dMode() => ref myFixGaps3dMode;

    public ref int FixGaps2dMode() => ref myFixGaps2dMode;

    public ref int FixNotchedEdgesMode() => ref myFixNotchedEdgesMode;

    public ref int FixSelfIntersectingEdgeMode() => ref myFixSelfIntersectingEdgeMode;

    public ref int FixIntersectingEdgesMode() => ref myFixIntersectingEdgesMode;

    public ref int FixNonAdjacentIntersectingEdgesMode() => ref myFixNonAdjacentIntersectingEdgesMode;

    public ref int FixTailMode() => ref myFixTailMode;

    public bool StatusReorder(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusReorder, status);

    public bool StatusSmall(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusSmall, status);

    public bool StatusConnected(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusConnected, status);

    public bool StatusEdgeCurves(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusEdgeCurves, status);

    public bool StatusDegenerated(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusDegenerated, status);

    public bool StatusLacking(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusLacking, status);

    public bool StatusSelfIntersection(ShapeExtend_Status status) =>
        ShapeExtend.DecodeStatus(myStatusSelfIntersection, status);

    public bool StatusGaps3d(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusGaps3d, status);

    public bool StatusGaps2d(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusGaps2d, status);

    public bool StatusClosed(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusClosed, status);

    public bool StatusNotches(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusNotches, status);

    public bool StatusFixTails(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusFixTails, status);

    /// <summary>The status of the last advanced fix (by edge number).</summary>
    public bool LastFixStatus(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myLastFixStatus, status);

    /// <summary>Returns the tool fixing the edges.</summary>
    public ShapeFix_Edge FixEdgeTool() => myFixEdge;

    /// <summary>True if a segment was removed by the last fix of self-intersection.</summary>
    public bool StatusRemovedSegment() => myStatusRemovedSegment;

    // ---- ShapeFix_Wire.cxx, part 1 (to line 2129) ----

    //PART1-BEGIN
    public ShapeFix_Wire()
    {
        throw new NotImplementedException();
    }

    public ShapeFix_Wire(in TopoDS_Wire wire, in TopoDS_Face face, double prec)
    {
        throw new NotImplementedException();
    }

    public override void SetPrecision(double prec) => throw new NotImplementedException();

    public void SetMaxTailAngle(double theMaxTailAngle) => throw new NotImplementedException();

    public void SetMaxTailWidth(double theMaxTailWidth) => throw new NotImplementedException();

    public void ClearModes() => throw new NotImplementedException();

    public void ClearStatuses() => throw new NotImplementedException();

    public void Init(in TopoDS_Wire wire, in TopoDS_Face face, double prec) => throw new NotImplementedException();

    public void Init(ShapeAnalysis_Wire saw) => throw new NotImplementedException();

    public void Load(in TopoDS_Wire wire) => throw new NotImplementedException();

    public void Load(ShapeExtend_WireData sbwd) => throw new NotImplementedException();

    public int NbEdges() => throw new NotImplementedException();

    public bool Perform(Message_ProgressRange theProgress = default) => throw new NotImplementedException();

    public bool FixReorder(bool theModeBoth = false) => throw new NotImplementedException();

    public int FixSmall(bool lockvtx, double precsmall = 0.0) => throw new NotImplementedException();

    public bool FixConnected(double prec = -1.0) => throw new NotImplementedException();

    public bool FixEdgeCurves() => throw new NotImplementedException();

    public bool FixDegenerated() => throw new NotImplementedException();

    public bool FixSelfIntersection() => throw new NotImplementedException();

    public bool FixLacking(bool force = false) => throw new NotImplementedException();

    public bool FixClosed(double prec = -1.0) => throw new NotImplementedException();

    public bool FixReorder(ShapeAnalysis_WireOrder wi) => throw new NotImplementedException();

    public bool FixSmall(int num, bool lockvtx, double precsmall) => throw new NotImplementedException();

    public bool FixConnected(int num, double prec, bool theUpdateWire = true) => throw new NotImplementedException();

    public bool FixSeam(int num) => throw new NotImplementedException();

    public bool FixShifted() => throw new NotImplementedException();
    //PART1-END
}
