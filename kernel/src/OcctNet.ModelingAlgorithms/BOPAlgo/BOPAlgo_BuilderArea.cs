// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderArea.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderArea.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The root of the algorithms that build areas from a set of shapes: faces from edges (BOPAlgo_BuilderFace) and solids
/// from faces (BOPAlgo_BuilderSolid).
/// </summary>
public abstract class BOPAlgo_BuilderArea : BOPAlgo_Algo
{
    protected IntTools_Context? myContext;
    protected NCollection_List<TopoDS_Shape> myShapes = new();
    protected NCollection_List<TopoDS_Shape> myLoops = new();
    protected NCollection_List<TopoDS_Shape> myLoopsInternal = new();
    protected NCollection_List<TopoDS_Shape> myAreas = new();
    protected NCollection_IndexedMap<TopoDS_Shape> myShapesToAvoid = new(TopTools_ShapeMapHasher.Instance);
    protected bool myAvoidInternalShapes;

    protected BOPAlgo_BuilderArea()
    {
        myAvoidInternalShapes = false;
    }

    /// <summary>Sets the context for the algorithms.</summary>
    public void SetContext(IntTools_Context? theContext) => myContext = theContext;

    /// <summary>The input shapes.</summary>
    public NCollection_List<TopoDS_Shape> Shapes() => myShapes;

    /// <summary>Sets the shapes for building areas (copies the list).</summary>
    public void SetShapes(NCollection_List<TopoDS_Shape> theLS) => myShapes = new NCollection_List<TopoDS_Shape>(theLS);

    /// <summary>The found loops.</summary>
    public NCollection_List<TopoDS_Shape> Loops() => myLoops;

    /// <summary>The found areas.</summary>
    public NCollection_List<TopoDS_Shape> Areas() => myAreas;

    /// <summary>Defines the preventing of addition of internal parts into the result.</summary>
    public void SetAvoidInternalShapes(bool theAvoidInternal) => myAvoidInternalShapes = theAvoidInternal;

    /// <summary>True if the internal parts are not added into the result.</summary>
    public bool IsAvoidInternalShapes() => myAvoidInternalShapes;

    protected abstract void PerformShapesToAvoid(Message_ProgressRange theRange);

    protected abstract void PerformLoops(Message_ProgressRange theRange);

    protected abstract void PerformAreas(Message_ProgressRange theRange);

    protected abstract void PerformInternalShapes(Message_ProgressRange theRange);
}
