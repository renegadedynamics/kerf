// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireEdgeSet.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireEdgeSet.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>The edges given to BOPAlgo_WireSplitter (the start elements) and the wires it makes (the shapes).</summary>
/// <remarks>A working object: OCCT passes it by pointer to the wire splitter, which fills it.</remarks>
public sealed class BOPAlgo_WireEdgeSet
{
    private Face myFace;
    private readonly NCollection_List<Shape> myStartShapes = new();
    private readonly NCollection_List<Shape> myShapes = new();

    public BOPAlgo_WireEdgeSet()
    {
        myFace = new Face();
    }

    public void Clear()
    {
        myStartShapes.Clear();
        myShapes.Clear();
    }

    public void SetFace(in Face aF) => myFace = aF;

    public Face Face() => myFace;

    public void AddStartElement(in Shape aE) => myStartShapes.Append(aE);

    public NCollection_List<Shape> StartElements() => myStartShapes;

    public void AddShape(in Shape aW) => myShapes.Append(aW);

    public NCollection_List<Shape> Shapes() => myShapes;
}
