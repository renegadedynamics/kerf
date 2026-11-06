// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireEdgeSet.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireEdgeSet.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Kerf.Engine.Booleans;

/// <summary>The edges given to BOPAlgo_WireSplitter (the start elements) and the wires it makes (the shapes).</summary>
/// <remarks>A working object: OCCT passes it by pointer to the wire splitter, which fills it.</remarks>
internal sealed class WireEdgeSet
{
    private Face _face;
    private readonly NCollection_List<Shape> _startShapes = new();
    private readonly NCollection_List<Shape> _shapes = new();

    public WireEdgeSet()
    {
        _face = new Face();
    }

    public void Clear()
    {
        _startShapes.Clear();
        _shapes.Clear();
    }

    public void SetFace(in Face aF) => _face = aF;

    public Face Face() => _face;

    public void AddStartElement(in Shape aE) => _startShapes.Append(aE);

    public NCollection_List<Shape> StartElements() => _startShapes;

    public void AddShape(in Shape aW) => _shapes.Append(aW);

    public NCollection_List<Shape> Shapes() => _shapes;
}
