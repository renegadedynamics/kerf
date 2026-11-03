// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ToolsProvider.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ToolsProvider.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans;

/// <summary>Adds the tools to the arguments of the General Fuse builder.</summary>
public class ToolsProvider : BooleanBuilder
{
    protected NCollection_List<Shape> _tools = new();
    protected NCollection_Map<Shape> _mapTools = new(ShapeHasher.Instance);

    public ToolsProvider()
    {
    }

    public override void Clear()
    {
        base.Clear();
        _tools.Clear();
        _mapTools.Clear();
    }

    /// <summary>Adds a tool (once).</summary>
    public virtual void AddTool(in Shape shape)
    {
        if (_mapTools.Add(shape))
        {
            _tools.Append(shape);
        }
    }

    /// <summary>Sets the tools.</summary>
    public virtual void SetTools(NCollection_List<Shape> shapes)
    {
        _tools.Clear();
        for (NCollection_List<Shape>.Iterator it = new(shapes); it.More(); it.Next())
        {
            AddTool(it.Value());
        }
    }

    /// <summary>The tools.</summary>
    public NCollection_List<Shape> Tools() => _tools;
}
