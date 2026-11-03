// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ToolsProvider.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_ToolsProvider.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2017-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

using Kerf.Topology;

namespace Occt;

/// <summary>Adds the tools to the arguments of the General Fuse builder.</summary>
public class BOPAlgo_ToolsProvider : BOPAlgo_Builder
{
    protected NCollection_List<Shape> myTools = new();
    protected NCollection_Map<Shape> myMapTools = new(ShapeHasher.Instance);

    public BOPAlgo_ToolsProvider()
    {
    }

    public override void Clear()
    {
        base.Clear();
        myTools.Clear();
        myMapTools.Clear();
    }

    /// <summary>Adds a tool (once).</summary>
    public virtual void AddTool(in Shape theShape)
    {
        if (myMapTools.Add(theShape))
        {
            myTools.Append(theShape);
        }
    }

    /// <summary>Sets the tools.</summary>
    public virtual void SetTools(NCollection_List<Shape> theShapes)
    {
        myTools.Clear();
        for (NCollection_List<Shape>.Iterator aIt = new(theShapes); aIt.More(); aIt.Next())
        {
            AddTool(aIt.Value());
        }
    }

    /// <summary>The tools.</summary>
    public NCollection_List<Shape> Tools() => myTools;
}
