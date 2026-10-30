// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_ConnexityBlock.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A connexity block: a set of connected shapes, the loops made of them, and whether the block is regular.</summary>
/// <remarks>
/// A working object of the wire and shell splitters, which change its lists in place, so a class (decision 6403).
/// OCCT copies blocks by value (into lists, into the splitters' solvers): the port writes those copies out with the
/// copy constructor, which copies both lists. The allocator goes.
/// </remarks>
public sealed class BOPTools_ConnexityBlock
{
    private bool myRegular;
    private readonly NCollection_List<TopoDS_Shape> myShapes;
    private readonly NCollection_List<TopoDS_Shape> myLoops;

    /// <summary>An empty, regular block.</summary>
    public BOPTools_ConnexityBlock()
    {
        myRegular = true;
        myShapes = new NCollection_List<TopoDS_Shape>();
        myLoops = new NCollection_List<TopoDS_Shape>();
    }

    /// <summary>A copy of the block (OCCT's implicit copy constructor): the lists are copied.</summary>
    public BOPTools_ConnexityBlock(BOPTools_ConnexityBlock theOther)
    {
        myRegular = theOther.myRegular;
        myShapes = new NCollection_List<TopoDS_Shape>(theOther.myShapes);
        myLoops = new NCollection_List<TopoDS_Shape>(theOther.myLoops);
    }

    /// <summary>The shapes of the block (the block's own list).</summary>
    public NCollection_List<TopoDS_Shape> Shapes() => myShapes;

    /// <summary>The shapes of the block, to change.</summary>
    internal NCollection_List<TopoDS_Shape> ChangeShapes() => myShapes;

    /// <summary>Sets whether the block is regular.</summary>
    internal void SetRegular(bool theFlag) => myRegular = theFlag;

    /// <summary>True if the block is regular.</summary>
    public bool IsRegular() => myRegular;

    /// <summary>The loops made of the block's shapes (the block's own list).</summary>
    public NCollection_List<TopoDS_Shape> Loops() => myLoops;

    /// <summary>The loops, to change.</summary>
    internal NCollection_List<TopoDS_Shape> ChangeLoops() => myLoops;
}
