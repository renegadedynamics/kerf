// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_ConnexityBlock.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>A connexity block: a set of connected shapes, the loops made of them, and whether the block is regular.</summary>
/// <remarks>
/// A working object of the wire and shell splitters, which change its lists in place, so a class (decision 716).
/// OCCT copies blocks by value (into lists, into the splitters' solvers): the port writes those copies out with the
/// copy constructor, which copies both lists. The allocator goes.
/// </remarks>
internal sealed class ConnexityBlock
{
    private bool _regular;
    private readonly NCollection_List<Shape> _shapes;
    private readonly NCollection_List<Shape> _loops;

    /// <summary>An empty, regular block.</summary>
    public ConnexityBlock()
    {
        _regular = true;
        _shapes = new NCollection_List<Shape>();
        _loops = new NCollection_List<Shape>();
    }

    /// <summary>A copy of the block (OCCT's implicit copy constructor): the lists are copied.</summary>
    public ConnexityBlock(ConnexityBlock other)
    {
        _regular = other._regular;
        _shapes = new NCollection_List<Shape>(other._shapes);
        _loops = new NCollection_List<Shape>(other._loops);
    }

    /// <summary>The shapes of the block (the block's own list).</summary>
    public NCollection_List<Shape> Shapes() => _shapes;

    /// <summary>The shapes of the block, to change.</summary>
    internal NCollection_List<Shape> ChangeShapes() => _shapes;

    /// <summary>Sets whether the block is regular.</summary>
    internal void SetRegular(bool flag) => _regular = flag;

    /// <summary>True if the block is regular.</summary>
    public bool IsRegular => _regular;

    /// <summary>The loops made of the block's shapes (the block's own list).</summary>
    public NCollection_List<Shape> Loops() => _loops;

    /// <summary>The loops, to change.</summary>
    internal NCollection_List<Shape> ChangeLoops() => _loops;
}
