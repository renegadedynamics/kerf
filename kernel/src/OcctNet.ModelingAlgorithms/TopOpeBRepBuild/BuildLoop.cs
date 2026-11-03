// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Loop.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Loop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// A loop: an existing shape (a wire, a shell) or a block of connex elements (edges, faces) given by a
/// <see cref="BlockIterator"/>.
/// </summary>
/// <remarks><c>Dump</c> is left out (decision 493).</remarks>
public class BuildLoop : Transient
{
    protected bool _isShape;
    protected Shape _shape = new();
    protected BlockIterator _blockIterator;

    /// <summary>The loop of the existing shape <paramref name="s"/>.</summary>
    public BuildLoop(in Shape s)
    {
        _isShape = true;
        _shape = s;
        _blockIterator = new BlockIterator(0, 0);
    }

    /// <summary>The loop of the block of elements <paramref name="bi"/>.</summary>
    public BuildLoop(in BlockIterator bi)
    {
        _isShape = false;
        _blockIterator = bi;
    }

    /// <summary>Whether the loop is an existing shape.</summary>
    public virtual bool IsShape => _isShape;

    /// <summary>The shape (null for a block).</summary>
    public virtual Shape Shape() => _shape;

    /// <summary>The block's iterator.</summary>
    public BlockIterator BlockIterator() => _blockIterator;
}
