// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Loop.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_Loop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A loop: an existing shape (a wire, a shell) or a block of connex elements (edges, faces) given by a
/// <see cref="TopOpeBRepBuild_BlockIterator"/>.
/// </summary>
/// <remarks><c>Dump</c> is left out (decision 493).</remarks>
public class TopOpeBRepBuild_Loop : Standard_Transient
{
    protected bool myIsShape;
    protected TopoDS_Shape myShape = new();
    protected TopOpeBRepBuild_BlockIterator myBlockIterator;

    /// <summary>The loop of the existing shape <paramref name="S"/>.</summary>
    public TopOpeBRepBuild_Loop(in TopoDS_Shape S)
    {
        myIsShape = true;
        myShape = S;
        myBlockIterator = new TopOpeBRepBuild_BlockIterator(0, 0);
    }

    /// <summary>The loop of the block of elements <paramref name="BI"/>.</summary>
    public TopOpeBRepBuild_Loop(in TopOpeBRepBuild_BlockIterator BI)
    {
        myIsShape = false;
        myBlockIterator = BI;
    }

    /// <summary>Whether the loop is an existing shape.</summary>
    public virtual bool IsShape() => myIsShape;

    /// <summary>The shape (null for a block).</summary>
    public virtual TopoDS_Shape Shape() => myShape;

    /// <summary>The block's iterator.</summary>
    public TopOpeBRepBuild_BlockIterator BlockIterator() => myBlockIterator;
}
