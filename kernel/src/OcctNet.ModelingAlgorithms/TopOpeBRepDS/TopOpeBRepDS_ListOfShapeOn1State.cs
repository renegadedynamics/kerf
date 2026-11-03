// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ListOfShapeOn1State.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ListOfShapeOn1State.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>A list of shapes (the parts of a shape in one state) and whether the shape is split.</summary>
/// <remarks>The legacy builder's storage of splits; ported with the data structure's core for unit 7R.</remarks>
public sealed class TopOpeBRepDS_ListOfShapeOn1State
{
    private readonly NCollection_List<Shape> myList = new();
    private int mySplits;

    /// <summary>An empty list, not split.</summary>
    public TopOpeBRepDS_ListOfShapeOn1State()
    {
        mySplits = 0;
    }

    /// <summary>The list.</summary>
    public NCollection_List<Shape> ListOnState() => myList;

    internal NCollection_List<Shape> ChangeListOnState() => myList;

    internal void Clear()
    {
        myList.Clear();
        mySplits = 0;
    }

    /// <summary>Whether the shape is split.</summary>
    public bool IsSplit()
    {
        bool res = false;
        if ((mySplits & 1) != 0)
        {
            res = true;
        }
        return res;
    }

    internal void Split(bool B = true)
    {
        int mask = 1;
        if (B)
        {
            mySplits |= mask;
        }
        else
        {
            mySplits &= ~mask;
        }
    }
}
