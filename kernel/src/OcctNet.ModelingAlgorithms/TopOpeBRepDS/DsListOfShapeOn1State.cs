// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ListOfShapeOn1State.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_ListOfShapeOn1State.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>A list of shapes (the parts of a shape in one state) and whether the shape is split.</summary>
/// <remarks>The legacy builder's storage of splits; ported with the data structure's core for unit 7R.</remarks>
public sealed class DsListOfShapeOn1State
{
    private readonly NCollection_List<Shape> _list = new();
    private int _splits;

    /// <summary>An empty list, not split.</summary>
    public DsListOfShapeOn1State()
    {
        _splits = 0;
    }

    /// <summary>The list.</summary>
    public NCollection_List<Shape> ListOnState() => _list;

    internal NCollection_List<Shape> ChangeListOnState() => _list;

    internal void Clear()
    {
        _list.Clear();
        _splits = 0;
    }

    /// <summary>Whether the shape is split.</summary>
    public bool IsSplit()
    {
        bool res = false;
        if ((_splits & 1) != 0)
        {
            res = true;
        }
        return res;
    }

    internal void Split(bool b = true)
    {
        int mask = 1;
        if (b)
        {
            _splits |= mask;
        }
        else
        {
            _splits &= ~mask;
        }
    }
}
