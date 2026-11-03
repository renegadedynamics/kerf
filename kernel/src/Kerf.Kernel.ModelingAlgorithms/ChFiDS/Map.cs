// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Map.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Map.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Fillets.Data;

/// <summary>The sub-shapes of a shape of one type with their ancestors of another type (an indexed data map).</summary>
/// <remarks>
/// A working object (decision 21): OCCT holds it by value in the fillet builders and never copies it. The keys and
/// lists are the caller's shapes.
/// </remarks>
public sealed class Map
{
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> _map =
        new(ShapeHasher.Instance);

    /// <summary>An empty map.</summary>
    public Map()
    {
    }

    /// <summary>Fills the map with the sub-shapes of type <paramref name="t1"/> of <paramref name="s"/> as keys and
    /// their ancestors of type <paramref name="t2"/> as items.</summary>
    public void Fill(in Shape s, ShapeType t1, ShapeType t2)
    {
        Exploration.MapShapesAndAncestors(s, t1, t2, _map);
    }

    public bool Contains(in Shape s)
    {
        return _map.Contains(s);
    }

    public NCollection_List<Shape> FindFromKey(in Shape s)
    {
        return _map.FindFromKey(s);
    }

    /// <summary>OCCT's <c>operator()(S)</c>.</summary>
    public NCollection_List<Shape> this[in Shape s] => FindFromKey(s);

    public NCollection_List<Shape> FindFromIndex(int i)
    {
        return _map.FindFromIndex(i);
    }

    /// <summary>OCCT's <c>operator()(I)</c>.</summary>
    public NCollection_List<Shape> this[int i] => FindFromIndex(i);
}
