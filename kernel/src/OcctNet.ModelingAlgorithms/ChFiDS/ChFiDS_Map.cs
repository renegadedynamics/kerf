// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Map.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_Map.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The sub-shapes of a shape of one type with their ancestors of another type (an indexed data map).</summary>
/// <remarks>
/// A working object (decision 21): OCCT holds it by value in the fillet builders and never copies it. The keys and
/// lists are the caller's shapes.
/// </remarks>
public sealed class ChFiDS_Map
{
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap =
        new(TopTools_ShapeMapHasher.Instance);

    /// <summary>An empty map.</summary>
    public ChFiDS_Map()
    {
    }

    /// <summary>Fills the map with the sub-shapes of type <paramref name="T1"/> of <paramref name="S"/> as keys and
    /// their ancestors of type <paramref name="T2"/> as items.</summary>
    public void Fill(in TopoDS_Shape S, TopAbs_ShapeEnum T1, TopAbs_ShapeEnum T2)
    {
        TopExp.MapShapesAndAncestors(S, T1, T2, myMap);
    }

    public bool Contains(in TopoDS_Shape S)
    {
        return myMap.Contains(S);
    }

    public NCollection_List<TopoDS_Shape> FindFromKey(in TopoDS_Shape S)
    {
        return myMap.FindFromKey(S);
    }

    /// <summary>OCCT's <c>operator()(S)</c>.</summary>
    public NCollection_List<TopoDS_Shape> this[in TopoDS_Shape S] => FindFromKey(S);

    public NCollection_List<TopoDS_Shape> FindFromIndex(int I)
    {
        return myMap.FindFromIndex(I);
    }

    /// <summary>OCCT's <c>operator()(I)</c>.</summary>
    public NCollection_List<TopoDS_Shape> this[int I] => FindFromIndex(I);
}
