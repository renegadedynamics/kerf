// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_StripeMap.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_StripeMap.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_StripeMap.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The stripes that end at each vertex (an indexed data map of vertices to lists of stripes).</summary>
/// <remarks>A working object (decision 21) of the fillet builders, keyed with OCCT's shape hasher (IsSame).</remarks>
public sealed class ChFiDS_StripeMap
{
    private readonly NCollection_IndexedDataMap<TopoDS_Vertex, NCollection_List<ChFiDS_Stripe>> mymap =
        new(VertexHasher.Instance);

    public ChFiDS_StripeMap()
    {
    }

    /// <summary>Appends the stripe <paramref name="F"/> to the list of the vertex <paramref name="V"/>.</summary>
    internal void Add(in TopoDS_Vertex V, ChFiDS_Stripe F)
    {
        int Index = mymap.FindIndex(V);
        if (Index == 0)
        {
            NCollection_List<ChFiDS_Stripe> Empty = new();
            Index = mymap.Add(V, Empty);
        }

        mymap[Index].Append(F);
    }

    public int Extent()
    {
        return mymap.Extent();
    }

    public NCollection_List<ChFiDS_Stripe> FindFromKey(in TopoDS_Vertex V)
    {
        return mymap.FindFromKey(V);
    }

    /// <summary>OCCT's <c>operator()(V)</c>.</summary>
    public NCollection_List<ChFiDS_Stripe> this[in TopoDS_Vertex V] => FindFromKey(V);

    public NCollection_List<ChFiDS_Stripe> FindFromIndex(int I)
    {
        return mymap.FindFromIndex(I);
    }

    /// <summary>OCCT's <c>operator()(I)</c>.</summary>
    public NCollection_List<ChFiDS_Stripe> this[int I] => FindFromIndex(I);

    public TopoDS_Vertex FindKey(int I)
    {
        return mymap.FindKey(I);
    }

    internal void Clear()
    {
        mymap.Clear();
    }

    // TopTools_ShapeMapHasher on vertices: IsSame.
    private sealed class VertexHasher : IEqualityComparer<TopoDS_Vertex>
    {
        public static readonly VertexHasher Instance = new();

        public bool Equals(TopoDS_Vertex S1, TopoDS_Vertex S2) => TopTools_ShapeMapHasher.Instance.Equals(S1, S2);

        public int GetHashCode(TopoDS_Vertex S) => TopTools_ShapeMapHasher.Instance.GetHashCode(S);
    }
}
