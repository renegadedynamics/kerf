// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_StripeMap.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_StripeMap.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_StripeMap.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Fillets.Data;

/// <summary>The stripes that end at each vertex (an indexed data map of vertices to lists of stripes).</summary>
/// <remarks>A working object (decision 21) of the fillet builders, keyed with OCCT's shape hasher (IsSame).</remarks>
internal sealed class StripeMap
{
    private readonly IndexedMap<Vertex, NCollection_List<Stripe>> _mymap =
        new(VertexHasher.Instance);

    public StripeMap()
    {
    }

    /// <summary>Appends the stripe <paramref name="f"/> to the list of the vertex <paramref name="v"/>.</summary>
    internal void Add(in Vertex v, Stripe f)
    {
        int index = _mymap.IndexOf(v) + 1;
        if (index == 0)
        {
            NCollection_List<Stripe> empty = new();
            index = _mymap.Add(v, empty) + 1;
        }

        _mymap.ValueAt(index - 1).Append(f);
    }

    public int Extent()
    {
        return _mymap.Count;
    }

    public NCollection_List<Stripe> FindFromKey(in Vertex v)
    {
        return _mymap.GetValue(v);
    }

    /// <summary>OCCT's <c>operator()(V)</c>.</summary>
    public NCollection_List<Stripe> this[in Vertex v] => FindFromKey(v);

    public NCollection_List<Stripe> FindFromIndex(int i)
    {
        return _mymap.ValueAt(i - 1);
    }

    /// <summary>OCCT's <c>operator()(I)</c>.</summary>
    public NCollection_List<Stripe> this[int i] => FindFromIndex(i);

    public Vertex FindKey(int i)
    {
        return _mymap.KeyAt(i - 1);
    }

    internal void Clear()
    {
        _mymap.Clear();
    }

    // TopTools_ShapeMapHasher on vertices: IsSame.
    private sealed class VertexHasher : IEqualityComparer<Vertex>
    {
        public static readonly VertexHasher Instance = new();

        public bool Equals(Vertex s1, Vertex s2) => ShapeHasher.Instance.Equals(s1, s2);

        public int GetHashCode(Vertex s) => ShapeHasher.Instance.GetHashCode(s);
    }
}
