// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_SubIterator.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_SubIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation.Std;

using Kerf.Engine.Geometry.Bounds;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// Finds the pairs of shapes, one from each of two sub-sets of the data structure's shapes, whose bounding boxes
/// interfere, and iterates them (sorted).
/// </summary>
/// <remarks>As in OCCT, the sub-sets are the caller's lists, held by reference, not copied.</remarks>
public class SubIterator
{
    protected DS? _ds;
    protected NCollection_DynamicArray<Pair> _list = new(1);
    protected int _iteratorIndex;
    protected NCollection_List<int>? _subSet1;
    protected NCollection_List<int>? _subSet2;

    public SubIterator()
    {
        _ds = null;
    }

    /// <summary>Sets the data structure, for the shapes and their boxes.</summary>
    public void SetDS(DS? pDS) => _ds = pDS;

    public DS DS() => _ds!;

    /// <summary>Sets the first set of indices (held by reference).</summary>
    public void SetSubSet1(NCollection_List<int> li) => _subSet1 = li;

    public NCollection_List<int> SubSet1() => _subSet1!;

    /// <summary>Sets the second set of indices (held by reference).</summary>
    public void SetSubSet2(NCollection_List<int> li) => _subSet2 = li;

    public NCollection_List<int> SubSet2() => _subSet2!;

    /// <summary>Sorts the pairs and starts the iteration.</summary>
    public void Initialize()
    {
        // sort interfering pairs for constant order of intersection
        Sorting.stable_sort(_list.AsSpan(), new Pair.LessThan());
        // initialize iterator to access the pairs
        _iteratorIndex = 0;
    }

    public bool More() => _iteratorIndex < _list.Length();

    public void Next() => ++_iteratorIndex;

    /// <summary>The indices of the current pair, the shape of the lower type first.</summary>
    public void Value(out int i1, out int i2)
    {
        int iT1, iT2, n1, n2;
        //
        Pair pkb = _list.Value(_iteratorIndex);
        pkb.Indices(out n1, out n2);
        //
        iT1 = (int)_ds!.ShapeInfo(n1).ShapeType();
        iT2 = (int)_ds.ShapeInfo(n2).ShapeType();
        //
        i1 = n1;
        i2 = n2;
        if (iT1 < iT2)
        {
            i1 = n2;
            i2 = n1;
        }
    }

    /// <summary>Finds the pairs of interfering boxes.</summary>
    public virtual void Prepare()
    {
        _list.Clear();
        //
        if (_ds is null)
        {
            return;
        }
        //
        if (_subSet1!.Extent() == 0 || _subSet2!.Extent() == 0)
        {
            return;
        }
        //
        Intersect();
    }

    /// <summary>The number of interfering pairs.</summary>
    public int ExpectedLength() => _list.Length();

    protected virtual void Intersect()
    {
        if (_subSet1!.Extent() == 0 || _subSet2!.Extent() == 0)
        {
            return;
        }

        // Construct BVH tree for each sub-set
        BoxTree[] bbTree = [new(), new()];
        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<int> subSet = i == 0 ? _subSet1 : _subSet2;
            bbTree[i].SetSize(subSet.Extent());
            for (NCollection_List<int>.Iterator it = new(subSet); it.More(); it.Next())
            {
                int nS = it.Value();
                ShapeInfo si = _ds!.ShapeInfo(nS);
                Box boxEx = si.Box();
                bbTree[i].Add(nS, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxEx));
            }
            bbTree[i].Build();
        }

        // Perform selection of the interfering pairs
        BoxPairSelector pairSelector = new();
        pairSelector.SetBVHSets(bbTree[0], bbTree[1]);
        pairSelector.Select();
        pairSelector.Sort();

        // Treat the selected pairs
        NCollection_LinearVector<PairSelector<NCollection_Vec3<double>>.PairIDs> pairs = pairSelector.Pairs();
        int nbPairs = pairs.Size();

        // Fence map
        HashSet<Pair> mpkFence = new();

        for (int iPair = 0; iPair < nbPairs; ++iPair)
        {
            PairSelector<NCollection_Vec3<double>>.PairIDs pair = pairs.Value(iPair);
            if (pair.ID1 == pair.ID2)
            {
                continue;
            }

            Pair dsPair = new(Min(pair.ID1, pair.ID2), Max(pair.ID1, pair.ID2));
            if (!mpkFence.Add(dsPair))
            {
                continue;
            }

            ShapeInfo si1 = _ds!.ShapeInfo(pair.ID1);
            ShapeInfo si2 = _ds.ShapeInfo(pair.ID2);

            ShapeType type1 = si1.ShapeType();
            ShapeType type2 = si2.ShapeType();

            int iType1 = DataTools.TypeToInteger(type1);
            int iType2 = DataTools.TypeToInteger(type2);

            // avoid interfering of the shape with its sub-shapes
            if (((iType1 < iType2) && si1.HasSubShape(pair.ID2)) || ((iType1 > iType2) && si2.HasSubShape(pair.ID1)))
            {
                continue;
            }

            _list.Append(dsPair);
        }
    }
}
