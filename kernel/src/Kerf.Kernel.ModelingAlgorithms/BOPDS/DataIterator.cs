// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Iterator.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Iterator.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PIterator.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// Finds the pairs of BRep sub-shapes of different arguments whose bounding boxes interfere, and iterates the pairs
/// of one interference type (vertex/vertex, vertex/edge, ...).
/// </summary>
/// <remarks>
/// <para>
/// A working object of the pave filler (plan section 6); OCCT's <c>BOPDS_PIterator</c> is a reference to it. The
/// pairs of a type are sorted (OCCT's <c>std::stable_sort</c>) when the type is initialized, so the order of
/// intersection doesn't depend on the box tree. The selection in <see cref="IntersectExt"/> runs its solvers in
/// parallel when <see cref="RunParallel"/>; each solver writes only its own indices, read afterwards in vector order,
/// so the parallel run gives the serial result (decision 580).
/// </para>
/// <para>
/// OCCT's default arguments <c>occ::handle&lt;IntTools_Context&gt;()</c> are null; its default fuzzy value
/// <c>Precision::Confusion()</c> is written as the constant 1e-7.
/// </para>
/// </remarks>
public class DataIterator
{
    //! Precision::Confusion(), OCCT's default fuzzy value
    private const double DefaultFuzzyValue = 1.0e-7;

    protected int _length;                                                      //!< Length of the intersection vector of particular intersection type
    protected DS? _ds;                                                    //!< Data Structure
    protected NCollection_DynamicArray<NCollection_DynamicArray<Pair>> _lists = new(0); //!< Pairs with interfering bounding boxes
    protected NCollection_DynamicArray<Pair>? _iteratorList;              //!< Iterator on each interfering type: its vector
    protected int _iteratorIndex;                                               //!< and its position
    protected bool _runParallel;                                                //!< Flag for parallel processing
    protected NCollection_DynamicArray<NCollection_DynamicArray<Pair>> _extLists = new(0); //!< Extra pairs of sub-shapes found after intersection of increased sub-shapes
    protected bool _useExt;                                                     //!< Information flag for using the extra lists

    /// <summary>An iterator without data structure.</summary>
    public DataIterator()
    {
        int i, nb;
        //
        _runParallel = false;
        _useExt = false;
        _ds = null;
        _length = 0;
        //
        nb = global::Kerf.Engine.Booleans.Data.DS.InterfTypeCount;
        for (i = 0; i < nb; ++i)
        {
            _lists.Append(new NCollection_DynamicArray<Pair>());
        }
        int nbExt = ExtInterfCount;
        for (i = 0; i < nbExt; ++i)
        {
            _extLists.Append(new NCollection_DynamicArray<Pair>());
        }
    }

    /// <summary>Switches the parallel processing on or off.</summary>
    public void SetRunParallel(bool flag) => _runParallel = flag;

    /// <summary>True if the processing is parallel.</summary>
    public bool RunParallel() => _runParallel;

    /// <summary>Sets the data structure to process.</summary>
    public void SetDS(DS? aDS) => _ds = aDS;

    /// <summary>The data structure.</summary>
    public DS DS() => _ds!;

    /// <summary>The number of pairs of the initialized type.</summary>
    public int ExpectedLength() => _length;

    /// <summary>Half the expected length (at least 1).</summary>
    public int BlockLength()
    {
        int nbIIs;
        double cfPredict = 0.5;

        nbIIs = ExpectedLength();
        if (nbIIs <= 1)
        {
            return 1;
        }
        //
        nbIIs = (int)(cfPredict * (double)nbIIs);
        return nbIIs;
    }

    /// <summary>Starts iterating the pairs of shapes of the two types, sorted.</summary>
    public void Initialize(ShapeType aType1, ShapeType aType2)
    {
        int iX;
        //
        _length = 0;
        _iteratorList = null;
        _iteratorIndex = 0;
        iX = DataTools.TypeToInteger(aType1, aType2);
        if (iX >= 0)
        {
            NCollection_DynamicArray<Pair> pairs =
                (_useExt && iX < ExtInterfCount) ? _extLists.Value(iX) : _lists.Value(iX);
            // sort interfering pairs for constant order of intersection
            Sorting.stable_sort(pairs.AsSpan(), new Pair.LessThan());
            // initialize iterator to access the pairs
            _iteratorList = pairs;
            _length = pairs.Length();
        }
    }

    /// <summary>True if there are more pairs.</summary>
    public bool More() => _iteratorList is not null && _iteratorIndex < _iteratorList.Length();

    /// <summary>Moves to the next pair.</summary>
    public void Next() => ++_iteratorIndex;

    /// <summary>The indices of the current pair, the shape of the lower type (vertex before edge before face) first.</summary>
    public void Value(out int i1, out int i2)
    {
        int iT1, iT2, n1, n2;
        //
        Pair pair = _iteratorList!.Value(_iteratorIndex);
        pair.Indices(out n1, out n2);
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

    /// <summary>
    /// Finds the pairs of interfering boxes. With <paramref name="checkOBB"/>, the oriented boxes of the context
    /// (enlarged by the fuzzy value) must interfere too.
    /// </summary>
    public virtual void Prepare(IntersectionContext? ctx = null, bool checkOBB = false, double fuzzyValue = DefaultFuzzyValue)
    {
        int i, nbInterfTypes;
        //
        nbInterfTypes = global::Kerf.Engine.Booleans.Data.DS.InterfTypeCount;
        _length = 0;
        for (i = 0; i < nbInterfTypes; ++i)
        {
            _lists.Value(i).Clear();
        }
        //
        if (_ds is null)
        {
            return;
        }
        Intersect(ctx, checkOBB, fuzzyValue);
    }

    /// <summary>
    /// Intersects the boxes of the shapes of <paramref name="indices"/> (taken from their same-domain shapes)
    /// with the tree of all boxes; the pairs found go to the extra lists (V/V, V/E, V/F), which
    /// <see cref="Initialize"/> then uses.
    /// </summary>
    public void IntersectExt(NCollection_IntMap indices)
    {
        if (_ds is null)
        {
            return;
        }
        int nb = _ds.SourceShapeCount;
        BoxTree boxTree = new();
        boxTree.SetSize(nb);
        NCollection_DynamicArray<BOPDS_TSR> vtsr = new(indices.Extent());
        //
        for (int i = 0; i < nb; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (!si.IsInterfering || (si.ShapeType() == ShapeType.Solid))
            {
                continue;
            }
            //
            if (indices.Contains(i))
            {
                int nVSD = i;
                _ds.HasShapeSD(i, ref nVSD);
                ShapeInfo sisd = _ds.ShapeInfo(nVSD);
                Box box = sisd.Box();
                boxTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(box));
                BOPDS_TSR tsr = vtsr.Append(new BOPDS_TSR());
                tsr.SetHasBRep(true);
                tsr.SetBVHSet(boxTree);
                tsr.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(box));
                tsr.SetIndex(i);
            }
            else
            {
                boxTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(si.Box()));
            }
        }

        boxTree.Build();

        // Perform selection
        BooleanParallel.Perform(_runParallel, vtsr);

        // Treat selections

        // Fence map to avoid duplicating pairs
        NCollection_Map<Pair> mpFence = new();

        int nbV = vtsr.Length();
        for (int k = 0; k < nbV; ++k)
        {
            BOPDS_TSR tsRi = vtsr.Value(k);
            NCollection_List<int> li = tsRi.Indices();
            if (li.IsEmpty())
            {
                continue;
            }

            int i = tsRi.Index();
            ShapeInfo si = _ds.ShapeInfo(i);
            int iRankI = _ds.Rank(i);
            ShapeType ti = si.ShapeType();
            int iTI = DataTools.TypeToInteger(ti);

            for (NCollection_List<int>.Iterator itLI = new(li); itLI.More(); itLI.Next())
            {
                int j = itLI.Value(); // Index in DS
                int iRankJ = _ds.Rank(j);
                if (iRankI == iRankJ)
                {
                    continue;
                }

                ShapeInfo sj = _ds.ShapeInfo(j);
                ShapeType tj = sj.ShapeType();
                int iTJ = DataTools.TypeToInteger(tj);

                // avoid interfering of the shape with its sub-shapes
                if (((iTI < iTJ) && si.HasSubShape(j)) || ((iTI > iTJ) && sj.HasSubShape(i)))
                {
                    continue;
                }

                Pair pair = new(i, j);
                if (mpFence.Add(pair))
                {
                    int iX = DataTools.TypeToInteger(ti, tj);
                    if (iX < ExtInterfCount)
                    {
                        _extLists.Value(iX).Append(pair);
                    }
                }
            }
        }

        _useExt = true;
    }

    /// <summary>
    /// The number of extra interference types: the extra lists hold V/V, V/E and V/F pairs (and E/E, never filled).
    /// </summary>
    public static int ExtInterfCount => 4;

    /// <summary>
    /// Intersects the boxes of the sub-shapes of the arguments with the tree and keeps the interfering pairs of
    /// shapes of different arguments for the geometrical intersection.
    /// </summary>
    protected virtual void Intersect(IntersectionContext? ctx = null, bool checkOBB = false, double fuzzyValue = DefaultFuzzyValue)
    {
        int nb = _ds!.SourceShapeCount;

        // Prepare BVH
        BoxTree boxTree = new();
        boxTree.SetSize(nb);
        for (int i = 0; i < nb; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (!si.HasBRep)
            {
                continue;
            }
            Box box = si.Box();
            boxTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(box));
        }

        // Build BVH
        boxTree.Build();

        // Select pairs of shapes with interfering bounding boxes
        BoxPairSelector pairSelector = new();
        pairSelector.SetBVHSets(boxTree, boxTree);
        pairSelector.SetSame(true);
        pairSelector.Select();
        pairSelector.Sort();

        // Treat the selected pairs
        NCollection_LinearVector<PairSelector<NCollection_Vec3<double>>.PairIDs> pairs = pairSelector.Pairs();
        int nbPairs = pairs.Size();

        int iPair = 0;

        int nbR = _ds.RangeCount;
        for (int iR = 0; iR < nbR; ++iR)
        {
            IndexRange range = _ds.Range(iR);

            for (; iPair < nbPairs; ++iPair)
            {
                PairSelector<NCollection_Vec3<double>>.PairIDs pair = pairs.Value(iPair);
                if (!range.Contains(pair.ID1))
                {
                    // Go to the next range
                    break;
                }

                if (range.Contains(pair.ID2))
                {
                    // Go to the next pair
                    continue;
                }

                ShapeInfo si1 = _ds.ShapeInfo(pair.ID1);
                ShapeInfo si2 = _ds.ShapeInfo(pair.ID2);

                ShapeType type1 = si1.ShapeType();
                ShapeType type2 = si2.ShapeType();

                int iType1 = DataTools.TypeToInteger(type1);
                int iType2 = DataTools.TypeToInteger(type2);

                // avoid interfering of the shape with its sub-shapes
                if (((iType1 < iType2) && si1.HasSubShape(pair.ID2))
                    || ((iType1 > iType2) && si2.HasSubShape(pair.ID1)))
                {
                    continue;
                }

                if (checkOBB)
                {
                    // Check intersection of Oriented bounding boxes of the shapes
                    OrientedBox obb1 = ctx!.OBB(si1.Shape(), fuzzyValue);
                    OrientedBox obb2 = ctx.OBB(si2.Shape(), fuzzyValue);

                    if (obb1.IsOut(obb2))
                    {
                        continue;
                    }
                }

                int iX = DataTools.TypeToInteger(type1, type2);
                _lists.Value(iX).Append(new Pair(Min(pair.ID1, pair.ID2), Max(pair.ID1, pair.ID2)));
            }
        }
    }

    /// <summary>A box selector run as a solver of <see cref="IntersectExt"/>'s parallel loop.</summary>
    internal sealed class BOPDS_TSR : BoxTreeSelector, BooleanParallel.ISolver
    {
        private bool _hasBRep;
        private int _index;

        public BOPDS_TSR()
        {
            _hasBRep = false;
            _index = -1;
        }

        public void SetHasBRep(bool bFlag) => _hasBRep = bFlag;

        public void SetIndex(int index) => _index = index;

        public int Index() => _index;

        public void Perform()
        {
            if (_hasBRep)
            {
                Select();
            }
        }
    }
}
