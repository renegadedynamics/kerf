// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_SubIterator.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_SubIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

using Kerf.Engine.Geometry.Bounds;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// Finds the pairs of shapes, one from each of two sub-sets of the data structure's shapes, whose bounding boxes
/// interfere, and iterates them (sorted).
/// </summary>
/// <remarks>As in OCCT, the sub-sets are the caller's lists, held by reference, not copied.</remarks>
public class BOPDS_SubIterator
{
    protected BOPDS_DS? myDS;
    protected NCollection_DynamicArray<BOPDS_Pair> myList = new(1);
    protected int myIteratorIndex;
    protected NCollection_List<int>? mySubSet1;
    protected NCollection_List<int>? mySubSet2;

    public BOPDS_SubIterator()
    {
        myDS = null;
    }

    /// <summary>Sets the data structure, for the shapes and their boxes.</summary>
    public void SetDS(BOPDS_DS? pDS) => myDS = pDS;

    public BOPDS_DS DS() => myDS!;

    /// <summary>Sets the first set of indices (held by reference).</summary>
    public void SetSubSet1(NCollection_List<int> theLI) => mySubSet1 = theLI;

    public NCollection_List<int> SubSet1() => mySubSet1!;

    /// <summary>Sets the second set of indices (held by reference).</summary>
    public void SetSubSet2(NCollection_List<int> theLI) => mySubSet2 = theLI;

    public NCollection_List<int> SubSet2() => mySubSet2!;

    /// <summary>Sorts the pairs and starts the iteration.</summary>
    public void Initialize()
    {
        // sort interfering pairs for constant order of intersection
        Sorting.stable_sort(myList.AsSpan(), new BOPDS_Pair.LessThan());
        // initialize iterator to access the pairs
        myIteratorIndex = 0;
    }

    public bool More() => myIteratorIndex < myList.Length();

    public void Next() => ++myIteratorIndex;

    /// <summary>The indices of the current pair, the shape of the lower type first.</summary>
    public void Value(out int theI1, out int theI2)
    {
        int iT1, iT2, n1, n2;
        //
        BOPDS_Pair aPKB = myList.Value(myIteratorIndex);
        aPKB.Indices(out n1, out n2);
        //
        iT1 = (int)myDS!.ShapeInfo(n1).ShapeType();
        iT2 = (int)myDS.ShapeInfo(n2).ShapeType();
        //
        theI1 = n1;
        theI2 = n2;
        if (iT1 < iT2)
        {
            theI1 = n2;
            theI2 = n1;
        }
    }

    /// <summary>Finds the pairs of interfering boxes.</summary>
    public virtual void Prepare()
    {
        myList.Clear();
        //
        if (myDS is null)
        {
            return;
        }
        //
        if (mySubSet1!.Extent() == 0 || mySubSet2!.Extent() == 0)
        {
            return;
        }
        //
        Intersect();
    }

    /// <summary>The number of interfering pairs.</summary>
    public int ExpectedLength() => myList.Length();

    protected virtual void Intersect()
    {
        if (mySubSet1!.Extent() == 0 || mySubSet2!.Extent() == 0)
        {
            return;
        }

        // Construct BVH tree for each sub-set
        BOPTools_BoxTree[] aBBTree = [new(), new()];
        for (int i = 0; i < 2; ++i)
        {
            NCollection_List<int> aSubSet = i == 0 ? mySubSet1 : mySubSet2;
            aBBTree[i].SetSize(aSubSet.Extent());
            for (NCollection_List<int>.Iterator it = new(aSubSet); it.More(); it.Next())
            {
                int nS = it.Value();
                BOPDS_ShapeInfo aSI = myDS!.ShapeInfo(nS);
                Box aBoxEx = aSI.Box();
                aBBTree[i].Add(nS, Tools.Bnd2BVH(aBoxEx));
            }
            aBBTree[i].Build();
        }

        // Perform selection of the interfering pairs
        BOPTools_BoxPairSelector aPairSelector = new();
        aPairSelector.SetBVHSets(aBBTree[0], aBBTree[1]);
        aPairSelector.Select();
        aPairSelector.Sort();

        // Treat the selected pairs
        NCollection_LinearVector<BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs> aPairs = aPairSelector.Pairs();
        int aNbPairs = aPairs.Size();

        // Fence map
        NCollection_Map<BOPDS_Pair> aMPKFence = new();

        for (int iPair = 0; iPair < aNbPairs; ++iPair)
        {
            BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs aPair = aPairs.Value(iPair);
            if (aPair.ID1 == aPair.ID2)
            {
                continue;
            }

            BOPDS_Pair aDSPair = new(Min(aPair.ID1, aPair.ID2), Max(aPair.ID1, aPair.ID2));
            if (!aMPKFence.Add(aDSPair))
            {
                continue;
            }

            BOPDS_ShapeInfo aSI1 = myDS!.ShapeInfo(aPair.ID1);
            BOPDS_ShapeInfo aSI2 = myDS.ShapeInfo(aPair.ID2);

            ShapeType aType1 = aSI1.ShapeType();
            ShapeType aType2 = aSI2.ShapeType();

            int iType1 = BOPDS_Tools.TypeToInteger(aType1);
            int iType2 = BOPDS_Tools.TypeToInteger(aType2);

            // avoid interfering of the shape with its sub-shapes
            if (((iType1 < iType2) && aSI1.HasSubShape(aPair.ID2)) || ((iType1 > iType2) && aSI2.HasSubShape(aPair.ID1)))
            {
                continue;
            }

            myList.Append(aDSPair);
        }
    }
}
