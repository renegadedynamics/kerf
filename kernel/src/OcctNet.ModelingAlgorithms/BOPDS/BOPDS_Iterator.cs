// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Iterator.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Iterator.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PIterator.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry.Bounds;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
public class BOPDS_Iterator
{
    //! Precision::Confusion(), OCCT's default fuzzy value
    private const double DefaultFuzzyValue = 1.0e-7;

    protected int myLength;                                                      //!< Length of the intersection vector of particular intersection type
    protected BOPDS_DS? myDS;                                                    //!< Data Structure
    protected NCollection_DynamicArray<NCollection_DynamicArray<BOPDS_Pair>> myLists = new(0); //!< Pairs with interfering bounding boxes
    protected NCollection_DynamicArray<BOPDS_Pair>? myIteratorList;              //!< Iterator on each interfering type: its vector
    protected int myIteratorIndex;                                               //!< and its position
    protected bool myRunParallel;                                                //!< Flag for parallel processing
    protected NCollection_DynamicArray<NCollection_DynamicArray<BOPDS_Pair>> myExtLists = new(0); //!< Extra pairs of sub-shapes found after intersection of increased sub-shapes
    protected bool myUseExt;                                                     //!< Information flag for using the extra lists

    /// <summary>An iterator without data structure.</summary>
    public BOPDS_Iterator()
    {
        int i, aNb;
        //
        myRunParallel = false;
        myUseExt = false;
        myDS = null;
        myLength = 0;
        //
        aNb = BOPDS_DS.NbInterfTypes();
        for (i = 0; i < aNb; ++i)
        {
            myLists.Append(new NCollection_DynamicArray<BOPDS_Pair>());
        }
        int aNbExt = NbExtInterfs();
        for (i = 0; i < aNbExt; ++i)
        {
            myExtLists.Append(new NCollection_DynamicArray<BOPDS_Pair>());
        }
    }

    /// <summary>Switches the parallel processing on or off.</summary>
    public void SetRunParallel(bool theFlag) => myRunParallel = theFlag;

    /// <summary>True if the processing is parallel.</summary>
    public bool RunParallel() => myRunParallel;

    /// <summary>Sets the data structure to process.</summary>
    public void SetDS(BOPDS_DS? aDS) => myDS = aDS;

    /// <summary>The data structure.</summary>
    public BOPDS_DS DS() => myDS!;

    /// <summary>The number of pairs of the initialized type.</summary>
    public int ExpectedLength() => myLength;

    /// <summary>Half the expected length (at least 1).</summary>
    public int BlockLength()
    {
        int aNbIIs;
        double aCfPredict = 0.5;

        aNbIIs = ExpectedLength();
        if (aNbIIs <= 1)
        {
            return 1;
        }
        //
        aNbIIs = (int)(aCfPredict * (double)aNbIIs);
        return aNbIIs;
    }

    /// <summary>Starts iterating the pairs of shapes of the two types, sorted.</summary>
    public void Initialize(TopAbs_ShapeEnum aType1, TopAbs_ShapeEnum aType2)
    {
        int iX;
        //
        myLength = 0;
        myIteratorList = null;
        myIteratorIndex = 0;
        iX = BOPDS_Tools.TypeToInteger(aType1, aType2);
        if (iX >= 0)
        {
            NCollection_DynamicArray<BOPDS_Pair> aPairs =
                (myUseExt && iX < NbExtInterfs()) ? myExtLists.Value(iX) : myLists.Value(iX);
            // sort interfering pairs for constant order of intersection
            Sorting.stable_sort(aPairs.AsSpan(), new BOPDS_Pair.LessThan());
            // initialize iterator to access the pairs
            myIteratorList = aPairs;
            myLength = aPairs.Length();
        }
    }

    /// <summary>True if there are more pairs.</summary>
    public bool More() => myIteratorList is not null && myIteratorIndex < myIteratorList.Length();

    /// <summary>Moves to the next pair.</summary>
    public void Next() => ++myIteratorIndex;

    /// <summary>The indices of the current pair, the shape of the lower type (vertex before edge before face) first.</summary>
    public void Value(out int theI1, out int theI2)
    {
        int iT1, iT2, n1, n2;
        //
        BOPDS_Pair aPair = myIteratorList!.Value(myIteratorIndex);
        aPair.Indices(out n1, out n2);
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

    /// <summary>
    /// Finds the pairs of interfering boxes. With <paramref name="theCheckOBB"/>, the oriented boxes of the context
    /// (enlarged by the fuzzy value) must interfere too.
    /// </summary>
    public virtual void Prepare(IntTools_Context? theCtx = null, bool theCheckOBB = false, double theFuzzyValue = DefaultFuzzyValue)
    {
        int i, aNbInterfTypes;
        //
        aNbInterfTypes = BOPDS_DS.NbInterfTypes();
        myLength = 0;
        for (i = 0; i < aNbInterfTypes; ++i)
        {
            myLists.Value(i).Clear();
        }
        //
        if (myDS is null)
        {
            return;
        }
        Intersect(theCtx, theCheckOBB, theFuzzyValue);
    }

    /// <summary>
    /// Intersects the boxes of the shapes of <paramref name="theIndices"/> (taken from their same-domain shapes)
    /// with the tree of all boxes; the pairs found go to the extra lists (V/V, V/E, V/F), which
    /// <see cref="Initialize"/> then uses.
    /// </summary>
    public void IntersectExt(NCollection_IntMap theIndices)
    {
        if (myDS is null)
        {
            return;
        }
        int aNb = myDS.NbSourceShapes();
        BOPTools_BoxTree aBoxTree = new();
        aBoxTree.SetSize(aNb);
        NCollection_DynamicArray<BOPDS_TSR> aVTSR = new(theIndices.Extent());
        //
        for (int i = 0; i < aNb; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (!aSI.IsInterfering() || (aSI.ShapeType() == TopAbs_SOLID))
            {
                continue;
            }
            //
            if (theIndices.Contains(i))
            {
                int nVSD = i;
                myDS.HasShapeSD(i, ref nVSD);
                BOPDS_ShapeInfo aSISD = myDS.ShapeInfo(nVSD);
                Box aBox = aSISD.Box();
                aBoxTree.Add(i, Tools.Bnd2BVH(aBox));
                BOPDS_TSR aTSR = aVTSR.Append(new BOPDS_TSR());
                aTSR.SetHasBRep(true);
                aTSR.SetBVHSet(aBoxTree);
                aTSR.SetBox(Tools.Bnd2BVH(aBox));
                aTSR.SetIndex(i);
            }
            else
            {
                aBoxTree.Add(i, Tools.Bnd2BVH(aSI.Box()));
            }
        }

        aBoxTree.Build();

        // Perform selection
        BOPTools_Parallel.Perform(myRunParallel, aVTSR);

        // Treat selections

        // Fence map to avoid duplicating pairs
        NCollection_Map<BOPDS_Pair> aMPFence = new();

        int aNbV = aVTSR.Length();
        for (int k = 0; k < aNbV; ++k)
        {
            BOPDS_TSR aTSRi = aVTSR.Value(k);
            NCollection_List<int> aLI = aTSRi.Indices();
            if (aLI.IsEmpty())
            {
                continue;
            }

            int i = aTSRi.Index();
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            int iRankI = myDS.Rank(i);
            TopAbs_ShapeEnum aTI = aSI.ShapeType();
            int iTI = BOPDS_Tools.TypeToInteger(aTI);

            for (NCollection_List<int>.Iterator itLI = new(aLI); itLI.More(); itLI.Next())
            {
                int j = itLI.Value(); // Index in DS
                int iRankJ = myDS.Rank(j);
                if (iRankI == iRankJ)
                {
                    continue;
                }

                BOPDS_ShapeInfo aSJ = myDS.ShapeInfo(j);
                TopAbs_ShapeEnum aTJ = aSJ.ShapeType();
                int iTJ = BOPDS_Tools.TypeToInteger(aTJ);

                // avoid interfering of the shape with its sub-shapes
                if (((iTI < iTJ) && aSI.HasSubShape(j)) || ((iTI > iTJ) && aSJ.HasSubShape(i)))
                {
                    continue;
                }

                BOPDS_Pair aPair = new(i, j);
                if (aMPFence.Add(aPair))
                {
                    int iX = BOPDS_Tools.TypeToInteger(aTI, aTJ);
                    if (iX < NbExtInterfs())
                    {
                        myExtLists.Value(iX).Append(aPair);
                    }
                }
            }
        }

        myUseExt = true;
    }

    /// <summary>
    /// The number of extra interference types: the extra lists hold V/V, V/E and V/F pairs (and E/E, never filled).
    /// </summary>
    public static int NbExtInterfs() => 4;

    /// <summary>
    /// Intersects the boxes of the sub-shapes of the arguments with the tree and keeps the interfering pairs of
    /// shapes of different arguments for the geometrical intersection.
    /// </summary>
    protected virtual void Intersect(IntTools_Context? theCtx = null, bool theCheckOBB = false, double theFuzzyValue = DefaultFuzzyValue)
    {
        int aNb = myDS!.NbSourceShapes();

        // Prepare BVH
        BOPTools_BoxTree aBoxTree = new();
        aBoxTree.SetSize(aNb);
        for (int i = 0; i < aNb; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (!aSI.HasBRep())
            {
                continue;
            }
            Box aBox = aSI.Box();
            aBoxTree.Add(i, Tools.Bnd2BVH(aBox));
        }

        // Build BVH
        aBoxTree.Build();

        // Select pairs of shapes with interfering bounding boxes
        BOPTools_BoxPairSelector aPairSelector = new();
        aPairSelector.SetBVHSets(aBoxTree, aBoxTree);
        aPairSelector.SetSame(true);
        aPairSelector.Select();
        aPairSelector.Sort();

        // Treat the selected pairs
        NCollection_LinearVector<BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs> aPairs = aPairSelector.Pairs();
        int aNbPairs = aPairs.Size();

        int iPair = 0;

        int aNbR = myDS.NbRanges();
        for (int iR = 0; iR < aNbR; ++iR)
        {
            BOPDS_IndexRange aRange = myDS.Range(iR);

            for (; iPair < aNbPairs; ++iPair)
            {
                BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs aPair = aPairs.Value(iPair);
                if (!aRange.Contains(aPair.ID1))
                {
                    // Go to the next range
                    break;
                }

                if (aRange.Contains(aPair.ID2))
                {
                    // Go to the next pair
                    continue;
                }

                BOPDS_ShapeInfo aSI1 = myDS.ShapeInfo(aPair.ID1);
                BOPDS_ShapeInfo aSI2 = myDS.ShapeInfo(aPair.ID2);

                TopAbs_ShapeEnum aType1 = aSI1.ShapeType();
                TopAbs_ShapeEnum aType2 = aSI2.ShapeType();

                int iType1 = BOPDS_Tools.TypeToInteger(aType1);
                int iType2 = BOPDS_Tools.TypeToInteger(aType2);

                // avoid interfering of the shape with its sub-shapes
                if (((iType1 < iType2) && aSI1.HasSubShape(aPair.ID2))
                    || ((iType1 > iType2) && aSI2.HasSubShape(aPair.ID1)))
                {
                    continue;
                }

                if (theCheckOBB)
                {
                    // Check intersection of Oriented bounding boxes of the shapes
                    OrientedBox anOBB1 = theCtx!.OBB(aSI1.Shape(), theFuzzyValue);
                    OrientedBox anOBB2 = theCtx.OBB(aSI2.Shape(), theFuzzyValue);

                    if (anOBB1.IsOut(anOBB2))
                    {
                        continue;
                    }
                }

                int iX = BOPDS_Tools.TypeToInteger(aType1, aType2);
                myLists.Value(iX).Append(new BOPDS_Pair(Min(aPair.ID1, aPair.ID2), Max(aPair.ID1, aPair.ID2)));
            }
        }
    }

    /// <summary>A box selector run as a solver of <see cref="IntersectExt"/>'s parallel loop.</summary>
    internal sealed class BOPDS_TSR : BOPTools_BoxTreeSelector, BOPTools_Parallel.ISolver
    {
        private bool myHasBRep;
        private int myIndex;

        public BOPDS_TSR()
        {
            myHasBRep = false;
            myIndex = -1;
        }

        public void SetHasBRep(bool bFlag) => myHasBRep = bFlag;

        public void SetIndex(int theIndex) => myIndex = theIndex;

        public int Index() => myIndex;

        public void Perform()
        {
            if (myHasBRep)
            {
                Select();
            }
        }
    }
}
