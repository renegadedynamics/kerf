// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IteratorSI.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IteratorSI.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PIteratorSI.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Finds the pairs of BRep sub-shapes whose bounding boxes interfere within each argument too (self-interference,
/// for the checker), and iterates them by interference type.
/// </summary>
public class BOPDS_IteratorSI : BOPDS_Iterator
{
    /// <summary>
    /// Keeps only the interference types up to the level: 0 V/V; 1 and V/E; 2 and E/E; 3 and V/F; 4 and E/F; any
    /// other level all types.
    /// </summary>
    public void UpdateByLevelOfCheck(int theLevel)
    {
        int i, aNbInterfTypes;
        //
        aNbInterfTypes = BOPDS_DS.NbInterfTypes();
        for (i = theLevel + 1; i < aNbInterfTypes; ++i)
        {
            myLists.Value(i).Clear();
        }
    }

    /// <inheritdoc/>
    protected override void Intersect(IntTools_Context? theCtx = null, bool theCheckOBB = false, double theFuzzyValue = 1.0e-7)
    {
        int aNbS = myDS!.NbSourceShapes();

        BOPTools_BoxTree aBBTree = new();
        aBBTree.SetSize(aNbS);

        for (int i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (!aSI.IsInterfering())
            {
                continue;
            }
            //
            Bnd_Box aBoxEx = aSI.Box();
            aBBTree.Add(i, Bnd_Tools.Bnd2BVH(aBoxEx));
        }

        aBBTree.Build();

        // Select pairs of shapes with interfering bounding boxes
        BOPTools_BoxPairSelector aPairSelector = new();
        aPairSelector.SetBVHSets(aBBTree, aBBTree);
        aPairSelector.SetSame(true);
        aPairSelector.Select();
        aPairSelector.Sort();

        // Treat the selected pairs
        NCollection_LinearVector<BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs> aPairs = aPairSelector.Pairs();
        int aNbPairs = aPairs.Size();

        for (int iPair = 0; iPair < aNbPairs; ++iPair)
        {
            BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs aPair = aPairs.Value(iPair);

            BOPDS_ShapeInfo aSI1 = myDS.ShapeInfo(aPair.ID1);
            BOPDS_ShapeInfo aSI2 = myDS.ShapeInfo(aPair.ID2);

            TopAbs_ShapeEnum aType1 = aSI1.ShapeType();
            TopAbs_ShapeEnum aType2 = aSI2.ShapeType();

            int iType1 = BOPDS_Tools.TypeToInteger(aType1);
            int iType2 = BOPDS_Tools.TypeToInteger(aType2);

            // avoid interfering of the shape with its sub-shapes
            if (((iType1 < iType2) && aSI1.HasSubShape(aPair.ID2)) || ((iType1 > iType2) && aSI2.HasSubShape(aPair.ID1)))
            {
                continue;
            }

            if (theCheckOBB)
            {
                // Check intersection of Oriented bounding boxes of the shapes
                Bnd_OBB anOBB1 = theCtx!.OBB(aSI1.Shape(), theFuzzyValue);
                Bnd_OBB anOBB2 = theCtx.OBB(aSI2.Shape(), theFuzzyValue);

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
