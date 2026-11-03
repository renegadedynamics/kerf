// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IteratorSI.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IteratorSI.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PIteratorSI.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// Finds the pairs of BRep sub-shapes whose bounding boxes interfere within each argument too (self-interference,
/// for the checker), and iterates them by interference type.
/// </summary>
public class IteratorSI : DataIterator
{
    /// <summary>
    /// Keeps only the interference types up to the level: 0 V/V; 1 and V/E; 2 and E/E; 3 and V/F; 4 and E/F; any
    /// other level all types.
    /// </summary>
    public void UpdateByLevelOfCheck(int level)
    {
        int i, nbInterfTypes;
        //
        nbInterfTypes = global::Kerf.Engine.Booleans.Data.DS.InterfTypeCount;
        for (i = level + 1; i < nbInterfTypes; ++i)
        {
            _lists[i].Clear();
        }
    }

    /// <inheritdoc/>
    protected override void Intersect(IntersectionContext? ctx = null, bool checkOBB = false, double fuzzyValue = 1.0e-7)
    {
        int nbS = _ds!.SourceShapeCount;

        BoxTree bbTree = new();
        bbTree.SetSize(nbS);

        for (int i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (!si.IsInterfering)
            {
                continue;
            }
            //
            Box boxEx = si.Box();
            bbTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxEx));
        }

        bbTree.Build();

        // Select pairs of shapes with interfering bounding boxes
        BoxPairSelector pairSelector = new();
        pairSelector.SetBVHSets(bbTree, bbTree);
        pairSelector.SetSame(true);
        pairSelector.Select();
        pairSelector.Sort();

        // Treat the selected pairs
        NCollection_LinearVector<PairSelector<NCollection_Vec3<double>>.PairIDs> pairs = pairSelector.Pairs();
        int nbPairs = pairs.Size();

        for (int iPair = 0; iPair < nbPairs; ++iPair)
        {
            PairSelector<NCollection_Vec3<double>>.PairIDs pair = pairs.Value(iPair);

            ShapeInfo si1 = _ds.ShapeInfo(pair.ID1);
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

            if (checkOBB)
            {
                // Check intersection of Oriented bounding boxes of the shapes
                Obb obb1 = ctx!.OBB(si1.Shape(), fuzzyValue);
                Obb obb2 = ctx.OBB(si2.Shape(), fuzzyValue);

                if (obb1.IsOut(obb2))
                {
                    continue;
                }
            }

            int iX = DataTools.TypeToInteger(type1, type2);
            _lists[iX].Add(new Pair(Min(pair.ID1, pair.ID2), Max(pair.ID1, pair.ID2)));
        }
    }
}
