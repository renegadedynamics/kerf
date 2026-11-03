// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_BVHTraversal.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_BVHTraversal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;

namespace Occt;

/// <summary>
/// The dual-tree traversal of two IntPatch_PolyhedronBVH sets: collects the pairs of triangles whose boxes overlap,
/// in the order the traversal meets them.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Pairs"/> returns its own array. The pairs are not sorted (OCCT doesn't
/// sort them), so their order is the traversal's, which IntPatch_InterferencePolyhedron's section lines depend on.
/// </remarks>
public class IntPatch_BVHTraversal : PairTraverse<double, BVH_Vec3d, double>
{
    /// <summary>A pair of triangle indices (both 1-based, the polyhedra's own indices).</summary>
    public readonly record struct TrianglePair(int First = 0, int Second = 0);

    private IntPatch_PolyhedronBVH? mySet1;                                    //!< First BVH set
    private IntPatch_PolyhedronBVH? mySet2;                                    //!< Second BVH set
    private bool mySelfInterference;                                           //!< Self-interference mode flag
    private readonly NCollection_DynamicArray<TrianglePair> myPairs = new();   //!< Collected triangle pairs

    /// <summary>An empty traversal.</summary>
    public IntPatch_BVHTraversal()
    {
        mySet1 = null;
        mySet2 = null;
        mySelfInterference = false;
    }

    /// <summary>
    /// Collects the pairs of triangles of the two sets whose boxes overlap; in self-interference mode, only the pairs
    /// whose first index is below the second. Returns the number of pairs.
    /// </summary>
    public int Perform(IntPatch_PolyhedronBVH theSet1, IntPatch_PolyhedronBVH theSet2, bool theSelfInterference = false)
    {
        myPairs.Clear();
        mySet1 = theSet1;
        mySet2 = theSet2;
        mySelfInterference = theSelfInterference;

        if (!theSet1.IsInitialized() || !theSet2.IsInitialized())
        {
            return 0;
        }

        if (theSet1.Size() == 0 || theSet2.Size() == 0)
        {
            return 0;
        }

        // Get BVH trees (builds them if necessary)
        Tree<double, BVH_Vec3d>? aBVH1 = theSet1.BVH();
        Tree<double, BVH_Vec3d>? aBVH2 = theSet2.BVH();

        if (aBVH1 is null || aBVH2 is null)
        {
            return 0;
        }

        // Perform dual-tree traversal
        return Select(aBVH1, aBVH2);
    }

    /// <summary>The collected pairs (the traversal's own array).</summary>
    public NCollection_DynamicArray<TrianglePair> Pairs() => myPairs;

    /// <summary>Clears the collected pairs.</summary>
    public void Clear() => myPairs.Clear();

    /// <summary>Rejects a pair of nodes whose boxes don't overlap.</summary>
    public override bool RejectNode(in BVH_Vec3d theCMin1, in BVH_Vec3d theCMax1, in BVH_Vec3d theCMin2,
                                    in BVH_Vec3d theCMax2, out double theMetric)
    {
        theMetric = 0.0;

        // AABB overlap test: reject if boxes don't overlap
        // Two boxes overlap if and only if they overlap on all three axes
        if (theCMin1.X() > theCMax2.X() || theCMax1.X() < theCMin2.X())
        {
            return true; // No overlap on X axis
        }
        if (theCMin1.Y() > theCMax2.Y() || theCMax1.Y() < theCMin2.Y())
        {
            return true; // No overlap on Y axis
        }
        if (theCMin1.Z() > theCMax2.Z() || theCMax1.Z() < theCMin2.Z())
        {
            return true; // No overlap on Z axis
        }

        return false; // Boxes overlap
    }

    /// <summary>Stores the pair of leaf elements (by their polyhedra's indices).</summary>
    public override bool Accept(int theIndex1, int theIndex2)
    {
        if (mySet1 is null || mySet2 is null)
        {
            return false;
        }

        // Get original 1-based triangle indices
        int anOrigIdx1 = mySet1.OriginalIndex(theIndex1);
        int anOrigIdx2 = mySet2.OriginalIndex(theIndex2);

        // In self-interference mode, skip pairs where first index >= second index
        // to avoid testing the same pair twice (and to avoid self-intersection)
        if (mySelfInterference && anOrigIdx1 >= anOrigIdx2)
        {
            return false;
        }

        // Store the pair
        myPairs.Append(new TrianglePair(anOrigIdx1, anOrigIdx2));

        return true;
    }
}
