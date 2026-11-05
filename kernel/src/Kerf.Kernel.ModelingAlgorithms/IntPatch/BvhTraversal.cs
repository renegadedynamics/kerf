// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_BVHTraversal.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_BVHTraversal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using Occt;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The dual-tree traversal of two IntPatch_PolyhedronBVH sets: collects the pairs of triangles whose boxes overlap,
/// in the order the traversal meets them.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="Pairs"/> returns its own array. The pairs are not sorted (OCCT doesn't
/// sort them), so their order is the traversal's, which IntPatch_InterferencePolyhedron's section lines depend on.
/// </remarks>
internal class BvhTraversal : PairTraverse<double, BVH_Vec3d, double>
{
    /// <summary>A pair of triangle indices (both 1-based, the polyhedra's own indices).</summary>
    public readonly record struct TrianglePair(int First = 0, int Second = 0);

    private PolyhedronBvh? _set1;                                    //!< First BVH set
    private PolyhedronBvh? _set2;                                    //!< Second BVH set
    private bool _selfInterference;                                           //!< Self-interference mode flag
    private readonly List<TrianglePair> _pairs = new();   //!< Collected triangle pairs

    /// <summary>An empty traversal.</summary>
    public BvhTraversal()
    {
        _set1 = null;
        _set2 = null;
        _selfInterference = false;
    }

    /// <summary>
    /// Collects the pairs of triangles of the two sets whose boxes overlap; in self-interference mode, only the pairs
    /// whose first index is below the second. Returns the number of pairs.
    /// </summary>
    public int Perform(PolyhedronBvh set1, PolyhedronBvh set2, bool selfInterference = false)
    {
        _pairs.Clear();
        _set1 = set1;
        _set2 = set2;
        _selfInterference = selfInterference;

        if (!set1.IsInitialized || !set2.IsInitialized)
        {
            return 0;
        }

        if (set1.Size() == 0 || set2.Size() == 0)
        {
            return 0;
        }

        // Get BVH trees (builds them if necessary)
        Tree<double, BVH_Vec3d>? bvh1 = set1.BVH();
        Tree<double, BVH_Vec3d>? bvh2 = set2.BVH();

        if (bvh1 is null || bvh2 is null)
        {
            return 0;
        }

        // Perform dual-tree traversal
        return Select(bvh1, bvh2);
    }

    /// <summary>The collected pairs (the traversal's own array).</summary>
    public List<TrianglePair> Pairs() => _pairs;

    /// <summary>Clears the collected pairs.</summary>
    public void Clear() => _pairs.Clear();

    /// <summary>Rejects a pair of nodes whose boxes don't overlap.</summary>
    public override bool RejectNode(in BVH_Vec3d cMin1, in BVH_Vec3d cMax1, in BVH_Vec3d cMin2,
                                    in BVH_Vec3d cMax2, out double metric)
    {
        metric = 0.0;

        // AABB overlap test: reject if boxes don't overlap
        // Two boxes overlap if and only if they overlap on all three axes
        if (cMin1.X() > cMax2.X() || cMax1.X() < cMin2.X())
        {
            return true; // No overlap on X axis
        }
        if (cMin1.Y() > cMax2.Y() || cMax1.Y() < cMin2.Y())
        {
            return true; // No overlap on Y axis
        }
        if (cMin1.Z() > cMax2.Z() || cMax1.Z() < cMin2.Z())
        {
            return true; // No overlap on Z axis
        }

        return false; // Boxes overlap
    }

    /// <summary>Stores the pair of leaf elements (by their polyhedra's indices).</summary>
    public override bool Accept(int index1, int index2)
    {
        if (_set1 is null || _set2 is null)
        {
            return false;
        }

        // Get original 1-based triangle indices
        int origIdx1 = _set1.OriginalIndex(index1);
        int origIdx2 = _set2.OriginalIndex(index2);

        // In self-interference mode, skip pairs where first index >= second index
        // to avoid testing the same pair twice (and to avoid self-intersection)
        if (_selfInterference && origIdx1 >= origIdx2)
        {
            return false;
        }

        // Store the pair
        _pairs.Add(new TrianglePair(origIdx1, origIdx2));

        return true;
    }
}
