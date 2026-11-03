// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyhedronBVH.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyhedronBVH.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bvh;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.Bvh.Constants;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The triangles of an IntPatch_Polyhedron as a BVH primitive set (built with the linear builder), for
/// IntPatch_InterferencePolyhedron. Triangles with a void box (degenerate ones) are left out.
/// </summary>
/// <remarks>
/// A working object (decision 21): it references the polyhedron it wraps, which must not change while the set is used,
/// and reorders its own index map when the tree is built. Indices of the set are 0-based;
/// <see cref="OriginalIndex"/> gives the polyhedron's 1-based triangle index.
/// </remarks>
public class PolyhedronBvh : PrimitiveSet<double, BVH_Vec3d>
{
    private Polyhedron? _poly;                            //!< Reference to the wrapped polyhedron
    private readonly NCollection_DynamicArray<int> _indexMap = new(); //!< Maps current indices to original 1-based indices

    /// <summary>An empty BVH set.</summary>
    public PolyhedronBvh()
        : base(new LinearBuilder<double, BVH_Vec3d>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth))
    {
        _poly = null;
    }

    /// <summary>The BVH set of the given polyhedron (which must stay valid while the set is used).</summary>
    public PolyhedronBvh(Polyhedron poly)
        : base(new LinearBuilder<double, BVH_Vec3d>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth))
    {
        _poly = null;
        Init(poly);
    }

    /// <summary>Initializes the set from the given polyhedron.</summary>
    public void Init(Polyhedron poly)
    {
        _poly = poly;

        int nbTriangles = PolyhedronTool.NbTriangles(poly);

        // Filter out degenerate triangles (those with void component bounding boxes)
        // by only including triangles with valid boxes in the index mapping.
        // This matches the behavior of the former Bnd_BoundSortBox-based approach,
        // which never matched void boxes.
        NCollection_HArray1<Box> compBnd = PolyhedronTool.ComponentsBounding(poly);

        _indexMap.Clear();
        for (int idx = 1; idx <= nbTriangles; ++idx)
        {
            if (!compBnd.Value(idx).IsVoid)
            {
                _indexMap.Append(idx); // Store 1-based original index
            }
        }

        // Mark as dirty to trigger BVH rebuild
        MarkDirty();
    }

    /// <summary>Clears the set.</summary>
    public void Clear()
    {
        _poly = null;
        _indexMap.Clear();
        MarkDirty();
    }

    /// <summary>The number of triangles.</summary>
    public override int Size()
    {
        if (_poly is null)
        {
            return 0;
        }
        return _indexMap.Size();
    }

    /// <summary>The box of the triangle of index <paramref name="index"/> (0-based, after reordering).</summary>
    public override Box<double, BVH_Vec3d> Box(int index)
    {
        Box<double, BVH_Vec3d> box = new();

        if (_poly is null || index < 0 || index >= Size())
        {
            return box;
        }

        // Get original 1-based triangle index
        int origIdx = _indexMap.Value(index);

        // Use pre-computed component bounding boxes from the polyhedron.
        // These boxes include deflection enlargement, matching the behavior
        // of the former Bnd_BoundSortBox-based approach. Degenerate triangles
        // (with void boxes) are already excluded during Init().
        NCollection_HArray1<Box> compBnd = PolyhedronTool.ComponentsBounding(_poly);
        Box bndBox = compBnd.Value(origIdx);

        bndBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax,
                    out double zmax);

        box.Add(new BVH_Vec3d(xmin, ymin, zmin));
        box.Add(new BVH_Vec3d(xmax, ymax, zmax));

        return box;
    }

    /// <summary>The coordinate of the centroid of the triangle <paramref name="index"/> on the axis (0, 1 or 2).</summary>
    public override double Center(int index, int axis)
    {
        if (_poly is null || index < 0 || index >= Size())
        {
            return 0.0;
        }

        // Get original 1-based triangle index
        int origIdx = _indexMap.Value(index);

        // Get triangle vertex indices
        PolyhedronTool.Triangle(_poly, origIdx, out int p1, out int p2, out int p3);

        // Get vertex coordinates
        Point3D pnt1 = PolyhedronTool.Point(_poly, p1);
        Point3D pnt2 = PolyhedronTool.Point(_poly, p2);
        Point3D pnt3 = PolyhedronTool.Point(_poly, p3);

        // Compute centroid coordinate along specified axis
        switch (axis)
        {
            case 0:
                return (pnt1.X() + pnt2.X() + pnt3.X()) / 3.0;
            case 1:
                return (pnt1.Y() + pnt2.Y() + pnt3.Y()) / 3.0;
            case 2:
                return (pnt1.Z() + pnt2.Z() + pnt3.Z()) / 3.0;
            default:
                return 0.0;
        }
    }

    /// <summary>Swaps two triangles of the set (while the tree is built).</summary>
    public override void Swap(int index1, int index2)
    {
        if (index1 != index2)
        {
            (_indexMap[index1], _indexMap[index2]) = (_indexMap[index2], _indexMap[index1]);
        }
    }

    /// <summary>
    /// The polyhedron's 1-based index of the triangle <paramref name="index"/> of the set (0-based, after
    /// reordering); 0 out of range.
    /// </summary>
    public int OriginalIndex(int index)
    {
        if (index < 0 || index >= Size())
        {
            return 0;
        }
        return _indexMap.Value(index);
    }

    /// <summary>True if the set wraps a polyhedron.</summary>
    public bool IsInitialized => _poly is not null;
}
