// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyhedronBVH.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyhedronBVH.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BVH_Constants;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;

namespace Occt;

/// <summary>
/// The triangles of an IntPatch_Polyhedron as a BVH primitive set (built with the linear builder), for
/// IntPatch_InterferencePolyhedron. Triangles with a void box (degenerate ones) are left out.
/// </summary>
/// <remarks>
/// A working object (decision 21): it references the polyhedron it wraps, which must not change while the set is used,
/// and reorders its own index map when the tree is built. Indices of the set are 0-based;
/// <see cref="OriginalIndex"/> gives the polyhedron's 1-based triangle index.
/// </remarks>
public class IntPatch_PolyhedronBVH : BVH_PrimitiveSet<double, BVH_Vec3d>
{
    private IntPatch_Polyhedron? myPoly;                            //!< Reference to the wrapped polyhedron
    private readonly NCollection_DynamicArray<int> myIndexMap = new(); //!< Maps current indices to original 1-based indices

    /// <summary>An empty BVH set.</summary>
    public IntPatch_PolyhedronBVH()
        : base(new BVH_LinearBuilder<double, BVH_Vec3d>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth))
    {
        myPoly = null;
    }

    /// <summary>The BVH set of the given polyhedron (which must stay valid while the set is used).</summary>
    public IntPatch_PolyhedronBVH(IntPatch_Polyhedron thePoly)
        : base(new BVH_LinearBuilder<double, BVH_Vec3d>(BVH_Constants_LeafNodeSizeDefault, BVH_Constants_MaxTreeDepth))
    {
        myPoly = null;
        Init(thePoly);
    }

    /// <summary>Initializes the set from the given polyhedron.</summary>
    public void Init(IntPatch_Polyhedron thePoly)
    {
        myPoly = thePoly;

        int aNbTriangles = IntPatch_PolyhedronTool.NbTriangles(thePoly);

        // Filter out degenerate triangles (those with void component bounding boxes)
        // by only including triangles with valid boxes in the index mapping.
        // This matches the behavior of the former Bnd_BoundSortBox-based approach,
        // which never matched void boxes.
        NCollection_HArray1<Bnd_Box> aCompBnd = IntPatch_PolyhedronTool.ComponentsBounding(thePoly);

        myIndexMap.Clear();
        for (int anIdx = 1; anIdx <= aNbTriangles; ++anIdx)
        {
            if (!aCompBnd.Value(anIdx).IsVoid())
            {
                myIndexMap.Append(anIdx); // Store 1-based original index
            }
        }

        // Mark as dirty to trigger BVH rebuild
        MarkDirty();
    }

    /// <summary>Clears the set.</summary>
    public void Clear()
    {
        myPoly = null;
        myIndexMap.Clear();
        MarkDirty();
    }

    /// <summary>The number of triangles.</summary>
    public override int Size()
    {
        if (myPoly is null)
        {
            return 0;
        }
        return myIndexMap.Size();
    }

    /// <summary>The box of the triangle of index <paramref name="theIndex"/> (0-based, after reordering).</summary>
    public override BVH_Box<double, BVH_Vec3d> Box(int theIndex)
    {
        BVH_Box<double, BVH_Vec3d> aBox = new();

        if (myPoly is null || theIndex < 0 || theIndex >= Size())
        {
            return aBox;
        }

        // Get original 1-based triangle index
        int anOrigIdx = myIndexMap.Value(theIndex);

        // Use pre-computed component bounding boxes from the polyhedron.
        // These boxes include deflection enlargement, matching the behavior
        // of the former Bnd_BoundSortBox-based approach. Degenerate triangles
        // (with void boxes) are already excluded during Init().
        NCollection_HArray1<Bnd_Box> aCompBnd = IntPatch_PolyhedronTool.ComponentsBounding(myPoly);
        Bnd_Box aBndBox = aCompBnd.Value(anOrigIdx);

        aBndBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax,
                    out double aZmax);

        aBox.Add(new BVH_Vec3d(aXmin, aYmin, aZmin));
        aBox.Add(new BVH_Vec3d(aXmax, aYmax, aZmax));

        return aBox;
    }

    /// <summary>The coordinate of the centroid of the triangle <paramref name="theIndex"/> on the axis (0, 1 or 2).</summary>
    public override double Center(int theIndex, int theAxis)
    {
        if (myPoly is null || theIndex < 0 || theIndex >= Size())
        {
            return 0.0;
        }

        // Get original 1-based triangle index
        int anOrigIdx = myIndexMap.Value(theIndex);

        // Get triangle vertex indices
        IntPatch_PolyhedronTool.Triangle(myPoly, anOrigIdx, out int aP1, out int aP2, out int aP3);

        // Get vertex coordinates
        gp_Pnt aPnt1 = IntPatch_PolyhedronTool.Point(myPoly, aP1);
        gp_Pnt aPnt2 = IntPatch_PolyhedronTool.Point(myPoly, aP2);
        gp_Pnt aPnt3 = IntPatch_PolyhedronTool.Point(myPoly, aP3);

        // Compute centroid coordinate along specified axis
        switch (theAxis)
        {
            case 0:
                return (aPnt1.X() + aPnt2.X() + aPnt3.X()) / 3.0;
            case 1:
                return (aPnt1.Y() + aPnt2.Y() + aPnt3.Y()) / 3.0;
            case 2:
                return (aPnt1.Z() + aPnt2.Z() + aPnt3.Z()) / 3.0;
            default:
                return 0.0;
        }
    }

    /// <summary>Swaps two triangles of the set (while the tree is built).</summary>
    public override void Swap(int theIndex1, int theIndex2)
    {
        if (theIndex1 != theIndex2)
        {
            (myIndexMap[theIndex1], myIndexMap[theIndex2]) = (myIndexMap[theIndex2], myIndexMap[theIndex1]);
        }
    }

    /// <summary>
    /// The polyhedron's 1-based index of the triangle <paramref name="theIndex"/> of the set (0-based, after
    /// reordering); 0 out of range.
    /// </summary>
    public int OriginalIndex(int theIndex)
    {
        if (theIndex < 0 || theIndex >= Size())
        {
            return 0;
        }
        return myIndexMap.Value(theIndex);
    }

    /// <summary>True if the set wraps a polyhedron.</summary>
    public bool IsInitialized() => myPoly is not null;
}
