// Port of: src/FoundationClasses/TKMath/BVH/BVH_Triangulation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Occt;

/// <summary>A triangulation as a BVH primitive set: vertices, and triangles as vertex indices.</summary>
public class BVH_Triangulation<T, TVec> : BVH_PrimitiveSet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Array of vertex coordinates.</summary>
    public NCollection_LinearVector<TVec> Vertices = new();

    /// <summary>Array of indices of triangle vertices (x, y, z; w unused).</summary>
    public NCollection_LinearVector<NCollection_Vec4<int>> Elements = new();

    /// <summary>Creates an empty triangulation.</summary>
    public BVH_Triangulation()
    {
    }

    /// <summary>Creates an empty triangulation built with the given builder.</summary>
    public BVH_Triangulation(BVH_Builder<T, TVec> theBuilder)
        : base(theBuilder)
    {
    }

    /// <summary>The total number of triangles.</summary>
    public override int Size() => Elements.Size();

    /// <summary>The AABB of the given triangle.</summary>
    public override BVH_Box<T, TVec> Box(int theIndex)
    {
        NCollection_Vec4<int> anIndex = Elements.Value(theIndex);

        TVec aPoint0 = Vertices.Value(anIndex.x());
        TVec aPoint1 = Vertices.Value(anIndex.y());
        TVec aPoint2 = Vertices.Value(anIndex.z());

        TVec aMinPoint = aPoint0, aMaxPoint = aPoint0;

        Occt.BVH.CwiseMin<T, TVec>(ref aMinPoint, aPoint1);
        Occt.BVH.CwiseMin<T, TVec>(ref aMinPoint, aPoint2);
        Occt.BVH.CwiseMax<T, TVec>(ref aMaxPoint, aPoint1);
        Occt.BVH.CwiseMax<T, TVec>(ref aMaxPoint, aPoint2);

        return new BVH_Box<T, TVec>(aMinPoint, aMaxPoint);
    }

    /// <summary>The centroid of the given triangle along the axis.</summary>
    public override T Center(int theIndex, int theAxis)
    {
        NCollection_Vec4<int> anIndex = Elements.Value(theIndex);

        TVec aPoint0 = Vertices.Value(anIndex.x());
        TVec aPoint1 = Vertices.Value(anIndex.y());
        TVec aPoint2 = Vertices.Value(anIndex.z());

        return (Occt.BVH.VecComp<T, TVec>(aPoint0, theAxis) + Occt.BVH.VecComp<T, TVec>(aPoint1, theAxis)
                + Occt.BVH.VecComp<T, TVec>(aPoint2, theAxis))
               * T.CreateTruncating(1.0 / 3.0);
    }

    /// <summary>Swaps two triangles.</summary>
    public override void Swap(int theIndex1, int theIndex2)
    {
        ref NCollection_Vec4<int> anIndices1 = ref Elements.ChangeValue(theIndex1);
        ref NCollection_Vec4<int> anIndices2 = ref Elements.ChangeValue(theIndex2);
        (anIndices1, anIndices2) = (anIndices2, anIndices1);
    }
}
