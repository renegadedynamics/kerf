// Port of: src/FoundationClasses/TKMath/BVH/BVH_Triangulation.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using System.Numerics;

namespace Kerf.Engine.Geometry.Bvh;

/// <summary>A triangulation as a BVH primitive set: vertices, and triangles as vertex indices.</summary>
internal class Triangulation<T, TVec> : PrimitiveSet<T, TVec>
    where T : unmanaged, INumber<T>, IMinMaxValue<T>
    where TVec : unmanaged, INCollection_Vec<T, TVec>
{
    /// <summary>Array of vertex coordinates.</summary>
    public NCollection_LinearVector<TVec> Vertices = new();

    /// <summary>Array of indices of triangle vertices (x, y, z; w unused).</summary>
    public NCollection_LinearVector<NCollection_Vec4<int>> Elements = new();

    /// <summary>Creates an empty triangulation.</summary>
    public Triangulation()
    {
    }

    /// <summary>Creates an empty triangulation built with the given builder.</summary>
    public Triangulation(Builder<T, TVec> builder)
        : base(builder)
    {
    }

    /// <summary>The total number of triangles.</summary>
    public override int Size() => Elements.Size();

    /// <summary>The AABB of the given triangle.</summary>
    public override Box<T, TVec> Box(int index)
    {
        NCollection_Vec4<int> anIndex = Elements.Value(index);

        TVec point0 = Vertices.Value(anIndex.X());
        TVec point1 = Vertices.Value(anIndex.Y());
        TVec point2 = Vertices.Value(anIndex.Z());

        TVec minPoint = point0, maxPoint = point0;

        Kerf.Engine.Geometry.Bvh.BvhTools.CwiseMin<T, TVec>(ref minPoint, point1);
        Kerf.Engine.Geometry.Bvh.BvhTools.CwiseMin<T, TVec>(ref minPoint, point2);
        Kerf.Engine.Geometry.Bvh.BvhTools.CwiseMax<T, TVec>(ref maxPoint, point1);
        Kerf.Engine.Geometry.Bvh.BvhTools.CwiseMax<T, TVec>(ref maxPoint, point2);

        return new Box<T, TVec>(minPoint, maxPoint);
    }

    /// <summary>The centroid of the given triangle along the axis.</summary>
    public override T Center(int index, int axis)
    {
        NCollection_Vec4<int> anIndex = Elements.Value(index);

        TVec point0 = Vertices.Value(anIndex.X());
        TVec point1 = Vertices.Value(anIndex.Y());
        TVec point2 = Vertices.Value(anIndex.Z());

        return (Kerf.Engine.Geometry.Bvh.BvhTools.VecComp<T, TVec>(point0, axis) + Kerf.Engine.Geometry.Bvh.BvhTools.VecComp<T, TVec>(point1, axis)
                + Kerf.Engine.Geometry.Bvh.BvhTools.VecComp<T, TVec>(point2, axis))
               * T.CreateTruncating(1.0 / 3.0);
    }

    /// <summary>Swaps two triangles.</summary>
    public override void Swap(int index1, int index2)
    {
        ref NCollection_Vec4<int> indices1 = ref Elements.ChangeValue(index1);
        ref NCollection_Vec4<int> indices2 = ref Elements.ChangeValue(index2);
        (indices1, indices2) = (indices2, indices1);
    }
}
