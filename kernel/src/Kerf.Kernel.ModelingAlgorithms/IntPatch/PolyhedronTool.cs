// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyhedronTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyhedronTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>The accessors of IntPatch_Polyhedron that IntPatch_InterferencePolyhedron and the BVH use.</summary>
/// <remarks>
/// A static class: nothing instantiates a generic over it (OCCT's Intf generics are instantiated over other
/// polyhedra), so it needn't be a struct (decision 338).
/// </remarks>
internal static class PolyhedronTool
{
    /// <summary>The box of the polyhedron.</summary>
    public static Box Bounding(Polyhedron polyh) => polyh.Bounding();

    /// <summary>The boxes of the triangles (the polyhedron's own array: don't modify).</summary>
    public static NCollection_HArray1<Box> ComponentsBounding(Polyhedron polyh) =>
        polyh.ComponentsBounding();

    /// <summary>The deflection of the polyhedron.</summary>
    public static double DeflectionOverEstimation(Polyhedron polyh) => polyh.DeflectionOverEstimation();

    /// <summary>The number of triangles.</summary>
    public static int NbTriangles(Polyhedron polyh) => polyh.TriangleCount;

    /// <summary>The three vertex indices of the triangle <paramref name="index"/>.</summary>
    public static void Triangle(Polyhedron polyh, int index, out int p1, out int p2, out int p3) =>
        polyh.Triangle(index, out p1, out p2, out p3);

    /// <summary>The point of index <paramref name="index"/>.</summary>
    public static Point3D Point(Polyhedron polyh, int index) => polyh.Point(index);

    /// <summary>The triangle connected to <paramref name="triang"/> through an edge (see IntPatch_Polyhedron.TriConnex).</summary>
    public static int TriConnex(Polyhedron polyh, int triang, int pivot, int pedge, out int triCon,
                                out int otherP) =>
        polyh.TriConnex(triang, pivot, pedge, out triCon, out otherP);
}
