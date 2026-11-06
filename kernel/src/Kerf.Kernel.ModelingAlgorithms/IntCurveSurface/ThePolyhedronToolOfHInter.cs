// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_ThePolyhedronToolOfHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_ThePolyhedronToolOfHInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.Polygons;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>Static access to a <see cref="ThePolyhedronOfHInter"/>, as the interference computes it.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members implementing
/// <see cref="IToolPolyhedron{Polyhedron}"/> (decision 202's approach). <c>Dump</c>, which prints to
/// std::cout, is left out.
/// </remarks>
internal readonly struct ThePolyhedronToolOfHInter
    : IToolPolyhedron<ThePolyhedronOfHInter>
{
    /// <summary>The box of the polyhedron.</summary>
    public static Box Bounding(in ThePolyhedronOfHInter polyh) => polyh.Bounding();

    /// <summary>The boxes of the triangles (the polyhedron's own array).</summary>
    public static NCollection_HArray1<Box> ComponentsBounding(in ThePolyhedronOfHInter polyh) =>
        polyh.ComponentsBounding();

    public static double DeflectionOverEstimation(in ThePolyhedronOfHInter polyh) =>
        polyh.DeflectionOverEstimation();

    public static int NbTriangles(in ThePolyhedronOfHInter polyh) => polyh.TriangleCount;

    /// <summary>The indices of the three points of the triangle <paramref name="index"/>.</summary>
    public static void Triangle(in ThePolyhedronOfHInter polyh, int index, out int p1, out int p2,
                                out int p3) => polyh.Triangle(index, out p1, out p2, out p3);

    /// <summary>The point of index <paramref name="index"/>.</summary>
    public static Point3D Point(in ThePolyhedronOfHInter polyh, int index) => polyh.Point(index);

    /// <summary>
    /// The triangle connected to <paramref name="triang"/> through the edge from <paramref name="pivot"/> to
    /// <paramref name="pedge"/> (0 on the boundary), and its other point.
    /// </summary>
    public static int TriConnex(in ThePolyhedronOfHInter polyh, int triang, int pivot, int pedge,
                                out int triCon, out int otherP) =>
        polyh.TriConnex(triang, pivot, pedge, out triCon, out otherP);

    /// <summary>True if the edge between the two points lies on the boundary of the parameter range.</summary>
    public static bool IsOnBound(in ThePolyhedronOfHInter polyh, int index1, int index2) =>
        polyh.IsOnBound(index1, index2);

    /// <summary>The maximum deflection between the boundary isolines and the boundary edges of the triangles.</summary>
    public static double GetBorderDeflection(in ThePolyhedronOfHInter polyh) =>
        polyh.GetBorderDeflection();
}
