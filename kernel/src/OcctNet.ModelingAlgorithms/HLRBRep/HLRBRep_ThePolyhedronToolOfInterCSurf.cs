// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolyhedronToolOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolyhedronToolOfInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>Static access to a <see cref="HLRBRep_ThePolyhedronOfInterCSurf"/>, as the interference computes it.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members implementing
/// <see cref="IIntf_ToolPolyhedron{Polyhedron}"/> (decision 202's approach). <c>Dump</c>, which prints to
/// std::cout, is left out.
/// </remarks>
public readonly struct HLRBRep_ThePolyhedronToolOfInterCSurf
    : IIntf_ToolPolyhedron<HLRBRep_ThePolyhedronOfInterCSurf>
{
    /// <summary>The box of the polyhedron.</summary>
    public static Box Bounding(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh) => thePolyh.Bounding();

    /// <summary>The boxes of the triangles (the polyhedron's own array).</summary>
    public static NCollection_HArray1<Box> ComponentsBounding(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh) =>
        thePolyh.ComponentsBounding();

    public static double DeflectionOverEstimation(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh) =>
        thePolyh.DeflectionOverEstimation();

    public static int NbTriangles(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh) => thePolyh.NbTriangles();

    /// <summary>The indices of the three points of the triangle <paramref name="Index"/>.</summary>
    public static void Triangle(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh, int Index, out int P1, out int P2,
                                out int P3) => thePolyh.Triangle(Index, out P1, out P2, out P3);

    /// <summary>The point of index <paramref name="Index"/>.</summary>
    public static Point3D Point(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh, int Index) => thePolyh.Point(Index);

    /// <summary>
    /// The triangle connected to <paramref name="Triang"/> through the edge from <paramref name="Pivot"/> to
    /// <paramref name="Pedge"/> (0 on the boundary), and its other point.
    /// </summary>
    public static int TriConnex(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh, int Triang, int Pivot, int Pedge,
                                out int TriCon, out int OtherP) =>
        thePolyh.TriConnex(Triang, Pivot, Pedge, out TriCon, out OtherP);

    /// <summary>True if the edge between the two points lies on the boundary of the parameter range.</summary>
    public static bool IsOnBound(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh, int Index1, int Index2) =>
        thePolyh.IsOnBound(Index1, Index2);

    /// <summary>The maximum deflection between the boundary isolines and the boundary edges of the triangles.</summary>
    public static double GetBorderDeflection(in HLRBRep_ThePolyhedronOfInterCSurf thePolyh) =>
        thePolyh.GetBorderDeflection();
}
