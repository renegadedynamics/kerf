// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyhedronTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PolyhedronTool.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The accessors of IntPatch_Polyhedron that IntPatch_InterferencePolyhedron and the BVH use.</summary>
/// <remarks>
/// A static class: nothing instantiates a generic over it (OCCT's Intf generics are instantiated over other
/// polyhedra), so it needn't be a struct (decision 338).
/// </remarks>
public static class IntPatch_PolyhedronTool
{
    /// <summary>The box of the polyhedron.</summary>
    public static Bnd_Box Bounding(IntPatch_Polyhedron thePolyh) => thePolyh.Bounding();

    /// <summary>The boxes of the triangles (the polyhedron's own array: don't modify).</summary>
    public static NCollection_HArray1<Bnd_Box> ComponentsBounding(IntPatch_Polyhedron thePolyh) =>
        thePolyh.ComponentsBounding();

    /// <summary>The deflection of the polyhedron.</summary>
    public static double DeflectionOverEstimation(IntPatch_Polyhedron thePolyh) => thePolyh.DeflectionOverEstimation();

    /// <summary>The number of triangles.</summary>
    public static int NbTriangles(IntPatch_Polyhedron thePolyh) => thePolyh.NbTriangles();

    /// <summary>The three vertex indices of the triangle <paramref name="Index"/>.</summary>
    public static void Triangle(IntPatch_Polyhedron thePolyh, int Index, out int P1, out int P2, out int P3) =>
        thePolyh.Triangle(Index, out P1, out P2, out P3);

    /// <summary>The point of index <paramref name="Index"/>.</summary>
    public static gp_Pnt Point(IntPatch_Polyhedron thePolyh, int Index) => thePolyh.Point(Index);

    /// <summary>The triangle connected to <paramref name="Triang"/> through an edge (see IntPatch_Polyhedron.TriConnex).</summary>
    public static int TriConnex(IntPatch_Polyhedron thePolyh, int Triang, int Pivot, int Pedge, out int TriCon,
                                out int OtherP) =>
        thePolyh.TriConnex(Triang, Pivot, Pedge, out TriCon, out OtherP);
}
