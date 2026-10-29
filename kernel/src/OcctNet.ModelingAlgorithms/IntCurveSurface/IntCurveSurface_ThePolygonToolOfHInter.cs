// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_ThePolygonToolOfHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_ThePolygonToolOfHInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Static access to a <see cref="IntCurveSurface_ThePolygonOfHInter"/>, as the interference computes it.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members implementing
/// <see cref="IIntf_ToolPolygon3d{Polygon3d}"/> (decision 202's approach). <c>Dump</c>, which prints to
/// std::cout, is left out.
/// </remarks>
public readonly struct IntCurveSurface_ThePolygonToolOfHInter : IIntf_ToolPolygon3d<IntCurveSurface_ThePolygonOfHInter>
{
    /// <summary>The bounding box of the polygon.</summary>
    public static Bnd_Box Bounding(in IntCurveSurface_ThePolygonOfHInter thePolygon) => thePolygon.Bounding();

    public static double DeflectionOverEstimation(in IntCurveSurface_ThePolygonOfHInter thePolygon) =>
        thePolygon.DeflectionOverEstimation();

    public static bool Closed(in IntCurveSurface_ThePolygonOfHInter thePolygon) => thePolygon.Closed();

    public static int NbSegments(in IntCurveSurface_ThePolygonOfHInter thePolygon) => thePolygon.NbSegments();

    /// <summary>The first point of the segment of range <paramref name="Index"/>.</summary>
    public static gp_Pnt BeginOfSeg(in IntCurveSurface_ThePolygonOfHInter thePolygon, int Index) =>
        thePolygon.BeginOfSeg(Index);

    /// <summary>The last point of the segment of range <paramref name="Index"/>.</summary>
    public static gp_Pnt EndOfSeg(in IntCurveSurface_ThePolygonOfHInter thePolygon, int Index) =>
        thePolygon.EndOfSeg(Index);
}
