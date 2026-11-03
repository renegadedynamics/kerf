// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolygonToolOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolygonToolOfInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>Static access to a <see cref="HLRBRep_ThePolygonOfInterCSurf"/>, as the interference computes it.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members implementing
/// <see cref="IIntf_ToolPolygon3d{Polygon3d}"/> (decision 202's approach). <c>Dump</c>, which prints to
/// std::cout, is left out.
/// </remarks>
public readonly struct HLRBRep_ThePolygonToolOfInterCSurf : IIntf_ToolPolygon3d<HLRBRep_ThePolygonOfInterCSurf>
{
    /// <summary>The bounding box of the polygon.</summary>
    public static Box Bounding(in HLRBRep_ThePolygonOfInterCSurf thePolygon) => thePolygon.Bounding();

    public static double DeflectionOverEstimation(in HLRBRep_ThePolygonOfInterCSurf thePolygon) =>
        thePolygon.DeflectionOverEstimation();

    public static bool Closed(in HLRBRep_ThePolygonOfInterCSurf thePolygon) => thePolygon.Closed();

    public static int NbSegments(in HLRBRep_ThePolygonOfInterCSurf thePolygon) => thePolygon.NbSegments();

    /// <summary>The first point of the segment of range <paramref name="Index"/>.</summary>
    public static Point3D BeginOfSeg(in HLRBRep_ThePolygonOfInterCSurf thePolygon, int Index) =>
        thePolygon.BeginOfSeg(Index);

    /// <summary>The last point of the segment of range <paramref name="Index"/>.</summary>
    public static Point3D EndOfSeg(in HLRBRep_ThePolygonOfInterCSurf thePolygon, int Index) =>
        thePolygon.EndOfSeg(Index);
}
