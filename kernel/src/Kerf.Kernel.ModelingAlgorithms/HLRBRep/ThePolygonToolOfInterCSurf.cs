// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolygonToolOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolygonToolOfInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Intersection.Polygons;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.HiddenLines;

/// <summary>Static access to a <see cref="ThePolygonOfInterCSurf"/>, as the interference computes it.</summary>
/// <remarks>
/// OCCT's static class is a <c>readonly struct</c> with static members implementing
/// <see cref="IToolPolygon3d{Polygon3d}"/> (decision 202's approach). <c>Dump</c>, which prints to
/// std::cout, is left out.
/// </remarks>
internal readonly struct ThePolygonToolOfInterCSurf : IToolPolygon3d<ThePolygonOfInterCSurf>
{
    /// <summary>The bounding box of the polygon.</summary>
    public static Box Bounding(in ThePolygonOfInterCSurf polygon) => polygon.Bounding();

    public static double DeflectionOverEstimation(in ThePolygonOfInterCSurf polygon) =>
        polygon.DeflectionOverEstimation();

    public static bool Closed(in ThePolygonOfInterCSurf polygon) => polygon.Closed();

    public static int NbSegments(in ThePolygonOfInterCSurf polygon) => polygon.SegmentCount;

    /// <summary>The first point of the segment of range <paramref name="index"/>.</summary>
    public static Point3D BeginOfSeg(in ThePolygonOfInterCSurf polygon, int index) =>
        polygon.BeginOfSeg(index);

    /// <summary>The last point of the segment of range <paramref name="index"/>.</summary>
    public static Point3D EndOfSeg(in ThePolygonOfInterCSurf polygon, int index) =>
        polygon.EndOfSeg(index);
}
