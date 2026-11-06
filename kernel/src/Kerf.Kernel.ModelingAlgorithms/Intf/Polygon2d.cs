// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Polygon2d.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Polygon2d.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Polygon2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Polygons;

/// <summary>The polygon information needed to compute interferences (<see cref="InterferencePolygon2d"/>).</summary>
internal abstract class Polygon2d
{
    protected Box2d _box = new();

    /// <summary>The bounding box of the polygon.</summary>
    public Box2d Bounding() => _box;

    /// <summary>True if the polyline is closed.</summary>
    public virtual bool Closed() => false;

    /// <summary>The tolerance of the polygon.</summary>
    public abstract double DeflectionOverEstimation();

    /// <summary>The number of segments in the polyline.</summary>
    public abstract int SegmentCount { get; }

    /// <summary>The points of the segment <paramref name="index"/> of the polygon.</summary>
    public abstract void Segment(int index, out Point2D begin, out Point2D end);
}
