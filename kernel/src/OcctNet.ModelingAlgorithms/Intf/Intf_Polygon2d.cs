// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Polygon2d.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Polygon2d.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_Polygon2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The polygon information needed to compute interferences (<see cref="Intf_InterferencePolygon2d"/>).</summary>
public abstract class Intf_Polygon2d
{
    protected Bnd_Box2d myBox = new();

    /// <summary>The bounding box of the polygon.</summary>
    public Bnd_Box2d Bounding() => myBox;

    /// <summary>True if the polyline is closed.</summary>
    public virtual bool Closed() => false;

    /// <summary>The tolerance of the polygon.</summary>
    public abstract double DeflectionOverEstimation();

    /// <summary>The number of segments in the polyline.</summary>
    public abstract int NbSegments();

    /// <summary>The points of the segment <paramref name="theIndex"/> of the polygon.</summary>
    public abstract void Segment(int theIndex, out gp_Pnt2d theBegin, out gp_Pnt2d theEnd);
}
