// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Point.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>The common behaviour of points in the plane (<see cref="Geom2d_CartesianPoint"/>).</summary>
public abstract class Geom2d_Point : Geom2d_Geometry
{
    /// <summary>The coordinates of the point.</summary>
    public abstract void Coord(out double X, out double Y);

    /// <summary>The point as a <see cref="Point2D"/>.</summary>
    public abstract Point2D Pnt2d();

    /// <summary>The X coordinate.</summary>
    public abstract double X();

    /// <summary>The Y coordinate.</summary>
    public abstract double Y();

    /// <summary>The distance to Other.</summary>
    public double Distance(Geom2d_Point Other)
    {
        Point2D P1 = this.Pnt2d();
        Point2D P2 = Other.Pnt2d();
        return P1.Distance(P2);
    }

    /// <summary>The square of the distance to Other.</summary>
    public double SquareDistance(Geom2d_Point Other)
    {
        Point2D P1 = this.Pnt2d();
        Point2D P2 = Other.Pnt2d();
        return P1.SquareDistance(P2);
    }
}
