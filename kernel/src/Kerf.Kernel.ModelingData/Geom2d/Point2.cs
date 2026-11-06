// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Point.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>The common behaviour of points in the plane (<see cref="CartesianPoint2"/>).</summary>
internal abstract class Point2 : Geometry2D
{
    /// <summary>The coordinates of the point.</summary>
    public abstract void Coord(out double x, out double y);

    /// <summary>The point as a <see cref="Point2D"/>.</summary>
    public abstract Point2D Pnt2d();

    /// <summary>The X coordinate.</summary>
    public abstract double X();

    /// <summary>The Y coordinate.</summary>
    public abstract double Y();

    /// <summary>The distance to Other.</summary>
    public double Distance(Point2 other)
    {
        Point2D p1 = this.Pnt2d();
        Point2D p2 = other.Pnt2d();
        return p1.DistanceTo(p2);
    }

    /// <summary>The square of the distance to Other.</summary>
    public double SquareDistance(Point2 other)
    {
        Point2D p1 = this.Pnt2d();
        Point2D p2 = other.Pnt2d();
        return p1.SquareDistanceTo(p2);
    }
}
