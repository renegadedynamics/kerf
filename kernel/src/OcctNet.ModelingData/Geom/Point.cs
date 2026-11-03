// Port of: src/ModelingData/TKG3d/Geom/Geom_Point.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Objects;

/// <summary>The common behaviour of points in 3D space (<see cref="CartesianPoint"/>).</summary>
public abstract class Point : Geometry3D
{
    /// <summary>The coordinates of the point.</summary>
    public abstract void Coord(out double x, out double y, out double z);

    /// <summary>The point as a <see cref="Point3D"/>.</summary>
    public abstract Point3D Pnt();

    /// <summary>The X coordinate.</summary>
    public abstract double X();

    /// <summary>The Y coordinate.</summary>
    public abstract double Y();

    /// <summary>The Z coordinate.</summary>
    public abstract double Z();

    /// <summary>The distance to Other.</summary>
    public double Distance(Point other)
    {
        Point3D p1 = this.Pnt();
        Point3D p2 = other.Pnt();
        return p1.Distance(p2);
    }

    /// <summary>The square of the distance to Other.</summary>
    public double SquareDistance(Point other)
    {
        Point3D p1 = this.Pnt();
        Point3D p2 = other.Pnt();
        return p1.SquareDistance(p2);
    }
}
