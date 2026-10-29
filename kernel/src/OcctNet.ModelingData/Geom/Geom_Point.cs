// Port of: src/ModelingData/TKG3d/Geom/Geom_Point.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Point.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The common behaviour of points in 3D space (<see cref="Geom_CartesianPoint"/>).</summary>
public abstract class Geom_Point : Geom_Geometry
{
    /// <summary>The coordinates of the point.</summary>
    public abstract void Coord(out double X, out double Y, out double Z);

    /// <summary>The point as a <see cref="gp_Pnt"/>.</summary>
    public abstract gp_Pnt Pnt();

    /// <summary>The X coordinate.</summary>
    public abstract double X();

    /// <summary>The Y coordinate.</summary>
    public abstract double Y();

    /// <summary>The Z coordinate.</summary>
    public abstract double Z();

    /// <summary>The distance to Other.</summary>
    public double Distance(Geom_Point Other)
    {
        gp_Pnt P1 = this.Pnt();
        gp_Pnt P2 = Other.Pnt();
        return P1.Distance(P2);
    }

    /// <summary>The square of the distance to Other.</summary>
    public double SquareDistance(Geom_Point Other)
    {
        gp_Pnt P1 = this.Pnt();
        gp_Pnt P2 = Other.Pnt();
        return P1.SquareDistance(P2);
    }
}
