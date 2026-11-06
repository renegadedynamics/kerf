// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevolution.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Primitives;
using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds revolved shapes: a meridian curve in the XZ plane of the axes, between VMin and VMax (its bounds by default),
/// rotated around the Z axis by angle (2π by default). The result has a lateral face of revolution, planar faces at
/// the ends of the meridian when they aren't on the axis, and for a portion two planar faces at u = 0 and u = angle.
/// </summary>
internal class MakeRevolution : MakeOneAxis
{
    private readonly Revolution _revolution;

    private static Curve2D Project(Curve3D m, in CoordinateSystem axis)
    {
        Curve2D? c;
        c = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(m, new PlaneSurface(axis));
        // OCCT passes a null projection on to BRepPrim_Revolution, which dereferences it when it is used
        return c!;
    }

    private static Curve2D Project(Curve3D m)
    {
        return Project(m, new CoordinateSystem(new Frame3D(Constants.Origin(), Constants.DY().Reversed(), Constants.DX())));
    }

    /// <summary>A revolution body by rotating a curve around Z.</summary>
    public MakeRevolution(Curve3D meridian)
    {
        _revolution = new Revolution(Constants.XOY(), meridian.FirstParameter, meridian.LastParameter, meridian, Project(meridian));
    }

    /// <summary>A revolution body by rotating a curve around Z, by angle.</summary>
    public MakeRevolution(Curve3D meridian, double angle)
    {
        _revolution = new Revolution(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                               meridian.FirstParameter,
                                               meridian.LastParameter,
                                               meridian,
                                               Project(meridian));
        _revolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around Z.</summary>
    public MakeRevolution(Curve3D meridian, double vMin, double vMax)
    {
        _revolution = new Revolution(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                               vMin,
                                               vMax,
                                               meridian,
                                               Project(meridian));
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around Z, by angle.</summary>
    public MakeRevolution(Curve3D meridian, double vMin, double vMax, double angle)
    {
        _revolution = new Revolution(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                               vMin,
                                               vMax,
                                               meridian,
                                               Project(meridian));
        _revolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve around the Z axis of the axes.</summary>
    public MakeRevolution(in Frame3D axes, Curve3D meridian)
    {
        _revolution = new Revolution(axes, meridian.FirstParameter, meridian.LastParameter, meridian, Project(meridian));
    }

    /// <summary>A revolution body by rotating a curve around the Z axis of the axes, by angle.</summary>
    public MakeRevolution(in Frame3D axes, Curve3D meridian, double angle)
    {
        _revolution = new Revolution(axes, meridian.FirstParameter, meridian.LastParameter, meridian, Project(meridian));
        _revolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around the Z axis of the axes.</summary>
    public MakeRevolution(in Frame3D axes, Curve3D meridian, double vMin, double vMax)
    {
        _revolution = new Revolution(axes, vMin, vMax, meridian, Project(meridian));
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around the Z axis of the axes, by angle.</summary>
    public MakeRevolution(in Frame3D axes, Curve3D meridian, double vMin, double vMax, double angle)
    {
        _revolution = new Revolution(axes, vMin, vMax, meridian, Project(meridian));
        _revolution.Angle(angle);
    }

    internal override OneAxis OneAxis()
    {
        return _revolution;
    }

    /// <summary>The algorithm.</summary>
    internal Revolution Revolution()
    {
        return _revolution;
    }
}
