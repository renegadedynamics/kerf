// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevolution.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Builds revolved shapes: a meridian curve in the XZ plane of the axes, between VMin and VMax (its bounds by default),
/// rotated around the Z axis by angle (2π by default). The result has a lateral face of revolution, planar faces at
/// the ends of the meridian when they aren't on the axis, and for a portion two planar faces at u = 0 and u = angle.
/// </summary>
public class BRepPrimAPI_MakeRevolution : BRepPrimAPI_MakeOneAxis
{
    private readonly BRepPrim_Revolution myRevolution;

    private static Curve2D Project(Curve3D M, in CoordinateSystem Axis)
    {
        Curve2D? C;
        C = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(M, new PlaneSurface(Axis));
        // OCCT passes a null projection on to BRepPrim_Revolution, which dereferences it when it is used
        return C!;
    }

    private static Curve2D Project(Curve3D M)
    {
        return Project(M, new CoordinateSystem(new Frame3D(Constants.Origin(), Constants.DY().Reversed(), Constants.DX())));
    }

    /// <summary>A revolution body by rotating a curve around Z.</summary>
    public BRepPrimAPI_MakeRevolution(Curve3D Meridian)
    {
        myRevolution = new BRepPrim_Revolution(Constants.XOY(), Meridian.FirstParameter(), Meridian.LastParameter(), Meridian, Project(Meridian));
    }

    /// <summary>A revolution body by rotating a curve around Z, by angle.</summary>
    public BRepPrimAPI_MakeRevolution(Curve3D Meridian, double angle)
    {
        myRevolution = new BRepPrim_Revolution(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                               Meridian.FirstParameter(),
                                               Meridian.LastParameter(),
                                               Meridian,
                                               Project(Meridian));
        myRevolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around Z.</summary>
    public BRepPrimAPI_MakeRevolution(Curve3D Meridian, double VMin, double VMax)
    {
        myRevolution = new BRepPrim_Revolution(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                               VMin,
                                               VMax,
                                               Meridian,
                                               Project(Meridian));
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around Z, by angle.</summary>
    public BRepPrimAPI_MakeRevolution(Curve3D Meridian, double VMin, double VMax, double angle)
    {
        myRevolution = new BRepPrim_Revolution(new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)),
                                               VMin,
                                               VMax,
                                               Meridian,
                                               Project(Meridian));
        myRevolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve around the Z axis of the axes.</summary>
    public BRepPrimAPI_MakeRevolution(in Frame3D Axes, Curve3D Meridian)
    {
        myRevolution = new BRepPrim_Revolution(Axes, Meridian.FirstParameter(), Meridian.LastParameter(), Meridian, Project(Meridian));
    }

    /// <summary>A revolution body by rotating a curve around the Z axis of the axes, by angle.</summary>
    public BRepPrimAPI_MakeRevolution(in Frame3D Axes, Curve3D Meridian, double angle)
    {
        myRevolution = new BRepPrim_Revolution(Axes, Meridian.FirstParameter(), Meridian.LastParameter(), Meridian, Project(Meridian));
        myRevolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around the Z axis of the axes.</summary>
    public BRepPrimAPI_MakeRevolution(in Frame3D Axes, Curve3D Meridian, double VMin, double VMax)
    {
        myRevolution = new BRepPrim_Revolution(Axes, VMin, VMax, Meridian, Project(Meridian));
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around the Z axis of the axes, by angle.</summary>
    public BRepPrimAPI_MakeRevolution(in Frame3D Axes, Curve3D Meridian, double VMin, double VMax, double angle)
    {
        myRevolution = new BRepPrim_Revolution(Axes, VMin, VMax, Meridian, Project(Meridian));
        myRevolution.Angle(angle);
    }

    internal override BRepPrim_OneAxis OneAxis()
    {
        return myRevolution;
    }

    /// <summary>The algorithm.</summary>
    internal BRepPrim_Revolution Revolution()
    {
        return myRevolution;
    }
}
