// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevolution.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Builds revolved shapes: a meridian curve in the XZ plane of the axes, between VMin and VMax (its bounds by default),
/// rotated around the Z axis by angle (2π by default). The result has a lateral face of revolution, planar faces at
/// the ends of the meridian when they aren't on the axis, and for a portion two planar faces at u = 0 and u = angle.
/// </summary>
public class BRepPrimAPI_MakeRevolution : BRepPrimAPI_MakeOneAxis
{
    private readonly BRepPrim_Revolution myRevolution;

    private static Geom2d_Curve Project(Geom_Curve M, in gp_Ax3 Axis)
    {
        Geom2d_Curve? C;
        C = GeomProjLib.Curve2d(M, new Geom_Plane(Axis));
        // OCCT passes a null projection on to BRepPrim_Revolution, which dereferences it when it is used
        return C!;
    }

    private static Geom2d_Curve Project(Geom_Curve M)
    {
        return Project(M, new gp_Ax3(new gp_Ax2(gp.Origin(), gp.DY().Reversed(), gp.DX())));
    }

    /// <summary>A revolution body by rotating a curve around Z.</summary>
    public BRepPrimAPI_MakeRevolution(Geom_Curve Meridian)
    {
        myRevolution = new BRepPrim_Revolution(gp.XOY(), Meridian.FirstParameter(), Meridian.LastParameter(), Meridian, Project(Meridian));
    }

    /// <summary>A revolution body by rotating a curve around Z, by angle.</summary>
    public BRepPrimAPI_MakeRevolution(Geom_Curve Meridian, double angle)
    {
        myRevolution = new BRepPrim_Revolution(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)),
                                               Meridian.FirstParameter(),
                                               Meridian.LastParameter(),
                                               Meridian,
                                               Project(Meridian));
        myRevolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around Z.</summary>
    public BRepPrimAPI_MakeRevolution(Geom_Curve Meridian, double VMin, double VMax)
    {
        myRevolution = new BRepPrim_Revolution(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)),
                                               VMin,
                                               VMax,
                                               Meridian,
                                               Project(Meridian));
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around Z, by angle.</summary>
    public BRepPrimAPI_MakeRevolution(Geom_Curve Meridian, double VMin, double VMax, double angle)
    {
        myRevolution = new BRepPrim_Revolution(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)),
                                               VMin,
                                               VMax,
                                               Meridian,
                                               Project(Meridian));
        myRevolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve around the Z axis of the axes.</summary>
    public BRepPrimAPI_MakeRevolution(in gp_Ax2 Axes, Geom_Curve Meridian)
    {
        myRevolution = new BRepPrim_Revolution(Axes, Meridian.FirstParameter(), Meridian.LastParameter(), Meridian, Project(Meridian));
    }

    /// <summary>A revolution body by rotating a curve around the Z axis of the axes, by angle.</summary>
    public BRepPrimAPI_MakeRevolution(in gp_Ax2 Axes, Geom_Curve Meridian, double angle)
    {
        myRevolution = new BRepPrim_Revolution(Axes, Meridian.FirstParameter(), Meridian.LastParameter(), Meridian, Project(Meridian));
        myRevolution.Angle(angle);
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around the Z axis of the axes.</summary>
    public BRepPrimAPI_MakeRevolution(in gp_Ax2 Axes, Geom_Curve Meridian, double VMin, double VMax)
    {
        myRevolution = new BRepPrim_Revolution(Axes, VMin, VMax, Meridian, Project(Meridian));
    }

    /// <summary>A revolution body by rotating a curve between VMin and VMax around the Z axis of the axes, by angle.</summary>
    public BRepPrimAPI_MakeRevolution(in gp_Ax2 Axes, Geom_Curve Meridian, double VMin, double VMax, double angle)
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
