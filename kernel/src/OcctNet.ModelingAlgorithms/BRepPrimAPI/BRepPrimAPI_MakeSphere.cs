// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeSphere.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeSphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Builds spheres or portions of spheres: a lateral spherical face, planar faces parallel to z = 0 where the sphere is
/// cut in V (angle1, angle2 in -π/2..π/2), and for a wedge (angle3 or angle in U) two planar faces at u = 0 and
/// u = angle.
/// </summary>
public class BRepPrimAPI_MakeSphere : BRepPrimAPI_MakeOneAxis
{
    private readonly BRepPrim_Sphere mySphere;

    /// <summary>A sphere of radius R.</summary>
    public BRepPrimAPI_MakeSphere(double R)
    {
        mySphere = new BRepPrim_Sphere(gp.XOY(), R);
    }

    /// <summary>A spherical wedge (a negative angle turns the axis over).</summary>
    public BRepPrimAPI_MakeSphere(double R, double angle)
    {
        mySphere = new BRepPrim_Sphere(new gp_Ax2(gp.Origin(), new gp_Dir((angle < 0.0 ? -1 : 1) * new gp_Vec(gp.DZ())), gp.DX()), R);
        mySphere.Angle(Math.Abs(angle));
    }

    /// <summary>A spherical segment between angle1 and angle2.</summary>
    public BRepPrimAPI_MakeSphere(double R, double angle1, double angle2)
    {
        mySphere = new BRepPrim_Sphere(R);
        mySphere.VMin(angle1);
        mySphere.VMax(angle2);
    }

    /// <summary>A spherical segment between angle1 and angle2, of angle3 in U.</summary>
    public BRepPrimAPI_MakeSphere(double R, double angle1, double angle2, double angle3)
    {
        mySphere = new BRepPrim_Sphere(R);
        mySphere.VMin(angle1);
        mySphere.VMax(angle2);
        mySphere.Angle(angle3);
    }

    /// <summary>A sphere at Center.</summary>
    public BRepPrimAPI_MakeSphere(in gp_Pnt Center, double R)
    {
        mySphere = new BRepPrim_Sphere(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), R);
    }

    /// <summary>A spherical wedge at Center.</summary>
    public BRepPrimAPI_MakeSphere(in gp_Pnt Center, double R, double angle)
    {
        mySphere = new BRepPrim_Sphere(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), R);
        mySphere.Angle(angle);
    }

    /// <summary>A spherical segment at Center.</summary>
    public BRepPrimAPI_MakeSphere(in gp_Pnt Center, double R, double angle1, double angle2)
    {
        mySphere = new BRepPrim_Sphere(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), R);
        mySphere.VMin(angle1);
        mySphere.VMax(angle2);
    }

    /// <summary>A spherical segment of angle3 in U at Center.</summary>
    public BRepPrimAPI_MakeSphere(in gp_Pnt Center, double R, double angle1, double angle2, double angle3)
    {
        mySphere = new BRepPrim_Sphere(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), R);
        mySphere.VMin(angle1);
        mySphere.VMax(angle2);
        mySphere.Angle(angle3);
    }

    /// <summary>A sphere on the axes.</summary>
    public BRepPrimAPI_MakeSphere(in gp_Ax2 Axis, double R)
    {
        mySphere = new BRepPrim_Sphere(Axis, R);
    }

    /// <summary>A spherical wedge on the axes.</summary>
    public BRepPrimAPI_MakeSphere(in gp_Ax2 Axis, double R, double angle)
    {
        mySphere = new BRepPrim_Sphere(Axis, R);
        mySphere.Angle(angle);
    }

    /// <summary>A spherical segment on the axes.</summary>
    public BRepPrimAPI_MakeSphere(in gp_Ax2 Axis, double R, double angle1, double angle2)
    {
        mySphere = new BRepPrim_Sphere(Axis, R);
        mySphere.VMin(angle1);
        mySphere.VMax(angle2);
    }

    /// <summary>A spherical segment of angle3 in U on the axes.</summary>
    public BRepPrimAPI_MakeSphere(in gp_Ax2 Axis, double R, double angle1, double angle2, double angle3)
    {
        mySphere = new BRepPrim_Sphere(Axis, R);
        mySphere.VMin(angle1);
        mySphere.VMax(angle2);
        mySphere.Angle(angle3);
    }

    internal override BRepPrim_OneAxis OneAxis()
    {
        return mySphere;
    }

    /// <summary>The algorithm.</summary>
    internal BRepPrim_Sphere Sphere()
    {
        return mySphere;
    }
}
