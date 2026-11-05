// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeSphere.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakeSphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Primitives;
using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Api;

/// <summary>
/// Builds spheres or portions of spheres: a lateral spherical face, planar faces parallel to z = 0 where the sphere is
/// cut in V (angle1, angle2 in -π/2..π/2), and for a wedge (angle3 or angle in U) two planar faces at u = 0 and
/// u = angle.
/// </summary>
internal class MakeSphere : MakeOneAxis
{
    private readonly PrimitiveSphere _sphere;

    /// <summary>A sphere of radius R.</summary>
    public MakeSphere(double r)
    {
        _sphere = new PrimitiveSphere(Constants.XOY(), r);
    }

    /// <summary>A spherical wedge (a negative angle turns the axis over).</summary>
    public MakeSphere(double r, double angle)
    {
        _sphere = new PrimitiveSphere(new Frame3D(Constants.Origin(), new Direction3D((angle < 0.0 ? -1 : 1) * new Vector3D(Constants.DZ())), Constants.DX()), r);
        _sphere.Angle(Math.Abs(angle));
    }

    /// <summary>A spherical segment between angle1 and angle2.</summary>
    public MakeSphere(double r, double angle1, double angle2)
    {
        _sphere = new PrimitiveSphere(r);
        _sphere.VMin(angle1);
        _sphere.VMax(angle2);
    }

    /// <summary>A spherical segment between angle1 and angle2, of angle3 in U.</summary>
    public MakeSphere(double r, double angle1, double angle2, double angle3)
    {
        _sphere = new PrimitiveSphere(r);
        _sphere.VMin(angle1);
        _sphere.VMax(angle2);
        _sphere.Angle(angle3);
    }

    /// <summary>A sphere at Center.</summary>
    public MakeSphere(in Point3D center, double r)
    {
        _sphere = new PrimitiveSphere(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), r);
    }

    /// <summary>A spherical wedge at Center.</summary>
    public MakeSphere(in Point3D center, double r, double angle)
    {
        _sphere = new PrimitiveSphere(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), r);
        _sphere.Angle(angle);
    }

    /// <summary>A spherical segment at Center.</summary>
    public MakeSphere(in Point3D center, double r, double angle1, double angle2)
    {
        _sphere = new PrimitiveSphere(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), r);
        _sphere.VMin(angle1);
        _sphere.VMax(angle2);
    }

    /// <summary>A spherical segment of angle3 in U at Center.</summary>
    public MakeSphere(in Point3D center, double r, double angle1, double angle2, double angle3)
    {
        _sphere = new PrimitiveSphere(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), r);
        _sphere.VMin(angle1);
        _sphere.VMax(angle2);
        _sphere.Angle(angle3);
    }

    /// <summary>A sphere on the axes.</summary>
    public MakeSphere(in Frame3D axis, double r)
    {
        _sphere = new PrimitiveSphere(axis, r);
    }

    /// <summary>A spherical wedge on the axes.</summary>
    public MakeSphere(in Frame3D axis, double r, double angle)
    {
        _sphere = new PrimitiveSphere(axis, r);
        _sphere.Angle(angle);
    }

    /// <summary>A spherical segment on the axes.</summary>
    public MakeSphere(in Frame3D axis, double r, double angle1, double angle2)
    {
        _sphere = new PrimitiveSphere(axis, r);
        _sphere.VMin(angle1);
        _sphere.VMax(angle2);
    }

    /// <summary>A spherical segment of angle3 in U on the axes.</summary>
    public MakeSphere(in Frame3D axis, double r, double angle1, double angle2, double angle3)
    {
        _sphere = new PrimitiveSphere(axis, r);
        _sphere.VMin(angle1);
        _sphere.VMax(angle2);
        _sphere.Angle(angle3);
    }

    internal override OneAxis OneAxis()
    {
        return _sphere;
    }

    /// <summary>The algorithm.</summary>
    internal PrimitiveSphere Sphere()
    {
        return _sphere;
    }
}
