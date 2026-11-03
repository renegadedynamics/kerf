// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cylinder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>The cylinder primitive.</summary>
internal sealed class PrimitiveCylinder : Revolution
{
    private readonly double _radius; // cylinder radius

    /// <summary>The STEP definition: the center of a face and the axis, the radius, and the height on the positive side.</summary>
    public PrimitiveCylinder(in Frame3D position, double radius, double height)
        : base(position, 0, height)
    {
        _radius = radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder at the origin.</summary>
    public PrimitiveCylinder(double radius)
        : base(Constants.XOY(), RealFirst(), RealLast())
    {
        _radius = radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder at <paramref name="center"/>.</summary>
    public PrimitiveCylinder(in Point3D center, double radius)
        : base(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), RealFirst(), RealLast())
    {
        _radius = radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder on <paramref name="axes"/>.</summary>
    public PrimitiveCylinder(in Frame3D axes, double radius)
        : base(axes, RealFirst(), RealLast())
    {
        _radius = radius;
        SetMeridian();
    }

    /// <summary>A cylinder at the origin on the Z axis, of height <paramref name="h"/> and radius <paramref name="r"/>.</summary>
    public PrimitiveCylinder(double r, double h)
        : base(Constants.XOY(), 0, h)
    {
        _radius = r;
        SetMeridian();
    }

    /// <summary>As above, at a given point.</summary>
    public PrimitiveCylinder(in Point3D center, double r, double h)
        : base(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), 0, h)
    {
        _radius = r;
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override Face MakeEmptyLateralFace()
    {
        CylindricalSurface c = new(new CoordinateSystem(Axes()), _radius);
        Face f = new();
        _builder.Builder().MakeFace(ref f, c, global::Kerf.Engine.Foundation.Precision.Confusion());
        return f;
    }

    private void SetMeridian()
    {
        Vector3D v = new(Axes().XDirection());
        v.Multiply(_radius);
        Axis3D a = Axes().Axis();
        a.Translate(v);
        LineCurve3D l = new(a);
        LineCurve2D l2d = new(new Point2D(_radius, 0), new Direction2D(Direction2D.D.Y));
        Meridian(l, l2d);
    }
}
