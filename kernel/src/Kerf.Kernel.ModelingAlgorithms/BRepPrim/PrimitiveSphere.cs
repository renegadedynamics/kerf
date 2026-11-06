// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Sphere.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>The sphere primitive.</summary>
internal sealed class PrimitiveSphere : Revolution
{
    private const double Pmin = -0.5 * M_PI;
    private const double Pmax = 0.5 * M_PI;

    private readonly double _radius;

    /// <summary>A sphere at the origin with the reference axes.</summary>
    public PrimitiveSphere(double radius)
        : base(Constants.XOY(), Pmin, Pmax)
    {
        _radius = radius;
        SetMeridian();
    }

    /// <summary>A sphere with the center and radius, on the reference axes (the STEP constructor).</summary>
    public PrimitiveSphere(in Point3D center, double radius)
        : base(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), Pmin, Pmax)
    {
        _radius = radius;
        SetMeridian();
    }

    /// <summary>A sphere on the given axes.</summary>
    public PrimitiveSphere(in Frame3D axes, double radius)
        : base(axes, Pmin, Pmax)
    {
        _radius = radius;
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override Face MakeEmptyLateralFace()
    {
        SphericalSurface s = new(new CoordinateSystem(Axes()), _radius);
        Face f = new();
        _builder.Builder().MakeFace(ref f, s, global::Kerf.Engine.Foundation.Precision.Confusion());
        return f;
    }

    private void SetMeridian()
    {
        // Offset for the parameters on the meridian, which is a circle
        SetMeridianOffset(2 * M_PI);

        Direction3D d = Axes().YDirection;
        d = d.Reversed();
        Frame3D a = new(Axes().Location, d, Axes().XDirection);
        CircleCurve3D c = new(a, _radius);
        CircleCurve2D c2d = new(new Axis2D(new Point2D(0, 0), new Direction2D(Direction2D.D.X)), _radius);
        Meridian(c, c2d);
    }
}
