// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Torus.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>The torus primitive.</summary>
internal sealed class PrimitiveTorus : Revolution
{
    private readonly double _major; // distance from the center of the pipe to the center of the torus
    private readonly double _minor; // radius of the pipe

    /// <summary>The STEP definition: center and axes, major and minor radii.</summary>
    public PrimitiveTorus(in Frame3D position, double major, double minor)
        : base(position, 0, 2 * M_PI)
    {
        _major = major;
        _minor = minor;
        SetMeridian();
    }

    /// <summary>A torus centered at the origin.</summary>
    public PrimitiveTorus(double major, double minor)
        : base(Constants.XOY(), 0, 2 * M_PI)
    {
        _major = major;
        _minor = minor;
        SetMeridian();
    }

    /// <summary>A torus at <paramref name="center"/>.</summary>
    public PrimitiveTorus(in Point3D center, double major, double minor)
        : base(new Frame3D(center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), 0, 2 * M_PI)
    {
        _major = major;
        _minor = minor;
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override Face MakeEmptyLateralFace()
    {
        ToroidalSurface t = new(new CoordinateSystem(Axes()), _major, _minor);
        Face f = new();
        _builder.Builder().MakeFace(ref f, t, global::Kerf.Engine.Foundation.Precision.Confusion());
        return f;
    }

    private void SetMeridian()
    {
        Direction3D d = Axes().YDirection;
        d = d.Reversed();
        Frame3D a = new(Axes().Location, d, Axes().XDirection);
        Vector3D v = new(Axes().XDirection);
        v = v.Multiplied(_major);
        a = a.Translated(v);
        CircleCurve3D c = new(a, _minor);
        CircleCurve2D c2d = new(new Axis2D(new Point2D(_major, 0), new Direction2D(Direction2D.D.X)), _minor);
        Meridian(c, c2d);
    }
}
