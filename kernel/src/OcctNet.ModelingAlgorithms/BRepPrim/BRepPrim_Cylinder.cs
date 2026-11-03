// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cylinder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>The cylinder primitive.</summary>
internal sealed class BRepPrim_Cylinder : BRepPrim_Revolution
{
    private readonly double myRadius; // cylinder radius

    /// <summary>The STEP definition: the center of a face and the axis, the radius, and the height on the positive side.</summary>
    public BRepPrim_Cylinder(in Frame3D Position, double Radius, double Height)
        : base(Position, 0, Height)
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder at the origin.</summary>
    public BRepPrim_Cylinder(double Radius)
        : base(Constants.XOY(), RealFirst(), RealLast())
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder at <paramref name="Center"/>.</summary>
    public BRepPrim_Cylinder(in Point3D Center, double Radius)
        : base(new Frame3D(Center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), RealFirst(), RealLast())
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder on <paramref name="Axes"/>.</summary>
    public BRepPrim_Cylinder(in Frame3D Axes, double Radius)
        : base(Axes, RealFirst(), RealLast())
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>A cylinder at the origin on the Z axis, of height <paramref name="H"/> and radius <paramref name="R"/>.</summary>
    public BRepPrim_Cylinder(double R, double H)
        : base(Constants.XOY(), 0, H)
    {
        myRadius = R;
        SetMeridian();
    }

    /// <summary>As above, at a given point.</summary>
    public BRepPrim_Cylinder(in Point3D Center, double R, double H)
        : base(new Frame3D(Center, new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X)), 0, H)
    {
        myRadius = R;
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override TopoDS_Face MakeEmptyLateralFace()
    {
        Geom_CylindricalSurface C = new(new CoordinateSystem(Axes()), myRadius);
        TopoDS_Face F = new();
        myBuilder.Builder().MakeFace(ref F, C, global::Kerf.Engine.Foundation.Precision.Confusion());
        return F;
    }

    private void SetMeridian()
    {
        Vector3D V = new(Axes().XDirection());
        V.Multiply(myRadius);
        Axis3D A = Axes().Axis();
        A.Translate(V);
        Geom_Line L = new(A);
        Geom2d_Line L2d = new(new Point2D(myRadius, 0), new Direction2D(Direction2D.D.Y));
        Meridian(L, L2d);
    }
}
