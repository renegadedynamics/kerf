// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cylinder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The cylinder primitive.</summary>
internal sealed class BRepPrim_Cylinder : BRepPrim_Revolution
{
    private readonly double myRadius; // cylinder radius

    /// <summary>The STEP definition: the center of a face and the axis, the radius, and the height on the positive side.</summary>
    public BRepPrim_Cylinder(in gp_Ax2 Position, double Radius, double Height)
        : base(Position, 0, Height)
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder at the origin.</summary>
    public BRepPrim_Cylinder(double Radius)
        : base(gp.XOY(), RealFirst(), RealLast())
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder at <paramref name="Center"/>.</summary>
    public BRepPrim_Cylinder(in gp_Pnt Center, double Radius)
        : base(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), RealFirst(), RealLast())
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>An infinite cylinder on <paramref name="Axes"/>.</summary>
    public BRepPrim_Cylinder(in gp_Ax2 Axes, double Radius)
        : base(Axes, RealFirst(), RealLast())
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>A cylinder at the origin on the Z axis, of height <paramref name="H"/> and radius <paramref name="R"/>.</summary>
    public BRepPrim_Cylinder(double R, double H)
        : base(gp.XOY(), 0, H)
    {
        myRadius = R;
        SetMeridian();
    }

    /// <summary>As above, at a given point.</summary>
    public BRepPrim_Cylinder(in gp_Pnt Center, double R, double H)
        : base(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), 0, H)
    {
        myRadius = R;
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override TopoDS_Face MakeEmptyLateralFace()
    {
        Geom_CylindricalSurface C = new(new gp_Ax3(Axes()), myRadius);
        TopoDS_Face F = new();
        myBuilder.Builder().MakeFace(ref F, C, Precision.Confusion());
        return F;
    }

    private void SetMeridian()
    {
        gp_Vec V = new(Axes().XDirection());
        V.Multiply(myRadius);
        gp_Ax1 A = Axes().Axis();
        A.Translate(V);
        Geom_Line L = new(A);
        Geom2d_Line L2d = new(new gp_Pnt2d(myRadius, 0), new gp_Dir2d(gp_Dir2d.D.Y));
        Meridian(L, L2d);
    }
}
