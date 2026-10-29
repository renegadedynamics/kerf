// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Sphere.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The sphere primitive.</summary>
internal sealed class BRepPrim_Sphere : BRepPrim_Revolution
{
    private const double PMIN = -0.5 * M_PI;
    private const double PMAX = 0.5 * M_PI;

    private readonly double myRadius;

    /// <summary>A sphere at the origin with the reference axes.</summary>
    public BRepPrim_Sphere(double Radius)
        : base(gp.XOY(), PMIN, PMAX)
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>A sphere with the center and radius, on the reference axes (the STEP constructor).</summary>
    public BRepPrim_Sphere(in gp_Pnt Center, double Radius)
        : base(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), PMIN, PMAX)
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>A sphere on the given axes.</summary>
    public BRepPrim_Sphere(in gp_Ax2 Axes, double Radius)
        : base(Axes, PMIN, PMAX)
    {
        myRadius = Radius;
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override TopoDS_Face MakeEmptyLateralFace()
    {
        Geom_SphericalSurface S = new(new gp_Ax3(Axes()), myRadius);
        TopoDS_Face F = new();
        myBuilder.Builder().MakeFace(ref F, S, Precision.Confusion());
        return F;
    }

    private void SetMeridian()
    {
        // Offset for the parameters on the meridian, which is a circle
        SetMeridianOffset(2 * M_PI);

        gp_Dir D = Axes().YDirection();
        D.Reverse();
        gp_Ax2 A = new(Axes().Location(), D, Axes().XDirection());
        Geom_Circle C = new(A, myRadius);
        Geom2d_Circle C2d = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(gp_Dir2d.D.X)), myRadius);
        Meridian(C, C2d);
    }
}
