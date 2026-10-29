// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Torus.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The torus primitive.</summary>
internal sealed class BRepPrim_Torus : BRepPrim_Revolution
{
    private readonly double myMajor; // distance from the center of the pipe to the center of the torus
    private readonly double myMinor; // radius of the pipe

    /// <summary>The STEP definition: center and axes, major and minor radii.</summary>
    public BRepPrim_Torus(in gp_Ax2 Position, double Major, double Minor)
        : base(Position, 0, 2 * M_PI)
    {
        myMajor = Major;
        myMinor = Minor;
        SetMeridian();
    }

    /// <summary>A torus centered at the origin.</summary>
    public BRepPrim_Torus(double Major, double Minor)
        : base(gp.XOY(), 0, 2 * M_PI)
    {
        myMajor = Major;
        myMinor = Minor;
        SetMeridian();
    }

    /// <summary>A torus at <paramref name="Center"/>.</summary>
    public BRepPrim_Torus(in gp_Pnt Center, double Major, double Minor)
        : base(new gp_Ax2(Center, new gp_Dir(gp_Dir.D.Z), new gp_Dir(gp_Dir.D.X)), 0, 2 * M_PI)
    {
        myMajor = Major;
        myMinor = Minor;
        SetMeridian();
    }

    /// <summary>The surface normal should be directed towards the outside.</summary>
    public override TopoDS_Face MakeEmptyLateralFace()
    {
        Geom_ToroidalSurface T = new(new gp_Ax3(Axes()), myMajor, myMinor);
        TopoDS_Face F = new();
        myBuilder.Builder().MakeFace(ref F, T, Precision.Confusion());
        return F;
    }

    private void SetMeridian()
    {
        gp_Dir D = Axes().YDirection();
        D.Reverse();
        gp_Ax2 A = new(Axes().Location(), D, Axes().XDirection());
        gp_Vec V = new(Axes().XDirection());
        V.Multiply(myMajor);
        A.Translate(V);
        Geom_Circle C = new(A, myMinor);
        Geom2d_Circle C2d = new(new gp_Ax2d(new gp_Pnt2d(myMajor, 0), new gp_Dir2d(gp_Dir2d.D.X)), myMinor);
        Meridian(C, C2d);
    }
}
