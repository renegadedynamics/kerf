// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>Geom_Axis1Placement and Geom_Axis2Placement against gp_Ax1 and gp_Ax2.</summary>
public class Geom_AxisPlacementTests
{
    private const double Tol = 1e-12;

    private static void AssertDir(in gp_Dir theExpected, in gp_Dir theActual)
    {
        Assert.Equal(theExpected.X(), theActual.X(), Tol);
        Assert.Equal(theExpected.Y(), theActual.Y(), Tol);
        Assert.Equal(theExpected.Z(), theActual.Z(), Tol);
    }

    private static void AssertPnt(in gp_Pnt theExpected, in gp_Pnt theActual)
    {
        Assert.Equal(theExpected.X(), theActual.X(), Tol);
        Assert.Equal(theExpected.Y(), theActual.Y(), Tol);
        Assert.Equal(theExpected.Z(), theActual.Z(), Tol);
    }

    private static void AssertAx2(in gp_Ax2 theExpected, Geom_Axis2Placement theActual)
    {
        AssertPnt(theExpected.Location(), theActual.Location());
        AssertDir(theExpected.Direction(), theActual.Direction());
        AssertDir(theExpected.XDirection(), theActual.XDirection());
        AssertDir(theExpected.YDirection(), theActual.YDirection());
        gp_Ax2 A = theActual.Ax2();
        AssertDir(theExpected.XDirection(), A.XDirection());
    }

    [Fact]
    public void Axis1Placement_matches_gp_Ax1()
    {
        gp_Ax1 A = new(new gp_Pnt(1, 2, 3), new gp_Dir(1, 2, 2));
        Geom_Axis1Placement G = new(A.Location(), A.Direction());
        Assert.Equal(A, G.Ax1());
        Assert.Equal(A, G.Axis());

        gp_Trsf T = gp_Trsf.Rotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 0.5);
        Geom_Axis1Placement GT = (Geom_Axis1Placement)G.Transformed(T);
        gp_Ax1 AT = A.Transformed(T);
        AssertPnt(AT.Location(), GT.Location());
        AssertDir(AT.Direction(), GT.Direction());

        AssertDir(A.Reversed().Direction(), G.Reversed().Direction());
        Assert.Equal(A.Direction(), G.Direction());
        Assert.Equal(M_PI, G.Angle(G.Reversed()), Tol);

        Geom_AxisPlacement GD = G.WithDirection(new gp_Dir(0, 0, 1));
        AssertDir(new gp_Dir(0, 0, 1), GD.Direction());
        AssertPnt(A.Location(), GD.Location());
        AssertPnt(new gp_Pnt(9, 9, 9), G.WithLocation(new gp_Pnt(9, 9, 9)).Location());
        Assert.Equal(A, G.Axis());
    }

    [Fact]
    public void Axis2Placement_matches_gp_Ax2()
    {
        // Vx not normal to N: the X direction is N ^ (Vx ^ N), as gp_Ax2 computes it
        gp_Pnt P = new(1, -2, 0.5);
        gp_Dir N = new(0, 0.6, 0.8);
        gp_Dir Vx = new(1, 1, 0);
        gp_Ax2 A = new(P, N, Vx);
        Geom_Axis2Placement G = new(P, N, Vx);
        AssertAx2(A, G);
        AssertAx2(A, new Geom_Axis2Placement(A));

        gp_Trsf T = gp_Trsf.Rotation(new gp_Ax1(new gp_Pnt(3, 0, 0), new gp_Dir(1, 1, 1)), 1.2)
                      .Multiplied(gp_Trsf.Translation(new gp_Vec(1, 2, 3)));
        AssertAx2(A.Transformed(T), (Geom_Axis2Placement)G.Transformed(T));
        AssertAx2(A.Mirrored(new gp_Pnt(0, 0, 0)), (Geom_Axis2Placement)G.Mirrored(new gp_Pnt(0, 0, 0)));

        gp_Dir aNewMain = new(1, 0, 0);
        gp_Ax2 AD = A;
        AD.SetDirection(aNewMain);
        AssertAx2(AD, (Geom_Axis2Placement)G.WithDirection(aNewMain));

        gp_Ax2 AX = A;
        AX.SetXDirection(new gp_Dir(1, 0, 0));
        AssertAx2(AX, G.WithXDirection(new gp_Dir(1, 0, 0)));

        gp_Ax2 AY = A;
        AY.SetYDirection(new gp_Dir(1, 0, 0));
        AssertAx2(AY, G.WithYDirection(new gp_Dir(1, 0, 0)));

        gp_Ax2 AOther = new(new gp_Pnt(5, 5, 5), new gp_Dir(0, 1, 0));
        AssertAx2(AOther, G.WithAx2(AOther));

        // The original is unchanged, and its copy is equal and unfrozen
        AssertAx2(A, G);
        G.Freeze();
        Geom_Axis2Placement C = (Geom_Axis2Placement)G.Copy();
        Assert.False(C.IsFrozen());
        AssertAx2(A, C);
        Assert.Throws<Standard_ImmutableObject>(() => G.SetXDirection(new gp_Dir(1, 0, 0)));
        Assert.Throws<Standard_ImmutableObject>(() => G.SetDirection(new gp_Dir(1, 0, 0)));
        Assert.Throws<Standard_ImmutableObject>(() => G.SetLocation(new gp_Pnt()));
        Assert.Throws<Standard_ImmutableObject>(() => G.Transform(T));
    }

    [Fact]
    public void Axis2Placement_SetDirection_parallel_to_X_throws()
    {
        Geom_Axis2Placement G = new(new gp_Ax2());
        Assert.Throws<Standard_ConstructionError>(() => G.WithDirection(new gp_Dir(1, 0, 0).Reversed()));
    }

    [Fact]
    public void Axis2Placement_SetAxis_keeps_X_and_Y_OCCT_bug()
    {
        // OCCT: Geom_AxisPlacement::SetAxis only assigns the axis, so a Geom_Axis2Placement's X and Y directions
        // are not recomputed as documented (docs/occt-issues.md). Reproduced on purpose.
        Geom_Axis2Placement G = new(new gp_Ax2());
        Geom_Axis2Placement S = (Geom_Axis2Placement)G.WithAxis(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 1)));
        AssertDir(new gp_Dir(0, 1, 1), S.Direction());
        AssertDir(new gp_Dir(1, 0, 0), S.XDirection());
        AssertDir(new gp_Dir(0, 1, 0), S.YDirection()); // no longer perpendicular to the main direction
    }
}
