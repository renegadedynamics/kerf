// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Geom_Axis1Placement and Geom_Axis2Placement against gp_Ax1 and gp_Ax2.</summary>
public class Geom_AxisPlacementTests
{
    private const double Tol = 1e-12;

    private static void AssertDir(in Direction3D theExpected, in Direction3D theActual)
    {
        Assert.Equal(theExpected.X(), theActual.X(), Tol);
        Assert.Equal(theExpected.Y(), theActual.Y(), Tol);
        Assert.Equal(theExpected.Z(), theActual.Z(), Tol);
    }

    private static void AssertPnt(in Point3D theExpected, in Point3D theActual)
    {
        Assert.Equal(theExpected.X(), theActual.X(), Tol);
        Assert.Equal(theExpected.Y(), theActual.Y(), Tol);
        Assert.Equal(theExpected.Z(), theActual.Z(), Tol);
    }

    private static void AssertAx2(in Frame3D theExpected, Axis2Placement theActual)
    {
        AssertPnt(theExpected.Location(), theActual.Location());
        AssertDir(theExpected.Direction(), theActual.Direction());
        AssertDir(theExpected.XDirection(), theActual.XDirection());
        AssertDir(theExpected.YDirection(), theActual.YDirection());
        Frame3D A = theActual.Ax2();
        AssertDir(theExpected.XDirection(), A.XDirection());
    }

    [Fact]
    public void Axis1Placement_matches_gp_Ax1()
    {
        Axis3D A = new(new Point3D(1, 2, 3), new Direction3D(1, 2, 2));
        Axis1Placement G = new(A.Location(), A.Direction());
        Assert.Equal(A, G.Ax1());
        Assert.Equal(A, G.Axis());

        Transform3D T = Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.5);
        Axis1Placement GT = (Axis1Placement)G.Transformed(T);
        Axis3D AT = A.Transformed(T);
        AssertPnt(AT.Location(), GT.Location());
        AssertDir(AT.Direction(), GT.Direction());

        AssertDir(A.Reversed().Direction(), G.Reversed().Direction());
        Assert.Equal(A.Direction(), G.Direction());
        Assert.Equal(M_PI, G.Angle(G.Reversed()), Tol);

        AxisPlacement GD = G.WithDirection(new Direction3D(0, 0, 1));
        AssertDir(new Direction3D(0, 0, 1), GD.Direction());
        AssertPnt(A.Location(), GD.Location());
        AssertPnt(new Point3D(9, 9, 9), G.WithLocation(new Point3D(9, 9, 9)).Location());
        Assert.Equal(A, G.Axis());
    }

    [Fact]
    public void Axis2Placement_matches_gp_Ax2()
    {
        // Vx not normal to N: the X direction is N ^ (Vx ^ N), as gp_Ax2 computes it
        Point3D P = new(1, -2, 0.5);
        Direction3D N = new(0, 0.6, 0.8);
        Direction3D Vx = new(1, 1, 0);
        Frame3D A = new(P, N, Vx);
        Axis2Placement G = new(P, N, Vx);
        AssertAx2(A, G);
        AssertAx2(A, new Axis2Placement(A));

        Transform3D T = Transform3D.Rotation(new Axis3D(new Point3D(3, 0, 0), new Direction3D(1, 1, 1)), 1.2)
                      .Multiplied(Transform3D.Translation(new Vector3D(1, 2, 3)));
        AssertAx2(A.Transformed(T), (Axis2Placement)G.Transformed(T));
        AssertAx2(A.Mirrored(new Point3D(0, 0, 0)), (Axis2Placement)G.Mirrored(new Point3D(0, 0, 0)));

        Direction3D aNewMain = new(1, 0, 0);
        Frame3D AD = A;
        AD = AD.WithDirection(aNewMain);
        AssertAx2(AD, (Axis2Placement)G.WithDirection(aNewMain));

        Frame3D AX = A;
        AX = AX.WithXDirection(new Direction3D(1, 0, 0));
        AssertAx2(AX, G.WithXDirection(new Direction3D(1, 0, 0)));

        Frame3D AY = A;
        AY = AY.WithYDirection(new Direction3D(1, 0, 0));
        AssertAx2(AY, G.WithYDirection(new Direction3D(1, 0, 0)));

        Frame3D AOther = new(new Point3D(5, 5, 5), new Direction3D(0, 1, 0));
        AssertAx2(AOther, G.WithAx2(AOther));

        // The original is unchanged, and its copy is equal and unfrozen
        AssertAx2(A, G);
        G.Freeze();
        Axis2Placement C = (Axis2Placement)G.Copy();
        Assert.False(C.IsFrozen);
        AssertAx2(A, C);
        Assert.Throws<ImmutableObjectException>(() => G.SetXDirection(new Direction3D(1, 0, 0)));
        Assert.Throws<ImmutableObjectException>(() => G.SetDirection(new Direction3D(1, 0, 0)));
        Assert.Throws<ImmutableObjectException>(() => G.SetLocation(new Point3D()));
        Assert.Throws<ImmutableObjectException>(() => G.Transform(T));
    }

    [Fact]
    public void Axis2Placement_SetDirection_parallel_to_X_throws()
    {
        Axis2Placement G = new(new Frame3D());
        Assert.Throws<ConstructionErrorException>(() => G.WithDirection(new Direction3D(1, 0, 0).Reversed()));
    }

    [Fact]
    public void Axis2Placement_SetAxis_keeps_X_and_Y_OCCT_bug()
    {
        // OCCT: Geom_AxisPlacement::SetAxis only assigns the axis, so a Geom_Axis2Placement's X and Y directions
        // are not recomputed as documented (docs/occt-issues.md). Reproduced on purpose.
        Axis2Placement G = new(new Frame3D());
        Axis2Placement S = (Axis2Placement)G.WithAxis(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 1)));
        AssertDir(new Direction3D(0, 1, 1), S.Direction());
        AssertDir(new Direction3D(1, 0, 0), S.XDirection());
        AssertDir(new Direction3D(0, 1, 0), S.YDirection()); // no longer perpendicular to the main direction
    }
}
