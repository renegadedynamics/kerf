// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Occt.GeomFill_FillingStyle;
using static Kerf.Engine.Foundation.Standard;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of GeomFill's filling half (unit 5L3): Coons, Curved and Stretch, BSplineCurves and BezierCurves, the
/// boundaries, CoonsAlgPatch, TgtOnCoons, CornerState and ConstrainedFilling. Native parity is FillRecordingsTests'.
/// </summary>
public class GeomFill_FillingTests
{
    private static NCollection_Array1<Point3D> Row(params Point3D[] thePoints)
    {
        NCollection_Array1<Point3D> aRow = new(1, thePoints.Length);
        for (int i = 0; i < thePoints.Length; i++)
        {
            aRow[i + 1] = thePoints[i];
        }
        return aRow;
    }

    private static NCollection_Array1<double> Weights(params double[] theW)
    {
        NCollection_Array1<double> aW = new(1, theW.Length);
        for (int i = 0; i < theW.Length; i++)
        {
            aW[i + 1] = theW[i];
        }
        return aW;
    }

    // Four rows of a 4 x 3 grid around (0,0)-(3,2), z = x * y / 4, as GeomFill_Curved and Stretch take them:
    // P1 row j = 1, P2 column i = NU, P3 row j = NV, P4 column i = 1
    private static Point3D G(int i, int j) => new(i - 1, j - 1, (i - 1) * (j - 1) / 4.0);

    private static (NCollection_Array1<Point3D>, NCollection_Array1<Point3D>, NCollection_Array1<Point3D>, NCollection_Array1<Point3D>) Grid()
    {
        return (Row(G(1, 1), G(2, 1), G(3, 1), G(4, 1)), Row(G(4, 1), G(4, 2), G(4, 3)),
                Row(G(1, 3), G(2, 3), G(3, 3), G(4, 3)), Row(G(1, 1), G(1, 2), G(1, 3)));
    }

    private static NCollection_Array2<Point3D> PolesOf(GeomFill_Filling theF)
    {
        NCollection_Array2<Point3D> aP = new(1, theF.NbUPoles(), 1, theF.NbVPoles());
        theF.Poles(aP);
        return aP;
    }

    [Fact]
    public void Stretch_and_Curved_keep_the_boundary_rows()
    {
        var (P1, P2, P3, P4) = Grid();
        foreach (GeomFill_Filling F in new GeomFill_Filling[] { new GeomFill_Stretch(P1, P2, P3, P4), new GeomFill_Curved(P1, P2, P3, P4) })
        {
            Assert.Equal(4, F.NbUPoles());
            Assert.Equal(3, F.NbVPoles());
            Assert.False(F.isRational());
            NCollection_Array2<Point3D> aP = PolesOf(F);
            for (int i = 1; i <= 4; i++)
            {
                Assert.Equal(P1[i], aP[i, 1]);
                Assert.Equal(P3[i], aP[i, 3]);
            }
            Assert.Equal(P4[2], aP[1, 2]);
            Assert.Equal(P2[2], aP[4, 2]);
        }
    }

    [Fact]
    public void Stretch_reproduces_a_bilinear_grid()
    {
        // the bilinear Coons patch of a bilinear grid is the grid itself
        var (P1, P2, P3, P4) = Grid();
        NCollection_Array2<Point3D> aP = PolesOf(new GeomFill_Stretch(P1, P2, P3, P4));
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                Assert.True(aP[i, j].Distance(G(i, j)) < 1e-14, $"{i},{j}");
            }
        }
    }

    [Fact]
    public void Coons_keeps_its_boundaries_and_blends_the_inner_poles()
    {
        // Coons takes P2 as the column i = 1 and P4 as the column i = NbUPoles
        NCollection_Array1<Point3D> P1 = Row(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0), new Point3D(3, 0, 0));
        NCollection_Array1<Point3D> P3 = Row(new Point3D(0, 3, 0), new Point3D(1, 3, 0), new Point3D(2, 3, 0), new Point3D(3, 3, 0));
        NCollection_Array1<Point3D> P2 = Row(new Point3D(0, 0, 0), new Point3D(0, 1, 0), new Point3D(0, 2, 0), new Point3D(0, 3, 0));
        NCollection_Array1<Point3D> P4 = Row(new Point3D(3, 0, 0), new Point3D(3, 1, 0), new Point3D(3, 2, 0), new Point3D(3, 3, 0));
        GeomFill_Coons F = new(P1, P2, P3, P4);
        NCollection_Array2<Point3D> aP = PolesOf(F);
        for (int k = 1; k <= 4; k++)
        {
            Assert.Equal(P1[k], aP[k, 1]);
            Assert.Equal(P3[k], aP[k, 4]);
            Assert.Equal(P2[k], aP[1, k]);
            Assert.Equal(P4[k], aP[4, k]);
        }
        // a planar square grid stays the grid
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                Assert.True(aP[i, j].Distance(new Point3D(i - 1, j - 1, 0)) < 1e-14, $"{i},{j}");
            }
        }
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => new GeomFill_Coons(P1, P2, Row(new Point3D()), P4));
    }

    [Fact]
    public void Rational_fillings_keep_the_boundary_weights()
    {
        var (P1, P2, P3, P4) = Grid();
        NCollection_Array1<double> W1 = Weights(1, 2, 2, 1), W2 = Weights(1, 1.5, 1), W3 = Weights(1, 0.5, 0.5, 1), W4 = Weights(1, 3, 1);
        foreach (GeomFill_Filling F in new GeomFill_Filling[]
                 {
                     new GeomFill_Stretch(P1, P2, P3, P4, W1, W2, W3, W4), new GeomFill_Curved(P1, P2, P3, P4, W1, W2, W3, W4),
                 })
        {
            Assert.True(F.isRational());
            NCollection_Array2<double> aW = new(1, 4, 1, 3);
            F.Weights(aW);
            for (int i = 1; i <= 4; i++)
            {
                Assert.Equal(W1[i], aW[i, 1]);
                Assert.Equal(W3[i], aW[i, 3]);
            }
            Assert.Equal(W4[2], aW[1, 2]);
            Assert.Equal(W2[2], aW[4, 2]);
        }
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => new GeomFill_Stretch(P1, P2, P3, P4, W1, W2, W1, W4.Length() == 3 ? Weights(1, 1) : W4));
    }

    [Fact]
    public void Coons_rational_weights_follow_OCCTs_column_order()
    {
        // OCCT: the poles' first column is P2 but the weights' is W4 (occt-issues.md)
        NCollection_Array1<Point3D> P1 = Row(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0));
        NCollection_Array1<Point3D> P3 = Row(new Point3D(0, 2, 0), new Point3D(1, 2, 0), new Point3D(2, 2, 0));
        NCollection_Array1<Point3D> P2 = Row(new Point3D(0, 0, 0), new Point3D(0, 1, 0), new Point3D(0, 2, 0));
        NCollection_Array1<Point3D> P4 = Row(new Point3D(2, 0, 0), new Point3D(2, 1, 0), new Point3D(2, 2, 0));
        NCollection_Array1<double> W = Weights(1, 1, 1), W2 = Weights(1, 5, 1), W4 = Weights(1, 7, 1);
        GeomFill_Coons F = new(P1, P2, P3, P4, W, W2, W, W4);
        NCollection_Array2<double> aW = new(1, 3, 1, 3);
        F.Weights(aW);
        Assert.Equal(7.0, aW[1, 2]);
        Assert.Equal(5.0, aW[3, 2]);
    }

    [Fact]
    public void Curved_with_two_rows_translates_the_first_along_the_second()
    {
        NCollection_Array1<Point3D> P1 = Row(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0));
        NCollection_Array1<Point3D> P2 = Row(new Point3D(0, 0, 0), new Point3D(0, 1, 1), new Point3D(0, 2, 0), new Point3D(0, 3, 0));
        GeomFill_Curved F = new(P1, P2);
        NCollection_Array2<Point3D> aP = PolesOf(F);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                Assert.Equal(P1[i].Translated(new Vector3D(P2[1], P2[j])), aP[i, j]);
            }
        }
        GeomFill_Curved R = new(P1, P2, Weights(2, 1, 2), Weights(2, 4, 2, 2));
        NCollection_Array2<double> aW = new(1, 3, 1, 4);
        R.Weights(aW);
        Assert.Equal(2.0 * 4 / 2, aW[1, 2]);
        Assert.Equal(1.0 * 4 / 2, aW[2, 2]);
    }

    //=============================================================================================
    // BSplineCurves and BezierCurves

    private static BSplineCurve3D BS(params Point3D[] thePoles)
    {
        int n = thePoles.Length;
        int aDeg = Math.Min(3, n - 1);
        int aNbK = n - aDeg + 1;
        NCollection_Array1<double> aK = new(1, aNbK);
        NCollection_Array1<int> aM = new(1, aNbK);
        for (int i = 1; i <= aNbK; i++)
        {
            aK[i] = i - 1;
            aM[i] = (i == 1 || i == aNbK) ? aDeg + 1 : 1;
        }
        return new BSplineCurve3D(Row(thePoles), aK, aM, aDeg);
    }

    // a contour (0,0) -> (4,0) -> (4,3) -> (0,3) -> (0,0), bumped in z, given in a shuffled order and direction
    private static BSplineCurve3D[] Contour() =>
    [
        BS(new Point3D(0, 0, 0), new Point3D(1, 0, 1), new Point3D(2, 0, -1), new Point3D(3, 0, 0.5), new Point3D(4, 0, 0)),
        BS(new Point3D(0, 3, 0), new Point3D(2, 3, 1), new Point3D(4, 3, 0)),            // C3, left to right
        BS(new Point3D(4, 3, 0), new Point3D(4, 2, -1), new Point3D(4, 1, 1), new Point3D(4, 0, 0)), // C2, reversed
        BS(new Point3D(0, 0, 0), new Point3D(0, 1.5, 2), new Point3D(0, 3, 0)),          // C4, bottom to top
    ];

    private static double MaxDistanceToSurface(Curve3D theC, Surface theS)
    {
        double aMax = 0;
        for (int k = 0; k <= 20; k++)
        {
            double t = theC.FirstParameter() + (theC.LastParameter() - theC.FirstParameter()) * k / 20.0;
            GeomAPI_ProjectPointOnSurf aProj = new(theC.Value(t), theS);
            aMax = Math.Max(aMax, aProj.LowerDistance());
        }
        return aMax;
    }

    [Theory]
    [InlineData(GeomFill_StretchStyle)]
    [InlineData(GeomFill_CoonsStyle)]
    [InlineData(GeomFill_CurvedStyle)]
    public void BSplineCurves_fill_four_curves_in_any_order_and_direction(GeomFill_FillingStyle theStyle)
    {
        BSplineCurve3D[] C = Contour();
        foreach (BSplineCurve3D c in C)
        {
            c.Freeze(); // frozen inputs: the filling works on copies
        }
        string[] aBefore = C.Select(c => string.Join(";", c.Poles())).ToArray();
        GeomFill_BSplineCurves aFill = new(C[0], C[1], C[2], C[3], theStyle);
        BSplineSurface aS = aFill.Surface()!;
        Assert.True(aS.IsFrozen);
        Assert.Same(aS, aFill.Surface());
        Assert.Equal(aBefore, C.Select(c => string.Join(";", c.Poles())).ToArray());
        foreach (BSplineCurve3D c in C)
        {
            Assert.True(MaxDistanceToSurface(c, aS) < 1e-7);
        }
        // the first curve is the boundary v = V1
        aS.Bounds(out double u1, out double u2, out double v1, out _);
        Assert.True(aS.Value(u1, v1).Distance(new Point3D(0, 0, 0)) < 1e-12);
        Assert.True(aS.Value(u2, v1).Distance(new Point3D(4, 0, 0)) < 1e-12);
    }

    [Fact]
    public void BSplineCurves_three_curves_are_closed_by_a_segment()
    {
        BSplineCurve3D[] C = Contour();
        GeomFill_BSplineCurves aFill = new(C[0], C[2], C[1], GeomFill_StretchStyle);
        BSplineSurface aS = aFill.Surface()!;
        for (int k = 0; k < 3; k++)
        {
            Assert.True(MaxDistanceToSurface(C[k], aS) < 1e-7);
        }
        // the fourth side is the segment (0,3,0)-(0,0,0)
        Assert.True(MaxDistanceToSurface(BS(new Point3D(0, 0, 0), new Point3D(0, 3, 0)), aS) < 1e-7);
    }

    [Fact]
    public void BSplineCurves_two_curves_make_a_ruled_or_a_translated_surface()
    {
        BSplineCurve3D C1 = BS(new Point3D(0, 0, 0), new Point3D(1, 0, 1), new Point3D(2, 0, 0));
        BSplineCurve3D C2 = BS(new Point3D(0, 3, 1), new Point3D(1, 3, 0), new Point3D(2, 3, 2), new Point3D(3, 3, 1));
        BSplineSurface aRuled = new GeomFill_BSplineCurves(C1, C2, GeomFill_StretchStyle).Surface()!;
        Assert.Equal(1, aRuled.VDegree());
        Assert.True(MaxDistanceToSurface(C1, aRuled) < 1e-7);
        Assert.True(MaxDistanceToSurface(C2, aRuled) < 1e-7);

        // Curved: C1 translated along a curve starting at C1's start
        BSplineCurve3D C3 = BS(new Point3D(0, 0, 0), new Point3D(0, 1, 1), new Point3D(0, 2, 0));
        BSplineSurface aT = new GeomFill_BSplineCurves(C1, C3, GeomFill_CurvedStyle).Surface()!;
        Assert.True(MaxDistanceToSurface(C1, aT) < 1e-7);
        Assert.True(MaxDistanceToSurface(C3, aT) < 1e-7);

        Assert.ThrowsAny<global::Kerf.Engine.Foundation.OutOfRangeException>(() => new GeomFill_BSplineCurves(C1, C2, GeomFill_CurvedStyle));
    }

    [Fact]
    public void BSplineCurves_curved_two_curves_leave_the_callers_curve_unchanged()
    {
        // C1's end is C2's start: OCCT reverses the caller's C1 (and builds from the unreversed copy); the port
        // builds the same surface and leaves C1 alone (decision 846, occt-issues.md)
        BSplineCurve3D C1 = BS(new Point3D(0, 0, 0), new Point3D(1, 0, 1), new Point3D(2, 0, 0));
        BSplineCurve3D C2 = BS(new Point3D(2, 0, 0), new Point3D(2, 1, 1), new Point3D(2, 2, 0));
        C1.Freeze();
        BSplineSurface aS = new GeomFill_BSplineCurves(C1, C2, GeomFill_CurvedStyle).Surface()!;
        Assert.Equal(new Point3D(0, 0, 0), C1.StartPoint());
        // the copy is not reversed: the poles are C1's translated by C2's poles from C2's start
        Assert.Equal(new Point3D(0, 0, 0), aS.Pole(1, 1));
        Assert.Equal(new Point3D(0, 2, 0), aS.Pole(1, 3));
    }

    [Fact]
    public void BSplineCurves_reject_open_contours_and_too_few_poles_for_Coons()
    {
        BSplineCurve3D[] C = Contour();
        BSplineCurve3D aFar = BS(new Point3D(10, 10, 10), new Point3D(11, 10, 10));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new GeomFill_BSplineCurves(C[0], C[1], C[2], aFar, GeomFill_StretchStyle));
        BSplineCurve3D L1 = BS(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        BSplineCurve3D L2 = BS(new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        BSplineCurve3D L3 = BS(new Point3D(1, 1, 0), new Point3D(0, 1, 0));
        BSplineCurve3D L4 = BS(new Point3D(0, 1, 0), new Point3D(0, 0, 0));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new GeomFill_BSplineCurves(L1, L2, L3, L4, GeomFill_CoonsStyle));
        BSplineSurface aPlane = new GeomFill_BSplineCurves(L1, L2, L3, L4, GeomFill_StretchStyle).Surface()!;
        Assert.True(aPlane.Value(0.5, 0.5).Distance(new Point3D(0.5, 0.5, 0)) < 1e-15);
    }

    private static BezierCurve3D BZ(params Point3D[] thePoles) => new(Row(thePoles));

    [Theory]
    [InlineData(GeomFill_StretchStyle)]
    [InlineData(GeomFill_CoonsStyle)]
    [InlineData(GeomFill_CurvedStyle)]
    public void BezierCurves_fill_two_three_and_four_curves(GeomFill_FillingStyle theStyle)
    {
        BezierCurve3D C1 = BZ(new Point3D(0, 0, 0), new Point3D(1, 0, 1), new Point3D(2, 0, 0));
        BezierCurve3D C2 = BZ(new Point3D(2, 0, 0), new Point3D(2, 1, -1), new Point3D(2, 2, 1), new Point3D(2, 3, 0));
        BezierCurve3D C3 = BZ(new Point3D(0, 3, 0), new Point3D(2, 3, 0)); // left to right
        BezierCurve3D C4 = BZ(new Point3D(0, 3, 0), new Point3D(0, 1.5, 1), new Point3D(0, 0, 0)); // top to bottom
        C1.Freeze();
        BezierSurface aS = new GeomFill_BezierCurves(C1, C2, C3, C4, theStyle).Surface()!;
        Assert.True(aS.IsFrozen);
        foreach (BezierCurve3D c in new[] { C1, C2, C3, C4 })
        {
            Assert.True(MaxDistanceToSurface(c, aS) < 1e-7);
        }
        if (theStyle == GeomFill_CoonsStyle)
        {
            Assert.True(aS.UDegree() >= 3 && aS.VDegree() >= 3);
        }
        BezierSurface a3 = new GeomFill_BezierCurves(C1, C2, C3, theStyle).Surface()!;
        Assert.True(MaxDistanceToSurface(C2, a3) < 1e-7);

        BezierCurve3D D2 = BZ(new Point3D(0, 0, 0), new Point3D(0, 1, 1), new Point3D(0, 2, 0));
        BezierSurface a2 = new GeomFill_BezierCurves(C1, D2, theStyle).Surface()!;
        Assert.True(MaxDistanceToSurface(C1, a2) < 1e-7);
    }

    [Fact]
    public void BezierCurves_rational_corners_get_the_same_weights()
    {
        BezierCurve3D C1 = new(Row(new Point3D(0, 0, 0), new Point3D(1, 0, 1), new Point3D(2, 0, 0)), Weights(1, 2, 1));
        BezierCurve3D C2 = BZ(new Point3D(2, 0, 0), new Point3D(2, 2, 0));
        BezierCurve3D C3 = new(Row(new Point3D(0, 2, 0), new Point3D(1, 2, 1), new Point3D(2, 2, 0)), Weights(2, 1, 2));
        BezierCurve3D C4 = BZ(new Point3D(0, 0, 0), new Point3D(0, 2, 0));
        BezierSurface aS = new GeomFill_BezierCurves(C1, C2, C3, C4, GeomFill_StretchStyle).Surface()!;
        Assert.True(aS.IsURational || aS.IsVRational);
        foreach (BezierCurve3D c in new[] { C1, C2, C3, C4 })
        {
            Assert.True(MaxDistanceToSurface(c, aS) < 1e-7);
        }
    }

    //=============================================================================================
    // Boundaries

    private static BSplineSurface Patch()
    {
        NCollection_Array2<Point3D> aP = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aP[i, j] = new Point3D(2.0 * (i - 1), 2.0 * (j - 1), Math.Sin(i) * Math.Cos(j));
            }
        }
        NCollection_Array1<double> K = new(1, 2);
        K[1] = 0;
        K[2] = 1;
        NCollection_Array1<int> M = new(1, 2);
        M[1] = M[2] = 4;
        return new BSplineSurface(aP, K, K, M, M, 3, 3);
    }

    private static void CheckD1(GeomFill_Boundary theB, double theW, double theTol)
    {
        const double h = 1e-6;
        theB.D1(theW, out Point3D P, out Vector3D V);
        Assert.True(P.Distance(theB.Value(theW)) < 1e-14);
        Vector3D aFd = new Vector3D(theB.Value(theW - h), theB.Value(theW + h)).Divided(2 * h);
        Assert.True(V.Subtracted(aFd).Magnitude() < theTol, $"{V} {aFd}");
    }

    [Fact]
    public void SimpleBound_evaluates_its_curve_and_its_reparametrisation()
    {
        BSplineSurface S = Patch();
        GeomFill_Boundary B = FillRecordings.MakeBound(S, 0, 0, 1e-4, 1e-2);
        Assert.False(B.HasNormals());
        Assert.False(B.IsDegenerated());
        Assert.Equal(1e-4, B.Tol3d());
        B.Tol3d(1e-3);
        Assert.Equal(1e-3, B.Tol3d());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => B.Norm(0.5));
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.FailureException>(() => B.D1Norm(0.5, out _, out _));
        B.Bounds(out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(1.0, l);
        CheckD1(B, 0.4, 1e-6);

        // reversed on [0, 1]: the ends swap
        Point3D aStart = B.Value(0), anEnd = B.Value(1);
        B.Reparametrize(0.0, 1.0, false, false, 1.0, 1.0, true);
        B.Points(out Point3D P1, out Point3D P2);
        Assert.True(P1.Distance(anEnd) < 1e-9);
        Assert.True(P2.Distance(aStart) < 1e-9);
        CheckD1(B, 0.3, 1e-5);
    }

    [Fact]
    public void BoundWithSurf_normals_are_the_surfaces()
    {
        BSplineSurface S = Patch();
        for (int k = 0; k < 4; k++)
        {
            GeomFill_Boundary B = FillRecordings.MakeBound(S, k, 1, 1e-4, 1e-2);
            Assert.True(B.HasNormals());
            foreach (double w in new[] { 0.0, 0.3, 0.8 })
            {
                Vector3D N = B.Norm(w);
                Assert.Equal(1.0, N.Magnitude(), 12);
                double u = k switch { 0 => w, 1 => 1, 2 => w, _ => 0 };
                double v = k switch { 0 => 0, 1 => w, 2 => 1, _ => w };
                S.D1(u, v, out Point3D P, out Vector3D DU, out Vector3D DV);
                Assert.True(N.Subtracted(DU.Crossed(DV).Normalized()).Magnitude() < 1e-12);
                Assert.True(B.Value(w).Distance(P) < 1e-12);

                B.D1Norm(w, out Vector3D N1, out Vector3D DN);
                Assert.True(N1.Subtracted(N).Magnitude() < 1e-12);
                const double h = 1e-6;
                Vector3D aFd = B.Norm(w + h).Subtracted(B.Norm(w - h)).Divided(2 * h);
                Assert.True(DN.Subtracted(aFd).Magnitude() < 1e-5, $"{DN} {aFd}");
            }
            CheckD1(B, 0.5, 1e-6);
        }
    }

    [Fact]
    public void DegeneratedBound_is_a_point_on_a_range()
    {
        GeomFill_DegeneratedBound B = new(new Point3D(1, 2, 3), 0.5, 2.0, 1e-3, 1e-2);
        Assert.True(B.IsDegenerated());
        Assert.Equal(new Point3D(1, 2, 3), B.Value(0.7));
        B.D1(1.0, out Point3D P, out Vector3D V);
        Assert.Equal(new Point3D(1, 2, 3), P);
        Assert.Equal(0.0, V.Magnitude());
        B.Reparametrize(0, 1, true, true, 2, 2, true);
        B.Bounds(out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(1.0, l);
    }

    //=============================================================================================
    // CoonsAlgPatch and TgtOnCoons

    [Fact]
    public void CoonsAlgPatch_derivatives_match_finite_differences()
    {
        BSplineSurface S = Patch();
        GeomFill_Boundary[] B = [.. Enumerable.Range(0, 4).Select(k => FillRecordings.MakeBound(S, k, 1, 1e-4, 1e-2))];
        GeomFill_CoonsAlgPatch K = new(B[0], B[1], B[2], B[3]);
        Assert.True(K.Corner(0).Distance(S.Value(0, 0)) < 1e-12);
        Assert.True(K.Corner(2).Distance(S.Value(1, 1)) < 1e-12);
        Assert.Same(B[2], K.Bound(2));
        K.Func(out Law_Function f1, out Law_Function f2);
        Assert.Same(f1, K.Func(0));
        Assert.Same(f2, K.Func(1));
        Assert.Equal(1.0, f1.Value(0.0), 15);
        Assert.Equal(1.0, f2.Value(1.0), 15);

        // the patch S(u,v) of D1U, D1V: integrate along v = 0.4 from the boundary u = 0 (Value is wrong, see below)
        const double h = 1e-6;
        foreach (double u in new[] { 0.2, 0.6 })
        {
            foreach (double v in new[] { 0.3, 0.7 })
            {
                Vector3D aDuvFd = K.D1U(u, v + h).Subtracted(K.D1U(u, v - h)).Divided(2 * h);
                Assert.True(K.DUV(u, v).Subtracted(aDuvFd).Magnitude() < 1e-5);
                Vector3D aDvuFd = K.D1V(u + h, v).Subtracted(K.D1V(u - h, v)).Divided(2 * h);
                Assert.True(K.DUV(u, v).Subtracted(aDvuFd).Magnitude() < 1e-5);
            }
        }
        // on the boundaries, D1U along v = 0 is the bound's derivative
        B[0].D1(0.4, out _, out Vector3D aT);
        Assert.True(K.D1U(0.4, 0.0).Subtracted(aT).Magnitude() < 1e-12);

        // OCCT evaluates bounds 0 and 2 at V in Value (occt-issues.md): the value at (u, v) depends on u only
        // through the blending of bounds 1 and 3
        Point3D aBug = K.Value(0.25, 0.5);
        double a0 = K.Func(0).Value(0.5), a1 = K.Func(1).Value(0.25);
        Xyz anExpected = B[0].Value(0.5).XYZ() * a0 + B[1].Value(0.5).XYZ() * a1 + B[2].Value(0.5).XYZ() * (1 - a0)
                            + B[3].Value(0.5).XYZ() * (1 - a1) - K.Corner(0).XYZ() * (a0 * (1 - a1))
                            - K.Corner(1).XYZ() * (a0 * a1) - K.Corner(2).XYZ() * (a1 * (1 - a0))
                            - K.Corner(3).XYZ() * ((1 - a0) * (1 - a1));
        Assert.True(aBug.Distance(new Point3D(anExpected)) < 1e-12);
    }

    [Fact]
    public void TgtOnCoons_is_orthogonal_to_the_normal_and_its_derivative_matches()
    {
        BSplineSurface S = Patch();
        GeomFill_Boundary[] B = [.. Enumerable.Range(0, 4).Select(k => FillRecordings.MakeBound(S, k, 1, 1e-4, 1e-2))];
        GeomFill_CoonsAlgPatch K = new(B[0], B[1], B[2], B[3]);
        for (int i = 0; i < 4; i++)
        {
            GeomFill_TgtOnCoons T = new(K, i);
            Assert.False(T.IsScalable());
            foreach (double w in new[] { 0.25, 0.6 })
            {
                Vector3D V = T.Value(w);
                Assert.True(Math.Abs(V.Dot(B[i].Norm(w))) < 1e-10 * Math.Max(1, V.Magnitude()));
                T.D1(w, out Vector3D T1, out Vector3D DT);
                Assert.True(T1.Subtracted(V).Magnitude() < 1e-12);
                Assert.True(DT.Subtracted(T.D1(w)).Magnitude() < 1e-12);
                const double h = 1e-6;
                Vector3D aFd = T.Value(w + h).Subtracted(T.Value(w - h)).Divided(2 * h);
                Assert.True(DT.Subtracted(aFd).Magnitude() < 1e-4, $"{i} {w}: {DT} {aFd}");
            }
        }
    }

    [Fact]
    public void CornerState_is_a_value_with_twins()
    {
        GeomFill_CornerState s = new();
        Assert.Equal(RealLast(), s.Gap());
        Assert.False(s.HasConstraint());
        Assert.False(s.IsToKill(out double aScal));
        Assert.Equal(1.0, aScal);
        GeomFill_CornerState t = s.WithGap(0.5).WithTgtAng(0.1).WithNorAng(0.2).Constrained();
        Assert.Equal(RealLast(), s.Gap()); // unchanged
        Assert.Equal(0.5, t.Gap());
        Assert.Equal(0.1, t.TgtAng());
        Assert.Equal(0.2, t.NorAng());
        Assert.True(t.HasConstraint());
        Assert.False(t.IsToKill(out _));
        GeomFill_CornerState k = t.Killed(0.25);
        Assert.True(k.IsToKill(out aScal));
        Assert.Equal(0.25, aScal);
        Assert.False(s.Killed(0.5).IsToKill(out _)); // not constrained
        Assert.Equal(t, s.WithGap(0.5).WithTgtAng(0.1).WithNorAng(0.2).Constrained());
        Assert.NotEqual(t, k);
        Assert.Contains("GeomFill_CornerState", t.ToString(), StringComparison.Ordinal);
    }

    //=============================================================================================
    // ConstrainedFilling

    private static double MaxBoundaryGap(GeomFill_ConstrainedFilling theF, Surface theS, int theI)
    {
        double aMax = 0;
        GeomFill_Boundary b = theF.Boundary(theI);
        for (int k = 0; k <= 10; k++)
        {
            GeomAPI_ProjectPointOnSurf aProj = new(b.Value(k / 10.0), theS);
            aMax = Math.Max(aMax, aProj.LowerDistance());
        }
        return aMax;
    }

    [Theory]
    [InlineData(0, false)]
    [InlineData(1, false)]
    [InlineData(1, true)]
    public void ConstrainedFilling_fills_four_boundaries(int theType, bool theNoCheck)
    {
        BSplineSurface S = Patch();
        GeomFill_Boundary[] B = [.. Enumerable.Range(0, 4).Select(k => FillRecordings.MakeBound(S, k, theType, 1e-4, 1e-2))];
        GeomFill_ConstrainedFilling F = new(8, 10);
        Assert.Throws<global::Kerf.Engine.Foundation.FailureException>(() => F.ReBuild());
        F.Init(B[2], B[0], B[3], B[1], theNoCheck); // any order
        BSplineSurface R = F.Surface()!;
        Assert.True(R.IsFrozen);
        for (int i = 0; i < 4; i++)
        {
            Assert.True(MaxBoundaryGap(F, R, i) < 1e-3, $"bound {i}");
        }
        // S's own boundaries and normals: the filling is close to S
        R.D1(0.5, 0.0, out _, out Vector3D du, out Vector3D dv);
        if (theType == 1)
        {
            Vector3D aN = du.Crossed(dv).Normalized();
            Vector3D aSN = F.Boundary(0).Norm(0.5);
            Assert.True(Math.Min(aN.Angle(aSN), Math.PI - aN.Angle(aSN)) < 0.05);
        }
        double[] r = new double[12];
        Assert.Equal(0, F.Eval(0.5, 0, r));

        StringWriter aW = new();
        F.CheckApprox(0, aW);
        F.CheckResult(0, aW);
        F.CheckTgteField(0, aW);
        F.CheckCoonsAlgPatch(0);
        Assert.Contains("Distance max", aW.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void ConstrainedFilling_three_sides_and_SetDomain()
    {
        // a triangle (0,0)-(4,0)-(0,4) on the plane z = 0, its sides as lines on the plane
        PlaneSurface aPlane = new(new Plane3D(new Point3D(), new Direction3D(Direction3D.D.Z)));
        GeomFill_Boundary Side(Point2D theA, Point2D theB)
        {
            LineCurve2D aL = new(theA, new Direction2D(new Vector2D(theA, theB)));
            CurveOnSurface aCoS = new(new CurveAdaptor2D(aL, 0.0, theA.Distance(theB)),
                                                new SurfaceAdaptor(aPlane));
            return new GeomFill_BoundWithSurf(aCoS, 1e-4, 1e-2);
        }
        Point2D A = new(0, 0), Bp = new(4, 0), C = new(0, 4);
        GeomFill_ConstrainedFilling F = new(8, 10);
        F.Init(Side(A, Bp), Side(C, A), Side(Bp, C)); // any order and direction
        BSplineSurface R = F.Surface()!;
        Assert.True(F.Boundary(2).IsDegenerated());
        for (int i = 0; i < 4; i++)
        {
            if (i != 2)
            {
                Assert.True(MaxBoundaryGap(F, R, i) < 1e-3, $"bound {i}");
            }
        }
        Assert.True(Math.Abs(R.Value(0.3, 0.4).Z()) < 1e-6); // planar

        // SetDomain on a constrained boundary, then ReBuild: a new surface, the old one unchanged
        GeomFill_BoundWithSurf aB0 = (GeomFill_BoundWithSurf)F.Boundary(0);
        string aBefore = string.Join(";", R.Poles());
        F.SetDomain(0.5, aB0);
        F.ReBuild();
        BSplineSurface R2 = F.Surface()!;
        Assert.NotSame(R, R2);
        Assert.Equal(aBefore, string.Join(";", R.Poles()));
        Assert.True(MaxBoundaryGap(F, R2, 0) < 1e-3);
    }
}
