// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ApproxInt's MultiLine, BRepApprox_ApproxLine, and the OCCT issues of unit 5F reproduced: each expected value was
/// printed by a native program built against OCCT 8.0.1 (decision 680) on the same inputs.
/// </summary>
public class ApproxIntTests
{
    private static Adaptor3d_Surface PlaneZ() => new GeomAdaptor_Surface(new Geom_Plane(new Plane3D()));

    private static Adaptor3d_Surface PlaneX() =>
        new GeomAdaptor_Surface(new Geom_Plane(new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0))));

    private static void AssertPnt(double x, double y, double z, Point3D P)
    {
        Assert.Equal(x, P.X());
        Assert.Equal(y, P.Y());
        Assert.Equal(z, P.Z());
    }

    // occt-issues.md, 5F: a second Compute at the same parameters returns the cached flag without its outputs, so
    // PrmPrmSvSurfaces.SeekPoint gives the origin with the parameters it was given.
    [Fact]
    public void PrmPrmSvSurfaces_SeekPointTwice_ReturnsTheOriginTheSecondTime()
    {
        GeomInt_ThePrmPrmSvSurfacesOfWLApprox sv = new(PlaneZ(), PlaneX());

        IntSurf_PntOn2S pt = new();
        Assert.True(sv.SeekPoint(0.01, 0.5, 0.5, 0.01, ref pt));
        AssertPnt(0, 0.5, 0, pt.Value());
        pt.Parameters(out double u1, out double v1, out double u2, out double v2);
        Assert.Equal((0.0, 0.5, 0.0, -0.5), (u1, v1, u2, v2));

        IntSurf_PntOn2S pt2 = new();
        Assert.True(sv.SeekPoint(0.01, 0.5, 0.5, 0.01, ref pt2));
        AssertPnt(0, 0, 0, pt2.Value()); // OCCT
        pt2.Parameters(out u1, out v1, out u2, out v2);
        Assert.Equal((0.01, 0.5, 0.5, 0.01), (u1, v1, u2, v2));
    }

    // The same issue through ImpPrmSvSurfaces.Compute (the quadric first): the second call leaves its outputs as given.
    [Fact]
    public void ImpPrmSvSurfaces_ComputeTwice_LeavesTheOutputsTheSecondTime()
    {
        IntSurf_Quadric Q = new(new Plane3D());
        GeomInt_TheImpPrmSvSurfacesOfWLApprox sv = new(Q, PlaneX());
        for (int k = 1; k <= 2; k++)
        {
            double u1 = 0.0, v1 = -0.5, u2 = 0.0001, v2 = 0.5;
            Point3D P = new(0, 0, 0);
            Vector3D T = new(0, 0, 0);
            Vector2D T1 = new(0, 0), T2 = new(0, 0);
            Assert.True(sv.Compute(ref u1, ref v1, ref u2, ref v2, ref P, ref T, ref T1, ref T2));
            if (k == 1)
            {
                Assert.Equal((0.0, -0.5, 0.0, 0.5), (u1, v1, u2, v2));
                AssertPnt(0, -0.5, 0, P);
                Assert.Equal((0.0, 1.0, 0.0), (T.X(), T.Y(), T.Z()));
                Assert.Equal((0.0, 1.0, 0.0, -1.0), (T1.X(), T1.Y(), T2.X(), T2.Y()));
            }
            else
            {
                Assert.Equal((0.0, -0.5, 0.0001, 0.5), (u1, v1, u2, v2)); // OCCT: not refined
                AssertPnt(0, 0, 0, P);
                Assert.Equal((0.0, 0.0, 0.0), (T.X(), T.Y(), T.Z()));
                Assert.Equal((0.0, 0.0, 0.0, 0.0), (T1.X(), T1.Y(), T2.X(), T2.Y()));
            }
        }
    }

    private static IntPatch_WLine StraightLine(int theNb)
    {
        IntSurf_LineOn2S L = new();
        for (int k = 1; k <= theNb; k++)
        {
            L.Add(new IntSurf_PntOn2S().WithValue(new Point3D(k, 0, 0), k, 0, k, 1));
        }
        return new IntPatch_WLine(L, false);
    }

    // occt-issues.md, 5F: Perform(line) on more than 2 * NbPntMax points with two knots adds a middle knot at
    // (indicemax - indicemin) / 2, an index count: from indicemin > 1 the curve starts before indicemin.
    [Theory]
    [InlineData(1, 300, 1.0)]
    [InlineData(150, 300, 75.0)]
    [InlineData(200, 300, 50.0)]
    public void WLApprox_PerformLine_MiddleKnotIsAnIndexCount(int theMin, int theMax, double theFirstX)
    {
        GeomInt_WLApprox app = new();
        app.Perform(StraightLine(300), true, true, true, theMin, theMax);
        Assert.True(app.IsDone());
        AppParCurves_MultiBSpCurve mb = app.Value(1);
        int np = mb.NbPoles();
        Assert.Equal(8, np);
        Assert.Equal(3, mb.Knots().Length());
        NCollection_Array1<Point3D> pp = new(1, np);
        mb.Curve(1, pp);
        Assert.Equal(theFirstX, pp[1].X()); // OCCT for theMin > 1: native gives 75 and 50
        Assert.Equal(300.0, pp[np].X());
    }

    [Fact]
    public void MultiLine_Values_TranslatedAndOnTheChosenSurface()
    {
        IntPatch_WLine WL = StraightLine(10);
        GeomInt_TheMultiLineOfWLApprox ml = new(WL, 1, 1, true, false, 1, 2, 3, 10, 20, 30, 40, false, 7, 3);
        Assert.Equal(3, ml.FirstPoint());
        Assert.Equal(7, ml.LastPoint());
        Assert.Equal(Approx_Status.Approx_NoPointsAdded, ml.WhatStatus());
        NCollection_Array1<Point3D> P = new(1, 1);
        NCollection_Array1<Point2D> P2 = new(1, 1);
        ml.Value(4, P, P2);
        AssertPnt(5, 2, 3, P[1]);
        // one 2d point, not on the first surface: (u2 + U2o, v2 + V2o)
        Assert.Equal((34.0, 41.0), (P2[1].X(), P2[1].Y()));
        NCollection_Array1<Vector3D> T = new(1, 1);
        Assert.False(ml.Tangency(4, T));

        ApproxInt_MultiLine<IntPatch_WLine> anEmpty = ml.MakeMLBetween(3, 7, 4);
        Assert.Equal((1, 1), (anEmpty.FirstPoint(), anEmpty.LastPoint()));
        Assert.False(ml.MakeMLOneMorePoint(3, 7, 5, out ApproxInt_MultiLine<IntPatch_WLine> anOther));
        Assert.Equal(0, anOther.NbP3d());
    }

    // The planes z = 0 and x = 0 meet on the Y axis; the line's points are (0, y, 0) with y = 0.1 k or 0.01 k^2.
    // Native OCCT gives the same: MakeMLBetween's resampling never reaches the number of points asked for on these
    // lines (an empty line), and MakeMLOneMorePoint adds the point (0, 0.95, 0) or (0, 0.905, 0).
    [Theory]
    [InlineData(false, 0.95)]
    [InlineData(true, 0.905)]
    public void MultiLine_WithSvSurfaces_AddsPoints(bool theNonUniform, double theNewY)
    {
        Adaptor3d_Surface S1 = PlaneZ(), S2 = PlaneX();
        Plane3D pl1 = new(), pl2 = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        IntSurf_LineOn2S L = new();
        for (int k = 0; k <= 20; k++)
        {
            double y = theNonUniform ? 0.01 * k * k : 0.1 * k;
            Point3D aP = new(0, y, 0);
            global::Kerf.Engine.Geometry.ElSLib.Parameters(pl1, aP, out double u1, out double v1);
            global::Kerf.Engine.Geometry.ElSLib.Parameters(pl2, aP, out double u2, out double v2);
            L.Add(new IntSurf_PntOn2S().WithValue(aP, u1, v1, u2, v2));
        }
        IntPatch_WLine WL = new(L, false);
        GeomInt_ThePrmPrmSvSurfacesOfWLApprox sv = new(S1, S2);
        GeomInt_TheMultiLineOfWLApprox ml = new(WL, sv, 1, 2, true, true, 0, 0, 0, 0, 0, 0, 0, true, 1, 21);
        Assert.Equal(Approx_Status.Approx_PointsAdded, ml.WhatStatus());
        NCollection_Array1<Vector3D> T = new(1, 1);
        NCollection_Array1<Vector2D> T2 = new(1, 2);
        Assert.True(ml.Tangency(5, T, T2));
        Assert.Equal((0.0, 1.0, 0.0), (T[1].X(), T[1].Y(), T[1].Z()));
        Assert.Equal((0.0, 1.0, 0.0, -1.0), (T2[1].X(), T2[1].Y(), T2[2].X(), T2[2].Y()));

        for (int n = 0; n <= 40; n += 20)
        {
            ApproxInt_MultiLine<IntPatch_WLine> aBetween = ml.MakeMLBetween(1, 21, n);
            Assert.Equal((1, 1), (aBetween.FirstPoint(), aBetween.LastPoint()));
            Assert.Equal(Approx_Status.Approx_NoPointsAdded, aBetween.WhatStatus());
        }

        Assert.True(ml.MakeMLOneMorePoint(1, 21, 11, out ApproxInt_MultiLine<IntPatch_WLine> anOther));
        Assert.Equal(22, anOther.LastPoint());
        NCollection_Array1<Point3D> P = new(1, 1);
        anOther.Value(11, P);
        AssertPnt(0, theNewY, 0, P[1]);
    }

    [Fact]
    public void BRepApproxLine_FromCurvesAndFromPoints()
    {
        NCollection_Array1<Point3D> p3 = new(1, 3);
        NCollection_Array1<Point2D> p2 = new(1, 3);
        NCollection_Array1<double> kn = new(1, 3);
        NCollection_Array1<int> mu = new(1, 3);
        for (int i = 1; i <= 3; i++)
        {
            p3[i] = new Point3D(i, 2 * i, 3 * i);
            p2[i] = new Point2D(-i, i);
            kn[i] = i;
            mu[i] = 1;
        }
        mu[1] = mu[3] = 2;
        BRepApprox_ApproxLine L = new(new Geom_BSplineCurve(p3, kn, mu, 1), null, new Geom2d_BSplineCurve(p2, kn, mu, 1));
        Assert.Equal(3, L.NbPnts());
        IntSurf_PntOn2S aP = L.Point(2);
        AssertPnt(2, 4, 6, aP.Value());
        aP.Parameters(out double u1, out double v1, out double u2, out double v2);
        Assert.Equal((0.0, 0.0, -2.0, 2.0), (u1, v1, u2, v2));

        IntSurf_LineOn2S lin = new();
        lin.Add(new IntSurf_PntOn2S().WithValue(new Point3D(1, 1, 1), 1, 2, 3, 4));
        BRepApprox_ApproxLine L2 = new(lin);
        Assert.Equal(1, L2.NbPnts());
        Assert.Equal(lin.Value(1), L2.Point(1));
    }
}
