// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingData.Tests;

/// <summary>Adaptor3d_TopolTool (OCCT has no GTest for it).</summary>
public class Adaptor3d_TopolToolTests
{
    /// <summary>
    /// A test-only surface adaptor over a Geom surface, with explicit bounds, until GeomAdaptor_Surface (unit 2A)
    /// is ported.
    /// </summary>
    private sealed class TestSurfaceAdaptor : Adaptor3d_Surface
    {
        private readonly Geom_Surface mySurf;
        private readonly double myU1, myU2, myV1, myV2;

        public TestSurfaceAdaptor(Geom_Surface theSurf, double theU1, double theU2, double theV1, double theV2)
        {
            mySurf = theSurf;
            myU1 = theU1;
            myU2 = theU2;
            myV1 = theV1;
            myV2 = theV2;
        }

        public TestSurfaceAdaptor(Geom_Surface theSurf)
        {
            mySurf = theSurf;
            theSurf.Bounds(out myU1, out myU2, out myV1, out myV2);
        }

        public override double FirstUParameter() => myU1;
        public override double LastUParameter() => myU2;
        public override double FirstVParameter() => myV1;
        public override double LastVParameter() => myV2;
        public override gp_Pnt EvalD0(double theU, double theV) => mySurf.Value(theU, theV);

        public override GeomAbs_SurfaceType GetType() => mySurf switch
        {
            Geom_Plane => GeomAbs_Plane,
            Geom_ConicalSurface => GeomAbs_Cone,
            Geom_BSplineSurface => GeomAbs_BSplineSurface,
            Geom_BezierSurface => GeomAbs_BezierSurface,
            _ => GeomAbs_OtherSurface,
        };

        public override gp_Cone Cone() => ((Geom_ConicalSurface)mySurf).Cone();
        public override Geom_BSplineSurface BSpline() => (Geom_BSplineSurface)mySurf;
        public override Geom_BezierSurface Bezier() => (Geom_BezierSurface)mySurf;
        public override int NbUKnots() => BSpline().NbUKnots();
        public override int NbVKnots() => BSpline().NbVKnots();
        public override int UDegree() => mySurf is Geom_BezierSurface B ? B.UDegree() : BSpline().UDegree();
        public override int VDegree() => mySurf is Geom_BezierSurface B ? B.VDegree() : BSpline().VDegree();
        public override int NbUPoles() => mySurf is Geom_BezierSurface B ? B.NbUPoles() : BSpline().NbUPoles();
        public override int NbVPoles() => mySurf is Geom_BezierSurface B ? B.NbVPoles() : BSpline().NbVPoles();

        public override double UResolution(double R3d)
        {
            BSpline().Resolution(R3d, out double U, out _);
            return U;
        }

        public override double VResolution(double R3d)
        {
            BSpline().Resolution(R3d, out _, out double V);
            return V;
        }
    }

    private static Geom_Plane Plane() => new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));

    /// <summary>A bicubic B-spline surface over [0, 1] x [0, 2], with interior knots, gently waved.</summary>
    private static Geom_BSplineSurface Wave(int theUDeg = 3, int theNbUKnots = 4)
    {
        int nbUPoles = theNbUKnots + theUDeg - 1, nbVPoles = 6;
        NCollection_Array2<gp_Pnt> poles = new(1, nbUPoles, 1, nbVPoles);
        for (int i = 1; i <= nbUPoles; i++)
        {
            for (int j = 1; j <= nbVPoles; j++)
            {
                poles[i, j] = new gp_Pnt(i, 2.0 * j, Math.Sin(i + 0.7 * j));
            }
        }
        NCollection_Array1<double> uKnots = new(1, theNbUKnots);
        NCollection_Array1<int> uMults = new(1, theNbUKnots);
        for (int i = 1; i <= theNbUKnots; i++)
        {
            uKnots[i] = (i - 1.0) / (theNbUKnots - 1);
            uMults[i] = (i == 1 || i == theNbUKnots) ? theUDeg + 1 : 1;
        }
        NCollection_Array1<double> vKnots = new(1, 4);
        NCollection_Array1<int> vMults = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            vKnots[i] = 2.0 * (i - 1.0) / 3.0;
            vMults[i] = (i == 1 || i == 4) ? 4 : 1;
        }
        return new Geom_BSplineSurface(poles, uKnots, vKnots, uMults, vMults, theUDeg, 3);
    }

    [Fact]
    public void Finite_domain_has_four_restrictions_and_two_vertices_each()
    {
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(Plane(), 0, 2, 1, 4));
        Assert.False(T.DomainIsInfinite());
        Assert.False(T.Has3d());
        Assert.Null(T.Edge());

        // Boundary run anticlockwise: V = Vinf, U = Usup, V = Vsup, U = Uinf; the last two are parameterised by -U, -V.
        gp_Pnt2d[] starts = [new(0, 1), new(2, 1), new(2, 4), new(0, 4)];
        gp_Pnt2d[] ends = [new(2, 1), new(2, 4), new(0, 4), new(0, 1)];
        int n = 0;
        for (T.Init(); T.More(); T.Next())
        {
            Adaptor2d_Curve2d C = T.Value();
            Assert.Equal(TopAbs_FORWARD, T.Orientation(C));
            Assert.True(C.Value(C.FirstParameter()).IsEqual(starts[n], 1e-15));
            Assert.True(C.Value(C.LastParameter()).IsEqual(ends[n], 1e-15));

            T.Initialize(C);
            int nv = 0;
            for (T.InitVertexIterator(); T.MoreVertex(); T.NextVertex())
            {
                Adaptor3d_HVertex V = T.Vertex();
                Assert.Equal(nv == 0 ? TopAbs_FORWARD : TopAbs_REVERSED, T.Orientation(V));
                Assert.True(V.Value().IsEqual(nv == 0 ? starts[n] : ends[n], 1e-15));
                nv++;
            }
            Assert.Equal(2, nv);
            Assert.Throws<Standard_DomainError>(() => T.Vertex());
            n++;
        }
        Assert.Equal(4, n);
        Assert.Throws<Standard_DomainError>(() => T.Value());
        Assert.Throws<Standard_DomainError>(() => T.Tol3d(new Adaptor3d_HVertex()));
        Assert.Throws<Standard_DomainError>(() => T.Pnt(new Adaptor3d_HVertex()));
        Assert.Throws<Standard_NotImplemented>(() => T.Initialize());
    }

    [Fact]
    public void Classify_and_IsThePointOn_finite_domain()
    {
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(Plane(), 0, 2, 1, 4));
        Assert.Equal(TopAbs_IN, T.Classify(new gp_Pnt2d(1, 2), 1e-7));
        Assert.Equal(TopAbs_ON, T.Classify(new gp_Pnt2d(2 + 1e-8, 2), 1e-7));
        Assert.Equal(TopAbs_OUT, T.Classify(new gp_Pnt2d(2.1, 2), 1e-7));
        Assert.Equal(TopAbs_OUT, T.Classify(new gp_Pnt2d(1, 0.5), 1e-7));
        Assert.False(T.IsThePointOn(new gp_Pnt2d(1, 2), 1e-7));
        Assert.True(T.IsThePointOn(new gp_Pnt2d(1, 4), 1e-7));
        Assert.False(T.IsThePointOn(new gp_Pnt2d(1, 5), 1e-7));
    }

    [Fact]
    public void Half_infinite_domain()
    {
        double inf = Precision.Infinite();
        // U in [0, +inf), V in (-inf, 3]: restrictions V = Vsup and U = Uinf.
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(Plane(), 0, inf, -inf, 3));
        Assert.True(T.DomainIsInfinite());
        int n = 0;
        for (T.Init(); T.More(); T.Next())
        {
            n++;
        }
        Assert.Equal(2, n);

        Assert.Equal(TopAbs_IN, T.Classify(new gp_Pnt2d(1e6, -1e6), 1e-7));
        Assert.Equal(TopAbs_ON, T.Classify(new gp_Pnt2d(0, -5), 1e-7));
        Assert.Equal(TopAbs_ON, T.Classify(new gp_Pnt2d(5, 3), 1e-7));
        Assert.Equal(TopAbs_OUT, T.Classify(new gp_Pnt2d(-1, 0), 1e-7));
        Assert.Equal(TopAbs_OUT, T.Classify(new gp_Pnt2d(1, 4), 1e-7));
        Assert.True(T.IsThePointOn(new gp_Pnt2d(0, 0), 1e-7));
        Assert.False(T.IsThePointOn(new gp_Pnt2d(1, 0), 1e-7));

        // The first restriction (V = Vsup, run along -U) has only one finite end.
        T.Init();
        T.Initialize(T.Value());
        int nv = 0;
        for (T.InitVertexIterator(); T.MoreVertex(); T.NextVertex())
        {
            nv++;
        }
        Assert.Equal(1, nv);
    }

    [Fact]
    public void Half_infinite_U_and_V_bounds_differ_at_exactly_bound_plus_Tol()
    {
        // OCCT tests U >= Usup + Tol but V > Vsup + Tol when the lower bound is infinite (occt-issues.md).
        double inf = Precision.Infinite();
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(Plane(), -inf, 1, -inf, 1));
        Assert.Equal(TopAbs_OUT, T.Classify(new gp_Pnt2d(1.5, 0), 0.5));
        Assert.Equal(TopAbs_ON, T.Classify(new gp_Pnt2d(0, 1.5), 0.5));
    }

    [Fact]
    public void Infinite_domain_is_all_IN()
    {
        double inf = Precision.Infinite();
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(Plane(), -inf, inf, -inf, inf));
        T.Init();
        Assert.False(T.More());
        Assert.Equal(TopAbs_IN, T.Classify(new gp_Pnt2d(1e10, -3), 1e-7));
        Assert.False(T.IsThePointOn(new gp_Pnt2d(0, 0), 1e-7));
    }

    [Fact]
    public void Cone_with_two_restrictions_adds_the_apex_iso()
    {
        // A cone unbounded in V: the restrictions U = Usup and U = Uinf, plus the apex iso.
        gp_Ax3 ax = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        Geom_ConicalSurface cone = new(ax, Math.PI / 6, 2.0);
        double inf = Precision.Infinite();
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(cone, 0, 2 * Math.PI, -inf, inf));
        int n = 0;
        Adaptor2d_Curve2d? last = null;
        for (T.Init(); T.More(); T.Next())
        {
            last = T.Value();
            n++;
        }
        Assert.Equal(3, n);
        Adaptor3d_TopolTool.GetConeApexParam(cone.Cone(), out double Ua, out double Va);
        Assert.Equal(0.0, Ua);
        Assert.Equal(-2.0 / Math.Sin(Math.PI / 6), Va, 1e-12);
        Assert.True(cone.Value(Ua, Va).IsEqual(cone.Cone().Apex(), 1e-12));
        Assert.True(last!.Value(last.FirstParameter()).IsEqual(new gp_Pnt2d(0, Va), 1e-12));
    }

    [Fact]
    public void Identical_ignores_orientation()
    {
        Adaptor3d_TopolTool T = new();
        Adaptor3d_HVertex A = new(new gp_Pnt2d(1, 1), TopAbs_FORWARD, 1e-8);
        Adaptor3d_HVertex B = new(new gp_Pnt2d(1, 1 + 1e-9), TopAbs_REVERSED, 1e-8);
        Adaptor3d_HVertex C = new(new gp_Pnt2d(1, 1.1), TopAbs_FORWARD, 1e-8);
        Assert.True(T.Identical(A, B));
        Assert.False(T.Identical(A, C));
    }

    [Fact]
    public void ComputeSamplePoints_by_type()
    {
        Adaptor3d_TopolTool P = new(new TestSurfaceAdaptor(Plane(), 0, 1, 0, 1));
        Assert.Equal(6, P.NbSamplesU()); // 2, raised to the minimum 6
        Assert.Equal(6, P.NbSamplesV());
        Assert.Equal(36, P.NbSamples());
        Assert.True(P.IsUniformSampling());

        // Uniform sample points (no SamplePnts): (Index / NbU, Index mod NbU) on a grid of step 1/(Nb + 1).
        P.SamplePoint(7, out gp_Pnt2d p2, out gp_Pnt p3);
        Assert.Equal(2.0 / 7, p2.X(), 1e-15);
        Assert.Equal(2.0 / 7, p2.Y(), 1e-15);
        Assert.True(p3.IsEqual(new gp_Pnt(p2.X(), p2.Y(), 0), 1e-15));

        // B-spline: NbKnots * Degree = 4 * 3 = 12 > 8, so the poles are analysed: 5 + sign changes of the second
        // differences.
        Geom_BSplineSurface bs = Wave();
        Adaptor3d_TopolTool B = new(new TestSurfaceAdaptor(bs));
        Assert.False(B.IsUniformSampling());
        Assert.InRange(B.NbSamplesU(), 5, 50);
        Assert.InRange(B.NbSamplesV(), 5, 50);
        Assert.Equal(B.NbSamplesU() * B.NbSamplesV(), B.NbSamples());
    }

    [Fact]
    public void SamplePnts_on_plane_is_uniform()
    {
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(Plane(), 0, 1, 2, 4));
        T.SamplePnts(0.01, 10, 10);
        NCollection_Array1<double> U = new(1, T.NbSamplesU());
        NCollection_Array1<double> V = new(1, T.NbSamplesV());
        T.UParameters(U);
        T.VParameters(V);
        Assert.Equal(6, U.Length());
        Assert.Equal(0.0, U[1]);
        Assert.Equal(1.0, U[6]);
        Assert.Equal(0.4, U[3], 1e-15);
        Assert.Equal(2.0, V[1]);
        Assert.Equal(4.0, V[6]);

        // With parameters, SamplePoint's index is 1-based, U first.
        T.SamplePoint(8, out gp_Pnt2d p2, out gp_Pnt p3);
        Assert.Equal(U[2], p2.X());
        Assert.Equal(V[2], p2.Y());
        Assert.Equal(0.0, p3.Z());
    }

    [Fact]
    public void BSplSamplePnts_keeps_ends_and_respects_minimum()
    {
        Geom_BSplineSurface bs = Wave();
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(bs));
        T.SamplePnts(1e-3, 4, 4);
        int nu = T.NbSamplesU(), nv = T.NbSamplesV();
        Assert.InRange(nu, 4, 10); // knots 0, 1/3, 2/3, 1 with 3 subdivisions each: 10 candidates
        Assert.InRange(nv, 4, 10);
        NCollection_Array1<double> U = new(1, nu);
        NCollection_Array1<double> V = new(1, nv);
        T.UParameters(U);
        T.VParameters(V);
        Assert.Equal(0.0, U[1]);
        Assert.Equal(1.0, U[nu]);
        Assert.Equal(0.0, V[1]);
        Assert.Equal(2.0, V[nv]);
        for (int i = 2; i <= nu; i++)
        {
            Assert.True(U[i] > U[i - 1]);
        }

        // Minimums above the candidates make the sampling uniform. With as many candidates in U as in V, OCCT
        // scales U by theNVmin / nbsv: 10 * 25 / 10 = 25 points in U, although theNUmin is 20.
        T.SamplePnts(1e-3, 20, 25);
        Assert.Equal(25, T.NbSamplesU());
        Assert.Equal(25, T.NbSamplesV());
        U = new(1, 25);
        T.UParameters(U);
        Assert.Equal(1.0 / 24, U[2], 1e-15);
    }

    [Fact]
    public void BSplSamplePnts_fewer_than_four_candidates()
    {
        // Degree 1 in U with one span: two candidate U parameters. The count is still set to 4, and OCCT leaves the
        // two extra parameters at 0 (occt-issues.md).
        Geom_BSplineSurface bs = Wave(theUDeg: 1, theNbUKnots: 2);
        Adaptor3d_TopolTool T = new(new TestSurfaceAdaptor(bs));
        T.SamplePnts(1e-3, 1, 1);
        Assert.Equal(4, T.NbSamplesU());
        NCollection_Array1<double> U = new(1, 4);
        T.UParameters(U);
        Assert.Equal([0.0, 1.0, 0.0, 0.0], [U[1], U[2], U[3], U[4]]);
    }
}
