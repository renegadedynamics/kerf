// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Adaptor3d_TopolTool (OCCT has no GTest for it).</summary>
public class Adaptor3d_TopolToolTests
{
    /// <summary>
    /// A test-only surface adaptor over a Geom surface, with explicit bounds, until GeomAdaptor_Surface (unit 2A)
    /// is ported.
    /// </summary>
    private sealed class TestSurfaceAdaptor : SurfaceAdaptorBase
    {
        private readonly Surface mySurf;
        private readonly double myU1, myU2, myV1, myV2;

        public TestSurfaceAdaptor(Surface theSurf, double theU1, double theU2, double theV1, double theV2)
        {
            mySurf = theSurf;
            myU1 = theU1;
            myU2 = theU2;
            myV1 = theV1;
            myV2 = theV2;
        }

        public TestSurfaceAdaptor(Surface theSurf)
        {
            mySurf = theSurf;
            theSurf.Bounds(out myU1, out myU2, out myV1, out myV2);
        }

        public override double FirstUParameter() => myU1;
        public override double LastUParameter() => myU2;
        public override double FirstVParameter() => myV1;
        public override double LastVParameter() => myV2;
        public override Point3D EvalD0(double theU, double theV) => mySurf.Value(theU, theV);

        public override SurfaceKind GetType() => mySurf switch
        {
            PlaneSurface => global::Kerf.Geometry.SurfaceKind.Plane,
            ConicalSurface => global::Kerf.Geometry.SurfaceKind.Cone,
            global::Kerf.Geometry.BSplineSurface => SurfaceKind.BSplineSurface,
            global::Kerf.Geometry.BezierSurface => SurfaceKind.BezierSurface,
            _ => OtherSurface,
        };

        public override Cone Cone() => ((ConicalSurface)mySurf).Cone();
        public override global::Kerf.Geometry.BSplineSurface BSpline() => (global::Kerf.Geometry.BSplineSurface)mySurf;
        public override global::Kerf.Geometry.BezierSurface Bezier() => (global::Kerf.Geometry.BezierSurface)mySurf;
        public override int UKnotCount => BSpline().UKnotCount;
        public override int VKnotCount => BSpline().VKnotCount;
        public override int UDegree() => mySurf is global::Kerf.Geometry.BezierSurface B ? B.UDegree() : BSpline().UDegree();
        public override int VDegree() => mySurf is global::Kerf.Geometry.BezierSurface B ? B.VDegree() : BSpline().VDegree();
        public override int UPoleCount => mySurf is global::Kerf.Geometry.BezierSurface B ? B.UPoleCount : BSpline().UPoleCount;
        public override int VPoleCount => mySurf is global::Kerf.Geometry.BezierSurface B ? B.VPoleCount : BSpline().VPoleCount;

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

    private static PlaneSurface Plane() => new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

    /// <summary>A bicubic B-spline surface over [0, 1] x [0, 2], with interior knots, gently waved.</summary>
    private static global::Kerf.Geometry.BSplineSurface Wave(int theUDeg = 3, int theNbUKnots = 4)
    {
        int nbUPoles = theNbUKnots + theUDeg - 1, nbVPoles = 6;
        NCollection_Array2<Point3D> poles = new(1, nbUPoles, 1, nbVPoles);
        for (int i = 1; i <= nbUPoles; i++)
        {
            for (int j = 1; j <= nbVPoles; j++)
            {
                poles[i, j] = new Point3D(i, 2.0 * j, Math.Sin(i + 0.7 * j));
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
        return new global::Kerf.Geometry.BSplineSurface(poles, uKnots, vKnots, uMults, vMults, theUDeg, 3);
    }

    [Fact]
    public void Finite_domain_has_four_restrictions_and_two_vertices_each()
    {
        TopolTool T = new(new TestSurfaceAdaptor(Plane(), 0, 2, 1, 4));
        Assert.False(T.DomainIsInfinite());
        Assert.False(T.Has3d());
        Assert.Null(T.Edge());

        // Boundary run anticlockwise: V = Vinf, U = Usup, V = Vsup, U = Uinf; the last two are parameterised by -U, -V.
        Point2D[] starts = [new(0, 1), new(2, 1), new(2, 4), new(0, 4)];
        Point2D[] ends = [new(2, 1), new(2, 4), new(0, 4), new(0, 1)];
        int n = 0;
        for (T.Init(); T.More(); T.Next())
        {
            Curve2DAdaptorBase C = T.Value();
            Assert.Equal(Forward, T.Orientation(C));
            Assert.True(C.Value(C.FirstParameter()).IsEqual(starts[n], 1e-15));
            Assert.True(C.Value(C.LastParameter()).IsEqual(ends[n], 1e-15));

            T.Initialize(C);
            int nv = 0;
            for (T.InitVertexIterator(); T.MoreVertex(); T.NextVertex())
            {
                HVertex V = T.Vertex();
                Assert.Equal(nv == 0 ? Forward : Reversed, T.Orientation(V));
                Assert.True(V.Value().IsEqual(nv == 0 ? starts[n] : ends[n], 1e-15));
                nv++;
            }
            Assert.Equal(2, nv);
            Assert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => T.Vertex());
            n++;
        }
        Assert.Equal(4, n);
        Assert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => T.Value());
        Assert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => T.Tol3d(new HVertex()));
        Assert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => T.Pnt(new HVertex()));
        Assert.Throws<global::Kerf.Engine.Foundation.MethodNotImplementedException>(() => T.Initialize());
    }

    [Fact]
    public void Classify_and_IsThePointOn_finite_domain()
    {
        TopolTool T = new(new TestSurfaceAdaptor(Plane(), 0, 2, 1, 4));
        Assert.Equal(In, T.Classify(new Point2D(1, 2), 1e-7));
        Assert.Equal(On, T.Classify(new Point2D(2 + 1e-8, 2), 1e-7));
        Assert.Equal(Out, T.Classify(new Point2D(2.1, 2), 1e-7));
        Assert.Equal(Out, T.Classify(new Point2D(1, 0.5), 1e-7));
        Assert.False(T.IsThePointOn(new Point2D(1, 2), 1e-7));
        Assert.True(T.IsThePointOn(new Point2D(1, 4), 1e-7));
        Assert.False(T.IsThePointOn(new Point2D(1, 5), 1e-7));
    }

    [Fact]
    public void Half_infinite_domain()
    {
        double inf = global::Kerf.Engine.Foundation.Precision.Infinite();
        // U in [0, +inf), V in (-inf, 3]: restrictions V = Vsup and U = Uinf.
        TopolTool T = new(new TestSurfaceAdaptor(Plane(), 0, inf, -inf, 3));
        Assert.True(T.DomainIsInfinite());
        int n = 0;
        for (T.Init(); T.More(); T.Next())
        {
            n++;
        }
        Assert.Equal(2, n);

        Assert.Equal(In, T.Classify(new Point2D(1e6, -1e6), 1e-7));
        Assert.Equal(On, T.Classify(new Point2D(0, -5), 1e-7));
        Assert.Equal(On, T.Classify(new Point2D(5, 3), 1e-7));
        Assert.Equal(Out, T.Classify(new Point2D(-1, 0), 1e-7));
        Assert.Equal(Out, T.Classify(new Point2D(1, 4), 1e-7));
        Assert.True(T.IsThePointOn(new Point2D(0, 0), 1e-7));
        Assert.False(T.IsThePointOn(new Point2D(1, 0), 1e-7));

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
        double inf = global::Kerf.Engine.Foundation.Precision.Infinite();
        TopolTool T = new(new TestSurfaceAdaptor(Plane(), -inf, 1, -inf, 1));
        Assert.Equal(Out, T.Classify(new Point2D(1.5, 0), 0.5));
        Assert.Equal(On, T.Classify(new Point2D(0, 1.5), 0.5));
    }

    [Fact]
    public void Infinite_domain_is_all_IN()
    {
        double inf = global::Kerf.Engine.Foundation.Precision.Infinite();
        TopolTool T = new(new TestSurfaceAdaptor(Plane(), -inf, inf, -inf, inf));
        T.Init();
        Assert.False(T.More());
        Assert.Equal(In, T.Classify(new Point2D(1e10, -3), 1e-7));
        Assert.False(T.IsThePointOn(new Point2D(0, 0), 1e-7));
    }

    [Fact]
    public void Cone_with_two_restrictions_adds_the_apex_iso()
    {
        // A cone unbounded in V: the restrictions U = Usup and U = Uinf, plus the apex iso.
        CoordinateSystem ax = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        ConicalSurface cone = new(ax, Math.PI / 6, 2.0);
        double inf = global::Kerf.Engine.Foundation.Precision.Infinite();
        TopolTool T = new(new TestSurfaceAdaptor(cone, 0, 2 * Math.PI, -inf, inf));
        int n = 0;
        Curve2DAdaptorBase? last = null;
        for (T.Init(); T.More(); T.Next())
        {
            last = T.Value();
            n++;
        }
        Assert.Equal(3, n);
        TopolTool.GetConeApexParam(cone.Cone(), out double Ua, out double Va);
        Assert.Equal(0.0, Ua);
        Assert.Equal(-2.0 / Math.Sin(Math.PI / 6), Va, 1e-12);
        Assert.True(cone.Value(Ua, Va).IsEqual(cone.Cone().Apex(), 1e-12));
        Assert.True(last!.Value(last.FirstParameter()).IsEqual(new Point2D(0, Va), 1e-12));
    }

    [Fact]
    public void Identical_ignores_orientation()
    {
        TopolTool T = new();
        HVertex A = new(new Point2D(1, 1), Forward, 1e-8);
        HVertex B = new(new Point2D(1, 1 + 1e-9), Reversed, 1e-8);
        HVertex C = new(new Point2D(1, 1.1), Forward, 1e-8);
        Assert.True(T.Identical(A, B));
        Assert.False(T.Identical(A, C));
    }

    [Fact]
    public void ComputeSamplePoints_by_type()
    {
        TopolTool P = new(new TestSurfaceAdaptor(Plane(), 0, 1, 0, 1));
        Assert.Equal(6, P.NbSamplesU()); // 2, raised to the minimum 6
        Assert.Equal(6, P.NbSamplesV());
        Assert.Equal(36, P.NbSamples());
        Assert.True(P.IsUniformSampling());

        // Uniform sample points (no SamplePnts): (Index / NbU, Index mod NbU) on a grid of step 1/(Nb + 1).
        P.SamplePoint(7, out Point2D p2, out Point3D p3);
        Assert.Equal(2.0 / 7, p2.X(), 1e-15);
        Assert.Equal(2.0 / 7, p2.Y(), 1e-15);
        Assert.True(p3.IsEqual(new Point3D(p2.X(), p2.Y(), 0), 1e-15));

        // B-spline: NbKnots * Degree = 4 * 3 = 12 > 8, so the poles are analysed: 5 + sign changes of the second
        // differences.
        global::Kerf.Geometry.BSplineSurface bs = Wave();
        TopolTool B = new(new TestSurfaceAdaptor(bs));
        Assert.False(B.IsUniformSampling());
        Assert.InRange(B.NbSamplesU(), 5, 50);
        Assert.InRange(B.NbSamplesV(), 5, 50);
        Assert.Equal(B.NbSamplesU() * B.NbSamplesV(), B.NbSamples());
    }

    [Fact]
    public void SamplePnts_on_plane_is_uniform()
    {
        TopolTool T = new(new TestSurfaceAdaptor(Plane(), 0, 1, 2, 4));
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
        T.SamplePoint(8, out Point2D p2, out Point3D p3);
        Assert.Equal(U[2], p2.X());
        Assert.Equal(V[2], p2.Y());
        Assert.Equal(0.0, p3.Z());
    }

    [Fact]
    public void BSplSamplePnts_keeps_ends_and_respects_minimum()
    {
        global::Kerf.Geometry.BSplineSurface bs = Wave();
        TopolTool T = new(new TestSurfaceAdaptor(bs));
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
        global::Kerf.Geometry.BSplineSurface bs = Wave(theUDeg: 1, theNbUKnots: 2);
        TopolTool T = new(new TestSurfaceAdaptor(bs));
        T.SamplePnts(1e-3, 1, 1);
        Assert.Equal(4, T.NbSamplesU());
        NCollection_Array1<double> U = new(1, 4);
        T.UParameters(U);
        Assert.Equal([0.0, 1.0, 0.0, 0.0], [U[1], U[2], U[3], U[4]]);
    }
}
