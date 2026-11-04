// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Geometry.Properties;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of GeomPlate's classes other than the builder: Aij, the point and curve constraints, the plate surface,
/// the average plane, the criteria and the approximation.
/// </summary>
public class GeomPlate_ClassesTests
{
    private static PlateSolver SolvedPlate()
    {
        var aPlate = new PlateSolver();
        aPlate.Load(new PinpointConstraint(new Xy(0, 0), new Xyz(0, 0, 0)));
        aPlate.Load(new PinpointConstraint(new Xy(1, 0), new Xyz(0, 0, 0)));
        aPlate.Load(new PinpointConstraint(new Xy(0, 1), new Xyz(0, 0, 0)));
        aPlate.Load(new PinpointConstraint(new Xy(1, 1), new Xyz(0, 0, 0)));
        aPlate.Load(new PinpointConstraint(new Xy(0.5, 0.5), new Xyz(0, 0, 0.3)));
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone);
        return aPlate;
    }

    private static PlaneSurface XYPlane() => new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));

    [Fact]
    public void Aij_compares_by_value()
    {
        var anA = new Aij(1, 2, new Vector3D(0, 0, 1));
        Assert.Equal(anA, new Aij(1, 2, new Vector3D(0, 0, 1)));
        Assert.NotEqual(anA, new Aij(2, 1, new Vector3D(0, 0, 1)));
        Assert.Equal(new Aij(0, 0, new Vector3D()), new Aij());
    }

    [Fact]
    public void Point_constraints_hold_their_point_and_derivatives()
    {
        var aP = new PointConstraint(new Point3D(1, 2, 3), 0, 0.5);
        aP.D0(out Point3D aPnt);
        Assert.Equal(new Point3D(1, 2, 3), aPnt);
        Assert.Equal(0.5, aP.G0Criterion());
        Assert.False(aP.HasPnt2dOnSurf);
        aP.SetPnt2dOnSurf(new Point2D(4, 5));
        Assert.True(aP.HasPnt2dOnSurf);
        Assert.Equal(new Point2D(4, 5), aP.Pnt2dOnSurf());
        EngineAssert.Throws<FailureException>(() => new PointConstraint(new Point3D(), 2));

        var aSphere = new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2.0);
        var aS = new PointConstraint(0.3, 0.4, aSphere, 2, 1e-4, 1e-3, 1e-2);
        aS.D2(out Point3D aP2, out Vector3D aD1U, out Vector3D aD1V, out Vector3D aD2U, out Vector3D aD2V, out Vector3D aD2UV);
        aSphere.D2(0.3, 0.4, out Point3D aQ, out Vector3D aE1U, out Vector3D aE1V, out Vector3D aE2U, out Vector3D aE2V, out Vector3D aE2UV);
        Assert.Equal(aQ, aP2);
        Assert.Equal(aE1U, aD1U);
        Assert.Equal(aE2UV, aD2UV);
        Assert.Equal(aE1V, aD1V);
        Assert.Equal(aE2U, aD2U);
        Assert.Equal(aE2V, aD2V);
        Assert.Equal(2, aS.Order());
        SLProps aProps = aS.LPropSurf();
        Assert.True(aProps.IsCurvatureDefined()); // the curvatures are computed on demand (as OCCT's)
        Assert.Equal(0.25, aProps.MeanCurvature() * aProps.MeanCurvature(), 12);
    }

    [Fact]
    public void Curve_constraints_on_a_surface_evaluate_through_it()
    {
        var aPlane = XYPlane();
        var aLine = new LineCurve2D(new Point2D(1, 1), new Direction2D(1, 0));
        var aCOS = new CurveOnSurface(new CurveAdaptor2D(aLine, 0, 5), new SurfaceAdaptor(aPlane));
        var aC = new CurveConstraint(aCOS, 1, 12);
        Assert.Equal(12, aC.PointCount);
        Assert.Equal(0.0, aC.FirstParameter());
        Assert.Equal(5.0, aC.LastParameter());
        Assert.Equal(5.0, aC.Length(), 9);
        aC.D1(2.0, out Point3D aP, out Vector3D aV1, out Vector3D aV2);
        Assert.Equal(new Point3D(3, 1, 0), aP);
        Assert.Equal(1.0, (aV1 ^ aV2).Z);
        Assert.Equal(0.0, aC.LPropSurf(2.0).GaussianCurvature());
        Assert.Equal(0.01, aC.G1Criterion(1.0));
        aC.SetG1Criterion(new Linear());
        Assert.Same(aCOS, aC.Curve3d());
        Assert.Null(aC.Curve2dOnSurf());

        // a 3D curve: only G0 queries
        var a3d = new CurveConstraint(new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 1), 0);
        EngineAssert.Throws<FailureException>(() => a3d.D1(0.5, out _, out _, out _));
        EngineAssert.Throws<FailureException>(() => a3d.G1Criterion(0.5));
        EngineAssert.Throws<FailureException>(() => a3d.LPropSurf(0.5));
        EngineAssert.Throws<FailureException>(() => new CurveConstraint(aCOS, 3));
    }

    [Fact]
    public void The_plate_surface_is_the_initial_surface_plus_the_plate()
    {
        PlateSolver aPlate = SolvedPlate();
        var aSurf = new PlateSurface(XYPlane(), aPlate);
        Point3D aP = aSurf.Value(0.5, 0.5);
        Assert.Equal(0.3, aP.Z, 12);
        Xyz aD = aPlate.Evaluate(new Xy(0.25, 0.75));
        Assert.Equal(new Point3D(0.25 + aD.X(), 0.75 + aD.Y(), aD.Z()), aSurf.Value(0.25, 0.75));
        aSurf.D2(0.3, 0.6, out Point3D aP2, out Vector3D aD1U, out _, out Vector3D aD2U, out _, out _);
        Assert.Equal(aSurf.Value(0.3, 0.6), aP2);
        Assert.Equal(new Vector3D(1, 0, 0) + new Vector3D(aPlate.EvaluateDerivative(new Xy(0.3, 0.6), 1, 0)), aD1U);
        Assert.Equal(new Vector3D(aPlate.EvaluateDerivative(new Xy(0.3, 0.6), 2, 0)), aD2U);
        EngineAssert.Throws<UndefinedDerivative3DException>(() => aSurf.D3(0.3, 0.6, out _, out _, out _, out _, out _, out _, out _, out _, out _, out _));

        // the surface keeps its own copy of the plate
        aPlate.Init();
        Assert.Equal(0.3, aSurf.Value(0.5, 0.5).Z, 12);

        // bounds: settable while unfrozen, a twin otherwise; Copy doesn't keep them, as OCCT's
        aSurf.SetBounds(0, 1, 0, 1);
        aSurf.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 1.0, 0.0, 1.0), (u1, u2, v1, v2));
        aSurf.Freeze();
        EngineAssert.Throws<ImmutableObjectException>(() => aSurf.SetBounds(0, 2, 0, 2));
        PlateSurface aWider = aSurf.WithBounds(-1, 2, -1, 2);
        aWider.ParameterBounds(out u1, out u2, out _, out _);
        Assert.Equal((-1.0, 2.0), (u1, u2));
        ((PlateSurface)aSurf.Copy()).ParameterBounds(out u1, out u2, out _, out _);
        Assert.Equal((0.0, 0.0), (u1, u2));
        EngineAssert.Throws<FailureException>(() => aSurf.WithBounds(1, 0, 0, 1));
        EngineAssert.Throws<FailureException>(() => aSurf.WithBounds(0, 0, 0, 1));
        aSurf.RealBounds(out u1, out u2, out v1, out v2);
        Assert.Equal((0.0, 1.0, 0.0, 1.0), (u1, u2, v1, v2));
        var aSeq = new NCollection_Sequence<Xy>();
        aSurf.Constraints(aSeq);
        Assert.Equal(5, aSeq.Length());
        Assert.Null(aSurf.UIso(0.5));
        EngineAssert.Throws<FailureException>(() => aSurf.IsCNu(1));
    }

    [Fact]
    public void The_plate_surface_evaluates_the_same_from_several_threads()
    {
        var aSurf = new PlateSurface(XYPlane(), SolvedPlate());
        aSurf.Freeze();
        double[] aSerial = Enumerable.Range(0, 400).Select(i => aSurf.Value(i / 400.0, (i * 7 % 400) / 400.0).Z).ToArray();
        double[] aParallel = new double[400];
        Parallel.For(0, 400, i => aParallel[i] = aSurf.Value(i / 400.0, (i * 7 % 400) / 400.0).Z);
        Assert.Equal(aSerial, aParallel);
    }

    [Fact]
    public void The_average_plane_of_planar_points_is_their_plane()
    {
        var aPts = new NCollection_HArray1<Point3D>(1, 6);
        Point3D[] aValues = [new(0, 0, 1), new(4, 0, 1), new(4, 3, 1), new(0, 3, 1), new(2, 1, 1), new(1, 2, 1)];
        for (int i = 1; i <= 6; i++)
        {
            aPts.SetValue(i, aValues[i - 1]);
        }
        var aBAP = new BuildAveragePlane(aPts, 4, 1e-7, 1, 1);
        Assert.True(aBAP.IsPlane());
        Assert.False(aBAP.IsLine());
        PlaneSurface aPlane = aBAP.Plane()!;
        Assert.True(aPlane.IsFrozen);
        Assert.Equal(1.0, Math.Abs(aPlane.Plane.Axis.Direction.Z), 12);
        Assert.Equal(1.0, aPlane.Plane.Location.Z, 12);
        EngineAssert.Throws<NoSuchObjectException>(() => aBAP.Line());
        aBAP.MinMaxBox(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal(5.0, Math.Sqrt((u2 - u1) * (u2 - u1) + (v2 - v1) * (v2 - v1)), 9);

        // aligned points: a plane through their line (a line only where BasePlan's axes degenerate)
        var aLinePts = new NCollection_HArray1<Point3D>(1, 3);
        aLinePts.SetValue(1, new Point3D(0, 0, 0));
        aLinePts.SetValue(2, new Point3D(1, 1, 1));
        aLinePts.SetValue(3, new Point3D(2, 2, 2));
        var aLine = new BuildAveragePlane(aLinePts, 0, 1e-7, 2, 1);
        Assert.NotEqual(aLine.IsPlane(), aLine.IsLine());
        Assert.Equal(0.0, aLine.Plane()!.Plane.DistanceTo(new Point3D(2, 2, 2)), 9);

        // from normals
        var aNormals = new NCollection_Sequence<Vector3D>();
        aNormals.Append(new Vector3D(0, 0.2, 1).Normalized());
        aNormals.Append(new Vector3D(0, -0.2, 1).Normalized());
        aNormals.Append(new Vector3D(0.2, 0, 1).Normalized());
        var aFromNormals = new BuildAveragePlane(aNormals, aPts);
        Assert.Equal(1.0, Math.Abs(aFromNormals.Plane()!.Plane.Axis.Direction.Z), 3);
    }

    [Fact]
    public void HalfSpace_accepts_normals_in_a_half_space_and_restores_on_failure()
    {
        var aNormals = new NCollection_Sequence<Vector3D>();
        var aBset = new NCollection_Sequence<Aij>();
        var aNew = new NCollection_Sequence<Vector3D>();
        aNew.Append(new Vector3D(0, 0, 1));
        aNew.Append(new Vector3D(1, 0, 1).Normalized());
        Assert.True(BuildAveragePlane.HalfSpace(aNew, aNormals, aBset, 1e-3, 1e-3));
        Assert.Equal(2, aNormals.Length());
        Assert.Equal(2, aBset.Length());

        // an opposite normal leaves no half-space: both sequences are restored
        var anOpposite = new NCollection_Sequence<Vector3D>();
        anOpposite.Append(new Vector3D(-1, 0, -1).Normalized());
        anOpposite.Append(new Vector3D(0, 0, -1));
        Assert.False(BuildAveragePlane.HalfSpace(anOpposite, aNormals, aBset, 1e-3, 1e-3));
        Assert.Equal(2, aNormals.Length());
        Assert.Equal(2, aBset.Length());
    }

    [Fact]
    public void MakeApprox_approximates_a_plate_surface_with_each_criterion()
    {
        var aSurf = new PlateSurface(XYPlane(), SolvedPlate());
        aSurf.SetBounds(0, 1, 0, 1);
        foreach (int aCrit in new[] { -1, 0, 1 })
        {
            var anApprox = new MakeApprox(aSurf, 1e-4, 4, 6, 1e-5, aCrit, Continuity.C1, 1.0);
            BSplineSurface aBS = anApprox.Surface()!;
            Assert.True(aBS.IsFrozen);
            Assert.True(anApprox.ApproxError() < 1e-3, $"{aCrit}: {anApprox.ApproxError()}");
            Assert.Equal(aSurf.Value(0.4, 0.7).Z, aBS.Value(0.4, 0.7).Z, 3);
            if (aCrit == -1)
            {
                Assert.Equal(0.0, anApprox.CriterionError());
            }
        }
        // another order leaves the surface null, as OCCT
        Assert.Null(new MakeApprox(aSurf, 1e-4, 4, 6, 1e-5, 2).Surface());
    }
}
