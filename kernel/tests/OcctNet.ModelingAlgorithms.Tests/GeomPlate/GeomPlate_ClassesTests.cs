// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of GeomPlate's classes other than the builder: Aij, the point and curve constraints, the plate surface,
/// the average plane, the criteria and the approximation.
/// </summary>
public class GeomPlate_ClassesTests
{
    private static Plate_Plate SolvedPlate()
    {
        var aPlate = new Plate_Plate();
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0, 0), new gp_XYZ(0, 0, 0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1, 0), new gp_XYZ(0, 0, 0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0, 1), new gp_XYZ(0, 0, 0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(1, 1), new gp_XYZ(0, 0, 0)));
        aPlate.Load(new Plate_PinpointConstraint(new gp_XY(0.5, 0.5), new gp_XYZ(0, 0, 0.3)));
        aPlate.SolveTI(3);
        Assert.True(aPlate.IsDone());
        return aPlate;
    }

    private static Geom_Plane XYPlane() => new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));

    [Fact]
    public void Aij_compares_by_value()
    {
        var anA = new GeomPlate_Aij(1, 2, new gp_Vec(0, 0, 1));
        Assert.Equal(anA, new GeomPlate_Aij(1, 2, new gp_Vec(0, 0, 1)));
        Assert.NotEqual(anA, new GeomPlate_Aij(2, 1, new gp_Vec(0, 0, 1)));
        Assert.Equal(new GeomPlate_Aij(0, 0, new gp_Vec()), new GeomPlate_Aij());
    }

    [Fact]
    public void Point_constraints_hold_their_point_and_derivatives()
    {
        var aP = new GeomPlate_PointConstraint(new gp_Pnt(1, 2, 3), 0, 0.5);
        aP.D0(out gp_Pnt aPnt);
        Assert.Equal(new gp_Pnt(1, 2, 3), aPnt);
        Assert.Equal(0.5, aP.G0Criterion());
        Assert.False(aP.HasPnt2dOnSurf());
        aP.SetPnt2dOnSurf(new gp_Pnt2d(4, 5));
        Assert.True(aP.HasPnt2dOnSurf());
        Assert.Equal(new gp_Pnt2d(4, 5), aP.Pnt2dOnSurf());
        Assert.Throws<Standard_Failure>(() => new GeomPlate_PointConstraint(new gp_Pnt(), 2));

        var aSphere = new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2.0);
        var aS = new GeomPlate_PointConstraint(0.3, 0.4, aSphere, 2, 1e-4, 1e-3, 1e-2);
        aS.D2(out gp_Pnt aP2, out gp_Vec aD1U, out gp_Vec aD1V, out gp_Vec aD2U, out gp_Vec aD2V, out gp_Vec aD2UV);
        aSphere.D2(0.3, 0.4, out gp_Pnt aQ, out gp_Vec aE1U, out gp_Vec aE1V, out gp_Vec aE2U, out gp_Vec aE2V, out gp_Vec aE2UV);
        Assert.Equal(aQ, aP2);
        Assert.Equal(aE1U, aD1U);
        Assert.Equal(aE2UV, aD2UV);
        Assert.Equal(aE1V, aD1V);
        Assert.Equal(aE2U, aD2U);
        Assert.Equal(aE2V, aD2V);
        Assert.Equal(2, aS.Order());
        GeomLProp_SLProps aProps = aS.LPropSurf();
        Assert.True(aProps.IsCurvatureDefined()); // the curvatures are computed on demand (as OCCT's)
        Assert.Equal(0.25, aProps.MeanCurvature() * aProps.MeanCurvature(), 12);
    }

    [Fact]
    public void Curve_constraints_on_a_surface_evaluate_through_it()
    {
        var aPlane = XYPlane();
        var aLine = new Geom2d_Line(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 0));
        var aCOS = new Adaptor3d_CurveOnSurface(new Geom2dAdaptor_Curve(aLine, 0, 5), new GeomAdaptor_Surface(aPlane));
        var aC = new GeomPlate_CurveConstraint(aCOS, 1, 12);
        Assert.Equal(12, aC.NbPoints());
        Assert.Equal(0.0, aC.FirstParameter());
        Assert.Equal(5.0, aC.LastParameter());
        Assert.Equal(5.0, aC.Length(), 9);
        aC.D1(2.0, out gp_Pnt aP, out gp_Vec aV1, out gp_Vec aV2);
        Assert.Equal(new gp_Pnt(3, 1, 0), aP);
        Assert.Equal(1.0, (aV1 ^ aV2).Z());
        Assert.Equal(0.0, aC.LPropSurf(2.0).GaussianCurvature());
        Assert.Equal(0.01, aC.G1Criterion(1.0));
        aC.SetG1Criterion(new Law_Linear());
        Assert.Same(aCOS, aC.Curve3d());
        Assert.Null(aC.Curve2dOnSurf());

        // a 3D curve: only G0 queries
        var a3d = new GeomPlate_CurveConstraint(new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 0, 1), 0);
        Assert.Throws<Standard_Failure>(() => a3d.D1(0.5, out _, out _, out _));
        Assert.Throws<Standard_Failure>(() => a3d.G1Criterion(0.5));
        Assert.Throws<Standard_Failure>(() => a3d.LPropSurf(0.5));
        Assert.Throws<Standard_Failure>(() => new GeomPlate_CurveConstraint(aCOS, 3));
    }

    [Fact]
    public void The_plate_surface_is_the_initial_surface_plus_the_plate()
    {
        Plate_Plate aPlate = SolvedPlate();
        var aSurf = new GeomPlate_Surface(XYPlane(), aPlate);
        gp_Pnt aP = aSurf.Value(0.5, 0.5);
        Assert.Equal(0.3, aP.Z(), 12);
        gp_XYZ aD = aPlate.Evaluate(new gp_XY(0.25, 0.75));
        Assert.Equal(new gp_Pnt(0.25 + aD.X(), 0.75 + aD.Y(), aD.Z()), aSurf.Value(0.25, 0.75));
        aSurf.D2(0.3, 0.6, out gp_Pnt aP2, out gp_Vec aD1U, out _, out gp_Vec aD2U, out _, out _);
        Assert.Equal(aSurf.Value(0.3, 0.6), aP2);
        Assert.Equal(new gp_Vec(1, 0, 0) + new gp_Vec(aPlate.EvaluateDerivative(new gp_XY(0.3, 0.6), 1, 0)), aD1U);
        Assert.Equal(new gp_Vec(aPlate.EvaluateDerivative(new gp_XY(0.3, 0.6), 2, 0)), aD2U);
        Assert.Throws<Geom_UndefinedDerivative>(() => aSurf.D3(0.3, 0.6, out _, out _, out _, out _, out _, out _, out _, out _, out _, out _));

        // the surface keeps its own copy of the plate
        aPlate.Init();
        Assert.Equal(0.3, aSurf.Value(0.5, 0.5).Z(), 12);

        // bounds: settable while unfrozen, a twin otherwise; Copy doesn't keep them, as OCCT's
        aSurf.SetBounds(0, 1, 0, 1);
        aSurf.Bounds(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 1.0, 0.0, 1.0), (u1, u2, v1, v2));
        aSurf.Freeze();
        Assert.Throws<Standard_ImmutableObject>(() => aSurf.SetBounds(0, 2, 0, 2));
        GeomPlate_Surface aWider = aSurf.WithBounds(-1, 2, -1, 2);
        aWider.Bounds(out u1, out u2, out _, out _);
        Assert.Equal((-1.0, 2.0), (u1, u2));
        ((GeomPlate_Surface)aSurf.Copy()).Bounds(out u1, out u2, out _, out _);
        Assert.Equal((0.0, 0.0), (u1, u2));
        Assert.Throws<Standard_Failure>(() => aSurf.WithBounds(1, 0, 0, 1));
        Assert.Throws<Standard_Failure>(() => aSurf.WithBounds(0, 0, 0, 1));
        aSurf.RealBounds(out u1, out u2, out v1, out v2);
        Assert.Equal((0.0, 1.0, 0.0, 1.0), (u1, u2, v1, v2));
        var aSeq = new NCollection_Sequence<gp_XY>();
        aSurf.Constraints(aSeq);
        Assert.Equal(5, aSeq.Length());
        Assert.Null(aSurf.UIso(0.5));
        Assert.Throws<Standard_Failure>(() => aSurf.IsCNu(1));
    }

    [Fact]
    public void The_plate_surface_evaluates_the_same_from_several_threads()
    {
        var aSurf = new GeomPlate_Surface(XYPlane(), SolvedPlate());
        aSurf.Freeze();
        double[] aSerial = Enumerable.Range(0, 400).Select(i => aSurf.Value(i / 400.0, (i * 7 % 400) / 400.0).Z()).ToArray();
        double[] aParallel = new double[400];
        Parallel.For(0, 400, i => aParallel[i] = aSurf.Value(i / 400.0, (i * 7 % 400) / 400.0).Z());
        Assert.Equal(aSerial, aParallel);
    }

    [Fact]
    public void The_average_plane_of_planar_points_is_their_plane()
    {
        var aPts = new NCollection_HArray1<gp_Pnt>(1, 6);
        gp_Pnt[] aValues = [new(0, 0, 1), new(4, 0, 1), new(4, 3, 1), new(0, 3, 1), new(2, 1, 1), new(1, 2, 1)];
        for (int i = 1; i <= 6; i++)
        {
            aPts.SetValue(i, aValues[i - 1]);
        }
        var aBAP = new GeomPlate_BuildAveragePlane(aPts, 4, 1e-7, 1, 1);
        Assert.True(aBAP.IsPlane());
        Assert.False(aBAP.IsLine());
        Geom_Plane aPlane = aBAP.Plane()!;
        Assert.True(aPlane.IsFrozen());
        Assert.Equal(1.0, Math.Abs(aPlane.Pln().Axis().Direction().Z()), 12);
        Assert.Equal(1.0, aPlane.Pln().Location().Z(), 12);
        Assert.Throws<Standard_NoSuchObject>(() => aBAP.Line());
        aBAP.MinMaxBox(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal(5.0, Math.Sqrt((u2 - u1) * (u2 - u1) + (v2 - v1) * (v2 - v1)), 9);

        // aligned points: a plane through their line (a line only where BasePlan's axes degenerate)
        var aLinePts = new NCollection_HArray1<gp_Pnt>(1, 3);
        aLinePts.SetValue(1, new gp_Pnt(0, 0, 0));
        aLinePts.SetValue(2, new gp_Pnt(1, 1, 1));
        aLinePts.SetValue(3, new gp_Pnt(2, 2, 2));
        var aLine = new GeomPlate_BuildAveragePlane(aLinePts, 0, 1e-7, 2, 1);
        Assert.NotEqual(aLine.IsPlane(), aLine.IsLine());
        Assert.Equal(0.0, aLine.Plane()!.Pln().Distance(new gp_Pnt(2, 2, 2)), 9);

        // from normals
        var aNormals = new NCollection_Sequence<gp_Vec>();
        aNormals.Append(new gp_Vec(0, 0.2, 1).Normalized());
        aNormals.Append(new gp_Vec(0, -0.2, 1).Normalized());
        aNormals.Append(new gp_Vec(0.2, 0, 1).Normalized());
        var aFromNormals = new GeomPlate_BuildAveragePlane(aNormals, aPts);
        Assert.Equal(1.0, Math.Abs(aFromNormals.Plane()!.Pln().Axis().Direction().Z()), 3);
    }

    [Fact]
    public void HalfSpace_accepts_normals_in_a_half_space_and_restores_on_failure()
    {
        var aNormals = new NCollection_Sequence<gp_Vec>();
        var aBset = new NCollection_Sequence<GeomPlate_Aij>();
        var aNew = new NCollection_Sequence<gp_Vec>();
        aNew.Append(new gp_Vec(0, 0, 1));
        aNew.Append(new gp_Vec(1, 0, 1).Normalized());
        Assert.True(GeomPlate_BuildAveragePlane.HalfSpace(aNew, aNormals, aBset, 1e-3, 1e-3));
        Assert.Equal(2, aNormals.Length());
        Assert.Equal(2, aBset.Length());

        // an opposite normal leaves no half-space: both sequences are restored
        var anOpposite = new NCollection_Sequence<gp_Vec>();
        anOpposite.Append(new gp_Vec(-1, 0, -1).Normalized());
        anOpposite.Append(new gp_Vec(0, 0, -1));
        Assert.False(GeomPlate_BuildAveragePlane.HalfSpace(anOpposite, aNormals, aBset, 1e-3, 1e-3));
        Assert.Equal(2, aNormals.Length());
        Assert.Equal(2, aBset.Length());
    }

    [Fact]
    public void MakeApprox_approximates_a_plate_surface_with_each_criterion()
    {
        var aSurf = new GeomPlate_Surface(XYPlane(), SolvedPlate());
        aSurf.SetBounds(0, 1, 0, 1);
        foreach (int aCrit in new[] { -1, 0, 1 })
        {
            var anApprox = new GeomPlate_MakeApprox(aSurf, 1e-4, 4, 6, 1e-5, aCrit, GeomAbs_Shape.GeomAbs_C1, 1.0);
            Geom_BSplineSurface aBS = anApprox.Surface()!;
            Assert.True(aBS.IsFrozen());
            Assert.True(anApprox.ApproxError() < 1e-3, $"{aCrit}: {anApprox.ApproxError()}");
            Assert.Equal(aSurf.Value(0.4, 0.7).Z(), aBS.Value(0.4, 0.7).Z(), 3);
            if (aCrit == -1)
            {
                Assert.Equal(0.0, anApprox.CriterionError());
            }
        }
        // another order leaves the surface null, as OCCT
        Assert.Null(new GeomPlate_MakeApprox(aSurf, 1e-4, 4, 6, 1e-5, 2).Surface());
    }
}
