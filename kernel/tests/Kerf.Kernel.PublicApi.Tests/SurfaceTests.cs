// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Surfaces and the analytic values behind them: evaluation, bounds, kinds, grids of poles.</summary>
public class SurfaceTests
{
    [Fact]
    public void A_plane_evaluates_and_is_unbounded()
    {
        var plane = new PlaneSurface(new Plane3D(new Point3D(0, 0, 5), Direction3D.UnitZ));
        Assert.Equal(new Point3D(1, 2, 5), plane.Evaluate(1, 2));
        Assert.Equal(SurfaceKind.Plane, plane.Kind);
        (Interval u, Interval v) = plane.Bounds;
        Assert.True(u.IsInfinite && v.IsInfinite);
        Assert.Equal(5, plane.Plane.Location.Z);
        SurfaceDerivatives d = plane.Derivatives(0, 0, 1);
        Assert.Equal(new Vector3D(1, 0, 0), d.DU);
        Assert.Equal(new Vector3D(0, 1, 0), d.DV);
    }

    [Fact]
    public void A_cylinder_is_periodic_in_u()
    {
        var cylinder = new CylindricalSurface(CoordinateSystem.World, 2);
        Assert.Equal(SurfaceKind.Cylinder, cylinder.Kind);
        Assert.Equal(2, cylinder.Radius);
        Assert.True(cylinder.IsUPeriodic);
        Assert.False(cylinder.IsVPeriodic);
        Assert.Equal(2 * Math.PI, cylinder.UPeriod, 12);
        Assert.True(cylinder.Evaluate(Math.PI / 2, 3).IsEqual(new Point3D(0, 2, 3), 1e-12));
        Assert.Equal(new Interval(0, 2 * Math.PI), cylinder.Bounds.U);
        Assert.Equal(2, cylinder.Cylinder.Radius);
        SurfaceDerivatives d = cylinder.Derivatives(0, 0, 2);
        Assert.True(d.DUU.IsEqual(new Vector3D(-2, 0, 0), 1e-12, 1e-12));
        Assert.True(cylinder.DerivativeN(0, 0, 0, 1).IsEqual(new Vector3D(0, 0, 1), 1e-12, 1e-12));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => cylinder.DerivativeN(0, 0, 0, 0));
    }

    [Fact]
    public void A_sphere_knows_its_area_and_volume()
    {
        var sphere = new SphericalSurface(CoordinateSystem.World, 1);
        Assert.Equal(SurfaceKind.Sphere, sphere.Kind);
        Assert.Equal(4 * Math.PI, sphere.Area, 12);
        Assert.Equal(4.0 / 3.0 * Math.PI, sphere.Volume, 12);
        Assert.True(sphere.Evaluate(0, Math.PI / 2).IsEqual(new Point3D(0, 0, 1), 1e-12));
    }

    [Fact]
    public void A_bspline_surface_is_built_from_a_grid()
    {
        Point3D[] values = [new(0, 0, 0), new(0, 1, 0), new(1, 0, 0), new(1, 1, 1)];
        var poles = new Grid<Point3D>(values, 2, 2);
        var surface = new BSplineSurface(poles, [0, 1], [0, 1], [2, 2], [2, 2], 1, 1);
        Assert.Equal(1, surface.UDegree);
        Assert.Equal(1, surface.VDegree);
        Assert.Equal(new Point3D(1, 1, 1), surface.Poles[1, 1]);
        Assert.Equal(1, surface.Weights[0, 0]);
        Assert.Equal(SurfaceKind.BSplineSurface, surface.Kind);
        Assert.True(surface.Evaluate(0.5, 0.5).IsEqual(new Point3D(0.5, 0.5, 0.25), 1e-12));
        Assert.Equal([0.0, 1.0], surface.UKnots);
    }

    [Fact]
    public void A_bezier_surface_is_built_from_a_grid()
    {
        Point3D[] values = [new(0, 0, 0), new(0, 1, 0), new(1, 0, 0), new(1, 1, 0)];
        var surface = new BezierSurface(new Grid<Point3D>(values, 2, 2));
        Assert.Equal(SurfaceKind.BezierSurface, surface.Kind);
        Assert.Equal(new Point3D(0.5, 0.5, 0), surface.Evaluate(0.5, 0.5));
        KernelAssert.Throws<ArgumentException>(() => new BezierSurface(new Grid<Point3D>(new[] { Point3D.Origin }, 1, 1)));
    }

    [Fact]
    public void A_trimmed_surface_is_its_basis_surfaces_kind()
    {
        var plane = new PlaneSurface(Plane3DAtOrigin());
        var trimmed = new TrimmedSurface(plane, 0, 1, 0, 2);
        Assert.Equal(SurfaceKind.Plane, trimmed.Kind);
        Assert.Equal((new Interval(0, 1), new Interval(0, 2)), trimmed.Bounds);
        Assert.IsType<PlaneSurface>(trimmed.BasisSurface);
    }

    [Fact]
    public void The_analytic_values_measure()
    {
        var plane = new Plane3D(new Point3D(0, 0, 1), Direction3D.UnitZ);
        Assert.Equal(2, plane.DistanceTo(new Point3D(5, 5, 3)));
        Assert.Equal(new Point3D(0, 0, 1), plane.Location);
        var circle = new Circle3D(Frame3D.World, 2);
        Assert.Equal(4 * Math.PI, circle.Length, 12);
        Assert.Equal(4 * Math.PI, circle.Area, 12);
        Assert.Equal(1, circle.DistanceTo(new Point3D(3, 0, 0)), 12);
        var line = new Line3D(Point3D.Origin, Direction3D.UnitX);
        Assert.Equal(3, line.DistanceTo(new Point3D(5, 3, 0)), 12);
        var torus = new Torus3D(CoordinateSystem.World, 3, 1);
        Assert.Equal(3, torus.MajorRadius);
        Assert.Equal(1, torus.MinorRadius);
        var cone = new Cone3D(CoordinateSystem.World, Math.PI / 4, 1);
        Assert.Equal(Math.PI / 4, cone.SemiAngle, 12);
        Assert.Equal(1, cone.RefRadius);
    }

    private static Plane3D Plane3DAtOrigin() => new(Point3D.Origin, Direction3D.UnitZ);
}
