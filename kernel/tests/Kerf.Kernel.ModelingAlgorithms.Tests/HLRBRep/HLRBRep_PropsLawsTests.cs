// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The hidden-line algorithm's local properties by the geometry they measure (unit 12Q, decision 1753): a sphere's
/// curvature is one over its radius every way, a cylinder's one over its radius round it and none along it, a torus's
/// at its outer equator one over the tube's radius and one over the ring's; a projected circle's curvature is one over
/// its radius, centred at its centre. The hidden-line algorithm reads only the normal of these: the rest is theirs.
/// </summary>
public class HLRBRep_PropsLawsTests
{
    private static readonly AlgorithmProjector Top = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    private static HlrSurface Surface(Shape theSolid, Func<Surface, bool> theWhich)
    {
        Face aFace = BRepOffset_MakeOffsetTests.SubShapes(theSolid, ShapeType.Face).Select(f => ShapeCasts.Face(f))
                                               .First(f => theWhich(global::Kerf.Engine.Topology.Brep.Tool.Surface(f)!));
        var aSurface = new HlrSurface();
        aSurface.Projector(new StrongBox<AlgorithmProjector>(Top));
        aSurface.Surface(aFace);
        return aSurface;
    }

    [Theory]
    [InlineData(0.3, 0.2)]
    [InlineData(2.0, -0.7)]
    [InlineData(4.0, 1.1)]
    public void A_sphere_curves_alike_every_way(double u, double v)
    {
        HlrSurface aSphere = Surface(new MakeSphere(5).Shape(), s => s is SphericalSurface);
        var aProps = new HiddenLineSLProps(aSphere, u, v, 2, 1e-9);
        Point3D aP = aProps.Value();
        Assert.Equal(5, aP.DistanceTo(Point3D.Origin), 9);
        Assert.True(aProps.IsTangentUDefined() && aProps.IsTangentVDefined() && aProps.IsNormalDefined);
        aProps.TangentU(out Direction3D aTu);
        aProps.TangentV(out Direction3D aTv);
        Direction3D aN = aProps.Normal();
        // the normal is along the radius, the tangents across it
        Assert.True(aN.IsParallel(new Direction3D(aP.X, aP.Y, aP.Z), 1e-9));
        Assert.Equal(0, aTu.Dot(aN), 9);
        Assert.Equal(0, aTv.Dot(aN), 9);
        Assert.Equal(0, aProps.D1U().Dot(new Vector3D(aN)), 9);
        Assert.True(aProps.IsCurvatureDefined());
        // umbilic in fact; OCCT's IsUmbilic asks the two to be equal to the last bit, which rounding breaks here
        // (occt-issues.md), so the test is on the curvatures
        Assert.Equal(aProps.MaxCurvature(), aProps.MinCurvature(), 12);
        Assert.Equal(0.2, Math.Abs(aProps.MaxCurvature()), 9);
        Assert.Equal(0.2, Math.Abs(aProps.MinCurvature()), 9);
        Assert.Equal(0.2, Math.Abs(aProps.MeanCurvature()), 9);
        Assert.Equal(0.04, aProps.GaussianCurvature(), 9);
    }

    [Theory]
    [InlineData(0.5, 3.0)]
    [InlineData(4.0, 7.0)]
    public void A_cylinder_curves_round_its_axis_and_not_along_it(double u, double v)
    {
        HlrSurface aCylinder = Surface(new ApiMakeCylinder(2, 10).Shape(), s => s is CylindricalSurface);
        var aProps = new HiddenLineSLProps(aCylinder, 2, 1e-9);
        aProps.SetParameters(u, v);
        Assert.Equal(2, Math.Sqrt((aProps.Value().X * aProps.Value().X) + (aProps.Value().Y * aProps.Value().Y)), 9);
        Assert.True(aProps.IsCurvatureDefined());
        Assert.False(aProps.IsUmbilic());
        Assert.Equal(0.5, Math.Max(Math.Abs(aProps.MaxCurvature()), Math.Abs(aProps.MinCurvature())), 9);
        Assert.Equal(0, Math.Min(Math.Abs(aProps.MaxCurvature()), Math.Abs(aProps.MinCurvature())), 9);
        Assert.Equal(0.25, Math.Abs(aProps.MeanCurvature()), 9);
        Assert.Equal(0, aProps.GaussianCurvature(), 9);
        // the direction of no curvature is the axis
        aProps.CurvatureDirections(out Direction3D aMax, out Direction3D aMin);
        Direction3D aFlat = Math.Abs(aProps.MaxCurvature()) < Math.Abs(aProps.MinCurvature()) ? aMax : aMin;
        Assert.True(aFlat.IsParallel(new Direction3D(0, 0, 1), 1e-9));
        // second derivatives: none along the axis, the radius inwards round it
        Assert.Equal(0, aProps.D2V().Magnitude, 9);
        Assert.Equal(0, aProps.DUV().Magnitude, 9);
        Assert.Equal(2, aProps.D2U().Magnitude, 9);
    }

    [Fact]
    public void A_torus_at_its_outer_equator_curves_by_its_tube_and_its_ring()
    {
        // ring 6, tube 2: at the outer equator, 1/2 round the tube and 1/8 round the axis, the same way
        HlrSurface aTorus = Surface(new MakeTorus(6, 2).Shape(), s => s is ToroidalSurface);
        var aProps = new HiddenLineSLProps(aTorus, 1.0, 0.0, 2, 1e-9);
        Assert.Equal(8, Math.Sqrt((aProps.Value().X * aProps.Value().X) + (aProps.Value().Y * aProps.Value().Y)), 9);
        double[] aCurvatures = [Math.Abs(aProps.MaxCurvature()), Math.Abs(aProps.MinCurvature())];
        Array.Sort(aCurvatures);
        Assert.Equal(0.125, aCurvatures[0], 9);
        Assert.Equal(0.5, aCurvatures[1], 9);
        Assert.Equal(1.0 / 16, aProps.GaussianCurvature(), 9);
        Assert.False(aProps.IsUmbilic());
        // and at the inner equator the two curve opposite ways
        aProps.SetParameters(1.0, Math.PI);
        Assert.Equal(-1.0 / 8, aProps.GaussianCurvature(), 9);
    }

    [Fact]
    public void A_plane_has_no_curvature()
    {
        HlrSurface aPlane = Surface(new MakeBox(1, 2, 3).Shape(), s => s is PlaneSurface);
        var aProps = new HiddenLineSLProps(aPlane, 0.5, 0.5, 2, 1e-9);
        Assert.True(aProps.IsCurvatureDefined());
        Assert.True(aProps.IsUmbilic());
        Assert.Equal(0, aProps.MaxCurvature(), 12);
        Assert.Equal(0, aProps.GaussianCurvature(), 12);
        Assert.Equal(0, aProps.D2U().Magnitude, 12);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(1.3)]
    [InlineData(4.0)]
    public void A_circle_seen_from_above_curves_by_its_radius_about_its_centre(double u)
    {
        Edge aCircle = new MakeEdge(new Circle3D(new Frame3D(new Point3D(1, 2, 0), new Direction3D(0, 0, 1)), 5)).Edge();
        var aCurve = new HiddenLineCurve();
        aCurve.Projector(new StrongBox<AlgorithmProjector>(Top));
        aCurve.Curve(aCircle);
        var aProps = new HiddenLineCLProps(aCurve, u, 2, 1e-9);
        Point2D aP = aProps.Value();
        Assert.Equal(5, aP.DistanceTo(new Point2D(1, 2)), 9);
        Assert.True(aProps.IsTangentDefined());
        aProps.Tangent(out Direction2D aT);
        Assert.Equal(0, aT.Dot(new Direction2D(new Vector2D(new Point2D(1, 2), aP))), 9);
        Assert.Equal(0.2, aProps.Curvature(), 9);
        aProps.Normal(out Direction2D aN);
        Assert.True(aN.IsParallel(new Direction2D(new Vector2D(aP, new Point2D(1, 2))), 1e-9));
        aProps.CentreOfCurvature(out Point2D aC);
        Assert.True(aC.DistanceTo(new Point2D(1, 2)) < 1e-9);
        Assert.Equal(5, aProps.D1().Magnitude, 9);
        Assert.Equal(5, aProps.D2().Magnitude, 9);
    }

    [Fact]
    public void A_line_seen_from_above_has_no_curvature()
    {
        Edge aLine = new MakeEdge(new Point3D(0, 0, 0), new Point3D(3, 4, 7)).Edge();
        var aCurve = new HiddenLineCurve();
        aCurve.Projector(new StrongBox<AlgorithmProjector>(Top));
        aCurve.Curve(aLine);
        var aProps = new HiddenLineCLProps(2, 1e-9);
        aProps.SetCurve(aCurve);
        aProps.SetParameter(0.5 * (aCurve.FirstParameter() + aCurve.LastParameter()));
        Assert.True(aProps.IsTangentDefined());
        aProps.Tangent(out Direction2D aT);
        Assert.True(aT.IsParallel(new Direction2D(3, 4), 1e-9));
        Assert.Equal(0, aProps.Curvature(), 12);
    }
}
