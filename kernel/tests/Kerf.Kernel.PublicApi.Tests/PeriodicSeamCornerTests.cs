// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// A periodic B-spline with a corner at its seam reads C1, its continuity being its inner knots' as OCCT's (decision
/// 35107). The public operations that read it, run on it and on the same curve clamped (whose corner is a vertex and
/// whose continuity is right): each gives the clamped curve's result, or fails as the clamped one does.
/// </summary>
public class PeriodicSeamCornerTests
{
    private static readonly MassOptions Measured = new() { Tolerance = 1e-9 };

    private static BSplineCurve3D Periodic() =>
        new([new(0, 0, 0), new(4, 0, 0), new(4, 4, 0), new(0, 4, 0)], [0.0, 1.0, 2.0, 3.0], [2, 1, 1, 2], 2, true);

    private static Face FaceOf(BSplineCurve3D theCurve) =>
        ShapeFactory.FromWire(ShapeFactory.FromEdges([ShapeFactory.FromCurve(theCurve).Value]).Value).Value;

    private static double Length(Result<Built<Shape>> theResult)
    {
        Assert.True(theResult.IsSuccess, theResult.Error?.Message);
        Assert.True(theResult.Value.Shape.Validate().IsValid);
        return theResult.Value.Shape.IntegrateLength(Measured);
    }

    private static double Volume(Result<Built<Shape>> theResult)
    {
        Assert.True(theResult.IsSuccess, theResult.Error?.Message);
        Assert.True(theResult.Value.Shape.Validate().IsValid);
        return theResult.Value.Shape.IntegrateVolume(Measured);
    }

    [Fact]
    public void Offsets_and_fillets_of_a_closed_curve_with_a_corner_at_its_seam_are_the_clamped_curves()
    {
        BSplineCurve3D aPeriodic = Periodic();
        BSplineCurve3D aClamped = aPeriodic.WithNotPeriodic();
        Assert.Equal(Continuity.C1, aPeriodic.Continuity);
        Face aFace = FaceOf(aPeriodic), aClampedFace = FaceOf(aClamped);

        // The 2D offsets (an offset curve over the whole period: the corner is the edge's vertex)
        foreach (double d in new[] { 0.3, -0.3 })
        {
            Assert.Equal(Length(aClampedFace.Offset2D(d)), Length(aFace.Offset2D(d)), 1e-9);
        }

        // The prism's offset, its side face's offset, and the fillet of its top edge
        Shape aSolid = aFace.Extrude(new Vector3D(0, 0, 3)).Value.Shape;
        Shape aClampedSolid = aClampedFace.Extrude(new Vector3D(0, 0, 3)).Value.Shape;
        Assert.Equal(Volume(aClampedSolid.Offset(0.3)), Volume(aSolid.Offset(0.3)), 1e-9);
        Face aSide = aSolid.AllFaces().Single(f => f.ToSurface().Kind != SurfaceKind.Plane);
        Face aClampedSide = aClampedSolid.AllFaces().Single(f => f.ToSurface().Kind != SurfaceKind.Plane);
        foreach (double d in new[] { 0.3, -0.3 })
        {
            Assert.Equal(Volume(((Shape)aClampedSide).Offset(d)), Volume(((Shape)aSide).Offset(d)), 1e-9);
        }
        Edge Top(Shape s) => s.AllEdges().Single(e => e.ToCurve().Kind == CurveKind.BSplineCurve && e.ToCurve().Evaluate(e.ToCurve().Range.Min).Z > 2.9);
        Assert.Equal(Volume(aClampedSolid.Fillet([Top(aClampedSolid)], 0.3)), Volume(aSolid.Fillet([Top(aSolid)], 0.3)), 1e-9);

        // A sweep along it fails as along the clamped curve
        Wire aProfile = ShapeFactory.FromEdges([ShapeFactory.Circle(new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0.05)).Value]).Value;
        foreach (TransitionMode aMode in Enum.GetValues<TransitionMode>())
        {
            var anOptions = new SweepOptions { MakeSolid = true, Transition = aMode };
            Assert.Equal(Sweeps.Sweep(aProfile, aClampedFace.OuterWire, anOptions).IsSuccess, Sweeps.Sweep(aProfile, aFace.OuterWire, anOptions).IsSuccess);
        }
    }

    [Fact]
    public void An_edge_across_the_seams_corner_is_measured_converted_and_discretised_with_the_corner()
    {
        BSplineCurve3D aPeriodic = Periodic();
        BSplineCurve3D aClamped = aPeriodic.WithNotPeriodic();
        Edge anAcross = ShapeFactory.FromCurve(aPeriodic, new Interval(2.5, 3.5)).Value;
        double aPieces = ShapeFactory.FromCurve(aClamped, new Interval(2.5, 3)).Value.IntegrateLength(Measured)
                         + ShapeFactory.FromCurve(aClamped, new Interval(0, 0.5)).Value.IntegrateLength(Measured);
        Assert.Equal(aPieces, anAcross.IntegrateLength(Measured), 1e-9);
        Assert.Equal(aPieces, ((Shape)anAcross).ConvertToBSpline().Value.Shape.IntegrateLength(Measured), 1e-9);
        // The discretisation stops at the corner
        Assert.Contains(anAcross.ToCurve().Discretize(1e-3), u => Math.Abs(u - 3.0) < 1e-9);
    }
}
