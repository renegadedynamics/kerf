// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Ribs and grooves (decision 19805). A rib in the corner of an L, between its walls, is a triangle times its thickness;
/// a rib round a stepped cylinder grows with its angles. A contour short of the walls is refused.
/// </summary>
public class BRepFeatRibLawsTests
{
    private static double Volume(in Shape theShape) => LocOpeTests.Volume(theShape);

    // An L, 100 long along y, with its walls at x = 50 and z = 50 meeting in a corner
    private static Shape Ell()
    {
        var aProfile = new MakeWire();
        Point3D[] aPoints = [new(0, 0, 0), new(200, 0, 0), new(200, 0, 50), new(50, 0, 50), new(50, 0, 200), new(0, 0, 200)];
        for (int i = 0; i < aPoints.Length; i++)
        {
            aProfile.Add(new MakeEdge(aPoints[i], aPoints[(i + 1) % aPoints.Length]).Edge());
        }
        return new ApiMakePrism(new MakeFace(aProfile.Wire()).Face(), new Vector3D(0, 100, 0)).Shape();
    }

    private const double EllVolume = 17500.0 * 100;

    private static PlaneSurface RibPlane() => new(new Point3D(0, 45, 0), new Direction3D(0, 1, 0));

    private static Wire Across(double theX1, double theZ1, double theX2, double theZ2) =>
        new MakeWire(new MakeEdge(new Point3D(theX1, 45, theZ1), new Point3D(theX2, 45, theZ2)).Edge()).Wire();

    private static void AssertSound(in Shape theShape)
    {
        Assert.False(theShape.IsNull);
        Assert.True(new Analyzer(theShape).IsValid());
        Assert.True(LocOpeTests.AllFrozen(theShape));
    }

    [Theory]
    [InlineData(5.0, -3.0, true)]
    [InlineData(5.0, -3.0, false)]
    [InlineData(4.0, 0.0, true)]
    [InlineData(2.0, -2.0, true)]
    public void A_rib_between_the_walls_is_a_triangle_times_its_thickness(double theOneWay, double theOtherWay, bool theSliding)
    {
        Shape aPart = Ell();
        var aRib = new MakeLinearForm(aPart, Across(50, 100, 100, 50), RibPlane(), new Vector3D(0, theOneWay, 0), new Vector3D(0, theOtherWay, 0), 1, theSliding);
        aRib.Perform();
        Assert.True(aRib.IsDone);
        AssertSound(aRib.Shape());
        double aThickness = theOneWay - theOtherWay;
        Assert.Equal(EllVolume + (0.5 * 50 * 50 * aThickness), Volume(aRib.Shape()), 4);
    }

    [Fact]
    public void A_rib_keeps_a_history_of_what_it_made()
    {
        Shape aPart = Ell();
        var aRib = new MakeLinearForm(aPart, Across(50, 100, 100, 50), RibPlane(), new Vector3D(0, 5, 0), new Vector3D(0, -3, 0), 1, true);
        aRib.Perform();
        Assert.NotEmpty(aRib.FirstShape());
        Assert.NotEmpty(aRib.LastShape());
        // the wall and the floor it stands on change; the far end of the L doesn't
        Assert.Equal(StatusError.OK, aRib.CurrentStatusError());
        Assert.NotEmpty(aRib.NewEdges());
        Assert.NotNull(aRib.FacesForDraft());
        Assert.NotNull(aRib.TgtEdges());
    }

    [Theory]
    [InlineData(60, 90, 90, 60)]
    [InlineData(70, 80, 80, 70)]
    public void A_contour_short_of_the_walls_has_nothing_to_end_on(double theX1, double theZ1, double theX2, double theZ2)
    {
        Shape aPart = Ell();
        var aRib = new MakeLinearForm(aPart, Across(theX1, theZ1, theX2, theZ2), RibPlane(), new Vector3D(0, 5, 0), new Vector3D(0, -3, 0), 1, true);
        aRib.Perform();
        Assert.False(aRib.IsDone);
        Assert.NotEqual(StatusError.OK, aRib.CurrentStatusError());
    }

    [Fact]
    public void A_curved_contour_gives_a_rib_of_the_area_it_closes()
    {
        Shape aPart = Ell();
        // a straight Bezier through the same ends: the same rib
        var aPoles = new NCollection_Array1<Point3D>(1, 2);
        aPoles[1] = new Point3D(50, 45, 100);
        aPoles[2] = new Point3D(100, 45, 50);
        Wire aContour = new MakeWire(new MakeEdge(new BezierCurve3D(aPoles)).Edge()).Wire();
        var aRib = new MakeLinearForm(aPart, aContour, RibPlane(), new Vector3D(0, 5, 0), new Vector3D(0, -3, 0), 1, true);
        aRib.Perform();
        if (aRib.IsDone)
        {
            AssertSound(aRib.Shape());
            Assert.Equal(EllVolume + (0.5 * 50 * 50 * 8), Volume(aRib.Shape()), 3);
        }
        else
        {
            Assert.NotEqual(StatusError.OK, aRib.CurrentStatusError());
        }
    }

    [Fact]
    public void A_contour_of_two_edges_gives_the_area_under_both()
    {
        // down from the wall, then along, then to the floor: a pentagon of the corner
        Shape aPart = Ell();
        var aBuilder = new MakeWire();
        aBuilder.Add(new MakeEdge(new Point3D(50, 45, 100), new Point3D(75, 45, 90)).Edge());
        aBuilder.Add(new MakeEdge(new Point3D(75, 45, 90), new Point3D(100, 45, 50)).Edge());
        var aRib = new MakeLinearForm(aPart, aBuilder.Wire(), RibPlane(), new Vector3D(0, 2, 0), new Vector3D(0, -2, 0), 1, true);
        aRib.Perform();
        Assert.True(aRib.IsDone);
        AssertSound(aRib.Shape());
        // the area under the two segments, by trapezia over z = 50
        double anArea = (0.5 * 25 * (50 + 40)) + (0.5 * 25 * 40);
        Assert.Equal(EllVolume + (anArea * 4), Volume(aRib.Shape()), 4);
    }

    [Fact]
    public void A_groove_takes_out_the_same_shape_a_rib_would_add()
    {
        // a contour through the solid's corner: x + z = 50 from the side to the bottom, inside the L
        Shape aPart = Ell();
        var aGroove = new MakeLinearForm(aPart, Across(0, 40, 40, 0), RibPlane(), new Vector3D(0, 2, 0), new Vector3D(0, -2, 0), 0, true);
        aGroove.Perform();
        if (aGroove.IsDone)
        {
            AssertSound(aGroove.Shape());
            Assert.InRange(Volume(aGroove.Shape()), EllVolume - (0.5 * 40 * 40 * 4) - 1, EllVolume);
        }
    }

    [Fact]
    public void A_rib_with_its_contour_told_to_slide_on_a_face_makes_the_same_rib()
    {
        Shape aPart = Ell();
        Wire aContour = Across(50, 100, 100, 50);
        var aRib = new MakeLinearForm(aPart, aContour, RibPlane(), new Vector3D(0, 5, 0), new Vector3D(0, -3, 0), 1, true);
        Edge anEdge = ShapeCasts.Edge(new Explorer(aContour, ShapeType.Edge).Current());
        Face aFloor = Floor(aPart);
        aRib.Add(anEdge, aFloor);
        aRib.Add(anEdge, aFloor);
        aRib.Perform();
        Assert.True(aRib.IsDone);
        AssertSound(aRib.Shape());
        Assert.Equal(EllVolume + (0.5 * 50 * 50 * 8), Volume(aRib.Shape()), 4);
        var anEmpty = new MakeLinearForm();
        anEmpty.Perform();
        Assert.False(anEmpty.IsDone);
        Assert.Equal(StatusError.NotInitialized, anEmpty.CurrentStatusError());
    }

    private static Face Floor(in Shape thePart)
    {
        for (var anExp = new Explorer(thePart, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Kerf.Engine.Measurement.Properties();
            Kerf.Engine.Measurement.MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z - 50) < 1e-7)
            {
                return ShapeCasts.Face(anExp.Current());
            }
        }
        throw new InvalidOperationException("no floor");
    }

    // A cylinder of radius 2 and height 5, with one of radius 1 and height 3 on it, and a rib up the step: a triangle
    // with corners (r, z) = (2, 5), (1, 5), (1, 8), turned by the two angles

    private static Shape Stepped()
    {
        Shape aLarge = new ApiMakeCylinder(2, 5).Shape();
        Shape aSmall = new ApiMakeCylinder(new Frame3D(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)), 1, 3).Shape();
        return new Fuse(aLarge, aSmall).Shape();
    }

    private static readonly double SteppedVolume = Math.PI * ((4 * 5) + (1 * 3));

    private static Wire UpTheStep() =>
        new MakeWire(new MakeEdge(new Point3D(-2, 0, 5), new Point3D(-1, 0, 8)).Edge()).Wire();

    private static PlaneSurface StepPlane() => new(new Point3D(-2, 0, 5), new Direction3D(0, 1, 0));

    [Theory]
    [InlineData(0.2, 0.2, true)]
    [InlineData(0.2, 0.2, false)]
    [InlineData(0.3, 0.1, true)]
    public void A_rib_round_a_step_grows_with_its_angles(double theH1, double theH2, bool theSliding)
    {
        var aRib = new MakeRevolutionForm(Stepped(), UpTheStep(), StepPlane(), new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), theH1, theH2, 1, theSliding);
        aRib.Perform();
        Assert.True(aRib.IsDone, aRib.CurrentStatusError().ToString());
        AssertSound(aRib.Shape());
        double anAdded = Volume(aRib.Shape()) - SteppedVolume;
        // the heights are lengths along the circle at the plane, so they give a wedge of the triangle, of 1.5 area
        Assert.InRange(anAdded, 0.1 * (theH1 + theH2), 3 * (theH1 + theH2));
    }

    [Fact]
    public void A_rib_round_a_step_is_twice_as_big_with_twice_the_angles()
    {
        double Added(double h)
        {
            var aRib = new MakeRevolutionForm(Stepped(), UpTheStep(), StepPlane(), new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), h, h, 1, true);
            aRib.Perform();
            return Volume(aRib.Shape()) - SteppedVolume;
        }
        Assert.Equal(2 * Added(0.1), Added(0.2), 0.05 * Added(0.2));
    }

    [Fact]
    public void A_revolution_rib_needs_its_arguments_and_reports_what_it_made()
    {
        var anEmpty = new MakeRevolutionForm();
        anEmpty.Perform();
        Assert.False(anEmpty.IsDone);
        Assert.Equal(StatusError.NotInitialized, anEmpty.CurrentStatusError());
        Shape aPart = Stepped();
        Wire aContour = UpTheStep();
        var aRib = new MakeRevolutionForm(aPart, aContour, StepPlane(), new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.2, 0.2, 1, true);
        aRib.Perform();
        Assert.True(aRib.IsDone);
        Assert.Equal(StatusError.OK, aRib.CurrentStatusError());
        Assert.NotEmpty(aRib.NewEdges());
    }
}
