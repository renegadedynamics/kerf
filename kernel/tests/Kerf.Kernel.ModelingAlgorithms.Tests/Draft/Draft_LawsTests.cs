// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Features.Draft;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Draft angles by their volumes (decision 1753): a plane tilted about the line where it meets the neutral plane adds or
/// removes a wedge, a cylinder becomes the frustum of a cone, and what's tangent to a drafted face follows it.
/// </summary>
public class Draft_LawsTests
{
    private static readonly Direction3D Up = new(0, 0, 1);
    private static readonly Plane3D Floor = new(new Point3D(0, 0, 0), Up);

    private static double Volume(in Shape theShape) => BRepOffset_MakeOffsetTests.Volume(theShape);

    private static IEnumerable<Face> Faces(Shape theShape)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            yield return ShapeCasts.Face(anExp.Current());
        }
    }

    private static Face PlaneWithNormal(in Shape theShape, Direction3D theNormal)
    {
        foreach (Face aF in Faces(theShape))
        {
            if (Tool.Surface(aF) is PlaneSurface aPlane)
            {
                Direction3D aN = aPlane.Plane.Axis.Direction;
                if (aF.Orientation == Orientation.Reversed)
                {
                    aN = aN.Reversed();
                }
                if (aN.IsEqual(theNormal, 1.0e-9))
                {
                    return aF;
                }
            }
        }
        throw new InvalidOperationException("no such face");
    }

    private static Face Lateral(in Shape theShape) => Faces(theShape).First(f => Tool.Surface(f) is not PlaneSurface);

    private static Shape Draft(in Shape theShape, params (Face Face, double Angle, Plane3D Plane)[] theDrafts)
    {
        ShapeSnapshot aSnap = ShapeSnapshot.Of(theShape);
        var aDraft = new DraftAngle(theShape);
        foreach ((Face aFace, double anAngle, Plane3D aPlane) in theDrafts)
        {
            aDraft.Add(aFace, Up, anAngle, aPlane);
            Assert.True(aDraft.AddDone(), aDraft.Status().ToString());
        }
        aDraft.Build();
        Assert.True(aDraft.IsDone, aDraft.Status().ToString());
        aSnap.AssertUnchanged(theShape);
        Shape aResult = aDraft.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        return aResult;
    }

    [Theory]
    [InlineData(5.0)]
    [InlineData(-5.0)]
    public void Each_side_of_a_box_adds_its_own_wedge(double theDegrees)
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        double a = theDegrees * Math.PI / 180;
        Shape aResult = Draft(aBox, (PlaneWithNormal(aBox, new Direction3D(1, 0, 0)), a, Floor), (PlaneWithNormal(aBox, new Direction3D(-1, 0, 0)), a, Floor));
        // the section at height z is 10 by 10 less 2 z tan(a), over z from 0 to 10
        Assert.Equal(1000 - Math.Tan(a) * 10 * 100, Volume(aResult), 6);
    }

    [Theory]
    [InlineData(5.0)]
    [InlineData(-5.0)]
    public void All_four_sides_make_a_frustum_of_a_pyramid(double theDegrees)
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        double a = theDegrees * Math.PI / 180;
        Shape aResult = Draft(aBox,
            (PlaneWithNormal(aBox, new Direction3D(1, 0, 0)), a, Floor), (PlaneWithNormal(aBox, new Direction3D(-1, 0, 0)), a, Floor),
            (PlaneWithNormal(aBox, new Direction3D(0, 1, 0)), a, Floor), (PlaneWithNormal(aBox, new Direction3D(0, -1, 0)), a, Floor));
        double t = 10 - (2 * 10 * Math.Tan(a));
        Assert.Equal(10.0 / 3 * ((10 * 10) + (10 * t) + (t * t)), Volume(aResult), 6);
    }

    [Theory]
    [InlineData(0.0, 5.0)]
    [InlineData(5.0, 5.0)]
    [InlineData(10.0, 5.0)]
    [InlineData(5.0, -5.0)]
    public void The_neutral_plane_stays_where_it_is(double theHeight, double theDegrees)
    {
        // the side keeps its place at that height, moving in above it and out below it
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        double a = theDegrees * Math.PI / 180;
        Shape aResult = Draft(aBox, (PlaneWithNormal(aBox, new Direction3D(1, 0, 0)), a, new Plane3D(new Point3D(0, 0, theHeight), Up)));
        double aMean = (10 - theHeight) - theHeight; // twice the mean height above the plane
        Assert.Equal(1000 - (100 * Math.Tan(a) * aMean / 2), Volume(aResult), 6);
    }

    [Theory]
    [InlineData(0.0, 5.0)]
    [InlineData(6.0, 5.0)]
    [InlineData(0.0, -5.0)]
    public void A_cylinder_becomes_the_frustum_of_a_cone(double theHeight, double theDegrees)
    {
        Shape aCylinder = new ApiMakeCylinder(5, 10).Shape();
        double a = theDegrees * Math.PI / 180;
        var aDraft = new DraftAngle(aCylinder);
        aDraft.Add(Lateral(aCylinder), Up, a, new Plane3D(new Point3D(0, 0, theHeight), Up));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        Assert.True(aDraft.IsDone);
        Assert.True(new Analyzer(aDraft.Shape()).IsValid());
        double aV = Volume(aDraft.Shape());
        // whichever way the angle turns it, the radius is 5 at the neutral height and changes by tan(a) per unit
        double Frustum(double sign)
        {
            double r0 = 5 - (sign * theHeight * Math.Tan(a) * -1), r1 = 5 - (sign * (10 - theHeight) * Math.Tan(a));
            return Math.PI * 10 / 3 * ((r0 * r0) + (r0 * r1) + (r1 * r1));
        }
        Assert.True(Math.Abs(aV - Frustum(1)) < 1e-6 || Math.Abs(aV - Frustum(-1)) < 1e-6, aV.ToString());
    }

    [Fact]
    public void A_cone_at_its_own_angle_is_redrafted_to_another()
    {
        Shape aCone = new ApiMakeCone(5, 3, 10).Shape();
        var aDraft = new DraftAngle(aCone);
        Face aFace = Lateral(aCone);
        double anAngle = DraftTools.Angle(aFace, Up);
        // the same angle at the same place is nothing new, so it's the same volume
        aDraft.Add(aFace, Up, anAngle, Floor);
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        Assert.True(aDraft.IsDone);
        Assert.Equal(Math.PI * 10 / 3 * (25 + 15 + 9), Volume(aDraft.Shape()), 5);
    }

    // A box with its vertical edge x = 10, y = 10 rounded: the two sides and the fillet are tangent, so drafting one
    // carries the others with it
    private static Shape Rounded()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aFillet = new MakeFillet(aBox);
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);
            Point3D aP1 = Tool.Pnt(aV1), aP2 = Tool.Pnt(aV2);
            if (Math.Abs(aP1.X - 10) < 1e-9 && Math.Abs(aP2.X - 10) < 1e-9 && Math.Abs(aP1.Y - 10) < 1e-9 && Math.Abs(aP2.Y - 10) < 1e-9)
            {
                aFillet.Add(2, anEdge);
            }
        }
        return aFillet.Shape();
    }

    [Fact]
    public void Drafting_a_side_carries_what_is_tangent_to_it()
    {
        Shape aShape = Rounded();
        double a = 3 * Math.PI / 180;
        Face aSide = PlaneWithNormal(aShape, new Direction3D(1, 0, 0));
        var aDraft = new DraftAngle(aShape);
        aDraft.Add(aSide, Up, a, Floor);
        Assert.True(aDraft.AddDone(), aDraft.Status().ToString());
        aDraft.Build();
        Assert.True(aDraft.IsDone, aDraft.Status().ToString());
        // the side, the fillet and the other side: all three turned
        Assert.Equal(3, aDraft.ModifiedFaces().Count);
        Assert.Equal(3, aDraft.ConnectedFaces(aSide).Count);
        Shape aResult = aDraft.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.InRange(Volume(aResult), 1000 - (2 * 2 * 4) - (2 * Math.Tan(a) * 500), Volume(aShape));
    }

    [Fact]
    public void Removing_a_face_takes_what_was_tangent_with_it()
    {
        Shape aShape = Rounded();
        Face aSide = PlaneWithNormal(aShape, new Direction3D(1, 0, 0));
        Face aFar = PlaneWithNormal(aShape, new Direction3D(-1, 0, 0));
        var aDraft = new DraftAngle(aShape);
        aDraft.Add(aSide, Up, 0.05, Floor);
        Assert.True(aDraft.AddDone());
        aDraft.Add(aFar, Up, 0.05, Floor);
        int aBoth = aDraft.ModifiedFaces().Count;
        aDraft.Remove(aSide);
        Assert.True(aDraft.AddDone());
        Assert.True(aDraft.ModifiedFaces().Count < aBoth);
        aDraft.Build();
        Assert.True(aDraft.IsDone);
        Assert.Equal(Volume(Draft(aShape, (aFar, 0.05, Floor))), Volume(aDraft.Shape()), 7);
    }

    [Fact]
    public void A_side_with_a_hole_in_it_takes_the_hole_along_unchanged()
    {
        // the hole goes through the far sides; the draft only tilts the side next to it
        Shape aBox = new MakeBox(20, 20, 10).Shape();
        Shape aHole = new ApiMakeCylinder(new Frame3D(new Point3D(5, 5, -1), Up), 2, 12).Shape();
        Shape aPart = new Cut(aBox, aHole).Shape();
        double aBefore = Volume(aPart);
        Shape aResult = Draft(aPart, (PlaneWithNormal(aPart, new Direction3D(1, 0, 0)), 0.05, Floor));
        Assert.Equal(aBefore - (0.5 * 10 * 10 * Math.Tan(0.05) * 20), Volume(aResult), 5);
    }

    [Fact]
    public void A_plane_beside_a_cylinder_cuts_it_where_the_two_meet()
    {
        // a stadium: a box with half a cylinder on the side x = 10, not tangent to its neighbours' planes by the sides
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Shape aBoss = new ApiMakeCylinder(new Frame3D(new Point3D(10, 5, 0), Up), 3, 10).Shape();
        Shape aPart = new Fuse(aBox, aBoss).Shape();
        Assert.True(new Analyzer(aPart).IsValid());
        double aBefore = Volume(aPart);
        Face aFar = PlaneWithNormal(aPart, new Direction3D(-1, 0, 0));
        Shape aResult = Draft(aPart, (aFar, 0.05, Floor));
        Assert.Equal(aBefore - (0.5 * 10 * 10 * Math.Tan(0.05) * 10), Volume(aResult), 5);
    }

    [Fact]
    public void A_sphere_or_a_torus_cannot_be_drafted()
    {
        foreach (Shape aShape in new[] { new MakeSphere(5).Shape(), new MakeTorus(6, 2).Shape() })
        {
            var aDraft = new DraftAngle(aShape);
            Face aFace = Lateral(aShape);
            aDraft.Add(aFace, Up, 0.05, Floor);
            Assert.False(aDraft.AddDone());
            Assert.Equal(DraftErrorStatus.FaceRecomputation, aDraft.Status());
            Assert.True(aDraft.ProblematicShape().IsSame(aFace));
            Assert.Throws<ConstructionErrorException>(() => aDraft.Add(aFace, Up, 0.05, Floor));
            aDraft.Clear();
            aDraft.Init(new MakeBox(1, 1, 1).Shape());
            Assert.True(aDraft.AddDone());
        }
    }

    [Fact]
    public void A_face_of_another_shape_cannot_be_added()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aDraft = new DraftAngle(aBox);
        Face aStranger = PlaneWithNormal(new MakeBox(1, 1, 1).Shape(), new Direction3D(1, 0, 0));
        Assert.Throws<NoSuchObjectException>(() => aDraft.Remove(aStranger));
        aDraft.Add(PlaneWithNormal(aBox, new Direction3D(1, 0, 0)), Up, 0.05, Floor);
        aDraft.Build();
        Assert.Throws<NoSuchObjectException>(() => aDraft.ConnectedFaces(aStranger));
    }

    [Fact]
    public void A_draft_whose_plane_runs_along_the_extraction_is_refused()
    {
        // the neutral plane contains the extraction direction, so there's no angle to measure from
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aDraft = new DraftAngle(aBox);
        Face aSide = PlaneWithNormal(aBox, new Direction3D(1, 0, 0));
        aDraft.Add(aSide, Up, 0.05, new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        Assert.False(aDraft.AddDone());
    }

    // The area of the part of a disc of radius r beyond the chord at x = d
    private static double Segment(double r, double d) => (r * r * Math.Acos(d / r)) - (d * Math.Sqrt((r * r) - (d * d)));

    [Theory]
    [InlineData(5.0)]
    [InlineData(-5.0)]
    public void The_flat_of_a_d_shaped_bar_tilts_and_the_round_meets_it_lower_or_higher(double theDegrees)
    {
        // a cylinder of radius 5 and 10 high with x > 3 cut off: its flat at x = 3 is drafted about the floor, so at
        // height z it stands at x = 3 -+ z tan(a), and the bar loses or gains the slices of disc between
        Shape aBar = new Cut(new ApiMakeCylinder(5, 10).Shape(), new MakeBox(new Point3D(3, -10, -1), 10, 20, 12).Shape()).Shape();
        double aBefore = Volume(aBar);
        Assert.Equal((Math.PI * 25 * 10) - (Segment(5, 3) * 10), aBefore, 6);
        Face aFlat = PlaneWithNormal(aBar, new Direction3D(1, 0, 0));
        Shape aResult = Draft(aBar, (aFlat, theDegrees * Math.PI / 180, Floor));
        double Expected(int theSign)
        {
            // the bar's section at height z is the disc less the segment beyond x = 3 - sign z tan(a)
            double aSum = 0;
            const int n = 2000;
            for (int i = 0; i < n; i++)
            {
                double z = (i + 0.5) * 10.0 / n;
                aSum += ((Math.PI * 25) - Segment(5, 3 - (theSign * z * Math.Tan(theDegrees * Math.PI / 180)))) * 10.0 / n;
            }
            return aSum;
        }
        double aV = Volume(aResult);
        Assert.True(Math.Abs(aV - Expected(1)) < 1e-4 || Math.Abs(aV - Expected(-1)) < 1e-4, $"{aV} against {Expected(1)} or {Expected(-1)}");
        Assert.NotEqual(aBefore, aV, 3);
    }

    // A prism of the disc of radius theA by theB, 10 high, kept as a surface of extrusion (not made a cylinder)
    private static Shape Prism(double theA, double theB)
    {
        Frame3D aFrame = new(new Point3D(0, 0, 0), Up);
        Edge aRim = theA == theB ? new MakeEdge(new Circle3D(aFrame, theA)).Edge() : new MakeEdge(new Ellipse3D(aFrame, theA, theB)).Edge();
        Face aDisc = new MakeFace(new MakeWire(aRim).Wire(), true).Face();
        return new ApiMakePrism(aDisc, new Vector3D(0, 0, 10), false, false).Shape();
    }

    [Fact]
    public void A_circle_extruded_on_its_own_is_refused_where_a_cylinder_is_drafted()
    {
        // the draft turns planes, cylinders and cones; a surface of extrusion only as the neighbour of one
        Shape aPrism = Prism(5, 5);
        Assert.IsType<ExtrusionSurface>(Tool.Surface(Lateral(aPrism)));
        var aDraft = new DraftAngle(aPrism);
        aDraft.Add(Lateral(aPrism), Up, 0.05, Floor);
        Assert.False(aDraft.AddDone());
        Assert.Equal(DraftErrorStatus.FaceRecomputation, aDraft.Status());
    }

    [Fact]
    public void An_ellipse_extruded_has_no_draft_and_is_refused()
    {
        // its drafted side would be no cone: the draft recomputes only extrusions of circles
        Shape aPrism = Prism(5, 3);
        var aDraft = new DraftAngle(aPrism);
        aDraft.Add(Lateral(aPrism), Up, 0.05, Floor);
        if (aDraft.AddDone())
        {
            aDraft.Build();
            Assert.False(aDraft.IsDone);
        }
        Assert.Equal(DraftErrorStatus.FaceRecomputation, aDraft.Status());
    }

    [Theory]
    [InlineData(5.0)]
    [InlineData(-3.0)]
    public void A_drafted_side_is_at_its_angle_to_the_direction(double theDegrees)
    {
        // a side of the box, square to the direction, is at 0; drafted by a, it's at a, one way or the other
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Face aSide = PlaneWithNormal(aBox, new Direction3D(1, 0, 0));
        Assert.Equal(0, DraftTools.Angle(aSide, Up), 12);
        Assert.Equal(Math.PI / 2, Math.Abs(DraftTools.Angle(PlaneWithNormal(aBox, Up), Up)), 12);
        double a = theDegrees * Math.PI / 180;
        Shape aResult = Draft(aBox, (aSide, a, Floor));
        Face aDrafted = Faces(aResult).Single(f => Tool.Surface(f) is PlaneSurface aP && Math.Abs(aP.Plane.Axis.Direction.Z) > 1e-6 && Math.Abs(aP.Plane.Axis.Direction.Z) < 0.5);
        Assert.Equal(Math.Abs(a), Math.Abs(DraftTools.Angle(aDrafted, Up)), 12);
        // a cylinder along the direction is at 0; the cone a drafted cylinder becomes, at the draft's angle
        Shape aCylinder = new ApiMakeCylinder(5, 10).Shape();
        Assert.Equal(0, DraftTools.Angle(Lateral(aCylinder), Up), 12);
        var aDraft = new DraftAngle(aCylinder);
        aDraft.Add(Lateral(aCylinder), Up, a, Floor);
        aDraft.Build();
        Assert.Equal(Math.Abs(a), Math.Abs(DraftTools.Angle(Lateral(aDraft.Shape()), Up)), 9);
    }

    [Fact]
    public void An_angle_is_only_for_planes_and_for_cylinders_and_cones_along_the_direction()
    {
        Face aSphere = Faces(new MakeSphere(5).Shape()).First();
        Assert.Throws<DomainErrorException>(() => DraftTools.Angle(aSphere, Up));
        Face aCylinder = Lateral(new ApiMakeCylinder(5, 10).Shape());
        Assert.Throws<DomainErrorException>(() => DraftTools.Angle(aCylinder, new Direction3D(1, 0, 0)));
        Face aCone = Lateral(new ApiMakeCone(5, 2, 10).Shape());
        Assert.Throws<DomainErrorException>(() => DraftTools.Angle(aCone, new Direction3D(0, 1, 0)));
        // a cone's side leans by its half angle, atan(3/10)
        Assert.Equal(Math.Atan(0.3), Math.Abs(DraftTools.Angle(aCone, Up)), 9);
    }
}
