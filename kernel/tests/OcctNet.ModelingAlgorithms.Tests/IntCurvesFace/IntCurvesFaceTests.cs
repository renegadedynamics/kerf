// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of IntCurvesFace: lines through the faces of a box (points, parameters, states, transitions, the
/// face's orientation, bounds of the segment, a line in a face's plane), a surface of extrusion met through its
/// polyhedron, a curve, and the shape intersector (sorted points, the nearest point).
/// </summary>
public class IntCurvesFaceTests
{
    private static Shape Box10() => TestShapes.Frozen(TestShapes.Box(10, 10, 10));

    private static Face FaceAt(in Shape theShape, int theIndex)
        => BRepGPropTestShapes.Faces(theShape)[theIndex];

    [Fact]
    public void Line_through_the_top_and_bottom_faces_of_a_box()
    {
        Shape aBox = Box10();
        Face aBottom = FaceAt(aBox, 0), aTop = FaceAt(aBox, 1);
        var aLine = new Line3D(new Point3D(3, 4, -5), new Direction3D(0, 0, 1));

        var aTopInter = new CurveSurfaceIntersector(aTop, 1e-7);
        Assert.Equal(Plane, aTopInter.SurfaceType());
        Assert.True(aTopInter.Bounding().IsVoid); // no polyhedron on a plane
        aTopInter.Perform(aLine, -RealLast(), RealLast());
        Assert.True(aTopInter.IsDone);
        Assert.Equal(1, aTopInter.PntCount);
        Assert.Equal(15.0, aTopInter.WParameter(1), 1e-9);
        Assert.True(aTopInter.Pnt(1).IsEqual(new Point3D(3, 4, 10), 1e-9));
        Assert.Equal(global::Kerf.Topology.Containment.In, aTopInter.State(1));
        Assert.True(aTopInter.Face().IsSame(aTop));
        // the box's faces point out: the line leaves the box at the top and enters it at the bottom
        Assert.Equal(TransitionOnCurve.Out, aTopInter.Transition(1));

        var aBottomInter = new CurveSurfaceIntersector(aBottom, 1e-7);
        aBottomInter.Perform(aLine, -RealLast(), RealLast());
        Assert.Equal(1, aBottomInter.PntCount);
        Assert.Equal(5.0, aBottomInter.WParameter(1), 1e-9);
        Assert.Equal(TransitionOnCurve.In, aBottomInter.Transition(1));

        // the reversed face reverses the transition
        var aReversed = new CurveSurfaceIntersector(ShapeCasts.Face(aTop.Reversed()), 1e-7);
        aReversed.Perform(aLine, -RealLast(), RealLast());
        Assert.Equal(TransitionOnCurve.In, aReversed.Transition(1));

        // the segment ends before the face
        aTopInter.Perform(aLine, 0.0, 14.0);
        Assert.True(aTopInter.IsDone);
        Assert.Equal(0, aTopInter.PntCount);
    }

    [Fact]
    public void Line_outside_on_and_in_the_plane_of_a_face()
    {
        Face aTop = FaceAt(Box10(), 1);
        var anInter = new CurveSurfaceIntersector(aTop, 1e-7);

        // misses the face
        anInter.Perform(new Line3D(new Point3D(20, 4, -5), new Direction3D(0, 0, 1)), -RealLast(), RealLast());
        Assert.Equal(0, anInter.PntCount);

        // through an edge: ON
        anInter.Perform(new Line3D(new Point3D(10, 4, -5), new Direction3D(0, 0, 1)), -RealLast(), RealLast());
        Assert.Equal(1, anInter.PntCount);
        Assert.Equal(On, anInter.State(1));

        // in the face's plane
        anInter.Perform(new Line3D(new Point3D(-5, 4, 10), new Direction3D(1, 0, 0)), -RealLast(), RealLast());
        Assert.Equal(0, anInter.PntCount);
        Assert.True(anInter.IsParallel);

        Assert.Equal(global::Kerf.Topology.Containment.In, anInter.ClassifyUVPoint(new Point2D(1, 1)));
    }

    [Fact]
    public void Extruded_tube_is_met_through_its_polyhedron()
    {
        Face aTube = TestShapes.Frozen(ClassificationTestShapes.ExtrudedTube(2.0, 3.0));
        var anInter = new CurveSurfaceIntersector(aTube, 1e-7);
        Assert.Equal(SurfaceOfExtrusion, anInter.SurfaceType());
        Box aBox = anInter.Bounding();
        Assert.False(aBox.IsVoid);

        // across the tube at height 1: two points, sorted along the line
        anInter.Perform(new Line3D(new Point3D(-5, 0, 1), new Direction3D(1, 0, 0)), -RealLast(), RealLast());
        Assert.Equal(2, anInter.PntCount);
        Assert.Equal(3.0, anInter.WParameter(1), 1e-7);
        Assert.Equal(7.0, anInter.WParameter(2), 1e-7);
        Assert.Equal(1.0, anInter.VParameter(1), 1e-7);
        Assert.Equal(Math.PI, anInter.UParameter(1), 1e-7);

        // above the tube
        anInter.Perform(new Line3D(new Point3D(-5, 0, 4), new Direction3D(1, 0, 0)), -RealLast(), RealLast());
        Assert.Equal(0, anInter.PntCount);

        // a circle around the axis, inside the tube's radius: no point; through it: two
        var aCircle = new CurveAdaptor(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0)), 1.0));
        anInter.Perform(aCircle, -RealLast(), RealLast());
        Assert.Equal(0, anInter.PntCount);
        var aBigCircle = new CurveAdaptor(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0)), 2.5));
        anInter.Perform(aBigCircle, -RealLast(), RealLast());
        Assert.Equal(2, anInter.PntCount);
        for (int i = 1; i <= 2; i++)
        {
            Point3D aP = anInter.Pnt(i);
            Assert.Equal(2.0, Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()), 1e-7);
        }
    }

    [Fact]
    public void ShapeIntersector_sorts_the_points_along_the_line()
    {
        Shape aBox = Box10();
        var anInter = new ShapeIntersector();
        anInter.Load(aBox, 1e-7);
        var aLine = new Line3D(new Point3D(3, 4, -5), new Direction3D(0, 0, 1));
        anInter.Perform(aLine, -RealLast(), RealLast());
        Assert.True(anInter.IsDone);
        Assert.Equal(2, anInter.PntCount);
        Assert.Equal(5.0, anInter.WParameter(1), 1e-9);
        Assert.Equal(15.0, anInter.WParameter(2), 1e-9);
        Assert.True(anInter.Face(1).IsSame(FaceAt(aBox, 0)));
        Assert.True(anInter.Face(2).IsSame(FaceAt(aBox, 1)));
        Assert.Equal(TransitionOnCurve.In, anInter.Transition(1));
        Assert.Equal(TransitionOnCurve.Out, anInter.Transition(2));
        Assert.Equal(global::Kerf.Topology.Containment.In, anInter.State(1));
        Assert.Equal(3.0, anInter.UParameter(2), 1e-9);
        Assert.Equal(4.0, anInter.VParameter(2), 1e-9);
        Assert.True(anInter.Pnt(2).IsEqual(new Point3D(3, 4, 10), 1e-9));

        // the nearest point, twice (the faces met are tried first the second time)
        for (int k = 0; k < 2; k++)
        {
            anInter.PerformNearest(aLine, -RealLast(), RealLast());
            Assert.True(anInter.IsDone);
            Assert.True(anInter.PntCount >= 1);
            Assert.Equal(5.0, anInter.WParameter(1), 1e-9);
        }

        // a curve: a circle around the box's centre crossing its four sides twice each
        var aCircle = new CurveAdaptor(new CircleCurve3D(new Frame3D(new Point3D(5, 5, 5), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 6.0));
        anInter.Perform(aCircle, -RealLast(), RealLast());
        Assert.Equal(8, anInter.PntCount);
        for (int i = 2; i <= 8; i++)
        {
            Assert.True(anInter.WParameter(i) >= anInter.WParameter(i - 1));
        }
    }

    [Fact]
    public void ShapeIntersector_second_Load_keeps_the_first_shape_OCCT_issue()
    {
        // OCCT's Load doesn't clear the faces loaded before, and counts only the new ones: after loading a second box
        // the intersector still meets the first box's faces (occt-issues.md, pinned)
        Shape aBox = Box10();
        Shape aFar = ClassificationTestShapes.Translated(aBox, new Vector3D(100, 0, 0));
        var anInter = new ShapeIntersector();
        anInter.Load(aBox, 1e-7);
        anInter.Load(aFar, 1e-7);
        anInter.Perform(new Line3D(new Point3D(105, 5, -5), new Direction3D(0, 0, 1)), -RealLast(), RealLast());
        Assert.Equal(0, anInter.PntCount); // the second box is missed
        anInter.Perform(new Line3D(new Point3D(5, 5, -5), new Direction3D(0, 0, 1)), -RealLast(), RealLast());
        Assert.Equal(2, anInter.PntCount); // the first box is met
    }
}
