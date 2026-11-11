// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

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
            Assert.Equal(2.0, Math.Sqrt(aP.X * aP.X + aP.Y * aP.Y), 1e-7);
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

    // Faces whose polyhedron the hull box can stand for: a cubic Bézier extruded along a slant, trimmed in both
    // directions and placed by a rotation and a move, and a bent B-spline surface
    private static IEnumerable<Face> HullFaces()
    {
        var aBezier = new BezierCurve3D([new Point3D(-3, -1, 0), new Point3D(-1, 4, 0.5), new Point3D(2, -3, 0), new Point3D(4, 2, 1)]);
        var anExtrusion = new ExtrusionSurface(aBezier, new Direction3D(0.2, -0.1, 1));
        Face anExtruded = BRepGPropTestShapes.NaturalFace(new TrimmedSurface(anExtrusion, 0.1, 0.9, -0.5, 2.0));
        yield return anExtruded;
        Transform3D aMove = new Transform3D().WithRotation(new Axis3D(new Point3D(1, 0, 0), new Direction3D(1, 1, 1)), 0.7);
        aMove = aMove.Multiplied(new Transform3D().WithTranslation(new Vector3D(2, -1, 3)));
        yield return ShapeCasts.Face(anExtruded.Moved(new Placement(aMove)));
        var aPoles = new NCollection_Array2<Point3D>(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles.SetValue(i, j, new Point3D(3 * (i - 2), 3 * (j - 2), (i == 2 && j == 2) ? 3 : 0.5 * (i - j)));
            }
        }
        var aKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        var aMults = NCollection_Array1<int>.From([3, 3]);
        yield return BRepGPropTestShapes.NaturalFace(new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 2, 2));
    }

    // Unit 15D: a line the intersector says is surely clear of its polyhedron's box is clear of it (LinBox finds no part of
    // the line in it, where OCCT's Perform returns with nothing), and a line through a face is never said to be; Perform
    // gives the same, without the polyhedron, as an intersector whose polyhedron was built first
    [Fact]
    public void A_line_clear_of_the_poles_box_is_clear_of_the_polyhedron()
    {
        int aCleared = 0, aMet = 0;
        foreach (Face aFace in HullFaces())
        {
            var aBuilt = new CurveSurfaceIntersector(TestShapes.Frozen(aFace), 1e-7);
            Box aPolyhedronBox = aBuilt.Bounding();
            Assert.False(aPolyhedronBox.IsVoid);
            Assert.False(aBuilt.SurelyMissesPolyhedron(new Line3D(new Point3D(100, 100, 100), new Direction3D(1, 0, 0)))); // built: not asked
            Direction3D[] aDirections = [new(1, 0, 0), new(0, 1, 0), new(0, 0, 1), new(1, 1, 0), new(1, -2, 3), new(-3, 1, 1), new(0.01, 0, 1)];
            for (int i = -6; i <= 6; i++)
            {
                for (int j = -6; j <= 6; j++)
                {
                    for (int k = -6; k <= 6; k += 3)
                    {
                        foreach (Direction3D aDirection in aDirections)
                        {
                            var aLine = new Line3D(new Point3D(i * Math.Abs(i), j * Math.Abs(j), 2.0 * k), aDirection); // nearer the faces, closer together
                            var aLazy = new CurveSurfaceIntersector(TestShapes.Frozen(aFace), 1e-7);
                            bool aClear = aLazy.SurelyMissesPolyhedron(aLine);
                            var aTool = new PolygonTool();
                            Box aLineBox = new();
                            aTool.LinBox(aLine, aPolyhedronBox, ref aLineBox);
                            if (aClear)
                            {
                                aCleared++;
                                // LinBox gives the slabs' common range even where it's empty, from beyond its end back to its start
                                Assert.True(aTool.SegmentCount == 0 || aTool.BeginParam(1) > aTool.EndParam(1));
                            }
                            aLazy.Perform(aLine, -RealLast(), RealLast());
                            aBuilt.Perform(aLine, -RealLast(), RealLast());
                            Assert.Equal(aBuilt.PntCount, aLazy.PntCount);
                            for (int n = 1; n <= aBuilt.PntCount; n++)
                            {
                                Assert.Equal(aBuilt.WParameter(n), aLazy.WParameter(n));
                                Assert.Equal(aBuilt.State(n), aLazy.State(n));
                            }
                            aMet += aBuilt.PntCount > 0 ? 1 : 0;
                            Assert.False(aClear && aBuilt.PntCount > 0);
                        }
                    }
                }
            }
        }
        Assert.True(aCleared > 1000, $"{aCleared} lines cleared");
        Assert.True(aMet > 100, $"{aMet} lines met a face");
    }
}
