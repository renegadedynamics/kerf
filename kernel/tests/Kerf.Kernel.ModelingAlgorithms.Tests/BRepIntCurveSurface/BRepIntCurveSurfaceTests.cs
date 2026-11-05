// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepIntCurveSurface_Inter: rays through a box (points face by face, parameters, states,
/// transitions), through its edges, missing it, through a located box and a holed face, a circle crossing the sides of
/// a box, and the errors when there is no current point.
/// </summary>
public class BRepIntCurveSurfaceTests
{
    private static Shape Box10() => TestShapes.Frozen(TestShapes.Box(10, 10, 10));

    private static List<(Point3D P, double U, double V, double W, Containment S, TransitionOnCurve T, Face F)> All(CurveShapeIntersector theInter)
    {
        var aList = new List<(Point3D, double, double, double, Containment, TransitionOnCurve, Face)>();
        for (; theInter.More(); theInter.Next())
        {
            aList.Add((theInter.Pnt(), theInter.U(), theInter.V(), theInter.W(), theInter.State(), theInter.Transition(), theInter.Face()));
            Assert.Equal(theInter.W(), theInter.Point().W());
            Assert.True(aList.Count < 100);
        }

        return aList;
    }

    [Fact]
    public void Ray_through_a_box()
    {
        Shape aBox = Box10();
        var anInter = new CurveShapeIntersector();
        anInter.Init(aBox, new Line3D(new Point3D(3, 4, -5), new Direction3D(0, 0, 1)), 1e-7);
        var aPoints = All(anInter);
        Assert.Equal(2, aPoints.Count);

        // the faces in exploration order: the bottom (entering), then the top (leaving)
        Assert.Equal(5.0, aPoints[0].W, 1e-9);
        Assert.True(aPoints[0].P.IsEqual(new Point3D(3, 4, 0), 1e-9));
        Assert.Equal(TransitionOnCurve.In, aPoints[0].T);
        Assert.Equal(global::Kerf.Topology.Containment.In, aPoints[0].S);
        Assert.True(aPoints[0].F.IsSame(BRepGPropTestShapes.Faces(aBox)[0]));
        Assert.Equal(15.0, aPoints[1].W, 1e-9);
        Assert.Equal(TransitionOnCurve.Out, aPoints[1].T);
        Assert.Equal(3.0, aPoints[1].U, 1e-9);
        Assert.Equal(4.0, aPoints[1].V, 1e-9);
    }

    [Fact]
    public void Ray_along_a_diagonal_and_through_an_edge()
    {
        Shape aBox = Box10();
        var anInter = new CurveShapeIntersector();
        // through the vertical edge x = 10, y = 0 at z = 5, from outside along the diagonal of the XY plane
        anInter.Init(aBox, new Line3D(new Point3D(15, -5, 5), new Direction3D(-1, 1, 0)), 1e-7);
        var aPoints = All(anInter);
        // the two faces meeting at the edge give an ON point each, then the opposite edge (0, 10) the same
        Assert.Equal(4, aPoints.Count);
        Assert.All(aPoints, p => Assert.Equal(On, p.S));
    }

    [Fact]
    public void Ray_missing_the_box()
    {
        var anInter = new CurveShapeIntersector();
        anInter.Init(Box10(), new Line3D(new Point3D(30, 4, -5), new Direction3D(0, 0, 1)), 1e-7);
        Assert.False(anInter.More());
        EngineAssert.Throws<NotDoneException>(() => anInter.Pnt());
        EngineAssert.Throws<NotDoneException>(() => anInter.W());
        EngineAssert.Throws<NotDoneException>(() => anInter.Point());
    }

    [Fact]
    public void Located_box_and_holed_face()
    {
        Shape aMoved = ClassificationTestShapes.Translated(Box10(), new Vector3D(0, 0, 100));
        var anInter = new CurveShapeIntersector();
        anInter.Load(aMoved, 1e-7);
        anInter.Init(new CurveAdaptor(new LineCurve3D(new Point3D(3, 4, 0), new Direction3D(0, 0, 1))));
        var aPoints = All(anInter);
        Assert.Equal(2, aPoints.Count);
        Assert.Equal(100.0, aPoints[0].W, 1e-9);
        Assert.Equal(110.0, aPoints[1].W, 1e-9);

        Shape aHoled = TestShapes.Frozen(ClassificationTestShapes.HoledSquare());
        anInter.Load(aHoled, 1e-7);
        anInter.Init(new CurveAdaptor(new LineCurve3D(new Point3D(5, 5, 3), new Direction3D(0, 0, 1))));
        Assert.False(anInter.More()); // through the hole
        anInter.Init(new CurveAdaptor(new LineCurve3D(new Point3D(2, 5, 3), new Direction3D(0, 0, -1))));
        aPoints = All(anInter);
        Assert.Single(aPoints);
        Assert.Equal(3.0, aPoints[0].W, 1e-9);
    }

    [Fact]
    public void Bounded_curves()
    {
        Shape aBox = Box10();
        var anInter = new CurveShapeIntersector();
        anInter.Load(aBox, 1e-7);

        // a segment ending inside the box meets only the bottom face
        anInter.Init(new CurveAdaptor(new LineCurve3D(new Point3D(3, 4, -5), new Direction3D(0, 0, 1)), 0.0, 10.0));
        var aPoints = All(anInter);
        Assert.Single(aPoints);
        Assert.Equal(5.0, aPoints[0].W, 1e-9);

        // a circle around the box's centre crosses its four sides twice each
        anInter.Init(new CurveAdaptor(new CircleCurve3D(new Frame3D(new Point3D(5, 5, 5), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 6.0)));
        aPoints = All(anInter);
        Assert.Equal(8, aPoints.Count);
        Assert.All(aPoints, p => Assert.Equal(6.0, p.P.DistanceTo(new Point3D(5, 5, 5)), 1e-9));
    }
}
