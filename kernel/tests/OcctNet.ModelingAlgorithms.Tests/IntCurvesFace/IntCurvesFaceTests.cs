// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.GeomAbs_SurfaceType;
using static Occt.IntCurveSurface_TransitionOnCurve;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of IntCurvesFace: lines through the faces of a box (points, parameters, states, transitions, the
/// face's orientation, bounds of the segment, a line in a face's plane), a surface of extrusion met through its
/// polyhedron, a curve, and the shape intersector (sorted points, the nearest point).
/// </summary>
public class IntCurvesFaceTests
{
    private static TopoDS_Shape Box10() => TestShapes.Frozen(TestShapes.Box(10, 10, 10));

    private static TopoDS_Face FaceAt(in TopoDS_Shape theShape, int theIndex)
        => BRepGPropTestShapes.Faces(theShape)[theIndex];

    [Fact]
    public void Line_through_the_top_and_bottom_faces_of_a_box()
    {
        TopoDS_Shape aBox = Box10();
        TopoDS_Face aBottom = FaceAt(aBox, 0), aTop = FaceAt(aBox, 1);
        var aLine = new gp_Lin(new gp_Pnt(3, 4, -5), new gp_Dir(0, 0, 1));

        var aTopInter = new IntCurvesFace_Intersector(aTop, 1e-7);
        Assert.Equal(GeomAbs_Plane, aTopInter.SurfaceType());
        Assert.True(aTopInter.Bounding().IsVoid()); // no polyhedron on a plane
        aTopInter.Perform(aLine, -RealLast(), RealLast());
        Assert.True(aTopInter.IsDone());
        Assert.Equal(1, aTopInter.NbPnt());
        Assert.Equal(15.0, aTopInter.WParameter(1), 1e-9);
        Assert.True(aTopInter.Pnt(1).IsEqual(new gp_Pnt(3, 4, 10), 1e-9));
        Assert.Equal(TopAbs_IN, aTopInter.State(1));
        Assert.True(aTopInter.Face().IsSame(aTop));
        // the box's faces point out: the line leaves the box at the top and enters it at the bottom
        Assert.Equal(IntCurveSurface_Out, aTopInter.Transition(1));

        var aBottomInter = new IntCurvesFace_Intersector(aBottom, 1e-7);
        aBottomInter.Perform(aLine, -RealLast(), RealLast());
        Assert.Equal(1, aBottomInter.NbPnt());
        Assert.Equal(5.0, aBottomInter.WParameter(1), 1e-9);
        Assert.Equal(IntCurveSurface_In, aBottomInter.Transition(1));

        // the reversed face reverses the transition
        var aReversed = new IntCurvesFace_Intersector(TopoDS.Face(aTop.Reversed()), 1e-7);
        aReversed.Perform(aLine, -RealLast(), RealLast());
        Assert.Equal(IntCurveSurface_In, aReversed.Transition(1));

        // the segment ends before the face
        aTopInter.Perform(aLine, 0.0, 14.0);
        Assert.True(aTopInter.IsDone());
        Assert.Equal(0, aTopInter.NbPnt());
    }

    [Fact]
    public void Line_outside_on_and_in_the_plane_of_a_face()
    {
        TopoDS_Face aTop = FaceAt(Box10(), 1);
        var anInter = new IntCurvesFace_Intersector(aTop, 1e-7);

        // misses the face
        anInter.Perform(new gp_Lin(new gp_Pnt(20, 4, -5), new gp_Dir(0, 0, 1)), -RealLast(), RealLast());
        Assert.Equal(0, anInter.NbPnt());

        // through an edge: ON
        anInter.Perform(new gp_Lin(new gp_Pnt(10, 4, -5), new gp_Dir(0, 0, 1)), -RealLast(), RealLast());
        Assert.Equal(1, anInter.NbPnt());
        Assert.Equal(TopAbs_ON, anInter.State(1));

        // in the face's plane
        anInter.Perform(new gp_Lin(new gp_Pnt(-5, 4, 10), new gp_Dir(1, 0, 0)), -RealLast(), RealLast());
        Assert.Equal(0, anInter.NbPnt());
        Assert.True(anInter.IsParallel());

        Assert.Equal(TopAbs_IN, anInter.ClassifyUVPoint(new gp_Pnt2d(1, 1)));
    }

    [Fact]
    public void Extruded_tube_is_met_through_its_polyhedron()
    {
        TopoDS_Face aTube = TestShapes.Frozen(ClassificationTestShapes.ExtrudedTube(2.0, 3.0));
        var anInter = new IntCurvesFace_Intersector(aTube, 1e-7);
        Assert.Equal(GeomAbs_SurfaceOfExtrusion, anInter.SurfaceType());
        Bnd_Box aBox = anInter.Bounding();
        Assert.False(aBox.IsVoid());

        // across the tube at height 1: two points, sorted along the line
        anInter.Perform(new gp_Lin(new gp_Pnt(-5, 0, 1), new gp_Dir(1, 0, 0)), -RealLast(), RealLast());
        Assert.Equal(2, anInter.NbPnt());
        Assert.Equal(3.0, anInter.WParameter(1), 1e-7);
        Assert.Equal(7.0, anInter.WParameter(2), 1e-7);
        Assert.Equal(1.0, anInter.VParameter(1), 1e-7);
        Assert.Equal(Math.PI, anInter.UParameter(1), 1e-7);

        // above the tube
        anInter.Perform(new gp_Lin(new gp_Pnt(-5, 0, 4), new gp_Dir(1, 0, 0)), -RealLast(), RealLast());
        Assert.Equal(0, anInter.NbPnt());

        // a circle around the axis, inside the tube's radius: no point; through it: two
        var aCircle = new GeomAdaptor_Curve(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 1), new gp_Dir(1, 0, 0), new gp_Dir(0, 1, 0)), 1.0));
        anInter.Perform(aCircle, -RealLast(), RealLast());
        Assert.Equal(0, anInter.NbPnt());
        var aBigCircle = new GeomAdaptor_Curve(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 1), new gp_Dir(1, 0, 0), new gp_Dir(0, 1, 0)), 2.5));
        anInter.Perform(aBigCircle, -RealLast(), RealLast());
        Assert.Equal(2, anInter.NbPnt());
        for (int i = 1; i <= 2; i++)
        {
            gp_Pnt aP = anInter.Pnt(i);
            Assert.Equal(2.0, Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()), 1e-7);
        }
    }

    [Fact]
    public void ShapeIntersector_sorts_the_points_along_the_line()
    {
        TopoDS_Shape aBox = Box10();
        var anInter = new IntCurvesFace_ShapeIntersector();
        anInter.Load(aBox, 1e-7);
        var aLine = new gp_Lin(new gp_Pnt(3, 4, -5), new gp_Dir(0, 0, 1));
        anInter.Perform(aLine, -RealLast(), RealLast());
        Assert.True(anInter.IsDone());
        Assert.Equal(2, anInter.NbPnt());
        Assert.Equal(5.0, anInter.WParameter(1), 1e-9);
        Assert.Equal(15.0, anInter.WParameter(2), 1e-9);
        Assert.True(anInter.Face(1).IsSame(FaceAt(aBox, 0)));
        Assert.True(anInter.Face(2).IsSame(FaceAt(aBox, 1)));
        Assert.Equal(IntCurveSurface_In, anInter.Transition(1));
        Assert.Equal(IntCurveSurface_Out, anInter.Transition(2));
        Assert.Equal(TopAbs_IN, anInter.State(1));
        Assert.Equal(3.0, anInter.UParameter(2), 1e-9);
        Assert.Equal(4.0, anInter.VParameter(2), 1e-9);
        Assert.True(anInter.Pnt(2).IsEqual(new gp_Pnt(3, 4, 10), 1e-9));

        // the nearest point, twice (the faces met are tried first the second time)
        for (int k = 0; k < 2; k++)
        {
            anInter.PerformNearest(aLine, -RealLast(), RealLast());
            Assert.True(anInter.IsDone());
            Assert.True(anInter.NbPnt() >= 1);
            Assert.Equal(5.0, anInter.WParameter(1), 1e-9);
        }

        // a curve: a circle around the box's centre crossing its four sides twice each
        var aCircle = new GeomAdaptor_Curve(new Geom_Circle(new gp_Ax2(new gp_Pnt(5, 5, 5), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 6.0));
        anInter.Perform(aCircle, -RealLast(), RealLast());
        Assert.Equal(8, anInter.NbPnt());
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
        TopoDS_Shape aBox = Box10();
        TopoDS_Shape aFar = ClassificationTestShapes.Translated(aBox, new gp_Vec(100, 0, 0));
        var anInter = new IntCurvesFace_ShapeIntersector();
        anInter.Load(aBox, 1e-7);
        anInter.Load(aFar, 1e-7);
        anInter.Perform(new gp_Lin(new gp_Pnt(105, 5, -5), new gp_Dir(0, 0, 1)), -RealLast(), RealLast());
        Assert.Equal(0, anInter.NbPnt()); // the second box is missed
        anInter.Perform(new gp_Lin(new gp_Pnt(5, 5, -5), new gp_Dir(0, 0, 1)), -RealLast(), RealLast());
        Assert.Equal(2, anInter.NbPnt()); // the first box is met
    }
}
