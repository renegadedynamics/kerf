// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>IntTools_FClass2d on known faces.</summary>
public class IntTools_FClass2dTests
{
    private static Face FirstFace(in Shape theShape, SurfaceKind theType)
    {
        for (Explorer anExp = new(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Face aF = ShapeCasts.Face(anExp.Current());
            if (new FaceSurfaceAdaptor(aF).GetType() == theType)
            {
                return aF;
            }
        }
        throw new InvalidOperationException("no face of the type");
    }

    [Fact]
    public void HoledSquare_StatesOfPoints()
    {
        var aC = new IntTools_FClass2d(ClassificationTestShapes.HoledSquare(), 1.0e-7);
        Assert.Equal(In, aC.Perform(new Point2D(2, 2)));
        Assert.Equal(In, aC.Perform(new Point2D(9.5, 3)));
        Assert.Equal(Out, aC.Perform(new Point2D(5, 5))); // in the hole
        Assert.Equal(Out, aC.Perform(new Point2D(12, 5)));
        Assert.Equal(Out, aC.Perform(new Point2D(-3, -3)));
        // On the outer and the inner boundary: the polygons can't decide, the face classifier says ON
        Assert.Equal(On, aC.Perform(new Point2D(0, 5)));
        Assert.Equal(On, aC.Perform(new Point2D(4, 5)));
        Assert.Equal(On, aC.Perform(new Point2D(10, 10)));
        Assert.Equal(Out, aC.PerformInfinitePoint());
    }

    [Fact]
    public void HoledSquare_TestOnRestriction()
    {
        // CSLib_Class2d offsets the point by Tol in coordinates normalized by the face's UV box (10 x 10 here)
        var aC = new IntTools_FClass2d(ClassificationTestShapes.HoledSquare(), 1.0e-7);
        Assert.Equal(In, aC.TestOnRestriction(new Point2D(2, 2), 0.01));
        Assert.Equal(Out, aC.TestOnRestriction(new Point2D(5, 5), 0.01));
        Assert.Equal(On, aC.TestOnRestriction(new Point2D(0.05, 5), 0.01));
        Assert.Equal(On, aC.TestOnRestriction(new Point2D(4.05, 5), 0.01));
        Assert.Equal(In, aC.TestOnRestriction(new Point2D(0.5, 5), 0.01));
    }

    [Fact]
    public void IsHole_IsTheLastWiresOrientation()
    {
        // As OCCT: the flag is set by the last wire with a non-zero area, so the holed square (outer wire, then the
        // clockwise hole) reports a hole
        Assert.True(new IntTools_FClass2d(ClassificationTestShapes.HoledSquare(), 1.0e-7).IsHole());
        Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        Face aF = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        aF.Orientation(Orientation.Forward);
        Assert.False(new IntTools_FClass2d(aF, 1.0e-7).IsHole());
    }

    [Fact]
    public void CylinderFace_PeriodicRecadre()
    {
        Shape aCyl = new BRepPrimAPI_MakeCylinder(2.0, 3.0, M_PI).Shape(); // a half cylinder: u in [0, pi]
        Face aF = FirstFace(aCyl, SurfaceKind.Cylinder);
        var aC = new IntTools_FClass2d(aF, Tool.Tolerance(aF));
        Assert.Equal(In, aC.Perform(new Point2D(1.0, 1.0)));
        Assert.Equal(Out, aC.Perform(new Point2D(4.0, 1.0)));
        // One period further: brought back by AdjustPeriodic only when asked
        Assert.Equal(In, aC.Perform(new Point2D(1.0 + 2 * M_PI, 1.0)));
        Assert.Equal(Out, aC.Perform(new Point2D(1.0 + 2 * M_PI, 1.0), false));
        Assert.Equal(In, aC.Perform(new Point2D(1.0 - 4 * M_PI, 1.0)));
        Assert.Equal(Out, aC.Perform(new Point2D(1.0, 4.0)));
        Assert.Equal(On, aC.Perform(new Point2D(1.0, 0.0)));
        Assert.Equal(On, aC.TestOnRestriction(new Point2D(1.0 + 2 * M_PI, 1.0e-3), 1.0e-2));
    }

    [Fact]
    public void Empty_EverythingIsIn()
    {
        var aC = new IntTools_FClass2d();
        Assert.Equal(In, aC.Perform(new Point2D(1, 1)));
        Assert.Equal(In, aC.TestOnRestriction(new Point2D(1, 1), 0.1));
        Assert.Equal(In, aC.PerformInfinitePoint());
    }

    [Fact]
    public void Queries_DontChangeTheAnswers()
    {
        // The face explorer made by the first undecided point restarts its probing at every point (decision 723)
        Face aF = ClassificationTestShapes.HoledSquare();
        var aReused = new IntTools_FClass2d(aF, 1.0e-7);
        var aRandom = new Random(6410);
        for (int i = 0; i < 400; i++)
        {
            // Points on or near the boundary lines x, y in {0, 4, 6, 10} go to the face classifier
            double x = aRandom.Next(4) switch { 0 => 0, 1 => 4, 2 => 6, _ => 10 } + (aRandom.NextDouble() - 0.5) * 1e-6;
            double y = aRandom.NextDouble() * 12 - 1;
            Point2D aP = (i & 1) == 0 ? new Point2D(x, y) : new Point2D(y, x);
            var aFresh = new IntTools_FClass2d(aF, 1.0e-7);
            Assert.Equal(aFresh.Perform(aP), aReused.Perform(aP));
            Assert.Equal(aFresh.TestOnRestriction(aP, 1e-3), aReused.TestOnRestriction(aP, 1e-3));
        }
    }
}
