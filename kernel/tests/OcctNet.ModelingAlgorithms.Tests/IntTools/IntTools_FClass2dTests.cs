// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>IntTools_FClass2d on known faces.</summary>
public class IntTools_FClass2dTests
{
    private static TopoDS_Face FirstFace(in TopoDS_Shape theShape, GeomAbs_SurfaceType theType)
    {
        for (TopExp_Explorer anExp = new(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExp.Current());
            if (new BRepAdaptor_Surface(aF).GetType() == theType)
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
        Assert.Equal(TopAbs_IN, aC.Perform(new gp_Pnt2d(2, 2)));
        Assert.Equal(TopAbs_IN, aC.Perform(new gp_Pnt2d(9.5, 3)));
        Assert.Equal(TopAbs_OUT, aC.Perform(new gp_Pnt2d(5, 5))); // in the hole
        Assert.Equal(TopAbs_OUT, aC.Perform(new gp_Pnt2d(12, 5)));
        Assert.Equal(TopAbs_OUT, aC.Perform(new gp_Pnt2d(-3, -3)));
        // On the outer and the inner boundary: the polygons can't decide, the face classifier says ON
        Assert.Equal(TopAbs_ON, aC.Perform(new gp_Pnt2d(0, 5)));
        Assert.Equal(TopAbs_ON, aC.Perform(new gp_Pnt2d(4, 5)));
        Assert.Equal(TopAbs_ON, aC.Perform(new gp_Pnt2d(10, 10)));
        Assert.Equal(TopAbs_OUT, aC.PerformInfinitePoint());
    }

    [Fact]
    public void HoledSquare_TestOnRestriction()
    {
        // CSLib_Class2d offsets the point by Tol in coordinates normalized by the face's UV box (10 x 10 here)
        var aC = new IntTools_FClass2d(ClassificationTestShapes.HoledSquare(), 1.0e-7);
        Assert.Equal(TopAbs_IN, aC.TestOnRestriction(new gp_Pnt2d(2, 2), 0.01));
        Assert.Equal(TopAbs_OUT, aC.TestOnRestriction(new gp_Pnt2d(5, 5), 0.01));
        Assert.Equal(TopAbs_ON, aC.TestOnRestriction(new gp_Pnt2d(0.05, 5), 0.01));
        Assert.Equal(TopAbs_ON, aC.TestOnRestriction(new gp_Pnt2d(4.05, 5), 0.01));
        Assert.Equal(TopAbs_IN, aC.TestOnRestriction(new gp_Pnt2d(0.5, 5), 0.01));
    }

    [Fact]
    public void IsHole_IsTheLastWiresOrientation()
    {
        // As OCCT: the flag is set by the last wire with a non-zero area, so the holed square (outer wire, then the
        // clockwise hole) reports a hole
        Assert.True(new IntTools_FClass2d(ClassificationTestShapes.HoledSquare(), 1.0e-7).IsHole());
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        TopoDS_Face aF = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        aF.Orientation(TopAbs_Orientation.TopAbs_FORWARD);
        Assert.False(new IntTools_FClass2d(aF, 1.0e-7).IsHole());
    }

    [Fact]
    public void CylinderFace_PeriodicRecadre()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(2.0, 3.0, M_PI).Shape(); // a half cylinder: u in [0, pi]
        TopoDS_Face aF = FirstFace(aCyl, GeomAbs_SurfaceType.GeomAbs_Cylinder);
        var aC = new IntTools_FClass2d(aF, BRep_Tool.Tolerance(aF));
        Assert.Equal(TopAbs_IN, aC.Perform(new gp_Pnt2d(1.0, 1.0)));
        Assert.Equal(TopAbs_OUT, aC.Perform(new gp_Pnt2d(4.0, 1.0)));
        // One period further: brought back by AdjustPeriodic only when asked
        Assert.Equal(TopAbs_IN, aC.Perform(new gp_Pnt2d(1.0 + 2 * M_PI, 1.0)));
        Assert.Equal(TopAbs_OUT, aC.Perform(new gp_Pnt2d(1.0 + 2 * M_PI, 1.0), false));
        Assert.Equal(TopAbs_IN, aC.Perform(new gp_Pnt2d(1.0 - 4 * M_PI, 1.0)));
        Assert.Equal(TopAbs_OUT, aC.Perform(new gp_Pnt2d(1.0, 4.0)));
        Assert.Equal(TopAbs_ON, aC.Perform(new gp_Pnt2d(1.0, 0.0)));
        Assert.Equal(TopAbs_ON, aC.TestOnRestriction(new gp_Pnt2d(1.0 + 2 * M_PI, 1.0e-3), 1.0e-2));
    }

    [Fact]
    public void Empty_EverythingIsIn()
    {
        var aC = new IntTools_FClass2d();
        Assert.Equal(TopAbs_IN, aC.Perform(new gp_Pnt2d(1, 1)));
        Assert.Equal(TopAbs_IN, aC.TestOnRestriction(new gp_Pnt2d(1, 1), 0.1));
        Assert.Equal(TopAbs_IN, aC.PerformInfinitePoint());
    }

    [Fact]
    public void Queries_DontChangeTheAnswers()
    {
        // The face explorer made by the first undecided point restarts its probing at every point (decision 723)
        TopoDS_Face aF = ClassificationTestShapes.HoledSquare();
        var aReused = new IntTools_FClass2d(aF, 1.0e-7);
        var aRandom = new Random(6410);
        for (int i = 0; i < 400; i++)
        {
            // Points on or near the boundary lines x, y in {0, 4, 6, 10} go to the face classifier
            double x = aRandom.Next(4) switch { 0 => 0, 1 => 4, 2 => 6, _ => 10 } + (aRandom.NextDouble() - 0.5) * 1e-6;
            double y = aRandom.NextDouble() * 12 - 1;
            gp_Pnt2d aP = (i & 1) == 0 ? new gp_Pnt2d(x, y) : new gp_Pnt2d(y, x);
            var aFresh = new IntTools_FClass2d(aF, 1.0e-7);
            Assert.Equal(aFresh.Perform(aP), aReused.Perform(aP));
            Assert.Equal(aFresh.TestOnRestriction(aP, 1e-3), aReused.TestOnRestriction(aP, 1e-3));
        }
    }
}
