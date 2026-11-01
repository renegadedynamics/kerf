// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.GeomAbs_Shape;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeUpgrade's divide and convert tools, RemoveInternalWires, ShellSewing and the package function (unit 7B), on
/// frozen inputs (decision 8402): the caller's shape is never written, the results are frozen, and the caller's own
/// sub-shapes come back where nothing changed. The numbers of faces are the heal group's (split_number,
/// split_two_numbers) and the shapes' own.
/// </summary>
public class ShapeUpgrade_DivideTests
{
    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static bool AllFrozen(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            if (!aMap[i].TShape()!.IsFrozen())
            {
                return false;
            }
        }
        return true;
    }

    private static double Area(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static TopoDS_Shape Cylinder() => new BRepPrimAPI_MakeCylinder(5, 10).Shape();

    private static TopoDS_Face FirstFace(in TopoDS_Shape theShape) =>
        TopoDS.Face(new TopExp_Explorer(theShape, TopAbs_FACE).Current());

    [Fact]
    public void ShapeDivideAngle_OnACylinder_SplitsTheLateralFaceInFour_TheInputIsUnchanged()
    {
        TopoDS_Shape aCyl = Cylinder();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);

        var aTool = new ShapeUpgrade_ShapeDivideAngle(95 * Math.PI / 180, aCyl);
        Assert.True(aTool.Perform());
        TopoDS_Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(AllFrozen(aRes));
        Assert.True(aTool.Status(ShapeExtend_DONE2));
        Assert.Equal(6, Count(aRes, TopAbs_FACE));
        Assert.Equal(Area(aCyl), Area(aRes), 6);
        Assert.Equal(95 * Math.PI / 180, aTool.MaxAngle());
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
    }

    [Fact]
    public void ShapeDivideClosed_OnACylinder_SplitsTheLateralFaceInTwo()
    {
        TopoDS_Shape aCyl = Cylinder();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);

        var aTool = new ShapeUpgrade_ShapeDivideClosed(aCyl);
        Assert.True(aTool.Perform());
        TopoDS_Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(4, Count(aRes, TopAbs_FACE));
        Assert.Equal(Area(aCyl), Area(aRes), 6);
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
    }

    [Theory]
    [InlineData(10, 0, 10)]
    [InlineData(3, 5, 15)]
    public void ShapeDivideArea_ByNumber_SplitsTheSphericalFace(int theNumber1, int theNumber2, int theNbFaces)
    {
        TopoDS_Shape aSphere = new BRepPrimAPI_MakeSphere(10, 0, Math.PI / 2).Shape();
        TopoDS_Face aFace = TopoDS.Face(FirstFace(aSphere));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);

        // DT_SplitByNumber result a_1 n1 [n2] (heal/split_number, heal/split_two_numbers)
        var aTool = new ShapeUpgrade_ShapeDivideArea(aFace);
        aTool.SetSplittingByNumber(true);
        if (theNumber2 == 0)
        {
            aTool.NbParts() = theNumber1;
        }
        else
        {
            aTool.SetNumbersUVSplits(theNumber1, theNumber2);
        }
        aTool.Perform();
        TopoDS_Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aFace);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(theNbFaces, Count(aRes, TopAbs_FACE));
        Assert.Equal(Area(aFace), Area(aRes), 6);
    }

    [Fact]
    public void ShapeDivideArea_ByMaximalArea_LeavesNoFaceLarger()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var aTool = new ShapeUpgrade_ShapeDivideArea(aBox);
        aTool.MaxArea() = 150;
        Assert.True(aTool.Perform());
        TopoDS_Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aRes));
        for (var anExp = new TopExp_Explorer(aRes, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            Assert.True(Area(anExp.Current()) <= 150 + 1e-6, $"{Area(anExp.Current())}");
        }
        Assert.Equal(2200, Area(aRes), 6);
    }

    // A C0 B-spline of degree 2 with a kink at its interior knot of multiplicity 2.
    private static Geom_BSplineCurve KinkedCurve()
    {
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 5);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 1, 0);
        aPoles[3] = new gp_Pnt(2, 0, 0);
        aPoles[4] = new gp_Pnt(3, 2, 0);
        aPoles[5] = new gp_Pnt(4, 0, 0);
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 3;
        aMults[2] = 2;
        aMults[3] = 3;
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 2);
    }

    [Fact]
    public void ShapeDivideContinuity_OnAFreeC0Edge_SplitsItAtTheKink()
    {
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(KinkedCurve()).Edge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anEdge);

        var aTool = new ShapeUpgrade_ShapeDivideContinuity(anEdge);
        Assert.True(aTool.Perform());
        TopoDS_Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(anEdge);
        Assert.True(AllFrozen(aRes));
        Assert.True(aTool.Status(ShapeExtend_DONE1));
        Assert.Equal(TopAbs_WIRE, aRes.ShapeType());
        Assert.Equal(2, Count(aRes, TopAbs_EDGE));
        for (var anExp = new TopExp_Explorer(aRes, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(BRep_Tool.Curve(TopoDS.Edge(anExp.Current()), out _, out _)!.Continuity() >= GeomAbs_C1);
        }
    }

    [Fact]
    public void ShapeDivideContinuity_OnASmoothShape_ReturnsTheCallersShape()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        var aTool = new ShapeUpgrade_ShapeDivideContinuity(aBox);
        Assert.False(aTool.Perform());
        Assert.True(aTool.Result().IsEqual(aBox));
    }

    [Fact]
    public void ShapeConvertToBezier_OnACylinder_ConvertsEverything_TheInputIsUnchanged()
    {
        TopoDS_Shape aCyl = Cylinder();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);

        var aTool = new ShapeUpgrade_ShapeConvertToBezier(aCyl);
        aTool.SetSurfaceConversion(true);
        aTool.Set2dConversion(true);
        aTool.Set3dConversion(true);
        Assert.True(aTool.Perform());
        TopoDS_Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(AllFrozen(aRes));
        for (var anExp = new TopExp_Explorer(aRes, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(anExp.Current());
            Geom_Surface aS = BRep_Tool.Surface(aF)!;
            Assert.True(aS is Geom_BezierSurface or Geom_CylindricalSurface or Geom_RectangularTrimmedSurface,
                        aS.GetType().Name);
        }
        for (var anExp = new TopExp_Explorer(aRes, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            if (!BRep_Tool.Degenerated(anE))
            {
                Geom_Curve? aC = BRep_Tool.Curve(anE, out double cf, out double cl);
                Assert.True(aC is Geom_BezierCurve, $"{aC?.GetType().Name} {cf} {cl} {(aC as Geom_TrimmedCurve)?.BasisCurve().GetType().Name}");
            }
        }
        Assert.Equal(Area(aCyl), Area(aRes), 3);
    }

    [Fact]
    public void ShapeConvertToBezier_WithNothingToConvert_ReturnsTheCallersShape()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var aTool = new ShapeUpgrade_ShapeConvertToBezier(aBox);
        aTool.Set3dConversion(true);
        aTool.Set3dLineConversion(false);
        aTool.Perform();
        TopoDS_Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aBox);
        Assert.True(ReferenceEquals(aRes.TShape(), aBox.TShape()));
    }

    [Fact]
    public void ShapeConvertToBezier_TwiceOnItsOwnFrozenResult_GivesBezierEdgesWithRangeZeroOne()
    {
        TopoDS_Shape aCyl = Cylinder();
        var aTool = new ShapeUpgrade_ShapeConvertToBezier(aCyl);
        aTool.SetSurfaceConversion(true);
        aTool.Set2dConversion(true);
        aTool.Set3dConversion(true);
        aTool.Perform();
        TopoDS_Shape aFirst = aTool.Result();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);

        // the second run changes the frozen Bezier curves OCCT changes in place (decision 8401)
        var aTool2 = new ShapeUpgrade_ShapeConvertToBezier(aFirst);
        aTool2.SetSurfaceConversion(true);
        aTool2.Set2dConversion(true);
        aTool2.Set3dConversion(true);
        aTool2.Perform();
        TopoDS_Shape aSecond = aTool2.Result();

        aSnap.AssertUnchanged(aFirst);
        Assert.True(AllFrozen(aSecond));
        for (var anExp = new TopExp_Explorer(aSecond, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            if (BRep_Tool.Curve(anE, out double f, out double l) is Geom_BezierCurve)
            {
                Assert.Equal(0.0, f);
                Assert.Equal(1.0, l);
            }
        }
    }

    [Fact]
    public void RemoveInternalWires_RemovesTheHolesAndTheFaceInside()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(5, 5, -1), new gp_Dir(0, 0, 1)), 1, 12).Shape();
        TopoDS_Shape aHoled = new BRepAlgoAPI_Cut(aBox, aCyl).Shape();
        Assert.Equal(7, Count(aHoled, TopAbs_FACE));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aHoled);

        var aTool = new ShapeUpgrade_RemoveInternalWires(aHoled);
        aTool.MinArea() = 10;
        Assert.True(aTool.Perform());
        TopoDS_Shape aRes = aTool.GetResult();

        aSnap.AssertUnchanged(aHoled);
        Assert.True(AllFrozen(aRes));
        Assert.True(aTool.Status(ShapeExtend_DONE1));
        Assert.True(aTool.Status(ShapeExtend_DONE2));
        Assert.Equal(2, aTool.RemovedWires().Length());
        Assert.Equal(1, aTool.RemovedFaces().Length());
        Assert.Equal(6, Count(aRes, TopAbs_FACE));
        Assert.Equal(6, Count(aRes, TopAbs_WIRE));
    }

    [Fact]
    public void ShellSewing_SewsAShellOfSeparateFaces()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        var B = new BRep_Builder();
        var aShell = new TopoDS_Shell();
        B.MakeShell(ref aShell);
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            B.Add(aShell, new BRepBuilderAPI_Copy(anExp.Current()).Shape());
        }
        aShell.TShape()!.FreezeGraph();
        Assert.Equal(24, Count(aShell, TopAbs_EDGE));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);

        TopoDS_Shape aRes = new ShapeUpgrade_ShellSewing().ApplySewing(aShell, 1e-6);

        aSnap.AssertUnchanged(aShell);
        Assert.False(aRes.IsNull());
        Assert.True(AllFrozen(aRes));
        Assert.Equal(12, Count(aRes, TopAbs_EDGE));
        Assert.Equal(8, Count(aRes, TopAbs_VERTEX));
    }

    [Fact]
    public void ShellSewing_WithoutShells_ReturnsANullShape()
    {
        TopoDS_Shape anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Shape();
        Assert.True(new ShapeUpgrade_ShellSewing().ApplySewing(anEdge).IsNull());
    }

    [Fact]
    public void C0BSplineToSequenceOfC1BSplineCurve_SplitsAtTheKink_TheFrozenCurveIsUnchanged()
    {
        Geom_BSplineCurve aCurve = KinkedCurve();
        aCurve.Freeze();
        NCollection_HSequence<Geom_BoundedCurve>? aSeq = null;

        Assert.True(ShapeUpgrade.C0BSplineToSequenceOfC1BSplineCurve(aCurve, ref aSeq));

        Assert.Equal(2, aSeq!.Length());
        Assert.Equal(3, aCurve.NbKnots());
        Assert.Equal(new gp_Pnt(2, 0, 0).X(), ((Geom_BSplineCurve)aSeq.Value(1)).EndPoint().X(), 12);
        Assert.Equal(new gp_Pnt(2, 0, 0).X(), ((Geom_BSplineCurve)aSeq.Value(2)).StartPoint().X(), 12);

        // a C1 curve: false, the sequence untouched
        NCollection_HSequence<Geom_BoundedCurve>? aNone = null;
        Assert.False(ShapeUpgrade.C0BSplineToSequenceOfC1BSplineCurve((Geom_BSplineCurve)aSeq.Value(1), ref aNone));
        Assert.Null(aNone);
    }

    [Fact]
    public void SplitCurve3d_SplitsABSplineAtTheValuesGiven_TheCurveGivenIsUnchanged()
    {
        Geom_BSplineCurve aCurve = KinkedCurve();
        aCurve.Freeze();
        var aTool = new ShapeUpgrade_SplitCurve3d();
        aTool.Init(aCurve, 0, 2);
        var aValues = new NCollection_HSequence<double>();
        aValues.Append(0.5);
        aValues.Append(1.5);
        aValues.Append(3.0); // outside: ignored
        aTool.SetSplitValues(aValues);
        aTool.Build(true);

        Assert.True(aTool.Status(ShapeExtend_DONE1));
        Assert.True(aTool.Status(ShapeExtend_DONE3));
        Assert.Equal(4, aTool.SplitValues()!.Length());
        Assert.Equal(3, aTool.GetCurves()!.Length());
        Assert.Equal(0.5, aTool.GetCurves()!.Value(1).LastParameter(), 12);
        Assert.Equal(3, aCurve.NbKnots());
    }

    [Fact]
    public void ConvertCurve3dToBezier_OnACircle_GivesBezierArcs()
    {
        var aCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 2);
        var aTool = new ShapeUpgrade_ConvertCurve3dToBezier();
        aTool.Init(aCircle, 0, 2 * Math.PI);
        aTool.Perform(true);

        Assert.True(aTool.Status(ShapeExtend_DONE1));
        NCollection_HArray1<Geom_Curve> aCurves = aTool.GetCurves()!;
        Assert.True(aCurves.Length() > 1);
        for (int i = 1; i <= aCurves.Length(); i++)
        {
            Assert.IsType<Geom_BezierCurve>(aCurves.Value(i));
        }
        Assert.Equal(aCurves.Length() + 1, aTool.SplitParams().Length());
    }

    [Fact]
    public void SplitCurve2dContinuity_RemovesAKnotWithinTheTolerance()
    {
        // a straight 2D B-spline with a needless knot of full multiplicity: C0 nominally, the knot can be removed
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i] = new gp_Pnt2d(i - 1, 0);
        }
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 3;
        aMults[2] = 2;
        aMults[3] = 3;
        var aCurve = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 2);
        aCurve.Freeze();

        var aTool = new ShapeUpgrade_SplitCurve2dContinuity();
        aTool.SetCriterion(GeomAbs_C1);
        aTool.Init(aCurve, 0, 2);
        aTool.Compute();

        Assert.True(aTool.Status(ShapeExtend_DONE3));
        Assert.Equal(2, aTool.SplitValues()!.Length());
        Assert.Equal(2, aCurve.Multiplicity(2));
    }
}

/// <summary>ShapeAlgo's process-wide AlgoContainer (plan 2.3, decision 8403), in the non-parallel collection.</summary>
[Collection(nameof(ShapeAlgoGlobalStateCollection))]
public class ShapeAlgoTests
{
    [Fact]
    public void Init_CreatesTheDefaultContainer_SetAlgoContainerReplacesIt()
    {
        ShapeAlgo.Init();
        ShapeAlgo_AlgoContainer? aDefault = ShapeAlgo.AlgoContainer();
        Assert.NotNull(aDefault);
        Assert.NotNull(aDefault.ToolContainer());
        try
        {
            var aMine = new ShapeAlgo_AlgoContainer();
            ShapeAlgo.SetAlgoContainer(aMine);
            Assert.Same(aMine, ShapeAlgo.AlgoContainer());
            ShapeAlgo.Init(); // once per process: doesn't replace it
            Assert.Same(aMine, ShapeAlgo.AlgoContainer());
        }
        finally
        {
            ShapeAlgo.SetAlgoContainer(aDefault);
        }
    }

    [Fact]
    public void AlgoContainer_OuterWireAndC0ShapeToC1Shape()
    {
        var aContainer = new ShapeAlgo_AlgoContainer();
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        TopoDS_Face aFace = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        Assert.True(aContainer.OuterWire(aFace).IsSame(ShapeAnalysis.OuterWire(aFace)));
        Assert.True(aContainer.C0ShapeToC1Shape(aBox, 1e-3).IsEqual(aBox));
        Assert.IsType<ShapeFix_Shape>(aContainer.ToolContainer()!.FixShape());
    }

    [Fact]
    public void HomoWires_GivesWiresWithTheSameNumberOfEdges()
    {
        var aContainer = new ShapeAlgo_AlgoContainer();
        TopoDS_Wire aW1 = new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), true).Wire();
        TopoDS_Wire aW2 = new BRepBuilderAPI_MakePolygon(new gp_Pnt(0, 0, 1), new gp_Pnt(2, 0, 1), new gp_Pnt(2, 2, 1),
                                                         new gp_Pnt(0, 2, 1), true).Wire();
        Assert.True(aContainer.HomoWires(aW1, aW2, out TopoDS_Wire aOut1, out TopoDS_Wire aOut2, false));
        int n1 = 0, n2 = 0;
        for (var it = new TopoDS_Iterator(aOut1); it.More(); it.Next())
        {
            n1++;
        }
        for (var it = new TopoDS_Iterator(aOut2); it.More(); it.Next())
        {
            n2++;
        }
        Assert.Equal(n1, n2);
    }
}

[CollectionDefinition(nameof(ShapeAlgoGlobalStateCollection), DisableParallelization = true)]
public sealed class ShapeAlgoGlobalStateCollection;
