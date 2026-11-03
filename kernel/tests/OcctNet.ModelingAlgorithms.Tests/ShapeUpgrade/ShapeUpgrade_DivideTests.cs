// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeUpgrade's divide and convert tools, RemoveInternalWires, ShellSewing and the package function (unit 7B), on
/// frozen inputs (decision 999): the caller's shape is never written, the results are frozen, and the caller's own
/// sub-shapes come back where nothing changed. The numbers of faces are the heal group's (split_number,
/// split_two_numbers) and the shapes' own.
/// </summary>
public class ShapeUpgrade_DivideTests
{
    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static bool AllFrozen(in Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            if (!aMap[i].TShape()!.IsFrozen)
            {
                return false;
            }
        }
        return true;
    }

    private static double Area(in Shape theShape)
    {
        var aProps = new Properties();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static Shape Cylinder() => new BRepPrimAPI_MakeCylinder(5, 10).Shape();

    private static Face FirstFace(in Shape theShape) =>
        ShapeCasts.Face(new Explorer(theShape, ShapeType.Face).Current());

    [Fact]
    public void ShapeDivideAngle_OnACylinder_SplitsTheLateralFaceInFour_TheInputIsUnchanged()
    {
        Shape aCyl = Cylinder();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);

        var aTool = new ShapeUpgrade_ShapeDivideAngle(95 * Math.PI / 180, aCyl);
        Assert.True(aTool.Perform());
        Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(AllFrozen(aRes));
        Assert.True(aTool.Status(ShapeExtend_DONE2));
        Assert.Equal(6, Count(aRes, ShapeType.Face));
        Assert.Equal(Area(aCyl), Area(aRes), 6);
        Assert.Equal(95 * Math.PI / 180, aTool.MaxAngle());
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
    }

    [Fact]
    public void ShapeDivideClosed_OnACylinder_SplitsTheLateralFaceInTwo()
    {
        Shape aCyl = Cylinder();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);

        var aTool = new ShapeUpgrade_ShapeDivideClosed(aCyl);
        Assert.True(aTool.Perform());
        Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(4, Count(aRes, ShapeType.Face));
        Assert.Equal(Area(aCyl), Area(aRes), 6);
        Assert.True(new BRepCheck_Analyzer(aRes).IsValid());
    }

    [Theory]
    [InlineData(10, 0, 10)]
    [InlineData(3, 5, 15)]
    public void ShapeDivideArea_ByNumber_SplitsTheSphericalFace(int theNumber1, int theNumber2, int theNbFaces)
    {
        Shape aSphere = new BRepPrimAPI_MakeSphere(10, 0, Math.PI / 2).Shape();
        Face aFace = ShapeCasts.Face(FirstFace(aSphere));
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
        Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aFace);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(theNbFaces, Count(aRes, ShapeType.Face));
        Assert.Equal(Area(aFace), Area(aRes), 6);
    }

    [Fact]
    public void ShapeDivideArea_ByMaximalArea_LeavesNoFaceLarger()
    {
        Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var aTool = new ShapeUpgrade_ShapeDivideArea(aBox);
        aTool.MaxArea() = 150;
        Assert.True(aTool.Perform());
        Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aRes));
        for (var anExp = new Explorer(aRes, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Assert.True(Area(anExp.Current()) <= 150 + 1e-6, $"{Area(anExp.Current())}");
        }
        Assert.Equal(2200, Area(aRes), 6);
    }

    // A C0 B-spline of degree 2 with a kink at its interior knot of multiplicity 2.
    private static BSplineCurve3D KinkedCurve()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 5);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 1, 0);
        aPoles[3] = new Point3D(2, 0, 0);
        aPoles[4] = new Point3D(3, 2, 0);
        aPoles[5] = new Point3D(4, 0, 0);
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 3;
        aMults[2] = 2;
        aMults[3] = 3;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 2);
    }

    [Fact]
    public void ShapeDivideContinuity_OnAFreeC0Edge_SplitsItAtTheKink()
    {
        Edge anEdge = new BRepBuilderAPI_MakeEdge(KinkedCurve()).Edge();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anEdge);

        var aTool = new ShapeUpgrade_ShapeDivideContinuity(anEdge);
        Assert.True(aTool.Perform());
        Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(anEdge);
        Assert.True(AllFrozen(aRes));
        Assert.True(aTool.Status(ShapeExtend_DONE1));
        Assert.Equal(ShapeType.Wire, aRes.ShapeType());
        Assert.Equal(2, Count(aRes, ShapeType.Edge));
        for (var anExp = new Explorer(aRes, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.True(Tool.Curve(ShapeCasts.Edge(anExp.Current()), out _, out _)!.Continuity() >= C1);
        }
    }

    [Fact]
    public void ShapeDivideContinuity_OnASmoothShape_ReturnsTheCallersShape()
    {
        Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        var aTool = new ShapeUpgrade_ShapeDivideContinuity(aBox);
        Assert.False(aTool.Perform());
        Assert.True(aTool.Result().IsEqual(aBox));
    }

    [Fact]
    public void ShapeConvertToBezier_OnACylinder_ConvertsEverything_TheInputIsUnchanged()
    {
        Shape aCyl = Cylinder();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);

        var aTool = new ShapeUpgrade_ShapeConvertToBezier(aCyl);
        aTool.SetSurfaceConversion(true);
        aTool.Set2dConversion(true);
        aTool.Set3dConversion(true);
        Assert.True(aTool.Perform());
        Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(AllFrozen(aRes));
        for (var anExp = new Explorer(aRes, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Face aF = ShapeCasts.Face(anExp.Current());
            Surface aS = Tool.Surface(aF)!;
            Assert.True(aS is BezierSurface or CylindricalSurface or TrimmedSurface,
                        global::Kerf.Engine.Foundation.OcctTypeNames.Of(aS.GetType()));
        }
        for (var anExp = new Explorer(aRes, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            if (!Tool.Degenerated(anE))
            {
                Curve3D? aC = Tool.Curve(anE, out double cf, out double cl);
                Assert.True(aC is BezierCurve3D, $"{aC?.GetType().Name} {cf} {cl} {(aC as TrimmedCurve3D)?.BasisCurve().GetType().Name}");
            }
        }
        Assert.Equal(Area(aCyl), Area(aRes), 3);
    }

    [Fact]
    public void ShapeConvertToBezier_WithNothingToConvert_ReturnsTheCallersShape()
    {
        Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var aTool = new ShapeUpgrade_ShapeConvertToBezier(aBox);
        aTool.Set3dConversion(true);
        aTool.Set3dLineConversion(false);
        aTool.Perform();
        Shape aRes = aTool.Result();

        aSnap.AssertUnchanged(aBox);
        Assert.True(ReferenceEquals(aRes.TShape(), aBox.TShape()));
    }

    [Fact]
    public void ShapeConvertToBezier_TwiceOnItsOwnFrozenResult_GivesBezierEdgesWithRangeZeroOne()
    {
        Shape aCyl = Cylinder();
        var aTool = new ShapeUpgrade_ShapeConvertToBezier(aCyl);
        aTool.SetSurfaceConversion(true);
        aTool.Set2dConversion(true);
        aTool.Set3dConversion(true);
        aTool.Perform();
        Shape aFirst = aTool.Result();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);

        // the second run changes the frozen Bezier curves OCCT changes in place (decision 998)
        var aTool2 = new ShapeUpgrade_ShapeConvertToBezier(aFirst);
        aTool2.SetSurfaceConversion(true);
        aTool2.Set2dConversion(true);
        aTool2.Set3dConversion(true);
        aTool2.Perform();
        Shape aSecond = aTool2.Result();

        aSnap.AssertUnchanged(aFirst);
        Assert.True(AllFrozen(aSecond));
        for (var anExp = new Explorer(aSecond, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            if (Tool.Curve(anE, out double f, out double l) is BezierCurve3D)
            {
                Assert.Equal(0.0, f);
                Assert.Equal(1.0, l);
            }
        }
    }

    [Fact]
    public void RemoveInternalWires_RemovesTheHolesAndTheFaceInside()
    {
        Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        Shape aCyl = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(5, 5, -1), new Direction3D(0, 0, 1)), 1, 12).Shape();
        Shape aHoled = new BRepAlgoAPI_Cut(aBox, aCyl).Shape();
        Assert.Equal(7, Count(aHoled, ShapeType.Face));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aHoled);

        var aTool = new ShapeUpgrade_RemoveInternalWires(aHoled);
        aTool.MinArea() = 10;
        Assert.True(aTool.Perform());
        Shape aRes = aTool.GetResult();

        aSnap.AssertUnchanged(aHoled);
        Assert.True(AllFrozen(aRes));
        Assert.True(aTool.Status(ShapeExtend_DONE1));
        Assert.True(aTool.Status(ShapeExtend_DONE2));
        Assert.Equal(2, aTool.RemovedWires().Length());
        Assert.Equal(1, aTool.RemovedFaces().Length());
        Assert.Equal(6, Count(aRes, ShapeType.Face));
        Assert.Equal(6, Count(aRes, ShapeType.Wire));
    }

    [Fact]
    public void ShellSewing_SewsAShellOfSeparateFaces()
    {
        Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        var B = new BrepBuilder();
        var aShell = new Shell();
        B.MakeShell(ref aShell);
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            B.Add(aShell, new BRepBuilderAPI_Copy(anExp.Current()).Shape());
        }
        aShell.TShape()!.FreezeGraph();
        Assert.Equal(24, Count(aShell, ShapeType.Edge));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);

        Shape aRes = new ShapeUpgrade_ShellSewing().ApplySewing(aShell, 1e-6);

        aSnap.AssertUnchanged(aShell);
        Assert.False(aRes.IsNull);
        Assert.True(AllFrozen(aRes));
        Assert.Equal(12, Count(aRes, ShapeType.Edge));
        Assert.Equal(8, Count(aRes, ShapeType.Vertex));
    }

    [Fact]
    public void ShellSewing_WithoutShells_ReturnsANullShape()
    {
        Shape anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Shape();
        Assert.True(new ShapeUpgrade_ShellSewing().ApplySewing(anEdge).IsNull);
    }

    [Fact]
    public void C0BSplineToSequenceOfC1BSplineCurve_SplitsAtTheKink_TheFrozenCurveIsUnchanged()
    {
        BSplineCurve3D aCurve = KinkedCurve();
        aCurve.Freeze();
        NCollection_HSequence<BoundedCurve3D>? aSeq = null;

        Assert.True(ShapeUpgrade.C0BSplineToSequenceOfC1BSplineCurve(aCurve, ref aSeq));

        Assert.Equal(2, aSeq!.Length());
        Assert.Equal(3, aCurve.KnotCount);
        Assert.Equal(new Point3D(2, 0, 0).X(), ((BSplineCurve3D)aSeq.Value(1)).EndPoint().X(), 12);
        Assert.Equal(new Point3D(2, 0, 0).X(), ((BSplineCurve3D)aSeq.Value(2)).StartPoint().X(), 12);

        // a C1 curve: false, the sequence untouched
        NCollection_HSequence<BoundedCurve3D>? aNone = null;
        Assert.False(ShapeUpgrade.C0BSplineToSequenceOfC1BSplineCurve((BSplineCurve3D)aSeq.Value(1), ref aNone));
        Assert.Null(aNone);
    }

    [Fact]
    public void SplitCurve3d_SplitsABSplineAtTheValuesGiven_TheCurveGivenIsUnchanged()
    {
        BSplineCurve3D aCurve = KinkedCurve();
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
        Assert.Equal(3, aCurve.KnotCount);
    }

    [Fact]
    public void ConvertCurve3dToBezier_OnACircle_GivesBezierArcs()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 2);
        var aTool = new ShapeUpgrade_ConvertCurve3dToBezier();
        aTool.Init(aCircle, 0, 2 * Math.PI);
        aTool.Perform(true);

        Assert.True(aTool.Status(ShapeExtend_DONE1));
        NCollection_HArray1<Curve3D> aCurves = aTool.GetCurves()!;
        Assert.True(aCurves.Length() > 1);
        for (int i = 1; i <= aCurves.Length(); i++)
        {
            Assert.IsType<BezierCurve3D>(aCurves.Value(i));
        }
        Assert.Equal(aCurves.Length() + 1, aTool.SplitParams().Length());
    }

    [Fact]
    public void SplitCurve2dContinuity_RemovesAKnotWithinTheTolerance()
    {
        // a straight 2D B-spline with a needless knot of full multiplicity: C0 nominally, the knot can be removed
        var aPoles = new NCollection_Array1<Point2D>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i] = new Point2D(i - 1, 0);
        }
        var aKnots = new NCollection_Array1<double>(1, 3);
        aKnots[1] = 0;
        aKnots[2] = 1;
        aKnots[3] = 2;
        var aMults = new NCollection_Array1<int>(1, 3);
        aMults[1] = 3;
        aMults[2] = 2;
        aMults[3] = 3;
        var aCurve = new BSplineCurve2D(aPoles, aKnots, aMults, 2);
        aCurve.Freeze();

        var aTool = new ShapeUpgrade_SplitCurve2dContinuity();
        aTool.SetCriterion(C1);
        aTool.Init(aCurve, 0, 2);
        aTool.Compute();

        Assert.True(aTool.Status(ShapeExtend_DONE3));
        Assert.Equal(2, aTool.SplitValues()!.Length());
        Assert.Equal(2, aCurve.Multiplicity(2));
    }
}

/// <summary>ShapeAlgo's process-wide AlgoContainer (plan 2.3, decision 1000), in the non-parallel collection.</summary>
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
        Shape aBox = new BRepPrimAPI_MakeBox(1, 2, 3).Shape();
        Face aFace = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        Assert.True(aContainer.OuterWire(aFace).IsSame(ShapeAnalysis.OuterWire(aFace)));
        Assert.True(aContainer.C0ShapeToC1Shape(aBox, 1e-3).IsEqual(aBox));
        Assert.IsType<ShapeFix_Shape>(aContainer.ToolContainer()!.FixShape());
    }

    [Fact]
    public void HomoWires_GivesWiresWithTheSameNumberOfEdges()
    {
        var aContainer = new ShapeAlgo_AlgoContainer();
        Wire aW1 = new BRepBuilderAPI_MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0), true).Wire();
        Wire aW2 = new BRepBuilderAPI_MakePolygon(new Point3D(0, 0, 1), new Point3D(2, 0, 1), new Point3D(2, 2, 1),
                                                         new Point3D(0, 2, 1), true).Wire();
        Assert.True(aContainer.HomoWires(aW1, aW2, out Wire aOut1, out Wire aOut2, false));
        int n1 = 0, n2 = 0;
        for (var it = new Iterator(aOut1); it.More(); it.Next())
        {
            n1++;
        }
        for (var it = new Iterator(aOut2); it.More(); it.Next())
        {
            n2++;
        }
        Assert.Equal(n1, n2);
    }
}

[CollectionDefinition(nameof(ShapeAlgoGlobalStateCollection), DisableParallelization = true)]
public sealed class ShapeAlgoGlobalStateCollection;
