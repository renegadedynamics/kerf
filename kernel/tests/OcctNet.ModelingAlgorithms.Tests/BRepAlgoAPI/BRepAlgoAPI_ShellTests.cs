// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using OcctNet.TestHarness;
using static Occt.BOPAlgo_Operation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests.BRepAlgoAPI;

/// <summary>
/// The Boolean API shell of unit 6A (decision 5004) over the stub builder (decision 6181): publication, frozen
/// inputs, histories, errors, the options struct and the functional twins. The stub handles arguments that don't
/// interfere, whose results are OCCT's: the operations on disjoint boxes.
/// </summary>
public class BRepAlgoAPI_ShellTests
{
    private static TopoDS_Shape Box(double x, double y = 0, double z = 0, double d = 1) =>
        new BRepPrimAPI_MakeBox(new gp_Pnt(x, y, z), d, d, d).Shape();

    private static TopoDS_Shape UnfrozenBox(double x) =>
        new BRepPrimAPI_MakeBox(new gp_Pnt(x, 0, 0), 1, 1, 1).InternalShape();

    private static NCollection_List<TopoDS_Shape> List(params TopoDS_Shape[] theShapes) => new(theShapes);

    private static bool AllFrozen(in TopoDS_Shape theShape)
    {
        if (theShape.TShape() is { } aT && !aT.IsFrozen())
        {
            return false;
        }
        for (TopoDS_Iterator anIt = new(theShape); anIt.More(); anIt.Next())
        {
            if (!AllFrozen(anIt.Value()))
            {
                return false;
            }
        }
        return true;
    }

    private static HashSet<TopoDS_TShape> TShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        HashSet<TopoDS_TShape> aSet = new(ReferenceEqualityComparer.Instance);
        for (TopExp_Explorer anExp = new(theShape, theType); anExp.More(); anExp.Next())
        {
            aSet.Add(anExp.Current().TShape()!);
        }
        return aSet;
    }

    [Fact]
    public void Fuse_DisjointBoxes_SharesTheArgumentsAndFreezesTheResult()
    {
        TopoDS_Shape a = Box(0), b = Box(5);
        ShapeSnapshot sa = ShapeSnapshot.Of(a), sb = ShapeSnapshot.Of(b);

        var aFuse = new BRepAlgoAPI_Fuse(a, b);
        Assert.True(aFuse.IsDone());
        Assert.False(aFuse.HasErrors());

        TopoDS_Shape anInternal = aFuse.InternalShape();
        Assert.False(anInternal.TShape()!.IsFrozen()); // the internal read doesn't publish
        TopoDS_Shape aResult = aFuse.Shape();
        Assert.Equal(TopAbs_COMPOUND, aResult.ShapeType());
        Assert.True(AllFrozen(aResult));

        // the result shares every argument sub-shape (nothing split)
        Assert.True(TShapes(aResult, TopAbs_FACE).SetEquals(TShapes(a, TopAbs_FACE).Concat(TShapes(b, TopAbs_FACE))));
        Assert.Equal(2, TShapes(aResult, TopAbs_SOLID).Count);

        sa.AssertUnchanged(a);
        sb.AssertUnchanged(b);
        Assert.False(aFuse.IsDeleted(a));
        Assert.False(aFuse.HasModified());
        Assert.Empty(aFuse.Modified(a));
        Assert.Empty(aFuse.Generated(a));
    }

    [Fact]
    public void Cut_DisjointTool_IsDeletedInTheHistory()
    {
        TopoDS_Shape a = Box(0), b = Box(5);
        var aCut = new BRepAlgoAPI_Cut(a, b);
        Assert.True(aCut.IsDone());
        TopoDS_Shape aResult = aCut.Shape();
        Assert.Equal(TShapes(a, TopAbs_SOLID), TShapes(aResult, TopAbs_SOLID));
        Assert.True(aCut.IsDeleted(b));
        for (TopExp_Explorer anExp = new(b, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.True(aCut.IsDeleted(anExp.Current()));
        }
        Assert.False(aCut.IsDeleted(a));
        Assert.True(aCut.HasDeleted());

        BRepTools_History aHistory = aCut.History()!;
        Assert.True(aHistory.IsRemoved(b));
        Assert.True(AllFrozen(aResult));
    }

    [Fact]
    public void Cut21_ThroughTheConstructorFlag_KeepsTheTool()
    {
        TopoDS_Shape a = Box(0), b = Box(5);
        var aFiller = new BOPAlgo_PaveFiller();
        aFiller.SetArguments(List(a, b));
        aFiller.Perform();
        Assert.False(aFiller.HasErrors());

        var aCut = new BRepAlgoAPI_Cut(a, b, aFiller, false);
        Assert.Equal(BOPAlgo_CUT21, aCut.Operation());
        Assert.True(aCut.IsDone());
        Assert.Equal(TShapes(b, TopAbs_SOLID), TShapes(aCut.Shape(), TopAbs_SOLID));
        Assert.True(aCut.IsDeleted(a));
    }

    [Fact]
    public void Common_Disjoint_IsEmpty()
    {
        var aCommon = new BRepAlgoAPI_Common(Box(0), Box(5));
        Assert.True(aCommon.IsDone());
        TopoDS_Shape aResult = aCommon.Shape();
        Assert.Equal(TopAbs_COMPOUND, aResult.ShapeType());
        Assert.False(new TopoDS_Iterator(aResult).More());
    }

    [Fact]
    public void UnfrozenInputs_ThroughTheClassApi_AreNotWritten()
    {
        TopoDS_Shape a = UnfrozenBox(0), b = UnfrozenBox(5);
        string ta = BRepTextComparison.Write(a), tb = BRepTextComparison.Write(b);
        var aFuse = new BRepAlgoAPI_Fuse(a, b);
        Assert.True(aFuse.IsDone());
        // as in OCCT, putting an unfrozen argument into the result clears its Free flag (TopoDS_Builder::Add); nothing
        // else changes
        BRepTextDiff da = BRepTextComparison.Compare(ta, BRepTextComparison.Write(a));
        Assert.True(da.IsIdenticalMaskingFree, da.ToString());
        Assert.Equal(1, da.FreeFlagDifferences);
        Assert.True(BRepTextComparison.Compare(tb, BRepTextComparison.Write(b)).IsIdenticalMaskingFree);
        Assert.False(a.TShape()!.IsFrozen()); // the class API doesn't freeze its inputs until the result is published
        aFuse.Shape();
        Assert.True(a.TShape()!.IsFrozen()); // the published result shares them
    }

    [Fact]
    public void ModifiedAndHistory_FreezeTheWholeResult()
    {
        var aFuse = new BRepAlgoAPI_Fuse(UnfrozenBox(0), UnfrozenBox(5));
        Assert.False(aFuse.InternalShape().TShape()!.IsFrozen());
        aFuse.Modified(aFuse.Shape1());
        Assert.True(AllFrozen(aFuse.InternalShape()));

        var aFuse2 = new BRepAlgoAPI_Fuse(UnfrozenBox(0), UnfrozenBox(5));
        Assert.NotNull(aFuse2.History());
        Assert.True(AllFrozen(aFuse2.InternalShape()));

        var aFuse3 = new BRepAlgoAPI_Fuse(UnfrozenBox(0), UnfrozenBox(5));
        Assert.Empty(aFuse3.SectionEdges());
        Assert.True(AllFrozen(aFuse3.InternalShape()));
    }

    [Fact]
    public void History_NotFilled_IsNull()
    {
        var aFuse = new BRepAlgoAPI_Fuse();
        aFuse.SetArguments(List(Box(0)));
        aFuse.SetTools(List(Box(5)));
        aFuse.SetToFillHistory(false);
        aFuse.Build();
        Assert.True(aFuse.IsDone());
        Assert.False(aFuse.HasHistory());
        Assert.Null(aFuse.History());
        Assert.False(aFuse.IsDeleted(aFuse.Shape2()));
    }

    [Fact]
    public void GeneralFuse_TwoArguments_And_OneArgumentFails()
    {
        var aGF = new BRepAlgoAPI_BuilderAlgo();
        aGF.SetArguments(List(Box(0), Box(5)));
        aGF.Build();
        Assert.True(aGF.IsDone());
        Assert.Equal(2, TShapes(aGF.Shape(), TopAbs_SOLID).Count);

        // OCCT's BOPAlgo_Builder::CheckData wants two arguments
        var aGF1 = new BRepAlgoAPI_BuilderAlgo();
        aGF1.SetArguments(List(Box(0)));
        aGF1.Build();
        Assert.False(aGF1.IsDone());
        Assert.True(aGF1.HasError(typeof(BOPAlgo_AlertTooFewArguments)));
    }

    [Fact]
    public void Errors_TooFewArguments_BOPNotSet_NullInput()
    {
        var aBOP = new BRepAlgoAPI_BooleanOperation();
        aBOP.SetArguments(List(Box(0)));
        aBOP.Build();
        Assert.True(aBOP.HasError(typeof(BOPAlgo_AlertTooFewArguments)));
        Assert.True(aBOP.Shape().IsNull()); // OCCT's BRepAlgoAPI_Algo::Shape() doesn't check IsDone

        aBOP.SetTools(List(Box(5)));
        aBOP.Build();
        Assert.True(aBOP.HasError(typeof(BOPAlgo_AlertBOPNotSet)));
        using (StringWriter aText = new())
        {
            aBOP.DumpErrors(aText);
            Assert.Equal("BOPAlgo_AlertBOPNotSet\n", aText.ToString());
        }

        var aFuse = new BRepAlgoAPI_Fuse(Box(0), new TopoDS_Shape());
        Assert.True(aFuse.HasError(typeof(BOPAlgo_AlertNullInputShapes)));
        Assert.False(aFuse.IsDone());
    }

    [Fact]
    public void InterferingArguments_StubFillerFails()
    {
        TopoDS_Shape a = Box(0), b = Box(0.5);
        var aFuse = new BRepAlgoAPI_Fuse(a, b);
        Assert.True(aFuse.HasErrors());
        Assert.True(aFuse.HasError(typeof(BOPAlgo_AlertIntersectionFailed)));
        Assert.False(aFuse.IsDone());
        using StringWriter aText = new();
        aFuse.DumpErrors(aText);
        Assert.Contains("BOPAlgo_AlertIntersectionFailed", aText.ToString());
    }

    [Fact]
    public void Cut_ToolOfLowerDimension_IsNotAllowed()
    {
        // OCCT's BOPAlgo_BOP::CheckData: the tools' smallest dimension must be at least the objects' largest
        TopoDS_Shape aFace = new BRepBuilderAPI_MakeFace(new gp_Pln(new gp_Pnt(10, 10, 10), new gp_Dir(0, 0, 1)), 0, 1, 0, 1).Shape();
        var aCut = new BRepAlgoAPI_Cut(Box(0), aFace);
        Assert.True(aCut.HasError(typeof(BOPAlgo_AlertBOPNotAllowed)));
    }

    [Fact]
    public void Splitter_KeepsTheObjects()
    {
        TopoDS_Shape a = Box(0), b = Box(5);
        var aSplitter = new BRepAlgoAPI_Splitter();
        aSplitter.SetArguments(List(a));
        aSplitter.SetTools(List(b));
        aSplitter.Build();
        Assert.True(aSplitter.IsDone());
        Assert.Equal(TShapes(a, TopAbs_SOLID), TShapes(aSplitter.Shape(), TopAbs_SOLID));
        Assert.True(aSplitter.IsDeleted(b));

        var aNone = new BRepAlgoAPI_Splitter();
        aNone.SetArguments(List(a));
        aNone.Build();
        Assert.True(aNone.HasError(typeof(BOPAlgo_AlertTooFewArguments)));
    }

    [Fact]
    public void Section_Constructors_DisjointGiveEmptyCompounds()
    {
        TopoDS_Shape a = Box(0);
        gp_Pln aFar = new(new gp_Pnt(0, 0, 10), new gp_Dir(0, 0, 1));

        var s1 = new BRepAlgoAPI_Section(a, Box(5));
        Assert.True(s1.IsDone());
        Assert.False(new TopoDS_Iterator(s1.Shape()).More());
        Assert.True(s1.IsDeleted(a));

        // a plane becomes an unbounded face; one far from the box doesn't interfere, one through it does
        var s2 = new BRepAlgoAPI_Section(a, aFar);
        Assert.Equal(TopAbs_FACE, s2.Shape2().ShapeType());
        Assert.True(s2.IsDone());
        Assert.False(new TopoDS_Iterator(s2.Shape()).More());

        var s2b = new BRepAlgoAPI_Section(a, new gp_Pln(new gp_Pnt(0, 0, 0.5), new gp_Dir(0, 0, 1)));
        Assert.True(s2b.HasError(typeof(BOPAlgo_AlertIntersectionFailed)));

        var s3 = new BRepAlgoAPI_Section(new Geom_Plane(aFar), a);
        Assert.Equal(TopAbs_FACE, s3.Shape1().ShapeType());
        Assert.True(s3.IsDone());

        var s4 = new BRepAlgoAPI_Section(new Geom_Plane(aFar), new Geom_Plane(new gp_Pln(new gp_Pnt(), new gp_Dir(1, 0, 0))), false);
        Assert.Equal(TopAbs_FACE, s4.Shape1().ShapeType());
        Assert.False(s4.IsDone());

        var s5 = new BRepAlgoAPI_Section(a, Box(5), false);
        Assert.False(s5.IsDone());
        s5.Approximation(true);
        s5.ComputePCurveOn1(true);
        s5.ComputePCurveOn2(true);
        s5.Init1(Box(0, 3));
        s5.Init2(Box(9));
        s5.Build();
        Assert.True(s5.IsDone());
        Assert.False(s5.HasAncestorFaceOn1(new TopoDS_Shape(), out TopoDS_Shape aF));
        Assert.True(aF.IsNull());
    }

    [Fact]
    public void Options_DefaultsAndEquality()
    {
        BRepAlgoAPI_Options o = new();
        Assert.Equal(Precision.Confusion(), o.FuzzyValue);
        Assert.Equal(BOPAlgo_GlueEnum.BOPAlgo_GlueOff, o.Glue);
        Assert.False(o.UseOBB);
        Assert.True(o.CheckInverted);
        Assert.False(o.RunParallel);
        Assert.False(o.NonDestructive);
        Assert.False(o.Simplify);
        Assert.NotEqual(o, default);

        BRepAlgoAPI_Options p = new() { FuzzyValue = 1e-5, RunParallel = true };
        BRepAlgoAPI_Options q = new() { FuzzyValue = 1e-5, RunParallel = true };
        Assert.Equal(p, q);
        Assert.True(p == q);
        Assert.Equal(p.GetHashCode(), q.GetHashCode());
        Assert.NotEqual(o, p);
        Assert.True(o != p);

        BRepAlgoAPI_Options n1 = new() { FuzzyValue = double.NaN }, n2 = new() { FuzzyValue = double.NaN };
        Assert.Equal(n1, n2);

        Dictionary<BRepAlgoAPI_Options, int> aCache = new() { [p] = 1 };
        Assert.Equal(1, aCache[q]);
        Assert.Contains("FuzzyValue=1E-05", p.ToString());
        Assert.False(new BRepAlgoAPI_Options { Glue = BOPAlgo_GlueEnum.BOPAlgo_GlueFull }.Equals(o));
    }

    [Fact]
    public void Twins_FrozenResultAndHistory_FreezeTheirInputs()
    {
        TopoDS_Shape a = UnfrozenBox(0), b = UnfrozenBox(5);
        string ta = BRepTextComparison.Write(a);
        TopoDS_Shape aCut = Occt.BRepAlgoAPI.Cut([a], [b], new BRepAlgoAPI_Options(), out BRepTools_History aHistory);
        Assert.True(AllFrozen(aCut));
        Assert.True(AllFrozen(a));
        Assert.True(AllFrozen(b));
        // frozen before the operation: not even the Free flag changes (decision 5002)
        Assert.Equal(ta, BRepTextComparison.Write(a));
        Assert.True(aHistory.IsRemoved(b));
        Assert.False(aHistory.IsRemoved(a));

        TopoDS_Shape aFuse = Occt.BRepAlgoAPI.Fuse([Box(0)], [Box(5)], new BRepAlgoAPI_Options { RunParallel = true });
        Assert.Equal(2, TShapes(aFuse, TopAbs_SOLID).Count);
        TopoDS_Shape aCommon = Occt.BRepAlgoAPI.Common([Box(0)], [Box(5)], new BRepAlgoAPI_Options());
        Assert.False(new TopoDS_Iterator(aCommon).More());
        TopoDS_Shape aSplit = Occt.BRepAlgoAPI.Split([a], [b], new BRepAlgoAPI_Options(), out BRepTools_History aSplitHistory);
        Assert.Equal(TShapes(a, TopAbs_SOLID), TShapes(aSplit, TopAbs_SOLID));
        Assert.True(aSplitHistory.IsRemoved(b));
        TopoDS_Shape aSection = Occt.BRepAlgoAPI.Section([a], [b], new BRepAlgoAPI_Options(), new BOPAlgo_SectionAttribute(true, true, true));
        Assert.False(new TopoDS_Iterator(aSection).More());
    }

    [Fact]
    public void Twins_Errors_ThrowWithTheReport()
    {
        var anError = Assert.Throws<BRepAlgoAPI_Failure>(
            () => Occt.BRepAlgoAPI.Fuse([Box(0)], [Box(0.5)], new BRepAlgoAPI_Options()));
        Assert.Contains("BOPAlgo_AlertIntersectionFailed", anError.Message);
        Assert.True(anError.Report().HasAlert(typeof(BOPAlgo_AlertIntersectionFailed), Message_Gravity.Message_Fail));
        Assert.IsAssignableFrom<StdFail_NotDone>(anError);

        Assert.Throws<BRepAlgoAPI_Failure>(() => Occt.BRepAlgoAPI.Cut([Box(0)], [], new BRepAlgoAPI_Options()));
        Assert.Throws<Standard_NotImplemented>(
            () => Occt.BRepAlgoAPI.Fuse([Box(0)], [Box(5)], new BRepAlgoAPI_Options { Simplify = true }));
    }

    [Fact]
    public void Twin_MatchesTheClassResult_Canonically()
    {
        TopoDS_Shape a = Box(0), b = Box(5), c = Box(0, 7);
        var aFuse = new BRepAlgoAPI_Fuse();
        aFuse.SetArguments(List(a, c));
        aFuse.SetTools(List(b));
        aFuse.Build();
        TopoDS_Shape aClass = aFuse.Shape();
        TopoDS_Shape aTwin = Occt.BRepAlgoAPI.Fuse([a, c], [b], new BRepAlgoAPI_Options());
        CanonicalShapeComparer.AssertIdentical(aClass, aTwin);
    }

    [Fact]
    public void Parallel_GivesTheSerialResult()
    {
        TopoDS_Shape[] aBoxes = [.. Enumerable.Range(0, 12).Select(i => Box(3 * (i % 4), 3 * (i / 4)))];
        DeterminismHarness.AssertSerialEqualsParallel([.. aBoxes], () =>
        {
            var aFuse = new BRepAlgoAPI_Fuse();
            aFuse.SetArguments(List(aBoxes[..6]));
            aFuse.SetTools(List(aBoxes[6..]));
            aFuse.SetRunParallel(true);
            aFuse.Build();
            Assert.True(aFuse.IsDone());
            return (aFuse.Shape(), aFuse.History());
        });
    }

    [Fact]
    public void Options_Forwarded()
    {
        var aFuse = new BRepAlgoAPI_Fuse();
        Assert.Equal(Precision.Confusion(), aFuse.FuzzyValue());
        aFuse.SetFuzzyValue(1e-12);
        Assert.Equal(Precision.Confusion(), aFuse.FuzzyValue());
        aFuse.SetFuzzyValue(1e-3);
        Assert.Equal(1e-3, aFuse.FuzzyValue());
        aFuse.SetUseOBB(true);
        Assert.True(aFuse.UseOBB());
        aFuse.SetRunParallel(true);
        Assert.True(aFuse.RunParallel());
        aFuse.SetGlue(BOPAlgo_GlueEnum.BOPAlgo_GlueShift);
        Assert.Equal(BOPAlgo_GlueEnum.BOPAlgo_GlueShift, aFuse.Glue());
        aFuse.SetNonDestructive(true);
        Assert.True(aFuse.NonDestructive());
        aFuse.SetCheckInverted(false);
        Assert.False(aFuse.CheckInverted());
        Assert.False(aFuse.HasWarnings());
        aFuse.ClearWarnings();
        Assert.NotNull(aFuse.GetReport());
    }

    [Fact]
    public void BuilderSetArguments_Twice_KeepsNothing_OcctIssue()
    {
        // OCCT: BOPAlgo_Builder::SetArguments clears the arguments but not the fence of added ones, so setting the same
        // arguments again leaves none (occt-issues.md, unit 6A)
        TopoDS_Shape a = Box(0), b = Box(5);
        var aBuilder = new BOPAlgo_Builder();
        aBuilder.SetArguments(List(a, b));
        Assert.Equal(2, aBuilder.Arguments().Extent());
        aBuilder.SetArguments(List(a, b));
        Assert.Equal(0, aBuilder.Arguments().Extent());
    }
}
