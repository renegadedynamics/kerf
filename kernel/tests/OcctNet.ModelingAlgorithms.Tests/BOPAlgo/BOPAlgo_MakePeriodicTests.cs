// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_MakePeriodic (unit 6G): periodicity in X, Y and Z with and without trimming, repetitions, twins, the
/// history, frozen inputs left unchanged, frozen results, and serial = parallel.
/// </summary>
public class BOPAlgo_MakePeriodicTests
{
    internal static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    internal static double Volume(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static double Area(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    // The box of the corpus's periodicity/A1, trimmed to 5 x 8 x 12 (only 10 in Z: the box is shorter)
    private static BOPAlgo_MakePeriodic TrimmedBox(TopoDS_Shape theBox, bool theParallel = false)
    {
        BOPAlgo_MakePeriodic aMaker = new();
        aMaker.SetShape(theBox);
        aMaker.MakePeriodic(0, true, 5);
        aMaker.SetTrimmed(0, false, 2);
        aMaker.MakePeriodic(1, true, 8);
        aMaker.SetTrimmed(1, false, 1);
        aMaker.MakePeriodic(2, true, 12);
        aMaker.SetTrimmed(2, false, -1);
        aMaker.SetRunParallel(theParallel);
        aMaker.Perform();
        return aMaker;
    }

    private static TopoDS_Shape FaceAt(in TopoDS_Shape theShape, double theX)
    {
        for (TopExp_Explorer anExp = new(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().X() - theX) < 1e-9)
            {
                return anExp.Current();
            }
        }
        return new TopoDS_Shape();
    }

    private static TopoDS_Shape Box(double theDX = 10.0, double theDY = 10.0, double theDZ = 10.0)
        => new BRepPrimAPI_MakeBox(theDX, theDY, theDZ).Shape();

    [Fact]
    public void A_box_is_trimmed_to_the_periods_and_repeated()
    {
        BOPAlgo_MakePeriodic aMaker = TrimmedBox(Box());
        Assert.False(aMaker.HasErrors());
        TopoDS_Shape aP = aMaker.Shape();
        Assert.Equal(1, Count(aP, TopAbs_SOLID));
        Assert.Equal(6, Count(aP, TopAbs_FACE));
        Assert.Equal(340.0, Area(aP), 1e-9);
        Assert.Equal(400.0, Volume(aP), 1e-9);

        aMaker.RepeatShape(0, 5);
        Assert.Equal(6, Count(aMaker.RepeatedShape(), TopAbs_SOLID));
        aMaker.RepeatShape(1, 5);
        aMaker.RepeatShape(2, 5);
        Assert.False(aMaker.HasWarnings());
        TopoDS_Shape aR = aMaker.RepeatedShape();
        Assert.Equal(216, Count(aR, TopAbs_SOLID));
        Assert.Equal(936, Count(aR, TopAbs_FACE));
        Assert.Equal(588, Count(aR, TopAbs_VERTEX));
        Assert.Equal(73440.0, Area(aR), 1e-6);
        Assert.Equal(86400.0, Volume(aR), 1e-6);

        // the repetitions are cleared, the periodic shape stays: a copy of the one handed out, which the gluing of the
        // repetitions wrote into (natively the same shape, changed in place)
        aMaker.ClearRepetitions();
        Assert.True(aMaker.RepeatedShape().IsNull());
        Assert.False(aMaker.Shape().IsSame(aP));
        Assert.Equal(400.0, Volume(aMaker.Shape()), 1e-9);
        Assert.True(aP.TShape()!.IsFrozen);
    }

    [Fact]
    public void The_faces_of_the_trimmed_box_have_their_twins_in_X_and_Y()
    {
        // The box is 10 high and the Z period 12: its Z faces don't touch their copies and have no twins
        BOPAlgo_MakePeriodic aMaker = TrimmedBox(Box());
        TopoDS_Shape aP = aMaker.Shape();
        for (TopExp_Explorer anExp = new(aP, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            var aProps = new GProp_GProps();
            BRepGProp.SurfaceProperties(anExp.Current(), aProps);
            Point3D aC = aProps.CentreOfMass();
            bool anIsZ = Math.Abs(aC.Z()) < 1e-9 || Math.Abs(aC.Z() - 10.0) < 1e-9;
            Assert.Equal(anIsZ ? 0 : 1, aMaker.GetTwins(anExp.Current()).Count);
        }
    }

    [Fact]
    public void Repetitions_with_and_without_publishing_in_between_give_the_same_solids()
    {
        // Unpublished, the repetitions glue unfrozen shapes; published (frozen), non-destructively
        BOPAlgo_MakePeriodic aMaker = TrimmedBox(Box());
        aMaker.InternalRepeatShape(0, 2);
        aMaker.InternalRepeatShape(1, -2);
        aMaker.InternalRepeatShape(2, 1);
        Assert.False(aMaker.HasWarnings());
        Assert.Equal(18, Count(aMaker.RepeatedShape(), TopAbs_SOLID));

        // the periodic shape published first: its frozen sub-shapes are shared by the shape to repeat
        BOPAlgo_MakePeriodic aMaker2 = TrimmedBox(Box());
        aMaker2.Shape();
        aMaker2.InternalRepeatShape(0, 2);
        aMaker2.InternalRepeatShape(1, -2);
        aMaker2.InternalRepeatShape(2, 1);
        Assert.False(aMaker2.HasWarnings());
        // the gluing of frozen sub-shapes is non-destructive: the same solids, built differently
        Assert.Equal(18, Count(aMaker2.RepeatedShape(), TopAbs_SOLID));
        Assert.Equal(Volume(aMaker.RepeatedShape()), Volume(aMaker2.RepeatedShape()), 1e-9);
        Assert.Equal(Count(aMaker.RepeatedShape(), TopAbs_FACE), Count(aMaker2.RepeatedShape(), TopAbs_FACE));
    }

    [Fact]
    public void Opposite_sides_are_twins()
    {
        // A box made periodic along X with its own length: its ends are associated
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new Point3D(0, 0, 0), 4.0, 3.0, 2.0).Shape();
        BOPAlgo_MakePeriodic aMaker = new();
        aMaker.SetShape(aBox);
        aMaker.MakeXPeriodic(true, 4.0);
        aMaker.Perform();
        Assert.False(aMaker.HasErrors());
        TopoDS_Shape aP = aMaker.Shape();
        TopoDS_Shape aXMin = FaceAt(aP, 0.0), aXMax = FaceAt(aP, 4.0);
        Assert.True(aMaker.GetTwins(aXMin).Single().IsSame(aXMax));
        Assert.True(aMaker.GetTwins(aXMax).Single().IsSame(aXMin));
        Assert.Empty(aMaker.GetTwins(aBox));

        // After two repetitions the group of the ends holds the four faces at x = 0, 4, 8 and 12. The periodic shape
        // was handed out (frozen), so the repetition works on a copy of it: its faces are found again
        aMaker.RepeatShape(0, 2);
        TopoDS_Shape aR = aMaker.RepeatedShape();
        aXMin = FaceAt(aMaker.Shape(), 0.0);
        Assert.Equal(3, Count(aR, TopAbs_SOLID));
        IReadOnlyList<TopoDS_Shape> aTwins = [.. aMaker.GetTwins(aXMin)];
        Assert.Equal(3, aTwins.Count);
        Assert.All(aTwins, s => Assert.True(s.TShape()!.IsFrozen));
    }

    [Fact]
    public void The_history_answers_for_the_input_and_the_input_is_unchanged()
    {
        TopoDS_Shape aBox = Box();
        string aText = BRepTextComparison.Write(aBox);
        BOPAlgo_MakePeriodic aMaker = TrimmedBox(aBox);
        BRepTools_History aHistory = aMaker.History()!;
        Assert.Equal(aText, BRepTextComparison.Write(aBox));
        // the box is trimmed in all directions: its solid is modified, and the result is frozen
        Assert.True(aHistory.HasModified());
        TopoDS_Shape aSolid = new TopExp_Explorer(aBox, TopAbs_SOLID).Current();
        Assert.True(aHistory.Modified(aSolid).Single().IsSame(new TopExp_Explorer(aMaker.Shape(), TopAbs_SOLID).Current()));
        Assert.True(aMaker.Shape().TShape()!.IsFrozen);

        // repetitions add the generated copies
        aMaker.RepeatShape(0, 1);
        Assert.Equal(2, aMaker.History()!.Generated(aSolid).Count);
        Assert.True(aMaker.RepeatedShape().TShape()!.IsFrozen);
        Assert.Equal(aText, BRepTextComparison.Write(aBox));
    }

    [Fact]
    public void An_untrimmed_frozen_input_is_never_written()
    {
        // Untrimmed, the input itself is split by its translated copies (non-destructively, as OCCT does)
        TopoDS_Shape aBox = Box(5.0, 8.0, 3.0);
        string aText = BRepTextComparison.Write(aBox);
        BOPAlgo_MakePeriodic aMaker = new();
        aMaker.SetShape(aBox);
        aMaker.MakeXPeriodic(true, 5.0);
        aMaker.MakeYPeriodic(true, 8.0);
        aMaker.Perform();
        Assert.False(aMaker.HasErrors());
        Assert.Equal(aText, BRepTextComparison.Write(aBox));
        Assert.Equal(120.0, Volume(aMaker.Shape()), 1e-9);
    }

    [Fact]
    public void No_periodicity_is_an_error()
    {
        BOPAlgo_MakePeriodic aMaker = new();
        aMaker.SetShape(Box());
        aMaker.MakeXPeriodic(true, 0.0);
        aMaker.MakeYPeriodic(false, 3.0);
        aMaker.Perform();
        Assert.True(aMaker.HasError(typeof(BOPAlgo_AlertNoPeriodicityRequired)));
        Assert.Null(aMaker.History());
    }

    [Fact]
    public void The_parameters_and_their_accessors_are_OCCT_s()
    {
        BOPAlgo_MakePeriodic aMaker = new();
        Assert.True(aMaker.IsInputXTrimmed());
        aMaker.SetTrimmed(1, false, 3.0); // not periodic in Y: ignored
        Assert.True(aMaker.IsInputYTrimmed());
        aMaker.MakePeriodic(4, true, 2.5); // 4 % 3 = Y
        Assert.True(aMaker.IsYPeriodic());
        Assert.Equal(2.5, aMaker.YPeriod());
        aMaker.SetTrimmed(-1, false, 3.0); // |-1 % 3| = Y
        Assert.Equal(3.0, aMaker.YPeriodFirst());
        // OCCT: SetYTrimmed takes the start as a bool
        aMaker.SetYTrimmed(false, true);
        Assert.Equal(1.0, aMaker.YPeriodFirst());
        aMaker.MakeYPeriodic(false, 2.5);
        Assert.Equal(0.0, aMaker.YPeriod());

        var aParams = new BOPAlgo_MakePeriodic.PeriodicityParams();
        Assert.True(aParams.myIsTrimmed[2]);
        Assert.Equal(new BOPAlgo_MakePeriodic.PeriodicityParams(), aParams);
        aParams.myPeriodic[0] = true;
        aParams.myPeriod[0] = 2.0;
        Assert.NotEqual(new BOPAlgo_MakePeriodic.PeriodicityParams(), aParams);
        aMaker.SetPeriodicityParameters(aParams);
        Assert.Equal(aParams, aMaker.PeriodicityParameters());
        Assert.Equal(2.0, aMaker.XPeriod());
        aMaker.Clear();
        Assert.False(aMaker.IsXPeriodic());
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(1, 1, -1), new Direction3D(0, 0, 1)), 2.0, 5.0).Shape();
        DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            BOPAlgo_MakePeriodic aMaker = new();
            aMaker.SetShape(aCylinder);
            aMaker.MakeXPeriodic(true, 3.0);
            aMaker.SetXTrimmed(false);
            aMaker.MakeZPeriodic(true, 4.0);
            aMaker.SetRunParallel(true);
            aMaker.Perform();
            aMaker.RepeatShape(0, 2);
            aMaker.RepeatShape(2, -1);
            return BRepTextComparison.Write(aMaker.RepeatedShape())
                   + DeterminismHarness.HistoryText([aCylinder], aMaker.RepeatedShape(), aMaker.History());
        }, 2);
    }
}
