// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_MakeConnected (unit 6G): gluing, the materials on both sides of the shared faces, origins and history,
/// periodicity and repetition of the connected shape, frozen arguments left unchanged, frozen results, and
/// serial = parallel.
/// </summary>
public class BOPAlgo_MakeConnectedTests
{
    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType) => BOPAlgo_MakePeriodicTests.Count(theShape, theType);

    // [0, 10]^3 and [10, 20] x [0, 10]^2: they touch along the face x = 10
    private static TopoDS_Shape BoxA() => new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    private static TopoDS_Shape BoxB() => new BRepPrimAPI_MakeBox(new Point3D(10, 0, 0), 10.0, 10.0, 10.0).Shape();

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

    private static BOPAlgo_MakeConnected Connected(params TopoDS_Shape[] theShapes)
    {
        BOPAlgo_MakeConnected aTool = new();
        foreach (TopoDS_Shape aS in theShapes)
        {
            aTool.AddArgument(aS);
        }
        aTool.Perform();
        return aTool;
    }

    [Fact]
    public void Touching_boxes_share_a_face_with_a_material_on_each_side()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        string aTextA = BRepTextComparison.Write(aA), aTextB = BRepTextComparison.Write(aB);
        BOPAlgo_MakeConnected aTool = Connected(aA, aB);
        Assert.False(aTool.HasErrors());
        TopoDS_Shape aC = aTool.Shape();
        Assert.Equal(2, Count(aC, TopAbs_SOLID));
        Assert.Equal(11, Count(aC, TopAbs_FACE));
        Assert.True(aC.TShape()!.IsFrozen);
        Assert.Equal(aTextA, BRepTextComparison.Write(aA));
        Assert.Equal(aTextB, BRepTextComparison.Write(aB));

        // The shared face: A on one side, B on the other
        TopoDS_Shape aShared = FaceAt(aC, 10.0);
        List<TopoDS_Shape> aSides = [.. aTool.MaterialsOnPositiveSide(aShared), .. aTool.MaterialsOnNegativeSide(aShared)];
        Assert.Equal(2, aSides.Count);
        Assert.Contains(aSides, s => s.IsSame(aA));
        Assert.Contains(aSides, s => s.IsSame(aB));
        // An outer face: one material, on its inner side
        TopoDS_Shape anOuter = FaceAt(aC, 0.0);
        Assert.Equal(1, aTool.MaterialsOnPositiveSide(anOuter).Count + aTool.MaterialsOnNegativeSide(anOuter).Count);

        // Origins: the shared face comes from both boxes' faces
        Assert.Equal(2, aTool.GetOrigins(aShared).Count);
        // History: B's face at x = 10 is modified into the shared face (A's is kept)
        TopoDS_Shape aFaceB = FaceAt(aB, 10.0), aFaceA = FaceAt(aA, 10.0);
        bool aModifiedB = aTool.GetModified(aFaceB).Any(s => s.IsSame(aShared));
        bool aKeptA = aFaceA.IsSame(aShared) || aTool.GetModified(aFaceA).Any(s => s.IsSame(aShared));
        Assert.True(aModifiedB || aKeptA);
        Assert.NotNull(aTool.History());
    }

    [Fact]
    public void One_argument_is_its_own_result()
    {
        TopoDS_Shape aA = BoxA();
        BOPAlgo_MakeConnected aTool = Connected(aA);
        Assert.False(aTool.HasErrors());
        Assert.True(aTool.Shape().IsSame(aA));
        Assert.Equal(1, aTool.MaterialsOnPositiveSide(FaceAt(aA, 0.0)).Count + aTool.MaterialsOnNegativeSide(FaceAt(aA, 0.0)).Count);
    }

    [Fact]
    public void Invalid_arguments_are_errors()
    {
        BOPAlgo_MakeConnected aTool = new();
        aTool.Perform();
        Assert.True(aTool.HasError(typeof(BOPAlgo_AlertTooFewArguments)));

        TopoDS_Shape aFace = new BRepBuilderAPI_MakeFace(new Plane3D(), 0, 1, 0, 1).Shape();
        aTool = Connected(BoxA(), aFace);
        Assert.True(aTool.HasError(typeof(BOPAlgo_AlertMultiDimensionalArguments)));

        TopoDS_Compound anEmpty = new();
        new BRep_Builder().MakeCompound(ref anEmpty);
        aTool = Connected(anEmpty);
        Assert.True(aTool.HasError(typeof(BOPAlgo_AlertTooFewArguments)));

        // Repetition before periodicity: a warning
        aTool = Connected(BoxA(), BoxB());
        aTool.RepeatShape(0, 1);
        Assert.True(aTool.HasWarning(typeof(BOPAlgo_AlertShapeIsNotPeriodic)));
    }

    [Fact]
    public void The_connected_shape_is_made_periodic_and_repeated()
    {
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        string aTextA = BRepTextComparison.Write(aA);
        BOPAlgo_MakeConnected aTool = Connected(aA, aB);
        var aParams = new BOPAlgo_MakePeriodic.PeriodicityParams();
        aParams.myPeriodic[0] = true;
        aParams.myPeriod[0] = 20.0;
        aParams.myPeriodic[1] = true;
        aParams.myPeriod[1] = 10.0;
        aTool.MakePeriodic(aParams);
        Assert.False(aTool.HasWarnings());
        Assert.Equal(2, Count(aTool.PeriodicShape(), TopAbs_SOLID));
        // the faces at x = 0 and x = 20 are twins
        TopoDS_Shape aP = aTool.PeriodicShape();
        Assert.True(aTool.PeriodicityTool().GetTwins(FaceAt(aP, 0.0)).Single().IsSame(FaceAt(aP, 20.0)));
        aTool.RepeatShape(0, 1);
        aTool.RepeatShape(1, -2);
        Assert.False(aTool.HasWarnings());
        TopoDS_Shape aR = aTool.PeriodicShape();
        Assert.Equal(12, Count(aR, TopAbs_SOLID));
        Assert.Equal(12000.0, BOPAlgo_MakePeriodicTests.Volume(aR), 1e-6);
        // the faces the copies share at x = 20 have a copy of A on one side and of B on the other, both from the
        // original arguments
        TopoDS_Shape aShared20 = FaceAt(aR, 20.0);
        List<TopoDS_Shape> aSides = [.. aTool.MaterialsOnPositiveSide(aShared20), .. aTool.MaterialsOnNegativeSide(aShared20)];
        Assert.Equal(2, aSides.Count);
        // the A solid generates its copies
        TopoDS_Shape aSolidA = new TopExp_Explorer(aA, TopAbs_SOLID).Current();
        Assert.True(aTool.History()!.Generated(aSolidA).Count >= 5);

        aTool.ClearRepetitions();
        Assert.Equal(2, Count(aTool.PeriodicShape(), TopAbs_SOLID));
        Assert.True(aTool.PeriodicShape().TShape()!.IsFrozen);
        Assert.Equal(aTextA, BRepTextComparison.Write(aA));
    }

    [Fact]
    public void The_caller_s_faces_answer_through_the_copies_the_algorithm_wrote_into()
    {
        // A period longer than the boxes: nothing is split, so natively every face of the result is an argument's own
        // face; the port works on copies of them (the arguments are frozen) and answers for the arguments' faces
        TopoDS_Shape aA = BoxA(), aB = BoxB();
        string aTextA = BRepTextComparison.Write(aA);
        BOPAlgo_MakeConnected aTool = Connected(aA, aB);
        var aParams = new BOPAlgo_MakePeriodic.PeriodicityParams();
        aParams.myPeriodic[0] = true;
        aParams.myPeriod[0] = 30.0;
        aTool.MakePeriodic(aParams);
        Assert.False(aTool.HasWarnings());
        TopoDS_Shape anOuter = FaceAt(aA, 0.0);
        Assert.Equal(1, aTool.MaterialsOnPositiveSide(anOuter).Count + aTool.MaterialsOnNegativeSide(anOuter).Count);
        aTool.RepeatShape(0, 1);
        Assert.Equal(4, Count(aTool.PeriodicShape(), TopAbs_SOLID));
        Assert.Equal(1, aTool.MaterialsOnPositiveSide(anOuter).Count + aTool.MaterialsOnNegativeSide(anOuter).Count);
        Assert.Equal(aTextA, BRepTextComparison.Write(aA));
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        TopoDS_Shape aA = BoxA();
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(10, 5, 0), new Direction3D(0, 0, 1)), 3.0, 10.0).Shape();
        DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            BOPAlgo_MakeConnected aTool = new();
            aTool.AddArgument(aA);
            aTool.AddArgument(aCyl);
            aTool.SetRunParallel(true);
            aTool.Perform();
            var aParams = new BOPAlgo_MakePeriodic.PeriodicityParams();
            aParams.myPeriodic[1] = true;
            aParams.myPeriod[1] = 10.0;
            aTool.MakePeriodic(aParams);
            aTool.RepeatShape(1, 1);
            return BRepTextComparison.Write(aTool.PeriodicShape())
                   + DeterminismHarness.HistoryText([aA, aCyl], aTool.PeriodicShape(), aTool.History());
        }, 2);
    }
}
