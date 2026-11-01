// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeProcess: the context's parameters and scopes, the cached resource files, the performer, the operators on
/// frozen inputs (decision 1022), the history and the messages, and ShapeProcessAPI_ApplySequence.
/// </summary>
public class ShapeProcessTests
{
    private static string Metadata(string theKey) =>
        typeof(ShapeProcessTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static string HealData => Path.Combine(Metadata("OcctSourceDir"), "tests", "heal", "data");

    private static TopoDS_Shape FrozenBox()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        Assert.True(aBox.TShape()!.IsFrozen());
        return aBox;
    }

    private static TopoDS_Shape FrozenCylinder()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5, 10).Shape();
        Assert.True(aCyl.TShape()!.IsFrozen());
        return aCyl;
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private sealed class Capture : Message_Printer
    {
        public readonly System.Text.StringBuilder Text = new();

        public Capture() => myTraceLevel = Message_Gravity.Message_Trace;

        protected override void send(string theString, Message_Gravity theGravity) => Text.Append(theString).Append('\n');
    }

    private string WriteResource(string theName, string theText)
    {
        string aDir = Path.Combine(Path.GetTempPath(), "occtnet_shapeprocess_" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(aDir);
        File.WriteAllText(Path.Combine(aDir, theName), theText);
        return aDir;
    }

    [Fact]
    public void Context_reads_parameters_in_its_scope_with_references()
    {
        string aDir = WriteResource("Params",
                                    "Seq.Op.Real : 2.5\nSeq.Op.Int : 7\nSeq.Op.Ref : &Global.Tol\nGlobal.Tol : 0.25\n"
                                    + "Seq.Op.Bool : 1\nSeq.Op.Cont : c2\nSeq.Op.Bad : x\nSeq.Op.Empty :\nSeq.Op.Dangling : &Nowhere\n");
        var aContext = new ShapeProcess_Context("Params", "Seq", aDir);
        aContext.SetScope("Op");

        Assert.Equal(2.5, aContext.RealVal("Real", 0));
        Assert.Equal(7, aContext.IntegerVal("Int", 0));
        Assert.Equal(0.25, aContext.RealVal("Ref", 0));
        Assert.Equal(-1, aContext.IntegerVal("Ref", -1)); // 0.25 isn't an integer
        Assert.True(aContext.BooleanVal("Bool", false));
        Assert.Equal(3.0, aContext.RealVal("Bad", 3.0));
        Assert.Equal(3.0, aContext.RealVal("Dangling", 3.0));
        Assert.Equal(3.0, aContext.RealVal("Missing", 3.0));
        Assert.True(aContext.IsParamSet("Real"));
        Assert.False(aContext.IsParamSet("Global.Tol"));
        Assert.Equal("x", aContext.StringVal("Bad", null));
        Assert.Equal("def", aContext.StringVal("Missing", "def"));
        // OCCT reads the first character of an empty value: Standard_OutOfRange, which the performer catches
        Assert.ThrowsAny<Standard_OutOfRange>(() => aContext.RealVal("Empty", 0));

        aContext.UnSetScope();
        Assert.Equal(-1.0, aContext.RealVal("Real", -1.0));
        aContext.UnSetScope();
        Assert.Equal(0.25, aContext.RealVal("Global.Tol", 0));

        var aShapeContext = new ShapeProcess_ShapeContext("Params", "Seq.Op", aDir);
        Assert.Equal(GeomAbs_Shape.GeomAbs_C2, aShapeContext.ContinuityVal("Cont", GeomAbs_Shape.GeomAbs_C0));
        Assert.Equal(GeomAbs_Shape.GeomAbs_C0, aShapeContext.ContinuityVal("Bad", GeomAbs_Shape.GeomAbs_C0));

        // an empty context has no resources
        var anEmpty = new ShapeProcess_Context();
        Assert.Null(anEmpty.ResourceManager());
        Assert.Equal(1.5, anEmpty.RealVal("Real", 1.5));
        Assert.Equal(1, anEmpty.TraceLevel());
    }

    [Fact]
    public void Resource_files_are_cached_and_reloaded_when_they_change()
    {
        string aDir = WriteResource("Cached", "a : 1\n");
        var aContext = new ShapeProcess_Context();
        Resource_Manager aFirst = aContext.LoadResourceManager("Cached", aDir);
        Assert.Equal(1, aFirst.Integer("a"));

        // each context gets its own copy
        aFirst.SetResource("a", 5);
        Assert.Equal(1, aContext.LoadResourceManager("Cached", aDir).Integer("a"));

        File.WriteAllText(Path.Combine(aDir, "Cached"), "a : 2\n");
        File.SetLastWriteTimeUtc(Path.Combine(aDir, "Cached"), DateTime.UtcNow.AddMinutes(1));
        Assert.Equal(2, aContext.LoadResourceManager("Cached", aDir).Integer("a"));

        // the compiled-in defaults without directories
        Assert.Equal("DirectFaces", aContext.LoadResourceManager("IGES").Value("ToIGES.exec.op"));
        Assert.False(aContext.LoadResourceManager("Cached").Find("a"));
    }

    [Fact]
    public void Operation_flags_and_names()
    {
        Assert.Equal((ShapeProcess.Operation.FixShape, true), ShapeProcess.ToOperationFlag("FixShape"));
        Assert.Equal((ShapeProcess.Operation.DirectFaces, true), ShapeProcess.ToOperationFlag("DirectFaces"));
        Assert.Equal((ShapeProcess.Operation.First, false), ShapeProcess.ToOperationFlag("NoSuchOperation"));
        Assert.Equal((ShapeProcess.Operation.First, false), ShapeProcess.ToOperationFlag(null));
        var aFlags = new ShapeProcess.OperationsFlags(ShapeProcess.Operation.SameParameter).With(ShapeProcess.Operation.FixShape);
        Assert.True(aFlags.Test(ShapeProcess.Operation.FixShape));
        Assert.False(aFlags.Test(ShapeProcess.Operation.DirectFaces));
        Assert.Equal(2, aFlags.Count());
        Assert.False(ShapeProcess.Perform(null, aFlags));
        Assert.False(ShapeProcess.Perform(new ShapeProcess_ShapeContext(FrozenBox(), ""), new ShapeProcess.OperationsFlags()));
    }

    [Fact]
    public void The_library_registers_every_operator()
    {
        ShapeProcess_OperLibrary.Init();
        ShapeProcess_OperLibrary.Init();
        foreach (string aName in new[] { "DirectFaces", "SameParameter", "SetTolerance", "SplitAngle", "BSplineRestriction",
                                         "ElementaryToRevolution", "SweptToElementary", "SurfaceToBSpline", "ToBezier",
                                         "SplitContinuity", "SplitClosedFaces", "FixWireGaps", "FixFaceSize", "DropSmallSolids",
                                         "DropSmallEdges", "FixShape", "SplitClosedEdges", "SplitCommonVertex" })
        {
            Assert.True(ShapeProcess.FindOperator(aName, out ShapeProcess_Operator? anOperator), aName);
            Assert.IsType<ShapeProcess_UOperator>(anOperator);
        }
        Assert.False(ShapeProcess.FindOperator("NoSuchOperator", out _));
        // the names are taken
        Assert.False(ShapeProcess.RegisterOperator("FixShape", new ShapeProcess_UOperator((_, _) => true)));
    }

    [Fact]
    public void A_sequence_runs_on_a_frozen_shape_and_leaves_it_unchanged()
    {
        ShapeProcess_OperLibrary.Init();
        // a cylinder: SplitAngle splits its lateral face, SetTolerance and SameParameter write tolerances in place
        string aDir = WriteResource("Seq", "Run.exec.op : SetTolerance, SameParameter, SplitAngle; NoSuchOperator\n"
                                           + "Run.SplitAngle.Angle : 1.6\nRun.SetTolerance.Mode : 1\nRun.SetTolerance.Value : 1e-3\n");
        TopoDS_Shape aCyl = FrozenCylinder();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        var aPrinter = new Capture();
        var aContext = new ShapeProcess_ShapeContext(aCyl, "Seq", "", aDir);
        aContext.SetMessenger(new Message_Messenger(aPrinter));
        Assert.True(ShapeProcess.Perform(aContext, "Run"));
        aSnap.AssertUnchanged(aCyl);

        Assert.Contains("Error: Shape Processing: Operator NoSuchOperator is not found", aPrinter.Text.ToString(), StringComparison.Ordinal);
        TopoDS_Shape aResult = aContext.Result();
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.Equal(6, Count(aResult, TopAbs_FACE)); // the lateral face in 4
        Assert.True(Math.Abs(BRep_Tool.Tolerance(TopoDS.Vertex(new TopExp_Explorer(aResult, TopAbs_VERTEX).Current())) - 1e-3) < 1e-9);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());

        // the history is keyed by the caller's sub-shapes
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aMap = aContext.Map();
        Assert.True(aMap.IsBound(aCyl));
        int aSplit = 0;
        for (var anExp = new TopExp_Explorer(aCyl, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            TopoDS_Shape aFace = anExp.Current().Located(new TopLoc_Location());
            if (aMap.Find(aFace, out TopoDS_Shape aNew))
            {
                Assert.True(aNew.TShape()!.IsFrozen());
                aSplit += aNew.ShapeType() == TopAbs_COMPOUND || aNew.ShapeType() == TopAbs_SHELL ? 1 : 0;
            }
        }
        Assert.Equal(1, aSplit);

        // the trace level 2 lists the sequence and the operators
        var aTraced = new ShapeProcess_ShapeContext(aCyl, "Seq", "", aDir);
        var aTrace = new Capture();
        aTraced.SetMessenger(new Message_Messenger(aTrace));
        aTraced.SetTraceLevel(2);
        ShapeProcess.Perform(aTraced, "Run");
        Assert.Contains("Info: Shape Processing: Sequence of operators: SetTolerance,SameParameter,SplitAngle,NoSuchOperator",
                        aTrace.Text.ToString(), StringComparison.Ordinal);
        Assert.Contains("Info: Shape Processing: Operator 3/4: SplitAngle", aTrace.Text.ToString(), StringComparison.Ordinal);
        aSnap.AssertUnchanged(aCyl);
    }

    [Fact]
    public void A_shape_needing_nothing_comes_back_itself()
    {
        ShapeProcess_OperLibrary.Init();
        TopoDS_Shape aBox = FrozenBox();
        foreach (ShapeProcess.Operation anOperation in new[] { ShapeProcess.Operation.DirectFaces, ShapeProcess.Operation.SplitAngle,
                                                               ShapeProcess.Operation.SplitClosedFaces, ShapeProcess.Operation.FixShape,
                                                               ShapeProcess.Operation.ElementaryToRevolution })
        {
            var aContext = new ShapeProcess_ShapeContext(aBox, "");
            Assert.True(ShapeProcess.Perform(aContext, new ShapeProcess.OperationsFlags(anOperation)));
            Assert.Equal(aBox, aContext.Result());
            Assert.True(ReferenceEquals(aBox.TShape(), aContext.Result().TShape()), anOperation.ToString());
        }
    }

    [Fact]
    public void Each_operator_alone_leaves_frozen_inputs_unchanged()
    {
        ShapeProcess_OperLibrary.Init();
        TopoDS_Shape[] aShapes = [FrozenBox(), FrozenCylinder(), new BRepPrimAPI_MakeSphere(4).Shape(), new BRepPrimAPI_MakeTorus(10, 2).Shape()];
        foreach (TopoDS_Shape aShape in aShapes)
        {
            ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);
            for (ShapeProcess.Operation anOperation = ShapeProcess.Operation.First; anOperation <= ShapeProcess.Operation.Last; anOperation++)
            {
                var aContext = new ShapeProcess_ShapeContext(aShape, "");
                ShapeProcess.Perform(aContext, new ShapeProcess.OperationsFlags(anOperation));
                // ToBezier with its default modes (nothing converted) gives a null result, as natively
                Assert.True(aContext.Result().IsNull() || aContext.Result().TShape()!.IsFrozen());
                aSnap.AssertUnchanged(aShape);
            }
            // an operator run on its own works on a copy too
            Assert.True(ShapeProcess.FindOperator("SetTolerance", out ShapeProcess_Operator? aSetTol));
            var aDirect = new ShapeProcess_ShapeContext(aShape, "");
            Assert.True(aSetTol!.Perform(aDirect));
            aSnap.AssertUnchanged(aShape);
        }
    }

    [Fact]
    public void A_missing_sequence_warns_and_does_nothing()
    {
        ShapeProcess_OperLibrary.Init();
        var aPrinter = new Capture();
        var aContext = new ShapeProcess_ShapeContext(FrozenBox(), "IGES");
        aContext.SetMessenger(new Message_Messenger(aPrinter));
        Assert.False(ShapeProcess.Perform(aContext, "NoSuchSequence"));
        Assert.Equal("Warning: Shape Processing: Sequence not defined for NoSuchSequence, nothing to do\n", aPrinter.Text.ToString());
        aContext.SetTraceLevel(0);
        aPrinter.Text.Clear();
        Assert.False(ShapeProcess.Perform(aContext, "NoSuchSequence"));
        Assert.Equal("", aPrinter.Text.ToString());
    }

    [Fact]
    public void ApplySequence_runs_the_heal_groups_ToV4_and_prints_statistics()
    {
        TopoDS_Shape aCyl = FrozenCylinder();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        var aSeq = new ShapeProcessAPI_ApplySequence("MDTV", "ToV4", HealData);
        var aPrinter = new Capture();
        aSeq.Context().SetMessenger(new Message_Messenger(aPrinter));
        TopoDS_Shape aResult = aSeq.PrepareShape(aCyl, true, TopAbs_FACE);
        aSnap.AssertUnchanged(aCyl);
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.True(aSeq.Map().Extent() > 0);
        aSeq.PrintPreparationResult();
        string aText = aPrinter.Text.ToString();
        // the message file's texts, their leading blanks removed as OCCT's loader removes them
        Assert.Contains("Mapping:\nShells:\nResult is Shell                 : 1\n", aText, StringComparison.Ordinal);
        Assert.Contains("Preparation ratio:\nShells: 100 per cent\nFaces : 100 per cent\n", aText, StringComparison.Ordinal);

        // a sequence the file doesn't define: the shape itself
        var aNone = new ShapeProcessAPI_ApplySequence("MDTV", "Nothing", HealData);
        Assert.Equal(aCyl, aNone.PrepareShape(aCyl));
        Assert.True(aNone.PrepareShape(new TopoDS_Shape()).IsNull());
    }
}

/// <summary>The tests that register operators in ShapeProcess's process-wide registry run one at a time.</summary>
[CollectionDefinition(nameof(ShapeProcessGlobalStateCollection), DisableParallelization = true)]
public sealed class ShapeProcessGlobalStateCollection;

/// <summary>An operator of the caller's, registered process-wide, run by name and by its exception.</summary>
[Collection(nameof(ShapeProcessGlobalStateCollection))]
public class ShapeProcessRegistryTests
{
    [Fact]
    public void A_registered_operator_runs_and_its_failure_is_reported()
    {
        string aName = "OcctNetTest_" + Guid.NewGuid().ToString("N");
        int aRuns = 0;
        Assert.True(ShapeProcess.RegisterOperator(aName, new ShapeProcess_UOperator((c, _) =>
        {
            aRuns++;
            if (c.IntegerVal("Fail", 0) != 0)
            {
                throw new Standard_ConstructionError("broken");
            }
            return true;
        })));
        Assert.False(ShapeProcess.RegisterOperator(aName, new ShapeProcess_UOperator((_, _) => false)));

        string aDir = Path.Combine(Path.GetTempPath(), "occtnet_shapeprocess_" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(aDir);
        File.WriteAllText(Path.Combine(aDir, "Custom"), $"Ok.exec.op : {aName}\nBad.exec.op : {aName}\nBad.{aName}.Fail : 1\n");

        var aContext = new ShapeProcess_Context("Custom", "", aDir);
        var aPrinter = new System.Text.StringBuilder();
        aContext.SetMessenger(new Message_Messenger(new StringPrinter(aPrinter)));
        Assert.True(ShapeProcess.Perform(aContext, "Ok"));
        Assert.False(ShapeProcess.Perform(aContext, "Bad"));
        Assert.Equal(2, aRuns);
        Assert.Equal($"Error: Shape Processing: Operator {aName} failed with exception broken\n", aPrinter.ToString());
    }

    private sealed class StringPrinter(System.Text.StringBuilder theText) : Message_Printer
    {
        protected override void send(string theString, Message_Gravity theGravity) => theText.Append(theString).Append('\n');
    }
}
