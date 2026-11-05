// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Kernel.DataExchange.Tests.Interface;
using Kerf.Kernel.DataExchange.Tests.Transfer;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Kernel.DataExchange.Tests.Interface.ToyNorm;

namespace Kerf.Kernel.DataExchange.Tests.TransferBRep;

/// <summary>TransferBRep's binders, mappers, services and reader on real shapes (a box's faces).</summary>
[Collection(ProcessWideMessages.Name)]
public class TransferBRepTests
{
    private readonly Shape myBox = new MakeBox(1.0, 2.0, 3.0).Shape();
    private readonly List<Shape> myFaces = [];

    public TransferBRepTests()
    {
        for (var anExp = new Explorer(myBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            myFaces.Add(anExp.Current());
        }
    }

    [Fact]
    public void Shape_binders()
    {
        var aBinder = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeBinder();
        Assert.Equal(ShapeType.Shape, aBinder.ShapeType());
        Assert.Equal("TopoDS_Shape", aBinder.ResultTypeName());
        Assert.Equal(typeof(TShape), aBinder.ResultType());
        aBinder.SetResult(myFaces[0]);
        Assert.Equal(ShapeType.Face, aBinder.ShapeType());
        Assert.Equal("TopoDS_Face", aBinder.ResultTypeName());
        Assert.True(aBinder.Face().IsSame(myFaces[0]));
        EngineAssert.Throws<TypeMismatchException>(() => aBinder.Edge());
        aBinder.SetAlreadyUsed();
        EngineAssert.Throws<global::Kerf.Engine.Exchange.Framework.Transfer.TransferFailureException>(() => aBinder.SetResult(myBox));

        var aSolid = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeBinder(myBox);
        Assert.True(aSolid.HasResult);
        Assert.Equal(ShapeType.Solid, aSolid.ShapeType());
        Assert.True(aSolid.Solid().IsSame(myBox));

        var anOfShape = new global::Kerf.Engine.Exchange.Framework.Transfer.BinderOfShape();
        Assert.False(anOfShape.HasResult);
        anOfShape.CResult() = myFaces[1];
        Assert.True(anOfShape.HasResult);
        Assert.True(anOfShape.Result().IsSame(myFaces[1]));

        var aList = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeListBinder();
        aList.AddResult(myFaces[0]);
        Assert.False(aList.IsMultiple());
        aList.AddResult(myBox);
        aList.SetResult(1, myFaces[2]);
        Assert.True(aList.IsMultiple());
        Assert.Equal(2, aList.ShapeCount);
        Assert.Equal(ShapeType.Solid, aList.ShapeType(2));
        Assert.True(aList.Face(1).IsSame(myFaces[2]));
        Assert.True(aList.Solid(2).IsSame(myBox));
        Assert.Equal("list(TopoDS_Shape)", aList.ResultTypeName());
        Assert.Equal(typeof(global::Kerf.Engine.Exchange.Framework.Transfer.ShapeListBinder), aList.ResultType());
    }

    [Fact]
    public void Shape_mappers_key_a_finder_process()
    {
        Shape aFace = myFaces[0];
        var m1 = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeMapper(aFace);
        var m2 = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeMapper(aFace.Reversed());
        var m3 = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeMapper(myFaces[1]);
        Assert.True(m1.Equates(m2)); // the same shape, whatever the orientation
        Assert.False(m1.Equates(m3));
        Assert.False(m1.Equates(new global::Kerf.Engine.Exchange.Framework.Transfer.TransientMapper(null)));
        Assert.Equal("TopoDS_Face", m1.ValueTypeName());
        Assert.Equal(typeof(TShape), m1.ValueType());

        var FP = new global::Kerf.Engine.Exchange.Framework.Transfer.FinderProcess();
        Assert.Equal(External, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeState(FP, aFace));
        Assert.Equal(External, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeState(null, aFace));
        Assert.Null(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ResultFromShape(FP, aFace));
        var aRes = new TCollection_HAsciiString("face");
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetTransientFromShape(FP, aFace, aRes);
        Assert.Same(aRes, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.TransientFromShape(FP, aFace.Reversed()));
        Assert.Equal(Forward, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeState(FP, aFace));
        Assert.Equal(Reversed, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeState(FP, aFace.Reversed()));
        Assert.NotNull(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ResultFromShape(FP, aFace));
        global::Kerf.Engine.Exchange.Framework.Transfer.ShapeMapper aMapped = Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(FP, aFace.Reversed());
        Assert.Same(FP.Mapped(1), aMapped);
        Assert.NotSame(FP.Mapped(1), Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(FP, myFaces[1]));
        Assert.Null(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.TransientFromShape(FP, new Shape()));
        Assert.Equal(" Type:TopoDS_Face", Trace(FP, aMapped));
    }

    private static string Trace(global::Kerf.Engine.Exchange.Framework.Transfer.FinderProcess theFP, global::Kerf.Engine.Exchange.Framework.Transfer.Finder theFinder)
    {
        var aText = new StringWriter();
        theFP.PrintTrace(theFinder, aText);
        return aText.ToString();
    }

    [Fact]
    public void Shapes_of_a_transient_process()
    {
        ToyEntity A = new("A"), B = new("B"), C = new("C"), D = new("D");
        var TP = new global::Kerf.Engine.Exchange.Framework.Transfer.TransientProcess();
        TP.SetRootManagement(true);
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetShapeResult(TP, A, myFaces[0]);
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetShapeResult(TP, B, new Shape()); // a null shape: nothing bound
        Assert.Null(TP.Find(B));
        var aList = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeListBinder();
        aList.AddResult(myFaces[1]);
        aList.AddResult(myFaces[2]);
        TP.Bind(B, aList);
        TP.BindTransient(C, new TopoDS_HShape(myBox));
        var aChain = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeBinder(myFaces[3]);
        aChain.AddResult(new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeBinder(myFaces[4]));
        TP.Bind(D, aChain);
        TP.SetRoot(A);
        TP.SetRoot(D);

        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(TP, A).IsSame(myFaces[0]));
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(TP, C).IsSame(myBox)); // an HShape result
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(TP, B).IsNull); // a list: no single shape
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(TP, new TopoDS_HShape(myFaces[5])).IsSame(myFaces[5]));
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(TP, aChain).IsSame(myFaces[3])); // a binder as entity
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(TP, new ToyEntity("Z")).IsNull);

        Assert.Equal(3, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Shapes(TP)!.Length()); // the roots: A, and D with its next result
        Assert.Equal(6, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Shapes(TP, false)!.Length());
        var anEnts = new NCollection_HSequence<Transient>();
        anEnts.Append(B);
        anEnts.Append(C);
        Assert.Equal(3, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Shapes(TP, anEnts)!.Length());
        Assert.Null(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Shapes(null));
    }

    [Fact]
    public void Transfer_result_info()
    {
        ToyEntity A = new("A"), B = new("B");
        var TP = new global::Kerf.Engine.Exchange.Framework.Transfer.TransientProcess();
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetShapeResult(TP, A, myFaces[0]);
        TP.AddWarning(A, "w");
        TP.AddFail(B, "f");
        TP.BindTransient(new TCollection_HAsciiString("x"), new TCollection_HAsciiString("y")); // not of the type
        var aTypes = new NCollection_HSequence<Transient>();
        aTypes.Append(new ToyEntity("type"));
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.TransferResultInfo(TP, aTypes, out NCollection_HSequence<global::Kerf.Engine.Exchange.Framework.Transfer.TransferResultInfo> anInfos);
        global::Kerf.Engine.Exchange.Framework.Transfer.TransferResultInfo anInfo = anInfos.Value(1);
        Assert.Equal(1, anInfo.ResultWarning());
        Assert.Equal(1, anInfo.NoResultFail());
        Assert.Equal(0, anInfo.Result());
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.TransferResultInfo(TP, null, out anInfos);
        Assert.Equal(0, anInfos.Length());

        var FP = new global::Kerf.Engine.Exchange.Framework.Transfer.FinderProcess();
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetTransientFromShape(FP, myFaces[0], A);
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetTransientFromShape(FP, myBox, B);
        var aShapeTypes = new NCollection_HSequence<int>();
        aShapeTypes.Append((int)ShapeType.Face);
        aShapeTypes.Append((int)ShapeType.Shape);
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.TransferResultInfo(FP, aShapeTypes, out anInfos);
        Assert.Equal(1, anInfos.Value(1).Result());
        Assert.Equal(2, anInfos.Value(2).Result());
        anInfos.Value(2).Clear();
        Assert.Equal(0, anInfos.Value(2).Result());

        // the printed report (XSTEP.us's texts)
        StaticSet.LoadMessages();
        var aPrinter = new StringPrinter();
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.PrintResultInfo(aPrinter, new Msg("Header"), anInfos.Value(1), false);
        string aText = aPrinter.Text.ToString();
        Assert.Contains("Result.Print.MSG38", aText, StringComparison.Ordinal); // OCCT 8 has no texts for these keys
        Assert.Equal(4, aText.Split('\n', StringSplitOptions.RemoveEmptyEntries).Length); // header, result, two totals
        var aPrinter2 = new StringPrinter();
        Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.PrintResultInfo(aPrinter2, new Msg("Header"), anInfos.Value(1));
        Assert.Equal(11, aPrinter2.Text.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries).Length);
    }

    [Fact]
    public void Checks_bound_to_shapes()
    {
        var FP = new global::Kerf.Engine.Exchange.Framework.Transfer.FinderProcess();
        var aModel = Model();
        ToyEntity anEnt = new("E");
        aModel.AddEntity(anEnt);
        var aMapper = new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeMapper(myFaces[0]);
        FP.BindTransient(aMapper, anEnt);

        var aChecks = new CheckIterator();
        var c1 = new FrameworkCheck(aMapper);
        c1.AddFail("bad face");
        var c2 = new FrameworkCheck(new TopoDS_HShape(myFaces[1]));
        c2.AddWarning("odd face");
        var c3 = new FrameworkCheck(anEnt);
        c3.AddWarning("entity");
        var c4 = new FrameworkCheck(anEnt); // empty: ignored
        aChecks.Add(c1, 1);
        aChecks.Add(c2, 2);
        aChecks.Add(c3, 3);
        aChecks.Add(c4, 4);

        Assert.Equal(1, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Checked(aChecks).Length()); // the entity only
        Assert.Equal(3, Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Checked(aChecks, true).Length());
        NCollection_HSequence<Shape> aShapes = Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.CheckedShapes(aChecks);
        Assert.Equal(2, aShapes.Length());
        Assert.True(aShapes.Value(1).IsSame(myFaces[0]));

        // the check of a shape, through any of its holders
        CheckIterator anObj = Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.CheckObject(aChecks, new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeBinder(myFaces[1]));
        Assert.False(anObj.IsEmpty(false));
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.CheckObject(aChecks, anEnt).Complies(FrameworkCheckStatus.CheckWarning));
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.CheckObject(aChecks, new ToyEntity("X")).IsEmpty(false));

        // the check-list in terms of the model's entities
        CheckIterator aResult = Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ResultCheckList(aChecks, FP, aModel);
        aResult.Start();
        Assert.Same(anEnt, aResult.Value()!.Entity());
        Assert.Equal(1, aResult.Number());
        Assert.True(Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ResultCheckList(aChecks, null, aModel).IsEmpty(false));
    }

    /// <summary>The toy norm's shape actor: a ToyEntity becomes a box of size its name's length (a frozen-free working shape).</summary>
    private sealed class ShapeActor : global::Kerf.Engine.Exchange.Framework.Transfer.ActorOfTransientProcess
    {
        public override bool Recognize(Transient? start) => start is ToyEntity;

        public override global::Kerf.Engine.Exchange.Framework.Transfer.Binder? Transfer(Transient? start, global::Kerf.Engine.Exchange.Framework.Transfer.TransientProcess? TP,
                                                  ProgressRange theProgress = default)
        {
            var anEnt = (ToyEntity)start!;
            if (anEnt.Fails)
            {
                TP!.AddFail(anEnt, "no shape for " + anEnt.Name);
                return null;
            }
            return new global::Kerf.Engine.Exchange.Framework.Transfer.ShapeBinder(new MakeBox(1.0, 1.0, anEnt.Name.Length).Shape());
        }
    }

    [Fact]
    public void The_reader_transfers_roots_one_or_a_list()
    {
        ToyEntity A = new("A"), B = new("BB"), C = new("CCC");
        A.Ref(B);
        ToyModel aModel = Model(A, B, C);
        var aReader = new global::Kerf.Engine.Exchange.Framework.Transfer.BrepTransferReader();
        Assert.False(aReader.BeginTransfer()); // no model
        aReader.SetFileStatus(-1);
        Assert.True(aReader.FileNotFound());
        aReader.SetFileStatus(1);
        Assert.True(aReader.SyntaxError());
        aReader.SetModel(aModel);
        aReader.SetProtocol(ToyNorm.ProtocolInstance);
        aReader.SetActor(new ShapeActor());
        Assert.True(aReader.CheckStatusModel(false));
        Assert.True(aReader.CheckListModel().IsEmpty(false));

        aReader.TransferRoots();
        Assert.True(aReader.IsDone);
        Assert.Equal(2, aReader.ShapeCount); // A and C
        Assert.True(aReader.Shape(1).TShape()!.IsFrozen);
        Shape aOne = aReader.OneShape();
        Assert.Equal(ShapeType.Compound, aOne.Type);
        Assert.True(aOne.TShape()!.IsFrozen);
        Assert.Equal(0, aReader.TransientCount);
        Assert.True(aReader.ShapeResult(C).IsSame(aReader.Shape(2)));
        Assert.True(aReader.CheckStatusResult(false));

        // OCCT: Transfer and TransferList don't clear the former shapes (TransferRoots does)
        Assert.True(aReader.Transfer(2));
        Assert.Equal(3, aReader.ShapeCount);
        aReader.Clear();
        Assert.True(aReader.Transfer(2));
        Assert.Equal(ShapeType.Solid, aReader.OneShape().Type);
        Assert.False(aReader.Transfer(4));
        aReader.ModeNewTransfer() = true;
        global::Kerf.Engine.Exchange.Framework.Transfer.TransientProcess? aBefore = aReader.TransientProcess();
        var aList = new NCollection_HSequence<Transient>();
        aList.Append(C);
        aList.Append(new ToyEntity("not in the model"));
        aList.Append(A);
        aReader.TransferList(aList);
        Assert.NotSame(aBefore, aReader.TransientProcess());
        Assert.Equal(3, aReader.ShapeCount);

        C.Fails = true;
        aReader.TransferRoots();
        Assert.Equal(1, aReader.ShapeCount);
        Assert.False(aReader.CheckStatusResult(true));
        Assert.False(aReader.CheckListResult().IsEmpty(true));
        aReader.Clear();
        Assert.Equal(0, aReader.ShapeCount);
        Assert.True(aReader.OneShape().IsNull);
    }
}
