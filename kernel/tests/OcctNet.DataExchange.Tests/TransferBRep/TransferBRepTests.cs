// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using OcctNet.DataExchange.Tests.Interface;
using OcctNet.DataExchange.Tests.Transfer;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static OcctNet.DataExchange.Tests.Interface.ToyNorm;

namespace OcctNet.DataExchange.Tests.TransferBRep;

/// <summary>TransferBRep's binders, mappers, services and reader on real shapes (a box's faces).</summary>
[Collection(ProcessWideMessages.Name)]
public class TransferBRepTests
{
    private readonly Shape myBox = new BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape();
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
        var aBinder = new TransferBRep_ShapeBinder();
        Assert.Equal(ShapeType.Shape, aBinder.ShapeType());
        Assert.Equal("TopoDS_Shape", aBinder.ResultTypeName());
        Assert.Equal(typeof(TShape), aBinder.ResultType());
        aBinder.SetResult(myFaces[0]);
        Assert.Equal(ShapeType.Face, aBinder.ShapeType());
        Assert.Equal("TopoDS_Face", aBinder.ResultTypeName());
        Assert.True(aBinder.Face().IsSame(myFaces[0]));
        Assert.Throws<TypeMismatchException>(() => aBinder.Edge());
        aBinder.SetAlreadyUsed();
        Assert.Throws<Transfer_TransferFailure>(() => aBinder.SetResult(myBox));

        var aSolid = new TransferBRep_ShapeBinder(myBox);
        Assert.True(aSolid.HasResult());
        Assert.Equal(ShapeType.Solid, aSolid.ShapeType());
        Assert.True(aSolid.Solid().IsSame(myBox));

        var anOfShape = new TransferBRep_BinderOfShape();
        Assert.False(anOfShape.HasResult());
        anOfShape.CResult() = myFaces[1];
        Assert.True(anOfShape.HasResult());
        Assert.True(anOfShape.Result().IsSame(myFaces[1]));

        var aList = new TransferBRep_ShapeListBinder();
        aList.AddResult(myFaces[0]);
        Assert.False(aList.IsMultiple());
        aList.AddResult(myBox);
        aList.SetResult(1, myFaces[2]);
        Assert.True(aList.IsMultiple());
        Assert.Equal(2, aList.NbShapes());
        Assert.Equal(ShapeType.Solid, aList.ShapeType(2));
        Assert.True(aList.Face(1).IsSame(myFaces[2]));
        Assert.True(aList.Solid(2).IsSame(myBox));
        Assert.Equal("list(TopoDS_Shape)", aList.ResultTypeName());
        Assert.Equal(typeof(TransferBRep_ShapeListBinder), aList.ResultType());
    }

    [Fact]
    public void Shape_mappers_key_a_finder_process()
    {
        Shape aFace = myFaces[0];
        var m1 = new TransferBRep_ShapeMapper(aFace);
        var m2 = new TransferBRep_ShapeMapper(aFace.Reversed());
        var m3 = new TransferBRep_ShapeMapper(myFaces[1]);
        Assert.True(m1.Equates(m2)); // the same shape, whatever the orientation
        Assert.False(m1.Equates(m3));
        Assert.False(m1.Equates(new Transfer_TransientMapper(null)));
        Assert.Equal("TopoDS_Face", m1.ValueTypeName());
        Assert.Equal(typeof(TShape), m1.ValueType());

        var FP = new Transfer_FinderProcess();
        Assert.Equal(External, Occt.TransferBRep.ShapeState(FP, aFace));
        Assert.Equal(External, Occt.TransferBRep.ShapeState(null, aFace));
        Assert.Null(Occt.TransferBRep.ResultFromShape(FP, aFace));
        var aRes = new TCollection_HAsciiString("face");
        Occt.TransferBRep.SetTransientFromShape(FP, aFace, aRes);
        Assert.Same(aRes, Occt.TransferBRep.TransientFromShape(FP, aFace.Reversed()));
        Assert.Equal(Forward, Occt.TransferBRep.ShapeState(FP, aFace));
        Assert.Equal(Reversed, Occt.TransferBRep.ShapeState(FP, aFace.Reversed()));
        Assert.NotNull(Occt.TransferBRep.ResultFromShape(FP, aFace));
        TransferBRep_ShapeMapper aMapped = Occt.TransferBRep.ShapeMapper(FP, aFace.Reversed());
        Assert.Same(FP.Mapped(1), aMapped);
        Assert.NotSame(FP.Mapped(1), Occt.TransferBRep.ShapeMapper(FP, myFaces[1]));
        Assert.Null(Occt.TransferBRep.TransientFromShape(FP, new Shape()));
        Assert.Equal(" Type:TopoDS_Face", Trace(FP, aMapped));
    }

    private static string Trace(Transfer_FinderProcess theFP, Transfer_Finder theFinder)
    {
        var aText = new StringWriter();
        theFP.PrintTrace(theFinder, aText);
        return aText.ToString();
    }

    [Fact]
    public void Shapes_of_a_transient_process()
    {
        ToyEntity A = new("A"), B = new("B"), C = new("C"), D = new("D");
        var TP = new Transfer_TransientProcess();
        TP.SetRootManagement(true);
        Occt.TransferBRep.SetShapeResult(TP, A, myFaces[0]);
        Occt.TransferBRep.SetShapeResult(TP, B, new Shape()); // a null shape: nothing bound
        Assert.Null(TP.Find(B));
        var aList = new TransferBRep_ShapeListBinder();
        aList.AddResult(myFaces[1]);
        aList.AddResult(myFaces[2]);
        TP.Bind(B, aList);
        TP.BindTransient(C, new TopoDS_HShape(myBox));
        var aChain = new TransferBRep_ShapeBinder(myFaces[3]);
        aChain.AddResult(new TransferBRep_ShapeBinder(myFaces[4]));
        TP.Bind(D, aChain);
        TP.SetRoot(A);
        TP.SetRoot(D);

        Assert.True(Occt.TransferBRep.ShapeResult(TP, A).IsSame(myFaces[0]));
        Assert.True(Occt.TransferBRep.ShapeResult(TP, C).IsSame(myBox)); // an HShape result
        Assert.True(Occt.TransferBRep.ShapeResult(TP, B).IsNull); // a list: no single shape
        Assert.True(Occt.TransferBRep.ShapeResult(TP, new TopoDS_HShape(myFaces[5])).IsSame(myFaces[5]));
        Assert.True(Occt.TransferBRep.ShapeResult(TP, aChain).IsSame(myFaces[3])); // a binder as entity
        Assert.True(Occt.TransferBRep.ShapeResult(TP, new ToyEntity("Z")).IsNull);

        Assert.Equal(3, Occt.TransferBRep.Shapes(TP)!.Length()); // the roots: A, and D with its next result
        Assert.Equal(6, Occt.TransferBRep.Shapes(TP, false)!.Length());
        var anEnts = new NCollection_HSequence<Transient>();
        anEnts.Append(B);
        anEnts.Append(C);
        Assert.Equal(3, Occt.TransferBRep.Shapes(TP, anEnts)!.Length());
        Assert.Null(Occt.TransferBRep.Shapes(null));
    }

    [Fact]
    public void Transfer_result_info()
    {
        ToyEntity A = new("A"), B = new("B");
        var TP = new Transfer_TransientProcess();
        Occt.TransferBRep.SetShapeResult(TP, A, myFaces[0]);
        TP.AddWarning(A, "w");
        TP.AddFail(B, "f");
        TP.BindTransient(new TCollection_HAsciiString("x"), new TCollection_HAsciiString("y")); // not of the type
        var aTypes = new NCollection_HSequence<Transient>();
        aTypes.Append(new ToyEntity("type"));
        Occt.TransferBRep.TransferResultInfo(TP, aTypes, out NCollection_HSequence<TransferBRep_TransferResultInfo> anInfos);
        TransferBRep_TransferResultInfo anInfo = anInfos.Value(1);
        Assert.Equal(1, anInfo.ResultWarning());
        Assert.Equal(1, anInfo.NoResultFail());
        Assert.Equal(0, anInfo.Result());
        Occt.TransferBRep.TransferResultInfo(TP, null, out anInfos);
        Assert.Equal(0, anInfos.Length());

        var FP = new Transfer_FinderProcess();
        Occt.TransferBRep.SetTransientFromShape(FP, myFaces[0], A);
        Occt.TransferBRep.SetTransientFromShape(FP, myBox, B);
        var aShapeTypes = new NCollection_HSequence<int>();
        aShapeTypes.Append((int)ShapeType.Face);
        aShapeTypes.Append((int)ShapeType.Shape);
        Occt.TransferBRep.TransferResultInfo(FP, aShapeTypes, out anInfos);
        Assert.Equal(1, anInfos.Value(1).Result());
        Assert.Equal(2, anInfos.Value(2).Result());
        anInfos.Value(2).Clear();
        Assert.Equal(0, anInfos.Value(2).Result());

        // the printed report (XSTEP.us's texts)
        Interface_StaticSet.LoadMessages();
        var aPrinter = new StringPrinter();
        Occt.TransferBRep.PrintResultInfo(aPrinter, new Msg("Header"), anInfos.Value(1), false);
        string aText = aPrinter.Text.ToString();
        Assert.Contains("Result.Print.MSG38", aText, StringComparison.Ordinal); // OCCT 8 has no texts for these keys
        Assert.Equal(4, aText.Split('\n', StringSplitOptions.RemoveEmptyEntries).Length); // header, result, two totals
        var aPrinter2 = new StringPrinter();
        Occt.TransferBRep.PrintResultInfo(aPrinter2, new Msg("Header"), anInfos.Value(1));
        Assert.Equal(11, aPrinter2.Text.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries).Length);
    }

    [Fact]
    public void Checks_bound_to_shapes()
    {
        var FP = new Transfer_FinderProcess();
        var aModel = Model();
        ToyEntity anEnt = new("E");
        aModel.AddEntity(anEnt);
        var aMapper = new TransferBRep_ShapeMapper(myFaces[0]);
        FP.BindTransient(aMapper, anEnt);

        var aChecks = new Interface_CheckIterator();
        var c1 = new Interface_Check(aMapper);
        c1.AddFail("bad face");
        var c2 = new Interface_Check(new TopoDS_HShape(myFaces[1]));
        c2.AddWarning("odd face");
        var c3 = new Interface_Check(anEnt);
        c3.AddWarning("entity");
        var c4 = new Interface_Check(anEnt); // empty: ignored
        aChecks.Add(c1, 1);
        aChecks.Add(c2, 2);
        aChecks.Add(c3, 3);
        aChecks.Add(c4, 4);

        Assert.Equal(1, Occt.TransferBRep.Checked(aChecks).Length()); // the entity only
        Assert.Equal(3, Occt.TransferBRep.Checked(aChecks, true).Length());
        NCollection_HSequence<Shape> aShapes = Occt.TransferBRep.CheckedShapes(aChecks);
        Assert.Equal(2, aShapes.Length());
        Assert.True(aShapes.Value(1).IsSame(myFaces[0]));

        // the check of a shape, through any of its holders
        Interface_CheckIterator anObj = Occt.TransferBRep.CheckObject(aChecks, new TransferBRep_ShapeBinder(myFaces[1]));
        Assert.False(anObj.IsEmpty(false));
        Assert.True(Occt.TransferBRep.CheckObject(aChecks, anEnt).Complies(Interface_CheckStatus.Interface_CheckWarning));
        Assert.True(Occt.TransferBRep.CheckObject(aChecks, new ToyEntity("X")).IsEmpty(false));

        // the check-list in terms of the model's entities
        Interface_CheckIterator aResult = Occt.TransferBRep.ResultCheckList(aChecks, FP, aModel);
        aResult.Start();
        Assert.Same(anEnt, aResult.Value()!.Entity());
        Assert.Equal(1, aResult.Number());
        Assert.True(Occt.TransferBRep.ResultCheckList(aChecks, null, aModel).IsEmpty(false));
    }

    /// <summary>The toy norm's shape actor: a ToyEntity becomes a box of size its name's length (a frozen-free working shape).</summary>
    private sealed class ShapeActor : Transfer_ActorOfTransientProcess
    {
        public override bool Recognize(Transient? start) => start is ToyEntity;

        public override Transfer_Binder? Transfer(Transient? start, Transfer_TransientProcess? TP,
                                                  ProgressRange theProgress = default)
        {
            var anEnt = (ToyEntity)start!;
            if (anEnt.Fails)
            {
                TP!.AddFail(anEnt, "no shape for " + anEnt.Name);
                return null;
            }
            return new TransferBRep_ShapeBinder(new BRepPrimAPI_MakeBox(1.0, 1.0, anEnt.Name.Length).Shape());
        }
    }

    [Fact]
    public void The_reader_transfers_roots_one_or_a_list()
    {
        ToyEntity A = new("A"), B = new("BB"), C = new("CCC");
        A.Ref(B);
        ToyModel aModel = Model(A, B, C);
        var aReader = new TransferBRep_Reader();
        Assert.False(aReader.BeginTransfer()); // no model
        aReader.SetFileStatus(-1);
        Assert.True(aReader.FileNotFound());
        aReader.SetFileStatus(1);
        Assert.True(aReader.SyntaxError());
        aReader.SetModel(aModel);
        aReader.SetProtocol(Protocol);
        aReader.SetActor(new ShapeActor());
        Assert.True(aReader.CheckStatusModel(false));
        Assert.True(aReader.CheckListModel().IsEmpty(false));

        aReader.TransferRoots();
        Assert.True(aReader.IsDone());
        Assert.Equal(2, aReader.NbShapes()); // A and C
        Assert.True(aReader.Shape(1).TShape()!.IsFrozen);
        Shape aOne = aReader.OneShape();
        Assert.Equal(ShapeType.Compound, aOne.ShapeType());
        Assert.True(aOne.TShape()!.IsFrozen);
        Assert.Equal(0, aReader.NbTransients());
        Assert.True(aReader.ShapeResult(C).IsSame(aReader.Shape(2)));
        Assert.True(aReader.CheckStatusResult(false));

        // OCCT: Transfer and TransferList don't clear the former shapes (TransferRoots does)
        Assert.True(aReader.Transfer(2));
        Assert.Equal(3, aReader.NbShapes());
        aReader.Clear();
        Assert.True(aReader.Transfer(2));
        Assert.Equal(ShapeType.Solid, aReader.OneShape().ShapeType());
        Assert.False(aReader.Transfer(4));
        aReader.ModeNewTransfer() = true;
        Transfer_TransientProcess? aBefore = aReader.TransientProcess();
        var aList = new NCollection_HSequence<Transient>();
        aList.Append(C);
        aList.Append(new ToyEntity("not in the model"));
        aList.Append(A);
        aReader.TransferList(aList);
        Assert.NotSame(aBefore, aReader.TransientProcess());
        Assert.Equal(3, aReader.NbShapes());

        C.Fails = true;
        aReader.TransferRoots();
        Assert.Equal(1, aReader.NbShapes());
        Assert.False(aReader.CheckStatusResult(true));
        Assert.False(aReader.CheckListResult().IsEmpty(true));
        aReader.Clear();
        Assert.Equal(0, aReader.NbShapes());
        Assert.True(aReader.OneShape().IsNull);
    }
}
