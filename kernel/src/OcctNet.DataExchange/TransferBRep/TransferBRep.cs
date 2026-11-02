// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.Message_Gravity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// This package gathers services to simply read files and convert them to Shapes from CasCade. IE. it can be used in
/// conjunction with purely CasCade software.
/// </summary>
/// <remarks>
/// The shapes these services get from binders are the transfer's own: the readers publish what they return (decision
/// 12407). <c>BRepCheck</c> (declared by OCCT, its body commented out) is left out.
/// </remarks>
public static class TransferBRep
{
    private static void ShapeAppend(Transfer_Binder? binder, NCollection_HSequence<TopoDS_Shape> shapes)
    {
        if (binder is null)
        {
            return;
        }
        if (binder.IsKind(typeof(TransferBRep_BinderOfShape)))
        {
            TransferBRep_BinderOfShape shbind = binder as TransferBRep_BinderOfShape;
            if (shbind.HasResult())
            {
                shapes.Append(shbind.Result());
            }
        }
        else if (binder.IsKind(typeof(TransferBRep_ShapeListBinder)))
        {
            TransferBRep_ShapeListBinder slbind = binder as TransferBRep_ShapeListBinder;
            int i, nb = slbind.NbShapes();
            for (i = 1; i <= nb; i++)
            {
                shapes.Append(slbind.Shape(i));
            }
        }
        else if (binder.IsKind(typeof(Transfer_SimpleBinderOfTransient)))
        {
            Transfer_SimpleBinderOfTransient trbind = binder as Transfer_SimpleBinderOfTransient;
            TopoDS_HShape? hs = trbind.Result() as TopoDS_HShape;
            if (hs is not null)
            {
                shapes.Append(hs.Shape());
            }
        }
        Transfer_Binder? nextr = binder.NextResult();
        if (nextr is not null)
        {
            ShapeAppend(nextr, shapes);
        }
    }

    /// <summary>Get the Shape recorded in a Binder. If the Binder brings a multiple result, search for the Shape.</summary>
    public static TopoDS_Shape ShapeResult(Transfer_Binder? binder)
    {
        TopoDS_Shape shape = new TopoDS_Shape();
        Transfer_Binder? bnd = binder;
        while (bnd is not null)
        {
            TransferBRep_BinderOfShape? shb = bnd as TransferBRep_BinderOfShape;
            if (shb is not null)
            {
                return shb.Result();
            }
            Transfer_SimpleBinderOfTransient? hsb = bnd as Transfer_SimpleBinderOfTransient;
            if (hsb is not null)
            {
                TopoDS_HShape? hsp = hsb.Result() as TopoDS_HShape;
                if (hsp is not null)
                {
                    return hsp.Shape();
                }
            }
            bnd = bnd.NextResult();
        }
        return shape;
    }

    /// <summary>
    /// Get the Shape recorded in a TransientProcess as result of the Transfer of an entity. I.E. in the binder bound to
    /// that Entity. If no result or result not a single Shape, returns a Null Shape.
    /// </summary>
    public static TopoDS_Shape ShapeResult(Transfer_TransientProcess TP, Standard_Transient? ent)
    {
        TopoDS_Shape shape = new TopoDS_Shape();
        Transfer_Binder? binder = TP.Find(ent);
        if (binder is null)
        {
            binder = ent as Transfer_Binder;
        }
        if (binder is not null)
        {
            return TransferBRep.ShapeResult(binder);
        }
        TopoDS_HShape? hsp = ent as TopoDS_HShape;
        if (hsp is not null)
        {
            return hsp.Shape();
        }
        return shape;
    }

    /// <summary>
    /// Sets a Shape as a result for a starting entity <paramref name="ent"/> (reverse of ShapeResult). It simply
    /// creates a ShapeBinder then binds it to the entity.
    /// </summary>
    public static void SetShapeResult(Transfer_TransientProcess? TP, Standard_Transient? ent, in TopoDS_Shape result)
    {
        if (result.IsNull() || ent is null || TP is null)
        {
            return;
        }
        TP.Bind(ent, new TransferBRep_ShapeBinder(result));
    }

    /// <summary>
    /// Gets the Shapes recorded in a TransientProcess as result of a Transfer, considers roots only or all results
    /// according <paramref name="roots"/>, returns them as a HSequence.
    /// </summary>
    public static NCollection_HSequence<TopoDS_Shape>? Shapes(Transfer_TransientProcess? TP, bool roots = true)
    {
        NCollection_HSequence<TopoDS_Shape>? shapes = null;
        if (TP is null)
        {
            return shapes;
        }
        shapes = new NCollection_HSequence<TopoDS_Shape>();

        Transfer_IteratorOfProcessForTransient list = roots ? TP.RootResult() : TP.CompleteResult();

        for (list.Start(); list.More(); list.Next())
        {
            Transfer_Binder binder = list.Value();
            ShapeAppend(binder, shapes);
        }
        return shapes;
    }

    /// <summary>
    /// Gets the Shapes recorded in a TransientProcess as result of a Transfer, for a given list of starting entities,
    /// returns the shapes as a HSequence.
    /// </summary>
    public static NCollection_HSequence<TopoDS_Shape>? Shapes(Transfer_TransientProcess? TP,
                                                              NCollection_HSequence<Standard_Transient>? list)
    {
        NCollection_HSequence<TopoDS_Shape>? shapes = null;
        if (TP is null && list is null) // OCCT: && where || is meant (either null is dereferenced below)
        {
            return shapes;
        }
        shapes = new NCollection_HSequence<TopoDS_Shape>();

        int ie, ne = list.Length();
        for (ie = 1; ie <= ne; ie++)
        {
            Transfer_Binder? binder = TP.Find(list.Value(ie));
            ShapeAppend(binder, shapes);
        }

        return shapes;
    }

    /// <summary>
    /// Returns a Status regarding a Shape in a FinderProcess: FORWARD means bound with SAME Orientation; REVERSED
    /// means bound with REVERSE Orientation; EXTERNAL means NOT BOUND; INTERNAL is not used.
    /// </summary>
    public static TopAbs_Orientation ShapeState(Transfer_FinderProcess? FP, in TopoDS_Shape shape)
    {
        if (FP is null || shape.IsNull())
        {
            return TopAbs_EXTERNAL;
        }
        TransferBRep_ShapeMapper? sm = new TransferBRep_ShapeMapper(shape);
        int index = FP.MapIndex(sm);
        if (index == 0)
        {
            return TopAbs_EXTERNAL;
        }
        sm = FP.Mapped(index) as TransferBRep_ShapeMapper;
        if (sm is null)
        {
            return TopAbs_EXTERNAL;
        }
        TopoDS_Shape mapped = sm.Value();
        //  equality is assumed, we only test the orientation
        if (mapped.Orientation() != shape.Orientation())
        {
            return TopAbs_REVERSED;
        }
        return TopAbs_FORWARD;
    }

    /// <summary>Returns the result (as a Binder) attached to a given Shape. Null if none.</summary>
    public static Transfer_Binder? ResultFromShape(Transfer_FinderProcess? FP, in TopoDS_Shape shape)
    {
        Transfer_Binder? res = null;
        if (FP is null || shape.IsNull())
        {
            return res;
        }
        var sm = new TransferBRep_ShapeMapper(shape);
        return FP.Find(sm);
    }

    /// <summary>Returns the result as pure Transient attached to a Shape; first one if multiple result.</summary>
    public static Standard_Transient? TransientFromShape(Transfer_FinderProcess? FP, in TopoDS_Shape shape)
    {
        Standard_Transient? res = null;
        if (FP is null || shape.IsNull())
        {
            return res;
        }
        var sm = new TransferBRep_ShapeMapper(shape);
        return FP.FindTransient(sm);
    }

    /// <summary>Binds a Transient Result to a Shape in a FinderProcess (as first result if multiple : does not add it to existing one).</summary>
    public static void SetTransientFromShape(Transfer_FinderProcess? FP, in TopoDS_Shape shape, Standard_Transient? result)
    {
        if (FP is null || shape.IsNull())
        {
            return;
        }
        var sm = new TransferBRep_ShapeMapper(shape);
        FP.BindTransient(sm, result);
    }

    /// <summary>
    /// Returns a ShapeMapper for a given Shape (location included). Either <paramref name="shape"/> is already mapped,
    /// then its Mapper is returned. Or it is not, then a new one is created then returned, BUT it is not mapped here
    /// (use Bind or FindElseBind to do this).
    /// </summary>
    public static TransferBRep_ShapeMapper ShapeMapper(Transfer_FinderProcess FP, in TopoDS_Shape shape)
    {
        var mapper = new TransferBRep_ShapeMapper(shape);
        int index = FP.MapIndex(mapper);
        if (index == 0)
        {
            return mapper;
        }
        return FP.Mapped(index) as TransferBRep_ShapeMapper;
    }

    // Functions to collect transfer result information

    private static void FillInfo(Transfer_Binder Binder, Interface_Check Check, TransferBRep_TransferResultInfo Info)
    {
        int R = 0, RW = 0, RF = 0, RWF = 0, NR = 0, NRW = 0, NRF = 0, NRWF = 0;
        if (Binder.HasResult())
        {
            if (Check.HasWarnings() && Check.HasFailed())
            {
                RWF++;
            }
            else if (Check.HasWarnings())
            {
                RW++;
            }
            else if (Check.HasFailed())
            {
                RF++;
            }
            else
            {
                R++;
            }
        }
        else if (Check.HasWarnings() && Check.HasFailed())
        {
            NRWF++;
        }
        else if (Check.HasWarnings())
        {
            NRW++;
        }
        else if (Check.HasFailed())
        {
            NRF++;
        }
        else
        {
            NR++;
        }
        Info.Result() += R;
        Info.ResultWarning() += RW;
        Info.ResultFail() += RF;
        Info.ResultWarningFail() += RWF;
        Info.NoResult() += NR;
        Info.NoResultWarning() += NRW;
        Info.NoResultFail() += NRF;
        Info.NoResultWarningFail() += NRWF;
    }

    /// <summary>
    /// Fills sequence of TransferResultInfo for each type of entity given in the EntityTypes (entity are given as
    /// objects). Method IsKind applied to the entities in TP is used to compare with entities in EntityTypes.
    /// </summary>
    public static void TransferResultInfo(Transfer_TransientProcess? TP, NCollection_HSequence<Standard_Transient>? EntityTypes,
                                          out NCollection_HSequence<TransferBRep_TransferResultInfo> InfoSeq)
    {
        // create output Sequence in accordance with required ShapeTypes
        InfoSeq = new NCollection_HSequence<TransferBRep_TransferResultInfo>();
        if (TP is null || EntityTypes is null)
        {
            return;
        }
        int SeqLen = EntityTypes.Length();
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= SeqLen; i++)
        {
            InfoSeq.Append(new TransferBRep_TransferResultInfo());
        }

        // fill Sequence
        int NbMapped = TP.NbMapped();
        for (i = 1; i <= NbMapped; i++)
        {
            Standard_Transient? Entity = TP.Mapped(i);

            Transfer_Binder? Binder = TP.Find(Entity);
            if (Binder is null)
            {
                continue;
            }
            Interface_Check Check = Binder.Check();

            // find appropriate element in the Sequence
            for (int index = 1; index <= SeqLen; index++)
            {
                if (Entity.IsKind(EntityTypes.Value(index).DynamicType()))
                {
                    TransferBRep_TransferResultInfo Info = InfoSeq.Value(index);
                    // fill element
                    FillInfo(Binder, Check, Info);
                }
            }
        }
    }

    /// <summary>
    /// Fills sequence of TransferResultInfo for each type of shape given in the ShapeTypes (which are in fact
    /// considered as TopAbs_ShapeEnum). The Finders in the FP are considered as ShapeMappers.
    /// </summary>
    public static void TransferResultInfo(Transfer_FinderProcess? FP, NCollection_HSequence<int>? ShapeTypes,
                                          out NCollection_HSequence<TransferBRep_TransferResultInfo> InfoSeq)
    {
        // create output Sequence in accordance with required ShapeTypes
        InfoSeq = new NCollection_HSequence<TransferBRep_TransferResultInfo>();
        if (FP is null || ShapeTypes is null)
        {
            return;
        }
        int SeqLen = ShapeTypes.Length();
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= SeqLen; i++)
        {
            InfoSeq.Append(new TransferBRep_TransferResultInfo());
        }

        // fill Sequence
        int NbMapped = FP.NbMapped();
        for (i = 1; i <= NbMapped; i++)
        {
            TransferBRep_ShapeMapper? Mapper = FP.Mapped(i) as TransferBRep_ShapeMapper;
            Transfer_Binder? Binder = FP.Find(Mapper);
            if (Binder is null)
            {
                continue;
            }
            Interface_Check Check = Binder.Check();

            TopoDS_Shape S = Mapper.Value();
            TopAbs_ShapeEnum ShapeType = S.ShapeType();

            // find appropriate element in the Sequence
            for (int index = 1; index <= SeqLen; index++)
            {
                // JR/Hp :
                TopAbs_ShapeEnum CurrentType = (TopAbs_ShapeEnum)ShapeTypes.Value(index);
                //      TopAbs_ShapeEnum CurrentType = (TopAbs_ShapeEnum)ShapeTypes->Value (index);
                if (CurrentType == ShapeType || CurrentType == TopAbs_SHAPE)
                {
                    TransferBRep_TransferResultInfo Info = InfoSeq.Value(index);
                    // fill element
                    FillInfo(Binder, Check, Info);
                }
            }
        }
    }

    //  ########  CHECK LOURD  ########
    // BRepCheck (TransferBRep_Analyzer) is commented out in OCCT: left out.

    //  ###  conversion resultat -> starting

    /// <summary>
    /// Takes a starting CheckIterator which brings checks bound with starting objects (Shapes, Transient from an
    /// Imagine appli ...) and converts it to a CheckIterator in which checks are bound with results in an
    /// InterfaceModel. Mapping is recorded in the FinderProcess. Starting objects for which no individual result is
    /// recorded remain in their state.
    /// </summary>
    public static Interface_CheckIterator ResultCheckList(Interface_CheckIterator chl, Transfer_FinderProcess? FP,
                                                          Interface_InterfaceModel? model)
    {
        var nchl = new Interface_CheckIterator();
        if (FP is null || model is null)
        {
            return nchl;
        }
        nchl.SetModel(model);
        for (chl.Start(); chl.More(); chl.Next())
        {
            int num = 0;
            Interface_Check ach = chl.Value();
            if (ach.NbFails() + ach.NbWarnings() == 0)
            {
                continue;
            }
            Transfer_Finder? starting = ach.Entity() as Transfer_Finder;
            Standard_Transient? ent = null;
            if (starting is not null)
            {
                ent = FP.FindTransient(starting);
            }
            if (ent is not null)
            {
                ach.SetEntity(ent);
                num = model.Number(ent);
            }
            nchl.Add(ach, num);
        }
        return nchl;
    }

    /// <summary>
    /// Returns the list of objects to which a non-empty Check is bound in a check-list. Objects are transients, they
    /// can then be either Imagine objects entities for an Interface Norm. <paramref name="alsoshapes"/> commands Shapes
    /// to be returned too (as ShapeMapper), see also CheckedShapes.
    /// </summary>
    public static NCollection_HSequence<Standard_Transient> Checked(Interface_CheckIterator chl, bool alsoshapes = false)
    {
        var ls = new NCollection_HSequence<Standard_Transient>();
        for (chl.Start(); chl.More(); chl.Next())
        {
            Interface_Check ach = chl.Value();
            if (ach.NbFails() + ach.NbWarnings() == 0)
            {
                continue;
            }
            Standard_Transient? ent = ach.Entity();
            if (ent is null)
            {
                continue;
            }
            if (!alsoshapes)
            {
                if (ent.IsKind(typeof(TransferBRep_BinderOfShape)) || ent.IsKind(typeof(TopoDS_HShape))
                    || ent.IsKind(typeof(TransferBRep_ShapeMapper)))
                {
                    continue;
                }
            }
            ls.Append(ent);
        }
        return ls;
    }

    /// <summary>Returns the list of shapes to which a non-empty Check is bound in a check-list.</summary>
    public static NCollection_HSequence<TopoDS_Shape> CheckedShapes(Interface_CheckIterator chl)
    {
        var ls = new NCollection_HSequence<TopoDS_Shape>();
        for (chl.Start(); chl.More(); chl.Next())
        {
            Interface_Check ach = chl.Value();
            if (ach.NbFails() + ach.NbWarnings() == 0)
            {
                continue;
            }
            Standard_Transient? ent = ach.Entity();
            if (ent is null)
            {
                continue;
            }
            TopoDS_HShape? hs = ent as TopoDS_HShape;
            TransferBRep_BinderOfShape? sb = ent as TransferBRep_BinderOfShape;
            TransferBRep_ShapeMapper? sm = ent as TransferBRep_ShapeMapper;
            if (hs is not null)
            {
                ls.Append(hs.Shape());
            }
            if (sb is not null)
            {
                ls.Append(sb.Result());
            }
            if (sm is not null)
            {
                ls.Append(sm.Value());
            }
        }
        return ls;
    }

    /// <summary>
    /// Returns the check-list bound to a given object, generally none (if OK) or one check. <paramref name="obj"/> can
    /// be, either a true Transient object or entity, or a ShapeMapper, in that case the Shape is considered.
    /// </summary>
    public static Interface_CheckIterator CheckObject(Interface_CheckIterator chl, Standard_Transient? obj)
    {
        TopoDS_Shape S = new TopoDS_Shape();
        TopoDS_HShape? hs = obj as TopoDS_HShape;
        TransferBRep_BinderOfShape? sb = obj as TransferBRep_BinderOfShape;
        TransferBRep_ShapeMapper? sm = obj as TransferBRep_ShapeMapper;
        if (hs is not null)
        {
            S = hs.Shape();
        }
        if (sb is not null)
        {
            S = sb.Result();
        }
        if (sm is not null)
        {
            S = sm.Value();
        }
        var nchl = new Interface_CheckIterator();

        for (chl.Start(); chl.More(); chl.Next())
        {
            Interface_Check ach = chl.Value();
            if (ach.NbFails() + ach.NbWarnings() == 0)
            {
                continue;
            }
            Standard_Transient? ent = ach.Entity();
            if (ent is null)
            {
                continue;
            }
            if (S.IsNull())
            {
                if (ent == obj)
                {
                    Interface_Check bch = ach;
                    bch.SetEntity(ent);
                    nchl.Add(bch, 0);
                }
            }
            else
            {
                TopoDS_Shape sh = new TopoDS_Shape();
                TopoDS_HShape? hsh = ent as TopoDS_HShape;
                TransferBRep_BinderOfShape? sbs = ent as TransferBRep_BinderOfShape;
                TransferBRep_ShapeMapper? smp = ent as TransferBRep_ShapeMapper;
                if (hsh is not null)
                {
                    sh = hsh.Shape();
                }
                if (sbs is not null)
                {
                    sh = sbs.Result();
                }
                if (smp is not null)
                {
                    sh = smp.Value();
                }
                if (sh == S)
                {
                    Interface_Check bch = ach;
                    bch.SetEntity(ent);
                    nchl.Add(bch, 0);
                }
            }
        }
        return nchl;
    }

    /// <summary>Prints the results of transfer to given printer with given header.</summary>
    public static void PrintResultInfo(Message_Printer Printer, Message_Msg Header, TransferBRep_TransferResultInfo ResultInfo,
                                       bool printEmpty = true)
    {
        int R, RW, RF, RWF, NR, NRW, NRF, NRWF;
        R = ResultInfo.Result();
        RW = ResultInfo.ResultWarning();
        RF = ResultInfo.ResultFail();
        RWF = ResultInfo.ResultWarningFail();
        NR = ResultInfo.NoResult();
        NRW = ResultInfo.NoResultWarning();
        NRF = ResultInfo.NoResultFail();
        NRWF = ResultInfo.NoResultWarningFail();

        var aLocalHeader = new Message_Msg(Header);
        Printer.Send(aLocalHeader.Get(), Message_Info);

        var EPMSG30 = new Message_Msg("Result.Print.MSG30"); //    Result: %d
        EPMSG30.Arg(R);
        Printer.Send(EPMSG30.Get(), Message_Info);
        if (printEmpty || (RW > 0))
        {
            var EPMSG32 = new Message_Msg("Result.Print.MSG32"); //    Result + Warning(s): %d
            EPMSG32.Arg(RW);
            Printer.Send(EPMSG32.Get(), Message_Info);
        }
        if (printEmpty || (RF > 0))
        {
            var EPMSG34 = new Message_Msg("Result.Print.MSG34"); //    Result + Fail(s): %d
            EPMSG34.Arg(RF);
            Printer.Send(EPMSG34.Get(), Message_Info);
        }
        if (printEmpty || (RWF > 0))
        {
            var EPMSG36 = new Message_Msg("Result.Print.MSG36"); //    Result + Warning(s) + Fail(s): %d
            EPMSG36.Arg(RWF);
            Printer.Send(EPMSG36.Get(), Message_Info);
        }
        var EPMSG38 = new Message_Msg("Result.Print.MSG38"); //    TOTAL Result: %d
        EPMSG38.Arg(R + RW + RF + RWF);
        Printer.Send(EPMSG38.Get(), Message_Info);
        if (printEmpty || (NR > 0))
        {
            var EPMSG40 = new Message_Msg("Result.Print.MSG40"); //    No Result: %d
            EPMSG40.Arg(NR);
            Printer.Send(EPMSG40.Get(), Message_Info);
        }
        if (printEmpty || (NRW > 0))
        {
            var EPMSG42 = new Message_Msg("Result.Print.MSG42"); //    No Result + Warning(s): %d
            EPMSG42.Arg(NRW);
            Printer.Send(EPMSG42.Get(), Message_Info);
        }
        if (printEmpty || (NRF > 0))
        {
            var EPMSG44 = new Message_Msg("Result.Print.MSG44"); //    No Result + Fail(s): %d
            EPMSG44.Arg(NRF);
            Printer.Send(EPMSG44.Get(), Message_Info);
        }
        if (printEmpty || (NRWF > 0))
        {
            var EPMSG46 = new Message_Msg("Result.Print.MSG46"); //    No Result + Warning(s) + Fail(s): %d
            EPMSG46.Arg(NRWF);
            Printer.Send(EPMSG46.Get(), Message_Info);
        }
        var EPMSG48 = new Message_Msg("Result.Print.MSG48"); //    TOTAL No Result: %d
        EPMSG48.Arg(NR + NRW + NRF + NRWF);
        Printer.Send(EPMSG48.Get(), Message_Info);
    }
}
