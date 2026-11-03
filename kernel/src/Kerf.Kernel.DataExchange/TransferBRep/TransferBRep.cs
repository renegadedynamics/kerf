// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Framework.Transfer;

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
    private static void ShapeAppend(Binder? binder, NCollection_HSequence<Shape> shapes)
    {
        if (binder is null)
        {
            return;
        }
        if (binder.IsKind(typeof(BinderOfShape)))
        {
            BinderOfShape shbind = binder as BinderOfShape;
            if (shbind.HasResult)
            {
                shapes.Append(shbind.Result());
            }
        }
        else if (binder.IsKind(typeof(ShapeListBinder)))
        {
            ShapeListBinder slbind = binder as ShapeListBinder;
            int i, nb = slbind.ShapeCount;
            for (i = 1; i <= nb; i++)
            {
                shapes.Append(slbind.Shape(i));
            }
        }
        else if (binder.IsKind(typeof(SimpleBinderOfTransient)))
        {
            SimpleBinderOfTransient trbind = binder as SimpleBinderOfTransient;
            TopoDS_HShape? hs = trbind.Result() as TopoDS_HShape;
            if (hs is not null)
            {
                shapes.Append(hs.Shape());
            }
        }
        Binder? nextr = binder.NextResult();
        if (nextr is not null)
        {
            ShapeAppend(nextr, shapes);
        }
    }

    /// <summary>Get the Shape recorded in a Binder. If the Binder brings a multiple result, search for the Shape.</summary>
    public static Shape ShapeResult(Binder? binder)
    {
        Shape shape = new Shape();
        Binder? bnd = binder;
        while (bnd is not null)
        {
            BinderOfShape? shb = bnd as BinderOfShape;
            if (shb is not null)
            {
                return shb.Result();
            }
            SimpleBinderOfTransient? hsb = bnd as SimpleBinderOfTransient;
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
    public static Shape ShapeResult(TransientProcess tp, Transient? ent)
    {
        Shape shape = new Shape();
        Binder? binder = tp.Find(ent);
        if (binder is null)
        {
            binder = ent as Binder;
        }
        if (binder is not null)
        {
            return global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(binder);
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
    public static void SetShapeResult(TransientProcess? tp, Transient? ent, in Shape result)
    {
        if (result.IsNull || ent is null || tp is null)
        {
            return;
        }
        tp.Bind(ent, new ShapeBinder(result));
    }

    /// <summary>
    /// Gets the Shapes recorded in a TransientProcess as result of a Transfer, considers roots only or all results
    /// according <paramref name="roots"/>, returns them as a HSequence.
    /// </summary>
    public static NCollection_HSequence<Shape>? Shapes(TransientProcess? tp, bool roots = true)
    {
        NCollection_HSequence<Shape>? shapes = null;
        if (tp is null)
        {
            return shapes;
        }
        shapes = new NCollection_HSequence<Shape>();

        IteratorOfProcessForTransient list = roots ? tp.RootResult() : tp.CompleteResult();

        for (list.Start(); list.More(); list.Next())
        {
            Binder binder = list.Value();
            ShapeAppend(binder, shapes);
        }
        return shapes;
    }

    /// <summary>
    /// Gets the Shapes recorded in a TransientProcess as result of a Transfer, for a given list of starting entities,
    /// returns the shapes as a HSequence.
    /// </summary>
    public static NCollection_HSequence<Shape>? Shapes(TransientProcess? tp,
                                                              NCollection_HSequence<Transient>? list)
    {
        NCollection_HSequence<Shape>? shapes = null;
        if (tp is null && list is null) // OCCT: && where || is meant (either null is dereferenced below)
        {
            return shapes;
        }
        shapes = new NCollection_HSequence<Shape>();

        int ie, ne = list.Length();
        for (ie = 1; ie <= ne; ie++)
        {
            Binder? binder = tp.Find(list.Value(ie));
            ShapeAppend(binder, shapes);
        }

        return shapes;
    }

    /// <summary>
    /// Returns a Status regarding a Shape in a FinderProcess: FORWARD means bound with SAME Orientation; REVERSED
    /// means bound with REVERSE Orientation; EXTERNAL means NOT BOUND; INTERNAL is not used.
    /// </summary>
    public static Orientation ShapeState(FinderProcess? fp, in Shape shape)
    {
        if (fp is null || shape.IsNull)
        {
            return External;
        }
        ShapeMapper? sm = new ShapeMapper(shape);
        int index = fp.MapIndex(sm);
        if (index == 0)
        {
            return External;
        }
        sm = fp.Mapped(index) as ShapeMapper;
        if (sm is null)
        {
            return External;
        }
        Shape mapped = sm.Value();
        //  equality is assumed, we only test the orientation
        if (mapped.Orientation != shape.Orientation)
        {
            return Reversed;
        }
        return Forward;
    }

    /// <summary>Returns the result (as a Binder) attached to a given Shape. Null if none.</summary>
    public static Binder? ResultFromShape(FinderProcess? fp, in Shape shape)
    {
        Binder? res = null;
        if (fp is null || shape.IsNull)
        {
            return res;
        }
        var sm = new ShapeMapper(shape);
        return fp.Find(sm);
    }

    /// <summary>Returns the result as pure Transient attached to a Shape; first one if multiple result.</summary>
    public static Transient? TransientFromShape(FinderProcess? fp, in Shape shape)
    {
        Transient? res = null;
        if (fp is null || shape.IsNull)
        {
            return res;
        }
        var sm = new ShapeMapper(shape);
        return fp.FindTransient(sm);
    }

    /// <summary>Binds a Transient Result to a Shape in a FinderProcess (as first result if multiple : does not add it to existing one).</summary>
    public static void SetTransientFromShape(FinderProcess? fp, in Shape shape, Transient? result)
    {
        if (fp is null || shape.IsNull)
        {
            return;
        }
        var sm = new ShapeMapper(shape);
        fp.BindTransient(sm, result);
    }

    /// <summary>
    /// Returns a ShapeMapper for a given Shape (location included). Either <paramref name="shape"/> is already mapped,
    /// then its Mapper is returned. Or it is not, then a new one is created then returned, BUT it is not mapped here
    /// (use Bind or FindElseBind to do this).
    /// </summary>
    public static ShapeMapper ShapeMapper(FinderProcess fp, in Shape shape)
    {
        var mapper = new ShapeMapper(shape);
        int index = fp.MapIndex(mapper);
        if (index == 0)
        {
            return mapper;
        }
        return fp.Mapped(index) as ShapeMapper;
    }

    // Functions to collect transfer result information

    private static void FillInfo(Binder binder, FrameworkCheck check, TransferResultInfo info)
    {
        int r = 0, rw = 0, rf = 0, rwf = 0, nr = 0, nrw = 0, nrf = 0, nrwf = 0;
        if (binder.HasResult)
        {
            if (check.HasWarnings && check.HasFailed)
            {
                rwf++;
            }
            else if (check.HasWarnings)
            {
                rw++;
            }
            else if (check.HasFailed)
            {
                rf++;
            }
            else
            {
                r++;
            }
        }
        else if (check.HasWarnings && check.HasFailed)
        {
            nrwf++;
        }
        else if (check.HasWarnings)
        {
            nrw++;
        }
        else if (check.HasFailed)
        {
            nrf++;
        }
        else
        {
            nr++;
        }
        info.Result() += r;
        info.ResultWarning() += rw;
        info.ResultFail() += rf;
        info.ResultWarningFail() += rwf;
        info.NoResult() += nr;
        info.NoResultWarning() += nrw;
        info.NoResultFail() += nrf;
        info.NoResultWarningFail() += nrwf;
    }

    /// <summary>
    /// Fills sequence of TransferResultInfo for each type of entity given in the EntityTypes (entity are given as
    /// objects). Method IsKind applied to the entities in TP is used to compare with entities in EntityTypes.
    /// </summary>
    public static void TransferResultInfo(TransientProcess? tp, NCollection_HSequence<Transient>? entityTypes,
                                          out NCollection_HSequence<TransferResultInfo> infoSeq)
    {
        // create output Sequence in accordance with required ShapeTypes
        infoSeq = new NCollection_HSequence<TransferResultInfo>();
        if (tp is null || entityTypes is null)
        {
            return;
        }
        int seqLen = entityTypes.Length();
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= seqLen; i++)
        {
            infoSeq.Append(new TransferResultInfo());
        }

        // fill Sequence
        int nbMapped = tp.MappedCount;
        for (i = 1; i <= nbMapped; i++)
        {
            Transient? entity = tp.Mapped(i);

            Binder? binder = tp.Find(entity);
            if (binder is null)
            {
                continue;
            }
            FrameworkCheck check = binder.Check();

            // find appropriate element in the Sequence
            for (int index = 1; index <= seqLen; index++)
            {
                if (entity.IsKind(entityTypes.Value(index).DynamicType()))
                {
                    TransferResultInfo info = infoSeq.Value(index);
                    // fill element
                    FillInfo(binder, check, info);
                }
            }
        }
    }

    /// <summary>
    /// Fills sequence of TransferResultInfo for each type of shape given in the ShapeTypes (which are in fact
    /// considered as TopAbs_ShapeEnum). The Finders in the FP are considered as ShapeMappers.
    /// </summary>
    public static void TransferResultInfo(FinderProcess? fp, NCollection_HSequence<int>? shapeTypes,
                                          out NCollection_HSequence<TransferResultInfo> infoSeq)
    {
        // create output Sequence in accordance with required ShapeTypes
        infoSeq = new NCollection_HSequence<TransferResultInfo>();
        if (fp is null || shapeTypes is null)
        {
            return;
        }
        int seqLen = shapeTypes.Length();
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= seqLen; i++)
        {
            infoSeq.Append(new TransferResultInfo());
        }

        // fill Sequence
        int nbMapped = fp.MappedCount;
        for (i = 1; i <= nbMapped; i++)
        {
            ShapeMapper? mapper = fp.Mapped(i) as ShapeMapper;
            Binder? binder = fp.Find(mapper);
            if (binder is null)
            {
                continue;
            }
            FrameworkCheck check = binder.Check();

            Shape s = mapper.Value();
            ShapeType shapeType = s.Type;

            // find appropriate element in the Sequence
            for (int index = 1; index <= seqLen; index++)
            {
                // JR/Hp :
                ShapeType currentType = (ShapeType)shapeTypes.Value(index);
                //      TopAbs_ShapeEnum CurrentType = (TopAbs_ShapeEnum)ShapeTypes->Value (index);
                if (currentType == shapeType || currentType == ShapeType.Shape)
                {
                    TransferResultInfo info = infoSeq.Value(index);
                    // fill element
                    FillInfo(binder, check, info);
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
    public static CheckIterator ResultCheckList(CheckIterator chl, FinderProcess? fp,
                                                          InterfaceModel? model)
    {
        var nchl = new CheckIterator();
        if (fp is null || model is null)
        {
            return nchl;
        }
        nchl.SetModel(model);
        for (chl.Start(); chl.More(); chl.Next())
        {
            int num = 0;
            FrameworkCheck ach = chl.Value();
            if (ach.FailCount + ach.WarningCount == 0)
            {
                continue;
            }
            Finder? starting = ach.Entity() as Finder;
            Transient? ent = null;
            if (starting is not null)
            {
                ent = fp.FindTransient(starting);
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
    public static NCollection_HSequence<Transient> Checked(CheckIterator chl, bool alsoshapes = false)
    {
        var ls = new NCollection_HSequence<Transient>();
        for (chl.Start(); chl.More(); chl.Next())
        {
            FrameworkCheck ach = chl.Value();
            if (ach.FailCount + ach.WarningCount == 0)
            {
                continue;
            }
            Transient? ent = ach.Entity();
            if (ent is null)
            {
                continue;
            }
            if (!alsoshapes)
            {
                if (ent.IsKind(typeof(BinderOfShape)) || ent.IsKind(typeof(TopoDS_HShape))
                    || ent.IsKind(typeof(ShapeMapper)))
                {
                    continue;
                }
            }
            ls.Append(ent);
        }
        return ls;
    }

    /// <summary>Returns the list of shapes to which a non-empty Check is bound in a check-list.</summary>
    public static NCollection_HSequence<Shape> CheckedShapes(CheckIterator chl)
    {
        var ls = new NCollection_HSequence<Shape>();
        for (chl.Start(); chl.More(); chl.Next())
        {
            FrameworkCheck ach = chl.Value();
            if (ach.FailCount + ach.WarningCount == 0)
            {
                continue;
            }
            Transient? ent = ach.Entity();
            if (ent is null)
            {
                continue;
            }
            TopoDS_HShape? hs = ent as TopoDS_HShape;
            BinderOfShape? sb = ent as BinderOfShape;
            ShapeMapper? sm = ent as ShapeMapper;
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
    public static CheckIterator CheckObject(CheckIterator chl, Transient? obj)
    {
        Shape s = new Shape();
        TopoDS_HShape? hs = obj as TopoDS_HShape;
        BinderOfShape? sb = obj as BinderOfShape;
        ShapeMapper? sm = obj as ShapeMapper;
        if (hs is not null)
        {
            s = hs.Shape();
        }
        if (sb is not null)
        {
            s = sb.Result();
        }
        if (sm is not null)
        {
            s = sm.Value();
        }
        var nchl = new CheckIterator();

        for (chl.Start(); chl.More(); chl.Next())
        {
            FrameworkCheck ach = chl.Value();
            if (ach.FailCount + ach.WarningCount == 0)
            {
                continue;
            }
            Transient? ent = ach.Entity();
            if (ent is null)
            {
                continue;
            }
            if (s.IsNull)
            {
                if (ent == obj)
                {
                    FrameworkCheck bch = ach;
                    bch.SetEntity(ent);
                    nchl.Add(bch, 0);
                }
            }
            else
            {
                Shape sh = new Shape();
                TopoDS_HShape? hsh = ent as TopoDS_HShape;
                BinderOfShape? sbs = ent as BinderOfShape;
                ShapeMapper? smp = ent as ShapeMapper;
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
                if (sh == s)
                {
                    FrameworkCheck bch = ach;
                    bch.SetEntity(ent);
                    nchl.Add(bch, 0);
                }
            }
        }
        return nchl;
    }

    /// <summary>Prints the results of transfer to given printer with given header.</summary>
    public static void PrintResultInfo(Printer printer, Msg header, TransferResultInfo resultInfo,
                                       bool printEmpty = true)
    {
        int r, rw, rf, rwf, nr, nrw, nrf, nrwf;
        r = resultInfo.Result();
        rw = resultInfo.ResultWarning();
        rf = resultInfo.ResultFail();
        rwf = resultInfo.ResultWarningFail();
        nr = resultInfo.NoResult();
        nrw = resultInfo.NoResultWarning();
        nrf = resultInfo.NoResultFail();
        nrwf = resultInfo.NoResultWarningFail();

        var localHeader = new Msg(header);
        printer.Send(localHeader.Get(), Info);

        var epmsg30 = new Msg("Result.Print.MSG30"); //    Result: %d
        epmsg30.Arg(r);
        printer.Send(epmsg30.Get(), Info);
        if (printEmpty || (rw > 0))
        {
            var epmsg32 = new Msg("Result.Print.MSG32"); //    Result + Warning(s): %d
            epmsg32.Arg(rw);
            printer.Send(epmsg32.Get(), Info);
        }
        if (printEmpty || (rf > 0))
        {
            var epmsg34 = new Msg("Result.Print.MSG34"); //    Result + Fail(s): %d
            epmsg34.Arg(rf);
            printer.Send(epmsg34.Get(), Info);
        }
        if (printEmpty || (rwf > 0))
        {
            var epmsg36 = new Msg("Result.Print.MSG36"); //    Result + Warning(s) + Fail(s): %d
            epmsg36.Arg(rwf);
            printer.Send(epmsg36.Get(), Info);
        }
        var epmsg38 = new Msg("Result.Print.MSG38"); //    TOTAL Result: %d
        epmsg38.Arg(r + rw + rf + rwf);
        printer.Send(epmsg38.Get(), Info);
        if (printEmpty || (nr > 0))
        {
            var epmsg40 = new Msg("Result.Print.MSG40"); //    No Result: %d
            epmsg40.Arg(nr);
            printer.Send(epmsg40.Get(), Info);
        }
        if (printEmpty || (nrw > 0))
        {
            var epmsg42 = new Msg("Result.Print.MSG42"); //    No Result + Warning(s): %d
            epmsg42.Arg(nrw);
            printer.Send(epmsg42.Get(), Info);
        }
        if (printEmpty || (nrf > 0))
        {
            var epmsg44 = new Msg("Result.Print.MSG44"); //    No Result + Fail(s): %d
            epmsg44.Arg(nrf);
            printer.Send(epmsg44.Get(), Info);
        }
        if (printEmpty || (nrwf > 0))
        {
            var epmsg46 = new Msg("Result.Print.MSG46"); //    No Result + Warning(s) + Fail(s): %d
            epmsg46.Arg(nrwf);
            printer.Send(epmsg46.Get(), Info);
        }
        var epmsg48 = new Msg("Result.Print.MSG48"); //    TOTAL No Result: %d
        epmsg48.Arg(nr + nrw + nrf + nrwf);
        printer.Send(epmsg48.Get(), Info);
    }
}
