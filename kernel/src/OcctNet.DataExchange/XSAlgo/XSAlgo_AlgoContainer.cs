// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_AlgoContainer.hxx
// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_AlgoContainer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The algorithms of Shape Healing and others for the data exchange processors: shape processing with a resource
/// file's sequence, pcurve checking, and the merge of the processing's history into a transfer process. OCCT keeps it
/// for compatibility (its readers and writers use <see cref="XSAlgo_ShapeProcessor"/>).
/// </summary>
/// <remarks>
/// The parameters OCCT reads from <c>Interface_Static</c> come from the <see cref="Interface_StaticSet"/> the caller
/// passes (decision 1181). <c>PrepareForTransfer</c> is left out (decision 12405).
/// </remarks>
public class XSAlgo_AlgoContainer : Standard_Transient
{
    /// <summary>Empty constructor.</summary>
    public XSAlgo_AlgoContainer()
    {
    }

    /// <summary>
    /// Does shape processing with specified tolerances. <paramref name="thePrscfile"/>: the static holding the name of
    /// the resource file; <paramref name="thePseq"/>: the static holding the name of the sequence of operators defined
    /// in the resource file for Shape Processing; <paramref name="theInfo"/>: information to be recorded in the
    /// translation map (the context; an existing context is reused); <paramref name="theNonManifold"/>: flag to proceed
    /// with non-manifold topology; <paramref name="theDetailingLevel"/>: the lowest shape type to be processed, lower
    /// shapes are ignored. Returns the processed shape.
    /// </summary>
    public virtual TopoDS_Shape ProcessShape(in TopoDS_Shape theShape, double thePrec, double theMaxTol, string thePrscfile,
                                             string thePseq, ref Standard_Transient? theInfo, Interface_StaticSet theStatics,
                                             Message_ProgressRange theProgress = default, bool theNonManifold = false,
                                             TopAbs_ShapeEnum theDetailingLevel = TopAbs_VERTEX)
    {
        if (theShape.IsNull())
        {
            return theShape;
        }

        ShapeProcess_ShapeContext? aContext = theInfo as ShapeProcess_ShapeContext;
        if (aContext is null)
        {
            string aRscfile = theStatics.CVal(thePrscfile);
            aContext = new ShapeProcess_ShapeContext(theShape, aRscfile);
            if (!aContext.ResourceManager().IsInitialized())
            {
                // If resource file wasn't found, use static values instead
                theStatics.FillMap(aContext.ResourceManager().GetMap());
            }
            aContext.SetDetalisation(theDetailingLevel);
        }
        aContext.SetNonManifold(theNonManifold);
        theInfo = aContext;

        string aSeq = theStatics.CVal(thePseq);
        // OCCT: if (!aSeq) aSeq = thePseq; CVal never returns a null string

        // if resource file is not loaded or does not define <seq>.exec.op,
        // do default fixes
        Resource_Manager aRsc = aContext.ResourceManager();
        string aStr = aSeq + ".exec.op";
        if (!aRsc.Find(aStr))
        {
            // if reading, do default ShapeFix
            if (thePseq.StartsWith("read.", StringComparison.Ordinal))
            {
                try
                {
                    var aMsg = new ShapeExtend_MsgRegistrator();
                    ShapeFix_Shape aSfs = ShapeAlgo.AlgoContainer().ToolContainer().FixShape();
                    aSfs.Init(theShape);
                    aSfs.SetMsgRegistrator(aMsg);
                    aSfs.SetPrecision(thePrec);
                    aSfs.SetMaxTolerance(theMaxTol);
                    aSfs.FixFaceTool().FixWireTool().FixSameParameterMode() = 0;
                    aSfs.FixSolidTool().CreateOpenSolidMode() = false;
                    aSfs.Perform(theProgress);

                    TopoDS_Shape aShape = aSfs.Shape();
                    if (!aShape.IsNull() && aShape != theShape)
                    {
                        aContext.RecordModification(aSfs.Context(), aMsg);
                        aContext.SetResult(aShape);
                    }
                }
                catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
                {
                    // OCC_CATCH_SIGNALS: a Standard_Failure, or a system signal (decision 1183)
                }
                return aContext.Result();
            }
            // for writing, define default sequence of DirectFaces
            else if (thePseq.StartsWith("write.", StringComparison.Ordinal))
            {
                aRsc.SetResource(aStr, "DirectFaces");
            }
        }

        // Define runtime tolerances and do Shape Processing
        aRsc.SetResource("Runtime.Tolerance", thePrec);
        aRsc.SetResource("Runtime.MaxTolerance", theMaxTol);

        if (!ShapeProcess.Perform(aContext, aSeq, theProgress))
        {
            return theShape; // return original shape
        }

        return aContext.Result();
    }

    /// <summary>Checks quality of pcurve of the edge on the given face, and corrects it if necessary.</summary>
    public virtual bool CheckPCurve(in TopoDS_Edge theEdge, in TopoDS_Face theFace, double thePrecision, bool theIsSeam,
                                    Interface_StaticSet theStatics)
    {
        return XSAlgo_ShapeProcessor.CheckPCurve(theEdge, theFace, thePrecision, theIsSeam, theStatics);
    }

    /// <summary>
    /// Updates translation map (TP) with information resulting from ShapeProcessing. Parameter
    /// <paramref name="startTPitem"/> can be used for optimisation, to restrict modifications to entities stored in TP
    /// starting from item <paramref name="startTPitem"/>.
    /// </summary>
    public virtual void MergeTransferInfo(Transfer_TransientProcess TP, Standard_Transient? info, int startTPitem = 1)
    {
        ShapeProcess_ShapeContext? context = info as ShapeProcess_ShapeContext;
        if (context is null)
        {
            return;
        }

        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> map = context.Map();
        ShapeExtend_MsgRegistrator? msg = context.Messages();
        if (map.Extent() <= 0 && (msg is null || msg.MapShape().Extent() <= 0))
        {
            return;
        }

        int i = startTPitem > 0 ? startTPitem : 1;
        for (; i <= TP.NbMapped(); i++)
        {
            Transfer_Binder? bnd = TP.MapItem(i);
            TransferBRep_ShapeBinder? sb = bnd as TransferBRep_ShapeBinder;
            if (sb is null || sb.Result().IsNull())
            {
                continue;
            }

            TopoDS_Shape orig = sb.Result();
            if (map.IsBound(orig))
            {
                sb.SetResult(map.Find(orig));
            }
            else if (!orig.Location().IsIdentity())
            {
                var aNullLoc = new TopLoc_Location();
                TopoDS_Shape atmpSh = orig.Located(aNullLoc);
                if (map.IsBound(atmpSh))
                {
                    sb.SetResult(map.Find(atmpSh));
                }
            }
            else
            {
                // Some of edges may be modified.
                var aReShape = new BRepTools_ReShape();
                bool hasModifiedEdges = false;
                var anExpSE = new TopExp_Explorer(orig, TopAbs_EDGE);

                // Remember modifications.
                for (; anExpSE.More(); anExpSE.Next())
                {
                    if (map.IsBound(anExpSE.Current()))
                    {
                        hasModifiedEdges = true;
                        TopoDS_Shape aModifiedShape = map.Find(anExpSE.Current());
                        aReShape.Replace(anExpSE.Current(), aModifiedShape);
                    }
                }

                // Apply modifications and store result in binder.
                if (hasModifiedEdges)
                {
                    TopoDS_Shape aRes = aReShape.Apply(orig);
                    sb.SetResult(aRes);
                }
            }

            // update messages
            if (msg is not null)
            {
                NCollection_DataMap<TopoDS_Shape, NCollection_List<Message_Msg>> msgmap = msg.MapShape();
                if (msgmap.IsBound(orig))
                {
                    NCollection_List<Message_Msg> msglist = msgmap.Find(orig);
                    for (var iter = new NCollection_List<Message_Msg>.Iterator(msglist); iter.More(); iter.Next())
                    {
                        Message_Msg mess = iter.Value();
                        sb.AddWarning(mess.Value(), mess.Original());
                    }
                }
            }
        }
    }

    /// <summary>
    /// Updates translation map (FP) with information resulting from ShapeProcessing. The map of modified shapes is
    /// iterated in the order the processing recorded it, where OCCT's hash order varies (decision 12406).
    /// </summary>
    public virtual void MergeTransferInfo(Transfer_FinderProcess FP, Standard_Transient? info)
    {
        ShapeProcess_ShapeContext? context = info as ShapeProcess_ShapeContext;
        if (context is null)
        {
            return;
        }

        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> map = context.Map();
        ShapeExtend_MsgRegistrator? msg = context.Messages();
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> ShapeShapeIterator in map)
        {
            TopoDS_Shape orig = ShapeShapeIterator.Key, res = ShapeShapeIterator.Value;
            TransferBRep_ShapeMapper resMapper = TransferBRep.ShapeMapper(FP, res);
            Transfer_Binder? resBinder = FP.Find(resMapper);

            if (resBinder is null)
            {
                resBinder = new TransferBRep_ShapeBinder(res);
                // if <orig> shape was split, put entities corresponding to new shapes
                //  into Transfer_TransientListBinder.
                if (orig.ShapeType() > res.ShapeType())
                {
                    TopoDS_Shape sub = new TopoDS_Shape();
                    var TransientListBinder = new Transfer_TransientListBinder();
                    for (var it = new TopoDS_Iterator(res); it.More(); it.Next())
                    {
                        Transfer_Finder? subMapper = TransferBRep.ShapeMapper(FP, it.Value());
                        if (subMapper is null)
                        {
                            continue;
                        }

                        Standard_Transient? tr = FP.FindTransient(subMapper);
                        if (tr is null)
                        {
                            continue;
                        }
                        TransientListBinder.AddResult(tr);
                        sub = it.Value();
                    }
                    if (TransientListBinder.NbTransients() == 1)
                    {
                        resBinder = new TransferBRep_ShapeBinder(sub);
                    }
                    else if (TransientListBinder.NbTransients() > 1)
                    {
                        resBinder.AddResult(TransientListBinder);
                    }
                }
            }

            TransferBRep_ShapeMapper origMapper = TransferBRep.ShapeMapper(FP, orig);
            Transfer_Binder? origBinder = FP.Find(origMapper);
            if (origBinder is null)
            {
                FP.Bind(origMapper, resBinder);
            }
            else
            {
                origBinder.AddResult(resBinder);
            }

            // update messages
            if (msg is not null)
            {
                NCollection_DataMap<TopoDS_Shape, NCollection_List<Message_Msg>> msgmap = msg.MapShape();
                if (msgmap.IsBound(orig))
                {
                    NCollection_List<Message_Msg> msglist = msgmap.Find(orig);
                    for (var iter = new NCollection_List<Message_Msg>.Iterator(msglist); iter.More(); iter.Next())
                    {
                        Message_Msg mess = iter.Value();
                        resBinder.AddWarning(mess.Value(), mess.Original());
                    }
                }
            }
        }
    }
}
