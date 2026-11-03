// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_AlgoContainer.hxx
// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_AlgoContainer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Resources;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Process;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The algorithms of Shape Healing and others for the data exchange processors: shape processing with a resource
/// file's sequence, pcurve checking, and the merge of the processing's history into a transfer process. OCCT keeps it
/// for compatibility (its readers and writers use <see cref="XSAlgo_ShapeProcessor"/>).
/// </summary>
/// <remarks>
/// The parameters OCCT reads from <c>Interface_Static</c> come from the <see cref="Interface_StaticSet"/> the caller
/// passes (decision 1181). <c>PrepareForTransfer</c> is left out (decision 1203).
/// </remarks>
public class XSAlgo_AlgoContainer : Transient
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
    public virtual Shape ProcessShape(in Shape theShape, double thePrec, double theMaxTol, string thePrscfile,
                                             string thePseq, ref Transient? theInfo, Interface_StaticSet theStatics,
                                             ProgressRange theProgress = default, bool theNonManifold = false,
                                             ShapeType theDetailingLevel = ShapeType.Vertex)
    {
        if (theShape.IsNull)
        {
            return theShape;
        }

        ShapeContext? aContext = theInfo as ShapeContext;
        if (aContext is null)
        {
            string aRscfile = theStatics.CVal(thePrscfile);
            aContext = new ShapeContext(theShape, aRscfile);
            if (!aContext.ResourceManager().IsInitialized)
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
        Manager aRsc = aContext.ResourceManager();
        string aStr = aSeq + ".exec.op";
        if (!aRsc.Find(aStr))
        {
            // if reading, do default ShapeFix
            if (thePseq.StartsWith("read.", StringComparison.Ordinal))
            {
                try
                {
                    var aMsg = new MsgRegistrator();
                    ShapeFixer aSfs = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().ToolContainer().FixShape();
                    aSfs.Init(theShape);
                    aSfs.SetMsgRegistrator(aMsg);
                    aSfs.SetPrecision(thePrec);
                    aSfs.SetMaxTolerance(theMaxTol);
                    aSfs.FixFaceTool().FixWireTool().FixSameParameterMode() = 0;
                    aSfs.FixSolidTool().CreateOpenSolidMode() = false;
                    aSfs.Perform(theProgress);

                    Shape aShape = aSfs.Shape();
                    if (!aShape.IsNull && aShape != theShape)
                    {
                        aContext.RecordModification(aSfs.Context(), aMsg);
                        aContext.SetResult(aShape);
                    }
                }
                catch (Exception anException) when (ModelAlgo.IsCaughtFailure(anException))
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

        if (!global::Kerf.Engine.Healing.Process.ShapeProcess.Perform(aContext, aSeq, theProgress))
        {
            return theShape; // return original shape
        }

        return aContext.Result();
    }

    /// <summary>Checks quality of pcurve of the edge on the given face, and corrects it if necessary.</summary>
    public virtual bool CheckPCurve(in Edge theEdge, in Face theFace, double thePrecision, bool theIsSeam,
                                    Interface_StaticSet theStatics)
    {
        return XSAlgo_ShapeProcessor.CheckPCurve(theEdge, theFace, thePrecision, theIsSeam, theStatics);
    }

    /// <summary>
    /// Updates translation map (TP) with information resulting from ShapeProcessing. Parameter
    /// <paramref name="startTPitem"/> can be used for optimisation, to restrict modifications to entities stored in TP
    /// starting from item <paramref name="startTPitem"/>.
    /// </summary>
    public virtual void MergeTransferInfo(Transfer_TransientProcess TP, Transient? info, int startTPitem = 1)
    {
        ShapeContext? context = info as ShapeContext;
        if (context is null)
        {
            return;
        }

        NCollection_DataMap<Shape, Shape> map = context.Map();
        MsgRegistrator? msg = context.Messages();
        if (map.Extent() <= 0 && (msg is null || msg.MapShape().Extent() <= 0))
        {
            return;
        }

        int i = startTPitem > 0 ? startTPitem : 1;
        for (; i <= TP.NbMapped(); i++)
        {
            Transfer_Binder? bnd = TP.MapItem(i);
            TransferBRep_ShapeBinder? sb = bnd as TransferBRep_ShapeBinder;
            if (sb is null || sb.Result().IsNull)
            {
                continue;
            }

            Shape orig = sb.Result();
            if (map.IsBound(orig))
            {
                sb.SetResult(map.Find(orig));
            }
            else if (!orig.Location().IsIdentity)
            {
                var aNullLoc = new Placement();
                Shape atmpSh = orig.Located(aNullLoc);
                if (map.IsBound(atmpSh))
                {
                    sb.SetResult(map.Find(atmpSh));
                }
            }
            else
            {
                // Some of edges may be modified.
                var aReShape = new ReShape();
                bool hasModifiedEdges = false;
                var anExpSE = new Explorer(orig, ShapeType.Edge);

                // Remember modifications.
                for (; anExpSE.More(); anExpSE.Next())
                {
                    if (map.IsBound(anExpSE.Current()))
                    {
                        hasModifiedEdges = true;
                        Shape aModifiedShape = map.Find(anExpSE.Current());
                        aReShape.Replace(anExpSE.Current(), aModifiedShape);
                    }
                }

                // Apply modifications and store result in binder.
                if (hasModifiedEdges)
                {
                    Shape aRes = aReShape.Apply(orig);
                    sb.SetResult(aRes);
                }
            }

            // update messages
            if (msg is not null)
            {
                NCollection_DataMap<Shape, NCollection_List<Msg>> msgmap = msg.MapShape();
                if (msgmap.IsBound(orig))
                {
                    NCollection_List<Msg> msglist = msgmap.Find(orig);
                    for (var iter = new NCollection_List<Msg>.Iterator(msglist); iter.More(); iter.Next())
                    {
                        Msg mess = iter.Value();
                        sb.AddWarning(mess.Value(), mess.Original());
                    }
                }
            }
        }
    }

    /// <summary>
    /// Updates translation map (FP) with information resulting from ShapeProcessing. The map of modified shapes is
    /// iterated in the order the processing recorded it, where OCCT's hash order varies (decision 1204).
    /// </summary>
    public virtual void MergeTransferInfo(Transfer_FinderProcess FP, Transient? info)
    {
        ShapeContext? context = info as ShapeContext;
        if (context is null)
        {
            return;
        }

        NCollection_DataMap<Shape, Shape> map = context.Map();
        MsgRegistrator? msg = context.Messages();
        foreach (KeyValuePair<Shape, Shape> ShapeShapeIterator in map)
        {
            Shape orig = ShapeShapeIterator.Key, res = ShapeShapeIterator.Value;
            TransferBRep_ShapeMapper resMapper = TransferBRep.ShapeMapper(FP, res);
            Transfer_Binder? resBinder = FP.Find(resMapper);

            if (resBinder is null)
            {
                resBinder = new TransferBRep_ShapeBinder(res);
                // if <orig> shape was split, put entities corresponding to new shapes
                //  into Transfer_TransientListBinder.
                if (orig.ShapeType() > res.ShapeType())
                {
                    Shape sub = new Shape();
                    var TransientListBinder = new Transfer_TransientListBinder();
                    for (var it = new Iterator(res); it.More(); it.Next())
                    {
                        Transfer_Finder? subMapper = TransferBRep.ShapeMapper(FP, it.Value());
                        if (subMapper is null)
                        {
                            continue;
                        }

                        Transient? tr = FP.FindTransient(subMapper);
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
                NCollection_DataMap<Shape, NCollection_List<Msg>> msgmap = msg.MapShape();
                if (msgmap.IsBound(orig))
                {
                    NCollection_List<Msg> msglist = msgmap.Find(orig);
                    for (var iter = new NCollection_List<Msg>.Iterator(msglist); iter.More(); iter.Next())
                    {
                        Msg mess = iter.Value();
                        resBinder.AddWarning(mess.Value(), mess.Original());
                    }
                }
            }
        }
    }
}
