// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_AlgoContainer.hxx
// Port of: src/DataExchange/TKXSBase/XSAlgo/XSAlgo_AlgoContainer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Resources;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Process;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// The algorithms of Shape Healing and others for the data exchange processors: shape processing with a resource
/// file's sequence, pcurve checking, and the merge of the processing's history into a transfer process. OCCT keeps it
/// for compatibility (its readers and writers use <see cref="ShapeProcessor"/>).
/// </summary>
/// <remarks>
/// The parameters OCCT reads from <c>Interface_Static</c> come from the <see cref="StaticSet"/> the caller
/// passes (decision 1181). <c>PrepareForTransfer</c> is left out (decision 1203).
/// </remarks>
internal class FrameworkAlgoContainer : Transient
{
    /// <summary>Empty constructor.</summary>
    public FrameworkAlgoContainer()
    {
    }

    /// <summary>
    /// Does shape processing with specified tolerances. <paramref name="prscfile"/>: the static holding the name of
    /// the resource file; <paramref name="pseq"/>: the static holding the name of the sequence of operators defined
    /// in the resource file for Shape Processing; <paramref name="info"/>: information to be recorded in the
    /// translation map (the context; an existing context is reused); <paramref name="nonManifold"/>: flag to proceed
    /// with non-manifold topology; <paramref name="detailingLevel"/>: the lowest shape type to be processed, lower
    /// shapes are ignored. Returns the processed shape.
    /// </summary>
    public virtual Shape ProcessShape(in Shape shape, double prec, double maxTol, string prscfile,
                                             string pseq, ref Transient? info, StaticSet statics,
                                             ProgressRange progress = default, bool nonManifold = false,
                                             ShapeType detailingLevel = ShapeType.Vertex)
    {
        if (shape.IsNull)
        {
            return shape;
        }

        ShapeContext? context = info as ShapeContext;
        if (context is null)
        {
            string rscfile = statics.CVal(prscfile);
            context = new ShapeContext(shape, rscfile);
            if (!context.ResourceManager().IsInitialized)
            {
                // If resource file wasn't found, use static values instead
                statics.FillMap(context.ResourceManager().GetMap());
            }
            context.SetDetalisation(detailingLevel);
        }
        context.SetNonManifold(nonManifold);
        info = context;

        string seq = statics.CVal(pseq);
        // OCCT: if (!aSeq) aSeq = thePseq; CVal never returns a null string

        // if resource file is not loaded or does not define <seq>.exec.op,
        // do default fixes
        Manager rsc = context.ResourceManager();
        string str = seq + ".exec.op";
        if (!rsc.Find(str))
        {
            // if reading, do default ShapeFix
            if (pseq.StartsWith("read.", StringComparison.Ordinal))
            {
                try
                {
                    var msg = new MsgRegistrator();
                    ShapeFixer sfs = global::Kerf.Engine.Healing.ShapeAlgo.AlgoContainer().ToolContainer().FixShape();
                    sfs.Init(shape);
                    sfs.SetMsgRegistrator(msg);
                    sfs.SetPrecision(prec);
                    sfs.SetMaxTolerance(maxTol);
                    sfs.FixFaceTool().FixWireTool().FixSameParameterMode() = 0;
                    sfs.FixSolidTool().CreateOpenSolidMode() = false;
                    sfs.Perform(progress);

                    Shape aShape = sfs.Shape();
                    if (!aShape.IsNull && aShape != shape)
                    {
                        context.RecordModification(sfs.Context(), msg);
                        context.SetResult(aShape);
                    }
                }
                catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
                {
                    // OCC_CATCH_SIGNALS: a Standard_Failure, or a system signal (decision 1183)
                }
                return context.Result();
            }
            // for writing, define default sequence of DirectFaces
            else if (pseq.StartsWith("write.", StringComparison.Ordinal))
            {
                rsc.SetResource(str, "DirectFaces");
            }
        }

        // Define runtime tolerances and do Shape Processing
        rsc.SetResource("Runtime.Tolerance", prec);
        rsc.SetResource("Runtime.MaxTolerance", maxTol);

        if (!global::Kerf.Engine.Healing.Process.ShapeProcess.Perform(context, seq, progress))
        {
            return shape; // return original shape
        }

        return context.Result();
    }

    /// <summary>Checks quality of pcurve of the edge on the given face, and corrects it if necessary.</summary>
    public virtual bool CheckPCurve(in Edge edge, in Face face, double precision, bool isSeam,
                                    StaticSet statics)
    {
        return ShapeProcessor.CheckPCurve(edge, face, precision, isSeam, statics);
    }

    /// <summary>
    /// Updates translation map (TP) with information resulting from ShapeProcessing. Parameter
    /// <paramref name="startTPitem"/> can be used for optimisation, to restrict modifications to entities stored in TP
    /// starting from item <paramref name="startTPitem"/>.
    /// </summary>
    public virtual void MergeTransferInfo(TransientProcess tp, Transient? info, int startTPitem = 1)
    {
        ShapeContext? context = info as ShapeContext;
        if (context is null)
        {
            return;
        }

        Dictionary<Shape, Shape> map = context.Map();
        MsgRegistrator? msg = context.Messages();
        if (map.Count <= 0 && (msg is null || msg.MapShape().Count <= 0))
        {
            return;
        }

        int i = startTPitem > 0 ? startTPitem : 1;
        for (; i <= tp.MappedCount; i++)
        {
            Binder? bnd = tp.MapItem(i);
            ShapeBinder? sb = bnd as ShapeBinder;
            if (sb is null || sb.Result().IsNull)
            {
                continue;
            }

            Shape orig = sb.Result();
            if (map.ContainsKey(orig))
            {
                sb.SetResult(map.GetValue(orig));
            }
            else if (!orig.Placement.IsIdentity)
            {
                var nullLoc = new Placement();
                Shape atmpSh = orig.Located(nullLoc);
                if (map.ContainsKey(atmpSh))
                {
                    sb.SetResult(map.GetValue(atmpSh));
                }
            }
            else
            {
                // Some of edges may be modified.
                var reShape = new ReShape();
                bool hasModifiedEdges = false;
                var expSE = new Explorer(orig, ShapeType.Edge);

                // Remember modifications.
                for (; expSE.More(); expSE.Next())
                {
                    if (map.ContainsKey(expSE.Current()))
                    {
                        hasModifiedEdges = true;
                        Shape modifiedShape = map.GetValue(expSE.Current());
                        reShape.Replace(expSE.Current(), modifiedShape);
                    }
                }

                // Apply modifications and store result in binder.
                if (hasModifiedEdges)
                {
                    Shape res = reShape.Apply(orig);
                    sb.SetResult(res);
                }
            }

            // update messages
            if (msg is not null)
            {
                Dictionary<Shape, NCollection_List<Msg>> msgmap = msg.MapShape();
                if (msgmap.ContainsKey(orig))
                {
                    NCollection_List<Msg> msglist = msgmap.GetValue(orig);
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
    public virtual void MergeTransferInfo(FinderProcess fp, Transient? info)
    {
        ShapeContext? context = info as ShapeContext;
        if (context is null)
        {
            return;
        }

        Dictionary<Shape, Shape> map = context.Map();
        MsgRegistrator? msg = context.Messages();
        foreach (KeyValuePair<Shape, Shape> shapeShapeIterator in map)
        {
            Shape orig = shapeShapeIterator.Key, res = shapeShapeIterator.Value;
            ShapeMapper resMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, res);
            Binder? resBinder = fp.Find(resMapper);

            if (resBinder is null)
            {
                resBinder = new ShapeBinder(res);
                // if <orig> shape was split, put entities corresponding to new shapes
                //  into Transfer_TransientListBinder.
                if (orig.Type > res.Type)
                {
                    Shape sub = new Shape();
                    var transientListBinder = new TransientListBinder();
                    for (var it = new Iterator(res); it.More(); it.Next())
                    {
                        Finder? subMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, it.Value());
                        if (subMapper is null)
                        {
                            continue;
                        }

                        Transient? tr = fp.FindTransient(subMapper);
                        if (tr is null)
                        {
                            continue;
                        }
                        transientListBinder.AddResult(tr);
                        sub = it.Value();
                    }
                    if (transientListBinder.TransientCount == 1)
                    {
                        resBinder = new ShapeBinder(sub);
                    }
                    else if (transientListBinder.TransientCount > 1)
                    {
                        resBinder.AddResult(transientListBinder);
                    }
                }
            }

            ShapeMapper origMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, orig);
            Binder? origBinder = fp.Find(origMapper);
            if (origBinder is null)
            {
                fp.Bind(origMapper, resBinder);
            }
            else
            {
                origBinder.AddResult(resBinder);
            }

            // update messages
            if (msg is not null)
            {
                Dictionary<Shape, NCollection_List<Msg>> msgmap = msg.MapShape();
                if (msgmap.ContainsKey(orig))
                {
                    NCollection_List<Msg> msglist = msgmap.GetValue(orig);
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
