// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_10.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

public partial class PaveFiller
{
    /// <summary>
    /// Turns the non-destructive mode on if an argument is locked: if a write into it would be refused. Outside a
    /// working overlay frozen shapes are locked, so every operation on a published shape is non-destructive
    /// (decision 577), and so is one on an argument holding frozen sub-shapes (decision 1056). Inside an operation's
    /// overlay a frozen shape takes writes (its working copy does), so a Boolean that the operation runs on it is
    /// destructive, as OCCT's on an unlocked shape.
    /// </summary>
    protected void SetNonDestructive()
    {
        if (!_isPrimary || _nonDestructive)
        {
            return;
        }
        //
        bool bFlag;
        NCollection_List<Shape>.Iterator itLS = new();
        //
        bFlag = false;
        itLS.Initialize(_arguments);
        for (; itLS.More() && (!bFlag); itLS.Next())
        {
            Shape s = itLS.Value();
            // Kerf: OCCT's s.Locked(), asked of the overlay, and of the argument's sub-shapes too: an unfrozen argument
            // may hold frozen sub-shapes (ported code's intermediate shapes sharing a caller's unchanged ones, decision
            // 1013), which a destructive operation could write only in an overlay (decision 1056)
            TShape tShape = s.TShape() ?? throw new NullObjectException("TopoDS_Shape: the shape is null");
            bFlag = WorkingOverlay.IsLocked(tShape) || HoldsLocked(tShape);
        }
        _nonDestructive = bFlag;
    }

    // True if a frozen TShape below tShape would refuse a write: none does in an overlay, any does outside one. A
    // frozen TShape's sub-graph is frozen, so the walk stops at the first.
    private static bool HoldsLocked(TShape tShape)
    {
        if (WorkingOverlay.Current is not null)
        {
            return false;
        }
        var visited = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        var stack = new Stack<TShape>();
        stack.Push(tShape);
        while (stack.Count > 0)
        {
            TShape t = stack.Pop();
            if (t.IsFrozen)
            {
                return true;
            }
            if (!visited.Add(t))
            {
                continue;
            }
            for (var it = new Iterator(new Shape(t, new Placement(), Orientation.Forward)); it.More(); it.Next())
            {
                if (it.Value().TShape() is { } child)
                {
                    stack.Push(child);
                }
            }
        }
        return false;
    }

    /// <summary>Increases the tolerance of the edge and of its vertices (not of an argument edge when non-destructive).</summary>
    protected void UpdateEdgeTolerance(int nE, double tol)
    {
        ShapeInfo sie = _ds!.ChangeShapeInfo(nE);
        NCollection_List<int> li = sie.SubShapes();

        // For the safe input mode avoid modifying the input shapes
        if (_nonDestructive)
        {
            if (!_ds.IsNewShape(nE))
            {
                return;
            }

            NCollection_List<int>.Iterator itLI0 = new(li);
            for (; itLI0.More(); itLI0.Next())
            {
                int nV = itLI0.Value(), nVSD = 0;
                if (!_ds.IsNewShape(nV) && !_ds.HasShapeSD(nV, ref nVSD))
                {
                    return;
                }
            }
        }

        // Update edge
        Edge e = ShapeCasts.Edge(_ds.Shape(nE));
        new BrepBuilder().UpdateEdge(e, tol);
        ref Box boxE = ref sie.ChangeBox();
        Bounds.Add(e, ref boxE);
        boxE.SetGap(boxE.GetGap() + global::Kerf.Engine.Foundation.Precision.Confusion());

        // Update vertices
        NCollection_List<int>.Iterator itLI = new(li);
        for (; itLI.More(); itLI.Next())
        {
            int nV = itLI.Value();
            UpdateVertex(nV, tol);
        }
    }

    /// <summary>
    /// Increases the tolerance of the vertex. In non-destructive mode an argument vertex without a same-domain one gets
    /// a new vertex instead, bound as its same-domain vertex. Returns the index of the vertex updated.
    /// </summary>
    protected int UpdateVertex(int nV, double aTolNew)
    {
        int nVNew;
        double tolV;
        BrepBuilder bb = new();

        nVNew = nV;
        if (_ds!.IsNewShape(nVNew) || _ds.HasShapeSD(nV, ref nVNew) || !_nonDestructive)
        {
            // nV is a new vertex, it has SD or non-destructive mode is not in force
            Vertex vsd = ShapeCasts.Vertex(_ds.Shape(nVNew));
            tolV = Tool.Tolerance(vsd);
            if (tolV < aTolNew)
            {
                bb.UpdateVertex(vsd, aTolNew);
                ShapeInfo siv = _ds.ChangeShapeInfo(nVNew);
                ref Box boxV = ref siv.ChangeBox();
                Bounds.Add(vsd, ref boxV);
                boxV.SetGap(boxV.GetGap() + global::Kerf.Engine.Foundation.Precision.Confusion());
                _increasedSS.Add(nV);
            }
            return nVNew;
        }
        //
        // nV is old vertex
        Vertex v = ShapeCasts.Vertex(_ds.Shape(nV));
        tolV = Tool.Tolerance(v);
        //
        // create new vertex
        Vertex vNew = new();
        Point3D pv = Tool.Pnt(v);
        bb.MakeVertex(ref vNew, pv, Max(tolV, aTolNew));
        //
        // append new vertex to DS
        ShapeInfo siv0 = new();
        siv0.SetShapeType(ShapeType.Vertex);
        siv0.SetShape(vNew);
        nVNew = _ds.Append(siv0);
        //
        // bounding box for the new vertex
        ShapeInfo sids = _ds.ChangeShapeInfo(nVNew);
        ref Box boxDS = ref sids.ChangeBox();
        Bounds.Add(vNew, ref boxDS);
        boxDS.SetGap(boxDS.GetGap() + global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        // add vertex to SD map
        _ds.AddShapeSD(nV, nVNew);

        // Add new vertex to map of vertices to avoid further extension
        _vertsToAvoidExtension.Add(nVNew);

        if (tolV < aTolNew)
        {
            _increasedSS.Add(nV);
        }

        return nVNew;
    }

    /// <summary>Replaces the vertices of the pave blocks by their same-domain vertices.</summary>
    protected void UpdatePaveBlocksWithSDVertices() => _ds!.UpdatePaveBlocksWithSDVertices();

    /// <summary>Replaces the vertices of the common blocks by their same-domain vertices.</summary>
    protected void UpdateCommonBlocksWithSDVertices()
    {
        if (!_nonDestructive)
        {
            UpdatePaveBlocksWithSDVertices();
            return;
        }
        int nbPBP;
        //
        List<NCollection_List<PaveBlock>> pbp = _ds!.ChangePaveBlocksPool();
        nbPBP = pbp.Count;
        if (nbPBP == 0)
        {
            return;
        }
        //
        int i, nV1, nV2;
        double tolV;
        HashSet<CommonBlock> mcb = new();
        NCollection_List<PaveBlock>.Iterator itPB = new();
        PaveBlock pb;
        //
        tolV = global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        for (i = 0; i < nbPBP; ++i)
        {
            NCollection_List<PaveBlock> lpb = pbp[i];
            itPB.Initialize(lpb);
            for (; itPB.More(); itPB.Next())
            {
                pb = itPB.Value();
                CommonBlock? cb = _ds.CommonBlock(pb);
                if (cb is null)
                {
                    continue;
                }
                //
                if (mcb.Add(cb))
                {
                    pb.Indices(out nV1, out nV2);
                    UpdateVertex(nV1, tolV);
                    UpdateVertex(nV2, tolV);
                    _ds.UpdateCommonBlockWithSDVertices(cb);
                }
            }
        }
        UpdatePaveBlocksWithSDVertices();
    }

    private static void UpdateIntfsWithSDVertices<TInterfType>(DS ds, List<TInterfType> interfs)
        where TInterfType : Interf
    {
        for (int i = 0; i < interfs.Count; i++)
        {
            TInterfType intf = interfs[i];
            if (intf.HasIndexNew(out int ind))
            {
                int indSD = 0;
                if (ds.HasShapeSD(ind, ref indSD))
                {
                    intf.SetIndexNew(indSD);
                }
            }
        }
    }

    /// <summary>Replaces the new vertices of the interferences by their same-domain vertices.</summary>
    protected void UpdateInterfsWithSDVertices()
    {
        UpdateIntfsWithSDVertices(_ds!, _ds!.InterfVV());
        UpdateIntfsWithSDVertices(_ds, _ds.InterfVE());
        UpdateIntfsWithSDVertices(_ds, _ds.InterfVF());
        UpdateIntfsWithSDVertices(_ds, _ds.InterfEE());
        UpdateIntfsWithSDVertices(_ds, _ds.InterfEF());
    }
}
