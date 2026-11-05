// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

internal partial class PaveFiller
{
    /// <summary>The vertex/vertex interferences: the chains of coinciding vertices become same-domain vertices.</summary>
    protected virtual void PerformVV(ProgressRange range)
    {
        int n1, n2, iFlag, size;
        //
        _iterator!.Initialize(ShapeType.Vertex, ShapeType.Vertex);
        size = _iterator.ExpectedLength();
        using var ps = new ProgressScope(range, null, 2.0);
        if (size == 0)
        {
            return;
        }
        //
        List<InterfVV> vVs = _ds!.InterfVV();
        //
        //-----------------------------------------------------scope f
        IndexedMap<int, NCollection_List<int>> mili = new();
        NCollection_List<NCollection_List<int>> mBlocks = new();
        //
        // 1. Map V/LV
        // Split progress range on intersection stage and making blocks. Display only intersection stage.
        using (var psLoop = new ProgressScope(ps.Next(1.0), "Performing Vertex-Vertex intersection", size))
        {
            for (; _iterator.More(); _iterator.Next(), psLoop.Next())
            {
                if (UserBreak(ps))
                {
                    return;
                }
                _iterator.Value(out n1, out n2);
                //
                if (_ds.HasInterf(n1, n2))
                {
                    BuilderTools.FillMap(n1, n2, mili);
                    continue;
                }

                // Check for SD vertices
                int n1SD = n1;
                _ds.HasShapeSD(n1, ref n1SD);

                int n2SD = n2;
                _ds.HasShapeSD(n2, ref n2SD);

                Vertex v1 = ShapeCasts.Vertex(_ds.Shape(n1SD));
                Vertex v2 = ShapeCasts.Vertex(_ds.Shape(n2SD));

                iFlag = AlgoTools.ComputeVV(v1, v2, _fuzzyValue);
                if (iFlag == 0)
                {
                    BuilderTools.FillMap(n1, n2, mili);
                }
            }
        }
        //
        // 2. Make blocks
        BuilderTools.MakeBlocks(mili, mBlocks);
        //
        // 3. Make vertices
        NCollection_List<NCollection_List<int>>.Iterator itB = new(mBlocks);
        for (; itB.More(); itB.Next())
        {
            if (UserBreak(ps))
            {
                return;
            }
            NCollection_List<int> li = itB.Value();
            MakeSDVertices(li);
        }
        //
        // In OCCT's bucket order (decision 784): it decides the order in which InitPaveBlocksForVertex adds the edges'
        // lists to the pave-block pool, and so the order of MakeSplitEdges' new edges.
        NCollection_IntDataMap<int> dmii = _ds.ShapesSD();
        foreach (KeyValuePair<int, int> itDMII in dmii)
        {
            if (UserBreak(ps))
            {
                return;
            }
            n1 = itDMII.Key;
            _ds.InitPaveBlocksForVertex(n1);
        }
        //
        //-----------------------------------------------------scope t
        mBlocks.Clear();
        mili.Clear();
    }

    /// <summary>
    /// Makes one vertex of the given coinciding vertices (or updates their existing same-domain vertex), binds it as
    /// their same-domain vertex and adds the vertex/vertex interferences. Returns its index.
    /// </summary>
    protected int MakeSDVertices(NCollection_List<int> vertIndices, bool addInterfs = true)
    {
        Vertex vsd = new(), vn = new();
        int nSD = -1;
        NCollection_List<int>.Iterator itLI = new(vertIndices);
        NCollection_List<Shape> lv = new();
        for (; itLI.More(); itLI.Next())
        {
            int nX = itLI.Value(), nSD1 = 0;
            if (_ds!.HasShapeSD(nX, ref nSD1))
            {
                Shape vsd1 = _ds.Shape(nSD1);
                if (nSD == -1)
                {
                    vsd = ShapeCasts.Vertex(vsd1);
                    nSD = nSD1;
                }
                else
                {
                    lv.Append(vsd1);
                }
            }
            Shape v = _ds.Shape(nX);
            lv.Append(v);
        }
        AlgoTools.MakeVertex(lv, ref vn);
        int nV;
        if (nSD != -1)
        {
            // update old SD vertex with new value (a vertex the filler made: never an argument's)
            BrepTVertex tVertex = (BrepTVertex)vsd.TShape()!;
            tVertex.Pnt(Tool.Pnt(vn));
            tVertex.Tolerance(Tool.Tolerance(vn));
            vn = vsd;
            nV = nSD;
        }
        else
        {
            // Append new vertex to the DS
            ShapeInfo sIn = new();
            sIn.SetShapeType(ShapeType.Vertex);
            sIn.SetShape(vn);
            nV = _ds!.Append(sIn);
        }
        ShapeInfo sids = _ds!.ChangeShapeInfo(nV);
        ref Box box = ref sids.ChangeBox();
        box.Add(Tool.Pnt(vn));
        box.SetGap(Tool.Tolerance(vn) + global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        // Fill ShapesSD
        List<InterfVV> vVs = _ds.InterfVV();
        //
        itLI.Initialize(vertIndices);
        for (; itLI.More(); itLI.Next())
        {
            int n1 = itLI.Value();
            _ds.AddShapeSD(n1, nV);
            //
            int iR1 = _ds.Rank(n1);
            Shape v1 = _ds.Shape(n1);
            //
            NCollection_List<int>.Iterator itLI2 = new();
            itLI2.Reset(vertIndices, itLI.Index);
            itLI2.Next();
            for (; itLI2.More(); itLI2.Next())
            {
                int n2 = itLI2.Value();
                //
                if (iR1 >= 0 && iR1 == _ds.Rank(n2))
                {
                    // add warning status
                    Shape v2 = _ds.Shape(n2);
                    //
                    Compound wc = new();
                    new BrepBuilder().MakeCompound(ref wc);
                    new BrepBuilder().Add(wc, v1);
                    new BrepBuilder().Add(wc, v2);
                    //
                    AddWarning(new AlertSelfInterferingShape(wc));
                }
                //
                if (addInterfs)
                {
                    if (_ds.AddInterf(n1, n2))
                    {
                        InterfVV vv =  new InterfVV();
                        vVs.Add(vv);
                        vv.SetIndices(n1, n2);
                        vv.SetIndexNew(nV);
                    }
                }
            }
        }
        return nV;
    }
}
