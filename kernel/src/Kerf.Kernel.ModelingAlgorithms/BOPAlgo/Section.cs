// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Section.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Section.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>The section of the arguments: the edges and vertices where they meet.</summary>
/// <remarks>
/// The result is a compound of the section edges and vertices: the intersections of faces, the edges shared with
/// faces, the vertices found in faces, and the parts of the arguments' boundaries shared by several arguments.
/// </remarks>
public class Section : BooleanBuilder
{
    public Section()
    {
        Clear();
    }

    protected override void CheckData()
    {
        int nbArgs;
        //
        nbArgs = _arguments.Extent();
        if (nbArgs == 0)
        {
            AddError(new AlertTooFewArguments());
            return;
        }
        //
        CheckFiller();
    }

    /// <summary>The steps of the operation (OCCT's nested enum).</summary>
    protected new enum BOPAlgo_PIOperation
    {
        PIOperation_TreatVertices = 0,
        PIOperation_TreatEdges,
        PIOperation_BuildSection,
        PIOperation_FillHistory,
        PIOperation_PostTreat,
        PIOperation_Last,
    }

    protected override void FillPIConstants(double whole, PISteps steps)
    {
        // Fill in the constants:
        if (_fillHistory)
        {
            // for FillHistroty, which takes about 10% of the whole operation
            steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_FillHistory, 10.0 * whole / 100.0);
        }

        // and for PostTreat, which takes about 5% of the whole operation
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PostTreat, 5.0 * whole / 100.0);
    }

    protected override void FillPISteps(PISteps steps)
    {
        // Compute the rest of the operations - all depend on the number of sub-shapes of certain type
        NbShapes nbShapes = GetNbShapes();
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatVertices, nbShapes.NbVertices());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_TreatEdges, nbShapes.NbEdges());
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_BuildSection, nbShapes.NbEdges() + nbShapes.NbFaces());
    }

    protected override void PerformInternal1(PaveFiller filler, ProgressRange range)
    {
        using var ps = new ProgressScope(range, "Building result of SECTION operation", 100);
        _paveFiller = filler;
        _ds = _paveFiller.PDS();
        _context = _paveFiller.Context();
        //
        // 1. CheckData
        CheckData();
        if (HasErrors)
        {
            return;
        }
        //
        // 2. Prepare
        Prepare();
        if (HasErrors)
        {
            return;
        }
        //
        PISteps steps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
        AnalyzeProgress(100.0, steps);
        // 3. Fill Images
        // 3.1 Vertices
        FillImagesVertices(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatVertices)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Vertex);
        if (HasErrors)
        {
            return;
        }
        // 3.2 Edges
        FillImagesEdges(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_TreatEdges)));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Edge);
        if (HasErrors)
        {
            return;
        }
        // 4. Section
        BuildSection(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_BuildSection)));
        if (HasErrors)
        {
            return;
        }
        // 5.History
        PrepareHistory(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_FillHistory)));
        if (HasErrors)
        {
            return;
        }
        // 6. Post-treatment
        PostTreat(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PostTreat)));
    }

    /// <summary>Builds the section: the section edges and vertices, and the arguments' shared boundaries.</summary>
    protected virtual void BuildSection(ProgressRange range)
    {
        using var ps = new ProgressScope(range, "Building the result of Section operation", 1);
        int i, nbMS, nbLE;
        int j, nE, nV, nb, nbF, nbPBSc;
        Shape rc = new(), rc1 = new();
        BrepBuilder bb = new();
        Explorer exp = new();
        NCollection_List<Shape> lsa = new(), ls = new();
        NCollection_List<Shape>.Iterator it = new(), itIm = new(), itLS = new();
        NCollection_IndexedDataMap<Shape, int> msi = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> ms = new(ShapeHasher.Instance);
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        NCollection_IntMap.Iterator itMI = new();
        NCollection_List<PaveBlock>.Iterator itPB = new();
        //
        GetReport().Clear();
        //
        AlgoTools.MakeContainer(ShapeType.Compound, ref rc1);
        //
        // 1. aRC1
        nb = _ds!.SourceShapeCount;
        for (i = 0; i < nb; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            if (UserBreak(ps))
            {
                return;
            }
            //
            FaceInfo fi = _ds.FaceInfo(i);
            //
            // 1.1 Vertices that are section vertices (in OCCT's integer-map order, decision 784)
            NCollection_IntMap mvSc = fi.VerticesSc();
            itMI.Initialize(mvSc);
            for (; itMI.More(); itMI.Next())
            {
                nV = itMI.Key();
                Shape v = _ds.Shape(nV);
                bb.Add(rc1, v);
            }
            //
            // 1.2 Vertices that are in a face
            NCollection_IntMap mi = fi.VerticesIn();
            itMI.Initialize(mi);
            for (; itMI.More(); itMI.Next())
            {
                nV = itMI.Key();
                if (nV < 0)
                {
                    continue;
                }
                if (_ds.IsNewShape(nV) || _ds.HasInterf(nV))
                {
                    Shape v = _ds.Shape(nV);
                    bb.Add(rc1, v);
                }
            }
            //
            // 1.3 Section edges
            NCollection_IndexedMap<PaveBlock> mpbSc = fi.PaveBlocksSc();
            //
            nbPBSc = mpbSc.Extent();
            for (j = 1; j <= nbPBSc; ++j)
            {
                PaveBlock pb = mpbSc[j];
                nE = pb.Edge();
                Shape e = _ds.Shape(nE);
                bb.Add(rc1, e);
            }
        }
        //
        // 2. Common blocks between an edge and a face
        NCollection_DynamicArray<NCollection_List<PaveBlock>> pbp = _ds.PaveBlocksPool();
        //
        nb = pbp.Length();
        for (i = 0; i < nb; ++i)
        {
            NCollection_List<PaveBlock> lpb = pbp.Value(i);
            itPB.Initialize(lpb);
            for (; itPB.More(); itPB.Next())
            {
                PaveBlock pb = itPB.Value();
                CommonBlock? cb = _ds.CommonBlock(pb);
                if (cb is not null)
                {
                    NCollection_List<int> lf = cb.Faces();
                    nbF = lf.Extent();
                    if (nbF != 0)
                    {
                        PaveBlock pbr = cb.PaveBlock1();
                        nE = pbr.Edge();
                        Shape e = _ds.Shape(nE);
                        bb.Add(rc1, e);
                    }
                }
            }
        }
        //
        it.Initialize(_arguments);
        for (; it.More(); it.Next())
        {
            Shape sa = it.Value();
            if (mFence.Add(sa))
            {
                lsa.Append(sa);
            }
        }
        //
        mFence.Clear();
        //
        // 3. Treatment boundaries of arguments
        //
        // 3.1 Set to treat => aLS
        it.Initialize(lsa);
        for (; it.More(); it.Next())
        {
            if (UserBreak(ps))
            {
                return;
            }
            Shape sa = it.Value();
            //
            ls.Clear();
            ms.Clear();
            mFence.Clear();
            //
            exp.Init(sa, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Shape e = exp.Current();
                if (mFence.Add(e))
                {
                    ls.Append(e);
                }
            }
            exp.Init(sa, ShapeType.Vertex);
            for (; exp.More(); exp.Next())
            {
                Shape e = exp.Current();
                if (mFence.Add(e))
                {
                    ls.Append(e);
                }
            }
            //
            // 3.2 aMSI
            itLS.Initialize(ls);
            for (; itLS.More(); itLS.Next())
            {
                Shape s = itLS.Value();
                //
                if (_images.IsBound(s))
                {
                    NCollection_List<Shape> lsIm = _images.Find(s);
                    itIm.Initialize(lsIm);
                    for (; itIm.More(); itIm.Next())
                    {
                        Shape sIm = itIm.Value();
                        Exploration.MapShapes(sIm, ShapeType.Vertex, ms);
                        Exploration.MapShapes(sIm, ShapeType.Edge, ms);
                    }
                }
                else
                {
                    Exploration.MapShapes(s, ShapeType.Vertex, ms);
                    Exploration.MapShapes(s, ShapeType.Edge, ms);
                }
            }
            //
            nbMS = ms.Extent();
            for (i = 1; i <= nbMS; ++i)
            {
                Shape s = ms[i];
                if (msi.Contains(s))
                {
                    ref int iCnt = ref msi.ChangeFromKey(s);
                    ++iCnt;
                }
                else
                {
                    msi.Add(s, 1);
                }
            }
        }
        //
        ms.Clear();
        mFence.Clear();
        //
        // 4. Build the result
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance);
        //
        Exploration.MapShapesAndAncestors(rc1, ShapeType.Vertex, ShapeType.Edge, mve);
        //
        nbMS = msi.Extent();
        for (i = 1; i <= nbMS; ++i)
        {
            Shape v = msi.FindKey(i);
            int iCnt = msi.FindFromIndex(i);
            if (iCnt > 1)
            {
                Exploration.MapShapesAndAncestors(v, ShapeType.Vertex, ShapeType.Edge, mve);
            }
        }
        //
        AlgoTools.MakeContainer(ShapeType.Compound, ref rc);
        //
        nbMS = mve.Extent();
        for (i = 1; i <= nbMS; ++i)
        {
            if (UserBreak(ps))
            {
                return;
            }
            Shape v = mve.FindKey(i);
            NCollection_List<Shape> le = mve.FindFromIndex(i);
            nbLE = le.Extent();
            if (nbLE == 0)
            {
                // alone vertices
                if (mFence.Add(v))
                {
                    bb.Add(rc, v);
                }
            }
            else
            {
                // edges
                it.Initialize(le);
                for (; it.More(); it.Next())
                {
                    Shape e = it.Value();
                    if (mFence.Add(e))
                    {
                        bb.Add(rc, e);
                    }
                }
            }
        }
        //
        _shape = rc;
    }
}
