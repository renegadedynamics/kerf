// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

public partial class BooleanBuilder
{
    /// <summary>Fills the images of the vertices: each vertex to its same-domain vertex.</summary>
    protected void FillImagesVertices(ProgressRange range)
    {
        using var ps = new ProgressScope(range, "Filling splits of vertices", _ds!.ShapesSD().Extent());
        // the DS's integer map iterates in OCCT's order (decision 784)
        NCollection_IntDataMap<int>.Iterator it = new(_ds.ShapesSD());
        for (; it.More(); it.Next(), ps.Next())
        {
            if (UserBreak(ps))
            {
                return;
            }
            int nV = it.Key();
            int nVSD = it.Value();

            Shape v = _ds.Shape(nV);
            Shape vsd = _ds.Shape(nVSD);
            // Add to Images map
            _images.SetValueRef(v, new NCollection_List<Shape>()).Append(vsd);
            // Add to SD map
            _shapesSD[v] = vsd;
            // Add to Origins map
            if (!_origins.TryGetValue(vsd, out NCollection_List<Shape>? pLOr))
            {
                pLOr = _origins.SetValueRef(vsd, new NCollection_List<Shape>());
            }
            pLOr.Append(v);
        }
    }

    /// <summary>Fills the images of the edges: the edges of their (real) pave blocks.</summary>
    protected void FillImagesEdges(ProgressRange range)
    {
        int i, nbS = _ds!.SourceShapeCount;
        using var ps = new ProgressScope(range, "Filling splits of edges", nbS);
        for (i = 0; i < nbS; ++i, ps.Next())
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Edge)
            {
                continue;
            }
            //
            // Check if the pave blocks for the edge have been initialized
            if (!si.HasReference)
            {
                continue;
            }
            //
            Shape e = si.Shape();
            NCollection_List<PaveBlock> lpb = _ds.PaveBlocks(i);
            //
            // Fill the images of the edge from the list of its pave blocks.
            // The small edges, having no pave blocks, will have the empty list
            // of images and, thus, will be avoided in the result.
            NCollection_List<Shape> pLS = _images.SetValueRef(e, new NCollection_List<Shape>());
            //
            NCollection_List<PaveBlock>.Iterator itPB = new(lpb);
            for (; itPB.More(); itPB.Next())
            {
                PaveBlock pb = itPB.Value();
                PaveBlock pbr = _ds.RealPaveBlock(pb);
                //
                int nSpR = pbr.Edge();
                Shape spR = _ds.Shape(nSpR);
                pLS.Append(spR);
                //
                if (!_origins.TryGetValue(spR, out NCollection_List<Shape>? pLOr))
                {
                    pLOr = _origins.SetValueRef(spR, new NCollection_List<Shape>());
                }
                pLOr.Append(e);
                //
                if (_ds.IsCommonBlockOnEdge(pb))
                {
                    int nSp = pb.Edge();
                    Shape sp = _ds.Shape(nSp);
                    _shapesSD[sp] = spR;
                }
            }
            if (UserBreak(ps))
            {
                return;
            }
        }
    }

    /// <summary>Adds the images of the arguments of the given type (or the arguments themselves) into the result.</summary>
    protected virtual void BuildResult(ShapeType type)
    {
        // Fence map
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        // Iterate on all arguments of given type
        // and add their images into result
        NCollection_List<Shape>.Iterator itA = new(_arguments);
        for (; itA.More(); itA.Next())
        {
            Shape s = itA.Value();
            if (s.ShapeType() != type)
            {
                continue;
            }
            // Get images
            if (!_images.TryGetValue(s, out NCollection_List<Shape>? pLSIm))
            {
                // No images -> add the argument shape itself into result
                if (mFence.Add(s))
                {
                    new BrepBuilder().Add(_shape, s);
                }
            }
            else
            {
                // Add images of the argument shape into result
                NCollection_List<Shape>.Iterator itIm = new(pLSIm);
                for (; itIm.More(); itIm.Next())
                {
                    Shape sIm = itIm.Value();
                    if (mFence.Add(sIm))
                    {
                        new BrepBuilder().Add(_shape, sIm);
                    }
                }
            }
        }
    }

    /// <summary>Fills the images of the containers (wires, shells, compsolids) of the given type.</summary>
    protected void FillImagesContainers(ShapeType type, ProgressRange range)
    {
        int i, nbS;
        //
        nbS = _ds!.SourceShapeCount;
        using var ps = new ProgressScope(range, "Building splits of containers", 1);
        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() == type)
            {
                Shape c = si.Shape();
                FillImagesContainer(c, type);
            }
            if (UserBreak(ps))
            {
                return;
            }
        } // for (; aItS.More(); aItS.Next()) {
    }

    /// <summary>Fills the images of the compounds.</summary>
    protected void FillImagesCompounds(ProgressRange range)
    {
        int i, nbS;
        HashSet<Shape> mfp = new(ShapeHasher.Instance);
        //
        nbS = _ds!.SourceShapeCount;
        using var ps = new ProgressScope(range, "Building splits of compounds", nbS);
        for (i = 0; i < nbS; ++i, ps.Next())
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() == ShapeType.Compound)
            {
                Shape c = si.Shape();
                FillImagesCompound(c, mfp);
            }
            if (UserBreak(ps))
            {
                return;
            }
        } // for (; aItS.More(); aItS.Next()) {
    }

    /// <summary>Makes the image of the container from the splits of its sub-shapes, if any of them is split.</summary>
    protected void FillImagesContainer(in Shape s, ShapeType type)
    {
        // Check if any of the sub-shapes of the container have been modified
        Iterator it = new(s);
        for (; it.More(); it.Next())
        {
            Shape ss = it.Value();
            if (_images.TryGetValue(ss, out NCollection_List<Shape>? pLFIm)
                && ((pLFIm.Extent() != 1) || !pLFIm.First().IsSame(ss)))
            {
                break;
            }
        }

        if (!it.More())
        {
            // Non of the sub-shapes have been modified.
            // No need to create the new container.
            return;
        }

        BrepBuilder bb = new();
        // Make the new container of the splits of its sub-shapes
        Shape cIm = new();
        AlgoTools.MakeContainer(type, ref cIm);

        it.Initialize(s);
        for (; it.More(); it.Next())
        {
            Shape ss = it.Value();

            if (!_images.TryGetValue(ss, out NCollection_List<Shape>? pLSSIm))
            {
                // No splits, add the sub-shape itself
                bb.Add(cIm, ss);
                continue;
            }

            // Add the splits
            NCollection_List<Shape>.Iterator itIm = new(pLSSIm);
            for (; itIm.More(); itIm.Next())
            {
                Shape ssIm = itIm.Value();
                if (!ssIm.IsEqual(ss)
                    && AlgoTools.IsSplitToReverseWithWarn(ssIm, ss, _context, _report))
                {
                    ssIm.Reverse();
                }
                bb.Add(cIm, ssIm);
            }
        }

        cIm.Closed(Tool.IsClosed(cIm));
        _images.SetValueRef(s, new NCollection_List<Shape>()).Append(cIm);
    }

    /// <summary>Makes the image of the compound (and of its sub-compounds) from the images of its components.</summary>
    protected void FillImagesCompound(in Shape s, HashSet<Shape> mfp)
    {
        bool bInterferred;
        Orientation orX;
        Iterator it = new();
        BrepBuilder bb = new();
        NCollection_List<Shape>.Iterator itIm = new();
        //
        if (!mfp.Add(s))
        {
            return;
        }
        //
        bInterferred = false;
        it.Initialize(s);
        for (; it.More(); it.Next())
        {
            Shape aSx = it.Value();
            if (aSx.ShapeType() == ShapeType.Compound)
            {
                FillImagesCompound(aSx, mfp);
            }
            if (_images.ContainsKey(aSx))
            {
                bInterferred = true;
            }
        }
        if (!bInterferred)
        {
            return;
        }
        //
        Shape cIm = new();
        AlgoTools.MakeContainer(ShapeType.Compound, ref cIm);
        //
        it.Initialize(s);
        for (; it.More(); it.Next())
        {
            Shape sx = it.Value();
            orX = sx.Orientation();
            if (_images.ContainsKey(sx))
            {
                NCollection_List<Shape> lfIm = _images.GetValue(sx);
                itIm.Initialize(lfIm);
                for (; itIm.More(); itIm.Next())
                {
                    Shape sxIm = itIm.Value();
                    sxIm.Orientation(orX);
                    bb.Add(cIm, sxIm);
                }
            }
            else
            {
                bb.Add(cIm, sx);
            }
        }
        //
        NCollection_List<Shape> lsIm = new();
        lsIm.Append(cIm);
        _images[s] = lsIm;
    }
}
