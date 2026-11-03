// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

public partial class BooleanBuilder
{
    /// <summary>
    /// The shapes generated from <paramref name="s"/> and kept in the result: an edge generates the vertices of its
    /// edge/edge and edge/face intersections, a face those of its edge/face intersections and its section edges and
    /// vertices.
    /// </summary>
    protected virtual NCollection_List<Shape> LocGenerated(in Shape s)
    {
        // The rules for Generated shapes are these:
        // 1. The EDGE may be generated from the FACES as an intersection edge;
        // 2. The VERTEX may be generated from the EDGES and FACES as an intersection vertex.
        //
        // The list of generated elements will contain only those which are contained
        // in the result of the operation.

        _histShapes.Clear();

        if (s.IsNull)
        {
            return _histShapes;
        }

        // Only EDGES and FACES should be considered
        ShapeType type = s.ShapeType();
        if (type != ShapeType.Edge && type != ShapeType.Face)
        {
            // Wrong type
            return _histShapes;
        }

        // Check that DS contains the shape, i.e. it is from the arguments of the operation
        int nS = _ds!.Index(s);
        if (nS < 0)
        {
            // Unknown shape
            return _histShapes;
        }

        // Check that the shape has participated in any intersections
        ShapeInfo si = _ds.ShapeInfo(nS);
        if (!si.HasReference)
        {
            // Untouched shape
            return _histShapes;
        }

        // Analyze all types of Interferences which can produce
        // new vertices - Edge/Edge and Edge/Face
        List<InterfEE> eEs = _ds.InterfEE();
        List<InterfEF> eFs = _ds.InterfEF();

        // Fence map to avoid duplicates in the list of Generated;
        NCollection_IntMap mFence = new();

        // Analyze each interference and find those in which the given shape has participated

        // No need to analyze Edge/Edge interferences for the shapes of type FACE
        bool isFace = type == ShapeType.Face;

        for (int k = isFace ? 1 : 0; k < 2; ++k)
        {
            int nbLines = k == 0 ? eEs.Count : eFs.Count;
            for (int i = 0; i < nbLines; ++i)
            {
                Interf aInt = k == 0 ? eEs[i] : eFs[i];
                if (!aInt.HasIndexNew())
                {
                    // No new vertices created
                    continue;
                }

                if (!aInt.Contains(nS))
                {
                    continue;
                }

                int nVNew = aInt.IndexNew();
                _ds.HasShapeSD(nVNew, ref nVNew);
                if (!mFence.Add(nVNew))
                {
                    continue;
                }

                // Get the new vertex
                Shape vNew = _ds.Shape(nVNew);

                // Check that the result shape contains vertex
                if (_mapShape.Contains(vNew))
                {
                    // Save the vertex as generated
                    _histShapes.Append(vNew);
                }
            }
        }

        if (!isFace)
        {
            return _histShapes;
        }

        // For the FACE it is also necessary to collect all
        // section elements created in FACE/FACE interferences.
        // This information is available in the FaceInfo structure.
        FaceInfo fi = _ds.FaceInfo(nS);

        // Section edges of the face
        IndexedSet<PaveBlock> mpbSc = fi.PaveBlocksSc();
        // Save section edges contained in the result shape
        int nb = mpbSc.Count;
        for (int i = 0; i < nb; ++i)
        {
            Shape eNew = _ds.Shape(mpbSc[i].Edge());
            if (_mapShape.Contains(eNew))
            {
                _histShapes.Append(eNew);
            }
        }

        // Section vertices of the face
        NCollection_IntMap mvSc = fi.VerticesSc();
        // Save section vertices contained in the result shape (OCCT's order, decision 784)
        foreach (int itM in mvSc)
        {
            Shape vNew = _ds.Shape(itM);
            if (_mapShape.Contains(vNew))
            {
                _histShapes.Append(vNew);
            }
        }

        return _histShapes;
    }

    /// <summary>The splits of <paramref name="s"/> (null if it has no images).</summary>
    protected virtual NCollection_List<Shape>? LocModified(in Shape s)
        => _images.TryGetValue(s, out NCollection_List<Shape>? list) ? list : null;

    /// <summary>OCCT's history of the result: modified, generated and deleted argument sub-shapes.</summary>
    protected void PrepareHistory(ProgressRange range)
    {
        if (!HasHistory)
        {
            return;
        }

        // Initializing history tool
        _history = new History();

        // Map the result shape
        _mapShape.Clear();
        Exploration.MapShapes(_shape, _mapShape);

        // Among all input shapes find:
        // - Shapes that have been modified (split). Add the splits kept in the result
        //   shape as Modified from the shape;
        // - Shapes that have created new geometries (i.e. generated new shapes). Add
        //   the generated elements kept in the result shape as Generated from the shape;
        // - Shapes that have no trace in the result shape. Add them as Deleted
        //   during the operation.
        int nbS = _ds!.SourceShapeCount;
        using var ps = new ProgressScope(range, "Preparing history information", nbS);
        for (int i = 0; i < nbS; ++i, ps.Next())
        {
            Shape s = _ds.Shape(i);

            // Check if History information is available for this kind of shape.
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(s))
            {
                continue;
            }

            if (UserBreak(ps))
            {
                return;
            }

            bool isModified = false;

            // Check if the shape has any splits
            NCollection_List<Shape>? pLSp = LocModified(s);
            if (pLSp is not null)
            {
                // Find all splits of the shape which are kept in the result
                NCollection_List<Shape>.Iterator it0 = new(pLSp);
                for (; it0.More(); it0.Next())
                {
                    Shape sp = it0.Value();
                    // Check if the result shape contains the split
                    if (_mapShape.Contains(sp))
                    {
                        // Add modified shape with proper orientation
                        ShapeType type = sp.ShapeType();
                        if (type == ShapeType.Vertex || type == ShapeType.Solid)
                        {
                            sp.Orientation(s.Orientation());
                        }
                        else if (AlgoTools.IsSplitToReverse(sp, s, _context))
                        {
                            sp.Reverse();
                        }

                        _history.AddModified(s, sp);
                        isModified = true;
                    }
                }
            }

            // Check if the shape has Generated elements
            NCollection_List<Shape> genShapes = LocGenerated(s);
            NCollection_List<Shape>.Iterator it = new(genShapes);
            for (; it.More(); it.Next())
            {
                Shape g = it.Value();
                if (_mapShape.Contains(g))
                {
                    _history.AddGenerated(s, g);
                }
            }

            // Check if the shape has been deleted, i.e. it is not contained in the result
            // and has no Modified shapes.
            if (!isModified && !_mapShape.Contains(s))
            {
                _history.Remove(s);
            }
        }
    }
}
