// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeToHLR.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeToHLR.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Topology;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// Loads an outlined shape in a new HLRBRep_Data: one edge data per edge of the outlined shape (its vertices, their
/// outline and internal flags, its smoothness between its two faces, its range), and one face data per face with its
/// wires of edge indices and flags.
/// </summary>
/// <remarks>
/// OCCT's static class. The outlining writes into the caller's shape (its vertices' representations on the split
/// edges), so <see cref="Load"/> outlines it and reads the outlined shape in one working overlay (unit 10H6): the
/// data structure holds the caller's own sub-shapes, as OCCT's, and the reads see the outlining's writes.
/// </remarks>
public static class ShapeToHlr
{
    /// <summary>Outlines S for the projector P (nbIso isolines) and loads the result in a new data structure.</summary>
    public static HiddenLineData Load(OutLiner s, in AlgorithmProjector p,
                                    NCollection_DataMap<Shape, AdaptorTool> mst, int nbIso = 0)
    {
        // Kerf: the outlining and the reads of what it wrote, in one working overlay (unit 10H6); the outlined shape
        // then holds what stands for the caller's sub-shapes the outlining wrote into
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(s.OriginalShape());
        HiddenLineData ds = LoadCore(s, p, mst, nbIso);
        if (overlay is not null)
        {
            s.OutLinedShape(overlay.Finish(s.InternalOutLinedShape()));
        }
        return ds;
    }

    private static HiddenLineData LoadCore(OutLiner s, in AlgorithmProjector p,
                                           NCollection_DataMap<Shape, AdaptorTool> mst, int nbIso)
    {
        s.Fill(p, mst, nbIso);
        TopologyData topDS = s.DataStructure();

        NCollection_IndexedMap<Shape> fm = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> em = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> verticesToEdges =
            new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> edgesToFaces =
            new(ShapeHasher.Instance);

        Explorer exshell = new(), exface = new();
        Shape outLined = s.InternalOutLinedShape();

        for (exshell.Init(outLined, ShapeType.Shell); exshell.More(); exshell.Next())
        { // faces in a shell
            for (exface.Init(exshell.Current(), ShapeType.Face); exface.More(); exface.Next())
            {
                if (!fm.Contains(exface.Current()))
                {
                    fm.Add(exface.Current());
                }
            }
        }

        for (exface.Init(outLined, ShapeType.Face, ShapeType.Shell); exface.More(); exface.Next())
        { // faces not in a shell
            if (!fm.Contains(exface.Current()))
            {
                fm.Add(exface.Current());
            }
        }

        Exploration.MapShapes(outLined, ShapeType.Edge, em);

        int i;
        int nbEdge = em.Extent();

        for (i = 1; i <= nbEdge; i++)
        { // vertices back to edges
            Exploration.MapShapesAndAncestors(em.FindKey(i), ShapeType.Vertex, ShapeType.Edge, verticesToEdges);
        }

        int nbVert = verticesToEdges.Extent();
        int nbFace = fm.Extent();

        Vertex vf, vl;
        NCollection_List<Shape>.Iterator itn = new();
        int i1, i2;
        bool o1, o2;
        bool c1, c2;
        double pf, pl;
        float tf, tl;

        // Create the data structure
        HiddenLineData ds = new(nbVert, nbEdge, nbFace);
        int ed = 0;
        if (nbEdge != 0)
        {
            ed = 1;
        }

        for (i = 1; i <= nbFace; i++)
        { // test of Double edges
            Exploration.MapShapesAndAncestors(fm.FindKey(i), ShapeType.Edge, ShapeType.Face, edgesToFaces);
        }

        for (i = 1; i <= nbEdge; i++)
        { // load the Edges
            Edge edg = ShapeCasts.Edge(em.FindKey(i));
            Exploration.Vertices(edg, out vf, out vl);
            Tool.Range(edg, out pf, out pl);
            bool reg1 = false;
            bool regn = false;
            int inde = edgesToFaces.FindIndex(edg);
            if (inde > 0)
            {
                if (edgesToFaces[inde].Extent() == 2)
                {
                    itn.Initialize(edgesToFaces[inde]);
                    Face f1 = ShapeCasts.Face(itn.Value());
                    itn.Next();
                    Face f2 = ShapeCasts.Face(itn.Value());
                    Continuity rg = Tool.Continuity(edg, f1, f2);
                    reg1 = rg >= G1;
                    regn = rg >= G2;
                }
            }

            if (vf.IsNull)
            {
                i1 = 0;
                o1 = false;
                c1 = false;
                pf = RealFirst();
                tf = (float)Epsilon(pf);
            }
            else
            {
                i1 = verticesToEdges.FindIndex(vf);
                o1 = topDS.IsOutV(vf);
                c1 = topDS.IsIntV(vf);
                tf = (float)Tool.Tolerance(vf);
            }

            if (vl.IsNull)
            {
                i2 = 0;
                o2 = false;
                c2 = false;
                pl = RealLast();
                tl = (float)Epsilon(pl);
            }
            else
            {
                i2 = verticesToEdges.FindIndex(vl);
                o2 = topDS.IsOutV(vl);
                c2 = topDS.IsIntV(vl);
                tl = (float)Tool.Tolerance(vl);
            }

            ds.EDataArray()[ed].Set(reg1, regn, edg, i1, i2, o1, o2, c1, c2, pf, tf, pl, tl);
            ds.EdgeMap().Add(edg);
            ed++;
        }

        ExploreShape(s, ds, fm, em);
        return ds;
    }

    private static void ExploreFace(OutLiner s, HiddenLineData ds, NCollection_IndexedMap<Shape> fm,
                                    NCollection_IndexedMap<Shape> em, ref int i, in Face f, bool closed)
    {
        i++;
        Explorer ex1 = new(), ex2 = new();
        TopologyData topDS = s.DataStructure();
        Orientation orient = fm.FindKey(i).Orientation();
        Face theFace = ShapeCasts.Face(fm.FindKey(i));
        theFace.Orientation(Forward);
        HiddenLineFaceData fd = ds.FDataArray()[i];

        int nw = 0;

        for (ex1.Init(theFace, ShapeType.Wire); ex1.More(); ex1.Next())
        {
            nw++;
        }

        fd.Set(theFace, orient, closed, nw);
        nw = 0;

        for (ex1.Init(theFace, ShapeType.Wire); ex1.More(); ex1.Next())
        {
            nw++;
            int ne = 0;

            for (ex2.Init(ex1.Current(), ShapeType.Edge); ex2.More(); ex2.Next())
            {
                Edge edge = ShapeCasts.Edge(ex2.Current());
                if (!Tool.Degenerated(edge))
                {
                    ne++;
                }
            }

            fd.SetWire(nw, ne);
            ne = 0;

            for (ex2.Init(ex1.Current(), ShapeType.Edge); ex2.More(); ex2.Next())
            {
                Edge e = ShapeCasts.Edge(ex2.Current());
                if (Tool.Degenerated(e))
                {
                    continue;
                }
                ne++;
                int ie = em.FindIndex(e);
                Orientation orientE = e.Orientation();
                bool Int = topDS.IsIntLFaceEdge(f, e);
                bool iso = topDS.IsIsoLFaceEdge(f, e);
                bool Out = topDS.IsOutLFaceEdge(f, e);
                bool dbl = BrepTools.IsReallyClosed(ShapeCasts.Edge(e), theFace);
                fd.SetWEdge(nw, ne, ie, orientE, Out, Int, dbl, iso);
            }
        }

        ds.FaceMap().Add(theFace);
    }

    private static void ExploreShape(OutLiner s, HiddenLineData ds, NCollection_IndexedMap<Shape> fm,
                                     NCollection_IndexedMap<Shape> em)
    {
        HashSet<Shape> shapeMap = new(ShapeHasher.Instance);
        Explorer exshell = new(), exface = new(), exedge = new();
        int i = 0;
        Shape original = s.OriginalShape();

        for (exshell.Init(original, ShapeType.Shell); exshell.More(); exshell.Next())
        { // faces in a shell (open or close)

            bool closed = exshell.Current().Closed();

            if (!closed)
            {
                int ie;
                int nbEdge = em.Extent();
                int[] flag = new int[nbEdge + 1];

                for (ie = 1; ie <= nbEdge; ie++)
                {
                    flag[ie] = 0;
                }

                for (exedge.Init(exshell.Current(), ShapeType.Edge); exedge.More(); exedge.Next())
                {
                    Edge e = ShapeCasts.Edge(exedge.Current());
                    ie = em.FindIndex(e);
                    Orientation orient = e.Orientation();
                    if (!Tool.Degenerated(e))
                    {
                        if (orient == Forward)
                        {
                            flag[ie] += 1;
                        }
                        else if (orient == Reversed)
                        {
                            flag[ie] -= 1;
                        }
                    }
                }
                closed = true;

                for (ie = 1; ie <= nbEdge && closed; ie++)
                {
                    closed = (flag[ie] == 0);
                }
            }

            for (exface.Init(exshell.Current(), ShapeType.Face); exface.More(); exface.Next())
            {
                if (shapeMap.Add(exface.Current()))
                {
                    ExploreFace(s, ds, fm, em, ref i, ShapeCasts.Face(exface.Current()), closed);
                }
            }
        }

        for (exface.Init(original, ShapeType.Face, ShapeType.Shell); exface.More(); exface.Next())
        { // faces not in a shell
            if (shapeMap.Add(exface.Current()))
            {
                ExploreFace(s, ds, fm, em, ref i, ShapeCasts.Face(exface.Current()), false);
            }
        }
    }
}
