// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeToHLR.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeToHLR.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Loads an outlined shape in a new HLRBRep_Data: one edge data per edge of the outlined shape (its vertices, their
/// outline and internal flags, its smoothness between its two faces, its range), and one face data per face with its
/// wires of edge indices and flags.
/// </summary>
/// <remarks>
/// OCCT's static class. OCCT's <c>ExploreShape</c> walks the outliner's original shape; the port walks its working
/// shape, which is the original or its copy at the entry (decision 1095), so the faces it finds are the ones the
/// outlining recorded.
/// </remarks>
public static class HLRBRep_ShapeToHLR
{
    /// <summary>Outlines S for the projector P (nbIso isolines) and loads the result in a new data structure.</summary>
    public static HLRBRep_Data Load(HLRTopoBRep_OutLiner S, in HLRAlgo_Projector P,
                                    NCollection_DataMap<Shape, BRepTopAdaptor_Tool> MST, int nbIso = 0)
    {
        S.Fill(P, MST, nbIso);
        HLRTopoBRep_Data TopDS = S.DataStructure();

        NCollection_IndexedMap<Shape> FM = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> EM = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VerticesToEdges =
            new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> EdgesToFaces =
            new(ShapeHasher.Instance);

        Explorer exshell = new(), exface = new();
        Shape anOutLined = S.InternalOutLinedShape();

        for (exshell.Init(anOutLined, ShapeType.Shell); exshell.More(); exshell.Next())
        { // faces in a shell
            for (exface.Init(exshell.Current(), ShapeType.Face); exface.More(); exface.Next())
            {
                if (!FM.Contains(exface.Current()))
                {
                    FM.Add(exface.Current());
                }
            }
        }

        for (exface.Init(anOutLined, ShapeType.Face, ShapeType.Shell); exface.More(); exface.Next())
        { // faces not in a shell
            if (!FM.Contains(exface.Current()))
            {
                FM.Add(exface.Current());
            }
        }

        Exploration.MapShapes(anOutLined, ShapeType.Edge, EM);

        int i;
        int nbEdge = EM.Extent();

        for (i = 1; i <= nbEdge; i++)
        { // vertices back to edges
            Exploration.MapShapesAndAncestors(EM.FindKey(i), ShapeType.Vertex, ShapeType.Edge, VerticesToEdges);
        }

        int nbVert = VerticesToEdges.Extent();
        int nbFace = FM.Extent();

        Vertex VF, VL;
        NCollection_List<Shape>.Iterator itn = new();
        int i1, i2;
        bool o1, o2;
        bool c1, c2;
        double pf, pl;
        float tf, tl;

        // Create the data structure
        HLRBRep_Data DS = new(nbVert, nbEdge, nbFace);
        int ed = 0;
        if (nbEdge != 0)
        {
            ed = 1;
        }

        for (i = 1; i <= nbFace; i++)
        { // test of Double edges
            Exploration.MapShapesAndAncestors(FM.FindKey(i), ShapeType.Edge, ShapeType.Face, EdgesToFaces);
        }

        for (i = 1; i <= nbEdge; i++)
        { // load the Edges
            Edge Edg = ShapeCasts.Edge(EM.FindKey(i));
            Exploration.Vertices(Edg, out VF, out VL);
            Tool.Range(Edg, out pf, out pl);
            bool reg1 = false;
            bool regn = false;
            int inde = EdgesToFaces.FindIndex(Edg);
            if (inde > 0)
            {
                if (EdgesToFaces[inde].Extent() == 2)
                {
                    itn.Initialize(EdgesToFaces[inde]);
                    Face F1 = ShapeCasts.Face(itn.Value());
                    itn.Next();
                    Face F2 = ShapeCasts.Face(itn.Value());
                    Continuity rg = Tool.Continuity(Edg, F1, F2);
                    reg1 = rg >= G1;
                    regn = rg >= G2;
                }
            }

            if (VF.IsNull)
            {
                i1 = 0;
                o1 = false;
                c1 = false;
                pf = RealFirst();
                tf = (float)Epsilon(pf);
            }
            else
            {
                i1 = VerticesToEdges.FindIndex(VF);
                o1 = TopDS.IsOutV(VF);
                c1 = TopDS.IsIntV(VF);
                tf = (float)Tool.Tolerance(VF);
            }

            if (VL.IsNull)
            {
                i2 = 0;
                o2 = false;
                c2 = false;
                pl = RealLast();
                tl = (float)Epsilon(pl);
            }
            else
            {
                i2 = VerticesToEdges.FindIndex(VL);
                o2 = TopDS.IsOutV(VL);
                c2 = TopDS.IsIntV(VL);
                tl = (float)Tool.Tolerance(VL);
            }

            DS.EDataArray()[ed].Set(reg1, regn, Edg, i1, i2, o1, o2, c1, c2, pf, tf, pl, tl);
            DS.EdgeMap().Add(Edg);
            ed++;
        }

        ExploreShape(S, DS, FM, EM);
        return DS;
    }

    private static void ExploreFace(HLRTopoBRep_OutLiner S, HLRBRep_Data DS, NCollection_IndexedMap<Shape> FM,
                                    NCollection_IndexedMap<Shape> EM, ref int i, in Face F, bool closed)
    {
        i++;
        Explorer Ex1 = new(), Ex2 = new();
        HLRTopoBRep_Data TopDS = S.DataStructure();
        Orientation orient = FM.FindKey(i).Orientation();
        Face theFace = ShapeCasts.Face(FM.FindKey(i));
        theFace.Orientation(Forward);
        HLRBRep_FaceData fd = DS.FDataArray()[i];

        int nw = 0;

        for (Ex1.Init(theFace, ShapeType.Wire); Ex1.More(); Ex1.Next())
        {
            nw++;
        }

        fd.Set(theFace, orient, closed, nw);
        nw = 0;

        for (Ex1.Init(theFace, ShapeType.Wire); Ex1.More(); Ex1.Next())
        {
            nw++;
            int ne = 0;

            for (Ex2.Init(Ex1.Current(), ShapeType.Edge); Ex2.More(); Ex2.Next())
            {
                Edge anEdge = ShapeCasts.Edge(Ex2.Current());
                if (!Tool.Degenerated(anEdge))
                {
                    ne++;
                }
            }

            fd.SetWire(nw, ne);
            ne = 0;

            for (Ex2.Init(Ex1.Current(), ShapeType.Edge); Ex2.More(); Ex2.Next())
            {
                Edge E = ShapeCasts.Edge(Ex2.Current());
                if (Tool.Degenerated(E))
                {
                    continue;
                }
                ne++;
                int ie = EM.FindIndex(E);
                Orientation anOrientE = E.Orientation();
                bool Int = TopDS.IsIntLFaceEdge(F, E);
                bool Iso = TopDS.IsIsoLFaceEdge(F, E);
                bool Out = TopDS.IsOutLFaceEdge(F, E);
                bool Dbl = BrepTools.IsReallyClosed(ShapeCasts.Edge(E), theFace);
                fd.SetWEdge(nw, ne, ie, anOrientE, Out, Int, Dbl, Iso);
            }
        }

        DS.FaceMap().Add(theFace);
    }

    private static void ExploreShape(HLRTopoBRep_OutLiner S, HLRBRep_Data DS, NCollection_IndexedMap<Shape> FM,
                                     NCollection_IndexedMap<Shape> EM)
    {
        NCollection_Map<Shape> ShapeMap = new(ShapeHasher.Instance);
        Explorer exshell = new(), exface = new(), exedge = new();
        int i = 0;
        Shape anOriginal = S.InternalWorkingShape(); // decision 1095

        for (exshell.Init(anOriginal, ShapeType.Shell); exshell.More(); exshell.Next())
        { // faces in a shell (open or close)

            bool closed = exshell.Current().Closed();

            if (!closed)
            {
                int ie;
                int nbEdge = EM.Extent();
                int[] flag = new int[nbEdge + 1];

                for (ie = 1; ie <= nbEdge; ie++)
                {
                    flag[ie] = 0;
                }

                for (exedge.Init(exshell.Current(), ShapeType.Edge); exedge.More(); exedge.Next())
                {
                    Edge E = ShapeCasts.Edge(exedge.Current());
                    ie = EM.FindIndex(E);
                    Orientation orient = E.Orientation();
                    if (!Tool.Degenerated(E))
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
                if (ShapeMap.Add(exface.Current()))
                {
                    ExploreFace(S, DS, FM, EM, ref i, ShapeCasts.Face(exface.Current()), closed);
                }
            }
        }

        for (exface.Init(anOriginal, ShapeType.Face, ShapeType.Shell); exface.More(); exface.Next())
        { // faces not in a shell
            if (ShapeMap.Add(exface.Current()))
            {
                ExploreFace(S, DS, FM, EM, ref i, ShapeCasts.Face(exface.Current()), false);
            }
        }
    }
}
