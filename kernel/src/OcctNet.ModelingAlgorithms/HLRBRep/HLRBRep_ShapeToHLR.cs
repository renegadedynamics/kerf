// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeToHLR.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ShapeToHLR.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Loads an outlined shape in a new HLRBRep_Data: one edge data per edge of the outlined shape (its vertices, their
/// outline and internal flags, its smoothness between its two faces, its range), and one face data per face with its
/// wires of edge indices and flags.
/// </summary>
/// <remarks>
/// OCCT's static class. OCCT's <c>ExploreShape</c> walks the outliner's original shape; the port walks its working
/// shape, which is the original or its copy at the entry (decision 10605), so the faces it finds are the ones the
/// outlining recorded.
/// </remarks>
public static class HLRBRep_ShapeToHLR
{
    /// <summary>Outlines S for the projector P (nbIso isolines) and loads the result in a new data structure.</summary>
    public static HLRBRep_Data Load(HLRTopoBRep_OutLiner S, in HLRAlgo_Projector P,
                                    NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> MST, int nbIso = 0)
    {
        S.Fill(P, MST, nbIso);
        HLRTopoBRep_Data TopDS = S.DataStructure();

        NCollection_IndexedMap<TopoDS_Shape> FM = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> EM = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VerticesToEdges =
            new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> EdgesToFaces =
            new(TopTools_ShapeMapHasher.Instance);

        TopExp_Explorer exshell = new(), exface = new();
        TopoDS_Shape anOutLined = S.InternalOutLinedShape();

        for (exshell.Init(anOutLined, TopAbs_SHELL); exshell.More(); exshell.Next())
        { // faces in a shell
            for (exface.Init(exshell.Current(), TopAbs_FACE); exface.More(); exface.Next())
            {
                if (!FM.Contains(exface.Current()))
                {
                    FM.Add(exface.Current());
                }
            }
        }

        for (exface.Init(anOutLined, TopAbs_FACE, TopAbs_SHELL); exface.More(); exface.Next())
        { // faces not in a shell
            if (!FM.Contains(exface.Current()))
            {
                FM.Add(exface.Current());
            }
        }

        TopExp.MapShapes(anOutLined, TopAbs_EDGE, EM);

        int i;
        int nbEdge = EM.Extent();

        for (i = 1; i <= nbEdge; i++)
        { // vertices back to edges
            TopExp.MapShapesAndAncestors(EM.FindKey(i), TopAbs_VERTEX, TopAbs_EDGE, VerticesToEdges);
        }

        int nbVert = VerticesToEdges.Extent();
        int nbFace = FM.Extent();

        TopoDS_Vertex VF, VL;
        NCollection_List<TopoDS_Shape>.Iterator itn = new();
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
            TopExp.MapShapesAndAncestors(FM.FindKey(i), TopAbs_EDGE, TopAbs_FACE, EdgesToFaces);
        }

        for (i = 1; i <= nbEdge; i++)
        { // load the Edges
            TopoDS_Edge Edg = TopoDS.Edge(EM.FindKey(i));
            TopExp.Vertices(Edg, out VF, out VL);
            BRep_Tool.Range(Edg, out pf, out pl);
            bool reg1 = false;
            bool regn = false;
            int inde = EdgesToFaces.FindIndex(Edg);
            if (inde > 0)
            {
                if (EdgesToFaces[inde].Extent() == 2)
                {
                    itn.Initialize(EdgesToFaces[inde]);
                    TopoDS_Face F1 = TopoDS.Face(itn.Value());
                    itn.Next();
                    TopoDS_Face F2 = TopoDS.Face(itn.Value());
                    GeomAbs_Shape rg = BRep_Tool.Continuity(Edg, F1, F2);
                    reg1 = rg >= GeomAbs_G1;
                    regn = rg >= GeomAbs_G2;
                }
            }

            if (VF.IsNull())
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
                tf = (float)BRep_Tool.Tolerance(VF);
            }

            if (VL.IsNull())
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
                tl = (float)BRep_Tool.Tolerance(VL);
            }

            DS.EDataArray()[ed].Set(reg1, regn, Edg, i1, i2, o1, o2, c1, c2, pf, tf, pl, tl);
            DS.EdgeMap().Add(Edg);
            ed++;
        }

        ExploreShape(S, DS, FM, EM);
        return DS;
    }

    private static void ExploreFace(HLRTopoBRep_OutLiner S, HLRBRep_Data DS, NCollection_IndexedMap<TopoDS_Shape> FM,
                                    NCollection_IndexedMap<TopoDS_Shape> EM, ref int i, in TopoDS_Face F, bool closed)
    {
        i++;
        TopExp_Explorer Ex1 = new(), Ex2 = new();
        HLRTopoBRep_Data TopDS = S.DataStructure();
        TopAbs_Orientation orient = FM.FindKey(i).Orientation();
        TopoDS_Face theFace = TopoDS.Face(FM.FindKey(i));
        theFace.Orientation(TopAbs_FORWARD);
        HLRBRep_FaceData fd = DS.FDataArray()[i];

        int nw = 0;

        for (Ex1.Init(theFace, TopAbs_WIRE); Ex1.More(); Ex1.Next())
        {
            nw++;
        }

        fd.Set(theFace, orient, closed, nw);
        nw = 0;

        for (Ex1.Init(theFace, TopAbs_WIRE); Ex1.More(); Ex1.Next())
        {
            nw++;
            int ne = 0;

            for (Ex2.Init(Ex1.Current(), TopAbs_EDGE); Ex2.More(); Ex2.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(Ex2.Current());
                if (!BRep_Tool.Degenerated(anEdge))
                {
                    ne++;
                }
            }

            fd.SetWire(nw, ne);
            ne = 0;

            for (Ex2.Init(Ex1.Current(), TopAbs_EDGE); Ex2.More(); Ex2.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(Ex2.Current());
                if (BRep_Tool.Degenerated(E))
                {
                    continue;
                }
                ne++;
                int ie = EM.FindIndex(E);
                TopAbs_Orientation anOrientE = E.Orientation();
                bool Int = TopDS.IsIntLFaceEdge(F, E);
                bool Iso = TopDS.IsIsoLFaceEdge(F, E);
                bool Out = TopDS.IsOutLFaceEdge(F, E);
                bool Dbl = BRepTools.IsReallyClosed(TopoDS.Edge(E), theFace);
                fd.SetWEdge(nw, ne, ie, anOrientE, Out, Int, Dbl, Iso);
            }
        }

        DS.FaceMap().Add(theFace);
    }

    private static void ExploreShape(HLRTopoBRep_OutLiner S, HLRBRep_Data DS, NCollection_IndexedMap<TopoDS_Shape> FM,
                                     NCollection_IndexedMap<TopoDS_Shape> EM)
    {
        NCollection_Map<TopoDS_Shape> ShapeMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exshell = new(), exface = new(), exedge = new();
        int i = 0;
        TopoDS_Shape anOriginal = S.InternalWorkingShape(); // decision 10605

        for (exshell.Init(anOriginal, TopAbs_SHELL); exshell.More(); exshell.Next())
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

                for (exedge.Init(exshell.Current(), TopAbs_EDGE); exedge.More(); exedge.Next())
                {
                    TopoDS_Edge E = TopoDS.Edge(exedge.Current());
                    ie = EM.FindIndex(E);
                    TopAbs_Orientation orient = E.Orientation();
                    if (!BRep_Tool.Degenerated(E))
                    {
                        if (orient == TopAbs_FORWARD)
                        {
                            flag[ie] += 1;
                        }
                        else if (orient == TopAbs_REVERSED)
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

            for (exface.Init(exshell.Current(), TopAbs_FACE); exface.More(); exface.Next())
            {
                if (ShapeMap.Add(exface.Current()))
                {
                    ExploreFace(S, DS, FM, EM, ref i, TopoDS.Face(exface.Current()), closed);
                }
            }
        }

        for (exface.Init(anOriginal, TopAbs_FACE, TopAbs_SHELL); exface.More(); exface.Next())
        { // faces not in a shell
            if (ShapeMap.Add(exface.Current()))
            {
                ExploreFace(S, DS, FM, EM, ref i, TopoDS.Face(exface.Current()), false);
            }
        }
    }
}
