// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_cherche_face1 to ChFi3d_IsPseudoSeam)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace Occt;

internal static partial class ChFi3d_Builder_0
{
    //=======================================================================
    // function : cherche_face1
    // purpose  : find face F different from F1 in the map.
    // The map contains two faces adjacent to an edge
    //=======================================================================

    /// <summary>The face of <paramref name="map"/> other than <paramref name="F1"/>.</summary>
    /// <exception cref="ConstructionErrorException">None is found and <paramref name="F"/> is null.</exception>
    internal static void ChFi3d_cherche_face1(NCollection_List<Shape> map, in Face F1,
                                              ref Face F)
    {
        Face Fcur;
        bool trouve = false;
        foreach (Shape It in map)
        {
            if (trouve)
            {
                break;
            }
            Fcur = ShapeCasts.Face(It);
            if (!Fcur.IsSame(F1))
            {
                F = Fcur;
                trouve = true;
            }
        }
        if (F.IsNull)
        {
            throw new ConstructionErrorException("Failed to find face");
        }
    }

    //=======================================================================
    // function : cherche_element
    // purpose  : find edge E of F1 other than E1 and containing vertex V
    // Vtx is the other vertex of E
    //=======================================================================

    /// <summary>The edge <paramref name="E"/> of <paramref name="F1"/>, other than <paramref name="E1"/>, on
    /// <paramref name="V"/>; <paramref name="Vtx"/> its other vertex.</summary>
    /// <exception cref="ConstructionErrorException">None is found and <paramref name="E"/> is null.</exception>
    internal static void ChFi3d_cherche_element(in Vertex V, in Edge E1, in Face F1,
                                                ref Edge E, ref Vertex Vtx)
    {
        int ie;
        Vertex V1, V2;
        bool trouve = false;
        Edge Ecur;
        NCollection_IndexedMap<Shape> MapE = new(ShapeHasher.Instance);
        Exploration.MapShapes(F1, ShapeType.Edge, MapE);
        for (ie = 1; ie <= MapE.Extent() && !trouve; ie++)
        {
            Ecur = ShapeCasts.Edge(MapE.FindKey(ie));
            if (!Ecur.IsSame(E1))
            {
                NCollection_IndexedMap<Shape> MapV = new(ShapeHasher.Instance);
                Exploration.MapShapes(Ecur, ShapeType.Vertex, MapV);
                if (MapV.Extent() == 2)
                {
                    V1 = ShapeCasts.Vertex(MapV.FindKey(1));
                    V2 = ShapeCasts.Vertex(MapV.FindKey(2));
                    if (V1.IsSame(V))
                    {
                        Vtx = V2;
                        E = Ecur;
                        trouve = true;
                    }
                    else if (V2.IsSame(V))
                    {
                        Vtx = V1;
                        E = Ecur;
                        trouve = true;
                    }
                }
            }
        }
        if (E.IsNull)
        {
            throw new ConstructionErrorException("Failed to find element");
        }
    }

    //=======================================================================
    // function : cherche_edge
    // purpose  : find edge E of F1 other than the list of edges E1 and
    //           containing vertex V  Vtx is the other vertex of E.
    //=======================================================================

    /// <summary>The edge <paramref name="E"/> of <paramref name="F1"/>, none of <paramref name="E1"/>, on
    /// <paramref name="V"/>; <paramref name="Vtx"/> its other vertex.</summary>
    /// <exception cref="ConstructionErrorException">None is found and <paramref name="E"/> is null.</exception>
    internal static void ChFi3d_cherche_edge(in Vertex V, NCollection_Array1<Shape> E1,
                                             in Face F1, ref Edge E, ref Vertex Vtx)
    {
        int ie, i;
        Vertex V1, V2;
        bool trouve = false;
        Edge Ecur;
        bool same;
        NCollection_IndexedMap<Shape> MapE = new(ShapeHasher.Instance);
        Exploration.MapShapes(F1, ShapeType.Edge, MapE);
        for (ie = 1; ie <= MapE.Extent() && !trouve; ie++)
        {
            Ecur = ShapeCasts.Edge(MapE.FindKey(ie));
            same = false;
            for (i = E1.Lower(); i <= E1.Upper(); i++)
            {
                if (Ecur.IsSame(E1.Value(i)))
                {
                    same = true;
                }
            }
            if (!same)
            {
                NCollection_IndexedMap<Shape> MapV = new(ShapeHasher.Instance);
                Exploration.MapShapes(Ecur, ShapeType.Vertex, MapV);
                if (MapV.Extent() == 2)
                {
                    V1 = ShapeCasts.Vertex(MapV.FindKey(1));
                    V2 = ShapeCasts.Vertex(MapV.FindKey(2));
                    if (V1.IsSame(V))
                    {
                        Vtx = V2;
                        E = Ecur;
                        trouve = true;
                    }
                    else if (V2.IsSame(V))
                    {
                        Vtx = V1;
                        E = Ecur;
                        trouve = true;
                    }
                }
            }
        }
        if (E.IsNull)
        {
            throw new ConstructionErrorException("Failed to find edge");
        }
    }

    //=======================================================================
    // function : nbface
    // purpose  : calculates the number of faces common to a vertex
    //
    //=======================================================================

    /// <summary>The number of distinct faces of <paramref name="mapVF"/>.</summary>
    internal static int ChFi3d_nbface(NCollection_List<Shape> mapVF)
    {
        int nface = 0;
        int fj = 0;
        foreach (Shape cur in mapVF)
        {
            fj++;
            int kf = 1;
            foreach (Shape JtF in mapVF)
            {
                if (!(kf < fj))
                {
                    break;
                }
                if (cur.IsSame(JtF))
                {
                    break;
                }
                kf++;
            }
            if (kf == fj)
            {
                nface++;
            }
        }
        return nface;
    }

    //=======================================================================
    // function : edge_common_faces
    // purpose  :  determines two faces sharing an edge.
    //            F1 = F2 if there is an edge to parce
    //=======================================================================

    /// <summary>The two faces of an edge (the same one twice for a seam).</summary>
    internal static void ChFi3d_edge_common_faces(NCollection_List<Shape> mapEF, out Face F1,
                                                  out Face F2)
    {
        Face F;
        bool trouve;
        F1 = ShapeCasts.Face(mapEF.First());
        F2 = new Face();
        trouve = false;
        foreach (Shape It in mapEF)
        {
            if (trouve)
            {
                break;
            }
            F = ShapeCasts.Face(It);
            if (!F.IsSame(F1))
            {
                F2 = F;
                trouve = true;
            }
        }
        if (!trouve)
        {
            F2 = F1;
        }
    }

    /***********************************************************/
    // gives the angle between edges E1 and E2.
    // Vtx is the vertex common to the edges
    /************************************************************/

    /// <summary>The angle between the edges <paramref name="E1"/> and <paramref name="E2"/> leaving their common
    /// vertex <paramref name="Vtx"/>.</summary>
    internal static double ChFi3d_AngleEdge(in Vertex Vtx, in Edge E1, in Edge E2)
    {
        double angle;
        EdgeCurveAdaptor BCurv1 = new(E1);
        EdgeCurveAdaptor BCurv2 = new(E2);
        double parE1, parE2;
        parE1 = Tool.Parameter(Vtx, E1);
        parE2 = Tool.Parameter(Vtx, E2);
        BCurv1.D1(parE1, out Point3D P1, out Vector3D dir1);
        BCurv2.D1(parE2, out Point3D P2, out Vector3D dir2);
        if (!Vtx.IsSame(Exploration.FirstVertex(E1)))
        {
            dir1.Reverse();
        }
        if (!Vtx.IsSame(Exploration.FirstVertex(E2)))
        {
            dir2.Reverse();
        }
        angle = Math.Abs(dir1.Angle(dir2));
        return angle;
    }

    //==================================================================
    // ChercheBordsLibres
    // determines if vertex V1 has edges on free borders
    // edgelibre1 and edgelibre2.
    // It is supposed that a top can have only 2 edges on free borders
    //===================================================================

    /// <summary>Whether <paramref name="V1"/> has two edges on free borders, and these edges.</summary>
    internal static void ChFi3d_ChercheBordsLibres(ChFiDS_Map myVEMap, in Vertex V1, out bool bordlibre,
                                                   ref Edge edgelibre1, ref Edge edgelibre2)
    {
        bordlibre = false;
        int nboccur;
        foreach (Shape ItE in myVEMap[V1])
        {
            if (bordlibre)
            {
                break;
            }
            nboccur = 0;
            Edge cur = ShapeCasts.Edge(ItE);
            if (!Tool.Degenerated(cur))
            {
                foreach (Shape ItE1 in myVEMap[V1])
                {
                    Edge cur1 = ShapeCasts.Edge(ItE1);
                    if (cur1.IsSame(cur))
                    {
                        nboccur++;
                    }
                }
            }
            if (nboccur == 1)
            {
                edgelibre1 = cur;
                bordlibre = true;
            }
        }
        if (bordlibre)
        {
            bordlibre = false;
            foreach (Shape ItE in myVEMap[V1])
            {
                if (bordlibre)
                {
                    break;
                }
                nboccur = 0;
                Edge cur = ShapeCasts.Edge(ItE);
                if (!Tool.Degenerated(cur) && !cur.IsSame(edgelibre1))
                {
                    foreach (Shape ItE1 in myVEMap[V1])
                    {
                        Edge cur1 = ShapeCasts.Edge(ItE1);
                        if (cur1.IsSame(cur))
                        {
                            nboccur++;
                        }
                    }
                }
                if (nboccur == 1)
                {
                    edgelibre2 = cur;
                    bordlibre = true;
                }
            }
        }
    }

    //=======================================================================
    // function : NbNotDegeneratedEdges
    // purpose  : calculate the number of non-degenerated edges of Map VEMap(Vtx)
    // Attention the edges of junctions are taken into account twice
    //=======================================================================

    /// <summary>The number of non-degenerated edges of <paramref name="Vtx"/> in the map (the edges of junctions
    /// twice).</summary>
    internal static int ChFi3d_NbNotDegeneratedEdges(in Vertex Vtx, ChFiDS_Map VEMap)
    {
        int nba = VEMap[Vtx].Extent();
        foreach (Shape ItE in VEMap[Vtx])
        {
            Edge cur = ShapeCasts.Edge(ItE);
            if (Tool.Degenerated(cur))
            {
                nba--;
            }
        }
        return nba;
    }

    //=======================================================================
    // function : NbSharpEdges
    // purpose  : calculate the number of sharp edges of Map VEMap(Vtx)
    // Attention the edges of junctions are taken into account twice
    //=======================================================================

    /// <summary>The number of sharp edges of <paramref name="Vtx"/> in the map (neither degenerated nor between
    /// G2-tangent faces; the edges of junctions twice).</summary>
    internal static int ChFi3d_NbSharpEdges(in Vertex Vtx, ChFiDS_Map VEMap, ChFiDS_Map EFMap)
    {
        int nba = VEMap[Vtx].Extent();
        foreach (Shape ItE in VEMap[Vtx])
        {
            Edge cur = ShapeCasts.Edge(ItE);
            if (Tool.Degenerated(cur))
            {
                nba--;
            }
            else
            {
                ChFi3d_conexfaces(cur, out Face F1, out Face F2, EFMap);
                if (!F2.IsNull && ChFi3d.IsTangentFaces(cur, F1, F2, G2))
                {
                    nba--;
                }
            }
        }
        return nba;
    }

    //=======================================================================
    // function : NumberOfEdges
    // purpose  : calculate the number of edges arriving to the top Vtx
    // degenerated edges are not taken into account.
    //=======================================================================

    /// <summary>The number of edges at the vertex, degenerated edges excluded.</summary>
    internal static int ChFi3d_NumberOfEdges(in Vertex Vtx, ChFiDS_Map VEMap)
    {
        int nba;
        Edge edgelibre1 = new(), edgelibre2 = new();
        nba = ChFi3d_NbNotDegeneratedEdges(Vtx, VEMap);
        ChFi3d_ChercheBordsLibres(VEMap, Vtx, out bool bordlibre, ref edgelibre1, ref edgelibre2);
        if (bordlibre)
        {
            nba = (nba - 2) / 2 + 2;
        }
        else
        {
            nba = nba / 2;
        }
        return nba;
    }

    //=======================================================================
    // function : NumberOfSharpEdges
    // purpose  : calculate the number of edges arriving to the top Vtx
    // degenerated edges are not taken into account.
    //=======================================================================

    /// <summary>The number of sharp edges at the vertex.</summary>
    internal static int ChFi3d_NumberOfSharpEdges(in Vertex Vtx, ChFiDS_Map VEMap, ChFiDS_Map EFmap)
    {
        int nba;
        Edge edgelibre1 = new(), edgelibre2 = new();
        nba = ChFi3d_NbSharpEdges(Vtx, VEMap, EFmap);
        ChFi3d_ChercheBordsLibres(VEMap, Vtx, out bool bordlibre, ref edgelibre1, ref edgelibre2);
        if (bordlibre)
        {
            nba = (nba - 2) / 2 + 2;
        }
        else
        {
            nba = nba / 2;
        }
        return nba;
    }

    //=====================================================
    // function cherche_vertex
    // finds common vertex between two edges
    //=====================================================

    /// <summary>The vertex common to <paramref name="E1"/> and <paramref name="E2"/>, if any.</summary>
    internal static void ChFi3d_cherche_vertex(in Edge E1, in Edge E2, ref Vertex vertex,
                                               out bool trouve)
    {
        int i, j;
        Vertex Vcur1, Vcur2;
        trouve = false;
        NCollection_IndexedMap<Shape> MapV1 = new(ShapeHasher.Instance),
                                             MapV2 = new(ShapeHasher.Instance);
        Exploration.MapShapes(E1, ShapeType.Vertex, MapV1);
        Exploration.MapShapes(E2, ShapeType.Vertex, MapV2);
        for (i = 1; i <= MapV1.Extent() && !trouve; i++)
        {
            Shape alocalshape = MapV1.FindKey(i);
            Vcur1 = ShapeCasts.Vertex(alocalshape);
            //    Vcur1=TopoDS::Vertex(TopoDS_Shape (MapV1(i)));
            for (j = 1; j <= MapV2.Extent() && !trouve; j++)
            {
                Shape aLocalShape = MapV2.FindKey(j);
                Vcur2 = ShapeCasts.Vertex(aLocalShape);
                //      Vcur2=TopoDS::Vertex(TopoDS_Shape (MapV2(j)));
                if (Vcur2.IsSame(Vcur1))
                {
                    vertex = Vcur1;
                    trouve = true;
                }
            }
        }
    }

    //=======================================================================
    // function : ChFi3d_Couture
    // purpose  : determine si F a une arete de couture
    //=======================================================================

    /// <summary>Whether <paramref name="F"/> has a seam edge, and the edge.</summary>
    internal static void ChFi3d_Couture(in Face F, out bool couture, ref Edge edgecouture)
    {
        Edge Ecur;
        couture = false;
        NCollection_IndexedMap<Shape> MapE1 = new(ShapeHasher.Instance);
        Exploration.MapShapes(F, ShapeType.Edge, MapE1);
        Surface? Surf = Tool.Surface(F, out Placement Loc);
        for (int i = 1; i <= MapE1.Extent() && !couture; i++)
        {
            Shape aLocalShape = MapE1.FindKey(i);
            Ecur = ShapeCasts.Edge(aLocalShape);
            //      Ecur=TopoDS::Edge(TopoDS_Shape (MapE1(i)));
            if (Tool.IsClosed(Ecur, Surf, Loc))
            {
                couture = true;
                edgecouture = Ecur;
            }
        }
    }

    //=================================================================================================

    /// <summary>Whether <paramref name="F"/> has a seam edge on <paramref name="V"/>, and the edge.</summary>
    internal static void ChFi3d_CoutureOnVertex(in Face F, in Vertex V, out bool couture,
                                                ref Edge edgecouture)
    {
        Edge Ecur;
        couture = false;
        NCollection_IndexedMap<Shape> MapE1 = new(ShapeHasher.Instance);
        Exploration.MapShapes(F, ShapeType.Edge, MapE1);
        Surface? Surf = Tool.Surface(F, out Placement Loc);
        for (int i = 1; i <= MapE1.Extent(); i++)
        {
            Shape aLocalShape = MapE1.FindKey(i);
            Ecur = ShapeCasts.Edge(aLocalShape);
            //      Ecur=TopoDS::Edge(TopoDS_Shape (MapE1(i)));
            if (Tool.IsClosed(Ecur, Surf, Loc))
            {
                Exploration.Vertices(Ecur, out Vertex Vf, out Vertex Vl);
                if (Vf.IsSame(V) || Vl.IsSame(V))
                {
                    couture = true;
                    edgecouture = Ecur;
                    break;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Whether the closed edge <paramref name="E"/> of <paramref name="F"/> has a neighbouring seam, really
    /// closed, through one of its vertices.</summary>
    internal static bool ChFi3d_IsPseudoSeam(in Edge E, in Face F)
    {
        if (!Tool.IsClosed(E, F))
        {
            return false;
        }

        bool NeighborSeamFound = false;
        Exploration.Vertices(E, out Vertex Vf, out Vertex Vl);
        Explorer Explo = new(F, ShapeType.Edge);
        for (; Explo.More(); Explo.Next())
        {
            Edge Ecur = ShapeCasts.Edge(Explo.Current());
            if (!Ecur.IsSame(E))
            {
                Exploration.Vertices(Ecur, out Vertex V1, out Vertex V2);
                if ((V1.IsSame(Vf) || V1.IsSame(Vl) || V2.IsSame(Vf) || V2.IsSame(Vl))
                    && BrepTools.IsReallyClosed(Ecur, F))
                {
                    NeighborSeamFound = true;
                    break;
                }
            }
        }
        return NeighborSeamFound;
    }
}
