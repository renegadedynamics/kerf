// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_cherche_face1 to ChFi3d_IsPseudoSeam)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Fillets;

internal static partial class Builder_0
{
    //=======================================================================
    // function : cherche_face1
    // purpose  : find face F different from F1 in the map.
    // The map contains two faces adjacent to an edge
    //=======================================================================

    /// <summary>The face of <paramref name="map"/> other than <paramref name="f1"/>.</summary>
    /// <exception cref="ConstructionErrorException">None is found and <paramref name="f"/> is null.</exception>
    internal static void ChFi3d_cherche_face1(NCollection_List<Shape> map, in Face f1,
                                              ref Face f)
    {
        Face fcur;
        bool trouve = false;
        foreach (Shape it in map)
        {
            if (trouve)
            {
                break;
            }
            fcur = ShapeCasts.Face(it);
            if (!fcur.IsSame(f1))
            {
                f = fcur;
                trouve = true;
            }
        }
        if (f.IsNull)
        {
            throw new ConstructionErrorException("Failed to find face");
        }
    }

    //=======================================================================
    // function : cherche_element
    // purpose  : find edge E of F1 other than E1 and containing vertex V
    // Vtx is the other vertex of E
    //=======================================================================

    /// <summary>The edge <paramref name="e"/> of <paramref name="f1"/>, other than <paramref name="e1"/>, on
    /// <paramref name="v"/>; <paramref name="vtx"/> its other vertex.</summary>
    /// <exception cref="ConstructionErrorException">None is found and <paramref name="e"/> is null.</exception>
    internal static void ChFi3d_cherche_element(in Vertex v, in Edge e1, in Face f1,
                                                ref Edge e, ref Vertex vtx)
    {
        int ie;
        Vertex v1, v2;
        bool trouve = false;
        Edge ecur;
        IndexedSet<Shape> mapE = new(ShapeHasher.Instance);
        Exploration.MapShapes(f1, ShapeType.Edge, mapE);
        for (ie = 1; ie <= mapE.Count && !trouve; ie++)
        {
            ecur = ShapeCasts.Edge(mapE[ie - 1]);
            if (!ecur.IsSame(e1))
            {
                IndexedSet<Shape> mapV = new(ShapeHasher.Instance);
                Exploration.MapShapes(ecur, ShapeType.Vertex, mapV);
                if (mapV.Count == 2)
                {
                    v1 = ShapeCasts.Vertex(mapV[0]);
                    v2 = ShapeCasts.Vertex(mapV[1]);
                    if (v1.IsSame(v))
                    {
                        vtx = v2;
                        e = ecur;
                        trouve = true;
                    }
                    else if (v2.IsSame(v))
                    {
                        vtx = v1;
                        e = ecur;
                        trouve = true;
                    }
                }
            }
        }
        if (e.IsNull)
        {
            throw new ConstructionErrorException("Failed to find element");
        }
    }

    //=======================================================================
    // function : cherche_edge
    // purpose  : find edge E of F1 other than the list of edges E1 and
    //           containing vertex V  Vtx is the other vertex of E.
    //=======================================================================

    /// <summary>The edge <paramref name="e"/> of <paramref name="f1"/>, none of <paramref name="e1"/>, on
    /// <paramref name="v"/>; <paramref name="vtx"/> its other vertex.</summary>
    /// <exception cref="ConstructionErrorException">None is found and <paramref name="e"/> is null.</exception>
    internal static void ChFi3d_cherche_edge(in Vertex v, NCollection_Array1<Shape> e1,
                                             in Face f1, ref Edge e, ref Vertex vtx)
    {
        int ie, i;
        Vertex v1, v2;
        bool trouve = false;
        Edge ecur;
        bool same;
        IndexedSet<Shape> mapE = new(ShapeHasher.Instance);
        Exploration.MapShapes(f1, ShapeType.Edge, mapE);
        for (ie = 1; ie <= mapE.Count && !trouve; ie++)
        {
            ecur = ShapeCasts.Edge(mapE[ie - 1]);
            same = false;
            for (i = e1.Lower(); i <= e1.Upper(); i++)
            {
                if (ecur.IsSame(e1.Value(i)))
                {
                    same = true;
                }
            }
            if (!same)
            {
                IndexedSet<Shape> mapV = new(ShapeHasher.Instance);
                Exploration.MapShapes(ecur, ShapeType.Vertex, mapV);
                if (mapV.Count == 2)
                {
                    v1 = ShapeCasts.Vertex(mapV[0]);
                    v2 = ShapeCasts.Vertex(mapV[1]);
                    if (v1.IsSame(v))
                    {
                        vtx = v2;
                        e = ecur;
                        trouve = true;
                    }
                    else if (v2.IsSame(v))
                    {
                        vtx = v1;
                        e = ecur;
                        trouve = true;
                    }
                }
            }
        }
        if (e.IsNull)
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
            foreach (Shape jtF in mapVF)
            {
                if (!(kf < fj))
                {
                    break;
                }
                if (cur.IsSame(jtF))
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
    internal static void ChFi3d_edge_common_faces(NCollection_List<Shape> mapEF, out Face f1,
                                                  out Face f2)
    {
        Face f;
        bool trouve;
        f1 = ShapeCasts.Face(mapEF.First());
        f2 = new Face();
        trouve = false;
        foreach (Shape it in mapEF)
        {
            if (trouve)
            {
                break;
            }
            f = ShapeCasts.Face(it);
            if (!f.IsSame(f1))
            {
                f2 = f;
                trouve = true;
            }
        }
        if (!trouve)
        {
            f2 = f1;
        }
    }

    /***********************************************************/
    // gives the angle between edges E1 and E2.
    // Vtx is the vertex common to the edges
    /************************************************************/

    /// <summary>The angle between the edges <paramref name="e1"/> and <paramref name="e2"/> leaving their common
    /// vertex <paramref name="vtx"/>.</summary>
    internal static double ChFi3d_AngleEdge(in Vertex vtx, in Edge e1, in Edge e2)
    {
        double angle;
        EdgeCurveAdaptor bCurv1 = new(e1);
        EdgeCurveAdaptor bCurv2 = new(e2);
        double parE1, parE2;
        parE1 = Tool.Parameter(vtx, e1);
        parE2 = Tool.Parameter(vtx, e2);
        bCurv1.D1(parE1, out Point3D p1, out Vector3D dir1);
        bCurv2.D1(parE2, out Point3D p2, out Vector3D dir2);
        if (!vtx.IsSame(Exploration.FirstVertex(e1)))
        {
            dir1 = dir1.Reversed();
        }
        if (!vtx.IsSame(Exploration.FirstVertex(e2)))
        {
            dir2 = dir2.Reversed();
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

    /// <summary>Whether <paramref name="v1"/> has two edges on free borders, and these edges.</summary>
    internal static void ChFi3d_ChercheBordsLibres(Map myVEMap, in Vertex v1, out bool bordlibre,
                                                   ref Edge edgelibre1, ref Edge edgelibre2)
    {
        bordlibre = false;
        int nboccur;
        foreach (Shape itE in myVEMap[v1])
        {
            if (bordlibre)
            {
                break;
            }
            nboccur = 0;
            Edge cur = ShapeCasts.Edge(itE);
            if (!Tool.Degenerated(cur))
            {
                foreach (Shape itE1 in myVEMap[v1])
                {
                    Edge cur1 = ShapeCasts.Edge(itE1);
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
            foreach (Shape itE in myVEMap[v1])
            {
                if (bordlibre)
                {
                    break;
                }
                nboccur = 0;
                Edge cur = ShapeCasts.Edge(itE);
                if (!Tool.Degenerated(cur) && !cur.IsSame(edgelibre1))
                {
                    foreach (Shape itE1 in myVEMap[v1])
                    {
                        Edge cur1 = ShapeCasts.Edge(itE1);
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

    /// <summary>The number of non-degenerated edges of <paramref name="vtx"/> in the map (the edges of junctions
    /// twice).</summary>
    internal static int ChFi3d_NbNotDegeneratedEdges(in Vertex vtx, Map veMap)
    {
        int nba = veMap[vtx].Extent();
        foreach (Shape itE in veMap[vtx])
        {
            Edge cur = ShapeCasts.Edge(itE);
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

    /// <summary>The number of sharp edges of <paramref name="vtx"/> in the map (neither degenerated nor between
    /// G2-tangent faces; the edges of junctions twice).</summary>
    internal static int ChFi3d_NbSharpEdges(in Vertex vtx, Map veMap, Map efMap)
    {
        int nba = veMap[vtx].Extent();
        foreach (Shape itE in veMap[vtx])
        {
            Edge cur = ShapeCasts.Edge(itE);
            if (Tool.Degenerated(cur))
            {
                nba--;
            }
            else
            {
                ChFi3d_conexfaces(cur, out Face f1, out Face f2, efMap);
                if (!f2.IsNull && global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(cur, f1, f2, G2))
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
    internal static int ChFi3d_NumberOfEdges(in Vertex vtx, Map veMap)
    {
        int nba;
        Edge edgelibre1 = new(), edgelibre2 = new();
        nba = ChFi3d_NbNotDegeneratedEdges(vtx, veMap);
        ChFi3d_ChercheBordsLibres(veMap, vtx, out bool bordlibre, ref edgelibre1, ref edgelibre2);
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
    internal static int ChFi3d_NumberOfSharpEdges(in Vertex vtx, Map veMap, Map eFmap)
    {
        int nba;
        Edge edgelibre1 = new(), edgelibre2 = new();
        nba = ChFi3d_NbSharpEdges(vtx, veMap, eFmap);
        ChFi3d_ChercheBordsLibres(veMap, vtx, out bool bordlibre, ref edgelibre1, ref edgelibre2);
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

    /// <summary>The vertex common to <paramref name="e1"/> and <paramref name="e2"/>, if any.</summary>
    internal static void ChFi3d_cherche_vertex(in Edge e1, in Edge e2, ref Vertex vertex,
                                               out bool trouve)
    {
        int i, j;
        Vertex vcur1, vcur2;
        trouve = false;
        IndexedSet<Shape> mapV1 = new(ShapeHasher.Instance),
                                             mapV2 = new(ShapeHasher.Instance);
        Exploration.MapShapes(e1, ShapeType.Vertex, mapV1);
        Exploration.MapShapes(e2, ShapeType.Vertex, mapV2);
        for (i = 1; i <= mapV1.Count && !trouve; i++)
        {
            Shape alocalshape = mapV1[i - 1];
            vcur1 = ShapeCasts.Vertex(alocalshape);
            //    Vcur1=TopoDS::Vertex(TopoDS_Shape (MapV1(i)));
            for (j = 1; j <= mapV2.Count && !trouve; j++)
            {
                Shape localShape = mapV2[j - 1];
                vcur2 = ShapeCasts.Vertex(localShape);
                //      Vcur2=TopoDS::Vertex(TopoDS_Shape (MapV2(j)));
                if (vcur2.IsSame(vcur1))
                {
                    vertex = vcur1;
                    trouve = true;
                }
            }
        }
    }

    //=======================================================================
    // function : ChFi3d_Couture
    // purpose  : determine si F a une arete de couture
    //=======================================================================

    /// <summary>Whether <paramref name="f"/> has a seam edge, and the edge.</summary>
    internal static void ChFi3d_Couture(in Face f, out bool couture, ref Edge edgecouture)
    {
        Edge ecur;
        couture = false;
        IndexedSet<Shape> mapE1 = new(ShapeHasher.Instance);
        Exploration.MapShapes(f, ShapeType.Edge, mapE1);
        Surface? surf = Tool.Surface(f, out Placement loc);
        for (int i = 1; i <= mapE1.Count && !couture; i++)
        {
            Shape localShape = mapE1[i - 1];
            ecur = ShapeCasts.Edge(localShape);
            //      Ecur=TopoDS::Edge(TopoDS_Shape (MapE1(i)));
            if (Tool.IsClosed(ecur, surf, loc))
            {
                couture = true;
                edgecouture = ecur;
            }
        }
    }

    //=================================================================================================

    /// <summary>Whether <paramref name="f"/> has a seam edge on <paramref name="v"/>, and the edge.</summary>
    internal static void ChFi3d_CoutureOnVertex(in Face f, in Vertex v, out bool couture,
                                                ref Edge edgecouture)
    {
        Edge ecur;
        couture = false;
        IndexedSet<Shape> mapE1 = new(ShapeHasher.Instance);
        Exploration.MapShapes(f, ShapeType.Edge, mapE1);
        Surface? surf = Tool.Surface(f, out Placement loc);
        for (int i = 0; i < mapE1.Count; i++)
        {
            Shape localShape = mapE1[i];
            ecur = ShapeCasts.Edge(localShape);
            //      Ecur=TopoDS::Edge(TopoDS_Shape (MapE1(i)));
            if (Tool.IsClosed(ecur, surf, loc))
            {
                Exploration.Vertices(ecur, out Vertex vf, out Vertex vl);
                if (vf.IsSame(v) || vl.IsSame(v))
                {
                    couture = true;
                    edgecouture = ecur;
                    break;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Whether the closed edge <paramref name="e"/> of <paramref name="f"/> has a neighbouring seam, really
    /// closed, through one of its vertices.</summary>
    internal static bool ChFi3d_IsPseudoSeam(in Edge e, in Face f)
    {
        if (!Tool.IsClosed(e, f))
        {
            return false;
        }

        bool neighborSeamFound = false;
        Exploration.Vertices(e, out Vertex vf, out Vertex vl);
        Explorer explo = new(f, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Edge ecur = ShapeCasts.Edge(explo.Current());
            if (!ecur.IsSame(e))
            {
                Exploration.Vertices(ecur, out Vertex v1, out Vertex v2);
                if ((v1.IsSame(vf) || v1.IsSame(vl) || v2.IsSame(vf) || v2.IsSame(vl))
                    && BrepTools.IsReallyClosed(ecur, f))
                {
                    neighborSeamFound = true;
                    break;
                }
            }
        }
        return neighborSeamFound;
    }
}
