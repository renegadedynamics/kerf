// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Inter2d.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Inter2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Wed Dec 24 18:08:39 2003 OCC4455

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Offsets;

/// <summary>
/// Computes the intersections between edges on a face and stores the result as descendants in an AsDes
/// (<see cref="AsDes"/>).
/// </summary>
/// <remarks>
/// A tool of the 3D offset. As in OCCT, it writes into the shapes it works on: the edges stored in the AsDes it is
/// given (pcurve ranges, 3D curves built by <see cref="BrepLibrary"/>, the parameters of the intersection vertices) and the
/// vertices stored there (tolerances, parameters on edges). Those are the offset's own working shapes, built from its
/// own copy of the caller's shape: they must be unfrozen; a frozen one throws at the first write. The vertices and the
/// edge (<see cref="ExtentEdge"/>) it makes are not frozen.
/// </remarks>
public static class Inter2d
{
    //=================================================================================================

    private static Vertex CommonVertex(in Edge e1, in Edge e2)
    {
        Vertex[] v1 = [new(), new()], v2 = [new(), new()];
        Vertex v = new();
        //
        Exploration.Vertices(e1, out v1[0], out v1[1], true);
        Exploration.Vertices(e2, out v2[0], out v2[1], true);
        // The first edge is the current one, the second edge is the next one.
        // We check last vertex of the first edge first.
        if (v1[1].IsSame(v2[0]) || v1[1].IsSame(v2[1]))
        {
            return v1[1];
        }
        if (v1[0].IsSame(v2[0]) || v1[0].IsSame(v2[1]))
        {
            return v1[0];
        }
        //
        return v;
    }

    private static int DefineClosedness(in Face face)
    {
        Explorer explo = new(face, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Edge edge = ShapeCasts.Edge(explo.Current());
            if (BrepTools.IsReallyClosed(edge, face))
            {
                Curve2D? pCurve = Tool.CurveOnSurface(edge, face, out double fpar, out _);
                Vector2D tangent = pCurve!.DN(fpar, 1);
                double crossProd1 = tangent ^ new Vector2D(Constants.DX2d());
                double crossProd2 = tangent ^ new Vector2D(Constants.DY2d());
                if (Math.Abs(crossProd2) < Math.Abs(crossProd1))
                { // pcurve is parallel to OY
                    return 1;
                }
                else
                {
                    return 2;
                }
            }
        }

        return 0;
    }

    private static void GetEdgesOrientedInFace(in Shape shape,
                                               in Face face,
                                               AsDes asDes,
                                               NCollection_Sequence<Shape> seqEdges)
    {
        NCollection_List<Shape> edges = asDes.Descendant(face);

        Explorer explo = new(shape, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Shape edge = explo.Current();
            NCollection_List<Shape>.Iterator itl = new(edges);
            for (; itl.More(); itl.Next())
            {
                Shape edgeInFace = itl.Value();
                if (edgeInFace.IsSame(edge))
                {
                    seqEdges.Append(edgeInFace);
                    break;
                }
            }
        }

        if (seqEdges.Length() == 1)
        {
            return;
        }

        IndexedMap<Shape, NCollection_List<Shape>> vEmap =
            new(ShapeHasher.Instance);
        for (int ii = 1; ii <= seqEdges.Length(); ii++)
        {
            Exploration.MapShapesAndAncestors(seqEdges[ii], ShapeType.Vertex, ShapeType.Edge, vEmap);
        }

        Vertex firstVertex = new();
        Edge firstEdge = new();
        for (int ii = 0; ii < vEmap.Count; ii++)
        {
            Vertex vertex = ShapeCasts.Vertex(vEmap.KeyAt(ii));
            NCollection_List<Shape> elist = vEmap.ValueAt(ii);
            if (elist.Extent() == 1)
            {
                Edge edge = ShapeCasts.Edge(elist.First());
                Exploration.Vertices(edge, out Vertex v1, out _, true); // with orientation
                if (v1.IsSame(vertex))
                {
                    firstVertex = vertex;
                    firstEdge = edge;
                    break;
                }
            }
        }

        if (firstEdge.IsNull) // closed set of edges
        {
            // double aPeriod = 0.;
            int indCoord = DefineClosedness(face);
            /*
            BRepAdaptor_Surface aBAsurf (theFace, false);
            if (IndCoord == 1)
              aPeriod = aBAsurf.LastUParameter() - aBAsurf.FirstUParameter();
            else if (IndCoord == 2)
              aPeriod = aBAsurf.LastVParameter() - aBAsurf.FirstVParameter();
            */

            if (indCoord != 0)
            {
                double maxDelta = 0.0;
                for (int ii = 0; ii < vEmap.Count; ii++)
                {
                    Vertex vertex = ShapeCasts.Vertex(vEmap.KeyAt(ii));
                    NCollection_List<Shape> elist = vEmap.ValueAt(ii);
                    Edge edge1 = ShapeCasts.Edge(elist.First());
                    Edge edge2 = ShapeCasts.Edge(elist.Last());
                    double param1 = Tool.Parameter(vertex, edge1);
                    double param2 = Tool.Parameter(vertex, edge2);
                    EdgeCurve2DAdaptor bAcurve1 = new(edge1, face);
                    EdgeCurve2DAdaptor bAcurve2 = new(edge2, face);
                    Point2D pnt1 = bAcurve1.Value(param1);
                    Point2D pnt2 = bAcurve2.Value(param2);
                    double delta = Math.Abs(pnt1.Coord(indCoord) - pnt2.Coord(indCoord));
                    if (delta > maxDelta)
                    {
                        maxDelta = delta;
                        firstVertex = vertex;
                    }
                }
                NCollection_List<Shape> elist1 = vEmap.GetValue(firstVertex);
                NCollection_List<Shape>.Iterator itl = new(elist1);
                for (; itl.More(); itl.Next())
                {
                    Edge edge = ShapeCasts.Edge(itl.Value());
                    Exploration.Vertices(edge, out Vertex v1, out _, true); // with orientation
                    if (v1.IsSame(firstVertex))
                    {
                        firstEdge = edge;
                        break;
                    }
                }
            }
        }

        int nbEdges = seqEdges.Length();
        seqEdges.Clear();
        seqEdges.Append(firstEdge);
        Edge edgeCur = firstEdge;
        for (; ; )
        {
            Vertex lastVertex = Exploration.LastVertex(edgeCur, true); // with orientation
            if (lastVertex.IsSame(firstVertex))
            {
                break;
            }

            NCollection_List<Shape> elist = vEmap.GetValue(lastVertex);
            if (elist.Extent() == 1)
            {
                break;
            }

            if (elist.First().IsSame(edgeCur))
            {
                edgeCur = ShapeCasts.Edge(elist.Last());
            }
            else
            {
                edgeCur = ShapeCasts.Edge(elist.First());
            }

            seqEdges.Append(edgeCur);
            if (seqEdges.Length() == nbEdges)
            {
                break;
            }
        }
    }

    //=======================================================================
    // function : Store
    // purpose  : Store the vertices <theLV> into AsDes for the edge <theEdge>.
    //           The vertices are added despite of the coincidence with
    //           already added vertices. When all vertices for all edges
    //           are added the coinciding chains of vertices should be fused
    //           using FuseVertices() method.
    //=======================================================================
    private static void Store(in Edge edge,
                              NCollection_List<Shape> lv,
                              double tol,
                              bool isToUpdate,
                              AsDes asDes2d,
                              IndexedMap<Shape, NCollection_List<Shape>> dmvv)
    {
        // Update vertices
        NCollection_List<Shape>.Iterator it = new(lv);
        for (; it.More(); it.Next())
        {
            Vertex v = ShapeCasts.Vertex(it.Value());
            new BrepBuilder().UpdateVertex(v, tol);
        }

        // Get vertices already added to the edge and check the distances to the new ones
        NCollection_List<Shape> lvEx = asDes2d.Descendant(edge);
        if (!isToUpdate && lvEx.IsEmpty())
        {
            if (lv.Extent() != 0)
            {
                asDes2d.Add(edge, lv);
            }
            return;
        }
        //
        ProjectPointOnCurve projPC = new();
        double tolE = 0.0;
        if (isToUpdate)
        {
            Curve3D? c = Tool.Curve(edge, out double t1, out double t2);
            projPC.Init(c!, t1, t2);
            tolE = Tool.Tolerance(edge);
        }
        //
        HashSet<Shape> mv = new(ShapeHasher.Instance);
        for (it.Initialize(lv); it.More(); it.Next())
        {
            Vertex v = ShapeCasts.Vertex(it.Value());
            if (!mv.Add(v))
            {
                continue;
            }
            //
            Point3D p = Tool.Pnt(v);
            double aTol = Tool.Tolerance(v);
            //
            NCollection_List<Shape> lvc = new();
            NCollection_List<Shape>.Iterator itEx = new(lvEx);
            for (; itEx.More(); itEx.Next())
            {
                Vertex vEx = ShapeCasts.Vertex(itEx.Value());
                if (v.IsSame(vEx))
                {
                    break;
                }
                Point3D pEx = Tool.Pnt(vEx);
                double tolVEx = Tool.Tolerance(vEx);
                if (p.IsEqual(pEx, aTol + tolVEx))
                {
                    lvc.Append(vEx);
                }
            }
            //
            if (itEx.More())
            {
                continue;
            }
            //
            if (isToUpdate)
            {
                // get parameter of the vertex on the edge
                projPC.Perform(p);
                if (projPC.NbPoints() == 0)
                {
                    continue;
                }
                //
                if (projPC.LowerDistance() > aTol + tolE)
                {
                    continue;
                }
                //
                double t = projPC.LowerDistanceParameter();
                Shape localShape = v.Oriented(Internal);
                new BrepBuilder().UpdateVertex(ShapeCasts.Vertex(localShape), t, edge, aTol);
            }
            //
            if (lvc.Extent() != 0)
            {
                NCollection_List<Shape>.Iterator itLV = new(lvc);
                for (; itLV.More(); itLV.Next())
                {
                    Shape vc = itLV.Value();
                    int indC = dmvv.IndexOf(vc) + 1;
                    if (indC == 0)
                    {
                        indC = dmvv.Add(vc, new NCollection_List<Shape>()) + 1;
                    }
                    dmvv.ValueAt(indC - 1).Append(v);
                }
                //
                int ind = dmvv.IndexOf(v) + 1;
                if (ind == 0)
                {
                    ind = dmvv.Add(v, new NCollection_List<Shape>()) + 1;
                }
                dmvv.ValueAt(ind - 1).Append(lvc);
            }
            asDes2d.Add(edge, v);
        }
    }

    //=======================================================================
    // function : Store
    // purpose  : Store the intersection vertices between two edges into AsDes
    //=======================================================================
    private static void Store(in Edge e1,
                              in Edge e2,
                              NCollection_List<Shape> lv1,
                              NCollection_List<Shape> lv2,
                              double tol,
                              AsDes asDes2d,
                              IndexedMap<Shape, NCollection_List<Shape>> dmvv)
    {
        for (int i = 0; i < 2; ++i)
        {
            Edge e = i == 0 ? e1 : e2;
            NCollection_List<Shape> lv = i == 0 ? lv1 : lv2;
            Store(e, lv, tol, false, asDes2d, dmvv);
        }
    }

    //=================================================================================================

    private static void EdgeInter(in Face F,
                                  FaceSurfaceAdaptor bAsurf,
                                  in Edge e1,
                                  in Edge e2,
                                  AsDes asDes,
                                  double tol,
                                  bool withOri,
                                  IndexedMap<Shape, NCollection_List<Shape>> aDMVV)
    {

        if (e1.IsSame(e2))
        {
            return;
        }

        double[] f = new double[3], l = new double[3];
        double tolDub = 1.0e-7;
        int i;

        Tool.Range(e1, out f[1], out l[1]);
        Tool.Range(e2, out f[2], out l[2]);

        EdgeCurveAdaptor ce1 = new(e1, F);
        EdgeCurveAdaptor ce2 = new(e2, F);

        Edge[] ei = [new(), new(), new()];
        ei[1] = e1;
        ei[2] = e2;
        NCollection_List<Shape> lv1 = new();
        NCollection_List<Shape> lv2 = new();
        BrepBuilder b = new();

        Vertex cv = new();
        if (!Exploration.CommonVertex(e1, e2, ref cv))
        {
            BrepLibrary.BuildCurve3d(e1);
            BrepLibrary.BuildCurve3d(e2);

            double tolSum = Tool.Tolerance(e1) + Tool.Tolerance(e2);
            tolSum = Max(tolSum, 1.0e-5);

            NCollection_Sequence<Point3D> resPoints = new();
            NCollection_Sequence<double> resParamsOnE1 = new(), resParamsOnE2 = new();
            Point3D degPoint = new();
            bool withDegen = Tool.Degenerated(e1) || Tool.Degenerated(e2);

            if (withDegen)
            {
                int ideg = (Tool.Degenerated(e1)) ? 1 : 2;
                Iterator iter = new(ei[ideg]);
                if (iter.More())
                {
                    Vertex vdeg = ShapeCasts.Vertex(iter.Value());
                    degPoint = Tool.Pnt(vdeg);
                }
                else
                {
                    EdgeCurveAdaptor cEdeg = new(ei[ideg], F);
                    degPoint = cEdeg.Value(cEdeg.FirstParameter());
                }
            }
            //
            Curve2D? pcurve1 = Tool.CurveOnSurface(e1, F, out f[1], out l[1]);
            Curve2D? pcurve2 = Tool.CurveOnSurface(e2, F, out f[2], out l[2]);
            CurveAdaptor2D gac1 = new(pcurve1!, f[1], l[1]);
            CurveAdaptor2D gac2 = new(pcurve2!, f[2], l[2]);
            GInter inter2d = new(gac1, gac2, tolDub, tolDub);
            for (i = 1; i <= inter2d.PointCount; i++)
            {
                Point3D p3d;
                if (withDegen)
                {
                    p3d = degPoint;
                }
                else
                {
                    Point2D p2d = inter2d.Point(i).Value();
                    p3d = bAsurf.Value(p2d.X(), p2d.Y());
                }
                resPoints.Append(p3d);
                resParamsOnE1.Append(inter2d.Point(i).ParamOnFirst());
                resParamsOnE2.Append(inter2d.Point(i).ParamOnSecond());
            }

            for (i = 1; i <= resPoints.Length(); i++)
            {
                double t1 = resParamsOnE1[i]; // ponc1.Parameter();
                double t2 = resParamsOnE2[i]; // ponc2.Parameter();
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(t1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(t2))
                {
                    // OCCT_DEBUG output left out
                    continue;
                }

                Point3D p = resPoints[i]; // ponc1.Value();
                Vertex newVertex = new LibraryMakeVertex(p).InternalVertex();
                newVertex.Orientation(Internal);
                b.UpdateVertex(newVertex, t1, e1, tol);
                b.UpdateVertex(newVertex, t2, e2, tol);
                Point3D p1 = ce1.Value(t1);
                Point3D p2 = ce2.Value(t2);
                double dist1, dist2, dist3;
                dist1 = p1.Distance(p);
                dist2 = p2.Distance(p);
                dist3 = p1.Distance(p2);
                dist1 = Max(dist1, dist2);
                dist1 = Max(dist1, dist3);
                b.UpdateVertex(newVertex, dist1);

                // OCCT_DEBUG checks of the solution left out
                // define the orientation of a new vertex
                Orientation oo1 = Reversed;
                Orientation oo2 = Reversed;
                if (withOri)
                {
                    EdgeCurve2DAdaptor pce1 = new(e1, F);
                    EdgeCurve2DAdaptor pce2 = new(e2, F);
                    Vector2D v1or, v2or;
                    pce1.D1(t1, out _, out Vector2D V1);
                    pce2.D1(t2, out _, out Vector2D V2);
                    v1or = V1;
                    v2or = V2;
                    if (e1.Orientation() == Reversed)
                    {
                        v1or = v1or.Reversed();
                    }
                    if (e2.Orientation() == Reversed)
                    {
                        v2or = v2or.Reversed();
                    }
                    double crossProd = v2or ^ V1;
                    // OCCT_DEBUG output left out
                    if (crossProd > 0.0)
                    {
                        oo1 = Forward;
                    }
                    crossProd = v1or ^ V2;
                    if (crossProd > 0.0)
                    {
                        oo2 = Forward;
                    }
                }
                lv1.Append(newVertex.Oriented(oo1));
                lv2.Append(newVertex.Oriented(oo2));
            }
        }

        //----------------------------------
        // Test at end.
        //---------------------------------
        double u1, u2;
        double tolConf = tol;
        Vertex[] v1v = [new(), new()], v2v = [new(), new()];
        Exploration.Vertices(e1, out v1v[0], out v1v[1]);
        Exploration.Vertices(e2, out v2v[0], out v2v[1]);

        int j;
        for (j = 0; j < 2; j++)
        {
            if (v1v[j].IsNull)
            {
                continue;
            }
            for (int k = 0; k < 2; k++)
            {
                if (v2v[k].IsNull)
                {
                    continue;
                }
                if (v1v[j].IsSame(v2v[k]))
                {
                    if (asDes.HasAscendant(v1v[j]))
                    {
                        continue;
                    }
                }
                //
                Point3D p1 = Tool.Pnt(v1v[j]);
                Point3D p2 = Tool.Pnt(v2v[k]);
                double dist = p1.Distance(p2);
                if (dist < tolConf)
                {
                    double aTol = Max(Tool.Tolerance(v1v[j]), Tool.Tolerance(v2v[k]));
                    Vertex v = new LibraryMakeVertex(p1).InternalVertex();
                    u1 = (j == 0) ? f[1] : l[1];
                    u2 = (k == 0) ? f[2] : l[2];
                    //
                    Shape localShape = v.Oriented(Internal);
                    b.UpdateVertex(ShapeCasts.Vertex(localShape), u1, e1, aTol);
                    b.UpdateVertex(ShapeCasts.Vertex(localShape), u2, e2, aTol);
                    //
                    lv1.Prepend(v.Oriented(v1v[j].Orientation()));
                    lv2.Prepend(v.Oriented(v2v[k].Orientation()));
                }
            }
        }

        bool affichPurge = false;

        if (!lv1.IsEmpty())
        {
            //----------------------------------
            // Remove all vertices.
            // There can be doubles
            //----------------------------------
            NCollection_List<Shape>.Iterator it1LV1 = new(), it1LV2 = new(), it2LV1 = new();
            bool purge = true;

            while (purge)
            {
                i = 1;
                purge = false;
                for (it1LV1.Initialize(lv1), it1LV2.Initialize(lv2); it1LV1.More(); it1LV1.Next(), it1LV2.Next())
                {
                    Vertex v1 = ShapeCasts.Vertex(it1LV1.Value());
                    Point3D p1 = Tool.Pnt(v1);
                    double tol1 = Tool.Tolerance(v1);

                    j = 1;
                    it2LV1.Initialize(lv1);
                    while (j < i)
                    {
                        Vertex v2 = ShapeCasts.Vertex(it2LV1.Value());
                        Point3D p2 = Tool.Pnt(v2);
                        double aTol = Max(tol1, Tool.Tolerance(v2));
                        if (p1.IsEqual(p2, aTol))
                        {
                            lv1.Remove(it1LV1);
                            lv2.Remove(it1LV2);
                            if (affichPurge)
                            {
                                Console.WriteLine("Doubles removed in EdgeInter.");
                            }
                            purge = true;
                            break;
                        }
                        j++;
                        it2LV1.Next();
                    }
                    if (purge)
                    {
                        break;
                    }
                    i++;
                }
            }
            //---------------------------------
            // Vertex storage in DS.
            //---------------------------------
            double tolStore = Tool.Tolerance(e1) + Tool.Tolerance(e2);
            tolStore = Max(tolStore, tol);
            Store(e1, e2, lv1, lv2, tolStore, asDes, aDMVV);
        }
    }

    //=================================================================================================

    private static void RefEdgeInter(in Face F,
                                     FaceSurfaceAdaptor bAsurf,
                                     in Edge e1,
                                     in Edge e2,
                                     Orientation or1,
                                     Orientation or2,
                                     AsDes asDes,
                                     double tol,
                                     bool withOri,
                                     in Vertex vref,
                                     ShapeImage imageVV,
                                     IndexedMap<Shape, NCollection_List<Shape>> aDMVV,
                                     out bool coincide)
    {
        //
        coincide = false;
        //
        if (e1.IsSame(e2))
        {
            return;
        }
        if (e1.IsNull || e2.IsNull)
        {
            return;
        }

        double[] f = new double[3], l = new double[3];
        double tolDub = 1.0e-7, tolLL = 0.0;
        int i;

        Curve2D? pcurve1 = Tool.CurveOnSurface(e1, F, out f[1], out l[1]);
        Curve2D? pcurve2 = Tool.CurveOnSurface(e2, F, out f[2], out l[2]);
        if (pcurve1 is null || pcurve2 is null)
        {
            return;
        }

        EdgeCurveAdaptor ce1 = new(e1, F);
        EdgeCurveAdaptor ce2 = new(e2, F);

        Edge[] ei = [new(), new(), new()];
        ei[1] = e1;
        ei[2] = e2;
        NCollection_List<Shape> lv1 = new();
        NCollection_List<Shape> lv2 = new();
        BrepBuilder b = new();

        BrepLibrary.BuildCurve3d(e1);
        BrepLibrary.BuildCurve3d(e2);

        NCollection_Sequence<Point3D> resPoints = new();
        NCollection_Sequence<double> resParamsOnE1 = new(), resParamsOnE2 = new();
        Point3D degPoint = new();
        bool withDegen = Tool.Degenerated(e1) || Tool.Degenerated(e2);

        if (withDegen)
        {
            int ideg = (Tool.Degenerated(e1)) ? 1 : 2;
            Iterator iter = new(ei[ideg]);
            if (iter.More())
            {
                Vertex vdeg = ShapeCasts.Vertex(iter.Value());
                degPoint = Tool.Pnt(vdeg);
            }
            else
            {
                EdgeCurveAdaptor cEdeg = new(ei[ideg], F);
                degPoint = cEdeg.Value(cEdeg.FirstParameter());
            }
        }
        //
        CurveAdaptor2D gac1 = new(pcurve1, f[1], l[1]);
        CurveAdaptor2D gac2 = new(pcurve2, f[2], l[2]);
        if ((gac1.GetType() == Line) && (gac2.GetType() == Line))
        {
            // Just quickly check if lines coincide
            double angle = Math.Abs(gac1.Line().Direction().Angle(gac2.Line().Direction()));
            if (angle <= 1.0e-8 || M_PI - angle <= 1.0e-8)
            {
                coincide = true;
                return;
            }
            else
            {
                // Take into account the intersection range of line-line intersection
                // (the smaller angle between curves, the bigger range)
                tolLL = IntersectionTools.ComputeIntRange(tolDub, tolDub, angle);
                tolLL = Min(tolLL, 1.0e-5);
            }
        }

        GInter inter2d = new(gac1, gac2, tolDub, tolDub);
        //
        if (!inter2d.IsDone || inter2d.PointCount == 0)
        {
            coincide = (inter2d.SegmentCount != 0 && (gac1.GetType() == Line)
                           && (gac2.GetType() == Line));
            return;
        }
        //
        for (i = 1; i <= inter2d.PointCount; i++)
        {
            Point3D p3d;
            if (withDegen)
            {
                p3d = degPoint;
            }
            else
            {
                Point2D p2d = inter2d.Point(i).Value();
                p3d = bAsurf.Value(p2d.X(), p2d.Y());
            }
            resPoints.Append(p3d);
            resParamsOnE1.Append(inter2d.Point(i).ParamOnFirst());
            resParamsOnE2.Append(inter2d.Point(i).ParamOnSecond());
        }

        for (i = 1; i <= resPoints.Length(); i++)
        {
            double t1 = resParamsOnE1[i]; // ponc1.Parameter();
            double t2 = resParamsOnE2[i]; // ponc2.Parameter();
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(t1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(t2))
            {
                // OCCT_DEBUG output left out
                continue;
            }

            Point3D p = resPoints[i]; // ponc1.Value();
            Vertex newVertex = new LibraryMakeVertex(p).InternalVertex();
            newVertex.Orientation(Internal);
            b.UpdateVertex(newVertex, t1, e1, tol);
            b.UpdateVertex(newVertex, t2, e2, tol);
            Point3D p1 = ce1.Value(t1);
            Point3D p2 = ce2.Value(t2);
            double dist1, dist2, dist3;
            dist1 = p1.Distance(p);
            dist2 = p2.Distance(p);
            dist3 = p1.Distance(p2);
            dist1 = Max(dist1, dist2);
            dist1 = Max(dist1, dist3);
            b.UpdateVertex(newVertex, dist1);

            // OCCT_DEBUG checks of the solution left out
            // define the orientation of a new vertex
            Orientation oo1 = Reversed;
            Orientation oo2 = Reversed;
            if (withOri)
            {
                EdgeCurve2DAdaptor pce1 = new(e1, F);
                EdgeCurve2DAdaptor pce2 = new(e2, F);
                Vector2D v1or, v2or;
                pce1.D1(t1, out _, out Vector2D v1);
                pce2.D1(t2, out _, out Vector2D v2);
                v1or = v1;
                v2or = v2;
                if (e1.Orientation() == Reversed)
                {
                    v1or = v1or.Reversed();
                }
                if (e2.Orientation() == Reversed)
                {
                    v2or = v2or.Reversed();
                }
                double crossProd = v2or ^ v1;
                // OCCT_DEBUG output left out
                if (crossProd > 0.0)
                {
                    oo1 = Forward;
                }
                crossProd = v1or ^ v2;
                if (crossProd > 0.0)
                {
                    oo2 = Forward;
                }
            }

            if (or1 != External)
            {
                oo1 = or1;
            }
            if (or2 != External)
            {
                oo2 = or2;
            }

            lv1.Append(newVertex.Oriented(oo1));
            lv2.Append(newVertex.Oriented(oo2));
        }

        //----------------------------------
        // Test at end.
        //---------------------------------
        double u1, u2;
        double tolConf = tol;
        Vertex[] v1v = [new(), new()], v2v = [new(), new()];
        Exploration.Vertices(e1, out v1v[0], out v1v[1]);
        Exploration.Vertices(e2, out v2v[0], out v2v[1]);

        int j;
        for (j = 0; j < 2; j++)
        {
            if (v1v[j].IsNull)
            {
                continue;
            }
            for (int k = 0; k < 2; k++)
            {
                if (v2v[k].IsNull)
                {
                    continue;
                }
                if (v1v[j].IsSame(v2v[k]))
                {
                    if (asDes.HasAscendant(v1v[j]))
                    {
                        continue;
                    }
                }
                //
                Point3D p1 = Tool.Pnt(v1v[j]);
                Point3D p2 = Tool.Pnt(v2v[k]);
                double dist = p1.Distance(p2);
                if (dist < tolConf)
                {
                    Vertex v = new LibraryMakeVertex(p1).InternalVertex();
                    u1 = (j == 0) ? f[1] : l[1];
                    u2 = (k == 0) ? f[2] : l[2];
                    Shape localShape = v.Oriented(Internal);
                    b.UpdateVertex(ShapeCasts.Vertex(localShape), u1, e1, tol);
                    b.UpdateVertex(ShapeCasts.Vertex(localShape), u2, e2, tol);
                    lv1.Prepend(v.Oriented(v1v[j].Orientation()));
                    lv2.Prepend(v.Oriented(v2v[k].Orientation()));
                }
            }
        }

        bool affichPurge = false;

        if (!lv1.IsEmpty())
        {
            //----------------------------------
            // Remove all vertices.
            // there can be doubles
            //----------------------------------
            NCollection_List<Shape>.Iterator it1LV1 = new(), it1LV2 = new(), it2LV1 = new();
            Point3D p1, p2;
            bool purge = true;

            while (purge)
            {
                i = 1;
                purge = false;
                for (it1LV1.Initialize(lv1), it1LV2.Initialize(lv2); it1LV1.More(); it1LV1.Next(), it1LV2.Next())
                {
                    j = 1;
                    it2LV1.Initialize(lv1);
                    while (j < i)
                    {
                        p1 = Tool.Pnt(ShapeCasts.Vertex(it1LV1.Value()));
                        p2 = Tool.Pnt(ShapeCasts.Vertex(it2LV1.Value()));
                        if (p1.IsEqual(p2, tol))
                        {
                            lv1.Remove(it1LV1);
                            lv2.Remove(it1LV2);
                            if (affichPurge)
                            {
                                Console.WriteLine("Doubles removed in EdgeInter.");
                            }
                            purge = true;
                            break;
                        }
                        j++;
                        it2LV1.Next();
                    }
                    if (purge)
                    {
                        break;
                    }
                    i++;
                }
            }
            //---------------------------------
            // Vertex storage in SD.
            //---------------------------------
            ////-----------------------------------------------------
            if (lv1.Extent() > 1)
            {
                // std::cout << "IFV - RefEdgeInter: remove vertex" << std::endl;
                Point3D pref = Tool.Pnt(vref);
                double dmin = RealLast();
                Vertex vmin = new();
                for (it1LV1.Initialize(lv1); it1LV1.More(); it1LV1.Next())
                {
                    Point3D p = Tool.Pnt(ShapeCasts.Vertex(it1LV1.Value()));
                    double d = p.SquareDistance(pref);
                    if (d < dmin)
                    {
                        dmin = d;
                        vmin = ShapeCasts.Vertex(it1LV1.Value());
                    }
                }
                for (it1LV1.Initialize(lv1), it1LV2.Initialize(lv2); it1LV1.More(); it1LV1.Next(), it1LV2.Next())
                {
                    if (!vmin.IsSame(it1LV1.Value()))
                    {
                        // OCCT: Remove moves the iterators to the next vertex and the loop's Next() then skips it,
                        // so a vertex following a removed one is kept
                        lv1.Remove(it1LV1);
                        lv2.Remove(it1LV2);
                        if (!it1LV1.More())
                        {
                            break;
                        }
                    }
                }
            }

            NCollection_List<Shape>.Iterator itl = new(lv1);
            for (; itl.More(); itl.Next())
            {
                Shape newVertex = itl.Value();
                newVertex.Orientation(Forward);
                if (imageVV.HasImage(vref))
                {
                    imageVV.Add(vref.Oriented(Forward), newVertex);
                }
                else
                {
                    imageVV.Bind(vref.Oriented(Forward), newVertex);
                }
            }

            ////-----------------------------------------------------
            double tolStore = Tool.Tolerance(e1) + Tool.Tolerance(e2);
            tolStore = Max(tolStore, tol);
            // Compare to Line-Line tolerance
            tolStore = Max(tolStore, tolLL);
            Store(e1, e2, lv1, lv2, tolStore, asDes, aDMVV);
        }
    }

    //======================================================================
    // function : EvaluateMaxSegment
    // purpose  : return MaxSegment to pass in approximation
    //======================================================================

    private static int EvaluateMaxSegment(CurveOnSurface aCurveOnSurface)
    {
        SurfaceAdaptorBase surf = aCurveOnSurface.GetSurface()!;
        Curve2DAdaptorBase curv2d = aCurveOnSurface.GetCurve()!;

        double nbSKnots = 0, nbC2dKnots = 0;

        if (surf.GetType() == SurfaceKind.BSplineSurface)
        {
            global::Kerf.Geometry.BSplineSurface bSpline = surf.BSpline();
            nbSKnots = Max(bSpline.UKnotCount, bSpline.VKnotCount);
        }
        if (curv2d.GetType() == BSplineCurve)
        {
            nbC2dKnots = curv2d.KnotCount;
        }
        int aReturn = (int)(30 + Max(nbSKnots, nbC2dKnots));
        return aReturn;
    }

    //=================================================================================================

    private static bool ExtendPCurve(Curve2D aPCurve,
                                     double anEf,
                                     double anEl,
                                     double a2Offset,
                                     out Curve2D? newPCurve)
    {
        newPCurve = aPCurve;
        if (newPCurve.GetType() == typeof(TrimmedCurve2D))
        {
            newPCurve = ((TrimmedCurve2D)newPCurve).BasisCurve();
        }

        double firstPar = newPCurve.FirstParameter();
        double lastPar = newPCurve.LastParameter();

        if (newPCurve is BoundedCurve2D && (firstPar > anEf - a2Offset || lastPar < anEl + a2Offset))
        {
            if (newPCurve.GetType() == typeof(BezierCurve2D))
            {
                BezierCurve2D bezier = (BezierCurve2D)newPCurve;
                if (bezier.PoleCount == 2)
                {
                    NCollection_Array1<Point2D> thePoles = bezier.InternalPoles();
                    Vector2D vec = new(thePoles[1], thePoles[2]);
                    newPCurve = new LineCurve2D(thePoles[1], new Direction2D(vec));
                    return true;
                }
            }
            else if (newPCurve.GetType() == typeof(BSplineCurve2D))
            {
                BSplineCurve2D bSpline = (BSplineCurve2D)newPCurve;
                if (bSpline.KnotCount == 2 && bSpline.PoleCount == 2)
                {
                    NCollection_Array1<Point2D> thePoles = bSpline.InternalPoles();
                    Vector2D vec = new(thePoles[1], thePoles[2]);
                    newPCurve = new LineCurve2D(thePoles[1], new Direction2D(vec));
                    return true;
                }
            }
        }

        firstPar = aPCurve.FirstParameter();
        lastPar = aPCurve.LastParameter();
        TrimmedCurve2D trCurve = new(aPCurve, firstPar, lastPar);

        // The curve is not prolonged on begin or end.
        // Trying to prolong it adding a segment to its bound.
        Point2D pBnd;
        Vector2D vBnd;
        Point2D pBeg;
        Direction2D dBnd = new();
        LineCurve2D lin;
        TrimmedCurve2D segment;
        CompCurveToBSplineCurve2D compCurve = new(trCurve, RationalC1);
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double delta = Max(a2Offset, 1.0);

        if (firstPar > anEf - a2Offset)
        {
            aPCurve.D1(firstPar, out pBnd, out vBnd);
            dBnd = dBnd.WithXY(vBnd.XY());
            pBeg = pBnd.Translated(new Vector2D(-delta * dBnd.XY()));
            lin = new LineCurve2D(pBeg, dBnd);
            segment = new TrimmedCurve2D(lin, 0, delta);

            if (!compCurve.Add(segment, tol))
            {
                return false;
            }
        }

        if (lastPar < anEl + a2Offset)
        {
            aPCurve.D1(lastPar, out pBeg, out vBnd);
            dBnd = dBnd.WithXY(vBnd.XY());
            lin = new LineCurve2D(pBeg, dBnd);
            segment = new TrimmedCurve2D(lin, 0, delta);

            if (!compCurve.Add(segment, tol))
            {
                return false;
            }
        }

        newPCurve = compCurve.BSplineCurve();
        return true;
    }

    //=================================================================================================

    //  Modified by skv - Fri Dec 26 17:00:55 2003 OCC4455 Begin
    // static void ExtentEdge(const TopoDS_Edge& E,TopoDS_Edge& NE)

    /// <summary>
    /// Extends the edge <paramref name="e"/> into a new edge <paramref name="ne"/> (with copies of its curve
    /// representations): its pcurves and 3D curve are prolonged by twice <paramref name="offset"/> where they are
    /// bounded, within the bounds of their surfaces. False if a pcurve couldn't be computed on the extended range.
    /// </summary>
    /// <remarks><paramref name="e"/> is only read. The new edge is not frozen.</remarks>
    public static bool ExtentEdge(in Edge e, out Edge ne, double offset)
    {
        // BRepLib::BuildCurve3d(E);

        Shape localShape = e.EmptyCopied();
        double a2Offset = 2.0 * Math.Abs(offset);
        BrepBuilder bb = new();
        int i, j;

        Tool.Range(e, out double ef, out double el);
        ne = ShapeCasts.Edge(localShape);
        //  NE = TopoDS::Edge(E.EmptyCopied());
        // Enough for analytic edges, for general case reconstruct the
        // geometry of the edge recalculating the intersection of surfaces.

        // BRepLib::BuildCurve3d(E);

        int nbPCurves = 0;
        double firstParOnPC = RealFirst(), lastParOnPC = RealLast();
        Curve2D? minPC = null;
        Surface? minSurf = null;
        Placement minLoc = new();

        NCollection_List<CurveRepresentation>.Iterator itr = new(((BrepTEdge)ne.TShape()!).ChangeCurves());
        for (; itr.More(); itr.Next())
        {
            CurveRepresentation curveRep = itr.Value();
            double firstPar, lastPar;
            if (curveRep.IsCurveOnSurface())
            {
                nbPCurves++;
                Curve2D theCurve = curveRep.PCurve()!;
                firstPar = theCurve.FirstParameter();
                lastPar = theCurve.LastParameter();

                if (theCurve is BoundedCurve2D && (firstPar > ef - a2Offset || lastPar < el + a2Offset))
                {
                    if (ExtendPCurve(theCurve, ef, el, a2Offset, out Curve2D? newPCurve))
                    {
                        curveRep.PCurve(newPCurve);
                        firstPar = newPCurve!.FirstParameter();
                        lastPar = newPCurve.LastParameter();
                        if (curveRep.IsCurveOnClosedSurface)
                        {
                            Curve2D pCurve2 = curveRep.PCurve2()!;
                            if (ExtendPCurve(pCurve2, ef, el, a2Offset, out newPCurve))
                            {
                                curveRep.PCurve2(newPCurve);
                            }
                        }
                    }
                }
                else if (theCurve.IsPeriodic)
                {
                    double delta = (theCurve.Period() - (el - ef)) * 0.5;
                    delta *= 0.95;
                    firstPar = ef - delta;
                    lastPar = el + delta;
                }
                else if (theCurve.IsClosed)
                {
                    lastPar -= 0.05 * (lastPar - firstPar);
                }

                // check FirstPar and LastPar: the pcurve should be in its surface
                theCurve = curveRep.PCurve()!;
                Surface theSurf = curveRep.Surface()!;
                theSurf.Bounds(out double umin, out double umax, out double vmin, out double vmax);
                NCollection_Sequence<Curve2D> boundLines = new();
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vmin))
                {
                    LineCurve2D line = new(new Point2D(0.0, vmin), new Direction2D(Direction2D.D.X));
                    boundLines.Append(line);
                }
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(umin))
                {
                    LineCurve2D line = new(new Point2D(umin, 0.0), new Direction2D(Direction2D.D.Y));
                    boundLines.Append(line);
                }
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(vmax))
                {
                    LineCurve2D line = new(new Point2D(0.0, vmax), new Direction2D(Direction2D.D.X));
                    boundLines.Append(line);
                }
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(umax))
                {
                    LineCurve2D line = new(new Point2D(umax, 0.0), new Direction2D(Direction2D.D.Y));
                    boundLines.Append(line);
                }

                NCollection_Sequence<double> params_ = new();
                GInter intCC = new();
                CurveAdaptor2D gAcurve = new(theCurve);
                for (i = 1; i <= boundLines.Length(); i++)
                {
                    CurveAdaptor2D gAline = new(boundLines[i]);
                    intCC.Perform(gAcurve, gAline, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PConfusion());
                    if (intCC.IsDone)
                    {
                        for (j = 1; j <= intCC.PointCount; j++)
                        {
                            IntersectionPoint2D ip = intCC.Point(j);
                            Point2D point = ip.Value();
                            if (point.X() >= umin && point.X() <= umax && point.Y() >= vmin && point.Y() <= vmax)
                            {
                                params_.Append(ip.ParamOnFirst());
                            }
                        }
                        for (j = 1; j <= intCC.SegmentCount; j++)
                        {
                            IntersectionSegment2D is_ = intCC.Segment(j);
                            if (is_.HasFirstPoint)
                            {
                                IntersectionPoint2D ip = is_.FirstPoint();
                                Point2D point = ip.Value();
                                if (point.X() >= umin && point.X() <= umax && point.Y() >= vmin
                                    && point.Y() <= vmax)
                                {
                                    params_.Append(ip.ParamOnFirst());
                                }
                            }
                            if (is_.HasLastPoint)
                            {
                                IntersectionPoint2D ip = is_.LastPoint();
                                Point2D point = ip.Value();
                                if (point.X() >= umin && point.X() <= umax && point.Y() >= vmin
                                    && point.Y() <= vmax)
                                {
                                    params_.Append(ip.ParamOnFirst());
                                }
                            }
                        }
                    }
                }
                if (!params_.IsEmpty())
                {
                    if (params_.Length() == 1)
                    {
                        Point2D pntFirst = theCurve.Value(firstPar);
                        if (pntFirst.X() >= umin && pntFirst.X() <= umax && pntFirst.Y() >= vmin
                            && pntFirst.Y() <= vmax)
                        {
                            if (lastPar > params_[1])
                            {
                                lastPar = params_[1];
                            }
                        }
                        else if (firstPar < params_[1])
                        {
                            firstPar = params_[1];
                        }
                    }
                    else
                    {
                        double fpar = RealLast(), lpar = RealFirst();
                        for (i = 1; i <= params_.Length(); i++)
                        {
                            if (params_[i] < fpar)
                            {
                                fpar = params_[i];
                            }
                            if (params_[i] > lpar)
                            {
                                lpar = params_[i];
                            }
                        }
                        if (firstPar < fpar)
                        {
                            firstPar = fpar;
                        }
                        if (lastPar > lpar)
                        {
                            lastPar = lpar;
                        }
                    }
                }
                //// end of check ////
                ((GCurve)curveRep).SetRange(firstPar, lastPar);
                // gp_Pnt2d Pfirst = theCurve->Value(FirstPar);
                // gp_Pnt2d Plast  = theCurve->Value(LastPar);
                //(occ::down_cast<BRep_CurveOnSurface>(CurveRep))->SetUVPoints( Pfirst, Plast );

                // update FirstParOnPC and LastParOnPC
                if (firstPar > firstParOnPC)
                {
                    firstParOnPC = firstPar;
                    minPC = theCurve;
                    minSurf = theSurf;
                    minLoc = curveRep.Location();
                }
                if (lastPar < lastParOnPC)
                {
                    lastParOnPC = lastPar;
                    minPC = theCurve;
                    minSurf = theSurf;
                    minLoc = curveRep.Location();
                }
            }
        }

        Curve3D? c3d = Tool.Curve(ne, out double f, out double l);
        if (nbPCurves != 0)
        {
            minLoc = e.Location() * minLoc;
            if (c3d is not null)
            {
                if (minPC!.IsClosed)
                {
                    f = firstParOnPC;
                    l = lastParOnPC;
                }
                else if (c3d.IsPeriodic)
                {
                    double delta = (c3d.Period() - (l - f)) * 0.5;
                    delta *= 0.95;
                    f -= delta;
                    l += delta;
                }
                else if (c3d.IsClosed)
                {
                    l -= 0.05 * (l - f);
                }
                else
                {
                    f = firstParOnPC;
                    l = lastParOnPC;
                    Transform3D minLocTrsf = minLoc.Transformation();
                    ProjectPointOnCurve projector = new();
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(firstParOnPC))
                    {
                        Point2D p2d1 = minPC.Value(firstParOnPC);
                        Point3D p1 = minSurf!.Value(p2d1.X(), p2d1.Y());
                        p1 = p1.Transformed(minLocTrsf);
                        projector.Init(p1, c3d);
                        if (projector.NbPoints() > 0)
                        {
                            f = projector.LowerDistanceParameter();
                        }
                        // OCCT_DEBUG output left out
                    }
                    if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(lastParOnPC))
                    {
                        Point2D p2d2 = minPC.Value(lastParOnPC);
                        Point3D p2 = minSurf!.Value(p2d2.X(), p2d2.Y());
                        p2 = p2.Transformed(minLocTrsf);
                        projector.Init(p2, c3d);
                        if (projector.NbPoints() > 0)
                        {
                            l = projector.LowerDistanceParameter();
                        }
                        // OCCT_DEBUG output left out
                    }
                }
                bb.Range(ne, f, l);
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(f) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
                {
                    BrepLibrary.SameParameter(ne, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                }
            }
            else if (!Tool.Degenerated(e)) // no 3d curve
            {
                minSurf = (Surface)minSurf!.Transformed(minLoc.Transformation());
                double max_deviation = 0.0;
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(firstParOnPC) || global::Kerf.Engine.Foundation.Precision.IsInfinite(lastParOnPC))
                {
                    if (minPC!.GetType() == typeof(LineCurve2D))
                    {
                        bool isLine = false;
                        if (minSurf.GetType() == typeof(PlaneSurface))
                        {
                            isLine = true;
                        }
                        else if (minSurf.GetType() == typeof(CylindricalSurface)
                                 || minSurf.GetType() == typeof(ConicalSurface))
                        {
                            LineCurve2D theLine = (LineCurve2D)minPC;
                            Direction2D lineDir = theLine.Direction();
                            if (lineDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                isLine = true;
                            }
                        }
                        if (isLine)
                        {
                            Point2D p2d1 = minPC.Value(0.0), p2d2 = minPC.Value(1.0);
                            Point3D p1 = minSurf.Value(p2d1.X(), p2d1.Y());
                            Point3D p2 = minSurf.Value(p2d2.X(), p2d2.Y());
                            Vector3D vec = new(p1, p2);
                            c3d = new LineCurve3D(p1, new Direction3D(vec));
                        }
                    }
                }
                else
                {
                    // OCCT copies the adaptors into handles; the copies are the adaptors themselves here
                    CurveAdaptor2D ac2d = new(minPC!, firstParOnPC, lastParOnPC);
                    SurfaceAdaptor gAsurf = new(minSurf);
                    CurveAdaptor2D hc2d = ac2d;
                    SurfaceAdaptor hSurf = gAsurf;
                    CurveOnSurface conS = new(hc2d, hSurf);
                    Continuity continuity = C1;
                    int maxDegree = 14;
                    int maxSegment = EvaluateMaxSegment(conS);
                    global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(global::Kerf.Engine.Foundation.Precision.Confusion(),
                                         conS,
                                         firstParOnPC,
                                         lastParOnPC,
                                         ref c3d,
                                         out max_deviation,
                                         out _,
                                         continuity,
                                         maxDegree,
                                         maxSegment);
                }
                bb.UpdateEdge(ne, c3d, max_deviation);
                // BB.Range( NE, FirstParOnPC, LastParOnPC );
                bool projectionSuccess = true;
                if (nbPCurves > 1)
                {
                    // BRepLib::SameParameter( NE, Precision::Confusion(), true );
                    for (itr.Initialize(((BrepTEdge)ne.TShape()!).ChangeCurves()); itr.More(); itr.Next())
                    {
                        CurveRepresentation curveRep = itr.Value();
                        double firstPar, lastPar;
                        if (curveRep.IsCurveOnSurface())
                        {
                            Curve2D theCurve = curveRep.PCurve()!;
                            Surface theSurf = curveRep.Surface()!;
                            Placement theLoc = curveRep.Location();
                            // OCCT: MinSurf was replaced above by its transformed copy, so this test never holds and
                            // the pcurve the 3D curve was built from is checked (and maybe projected) as well
                            if (theCurve == minPC && theSurf == minSurf && theLoc == minLoc)
                            {
                                continue;
                            }
                            firstPar = ((GCurve)curveRep).First();
                            lastPar = ((GCurve)curveRep).Last();
                            if (Math.Abs(firstPar - firstParOnPC) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                                || Math.Abs(lastPar - lastParOnPC) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                theLoc = e.Location() * theLoc;
                                theSurf = (Surface)theSurf.Transformed(theLoc.Transformation());

                                if (theCurve.GetType() == typeof(LineCurve2D) && theSurf is BoundedSurface)
                                {
                                    Direction2D theDir = ((LineCurve2D)theCurve).Direction();
                                    if (theDir.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular())
                                        || theDir.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                                    {
                                        theSurf.Bounds(out double u1, out double u2, out double v1, out double v2);
                                        Point2D origin = ((LineCurve2D)theCurve).Location();
                                        if (Math.Abs(origin.X() - u1) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                            || Math.Abs(origin.X() - u2) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                            || Math.Abs(origin.Y() - v1) <= global::Kerf.Engine.Foundation.Precision.Confusion()
                                            || Math.Abs(origin.Y() - v2) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                                        {
                                            BrepLibrary.SameParameter(ne, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                                            break;
                                        }
                                    }
                                }
                                if (c3d is not null && firstParOnPC < lastParOnPC)
                                {
                                    Curve2D? projPCurve =
                                        global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c3d, firstParOnPC, lastParOnPC, theSurf);
                                    if (projPCurve is null)
                                    {
                                        projectionSuccess = false;
                                    }
                                    else
                                    {
                                        curveRep.PCurve(projPCurve);
                                    }
                                }
                                else
                                {
                                    return false;
                                }
                            }
                        }
                    }
                }
                if (projectionSuccess)
                {
                    bb.Range(ne, firstParOnPC, lastParOnPC);
                }
                else
                {
                    bb.Range(ne, firstParOnPC, lastParOnPC, true);
                    BrepLibrary.SameParameter(ne, global::Kerf.Engine.Foundation.Precision.Confusion(), true);
                }
            }
        }
        else // no pcurves
        {
            double firstPar = c3d!.FirstParameter();
            double lastPar = c3d.LastParameter();

            if (c3d is BoundedCurve3D && (firstPar > ef - a2Offset || lastPar < el + a2Offset))
            {
                TrimmedCurve3D trCurve = new(c3d, firstPar, lastPar);

                // The curve is not prolonged on begin or end.
                // Trying to prolong it adding a segment to its bound.
                Point3D pBnd;
                Vector3D vBnd;
                Point3D pBeg;
                Direction3D dBnd = new();
                LineCurve3D lin;
                TrimmedCurve3D segment;
                CompCurveToBSplineCurve compCurve = new(trCurve, RationalC1);
                double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                double aDelta = Max(a2Offset, 1.0);

                if (firstPar > ef - a2Offset)
                {
                    c3d.D1(firstPar, out pBnd, out vBnd);
                    dBnd = dBnd.WithXYZ(vBnd.XYZ());
                    pBeg = pBnd.Translated(new Vector3D(-aDelta * dBnd.XYZ()));
                    lin = new LineCurve3D(pBeg, dBnd);
                    segment = new TrimmedCurve3D(lin, 0, aDelta);

                    if (!compCurve.Add(segment, tol))
                    {
                        return true;
                    }
                }

                if (lastPar < el + a2Offset)
                {
                    c3d.D1(lastPar, out pBeg, out vBnd);
                    dBnd = dBnd.WithXYZ(vBnd.XYZ());
                    lin = new LineCurve3D(pBeg, dBnd);
                    segment = new TrimmedCurve3D(lin, 0, aDelta);

                    if (!compCurve.Add(segment, tol))
                    {
                        return true;
                    }
                }

                c3d = compCurve.BSplineCurve()!;
                firstPar = c3d.FirstParameter();
                lastPar = c3d.LastParameter();
                bb.UpdateEdge(ne, c3d, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else if (c3d.IsPeriodic)
            {
                double delta = (c3d.Period() - (el - ef)) * 0.5;
                delta *= 0.95;
                firstPar = ef - delta;
                lastPar = el + delta;
            }
            else if (c3d.IsClosed)
            {
                lastPar -= 0.05 * (lastPar - firstPar);
            }

            bb.Range(ne, firstPar, lastPar);
        }
        return true;
    }

    //  Modified by skv - Fri Dec 26 17:00:57 2003 OCC4455 End

    //=================================================================================================

    private static bool UpdateVertex(in Vertex v, in Edge oe, in Edge ne, double tolConf)
    {
        EdgeCurveAdaptor oc = new(oe);
        EdgeCurveAdaptor nc = new(ne);
        double of = oc.FirstParameter();
        double ol = oc.LastParameter();
        double nf = nc.FirstParameter();
        double nl = nc.LastParameter();
        double u = 0.0;
        double parTol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Point3D p = Tool.Pnt(v);
        bool ok = false;

        if (p.Distance(oc.Value(of)) < tolConf)
        {
            if (of >= nf + parTol && of <= nl + parTol && p.Distance(nc.Value(of)) < tolConf)
            {
                ok = true;
                u = of;
            }
        }
        if (p.Distance(oc.Value(ol)) < tolConf)
        {
            if (ol >= nf + parTol && ol <= nl + parTol && p.Distance(nc.Value(ol)) < tolConf)
            {
                ok = true;
                u = ol;
            }
        }
        if (ok)
        {
            BrepBuilder b = new();
            // OCCT also makes the forward edge EE here, unused
            //    TopoDS_Edge EE = TopoDS::Edge(NE.Oriented(TopAbs_FORWARD));
            Shape localShape = v.Oriented(Internal);
            b.UpdateVertex(ShapeCasts.Vertex(localShape), u, ne, Tool.Tolerance(ne));
            //    B.UpdateVertex(TopoDS::Vertex(V.Oriented(TopAbs_INTERNAL)),
            //                   U,NE,BRep_Tool::Tolerance(NE));
        }
        return ok;
    }

    //=================================================================================================

    /// <summary>
    /// Computes the intersections between the edges stored in <paramref name="asDes"/> as descendants of
    /// <paramref name="f"/>: two edges are intersected if one of them is in <paramref name="newEdges"/>. When all faces
    /// of the shape are treated, the intersection vertices have to be fused with <see cref="FuseVertices"/>;
    /// <paramref name="dmvv"/> collects the vertices to fuse.
    /// </summary>
    public static void Compute(AsDes asDes,
                               in Face f,
                               IndexedSet<Shape> newEdges,
                               double tol,
                               Dictionary<Shape, NCollection_List<Shape>> edgeIntEdges,
                               IndexedMap<Shape, NCollection_List<Shape>> dmvv,
                               ProgressRange range = default)
    {

        // Do not intersect the edges of face
        HashSet<Shape> edgesOfFace = new(ShapeHasher.Instance);
        Explorer explo = new(f, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            edgesOfFace.Add(explo.Current());
        }

        //-----------------------------------------------------------
        // calculate intersections2d on faces touched by
        // intersection3d
        //---------------------------------------------------------
        NCollection_List<Shape>.Iterator it1LE = new();
        NCollection_List<Shape>.Iterator it2LE = new();

        //-----------------------------------------------
        // Intersection of edges 2*2.
        //-----------------------------------------------
        NCollection_List<Shape> le = asDes.Descendant(f);
        int j, i = 1;
        FaceSurfaceAdaptor bAsurf = new(f);
        //
        using ProgressScope ps = new(range, "Intersecting edges on faces", le.Extent());
        for (it1LE.Initialize(le); it1LE.More(); it1LE.Next(), ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Edge e1 = ShapeCasts.Edge(it1LE.Value());
            j = 1;
            it2LE.Initialize(le);

            while (j < i && it2LE.More())
            {
                Edge e2 = ShapeCasts.Edge(it2LE.Value());

                bool toIntersect = true;
                if (edgeIntEdges.ContainsKey(e1))
                {
                    NCollection_List<Shape> elist = edgeIntEdges.GetValue(e1);
                    NCollection_List<Shape>.Iterator itedges = new(elist);
                    for (; itedges.More(); itedges.Next())
                    {
                        if (e2.IsSame(itedges.Value()))
                        {
                            toIntersect = false;
                        }
                    }

                    if (toIntersect)
                    {
                        for (itedges.Initialize(elist); itedges.More(); itedges.Next())
                        {
                            Shape edge = itedges.Value();
                            if (edgeIntEdges.ContainsKey(edge))
                            {
                                NCollection_List<Shape> elist2 = edgeIntEdges.GetValue(edge);
                                NCollection_List<Shape>.Iterator itedges2 = new(elist2);
                                for (; itedges2.More(); itedges2.Next())
                                {
                                    if (e2.IsSame(itedges2.Value()))
                                    {
                                        toIntersect = false;
                                    }
                                }
                            }
                        }
                    }
                }

                //--------------------------------------------------------------
                // Intersections of New edges obtained by intersection
                // between them and with edges of restrictions
                //------------------------------------------------------
                if (toIntersect && (!edgesOfFace.Contains(e1) || !edgesOfFace.Contains(e2))
                    && (newEdges.Contains(e1) || newEdges.Contains(e2)))
                {

                    Shape localShape = f.Oriented(Forward);
                    EdgeInter(ShapeCasts.Face(localShape), bAsurf, e1, e2, asDes, tol, true, dmvv);
                    //          EdgeInter(TopoDS::Face(F.Oriented(TopAbs_FORWARD)),E1,E2,AsDes,Tol,true);
                }
                it2LE.Next();
                j++;
            }
            i++;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Computes the intersections between the offset edges of the face <paramref name="fi"/> (whose offset is
    /// <paramref name="ofi"/>); the intersection vertices are stored in <paramref name="asDes2d"/>. When all faces of
    /// the shape are treated, they have to be fused with <see cref="FuseVertices"/>; <paramref name="dmvv"/>
    /// collects the vertices to fuse. False if an edge couldn't be extended or the computation was stopped.
    /// </summary>
    public static bool ConnexIntByInt(in Face fi,
                                      Offset ofi,
                                      Dictionary<Shape, Shape> mes,
                                      Dictionary<Shape, Shape> build,
                                      AsDes asDes,
                                      AsDes asDes2d,
                                      double offset,
                                      double tol,
                                      Analyse analyse,
                                      IndexedSet<Shape> facesWithVerts,
                                      ShapeImage imageVV,
                                      Dictionary<Shape, NCollection_List<Shape>> edgeIntEdges,
                                      IndexedMap<Shape, NCollection_List<Shape>> dmvv,
                                      ProgressRange range = default)
    {

        Dictionary<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance);
        OffsetTool.MapVertexEdges(fi, mve);
        using ProgressScope ps = new(range, "Intersecting edges obtained as intersection of faces", 1, true);
        //---------------------
        // Extension of edges.
        //---------------------
        Edge ne;
        // OCCT.NET: MVE is keyed by shape and iterates in insertion order (OCCT's in hash order); the order only
        // decides the order in which the edges are extended and bound in MES
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in mve)
        {
            if (!ps.More())
            {
                return false;
            }
            NCollection_List<Shape> l = it.Value;
            bool yaBuild = false;
            NCollection_List<Shape>.Iterator itL = new(l);
            for (; itL.More(); itL.Next())
            {
                yaBuild = build.ContainsKey(itL.Value());
                if (yaBuild)
                {
                    break;
                }
            }
            if (yaBuild)
            {
                for (itL.Initialize(l); itL.More(); itL.Next())
                {
                    Edge ei = ShapeCasts.Edge(itL.Value());
                    if (ei.Orientation() != Forward && ei.Orientation() != Reversed)
                    {
                        continue;
                    }
                    Shape localShape = ofi.InternalGenerated(ei);
                    Edge oe = ShapeCasts.Edge(localShape);
                    if (!mes.ContainsKey(oe) && !build.ContainsKey(ei))
                    {
                        if (!ExtentEdge(oe, out ne, offset))
                        {
                            return false;
                        }
                        mes[oe] = ne;
                    }
                }
            }
        }

        Face fio = ofi.InternalFace();
        if (mes.ContainsKey(fio))
        {
            fio = ShapeCasts.Face(mes.GetValue(fio));
        }
        //
        FaceSurfaceAdaptor bAsurf = new(fio);

        Explorer exp = new(fi.Oriented(Forward), ShapeType.Wire);
        for (; exp.More(); exp.Next(), ps.Next())
        {
            if (!ps.More())
            {
                return false;
            }
            Wire w = ShapeCasts.Wire(exp.Current());
            WireExplorer wexp = new();
            bool end = false;
            Edge firstE, curE, nextE;

            Shape localWire = w.Oriented(Forward);
            Shape localFace = fi.Oriented(Forward);
            wexp.Init(ShapeCasts.Wire(localWire), ShapeCasts.Face(localFace));
            if (!wexp.More())
            {
                continue; // Protection from case when explorer does not contain edges.
            }
            curE = firstE = wexp.Current();
            // OCCT declares an unused indexed map of edges here

            while (!end)
            {
                wexp.Next();
                if (wexp.More())
                {
                    nextE = wexp.Current();
                }
                else
                {
                    nextE = firstE;
                    end = true;
                }
                if (curE.IsSame(nextE))
                {
                    continue;
                }

                Vertex vref = CommonVertex(curE, nextE);

                curE = analyse.EdgeReplacement(fi, curE);
                nextE = analyse.EdgeReplacement(fi, nextE);

                Shape localShape = ofi.InternalGenerated(curE);
                Edge ceo = ShapeCasts.Edge(localShape);
                localShape = ofi.InternalGenerated(nextE);
                Edge neo = ShapeCasts.Edge(localShape);
                //------------------------------------------
                // Inter processing of images of CurE NextE.
                //------------------------------------------
                bool doInter = true;
                Shape ne1 = new(), ne2 = new();
                NCollection_Sequence<Shape> ne1seq = new(), ne2seq = new();
                Orientation or1 = External, or2 = External;

                int choice = 0;
                if (build.ContainsKey(curE) && build.ContainsKey(nextE))
                {
                    choice = 1;
                    ne1 = build.GetValue(curE);
                    ne2 = build.GetValue(nextE);
                    GetEdgesOrientedInFace(ne1, fio, asDes, ne1seq);
                    GetEdgesOrientedInFace(ne2, fio, asDes, ne2seq);
                    or1 = Reversed;
                    or2 = Forward;
                }
                else if (build.ContainsKey(curE) && mes.ContainsKey(neo))
                {
                    choice = 2;
                    ne1 = build.GetValue(curE);
                    ne2 = mes.GetValue(neo);
                    ne2.Orientation(nextE.Orientation());
                    GetEdgesOrientedInFace(ne1, fio, asDes, ne1seq);
                    ne2seq.Append(ne2);
                    or1 = Reversed;
                    or2 = Forward;
                }
                else if (build.ContainsKey(nextE) && mes.ContainsKey(ceo))
                {
                    choice = 3;
                    ne1 = build.GetValue(nextE);
                    ne2 = mes.GetValue(ceo);
                    ne2.Orientation(curE.Orientation());
                    GetEdgesOrientedInFace(ne1, fio, asDes, ne1seq);
                    ne2seq.Append(ne2);
                    or1 = Forward;
                    or2 = Reversed;
                }
                else
                {
                    doInter = false;
                }
                if (doInter)
                {
                    //------------------------------------
                    // NE1,NE2 can be a compound of Edges.
                    //------------------------------------
                    Edge e1, e2;
                    if (choice == 1 || choice == 2)
                    {
                        e1 = ShapeCasts.Edge(ne1seq.Last());
                        e2 = ShapeCasts.Edge(ne2seq.First());
                    }
                    else // aChoice == 3
                    {
                        e1 = ShapeCasts.Edge(ne1seq.First());
                        e2 = ShapeCasts.Edge(ne2seq.Last());
                    }

                    if (e1.Orientation() == Reversed)
                    {
                        or1 = global::Kerf.Engine.Topology.TopAbs.Reverse(or1);
                    }
                    if (e2.Orientation() == Reversed)
                    {
                        or2 = global::Kerf.Engine.Topology.TopAbs.Reverse(or2);
                    }

                    RefEdgeInter(fio,
                                 bAsurf,
                                 e1,
                                 e2,
                                 or1,
                                 or2,
                                 asDes2d,
                                 tol,
                                 true,
                                 vref,
                                 imageVV,
                                 dmvv,
                                 out _);

                    if (edgeIntEdges.ContainsKey(e1))
                    {
                        edgeIntEdges.GetValue(e1).Append(e2);
                    }
                    else
                    {
                        NCollection_List<Shape> elist = new();
                        elist.Append(e2);
                        edgeIntEdges[e1] = elist;
                    }
                    if (edgeIntEdges.ContainsKey(e2))
                    {
                        edgeIntEdges.GetValue(e2).Append(e1);
                    }
                    else
                    {
                        NCollection_List<Shape> elist = new();
                        elist.Append(e1);
                        edgeIntEdges[e2] = elist;
                    }

                    //
                    // check if some of the offset edges have been
                    // generated out of the common vertex
                    if (build.ContainsKey(vref))
                    {
                        facesWithVerts.Add(fi);
                    }
                }
                else
                {
                    Vertex v = CommonVertex(ceo, neo);
                    if (!v.IsNull)
                    {
                        if (mes.ContainsKey(ceo))
                        {
                            UpdateVertex(v, ceo, ShapeCasts.Edge(mes.GetValue(ceo)), tol);
                            asDes2d.Add(mes.GetValue(ceo), v);
                        }
                        if (mes.ContainsKey(neo))
                        {
                            UpdateVertex(v, neo, ShapeCasts.Edge(mes.GetValue(neo)), tol);
                            asDes2d.Add(mes.GetValue(neo), v);
                        }
                    }
                }
                curE = wexp.Current();
            }
        }
        return true;
    }

    //=======================================================================
    // function : ConnexIntByIntInVert
    // purpose  : Intersection of the edges generated out of vertices
    //=======================================================================

    /// <summary>
    /// Computes the intersections between the offset edges generated from vertices and stored in
    /// <paramref name="asDes"/> as descendants of the offset of <paramref name="fi"/>; the intersection vertices are
    /// stored in <paramref name="asDes2d"/>. When all faces of the shape are treated, they have to be fused with
    /// <see cref="FuseVertices"/>; <paramref name="dmvv"/> collects the vertices to fuse.
    /// </summary>
    public static void ConnexIntByIntInVert(in Face fi,
                                            Offset ofi,
                                            Dictionary<Shape, Shape> mes,
                                            Dictionary<Shape, Shape> build,
                                            AsDes asDes,
                                            AsDes asDes2d,
                                            double tol,
                                            Analyse analyse,
                                            IndexedMap<Shape, NCollection_List<Shape>> dmvv,
                                            ProgressRange range = default)
    {
        Face fio = ofi.InternalFace();
        if (mes.ContainsKey(fio))
        {
            fio = ShapeCasts.Face(mes.GetValue(fio));
        }
        //
        HashSet<Shape> me = new(ShapeHasher.Instance);
        NCollection_List<Shape> le = asDes.Descendant(fio);
        NCollection_List<Shape>.Iterator itLE = new(le);
        for (; itLE.More(); itLE.Next())
        {
            Shape e = itLE.Value();
            me.Add(e);
        }
        //
        FaceSurfaceAdaptor bAsurf = new(fio);
        //
        using ProgressScope ps = new(range, "Intersecting edges created from vertices", 1, true);
        Explorer exp = new(fi.Oriented(Forward), ShapeType.Wire);
        for (; exp.More(); exp.Next(), ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Wire w = ShapeCasts.Wire(exp.Current());
            //
            WireExplorer wexp = new();
            bool end = false;
            Edge firstE, curE, nextE;
            //
            Shape localWire = w.Oriented(Forward);
            Shape localFace = fi.Oriented(Forward);
            wexp.Init(ShapeCasts.Wire(localWire), ShapeCasts.Face(localFace));
            if (!wexp.More())
            {
                continue; // Protection from case when explorer does not contain edges.
            }
            //
            curE = firstE = wexp.Current();
            while (!end)
            {
                wexp.Next();
                if (wexp.More())
                {
                    nextE = wexp.Current();
                }
                else
                {
                    nextE = firstE;
                    end = true;
                }
                if (curE.IsSame(nextE))
                {
                    continue;
                }
                //
                Vertex vref = CommonVertex(curE, nextE);
                if (!build.ContainsKey(vref))
                {
                    curE = nextE;
                    continue;
                }

                curE = analyse.EdgeReplacement(fi, curE);
                nextE = analyse.EdgeReplacement(fi, nextE);

                Shape localShape = ofi.InternalGenerated(curE);
                Edge ceo = ShapeCasts.Edge(localShape);
                localShape = ofi.InternalGenerated(nextE);
                Edge neo = ShapeCasts.Edge(localShape);
                //
                Shape ne1, ne2;
                Orientation or1 = External, or2 = External;

                if (build.ContainsKey(curE) && build.ContainsKey(nextE))
                {
                    ne1 = build.GetValue(curE);
                    ne2 = build.GetValue(nextE);
                }
                else if (build.ContainsKey(curE) && mes.ContainsKey(neo))
                {
                    ne1 = build.GetValue(curE);
                    ne2 = mes.GetValue(neo);
                }
                else if (build.ContainsKey(nextE) && mes.ContainsKey(ceo))
                {
                    ne1 = build.GetValue(nextE);
                    ne2 = mes.GetValue(ceo);
                }
                else
                {
                    curE = wexp.Current();
                    continue;
                }
                //
                Explorer exp1 = new(), exp2 = new();
                bool bCoincide;
                // intersect edges generated from vertex with the edges of the face
                Shape ne3 = build.GetValue(vref);
                //
                for (exp2.Init(ne3, ShapeType.Edge); exp2.More(); exp2.Next())
                {
                    Edge e3 = ShapeCasts.Edge(exp2.Current());
                    if (!me.Contains(e3))
                    {
                        continue;
                    }
                    //
                    // intersection with first edge
                    for (exp1.Init(ne1, ShapeType.Edge); exp1.More(); exp1.Next())
                    {
                        Edge e1 = ShapeCasts.Edge(exp1.Current());
                        ShapeImage emptyImage = new();
                        RefEdgeInter(fio,
                                     bAsurf,
                                     e1,
                                     e3,
                                     or1,
                                     or2,
                                     asDes2d,
                                     tol,
                                     true,
                                     vref,
                                     emptyImage,
                                     dmvv,
                                     out bCoincide);
                        if (bCoincide)
                        {
                            // in case of coincidence trim the edge E3 the same way as E1
                            Store(e3, asDes2d.Descendant(e1), tol, true, asDes2d, dmvv);
                        }
                    }
                    //
                    // intersection with second edge
                    for (exp1.Init(ne2, ShapeType.Edge); exp1.More(); exp1.Next())
                    {
                        Edge e2 = ShapeCasts.Edge(exp1.Current());
                        ShapeImage emptyImage = new();
                        RefEdgeInter(fio,
                                     bAsurf,
                                     e2,
                                     e3,
                                     or1,
                                     or2,
                                     asDes2d,
                                     tol,
                                     true,
                                     vref,
                                     emptyImage,
                                     dmvv,
                                     out bCoincide);
                        if (bCoincide)
                        {
                            // in case of coincidence trim the edge E3 the same way as E2
                            Store(e3, asDes2d.Descendant(e2), tol, true, asDes2d, dmvv);
                        }
                    }
                    //
                    // intersection of the edges generated from vertex
                    // among themselves
                    for (exp1.Init(ne3, ShapeType.Edge); exp1.More(); exp1.Next())
                    {
                        if (e3.IsSame(exp1.Current()))
                        {
                            break;
                        }
                    }
                    //
                    for (exp1.Next(); exp1.More(); exp1.Next())
                    {
                        Edge e3Next = ShapeCasts.Edge(exp1.Current());
                        if (me.Contains(e3Next))
                        {
                            ShapeImage emptyImage = new();
                            RefEdgeInter(fio,
                                         bAsurf,
                                         e3Next,
                                         e3,
                                         or1,
                                         or2,
                                         asDes2d,
                                         tol,
                                         true,
                                         vref,
                                         emptyImage,
                                         dmvv,
                                         out bCoincide);
                        }
                    }
                }
                curE = wexp.Current();
            }
        }
    }

    //=================================================================================================

    private static void MakeChain(in Shape v,
                                  IndexedMap<Shape, NCollection_List<Shape>> dmvv,
                                  HashSet<Shape> mDone,
                                  NCollection_List<Shape> chain)
    {
        if (mDone.Add(v))
        {
            chain.Append(v);
            int ind = dmvv.IndexOf(v) + 1;
            if (ind != 0)
            {
                NCollection_List<Shape> pLV = dmvv.ValueAt(ind - 1);
                NCollection_List<Shape>.Iterator it = new(pLV);
                for (; it.More(); it.Next())
                {
                    MakeChain(it.Value(), dmvv, mDone, chain);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Fuses the chains of coinciding vertices of <paramref name="dmvv"/> and updates
    /// <paramref name="asDes"/>, replacing the old vertices with the new ones. False if a vertex has no parameter
    /// on one of its edges.
    /// </summary>
    public static bool FuseVertices(IndexedMap<Shape, NCollection_List<Shape>> dmvv,
                                    AsDes asDes,
                                    ShapeImage imageVV)
    {
        BrepBuilder bb = new();
        HashSet<Shape> mvDone = new(ShapeHasher.Instance);
        int i, nb = dmvv.Count;
        for (i = 1; i <= nb; ++i)
        {
            Vertex v = ShapeCasts.Vertex(dmvv.KeyAt(i - 1));
            //
            // find chain of vertices
            NCollection_List<Shape> lvChain = new();
            MakeChain(v, dmvv, mvDone, lvChain);
            //
            if (lvChain.Extent() < 2)
            {
                continue;
            }
            //
            // make new vertex
            Vertex vNew = new();
            AlgoTools.MakeVertex(lvChain, ref vNew);
            //
            Vertex vNewInt = ShapeCasts.Vertex(vNew.Oriented(Internal));
            //
            NCollection_List<Shape>.Iterator it = new(lvChain);
            for (; it.More(); it.Next())
            {
                Shape vOld = it.Value();
                // update the parameters on edges
                Vertex vOldInt = ShapeCasts.Vertex(vOld.Oriented(Internal));
                NCollection_List<Shape> le = asDes.Ascendant(vOld);
                //
                NCollection_List<Shape>.Iterator itLE = new(le);
                for (; itLE.More(); itLE.Next())
                {
                    Edge e = ShapeCasts.Edge(itLE.Value());
                    double tolE = Tool.Tolerance(e);
                    if (!Tool.Parameter(vOldInt, e, out double t))
                    {
                        return false;
                    }
                    bb.UpdateVertex(vNewInt, t, e, tolE);
                }
                // and replace the vertex
                asDes.Replace(vOld, vNew);
                if (imageVV.IsImage(vOld))
                {
                    Vertex proVertex = ShapeCasts.Vertex(imageVV.ImageFrom(vOld));
                    imageVV.Add(proVertex, vNew.Oriented(Forward));
                }
            }
        }
        return true;
    }
}
