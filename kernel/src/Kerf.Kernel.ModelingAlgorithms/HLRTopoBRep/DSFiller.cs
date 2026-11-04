// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_DSFiller.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_DSFiller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.HiddenLines.Contours;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.HiddenLines.Contours.ContourLineType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.HiddenLines.Topology;

/// <summary>
/// Fills HLRTopoBRep_Data for a shape: the contours (outlines) of each face as new edges with their vertices, the
/// isolines, and the splits of the edges at the vertices inserted on them.
/// </summary>
/// <remarks>
/// OCCT's static class. It writes into the shape it is given (the point representations of the end vertices of the
/// edges it splits, in <c>ProcessEdges</c>): HLRTopoBRep_OutLiner runs it in a working overlay (unit 10H6); outside
/// one a frozen shape throws TopoDS_LockedShape. The contours are built as OCCT does
/// (decision 1084). The commented-out vertex map and interpolation of short walking lines are left out.
/// </remarks>
internal static class DSFiller
{
    /// <summary>Outlines each face of S with FO into DS (nbIso isolines per direction), then splits the edges.</summary>
    public static void Insert(in Shape s, Contour fo, TopologyData ds,
                              Dictionary<Shape, AdaptorTool> mst, int nbIso)
    {
        HashSet<Shape> shapeMap = new(ShapeHasher.Instance);
        Explorer ex = new(s, ShapeType.Face);
        ds.Clear();
        bool withPCurve = true; // instead of nbIso != 0;
        int f = 0;

        while (ex.More())
        {
            if (shapeMap.Add(ex.Current()))
            {
                f++;
                Face s1 = ShapeCasts.Face(ex.Current());
                s1.Orient(Forward);
                BrepTopolTool domain;
                SurfaceAdaptorBase? surface;
                if (mst.ContainsKey(s1))
                {
                    AdaptorTool brt = mst.GetValue(s1);
                    domain = brt.GetTopolTool();
                    surface = brt.GetSurface();
                }
                else
                {
                    AdaptorTool brt = new(s1, global::Kerf.Engine.Foundation.Precision.PConfusion());
                    mst[s1] = brt;
                    domain = brt.GetTopolTool();
                    surface = brt.GetSurface();
                }
                fo.Perform(surface!, domain);
                if (fo.IsDone)
                {
                    if (!fo.IsEmpty)
                    {
                        InsertFace(f, s1, fo, ds, withPCurve);
                    }
                }

                if (nbIso != 0)
                {
                    FaceIsoLiner.Perform(f, s1, ds, nbIso);
                }
            }
            ex.Next();
        }
        ProcessEdges(ds);
    }

    /// <summary>Builds the edges of the contours of F and records them as F's outlines and internal outlines.</summary>
    private static void InsertFace(int fi, in Face f, Contour fo, TopologyData ds, bool withPCurve)
    {
        double tol = Tool.Tolerance(f);
        NCollection_List<Shape> intL = ds.AddIntL(f);
        NCollection_List<Shape> outL = ds.AddOutL(f);

        Vertex vf = new(), vl = new();
        // OCCT's commented-out map of the vertices of the closed edges is left out.

        int nbLines = fo.LineCount;
        int curLine;
        for (curLine = 1; curLine <= nbLines; curLine++)
        {
            ContourLine line = fo.Line(curLine);
            int nbPoints = line.VertexCount;
            int curPoint;
            if (line.TypeContour() == ContourLineType.Restriction)
            {
                // OutLine on restriction
                Edge e = ((EdgeCurve2DAdaptor)line.Arc()).Edge();
                outL.Append(e);
                Exploration.Vertices(e, out vf, out vl);
                // insert the Internal points.

                for (curPoint = 1; curPoint <= nbPoints; curPoint++)
                {
                    ContourPoint p = line.Vertex(curPoint);
                    if (p.IsInternal)
                    {
                        if (p.Value().IsEqual(Tool.Pnt(vf), Tool.Tolerance(vf)))
                        {
                            if (p.Value().IsEqual(Tool.Pnt(vl), Tool.Tolerance(vl)))
                            {
                                InsertVertex(p, tol, e, ds);
                            }
                        }
                    }
                }
            }
            else
            {
                for (curPoint = 1; curPoint <= nbPoints; curPoint++)
                {
                    ContourPoint pf = line.Vertex(curPoint);
                    if (pf.IsInternal && curPoint != 1)
                    {
                        vf = vl;
                    }
                    else
                    {
                        vf = MakeVertex(pf, tol, ds);
                    }
                    double parF = pf.ParameterOnLine();

                    if (curPoint < nbPoints)
                    {
                        ContourPoint pl = line.Vertex(curPoint + 1);
                        vl = MakeVertex(pl, tol, ds);
                        double parL = pl.ParameterOnLine();

                        if ((parL - parF) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {

                            Curve3D? c = null;
                            Curve2D? c2d = null;
                            double first = parF;
                            double last = parL;
                            bool insuffisantNumberOfPoints = false;

                            switch (line.TypeContour())
                            {

                                case ContourLineType.Lin:
                                {
                                    c = new LineCurve3D(line.Line());
                                    if (withPCurve)
                                    {
                                        Surface s = Tool.Surface(f)!;
                                        double Tol = 1e-7;
                                        c2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, first, last, s, ref Tol);
                                    }
                                }
                                break;

                                case ContourLineType.Circle:
                                {
                                    c = new CircleCurve3D(line.Circle());
                                    if (withPCurve)
                                    {
                                        Surface s = Tool.Surface(f, out Placement loc)!;
                                        if (!loc.IsIdentity)
                                        {
                                            s = (Surface)s.Transformed(loc.Transformation);
                                        }
                                        double Tol = 1e-7;
                                        c2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, first, last, s, ref Tol);
                                    }
                                }
                                break;

                                case ContourLineType.Walking:
                                {
                                    // copy the points
                                    int ipF = (int)(parF);
                                    int ipL = (int)(parL);

                                    if (ipL - ipF < 1)
                                    {
                                        insuffisantNumberOfPoints = true;
                                        // OCCT_DEBUG trace left out
                                    }

                                    // OCCT's commented-out interpolation of the lines of fewer than 6 points is left out.

                                    else if (ipL - ipF < 5)
                                    {
                                        int nbp = ipL - ipF + 1;
                                        NCollection_Array1<double> knots = new(1, nbp);
                                        NCollection_Array1<int> mults = new(1, nbp);
                                        NCollection_Array1<Point3D> points = new(1, nbp);

                                        for (int i = 1; i <= nbp; i++)
                                        {
                                            knots.SetValue(i, (double)i);
                                            mults.SetValue(i, 1);
                                            points.SetValue(i, line.Point(i + ipF - 1).Value());
                                        }
                                        mults[1] = mults[nbp] = 2;
                                        c = new BSplineCurve3D(points, knots, mults, 1);

                                        if (withPCurve)
                                        {
                                            NCollection_Array1<Point2D> points2d = new(1, nbp);
                                            for (int i = 1; i <= nbp; i++)
                                            {
                                                double u, v;
                                                line.Point(i + ipF - 1).ParametersOnS2(out u, out v);
                                                points2d.SetValue(i, new Point2D(u, v));
                                            }
                                            c2d = new BSplineCurve2D(points2d, knots, mults, 1);
                                        }
                                        first = 1;
                                        last = nbp;
                                    }
                                    else
                                    {
                                        int nbp = ipL - ipF + 1;
                                        NCollection_Array1<double> knots = new(1, nbp);
                                        NCollection_Array1<int> mults = new(1, nbp);
                                        NCollection_Array1<Point3D> points = new(1, nbp);

                                        double maxx, maxy, maxz, maxu, maxv;
                                        double minx, miny, minz, minu, minv;
                                        maxx = maxy = maxz = maxu = maxv = -RealLast();
                                        minx = miny = minz = minu = minv = RealLast();

                                        for (int i = 1; i <= nbp; i++)
                                        {
                                            knots.SetValue(i, (double)i);
                                            mults.SetValue(i, 1);
                                            Point3D p = line.Point(i + ipF - 1).Value();
                                            if (p.X < minx)
                                            {
                                                minx = p.X;
                                            }
                                            if (p.Y < miny)
                                            {
                                                miny = p.Y;
                                            }
                                            if (p.Z < minz)
                                            {
                                                minz = p.Z;
                                            }
                                            if (p.X > maxx)
                                            {
                                                maxx = p.X;
                                            }
                                            if (p.Y > maxy)
                                            {
                                                maxy = p.Y;
                                            }
                                            if (p.Z > maxz)
                                            {
                                                maxz = p.Z;
                                            }
                                            points.SetValue(i, p);
                                        }
                                        mults[1] = mults[nbp] = 2;
                                        BSplineCurve3D appC;
                                        BSplineCurve2D? appC2d = null;
                                        appC = new BSplineCurve3D(points, knots, mults, 1);

                                        if (withPCurve)
                                        {
                                            NCollection_Array1<Point2D> points2d = new(1, nbp);
                                            for (int i = 1; i <= nbp; i++)
                                            {
                                                double u, v;
                                                line.Point(i + ipF - 1).ParametersOnS2(out u, out v);
                                                if (u < minu)
                                                {
                                                    minu = u;
                                                }
                                                if (v < minv)
                                                {
                                                    minv = v;
                                                }
                                                if (u > maxu)
                                                {
                                                    maxu = u;
                                                }
                                                if (v > maxv)
                                                {
                                                    maxv = v;
                                                }
                                                points2d.SetValue(i, new Point2D(u, v));
                                            }
                                            appC2d = new BSplineCurve2D(points2d, knots, mults, 1);
                                        }
                                        first = 1;
                                        last = nbp;

                                        ApproxLine appLine;
                                        BSplineCurve2D? cNull = null;
                                        appLine = new ApproxLine(appC, appC2d, cNull);

                                        int dmin = 4, dmax = 8, niter = 0;
                                        bool tg = false;
                                        ApproximationApprox approx = new();
                                        double tol3d, tol2d, TOL = 0.0001;

                                        maxx -= minx;
                                        maxy -= miny;
                                        maxz -= minz;
                                        maxu -= minu;
                                        maxv -= minv;
                                        if (maxy > maxx)
                                        {
                                            maxx = maxy;
                                        }
                                        if (maxz > maxx)
                                        {
                                            maxx = maxy; // OCCT: Maxy where Maxz was meant
                                        }
                                        if (maxv > maxu)
                                        {
                                            maxu = maxv;
                                        }

                                        tol3d = TOL * maxx;
                                        if (tol3d < 1e-12)
                                        {
                                            tol3d = 1e-12;
                                        }
                                        else if (tol3d > 0.1)
                                        {
                                            tol3d = 0.1;
                                        }
                                        tol2d = TOL * maxu;
                                        if (tol2d < 1e-12)
                                        {
                                            tol2d = 1e-12;
                                        }
                                        else if (tol2d > 0.1)
                                        {
                                            tol2d = 0.1;
                                        }

                                        // std::cout<<"\nHLRTopoBRep_DSFiller : nbp="<<nbp<<"  Tol3d="<<TOL3d<<"
                                        // Tol2d="<<TOL2d<<std::endl;

                                        approx.SetParameters(tol3d, tol2d, dmin, dmax, niter, 30, tg);
                                        approx.Perform(appLine, true, true, false, 1, nbp);
                                        if (!approx.IsDone())
                                        {
                                            c = appC;
                                            c2d = appC2d;
                                            first = 1;
                                            last = nbp;
                                        }
                                        else
                                        {
                                            MultiBSpCurve appVal = approx.Value(1);
                                            NCollection_Array1<Point3D> poles3d = new(1, appVal.NbPoles());
                                            appVal.Curve(1, poles3d);
                                            c = new BSplineCurve3D(poles3d, appVal.Knots(), appVal.Multiplicities(),
                                                                      appVal.Degree());

                                            MultiBSpCurve appVal2 = approx.Value(2);
                                            NCollection_Array1<Point2D> poles2d = new(1, appVal2.NbPoles());
                                            appVal2.Curve(2, poles2d);
                                            c2d = new BSplineCurve2D(poles2d, appVal2.Knots(),
                                                                          appVal2.Multiplicities(), appVal2.Degree());
                                            first = c2d.FirstParameter;
                                            last = c2d.LastParameter;
                                        }
                                    }
                                }
                                break;

                                case ContourLineType.Restriction:
                                {
                                    throw new ProgramErrorException("HLRTopoBRep_DSFiller::InsertFace : Restriction");
                                }
                            }

                            // compute the PCurve
                            // make the edge
                            if (!insuffisantNumberOfPoints)
                            {
                                Edge e = new();
                                BrepBuilder b = new();
                                b.MakeEdge(ref e, c, tol);
                                vf.Orient(Forward);
                                vl.Orient(Reversed);
                                b.Add(e, vf);
                                b.Add(e, vl);
                                b.Range(e, first, last);

                                if (c2d is not null)
                                {
                                    b.UpdateEdge(e, c2d, f, Tool.Tolerance(f));
                                }

                                // add the edge in the DS
                                if (!e.IsNull)
                                {
                                    intL.Append(e);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Correction of internal outlines: unite coinciding vertices
        double sqTol = tol * tol;

        NCollection_List<Shape>.Iterator itl1 = new(intL);
        for (; itl1.More(); itl1.Next())
        {
            Edge intLine = ShapeCasts.Edge(itl1.Value());
            intLine.Orient(Forward);
            Vertex[] ver = new Vertex[2];
            Exploration.Vertices(intLine, out ver[0], out ver[1]);
            NCollection_List<Shape>.Iterator itl2 = new(itl1);
            for (; itl2.More(); itl2.Next())
            {
                Edge intLine2 = ShapeCasts.Edge(itl2.Value());
                intLine2.Orient(Forward);
                if (intLine2.IsSame(intLine))
                {
                    continue;
                }
                Vertex[] ver2 = new Vertex[2];
                Exploration.Vertices(intLine2, out ver2[0], out ver2[1]);
                for (int i = 0; i < 2; i++)
                {
                    if (i == 1 && ver[0].IsSame(ver[1]))
                    {
                        continue;
                    }
                    Point3D pnt1 = Tool.Pnt(ver[i]);
                    for (int j = 0; j < 2; j++)
                    {
                        if (ver[i].IsSame(ver2[j]))
                        {
                            continue;
                        }
                        Point3D pnt2 = Tool.Pnt(ver2[j]);
                        if (pnt1.SquareDistanceTo(pnt2) <= sqTol)
                        {
                            BrepBuilder bb = new();
                            bb.Remove(intLine2, ver2[j]);
                            ver[i].Orient((j == 0) ? Forward : Reversed);
                            bb.Add(intLine2, ver[i]);
                        }
                    }
                }
            }
        }
    }

    /// <summary>The vertex of a contour point: the face's vertex, a vertex inserted on the arc it lies on, or a new vertex.</summary>
    private static Vertex MakeVertex(in ContourPoint p, double tol, TopologyData ds)
    {
        BrepBuilder b = new();
        Vertex v = new();
        if (p.IsVertex)
        {
            v = ((TopologyHVertex)p.Vertex()).Vertex();
            ds.AddOutV(v);
        }
        else
        {
            // if on arc, insert in the DS
            if (p.IsOnArc)
            {
                Edge e = ((EdgeCurve2DAdaptor)p.Arc()).Edge();
                double par = p.ParameterOnArc();
                Point3D p3d = p.Value();

                for (ds.InitVertex(e); ds.MoreVertex(); ds.NextVertex())
                {
                    Vertex curV = ds.Vertex();
                    double curP = ds.Parameter();
                    Point3D ppp = Tool.Pnt(curV);
                    double ttt = Tool.Tolerance(curV);
                    if (p3d.IsEqual(ppp, ttt))
                    {
                        v = curV;
                        break;
                    }
                    else if (par < curP)
                    {
                        b.MakeVertex(ref v, p.Value(), tol);
                        ds.InsertBefore(v, par);
                        break;
                    }
                }
                if (!ds.MoreVertex())
                {
                    b.MakeVertex(ref v, p.Value(), tol);
                    ds.Append(v, par);
                }
                ds.AddOutV(v);
            }
            // if internal create a vertex and insert in the DS
            else
            {
                b.MakeVertex(ref v, p.Value(), tol);
                if (p.IsInternal)
                {
                    ds.AddIntV(v);
                }
                else
                {
                    ds.AddOutV(v);
                }
            }
        }
        return v;
    }

    /// <summary>Inserts the vertex of a contour point on the edge E.</summary>
    private static void InsertVertex(in ContourPoint p, double tol, in Edge e, TopologyData ds)
    {
        BrepBuilder b = new();
        Vertex v = new();

        if (p.IsVertex)
        {
            v = ((TopologyHVertex)p.Vertex()).Vertex();
        }
        else
        {
            double par = p.ParameterOnLine();

            for (ds.InitVertex(e); ds.MoreVertex(); ds.NextVertex())
            {
                Vertex curV = ds.Vertex();
                double curP = ds.Parameter();
                if (p.Value().IsEqual(Tool.Pnt(curV), Tool.Tolerance(curV)))
                {
                    v = curV;
                    break;
                }
                else if (par < curP)
                {
                    b.MakeVertex(ref v, p.Value(), tol);
                    ds.InsertBefore(v, par);
                    break;
                }
            }
            if (!ds.MoreVertex())
            {
                b.MakeVertex(ref v, p.Value(), tol);
                ds.Append(v, par);
            }
        }
        ds.AddIntV(v);
    }

    /// <summary>Splits each edge with inserted vertices into new edges, recorded as its splits.</summary>
    private static void ProcessEdges(TopologyData ds)
    {
        BrepBuilder b = new();
        Edge newE;
        Vertex vf, vl, vi;
        double pf, pl, pi;

        for (ds.InitEdge(); ds.MoreEdge(); ds.NextEdge())
        {
            Edge e = ds.Edge();
            NCollection_List<Shape> splE = ds.AddSplE(e);
            vf = Exploration.FirstVertex(e);
            vl = Exploration.LastVertex(e);
            Tool.Range(e, out pf, out pl);
            vf.Orient(Forward);
            vl.Orient(Reversed);

            for (ds.InitVertex(e); ds.MoreVertex(); ds.NextVertex())
            {
                vi = ds.Vertex();
                pi = ds.Parameter();
                vi.Orient(Reversed);
                newE = e;
                newE.EmptyCopy();
                newE.Orient(Forward);
                b.Add(newE, vf);
                b.UpdateVertex(vf, pf, newE, Tool.Tolerance(vf));
                b.Add(newE, vi);
                b.UpdateVertex(vi, pi, newE, Tool.Tolerance(vi));
                newE.Orient(e.Orientation);
                splE.Append(newE);
                // prepare next edge
                vf = vi;
                pf = pi;
                vf.Orient(Forward);
            }
            newE = e;
            newE.EmptyCopy();
            newE.Orient(Forward);
            b.Add(newE, vf);
            b.UpdateVertex(vf, pf, newE, Tool.Tolerance(vf));
            b.Add(newE, vl);
            b.UpdateVertex(vl, pl, newE, Tool.Tolerance(vl));
            newE.Orient(e.Orientation);
            splE.Append(newE);
        }
    }
}
