// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_CompatibleWires.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_CompatibleWires.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Makes a sequence of wires compatible for a loft or a sweep: the same number of edges in each wire, with origins and
/// orientations chosen to avoid twisted results. Closed sections are cut by a polar method (or reordered when they
/// already have as many edges), open ones by reduced curvilinear abscissa; a first or last section may be a point.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The wires it returns hold the caller's edges where they weren't cut and new edges on
/// the caller's curves where they were; the caller's shapes are never written (decision 905). <see cref="Shape"/>,
/// <see cref="GeneratedShapes"/> and <see cref="Generated"/> hand out frozen shapes; ported callers (the lofts and
/// sweeps) read the algorithm's own containers through the <c>Internal</c> forms.
/// </para>
/// </remarks>
internal sealed class CompatibleWires
{
    private NCollection_Sequence<Shape> _work = new();
    private double _percent;
    private bool _degen1;
    private bool _degen2;
    private ThruSectionErrorStatus _status;
    private readonly Dictionary<Shape, NCollection_List<Shape>> _map = new(ShapeHasher.Instance);

    //=======================================================================
    // function : AddNewEdge
    // purpose  : for <theEdge> find all newest edges
    //           in <theEdgeNewEdges> recursively
    //=======================================================================

    private static void AddNewEdge(in Shape edge,
                                   Dictionary<Shape, NCollection_Sequence<Shape>> edgeNewEdges,
                                   NCollection_List<Shape> listNewEdges)
    {
        if (edgeNewEdges.ContainsKey(edge))
        {
            NCollection_Sequence<Shape> newEdges = edgeNewEdges.GetValue(edge);
            for (int i = 1; i <= newEdges.Length(); i++)
            {
                Shape anEdge = newEdges[i];
                AddNewEdge(anEdge, edgeNewEdges, listNewEdges);
            }
        }
        else
        {
            listNewEdges.Append(edge);
        }
    }

    private static void SeqOfVertices(in Wire w, NCollection_Sequence<Shape> s)
    {
        s.Clear();
        int jj;
        Explorer pe = new();
        for (pe.Init(w, ShapeType.Vertex); pe.More(); pe.Next())
        {
            bool trouve = false;
            for (jj = 1; jj <= s.Length() && (!trouve); jj++)
            {
                if (s.Value(jj).IsSame(pe.Current()))
                {
                    trouve = true;
                }
            }
            if (!trouve)
            {
                s.Append(pe.Current());
            }
        }
    }

    private static bool PlaneOfWire(in Wire w, out Plane3D p)
    {
        p = new Plane3D();
        bool isplane = true;
        FindSurface findPlanarSurf = new();
        Surface? s;
        Placement l;

        Properties gp = new();
        Point3D bary = new();
        bool isBaryDefined = false;

        // shielding for particular cases : only one edge circle or ellipse
        // on a closed wire !

        bool wClosed = w.ClosedFlag;
        if (!wClosed)
        {
            // it is checked if the vertices are the same.
            Exploration.Vertices(w, out Vertex v1, out Vertex v2);
            if (v1.IsSame(v2))
            {
                wClosed = true;
            }
        }

        if (wClosed)
        {
            int nbEdges = 0;
            Iterator iter = new();
            iter.Initialize(w);
            for (; iter.More(); iter.Next())
            {
                nbEdges++;
            }

            if (nbEdges == 1)
            {
                CurveAdaptor adC = new();
                iter.Initialize(w);
                adC.Load(Tool.Curve(ShapeCasts.Edge(iter.Value()), out _, out _)!);

                if (adC.GetType() == Circle)
                {
                    bary = adC.Circle().Location;
                    isBaryDefined = true;
                }

                if (adC.GetType() == Ellipse)
                {
                    bary = adC.Ellipse().Location;
                    isBaryDefined = true;
                }
            }
        }

        if (!isBaryDefined)
        {
            MassPropertiesBuilder.LinearProperties(w, gp);
            bary = gp.CentreOfMass();
        }

        findPlanarSurf.Init(w, -1, true);
        if (findPlanarSurf.Found())
        {
            s = findPlanarSurf.Surface();
            l = findPlanarSurf.Location();
            if (!l.IsIdentity)
            {
                s = (Surface)s!.Transformed(l.Transformation);
            }
            p = ((PlaneSurface)s!).Plane;
            p = p.WithLocation(bary);
        }
        else
        {
            // wire not plane !
            PrincipalProperties pp = gp.PrincipalProperties();
            Vector3D vec = new();
            double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
            pp.RadiusOfGyration(out double r1, out double r2, out double r3);
            double rMax = Max(Max(r1, r2), r3);
            if ((Math.Abs(rMax - r1) < tol && Math.Abs(rMax - r2) < tol)
                || (Math.Abs(rMax - r1) < tol && Math.Abs(rMax - r3) < tol)
                || (Math.Abs(rMax - r2) < tol && Math.Abs(rMax - r3) < tol))
            {
                isplane = false;
            }
            else
            {
                if (r1 >= r2 && r1 >= r3)
                {
                    vec = pp.FirstAxisOfInertia();
                }
                else if (r2 >= r1 && r2 >= r3)
                {
                    vec = pp.SecondAxisOfInertia();
                }
                else if (r3 >= r1 && r3 >= r2)
                {
                    vec = pp.ThirdAxisOfInertia();
                }
                Direction3D nDir = new(vec);
                if (r3 <= r2 && r3 <= r1)
                {
                    vec = pp.ThirdAxisOfInertia();
                }
                else if (r2 <= r1 && r2 <= r3)
                {
                    vec = pp.SecondAxisOfInertia();
                }
                else if (r1 <= r2 && r1 <= r3)
                {
                    vec = pp.FirstAxisOfInertia();
                }
                Direction3D xDir = new(vec);
                CoordinateSystem repere = new(bary, nDir, xDir);
                PlaneSurface gPlan = new(repere);
                p = gPlan.Plane;
            }
        }

        return isplane;
    }

    private static void WireContinuity(in Wire w, out Continuity contW)
    {
        contW = CN;
        Continuity cont;
        bool isDegenerated = false;

        WireExplorer exp = new();
        int nbEdges = 0;
        NCollection_HSequence<Shape> edges = new();
        for (exp.Init(w); exp.More(); exp.Next())
        {
            nbEdges++;
            edges.Append(exp.Current());
            if (Tool.Degenerated(exp.Current()))
            {
                isDegenerated = true;
            }
        }

        if (!isDegenerated)
        {

            bool testconti = true;

            for (int j = 1; j <= nbEdges; j++)
            {

                Edge edge1, edge2;

                if (j == nbEdges)
                {
                    edge1 = ShapeCasts.Edge(edges.Value(nbEdges));
                    edge2 = ShapeCasts.Edge(edges.Value(1));
                }
                else
                {
                    edge1 = ShapeCasts.Edge(edges.Value(j));
                    edge2 = ShapeCasts.Edge(edges.Value(j + 1));
                }

                Exploration.Vertices(edge1, out _, out Vertex v1, true);
                Exploration.Vertices(edge2, out Vertex v2, out _, true);
                double u1 = Tool.Parameter(v1, edge1);
                double u2 = Tool.Parameter(v2, edge2);
                EdgeCurveAdaptor curve1 = new(edge1);
                EdgeCurveAdaptor curve2 = new(edge2);
                double eps = Tool.Tolerance(v2) + Tool.Tolerance(v1);

                if (j == nbEdges)
                {
                    testconti = curve1.Value(u1).IsEqual(curve2.Value(u2), eps);
                }

                if (testconti)
                {
                    cont = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(curve1, curve2, u1, u2, eps, global::Kerf.Engine.Foundation.Precision.Angular());
                    if (cont <= contW)
                    {
                        contW = cont;
                    }
                }
            }
        }
    }

    private static void TrimEdge(in Edge currentEdge,
                                 NCollection_Sequence<double> cutValues,
                                 double t0,
                                 double t1,
                                 bool seqOrder,
                                 NCollection_Sequence<Shape> s)

    {
        s.Clear();
        int j, ndec = cutValues.Length();
        double m0, m1;
        Curve3D c = Tool.Curve(currentEdge, out double first, out double last)!;

        Vertex vbid = new(), v0, v1;
        Orientation currentOrient = currentEdge.Orientation;
        Exploration.Vertices(currentEdge, out Vertex vf, out Vertex vl);
        vbid.Nullify();

        if (seqOrder)
        {
            // from first to last
            m0 = first;
            v0 = vf;
            for (j = 1; j <= ndec; j++)
            {
                // piece of edge
                m1 = (cutValues.Value(j) - t0) * (last - first) / (t1 - t0) + first;
                if (Math.Abs(m0 - m1) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return;
                }
                Edge cutE = new LibraryMakeEdge(c, v0, vbid, m0, m1).InternalEdge();
                cutE.Orient(currentOrient);
                s.Append(cutE);
                m0 = m1;
                v0 = Exploration.LastVertex(cutE);
                if (j == ndec)
                {
                    // last piece
                    if (Math.Abs(m0 - last) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        return;
                    }
                    Edge lastE = new LibraryMakeEdge(c, v0, vl, m0, last).InternalEdge();
                    lastE.Orient(currentOrient);
                    s.Append(lastE);
                }
            }
        }
        else
        {
            // from last to first
            m1 = last;
            v1 = vl;
            for (j = ndec; j >= 1; j--)
            {
                // piece of edge
                m0 = (cutValues.Value(j) - t0) * (last - first) / (t1 - t0) + first;
                if (Math.Abs(m0 - m1) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return;
                }
                Edge cutE = new LibraryMakeEdge(c, vbid, v1, m0, m1).InternalEdge();
                cutE.Orient(currentOrient);
                s.Append(cutE);
                m1 = m0;
                v1 = Exploration.FirstVertex(cutE);
                if (j == 1)
                {
                    // last piece
                    if (Math.Abs(first - m1) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        return;
                    }
                    Edge lastE = new LibraryMakeEdge(c, vf, v1, first, m1).InternalEdge();
                    lastE.Orient(currentOrient);
                    s.Append(lastE);
                }
            }
        }
    }

    // OCCT: native iterates the map in the hash order of TShape addresses; the port iterates in binding order
    // (decision 18). The first key whose list holds V wins, so the order matters only when V is in several lists
    // (decision 908).
    private static bool SearchRoot(in Vertex v,
                                   Dictionary<Shape, NCollection_List<Shape>> map,
                                   out Vertex vRoot)
    {
        bool trouve = false;
        vRoot = new Vertex();
        vRoot.Nullify();
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in map)
        {
            NCollection_List<Shape> list = it.Value;
            NCollection_List<Shape>.Iterator itL = new();
            bool ilyest = false;
            for (itL.Initialize(list); itL.More(); itL.Next())
            {
                Vertex vcur = ShapeCasts.Vertex(itL.Value());
                if (vcur.IsSame(v))
                {
                    ilyest = true;
                }
                if (ilyest)
                {
                    break;
                }
            }
            if (ilyest)
            {
                trouve = true;
                vRoot = ShapeCasts.Vertex(it.Key);
            }
            if (trouve)
            {
                break;
            }
        }
        return trouve;
    }

    private static bool SearchVertex(NCollection_List<Shape> list, in Wire w, out Vertex vonW)
    {
        bool trouve = false;
        vonW = new Vertex();
        vonW.Nullify();
        NCollection_Sequence<Shape> seqV = new();
        SeqOfVertices(w, seqV);
        for (int ii = 1; ii <= seqV.Length(); ii++)
        {
            Vertex vi = ShapeCasts.Vertex(seqV.Value(ii));
            NCollection_List<Shape>.Iterator itL = new();
            bool ilyest = false;
            for (itL.Initialize(list); itL.More(); itL.Next())
            {
                Vertex vcur = ShapeCasts.Vertex(itL.Value());
                if (vcur.IsSame(vi))
                {
                    ilyest = true;
                }
                if (ilyest)
                {
                    break;
                }
            }
            if (ilyest)
            {
                trouve = true;
                vonW = vi;
            }
            if (trouve)
            {
                break;
            }
        }
        return trouve;
    }

    private static bool EdgeIntersectOnWire(in Point3D p1,
                                            in Point3D p2,
                                            double percent,
                                            Dictionary<Shape, NCollection_List<Shape>> map,
                                            in Wire w,
                                            ref Vertex vsol,
                                            ref Wire newW,
                                            Dictionary<Shape, NCollection_Sequence<Shape>> edgeNewEdges)
    {

        WireExplorer exp = new();

        // construction of the edge of intersection
        bool newVertex = false;
        Line3D droite = new(p1, new Direction3D(new Vector3D(p1, p2)));
        // ATTENTION : it is required to construct a half-straight
        //             but there is a bug in BRepExtrema_DistShapeShape
        //             it is enough to take 100 * distance between P1 and P2
        //             hoping that it is enough until the bug is corrected
        //  double dernierparam = Precision::Infinite();
        // ATTENTION : return !!
        //             100 is better than 10 but it is too much !
        //             finally, nothing is better than a blocking box
        //  double dernierparam = 100 * P1.Distance(P2);
        Box b = new();
        Bounds.Add(w, ref b);
        b.Get(out double x1, out double y1, out double z1, out double x2, out double y2, out double z2);
        Point3D bp1 = new(x1, y1, z1), bp2 = new(x2, y2, z2);
        double diag = bp1.DistanceTo(bp2);
        double dernierparam = diag;
        LibraryMakeEdge me = new(droite, 0.0, dernierparam);
        Edge eCur = new LibraryMakeEdge(droite, 0.0, p1.DistanceTo(p2)).InternalEdge();

        // calculate the intersection by BRepExtrema (point of min distance)
        DistanceShapeShape DSS = new(me.InternalEdge(), w);
        if (DSS.IsDone)
        {
            // choose the solution closest to P2
            int isol = 1;
            Point3D psol = DSS.PointOnShape2(isol);
            double dss = p2.DistanceTo(psol);
            for (int iss = 2; iss <= DSS.SolutionCount; iss++)
            {
                Point3D pss = DSS.PointOnShape2(iss);
                double dist = p2.DistanceTo(pss);
                if (dss > dist)
                {
                    dss = dist;
                    isol = iss;
                }
            }

            // is the solution a new vertex ?
            newVertex = (DSS.SupportTypeShape2(isol) != SupportType.IsVertex);
            if (newVertex)
            {
                Edge e = ShapeCasts.Edge(DSS.SupportOnShape2(isol));
                double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
                Tool.Range(e, out double first, out double last);
                tol = Max(tol, percent * Math.Abs(last - first));
                DSS.ParOnEdgeS2(isol, out double param);
                if (Math.Abs(first - param) < tol)
                {
                    newVertex = false;
                    vsol = Exploration.FirstVertex(e);
                }
                else if (Math.Abs(last - param) < tol)
                {
                    newVertex = false;
                    vsol = Exploration.LastVertex(e);
                }
                // check
                if (!newVertex)
                {
                    if (SearchRoot(vsol, map, out _))
                    {
                        newVertex = true;
                    }
                }
            }
            else
            {
                Shape localShape = DSS.SupportOnShape2(isol);
                vsol = ShapeCasts.Vertex(localShape);
                //      Vsol = TopoDS::Vertex(DSS.SupportOnShape2(isol));
            }

            // it is required to cut the edge
            if (newVertex)
            {
                Shape localShape = DSS.SupportOnShape2(isol);
                Edge e = ShapeCasts.Edge(localShape);
                //      TopoDS_Edge E = TopoDS::Edge(DSS.SupportOnShape2(isol));
                NCollection_Sequence<Shape> emptySeq = new();
                edgeNewEdges[e] = emptySeq;
                DSS.ParOnEdgeS2(isol, out double param);
                Tool.Range(e, out double first, out double last);
                LibraryMakeWire mw = new();
                for (exp.Init(w); exp.More(); exp.Next())
                {
                    if (e.IsSame(exp.Current()))
                    {
                        bool so = (exp.CurrentVertex().IsSame(Exploration.FirstVertex(e)));
                        NCollection_Sequence<Shape> se = new();
                        se.Clear();
                        NCollection_Sequence<double> sr = new();
                        sr.Clear();
                        sr.Append(param);
                        TrimEdge(e, sr, first, last, so, se);
                        if (se.IsEmpty())
                        {
                            return false;
                        }
                        edgeNewEdges.GetValueRef(e) = se;
                        Exploration.Vertices(ShapeCasts.Edge(se.Value(1)), out Vertex vv1, out Vertex vv2);
                        if (Exploration.FirstVertex(e).IsSame(vv1) || Exploration.LastVertex(e).IsSame(vv1))
                        {
                            vsol = vv2;
                        }
                        if (Exploration.FirstVertex(e).IsSame(vv2) || Exploration.LastVertex(e).IsSame(vv2))
                        {
                            vsol = vv1;
                        }
                        for (int k = 1; k <= se.Length(); k++)
                        {
                            mw.Add(ShapeCasts.Edge(se.Value(k)));
                        }
                    }
                    else
                    {
                        mw.Add(exp.Current());
                    }
                }
                newW = mw.InternalWire();
            }
            else
            {
                newW = w;
            }
        }

        return newVertex;
    }

    private static void Transform(bool withRotation,
                                  in Point3D p,
                                  in Point3D pos1,
                                  in Vector3D ax1,
                                  in Point3D pos2,
                                  in Vector3D ax2,
                                  out Point3D pnew)
    {

        pnew = p.Translated(pos1, pos2);
        Vector3D axe1 = ax1, axe2 = ax2;
        if (!axe1.IsParallel(axe2, 1.0e-4))
        {
            Vector3D vtrans = new(pos1, pos2), vsign = new();
            double alpha, beta, sign = 1;
            alpha = vtrans.Dot(axe1);
            beta = vtrans.Dot(axe2);
            if (alpha < -1.0e-7)
            {
                axe1 *= -1;
            }
            if (beta < 1.0e-7)
            {
                axe2 *= -1;
            }
            alpha = vtrans.Dot(axe1);
            beta = vtrans.Dot(axe2);
            Vector3D norm2 = axe1 ^ axe2;
            vsign = Vector3D.LinearForm(vtrans.Dot(axe1), axe2, -vtrans.Dot(axe2), axe1);
            alpha = vsign.Dot(axe1);
            beta = vsign.Dot(axe2);
            bool pasnul = (Math.Abs(alpha) > 1.0e-4 && Math.Abs(beta) > 1.0e-4);
            if (alpha * beta > 0.0 && pasnul)
            {
                sign = -1;
            }
            Axis3D norm = new(pos2, new Direction3D(norm2));
            double ang = axe1.AngleWithRef(axe2, norm2);
            if (!withRotation)
            {
                if (ang > M_PI / 2)
                {
                    ang = ang - M_PI;
                }
                if (ang < -M_PI / 2)
                {
                    ang = ang + M_PI;
                }
            }
            ang *= sign;
            pnew = pnew.Rotated(norm, ang);
        }
    }

    private static void BuildConnectedEdges(in Wire aWire,
                                            in Edge startEdge,
                                            in Vertex startVertex,
                                            NCollection_List<Shape> connectedEdges)
    {
        IndexedMap<Shape, NCollection_List<Shape>> mapVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aWire, ShapeType.Vertex, ShapeType.Edge, mapVE);
        Edge curEdge = startEdge;
        Vertex curVertex = startVertex;
        Vertex origin;
        Exploration.Vertices(startEdge, out Vertex v1, out Vertex v2);
        origin = (v1.IsSame(startVertex)) ? v2 : v1;

        // A walk round a wire meets each edge once. A section passing a vertex twice could circle for ever, growing the list (decision 21506)
        IndexedSet<Shape> edges = new(ShapeHasher.Instance);
        Exploration.MapShapes(aWire, ShapeType.Edge, edges);
        for (int walked = 0; ; walked++)
        {
            if (walked > edges.Count)
            {
                throw new global::Kerf.Engine.Foundation.FailureException("A section's wire can't be walked round from its start.");
            }
            NCollection_List<Shape>.Iterator itE = new(mapVE.GetValue(curVertex));
            for (; itE.More(); itE.Next())
            {
                Edge edge = ShapeCasts.Edge(itE.Value());
                if (!edge.IsSame(curEdge))
                {
                    connectedEdges.Append(edge);
                    Exploration.Vertices(edge, out v1, out v2);
                    curVertex = (v1.IsSame(curVertex)) ? v2 : v1;
                    curEdge = edge;
                    break;
                }
            }
            if (curVertex.IsSame(origin))
            {
                break;
            }
        }
    }

    //=================================================================================================

    /// <summary>An empty algorithm; <see cref="Init"/> gives it the sections.</summary>
    public CompatibleWires()
    {
        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>The algorithm on the sections <paramref name="sections"/> (wires, in order).</summary>
    public CompatibleWires(NCollection_Sequence<Shape> sections)
    {
        Init(sections);
    }

    //=================================================================================================

    /// <summary>Sets the sections (wires, in order) and resets the algorithm.</summary>
    public void Init(NCollection_Sequence<Shape> sections)
    {
        _work = new NCollection_Sequence<Shape>(sections);
        _percent = 0.1;
        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotDone;
        _map.Clear();
    }

    //=================================================================================================

    /// <summary>
    /// Sets the fraction of an edge's range within which an intersection is taken as the edge's vertex rather than a
    /// cut (only values strictly between 0 and 1 are taken; 0.1 by default).
    /// </summary>
    public void SetPercent(double percent = 0.01)
    {
        if (0.0 < percent && percent < 1.0)
        {
            _percent = percent;
        }
    }

    //=================================================================================================

    /// <summary>True if <see cref="Perform"/> succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _status == ThruSectionErrorStatus.ThruSectionErrorStatus_Done;
        }
    }

    /// <summary>The status of the last <see cref="Perform"/>.</summary>
    public ThruSectionErrorStatus GetStatus() => _status;

    //=================================================================================================

    /// <summary>The compatible wires, one per section, frozen.</summary>
    public IReadOnlyList<Shape> Shape()
    {
        var shapes = new Shape[_work.Length()];
        for (int i = 1; i <= _work.Length(); i++)
        {
            shapes[i - 1] = _work.Value(i);
            shapes[i - 1].TShape()?.FreezeGraph();
        }
        return ImmutableArrays.Own(shapes);
    }

    /// <summary>The algorithm's own sequence of wires, not frozen, for ported callers.</summary>
    internal NCollection_Sequence<Shape> InternalShape()
    {
        return _work;
    }

    //=================================================================================================

    /// <summary>The edges made from the edge <paramref name="subSection"/> of a section, frozen (empty if none).</summary>
    public IReadOnlyList<Shape> GeneratedShapes(in Edge subSection) => Published(InternalGeneratedShapes(subSection));

    private static readonly NCollection_List<Shape> Empty = new();

    /// <summary>The algorithm's own list of the edges made from <paramref name="subSection"/>, for ported callers.</summary>
    internal NCollection_List<Shape> InternalGeneratedShapes(in Edge subSection)
    {

        if (_map.ContainsKey(subSection))
        {
            return _map.GetValue(subSection);
        }
        else
        {
            return Empty;
        }
    }

    /// <summary>
    /// Each edge of the sections with the edges made from it (itself when it wasn't cut), in the order of the
    /// sections and their edges, frozen.
    /// </summary>
    public IReadOnlyList<KeyValuePair<Shape, IReadOnlyList<Shape>>> Generated()
    {
        var pairs = new KeyValuePair<Shape, IReadOnlyList<Shape>>[_map.Count];
        int i = 0;
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in _map)
        {
            Shape key = it.Key;
            key.TShape()?.FreezeGraph();
            pairs[i++] = new KeyValuePair<Shape, IReadOnlyList<Shape>>(key, Published(it.Value));
        }
        return ImmutableArrays.Own(pairs);
    }

    /// <summary>The algorithm's own map from the sections' edges to the edges made from them, for ported callers.</summary>
    internal Dictionary<Shape, NCollection_List<Shape>> InternalGenerated()
    {
        return _map;
    }

    private static IReadOnlyList<Shape> Published(NCollection_List<Shape> list)
    {
        if (list.IsEmpty())
        {
            return Array.Empty<Shape>();
        }
        var shapes = new Shape[list.Extent()];
        int i = 0;
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            shapes[i] = it.Value();
            shapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return ImmutableArrays.Own(shapes);
    }

    //=================================================================================================

    /// <summary>True if the first section is a point (a wire of degenerated edges).</summary>
    public bool IsDegeneratedFirstSection
    {
        get
        {
            return _degen1;
        }
    }

    //=================================================================================================

    /// <summary>True if the last section is a point (a wire of degenerated edges).</summary>
    public bool IsDegeneratedLastSection
    {
        get
        {
            return _degen2;
        }
    }

    //=================================================================================================

    /// <summary>
    /// Makes the wires compatible: same number of edges, origins and orientations chosen to avoid twisting
    /// (<paramref name="withRotation"/> lets the polar method follow the rotation between the sections' planes).
    /// </summary>
    public void Perform(bool withRotation = true)
    {
        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Done;
        // compute origin and orientation on wires to avoid twisted results
        // and update wires to have same number of edges

        // determination of report:
        // if the number of elements is the same and if the wires have discontinuities
        // by tangency, the report is not carried out by curvilinear abscissa
        int nbSects = _work.Length(), i;
        WireExplorer exp = new();
        int nbmax = 0, nbmin = 0;
        NCollection_Array1<int> nbEdges = new(1, nbSects);
        bool report;
        Continuity contS = CN;
        Continuity cont;
        for (i = 1; i <= nbSects; i++)
        {
            Shape localShape = _work[i].Oriented(Forward);
            _work[i] = ShapeCasts.Wire(localShape);
            //    myWork(i) = TopoDS::Wire(myWork(i).Oriented(TopAbs_FORWARD));
            Wire w = ShapeCasts.Wire(_work[i]);
            WireContinuity(w, out cont);
            if (cont < contS)
            {
                contS = cont;
            }
            nbEdges[i] = 0;
            for (exp.Init(w); exp.More(); exp.Next())
            {
                nbEdges[i]++;
            }
            if (i == 1)
            {
                nbmin = nbEdges[i];
            }
            if (nbmax < nbEdges[i])
            {
                nbmax = nbEdges[i];
            }
            if (nbmin > nbEdges[i])
            {
                nbmin = nbEdges[i];
            }
        }
        // if the number of elements is not the same or if all wires are at least
        // C1, the report is carried out by curvilinear abscissa of cuts, otherwise
        // a report vertex / Vertex is done
        report = (nbmax != nbmin || contS >= C1);

        // initialization of the map
        for (i = 1; i <= nbSects; i++)
        {
            Wire w = ShapeCasts.Wire(_work[i]);
            for (exp.Init(w); exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                _map[e] = new NCollection_List<Shape>();
                _map.GetValue(e).Append(e);
            }
        }

        // open/closed sections
        // initialisation of myDegen1, myDegen2
        int ideb = 1, ifin = _work.Length();
        // check if the first wire is punctual
        _degen1 = true;
        for (exp.Init(ShapeCasts.Wire(_work[ideb])); exp.More(); exp.Next())
        {
            _degen1 = _degen1 && (Tool.Degenerated(exp.Current()));
        }
        if (_degen1)
        {
            ideb++;
        }
        // check if the last wire is punctual
        _degen2 = true;
        for (exp.Init(ShapeCasts.Wire(_work[ifin])); exp.More(); exp.Next())
        {
            _degen2 = _degen2 && (Tool.Degenerated(exp.Current()));
        }
        if (_degen2)
        {
            ifin--;
        }

        bool wClosed, allClosed = true, allOpen = true;
        for (i = ideb; i <= ifin; i++)
        {
            wClosed = _work[i].ClosedFlag;
            if (!wClosed)
            {
                // check if the vertices are the same.
                Exploration.Vertices(ShapeCasts.Wire(_work[i]), out Vertex v1, out Vertex v2);
                if (v1.IsSame(v2))
                {
                    wClosed = true;
                }
            }
            allClosed = (allClosed && wClosed);
            allOpen = (allOpen && !wClosed);
        }

        if (allClosed)
        {
            // All sections are closed
            if (report)
            {
                // same number of elements
                SameNumberByPolarMethod(withRotation);
            }
            else
            {
                // origin
                ComputeOrigin(false);
            }
        }
        else if (allOpen)
        {
            // All sections are open
            // origin
            SearchOrigin();
            if (_status != ThruSectionErrorStatus.ThruSectionErrorStatus_Done)
            {
                return;
            }
            // same number of elements
            if (report)
            {
                SameNumberByACR(report);
            }
        }
        else
        {
            // There are open and closed sections :
            // not processed
            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotSameTopology;
            return;
        }
    }

    //=================================================================================================

    private void SameNumberByPolarMethod(bool withRotation)
    {

        // initialisation
        int nbSects = _work.Length();
        WireExplorer exp = new();
        Dictionary<Shape, NCollection_Sequence<Shape>> edgeNewEdges = new(ShapeHasher.Instance);

        bool allClosed = true;
        int i, ii, ideb = 1, ifin = nbSects;

        for (i = 1; i <= nbSects; i++)
        {
            WireCheck checker = new(ShapeCasts.Wire(_work[i]));
            allClosed = (allClosed && (checker.Closed() == StatusCheck.NoError));
            // allClosed = (allClosed && myWork(i).Closed());
        }
        if (!allClosed)
        {
            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotSameTopology;
            return;
        }

        // sections ponctuelles, sections bouclantes ?
        if (_degen1)
        {
            ideb++;
        }
        if (_degen2)
        {
            ifin--;
        }
        bool vClosed = (!_degen1) && (!_degen2) && (_work[ideb].IsSame(_work[ifin]));

        // Removing degenerated edges
        for (i = ideb; i <= ifin; i++)
        {
            bool hasDegEdge = false;
            Iterator itw = new(_work[i]);
            for (; itw.More(); itw.Next())
            {
                Edge edge = ShapeCasts.Edge(itw.Value());
                if (Tool.Degenerated(edge))
                {
                    hasDegEdge = true;
                    break;
                }
            }
            if (hasDegEdge)
            {
                Wire newWire = new();
                BrepBuilder bBuilder = new();
                bBuilder.MakeWire(ref newWire);
                for (itw.Initialize(_work[i]); itw.More(); itw.Next())
                {
                    Edge edge = ShapeCasts.Edge(itw.Value());
                    if (!Tool.Degenerated(edge))
                    {
                        bBuilder.Add(newWire, edge);
                    }
                }
                _work[i] = newWire;
            }
        }

        // Nombre max de decoupes possibles
        int nbMaxV = 0;
        for (i = 1; i <= nbSects; i++)
        {
            for (exp.Init(ShapeCasts.Wire(_work[i])); exp.More(); exp.Next())
            {
                nbMaxV++;
            }
        }

        // construction of tables of planes of wires
        NCollection_HArray1<Point3D> pos = new(1, nbSects);
        NCollection_HArray1<Vector3D> axe = new(1, nbSects);
        for (i = ideb; i <= ifin; i++)
        {
            if (PlaneOfWire(ShapeCasts.Wire(_work[i]), out Plane3D p))
            {
                pos.SetValue(i, p.Location);
                axe.SetValue(i, new Vector3D(p.Axis.Direction));
            }
        }
        NCollection_Sequence<Shape> seqV = new();
        if (_degen1)
        {
            SeqOfVertices(ShapeCasts.Wire(_work[1]), seqV);
            pos.SetValue(1, Tool.Pnt(ShapeCasts.Vertex(seqV.Value(1))));
            axe.SetValue(1, axe.Value(ideb));
        }
        if (_degen2)
        {
            SeqOfVertices(ShapeCasts.Wire(_work[nbSects]), seqV);
            pos.SetValue(nbSects, Tool.Pnt(ShapeCasts.Vertex(seqV.Value(1))));
            axe.SetValue(nbSects, axe.Value(ifin));
        }

        // construction of RMap, map of reports of wire i to wire i-1
        Dictionary<Shape, NCollection_List<Shape>> rMap = new(ShapeHasher.Instance);
        rMap.Clear();

        // loop on i
        for (i = ifin; i > ideb; i--)
        {

            Wire wire1 = ShapeCasts.Wire(_work[i]);

            // sequence of vertices of the first wire
            SeqOfVertices(wire1, seqV);
            if (seqV.Length() > nbMaxV)
            {
                _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Failed;
                return;
            }

            // loop on vertices of wire1
            for (ii = 1; ii <= seqV.Length(); ii++)
            {

                Vertex vi = ShapeCasts.Vertex(seqV.Value(ii));

                // init of RMap for Vi
                NCollection_List<Shape> init = new();
                init.Clear();
                rMap[vi] = init;

                // it is required to find intersection Vi - wire2
                Point3D pi = Tool.Pnt(vi);

                // return Pi in the current plane
                Transform(withRotation,
                          pi,
                          pos.Value(i),
                          axe.Value(i),
                          pos.Value(i - 1),
                          axe.Value(i - 1),
                          out Point3D pnew);

                // calculate the intersection
                bool newVertex;
                Vertex vsol = new();
                Wire newwire = new();
                if (pnew.DistanceTo(pos.Value(i - 1)) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    double percent = _percent;
                    newVertex = EdgeIntersectOnWire(pos.Value(i - 1),
                                                    pnew,
                                                    percent,
                                                    rMap,
                                                    ShapeCasts.Wire(_work[i - 1]),
                                                    ref vsol,
                                                    ref newwire,
                                                    edgeNewEdges);
                    if (newVertex)
                    {
                        _work[i - 1] = newwire;
                    }
                    rMap.GetValue(vi).Append(vsol);
                }

            } // loop on  ii
        } // loop on  i

        // initialisation of MapVLV, map of correspondences vertex - list of vertices
        Dictionary<Shape, NCollection_List<Shape>> mapVLV = new(ShapeHasher.Instance);
        SeqOfVertices(ShapeCasts.Wire(_work[ideb]), seqV);
        int sizeMap = seqV.Length();
        mapVLV.Clear();
        for (ii = 1; ii <= sizeMap; ii++)
        {
            Vertex vi = ShapeCasts.Vertex(seqV.Value(ii));
            NCollection_List<Shape> init = new();
            init.Clear();
            init.Append(vi);
            mapVLV[vi] = init;
            int nbV = 1;
            Vertex v0;
            v0 = vi;
            bool tantque = SearchRoot(v0, rMap, out Vertex v1);
            while (tantque)
            {
                mapVLV.GetValue(vi).Append(v1);
                nbV++;
                // test on NbV required for looping sections
                if (v1.IsSame(vi) || nbV >= _work.Length())
                {
                    tantque = false;
                }
                else
                {
                    v0 = v1;
                    tantque = SearchRoot(v0, rMap, out v1);
                }
            }
        }

        // loop on i
        for (i = ideb; i < ifin; i++)
        {

            Wire wire1 = ShapeCasts.Wire(_work[i]);

            // sequence of vertices of the first wire
            SeqOfVertices(wire1, seqV);
            if (seqV.Length() > nbMaxV || seqV.Length() > sizeMap)
            {
                _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Failed;
                return;
            }

            // next wire
            Wire wire2 = ShapeCasts.Wire(_work[i + 1]);

            // loop on vertices of wire1
            for (ii = 1; ii <= seqV.Length(); ii++)
            {

                Vertex vi = ShapeCasts.Vertex(seqV.Value(ii));
                bool intersect = true;
                if (SearchRoot(vi, mapVLV, out Vertex vRoot))
                {
                    NCollection_List<Shape> lVi = mapVLV.GetValue(vRoot);
                    intersect = (!SearchVertex(lVi, wire2, out _));
                }

                if (intersect)
                {
                    // it is necessary to find intersection Vi - wire2
                    Point3D pi = Tool.Pnt(vi);

                    // return Pi in the current plane
                    Transform(withRotation,
                              pi,
                              pos.Value(i),
                              axe.Value(i),
                              pos.Value(i + 1),
                              axe.Value(i + 1),
                              out Point3D pnew);

                    // calculate the intersection
                    bool newVertex;
                    Vertex vsol = new();
                    Wire newwire = new();
                    if (pnew.DistanceTo(pos.Value(i + 1)) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        double percent = _percent;
                        newVertex = EdgeIntersectOnWire(pos.Value(i + 1),
                                                        pnew,
                                                        percent,
                                                        mapVLV,
                                                        ShapeCasts.Wire(_work[i + 1]),
                                                        ref vsol,
                                                        ref newwire,
                                                        edgeNewEdges);
                        // VRoot is null when SearchRoot found nothing: the map then has no such key and both OCCT's
                        // operator() and the port's indexer throw Standard_NoSuchObject.
                        mapVLV.GetValue(vRoot).Append(vsol);
                        if (newVertex)
                        {
                            _work[i + 1] = newwire;
                        }
                    }
                }
            } // loop on ii
        } // loop on i

        // regularize wires following MapVLV
        Wire wire = ShapeCasts.Wire(_work[ideb]);

        // except for the last if the sections loop
        int ibout = ifin;
        if (vClosed)
        {
            ibout--;
        }

        for (i = ideb + 1; i <= ibout; i++)
        {

            LibraryMakeWire mw = new();

            exp.Init(wire);
            Edge eCur = exp.Current();
            Exploration.Vertices(eCur, out Vertex vf, out Vertex vl, true);
            double u1 = Tool.Parameter(vf, eCur);
            double u2 = Tool.Parameter(vl, eCur);
            EdgeCurveAdaptor curve = new(eCur);
            Point3D pPs = curve.Value(0.1 * (u1 + 9 * u2));
            NCollection_List<Shape>.Iterator itF = new(mapVLV.GetValue(vf)), itL = new(mapVLV.GetValue(vl));
            int rang = ideb;
            while (rang < i && itF.More() && itL.More())
            {
                itF.Next();
                itL.Next();
                rang++;
            }
            if (!itF.More() || !itL.More())
            {
                // Correspondence chain shorter than the section index; fail gracefully.
                _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Failed;
                return;
            }
            Vertex v1 = ShapeCasts.Vertex(itF.Value()), v2 = ShapeCasts.Vertex(itL.Value());
            Edge esol = new();
            double scalmax = 0.0;
            Iterator itW = new(_work[i]);

            for (; itW.More(); itW.Next())
            {
                Edge e = ShapeCasts.Edge(itW.Value());
                Exploration.Vertices(e, out Vertex vvf, out Vertex vvl, true);

                // parse candidate edges
                double scal1, scal2;
                if ((v1.IsSame(vvf) && v2.IsSame(vvl)) || (v2.IsSame(vvf) && v1.IsSame(vvl)))
                {
                    double u1param = Tool.Parameter(vvf, e);
                    double u2param = Tool.Parameter(vvl, e);
                    EdgeCurveAdaptor curveE = new(e);
                    Point3D pp1 = curveE.Value(0.1 * (u1param + 9 * u2param));
                    Point3D pp2 = curveE.Value(0.1 * (9 * u1param + u2param));

                    for (rang = i; rang > ideb; rang--)
                    {
                        Transform(withRotation,
                                  pp1,
                                  pos.Value(rang),
                                  axe.Value(rang),
                                  pos.Value(rang - 1),
                                  axe.Value(rang - 1),
                                  out pp1);
                        Transform(withRotation,
                                  pp2,
                                  pos.Value(rang),
                                  axe.Value(rang),
                                  pos.Value(rang - 1),
                                  axe.Value(rang - 1),
                                  out pp2);
                    }
                    Vector3D ns = new(pos.Value(ideb), pPs);
                    ns = ns.Normalized();
                    Vector3D n1 = new(pos.Value(ideb), pp1);
                    n1 = n1.Normalized();
                    Vector3D n2 = new(pos.Value(ideb), pp2);
                    n2 = n2.Normalized();
                    scal1 = n1.Dot(ns);
                    if (scal1 > scalmax)
                    {
                        scalmax = scal1;
                        esol = e;
                    }
                    scal2 = n2.Dot(ns);
                    if (scal2 > scalmax)
                    {
                        scalmax = scal2;
                        Shape localShape = e.Reversed();
                        esol = ShapeCasts.Edge(localShape);
                    }
                }
            } // end of for(; itW.More(); itW.Next())
            if (esol.IsNull)
            {
                _status = ThruSectionErrorStatus.ThruSectionErrorStatus_ProfilesInconsistent;
                return;
            }
            mw.Add(esol);

            NCollection_List<Shape> connectedEdges = new();
            BuildConnectedEdges(ShapeCasts.Wire(_work[i]), esol, v2, connectedEdges);

            NCollection_List<Shape>.Iterator itCE = new(connectedEdges);
            for (; exp.More() && itCE.More(); exp.Next(), itCE.Next())
            {
                eCur = exp.Current();
                Exploration.Vertices(eCur, out vf, out vl, true);
                u1 = Tool.Parameter(vf, eCur);
                u2 = Tool.Parameter(vl, eCur);
                curve.Initialize(eCur);
                pPs = curve.Value(0.1 * (u1 + 9 * u2));

                Edge e = ShapeCasts.Edge(itCE.Value());
                Exploration.Vertices(e, out Vertex vvf, out Vertex vvl, true);

                // parse candidate edges
                double scal1, scal2;
                u1 = Tool.Parameter(vvf, e);
                u2 = Tool.Parameter(vvl, e);
                curve.Initialize(e);
                Point3D pp1 = curve.Value(0.1 * (u1 + 9 * u2));
                Point3D pp2 = curve.Value(0.1 * (9 * u1 + u2));

                for (rang = i; rang > ideb; rang--)
                {
                    Transform(withRotation,
                              pp1,
                              pos.Value(rang),
                              axe.Value(rang),
                              pos.Value(rang - 1),
                              axe.Value(rang - 1),
                              out pp1);
                    Transform(withRotation,
                              pp2,
                              pos.Value(rang),
                              axe.Value(rang),
                              pos.Value(rang - 1),
                              axe.Value(rang - 1),
                              out pp2);
                }
                Vector3D ns = new(pos.Value(ideb), pPs);
                ns = ns.Normalized();
                Vector3D n1 = new(pos.Value(ideb), pp1);
                n1 = n1.Normalized();
                Vector3D n2 = new(pos.Value(ideb), pp2);
                n2 = n2.Normalized();
                scal1 = n1.Dot(ns);
                scal2 = n2.Dot(ns);
                if (scal2 > scal1)
                {
                    e.Reverse();
                }
                mw.Add(e);
            }
            _work[i] = mw.InternalWire();
        }

        // blocking sections?
        if (vClosed)
        {
            Iterator iter1 = new(_work[_work.Length()]), iter2 = new(_work[1]);
            for (; iter1.More(); iter1.Next(), iter2.Next())
            {
                Shape edge = iter1.Value();
                Shape newEdge = iter2.Value();
                if (!edge.IsSame(newEdge))
                {
                    NCollection_Sequence<Shape> seq = new();
                    seq.Append(newEdge);
                    edgeNewEdges[edge] = seq;
                }
            }
            _work[_work.Length()] = _work[1];
        }

        // check the number of edges for debug
        int nbmax = 0, nbmin = 0;
        for (i = ideb; i <= ifin; i++)
        {
            int nbEdges = 0;
            for (exp.Init(ShapeCasts.Wire(_work[i])); exp.More(); exp.Next())
            {
                nbEdges++;
            }
            if (i == ideb)
            {
                nbmin = nbEdges;
            }
            if (nbmax < nbEdges)
            {
                nbmax = nbEdges;
            }
            if (nbmin > nbEdges)
            {
                nbmin = nbEdges;
            }
        }
        if (nbmin != nbmax)
        {
            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Failed;
            return;
        }

        // Fill <myMap>
        foreach (Shape edge in new List<Shape>(_map.Keys))
        {
            NCollection_List<Shape> listOfNewEdges = new();

            // for each edge of <myMap> find all newest edges
            // in <EdgeNewEdges> recursively
            AddNewEdge(edge, edgeNewEdges, listOfNewEdges);

            _map.GetValueRef(edge) = listOfNewEdges;
        }
    }

    //=================================================================================================

    private void SameNumberByACR(bool report)
    {
        // find the dimension
        int ideb = 1, ifin = _work.Length();
        WireExplorer exp = new();

        // point sections, blocking  sections?
        if (_degen1)
        {
            ideb++;
        }
        if (_degen2)
        {
            ifin--;
        }
        bool vClosed = (!_degen1) && (!_degen2) && (_work[ideb].IsSame(_work[ifin]));

        int nbSects = _work.Length(), i;
        int nbmax = 0, nbmin = 0;
        NCollection_Array1<int> nbEdges = new(1, nbSects);
        for (i = 1; i <= nbSects; i++)
        {
            nbEdges[i] = 0;
            for (exp.Init(ShapeCasts.Wire(_work[i])); exp.More(); exp.Next())
            {
                nbEdges[i]++;
            }
            if (i == 1)
            {
                nbmin = nbEdges[i];
            }
            if (nbmax < nbEdges[i])
            {
                nbmax = nbEdges[i];
            }
            if (nbmin > nbEdges[i])
            {
                nbmin = nbEdges[i];
            }
        }

        if (nbmax > 1)
        {
            // several edges

            if (report || nbmin < nbmax)
            {
                // insertion of cuts
                int nbdec = (nbmax - 1) * nbSects + 1;
                NCollection_Array1<double> dec = new(1, nbdec);
                dec.Init(0);
                dec[2] = 1;

                NCollection_Array1<double> wireLen = new(1, nbSects);

                // calculate the table of cuts
                int j, k, l;
                for (i = 1; i <= nbSects; i++)
                {
                    // current wire
                    Wire wire1 = ShapeCasts.Wire(_work[i]);
                    int nbE = 0;
                    for (exp.Init(wire1); exp.More(); exp.Next())
                    {
                        nbE++;
                    }
                    // length and ACR of the wire
                    NCollection_Array1<double> acr = new(0, nbE);
                    acr.Init(0);
                    Fill.ComputeACR(wire1, acr);
                    wireLen[i] = acr[0];
                    // insertion of ACR of the wire in the table of cuts
                    for (j = 1; j < acr.Length() - 1; j++)
                    {
                        k = 1;
                        while (dec[k] < acr[j])
                        {
                            k++;
                            if (k > nbdec)
                            {
                                break;
                            }
                        }
                        if (dec[k - 1] < acr[j] && acr[j] < dec[k])
                        {
                            for (l = nbdec - 1; l >= k; l--)
                            {
                                dec[l + 1] = dec[l];
                            }
                            dec[k] = acr[j];
                        }
                    }
                }

                // table of cuts
                k = 1;
                while (dec[k] < 1)
                {
                    k++;
                    if (k > nbdec)
                    {
                        break;
                    }
                }
                nbdec = k - 1;
                NCollection_Array1<double> dec2 = new(1, nbdec);
                for (k = 1; k <= nbdec; k++)
                {
                    dec2[k] = dec[k];
                }

                // Check of cuts: are all the new edges long enough or not
                HashSet<int> cutsToRemove = new();
                for (k = 1; k <= nbdec; k++)
                {
                    double knot1 = dec2[k];
                    double knot2 = (k == nbdec) ? 1.0 : dec2[k + 1];
                    bool allLengthsNull = true; // OCCT: declared double, holding a bool
                    for (i = 1; i <= nbSects; i++)
                    {
                        double edgeLen = (knot2 - knot1) * wireLen[i];
                        if (edgeLen > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            allLengthsNull = false;
                            break;
                        }
                    }
                    if (allLengthsNull)
                    {
                        cutsToRemove.Add(k);
                    }
                }
                int newNbDec = nbdec - cutsToRemove.Count;
                NCollection_Array1<double> dec3 = new(1, newNbDec);
                i = 1;
                for (k = 1; k <= nbdec; k++)
                {
                    if (!cutsToRemove.Contains(k))
                    {
                        dec3[i++] = dec2[k];
                    }
                }
                ///////////////////

                // insertion of cuts in each wire
                for (i = 1; i <= nbSects; i++)
                {
                    Wire oldwire = ShapeCasts.Wire(_work[i]);
                    double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                    if (wireLen[i] > Constants.Resolution())
                    {
                        tol /= wireLen[i];
                    }
                    Wire newwire = Fill.InternalInsertACR(oldwire, dec3, tol);
                    WireExplorer exp1 = new(), exp2 = new();
                    exp1.Init(oldwire);
                    exp2.Init(newwire);
                    for (; exp1.More(); exp1.Next())
                    {
                        Edge ecur = exp1.Current();
                        if (!ecur.IsSame(ShapeCasts.Edge(exp2.Current())))
                        {
                            NCollection_List<Shape> le = new();
                            le.Clear();
                            Point3D p1 = new(), p2 = new();
                            Vertex v1 = exp1.CurrentVertex();
                            Exploration.Vertices(ecur, out Vertex vf, out Vertex vr, true);
                            if (v1.IsSame(vf))
                            {
                                p1 = Tool.Pnt(vr);
                            }
                            if (v1.IsSame(vr))
                            {
                                p1 = Tool.Pnt(vf);
                            }
                            Vertex v2 = exp2.CurrentVertex();
                            Exploration.Vertices(ShapeCasts.Edge(exp2.Current()), out vf, out vr, true);
                            if (v2.IsSame(vf))
                            {
                                p2 = Tool.Pnt(vr);
                            }
                            if (v2.IsSame(vr))
                            {
                                p2 = Tool.Pnt(vf);
                            }
                            while (p1.DistanceTo(p2) > 1.0e-3)
                            {
                                le.Append(exp2.Current());
                                exp2.Next();
                                v2 = exp2.CurrentVertex();
                                Exploration.Vertices(ShapeCasts.Edge(exp2.Current()), out vf, out vr, true);
                                if (v2.IsSame(vf))
                                {
                                    p2 = Tool.Pnt(vr);
                                }
                                if (v2.IsSame(vr))
                                {
                                    p2 = Tool.Pnt(vf);
                                }
                                if (p1.DistanceTo(p2) <= 1.0e-3)
                                {
                                    le.Append(exp2.Current());
                                    exp2.Next();
                                }
                            }

                            // NCollection_List<TopoDS_Shape>::Iterator itlist;
                            Edge ancestor;
                            bool found = false;

                            // OCCT iterates myMap in address order; the port in binding order (decision 908). An edge
                            // is in one list only, so the order doesn't change the result.
                            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itmap in _map)
                            {
                                if (found)
                                {
                                    break;
                                }
                                NCollection_List<Shape>.Iterator itlist = new(itmap.Value);
                                while (itlist.More() && (!found))
                                {
                                    Edge eCur = ShapeCasts.Edge(itlist.Value());

                                    if (ecur.IsSame(eCur))
                                    {
                                        ancestor = ShapeCasts.Edge(itmap.Key);
                                        found = true;
                                        // myMap(Ancestor).InsertBefore(LE, itlist): the list LE before the iterator's
                                        // item, the iterator staying on that item
                                        NCollection_List<Shape> list = _map.GetValue(ancestor);
                                        for (NCollection_List<Shape>.Iterator itLE = new(le); itLE.More(); itLE.Next())
                                        {
                                            list.InsertBefore(itLE.Value(), itlist);
                                        }
                                        list.Remove(itlist);
                                    }
                                    if (itlist.More())
                                    {
                                        itlist.Next();
                                    }
                                }
                            }
                        }
                        else
                        {
                            exp2.Next();
                        }
                    }
                    _work[i] = newwire;
                }
            }
        }

        // blocking sections ?
        if (vClosed)
        {
            _work[_work.Length()] = _work[1];
        }

        // check the number of edges for debug
        nbmax = 0;
        for (i = ideb; i <= ifin; i++)
        {
            nbEdges[i] = 0;
            for (exp.Init(ShapeCasts.Wire(_work[i])); exp.More(); exp.Next())
            {
                nbEdges[i]++;
            }
            if (i == ideb)
            {
                nbmin = nbEdges[i];
            }
            if (nbmax < nbEdges[i])
            {
                nbmax = nbEdges[i];
            }
            if (nbmin > nbEdges[i])
            {
                nbmin = nbEdges[i];
            }
        }
        if (nbmax != nbmin)
        {
            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Failed;
            return;
        }
    }

    //=================================================================================================

    private void ComputeOrigin(bool polar)
    {
        _ = polar;
        // reorganize the wires respecting orientation and origin

        WireExplorer exp = new();

        bool wClosed, allClosed = true;

        int nbSects = _work.Length();
        int i, ideb = 1, ifin = nbSects;

        // point sections, blocking sections
        if (_degen1)
        {
            ideb++;
        }
        if (_degen2)
        {
            ifin--;
        }
        bool vClosed = (!_degen1) && (!_degen2) && (_work[ideb].IsSame(_work[ifin]));

        for (i = ideb; i <= ifin; i++)
        {
            wClosed = _work[i].ClosedFlag;
            if (!wClosed)
            {
                // check if the vertices are the same.
                Exploration.Vertices(ShapeCasts.Wire(_work[i]), out Vertex v1, out Vertex v2);
                if (v1.IsSame(v2))
                {
                    wClosed = true;
                }
            }
            allClosed = (allClosed && wClosed);
        }
        if (!allClosed)
        {
            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotSameTopology;
            return;
        }

        // Consider that all wires have same number of edges (polar==false)
        NCollection_Sequence<Shape> prevSeq = new();
        NCollection_Sequence<Shape> prevEseq = new();
        int theLength = 0;
        Wire wire = ShapeCasts.Wire(_work[ideb]);
        for (exp.Init(wire); exp.More(); exp.Next())
        {
            prevSeq.Append(exp.CurrentVertex());
            prevEseq.Append(exp.Current());
            theLength++;
        }

        int nbs, nbSamples = 0;
        if (theLength <= 2)
        {
            nbSamples = 4;
        }
        PlaneOfWire(ShapeCasts.Wire(_work[ideb]), out Plane3D firstPlane);
        Point3D prevBary = firstPlane.Location;
        Vector3D normalOfFirstPlane = new(firstPlane.Axis.Direction);
        for (i = ideb + 1; i <= ifin; i++)
        {
            Wire aWire = ShapeCasts.Wire(_work[i]);

            // Compute offset vector as current bary center projected on first plane
            // to first bary center
            PlaneOfWire(aWire, out Plane3D curPlane);
            Point3D curBary = curPlane.Location;
            Vector3D vec = new(prevBary, curBary);
            Vector3D offsetProj = (vec * normalOfFirstPlane) * normalOfFirstPlane;
            curBary = curBary.Translated(-offsetProj); // projected current bary center
            Vector3D offset = new(curBary, prevBary);

            Wire newwire = new();
            BrepBuilder bb = new();
            bb.MakeWire(ref newwire);

            NCollection_Sequence<Shape> seqVertices = new(), seqEdges = new();
            for (exp.Init(aWire); exp.More(); exp.Next())
            {
                seqVertices.Append(exp.CurrentVertex());
                seqEdges.Append(exp.Current());
            }

            double minSumDist = global::Kerf.Engine.Foundation.Precision.Infinite();
            int jmin = 1, j, k, n;
            bool forward = false;
            if (i == _work.Length() && _degen2)
            {
                // last point section
                jmin = 1;
                forward = true;
            }
            else
            {
                for (j = 1; j <= theLength; j++)
                {
                    // Forward
                    double sumDist = 0.0;
                    for (k = j, n = 1; k <= theLength; k++, n++)
                    {
                        Vertex vprev = ShapeCasts.Vertex(prevSeq[n]);
                        Point3D pprev = Tool.Pnt(vprev);
                        Vertex v = ShapeCasts.Vertex(seqVertices[k]);
                        Point3D p = new(Tool.Pnt(v).XYZ() + offset.XYZ());
                        sumDist += pprev.DistanceTo(p);
                        if (nbSamples > 0)
                        {
                            Edge prevEdge = ShapeCasts.Edge(prevEseq[n]);
                            Edge curEdge = ShapeCasts.Edge(seqEdges[k]);
                            EdgeCurveAdaptor prevEcurve = new(prevEdge);
                            EdgeCurveAdaptor ecurve = new(curEdge);
                            double sampleOnPrev =
                              (prevEcurve.LastParameter() - prevEcurve.FirstParameter()) / nbSamples;
                            double sampleOnCur = (ecurve.LastParameter() - ecurve.FirstParameter()) / nbSamples;
                            for (nbs = 1; nbs <= nbSamples - 1; nbs++)
                            {
                                double parOnPrev =
                                  (prevEdge.Orientation == Forward)
                                    ? (prevEcurve.FirstParameter() + nbs * sampleOnPrev)
                                    : (prevEcurve.FirstParameter() + (nbSamples - nbs) * sampleOnPrev);
                                double parOnCur = (curEdge.Orientation == Forward)
                                                    ? (ecurve.FirstParameter() + nbs * sampleOnCur)
                                                    : (ecurve.FirstParameter() + (nbSamples - nbs) * sampleOnCur);
                                Point3D ponPrev = prevEcurve.Value(parOnPrev);
                                Point3D ponCur = new(ecurve.Value(parOnCur).XYZ() + offset.XYZ());
                                sumDist += ponPrev.DistanceTo(ponCur);
                            }
                        }
                    }
                    for (k = 1; k < j; k++, n++)
                    {
                        Vertex vprev = ShapeCasts.Vertex(prevSeq[n]);
                        Point3D pprev = Tool.Pnt(vprev);
                        Vertex v = ShapeCasts.Vertex(seqVertices[k]);
                        Point3D p = new(Tool.Pnt(v).XYZ() + offset.XYZ());
                        sumDist += pprev.DistanceTo(p);
                        if (nbSamples > 0)
                        {
                            Edge prevEdge = ShapeCasts.Edge(prevEseq[n]);
                            Edge curEdge = ShapeCasts.Edge(seqEdges[k]);
                            EdgeCurveAdaptor prevEcurve = new(prevEdge);
                            EdgeCurveAdaptor ecurve = new(curEdge);
                            double sampleOnPrev =
                              (prevEcurve.LastParameter() - prevEcurve.FirstParameter()) / nbSamples;
                            double sampleOnCur = (ecurve.LastParameter() - ecurve.FirstParameter()) / nbSamples;
                            for (nbs = 1; nbs <= nbSamples - 1; nbs++)
                            {
                                double parOnPrev =
                                  (prevEdge.Orientation == Forward)
                                    ? (prevEcurve.FirstParameter() + nbs * sampleOnPrev)
                                    : (prevEcurve.FirstParameter() + (nbSamples - nbs) * sampleOnPrev);
                                double parOnCur = (curEdge.Orientation == Forward)
                                                    ? (ecurve.FirstParameter() + nbs * sampleOnCur)
                                                    : (ecurve.FirstParameter() + (nbSamples - nbs) * sampleOnCur);
                                Point3D ponPrev = prevEcurve.Value(parOnPrev);
                                Point3D ponCur = new(ecurve.Value(parOnCur).XYZ() + offset.XYZ());
                                sumDist += ponPrev.DistanceTo(ponCur);
                            }
                        }
                    }
                    if (sumDist < minSumDist)
                    {
                        minSumDist = sumDist;
                        jmin = j;
                        forward = true;
                    }

                    // Backward
                    sumDist = 0.0;
                    for (k = j, n = 1; k >= 1; k--, n++)
                    {
                        Vertex vprev = ShapeCasts.Vertex(prevSeq[n]);
                        Point3D pprev = Tool.Pnt(vprev);
                        Vertex v = ShapeCasts.Vertex(seqVertices[k]);
                        Point3D p = new(Tool.Pnt(v).XYZ() + offset.XYZ());
                        sumDist += pprev.DistanceTo(p);
                        if (nbSamples > 0)
                        {
                            int k_cur = k - 1;
                            if (k_cur == 0)
                            {
                                k_cur = theLength;
                            }
                            Edge prevEdge = ShapeCasts.Edge(prevEseq[n]);
                            Edge curEdge = ShapeCasts.Edge(seqEdges[k_cur]);
                            EdgeCurveAdaptor prevEcurve = new(prevEdge);
                            EdgeCurveAdaptor ecurve = new(curEdge);
                            double sampleOnPrev =
                              (prevEcurve.LastParameter() - prevEcurve.FirstParameter()) / nbSamples;
                            double sampleOnCur = (ecurve.LastParameter() - ecurve.FirstParameter()) / nbSamples;
                            for (nbs = 1; nbs <= nbSamples - 1; nbs++)
                            {
                                double parOnPrev =
                                  (prevEdge.Orientation == Forward)
                                    ? (prevEcurve.FirstParameter() + nbs * sampleOnPrev)
                                    : (prevEcurve.FirstParameter() + (nbSamples - nbs) * sampleOnPrev);
                                double parOnCur = (curEdge.Orientation == Forward)
                                                    ? (ecurve.FirstParameter() + (nbSamples - nbs) * sampleOnCur)
                                                    : (ecurve.FirstParameter() + nbs * sampleOnCur);
                                Point3D ponPrev = prevEcurve.Value(parOnPrev);
                                Point3D ponCur = new(ecurve.Value(parOnCur).XYZ() + offset.XYZ());
                                sumDist += ponPrev.DistanceTo(ponCur);
                            }
                        }
                    }
                    for (k = theLength; k > j; k--, n++)
                    {
                        Vertex vprev = ShapeCasts.Vertex(prevSeq[n]);
                        Point3D pprev = Tool.Pnt(vprev);
                        Vertex v = ShapeCasts.Vertex(seqVertices[k]);
                        Point3D p = new(Tool.Pnt(v).XYZ() + offset.XYZ());
                        sumDist += pprev.DistanceTo(p);
                        if (nbSamples > 0)
                        {
                            Edge prevEdge = ShapeCasts.Edge(prevEseq[n]);
                            Edge curEdge = ShapeCasts.Edge(seqEdges[k - 1]);
                            EdgeCurveAdaptor prevEcurve = new(prevEdge);
                            EdgeCurveAdaptor ecurve = new(curEdge);
                            double sampleOnPrev =
                              (prevEcurve.LastParameter() - prevEcurve.FirstParameter()) / nbSamples;
                            double sampleOnCur = (ecurve.LastParameter() - ecurve.FirstParameter()) / nbSamples;
                            for (nbs = 1; nbs <= nbSamples - 1; nbs++)
                            {
                                double parOnPrev =
                                  (prevEdge.Orientation == Forward)
                                    ? (prevEcurve.FirstParameter() + nbs * sampleOnPrev)
                                    : (prevEcurve.FirstParameter() + (nbSamples - nbs) * sampleOnPrev);
                                double parOnCur = (curEdge.Orientation == Forward)
                                                    ? (ecurve.FirstParameter() + (nbSamples - nbs) * sampleOnCur)
                                                    : (ecurve.FirstParameter() + nbs * sampleOnCur);
                                Point3D ponPrev = prevEcurve.Value(parOnPrev);
                                Point3D ponCur = new(ecurve.Value(parOnCur).XYZ() + offset.XYZ());
                                sumDist += ponPrev.DistanceTo(ponCur);
                            }
                        }
                    }
                    if (sumDist < minSumDist)
                    {
                        minSumDist = sumDist;
                        jmin = j;
                        forward = false;
                    }
                }
            }

            prevSeq.Clear();
            prevEseq.Clear();
            if (forward)
            {
                for (j = jmin; j <= theLength; j++)
                {
                    bb.Add(newwire, ShapeCasts.Edge(seqEdges[j]));
                    prevSeq.Append(seqVertices[j]);
                    prevEseq.Append(seqEdges[j]);
                }
                for (j = 1; j < jmin; j++)
                {
                    bb.Add(newwire, ShapeCasts.Edge(seqEdges[j]));
                    prevSeq.Append(seqVertices[j]);
                    prevEseq.Append(seqEdges[j]);
                }
            }
            else
            {
                for (j = jmin - 1; j >= 1; j--)
                {
                    Shape localShape = seqEdges[j].Reversed();
                    bb.Add(newwire, ShapeCasts.Edge(localShape));
                    // PrevSeq.Append( SeqVertices(j) );
                    prevEseq.Append(seqEdges[j].Reversed());
                }
                for (j = theLength; j >= jmin; j--)
                {
                    Shape localShape = seqEdges[j].Reversed();
                    bb.Add(newwire, ShapeCasts.Edge(localShape));
                    // PrevSeq.Append( SeqVertices(j) );
                    prevEseq.Append(seqEdges[j].Reversed());
                }
                for (j = jmin; j >= 1; j--)
                {
                    prevSeq.Append(seqVertices[j]);
                }
                for (j = theLength; j > jmin; j--)
                {
                    prevSeq.Append(seqVertices[j]);
                }
            }

            newwire.Closed(true);
            newwire.Orient(Forward);
            _work[i] = newwire;

            prevBary = curBary;
        }

        // blocking sections ?
        if (vClosed)
        {
            _work[_work.Length()] = _work[1];
        }
    }

    //=================================================================================================

    private void SearchOrigin()
    {
        // reorganize the open wires respecting orientation and origin

        Plane3D p0, p;

        Vertex vdeb, vfin;
        Point3D pdeb, pfin; //,Psuiv;

        WireExplorer exp = new();

        bool allOpen = true;
        int ideb = 1, ifin = _work.Length();
        if (_degen1)
        {
            ideb++;
        }
        if (_degen2)
        {
            ifin--;
        }
        bool vClosed = (!_degen1) && (!_degen2) && (_work[ideb].IsSame(_work[ifin]));

        //  for (int i=ideb; i<=ifin; i++) {
        int i;
        for (i = ideb; i <= ifin; i++)
        {
            allOpen = (allOpen && !_work[i].ClosedFlag);
        }
        if (!allOpen)
        {
            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotSameTopology;
            return;
        }

        // init

        Wire wire1 = ShapeCasts.Wire(_work[ideb]);
        wire1.Orient(Forward);
        Exploration.Vertices(wire1, out vdeb, out vfin);
        pdeb = Tool.Pnt(vdeb);
        pfin = Tool.Pnt(vfin);
        bool isline0 = (!PlaneOfWire(wire1, out p0)), isline;
        _work[ideb] = wire1;
        // OCC86
        exp.Init(wire1);
        Edge e0 = exp.Current(), e;

        for (i = ideb + 1; i <= ifin; i++)
        {

            Wire wire = ShapeCasts.Wire(_work[i]);
            wire.Orient(Forward);

            NCollection_Sequence<Shape> seqEdges = new();
            seqEdges.Clear();
            int nbEdges = 0;
            // OCC86  for(anExp.Init(wire); anExp.More(); anExp.Next()) {
            for (exp.Init(wire), e = exp.Current(); exp.More(); exp.Next())
            {
                seqEdges.Append(exp.Current());
                nbEdges++;
            }
            Exploration.Vertices(wire, out vdeb, out vfin);
            isline = (!PlaneOfWire(wire, out p));

            Wire newwire = new();
            BrepBuilder bw = new();
            bw.MakeWire(ref newwire);
            bool parcours = true;

            if (isline0 || isline)
            {

                // particular case of straight segments
                Point3D p1 = Tool.Pnt(vdeb), p2 = Tool.Pnt(vfin);
                double dist1, dist2;
                dist1 = pdeb.DistanceTo(p1) + pfin.DistanceTo(p2);
                dist2 = pdeb.DistanceTo(p2) + pfin.DistanceTo(p1);
                parcours = (dist2 >= dist1);
            }

            else
            {
                // OCC86
                Point3D p1 = Tool.Pnt(vdeb), p1o = pdeb, p2 = Tool.Pnt(vfin), p2o = pfin;
                if (p1.IsEqual(p2, global::Kerf.Engine.Foundation.Precision.Confusion()) || p1o.IsEqual(p2o, global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    EdgeCurveAdaptor curve0 = new(e0), curve = new(e);
                    curve0.D0(curve0.FirstParameter() + global::Kerf.Engine.Foundation.Precision.Confusion(), out p2o);
                    curve.D0(curve.FirstParameter() + global::Kerf.Engine.Foundation.Precision.Confusion(), out p2);
                }
                Vector3D vDebFin0 = new(p1o, p2o), vDebFin = new(p1, p2);
                double aStraight = vDebFin0.AngleTo(vDebFin);
                parcours = (aStraight < M_PI / 2.0);
            }

            // reconstruction of the wire
            int rang;
            if (parcours)
            {
                for (rang = 1; rang <= nbEdges; rang++)
                {
                    Shape alocalshape = seqEdges.Value(rang);
                    bw.Add(newwire, ShapeCasts.Edge(alocalshape));
                    //	BW.Add(newwire,TopoDS::Edge(SeqEdges.Value(rang)));
                }
            }
            else
            {
                for (rang = nbEdges; rang >= 1; rang--)
                {
                    Shape alocalshape = seqEdges.Value(rang).Reversed();
                    bw.Add(newwire, ShapeCasts.Edge(alocalshape));
                    //	BW.Add(newwire,TopoDS::Edge(SeqEdges.Value(rang).Reversed()));
                }
            }

            // orientation of the wire
            // OCCT: newwire.Oriented(TopAbs_FORWARD) discards its result (a no-op; the new wire is FORWARD anyway)
            _work[i] = newwire;

            // passe to the next wire
            if (parcours)
            {
                pdeb = Tool.Pnt(vdeb);
                pfin = Tool.Pnt(vfin);
            }
            else
            {
                pfin = Tool.Pnt(vdeb);
                pdeb = Tool.Pnt(vfin);
            }
            p0 = p;
            isline0 = isline;
            // OCC86
            e0 = e;
        }

        // blocking sections ?
        if (vClosed)
        {
            _work[_work.Length()] = _work[1];
        }
    }
}
