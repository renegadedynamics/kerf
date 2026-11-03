// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Tools;

// The tolerance corrections of BOPTools_AlgoTools (BOPTools_AlgoTools_1.cxx).
//
// They are in-place utilities (design family 1): they grow the tolerances of the edges and vertices of the shape
// they are given (BRep_Builder.UpdateEdge/UpdateVertex in UpdateShape, the only write site), except those in the map
// to avoid. A frozen sub-shape throws TopoDS_LockedShape. Ported callers (BOPAlgo's post-passes) give them results
// they are still building (plan 2.3).
//
// Decision 6430: OCCT runs the five loops in parallel when asked. Their bodies write the tolerances of sub-shapes
// shared between the loop's items (a vertex of several edges or faces, an edge of two faces) with an unsynchronised
// read-compare-write, and the new values depend on the tolerances read first (CheckEdge's margin 0.1*Tol and its
// threshold, CorrectWires' and CorrectEdgeTolerance's comparisons). So native's parallel result depends on the
// threads' timing and can differ from its serial one (occt-issues.md). The port runs them serially whatever the flag,
// which is native's serial result (decision 580).
public static partial class AlgoTools
{
    // The loops write shared sub-shapes: always serial (decision 6430).
    private const bool ToRunCorrectionsInParallel = false;

    /// <summary>
    /// Provides valid tolerances for the shape: <see cref="CorrectPointOnCurve"/>, then
    /// <see cref="CorrectCurveOnSurface"/>. A tolerance that would reach <paramref name="aMaxTol"/> is not set; the
    /// sub-shapes in <paramref name="aMapToAvoid"/> are not changed. Writes into the shape's edges and vertices.
    /// </summary>
    /// <remarks><paramref name="bRunParallel"/> is ignored: the corrections run serially (decision 6430).</remarks>
    public static void CorrectTolerances(in Shape aShape, IndexedSet<Shape> aMapToAvoid, double aMaxTol = 0.0001,
                                         bool bRunParallel = false)
    {
        CorrectPointOnCurve(aShape, aMapToAvoid, aMaxTol, bRunParallel);
        CorrectCurveOnSurface(aShape, aMapToAvoid, aMaxTol, bRunParallel);
    }

    /// <summary>
    /// Provides valid tolerances for the shape in terms of BRepCheck_InvalidPointOnCurve: each vertex's tolerance
    /// grows to cover its distance to the points of its edges' 3D curves. Writes into the shape's vertices.
    /// </summary>
    /// <remarks><paramref name="bRunParallel"/> is ignored: the corrections run serially (decision 6430).</remarks>
    public static void CorrectPointOnCurve(in Shape aS, IndexedSet<Shape> aMapToAvoid, double aMaxTol = 0.0001,
                                           bool bRunParallel = false)
    {
        Explorer exp = new();
        List<BOPTools_CPC> vcpc = [];
        //
        exp.Init(aS, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            BOPTools_CPC cpc = new();
            vcpc.Add(cpc);
            cpc.SetEdge(e);
            cpc.SetMaxTol(aMaxTol);
            cpc.SetMapToAvoid(aMapToAvoid);
        }
        //
        //======================================================
        _ = bRunParallel;
        BooleanParallel.Perform(ToRunCorrectionsInParallel, vcpc);
        //======================================================
    }

    /// <summary>
    /// Provides valid tolerances for the shape in terms of BRepCheck_InvalidCurveOnSurface: vertices grow to cover
    /// their faces' wires (pcurve ends and 2D self-intersections near them), then edges grow to cover the deviation
    /// of their pcurves from their 3D curves. Writes into the shape's edges and vertices.
    /// </summary>
    /// <remarks><paramref name="bRunParallel"/> is ignored: the corrections run serially (decision 6430).</remarks>
    public static void CorrectCurveOnSurface(in Shape aS, IndexedSet<Shape> aMapToAvoid, double aMaxTol = 0.0001,
                                             bool bRunParallel = false)
    {
        Explorer expF = new(), expE = new();
        List<BOPTools_CWT> vcwt = [];
        List<BOPTools_CDT> vcdt = [];
        //
        expF.Init(aS, ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Face f = ShapeCasts.Face(expF.Current());
            //
            BOPTools_CWT cwt = new();
            vcwt.Add(cwt);
            cwt.SetFace(f);
            cwt.SetMapToAvoid(aMapToAvoid);
            //
            expE.Init(f, ShapeType.Edge);
            for (; expE.More(); expE.Next())
            {
                Edge e = ShapeCasts.Edge(expE.Current());
                //
                BOPTools_CDT cdt = new();
                vcdt.Add(cdt);
                cdt.SetEdge(e);
                cdt.SetFace(f);
                cdt.SetMaxTol(aMaxTol);
                cdt.SetMapToAvoid(aMapToAvoid);
            }
        }
        //
        //======================================================
        _ = bRunParallel;
        BooleanParallel.Perform(ToRunCorrectionsInParallel, vcwt);
        //======================================================
        BooleanParallel.Perform(ToRunCorrectionsInParallel, vcdt);
        //======================================================
    }

    /// <summary>
    /// Corrects the tolerances of the shape's sub-shapes where needed: each vertex at least its edges' tolerances,
    /// each edge and vertex of a face at least the face's. Writes into the shape's edges and vertices.
    /// </summary>
    /// <remarks><paramref name="bRunParallel"/> is ignored: the corrections run serially (decision 6430).</remarks>
    public static void CorrectShapeTolerances(in Shape aShape, IndexedSet<Shape> aMapToAvoid,
                                              bool bRunParallel = false)
    {
        Explorer exp = new();
        List<BOPTools_CVT> vcvt = [];
        List<BOPTools_CET> vcet = [];
        //
        exp.Init(aShape, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            BOPTools_CVT cvt = new();
            vcvt.Add(cvt);
            cvt.SetEdge(e);
            cvt.SetMapToAvoid(aMapToAvoid);
        }
        //
        //======================================================
        _ = bRunParallel;
        BooleanParallel.Perform(ToRunCorrectionsInParallel, vcvt);
        //======================================================
        //
        exp.Init(aShape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Face f = ShapeCasts.Face(exp.Current());
            BOPTools_CET cet = new();
            vcet.Add(cet);
            cet.SetFace(f);
            cet.SetMapToAvoid(aMapToAvoid);
        }
        //
        //======================================================
        BooleanParallel.Perform(ToRunCorrectionsInParallel, vcet);
        //======================================================
    }

    /// <summary>
    /// Computes the tolerance the edge needs on the face (<see cref="LibraryCheckCurveOnSurface"/>): the maximal
    /// distance between its 3D curve and its pcurve, and the parameter where it is reached. False if the check
    /// fails (the outputs are then 0, where OCCT leaves them unchanged).
    /// </summary>
    public static bool ComputeTolerance(in Face face, in Edge edge, out double maxDist, out double maxPar)
    {
        LibraryCheckCurveOnSurface cs = new();
        //
        cs.Init(edge, face);
        cs.Perform();
        if (!cs.IsDone)
        {
            maxDist = 0.0;
            maxPar = 0.0;
            return false;
        }
        //
        maxDist = cs.MaxDistance();
        maxPar = cs.MaxParameter();
        //
        return true;
    }

    // The loop bodies (OCCT's file classes, with the map to avoid held by reference).

    private sealed class BOPTools_CPC : BooleanParallel.ISolver
    {
        private double _maxTol = 1.0e-7;
        private Edge _edge;
        private IndexedSet<Shape>? _mypMapToAvoid;

        internal void SetEdge(in Edge aE) => _edge = aE;

        internal Edge Edge() => _edge;

        internal void SetMaxTol(double aMaxTol) => _maxTol = aMaxTol;

        internal double MaxTol() => _maxTol;

        internal void SetMapToAvoid(IndexedSet<Shape> aMapToAvoid) => _mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(_mypMapToAvoid is null, "mypMapToAvoid is null");
            CheckEdge(_edge, _maxTol, _mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CWT : BooleanParallel.ISolver
    {
        private Face _face;
        private IndexedSet<Shape>? _mypMapToAvoid;

        internal void SetFace(in Face aF) => _face = aF;

        internal void SetMapToAvoid(IndexedSet<Shape> aMapToAvoid) => _mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(_mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectWires(_face, _mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CDT : BooleanParallel.ISolver
    {
        private double _maxTol = 1.0e-7;
        private Edge _edge;
        private Face _face;
        private IndexedSet<Shape>? _mypMapToAvoid;

        internal void SetEdge(in Edge aE) => _edge = aE;

        internal void SetFace(in Face aF) => _face = aF;

        internal void SetMaxTol(double aMaxTol) => _maxTol = aMaxTol;

        internal void SetMapToAvoid(IndexedSet<Shape> aMapToAvoid) => _mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(_mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectEdgeTolerance(_edge, _face, _maxTol, _mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CVT : BooleanParallel.ISolver
    {
        private Edge _edge;
        private IndexedSet<Shape>? _mypMapToAvoid;

        internal void SetEdge(in Edge aE) => _edge = aE;

        internal void SetMapToAvoid(IndexedSet<Shape> aMapToAvoid) => _mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(_mypMapToAvoid is null, "mypMapToAvoid is null");
            CorrectVertexTolerance(_edge, _mypMapToAvoid!);
        }
    }

    private sealed class BOPTools_CET : BooleanParallel.ISolver
    {
        private Face _face;
        private IndexedSet<Shape>? _mypMapToAvoid;

        internal void SetFace(in Face aF) => _face = aF;

        internal void SetMapToAvoid(IndexedSet<Shape> aMapToAvoid) => _mypMapToAvoid = aMapToAvoid;

        public void Perform()
        {
            Standard_ProgramError_Raise_if(_mypMapToAvoid is null, "mypMapToAvoid is null");
            UpdateEdges(_face, _mypMapToAvoid!);
        }
    }

    // Correct tolerances for Vertices on Edge
    private static void CheckEdge(in Edge ed, double aMaxTol, IndexedSet<Shape> aMapToAvoid)
    {
        Edge e = ed;
        e.Orientation(Forward);
        double tolE = Tool.Tolerance(e);
        //
        BrepTEdge te = (BrepTEdge)e.TShape()!;
        //
        Placement eloc = e.Location();

        Iterator itS = new(e);
        for (; itS.More(); itS.Next())
        {
            Vertex v = ShapeCasts.Vertex(itS.Value());
            //
            BrepTVertex tv = (BrepTVertex)v.TShape()!;
            Point3D pv = tv.Pnt();
            //
            double tol = Tool.Tolerance(v);
            tol = Max(tol, tolE);
            double dd = 0.1 * tol;
            tol *= tol;
            //
            NCollection_List<CurveRepresentation>.Iterator itCR = new(te.Curves());
            while (itCR.More())
            {
                CurveRepresentation cr = itCR.Value();
                //
                if (cr.IsCurve3D)
                {
                    Curve3D? c = cr.Curve3D();
                    if (c is not null)
                    {
                        Placement l = (eloc * cr.Location()).Predivided(v.Location());
                        NCollection_List<PointRepresentation>.Iterator itPR = new(tv.Points());
                        while (itPR.More())
                        {
                            PointRepresentation pr = itPR.Value();
                            if (pr.IsPointOnCurve(c, l))
                            {
                                Point3D pc = c.Value(pr.Parameter());
                                pc = pc.Transformed(l.Transformation());
                                double d2 = pv.SquareDistance(pc);
                                if (d2 > tol)
                                {
                                    double newTolerance = Math.Sqrt(d2) + dd;
                                    if (newTolerance < aMaxTol)
                                    {
                                        UpdateShape(v, newTolerance, aMapToAvoid);
                                    }
                                }
                            }
                            itPR.Next();
                        }
                        //
                        Orientation orV = v.Orientation();
                        if (orV == Forward || orV == Reversed)
                        {
                            GCurve gc = (GCurve)cr;
                            Point3D pc;
                            if (orV == Forward)
                            {
                                pc = c.Value(gc.First());
                            }
                            else
                            {
                                pc = c.Value(gc.Last());
                            }
                            pc = pc.Transformed(l.Transformation());
                            //
                            double d2 = pv.SquareDistance(pc);
                            if (d2 > tol)
                            {
                                double newTolerance = Math.Sqrt(d2) + dd;
                                if (newTolerance < aMaxTol)
                                {
                                    UpdateShape(v, newTolerance, aMapToAvoid);
                                }
                            }
                        }
                    }
                }
                itCR.Next();
            } //  while (itcr.More()) {
        } // for (; aVExp.More(); aVExp.Next()) {
    }

    // Compute edge length and cache it in the map
    private static double MapEdgeLength(in Edge edge, Dictionary<Shape, double> mapEdgeLen)
    {
        if (!mapEdgeLen.TryGetValue(edge, out double len))
        {
            len = 0.0;
            if (!Tool.Degenerated(edge))
            {
                EdgeCurveAdaptor curve = new(edge);
                len = AbscissaPoint.Length(curve);
            }
            mapEdgeLen[edge] = len;
        }
        return len;
    }

    // OCCT's anonymous EdgeData: the data of an edge around a vertex, kept to avoid recomputing it.
    private readonly struct EdgeData(in Edge theEdge, double theVParameter, bool theIsClosed, CurveAdaptor2D theGAdaptor,
                                     double theFirst, double theLast)
    {
        public readonly Edge Edge = theEdge;                   // Edge
        public readonly double VParameter = theVParameter;            // Parameter of the vertex on the edge
        public readonly bool IsClosed = theIsClosed;                  // Closed flag of the edge
        public readonly CurveAdaptor2D GAdaptor = theGAdaptor;   // 2D adaptor for PCurve of the edge on the face
        public readonly double First = theFirst;                      // First parameter in the range
        public readonly double Last = theLast;                        // Last parameter in the rage
    }

    private static double IntersectCurves2d(in Vertex v, Surface s, in EdgeData eData1, in EdgeData eData2,
                                            Dictionary<Shape, double> mapEdgeLen)
    {
        GInter inter = new();
        // Range of the first edge
        double t11 = eData1.First;
        double t12 = eData1.Last;
        // Range of the second edge
        double t21 = eData2.First;
        double t22 = eData2.Last;

        double maxDist = 0.0;
        double tol2d = 1.0e-10;
        //
        TwoDDomain dom1 = new(eData1.GAdaptor.Value(t11),
                                    t11,
                                    tol2d,
                                    eData1.GAdaptor.Value(t12),
                                    t12,
                                    tol2d);
        TwoDDomain dom2 = new(eData2.GAdaptor.Value(t21),
                                    t21,
                                    tol2d,
                                    eData2.GAdaptor.Value(t22),
                                    t22,
                                    tol2d);
        //
        inter.Perform(eData1.GAdaptor, dom1, eData2.GAdaptor, dom2, tol2d, tol2d);
        if (!inter.IsDone || (inter.SegmentCount == 0 && inter.PointCount == 0))
        {
            return maxDist;
        }
        //
        double t1, t2, tint1, tint2, halfR1, halfR2, dist;
        int i, nb;
        Point3D p, pv;
        Point2D p2d;
        NCollection_List<IntersectionPoint2D> lp = new();
        NCollection_List<IntersectionPoint2D>.Iterator itLP;
        //
        pv = Tool.Pnt(v);
        t1 = eData1.VParameter;
        t2 = eData2.VParameter;
        //
        halfR1 = (t12 - t11) / 2.0;
        halfR2 = (t22 - t21) / 2.0;
        //
        nb = inter.SegmentCount;
        for (i = 1; i <= nb; ++i)
        {
            IntersectionSegment2D seg = inter.Segment(i);
            lp.Append(seg.FirstPoint());
            lp.Append(seg.LastPoint());
        }
        //
        nb = inter.PointCount;
        for (i = 1; i <= nb; ++i)
        {
            IntersectionPoint2D pnt = inter.Point(i);
            lp.Append(pnt);
        }
        //
        // evaluate the length of the smallest edge, so that not to return too large distance
        double len1 = MapEdgeLength(eData1.Edge, mapEdgeLen);
        double len2 = MapEdgeLength(eData1.Edge, mapEdgeLen); // OCCT: the first edge's length again
        const double maxEdgePartCoveredByVertex = 0.3;
        double maxThresDist = Min(len1, len2) * maxEdgePartCoveredByVertex;
        maxThresDist *= maxThresDist;
        itLP = new(lp);
        for (; itLP.More(); itLP.Next())
        {
            IntersectionPoint2D pnt = itLP.Value();
            //
            tint1 = pnt.ParamOnFirst();
            tint2 = pnt.ParamOnSecond();
            //
            if ((tint1 < t11 || tint1 > t12) || (tint2 < t21 || tint2 > t22))
            {
                // out of range;
                continue;
            }
            //
            if ((!eData1.IsClosed && Math.Abs(tint1 - t1) > halfR1)
                || (!eData2.IsClosed && Math.Abs(tint2 - t2) > halfR2))
            {
                // intersection is on the other end of the edge
                continue;
            }
            //
            p2d = pnt.Value();
            s.D0(p2d.X(), p2d.Y(), out p);
            dist = pv.SquareDistance(p);
            if (dist > maxDist && dist < maxThresDist)
            {
                maxDist = dist;
            }
        }
        //
        return maxDist;
    }

    private static void CorrectWires(in Face aFx, IndexedSet<Shape> aMapToAvoid)
    {
        int i, nbV;
        double tol, tol2, d2, d2max;
        Point3D p, pv;
        Point2D p2D;
        Face f;
        IndexedMap<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new();
        //
        f = aFx;
        f.Orientation(Forward);
        Surface s = Tool.Surface(aFx)!;

        Exploration.MapShapesAndUniqueAncestors(f, ShapeType.Vertex, ShapeType.Edge, mve, true);

        // OCCT's NCollection_DataMap<TopoDS_Shape, double> hashes with std::hash and compares with operator== (orientation included)
        Dictionary<Shape, double> mapEdgeLen = new(EqualityComparer<Shape>.Default);
        nbV = mve.Count;
        for (i = 1; i <= nbV; ++i)
        {
            Vertex v = ShapeCasts.Vertex(mve.KeyAt(i - 1));
            pv = Tool.Pnt(v);
            tol = Tool.Tolerance(v);
            tol2 = tol * tol;
            //
            d2max = -1.0;
            // Save edge's data to avoid its recalculation during intersection of 2d curves
            List<EdgeData> lePars = [];
            NCollection_List<Shape> le = mve.ValueAt(i - 1);
            it.Initialize(le);
            for (; it.More(); it.Next())
            {
                Edge e = ShapeCasts.Edge(it.Value());
                Curve2D? c2D = Tool.CurveOnSurface(e, f, out double t1, out double t2);
                if (c2D is null)
                {
                    continue;
                }

                double t = Tool.Parameter(v, e);
                bool isClosed = false;
                {
                    Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                    isClosed = v1.IsSame(v2);
                }
                //
                c2D.D0(t, out p2D);
                s.D0(p2D.X(), p2D.Y(), out p);
                d2 = pv.SquareDistance(p);
                if (d2 > d2max)
                {
                    d2max = d2;
                }
                EdgeData eData = new(e, t, isClosed, new CurveAdaptor2D(c2D), t1, t2);
                lePars.Add(eData);
            }
            //
            // check wires on self interference by intersecting 2d curves of the edges
            for (int i1 = 0; i1 < lePars.Count; ++i1)
            {
                EdgeData eData1 = lePars[i1];
                Shape e1 = eData1.Edge;

                for (int i2 = i1 + 1; i2 < lePars.Count; ++i2)
                {
                    EdgeData eData2 = lePars[i2];
                    Shape e2 = eData2.Edge;

                    if (e1.IsSame(e2))
                    {
                        continue;
                    }

                    d2 = IntersectCurves2d(v, s, eData1, eData2, mapEdgeLen);
                    if (d2 > d2max)
                    {
                        d2max = d2;
                    }
                }
            }
            //
            if (d2max > tol2)
            {
                tol = 1.01 * Math.Sqrt(d2max);
                UpdateShape(v, tol, aMapToAvoid);
            }
        } // for (i=1; i<=aNbV; ++i) {
    }

    private static void CorrectEdgeTolerance(in Edge myShape, in Face s, double aMaxTol,
                                             IndexedSet<Shape> aMapToAvoid)
    {
        //
        // 1. Minimum of conditions to Perform
        CurveRepresentation? myCref = null;
        CurveAdaptorBase? myHCurve = null;

        BrepTEdge tEx = (BrepTEdge)myShape.TShape()!;
        NCollection_List<CurveRepresentation>.Iterator itcrx = new(tEx.Curves());
        bool degenerated, sameParameterx, sameRangex;

        int unique = 0;

        degenerated = tEx.Degenerated();
        sameParameterx = tEx.SameParameter();
        sameRangex = tEx.SameRange();

        if (!sameRangex && sameParameterx)
        {
            return;
        }

        while (itcrx.More())
        {
            CurveRepresentation cr = itcrx.Value();
            if (cr.IsCurve3D)
            {
                unique++;
                if (myCref is null && cr.Curve3D() is not null)
                {
                    myCref = cr;
                }
            }
            itcrx.Next();
        }

        if (unique == 0)
        {
            return; //...No3DCurve
        }
        if (unique > 1)
        {
            return; //...Multiple3DCurve;
        }

        if (myCref is null && !degenerated)
        {
            itcrx.Initialize(tEx.Curves());
            while (itcrx.More())
            {
                CurveRepresentation cr = itcrx.Value();
                if (cr.IsCurveOnSurface())
                {
                    myCref = cr;
                    break;
                }
                itcrx.Next();
            }
        }

        else if (myCref is not null && degenerated)
        {
            return; //...InvalidDegeneratedFlag;
        }

        if (myCref is not null)
        {
            GCurve gCref = (GCurve)myCref;
            gCref.Range(out double first, out double last);
            if (last <= first)
            {
                return; // InvalidRange;
            }

            else
            {
                if (myCref.IsCurve3D)
                {
                    Curve3D c3dx = (Curve3D)myCref.Curve3D()!.Transformed(myCref.Location().Transformation());
                    CurveAdaptor gac3d = new(c3dx, first, last);
                    myHCurve = gac3d;
                }
                else
                { // curve on surface
                    Surface sref = myCref.Surface()!;
                    sref = (Surface)sref.Transformed(myCref.Location().Transformation());
                    Curve2D pCref = myCref.PCurve()!;
                    SurfaceAdaptor gahSref = new(sref);
                    CurveAdaptor2D ghpCref = new(pCref, first, last);
                    CurveOnSurface acSref = new(ghpCref, gahSref);
                    myHCurve = acSref;
                }
            }
        }

        //===============================================
        // 2. Tolerances in InContext
        {
            if (myCref is null)
            {
                return;
            }

            BrepTEdge te = (BrepTEdge)myShape.TShape()!;
            double tol = Tool.Tolerance(ShapeCasts.Edge(myShape));
            double newTol = tol;

            bool sameParameter = te.SameParameter();
            bool sameRange = te.SameRange();
            double first = myHCurve!.FirstParameter();
            double last = myHCurve.LastParameter();

            BrepTFace tf = (BrepTFace)s.TShape()!;
            Placement floc = s.Location();
            Placement tFloc = tf.Location();
            Surface? su = tf.Surface();
            Placement L = (floc * tFloc).Predivided(myShape.Location());
            bool pcurvefound = false;

            NCollection_List<CurveRepresentation>.Iterator itcr = new(te.Curves());
            while (itcr.More())
            {
                CurveRepresentation cr = itcr.Value();
                if (!ReferenceEquals(cr, myCref) && cr.IsCurveOnSurface(su, L))
                {
                    pcurvefound = true;
                    GCurve gc = (GCurve)cr;
                    gc.Range(out double f, out double l);
                    if (sameRange && (f != first || l != last))
                    {
                        return; // BRepCheck_InvalidSameRangeFlag;
                    }

                    Surface sb;
                    _ = cr.Surface();
                    sb = (Surface)su!.Transformed(L.Transformation());
                    Curve2D pc = cr.PCurve()!;
                    SurfaceAdaptor gahs = new(sb);
                    CurveAdaptor2D ghpc = new(pc, f, l);
                    CurveOnSurface acs = new(ghpc, gahs);

                    ValidateEdge validateEdge = new(myHCurve, acs, sameParameter);
                    validateEdge.Process();
                    validateEdge.UpdateTolerance(ref newTol);
                    if (validateEdge.IsDone && !validateEdge.CheckTolerance(tol))
                    {
                        if (newTol < aMaxTol)
                        {
                            UpdateShape(myShape, newTol, aMapToAvoid);
                            CorrectVertexTolerance(myShape, aMapToAvoid);
                        }
                    }

                    if (cr.IsCurveOnClosedSurface)
                    {
                        ghpc.Load(cr.PCurve2()!, f, l); // same bounds
                        acs.Load(ghpc, gahs);           // sans doute inutile

                        ValidateEdge validateEdgeOnClosedSurf = new(myHCurve, acs, sameParameter);
                        validateEdgeOnClosedSurf.Process();
                        validateEdgeOnClosedSurf.UpdateTolerance(ref newTol);
                        if (validateEdgeOnClosedSurf.IsDone && !validateEdgeOnClosedSurf.CheckTolerance(tol))
                        {
                            if (newTol < aMaxTol)
                            {
                                UpdateShape(myShape, newTol, aMapToAvoid);
                                CorrectVertexTolerance(myShape, aMapToAvoid);
                            }
                        }
                    }
                }
                itcr.Next();
            }

            if (!pcurvefound)
            {
                PlaneSurface? p;
                if (su is TrimmedSurface rts)
                {
                    p = rts.BasisSurface() as PlaneSurface;
                }
                else
                {
                    p = su as PlaneSurface;
                }
                if (p is null)
                { // not a plane
                    return;
                }

                else
                { // on fait la projection a la volee, comme BRep_Tool
                    p = (PlaneSurface)p.Transformed(L.Transformation());
                    // on projette Cref sur ce plan
                    SurfaceAdaptor gahs = new(p);

                    // Dub - Normalement myHCurve est une GeomAdaptor_Curve
                    // OCCT down-casts and dereferences: a reference curve on a surface gives a null handle here
                    CurveAdaptor gac = myHCurve as CurveAdaptor
                                            ?? throw new NullObjectException("BOPTools_AlgoTools::CorrectEdgeTolerance - no 3D curve");
                    Curve3D c3dx = gac.Curve()!;
                    Curve3D projOnPlane =
                        global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(new TrimmedCurve3D(c3dx, first, last),
                                                   p,
                                                   p.Position().Direction(),
                                                   true)!;

                    CurveAdaptor hCurve = new(projOnPlane);

                    ProjectedCurve proj = new(gahs, hCurve);
                    Curve2D pc = global::Kerf.Engine.Geometry.Adaptors.Geom2dAdaptor.MakeCurve(proj);
                    CurveAdaptor2D ghpc = new(pc, myHCurve.FirstParameter(), myHCurve.LastParameter());

                    CurveOnSurface acs = new(ghpc, gahs);

                    ValidateEdge validateProjEdge = new(myHCurve, acs, true);
                    validateProjEdge.Process();
                    validateProjEdge.UpdateTolerance(ref newTol);
                    if (validateProjEdge.IsDone && !validateProjEdge.CheckTolerance(tol))
                    {
                        if (newTol < aMaxTol)
                        {
                            UpdateShape(myShape, newTol, aMapToAvoid);
                            CorrectVertexTolerance(myShape, aMapToAvoid);
                        }
                    }
                }
            } // end of if (!pcurvefound) {
        } // end of  2. Tolerances in InContext
    }

    private static void CorrectVertexTolerance(in Edge aE, IndexedSet<Shape> aMapToAvoid)
    {
        double tolE, tolV;
        Iterator it = new();
        //
        tolE = Tool.Tolerance(aE);
        it.Initialize(aE);
        for (; it.More(); it.Next())
        {
            Vertex v = ShapeCasts.Vertex(it.Value());
            tolV = Tool.Tolerance(v);
            if (tolV < tolE)
            {
                UpdateShape(v, tolE, aMapToAvoid);
            }
        }
    }

    private static void UpdateEdges(in Face aF, IndexedSet<Shape> aMapToAvoid)
    {
        double tolF, tolE, tolV;
        Iterator itF = new(), itW = new();
        //
        tolE = tolF = Tool.Tolerance(aF);
        itF.Initialize(aF);
        for (; itF.More(); itF.Next())
        {
            Shape s = itF.Value();
            if (s.ShapeType() == ShapeType.Wire)
            {
                itW.Initialize(s);
                for (; itW.More(); itW.Next())
                {
                    Edge e = ShapeCasts.Edge(itW.Value());
                    tolE = Tool.Tolerance(e);
                    if (tolE < tolF)
                    {
                        UpdateShape(e, tolF, aMapToAvoid);
                        tolE = tolF;
                    }
                }
            }
            else
            {
                Vertex v = ShapeCasts.Vertex(itF.Value());
                tolV = Tool.Tolerance(v);
                if (tolV < tolE)
                {
                    UpdateShape(v, tolF, aMapToAvoid);
                }
            }
        }
    }

    // The only write site of the corrections: grows the edge's or vertex's tolerance (never shrinks it).
    private static void UpdateShape(in Shape aS, double aTol, IndexedSet<Shape> aMapToAvoid)
    {
        if (aMapToAvoid.Contains(aS))
        {
            return;
        }
        //
        ShapeType type;
        BrepBuilder bb = new();
        //
        type = aS.ShapeType();
        if (type == ShapeType.Edge)
        {
            Edge e = ShapeCasts.Edge(aS);
            bb.UpdateEdge(e, aTol);
        }
        else if (type == ShapeType.Vertex)
        {
            Vertex v = ShapeCasts.Vertex(aS);
            bb.UpdateVertex(v, aTol);
        }
    }
}
