// The check as it was before the shortcuts of "The check on small shapes" (docs/performance.md), kept for the comparison only:
// generated from commit 2fc84b5b4, and only its namespace differs. It is never in a package.
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_WireExplorer.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_WireExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using static Kerf.Engine.Foundation.Standard;
namespace Kerf.Engine.Healing.Checks.Full;

/// <summary>
/// Explores the edges of a wire in connection order: each edge is connected to the previous one by its origin.
/// </summary>
/// <remarks>
/// <para>
/// If the wire is not closed, only a segment of edges is returned, whose length depends on the edge the exploration
/// starts with. The wire is assumed valid: loops, wrongly oriented edges (two edges going into or out of a shared
/// vertex), branches and INTERNAL or EXTERNAL edges can stop the exploration early.
/// </para>
/// <para>
/// A working object, as OCCT's (which is used by value). An infinite wire without free vertices starts at an
/// infinite edge found in a hash set, whose iteration order isn't OCCT's (decision 18): with several such edges, the
/// first one can differ.
/// </para>
/// </remarks>
internal sealed class WireExplorer
{
    private readonly Dictionary<Shape, NCollection_List<Shape>> _map =
        new(ShapeHasher.Instance);

    private Edge _edge = new();
    private Vertex _vertex = new();
    private Face _face = new();
    private readonly HashSet<Shape> _doubles = new(ShapeHasher.Instance);
    private double _tolU;
    private double _tolV;

    /// <summary>An empty explorer (initialise it with <see cref="Init(in Wire)"/>).</summary>
    public WireExplorer()
    {
        _tolU = 0.0;
        _tolV = 0.0;
    }

    /// <summary>Starts an exploration of the wire.</summary>
    public WireExplorer(in Wire w)
    {
        Face f = new Face();
        Init(w, f);
    }

    /// <summary>
    /// Starts an exploration of the wire; the face is used to select the edge connected to the previous one in its
    /// parametric space.
    /// </summary>
    public WireExplorer(in Wire w, in Face f)
    {
        Init(w, f);
    }

    /// <summary>Starts an exploration of the wire.</summary>
    public void Init(in Wire w)
    {
        Face f = new Face();
        Init(w, f);
    }

    /// <summary>
    /// Starts an exploration of the wire; the face is used to select the edge connected to the previous one in its
    /// parametric space.
    /// </summary>
    public void Init(in Wire w, in Face f)
    {
        _edge = new Edge();
        _vertex = new Vertex();
        _map.Clear();
        _doubles.Clear();

        if (w.IsNull)
        {
            return;
        }

        double uMin = 0.0, uMax = 0.0, vMin = 0.0, vMax = 0.0;
        if (!f.IsNull)
        {
            // For the faces based on Cone, BSpline and Bezier compute the
            // UV bounds to precise the UV tolerance values
            SurfaceKind surfType = new FaceSurfaceAdaptor(f, false).GetType();
            if (surfType == SurfaceKind.Cone || surfType == SurfaceKind.BSplineSurface
                || surfType == SurfaceKind.BezierSurface)
            {
                BrepTools.UVBounds(f, out uMin, out uMax, out vMin, out vMax);
            }
        }

        Init(w, f, uMin, uMax, vMin, vMax);
    }

    /// <summary>
    /// Starts an exploration of the wire; the face is used to select the edge connected to the previous one in its
    /// parametric space, and UMin to VMax are its UV bounds.
    /// </summary>
    public void Init(in Wire w, in Face f, double uMin, double uMax, double vMin, double vMax)
    {
        _edge = new Edge();
        _vertex = new Vertex();
        _map.Clear();
        _doubles.Clear();

        if (w.IsNull)
        {
            return;
        }

        _face = f;
        double dfVertToler = 0.0;

        if (!_face.IsNull)
        {
            Surface? surf = Tool.Surface(_face, out Placement aL);
            SurfaceAdaptor gas = new SurfaceAdaptor(surf!);
            var exp = new Explorer(w, ShapeType.Vertex);
            for (; exp.More(); exp.Next())
            {
                Vertex v = ShapeCasts.Vertex(exp.Current());
                dfVertToler = Max(Tool.Tolerance(v), dfVertToler);
            }
            if (dfVertToler < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                // Use tolerance of edges
                for (var it0 = new Iterator(w); it0.More(); it0.Next())
                {
                    dfVertToler = Max(Tool.Tolerance(ShapeCasts.Edge(it0.Value())), dfVertToler);
                }

                if (dfVertToler < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    // empty wire
                    return;
                }
            }
            _tolU = 2.0 * gas.UResolution(dfVertToler);
            _tolV = 2.0 * gas.VResolution(dfVertToler);

            // uresolution for cone with infinite vmin vmax is too small.
            if (gas.GetType() == SurfaceKind.Cone)
            {
                gas.D1(uMin, vMin, out Point3D p, out Vector3D d1U, out Vector3D d1V);
                double tol1, tol2, maxtol = .0005 * (uMax - uMin);
                double a = d1U.Magnitude;

                if (a <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    tol1 = maxtol;
                }
                else
                {
                    tol1 = Min(maxtol, dfVertToler / a);
                }

                gas.D1(uMin, vMax, out p, out d1U, out d1V);
                a = d1U.Magnitude;
                if (a <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    tol2 = maxtol;
                }
                else
                {
                    tol2 = Min(maxtol, dfVertToler / a);
                }

                _tolU = 2.0 * Max(tol1, tol2);
            }

            if (gas.GetType() == SurfaceKind.BSplineSurface || gas.GetType() == SurfaceKind.BezierSurface)
            {
                double maxTol = Max(_tolU, _tolV);
                gas.D1((uMax - uMin) / 2.0, (vMax - vMin) / 2.0, out Point3D p, out Vector3D du, out Vector3D dv);
                double mod = Math.Sqrt(du * du + dv * dv);
                if (mod > Constants.Resolution())
                {
                    if (mod * maxTol / dfVertToler < 1.5)
                    {
                        maxTol = 1.5 * dfVertToler / mod;
                    }
                    _tolU = maxTol;
                    _tolV = maxTol;
                }
            }

        }

        // map of vertices to know if the wire is open
        var vmap = new IndexedSet<Shape>(ShapeHasher.Instance);
        //  map of infinite edges
        var infEmap = new HashSet<Shape>(ShapeHasher.Instance);

        // list the vertices
        Vertex v1 = new Vertex(), v2 = new Vertex();

        var it = new Iterator(w);
        while (it.More())
        {
            Edge e = ShapeCasts.Edge(it.Value());
            Orientation eori = e.Orientation;
            if (eori == Internal || eori == External)
            {
                it.Next();
                continue;
            }
            Exploration.Vertices(e, out v1, out v2, true);

            if (!v1.IsNull)
            {
                if (!_map.ContainsKey(v1))
                {
                    // OCCT binds a copy of one empty list; the port's lists are classes, so each vertex gets its own
                    _map[v1] = new NCollection_List<Shape>();
                }
                _map.GetValue(v1).Append(e);

                // add or remove in the vertex map
                v1.Orient(Forward);
                int currsize = vmap.Count, ind = vmap.Add(v1) + 1;
                if (currsize >= ind)
                {
                    vmap.Remove(v1);
                }
            }

            if (!v2.IsNull)
            {
                v2.Orient(Reversed);
                int currsize = vmap.Count, ind = vmap.Add(v2) + 1;
                if (currsize >= ind)
                {
                    vmap.Remove(v2);
                }
            }

            if (v1.IsNull || v2.IsNull)
            {
                Tool.Range(e, out double aF, out double aL);

                if (eori == Forward)
                {
                    if (aF == -global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        infEmap.Add(e);
                    }
                }
                else
                { // Eori == TopAbs_REVERSED
                    if (aL == global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        infEmap.Add(e);
                    }
                }
            }
            it.Next();
        }

        // Construction of the set of double edges.
        var it2 = new Iterator(w);
        var emap = new HashSet<Shape>(ShapeHasher.Instance);
        while (it2.More())
        {
            if (!emap.Add(it2.Value()))
            {
                _doubles.Add(it2.Value());
            }
            it2.Next();
        }

        // if vmap is not empty the wire is open, let us find the first vertex
        if (vmap.Count != 0)
        {
            int ind = 0;
            for (ind = 1; ind <= vmap.Count; ++ind)
            {
                if (vmap[ind - 1].Orientation == Forward)
                {
                    v1 = ShapeCasts.Vertex(vmap[ind - 1]);
                    break;
                }
            }
        }
        else
        {
            //   The wire is infinite Try to find the first vertex. It may be NULL.
            if (infEmap.Count != 0)
            {
                foreach (Shape key in infEmap)
                {
                    Edge edge = ShapeCasts.Edge(key);
                    Orientation ori = edge.Orientation;

                    Tool.Range(edge, out double aF, out double aL);
                    if ((ori == Forward && aF == -global::Kerf.Engine.Foundation.Precision.Infinite())
                        || (ori == Reversed && aL == global::Kerf.Engine.Foundation.Precision.Infinite()))
                    {
                        _edge = edge;
                        _vertex = new Vertex();

                        return;
                    }
                }
            }

            // use the first vertex in iterator
            it.Initialize(w);
            while (it.More())
            {
                Edge e = ShapeCasts.Edge(it.Value());
                Orientation eori = e.Orientation;
                if (eori == Internal || eori == External)
                {
                    // JYL 10-03-97 : waiting for correct processing
                    // of INTERNAL/EXTERNAL edges
                    it.Next();
                    continue;
                }
                Exploration.Vertices(e, out v1, out v2, true);
                break;
            }
        }

        if (v1.IsNull)
        {
            return;
        }
        if (!_map.ContainsKey(v1))
        {
            return;
        }

        NCollection_List<Shape> l = _map.GetValue(v1);
        _edge = ShapeCasts.Edge(l.First());
        l.RemoveFirst();
        _vertex = Exploration.FirstVertex(_edge, true);
    }

    /// <summary>True if there is a current edge.</summary>
    public bool More() => !_edge.IsNull;

    /// <summary>Moves to the next edge.</summary>
    public void Next()
    {
        _vertex = Exploration.LastVertex(_edge, true);

        if (_vertex.IsNull)
        {
            _edge = new Edge();
            return;
        }
        if (!_map.ContainsKey(_vertex))
        {
            _edge = new Edge();
            return;
        }

        NCollection_List<Shape> l = _map.GetValue(_vertex);

        if (l.IsEmpty())
        {
            _edge = new Edge();
        }
        else if (l.Extent() == 1)
        {
            //  Modified by Sergey KHROMOV - Fri Jun 21 10:28:01 2002 OCC325 Begin
            Edge nextEdge = ShapeCasts.Edge(l.First());

            Exploration.Vertices(nextEdge, out Vertex v1, out Vertex v2, true);

            if (!v1.IsSame(_vertex))
            {
                _edge = new Edge();
                return;
            }
            if (!_face.IsNull && v1.IsSame(v2))
            {
                Curve2D? prevPC = Tool.CurveOnSurface(_edge, _face, out double par11, out double par12);
                Curve2D? nextPC = Tool.CurveOnSurface(nextEdge, _face, out double par21, out double par22);
                double prevPar;
                double nextFPar;
                double nextLPar;

                if (prevPC is null || nextPC is null)
                {
                    _edge = new Edge();
                    return;
                }

                if (_edge.Orientation == Forward)
                {
                    prevPar = par12;
                }
                else
                {
                    prevPar = par11;
                }

                if (nextEdge.Orientation == Forward)
                {
                    nextFPar = par21;
                    nextLPar = par22;
                }
                else
                {
                    nextFPar = par22;
                    nextLPar = par21;
                }

                Point2D pPrev = prevPC.Value(prevPar);
                Point2D pNextF = nextPC.Value(nextFPar);
                Point2D pNextL = nextPC.Value(nextLPar);

                if (pPrev.SquareDistanceTo(pNextF) > pPrev.SquareDistanceTo(pNextL))
                {
                    _edge = new Edge();
                    return;
                }
            }
            //  Modified by Sergey KHROMOV - Fri Jun 21 11:08:16 2002 End
            _edge = ShapeCasts.Edge(l.First());
            l.Clear();
        }
        else
        {
            if (_face.IsNull)
            {
                // Without Face - try to return edges
                // as logically as possible
                // At first degenerated edges.
                Edge e = _edge;
                if (SelectDegenerated(l, ref e))
                {
                    _edge = e;
                    return;
                }
                // At second double edges.
                e = _edge;
                if (SelectDouble(_doubles, l, ref e))
                {
                    _edge = e;
                    return;
                }

                var it = new NCollection_List<Shape>.Iterator(l);
                bool notfound = true;
                while (it.More())
                {
                    if (!it.Value().IsSame(_edge))
                    {
                        _edge = ShapeCasts.Edge(it.Value());
                        l.Remove(it);
                        notfound = false;
                        break;
                    }
                    it.Next();
                }

                if (notfound)
                {
                    _edge = new Edge();
                    return;
                }
            }
            else
            {
                // If we have more than one edge attached to the list
                // probably wire that we explore contains a loop or loops.
                Curve2D? pCurve = Tool.CurveOnSurface(_edge, _face, out double dfFPar, out double dfLPar);
                if (pCurve is null)
                {
                    _edge = new Edge();
                    return;
                }
                // Note: current < myVertex > which is last on < myEdge >
                //       equals in 2D to following 2D points:
                //       edge is FORWARD  - point with MAX parameter on PCurve;
                //       edge is REVERSED - point with MIN parameter on PCurve.

                // Get 2D point equals to < myVertex > in 2D for current edge.
                Point2D pRef;
                if (_edge.Orientation == Reversed)
                {
                    pCurve.D0(dfFPar, out pRef);
                }
                else
                {
                    pCurve.D0(dfLPar, out pRef);
                }

                // Get next 2D point from current edge's PCurve with parameter
                // F + dP (REV) or L - dP (FOR)
                bool isrevese = (_edge.Orientation == Reversed);
                double dfMPar = GetNextParamOnPC(pCurve, pRef, dfFPar, dfLPar, _tolU, _tolV, isrevese);

                pCurve.D0(dfMPar, out Point2D pRefm);
                // Get vector from PRef to PRefm
                Vector2D eRefDir = new Vector2D(pRef, pRefm);
                if (eRefDir.SquareMagnitude < Constants.Resolution())
                {
                    _edge = new Edge();
                    return;
                }

                // Search the list of edges looking for the edge having hearest
                // 2D point of connected vertex to current one and smallest angle.
                // First process all degenerated edges, then - all others.

                var it = new NCollection_List<Shape>.Iterator();
                int k = 1, kMin = 0, iDone = 0;
                bool isDegenerated = true;
                double dmin = RealLast();
                double dfMinAngle = 3.0 * Math.PI, dfCurAngle = 3.0 * Math.PI;

                for (iDone = 0; iDone < 2; iDone++)
                {
                    it.Initialize(l);
                    while (it.More())
                    {
                        Edge e = ShapeCasts.Edge(it.Value());
                        if (e.IsSame(_edge))
                        {
                            it.Next();
                            k++;
                            continue;
                        }

                        Exploration.Vertices(e, out Vertex vert1, out Vertex vert2, true);
                        if (vert1.IsNull || vert2.IsNull)
                        {
                            it.Next();
                            k++;
                            continue;
                        }

                        pCurve = Tool.CurveOnSurface(e, _face, out dfFPar, out dfLPar);
                        if (pCurve is null)
                        {
                            it.Next();
                            k++;
                            continue;
                        }

                        Point2D pEb, pEe;
                        if (vert1.IsSame(vert2) == isDegenerated)
                        {
                            if (e.Orientation == Reversed)
                            {
                                pCurve.D0(dfLPar, out pEb);
                            }
                            else
                            {
                                pCurve.D0(dfFPar, out pEb);
                            }

                            if (Math.Abs(dfLPar - dfFPar) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                isrevese = (e.Orientation == Reversed);
                                isrevese = !isrevese;
                                double ePm =
                                    GetNextParamOnPC(pCurve, pEb, dfFPar, dfLPar, _tolU, _tolV, isrevese);

                                pCurve.D0(ePm, out pEe);
                                if (pEb.SquareDistanceTo(pEe) <= Constants.Resolution())
                                {
                                    // seems to be very short curve
                                    pCurve.D1(ePm, out pEe, out Vector2D aD);
                                    if (e.Orientation == Reversed)
                                    {
                                        pEe = pEe.WithXY(pEb.XY() - aD.XY());
                                    }
                                    else
                                    {
                                        pEe = pEe.WithXY(pEb.XY() + aD.XY());
                                    }

                                    if (pEb.SquareDistanceTo(pEe) <= Constants.Resolution())
                                    {
                                        it.Next();
                                        k++;
                                        continue;
                                    }
                                }
                                Vector2D eDir = new Vector2D(pEb, pEe);
                                dfCurAngle = Math.Abs(eDir.AngleTo(eRefDir));
                            }

                            if (dfCurAngle <= dfMinAngle)
                            {
                                double d = pRef.SquareDistanceTo(pEb);
                                if (d <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                                {
                                    d = 0.0;
                                }
                                if (Math.Abs(pEb.X - pRef.X) < _tolU && Math.Abs(pEb.Y - pRef.Y) < _tolV)
                                {
                                    if (d <= dmin)
                                    {
                                        dfMinAngle = dfCurAngle;
                                        kMin = k;
                                        dmin = d;
                                    }
                                }
                            }
                        }
                        it.Next();
                        k++;
                    } // while it

                    if (kMin == 0)
                    {
                        isDegenerated = false;
                        k = 1;
                        dmin = RealLast();
                    }
                    else
                    {
                        break;
                    }
                } // for iDone

                if (kMin == 0)
                {
                    // probably unclosed in 2d space wire
                    _edge = new Edge();
                    return;
                }

                // Selection the edge.
                it.Initialize(l);
                k = 1;
                while (it.More())
                {
                    if (k == kMin)
                    {
                        _edge = ShapeCasts.Edge(it.Value());
                        l.Remove(it);
                        break;
                    }
                    it.Next();
                    k++;
                }
            } // else face != NULL && l > 1
        } // else l > 1
    }

    /// <summary>The current edge.</summary>
    public Edge Current() => _edge;

    /// <summary>The orientation of the vertex connecting the current edge to the previous one, in the current edge.</summary>
    /// <exception cref="NoSuchObjectException">The vertex isn't in the current edge.</exception>
    public Orientation Orientation()
    {
        if (_vertex.IsNull && !_edge.IsNull)
        {
            // infinite edge
            return Forward;
        }

        var it = new Iterator(_edge, false);
        while (it.More())
        {
            if (_vertex.IsSame(it.Value()))
            {
                return it.Value().Orientation;
            }
            it.Next();
        }
        throw new NoSuchObjectException("BRepTools_WireExplorer::Orientation");
    }

    /// <summary>The vertex connecting the current edge to the previous one.</summary>
    public Vertex CurrentVertex() => _vertex;

    /// <summary>Clears the explorer.</summary>
    public void Clear()
    {
        _map.Clear();
        _doubles.Clear();
        _edge = new Edge();
        _face = new Face();
        _vertex = new Vertex();
    }

    private static bool SelectDouble(HashSet<Shape> doubles, NCollection_List<Shape> l, ref Edge e)
    {
        var it = new NCollection_List<Shape>.Iterator(l);

        for (; it.More(); it.Next())
        {
            Shape ce = it.Value();
            if (doubles.Contains(ce) && (!e.IsSame(ce)))
            {
                e = ShapeCasts.Edge(ce);
                l.Remove(it);
                return true;
            }
        }
        return false;
    }

    private static bool SelectDegenerated(NCollection_List<Shape> l, ref Edge e)
    {
        var it = new NCollection_List<Shape>.Iterator(l);
        while (it.More())
        {
            if (!it.Value().IsSame(e))
            {
                e = ShapeCasts.Edge(it.Value());
                if (Tool.Degenerated(e))
                {
                    l.Remove(it);
                    return true;
                }
            }
            it.Next();
        }
        return false;
    }

    private static double GetNextParamOnPC(Curve2D aPC,
                                           in Point2D aPRef,
                                           double fP,
                                           double lP,
                                           double tolU,
                                           double tolV,
                                           bool reverse)
    {
        double result = (reverse) ? fP : lP;
        double dP = Math.Abs(lP - fP) / 1000.0; // was / 16.;
        if (reverse)
        {
            double startPar = fP;
            bool nextPntOnEdge = false;
            while (!nextPntOnEdge && startPar < lP)
            {
                startPar += dP;
                aPC.D0(startPar, out Point2D pnt);
                if (Math.Abs(aPRef.X - pnt.X) < tolU && Math.Abs(aPRef.Y - pnt.Y) < tolV)
                {
                    continue;
                }
                else
                {
                    result = startPar;
                    nextPntOnEdge = true;
                    break;
                }
            }

            if (!nextPntOnEdge)
            {
                result = lP;
            }

            if (result > lP)
            {
                result = lP;
            }
        }
        else
        {
            double startPar = lP;
            bool nextPntOnEdge = false;
            while (!nextPntOnEdge && startPar > fP)
            {
                startPar -= dP;
                aPC.D0(startPar, out Point2D pnt);
                if (Math.Abs(aPRef.X - pnt.X) < tolU && Math.Abs(aPRef.Y - pnt.Y) < tolV)
                {
                    continue;
                }
                else
                {
                    result = startPar;
                    nextPntOnEdge = true;
                    break;
                }
            }

            if (!nextPntOnEdge)
            {
                result = fP;
            }

            if (result < fP)
            {
                result = fP;
            }
        }

        return result;
    }
}
