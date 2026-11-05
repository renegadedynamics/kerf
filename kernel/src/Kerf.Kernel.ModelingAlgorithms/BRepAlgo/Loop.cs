// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Loop.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepAlgo/BRepAlgo_Loop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Algo;

/// <summary>
/// Builds the wires (loops) of a face from a set of edges with pcurves on it: edges to cut at given vertices, and
/// constant edges; then, optionally, the faces those wires bound (<see cref="WiresToFaces"/>).
/// </summary>
/// <remarks>
/// A working object of the offset algorithms. As in OCCT, it writes into the edges it is given (it replaces their
/// vertices by close ones and raises vertex tolerances): they must be unfrozen, as ported callers' are; a frozen one
/// throws at the first write. The shapes handed out are not frozen.
/// </remarks>
internal sealed class Loop
{
    private Face _face = new();
    private readonly NCollection_List<Shape> _constEdges = new();
    private readonly NCollection_List<Shape> _edges = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _verOnEdges = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> _newWires = new();
    private readonly NCollection_List<Shape> _newFaces = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _cutEdges = new(ShapeHasher.Instance);
    private Dictionary<Shape, Shape> _verticesForSubstitute = new(ShapeHasher.Instance);
    private readonly ShapeImage _imageVV = new();
    private double _tolConf;

    /// <summary>An empty loop builder.</summary>
    public Loop()
    {
        _tolConf = 0.001;
    }

    /// <summary>Starts again on the face <paramref name="f"/>; the edges must have pcurves on it.</summary>
    public void Init(in Face f)
    {
        _constEdges.Clear();
        _edges.Clear();
        _verOnEdges.Clear();
        _newWires.Clear();
        _newFaces.Clear();
        _cutEdges.Clear();
        _face = f;
    }

    // Orders the sequence of vertices by increasing parameter on E (removing duplicates)
    private static void Bubble(in Edge e, NCollection_Sequence<Shape> seq)
    {
        // Remove duplicates
        for (int i = 1; i < seq.Length(); i++)
        {
            for (int j = i + 1; j <= seq.Length(); j++)
            {
                if (seq.Value(i) == seq.Value(j))
                {
                    seq.Remove(j);
                    j--;
                }
            }
        }

        bool invert = true;
        int nbPoints = seq.Length();
        double u1, u2;
        Vertex v1, v2;

        while (invert)
        {
            invert = false;
            for (int i = 1; i < nbPoints; i++)
            {
                Shape localV = seq.Value(i).Oriented(Internal);
                v1 = ShapeCasts.Vertex(localV);
                localV = seq.Value(i + 1).Oriented(Internal);
                v2 = ShapeCasts.Vertex(localV);

                u1 = Tool.Parameter(v1, e);
                u2 = Tool.Parameter(v2, e);
                if (u2 < u1)
                {
                    seq.Exchange(i, i + 1);
                    invert = true;
                }
            }
        }
    }

    /// <summary>Adds the edge <paramref name="e"/>, to be cut at the vertices <paramref name="lv"/>.</summary>
    public void AddEdge(in Edge e, NCollection_List<Shape> lv)
    {
        _edges.Append(e);
        _verOnEdges[e] = new NCollection_List<Shape>(lv);
    }

    /// <summary>Adds the constant edge <paramref name="e"/>, which can be in the result as it is.</summary>
    public void AddConstEdge(in Edge e)
    {
        _constEdges.Append(e);
    }

    /// <summary>Adds the constant edges <paramref name="le"/>.</summary>
    public void AddConstEdges(NCollection_List<Shape> le)
    {
        NCollection_List<Shape>.Iterator itl = new(le);
        for (; itl.More(); itl.Next())
        {
            _constEdges.Append(itl.Value());
        }
    }

    /// <summary>Sets the images of vertices into vertices (a copy).</summary>
    public void SetImageVV(ShapeImage imageVV)
    {
        _imageVV.Assign(imageVV);
    }

    // If the first or the last vertex of intersection coincides with the closing vertex, it is removed from SV; it will
    // be added at the beginning and the end of SV by the caller
    private static Vertex UpdateClosedEdge(in Edge e, NCollection_Sequence<Shape> sv)
    {
        Vertex[] vb = [new(), new()];
        Vertex vRes = new();
        Point3D p, pc;
        bool onStart = false, onEnd = false;
        //// modified by jgv, 13.04.04 for OCC5634 ////
        Exploration.Vertices(e, out Vertex v1, out _);
        double tol = Tool.Tolerance(v1);
        ///////////////////////////////////////////////

        if (sv.IsEmpty())
        {
            return vRes;
        }

        vb[0] = ShapeCasts.Vertex(sv.First());
        vb[1] = ShapeCasts.Vertex(sv.Last());
        pc = Tool.Pnt(v1);

        for (int i = 0; i < 2; i++)
        {
            p = Tool.Pnt(vb[i]);
            if (p.IsEqual(pc, tol))
            {
                vRes = vb[i];
                if (i == 0)
                {
                    onStart = true;
                }
                else
                {
                    onEnd = true;
                }
            }
        }
        if (onStart && onEnd)
        {
            if (!vb[0].IsSame(vb[1]))
            {
                // Two different vertices on the closing vertex
            }
            else
            {
                sv.Remove(1);
                if (!sv.IsEmpty())
                {
                    sv.Remove(sv.Length());
                }
            }
        }
        else if (onStart)
        {
            sv.Remove(1);
        }
        else if (onEnd)
        {
            sv.Remove(sv.Length());
        }

        return vRes;
    }

    // Removes the hanging edges
    private static void RemovePendingEdges(IndexedMap<Shape, NCollection_List<Shape>> mve)
    {
        //--------------------------------
        // Remove hanging edges.
        //--------------------------------
        NCollection_List<Shape>.Iterator itl = new();
        bool yaSupress = true;

        while (yaSupress)
        {
            yaSupress = false;
            NCollection_List<Shape> vToRemove = new();
            HashSet<Shape> eToRemove = new(ShapeHasher.Instance);

            for (int iV = 0; iV < mve.Count; iV++)
            {
                Shape vertex = mve.KeyAt(iV);
                NCollection_List<Shape> edges = mve.ValueAt(iV);
                if (edges.IsEmpty())
                {
                    vToRemove.Append(vertex);
                }
                if (edges.Extent() == 1)
                {
                    Edge e = ShapeCasts.Edge(edges.First());
                    Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                    if (!v1.IsSame(v2))
                    {
                        vToRemove.Append(vertex);
                        eToRemove.Add(edges.First());
                    }
                }
            }

            if (!vToRemove.IsEmpty())
            {
                yaSupress = true;
                for (itl.Initialize(vToRemove); itl.More(); itl.Next())
                {
                    mve.Remove(itl.Value());
                }
                if (eToRemove.Count != 0)
                {
                    for (int iV = 0; iV < mve.Count; iV++)
                    {
                        NCollection_List<Shape> le = mve.ValueAt(iV);
                        itl.Initialize(le);
                        while (itl.More())
                        {
                            if (eToRemove.Contains(itl.Value()))
                            {
                                le.Remove(itl);
                            }
                            else
                            {
                                itl.Next();
                            }
                        }
                    }
                }
            }
        }
    }

    // Whether E1's start and E2's end meet at V in the face's parametric space (100 times V's tolerance)
    private static bool SamePnt2d(in Vertex v, in Edge e1, in Edge e2, in Face f)
    {
        Point2D p1, p2;
        Shape localF = f.Oriented(Forward);
        Face ff = ShapeCasts.Face(localF);
        Curve2D c1 = Tool.CurveOnSurface(e1, ff, out double f1, out double l1)!;
        Curve2D c2 = Tool.CurveOnSurface(e2, ff, out double f2, out double l2)!;
        if (e1.Orientation == Forward)
        {
            p1 = c1.Value(f1);
        }
        else
        {
            p1 = c1.Value(l1);
        }

        if (e2.Orientation == Forward)
        {
            p2 = c2.Value(l2);
        }
        else
        {
            p2 = c2.Value(f2);
        }
        double tol = 100 * Tool.Tolerance(v);
        double dist = p1.DistanceTo(p2);
        return dist < tol;
    }

    // Finds the edge NE connected to CE by the vertex CV in LE and removes it from LE; also removes CE from LE if it is
    // there with the same orientation
    private static bool SelectEdge(in Face F,
                                   in Edge ce,
                                   in Vertex cv,
                                   out Edge ne,
                                   NCollection_List<Shape> le)
    {
        NCollection_List<Shape>.Iterator itl = new();
        ne = new Edge();

        for (itl.Initialize(le); itl.More(); itl.Next())
        {
            if (itl.Value().IsEqual(ce))
            {
                le.Remove(itl);
                break;
            }
        }

        if (le.Extent() > 1)
        {
            //--------------------------------------------------------------
            // Several edges possible.
            // - Test edges different from CE , Selection of edge
            // for which CV has U,V closer to the face
            // than corresponding to CE.
            // - If several edges give representation less than the tolerance.
            // discrimination on tangents.
            //--------------------------------------------------------------
            Face fForward = F;
            fForward.Orient(Forward);

            Curve2D c = Tool.CurveOnSurface(ce, fForward, out double f, out double l)!;
            int k = 1, kmin = 0;
            double dist, distmin = 100 * Tool.Tolerance(cv);
            double u;
            if (ce.Orientation == Forward)
            {
                u = l;
            }
            else
            {
                u = f;
            }

            Point2D p2, pv = c.Value(u);

            for (itl.Initialize(le); itl.More(); itl.Next())
            {
                Edge e = ShapeCasts.Edge(itl.Value());
                if (!e.IsSame(ce))
                {
                    c = Tool.CurveOnSurface(e, fForward, out f, out l)!;
                    if (e.Orientation == Forward)
                    {
                        u = f;
                    }
                    else
                    {
                        u = l;
                    }
                    p2 = c.Value(u);
                    dist = pv.DistanceTo(p2);
                    if (dist <= distmin)
                    {
                        kmin = k;
                        distmin = dist;
                    }
                }
                k++;
            }

            if (kmin == 0)
            {
                return false;
            }

            k = 1;
            itl.Initialize(le);
            while (k < kmin)
            {
                k++;
                itl.Next();
            }
            ne = ShapeCasts.Edge(itl.Value());
            le.Remove(itl);
        }
        else if (le.Extent() == 1)
        {
            ne = ShapeCasts.Edge(le.First());
            le.RemoveFirst();
        }
        else
        {
            return false;
        }
        return true;
    }

    // Keeps in the lists of new edges only the edges used in the wires
    private static void PurgeNewEdges(Dictionary<Shape, NCollection_List<Shape>> newEdges,
                                      HashSet<Shape> usedEdges)
    {
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in newEdges)
        {
            NCollection_List<Shape> lne = it.Value;
            NCollection_List<Shape>.Iterator itL = new(lne);

            while (itL.More())
            {
                Shape ne = itL.Value();
                if (!usedEdges.Contains(ne))
                {
                    lne.Remove(itL);
                }
                else
                {
                    itL.Next();
                }
            }
        }
    }

    // Adds E to the map vertex -> edges, first replacing its vertices by vertices of the map at the same place
    // (within theTolConf), recorded in VerticesForSubstitute; doubles the seams
    private static void StoreInMVE(in Face f,
                                   in Edge e,
                                   IndexedMap<Shape, NCollection_List<Shape>> mve,
                                   ref bool yaCouture,
                                   Dictionary<Shape, Shape> verticesForSubstitute,
                                   double tolConf)
    {
        Vertex v1, v2, v;
        Point3D p1, p;
        BrepBuilder bb = new();
        for (int iV = 0; iV < mve.Count; iV++)
        {
            v = ShapeCasts.Vertex(mve.KeyAt(iV));
            p = Tool.Pnt(v);
            NCollection_List<Shape> vList = new();
            Iterator verExp = new(e);
            for (; verExp.More(); verExp.Next())
            {
                vList.Append(verExp.Value());
            }
            NCollection_List<Shape>.Iterator itl = new(vList);
            for (; itl.More(); itl.Next())
            {
                v1 = ShapeCasts.Vertex(itl.Value());
                p1 = Tool.Pnt(v1);
                if (p.IsEqual(p1, tolConf) && !v.IsSame(v1))
                {
                    v.Orient(v1.Orientation);
                    if (verticesForSubstitute.ContainsKey(v1))
                    {
                        Shape oldNewV = verticesForSubstitute.GetValue(v1);
                        if (!oldNewV.IsSame(v))
                        {
                            verticesForSubstitute[oldNewV] = v;
                            verticesForSubstitute.GetValueRef(v1) = v;
                        }
                    }
                    else
                    {
                        if (verticesForSubstitute.ContainsKey(v))
                        {
                            Shape newNewV = verticesForSubstitute.GetValue(v);
                            if (!newNewV.IsSame(v1))
                            {
                                verticesForSubstitute[v1] = newNewV;
                            }
                        }
                        else
                        {
                            verticesForSubstitute[v1] = v;
                            // OCCT changes the map's values while iterating it: the keys are collected first
                            NCollection_List<Shape> keys = new();
                            foreach (KeyValuePair<Shape, Shape> mapit in verticesForSubstitute)
                            {
                                if (mapit.Value.IsSame(v1))
                                {
                                    keys.Append(mapit.Key);
                                }
                            }
                            for (NCollection_List<Shape>.Iterator itk = new(keys); itk.More(); itk.Next())
                            {
                                verticesForSubstitute.GetValueRef(itk.Value()) = v;
                            }
                        }
                    }
                    e.Free(true);
                    bb.Remove(e, v1);
                    bb.Add(e, v);
                }
            }
        }

        Exploration.Vertices(e, out v1, out v2);
        if (v1.IsNull && v2.IsNull)
        {
            yaCouture = false;
            return;
        }
        if (!mve.ContainsKey(v1))
        {
            mve.Add(v1, new NCollection_List<Shape>());
        }
        mve.GetValue(v1).Append(e);
        if (!v1.IsSame(v2))
        {
            if (!mve.ContainsKey(v2))
            {
                mve.Add(v2, new NCollection_List<Shape>());
            }
            mve.GetValue(v2).Append(e);
        }
        Surface? s = Tool.Surface(f, out Placement l);
        if (Tool.IsClosed(e, s, l))
        {
            mve.GetValue(v2).Append(e.Reversed());
            if (!v1.IsSame(v2))
            {
                mve.GetValue(v1).Append(e.Reversed());
            }
            yaCouture = true;
        }
    }

    /// <summary>Cuts the edges and builds the wires.</summary>
    public void Perform()
    {
        NCollection_List<Shape>.Iterator itl = new(), itl1 = new();
        Vertex v1, v2;
        bool yaCouture = false;

        //------------------------------------------------
        // Cut edges
        //------------------------------------------------
        for (itl.Initialize(_edges); itl.More(); itl.Next())
        {
            Edge edge = ShapeCasts.Edge(itl.Value());
            NCollection_List<Shape> lce = new();
            if (_verOnEdges.TryGetValue(edge, out NCollection_List<Shape>? pVertices))
            {
                CutEdge(edge, pVertices, lce);
                _cutEdges[edge] = lce;
            }
        }
        //-----------------------------------
        // Construction map vertex => edges
        //-----------------------------------
        IndexedMap<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance);

        // add cut edges.
        HashSet<Shape> emap = new(ShapeHasher.Instance);
        for (itl.Initialize(_edges); itl.More(); itl.Next())
        {
            if (_cutEdges.TryGetValue(itl.Value(), out NCollection_List<Shape>? pLCE))
            {
                for (itl1.Initialize(pLCE); itl1.More(); itl1.Next())
                {
                    Edge e = ShapeCasts.Edge(itl1.Value());
                    if (!emap.Add(e))
                    {
                        continue;
                    }
                    StoreInMVE(_face, e, mve, ref yaCouture, _verticesForSubstitute, _tolConf);
                }
            }
        }

        // add const edges
        // Sewn edges can be doubled or not in myConstEdges
        // => call only once StoreInMVE which should double them
        HashSet<Shape> dejaVu = new(ShapeHasher.Instance);
        for (itl.Initialize(_constEdges); itl.More(); itl.Next())
        {
            Edge e = ShapeCasts.Edge(itl.Value());
            if (dejaVu.Add(e))
            {
                StoreInMVE(_face, e, mve, ref yaCouture, _verticesForSubstitute, _tolConf);
            }
        }

        //-----------------------------------------------
        // Construction of wires and new faces.
        //----------------------------------------------
        Vertex vf, cv;
        Edge ce, ne, ef;
        BrepBuilder b = new();
        Wire nw = new();
        bool end;

        UpdateVEmap(mve);

        HashSet<Shape> usedEdges = new(ShapeHasher.Instance);

        while (mve.Count > 0)
        {
            b.MakeWire(ref nw);
            //--------------------------------
            // Removal of hanging edges.
            //--------------------------------
            RemovePendingEdges(mve);

            if (mve.Count == 0)
            {
                break;
            }
            //--------------------------------
            // Start edge.
            //--------------------------------
            ef = ce = ShapeCasts.Edge(mve.ValueAt(0).First());
            Exploration.Vertices(ce, out v1, out v2);
            //--------------------------------
            // VF vertex start of new wire
            //--------------------------------
            if (ce.Orientation == Forward)
            {
                cv = vf = v1;
            }
            else
            {
                cv = vf = v2;
            }
            if (!mve.ContainsKey(cv))
            {
                continue;
            }
            NCollection_List<Shape> listEdges = mve.GetValue(cv);
            for (itl.Initialize(listEdges); itl.More(); itl.Next())
            {
                if (itl.Value().IsEqual(ce))
                {
                    listEdges.Remove(itl);
                    break;
                }
            }
            end = false;

            while (!end)
            {
                //-------------------------------
                // Construction of a wire.
                //-------------------------------
                Exploration.Vertices(ce, out v1, out v2);
                if (!cv.IsSame(v1))
                {
                    cv = v1;
                }
                else
                {
                    cv = v2;
                }

                b.Add(nw, ce);
                usedEdges.Add(ce);

                if (!mve.ContainsKey(cv) || mve.GetValue(cv).IsEmpty())
                {
                    end = true;
                }
                else
                {
                    end = !SelectEdge(_face, ce, cv, out ne, mve.GetValue(cv));
                    if (!end)
                    {
                        ce = ne;
                        if (mve.GetValue(cv).IsEmpty())
                        {
                            mve.Remove(cv);
                        }
                    }
                }
            }
            //--------------------------------------------------
            // Add new wire to the set of wires
            //------------------------------------------------

            if (vf.IsSame(cv))
            {
                if (SamePnt2d(vf, ef, ce, _face))
                {
                    nw.Closed(true);
                    _newWires.Append(nw);
                }
                else if (Tool.Tolerance(vf) < _tolConf)
                {
                    BrepBuilder bb = new();
                    bb.UpdateVertex(vf, _tolConf);
                    if (SamePnt2d(vf, ef, ce, _face))
                    {
                        nw.Closed(true);
                        _newWires.Append(nw);
                    }
                }
            }
        }
        PurgeNewEdges(_cutEdges, usedEdges);
    }

    /// <summary>
    /// Cuts the edge <paramref name="e"/> at the vertices <paramref name="vOnE"/> (FORWARD for a start, REVERSED for an
    /// end) into the edges <paramref name="ne"/>, dropping pieces shorter than 0.001 in the face's parametric space.
    /// </summary>
    public void CutEdge(in Edge e, NCollection_List<Shape> vOnE, NCollection_List<Shape> ne)
    {
        Shape localE = e.Oriented(Forward);
        Edge we = ShapeCasts.Edge(localE);

        double u1, u2;
        Vertex v1, v2;
        NCollection_Sequence<Shape> sv = new();
        NCollection_List<Shape>.Iterator it = new(vOnE);
        BrepBuilder b = new();

        for (; it.More(); it.Next())
        {
            sv.Append(it.Value());
        }
        //--------------------------------
        // Parse vertices on the edge.
        //--------------------------------
        Bubble(we, sv);

        int nbVer = sv.Length();
        //----------------------------------------------------------------
        // Construction of new edges.
        // Note :  vertices at the extremities of edges are not
        //         onligatorily in the list of vertices
        //----------------------------------------------------------------
        if (sv.IsEmpty())
        {
            ne.Append(e);
            return;
        }
        Tool.Range(we, out double f, out double l);
        Exploration.Vertices(we, out Vertex vf, out Vertex vl);

        if (nbVer == 2)
        {
            if (sv.Value(1).IsEqual(vf) && sv.Value(2).IsEqual(vl))
            {
                ne.Append(e);
                return;
            }
        }
        //----------------------------------------------------
        // Processing of closed edges
        // If a vertex of intersection is on the common vertex
        // it should appear at the beginning and end of SV.
        //----------------------------------------------------
        Vertex vcei;

        if (!vf.IsNull && vf.IsSame(vl))
        {
            vcei = UpdateClosedEdge(we, sv);
            if (!vcei.IsNull)
            {
                Shape localV = vcei.Oriented(Forward);
                vf = ShapeCasts.Vertex(localV);
                localV = vcei.Oriented(Reversed);
                vl = ShapeCasts.Vertex(localV);
            }
            sv.Prepend(vf);
            sv.Append(vl);
        }
        else
        {
            //-----------------------------------------
            // Eventually all extremities of the edge.
            //-----------------------------------------
            if (!vf.IsNull && !vf.IsSame(sv.First()))
            {
                sv.Prepend(vf);
            }
            if (!vl.IsNull && !vl.IsSame(sv.Last()))
            {
                sv.Append(vl);
            }
        }

        while (!sv.IsEmpty())
        {
            while (!sv.IsEmpty() && sv.First().Orientation != Forward)
            {
                sv.Remove(1);
            }
            if (sv.IsEmpty())
            {
                break;
            }
            v1 = ShapeCasts.Vertex(sv.First());
            sv.Remove(1);
            if (sv.IsEmpty())
            {
                break;
            }
            if (sv.First().Orientation == Reversed)
            {
                v2 = ShapeCasts.Vertex(sv.First());
                sv.Remove(1);
                //-------------------------------------------
                // Copy the edge and restriction by V1 V2.
                //-------------------------------------------
                Shape newEdge = we.EmptyCopied();
                Shape localEdge = v1.Oriented(Forward);
                b.Add(newEdge, localEdge);
                localEdge = v2.Oriented(Reversed);
                b.Add(ShapeCasts.Edge(newEdge), localEdge);
                if (v1.IsSame(vf))
                {
                    u1 = f;
                }
                else
                {
                    Shape localV = v1.Oriented(Internal);
                    u1 = Tool.Parameter(ShapeCasts.Vertex(localV), we);
                }
                if (v2.IsSame(vl))
                {
                    u2 = l;
                }
                else
                {
                    Shape localV = v2.Oriented(Internal);
                    u2 = Tool.Parameter(ShapeCasts.Vertex(localV), we);
                }
                b.Range(ShapeCasts.Edge(newEdge), u1, u2);
                ne.Append(newEdge.Oriented(e.Orientation));
            }
        }

        // Remove edges with size <= tolerance
        double tol = 0.001; // 5.e-05; //5.e-07;
        it.Initialize(ne);
        while (it.More())
        {
            // skl : I change "E" to "EE"
            Edge ee = ShapeCasts.Edge(it.Value());
            Tool.Range(ee, out double fpar, out double lpar);
            if (lpar - fpar <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                ne.Remove(it);
            }
            else
            {
                Tool.UVPoints(ee, _face, out Point2D pf, out Point2D pl);
                if (pf.DistanceTo(pl) <= tol && !Tool.IsClosed(ee))
                {
                    ne.Remove(it);
                }
                else
                {
                    it.Next();
                }
            }
        }
    }

    /// <summary>The wires built (possibly none).</summary>
    public NCollection_List<Shape> NewWires() => _newWires;

    /// <summary>Builds the faces the wires bound on the face (<see cref="FaceRestrictor"/>).</summary>
    public void WiresToFaces()
    {
        if (!_newWires.IsEmpty())
        {
            FaceRestrictor fr = new();
            Shape localS = _face.Oriented(Forward);
            fr.Init(ShapeCasts.Face(localS), false);
            NCollection_List<Shape>.Iterator it = new(_newWires);
            for (; it.More(); it.Next())
            {
                fr.Add(ShapeCasts.Wire(it.Value()));
            }

            fr.InternalPerform();

            if (fr.IsDone)
            {
                Orientation oriF = _face.Orientation;
                for (; fr.More(); fr.Next())
                {
                    _newFaces.Append(fr.InternalCurrent().Oriented(oriF));
                }
            }
        }
    }

    /// <summary>The faces built by <see cref="WiresToFaces"/> (possibly none).</summary>
    public NCollection_List<Shape> NewFaces() => _newFaces;

    /// <summary>The edges cut from the edge <paramref name="e"/> and used in the wires (possibly none).</summary>
    public NCollection_List<Shape> NewEdges(in Edge e) => _cutEdges.GetValue(e);

    /// <summary>Fills <paramref name="verVerMap"/> with the vertices and their substitutes.</summary>
    public void GetVerticesForSubstitute(Dictionary<Shape, Shape> verVerMap)
    {
        verVerMap.Clear();
        foreach (KeyValuePair<Shape, Shape> item in _verticesForSubstitute)
        {
            verVerMap[item.Key] = item.Value;
        }
    }

    /// <summary>Sets the vertices and their substitutes (a copy of <paramref name="verVerMap"/>).</summary>
    public void VerticesForSubstitute(Dictionary<Shape, Shape> verVerMap)
    {
        _verticesForSubstitute = new Dictionary<Shape, Shape>(verVerMap,verVerMap.Comparer);
    }

    /// <summary>
    /// Merges the vertices of single edges in <paramref name="vEmap"/> that are images of the same vertex (the image
    /// set by <see cref="SetImageVV"/>) into one, then rebuilds the map.
    /// </summary>
    public void UpdateVEmap(IndexedMap<Shape, NCollection_List<Shape>> vEmap)
    {
        IndexedMap<Shape, NCollection_List<Shape>> verLver = new(ShapeHasher.Instance);

        for (int ii = 0; ii < vEmap.Count; ii++)
        {
            Vertex vertex = ShapeCasts.Vertex(vEmap.KeyAt(ii));
            NCollection_List<Shape> elist = vEmap.ValueAt(ii);
            if (elist.Extent() == 1 && _imageVV.IsImage(vertex))
            {
                Vertex proVertex = ShapeCasts.Vertex(_imageVV.ImageFrom(vertex));
                if (verLver.ContainsKey(proVertex))
                {
                    NCollection_List<Shape> vlist = verLver.GetValue(proVertex);
                    vlist.Append(vertex.Oriented(Forward));
                }
                else
                {
                    NCollection_List<Shape> vlist = new();
                    vlist.Append(vertex.Oriented(Forward));
                    verLver.Add(proVertex, vlist);
                }
            }
        }

        if (verLver.Count == 0)
        {
            return;
        }

        BrepBuilder bb = new();
        for (int ii = 0; ii < verLver.Count; ii++)
        {
            NCollection_List<Shape> vlist = verLver.ValueAt(ii);
            if (vlist.Extent() == 1)
            {
                continue;
            }

            double maxTol = 0.0;
            NCollection_Array1<Point3D> points = new(1, vlist.Extent());

            NCollection_List<Shape>.Iterator itl = new(vlist);
            int jj = 0;
            for (; itl.More(); itl.Next())
            {
                Vertex vertex = ShapeCasts.Vertex(itl.Value());
                double tol = Tool.Tolerance(vertex);
                maxTol = Max(maxTol, tol);
                Point3D pnt = Tool.Pnt(vertex);
                points[++jj] = pnt;
            }

            global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(points, out Frame3D axis, out _);
            Point3D centre = axis.Location;
            double maxDist = 0.0;
            for (jj = 1; jj <= points.Upper(); jj++)
            {
                double sqDist = centre.SquareDistanceTo(points[jj]);
                maxDist = Max(maxDist, sqDist);
            }
            maxDist = Math.Sqrt(maxDist);
            maxTol = Max(maxTol, maxDist);

            // Find constant vertex
            Vertex constVertex = new();
            for (itl.Initialize(vlist); itl.More(); itl.Next())
            {
                Vertex vertex = ShapeCasts.Vertex(itl.Value());
                NCollection_List<Shape> elist = vEmap.GetValue(vertex);
                Shape edge = elist.First();
                NCollection_List<Shape>.Iterator itcedges = new(_constEdges);
                for (; itcedges.More(); itcedges.Next())
                {
                    if (edge.IsSame(itcedges.Value()))
                    {
                        constVertex = vertex;
                        break;
                    }
                }
                if (!constVertex.IsNull)
                {
                    break;
                }
            }
            if (constVertex.IsNull)
            {
                constVertex = ShapeCasts.Vertex(vlist.First());
            }
            bb.UpdateVertex(constVertex, centre, maxTol);

            for (itl.Initialize(vlist); itl.More(); itl.Next())
            {
                Vertex vertex = ShapeCasts.Vertex(itl.Value());
                if (vertex.IsSame(constVertex))
                {
                    continue;
                }

                NCollection_List<Shape> elist = vEmap.GetValue(vertex);
                Edge edge = ShapeCasts.Edge(elist.First());
                edge.Orient(Forward);
                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                Vertex vertexToRemove = v1.IsSame(vertex) ? v1 : v2;
                edge.Free(true);
                bb.Remove(edge, vertexToRemove);
                bb.Add(edge, constVertex.Oriented(vertexToRemove.Orientation));
            }
        }

        IndexedSet<Shape> emap = new(ShapeHasher.Instance);
        for (int ii = 0; ii < vEmap.Count; ii++)
        {
            NCollection_List<Shape> elist = vEmap.ValueAt(ii);
            NCollection_List<Shape>.Iterator itl = new(elist);
            for (; itl.More(); itl.Next())
            {
                emap.Add(itl.Value());
            }
        }

        vEmap.Clear();
        for (int ii = 0; ii < emap.Count; ii++)
        {
            Exploration.MapShapesAndAncestors(emap[ii], ShapeType.Vertex, ShapeType.Edge, vEmap);
        }
    }

    /// <summary>The largest distance between vertices that are merged.</summary>
    public double GetTolConf() => _tolConf;

    /// <summary>Sets the largest distance between vertices that are merged.</summary>
    public void SetTolConf(double tolConf)
    {
        _tolConf = tolConf;
    }
}
