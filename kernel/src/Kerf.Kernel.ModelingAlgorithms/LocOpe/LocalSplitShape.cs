// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// Provides a tool to cut edges with vertices and faces with wires, and rebuilds the shape containing the edges and
/// the faces.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, it writes into what it is given: the point representations of the split edges' vertices and of the
/// added vertices, the continuities of the added wires' edges. So each public call writes in the process's working
/// overlay (decision 1013, unit 10H5: <see cref="WorkingShapes"/>); the first <see cref="DescendantShapes"/> ends the
/// splitting, with new TShapes only for the caller's sub-shapes that changed. The shapes it hands out are
/// frozen; ported callers (LocOpe_Spliter) read the <c>Internal</c> forms, which share the spliter's working shapes.
/// </para>
/// <para>
/// <see cref="LeftOf"/> before the splitting ends answers with the working faces, which later additions may still
/// change (they are not frozen).
/// </para>
/// </remarks>
internal sealed class LocalSplitShape
{
    private bool _done;
    private Shape _shape = new();
    // OCCT iterates this map keyed by the shapes' addresses (CanSplit, which only tests); the port in insertion order
    private readonly Dictionary<Shape, NCollection_List<Shape>> _map = new(ShapeHasher.Instance);
    private readonly HashSet<Shape> _dblE = new(ShapeHasher.Instance);
    private NCollection_List<Shape> _left = new();
    private WorkingShapes _work = new();
    private bool _ownsWork = true;

    /// <summary>Empty constructor.</summary>
    public LocalSplitShape()
    {
        _done = false;
    }

    /// <summary>Creates the process with the shape <paramref name="s"/>.</summary>
    public LocalSplitShape(in Shape s)
    {
        Init(s);
    }

    /// <summary>For LocOpe_Spliter: the process on its working shape, sharing its working shapes (it ends them).</summary>
    internal LocalSplitShape(in Shape s, WorkingShapes work)
    {
        _work = work;
        _ownsWork = false;
        InitWorking(s);
    }

    /// <summary>Initializes the process on the shape <paramref name="s"/>.</summary>
    public void Init(in Shape s)
    {
        if (_ownsWork)
        {
            _work = new WorkingShapes();
        }
        InitWorking(s);
    }

    private void InitWorking(in Shape s)
    {
        _done = false;
        _shape = s;
        _dblE.Clear();
        _map.Clear();
        Put(_shape);
    }

    /// <summary>Tests if it is possible to split the edge <paramref name="e"/>.</summary>
    public bool CanSplit(in Edge e)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        return InternalCanSplit(e);
    }

    private bool InternalCanSplit(in Edge e)
    {
        if (_done)
        {
            return false;
        }
        if (_map.Count == 0)
        {
            return false;
        }

        if (!_map.ContainsKey(e))
        {
            return false;
        }

        // On verifie que l`edge n`appartient pas a un wire deja reconstruit
        Explorer exp = new();
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in _map)
        {
            if (itm.Key.Type == ShapeType.Wire && !itm.Value.IsEmpty())
            {
                for (exp.Init(itm.Key, ShapeType.Edge); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(e))
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    /// <summary>Adds the vertex <paramref name="v"/> on the edge <paramref name="e"/>, at parameter <paramref name="p"/>.</summary>
    public void Add(in Vertex v, double p, in Edge e)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        if (_done)
        {
            return;
        }
        InternalAdd(ShapeCasts.Vertex(v), p, ShapeCasts.Edge(e));
    }

    // Add(V, P, E) on working shapes
    internal void InternalAdd(in Vertex v, double p, in Edge e)
    {
        if (!InternalCanSplit(e))
        {
            return;
        }

        BrepBuilder b = new();
        NCollection_List<Shape> le = _map.GetValue(e);
        if (le.IsEmpty())
        {
            le.Append(e);
        }
        NCollection_List<Shape>.Iterator itl = new(le);
        double f, l;
        NCollection_List<Shape> newList = new();
        for (; itl.More(); itl.Next())
        {
            Edge edg1 = ShapeCasts.Edge(itl.Value());
            Tool.Range(edg1, out f, out l);
            if (p > f + global::Kerf.Engine.Foundation.Precision.PConfusion() && p < l - global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                break;
            }
            newList.Append(edg1);
        }
        if (!itl.More())
        {
            return;
        }
        Edge edg = ShapeCasts.Edge(itl.Value());
        le.Remove(itl);
        if (v.Orientation == Forward || v.Orientation == Reversed)
        {
            edg.Orient(Forward);
            Exploration.Vertices(edg, out Vertex curV1, out Vertex curV2);
            double par1 = Tool.Parameter(curV1, edg);

            double par2 = Tool.Parameter(curV2, edg);

            Shape localShape = edg.EmptyCopied();
            Edge e1 = ShapeCasts.Edge(localShape);
            localShape = edg.EmptyCopied();
            Edge e2 = ShapeCasts.Edge(localShape);
            e1.Orient(Forward);
            e2.Orient(Forward);
            Vertex newVtx = v;
            double tolSplitV = Tool.Tolerance(v);

            curV1.Orient(Forward);

            b.Add(e1, curV1);
            // for degenerated edges tolerance of vertices should be set to maximal value
            // from tolerance of the vertex of the edge and tolerance of splitting vertex
            double tolV1 = (Tool.Degenerated(edg) ? Max(Tool.Tolerance(curV1), tolSplitV)
                                                         : Tool.Tolerance(curV1));

            b.UpdateVertex(curV1, par1, e1, tolV1);
            newVtx.Orient(Reversed);
            b.Add(e1, newVtx);
            b.UpdateVertex(newVtx, p, e1, Tool.Tolerance(v));
            newVtx.Orient(Forward);
            b.Add(e2, newVtx);
            b.UpdateVertex(newVtx, p, e2, Tool.Tolerance(v));

            curV2.Orient(Reversed);
            b.Add(e2, curV2);

            // for degenerated edges tolerance of vertices should be set to maximal value
            // from tolerance of the vertex of the edge and tolerance of splitting vertex
            double tolV2 = (Tool.Degenerated(edg) ? tolV1 : Tool.Tolerance(curV2));
            b.UpdateVertex(curV2, par2, e2, tolV2);

            newList.Append(e1);
            newList.Append(e2);
            for (; itl.More(); itl.Next())
            {
                Edge edg1 = ShapeCasts.Edge(itl.Value());
                newList.Append(edg1);
            }
            _map.Remove(e);
            _map[e] = newList;
        }
        else
        {
            Shape localShape = edg.EmptyCopied();
            Edge e1 = ShapeCasts.Edge(localShape);

            Explorer exp = new();
            for (exp.Init(edg, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                f = Tool.Parameter(vtx, edg);
                b.Add(e1, vtx);
                b.UpdateVertex(vtx, f, e1, Tool.Tolerance(vtx));
            }
            b.Add(e1, v);
            b.UpdateVertex(v, p, e1, Tool.Tolerance(v));
            le.Append(e1);
        }
    }

    /// <summary>Adds the list of wires <paramref name="lwires"/> on the face <paramref name="f"/>.</summary>
    public bool Add(NCollection_List<Shape> lwires, in Face f)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        if (_done)
        {
            return false;
        }
        NCollection_List<Shape> wires = new();
        for (NCollection_List<Shape>.Iterator it = new(lwires); it.More(); it.Next())
        {
            wires.Append(it.Value());
        }
        return InternalAdd(wires, ShapeCasts.Face(f));
    }

    // Add(Lwires, F) on working shapes
    internal bool InternalAdd(NCollection_List<Shape> lwires, in Face f)
    {
        if (_done)
        {
            return false;
        }

        NCollection_List<Shape> lf = _map.GetValue(f);
        if (lf.IsEmpty())
        {
            Rebuild(f);
        }

        // On cherche la face descendante de F qui continent le wire
        lf = _map.GetValue(f);
        NCollection_List<Shape>.Iterator itl = new(lf);

        BrepTools.Update(f);

        NCollection_List<Shape> lInside = new();
        for (; itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());
            NCollection_List<Shape>.Iterator itwires = new(lwires);
            for (; itwires.More(); itwires.Next())
            {
                Wire wire = ShapeCasts.Wire(itwires.Value());
                if (IsInside(fac, wire))
                {
                    lInside.Append(wire);
                }
            }
            if (lInside.Extent() != 0)
            {
                break;
            }
        }
        if (lInside.Extent() == 0 || !itl.More())
        {
            return false;
        }

        Face faceRef = ShapeCasts.Face(itl.Value());
        faceRef.Orient(Forward);
        lf.Remove(itl);

        NCollection_List<Shape> newWires = new();

        Dictionary<Shape, int> sectionsTimes = new(ShapeHasher.Instance);
        for (itl.Initialize(lInside); itl.More(); itl.Next())
        {
            sectionsTimes[itl.Value()] = 2;
        }

        NCollection_List<Shape> breakVertices = new();
        NCollection_List<Shape> breakOnWires = new();

        // OCCT iterates this map keyed by the vertices' addresses to count a wire's entries, which isn't order dependent
        Dictionary<Shape, Shape> verWireMap = new(ShapeHasher.Instance);
        int i;
        Explorer exploF = new(), exploW = new();
        for (itl.Initialize(lInside); itl.More(); itl.Next())
        {
            Wire section = ShapeCasts.Wire(itl.Value());
            Vertex[] Ver = new Vertex[2];
            Exploration.Vertices(section, out Ver[0], out Ver[1]);
            for (i = 0; i < 2; i++)
            {
                if (verWireMap.ContainsKey(Ver[i]))
                {
                    continue;
                }
                for (exploF.Init(faceRef, ShapeType.Wire); exploF.More(); exploF.Next())
                {
                    Shape wire = exploF.Current();
                    Shape ver = new();
                    for (exploW.Init(wire, ShapeType.Vertex); exploW.More(); exploW.Next())
                    {
                        ver = exploW.Current();
                        if (ver.IsSame(Ver[i]))
                        {
                            break;
                        }
                    }
                    if (ver.IsSame(Ver[i]))
                    {
                        verWireMap[ver] = wire;
                        break;
                    }
                }
            }
        }

        Dictionary<Shape, NCollection_List<Shape>> verSecMap = new(ShapeHasher.Instance);
        for (itl.Initialize(lInside); itl.More(); itl.Next())
        {
            Wire wire = ShapeCasts.Wire(itl.Value());
            Exploration.Vertices(wire, out Vertex v1, out Vertex v2);
            NCollection_List<Shape> lw1 = new(), lw2 = new();
            if (!verSecMap.ContainsKey(v1))
            {
                verSecMap[v1] = lw1;
            }
            verSecMap.GetValue(v1).Append(wire);
            if (!verSecMap.ContainsKey(v2))
            {
                verSecMap[v2] = lw2;
            }
            verSecMap.GetValue(v2).Append(wire);
        }

        Wire outerW = BrepTools.OuterWire(faceRef);
        Wire curWire = outerW;
        LibraryMakeWire mw;
        mw = new LibraryMakeWire();
        WireExplorer wexp = new(curWire, faceRef);
        for (;;)
        {
            Vertex theStartVertex = wexp.CurrentVertex(), curVertex = new();
            Edge curEdge = wexp.Current();
            Edge lastEdge = curEdge;
            mw.Add(curEdge);
            Wire sectionWire = new();
            Vertex breakVertex;
            wexp.Next();
            // Kerf: an open wire would wrap round for ever. A closed one closes within one wrap (decision 22501)
            int wraps = 0;
            if (!wexp.More())
            {
                wraps++;
                wexp.Init(curWire, faceRef);
            }
            for (;;)
            {
                if (mw.InternalWire().ClosedFlag)
                {
                    break;
                }
                curVertex = wexp.CurrentVertex();
                if (verSecMap.ContainsKey(curVertex))
                {
                    Shape localWire = ChooseDirection(lastEdge, curVertex, faceRef, verSecMap.GetValue(curVertex));
                    sectionWire = ShapeCasts.Wire(localWire);
                    break;
                }
                curEdge = wexp.Current();
                mw.Add(curEdge);
                lastEdge = curEdge;
                wexp.Next();
                if (!wexp.More())
                {
                    if (++wraps > 1)
                    {
                        throw new ConstructionErrorException("LocOpe_SplitShape: a wire to split doesn't close");
                    }
                    wexp.Init(curWire, faceRef);
                }
            }
            if (mw.InternalWire().ClosedFlag)
            {
                newWires.Append(mw.InternalWire());
                theStartVertex = ShapeCasts.Vertex(breakVertices.First());
                breakVertices.RemoveFirst();
                curWire = ShapeCasts.Wire(breakOnWires.First());
                breakOnWires.RemoveFirst();
                wexp.Init(curWire, faceRef);
                MoveTo(wexp, theStartVertex);
                mw = new LibraryMakeWire();
                continue;
            }
            breakVertex = curVertex;
            breakVertices.Append(breakVertex);
            breakOnWires.Append(curWire);
            for (;;)
            {
                mw.Add(sectionWire);
                (sectionsTimes.GetValueRef(sectionWire))--;
                if (sectionsTimes.GetValue(sectionWire) == 0)
                {
                    sectionsTimes.Remove(sectionWire);
                }
                if (mw.InternalWire().ClosedFlag)
                {
                    newWires.Append(mw.InternalWire());
                    if (sectionsTimes.Count == 0)
                    {
                        break;
                    }
                    theStartVertex = ShapeCasts.Vertex(breakVertices.First());
                    breakVertices.RemoveFirst();
                    curWire = ShapeCasts.Wire(breakOnWires.First());
                    breakOnWires.RemoveFirst();
                    wexp.Init(curWire, faceRef);
                    MoveTo(wexp, theStartVertex);
                    mw = new LibraryMakeWire();
                    break;
                }
                else
                {
                    Exploration.Vertices(sectionWire, out Vertex v1, out Vertex v2);
                    Vertex startVertex = (v1.IsSame(breakVertex)) ? v2 : v1;
                    curWire = ShapeCasts.Wire(verWireMap.GetValue(startVertex));

                    wexp.Init(curWire, faceRef);
                    MoveTo(wexp, startVertex);

                    NCollection_List<Shape> lsections = verSecMap.GetValue(startVertex);
                    if (lsections.Extent() == 1)
                    {
                        break;
                    }

                    // else: choose the way
                    Wire nextSectionWire =
                        ShapeCasts.Wire((sectionWire.IsSame(lsections.First())) ? lsections.Last() : lsections.First());

                    int times = 0;
                    foreach (KeyValuePair<Shape, Shape> itVW in verWireMap)
                    {
                        if (itVW.Value.IsSame(curWire))
                        {
                            times++;
                        }
                    }
                    if (times == 1) // it is inner touching wire
                    {
                        // InnerTouchingWiresOnVertex.Bind(aWire, aStartVertex);
                    }
                    else
                    {
                        // we have to choose the direction
                        Edge startEdge = wexp.Current();
                        NCollection_List<Shape> ldirs = new();
                        ldirs.Append(startEdge);
                        ldirs.Append(nextSectionWire);
                        Shape theDirection = ChooseDirection(sectionWire, startVertex, faceRef, ldirs);
                        if (theDirection.IsSame(startEdge))
                        {
                            break;
                        }
                    }
                    sectionWire = nextSectionWire;
                    breakVertex = startVertex;
                } // end of else (MW is not closed)
            } // end of for (;;) (loop on section wires)
            if (sectionsTimes.Count == 0)
            {
                break;
            }
        } // end of global for (;;)

        NCollection_List<Shape> newFaces = new();
        BrepBuilder bb = new();
        for (itl.Initialize(newWires); itl.More(); itl.Next())
        {
            Shape localFace = faceRef.EmptyCopied();
            Face newFace = ShapeCasts.Face(localFace);
            newFace.Orient(Forward);
            bb.Add(newFace, itl.Value());
            newFaces.Append(newFace);
        }

        // Inserting holes
        NCollection_List<Shape> holes = new();
        for (exploF.Init(faceRef, ShapeType.Wire); exploF.More(); exploF.Next())
        {
            Shape wire = exploF.Current();
            exploW.Init(wire, ShapeType.Edge);
            Shape edge = exploW.Current();
            bool found = false;
            for (itl.Initialize(newWires); itl.More(); itl.Next())
            {
                Shape newWire = itl.Value();
                for (exploW.Init(newWire, ShapeType.Edge); exploW.More(); exploW.Next())
                {
                    if (edge.IsSame(exploW.Current()))
                    {
                        found = true;
                        break;
                    }
                }
                if (found)
                {
                    break;
                }
            }
            if (!found)
            {
                holes.Append(wire);
            }
        }
        NCollection_List<Shape>.Iterator itlNewF = new();
        for (itl.Initialize(holes); itl.More(); itl.Next())
        {
            Wire hole = ShapeCasts.Wire(itl.Value());
            for (itlNewF.Initialize(newFaces); itlNewF.More(); itlNewF.Next())
            {
                Face newFace = ShapeCasts.Face(itlNewF.Value());
                if (IsInside(newFace, hole))
                {
                    bb.Add(newFace, hole);
                    break;
                }
            }
        }

        // Update "myMap"
        lf.Append(newFaces);

        // Update of descendants of wires
        for (exploF.Init(f, ShapeType.Wire); exploF.More(); exploF.Next())
        {
            NCollection_List<Shape> ls = _map.GetValue(exploF.Current());
            ls.Clear();
        }
        ///////////////////

        // JAG 10.11.95 Codage des regularites
        for (itl.Initialize(lInside); itl.More(); itl.Next())
        {
            for (exploW.Init(itl.Value(), ShapeType.Edge); exploW.More(); exploW.Next())
            {
                Edge edg = ShapeCasts.Edge(exploW.Current());
                if (!Tool.HasContinuity(edg, f, f))
                {
                    bb.Continuity(edg, f, f, CN);
                }
            }
        }
        return true;
    }

    /// <summary>Adds the wire <paramref name="w"/> on the face <paramref name="f"/>.</summary>
    public bool Add(in Wire w, in Face f)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        if (_done)
        {
            return false;
        }
        return InternalAdd(ShapeCasts.Wire(w), ShapeCasts.Face(f));
    }

    // Add(W, F) on working shapes
    internal bool InternalAdd(in Wire w, in Face f)
    {
        if (_done)
        {
            return false;
        }

        Explorer exp = new();
        NCollection_List<Shape> lf = _map.GetValue(f);
        if (lf.IsEmpty())
        {
            Rebuild(f);
        }
        try
        {
            if (!global::Kerf.Engine.Features.Local.LocOpe.Closed(w, f))
            {
                if (!AddOpenWire(w, f))
                {
                    return false;
                }
            }
            else
            {
                if (!AddClosedWire(w, f))
                {
                    return false;
                }
            }
        }
        catch (Exception e) when ((Refusal.IsFailure(e) || e is NullReferenceException) && e is not ImmutableObjectException)
        {
            // OCCT_DEBUG trace left out: "SpliShape internal problem detected, some faces may be lost"
            return false;
        }
        // JAG 10.11.95 Codage des regularites
        BrepBuilder b = new();
        for (exp.Init(w, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!Tool.HasContinuity(edg, f, f))
            {
                b.Continuity(edg, f, f, CN);
            }
        }
        return true;
    }

    private bool AddClosedWire(in Wire w, in Face f)
    {
        Explorer exp = new();

        // On cherche la face descendante de F qui continent le wire
        NCollection_List<Shape> lf = _map.GetValue(f);
        NCollection_List<Shape>.Iterator itl = new(lf);
        for (; itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());

            if (IsInside(fac, w))
            {
                break;
            }
        }
        if (!itl.More())
        {
            return false;
        }

        BrepBuilder b = new();

        Orientation orWire = w.Orientation;
        Shape localFace = f.EmptyCopied();
        Face newFace = ShapeCasts.Face(localFace);
        newFace.Orient(Forward);
        b.Add(newFace, w);
        FClass2d classif = new(newFace, global::Kerf.Engine.Foundation.Precision.PConfusion());
        if (classif.PerformInfinitePoint() == In)
        {
            // le wire donne defini un trou
            localFace = f.EmptyCopied();
            newFace = ShapeCasts.Face(localFace);
            newFace.Orient(Forward);
            orWire = global::Kerf.Engine.Topology.TopAbs.Reverse(orWire);
            b.Add(newFace, w.Oriented(orWire));
        }

        Face faceRef = ShapeCasts.Face(itl.Value());
        faceRef.Orient(Forward);
        lf.Remove(itl);

        localFace = faceRef.EmptyCopied();
        Face newRef = ShapeCasts.Face(localFace);
        newRef.Orient(Forward);

        // On suppose que les edges du wire ont des courbes 2d.
        // Comme on ne change pas de surface de base, pas besoin d`UpdateEdge.

        for (exp.Init(faceRef.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
        {
            Wire wir = ShapeCasts.Wire(exp.Current());
            if (IsInside(f, wir, w))
            {
                b.Add(newFace, wir);
            }
            else
            {
                b.Add(newRef, wir);
            }
        }
        b.Add(newRef, w.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(orWire)));
        lf.Append(newRef);
        lf.Append(newFace);
        return true;
    }

    private static bool CheckOverlapping(in Edge edge1, in Edge edge2, in Face face)
    {
        FaceSurfaceAdaptor adS = new(face, false);

        double maxTol = (Tool.Tolerance(edge1) + Tool.Tolerance(edge2));

        double maxTol2d = Max(adS.UResolution(maxTol), adS.VResolution(maxTol));
        double tolAng = Math.PI / 180.0;
        ProjectPointOnCurve2D proj = new();

        Curve2D crv1 = Tool.CurveOnSurface(edge1, face, out double f1, out double l1)!;
        Curve2D crv2 = Tool.CurveOnSurface(edge2, face, out double f2, out double l2)!;
        int j = 1, nbP = 4;
        double step = (l2 - f2) / nbP;
        for (; j < nbP; j++)
        {
            double par2 = f2 + step * j;
            crv2.D1(par2, out Point2D p2d, out Vector2D v2);

            proj.Init(p2d, crv1, f1, l1);
            // check intermediate points
            if (proj.NbPoints() == 0 || proj.LowerDistance() > maxTol2d)
            {
                return false;
            }
            double par1 = proj.LowerDistanceParameter();
            crv1.D1(par1, out Point2D p2d1, out Vector2D v1);

            if (!v1.IsParallel(v2, tolAng))
            {
                return false;
            }
        }
        return true;
    }

    private bool AddOpenWire(in Wire w, in Face F)
    {
        // On cherche la face descendante de F qui continent le wire
        NCollection_List<Shape> lf = _map.GetValue(F);
        NCollection_List<Shape>.Iterator itl = new(lf);
        Vertex vfirst, vlast;

        BrepTools.Update(F);

        double tolf, toll, tol1;

        Shape localShape = w.Oriented(Forward);
        Exploration.Vertices(ShapeCasts.Wire(localShape), out vfirst, out vlast);

        tolf = Tool.Tolerance(vfirst);
        toll = Tool.Tolerance(vlast);
        tol1 = Max(tolf, toll);

        Explorer exp = new(), exp2 = new();

        Wire wfirst = new(), wlast = new();
        for (; itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());
            if (!IsInside(fac, w))
            {
                continue;
            }

            fac.Orient(Forward);
            bool ffound = false;
            bool lfound = false;
            for (exp.Init(fac, ShapeType.Wire); exp.More(); exp.Next())
            {
                Wire wir = ShapeCasts.Wire(exp.Current());
                for (exp2.Init(wir, ShapeType.Vertex); exp2.More(); exp2.Next())
                {
                    if (!ffound && exp2.Current().IsSame(vfirst))
                    {
                        ffound = true;
                        wfirst = wir;
                    }
                    else if (!lfound && exp2.Current().IsSame(vlast))
                    {
                        lfound = true;
                        wlast = wir;
                    }
                    if (ffound && lfound)
                    {
                        break;
                    }
                }
                if (exp2.More())
                {
                    break;
                }
            }
            if (exp.More())
            {
                break;
            }
        }
        if (!itl.More())
        {
            return false;
        }

        Face faceRef = ShapeCasts.Face(itl.Value());
        faceRef.Orient(Forward);
        lf.Remove(itl);
        BrepBuilder b = new();

        FaceSurfaceAdaptor bas = new(faceRef, false);

        bool isPeriodic = bas.IsUPeriodic || bas.IsVPeriodic;

        tol1 = Max(bas.UResolution(tol1), bas.VResolution(tol1));

        if (wfirst.IsSame(wlast))
        {
            // on cree 2 faces en remplacement de itl.Value()
            // Essai JAG
            NCollection_List<Shape> wiresFirst = new();
            for (exp.Init(wfirst, ShapeType.Edge); exp.More(); exp.Next())
            {
                if (Tool.IsClosed(ShapeCasts.Edge(exp.Current()), faceRef))
                {
                    _dblE.Add(exp.Current());
                }
                wiresFirst.Append(exp.Current());
            }

            Orientation orient;
            Wire newW1 = new(), newW2 = new();
            b.MakeWire(ref newW1);
            newW1.Orient(Forward);
            b.MakeWire(ref newW2);
            newW2.Orient(Forward);

            int nbE = 0;
            for (exp.Init(w.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                nbE++;
                Edge edg = ShapeCasts.Edge(exp.Current());
                orient = edg.Orientation;
                wiresFirst.Append(edg);
                wiresFirst.Append(edg.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(orient)));
                _dblE.Add(edg);
            }

            IndexedSet<Shape> possE = new(ShapeHasher.Instance);
            // OCCT's default hasher: the shapes' TShapes, locations and orientations
            HashSet<Shape> mapE = new(EqualityComparer<Shape>.Default);
            Vertex vdeb, vfin;
            int nbPoss;

            // On recherche l`edge contenant Vlast
            Edge lastEdge = new();
            Point2D pfirst, plast;
            Vector2D dlast;
            Curve2D c2d;
            double f, l;

            for (exp.Init(w.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                for (exp2.Init(edg, ShapeType.Vertex); exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(vfirst))
                    {
                        break;
                    }
                }
                if (exp2.More())
                {
                    lastEdge = edg;
                    lastEdge.Orient(edg.Orientation);
                    break;
                }
            }

            Shape localFace = faceRef.Oriented(wfirst.Orientation);
            c2d = Tool.CurveOnSurface(lastEdge, ShapeCasts.Face(localFace), out f, out l)!;

            if (lastEdge.Orientation == Forward)
            {
                pfirst = c2d.Value(f);
            }
            else
            {
                pfirst = c2d.Value(l);
            }

            for (exp.Init(w.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                if (nbE > 1 && edg.IsSame(lastEdge))
                {
                    continue;
                }
                for (exp2.Init(edg, ShapeType.Vertex); exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(vlast))
                    {
                        break;
                    }
                }
                if (exp2.More())
                {
                    lastEdge = edg;
                    lastEdge.Orient(edg.Orientation);
                    break;
                }
            }
            localFace = faceRef.Oriented(wfirst.Orientation);
            GetDirection(lastEdge, ShapeCasts.Face(localFace), out plast, out dlast, false);

            bool cond = true;

            if (isPeriodic)
            {
                cond = !(vfirst.IsSame(vlast) && SameUV(pfirst, plast, bas));
            }
            else
            {
                cond = !(vfirst.IsSame(vlast));
            }

            while (cond)
            {
                possE.Clear();

                // On enchaine par la fin
                NCollection_List<Shape>.Iterator lexp1 = new(wiresFirst);
                for (; lexp1.More(); lexp1.Next())
                {
                    Edge edg = ShapeCasts.Edge(lexp1.Value());

                    orient = edg.Orientation;
                    Exploration.Vertices(edg, out vdeb, out vfin);

                    if ((orient == Forward && vlast.IsSame(vdeb))
                        || (orient == Reversed && vlast.IsSame(vfin)))
                    {
                        possE.Add(edg);
                    }
                }
                nbPoss = possE.Count;
                if (nbPoss == 0)
                {
                    break;
                }

                Edge nextEdge = new();
                if (nbPoss == 1)
                {
                    nextEdge = ShapeCasts.Edge(possE[0]);
                    Shape localFaceTemp = faceRef.Oriented(wfirst.Orientation);
                    GetDirection(nextEdge, ShapeCasts.Face(localFaceTemp), out plast, out dlast, false);
                }
                else if (nbPoss > 1)
                {
                    // Faire choix en U,V...
                    Shape localFaceTemp = faceRef.Oriented(wfirst.Orientation);

                    if (!ChoixUV(lastEdge, ShapeCasts.Face(localFaceTemp), possE, ref nextEdge, ref plast, ref dlast))
                    {
                        return false;
                    }
                }

                if (nbPoss >= 1)
                {
                    if (nextEdge.IsNull)
                    {
                        // loop is not closed. Split is not possible
                        return false;
                    }

                    if (mapE.Contains(nextEdge))
                    {
                        break;
                    }
                    b.Add(newW1, nextEdge);
                    mapE.Add(nextEdge);
                    lastEdge = nextEdge;

                    if (lastEdge.Orientation == Forward)
                    {
                        vlast = Exploration.LastVertex(lastEdge);
                    }
                    else
                    {
                        vlast = Exploration.FirstVertex(lastEdge);
                    }

                    toll = Tool.Tolerance(vlast);
                    tol1 = Max(tolf, toll);
                }
                // MODIFICATION PIERRE SMEYERS : si pas de possibilite, on sort avec erreur
                else
                {
                    // OCCT writes "erreur Spliter : pas de chainage du wire" to std::cout (left out)
                    return false;
                }
                // fin MODIF.

                tol1 = Max(bas.UResolution(tol1), bas.VResolution(tol1));
            }

            int nbAddBound = 0;
            NCollection_List<Shape>.Iterator lexp = new(wiresFirst);
            Shape e1 = new(), e2 = new();
            for (; lexp.More(); lexp.Next())
            {
                Edge edg = ShapeCasts.Edge(lexp.Value());
                if (!mapE.Contains(edg))
                {
                    b.Add(newW2, edg);
                    mapE.Add(edg);
                    nbAddBound++;
                    if (e1.IsNull)
                    {
                        e1 = edg;
                    }
                    else
                    {
                        e2 = edg;
                    }
                }
            }
            // check overlapping edges for second face
            if (nbAddBound < 2)
            {
                return false;
            }
            if (nbAddBound == 2 && !e1.IsNull && !e2.IsNull)
            {
                if (CheckOverlapping(ShapeCasts.Edge(e1), ShapeCasts.Edge(e2), faceRef))
                {
                    return false;
                }
            }

            nbAddBound = 0;

            Shape e11 = new(), e12 = new();
            Iterator itE = new(newW1, false);
            for (; itE.More(); itE.Next())
            {
                if (itE.Value().Type != ShapeType.Edge)
                {
                    continue;
                }
                nbAddBound++;
                if (e11.IsNull)
                {
                    e11 = itE.Value();
                }
                else
                {
                    e12 = itE.Value();
                }
            }
            // check overlapping edges for first face
            if (nbAddBound < 2)
            {
                return false;
            }
            if (nbAddBound == 2 && !e11.IsNull && !e12.IsNull)
            {
                if (CheckOverlapping(ShapeCasts.Edge(e11), ShapeCasts.Edge(e12), faceRef))
                {
                    return false;
                }
            }

            Face newF1, newF2;
            localFace = faceRef.EmptyCopied();
            newF1 = ShapeCasts.Face(localFace);
            newF1.Orient(Forward);
            localFace = faceRef.EmptyCopied();
            newF2 = ShapeCasts.Face(localFace);
            newF2.Orient(Forward);

            // modifs JAG 97.05.28

            b.Add(newF1, newW1);
            b.Add(newF2, newW2);
            for (exp.Init(faceRef.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
            {
                Wire wir = ShapeCasts.Wire(exp.Current());
                if (!wir.IsSame(wfirst))
                {
                    if (IsInside(newF1, wir))
                    {
                        b.Add(newF1, wir);
                    }
                    else if (IsInside(newF2, wir))
                    {
                        b.Add(newF2, wir);
                    }
                    else
                    {
                        // Ce wire est ni dans newF2 ni dans newF1
                        // Peut etre faut il construire une troisieme face
                        // OCCT writes "WARNING: LocOpe_SPlitShape : Ce wire est ni dans newF2 ni dans newF1" to
                        // std::cout (left out)
                    }
                }
            }
            lf.Append(newF1);
            lf.Append(newF2);

            // Mise a jour des descendants des wires
            for (exp.Init(F, ShapeType.Wire); exp.More(); exp.Next())
            {
                NCollection_List<Shape> ls = _map.GetValue(exp.Current());
                itl.Initialize(ls);
                for (; itl.More(); itl.Next())
                {
                    if (itl.Value().IsSame(wfirst))
                    {
                        break;
                    }
                }
                if (itl.More())
                { // on a trouve le wire
                    ls.Remove(itl);
                    ls.Append(newW1);
                    ls.Append(newW2);
                }
            }
        }
        else
        {
            // on ne cree qu`une seule face
            Wire outerW = BrepTools.OuterWire(faceRef);
            Wire newWire = new();
            Face newFace;
            b.MakeWire(ref newWire);
            newWire.Orient(Forward);
            Orientation orient, orRelat;

            if (wfirst.Orientation == wlast.Orientation)
            {
                orRelat = Forward;
            }
            else
            {
                orRelat = Reversed;
            }

            if (wlast.IsSame(outerW))
            {
                wlast = wfirst;
                wfirst = outerW;
            }

            // Edges de wfirst
            for (exp.Init(wfirst.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                b.Add(newWire, ShapeCasts.Edge(exp.Current()));
            }

            // Edges de wlast
            for (exp.Init(wlast.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                orient = global::Kerf.Engine.Topology.TopAbs.Compose(edg.Orientation, orRelat);
                b.Add(newWire, edg.Oriented(orient));
            }

            // Edges du wire ajoute, et dans les 2 sens
            for (exp.Init(w.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                orient = edg.Orientation;
                b.Add(newWire, edg.Oriented(orient));
                b.Add(newWire, edg.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(orient)));
                _dblE.Add(edg.Oriented(orient));
            }

            // on refait une face

            Shape localFace = faceRef.EmptyCopied();
            newFace = ShapeCasts.Face(localFace);
            faceRef.Orient(Forward);
            for (exp.Init(faceRef.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
            {
                Wire wir = ShapeCasts.Wire(exp.Current());
                if (wir.IsSame(wfirst))
                {
                    b.Add(newFace, newWire.Oriented(wir.Orientation));
                }
                else if (!wir.IsSame(wlast))
                {
                    b.Add(newFace, wir);
                }
            }
            lf.Append(newFace);

            // Mise a jour des descendants des wires
            for (exp.Init(F, ShapeType.Wire); exp.More(); exp.Next())
            {
                NCollection_List<Shape> ls = _map.GetValue(exp.Current());
                itl.Initialize(ls);
                bool touch = false;
                while (itl.More())
                {
                    if (itl.Value().IsSame(wfirst) || itl.Value().IsSame(wlast))
                    {
                        ls.Remove(itl);
                        touch = true;
                    }
                    else
                    {
                        itl.Next();
                    }
                }
                if (touch)
                {
                    ls.Append(newWire);
                }
            }
        }
        _ = tol1;
        return true;
    }

    /// <summary>
    /// Returns the "left" part defined by the wire <paramref name="w"/> on the face <paramref name="f"/>: a list of
    /// faces. <paramref name="f"/> is considered with its topological orientation in the original shape,
    /// <paramref name="w"/> with its orientation. Frozen once the splitting has ended.
    /// </summary>
    public NCollection_List<Shape> LeftOf(in Wire w, in Face f)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        NCollection_List<Shape> left = InternalLeftOf(ShapeCasts.Wire(w), ShapeCasts.Face(f));
        return _done && _ownsWork ? WorkingShapes.Published(left) : left;
    }

    // LeftOf on working shapes
    internal NCollection_List<Shape> InternalLeftOf(in Wire w, in Face f)
    {
        if (_shape.IsNull)
        {
            throw new NoSuchObjectException();
        }

        Explorer exp = new(), expw = new(), expf = new();
        exp.Init(_shape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(f))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new NoSuchObjectException();
        }
        _left = new NCollection_List<Shape>(); // OCCT clears the list it returned before

        Face theFace = ShapeCasts.Face(exp.Current());
        Orientation orFace = theFace.Orientation;
        NCollection_List<Shape>.Iterator itl = new(), itl2 = new();

        for (expw.Init(w, ShapeType.Edge); expw.More(); expw.Next())
        {
            // Kerf: once the splitting has ended, the wire's edges as they stand in the faces (unit 10H5)
            Edge edg = ShapeCasts.Edge(_work.Final(expw.Current()));
            for (itl.Initialize(_map.GetValue(theFace)); itl.More(); itl.Next())
            {
                Face fac = ShapeCasts.Face(itl.Value());
                fac.Orient(orFace);
                for (expf.Init(fac, ShapeType.Edge); expf.More(); expf.Next())
                {
                    Edge edgbis = ShapeCasts.Edge(expf.Current());
                    if (edgbis.IsSame(edg) && edgbis.Orientation == edg.Orientation)
                    {
                        for (itl2.Initialize(_left); itl2.More(); itl2.Next())
                        {
                            if (itl2.Value().IsSame(fac))
                            {
                                break;
                            }
                        }
                        if (!itl2.More())
                        { // la face n`est pas deja presente
                            _left.Append(fac);
                        }
                        break;
                    }
                }
                if (expf.More())
                { // face found
                    break;
                }
            }
        }
        return _left;
    }

    /// <summary>
    /// Returns the list of descendant shapes of <paramref name="s"/> (frozen). The first call ends the splitting.
    /// </summary>
    public NCollection_List<Shape> DescendantShapes(in Shape s)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        NCollection_List<Shape> list = InternalDescendantShapes(s);
        return _ownsWork ? WorkingShapes.Published(list) : list;
    }

    // DescendantShapes on working shapes; ends the splitting, and gives back the caller's unchanged sub-shapes when
    // the working shapes are this process's own
    internal NCollection_List<Shape> InternalDescendantShapes(in Shape s)
    {
        if (!_done)
        {
            Rebuild(_shape);
            _done = true;
            if (_ownsWork && _work.IsWritten)
            {
                var shapes = new List<Shape>();
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _map)
                {
                    shapes.AddRange(item.Value);
                }
                _work.Finish(shapes);
                foreach (Shape key in _map.Keys.ToList())
                {
                    _map.GetValueRef(key) = _work.Final(_map.GetValue(key));
                }
            }
        }
        // OCCT_DEBUG trace left out ("Le shape comporte des faces invalides" when myDblE isn't empty)
        return _map.GetValue(s);
    }

    /// <summary>Returns the "original" shape (the caller's).</summary>
    public Shape Shape() => _shape;

    private void Put(in Shape s)
    {
        if (!_map.ContainsKey(s))
        {
            NCollection_List<Shape> thelist = new();
            _map[s] = thelist;
            if (s.Type != ShapeType.Vertex)
            {
                for (Iterator theIterator = new(s); theIterator.More(); theIterator.Next())
                {
                    Put(theIterator.Value());
                }
            }
            else
            {
                _map.GetValue(s).Append(s);
            }
        }
    }

    // Kerf: walks the explorer to a vertex. Past the wire's end its vertex stays null and this would never end (decision 22501)
    private static void MoveTo(WireExplorer explorer, in Vertex vertex)
    {
        while (!explorer.CurrentVertex().IsSame(vertex))
        {
            if (!explorer.More())
            {
                throw new ConstructionErrorException("LocOpe_SplitShape: a vertex isn't on the wire it's mapped to");
            }
            explorer.Next();
        }
    }

    private static void UpdateToleraces(in Face face,
                                        Dictionary<Shape, NCollection_List<Shape>> map)
    {
        Explorer expE = new(face, ShapeType.Edge);
        for (; expE.More(); expE.Next())
        {
            if (!map.ContainsKey(expE.Current()))
            {
                continue;
            }
            NCollection_List<Shape> lEdges = map.GetValue(expE.Current());
            if (lEdges.Extent() <= 1)
            {
                continue;
            }

            NCollection_List<Shape>.Iterator itrE = new(lEdges);
            EdgeAnalysis sae = new();

            for (; itrE.More(); itrE.Next())
            {
                Edge curE = ShapeCasts.Edge(itrE.Value());
                double amaxdev = 0.0;
                if (sae.CheckSameParameter(curE, face, ref amaxdev))
                {
                    BrepBuilder b = new();
                    b.UpdateEdge(curE, amaxdev);
                }
            }
        }
    }

    private bool Rebuild(in Shape s)
    {
        if (s.Type == ShapeType.Face)
        {
            UpdateToleraces(ShapeCasts.Face(s), _map);
        }
        NCollection_List<Shape>.Iterator itr = new(_map.GetValue(s));
        if (itr.More())
        {
            return !itr.Value().IsSame(s);
        }
        bool rebuild = false;
        Iterator it = new();
        for (it.Initialize(s); it.More(); it.Next())
        {
            rebuild = Rebuild(it.Value()) || rebuild;
        }

        if (rebuild)
        {
            BrepBuilder b = new();
            Shape result = s.EmptyCopied();
            Orientation orient;
            for (it.Initialize(s); it.More(); it.Next())
            {
                orient = it.Value().Orientation;
                for (itr.Initialize(_map.GetValue(it.Value())); itr.More(); itr.Next())
                {
                    b.Add(result, itr.Value().Oriented(orient));
                }
            }
            // Assign "Closed" flag for Wires and Shells only
            if (result.Type == ShapeType.Wire || result.Type == ShapeType.Shell)
            {
                result.Closed(Tool.IsClosed(result));
            }
            _map.GetValue(s).Append(result);
        }
        else
        {
            _map.GetValue(s).Append(s);
        }
        return rebuild;
    }

    private static bool SameUV(in Point2D p1, in Point2D p2, FaceSurfaceAdaptor bas)
    {
        bool isSame = true;
        if (bas.IsUPeriodic)
        {
            isSame = (Math.Abs(p1.X - p2.X) < bas.UPeriod() * 0.5);
        }
        if (bas.IsVPeriodic)
        {
            isSame = (isSame && (Math.Abs(p1.Y - p2.Y) < bas.VPeriod() * 0.5));
        }
        return isSame;
    }

    private static bool IsInside(in Face f, in Wire w1, in Wire w2)
    {
        // Attention, c`est tres boeuf !!!!
        BrepBuilder b = new();
        Shape localShape = f.EmptyCopied();
        Face newFace = ShapeCasts.Face(localShape);

        newFace.Orient(Forward);
        b.Add(newFace, w2);
        FClass2d classif = new(newFace, global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool reversed = false;
        if (classif.PerformInfinitePoint() == In)
        {
            // le wire donne defini un trou
            reversed = true;
        }

        Explorer exp = new(w1, ShapeType.Edge);
        Edge edg = ShapeCasts.Edge(exp.Current());
        Explorer exp2 = new(edg, ShapeType.Vertex);
        Vertex vtx = ShapeCasts.Vertex(exp2.Current());
        double prm = Tool.Parameter(vtx, edg);
        Curve2D? c2d = Tool.CurveOnSurface(edg, f, out _, out _);
        if (c2d is null)
        {
            // OCCT_DEBUG trace left out ("Edge is not on surface")
            return false;
        }
        Point2D pt2d = c2d.Value(prm);
        if (!reversed)
        {
            return (classif.Perform(pt2d) == In);
        }
        else
        {
            return (classif.Perform(pt2d) == Out);
        }
    }

    private static bool IsInside(in Face F, in Wire w)
    {
        // Attention, c`est tres boeuf !!!!
        Explorer exp = new(w, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            double prm;
            Curve2D c2d = Tool.CurveOnSurface(edg, F, out double f, out double l)!;
            if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
            {
                prm = (f + l) / 2.0;
            }
            else
            {
                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
                {
                    prm = 0.0;
                }
                else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
                {
                    prm = l - 1.0;
                }
                else
                {
                    prm = f + 1.0;
                }
            }

            Point2D pt2d = c2d.Value(prm);
            FClass2d classif = new(F, global::Kerf.Engine.Foundation.Precision.PConfusion());
            Containment stat = classif.Perform(pt2d);
            if (stat == Out)
            {
                return false;
            }

            if (stat == On)
            {
                int nbPnt = 10;
                int nbOut = 0, nbIn = 0, nbOn = 0;
                int j = 1;
                for (; j <= nbPnt; j++)
                {
                    // check neighbouring point
                    prm = f + (l - f) / nbPnt * (j - 1);
                    pt2d = c2d.Value(prm);
                    stat = classif.Perform(pt2d);
                    if (stat == Out)
                    {
                        nbOut++;
                    }
                    else if (stat == In)
                    {
                        nbIn++;
                    }
                    else
                    {
                        nbOn++;
                    }
                }
                if (nbOut > nbIn + nbOn)
                {
                    return false;
                }
            }
        }
        return true;
    }

    private static void GetDirection(in Edge edge, in Face face, out Point2D pnt, out Vector2D dir,
                                     bool isFirstEnd)
    {
        Curve2D c2d = Tool.CurveOnSurface(edge, face, out double first, out double last)!;

        Orientation or = edge.Orientation;
        bool takeFirst = ((or == Forward && isFirstEnd) || (or == Reversed && !isFirstEnd));

        double dpar = (last - first) * 0.01;
        if (takeFirst)
        {
            c2d.D0(first, out pnt);
            Point2D nextPnt = c2d.Value(first + dpar);
            dir = new Vector2D(pnt, nextPnt);
        }

        else
        {
            c2d.D0(last, out pnt);
            Point2D prevPnt = c2d.Value(last - dpar);
            dir = new Vector2D(prevPnt, pnt);
        }
        if (or == Reversed)
        {
            dir = dir.Reversed();
        }
    }

    private static bool ChoixUV(in Edge last,
                                in Face f,
                                IndexedSet<Shape> poss,
                                ref Edge resEdge,
                                ref Point2D plst,
                                ref Vector2D dlst)
    {
        FaceSurfaceAdaptor surf = new(f, false); // no restriction
        surf.D0(plst.X, plst.Y, out Point3D aPlst);

        Direction2D ref2d = new(dlst);

        int index = 0, imin = 0;
        double angmax = -Math.PI, ang;

        for (index = 1; index <= poss.Count; index++)
        {
            Edge edge = ShapeCasts.Edge(poss[index - 1]);

            GetDirection(edge, f, out Point2D p2d, out Vector2D v2d, true);
            if (!SameUV(plst, p2d, surf))
            {
                continue;
            }

            surf.D0(p2d.X, p2d.Y, out Point3D pCur);

            if (!last.IsSame(edge))
            {
                ang = ref2d.AngleTo(new Direction2D(v2d));
            }
            else
            {
                ang = -Math.PI;
            }

            if ((ang > angmax))
            {
                imin = index;
                angmax = ang;
            }
        }

        if (imin != 0)
        {
            resEdge = ShapeCasts.Edge(poss[imin - 1]);
            GetDirection(resEdge, f, out plst, out dlst, false);
        }

        _ = aPlst;
        return imin != 0;
    }

    private static Shape ChooseDirection(in Shape refDir,
                                                in Vertex refVertex,
                                                in Face face,
                                                NCollection_List<Shape> ldirs)
    {
        Explorer explo = new(refDir, ShapeType.Edge);
        Edge refEdge = new();
        Vertex v1, v2;
        Orientation or = Forward;
        for (; explo.More(); explo.Next())
        {
            refEdge = ShapeCasts.Edge(explo.Current());
            Exploration.Vertices(refEdge, out v1, out v2);
            if (v1.IsSame(refVertex))
            {
                or = Reversed;
                break;
            }
            else if (v2.IsSame(refVertex))
            {
                or = Forward;
                break;
            }
        }

        Curve2D refCurve = Tool.CurveOnSurface(refEdge, face, out double refFirst, out double refLast)!;

        double refPar = (or == Forward) ? refLast : refFirst;
        refCurve.D1(refPar, out Point2D refPnt, out Vector2D refVec);
        if (or == Forward)
        {
            refVec = refVec.Reversed();
        }

        double first, last, par;
        double minAngle = RealLast(), angle;
        Shape targetDir = new();
        NCollection_List<Shape>.Iterator itl = new(ldirs);
        for (; itl.More(); itl.Next())
        {
            Shape shape = itl.Value();
            Edge edge = new();
            for (explo.Init(shape, ShapeType.Edge); explo.More(); explo.Next())
            {
                edge = ShapeCasts.Edge(explo.Current());
                Exploration.Vertices(edge, out v1, out v2);
                if (v1.IsSame(refVertex))
                {
                    or = Forward;
                    break;
                }
                else if (v2.IsSame(refVertex))
                {
                    or = Reversed;
                    break;
                }
            }
            Curve2D curve = Tool.CurveOnSurface(edge, face, out first, out last)!;
            par = (or == Forward) ? first : last;
            curve.D1(par, out refPnt, out Vector2D vec);
            if (or == Reversed)
            {
                vec = vec.Reversed();
            }
            angle = vec.AngleTo(refVec);
            if (angle < 0.0)
            {
                angle += 2.0 * Math.PI;
            }

            if (angle < minAngle)
            {
                minAngle = angle;
                targetDir = shape;
            }
        }

        return targetDir;
    }
}
