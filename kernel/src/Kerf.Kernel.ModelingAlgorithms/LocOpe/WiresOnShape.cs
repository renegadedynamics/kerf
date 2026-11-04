// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_WiresOnShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Construct;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// Edges and wires lying on a shape, bound to the faces (or edges) they lie on, and how their vertices lie on the
/// shape's vertices and edges: the input of <see cref="Spliter"/>.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, binding the edges (<see cref="BindAll"/>, which the edge iterator runs) writes into them and into the
/// shape: pcurves and same-parameter on the edges, raised vertex tolerances, and the edges crossing others are split.
/// So each public call writes in the working overlay of the splitting (decision 1013, unit 10H5:
/// <see cref="WorkingShapes"/>): <see cref="Spliter"/> goes on in the same overlay, and ends it. A WiresOnShape serves one
/// splitting.
/// </para>
/// <para>
/// The public accessors answer with the caller's shapes: an edge, face or vertex the operation copied is given back as
/// the caller's own; the edges it made (splits of a given edge) are its working shapes, frozen with the spliter's
/// result. Ported code reads the <c>Internal</c> forms, which give the working shapes.
/// </para>
/// </remarks>
internal sealed class WiresOnShape : Transient
{
    private Shape _shape = new();
    private readonly IndexedMap<Shape, Shape> _mapEF = new(ShapeHasher.Instance);
    private readonly HashSet<Shape> _facesWithSection = new(ShapeHasher.Instance);
    private bool _checkInterior;
    // OCCT iterates this map keyed by the shapes' addresses (in BindAll); the port in insertion order
    private readonly Dictionary<Shape, Shape> _map = new(ShapeHasher.Instance);
    private bool _done;
    private int _index;
    private WorkingShapes _work;

    public WiresOnShape(in Shape s)
    {
        _work = new WorkingShapes();
        _shape = s;
        _checkInterior = true;
        _done = false;
        _index = -1;
    }

    /// <summary>For ported operations that own the working shapes (LocOpe_Gluer): <paramref name="s"/> is one.</summary>
    internal WiresOnShape(in Shape s, WorkingShapes work)
    {
        _work = work;
        _shape = s;
        _checkInterior = true;
        _done = false;
        _index = -1;
    }

    /// <summary>The working shapes of the splitting (shared with the spliter).</summary>
    internal WorkingShapes WorkingShapes() => _work;

    /// <summary>The shape worked on.</summary>
    internal Shape InternalShape() => _shape;

    public void Init(in Shape s)
    {
        _work = new WorkingShapes();
        _shape = s;
        _checkInterior = true;
        _done = false;
        _map.Clear();
        _mapEF.Clear();
    }

    /// <summary>
    /// Adds splitting edges or wires for the whole initial shape, without saying on which faces or edges they lie: each
    /// edge is bound to the faces of the shape it lies in.
    /// </summary>
    public bool Add(NCollection_Sequence<Shape> edges)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        NCollection_Sequence<Shape> anEdges = new();
        int i = 1, nb = edges.Length();
        NCollection_Array1<Box> edgeBoxes = new(1, nb);
        for (; i <= nb; i++)
        {
            Shape curSplit = edges[i];
            Explorer expE = new(curSplit, global::Kerf.Topology.ShapeType.Edge);
            for (; expE.More(); expE.Next())
            {
                Shape curE = expE.Current();

                Box boxE = new();
                Bounds.AddClose(curE, ref boxE);
                if (boxE.IsVoid)
                {
                    continue;
                }
                double tolE = Tool.Tolerance(ShapeCasts.Edge(curE));
                boxE.SetGap(tolE);
                // OCCT: the box goes at the argument's index, the edge at the next free one; a wire of several edges
                // leaves them out of step (occt-issues.md)
                edgeBoxes.SetValue(i, boxE);
                anEdges.Append(curE);
            }
        }
        Explorer expFaces = new(_shape, ShapeType.Face);
        int numF = 1;
        NCollection_PackedMapOfInteger usedEdges = new();
        for (; expFaces.More(); expFaces.Next(), numF++)
        {
            Face curF = ShapeCasts.Face(expFaces.Current());
            Box boxF = new();
            Bounds.Add(curF, ref boxF);
            if (boxF.IsVoid)
            {
                continue;
            }
            FaceSurfaceAdaptor adF = new(curF, false);
            FClass2d? checkStateTool = null;

            // Initialize extrema projector once per face
            ExtPS extr = new();
            extr.Initialize(adF,
                              adF.FirstUParameter(),
                              adF.LastUParameter(),
                              adF.FirstVParameter(),
                              adF.LastVParameter(),
                              global::Kerf.Engine.Foundation.Precision.Confusion(),
                              global::Kerf.Engine.Foundation.Precision.Confusion());
            extr.SetFlag(ExtFlag_MIN);

            i = 1;
            nb = edgeBoxes.Length();
            for (; i <= nb; i++)
            {
                if (usedEdges.Contains(i))
                {
                    continue;
                }

                if (boxF.IsOut(edgeBoxes[i]))
                {
                    continue;
                }

                Edge curE = ShapeCasts.Edge(anEdges[i]);

                Curve3D? c = Tool.Curve(curE, out double f, out double l);
                if (c is null)
                {
                    usedEdges.Add(i);
                    continue;
                }
                Point3D p = c.Value((f + l) * 0.5);
                extr.Perform(p);

                if (!extr.IsDone || extr.ExtCount == 0)
                {
                    continue;
                }
                double tolE = Tool.Tolerance(ShapeCasts.Edge(curE));
                double tol2 = (tolE + global::Kerf.Engine.Foundation.Precision.Confusion()) * (tolE + global::Kerf.Engine.Foundation.Precision.Confusion());
                int n = 1;
                for (; n <= extr.ExtCount; n++)
                {
                    double dist2 = extr.SquareDistance(n);
                    if (dist2 > tol2)
                    {
                        continue;
                    }
                    POnSurf ps = extr.Point(n);
                    ps.Parameter(out double u, out double v);

                    checkStateTool ??= new FClass2d(curF, global::Kerf.Engine.Foundation.Precision.PConfusion());
                    if (checkStateTool.Perform(new Point2D(u, v)) == In)
                    {
                        BindCore(curE, curF);
                        usedEdges.Add(i);
                    }
                }
            }
        }
        _ = numF;
        return !usedEdges.IsEmpty();
    }

    /// <summary>Sets the flag of check of internal intersections (true by default).</summary>
    public void SetCheckInterior(bool toCheckInterior)
    {
        _checkInterior = toCheckInterior;
    }

    public void Bind(in Wire w, in Face f)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        Shape aW = w;
        Face aF = ShapeCasts.Face(f);
        for (Explorer exp = new(aW, global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
        {
            BindCore(ShapeCasts.Edge(exp.Current()), aF);
        }
    }

    public void Bind(in Compound comp, in Face f)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        Shape aComp = comp;
        Face aF = ShapeCasts.Face(f);
        for (Explorer exp = new(aComp, global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
        {
            BindCore(ShapeCasts.Edge(exp.Current()), aF);
        }
        _facesWithSection.Add(aF);
    }

    public void Bind(in Edge e, in Face f)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        BindCore(ShapeCasts.Edge(e), ShapeCasts.Face(f));
    }

    // Bind(E, F) on working shapes
    private void BindCore(in Edge e, in Face f)
    {
        if (!_mapEF.ContainsKey(e))
        {
            Explorer exp = new(f, global::Kerf.Topology.ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(e))
                {
                    break;
                }
            }
            if (!exp.More())
            {
                _mapEF.Add(e, f);
            }
        }
        else
        {
            throw new ConstructionErrorException();
        }
    }

    public void Bind(in Edge efromW, in Edge eonFace)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        _map[efromW] = eonFace;
    }

    public void BindAll()
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        if (_done)
        {
            return;
        }
        HashSet<Shape> theMap = new(ShapeHasher.Instance);

        // Detection des vertex a projeter ou a "binder" avec des vertex existants
        Dictionary<Shape, Shape> mapV = new(ShapeHasher.Instance);
        Explorer exp = new(), exp2 = new();
        foreach (KeyValuePair<Shape, Shape> ite in _map)
        {
            Edge eref = ShapeCasts.Edge(ite.Key);
            Edge eimg = ShapeCasts.Edge(ite.Value);

            PutPCurves(eref, eimg, _shape);

            for (exp.Init(eref, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (!theMap.Contains(vtx))
                { // pas deja traite
                    for (exp2.Init(eimg, ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        Vertex vtx2 = ShapeCasts.Vertex(exp2.Current());
                        if (vtx2.IsSame(vtx))
                        {
                            break;
                        }
                        else if (BrepTools.Compare(vtx, vtx2))
                        {
                            mapV[vtx] = vtx2;
                            break;
                        }
                    }
                    if (!exp2.More())
                    {
                        mapV[vtx] = eimg;
                    }
                    theMap.Add(vtx);
                }
            }
        }

        foreach (KeyValuePair<Shape, Shape> ite in mapV)
        {
            _map[ite.Key] = ite.Value;
        }

        IndexedMap<Shape, NCollection_List<Shape>> splits = new(ShapeHasher.Instance);
        int ind;
        HashSet<Shape> overlappedEdges = new(ShapeHasher.Instance);
        for (ind = 1; ind <= _mapEF.Count; ind++)
        {
            Edge edg = ShapeCasts.Edge(_mapEF.KeyAt(ind - 1));
            Face fac = ShapeCasts.Face(_mapEF.ValueAt(ind - 1));

            PutPCurve(edg, fac);
            Curve2D? pCurve = Tool.CurveOnSurface(edg, fac, out _, out _);
            if (pCurve is null)
            {
                continue;
            }

            if (_checkInterior)
            {
                bool isOverlapped = false;
                FindInternalIntersections(edg, fac, splits, ref isOverlapped);
                if (isOverlapped)
                {
                    overlappedEdges.Add(edg);
                }
            }
        }

        for (ind = 1; ind <= splits.Count; ind++)
        {
            Shape edge = splits.KeyAt(ind - 1);
            if (overlappedEdges.Contains(edge))
            {
                continue;
            }
            Shape face = _mapEF.GetValue(edge);
            // Remove "anEdge" from "myMapEF"
            _mapEF.Remove(edge);
            NCollection_List<Shape>.Iterator itl = new(splits.ValueAt(ind - 1));
            for (; itl.More(); itl.Next())
            {
                _mapEF.Add(itl.Value(), face);
            }
        }

        Dictionary<Shape, double> vertParam = new(ShapeHasher.Instance);

        for (ind = 1; ind <= _mapEF.Count; ind++)
        {
            Edge edg = ShapeCasts.Edge(_mapEF.KeyAt(ind - 1));
            Face fac = ShapeCasts.Face(_mapEF.ValueAt(ind - 1));
            // JAG 02.02.96 : On verifie les pcurves...

            for (exp.Init(edg, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (theMap.Contains(vtx))
                {
                    continue;
                }

                double vtx_param = Tool.Parameter(vtx, edg);
                EdgeCurve2DAdaptor bAcurve2d = new(edg, fac);

                Point2D p2d = bAcurve2d.Curve() is not null
                                   ? bAcurve2d.Value(vtx_param)
                                   : new Point2D(global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());

                Edge epro = new();
                double prm = global::Kerf.Engine.Foundation.Precision.Infinite();
                bool isProjected = _map.ContainsKey(vtx);

                // if vertex was already projected on the current edge on the previous face
                // it is necessary to check tolerance of the vertex in the 2D space on the current
                // face without projection and update tolerance of vertex if it is necessary
                if (isProjected)
                {
                    Shape sh = _map.GetValue(vtx);
                    if (sh.Type != global::Kerf.Topology.ShapeType.Edge)
                    {
                        continue;
                    }
                    epro = ShapeCasts.Edge(_map.GetValue(vtx));
                    if (vertParam.ContainsKey(vtx))
                    {
                        prm = vertParam.GetValue(vtx);
                    }
                }
                bool ok = Project(vtx, p2d, fac, ref epro, ref prm);
                if (ok && !isProjected)
                {
                    for (exp2.Init(epro, ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        Vertex vtx2 = ShapeCasts.Vertex(exp2.Current());
                        if (vtx2.IsSame(vtx))
                        {
                            _map[vtx] = vtx2;
                            theMap.Add(vtx);
                            break;
                        }
                        else if (BrepTools.Compare(vtx, vtx2))
                        {
                            Tool.Range(epro, fac, out double f1, out double l1);
                            if (!Tool.Degenerated(epro)
                                && (Math.Abs(prm - f1) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                                    || Math.Abs(prm - l1) <= global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            {
                                _map[vtx] = vtx2;
                                theMap.Add(vtx);
                                break;
                            }
                        }
                    }
                    if (!exp2.More())
                    {
                        _map[vtx] = epro;
                        vertParam[vtx] = prm;
                    }
                }
            }
        }

        //  Modified by Sergey KHROMOV - Mon Feb 12 16:26:50 2001 Begin
        foreach (KeyValuePair<Shape, Shape> ite in _map)
        {
            if (ite.Key.Type == global::Kerf.Topology.ShapeType.Edge)
            {
                _mapEF.Add(ite.Key, ite.Value);
            }
        }
        //  Modified by Sergey KHROMOV - Mon Feb 12 16:26:52 2001 End

        _done = true;
    }

    public bool IsDone => _done;

    public void InitEdgeIterator()
    {
        BindAll();
        _index = 1;
    }

    public bool MoreEdge() => _index <= _mapEF.Count;

    /// <summary>The current edge (the caller's edge if it was copied).</summary>
    public Edge Edge() => ShapeCasts.Edge(InternalEdge());

    /// <summary><see cref="Edge"/> as worked on, for ported code.</summary>
    internal Edge InternalEdge() => ShapeCasts.Edge(_mapEF.KeyAt(_index - 1));

    /// <summary>Returns the face of the shape on which the current edge is projected (the caller's face).</summary>
    public Face OnFace() => ShapeCasts.Face(InternalOnFace());

    /// <summary><see cref="OnFace"/> as worked on, for ported code.</summary>
    internal Face InternalOnFace() => ShapeCasts.Face(_mapEF.ValueAt(_index - 1));

    /// <summary>
    /// If the current edge is projected on an edge, returns true and sets the edge (the caller's edge) in
    /// <paramref name="e"/>. Otherwise, returns false.
    /// </summary>
    public bool OnEdge(out Edge e)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        bool isOn = InternalOnEdge(out e);
        return isOn;
    }

    /// <summary><see cref="OnEdge(out Edge)"/> as worked on, for ported code.</summary>
    internal bool InternalOnEdge(out Edge e)
    {
        e = new Edge();
        if (_map.ContainsKey(_mapEF.KeyAt(_index - 1)))
        {
            e = ShapeCasts.Edge(_map.GetValue(_mapEF.KeyAt(_index - 1)));
            return true;
        }
        return false;
    }

    public void NextEdge()
    {
        _index++;
    }

    /// <summary>
    /// If the vertex <paramref name="vw"/> of a wire lies on a vertex of the shape, returns true and sets that vertex
    /// (the caller's) in <paramref name="vs"/>.
    /// </summary>
    public bool OnVertex(in Vertex vw, out Vertex vs)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        bool isOn = InternalOnVertex(ShapeCasts.Vertex(vw), out vs);
        return isOn;
    }

    /// <summary><see cref="OnVertex"/> on working shapes, for ported code.</summary>
    internal bool InternalOnVertex(in Vertex vw, out Vertex vs)
    {
        vs = new Vertex();
        if (_map.ContainsKey(vw))
        {
            if (_map.GetValue(vw).Type == ShapeType.Vertex)
            {
                vs = ShapeCasts.Vertex(_map.GetValue(vw));
                return true;
            }
            return false;
        }
        return false;
    }

    /// <summary>
    /// If the vertex <paramref name="v"/> lies on an edge of the original shape, returns true and sets the concerned
    /// edge (the caller's) in <paramref name="ed"/>, and the parameter on the edge in <paramref name="prm"/>. Else
    /// returns false.
    /// </summary>
    public bool OnEdge(in Vertex v, out Edge ed, out double prm)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        bool isOn = InternalOnEdge(ShapeCasts.Vertex(v), out ed, out prm);
        return isOn;
    }

    /// <summary><see cref="OnEdge(in Vertex, out Edge, out double)"/> on working shapes.</summary>
    internal bool InternalOnEdge(in Vertex v, out Edge ed, out double prm)
    {
        ed = new Edge();
        prm = 0.0; // OCCT: unchanged
        if (!_map.ContainsKey(v) || _map.GetValue(v).Type == ShapeType.Vertex)
        {
            return false;
        }

        ed = ShapeCasts.Edge(_map.GetValue(v));
        prm = Project(v, ed);
        return true;
    }

    /// <summary>
    /// <see cref="OnEdge(in Vertex, out Edge, out double)"/> for the vertex <paramref name="v"/> of the
    /// edge <paramref name="edgeFrom"/>, projected through its pcurve where the shape's edge has no 3D curve.
    /// </summary>
    public bool OnEdge(in Vertex v, in Edge edgeFrom, out Edge ed, out double prm)
    {
        using WorkingOverlay.Scope scope = _work.Enter(); // Kerf: the working overlay (unit 10H5)
        bool isOn = InternalOnEdge(ShapeCasts.Vertex(v), ShapeCasts.Edge(edgeFrom), out ed, out prm);
        return isOn;
    }

    /// <summary><see cref="OnEdge(in Vertex, in Edge, out Edge, out double)"/> on working shapes.</summary>
    internal bool InternalOnEdge(in Vertex v, in Edge edgeFrom, out Edge ed, out double prm)
    {
        ed = new Edge();
        prm = 0.0; // OCCT: unchanged
        if (!_map.ContainsKey(v) || _map.GetValue(v).Type == ShapeType.Vertex)
        {
            return false;
        }

        ed = ShapeCasts.Edge(_map.GetValue(v));
        if (!_mapEF.ContainsKey(edgeFrom))
        {
            return false;
        }

        Shape shape = _mapEF.GetValue(edgeFrom);
        Curve3D? c = Tool.Curve(ed, out _, out _);
        if (c is null && shape.Type == ShapeType.Face)
        {
            Face face = ShapeCasts.Face(shape);
            double vtx_param = Tool.Parameter(v, edgeFrom);
            EdgeCurve2DAdaptor bAcurve2d = new(edgeFrom, face);
            Point2D p2d = bAcurve2d.Value(vtx_param);

            prm = Project(v, p2d, ed, face);
        }
        else
        {
            prm = Project(v, ShapeCasts.Edge(ed));
        }

        return true;
    }

    /// <summary>Tells if the face is to be split by a section (a compound bound to it).</summary>
    public bool IsFaceWithSection(in Shape aFace) => InternalIsFaceWithSection(aFace);

    /// <summary><see cref="IsFaceWithSection"/> on working shapes.</summary>
    internal bool InternalIsFaceWithSection(in Shape aFace) => _facesWithSection.Contains(aFace);

    //=================================================================================================

    private static bool Project(in Vertex v, in Point2D p2d, in Face F, ref Edge edge, ref double param)
    {
        double tolV = Tool.Tolerance(v);
        double dmin = (edge.IsNull ? RealLast() : tolV * tolV);

        bool valret = false;

        Surface surf = Tool.Surface(F)!;

        if (edge.IsNull)
        {
            Point3D toproj = Tool.Pnt(v);
            for (Explorer exp = new(F.Oriented(Forward), global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                Curve3D? c = Tool.Curve(edg, out double f, out double l);
                double curDist = global::Kerf.Engine.Foundation.Precision.Infinite();
                double curPar = global::Kerf.Engine.Foundation.Precision.Infinite();
                if (c is not null)
                {
                    curPar = Project(v, edg);
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(curPar))
                    {
                        continue;
                    }
                    c.D0(curPar, out Point3D curPBound);
                    curDist = curPBound.SquareDistanceTo(toproj);
                }

                else if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(p2d.X))
                {
                    Curve2D? c2d = Tool.CurveOnSurface(edg, F, out f, out l);
                    if (c2d is null)
                    {
                        continue;
                    }

                    curPar = Project(v, p2d, edg, F);
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(curPar))
                    {
                        continue;
                    }
                    Curve2D pc = Tool.CurveOnSurface(edg, F, out f, out l)!;
                    pc.D0(curPar, out Point2D pProj);
                    surf.D0(pProj.X, pProj.Y, out Point3D curPBound);
                    curDist = curPBound.SquareDistanceTo(toproj);
                }

                if (curDist < dmin)
                {
                    edge = edg;
                    edge.Orient(edg.Orientation);
                    dmin = curDist;
                    param = curPar;
                }
            }
            if (edge.IsNull)
            {
                return false;
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsInfinite(param))
        {
            Curve3D? c = Tool.Curve(edge, out _, out _);
            param = (c is not null ? Project(v, edge) : Project(v, p2d, edge, F));
        }

        double ttol = tolV + Tool.Tolerance(edge);
        if (dmin <= ttol * ttol)
        {
            valret = true;
            SurfaceAdaptor adSurf = new(surf);

            double uResolution = adSurf.UResolution(1.0);
            double vResolution = adSurf.UResolution(1.0); // OCCT: UResolution for V too (occt-issues.md)

            Curve2D? crvBound = Tool.CurveOnSurface(edge, F, out _, out _);
            if (crvBound is not null)
            {
                crvBound.D0(param, out Point2D pBound2d);

                // distance in 2D space recomputed in the 3D space in order to tolerance of vertex
                // cover gap in 2D space. For consistency with check of the validity in the  BRepCheck_Wire
                double dumax = 0.01 * (adSurf.LastUParameter() - adSurf.FirstUParameter());
                double dvmax = 0.01 * (adSurf.LastVParameter() - adSurf.FirstVParameter());

                Point2D pcur = p2d;
                double dumin = Math.Abs(pcur.X - pBound2d.X);
                double dvmin = Math.Abs(pcur.Y - pBound2d.Y);
                if (dumin > dumax && adSurf.IsUPeriodic)
                {
                    double x1 = pBound2d.X;
                    double shift = ShapeAnalysisAnalysis.AdjustToPeriod(x1, adSurf.FirstUParameter(), adSurf.LastUParameter());
                    x1 += shift;
                    pBound2d = pBound2d.WithX(x1);
                    double x2 = p2d.X;
                    shift = ShapeAnalysisAnalysis.AdjustToPeriod(x2, adSurf.FirstUParameter(), adSurf.LastUParameter());
                    x2 += shift;
                    dumin = Math.Abs(x2 - x1);
                    if (dumin > dumax && (Math.Abs(dumin - adSurf.UPeriod()) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    {
                        x2 = x1;
                        dumin = 0.0;
                    }
                    pcur = pcur.WithX(x2);
                }

                if (dvmin > dvmax && adSurf.IsVPeriodic)
                {
                    double y1 = pBound2d.Y;
                    double shift = ShapeAnalysisAnalysis.AdjustToPeriod(y1, adSurf.FirstVParameter(), adSurf.LastVParameter());
                    y1 += shift;
                    pBound2d = pBound2d.WithY(y1);
                    double y2 = p2d.Y;
                    shift = ShapeAnalysisAnalysis.AdjustToPeriod(y2, adSurf.FirstVParameter(), adSurf.LastVParameter());
                    y2 += shift;
                    dvmin = Math.Abs(y1 - y2);
                    if (dvmin > dvmax && (Math.Abs(dvmin - adSurf.VPeriod()) < global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        y2 = y1;
                        dvmin = 0.0;
                    }
                    pcur = pcur.WithY(y2);
                }
                double dist3d = tolV;
                if ((dumin > dumax) || (dvmin > dvmax))
                {
                    dumax = adSurf.UResolution(tolV);
                    dvmax = adSurf.VResolution(tolV);
                    double tol2d = 2.0 * Max(dumax, dvmax);
                    double dist2d = Max(dumin, dvmin);

                    if (dist2d > tol2d)
                    {
                        double dist3d1 = dist2d / Max(uResolution, vResolution);
                        if (dist3d1 > dist3d)
                        {
                            dist3d = dist3d1;
                        }
                    }
                }

                // added check by 3D the same as in the BRepCheck_Wire::SelfIntersect
                surf.D0(pBound2d.X, pBound2d.Y, out Point3D pBound);
                surf.D0(p2d.X, p2d.Y, out Point3D pv2d);
                double distPoints_3D = pv2d.SquareDistanceTo(pBound);
                double maxDist = Max(distPoints_3D, dist3d * dist3d);

                BrepBuilder b = new();
                if (tolV * tolV < maxDist)
                {
                    double newTol = Math.Sqrt(maxDist);
                    b.UpdateVertex(v, newTol);
                }
            }
        }
        // OCCT_DEBUG_MESH trace left out
        return valret;
    }

    //=================================================================================================

    private static double Project(in Vertex v, in Edge edge)
    {
        Point3D toproj = Tool.Pnt(v);
        ProjectPointOnCurve proj = new();

        Curve3D c = Tool.Curve(edge, out Placement loc, out double f, out double l)!;
        if (!loc.IsIdentity)
        {
            Geometry3D gg = c.Transformed(loc.Transformation);
            c = (Curve3D)gg;
        }
        proj.Init(toproj, c, f, l);

        return (proj.NbPoints() > 0 ? proj.LowerDistanceParameter() : global::Kerf.Engine.Foundation.Precision.Infinite());
    }

    //=================================================================================================

    private static double Project(in Vertex v, in Point2D p2d, in Edge edge, in Face face)
    {
        _ = v;
        ProjectPointOnCurve2D proj = new();

        Curve2D pc = Tool.CurveOnSurface(edge, face, out double f, out double l)!;

        proj.Init(p2d, pc, f, l);

        return proj.NbPoints() > 0 ? proj.LowerDistanceParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    //=================================================================================================

    // OCCT.NET: OCCT translates the projected pcurve in place; a frozen one (shared) is translated on a copy
    private static Curve2D Translatable(Curve2D curve) =>
        curve.IsFrozen ? (Curve2D)curve.Copy() : curve;

    private static void PutPCurve(in Edge edg, in Face fac)
    {
        BrepBuilder b = new();

        Surface s = Tool.Surface(fac, out Placement locFac)!;
        _ = locFac;

        if (s is TrimmedSurface trimmed)
        {
            s = trimmed.BasisSurface;
        }

        if (s.GetType() == typeof(PlaneSurface))
        {
            return;
        }

        BrepTools.UVBounds(fac, out double umin, out double umax, out double vmin, out double vmax);

        double f, l;

        Curve2D? aC2d = Tool.CurveOnSurface(edg, fac, out f, out l);
        if (aC2d is not null)
        {
            aC2d.D0((f + l) * 0.5, out Point2D p2d);
            bool isIn = true;
            if ((p2d.X < umin - global::Kerf.Engine.Foundation.Precision.PConfusion()) || (p2d.X > umax + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                isIn = false;
            }
            if ((p2d.Y < vmin - global::Kerf.Engine.Foundation.Precision.PConfusion()) || (p2d.Y > vmax + global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                isIn = false;
            }

            if (isIn)
            {
                return;
            }
        }

        Curve3D c = Tool.Curve(edg, out Placement loc, out f, out l)!;
        if (!loc.IsIdentity)
        {
            Geometry3D gg = c.Transformed(loc.Transformation);
            c = (Curve3D)gg;
        }

        if (c.GetType() != typeof(TrimmedCurve3D))
        {
            c = new TrimmedCurve3D(c, f, l);
        }

        s = Tool.Surface(fac)!;

        double tolFirst = -1, tolLast = -1;
        Exploration.Vertices(edg, out Vertex v1, out Vertex v2);
        if (!v1.IsNull)
        {
            tolFirst = Tool.Tolerance(v1);
        }
        if (!v2.IsNull)
        {
            tolLast = Tool.Tolerance(v2);
        }

        double tol2d = global::Kerf.Engine.Foundation.Precision.Confusion();
        ProjectCurveOnSurface toolProj = new();
        toolProj.Init(s, tol2d);

        toolProj.InternalPerform(c, f, l, out Curve2D? c2d, tolFirst, tolLast);
        if (c2d is null)
        {
            return;
        }

        Point2D pf = c2d.Value(f);
        Point2D pl = c2d.Value(l);
        s.D0(pf.X, pf.Y, out Point3D PF);
        s.D0(pl.X, pl.Y, out Point3D PL);
        if (edg.Orientation == Reversed)
        {
            v1 = Exploration.LastVertex(edg);
            v1.Reverse();
        }
        else
        {
            v1 = Exploration.FirstVertex(edg);
        }
        if (edg.Orientation == Reversed)
        {
            v2 = Exploration.FirstVertex(edg);
            v2.Reverse();
        }
        else
        {
            v2 = Exploration.LastVertex(edg);
        }

        if (!v1.IsNull && v2.IsNull)
        {
            // Handling of internal vertices
            // OCCT: V2 is null here, so BRep_Tool::Tolerance(V2) dereferences a null shape (occt-issues.md)
            double old1 = Tool.Tolerance(v1);
            double old2 = Tool.Tolerance(v2);
            Point3D pnt1 = Tool.Pnt(v1);
            Point3D pnt2 = Tool.Pnt(v2);
            double tol1 = pnt1.DistanceTo(PF);
            double tol2 = pnt2.DistanceTo(PL);
            b.UpdateVertex(v1, Max(old1, tol1));
            b.UpdateVertex(v2, Max(old2, tol2));
        }

        if (s.IsUPeriodic)
        {
            double up = s.UPeriod;
            double tolu = global::Kerf.Engine.Foundation.Precision.PConfusion(); // Epsilon(up);
            int nbtra = 0;
            double theUmin = Min(pf.X, pl.X);
            double theUmax = Max(pf.X, pl.X);

            if (theUmin < umin - tolu)
            {
                while (theUmin < umin - tolu)
                {
                    theUmin += up;
                    nbtra++;
                }
            }
            else if (theUmax > umax + tolu)
            {
                while (theUmax > umax + tolu)
                {
                    theUmax -= up;
                    nbtra--;
                }
            }

            if (nbtra != 0)
            {
                c2d = Translatable(c2d);
                c2d.Translate(new Vector2D(nbtra * up, 0.0));
            }
        }

        if (s.IsVPeriodic)
        {
            double vp = s.VPeriod;
            double tolv = global::Kerf.Engine.Foundation.Precision.PConfusion(); // Epsilon(vp);
            int nbtra = 0;
            double theVmin = Min(pf.Y, pl.Y);
            double theVmax = Max(pf.Y, pl.Y);

            if (theVmin < vmin - tolv)
            {
                while (theVmin < vmin - tolv)
                {
                    theVmin += vp;
                    theVmax += vp;
                    nbtra++;
                }
            }
            else if (theVmax > vmax + tolv)
            {
                while (theVmax > vmax + tolv)
                {
                    theVmax -= vp;
                    theVmin -= vp;
                    nbtra--;
                }
            }

            if (nbtra != 0)
            {
                c2d = Translatable(c2d);
                c2d.Translate(new Vector2D(0.0, nbtra * vp));
            }
        }
        b.UpdateEdge(edg, c2d, fac, Tool.Tolerance(edg));

        b.SameParameter(edg, false);
        BrepLibrary.SameParameter(edg, tol2d);
    }

    //=================================================================================================

    private static void PutPCurves(in Edge efrom, in Edge eto, in Shape myShape)
    {
        NCollection_List<Shape> lfaces = new();
        Explorer exp = new(), exp2 = new();

        for (exp.Init(myShape, ShapeType.Face); exp.More(); exp.Next())
        {
            for (exp2.Init(exp.Current(), global::Kerf.Topology.ShapeType.Edge); exp2.More(); exp2.Next())
            {
                if (exp2.Current().IsSame(eto))
                {
                    lfaces.Append(exp.Current());
                }
            }
        }

        if (lfaces.Extent() != 1 && lfaces.Extent() != 2)
        {
            throw new ConstructionErrorException();
        }

        // soit bord libre, soit connexite entre 2 faces, eventuellement edge closed

        if (lfaces.Extent() == 1)
        {
            return; // sera fait par PutPCurve.... on l`espere
        }

        BrepBuilder b = new();
        Surface s;
        Curve3D c;
        double umin, umax, vmin, vmax;
        double f, l;
        Placement loc;

        if (!lfaces.First().IsSame(lfaces.Last()))
        {
            NCollection_List<Shape>.Iterator itl = new(lfaces);
            for (; itl.More(); itl.Next())
            {
                Face fac = ShapeCasts.Face(itl.Value());

                if (Tool.CurveOnSurface(efrom, fac, out f, out l) is not null)
                {
                    continue;
                }
                s = Tool.Surface(fac, out _)!;
                if (s is TrimmedSurface trimmed)
                {
                    s = trimmed.BasisSurface;
                }
                if (s.GetType() == typeof(PlaneSurface))
                {
                    continue;
                }

                BrepTools.UVBounds(fac, out umin, out umax, out vmin, out vmax);
                c = Tool.Curve(efrom, out loc, out f, out l)!;
                if (!loc.IsIdentity)
                {
                    Geometry3D gg = c.Transformed(loc.Transformation);
                    c = (Curve3D)gg;
                }

                if (c.GetType() != typeof(TrimmedCurve3D))
                {
                    c = new TrimmedCurve3D(c, f, l);
                }

                s = Tool.Surface(fac)!;

                // Compute the tol2d
                double tol3d = Max(Tool.Tolerance(efrom), Tool.Tolerance(fac));
                SurfaceAdaptor gas = new(s, umin, umax, vmin, vmax);
                double tolU = gas.UResolution(tol3d);
                double tolV = gas.VResolution(tol3d);
                double tol2d = Max(tolU, tolV);

                Curve2D? c2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, s, umin, umax, vmin, vmax, ref tol2d);
                if (c2d is null)
                {
                    return;
                }

                Point2D pf = c2d.Value(f);
                Point2D pl = c2d.Value(l);

                if (s.IsUPeriodic)
                {
                    double up = s.UPeriod;
                    double tolu = global::Kerf.Engine.Foundation.Precision.PConfusion(); // Epsilon(up);
                    int nbtra = 0;
                    double theUmin = Min(pf.X, pl.X);
                    double theUmax = Max(pf.X, pl.X);

                    if (theUmin < umin - tolu)
                    {
                        while (theUmin < umin - tolu)
                        {
                            theUmin += up;
                            theUmax += up;
                            nbtra++;
                        }
                    }
                    else if (theUmax > umax + tolu)
                    {
                        while (theUmax > umax + tolu)
                        {
                            theUmax -= up;
                            theUmin -= up;
                            nbtra--;
                        }
                    }
                    if (nbtra != 0)
                    {
                        c2d = Translatable(c2d);
                        c2d.Translate(new Vector2D(nbtra * up, 0.0));
                    }
                }

                if (s.IsVPeriodic)
                {
                    double vp = s.VPeriod;
                    double tolv = global::Kerf.Engine.Foundation.Precision.PConfusion(); // Epsilon(vp);
                    int nbtra = 0;
                    double theVmin = Min(pf.Y, pl.Y);
                    double theVmax = Max(pf.Y, pl.Y);

                    if (theVmin < vmin - tolv)
                    {
                        while (theVmin < vmin - tolv)
                        {
                            theVmin += vp;
                            theVmax += vp;
                            nbtra++;
                        }
                    }
                    else if (theVmax > vmax + tolv)
                    {
                        while (theVmax > vmax + tolv)
                        {
                            theVmax -= vp;
                            theVmin -= vp;
                            nbtra--;
                        }
                    }
                    if (nbtra != 0)
                    {
                        c2d = Translatable(c2d);
                        c2d.Translate(new Vector2D(0.0, nbtra * vp));
                    }
                }
                b.UpdateEdge(efrom, c2d, fac, Tool.Tolerance(efrom));
            }
        }

        else
        {
            Face fac = ShapeCasts.Face(lfaces.First());
            if (!Tool.IsClosed(eto, fac))
            {
                throw new ConstructionErrorException();
            }

            Shape localE = efrom.Oriented(Forward);
            Shape localF = fac.Oriented(Forward);
            Curve2D? c2dff = Tool.CurveOnSurface(ShapeCasts.Edge(localE), ShapeCasts.Face(localF), out f, out l);

            localE = efrom.Oriented(Reversed);
            localF = fac.Oriented(Forward);
            Curve2D? c2dfr = Tool.CurveOnSurface(ShapeCasts.Edge(localE), ShapeCasts.Face(localF), out f, out l);

            localE = eto.Oriented(Forward);
            localF = fac.Oriented(Forward);
            Curve2D c2dtf = Tool.CurveOnSurface(ShapeCasts.Edge(localE), ShapeCasts.Face(localF), out f, out l)!;

            localE = eto.Oriented(Reversed);
            localF = fac.Oriented(Forward);
            Curve2D c2dtr = Tool.CurveOnSurface(ShapeCasts.Edge(localE), ShapeCasts.Face(localF), out f, out l)!;

            Point2D ptf = c2dtf.Value(f); // sur courbe frw
            Point2D ptr = c2dtr.Value(f); // sur courbe rev

            bool isoU = (Math.Abs(ptf.Y - ptr.Y) < Epsilon(ptf.X)); // meme V

            // Efrom et Eto dans le meme sens???

            c = Tool.Curve(efrom, out loc, out f, out l)!;
            if (!loc.IsIdentity)
            {
                Geometry3D gg = c.Transformed(loc.Transformation);
                c = (Curve3D)gg;
            }

            c.D1(f, out Point3D pt, out Vector3D d1f);

            Vertex firstVertex = Exploration.FirstVertex(efrom);
            double vtx_param = Tool.Parameter(firstVertex, efrom);
            EdgeCurve2DAdaptor bAcurve2d = new(efrom, fac);
            Point2D p2d = bAcurve2d.Value(vtx_param);

            double prmproj = Project(Exploration.FirstVertex(efrom), p2d, eto, fac);

            c = Tool.Curve(eto, out loc, out f, out l)!;
            if (!loc.IsIdentity)
            {
                Geometry3D gg = c.Transformed(loc.Transformation);
                c = (Curve3D)gg;
            }

            c.D1(prmproj, out pt, out Vector3D d1t);

            double sameOri = (d1t.Dot(d1f) > 0.0) ? 1.0 : 0.0;

            if (c2dff is null && c2dfr is null)
            {
                s = Tool.Surface(fac)!;
                if (s is TrimmedSurface trimmed)
                {
                    s = trimmed.BasisSurface;
                }

                c = Tool.Curve(efrom, out loc, out f, out l)!;
                if (!loc.IsIdentity)
                {
                    Geometry3D gg = c.Transformed(loc.Transformation);
                    c = (Curve3D)gg;
                }

                if (c.GetType() != typeof(TrimmedCurve3D))
                {
                    c = new TrimmedCurve3D(c, f, l);
                }

                // Compute the tol2d
                BrepTools.UVBounds(fac, out umin, out umax, out vmin, out vmax);

                double tol3d = Max(Tool.Tolerance(efrom), Tool.Tolerance(fac));
                SurfaceAdaptor gas = new(s, umin, umax, vmin, vmax);
                double tolU = gas.UResolution(tol3d);
                double tolV = gas.VResolution(tol3d);
                double tol2d = Max(tolU, tolV);

                Curve2D? c2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, s, umin, umax, vmin, vmax, ref tol2d);
                c2dff = c2d;
                c2dfr = c2d;
            }
            else if (c2dfr is null)
            {
                c2dfr = c2dff;
            }
            else if (c2dff is null)
            {
                c2dff = c2dfr;
            }

            Tool.Range(efrom, out f, out l);

            Point2D p2f = c2dff!.Value(f);
            Point2D p2r = c2dfr!.Value(f);

            if (isoU)
            {
                if (sameOri != 0.0)
                {
                    if (Math.Abs(ptf.X - p2f.X) > Epsilon(ptf.X))
                    {
                        c2dff = (Curve2D)c2dff.Translated(new Vector2D(ptf.X - p2f.X, 0.0));
                    }
                    if (Math.Abs(ptr.X - p2r.X) > Epsilon(ptr.X))
                    {
                        c2dfr = (Curve2D)c2dfr.Translated(new Vector2D(ptr.X - p2r.X, 0.0));
                    }
                }
                else
                {
                    if (Math.Abs(ptr.X - p2f.X) > Epsilon(ptr.X))
                    {
                        c2dff = (Curve2D)c2dff.Translated(new Vector2D(ptr.X - p2f.X, 0.0));
                    }

                    if (Math.Abs(ptf.X - p2r.X) > Epsilon(ptf.X))
                    {
                        c2dfr = (Curve2D)c2dfr.Translated(new Vector2D(ptf.X - p2r.X, 0.0));
                    }
                }

                // on est bien en U, recalage si periodique en V a faire
            }

            else
            { // !isoU soit isoV
                if (sameOri != 0.0)
                {
                    if (Math.Abs(ptf.Y - p2f.Y) > Epsilon(ptf.Y))
                    {
                        c2dff = (Curve2D)c2dff.Translated(new Vector2D(0.0, ptf.Y - p2f.Y));
                    }
                    if (Math.Abs(ptr.Y - p2r.Y) > Epsilon(ptr.Y))
                    {
                        c2dfr = (Curve2D)c2dfr.Translated(new Vector2D(0.0, ptr.Y - p2r.Y));
                    }
                }
                else
                {
                    if (Math.Abs(ptr.Y - p2f.Y) > Epsilon(ptr.Y))
                    {
                        c2dff = (Curve2D)c2dff.Translated(new Vector2D(0.0, ptr.Y - p2f.Y));
                    }
                    if (Math.Abs(ptf.Y - p2r.Y) > Epsilon(ptf.Y))
                    {
                        c2dfr = (Curve2D)c2dfr.Translated(new Vector2D(0.0, ptf.Y - p2r.Y));
                    }
                }
                // on est bien en V, recalage si periodique en U a faire
            }
            b.UpdateEdge(efrom, c2dff, c2dfr, fac, Tool.Tolerance(efrom));
        }
    }

    //=================================================================================================

    private static void FindInternalIntersections(in Edge edge,
                                                  in Face face,
                                                  IndexedMap<Shape, NCollection_List<Shape>> splits,
                                                  ref bool isOverlapped)
    {
        double tolExt = global::Kerf.Engine.Foundation.Precision.PConfusion();
        int i, j;

        FaceSurfaceAdaptor adSurf = new(face, false);
        _ = adSurf;
        NCollection_Sequence<double> splitPars = new();

        Vertex[] theVertices = new Vertex[2];
        Exploration.Vertices(edge, out theVertices[0], out theVertices[1]);
        Point3D[] thePnt = new Point3D[2];
        thePnt[0] = Tool.Pnt(theVertices[0]);
        thePnt[1] = Tool.Pnt(theVertices[1]);
        double[] tolV = new double[2];
        tolV[0] = Tool.Tolerance(theVertices[0]);
        tolV[1] = Tool.Tolerance(theVertices[1]);
        double ext = 16.0; // = 4 * 4 - to avoid creating microedges, area around vertices is increased
                           // up to 4 vertex tolerance. Such approach is usual for other topological
                           // algorithms, for example, Boolean Operations.
        double[] tolVExt = [ext * tolV[0] * tolV[0], ext * tolV[1] * tolV[1]];

        EdgeCurve2DAdaptor thePCurve = new(edge, face);
        Box2d theBox = new();
        Add2dCurve.Add(thePCurve, Tool.Tolerance(edge), ref theBox);

        double[] thePar = new double[2];
        Curve3D theCurve = Tool.Curve(edge, out thePar[0], out thePar[1])!;
        CurveAdaptor theGAcurve = new(theCurve, thePar[0], thePar[1]);
        double[] tolV2d = [theGAcurve.Resolution(tolV[0]), theGAcurve.Resolution(tolV[1])];
        tolV2d[0] = Max(tolV2d[0], global::Kerf.Engine.Foundation.Precision.PConfusion());
        tolV2d[1] = Max(tolV2d[1], global::Kerf.Engine.Foundation.Precision.PConfusion());
        double distMax = global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion();
        Explorer explo = new(face, global::Kerf.Topology.ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Edge anEdge = ShapeCasts.Edge(explo.Current());
            EdgeCurve2DAdaptor pCurve = new(anEdge, face);
            Box2d box = new();
            Add2dCurve.Add(pCurve, Tool.Tolerance(anEdge), ref box);
            if (theBox.IsOut(box))
            {
                continue;
            }

            Curve3D? curve = Tool.Curve(anEdge, out double fpar, out double lpar);
            CurveAdaptor gAcurve = new(curve ?? throw new NullObjectException(), fpar, lpar); // OCCT's Load throws on a null curve
            ExtCC extrema = new(theGAcurve, gAcurve, tolExt, tolExt);

            if (!extrema.IsDone || extrema.ExtCount == 0)
            {
                continue;
            }

            int nbExt = extrema.ExtCount;
            double maxTol = Tool.Tolerance(anEdge);
            double maxTol2 = maxTol * maxTol;
            if (extrema.IsParallel && extrema.SquareDistance(1) <= maxTol2)
            {
                isOverlapped = true;
                return;
            }
            // Check extremity distances
            double[] dists = new double[4];
            extrema.TrimmedSquareDistances(out dists[0], out dists[1], out dists[2], out dists[3],
                                             out Point3D p11, out Point3D p12, out Point3D p21, out Point3D p22);
            for (i = 0; i < 4; ++i)
            {
                if (i < 2)
                {
                    j = 0;
                }
                else
                {
                    j = 1;
                }
                if (dists[i] < tolVExt[j] / ext)
                {
                    return;
                }
            }

            for (i = 1; i <= nbExt; i++)
            {
                double dist = extrema.SquareDistance(i);
                if (dist > maxTol2)
                {
                    continue;
                }

                extrema.Points(i, out POnCurv pOnC1, out POnCurv pOnC2);
                double theIntPar = pOnC1.Parameter();
                double intPar = pOnC2.Parameter();
                for (j = 0; j < 2; j++) // try to find intersection on an extremity of "theEdge"
                {
                    if (Math.Abs(theIntPar - thePar[j]) <= tolV2d[j])
                    {
                        break;
                    }
                }
                // intersection found in the middle of the edge
                if (j >= 2) // intersection is inside "theEdge" => split
                {
                    Point3D point = curve!.Value(intPar);
                    Point3D pointInt = theCurve.Value(theIntPar);

                    if (pointInt.SquareDistanceTo(thePnt[0]) > tolVExt[0]
                        && pointInt.SquareDistanceTo(thePnt[1]) > tolVExt[1]
                        && point.SquareDistanceTo(thePnt[0]) > tolVExt[0]
                        && point.SquareDistanceTo(thePnt[1]) > tolVExt[1])
                    {
                        splitPars.Append(theIntPar);
                        if (dist > distMax)
                        {
                            distMax = dist;
                        }
                    }
                }
            }
        }

        if (splitPars.IsEmpty())
        {
            return;
        }

        // Sort
        for (i = 1; i < splitPars.Length(); i++)
        {
            for (j = i + 1; j <= splitPars.Length(); j++)
            {
                if (splitPars[i] > splitPars[j])
                {
                    double tmp = splitPars[i];
                    splitPars[i] = splitPars[j];
                    splitPars[j] = tmp;
                }
            }
        }
        // Remove repeating points
        i = 1;
        while (i < splitPars.Length())
        {
            Point3D pnt1 = theCurve.Value(splitPars[i]);
            Point3D pnt2 = theCurve.Value(splitPars[i + 1]);
            if (pnt1.SquareDistanceTo(pnt2) <= global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                splitPars.Remove(i + 1);
            }
            else
            {
                i++;
            }
        }

        // Split
        NCollection_List<Shape> newEdges = new();
        BrepBuilder bb = new();

        Vertex firstVertex = theVertices[0], lastVertex;
        double firstPar = thePar[0], lastPar;
        for (i = 1; i <= splitPars.Length() + 1; i++)
        {
            firstVertex.Orient(Forward);
            if (i <= splitPars.Length())
            {
                lastPar = splitPars[i];
                Point3D lastPoint = theCurve.Value(lastPar);
                lastVertex = new LibraryMakeVertex(lastPoint).InternalVertex();
                BrepBuilder b = new();
                b.UpdateVertex(lastVertex, Math.Sqrt(distMax));
            }
            else
            {
                lastPar = thePar[1];
                lastVertex = theVertices[1];
            }
            lastVertex.Orient(Reversed);

            Shape localShape = edge.EmptyCopied();
            Orientation orient = localShape.Orientation;
            localShape.Orient(Forward);
            Edge newEdge = ShapeCasts.Edge(localShape);
            bb.Range(newEdge, firstPar, lastPar);
            bb.Add(newEdge, firstVertex);
            bb.Add(newEdge, lastVertex);
            newEdge.Orient(orient);
            EdgeAnalysis sae = new();
            double amaxdev = 0.0;
            if (sae.CheckSameParameter(newEdge, face, ref amaxdev))
            {
                BrepBuilder b = new();
                b.UpdateEdge(newEdge, amaxdev);
            }

            if (orient == Forward)
            {
                newEdges.Append(newEdge);
            }
            else
            {
                newEdges.Prepend(newEdge);
            }
            firstVertex = lastVertex;
            firstPar = lastPar;
        }

        if (!newEdges.IsEmpty())
        {
            splits.Add(edge, newEdges);
        }
    }
}
