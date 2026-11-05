// Kerf: the fillets' topological reconstruction on the Boolean engine (unit 7R, option C of
// docs/phase7-plan.md 2.1, decision 992). Not a port: it answers for TopOpeBRepBuild_HBuilder, which Kerf does
// not port, what ChFi3d_Builder asks of it (ChFi3d_Reconstruction), reading the same data structure.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Fillets;

/// <summary>
/// The fillets' reconstruction: builds from the legacy data structure ChFi3d filled what OCCT's
/// <c>TopOpeBRepBuild_HBuilder</c> builds (the edges of its curves, the faces of its surfaces, the input's faces and
/// edges cut, the solids merged) with the Boolean engine's builders, and answers the same queries.
/// </summary>
/// <remarks>
/// <para>
/// <b>What the data structure says</b> (decision 1069): each curve has its points (CPIs: a vertex or a new point, the
/// parameter, a transition whose orientation for the state IN is FORWARD at the start of the kept part and REVERSED
/// at its end); each face of the input cut by a fillet has SCIs (the curve, its pcurve on the face, a transition); each
/// fillet surface has the SCIs of its boundary curves; each edge of the input a fillet cuts has CPIs at its points
/// (the states before and after them); the solid has SSIs on the fillet surfaces. The transitions' orientations for
/// the state IN orient each new edge in the faces it bounds, and each new face in the solid, as the HBuilder orients
/// them (<c>TopOpeBRepBuild_Builder::AddIntersectionEdges</c>, <c>BuildFaces</c>, <c>SplitSolid</c>).
/// </para>
/// <para>
/// <b>The steps</b> (decisions 1110 to 1115). <see cref="Perform"/>: (1) the vertices of the points and the edges of
/// the curves, each between a FORWARD and the next REVERSED point, with its pcurves on the faces and surfaces whose
/// SCIs name it (two on a seam); the edges of the input cut at their points, each part with its state; (2) the faces
/// of the surfaces, by <see cref="BuilderFace"/> from their edges. <see cref="MergeSolid"/>: (3) each face of
/// the solid with curves or cut edges is rebuilt by <see cref="BuilderFace"/> from the edges it keeps: its new
/// edges oriented by their transitions, the parts IN of its cut edges, its other edges whole (those the fillet removes
/// are left dangling and dropped), the wires the fillet does not touch classified apart, degenerated edges cut where a
/// fillet reaches them; (4) the pieces, the faces left whole and the fillet faces (oriented by the SSIs) are assembled
/// into shells by connexity, and a solid. The queries (5) answer from the maps those steps fill.
/// </para>
/// <para>
/// <b>Shapes.</b> The input is the shape the fillet builder works on, in its working overlay (decision 1087, unit
/// 10H5): the reconstruction writes only into
/// the shapes it makes (vertices of points, edges, parts and pieces, faces, shells, solids), and reuses the input's
/// faces and edges left whole and its vertices. Nothing is frozen here: ChFi3d's <c>Compute</c> raises the tolerances
/// of vertices, makes the new faces same-parameter and codes the regularities on the result afterwards; the builder
/// freezes once at the end.
/// </para>
/// </remarks>
internal sealed partial class BOPReconstruction : Reconstruction
{
    /// <summary>The build tool OCCT gives its HBuilder (kept for the record: the edges are built here).</summary>
    private readonly DsBuildTool _buildTool;

    private DsHDataStructure? _hds;
    private IntersectionContext _context = new();
    private readonly BrepBuilder _bb = new();

    /// <summary>The vertices made for the data structure's points.</summary>
    private readonly Dictionary<int, Vertex> _pointVertices = new();

    /// <summary>What <see cref="NewEdges"/> and <see cref="NewFaces"/> give: by curve and by surface.</summary>
    private readonly Dictionary<int, NCollection_List<Shape>> _newEdges = new();
    private readonly Dictionary<int, NCollection_List<Shape>> _newFaces = new();

    /// <summary>The parts of each input edge cut at points, forward, with their states.</summary>
    private readonly Dictionary<Shape, List<(Edge Part, Containment State)>> _edgeParts =
        new(ShapeHasher.Instance);

    /// <summary>The parts kept (state IN) of the edges and faces cut: <see cref="Splits"/>.</summary>
    private readonly Dictionary<Shape, NCollection_List<Shape>> _splitIN =
        new(ShapeHasher.Instance);

    /// <summary>What <see cref="MergeSolid"/> made of each solid or shell: <see cref="Merged"/>.</summary>
    private readonly Dictionary<Shape, NCollection_List<Shape>> _mergedIN =
        new(ShapeHasher.Instance);

    /// <summary>The SCIs of each curve, on faces (support kind SURFACE, a shape index) and surfaces.</summary>
    private readonly Dictionary<int, List<CurveSupport>> _curveSupports = new();

    private static readonly NCollection_List<Shape> EmptyList = new();

    /// <summary>A curve's SCI: on a face of the input (<see cref="Face"/> not null) or on a fillet surface.</summary>
    private readonly record struct CurveSupport(Face Face, int Surface, Curve2D PCurve,
                                                Orientation Orientation);

    /// <summary>A reconstruction for the fillet builder (<see cref="Reconstruction.Create"/>).</summary>
    internal BOPReconstruction(DsBuildTool bt) => _buildTool = bt;

    //=================================================================================================
    // ChFi3d_Reconstruction

    /// <inheritdoc/>
    public override void Perform(DsHDataStructure hds)
    {
        Clear();
        _hds = hds;
        CollectCurveSupports();
        BindPointsAtEdgeEnds();
        BuildEdges();
        SplitEdges();
        BuildFaces();
    }

    /// <inheritdoc/>
    public override void MergeSolid(in Shape s, Containment tb)
    {
        if (tb != In || _hds is null || _mergedIN.ContainsKey(s))
        {
            return;
        }
        NCollection_List<Shape> merged = new();
        MergeShape(s, merged);
        _mergedIN[s] = merged;
    }

    /// <inheritdoc/>
    public override bool IsSplit(in Shape s, Containment toBuild) =>
        toBuild == In && _splitIN.TryGetValue(s, out NCollection_List<Shape>? l) && !l.IsEmpty();

    /// <inheritdoc/>
    public override NCollection_List<Shape> Splits(in Shape s, Containment toBuild) =>
        toBuild == In && _splitIN.TryGetValue(s, out NCollection_List<Shape>? l) ? l : EmptyList;

    /// <inheritdoc/>
    public override NCollection_List<Shape> Merged(in Shape s, Containment toBuild) =>
        toBuild == In && _mergedIN.TryGetValue(s, out NCollection_List<Shape>? l) ? l : EmptyList;

    /// <inheritdoc/>
    public override NCollection_List<Shape> NewFaces(int i) =>
        _newFaces.TryGetValue(i, out NCollection_List<Shape>? l) ? l : EmptyList;

    /// <inheritdoc/>
    public override NCollection_List<Shape> NewEdges(int i) =>
        _newEdges.TryGetValue(i, out NCollection_List<Shape>? l) ? l : EmptyList;

    /// <inheritdoc/>
    public override DsHDataStructure? DataStructure() => _hds;

    public override string ToString() => "ChFi3d_BOPReconstruction, " + _buildTool;

    //=================================================================================================
    // Perform

    private DsDataStructure DS => _hds!.DS();

    private void Clear()
    {
        _hds = null;
        _context = new IntersectionContext();
        _pointVertices.Clear();
        _newEdges.Clear();
        _newFaces.Clear();
        _edgeParts.Clear();
        _splitIN.Clear();
        _mergedIN.Clear();
        _curveSupports.Clear();
    }

    /// <summary>The SCIs naming each curve: on the input's faces (shape interferences) and on the surfaces.</summary>
    private void CollectCurveSupports()
    {
        DsDataStructure ds = DS;
        for (int i = 1; i <= ds.NbShapes(); i++)
        {
            Shape s = ds.Shape(i);
            if (s.IsNull || s.Type != ShapeType.Face)
            {
                continue;
            }
            Face f = ShapeCasts.Face(s);
            foreach (DsInterference I in ds.ShapeInterferences(i))
            {
                if (I is DsSurfaceCurveInterference sci && sci.GeometryType() == DsKind.Curve
                    && sci.PCurve() is Curve2D pc)
                {
                    Orientation o = sci.Transition().Orientation(In);
                    AddCurveSupport(sci.Geometry(), new CurveSupport(f, 0, pc, o));
                }
            }
        }
        for (int iS = 1; iS <= ds.NbSurfaces(); iS++)
        {
            foreach (DsInterference I in ds.SurfaceInterferences(iS))
            {
                if (I is DsSurfaceCurveInterference sci && sci.GeometryType() == DsKind.Curve
                    && sci.PCurve() is Curve2D pc)
                {
                    Orientation o = sci.Transition().Orientation(In);
                    AddCurveSupport(sci.Geometry(), new CurveSupport(new Face(), iS, pc, o));
                }
            }
        }
    }

    private void AddCurveSupport(int curve, CurveSupport support)
    {
        if (!_curveSupports.TryGetValue(curve, out List<CurveSupport>? l))
        {
            _curveSupports[curve] = l = [];
        }
        l.Add(support);
    }

    /// <summary>The vertex of a point of the data structure: the input's vertex, or one made for the point.</summary>
    private Vertex VertexOf(DsKind kind, int index)
    {
        if (kind == DsKind.Vertex)
        {
            return ShapeCasts.Vertex(DS.Shape(index));
        }
        if (!_pointVertices.TryGetValue(index, out Vertex v))
        {
            DsPoint p = DS.Point(index);
            v = new Vertex();
            _bb.MakeVertex(ref v, p.Point(), Math.Max(p.Tolerance(), global::Kerf.Engine.Foundation.Precision.Confusion()));
            _pointVertices[index] = v;
        }
        return v;
    }

    /// <summary>Step 1: the edges of the curves, between their points.</summary>
    private void BuildEdges()
    {
        DsDataStructure ds = DS;
        for (int iC = 1; iC <= ds.NbCurves(); iC++)
        {
            List<(double T, Vertex V, Orientation O)> paves = [];
            foreach (DsInterference i in ds.CurveInterferences(iC))
            {
                if (i is DsCurvePointInterference cpi
                    && (cpi.GeometryType() == DsKind.Point || cpi.GeometryType() == DsKind.Vertex))
                {
                    paves.Add((cpi.Parameter(), VertexOf(cpi.GeometryType(), cpi.Geometry()),
                                cpi.Transition().Orientation(In)));
                }
            }
            if (paves.Count < 2)
            {
                continue; // a curve without points makes no edge (OCCT's BuildEdges: no pave)
            }
            paves.Sort((a, b) => a.T.CompareTo(b.T));
            NCollection_List<Shape> le = new();
            for (int k = 0; k + 1 < paves.Count; k++)
            {
                if (paves[k].O == Reversed || paves[k + 1].O == Forward)
                {
                    continue;
                }
                if (paves[k + 1].T - paves[k].T <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    continue; // two points at one parameter: no edge, as the HBuilder's pave set (decision 1159)
                }
                Edge e = MakeCurveEdge(iC, paves[k].V, paves[k].T, paves[k + 1].V, paves[k + 1].T);
                le.Append(e);
            }
            if (!le.IsEmpty())
            {
                _newEdges[iC] = le;
            }
        }
    }

    /// <summary>An edge on a curve between two of its points, with the pcurves of its SCIs.</summary>
    private Edge MakeCurveEdge(int iC, Vertex v1, double t1, Vertex v2, double t2)
    {
        DsCurve c = DS.Curve(iC);
        double tol = Math.Max(c.Tolerance(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Edge e = new();
        Curve3D? c3d = c.Curve();
        if (c3d is null)
        {
            _bb.MakeEdge(ref e);
            _bb.UpdateEdge(e, tol);
            _bb.Degenerated(e, true);
        }
        else
        {
            _bb.MakeEdge(ref e, c3d, tol);
        }
        _bb.Add(e, v1.Oriented(Forward));
        _bb.Add(e, v2.Oriented(Reversed));

        if (_curveSupports.TryGetValue(iC, out List<CurveSupport>? supports))
        {
            // the pcurves, by face or surface: two on the same one make a seam (the FORWARD one first)
            HashSet<int> done = [];
            for (int k = 0; k < supports.Count; k++)
            {
                if (done.Contains(k))
                {
                    continue;
                }
                CurveSupport s = supports[k];
                int twin = -1;
                for (int j = k + 1; j < supports.Count && twin < 0; j++)
                {
                    if (SameSupport(s, supports[j]))
                    {
                        twin = j;
                    }
                }
                GetSupport(s, out Surface surf, out Placement loc, out double surfTol);
                tol = Math.Max(tol, surfTol);
                if (twin < 0)
                {
                    _bb.UpdateEdge(e, s.PCurve, surf, loc, tol);
                }
                else
                {
                    done.Add(twin);
                    CurveSupport t = supports[twin];
                    (Curve2D f, Curve2D r) =
                        s.Orientation == Reversed ? (t.PCurve, s.PCurve) : (s.PCurve, t.PCurve);
                    _bb.UpdateEdge(e, f, r, surf, loc, tol);
                }
            }
        }
        _bb.Range(e, t1, t2);
        _bb.UpdateEdge(e, tol);
        return e;
    }

    private static bool SameSupport(in CurveSupport a, in CurveSupport b) =>
        a.Face.IsNull ? b.Face.IsNull && a.Surface == b.Surface : !b.Face.IsNull && a.Face.IsSame(b.Face);

    private void GetSupport(in CurveSupport s, out Surface surf, out Placement loc,
                            out double tol)
    {
        if (!s.Face.IsNull)
        {
            surf = Tool.Surface(s.Face, out loc)!;
            tol = 0.0;
        }
        else
        {
            DsSurface S = DS.Surface(s.Surface);
            surf = S.Surface()!;
            loc = new Placement();
            tol = S.Tolerance(); // OCCT's BuildFaces raises the edges to the surface's tolerance
        }
    }

    /// <summary>Step 1: the input's edges cut at their points, each part with its state.</summary>
    private void SplitEdges()
    {
        DsDataStructure ds = DS;
        for (int i = 1; i <= ds.NbShapes(); i++)
        {
            Shape s = ds.Shape(i);
            if (s.IsNull || s.Type != ShapeType.Edge)
            {
                continue;
            }
            Edge e = ShapeCasts.Edge(s.Oriented(Forward));
            if (Tool.Degenerated(e))
            {
                continue;
            }
            List<(double T, Vertex V, Containment Before, Containment After)> cuts = [];
            bool onlyPointsAtEnds = true;
            foreach (DsInterference I in ds.ShapeInterferences(i))
            {
                if (I is DsCurvePointInterference cpi
                    && (cpi.GeometryType() == DsKind.Point || cpi.GeometryType() == DsKind.Vertex))
                {
                    DsTransition t = cpi.Transition();
                    cuts.Add((cpi.Parameter(), VertexOf(cpi.GeometryType(), cpi.Geometry()), t.Before(), t.After()));
                    onlyPointsAtEnds &= cpi.GeometryType() == DsKind.Point && AtEnd(e, cpi.Parameter());
                }
            }
            if (cuts.Count == 0 || onlyPointsAtEnds)
            {
                // points only where the edge's vertices are (a fillet ending at a vertex the data structure gives as a
                // point): they cut nothing, and the edge is left whole (decision 1157)
                continue;
            }
            SplitEdge(e, cuts);
        }
    }

    private static bool AtEnd(Edge e, double t)
    {
        Tool.Range(e, out double f, out double l);
        return Math.Abs(t - f) <= global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(t - l) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>
    /// The points the data structure puts on an edge at one of its ends are its vertex there: the curves ending at
    /// them then end on the edge's vertex, as natively (decision 1157).
    /// </summary>
    private void BindPointsAtEdgeEnds()
    {
        DsDataStructure ds = DS;
        for (int i = 1; i <= ds.NbShapes(); i++)
        {
            Shape s = ds.Shape(i);
            if (s.IsNull || s.Type != ShapeType.Edge)
            {
                continue;
            }
            Edge e = ShapeCasts.Edge(s.Oriented(Forward));
            if (Tool.Degenerated(e))
            {
                continue;
            }
            Tool.Range(e, out double f, out double l);
            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
            foreach (DsInterference I in ds.ShapeInterferences(i))
            {
                if (I is not DsCurvePointInterference cpi || cpi.GeometryType() != DsKind.Point
                    || _pointVertices.ContainsKey(cpi.Geometry()))
                {
                    continue;
                }
                Vertex v = Math.Abs(cpi.Parameter() - f) <= global::Kerf.Engine.Foundation.Precision.PConfusion() ? v1
                                  : Math.Abs(cpi.Parameter() - l) <= global::Kerf.Engine.Foundation.Precision.PConfusion() ? v2 : new Vertex();
                if (!v.IsNull)
                {
                    _pointVertices[cpi.Geometry()] = v;
                }
            }
        }
    }

    private void SplitEdge(Edge e,
                           List<(double T, Vertex V, Containment Before, Containment After)> cuts)
    {
        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
        Tool.Range(e, out double f, out double l);
        cuts.Sort((a, b) => a.T.CompareTo(b.T));
        double tolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();

        // a point beyond an end, where the part IN runs to it: the fillet's section overflows the edge, and the part
        // kept is extended to the point along the edge's curve, as the HBuilder's pave set does (decision 1160)
        (double T, Vertex V) start = (f, v1), end = (l, v2);
        Curve3D? c3d = Tool.Curve(e, out double _, out double _);
        foreach ((double T, Vertex V, Containment Before, Containment After) c in cuts)
        {
            if (c3d is null)
            {
                break;
            }
            if (c.T > l + tolPar && c.Before == In && c.T <= c3d.LastParameter)
            {
                end = (c.T, c.V);
            }
            else if (c.T < f - tolPar && c.After == In && c.T >= c3d.FirstParameter && start.T == f)
            {
                start = (c.T, c.V);
            }
        }
        bool extended = start.T != f || end.T != l;

        // the paves: the edge's ends and its points inside, each with the state of the part that follows
        List<(double T, Vertex V)> paves = [start];
        List<Containment> states = [];
        Containment first = cuts[0].Before;
        states.Add(first);
        foreach ((double T, Vertex V, Containment Before, Containment After) c in cuts)
        {
            if (c.T <= f + tolPar)
            {
                states[0] = c.After; // a point at the start only gives the state of the first part
                continue;
            }
            if (c.T >= l - tolPar)
            {
                continue;
            }
            if (c.T - paves[^1].T <= tolPar)
            {
                states[^1] = c.After;
                continue;
            }
            paves.Add((c.T, c.V));
            states.Add(c.After);
        }
        paves.Add(end);
        if (paves.Count == 2 && !extended)
        {
            // not cut: one part, kept when its state is IN
            List<(Edge, Containment)> whole = [(e, states[0])];
            _edgeParts[e] = whole;
            NCollection_List<Shape> keep = new();
            if (states[0] != Out)
            {
                keep.Append(e);
            }
            _splitIN[e] = keep;
            return;
        }
        List<(Edge Part, Containment State)> parts = [];
        NCollection_List<Shape> aIN = new();
        for (int k = 0; k + 1 < paves.Count; k++)
        {
            AlgoTools.MakeSplitEdge(e, paves[k].V, paves[k].T, paves[k + 1].V, paves[k + 1].T,
                                             out Edge part);
            parts.Add((part, states[k]));
            if (states[k] != Out)
            {
                aIN.Append(part);
            }
        }
        _edgeParts[e] = parts;
        _splitIN[e] = aIN;
    }

    /// <summary>Step 2: the faces of the surfaces, from their edges oriented by the SCIs.</summary>
    private void BuildFaces()
    {
        DsDataStructure ds = DS;
        for (int iS = 1; iS <= ds.NbSurfaces(); iS++)
        {
            DsSurface s = ds.Surface(iS);
            if (s.Surface() is not Surface surf)
            {
                continue;
            }
            NCollection_List<Shape> le = new();
            foreach (DsInterference i in ds.SurfaceInterferences(iS))
            {
                if (i is not DsSurfaceCurveInterference sci || sci.GeometryType() != DsKind.Curve)
                {
                    continue;
                }
                Orientation o = sci.Transition().Orientation(In);
                if (o != Forward && o != Reversed)
                {
                    continue;
                }
                foreach (Shape e in NewEdges(sci.Geometry()))
                {
                    le.Append(e.Oriented(o));
                }
            }
            if (le.IsEmpty())
            {
                continue;
            }
            Face f = new();
            _bb.MakeFace(ref f, surf, Math.Max(s.Tolerance(), global::Kerf.Engine.Foundation.Precision.Confusion()));
            NCollection_List<Shape> faces = BuildAreas(f, le, out NCollection_List<Shape> loops);
            if (faces.IsEmpty())
            {
                // the SCIs' edges close a loop the face builder takes for a hole (a fillet surface whose normal runs
                // against its transitions): the HBuilder makes the face on it all the same (decision 1158)
                foreach (Shape w in loops)
                {
                    Face fw = ShapeCasts.Face(f.EmptyCopied());
                    _bb.Add(fw, w);
                    faces.Append(fw);
                }
            }
            if (!faces.IsEmpty())
            {
                _newFaces[iS] = faces;
            }
        }
    }

    /// <summary>The faces <see cref="BuilderFace"/> builds on a forward face from edges.</summary>
    private NCollection_List<Shape> BuildAreas(Face face, NCollection_List<Shape> edges) =>
        BuildAreas(face, edges, out _);

    private NCollection_List<Shape> BuildAreas(Face face, NCollection_List<Shape> edges,
                                                      out NCollection_List<Shape> loops)
    {
        BuilderFace bf = new();
        bf.SetFace(face);
        bf.SetShapes(edges);
        bf.SetContext(_context);
        bf.SetRunParallel(false);
        bf.SetAvoidInternalShapes(true); // the edges the fillet leaves dangling are dropped
        bf.Perform();
        loops = new NCollection_List<Shape>(bf.Loops());
        return new NCollection_List<Shape>(bf.Areas());
    }

    //=================================================================================================
    // MergeSolid

    /// <summary>Steps 3 and 4: the solid's (or shell's) faces split, fillet faces added, shells assembled.</summary>
    private void MergeShape(Shape s, NCollection_List<Shape> merged)
    {
        DsDataStructure ds = DS;
        NCollection_List<Shape> faces = new();
        List<Shape> whole = [];
        // the shell of the input each face kept comes from, for the shells' history
        Dictionary<Shape, Shape> origin = new(ShapeHasher.Instance);
        bool changed = false; // a solid the fillets do not touch is not split (natively it is then "deleted")
        for (Explorer exs = new(s, ShapeType.Shell); exs.More(); exs.Next())
        {
            for (Explorer exp = new(exs.Current(), ShapeType.Face); exp.More(); exp.Next())
            {
                Face f = ShapeCasts.Face(exp.Current());
                NCollection_List<Shape>? pieces = SplitFace(f);
                if (pieces is null)
                {
                    whole.Add(f);
                    origin[f] = exs.Current();
                    continue;
                }
                changed = true;
                foreach (Shape p in pieces)
                {
                    faces.Append(p.Oriented(f.Orientation));
                    origin[p] = exs.Current();
                }
            }
        }
        foreach (DsInterference i in ds.ShapeInterferences(s))
        {
            if (i is not DsSolidSurfaceInterference || i.GeometryType() != DsKind.Surface)
            {
                continue;
            }
            Orientation o = i.Transition().Orientation(In);
            if (o != Forward && o != Reversed)
            {
                continue;
            }
            foreach (Shape f in NewFaces(i.Geometry()))
            {
                faces.Append(f.Oriented(o));
                changed = true;
            }
        }
        foreach (Shape f in KeptWholeFaces(s, whole, faces))
        {
            faces.Append(f);
        }

        // the shells: the faces connected by their edges (a solid's outer shell and its voids' apart)
        List<Shell> shells = [];
        foreach (List<Shape> block in ConnectedBlocks(faces))
        {
            Shell shell = new();
            _bb.MakeShell(ref shell);
            foreach (Shape f in block)
            {
                _bb.Add(shell, f);
            }
            shell.Closed(Tool.IsClosed(shell));
            shells.Add(shell);
            // the input's shells this one comes from (decision 1161: the HBuilder splits solids and shells too)
            HashSet<Shape> aFrom = new(ShapeHasher.Instance);
            foreach (Shape f in block)
            {
                if (changed && origin.TryGetValue(f, out Shape old) && aFrom.Add(old))
                {
                    if (!_splitIN.TryGetValue(old, out NCollection_List<Shape>? l))
                    {
                        _splitIN[old] = l = new NCollection_List<Shape>();
                    }
                    l.Append(shell);
                }
            }
        }
        if (s.Type == ShapeType.Shell)
        {
            foreach (Shell shell in shells)
            {
                merged.Append(shell);
            }
            return;
        }
        Solid solid = new();
        _bb.MakeSolid(ref solid);
        foreach (Shell shell in shells)
        {
            _bb.Add(solid, shell);
        }
        merged.Append(solid);
        // BRepFilletAPI's Modified and IsDeleted read the solid's splits: native's give the solid made of it
        if (changed)
        {
            NCollection_List<Shape> solidSplits = new();
            solidSplits.Append(solid);
            _splitIN[s] = solidSplits;
        }
    }

    /// <summary>
    /// The faces left whole that stay: a face is dropped when one of its edges has lost a face it bounded (the
    /// pieces around dropped it: the face lies where a fillet swallowed it), until none is (decision 1114).
    /// </summary>
    private static List<Shape> KeptWholeFaces(Shape s, List<Shape> whole,
                                                     NCollection_List<Shape> others)
    {
        Dictionary<Shape, int> before = EdgeUses(new[] { s });
        List<Shape> kept = [.. whole];
        bool changed = true;
        while (changed && kept.Count > 0)
        {
            changed = false;
            Dictionary<Shape, int> now = EdgeUses(kept.Concat(others));
            for (int i = kept.Count - 1; i >= 0; i--)
            {
                for (Explorer exp = new(kept[i], ShapeType.Edge); exp.More(); exp.Next())
                {
                    Shape e = exp.Current();
                    if (!Tool.Degenerated(ShapeCasts.Edge(e))
                        && now.GetValueOrDefault(e) < before.GetValueOrDefault(e))
                    {
                        kept.RemoveAt(i);
                        changed = true;
                        break;
                    }
                }
            }
        }
        return kept;
    }

    /// <summary>How many faces of the shapes each edge bounds (a seam counts once).</summary>
    private static Dictionary<Shape, int> EdgeUses(IEnumerable<Shape> shapes)
    {
        Dictionary<Shape, int> uses = new(ShapeHasher.Instance);
        foreach (Shape s in shapes)
        {
            for (Explorer expf = new(s, ShapeType.Face); expf.More(); expf.Next())
            {
                HashSet<Shape> seen = new(ShapeHasher.Instance);
                for (Explorer exp = new(expf.Current(), ShapeType.Edge); exp.More(); exp.Next())
                {
                    if (seen.Add(exp.Current()))
                    {
                        uses[exp.Current()] = uses.GetValueOrDefault(exp.Current()) + 1;
                    }
                }
            }
        }
        return uses;
    }

    /// <summary>The faces in blocks connected by shared edges, in the faces' order.</summary>
    private static List<List<Shape>> ConnectedBlocks(NCollection_List<Shape> faces)
    {
        List<Shape> aFaces = [.. faces];
        int[] parent = Enumerable.Range(0, aFaces.Count).ToArray();
        int Root(int i)
        {
            while (parent[i] != i)
            {
                i = parent[i] = parent[parent[i]];
            }
            return i;
        }
        Dictionary<Shape, int> firstFace = new(ShapeHasher.Instance);
        for (int i = 0; i < aFaces.Count; i++)
        {
            for (Explorer exp = new(aFaces[i], ShapeType.Edge); exp.More(); exp.Next())
            {
                if (Tool.Degenerated(ShapeCasts.Edge(exp.Current())))
                {
                    continue;
                }
                if (firstFace.TryGetValue(exp.Current(), out int j))
                {
                    parent[Root(i)] = Root(j);
                }
                else
                {
                    firstFace[exp.Current()] = i;
                }
            }
        }
        Dictionary<int, List<Shape>> blocks = new();
        List<List<Shape>> order = [];
        for (int i = 0; i < aFaces.Count; i++)
        {
            int r = Root(i);
            if (!blocks.TryGetValue(r, out List<Shape>? b))
            {
                blocks[r] = b = [];
                order.Add(b);
            }
            b.Add(aFaces[i]);
        }
        return order;
    }

    /// <summary>
    /// Step 3: the pieces of a face with curves or cut edges (forward), or null if it has neither; recorded as the
    /// face's <see cref="Splits"/>.
    /// </summary>
    /// <remarks>
    /// As the HBuilder's face builder, the pieces are built from the kept edges only: the face's new edges oriented by
    /// their transitions, the parts of its cut edges in the state IN, and its edges not cut. Edges not cut that the
    /// fillet removes (the edge filleted, those of a face it swallows) are left dangling by the parts dropped, and
    /// <see cref="BuilderFace"/> drops them. A wire of the face that touches neither (no edge cut, no vertex
    /// on a kept edge) is classified (decision 1112): an inner wire is kept if it lies inside a piece built without it
    /// (a hole far from the fillet), dropped otherwise (the rim of a hole filleted); the outer wire is kept when the
    /// fillet's loops are only holes (a hole's rim filleted, a concave fillet around a window), classified as the inner
    /// ones otherwise (a face whose every edge is filleted). A concave fillet's new edges lie outside the face: the
    /// face grows to them, as natively.
    /// </remarks>
    private NCollection_List<Shape>? SplitFace(Face f)
    {
        if (_splitIN.TryGetValue(f, out NCollection_List<Shape>? done))
        {
            return done;
        }
        DsDataStructure ds = DS;
        Face ff = ShapeCasts.Face(f.Oriented(Forward));

        // the face's new edges, oriented by their transitions (both ways on a seam of the face)
        NCollection_List<Shape> le = new();
        HashSet<Shape> keptVertices = new(ShapeHasher.Instance);
        HashSet<(TShape, Orientation)> added = [];
        if (ds.HasShape(f))
        {
            foreach (DsInterference i in ds.ShapeInterferences(f))
            {
                if (i is not DsSurfaceCurveInterference sci || sci.GeometryType() != DsKind.Curve)
                {
                    continue;
                }
                Orientation o = sci.Transition().Orientation(In);
                if (o != Forward && o != Reversed)
                {
                    continue;
                }
                foreach (Shape e in NewEdges(sci.Geometry()))
                {
                    if (added.Add((e.TShape()!, o)))
                    {
                        le.Append(e.Oriented(o));
                        AddVertices(e, keptVertices);
                    }
                }
            }
        }
        bool hasCurves = !le.IsEmpty();

        // the face's edges: the kept parts of the cut ones, the others whole
        bool hasCuts = false;
        NCollection_List<Shape> whole = new();
        for (Explorer exp = new(ff, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            if (_edgeParts.TryGetValue(e, out List<(Edge Part, Containment State)>? parts))
            {
                hasCuts = true;
                foreach ((Edge part, Containment state) in parts)
                {
                    if (state != Out)
                    {
                        le.Append(part.Oriented(e.Orientation));
                        AddVertices(part, keptVertices);
                    }
                }
            }
            else if (Tool.Degenerated(e) && SplitDegenerated(ff, e, le) is List<Edge> degParts)
            {
                foreach (Edge part in degParts)
                {
                    le.Append(part);
                }
            }
            else
            {
                whole.Append(e);
            }
        }
        if (!hasCurves && !hasCuts)
        {
            return null;
        }

        // the wires the fillet does not touch are classified afterwards
        Wire outer = BrepTools.OuterWire(ff);
        List<NCollection_List<Shape>> isolated = [];
        NCollection_List<Shape>? outerIsolated = null;
        HashSet<Shape> isolatedEdges = new(ShapeHasher.Instance);
        for (Iterator itw = new(ff); itw.More(); itw.Next())
        {
            Shape w = itw.Value();
            if (w.Type != ShapeType.Wire)
            {
                continue;
            }
            bool touched = false;
            NCollection_List<Shape> we = new();
            for (Explorer exp = new(w, ShapeType.Edge); exp.More() && !touched; exp.Next())
            {
                Shape e = exp.Current();
                touched = _edgeParts.ContainsKey(e);
                for (Explorer exv = new(e, ShapeType.Vertex); exv.More() && !touched; exv.Next())
                {
                    touched = keptVertices.Contains(exv.Current());
                }
                we.Append(e);
            }
            if (touched)
            {
                continue;
            }
            foreach (Shape e in we)
            {
                isolatedEdges.Add(e);
            }
            if (w.IsSame(outer))
            {
                outerIsolated = we;
            }
            else
            {
                isolated.Add(we);
            }
        }
        foreach (Shape e in WholeEdgesKept(whole, isolatedEdges, keptVertices))
        {
            le.Append(e);
        }

        NCollection_List<Shape> pieces = BuildAreas(ff, le);
        if (outerIsolated is not null)
        {
            // the outer wire, not touched: it bounds what the face keeps when the fillet's loops are holes (a hole's
            // rim filleted, a concave fillet around a hole), and is dropped when a loop the fillet made replaces it
            if (pieces.IsEmpty())
            {
                foreach (Shape e in outerIsolated)
                {
                    le.Append(e);
                }
                pieces = BuildAreas(ff, le);
            }
            else
            {
                isolated.Add(outerIsolated);
            }
        }
        if (isolated.Count > 0)
        {
            // an inner wire the fillet does not touch is kept if it lies in a piece (a hole of it), dropped if it
            // lies where the fillet removed the face (the rim of a hole filleted)
            bool any = false;
            foreach (NCollection_List<Shape> we in isolated)
            {
                if (IsInside(we, pieces))
                {
                    any = true;
                    foreach (Shape e in we)
                    {
                        le.Append(e);
                    }
                }
            }
            if (any)
            {
                pieces = BuildAreas(ff, le);
            }
        }
        _splitIN[f] = pieces;
        return pieces;
    }

    /// <summary>
    /// The parts of a degenerated edge of the face (forward) cut where new edges reach its vertex in the face's
    /// parametric space (a fillet ending at a cone's apex), oriented as the edge; null if none reaches it inside.
    /// </summary>
    /// <remarks>
    /// All the parts are given to <see cref="BuilderFace"/>: its wire splitter follows the parametric space
    /// at the degenerated vertex, so the part under the fillet is left over and dropped (decision 1113).
    /// </remarks>
    private static List<Edge>? SplitDegenerated(Face ff, Edge e,
                                                       NCollection_List<Shape> theNew)
    {
        Vertex v = Exploration.FirstVertex(ShapeCasts.Edge(e.Oriented(Forward)));
        Curve2D? dc = Tool.CurveOnSurface(e, ff, out double f, out double l);
        if (v.IsNull || dc is null)
        {
            return null;
        }
        double tolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();
        List<double> aParams = [];
        foreach (Shape s in theNew)
        {
            Edge ne = ShapeCasts.Edge(s.Oriented(Forward));
            Exploration.Vertices(ne, out Vertex v1, out Vertex v2);
            bool a1 = v1.IsSame(v), a2 = v2.IsSame(v);
            if (!a1 && !a2)
            {
                continue;
            }
            Curve2D? nc = Tool.CurveOnSurface(ne, ff, out double nf, out double nl);
            if (nc is null)
            {
                continue;
            }
            foreach (double t in a1 && a2 ? new[] { nf, nl } : new[] { a1 ? nf : nl })
            {
                ProjectPointOnCurve2D proj = new(nc.Value(t), dc, f, l);
                if (proj.NbPoints() == 0)
                {
                    continue;
                }
                double u = proj.LowerDistanceParameter();
                if (u > f + tolPar && u < l - tolPar && aParams.TrueForAll(x => Math.Abs(x - u) > tolPar))
                {
                    aParams.Add(u);
                }
            }
        }
        if (aParams.Count == 0)
        {
            return null;
        }
        aParams.Sort();
        aParams.Insert(0, f);
        aParams.Add(l);
        Edge ef = ShapeCasts.Edge(e.Oriented(Forward));
        List<Edge> parts = [];
        for (int k = 0; k + 1 < aParams.Count; k++)
        {
            AlgoTools.MakeSplitEdge(ef, v, aParams[k], v, aParams[k + 1], out Edge part);
            parts.Add(ShapeCasts.Edge(part.Oriented(e.Orientation)));
        }
        return parts;
    }

    /// <summary>Whether a point inside an edge of the wire is inside one of the faces.</summary>
    private bool IsInside(NCollection_List<Shape> wire, NCollection_List<Shape> faces)
    {
        foreach (Shape s in wire)
        {
            Edge e = ShapeCasts.Edge(s);
            if (Tool.Degenerated(e))
            {
                continue;
            }
            foreach (Shape p in faces)
            {
                Face F = ShapeCasts.Face(p);
                Curve2D? c2d = Tool.CurveOnSurface(e, F, out double f, out double l);
                if (c2d is null)
                {
                    continue;
                }
                Point2D p2d = c2d.Value(AlgoTools2D.IntermediatePoint(f, l));
                if (_context.IsPointInFace(F, p2d))
                {
                    return true;
                }
            }
            return false;
        }
        return false;
    }

    /// <summary>
    /// The face's edges left whole, in touched wires, that it keeps: those connected through edges left whole to a
    /// vertex of a kept edge (decision 1155). A chain of them between two parts dropped lies under the fillet: alone
    /// it dangles and the face builder drops it, but round a seam two such chains close a loop (the rim of a hole that
    /// crosses a cylinder's seam) that it would build into a band.
    /// </summary>
    private static List<Shape> WholeEdgesKept(NCollection_List<Shape> whole,
                                                     HashSet<Shape> isolated,
                                                     HashSet<Shape> keptVertices)
    {
        List<Shape> edges = [.. whole.Where(e => !isolated.Contains(e))];
        int[] parent = Enumerable.Range(0, edges.Count).ToArray();
        int Root(int i)
        {
            while (parent[i] != i)
            {
                i = parent[i] = parent[parent[i]];
            }
            return i;
        }
        Dictionary<Shape, int> first = new(ShapeHasher.Instance);
        for (int i = 0; i < edges.Count; i++)
        {
            for (Explorer exv = new(edges[i], ShapeType.Vertex); exv.More(); exv.Next())
            {
                if (first.TryGetValue(exv.Current(), out int j))
                {
                    parent[Root(i)] = Root(j);
                }
                else
                {
                    first[exv.Current()] = i;
                }
            }
        }
        HashSet<int> anchored = [];
        foreach ((Shape v, int i) in first)
        {
            if (keptVertices.Contains(v))
            {
                anchored.Add(Root(i));
            }
        }
        return [.. edges.Where((e, i) => anchored.Contains(Root(i)))];
    }

    private static void AddVertices(in Shape e, HashSet<Shape> map)
    {
        for (Explorer exv = new(e, ShapeType.Vertex); exv.More(); exv.Next())
        {
            map.Add(exv.Current());
        }
    }
}
