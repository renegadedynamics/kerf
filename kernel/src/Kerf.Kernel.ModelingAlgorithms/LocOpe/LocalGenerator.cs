// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Generator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// Glues a generated shape (<see cref="GeneratedShape"/>) on a shape: fuses the coplanar faces and the
/// collinear edges along the generating edges, and rebuilds the solid.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, it writes into the shapes it works with: point representations of the shape's vertices on the fused
/// edges, pcurves of the generated shape's edges on the rebuilt faces. So the shape and the generated shape's
/// sub-shapes are written in a working overlay (decision 1013, unit 10H5: <see cref="WorkingShapes"/>), and the result
/// shares the caller's unchanged sub-shapes. <see cref="ResultingShape"/> and <see cref="DescendantFace"/> freeze what they hand out; LocOpe_Gluer
/// reads the <c>Internal</c> forms and ends the working shapes itself.
/// </para>
/// <para>
/// OCCT iterates its maps of faces to fuse and edges to fuse keyed by the shapes' addresses, so the order in which
/// faces are rebuilt (and with it the order of the result's faces) depends on the run natively; the port takes them in
/// the order they are found.
/// </para>
/// </remarks>
internal sealed class LocalGenerator
{
    private Shape _shape = new();
    private GeneratedShape? _gen;
    private bool _done;
    private Shape _res = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _modShapes = new(ShapeHasher.Instance);
    private WorkingShapes _work = new();
    private bool _ownsWork = true;

    /// <summary>Empty constructor.</summary>
    public LocalGenerator()
    {
        _done = false;
    }

    /// <summary>Creates the algorithm on the shape <paramref name="s"/>.</summary>
    public LocalGenerator(in Shape s)
    {
        _shape = s;
        _done = false;
    }

    /// <summary>
    /// For LocOpe_Gluer: the algorithm on its working shape, sharing its working shapes (it ends them), with a generated
    /// shape built on working shapes.
    /// </summary>
    internal LocalGenerator(in Shape s, WorkingShapes work)
    {
        _work = work;
        _ownsWork = false;
        _shape = s;
        _done = false;
    }

    /// <summary>Initializes the algorithm on the shape <paramref name="s"/>.</summary>
    public void Init(in Shape s)
    {
        _shape = s;
        _done = false;
    }

    public bool IsDone => _done;

    /// <summary>Returns the new shape (frozen).</summary>
    public Shape ResultingShape() => WorkingShapes.Published(InternalResultingShape());

    /// <summary><see cref="ResultingShape"/> without freezing, for ported code.</summary>
    internal Shape InternalResultingShape()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _res;
    }

    /// <summary>Returns the initial shape (the caller's).</summary>
    public Shape Shape() => _shape;

    // The generated shape's answers
    private static NCollection_List<Shape> GeneratingEdges(GeneratedShape g) => g.GeneratingEdges();

    private static Edge Generated(GeneratedShape g, in Vertex v) => g.Generated(v);

    private static Face Generated(GeneratedShape g, in Edge e) => g.Generated(e);

    private static NCollection_List<Shape> OrientedFaces(GeneratedShape g) => g.OrientedFaces();

    /// <summary>
    /// Glues <paramref name="g"/> on the shape. Ends the working shapes: the result shares the caller's unchanged
    /// sub-shapes.
    /// </summary>
    public void Perform(GeneratedShape g)
    {
        if (_shape.IsNull)
        {
            throw new NullObjectException();
        }
        if (_ownsWork)
        {
            _work = new WorkingShapes();
        }
        using (_work.Enter())
        {
            InternalPerform(g);
        }
        if (!_ownsWork || !_done || !_work.IsWritten)
        {
            return;
        }

        var shapes = new List<Shape> { _res };
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _modShapes)
        {
            shapes.AddRange(item.Value);
        }
        _work.Finish(shapes);
        _res = _work.Final(_res);
        foreach (Shape key in _modShapes.Keys.ToList())
        {
            _modShapes.GetValueRef(key) = _work.Final(_modShapes.GetValue(key));
        }
    }

    // Perform on the working shapes
    internal void InternalPerform(GeneratedShape g)
    {
        if (_shape.IsNull)
        {
            throw new NullObjectException();
        }
        _done = false;
        _res.Nullify();
        _modShapes.Clear();
        _gen = g;

        NCollection_List<Shape> ledges = GeneratingEdges(g);

        // On genere une liste des faces a gauche du wire. Equivalent du LeftOf.
        // Attention : il faudra bien propager pour ne pas oublier des faces
        // a l`interieur

        Explorer exp = new(), exp2 = new(), exp3 = new();
        HashSet<Shape> theLeft = new(ShapeHasher.Instance); // Faces a gauche

        // Edges et vertex generateurs; OCCT iterates GEdg keyed by the edges' addresses, the port in their order
        IndexedSet<Shape> gEdg = new(ShapeHasher.Instance);
        HashSet<Shape> gVtx = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator itl = new(), itl2 = new();

        for (itl.Initialize(ledges); itl.More(); itl.Next())
        {
            Edge edg = ShapeCasts.Edge(itl.Value());

            gEdg.Add(edg);
            for (exp2.Init(edg, ShapeType.Vertex); exp2.More(); exp2.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp2.Current());
                if (!gVtx.Contains(vtx))
                {
                    gVtx.Add(vtx);
                }
            }
            for (exp2.Init(_shape, ShapeType.Face); exp2.More(); exp2.Next())
            {
                Face fac = ShapeCasts.Face(exp2.Current());
                for (exp3.Init(fac, ShapeType.Edge); exp3.More(); exp3.Next())
                {
                    if (exp3.Current().IsSame(edg) && exp3.Current().Orientation == edg.Orientation)
                    {
                        theLeft.Add(fac);
                        NCollection_List<Shape> emptylist = new();
                        if (!_modShapes.ContainsKey(fac))
                        {
                            _modShapes[fac] = emptylist;
                        }
                        break;
                    }
                }
                if (exp3.More())
                {
                    break;
                }
            }
        }

        IndexedMap<Shape, NCollection_List<Shape>> theEFMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, theEFMap);

        // OCCT iterates these maps keyed by the shapes' addresses; the port in insertion order
        Dictionary<Shape, NCollection_List<Shape>> theEEMap = new(ShapeHasher.Instance);
        Dictionary<Shape, NCollection_List<Shape>> theFFMap = new(ShapeHasher.Instance);
        HashSet<Shape> toRemove = new(ShapeHasher.Instance);

        // search for face fusions
        for (int itm = 0; itm < gEdg.Count; itm++)
        {
            Edge edg = ShapeCasts.Edge(gEdg[itm]);
            if (!theEFMap.ContainsKey(edg))
            {
                continue;
            }
            for (itl2.Initialize(theEFMap.GetValue(edg)); itl2.More(); itl2.Next())
            {
                if (!theLeft.Contains(itl2.Value()))
                {
                    break;
                }
            }
            if (!itl2.More())
            { // edge "interne" au shell, ou bord libre
            }
            else
            {
                Face fac = ShapeCasts.Face(itl2.Value());
                Face facbis = Generated(g, edg);
                if (ToFuse(fac, facbis))
                {
                    // On recherche si une face a deja fusionne avec facbis
                    bool facbisfound = false;
                    foreach (KeyValuePair<Shape, NCollection_List<Shape>> itf in theFFMap)
                    {
                        if (itf.Key.IsSame(fac))
                        {
                            continue;
                        }
                        for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                        {
                            if (itl.Value().IsSame(facbis))
                            {
                                facbisfound = true;
                                break;
                            }
                        }
                        if (facbisfound)
                        {
                            theFFMap.GetValue(itf.Key).Append(fac);
                            toRemove.Add(fac);
                            toRemove.Add(edg);
                            break;
                        }
                    }

                    if (!facbisfound)
                    {
                        if (!theFFMap.ContainsKey(fac))
                        {
                            NCollection_List<Shape> thelist = new();
                            theFFMap[fac] = thelist;
                        }
                        for (itl.Initialize(theFFMap.GetValue(fac)); itl.More(); itl.Next())
                        {
                            if (itl.Value().IsSame(facbis))
                            {
                                break;
                            }
                        }
                        if (!itl.More())
                        {
                            theFFMap.GetValue(fac).Append(facbis);
                        }
                        toRemove.Add(edg);
                        toRemove.Add(facbis);
                    }
                }
                else
                { // face generee par edg : on la marque.
                    //	myFFromE.Bind(edg,facbis);
                }
            }
        }

        // Il faut ici ajouter dans toRemove les edges de connexites entre faces
        // a fusionner avec une meme face de base

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itf in theFFMap)
        {
            for (itl.Initialize(itf.Value); itl.More(); itl.Next())
            {
                for (exp.Init(itl.Value(), ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge ed = ShapeCasts.Edge(exp.Current());
                    if (toRemove.Contains(ed))
                    {
                        continue;
                    }
                    for (itl2.Initialize(itf.Value); itl2.More(); itl2.Next())
                    {
                        if (!itl2.Value().IsSame(itl.Value()))
                        {
                            for (exp2.Init(itl2.Value(), ShapeType.Edge); exp2.More(); exp2.Next())
                            {
                                if (ed.IsSame(exp2.Current()))
                                {
                                    toRemove.Add(ed);
                                    break;
                                }
                            }
                            if (exp2.More())
                            {
                                break;
                            }
                        }
                    }
                }
            }
        }

        NCollection_List<Shape> rebuildFace = new();
        HashSet<Shape> mapTreated = new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> dontFuse = new(ShapeHasher.Instance);
        Orientation orient, orface;

        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itf in theFFMap)
        {
            Face fac = ShapeCasts.Face(itf.Key);
            for (exp.Init(fac, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                if (mapTreated.Contains(edg))
                {
                    continue; // on saute l`edge
                }

                mapTreated.Add(edg);
                for (exp2.Init(edg, ShapeType.Vertex); exp2.More(); exp2.Next())
                {
                    Vertex vtx = ShapeCasts.Vertex(exp2.Current());
                    if (gVtx.Contains(vtx))
                    {
                        Edge edgbis = Generated(g, vtx);

                        if ((edgbis.IsNull || Tool.Degenerated(edgbis)) ||
                            //	      toRemove.Contains(edgbis) ||
                            !ToFuse(edg, edgbis))
                        {
                            continue;
                        }
                        // a voir
                        if (BrepTools.IsReallyClosed(edg, fac))
                        {
                            if (!theEEMap.ContainsKey(edg))
                            {
                                NCollection_List<Shape> thelist1 = new();
                                theEEMap[edg] = thelist1;
                                theEEMap.GetValue(edg).Append(edgbis);
                                toRemove.Add(edgbis); // toujours vrai pour edge double
                                bool fuseEdge = true;
                                Exploration.Vertices(edg, out Vertex Vf, out Vertex Vl);
                                bool connectLast = (Vl.IsSame(vtx));
                                for (exp3.Init(fac.Oriented(Forward), ShapeType.Edge); exp3.More(); exp3.Next())
                                {
                                    Edge eee = ShapeCasts.Edge(exp3.Current());
                                    orient = eee.Orientation;
                                    if (!eee.IsSame(edg))
                                    {
                                        Exploration.Vertices(eee, out Vf, out Vl);
                                        if ((Vf.IsSame(vtx) || Vl.IsSame(vtx)) && !toRemove.Contains(eee))
                                        {
                                            fuseEdge = false;
                                            // On recherche celui qu`il ne faut pas fusionner

                                            if ((Vf.IsSame(vtx) && orient == Forward)
                                                || (Vl.IsSame(vtx) && orient == Reversed))
                                            {
                                                if (connectLast)
                                                {
                                                    dontFuse[edg] = fac.Oriented(Forward);
                                                }
                                                else
                                                {
                                                    dontFuse[edg] = fac.Oriented(Reversed);
                                                }
                                            }
                                            else
                                            {
                                                if (connectLast)
                                                {
                                                    dontFuse[edg] = fac.Oriented(Reversed);
                                                }
                                                else
                                                {
                                                    dontFuse[edg] = fac.Oriented(Forward);
                                                }
                                            }
                                            break;
                                        }
                                    }
                                }
                                if (fuseEdge)
                                {
                                    toRemove.Add(vtx);
                                }
                            }
                        }

                        else
                        {
                            /*  A VOIR
                                  if (!BRep_Tool::IsClosed(edg,fac)) {
                            */
                            if (!theEEMap.ContainsKey(edg))
                            {
                                NCollection_List<Shape> thelist2 = new();
                                theEEMap[edg] = thelist2;
                            }
                            theEEMap.GetValue(edg).Append(edgbis);
                            NCollection_List<Shape> L = theEEMap.GetValue(edg);
                            NCollection_List<Shape>.Iterator lit = new(L);
                            bool ok = true;
                            for (; lit.More(); lit.Next())
                            {
                                if (lit.Value().IsSame(edgbis))
                                {
                                    ok = false;
                                    break;
                                }
                            }
                            if (ok)
                            {
                                theEEMap.GetValue(edg).Append(edgbis);
                            }

                            itl.Initialize(theEFMap.GetValue(edg));
                            bool fuseEdge = ToFuse(edg, fac, vtx, toRemove);
                            if (!fuseEdge)
                            {
                                dontFuse[edg] = fac;
                            }
                            else
                            {
                                for (; itl.More(); itl.Next())
                                {
                                    if (!itl.Value().IsSame(fac))
                                    {
                                        if (theFFMap.ContainsKey(itl.Value()))
                                        {
                                            fuseEdge = ToFuse(edg, ShapeCasts.Face(itl.Value()), vtx, toRemove);
                                            // edge a fusionner
                                            if (fuseEdge)
                                            {
                                                toRemove.Add(vtx);
                                            }
                                            else
                                            {
                                                if (toRemove.Contains(vtx))
                                                {
                                                    toRemove.Remove(vtx);
                                                }
                                                dontFuse[edg] = itl.Value();
                                            }
                                        }
                                        else
                                        { // on marque comme face a reconstruire
                                            rebuildFace.Append(itl.Value());
                                            if (toRemove.Contains(vtx))
                                            {
                                                toRemove.Remove(vtx);
                                            }
                                            dontFuse[edg] = itl.Value();
                                        }

                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        for (itl.Initialize(rebuildFace); itl.More(); itl.Next())
        {
            NCollection_List<Shape> thelist3 = new();
            theFFMap[itl.Value()] = thelist3;
        }

        BrepBuilder b = new();
        Face newface;
        Edge newedg;
        Vertex newvtx = new();
        Placement loc;
        double tol = 0.0, prm = 0.0, f, l, uminc = 0.0, umaxc = 0.0;
        Point2D pf, pl;

        Surface s;
        PlaneSurface? p;
        Curve3D c = null!;

        // Fusion des edges
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> ite in theEEMap)
        {
            bool keepNewEdge = false;
            Edge edg = ShapeCasts.Edge(ite.Key);
            Tool.Range(edg, out f, out l);
            Shape localEdge = edg.EmptyCopied();
            newedg = ShapeCasts.Edge(localEdge);
            newedg.Orient(Forward);
            for (exp.Init(edg.Oriented(Forward), ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                prm = Tool.Parameter(vtx, edg);

                newvtx.Nullify();
                for (itl.Initialize(theEEMap.GetValue(edg)); itl.More(); itl.Next())
                {
                    Edge edgbis = ShapeCasts.Edge(itl.Value());
                    for (exp2.Init(edgbis, ShapeType.Vertex); exp2.More(); exp2.Next())
                    {
                        if (exp2.Current().IsSame(vtx))
                        {
                            break;
                        }
                    }
                    if (exp2.More())
                    {
                        for (exp2.ReInit(); exp2.More(); exp2.Next())
                        {
                            if (!exp2.Current().IsSame(vtx))
                            {
                                newvtx = ShapeCasts.Vertex(exp2.Current());
                                prm = NewParameter(edg, vtx, newedg, newvtx);
                                break;
                            }
                        }
                        break;
                    }
                }

                if (toRemove.Contains(vtx) || (prm < l && prm > f))
                {
                    b.Add(newedg, newvtx.Oriented(vtx.Orientation));
                    tol = Tool.Tolerance(newvtx);
                    b.UpdateVertex(newvtx, prm, newedg, tol);
                    toRemove.Add(itl.Value()); // i-e edgbis
                    keepNewEdge = true;
                }
                else
                {
                    b.Add(newedg, vtx.Oriented(vtx.Orientation));
                    tol = Tool.Tolerance(vtx);
                    b.UpdateVertex(vtx, prm, newedg, tol);
                }
            }
            if (keepNewEdge)
            {
                NCollection_List<Shape> emptylist = new();
                if (!_modShapes.ContainsKey(edg))
                {
                    _modShapes[edg] = emptylist;
                }
                _modShapes.GetValue(edg).Append(newedg);
                toRemove.Add(edg);
            }
        }

        HashSet<Shape> edgAdded = new(ShapeHasher.Instance);

        // Fusion des faces, ou reconstruction
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itf in theFFMap)
        {
            Face fac = ShapeCasts.Face(itf.Key);
            bool modFace = false;
            NCollection_List<Shape> listofedg = new();

            edgAdded.Clear();

            for (exp.Init(_shape, ShapeType.Face); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(fac))
                {
                    break;
                }
            }
            orface = exp.Current().Orientation;
            Shape localFaceEmptyCopied = fac.EmptyCopied();
            newface = ShapeCasts.Face(localFaceEmptyCopied);
            newface.Orient(Forward);
            s = Tool.Surface(fac)!;
            if (s.GetType() == typeof(TrimmedSurface))
            {
                s = ((TrimmedSurface)s).BasisSurface;
            }
            p = s as PlaneSurface;
            Wire wir;
            for (exp.Init(fac.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
            {
                wir = ShapeCasts.Wire(exp.Current());
                for (exp2.Init(wir, ShapeType.Edge); exp2.More(); exp2.Next())
                {
                    Edge edg = ShapeCasts.Edge(exp2.Current());
                    if (toRemove.Contains(edg) || _modShapes.ContainsKey(edg))
                    {
                        break;
                    }
                }
                if (!exp2.More())
                { // wire non modifie
                    for (exp2.Init(wir, ShapeType.Edge); exp2.More(); exp2.Next())
                    {
                        listofedg.Append(exp2.Current());
                    }
                }
                else
                {
                    if (!modFace)
                    {
                        // Petit truc crad pour les p-curves sur les cylindres...
                        if (p is null)
                        {
                            BrepTools.UVBounds(fac, out uminc, out umaxc, out _, out _);
                        }

                        // premier passage : on met les wires non touches des faces
                        // en vis a vis

                        for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                        {
                            Face facbis = ShapeCasts.Face(itl.Value());
                            NCollection_List<Shape> gFaces = OrientedFaces(g);
                            for (itl2.Initialize(gFaces); itl2.More(); itl2.Next())
                            {
                                if (itl2.Value().IsSame(facbis))
                                {
                                    break;
                                }
                            }
                            if (itl2.More())
                            {
                                orient = itl2.Value().Orientation;
                                facbis.Orient(orient);
                            }
                            else
                            { // on fusionne avec une autre face du shape...
                                for (exp2.Init(_shape, ShapeType.Face); exp2.More(); exp2.Next())
                                {
                                    if (exp2.Current().IsSame(facbis))
                                    {
                                        facbis.Orient(exp2.Current().Orientation);
                                        break;
                                    }
                                }
                            }

                            for (exp3.Init(facbis, ShapeType.Wire); exp3.More(); exp3.Next())
                            {
                                for (exp2.Init(exp3.Current(), ShapeType.Edge); exp2.More(); exp2.Next())
                                {
                                    if (toRemove.Contains(exp2.Current()))
                                    {
                                        break;
                                    }
                                }
                                if (!exp2.More())
                                {
                                    Wire theNew = new();
                                    b.MakeWire(ref theNew); // FORWARD
                                    for (exp2.ReInit(); exp2.More(); exp2.Next())
                                    {
                                        Edge edg = ShapeCasts.Edge(exp2.Current());

                                        orient = global::Kerf.Engine.Topology.TopAbs.Compose(orface, edg.Orientation);
                                        listofedg.Append(edg.Oriented(orient));
                                        edgAdded.Add(edg);
                                        if (p is null)
                                        {
                                            // on met les courbes 2d si on n`est pas sur un plan
                                            // on ne devrait pas avoir de pb d`edge de couture.
                                            tol = Tool.Tolerance(edg);
                                            c = Tool.Curve(edg, out loc, out f, out l)!;
                                            if (!loc.IsIdentity)
                                            {
                                                Geometry3D gg = c.Transformed(loc.Transformation);
                                                c = (Curve3D)gg;
                                            }
                                            if (c.GetType() == typeof(TrimmedCurve3D))
                                            {
                                                c = ((TrimmedCurve3D)c).BasisCurve;
                                            }

                                            Curve2D c2d0 = Translatable(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, f, l, s, ref tol)!);

                                            // Tentative de recalage dans la facette
                                            pf = c2d0.Value(f);
                                            pl = c2d0.Value(l);
                                            double tttol = global::Kerf.Engine.Foundation.Precision.Angular();
                                            while (Min(pf.X, pl.X) >= umaxc - tttol)
                                            {
                                                c2d0.Translate(new Vector2D(-2.0 * Math.PI, 0));
                                                pf = c2d0.Value(f);
                                                pl = c2d0.Value(l);
                                            }

                                            while (Max(pf.X, pl.X) <= uminc + tttol)
                                            {
                                                c2d0.Translate(new Vector2D(2.0 * Math.PI, 0));
                                                pf = c2d0.Value(f);
                                                pl = c2d0.Value(l);
                                            }

                                            if (!BrepTools.IsReallyClosed(edg, facbis))
                                            {
                                                b.UpdateEdge(edg, c2d0, newface, tol);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        modFace = true;
                    }

                    // reconstruction du wire

                    Curve2D? c2d, c2d1 = null;

                    for (exp2.Init(wir, ShapeType.Edge); exp2.More(); exp2.Next())
                    {
                        Edge edg = ShapeCasts.Edge(exp2.Current());
                        orient = edg.Orientation;
                        if (!toRemove.Contains(edg) && !theEEMap.ContainsKey(edg))
                        {
                            listofedg.Append(edg);
                        }
                        else if (_modShapes.ContainsKey(edg) || theEEMap.ContainsKey(edg))
                        {
                            if (_modShapes.ContainsKey(edg))
                            {
                                newedg = ShapeCasts.Edge(_modShapes.GetValue(edg).First());
                            }
                            else
                            {
                                newedg = edg;
                            }
                            listofedg.Append(newedg.Oriented(orient));
                            c = Tool.Curve(newedg, out loc, out f, out l)!;
                            if (!loc.IsIdentity)
                            {
                                Geometry3D gg = c.Transformed(loc.Transformation);
                                c = (Curve3D)gg;
                            }
                            if (c.GetType() == typeof(TrimmedCurve3D))
                            {
                                c = ((TrimmedCurve3D)c).BasisCurve;
                            }
                            if (p is null)
                            { // on met les courbes 2d si on n`est pas
                                // sur un plan
                                Orientation oredonfafw = orient;
                                tol = Tool.Tolerance(newedg);
                                Shape localEdge = edg.Oriented(oredonfafw);
                                Shape localFace = fac.Oriented(Forward);
                                c2d = Tool.CurveOnSurface(ShapeCasts.Edge(localEdge), ShapeCasts.Face(localFace), out f, out l);

                                if (!BrepTools.IsReallyClosed(edg, fac))
                                {
                                    b.UpdateEdge(newedg, c2d, newface, tol);
                                }
                                else if (c2d1 is null)
                                {
                                    c2d1 = c2d;
                                }
                                else
                                {
                                    if (orient == Forward)
                                    {
                                        b.UpdateEdge(newedg, c2d, c2d1, newface, tol);
                                    }
                                    else
                                    {
                                        b.UpdateEdge(newedg, c2d1, c2d, newface, tol);
                                    }
                                }
                            }
                            bool addPart = false;
                            if (dontFuse.ContainsKey(edg))
                            {
                                if (!BrepTools.IsReallyClosed(edg, fac))
                                {
                                    if (dontFuse.GetValue(edg).IsSame(fac))
                                    {
                                        if (_modShapes.ContainsKey(edg))
                                        {
                                            addPart = true;
                                        }
                                    }
                                    else if (!toRemove.Contains(edg))
                                    {
                                        addPart = true;
                                    }
                                }
                                else
                                {
                                    if (_modShapes.ContainsKey(edg))
                                    { // edg raccourci
                                        if (orient == dontFuse.GetValue(edg).Orientation)
                                        {
                                            addPart = true;
                                        }
                                    }
                                    else
                                    {
                                        if (orient == global::Kerf.Engine.Topology.TopAbs.Reverse(dontFuse.GetValue(edg).Orientation))
                                        {
                                            addPart = true;
                                        }
                                    }
                                }
                            }
                            if (addPart)
                            {
                                itl2.Initialize(theEEMap.GetValue(edg));
                                Vector3D dir1, dir2;
                                for (; itl2.More(); itl2.Next())
                                {
                                    if (edgAdded.Contains(itl2.Value()))
                                    {
                                        continue;
                                    }
                                    Edge edgbis = ShapeCasts.Edge(itl2.Value());
                                    Iterator it1 = new(newedg), it2 = new();
                                    for (; it1.More(); it1.Next())
                                    {
                                        for (it2.Initialize(edgbis); it2.More(); it2.Next())
                                        {
                                            if (it1.Value().IsSame(it2.Value()))
                                            {
                                                break;
                                            }
                                        }
                                        if (it2.More())
                                        {
                                            break;
                                        }
                                    }

                                    if (it1.More())
                                    {
                                        double prmvt = Tool.Parameter(ShapeCasts.Vertex(it1.Value()), newedg);
                                        c.D1(prmvt, out Point3D ptbid, out dir1);

                                        c = Tool.Curve(edgbis, out loc, out f, out l)!;
                                        if (!loc.IsIdentity)
                                        {
                                            Geometry3D gg = c.Transformed(loc.Transformation);
                                            c = (Curve3D)gg;
                                        }
                                        if (c.GetType() == typeof(TrimmedCurve3D))
                                        {
                                            c = ((TrimmedCurve3D)c).BasisCurve;
                                        }
                                        prmvt = Tool.Parameter(ShapeCasts.Vertex(it1.Value()), edgbis);
                                        c.D1(prmvt, out ptbid, out dir2);
                                    }
                                    else
                                    {
                                        dir1 = dir2 = new Vector3D(1, 0, 0);
                                    }
                                    edgAdded.Add(edgbis);
                                    if (dir1.Dot(dir2) < 0.0)
                                    {
                                        listofedg.Append(edgbis.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(orient)));
                                    }
                                    else
                                    {
                                        listofedg.Append(edgbis.Oriented(orient));
                                    }

                                    if (p is null)
                                    {
                                        // on met les courbes 2d si on n`est pas sur un plan
                                        // C est la courbe de edgbis, f et l s`y rapportent
                                        Curve2D ptc = Translatable(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, f, l, s, ref tol)!);
                                        if (s.IsUPeriodic)
                                        {
                                            double uref;
                                            if (dontFuse.ContainsKey(edg))
                                            {
                                                Orientation oredge = dontFuse.GetValue(edg).Orientation;
                                                if (_modShapes.ContainsKey(edg))
                                                { // edge raccourci...
                                                    Shape localShape = edg.Oriented(oredge);
                                                    Shape localFace = fac.Oriented(Forward);
                                                    Tool.UVPoints(ShapeCasts.Edge(localShape), ShapeCasts.Face(localFace), out pf, out pl);
                                                }
                                                else
                                                {
                                                    Shape localShape = edg.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(oredge));
                                                    Shape localFace = fac.Oriented(Forward);
                                                    Tool.UVPoints(ShapeCasts.Edge(localShape), ShapeCasts.Face(localFace), out pf, out pl);
                                                }
                                                uref = pf.X;
                                            }
                                            else
                                            {
                                                Tool.UVPoints(edg, fac, out pf, out pl);
                                                uref = pf.X;
                                            }

                                            double newU = (ptc.Value(f)).X;

                                            if (Math.Abs(newU - uref) > Epsilon(s.UPeriod))
                                            {
                                                ptc.Translate(new Vector2D((uref - newU), 0.0));
                                            }
                                        }

                                        b.UpdateEdge(edgbis, ptc, newface, tol);
                                    }
                                }
                            }
                        }
                    }

                    // Recuperation des edges sur les faces a fusionner.

                    // ICICICICI

                    bool includeinw = false;
                    for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                    {
                        Face facbis = ShapeCasts.Face(itl.Value());
                        bool genface = true;
                        NCollection_List<Shape> gFaces = OrientedFaces(g);
                        for (itl2.Initialize(gFaces); itl2.More(); itl2.Next())
                        {
                            if (itl2.Value().IsSame(facbis))
                            {
                                break;
                            }
                        }
                        if (itl2.More())
                        {
                            orient = itl2.Value().Orientation;
                            facbis.Orient(orient);
                        }
                        else
                        { // on fusionne avec une autre face du shape...
                            genface = false;
                            for (exp2.Init(_shape, ShapeType.Face); exp2.More(); exp2.Next())
                            {
                                if (exp2.Current().IsSame(facbis))
                                {
                                    facbis.Orient(exp2.Current().Orientation);
                                    break;
                                }
                            }
                        }

                        for (exp3.Init(facbis, ShapeType.Wire); exp3.More(); exp3.Next())
                        {
                            for (exp2.Init(exp3.Current(), ShapeType.Edge); exp2.More(); exp2.Next())
                            {
                                if (toRemove.Contains(exp2.Current()))
                                {
                                    break;
                                }
                            }
                            if (genface)
                            {
                                includeinw = true;
                            }
                            else
                            {
                                bool isIncludedInW = false;
                                if (exp2.More())
                                {
                                    for (exp2.ReInit(); exp2.More(); exp2.Next())
                                    {
                                        if (!toRemove.Contains(exp2.Current()))
                                        {
                                            continue;
                                        }
                                        Exploration.Vertices(ShapeCasts.Edge(exp2.Current()), out Vertex vf, out Vertex vl);
                                        Explorer exp4 = new();
                                        for (exp4.Init(wir, ShapeType.Vertex); exp4.More(); exp4.Next())
                                        {
                                            if (exp4.Current().IsSame(vf) || exp4.Current().IsSame(vl))
                                            {
                                                isIncludedInW = true;
                                                break;
                                            }
                                        }
                                        if (isIncludedInW)
                                        {
                                            break;
                                        }
                                    }
                                }
                                // OCCT computes isIncludedInW and doesn't use it (includeinw stays as it was)
                            }

                            if (!includeinw)
                            {
                                continue;
                            }

                            for (exp2.ReInit(); exp2.More(); exp2.Next())
                            {
                                Edge edg = ShapeCasts.Edge(exp2.Current());
                                if (!toRemove.Contains(edg) && !edgAdded.Contains(edg))
                                {
                                    orient = global::Kerf.Engine.Topology.TopAbs.Compose(orface, edg.Orientation);
                                    listofedg.Append(edg.Oriented(orient));
                                    edgAdded.Add(edg);
                                    if (p is null)
                                    {
                                        // on met les courbes 2d si on n`est pas sur un plan
                                        // on ne devrait pas avoir de pb d`edge de couture.
                                        tol = Tool.Tolerance(edg);
                                        c = Tool.Curve(edg, out loc, out f, out l)!;
                                        if (!loc.IsIdentity)
                                        {
                                            Geometry3D gg = c.Transformed(loc.Transformation);
                                            c = (Curve3D)gg;
                                        }
                                        if (c.GetType() == typeof(TrimmedCurve3D))
                                        {
                                            c = ((TrimmedCurve3D)c).BasisCurve;
                                        }

                                        c2d = Translatable(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, f, l, s, ref tol)!);

                                        // Tentative de recalage dans la facette
                                        pf = c2d.Value(f);
                                        pl = c2d.Value(l);
                                        double tttol = global::Kerf.Engine.Foundation.Precision.Angular();
                                        while (Min(pf.X, pl.X) >= umaxc - tttol)
                                        {
                                            c2d.Translate(new Vector2D(-2.0 * Math.PI, 0));
                                            pf = c2d.Value(f);
                                            pl = c2d.Value(l);
                                        }

                                        while (Max(pf.X, pl.X) <= uminc + tttol)
                                        {
                                            c2d.Translate(new Vector2D(2.0 * Math.PI, 0));
                                            pf = c2d.Value(f);
                                            pl = c2d.Value(l);
                                        }

                                        if (!BrepTools.IsReallyClosed(edg, facbis))
                                        {
                                            b.UpdateEdge(edg, c2d, newface, tol);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (!listofedg.IsEmpty())
            {
                Loop L = new();
                L.Init(newface);
                L.AddConstEdges(listofedg);
                L.Perform();
                L.WiresToFaces();
                NCollection_List<Shape> listoffaces = L.NewFaces();
                toRemove.Add(fac);
                _modShapes[fac] = new NCollection_List<Shape>(listoffaces);
                for (itl.Initialize(itf.Value); itl.More(); itl.Next())
                {
                    _modShapes[itl.Value()] = new NCollection_List<Shape>(listoffaces);
                }
            }
        }

        // JAG 16.09.96 : on utilise LocOpe_BuildShape (OCCT's former construction of the shell is commented out)

        // 06.11.96
        // Debug temporaire. Il faudra prevoir un syntaxe de BuildShape
        // qui impose une ori de certaines faces.

        Face faceRefOri = new();

        NCollection_List<Shape> lfres = new();
        for (exp.Init(_shape, ShapeType.Face); exp.More(); exp.Next())
        {
            Face fac = ShapeCasts.Face(exp.Current());
            if (!theLeft.Contains(fac))
            {
                if (!toRemove.Contains(exp.Current()))
                {
                    lfres.Append(fac);
                    if (!_modShapes.ContainsKey(fac))
                    {
                        NCollection_List<Shape> emptylist = new();
                        _modShapes[fac] = emptylist;
                    }
                    _modShapes.GetValue(fac).Append(fac);
                    if (faceRefOri.IsNull)
                    {
                        faceRefOri = fac;
                    }
                }
                else if (_modShapes.ContainsKey(fac))
                {
                    lfres.Append(_modShapes.GetValue(fac).First().Oriented(fac.Orientation));
                }
            }
        }

        Orientation orsolid = _shape.Orientation;
        for (itl.Initialize(OrientedFaces(g)); itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());
            if (toRemove.Contains(fac))
            {
                continue;
            }

            if (orsolid == Forward)
            {
                lfres.Append(fac);
            }
            else
            {
                lfres.Append(fac.Reversed());
            }
            if (!_modShapes.ContainsKey(fac))
            {
                NCollection_List<Shape> emptylist = new();
                _modShapes[fac] = emptylist;
            }
            _modShapes.GetValue(fac).Append(fac);
        }

        BuildShape bs = new(lfres);
        _res = bs.InternalShape();
        // Suite debug du 06.11.96
        if (_res.Type == ShapeType.Solid)
        {
            for (exp.Init(_res, ShapeType.Face); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(faceRefOri))
                {
                    break;
                }
            }
            if (exp.More() && exp.Current().Orientation != faceRefOri.Orientation)
            {
                // Si un seul Shell, on change son orientation
                Solid newSol = new();
                b.MakeSolid(ref newSol);
                exp.Init(_res, ShapeType.Shell);
                b.Add(newSol, exp.Current().Reversed());
                _res.Nullify();
                _res = newSol;
            }
        }

        // recodage des regularites qui existaient sur le shape colle

        _done = true;
    }

    /// <summary>
    /// Returns the descendant faces of <paramref name="f"/> (frozen). <paramref name="f"/> may belong to the original
    /// shape or to the "generated" shape. The list may hold a null face (when <paramref name="f"/> disappears).
    /// </summary>
    public NCollection_List<Shape> DescendantFace(in Face f) =>
        WorkingShapes.Published(InternalDescendantFace(f));

    /// <summary>
    /// <see cref="DescendantFace"/> without freezing, for ported code.
    /// </summary>
    internal NCollection_List<Shape> InternalDescendantFace(in Face f)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _modShapes.GetValue(f);
    }

    // Kerf: OCCT translates the projected pcurve in place; a frozen one (shared) is translated on a copy
    private static Curve2D Translatable(Curve2D curve) =>
        curve.IsFrozen ? (Curve2D)curve.Copy() : curve;

    private static bool ToFuse(in Face f1, in Face f2)
    {
        if (f1.IsNull || f2.IsNull)
        {
            return false;
        }

        double tollin = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolang = global::Kerf.Engine.Foundation.Precision.Angular();

        Surface s1 = Tool.Surface(f1, out Placement loc1)!;
        Surface s2 = Tool.Surface(f2, out Placement loc2)!;

        if (s1.GetType() == typeof(TrimmedSurface))
        {
            s1 = ((TrimmedSurface)s1).BasisSurface;
        }

        if (s2.GetType() == typeof(TrimmedSurface))
        {
            s2 = ((TrimmedSurface)s2).BasisSurface;
        }

        if (s1.GetType() != s2.GetType())
        {
            return false;
        }

        bool valRet = false;
        if (s1.GetType() == typeof(PlaneSurface))
        {
            Plane3D pl1 = ((PlaneSurface)s1).Plane;
            Plane3D pl2 = ((PlaneSurface)s2).Plane;

            pl1 = pl1.Transformed(loc1.Transformation);
            pl2 = pl2.Transformed(loc2.Transformation);

            if (pl1.Position.IsCoplanar(pl2.Position, tollin, tolang))
            {
                valRet = true;
            }
        }

        return valRet;
    }

    private static bool ToFuse(in Edge e1, in Edge e2)
    {
        if (e1.IsNull || e2.IsNull)
        {
            return false;
        }

        double tollin = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolang = global::Kerf.Engine.Foundation.Precision.Angular();

        Curve3D c1 = Tool.Curve(e1, out Placement loc1, out _, out _)!;
        if (!loc1.IsIdentity)
        {
            Geometry3D cc1 = c1.Transformed(loc1.Transformation);
            c1 = (Curve3D)cc1;
        }

        Curve3D c2 = Tool.Curve(e2, out Placement loc2, out _, out _)!;
        if (!loc2.IsIdentity)
        {
            Geometry3D cc2 = c2.Transformed(loc2.Transformation);
            c2 = (Curve3D)cc2;
        }

        if (c1.GetType() == typeof(TrimmedCurve3D))
        {
            c1 = ((TrimmedCurve3D)c1).BasisCurve;
        }
        if (c2.GetType() == typeof(TrimmedCurve3D))
        {
            c2 = ((TrimmedCurve3D)c2).BasisCurve;
        }

        if (c1.GetType() != c2.GetType())
        {
            return false;
        }

        bool valRet = false;
        if (c1.GetType() == typeof(LineCurve3D))
        {
            Line3D li1 = ((LineCurve3D)c1).Line;
            Line3D li2 = ((LineCurve3D)c2).Line;

            if (li1.Position.IsCoaxial(li2.Position, tolang, tollin))
            {
                valRet = true;
            }
        }

        return valRet;
    }

    private static bool ToFuse(in Edge e, in Face f, in Vertex v, HashSet<Shape> toRemove)
    {
        Explorer exp = new();
        for (exp.Init(f, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge eee = ShapeCasts.Edge(exp.Current());
            if (!eee.IsSame(e))
            {
                Exploration.Vertices(eee, out Vertex vf, out Vertex vl);
                if ((vf.IsSame(v) || vl.IsSame(v)) && !toRemove.Contains(eee))
                {
                    return false;
                }
            }
        }
        return true;
    }

    private static double NewParameter(in Edge edg, in Vertex vtx, in Edge newEdg, in Vertex newVtx)
    {
        Point3D p = Tool.Pnt(newVtx);

        Curve3D c = Tool.Curve(edg, out Placement loc, out _, out _)!;
        if (!loc.IsIdentity)
        {
            Geometry3D gg = c.Transformed(loc.Transformation);
            c = (Curve3D)gg;
        }
        if (c.GetType() == typeof(TrimmedCurve3D))
        {
            c = ((TrimmedCurve3D)c).BasisCurve;
        }

        if (c.GetType() == typeof(LineCurve3D))
        {
            return global::Kerf.Engine.Geometry.ElCLib.Parameter(((LineCurve3D)c).Line, p);
        }
        else if (c.GetType() == typeof(CircleCurve3D))
        {
            double prm = global::Kerf.Engine.Geometry.ElCLib.Parameter(((CircleCurve3D)c).Circle, p);
            // Vtx vient d`une exploration de Edg orientee FORWARD

            Orientation orient = global::Kerf.Engine.Topology.TopAbs.Reverse(vtx.Orientation);
            if (orient == Forward || orient == Reversed)
            {
                Explorer exp = new(newEdg.Oriented(Forward), ShapeType.Vertex);
                for (; exp.More(); exp.Next())
                {
                    if (exp.Current().Orientation == orient)
                    {
                        break;
                    }
                }
                if (exp.More())
                {
                    double prmmax = Tool.Parameter(ShapeCasts.Vertex(exp.Current()), newEdg);
                    if (Math.Abs(prmmax - prm) <= Epsilon(2.0 * Math.PI))
                    {
                        if (orient == Reversed)
                        {
                            prm -= 2.0 * Math.PI;
                        }
                        else
                        {
                            prm += 2.0 * Math.PI;
                        }
                    }
                }
            }
            return prm;
        }
        return 0;
    }
}
