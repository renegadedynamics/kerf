// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Spliter.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Spliter.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_Spliter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
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
/// Splits a shape by the edges and wires of a <see cref="WiresOnShape"/> lying on it, and tells the faces on
/// their left.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, the splitting writes into the shape and the splitting edges (vertex tolerances and point
/// representations, continuities). It works in the wires-on-shape's working overlay (decision 1013, unit 10H5); at the
/// end of <see cref="Perform"/> the result holds new TShapes only for the caller's sub-shapes that changed. The shapes it hands out are frozen; ported callers (LocOpe_Gluer) read the <c>Internal</c> forms, and
/// end the working shapes themselves.
/// </para>
/// </remarks>
internal sealed class Spliter
{
    private Shape _shape = new();
    private bool _done;
    private Shape _res = new();
    // OCCT iterates this map keyed by the shapes' addresses; each entry is handled alone (the order isn't observable)
    private readonly Dictionary<Shape, NCollection_List<Shape>> _map = new(ShapeHasher.Instance);
    private readonly NCollection_List<Shape> _dLeft = new();
    private readonly NCollection_List<Shape> _left = new();
    private WorkingShapes? _work;
    private Shape _input = new();

    /// <summary>Empty constructor.</summary>
    public Spliter()
    {
        _done = false;
    }

    /// <summary>Creates the algorithm on the shape <paramref name="s"/>.</summary>
    public Spliter(in Shape s)
    {
        _input = s;
        _shape = s;
        _done = false;
    }

    /// <summary>Initializes the algorithm on the shape <paramref name="s"/>.</summary>
    public void Init(in Shape s)
    {
        _input = s;
        _shape = s;
        _done = false;
    }

    /// <summary>
    /// Splits the shape by the edges of <paramref name="pw"/>, which must be on the same shape. Ends the working
    /// shapes: the result shares the caller's unchanged sub-shapes.
    /// </summary>
    public void Perform(WiresOnShape pw)
    {
        // Kerf: in the working overlay of the wires on the shape, which write into it (unit 10H5)
        using WorkingOverlay.Scope scope = pw.WorkingShapes().Enter();
        InternalPerform(pw);
        if (!_done || _work is null || !_work.IsWritten)
        {
            return;
        }

        var shapes = new List<Shape> { _res };
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> item in _map)
        {
            shapes.AddRange(item.Value);
        }
        shapes.AddRange(_left);
        _work.Finish(shapes);
        _res = _work.Final(_res);
        foreach (Shape key in _map.Keys.ToList())
        {
            _map.GetValueRef(key) = _work.Final(_map.GetValue(key));
        }
        NCollection_List<Shape> dLeft = _work.Final(_dLeft), left = _work.Final(_left);
        _dLeft.Assign(dLeft);
        _left.Assign(left);
    }

    /// <summary>
    /// <see cref="Perform"/> for ported operations that own the working shapes (LocOpe_Gluer): the result is left
    /// unfrozen, on the working shapes, which the caller ends.
    /// </summary>
    internal void InternalPerform(WiresOnShape pw)
    {
        if (_shape.IsNull)
        {
            throw new NullObjectException();
        }
        _done = false;
        _map.Clear();
        _res.Nullify();
        // Kerf: the wires on the shape's working overlay (unit 10H5)
        _work = pw.WorkingShapes();
        _shape = _input;

        Put(_shape, _map);

        HashSet<Shape> mapV = new(ShapeHasher.Instance), mapE = new(ShapeHasher.Instance);
        Dictionary<Shape, Shape> edgOnEdg = new(ShapeHasher.Instance);
        IndexedMap<Shape, NCollection_List<Shape>> mapFE = new(ShapeHasher.Instance);
        Explorer exp = new(), exp2 = new();

        // 1ere etape : substitution des vertex

        NCollection_List<Shape> lsubs = new();
        Substitution theSubs = new();
        BrepBuilder bb = new();

        for (pw.InitEdgeIterator(); pw.MoreEdge(); pw.NextEdge())
        {
            Edge edg = pw.InternalEdge();
            mapE.Add(edg);
            for (exp.Init(edg, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (!mapV.Contains(vtx))
                {
                    if (pw.InternalOnVertex(vtx, out Vertex vb))
                    {
                        mapV.Add(vtx);
                        if (vtx.IsSame(vb))
                        {
                            continue;
                        }
                        lsubs.Clear();
                        Vertex vsub = ShapeCasts.Vertex(vtx.Oriented(Forward));
                        Point3D p1 = Tool.Pnt(vsub), p2 = Tool.Pnt(vb);
                        double d = p1.DistanceTo(p2);
                        d = d + Tool.Tolerance(vb);
                        bb.UpdateVertex(vsub, d);
                        lsubs.Append(vsub);
                        theSubs.Substitute(vb.Oriented(Forward), lsubs);
                    }
                }
            }
        }

        theSubs.Build(_shape);
        if (theSubs.IsCopied(_shape))
        {
            // on n`a fait que des substitutions de vertex. Donc chaque element
            // est remplace par lui meme ou par un seul element du meme type.
            foreach (Shape key in _map.Keys.ToList())
            {
                if (theSubs.IsCopied(key))
                {
                    NCollection_List<Shape> lsub = theSubs.InternalCopy(key);
                    // OCCT_DEBUG check left out (Standard_ConstructionError if lsub.Extent() != 1)
                    _map.GetValue(key).Clear();
                    _map.GetValue(key).Append(lsub.First());
                }
            }
        }

        _res = _map.GetValue(_shape).First();
        LocalSplitShape theCFace = new(_res, _work);

        // Adds every vertices lying on an edge of the shape, and prepares
        // work to rebuild wires on each face
        double prm;

        HashSet<Shape> theFacesWithSection = new(ShapeHasher.Instance);
        for (pw.InitEdgeIterator(); pw.MoreEdge(); pw.NextEdge())
        {
            Edge edg = pw.InternalEdge();
            for (exp.Init(edg, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                if (!mapV.Contains(vtx))
                {
                    mapV.Add(vtx);
                    if (pw.InternalOnEdge(vtx, edg, out Edge ed, out prm))
                    {
                        // on devrait verifier que le vtx n`existe pas deja sur l`edge
                        if (!_map.ContainsKey(ed))
                        {
                            continue;
                        }
                        ed = ShapeCasts.Edge(_map.GetValue(ed).First());
                        theCFace.InternalAdd(vtx, prm, ed);
                    }
                }
            }
            if (pw.InternalOnEdge(out Edge ebis))
            {
                edgOnEdg[edg] = ebis;
            }
            else
            {
                Face fac = pw.InternalOnFace();
                if (!_map.ContainsKey(fac))
                {
                    continue;
                }
                bool isFaceWithSec = pw.InternalIsFaceWithSection(fac);
                fac = ShapeCasts.Face(_map.GetValue(fac).First());
                if (isFaceWithSec)
                {
                    theFacesWithSection.Add(fac);
                }
                if (!mapFE.ContainsKey(fac))
                {
                    NCollection_List<Shape> thelist = new();
                    mapFE.Add(fac, thelist);
                }
                mapFE.GetValue(fac).Append(edg);
            }
        }

        // Rebuilds wires on each face of the shape

        NCollection_List<Shape>.Iterator itl = new();
        for (int i1 = 0; i1 < mapFE.Count; i1++)
        {
            Face fac = ShapeCasts.Face(mapFE.KeyAt(i1));
            //  Modified by skv - Mon May 31 12:32:54 2004 OCC5865 Begin
            RebuildWires(ref mapFE.ValueAt(i1), pw);
            NCollection_List<Shape> ledges = mapFE.ValueAt(i1);
            //  Modified by skv - Mon May 31 12:32:54 2004 OCC5865 End
            if (theFacesWithSection.Contains(fac))
            {
                theCFace.InternalAdd(ledges, fac);
            }
            else
            {
                for (itl.Initialize(ledges); itl.More(); itl.Next())
                {
                    theCFace.InternalAdd(ShapeCasts.Wire(itl.Value()), fac);
                }
            }
        }

        // Mise a jour des descendants

        foreach (Shape sori in _map.Keys.ToList())
        {
            Shape scib = _map.GetValue(sori).First();
            _map.GetValueRef(sori) = theCFace.InternalDescendantShapes(scib);
        }

        NCollection_List<Shape> lres = _map.GetValue(_shape);

        ShapeType typS = _shape.Type;
        if (typS == ShapeType.Face && lres.Extent() >= 2)
        {
            BrepBuilder b = new();
            _res.Nullify();
            Shell shell = new();
            b.MakeShell(ref shell);
            _res = shell;
            _res.Orient(Forward);
            for (itl.Initialize(lres); itl.More(); itl.Next())
            {
                b.Add(_res, itl.Value().Oriented(_shape.Orientation));
            }
        }
        else if (typS == ShapeType.Edge && lres.Extent() >= 2)
        {
            BrepBuilder b = new();
            _res.Nullify();
            Wire wire = new();
            b.MakeWire(ref wire);
            _res = wire;
            _res.Orient(Forward);
            for (itl.Initialize(lres); itl.More(); itl.Next())
            {
                b.Add(_res, itl.Value().Oriented(_shape.Orientation));
            }
        }
        else
        {
            if (lres.Extent() != 1)
            {
                return;
            }
            _res = lres.First();
        }

        theSubs.Clear();
        foreach (KeyValuePair<Shape, Shape> itee in edgOnEdg)
        {
            Edge e1 = ShapeCasts.Edge(itee.Key);
            // on recherche dans les descendants de e2 l`edge qui correspont a e1

            Exploration.Vertices(e1, out Vertex vf1, out Vertex vl1);
            lsubs.Clear();
            for (itl.Initialize(_map.GetValue(itee.Value)); itl.More(); itl.Next())
            {
                Edge e2 = ShapeCasts.Edge(itl.Value());
                Exploration.Vertices(e2, out Vertex vf2, out Vertex vl2);

                if (!vl1.IsSame(vf1))
                {
                    if (vf1.IsSame(vf2) && vl1.IsSame(vl2))
                    {
                        lsubs.Append(e2.Oriented(Forward));
                        //	break;
                    }
                    else if (vf1.IsSame(vl2) && vl1.IsSame(vf2))
                    {
                        lsubs.Append(e2.Oriented(Reversed));
                        //	break;
                    }
                }
                else
                { // discrimination sur les tangentes
                    if (vf2.IsSame(vl2) && vl2.IsSame(vl1))
                    { // tout au meme point
                        Curve3D c = Tool.Curve(e1, out Placement loc, out double f, out _)!;
                        c.D1(f, out Point3D pbid, out Vector3D v1);
                        v1 = v1.Transformed(loc.Transformation);

                        c = Tool.Curve(e2, out loc, out f, out _)!;
                        c.D1(f, out pbid, out Vector3D v2);
                        v2 = v2.Transformed(loc.Transformation);
                        if (v1.Dot(v2) > 0.0)
                        {
                            lsubs.Append(e2.Oriented(Forward));
                        }
                        else
                        {
                            lsubs.Append(e2.Oriented(Reversed));
                        }
                    }
                }
            }
            if (lsubs.Extent() >= 2)
            { // il faut faire un choix
                Select(e1, lsubs);
            }
            if (lsubs.Extent() == 1)
            {
                Shape ebase = lsubs.First();
                lsubs.Clear();
                lsubs.Append(e1.Oriented(ebase.Orientation));
                theSubs.Substitute(ebase, lsubs);
            }
            else
            {
                // OCCT_DEBUG trace left out ("Pb pour substitution")
            }
        }

        theSubs.Build(_res);

        foreach (Shape key in _map.Keys.ToList())
        {
            NCollection_List<Shape> ldesc = _map.GetValue(key);
            NCollection_List<Shape> newdesc = new();
            for (itl.Initialize(ldesc); itl.More(); itl.Next())
            {
                if (theSubs.IsCopied(itl.Value()))
                {
                    NCollection_List<Shape> lsub = theSubs.InternalCopy(itl.Value());
                    // OCCT_DEBUG check left out (Standard_ConstructionError if lsub.Extent() != 1)
                    newdesc.Append(lsub.First());
                }
                else
                {
                    newdesc.Append(itl.Value());
                }
            }
            _map.GetValueRef(key) = newdesc;
        }

        if (theSubs.IsCopied(_res))
        {
            _res = theSubs.InternalCopy(_res).First();
        }

        ////remove superfluous vertices on degenerated edges
        theSubs.Clear();
        IndexedSet<Shape> emap = new(ShapeHasher.Instance);
        Exploration.MapShapes(_res, ShapeType.Edge, emap);
        NCollection_Sequence<Shape> degEdges = new();
        int i, j;
        for (i = 1; i <= emap.Count; i++)
        {
            Edge edge = ShapeCasts.Edge(emap[i - 1]);
            if (Tool.Degenerated(edge))
            {
                degEdges.Append(edge);
            }
        }

        NCollection_Sequence<Shape> degWires = new();
        for (;;)
        {
            if (degEdges.IsEmpty())
            {
                break;
            }
            Wire degWire = new();
            bb.MakeWire(ref degWire);
            bb.Add(degWire, degEdges[1]);
            degEdges.Remove(1);
            for (;;)
            {
                Exploration.Vertices(degWire, out Vertex vfirst, out Vertex vlast);
                bool found = false;
                for (i = 1; i <= degEdges.Length(); i++)
                {
                    Edge edge = ShapeCasts.Edge(degEdges[i]);
                    Exploration.Vertices(edge, out Vertex V1, out Vertex V2);
                    if (V1.IsSame(vfirst) || V1.IsSame(vlast) || V2.IsSame(vfirst) || V2.IsSame(vlast))
                    {
                        bb.Add(degWire, edge);
                        degEdges.Remove(i);
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    break;
                }
            }
            degWires.Append(degWire);
        }

        for (i = 1; i <= degWires.Length(); i++)
        {
            IndexedSet<Shape> vmap = new(ShapeHasher.Instance);
            Exploration.MapShapes(degWires[i], ShapeType.Vertex, vmap);
            NCollection_List<Shape> lv = new();
            lv.Append(vmap[0].Oriented(Forward));
            for (j = 2; j <= vmap.Count; j++)
            {
                if (!vmap[j - 1].IsSame(vmap[0]))
                {
                    theSubs.Substitute(vmap[j - 1], lv);
                }
            }
        }
        theSubs.Build(_res);
        if (theSubs.IsCopied(_res))
        {
            _res = theSubs.InternalCopy(_res).First();
        }
        ////

        _dLeft.Clear();
        _left.Clear();
        mapV.Clear();

        for (exp.Init(_res, ShapeType.Face); exp.More(); exp.Next())
        {
            Face fac = ShapeCasts.Face(exp.Current());
            bool isFound = false;
            for (exp2.Init(fac, ShapeType.Edge); exp2.More(); exp2.Next())
            {
                Edge edg = ShapeCasts.Edge(exp2.Current());
                foreach (Shape itms in mapE)
                {
                    if (itms.IsSame(edg) && edg.Orientation == itms.Orientation)
                    {
                        isFound = true;
                        break;
                    }
                }
                if (isFound)
                {
                    break;
                }
            }
            if (exp2.More())
            {
                _dLeft.Append(fac);
                _left.Append(fac);
            }
            else
            {
                mapV.Add(fac);
            }
        }

        // Map des edges ou les connexions sont possibles
        // OCCT takes the first edge of this map and the first face of mapV that has it, in the order of their
        // addresses: so the order of Left() natively depends on the run (the port: the maps' order)
        HashSet<Shape> mapebord = new(ShapeHasher.Instance);
        for (itl.Initialize(_left); itl.More(); itl.Next())
        {
            for (exp.Init(itl.Value(), ShapeType.Edge); exp.More(); exp.Next())
            {
                if (!mapE.Contains(exp.Current()))
                {
                    if (!mapebord.Add(exp.Current()))
                    {
                        mapebord.Remove(exp.Current());
                    }
                }
            }
        }

        while (mapebord.Count != 0)
        {
            Shape edg = new();
            foreach (Shape first in mapebord)
            {
                edg = first;
                break;
            }

            Shape aFound = new();
            foreach (Shape fac in mapV)
            {
                for (exp.Init(fac, ShapeType.Edge); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(edg))
                    {
                        break;
                    }
                }
                if (exp.More())
                {
                    aFound = fac;
                    break; // face a gauche
                }
            }
            if (!aFound.IsNull)
            {
                Shape fac = aFound;
                for (exp.Init(fac, ShapeType.Edge); exp.More(); exp.Next())
                {
                    if (!mapebord.Add(exp.Current()))
                    {
                        mapebord.Remove(exp.Current());
                    }
                }
                mapV.Remove(fac);
                _left.Append(fac);
            }
            else
            {
                mapebord.Remove(edg);
            }
        }

        _done = true;
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
    public Shape Shape() => _input;

    /// <summary>Returns the faces which are the left of the projected wires (frozen).</summary>
    public NCollection_List<Shape> DirectLeft() => WorkingShapes.Published(InternalDirectLeft());

    /// <summary><see cref="DirectLeft"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalDirectLeft()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _dLeft;
    }

    /// <summary>
    /// Returns the faces of the "left" part on the shape, built from <see cref="DirectLeft"/> with the faces connected
    /// to this set, and so on (frozen).
    /// </summary>
    public NCollection_List<Shape> Left() => WorkingShapes.Published(InternalLeft());

    /// <summary><see cref="Left"/> without freezing, for ported code.</summary>
    internal NCollection_List<Shape> InternalLeft()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _left;
    }

    /// <summary>Returns the list of descendant shapes of <paramref name="f"/>, a sub-shape of the caller's shape (frozen).</summary>
    public NCollection_List<Shape> DescendantShapes(in Shape f) =>
        WorkingShapes.Published(InternalDescendantShapes(f));

    /// <summary>
    /// <see cref="DescendantShapes"/> without freezing, for ported code.
    /// </summary>
    internal NCollection_List<Shape> InternalDescendantShapes(in Shape f)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        Shape aF = f;
        if (_map.ContainsKey(aF))
        {
            return _map.GetValue(aF);
        }
        else
        {
            return new NCollection_List<Shape>();
        }
    }

    //  Modified by skv - Mon May 31 12:31:39 2004 OCC5865 Begin
    private static void RebuildWires(ref NCollection_List<Shape> ledge, WiresOnShape pw)
    {
        BuildWires theBuild = new(ledge, pw);
        //  Modified by skv - Mon May 31 12:31:40 2004 OCC5865 End
        if (!theBuild.IsDone)
        {
            throw new ConstructionErrorException();
        }
        ledge = new NCollection_List<Shape>(theBuild.InternalResult());
    }

    private static void Put(in Shape s, Dictionary<Shape, NCollection_List<Shape>> map)
    {
        if (map.ContainsKey(s))
        {
            return;
        }
        NCollection_List<Shape> thelist = new();
        map[s] = thelist;
        map.GetValue(s).Append(s);
        for (Iterator it = new(s); it.More(); it.Next())
        {
            Put(it.Value(), map);
        }
    }

    private static void Select(in Edge ebase, NCollection_List<Shape> lsubs)
    {
        // Choix d`un point

        double dmin = RealLast();
        int i = 0, imin = 0;

        Curve3D c = Tool.Curve(ebase, out Placement loc, out double f, out double l)!;

        if (!loc.IsIdentity)
        {
            Geometry3D gg = c.Transformed(loc.Transformation);
            c = (Curve3D)gg;
        }
        Point3D pt = c.Value((f + l) / 2.0);

        ProjectPointOnCurve proj = new();
        NCollection_List<Shape>.Iterator itl = new(lsubs);
        for (; itl.More(); itl.Next())
        {
            i++;
            Edge edg = ShapeCasts.Edge(itl.Value());
            c = Tool.Curve(edg, out loc, out f, out l)!;
            if (!loc.IsIdentity)
            {
                Geometry3D gg = c.Transformed(loc.Transformation);
                c = (Curve3D)gg;
            }
            proj.Init(pt, c, f, l);
            if (proj.NbPoints() > 0)
            {
                if (proj.LowerDistance() < dmin)
                {
                    imin = i;
                    dmin = proj.LowerDistance();
                }
            }
        }
        if (imin == 0)
        {
            lsubs.Clear();
        }
        else
        {
            itl.Initialize(lsubs);
            i = 1;
            while (i < imin)
            {
                lsubs.Remove(itl);
                i++;
            }
            itl.Next();
            while (itl.More())
            {
                lsubs.Remove(itl);
            }
        }
    }
}
