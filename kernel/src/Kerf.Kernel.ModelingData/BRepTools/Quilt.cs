// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Quilt.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Quilt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// Glues faces into shells: faces are added shape by shape, edges and vertices of a new shape can be bound to edges
/// and vertices of shapes added before (the faces using them are then copied), and <see cref="Shells"/> returns a
/// compound of shells made from the faces, flagged closed or not.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Shells"/> and <see cref="Copy"/> freeze what they return
/// (docs/phase3-design.md, section 4.2), so call them once all the shapes are added: a later <see cref="Add"/> that
/// gives a pcurve to a copied edge they returned throws.
/// </para>
/// <para>
/// Adding a face that uses a bound edge gives the edge it is bound to a pcurve on that face, in place, as in OCCT. An
/// edge bound with <see cref="Bind(in Edge, in Edge)"/> to a frozen edge (any shape a caller holds)
/// therefore makes <see cref="Add"/> throw <see cref="LockedShapeException"/>, as OCCT does for a locked one; ported code
/// passes edges it built itself.
/// </para>
/// </remarks>
public sealed class Quilt
{
    private readonly NCollection_IndexedDataMap<Shape, Shape> _bounds = new(ShapeHasher.Instance);
    private bool _hasCopy;

    /// <summary>An empty quilt.</summary>
    public Quilt()
    {
        _hasCopy = false;
    }

    // test if the shape must be copied
    // i.e. it contains a bound subshape
    private static bool NeedCopied(in Shape shape, NCollection_IndexedDataMap<Shape, Shape> myBounds)
    {
        bool isCopied = false;
        var itv = new Iterator(shape);
        for (; itv.More(); itv.Next())
        {
            if (myBounds.Contains(itv.Value()))
            {
                isCopied = true;
                break;
            }
        }
        return isCopied;
    }

    private static void CopyShape(in Edge e, NCollection_IndexedDataMap<Shape, Shape> myBounds)
    {
        Edge ne = e;
        ne.EmptyCopy();
        ne.Orientation(Forward);
        BrepBuilder b = new();
        // add the edges
        var itv = new Iterator();
        itv.Initialize(e, false); // TCollection_DataMap
        for (; itv.More(); itv.Next())
        {
            Shape v = itv.Value();
            if (myBounds.Contains(v))
            {
                b.Add(ne, myBounds.FindFromKey(v).Oriented(v.Orientation()));
            }
            else
            {
                b.Add(ne, v);
            }
        }
        // set the 3d range
        Tool.Range(e, out double f, out double l);
        b.Range(ne, f, l);
        myBounds.Add(e, ne.Oriented(Forward));
    }

    /// <summary>
    /// Adds the faces of the shape: a face containing a bound edge or vertex is copied with the substitutions.
    /// </summary>
    public void Add(in Shape s)
    {
        //  Binds all the faces of S
        //   - to the face itself if it is not copied
        //   - to the copy if it is copied
        if (_bounds.Contains(s))
        {
            return;
        }

        BrepBuilder b = new();
        for (var wex = new Explorer(s, ShapeType.Wire, ShapeType.Face); wex.More(); wex.Next())
        {
            _bounds.Add(wex.Current(), wex.Current());
        }

        for (var eex = new Explorer(s, ShapeType.Edge, ShapeType.Wire); eex.More(); eex.Next())
        {
            _bounds.Add(eex.Current(), eex.Current());
        }

        for (var vex = new Explorer(s, ShapeType.Vertex, ShapeType.Edge); vex.More(); vex.Next())
        {
            _bounds.Add(vex.Current(), vex.Current());
        }

        // explore the faces
        for (var fex = new Explorer(s, ShapeType.Face); fex.More(); fex.Next())
        {
            // explore the edges of the face and try to copy them
            // if one edge is bound the face must be copied

            bool copyFace = false;
            Face f = ShapeCasts.Face(fex.Current());

            if (_hasCopy)
            { // if their is no binding, do not test for copy

                for (var fed = new Explorer(f, ShapeType.Edge); fed.More(); fed.Next())
                {
                    if (_bounds.Contains(fed.Current()))
                    {
                        copyFace = true;
                    }
                    else
                    {
                        // test if the edge must be copied
                        // i.e. it contains a bound vertex

                        bool copyEdge = NeedCopied(fed.Current(), _bounds);
                        Edge e = ShapeCasts.Edge(fed.Current());

                        if (copyEdge)
                        {
                            // copy of an edge

                            copyFace = true;
                            CopyShape(e, _bounds);
                        }
                    }
                }
            }

            // NF will be the copy of F or F itself
            Face nf = f;

            if (copyFace)
            {
                // copy of a face

                nf.EmptyCopy();
                nf.Orientation(Forward);

                for (var itw = new Iterator(f, false); itw.More(); itw.Next())
                {
                    Wire w = ShapeCasts.Wire(itw.Value());

                    Wire nw = new();
                    b.MakeWire(ref nw);
                    var ite = new Iterator(w, false);
                    double uFirst, uLast;

                    // Reconstruction of wires.

                    for (; ite.More(); ite.Next())
                    {
                        Edge e = ShapeCasts.Edge(ite.Value());
                        Orientation oe = e.Orientation();
                        if (_bounds.Contains(e))
                        {
                            Edge ne = ShapeCasts.Edge(_bounds.FindFromKey(e));
                            // pcurve.
                            if (ne.Orientation() == Forward)
                            {
                                b.UpdateEdge(ne,
                                             Tool.CurveOnSurface(e, f, out uFirst, out uLast),
                                             f,
                                             Tool.Tolerance(e));
                            }
                            else
                            {
                                // If NE is REVERSED
                                // => the 3D curves do not have the same orientation.
                                // (This is a convention, cf. BRepTools_Quilt.cdl and the Bind
                                // method.)
                                // => the PCurve of E on F must be reversed.

                                oe = global::Kerf.Engine.Topology.TopAbs.Reverse(oe);
                                Curve2D ce = Tool.CurveOnSurface(e, f, out uFirst, out uLast)!;
                                Curve2D nce = ce.Reversed();
                                b.UpdateEdge(ne, nce, f, Tool.Tolerance(e));
                                double tmp = uFirst;
                                uFirst = ce.ReversedParameter(uLast);
                                uLast = ce.ReversedParameter(tmp);
                            }
                            // pcurve range
                            b.Range(ne, f, uFirst, uLast);

                            b.Add(nw, ne.Oriented(oe));
                        }
                        else
                        {
                            b.Add(nw, e);
                        }
                    }
                    nw.Orientation(w.Orientation());
                    b.Add(nf, nw);
                }
                nf.Orientation(f.Orientation());
            }

            // binds the face to itself or its copy
            _bounds.Add(f, nf);
        }
    }

    /// <summary>
    /// Binds <paramref name="vnew"/> to be the new vertex instead of <paramref name="vold"/>: the faces of the next
    /// added shape containing <paramref name="vold"/> are copied with <paramref name="vnew"/>.
    /// </summary>
    public void Bind(in Vertex vold, in Vertex vnew)
    {
        if (!_bounds.Contains(vold))
        {
            _bounds.Add(vold, vnew);
        }
    }

    /// <summary>
    /// Binds <paramref name="enew"/> (in a shape added before) to be the new edge instead of <paramref name="eold"/>
    /// (in the next added shape): the faces containing <paramref name="eold"/> are copied with
    /// <paramref name="enew"/>, and the vertices of <paramref name="eold"/> are bound to those of
    /// <paramref name="enew"/> with the same orientation. Edges of different orientations have opposite curves: the
    /// pcurves of <paramref name="eold"/> are reversed in the new faces.
    /// </summary>
    public void Bind(in Edge eold, in Edge enew)
    {
        if (!_bounds.Contains(eold))
        {
            Edge eNew = enew;
            if (IsCopied(enew))
            {
                eNew = ShapeCasts.Edge(InternalCopy(enew));
                eNew.Orientation(enew.Orientation());
            }

            if (eold.Orientation() != eNew.Orientation())
            {
                _bounds.Add(eold.Oriented(Forward), eNew.Oriented(Reversed));
            }
            else
            {
                _bounds.Add(eold.Oriented(Forward), eNew.Oriented(Forward));
            }
            // if new binding bind also the vertices
            var itold = new Iterator(eold);
            while (itold.More())
            {
                if (!_bounds.Contains(itold.Value()))
                {
                    // find the vertex of Enew with same orientation
                    Orientation orien = itold.Value().Orientation();
                    var itnew = new Iterator(eNew);
                    while (itnew.More())
                    {
                        if (itnew.Value().Orientation() == orien)
                        {
                            Vertex vNew = ShapeCasts.Vertex(itnew.Value());
                            if (IsCopied(vNew))
                            {
                                // if VNew has been copied take the copy
                                vNew = ShapeCasts.Vertex(InternalCopy(vNew));
                            }
                            _bounds.Add(itold.Value(), vNew);
                            break;
                        }
                        itnew.Next();
                    }
                }
                itold.Next();
            }
            _hasCopy = true;
        }
    }

    /// <summary>True if the shape (a vertex, an edge or a face) has been copied.</summary>
    public bool IsCopied(in Shape s)
    {
        if (_bounds.Contains(s))
        {
            return !s.IsSame(_bounds.FindFromKey(s));
        }
        else
        {
            return false;
        }
    }

    /// <summary>The shape substituted for <paramref name="s"/> in the quilt, frozen.</summary>
    /// <exception cref="NoSuchObjectException">The shape hasn't been copied.</exception>
    public Shape Copy(in Shape s)
    {
        Shape copy = InternalCopy(s);
        copy.TShape()?.FreezeGraph();
        return copy;
    }

    /// <summary>The shape substituted for <paramref name="s"/>, as <see cref="Copy"/>, without freezing it (ported code).</summary>
    internal Shape InternalCopy(in Shape s)
    {
        Standard_NoSuchObject_Raise_if(!IsCopied(s), "BRepTools_Quilt::Copy");
        return _bounds.FindFromKey(s);
    }

    /// <summary>
    /// A compound of shells made from the current faces, each flagged closed or not, frozen. Shapes added without faces
    /// (free edges and vertices) are added to the compound too.
    /// </summary>
    public Shape Shells()
    {
        Shape result = InternalShells();
        result.TShape()!.FreezeGraph();
        return result;
    }

    /// <summary>The compound of shells, as <see cref="Shells"/>, without freezing it (ported code).</summary>
    internal Shape InternalShells()
    {
        // Outline of the algorithm
        //
        // In the map M we bind the free edges to their shells
        // We explore all the faces in myBounds
        // For each one we search the edges in the map and either :
        //
        // - Start a new shell if no edge is a free edge.
        // - Add the face to an existing shell
        // - Connect other shells if the face touch more than one shell

        // In the Map M the Shell is bound with the relative orientation of E
        // in the shell
        // In the map MF we binb the face to its shell.
        // In the Map MF the Shell is bound with the relative orientation of F
        // in the shell

        var m = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        var mf = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        BrepBuilder b = new();
        Compound result = new();

        b.MakeCompound(ref result);

        var mapOtherShape = new HashSet<Shape>(ShapeHasher.Instance); // gka
        var edgesFaces = new HashSet<Shape>(ShapeHasher.Instance);

        // loop on the face in myBounds
        for (int ii = 1; ii <= _bounds.Extent(); ii++)
        {
            Shape shape = _bounds.FindFromIndex(ii); // it.Value();
            if (shape.ShapeType() == ShapeType.Face)
            {
                for (var expEdg = new Explorer(shape, ShapeType.Edge); expEdg.More(); expEdg.Next())
                { // gka
                    edgesFaces.Add(expEdg.Current());
                }

                Shell SH = new();
                Orientation newO;

                var itf1 = new Explorer(shape, ShapeType.Edge);
                for (; itf1.More(); itf1.Next())
                {
                    Shape e = itf1.Current();
                    if (m.IsBound(e))
                    {
                        SH = ShapeCasts.Shell(m[e]);
                        if (SH.Orientation() == e.Orientation())
                        {
                            newO = global::Kerf.Engine.Topology.TopAbs.Reverse(shape.Orientation());
                        }
                        else
                        {
                            newO = shape.Orientation();
                        }

                        mf.Bind(shape, SH.Oriented(newO));
                        break;
                    }
                }

                if (SH.IsNull)
                {
                    // Create a new shell, closed. Add it to the result.
                    b.MakeShell(ref SH);
                    SH.Closed(true);
                    b.Add(result, SH);
                    mf.Bind(shape, SH.Oriented(shape.Orientation()));
                }

                // Add the face to the shell
                SH.Free(true);
                Shape arefShape = SH.Oriented(Forward);
                b.Add(arefShape, shape.Oriented(mf[shape].Orientation()));

                var itf = new Explorer(shape.Oriented(Forward), ShapeType.Edge);

                for (; itf.More(); itf.Next())
                {
                    Edge e = ShapeCasts.Edge(itf.Current());

                    if (m.IsBound(e))
                    {
                        Shape oldShell = m[e];
                        if (!oldShell.IsSame(SH))
                        {
                            // Fuse the old shell with the new one
                            // Compare the orientation of E in SH and in oldshell.
                            Orientation orien = e.Orientation();
                            if (mf[shape].Orientation() == Reversed)
                            {
                                orien = global::Kerf.Engine.Topology.TopAbs.Reverse(orien);
                            }

                            bool rev = (orien == oldShell.Orientation());
                            // if rev = True oldShell has to be reversed.

                            // Add the faces of oldShell in SH.
                            for (var its = new Iterator(oldShell); its.More(); its.Next())
                            {
                                Face fo = ShapeCasts.Face(its.Value());
                                Orientation newOFo;
                                // update the orientation of Fo in SH.
                                if (rev)
                                {
                                    newOFo = global::Kerf.Engine.Topology.TopAbs.Reverse(mf[fo].Orientation());
                                }
                                else
                                {
                                    newOFo = mf[fo].Orientation();
                                }

                                mf.Bind(fo, SH.Oriented(newOFo));
                                Shape arefShapeFo = SH.Oriented(Forward);
                                b.Add(arefShapeFo, fo.Oriented(newOFo));
                            }
                            // Rebind the free edges of the old shell to the new shell
                            // gka BUG 6491
                            var aexp = new Explorer(SH, ShapeType.Edge);
                            for (; aexp.More(); aexp.Next())
                            {
                                if (!m.IsBound(aexp.Current()))
                                {
                                    continue;
                                }
                                Shape ae = aexp.Current();
                                Shape @as = m.Find(ae);
                                if (@as.IsSame(oldShell))
                                {
                                    // update the orientation of free edges in SH.
                                    if (rev)
                                    {
                                        newO = global::Kerf.Engine.Topology.TopAbs.Reverse(@as.Orientation());
                                    }
                                    else
                                    {
                                        newO = @as.Orientation();
                                    }

                                    m.Bind(ae, SH.Oriented(newO));
                                }
                            }
                            // remove the old shell from the result
                            b.Remove(result, oldShell.Oriented(Forward));
                        }
                        {
                            // Test if SH is always orientable.
                            Orientation orien = e.Orientation();
                            if (mf[shape].Orientation() == Reversed)
                            {
                                orien = global::Kerf.Engine.Topology.TopAbs.Reverse(orien);
                            }

                            if (m[e].Orientation() == orien)
                            {
                                SH.Orientable(false);
                            }
                        }

                        // remove the edge from M (no more a free edge)
                        m.UnBind(e);
                    }
                    else
                    {
                        newO = e.Orientation();
                        if (mf[shape].Orientation() == Reversed)
                        {
                            newO = global::Kerf.Engine.Topology.TopAbs.Reverse(newO);
                        }
                        if (!e.IsNull)
                        {
                            m.Bind(e, SH.Oriented(newO));
                        }
                        else
                        {
                            continue;
                        }
                    }
                }

                // freeze the shell
                SH.Free(false);
            }
            else
            {
                mapOtherShape.Add(shape);
            }
        }

        // Unclose all shells having free edges
        foreach (KeyValuePair<Shape, Shape> it in m)
        {
            Shape s = it.Value;
            s.Closed(false);
        }

        // gka version for free edges
        foreach (Shape itother in mapOtherShape)
        {
            if (!edgesFaces.Contains(itother) && _bounds.Contains(itother))
            {
                Shape sh = _bounds.FindFromKey(itother);
                b.Add(result, sh);
            }
        }
        return result;
    }
}
