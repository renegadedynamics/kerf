// Port of: src/ModelingData/TKBRep/TopExp/TopExp.hxx
// Port of: src/ModelingData/TKBRep/TopExp/TopExp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Topology;

/// <summary>
/// Tools to explore shapes: maps of sub-shapes and of their ancestors, and the vertices of edges and wires.
/// </summary>
/// <remarks>
/// The maps are filled, not created: the caller passes a map with the comparer it wants, normally
/// <see cref="ShapeHasher.Instance"/> (OCCT's maps here are typed with TopTools_ShapeMapHasher).
/// </remarks>
internal static class Exploration
{
    /// <summary>Adds the sub-shapes of <paramref name="s"/> of type <paramref name="t"/> to <paramref name="m"/>.</summary>
    public static void MapShapes(in Shape s, ShapeType t, IndexedSet<Shape> m)
    {
        var ex = new Explorer(s, t);
        while (ex.More())
        {
            m.Add(ex.Current());
            ex.Next();
        }
    }

    /// <summary>Adds <paramref name="s"/> and all its sub-shapes to <paramref name="m"/>.</summary>
    /// <param name="s">The shape.</param>
    /// <param name="m">The map to fill.</param>
    /// <param name="cumOri">Compose the sub-shapes' orientations with their parents'.</param>
    /// <param name="cumLoc">Multiply the sub-shapes' locations by their parents'.</param>
    public static void MapShapes(in Shape s,
                                 IndexedSet<Shape> m,
                                 bool cumOri = true,
                                 bool cumLoc = true)
    {
        m.Add(s);
        var it = new Iterator(s, cumOri, cumLoc);
        while (it.More())
        {
            // OCCT: the recursion uses the default cumOri and cumLoc
            MapShapes(it.Value(), m);
            it.Next();
        }
    }

    /// <summary>Adds <paramref name="s"/> and all its sub-shapes to <paramref name="m"/>.</summary>
    /// <param name="s">The shape.</param>
    /// <param name="m">The map to fill.</param>
    /// <param name="cumOri">Compose the sub-shapes' orientations with their parents'.</param>
    /// <param name="cumLoc">Multiply the sub-shapes' locations by their parents'.</param>
    public static void MapShapes(in Shape s, HashSet<Shape> m, bool cumOri = true, bool cumLoc = true)
    {
        m.Add(s);
        var it = new Iterator(s, cumOri, cumLoc);
        for (; it.More(); it.Next())
        {
            MapShapes(it.Value(), m);
        }
    }

    /// <summary>
    /// Maps each sub-shape of <paramref name="s"/> of type <paramref name="ts"/> to the list of its ancestors of type
    /// <paramref name="ta"/>. For example, all the faces of each edge. Sub-shapes outside any ancestor get an empty list.
    /// </summary>
    public static void MapShapesAndAncestors(in Shape s,
                                             ShapeType ts,
                                             ShapeType ta,
                                             IndexedMap<Shape, NCollection_List<Shape>> m)
    {
        // visit ancestors
        var exa = new Explorer(s, ta);
        while (exa.More())
        {
            // visit shapes
            Shape anc = exa.Current();
            var exs = new Explorer(anc, ts);
            while (exs.More())
            {
                int index = m.IndexOf(exs.Current()) + 1;
                if (index == 0)
                {
                    // OCCT copies an empty list; the port's lists are classes, so each key gets its own
                    index = m.Add(exs.Current(), new NCollection_List<Shape>()) + 1;
                }

                m.ValueAt(index - 1).Append(anc);
                exs.Next();
            }

            exa.Next();
        }

        // visit shapes not under ancestors
        var ex = new Explorer(s, ts, ta);
        while (ex.More())
        {
            int index = m.IndexOf(ex.Current()) + 1;
            if (index == 0)
            {
                m.Add(ex.Current(), new NCollection_List<Shape>());
            }

            ex.Next();
        }
    }

    /// <summary>
    /// As <see cref="MapShapesAndAncestors"/>, but each ancestor is in a list once: compared with IsSame, or with
    /// IsEqual if <paramref name="useOrientation"/>.
    /// </summary>
    public static void MapShapesAndUniqueAncestors(in Shape s,
                                                   ShapeType ts,
                                                   ShapeType ta,
                                                   IndexedMap<Shape, NCollection_List<Shape>> m,
                                                   bool useOrientation = false)
    {
        // visit ancestors
        var exa = new Explorer(s, ta);
        while (exa.More())
        {
            // visit shapes
            Shape anc = exa.Current();
            var exs = new Explorer(anc, ts);
            while (exs.More())
            {
                int index = m.IndexOf(exs.Current()) + 1;
                if (index == 0)
                {
                    index = m.Add(exs.Current(), new NCollection_List<Shape>()) + 1;
                }

                NCollection_List<Shape> list = m.ValueAt(index - 1);
                // check if anc already exists in a list
                var it = new NCollection_List<Shape>.Iterator(list);
                for (; it.More(); it.Next())
                {
                    if (useOrientation ? anc.IsEqual(it.Value()) : anc.IsSame(it.Value()))
                    {
                        break;
                    }
                }

                if (!it.More())
                {
                    list.Append(anc);
                }

                exs.Next();
            }

            exa.Next();
        }

        // visit shapes not under ancestors
        var ex = new Explorer(s, ts, ta);
        while (ex.More())
        {
            int index = m.IndexOf(ex.Current()) + 1;
            if (index == 0)
            {
                m.Add(ex.Current(), new NCollection_List<Shape>());
            }

            ex.Next();
        }
    }

    /// <summary>The vertex of <paramref name="e"/> with orientation FORWARD, or a null vertex.</summary>
    /// <param name="e">The edge.</param>
    /// <param name="cumOri">Take the edge's orientation into account.</param>
    public static Vertex FirstVertex(in Edge e, bool cumOri = false)
    {
        var ite = new Iterator(e, cumOri);
        while (ite.More())
        {
            if (ite.Value().Orientation == Forward)
            {
                return ShapeCasts.Vertex(ite.Value());
            }

            ite.Next();
        }

        return new Vertex();
    }

    /// <summary>The vertex of <paramref name="e"/> with orientation REVERSED, or a null vertex.</summary>
    /// <param name="e">The edge.</param>
    /// <param name="cumOri">Take the edge's orientation into account.</param>
    public static Vertex LastVertex(in Edge e, bool cumOri = false)
    {
        var ite = new Iterator(e, cumOri);
        while (ite.More())
        {
            if (ite.Value().Orientation == Reversed)
            {
                return ShapeCasts.Vertex(ite.Value());
            }

            ite.Next();
        }

        return new Vertex();
    }

    /// <summary>The first and last vertices of <paramref name="e"/> (null where there is none).</summary>
    /// <param name="e">The edge.</param>
    /// <param name="vfirst">The FORWARD vertex.</param>
    /// <param name="vlast">The REVERSED vertex.</param>
    /// <param name="cumOri">Take the edge's orientation into account.</param>
    public static void Vertices(in Edge e, out Vertex vfirst, out Vertex vlast, bool cumOri = false)
    {
        // OCCT assigns the outputs only where it finds a vertex and nullifies the others: the same results
        vfirst = new Vertex();
        vlast = new Vertex();
        bool isFirstDefined = false;
        bool isLastDefined = false;

        var ite = new Iterator(e, cumOri);
        while (ite.More())
        {
            Shape v = ite.Value();
            if (v.Orientation == Forward)
            {
                vfirst = ShapeCasts.Vertex(v);
                isFirstDefined = true;
            }
            else if (v.Orientation == Reversed)
            {
                vlast = ShapeCasts.Vertex(v);
                isLastDefined = true;
            }

            ite.Next();
        }

        if (!isFirstDefined)
        {
            vfirst.Nullify();
        }

        if (!isLastDefined)
        {
            vlast.Nullify();
        }
    }

    /// <summary>
    /// The first and last vertices of the open wire <paramref name="w"/>; for a closed wire, the common vertex of its
    /// first and last edges, FORWARD and REVERSED. Null vertices if the wire is neither (non-manifold).
    /// </summary>
    public static void Vertices(in Wire w, out Vertex vfirst, out Vertex vlast)
    {
        vfirst = vlast = new Vertex(); // nullify

        var vmap = new HashSet<Shape>(ShapeHasher.Instance);
        var it = new Iterator(w);
        Vertex v1 = new Vertex(), v2 = new Vertex();

        while (it.More())
        {
            Edge e = ShapeCasts.Edge(it.Value());
            if (e.Orientation == Reversed)
            {
                Exploration.Vertices(e, out v2, out v1);
            }
            else
            {
                Exploration.Vertices(e, out v1, out v2);
            }

            // add or remove in the vertex map
            v1.Orient(Forward);
            v2.Orient(Reversed);
            if (!vmap.Add(v1))
            {
                vmap.Remove(v1);
            }

            if (!vmap.Add(v2))
            {
                vmap.Remove(v2);
            }

            it.Next();
        }

        if (vmap.Count == 0)
        { // closed
            Shape localShape = v2.Oriented(Forward);
            vfirst = ShapeCasts.Vertex(localShape);
            localShape = v2.Oriented(Reversed);
            vlast = ShapeCasts.Vertex(localShape);
        }
        else if (vmap.Count == 2)
        { // open
            // OCCT walks the map for the FORWARD key, then for the REVERSED one; with two keys the order doesn't matter
            foreach (Shape key in vmap)
            {
                if (key.Orientation == Forward)
                {
                    vfirst = ShapeCasts.Vertex(key);
                    break;
                }
            }

            foreach (Shape key in vmap)
            {
                if (key.Orientation == Reversed)
                {
                    vlast = ShapeCasts.Vertex(key);
                    break;
                }
            }
        }
    }

    /// <summary>
    /// The vertex shared by <paramref name="e1"/> and <paramref name="e2"/>, if any: true, with the vertex of E1
    /// (its first if both are shared). <paramref name="v"/> is left unchanged if there is none.
    /// </summary>
    public static bool CommonVertex(in Edge e1, in Edge e2, ref Vertex v)
    {
        Exploration.Vertices(e1, out Vertex firstVertex1, out Vertex lastVertex1);
        Exploration.Vertices(e2, out Vertex firstVertex2, out Vertex lastVertex2);

        if (firstVertex1.IsSame(firstVertex2) || firstVertex1.IsSame(lastVertex2))
        {
            v = firstVertex1;
            return true;
        }

        if (lastVertex1.IsSame(firstVertex2) || lastVertex1.IsSame(lastVertex2))
        {
            v = lastVertex1;
            return true;
        }

        return false;
    } // CommonVertex
}
