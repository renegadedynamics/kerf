// Port of: src/ModelingData/TKBRep/TopExp/TopExp.hxx
// Port of: src/ModelingData/TKBRep/TopExp/TopExp.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Tools to explore shapes: maps of sub-shapes and of their ancestors, and the vertices of edges and wires.
/// </summary>
/// <remarks>
/// The maps are filled, not created: the caller passes a map with the comparer it wants, normally
/// <see cref="TopTools_ShapeMapHasher.Instance"/> (OCCT's maps here are typed with TopTools_ShapeMapHasher).
/// </remarks>
public static class TopExp
{
    /// <summary>Adds the sub-shapes of <paramref name="S"/> of type <paramref name="T"/> to <paramref name="M"/>.</summary>
    public static void MapShapes(in TopoDS_Shape S, TopAbs_ShapeEnum T, NCollection_IndexedMap<TopoDS_Shape> M)
    {
        var Ex = new TopExp_Explorer(S, T);
        while (Ex.More())
        {
            M.Add(Ex.Current());
            Ex.Next();
        }
    }

    /// <summary>Adds <paramref name="S"/> and all its sub-shapes to <paramref name="M"/>.</summary>
    /// <param name="S">The shape.</param>
    /// <param name="M">The map to fill.</param>
    /// <param name="cumOri">Compose the sub-shapes' orientations with their parents'.</param>
    /// <param name="cumLoc">Multiply the sub-shapes' locations by their parents'.</param>
    public static void MapShapes(in TopoDS_Shape S,
                                 NCollection_IndexedMap<TopoDS_Shape> M,
                                 bool cumOri = true,
                                 bool cumLoc = true)
    {
        M.Add(S);
        var It = new TopoDS_Iterator(S, cumOri, cumLoc);
        while (It.More())
        {
            // OCCT: the recursion uses the default cumOri and cumLoc
            MapShapes(It.Value(), M);
            It.Next();
        }
    }

    /// <summary>Adds <paramref name="S"/> and all its sub-shapes to <paramref name="M"/>.</summary>
    /// <param name="S">The shape.</param>
    /// <param name="M">The map to fill.</param>
    /// <param name="cumOri">Compose the sub-shapes' orientations with their parents'.</param>
    /// <param name="cumLoc">Multiply the sub-shapes' locations by their parents'.</param>
    public static void MapShapes(in TopoDS_Shape S, NCollection_Map<TopoDS_Shape> M, bool cumOri = true, bool cumLoc = true)
    {
        M.Add(S);
        var It = new TopoDS_Iterator(S, cumOri, cumLoc);
        for (; It.More(); It.Next())
        {
            MapShapes(It.Value(), M);
        }
    }

    /// <summary>
    /// Maps each sub-shape of <paramref name="S"/> of type <paramref name="TS"/> to the list of its ancestors of type
    /// <paramref name="TA"/>. For example, all the faces of each edge. Sub-shapes outside any ancestor get an empty list.
    /// </summary>
    public static void MapShapesAndAncestors(in TopoDS_Shape S,
                                             TopAbs_ShapeEnum TS,
                                             TopAbs_ShapeEnum TA,
                                             NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> M)
    {
        // visit ancestors
        var exa = new TopExp_Explorer(S, TA);
        while (exa.More())
        {
            // visit shapes
            TopoDS_Shape anc = exa.Current();
            var exs = new TopExp_Explorer(anc, TS);
            while (exs.More())
            {
                int index = M.FindIndex(exs.Current());
                if (index == 0)
                {
                    // OCCT copies an empty list; the port's lists are classes, so each key gets its own
                    index = M.Add(exs.Current(), new NCollection_List<TopoDS_Shape>());
                }

                M[index].Append(anc);
                exs.Next();
            }

            exa.Next();
        }

        // visit shapes not under ancestors
        var ex = new TopExp_Explorer(S, TS, TA);
        while (ex.More())
        {
            int index = M.FindIndex(ex.Current());
            if (index == 0)
            {
                M.Add(ex.Current(), new NCollection_List<TopoDS_Shape>());
            }

            ex.Next();
        }
    }

    /// <summary>
    /// As <see cref="MapShapesAndAncestors"/>, but each ancestor is in a list once: compared with IsSame, or with
    /// IsEqual if <paramref name="useOrientation"/>.
    /// </summary>
    public static void MapShapesAndUniqueAncestors(in TopoDS_Shape S,
                                                   TopAbs_ShapeEnum TS,
                                                   TopAbs_ShapeEnum TA,
                                                   NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> M,
                                                   bool useOrientation = false)
    {
        // visit ancestors
        var exa = new TopExp_Explorer(S, TA);
        while (exa.More())
        {
            // visit shapes
            TopoDS_Shape anc = exa.Current();
            var exs = new TopExp_Explorer(anc, TS);
            while (exs.More())
            {
                int index = M.FindIndex(exs.Current());
                if (index == 0)
                {
                    index = M.Add(exs.Current(), new NCollection_List<TopoDS_Shape>());
                }

                NCollection_List<TopoDS_Shape> aList = M[index];
                // check if anc already exists in a list
                var it = new NCollection_List<TopoDS_Shape>.Iterator(aList);
                for (; it.More(); it.Next())
                {
                    if (useOrientation ? anc.IsEqual(it.Value()) : anc.IsSame(it.Value()))
                    {
                        break;
                    }
                }

                if (!it.More())
                {
                    aList.Append(anc);
                }

                exs.Next();
            }

            exa.Next();
        }

        // visit shapes not under ancestors
        var ex = new TopExp_Explorer(S, TS, TA);
        while (ex.More())
        {
            int index = M.FindIndex(ex.Current());
            if (index == 0)
            {
                M.Add(ex.Current(), new NCollection_List<TopoDS_Shape>());
            }

            ex.Next();
        }
    }

    /// <summary>The vertex of <paramref name="E"/> with orientation FORWARD, or a null vertex.</summary>
    /// <param name="E">The edge.</param>
    /// <param name="CumOri">Take the edge's orientation into account.</param>
    public static TopoDS_Vertex FirstVertex(in TopoDS_Edge E, bool CumOri = false)
    {
        var ite = new TopoDS_Iterator(E, CumOri);
        while (ite.More())
        {
            if (ite.Value().Orientation() == TopAbs_FORWARD)
            {
                return TopoDS.Vertex(ite.Value());
            }

            ite.Next();
        }

        return new TopoDS_Vertex();
    }

    /// <summary>The vertex of <paramref name="E"/> with orientation REVERSED, or a null vertex.</summary>
    /// <param name="E">The edge.</param>
    /// <param name="CumOri">Take the edge's orientation into account.</param>
    public static TopoDS_Vertex LastVertex(in TopoDS_Edge E, bool CumOri = false)
    {
        var ite = new TopoDS_Iterator(E, CumOri);
        while (ite.More())
        {
            if (ite.Value().Orientation() == TopAbs_REVERSED)
            {
                return TopoDS.Vertex(ite.Value());
            }

            ite.Next();
        }

        return new TopoDS_Vertex();
    }

    /// <summary>The first and last vertices of <paramref name="E"/> (null where there is none).</summary>
    /// <param name="E">The edge.</param>
    /// <param name="Vfirst">The FORWARD vertex.</param>
    /// <param name="Vlast">The REVERSED vertex.</param>
    /// <param name="CumOri">Take the edge's orientation into account.</param>
    public static void Vertices(in TopoDS_Edge E, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast, bool CumOri = false)
    {
        // OCCT assigns the outputs only where it finds a vertex and nullifies the others: the same results
        Vfirst = new TopoDS_Vertex();
        Vlast = new TopoDS_Vertex();
        bool isFirstDefined = false;
        bool isLastDefined = false;

        var ite = new TopoDS_Iterator(E, CumOri);
        while (ite.More())
        {
            TopoDS_Shape aV = ite.Value();
            if (aV.Orientation() == TopAbs_FORWARD)
            {
                Vfirst = TopoDS.Vertex(aV);
                isFirstDefined = true;
            }
            else if (aV.Orientation() == TopAbs_REVERSED)
            {
                Vlast = TopoDS.Vertex(aV);
                isLastDefined = true;
            }

            ite.Next();
        }

        if (!isFirstDefined)
        {
            Vfirst.Nullify();
        }

        if (!isLastDefined)
        {
            Vlast.Nullify();
        }
    }

    /// <summary>
    /// The first and last vertices of the open wire <paramref name="W"/>; for a closed wire, the common vertex of its
    /// first and last edges, FORWARD and REVERSED. Null vertices if the wire is neither (non-manifold).
    /// </summary>
    public static void Vertices(in TopoDS_Wire W, out TopoDS_Vertex Vfirst, out TopoDS_Vertex Vlast)
    {
        Vfirst = Vlast = new TopoDS_Vertex(); // nullify

        var vmap = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var it = new TopoDS_Iterator(W);
        TopoDS_Vertex V1 = new TopoDS_Vertex(), V2 = new TopoDS_Vertex();

        while (it.More())
        {
            TopoDS_Edge E = TopoDS.Edge(it.Value());
            if (E.Orientation() == TopAbs_REVERSED)
            {
                TopExp.Vertices(E, out V2, out V1);
            }
            else
            {
                TopExp.Vertices(E, out V1, out V2);
            }

            // add or remove in the vertex map
            V1.Orientation(TopAbs_FORWARD);
            V2.Orientation(TopAbs_REVERSED);
            if (!vmap.Add(V1))
            {
                vmap.Remove(V1);
            }

            if (!vmap.Add(V2))
            {
                vmap.Remove(V2);
            }

            it.Next();
        }

        if (vmap.IsEmpty())
        { // closed
            TopoDS_Shape aLocalShape = V2.Oriented(TopAbs_FORWARD);
            Vfirst = TopoDS.Vertex(aLocalShape);
            aLocalShape = V2.Oriented(TopAbs_REVERSED);
            Vlast = TopoDS.Vertex(aLocalShape);
        }
        else if (vmap.Extent() == 2)
        { // open
            // OCCT walks the map for the FORWARD key, then for the REVERSED one; with two keys the order doesn't matter
            foreach (TopoDS_Shape aKey in vmap)
            {
                if (aKey.Orientation() == TopAbs_FORWARD)
                {
                    Vfirst = TopoDS.Vertex(aKey);
                    break;
                }
            }

            foreach (TopoDS_Shape aKey in vmap)
            {
                if (aKey.Orientation() == TopAbs_REVERSED)
                {
                    Vlast = TopoDS.Vertex(aKey);
                    break;
                }
            }
        }
    }

    /// <summary>
    /// The vertex shared by <paramref name="E1"/> and <paramref name="E2"/>, if any: true, with the vertex of E1
    /// (its first if both are shared). <paramref name="V"/> is left unchanged if there is none.
    /// </summary>
    public static bool CommonVertex(in TopoDS_Edge E1, in TopoDS_Edge E2, ref TopoDS_Vertex V)
    {
        TopExp.Vertices(E1, out TopoDS_Vertex firstVertex1, out TopoDS_Vertex lastVertex1);
        TopExp.Vertices(E2, out TopoDS_Vertex firstVertex2, out TopoDS_Vertex lastVertex2);

        if (firstVertex1.IsSame(firstVertex2) || firstVertex1.IsSame(lastVertex2))
        {
            V = firstVertex1;
            return true;
        }

        if (lastVertex1.IsSame(firstVertex2) || lastVertex1.IsSame(lastVertex2))
        {
            V = lastVertex1;
            return true;
        }

        return false;
    } // CommonVertex
}
