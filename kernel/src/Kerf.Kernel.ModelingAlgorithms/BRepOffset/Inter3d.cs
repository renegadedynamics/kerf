// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Inter3d.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Inter3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Fri Dec 26 12:20:14 2003 OCC4455

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Offsets;

/// <summary>
/// Computes the connection of the offset and not offset faces according to the connection type required, and stores
/// the result in the AsDes tool.
/// </summary>
/// <remarks>
/// A working object of the 3D offset (decision 21): the maps it returns (<see cref="TouchedFaces"/>,
/// <see cref="NewEdges"/>) and the AsDes it fills are its own and the caller's working state, as OCCT's references are.
/// As in OCCT, <see cref="ContextIntByInt"/> and <see cref="ContextIntByArc"/> write into the vertices of the context
/// faces' free edges (<c>BRep_Builder.UpdateVertex</c> on the extended edge) and the new compounds it builds; those
/// are the offset's own working shapes (BRepOffset_MakeOffset works on its own copy of the caller's shape) and must be
/// unfrozen: a frozen one throws <see cref="ImmutableObjectException"/> at the first write. The offsets of
/// <c>MapSF</c> are only read.
/// </remarks>
internal sealed class Inter3d
{
    private readonly AsDes _asDes;
    private readonly IndexedSet<Shape> _touched = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NCollection_List<Shape>> _done = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _newEdges = new(ShapeHasher.Instance);
    private readonly Containment _side;
    private readonly double _tol;

    //=================================================================================================

    /// <summary>A tool storing its intersections into <paramref name="asDes"/>.</summary>
    public Inter3d(AsDes asDes, Containment side, double tol)
    {
        _asDes = asDes;
        _side = side;
        _tol = tol;
    }

    //=================================================================================================

    private static void ExtentEdge(in Face F, in Edge e, out Edge ne)
    {
        Shape localShape = e.EmptyCopied();
        ne = ShapeCasts.Edge(localShape);
        //  NE = TopoDS::Edge(E.EmptyCopied());

        // Enough for analytic edges, in general case reconstruct the
        // geometry of the edge recalculating the intersection of surfaces.

        ne.Orient(Forward);
        double f, l;
        Tool.Range(e, out f, out l);
        double length = l - f;
        f -= 100 * length;
        l += 100 * length;

        BrepBuilder b = new();
        b.Range(ne, f, l);
        EdgeCurveAdaptor ce = new(e);
        Vertex v1 = new LibraryMakeVertex(ce.Value(f)).InternalVertex();
        Vertex v2 = new LibraryMakeVertex(ce.Value(l)).InternalVertex();
        b.Add(ne, v1.Oriented(Forward));
        b.Add(ne, v2.Oriented(Reversed));
        ne.Orient(e.Orientation);
    }

    //=================================================================================================

    /// <summary>Computes the intersections of the given faces among each other.</summary>
    public void CompletInt(NCollection_List<Shape> setOfFaces, ShapeImage initOffsetFace,
                           ProgressRange range)
    {
        //---------------------------------------------------------------
        // Calculate the intersections of offset faces
        // Distinction of intersection between faces // tangents.
        //---------------------------------------------------------------

        // Prepare tools for sorting the bounding boxes
        BoxTree bbTree = new();
        bbTree.SetSize(setOfFaces.Extent());
        //
        IndexedMap<Shape, Box> mFaces = new(ShapeHasher.Instance);
        // Construct bounding boxes for faces and add them to the tree
        NCollection_List<Shape>.Iterator itL = new(setOfFaces);
        for (; itL.More(); itL.Next())
        {
            Face f = ShapeCasts.Face(itL.Value());
            //
            // compute bounding box
            Box boxF = new();
            Bounds.Add(f, ref boxF);
            //
            int i = mFaces.Add(f, boxF) + 1;
            //
            bbTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxF));
        }

        // Build BVH
        bbTree.Build();

        // Perform selection of the pairs
        BoxPairSelector selector = new();
        selector.SetBVHSets(bbTree, bbTree);
        selector.SetSame(true);
        selector.Select();
        selector.Sort();

        // Treat the selected pairs
        NCollection_LinearVector<PairSelector<NCollection_Vec3<double>>.PairIDs> pairs = selector.Pairs();
        int nbPairs = pairs.Size();
        using ProgressScope ps = new(range, "Complete intersection", nbPairs);
        for (int iPair = 0; iPair < nbPairs; ++iPair, ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            PairSelector<NCollection_Vec3<double>>.PairIDs pair = pairs.Value(iPair);

            Face f1 = ShapeCasts.Face(mFaces.KeyAt(Min(pair.ID1, pair.ID2) - 1));
            Face f2 = ShapeCasts.Face(mFaces.KeyAt(Max(pair.ID1, pair.ID2) - 1));

            // intersect faces
            FaceInter(f1, f2, initOffsetFace);
        }
    }

    //=======================================================================
    // function : FaceInter
    // purpose  : Performs intersection of the given faces
    //=======================================================================

    /// <summary>Computes the intersection of a pair of faces.</summary>
    public void FaceInter(in Face f1, in Face f2, ShapeImage initOffsetFace)
    {
        NCollection_List<Shape> lInt1 = new(), lInt2 = new();
        Edge nullEdge = new();
        Face nullFace = new();

        if (f1.IsSame(f2))
        {
            return;
        }
        if (IsDone(f1, f2))
        {
            return;
        }

        Shape initF1 = initOffsetFace.ImageFrom(f1);
        Shape initF2 = initOffsetFace.ImageFrom(f2);
        if (initF1.IsSame(initF2))
        {
            return;
        }

        bool interPipes = (initF2.Type == ShapeType.Edge && initF1.Type == ShapeType.Edge);
        bool interFaces = (initF1.Type == ShapeType.Face && initF2.Type == ShapeType.Face);
        NCollection_List<Shape> le = new(), lv = new();
        lInt1.Clear();
        lInt2.Clear();
        if (OffsetTool.FindCommonShapes(f1, f2, le, lv) || _asDes.HasCommonDescendant(f1, f2, le))
        {
            //-------------------------------------------------
            // F1 and F2 share shapes.
            //-------------------------------------------------
            if (le.IsEmpty() && !lv.IsEmpty())
            {
                if (interPipes)
                {
                    //----------------------
                    // tubes share a vertex.
                    //----------------------
                    Edge ee1 = ShapeCasts.Edge(initF1);
                    Edge ee2 = ShapeCasts.Edge(initF2);
                    Vertex[] ve1 = [new(), new()], ve2 = [new(), new()];
                    Exploration.Vertices(ee1, out ve1[0], out ve1[1]);
                    Exploration.Vertices(ee2, out ve2[0], out ve2[1]);
                    Vertex v = new();
                    for (int i = 0; i < 2; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            if (ve1[i].IsSame(ve2[j]))
                            {
                                v = ve1[i];
                            }
                        }
                    }
                    if (!initOffsetFace.HasImage(v))
                    { // no sphere
                        OffsetTool.PipeInter(f1, f2, lInt1, lInt2, _side);
                    }
                }
                else
                {
                    //--------------------------------------------------------
                    // Intersection having only common vertices
                    // and supports having common edges.
                    // UNSUFFICIENT, but a larger criterion shakes too
                    // many sections.
                    //--------------------------------------------------------
                    if (interFaces)
                    {
                        if (OffsetTool.FindCommonShapes(ShapeCasts.Face(initF1), ShapeCasts.Face(initF2), le, lv))
                        {
                            if (!le.IsEmpty())
                            {
                                OffsetTool.Inter3D(f1, f2, lInt1, lInt2, _side, nullEdge, nullFace, nullFace);
                            }
                        }
                        else
                        {
                            OffsetTool.Inter3D(f1, f2, lInt1, lInt2, _side, nullEdge, nullFace, nullFace);
                        }
                    }
                }
            }
        }
        else
        {
            if (interPipes)
            {
                OffsetTool.PipeInter(f1, f2, lInt1, lInt2, _side);
            }
            else
            {
                OffsetTool.Inter3D(f1, f2, lInt1, lInt2, _side, nullEdge, nullFace, nullFace);
            }
        }
        Store(f1, f2, lInt1, lInt2);
    }

    //=================================================================================================

    /// <summary>Computes the connections of the offset faces that have to be connected by arcs.</summary>
    public void ConnexIntByArc(NCollection_List<Shape> setOfFaces, in Shape shapeInit,
                               Analyse analyse, ShapeImage initOffsetFace,
                               ProgressRange range)
    {
        TypeOfConcavity ot = TypeOfConcavity.Concave;
        if (_side == Out)
        {
            ot = TypeOfConcavity.Convex;
        }
        Explorer exp = new(shapeInit, ShapeType.Edge);
        NCollection_List<Shape> lInt1 = new(), lInt2 = new();
        Face f1 = new(), f2 = new();
        Edge nullEdge = new();
        Face nullFace = new();
        using ProgressScope psOuter = new(range, null, 2);
        using ProgressScope psIntF = new(psOuter.Next(), "Intersecting offset faces", 1, true);
        //---------------------------------------------------------------------
        // etape 1 : Intersection of faces // corresponding to the initial faces
        //           separated by a concave edge if offset > 0, otherwise convex.
        //---------------------------------------------------------------------
        for (; exp.More(); exp.Next(), psIntF.Next())
        {
            if (!psIntF.More())
            {
                return;
            }
            Edge e = ShapeCasts.Edge(exp.Current());
            NCollection_List<OffsetInterval> L = analyse.Type(e);
            if (!L.IsEmpty() && L.First().Type() == ot)
            {
                //-----------------------------------------------------------
                // edge is of the proper type , return adjacent faces.
                //-----------------------------------------------------------
                NCollection_List<Shape> anc = analyse.Ancestors(e);
                if (anc.Extent() == 2)
                {

                    Face initF1 = ShapeCasts.Face(anc.First());
                    Face initF2 = ShapeCasts.Face(anc.Last());
                    f1 = ShapeCasts.Face(initOffsetFace.Image(initF1).First());
                    f2 = ShapeCasts.Face(initOffsetFace.Image(initF2).First());
                    if (!IsDone(f1, f2))
                    {
                        OffsetTool.Inter3D(f1, f2, lInt1, lInt2, _side, e, initF1, initF2);
                        Store(f1, f2, lInt1, lInt2);
                    }
                }
            }
        }
        //---------------------------------------------------------------------
        // etape 2 : Intersections of tubes sharing a vertex without sphere with:
        //           - tubes on each other edge sharing the vertex
        //           - faces containing an edge connected to vertex that has no tubes.
        //---------------------------------------------------------------------
        Vertex[] v = [new(), new()];
        NCollection_List<Shape>.Iterator it = new();
        using ProgressScope psIntT = new(psOuter.Next(), "Intersecting tubes", 1, true);
        for (exp.Init(shapeInit, ShapeType.Edge); exp.More(); exp.Next(), psIntT.Next())
        {
            if (!psIntT.More())
            {
                return;
            }
            Edge e1 = ShapeCasts.Edge(exp.Current());
            if (initOffsetFace.HasImage(e1))
            {
                //---------------------------
                // E1 generated a tube.
                //---------------------------
                f1 = ShapeCasts.Face(initOffsetFace.Image(e1).First());
                Exploration.Vertices(e1, out v[0], out v[1]);
                NCollection_List<Shape> ancE1 = analyse.Ancestors(e1);

                for (int i = 0; i < 2; i++)
                {
                    if (!initOffsetFace.HasImage(v[i]))
                    {
                        //-----------------------------
                        // the vertex has no sphere.
                        //-----------------------------
                        NCollection_List<Shape> anc = analyse.Ancestors(v[i]);
                        NCollection_List<Shape> tangOnV = new();
                        analyse.TangentEdges(e1, v[i], tangOnV);
                        HashSet<Shape> mtev = new(ShapeHasher.Instance);
                        for (it.Initialize(tangOnV); it.More(); it.Next())
                        {
                            mtev.Add(it.Value());
                        }
                        for (it.Initialize(anc); it.More(); it.Next())
                        {
                            Edge e2 = ShapeCasts.Edge(it.Value());
                            //  Modified by skv - Fri Jan 16 16:27:54 2004 OCC4455 Begin
                            //            if (E1.IsSame(E2) || MTEV.Contains(E2)) continue;
                            bool isToSkip = false;

                            if (!e1.IsSame(e2))
                            {
                                NCollection_List<OffsetInterval> aL = analyse.Type(e2);

                                isToSkip =
                                  (mtev.Contains(e2) && (aL.IsEmpty() || (!aL.IsEmpty() && aL.First().Type() != ot)));
                            }

                            if (e1.IsSame(e2) || isToSkip)
                            {
                                continue;
                            }
                            //  Modified by skv - Fri Jan 16 16:27:54 2004 OCC4455 End
                            if (initOffsetFace.HasImage(e2))
                            {
                                //-----------------------------
                                // E2 generated a tube.
                                //-----------------------------
                                f2 = ShapeCasts.Face(initOffsetFace.Image(e2).First());
                                if (!IsDone(f1, f2))
                                {
                                    //---------------------------------------------------------------------
                                    // Intersection tube/tube if the edges are not tangent (AFINIR).
                                    //----------------------------------------------------------------------
                                    OffsetTool.PipeInter(f1, f2, lInt1, lInt2, _side);
                                    Store(f1, f2, lInt1, lInt2);
                                }
                            }
                            else
                            {
                                //-------------------------------------------------------
                                // Intersection of the tube of E1 with faces //
                                // to face containing E2 if they are not tangent
                                // to the tube or if E2 is not a tangent edge.
                                //-------------------------------------------------------
                                NCollection_List<OffsetInterval> l = analyse.Type(e2);
                                if (!l.IsEmpty() && l.First().Type() == TypeOfConcavity.Tangential)
                                {
                                    continue;
                                }
                                NCollection_List<Shape> ancE2 = analyse.Ancestors(e2);
                                bool tangentFaces = false;
                                if (ancE2.Extent() == 2)
                                {
                                    Face initF2 = ShapeCasts.Face(ancE2.First());
                                    tangentFaces = (initF2.IsSame(ancE1.First()) || initF2.IsSame(ancE1.Last()));
                                    if (!tangentFaces)
                                    {
                                        f2 = ShapeCasts.Face(initOffsetFace.Image(initF2).First());
                                        if (!IsDone(f1, f2))
                                        {
                                            OffsetTool.Inter3D(f1,
                                                                    f2,
                                                                    lInt1,
                                                                    lInt2,
                                                                    _side,
                                                                    nullEdge,
                                                                    nullFace,
                                                                    nullFace);
                                            Store(f1, f2, lInt1, lInt2);
                                        }
                                    }
                                    initF2 = ShapeCasts.Face(ancE2.Last());
                                    tangentFaces = (initF2.IsSame(ancE1.First()) || initF2.IsSame(ancE1.Last()));
                                    if (!tangentFaces)
                                    {
                                        f2 = ShapeCasts.Face(initOffsetFace.Image(initF2).First());
                                        if (!IsDone(f1, f2))
                                        {
                                            OffsetTool.Inter3D(f1,
                                                                    f2,
                                                                    lInt1,
                                                                    lInt2,
                                                                    _side,
                                                                    nullEdge,
                                                                    nullFace,
                                                                    nullFace);
                                            Store(f1, f2, lInt1, lInt2);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Computes the intersections of the offset faces that have to be connected by sharp edges, i.e. the
    /// intersections between the extended offset faces.
    /// </summary>
    public void ConnexIntByInt(in Shape si, Dictionary<Shape, Offset> mapSF,
                               Analyse analyse, Dictionary<Shape, Shape> mes,
                               Dictionary<Shape, Shape> build,
                               NCollection_List<Shape> failed, ProgressRange range,
                               bool bIsPlanar = false)
    {
        IndexedSet<Shape> vEmap = new(ShapeHasher.Instance);
        Face F1 = new(), F2 = new(), of1 = new(), of2 = new(), NF1 = new(), NF2 = new();
        Containment curSide = _side;
        BrepBuilder b = new();
        bool bEdge;
        int i, nb = 0;
        NCollection_List<Shape>.Iterator it = new(), it1 = new(), itF1 = new(), itF2 = new();
        //
        Exploration.MapShapes(si, ShapeType.Edge, vEmap);
        // Take the vertices for treatment
        using ProgressScope psOuter = new(range, null, 10);
        if (bIsPlanar)
        {
            nb = vEmap.Count;
            for (i = 1; i <= nb; ++i)
            {
                Edge aE = ShapeCasts.Edge(vEmap[i - 1]);
                Shape fGen = analyse.Generated(aE);
                if (!fGen.IsNull)
                {
                    Exploration.MapShapes(fGen, ShapeType.Edge, vEmap);
                }
            }

            // Add vertices for treatment
            Exploration.MapShapes(si, ShapeType.Vertex, vEmap);

            for (NCollection_List<Shape>.Iterator itNF = new(analyse.NewFaces()); itNF.More(); itNF.Next())
            {
                Exploration.MapShapes(itNF.Value(), ShapeType.Vertex, vEmap);
            }
        }
        //
        Dictionary<Shape, NCollection_List<Shape>> dmvlf1 = new(ShapeHasher.Instance),
                                                                         dmvlf2 = new(ShapeHasher.Instance),
                                                                         dmIntFF = new(ShapeHasher.Instance);
        IndexedMap<Shape, NCollection_List<Shape>> dmIntE = new(ShapeHasher.Instance);
        //
        if (bIsPlanar)
        {
            // Find internal edges in the faces to skip them while preparing faces
            // for intersection through vertices
            Dictionary<Shape, HashSet<Shape>> dmfei = new(ShapeHasher.Instance);
            {
                for (Explorer expF = new(si, ShapeType.Face); expF.More(); expF.Next())
                {
                    Shape fx = expF.Current();

                    HashSet<Shape> mei = new(ShapeHasher.Instance);
                    for (Explorer expE = new(fx, ShapeType.Edge); expE.More(); expE.Next())
                    {
                        Shape ex = expE.Current();
                        if (ex.Orientation != Forward && ex.Orientation != Reversed)
                        {
                            mei.Add(ex);
                        }
                    }
                    if (mei.Count != 0)
                    {
                        dmfei[fx] = mei;
                    }
                }
            }

            // Analyze faces connected through vertices
            for (i = nb + 1, nb = vEmap.Count; i <= nb; ++i)
            {
                if (!psOuter.More())
                {
                    return;
                }
                Shape s = vEmap[i - 1];
                if (s.Type != ShapeType.Vertex)
                {
                    continue;
                }

                // Find faces connected to the vertex
                NCollection_List<Shape> lf = new();
                {
                    NCollection_List<Shape> le = analyse.Ancestors(s);
                    for (NCollection_List<Shape>.Iterator itLE = new(le); itLE.More(); itLE.Next())
                    {
                        NCollection_List<Shape> lea = analyse.Ancestors(itLE.Value());
                        for (NCollection_List<Shape>.Iterator itLEA = new(lea); itLEA.More(); itLEA.Next())
                        {
                            if (!lf.Contains(itLEA.Value()))
                            {
                                lf.Append(itLEA.Value());
                            }
                        }
                    }
                }

                if (lf.Extent() < 2)
                {
                    continue;
                }

                // build lists of faces connected to the same vertex by looking for
                // the pairs in which the vertex is alone (not connected to shared edges)
                NCollection_List<Shape> lf1 = new(), lf2 = new();

                it.Initialize(lf);
                for (; it.More(); it.Next())
                {
                    Shape fv1 = it.Value();

                    // get edges of first face connected to current vertex
                    HashSet<Shape> me = new(ShapeHasher.Instance);
                    HashSet<Shape>? pF1Internal =
                      dmfei.TryGetValue(fv1, out HashSet<Shape>? f1Internal) ? f1Internal : null;
                    NCollection_List<Shape>? pLE1 = analyse.Descendants(fv1);
                    if (pLE1 is null)
                    {
                        continue;
                    }
                    NCollection_List<Shape>.Iterator itLE1 = new(pLE1);
                    for (; itLE1.More(); itLE1.Next())
                    {
                        Shape aE = itLE1.Value();
                        if (pF1Internal is not null && pF1Internal.Contains(aE))
                        {
                            break;
                        }

                        for (Iterator itV = new(aE); itV.More(); itV.Next())
                        {
                            if (s.IsSame(itV.Value()))
                            {
                                me.Add(aE);
                                break;
                            }
                        }
                    }
                    if (itLE1.More())
                    {
                        continue;
                    }

                    // get to the next face in the list
                    it1.Reset(lf, it.Index); // it1 = it
                    for (it1.Next(); it1.More(); it1.Next())
                    {
                        Face fv2 = ShapeCasts.Face(it1.Value());

                        HashSet<Shape>? pF2Internal =
                          dmfei.TryGetValue(fv2, out HashSet<Shape>? f2Internal) ? f2Internal : null;

                        NCollection_List<Shape>? pLE2 = analyse.Descendants(fv2);
                        if (pLE2 is null)
                        {
                            continue;
                        }
                        NCollection_List<Shape>.Iterator itLE2 = new(pLE2);
                        for (; itLE2.More(); itLE2.Next())
                        {
                            Shape ev2 = itLE2.Value();
                            if (!me.Contains(ev2))
                            {
                                continue;
                            }

                            if (pF2Internal is not null && pF2Internal.Contains(ev2))
                            {
                                // Avoid intersection of faces connected by internal edge
                                break;
                            }

                            if (analyse.HasAncestor(ev2) && analyse.Ancestors(ev2).Extent() == 2)
                            {
                                // Faces will be intersected through the edge
                                break;
                            }
                        }

                        if (!itLE2.More())
                        {
                            lf1.Append(fv1);
                            lf2.Append(fv2);
                        }
                    }
                }
                //
                if (lf1.Extent() != 0)
                {
                    dmvlf1[s] = lf1;
                    dmvlf2[s] = lf2;
                }
            }
        }
        //
        nb = vEmap.Count;
        using ProgressScope psInter = new(psOuter.Next(8), "Intersecting offset faces", nb);
        for (i = 1; i <= nb; ++i, psInter.Next())
        {
            if (!psInter.More())
            {
                return;
            }
            Shape s = vEmap[i - 1];
            //
            Edge E = new();
            NCollection_List<Shape> lf1 = new(), lf2 = new();
            //
            bEdge = (s.Type == ShapeType.Edge);
            if (bEdge)
            {
                // faces connected by the edge
                E = ShapeCasts.Edge(s);
                //
                NCollection_List<OffsetInterval> l = analyse.Type(E);
                if (l.IsEmpty())
                {
                    continue;
                }
                //
                TypeOfConcavity ot = l.First().Type();
                if (ot != TypeOfConcavity.Convex && ot != TypeOfConcavity.Concave)
                {
                    continue;
                }
                //
                if (ot == TypeOfConcavity.Concave)
                {
                    curSide = In;
                }
                else
                {
                    curSide = Out;
                }
                //-----------------------------------------------------------
                // edge is of the proper type, return adjacent faces.
                //-----------------------------------------------------------
                NCollection_List<Shape> anc = analyse.Ancestors(E);
                if (anc.Extent() != 2)
                {
                    continue;
                }
                //
                F1 = ShapeCasts.Face(anc.First());
                F2 = ShapeCasts.Face(anc.Last());
                //
                lf1.Append(F1);
                lf2.Append(F2);
            }
            else
            {
                if (!dmvlf1.ContainsKey(s))
                {
                    continue;
                }
                //
                lf1.Assign(dmvlf1.GetValue(s));
                lf2.Assign(dmvlf2.GetValue(s));
                //
                curSide = _side;
            }
            //
            itF1.Initialize(lf1);
            itF2.Initialize(lf2);
            for (; itF1.More() && itF2.More(); itF1.Next(), itF2.Next())
            {
                F1 = ShapeCasts.Face(itF1.Value());
                F2 = ShapeCasts.Face(itF2.Value());
                //
                of1 = ShapeCasts.Face(mapSF.GetValue(F1).InternalFace());
                of2 = ShapeCasts.Face(mapSF.GetValue(F2).InternalFace());
                if (!mes.ContainsKey(of1))
                {
                    bool enlargeU = true;
                    bool enlargeVfirst = true, enlargeVlast = true;
                    OffsetTool.CheckBounds(F1, analyse, out enlargeU, out enlargeVfirst, out enlargeVlast);
                    OffsetTool.EnLargeFace(of1, ref NF1, true, true, enlargeU, enlargeVfirst, enlargeVlast);
                    mes[of1] = NF1;
                }
                else
                {
                    NF1 = ShapeCasts.Face(mes.GetValue(of1));
                }
                //
                if (!mes.ContainsKey(of2))
                {
                    bool enlargeU = true;
                    bool enlargeVfirst = true, enlargeVlast = true;
                    OffsetTool.CheckBounds(F2, analyse, out enlargeU, out enlargeVfirst, out enlargeVlast);
                    OffsetTool.EnLargeFace(of2, ref NF2, true, true, enlargeU, enlargeVfirst, enlargeVlast);
                    mes[of2] = NF2;
                }
                else
                {
                    NF2 = ShapeCasts.Face(mes.GetValue(of2));
                }
                //
                if (!IsDone(NF1, NF2))
                {
                    NCollection_List<Shape> LInt1 = new(), LInt2 = new();
                    OffsetTool.Inter3D(NF1, NF2, LInt1, LInt2, curSide, E, F1, F2);
                    SetDone(NF1, NF2);
                    if (!LInt1.IsEmpty())
                    {
                        Store(NF1, NF2, LInt1, LInt2);
                        //
                        Compound c = new();
                        b.MakeCompound(ref c);
                        //
                        if (build.ContainsKey(s))
                        {
                            Shape se = build.GetValue(s);
                            Explorer exp = new(se, ShapeType.Edge);
                            for (; exp.More(); exp.Next())
                            {
                                Shape ne = exp.Current();
                                b.Add(c, ne);
                            }
                        }
                        //
                        it.Initialize(LInt1);
                        for (; it.More(); it.Next())
                        {
                            Shape ne = it.Value();
                            b.Add(c, ne);
                            //
                            // keep connection from new edge to shape from which it was created
                            NCollection_List<Shape> pLS = dmIntE.ValueAt(dmIntE.Add(ne, new NCollection_List<Shape>()));
                            pLS.Append(s);
                            // keep connection to faces created the edge as well
                            NCollection_List<Shape> pLFF = dmIntFF.SetValueRef(ne, new NCollection_List<Shape>());
                            pLFF.Append(F1);
                            pLFF.Append(F2);
                        }
                        //
                        build[s] = c;
                    }
                    else
                    {
                        failed.Append(s);
                    }
                }
                else
                { // IsDone(NF1,NF2)
                    //  Modified by skv - Fri Dec 26 12:20:13 2003 OCC4455 Begin
                    NCollection_List<Shape> lInt1 = _asDes.Descendant(NF1);
                    NCollection_List<Shape> lInt2 = _asDes.Descendant(NF2);

                    if (!lInt1.IsEmpty())
                    {
                        Compound c = new();
                        b.MakeCompound(ref c);
                        //
                        if (build.ContainsKey(s))
                        {
                            Shape se = build.GetValue(s);
                            Explorer exp = new(se, ShapeType.Edge);
                            for (; exp.More(); exp.Next())
                            {
                                Shape ne = exp.Current();
                                b.Add(c, ne);
                            }
                        }
                        //
                        for (it.Initialize(lInt1); it.More(); it.Next())
                        {
                            Shape e1 = it.Value();
                            //
                            for (it1.Initialize(lInt2); it1.More(); it1.Next())
                            {
                                Shape e2 = it1.Value();
                                if (e1.IsSame(e2))
                                {
                                    b.Add(c, e1);
                                    //
                                    if (dmIntE.TryGetValue(e1, out NCollection_List<Shape> pLS))
                                    {
                                        pLS.Append(s);
                                    }
                                }
                            }
                        }
                        build[s] = c;
                    }
                    else
                    {
                        failed.Append(s);
                    }
                }
            }
            //  Modified by skv - Fri Dec 26 12:20:14 2003 OCC4455 End
        }
        //
        // create unique intersection for each localized shared part
        nb = dmIntE.Count;
        using ProgressScope psPostTreat = new(psOuter.Next(2), "Creating unique intersection", nb);
        for (i = 1; i <= nb; ++i, psPostTreat.Next())
        {
            if (!psPostTreat.More())
            {
                return;
            }
            NCollection_List<Shape> ls = dmIntE.ValueAt(i - 1);
            if (ls.Extent() < 2)
            {
                continue;
            }
            //
            // intersection edge
            Edge e = ShapeCasts.Edge(dmIntE.KeyAt(i - 1));
            // faces created the edge
            NCollection_List<Shape> lff = dmIntFF.GetValue(e);
            Shape f1 = lff.First();
            Shape f2 = lff.Last();

            // Build really localized blocks from the original shapes in <aLS>:
            // 1. Find edges from original faces connected to two or more shapes in <aLS>;
            // 2. Make connexity blocks from edges in <aLS> and found connection edges;
            // 3. Check if the vertices from <aLS> are not connected by these connection edges:
            //    a. If so - add these vertices to Connexity Block containing the corresponding
            //       connexity edge;
            //    b. If not - add this vertex to list of connexity blocks
            // 4. Create unique intersection edge for each connexity block

            // list of vertices
            NCollection_List<Shape> lv = new();
            // compound of edges to build connexity blocks
            Compound ce = new();
            b.MakeCompound(ref ce);
            HashSet<Shape> ms = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator itLS = new(ls);
            for (; itLS.More(); itLS.Next())
            {
                Shape s = itLS.Value();
                ms.Add(s);
                if (s.Type == ShapeType.Edge)
                {
                    b.Add(ce, s);
                }
                else
                {
                    lv.Append(s);
                }
            }
            //
            // look for additional edges to connect the shared parts
            HashSet<Shape> meConnection = new(ShapeHasher.Instance);
            for (int j = 0; j < 2; ++j)
            {
                Shape f = j == 0 ? f1 : f2;
                //
                Explorer exp = new(f, ShapeType.Edge);
                for (; exp.More(); exp.Next())
                {
                    Shape ef = exp.Current();
                    if (ms.Contains(ef) || meConnection.Contains(ef))
                    {
                        continue;
                    }
                    //
                    Exploration.Vertices(ShapeCasts.Edge(ef), out Vertex v1, out Vertex v2);
                    //
                    // find parts to which the edge is connected
                    int iCounter = 0;
                    itLS.Initialize(ls);
                    for (; itLS.More(); itLS.Next())
                    {
                        Shape s = itLS.Value();
                        // iterator is not suitable here, because aS may be a vertex
                        Explorer expV = new(s, ShapeType.Vertex);
                        for (; expV.More(); expV.Next())
                        {
                            Shape v = expV.Current();
                            if (v.IsSame(v1) || v.IsSame(v2))
                            {
                                ++iCounter;
                                break;
                            }
                        }
                    }
                    //
                    if (iCounter >= 2)
                    {
                        b.Add(ce, ef);
                        meConnection.Add(ef);
                    }
                }
            }
            //
            NCollection_List<Shape> lcbe = new();
            AlgoTools.MakeConnexityBlocks(ce, ShapeType.Vertex, ShapeType.Edge, lcbe);
            //
            // create connexity blocks for alone vertices
            NCollection_List<Shape> lcbv = new();
            NCollection_List<Shape>.Iterator itLV = new(lv);
            for (; itLV.More(); itLV.Next())
            {
                Shape v = itLV.Value();
                // check if this vertex is contained in some connexity block of edges
                NCollection_List<Shape>.Iterator itLCB1 = new(lcbe);
                for (; itLCB1.More(); itLCB1.Next())
                {
                    ref Shape cb = ref itLCB1.ChangeValue();
                    Explorer expV = new(cb, ShapeType.Vertex);
                    for (; expV.More(); expV.Next())
                    {
                        if (v.IsSame(expV.Current()))
                        {
                            b.Add(cb, v);
                            break;
                        }
                    }
                    if (expV.More())
                    {
                        break;
                    }
                }
                //
                if (!itLCB1.More())
                {
                    Compound cv = new();
                    b.MakeCompound(ref cv);
                    b.Add(cv, v);
                    lcbv.Append(cv);
                }
            }
            //
            lcbe.Append(lcbv);
            //
            if (lcbe.Extent() == 1)
            {
                continue;
            }
            //
            Shape nf1 = mes.GetValue(mapSF.GetValue(f1).InternalFace());
            Shape nf2 = mes.GetValue(mapSF.GetValue(f2).InternalFace());
            //
            NCollection_List<Shape>.Iterator itLCB = new(lcbe);
            for (itLCB.Next(); itLCB.More(); itLCB.Next())
            {
                // make new edge with different tedge instance
                Edge newEdge;
                Vertex v1, v2;
                double t1, t2;
                //
                Exploration.Vertices(e, out v1, out v2);
                Tool.Range(e, out t1, out t2);
                //
                AlgoTools.MakeSplitEdge(e, v1, t1, v2, t2, out newEdge);
                //
                _asDes.Add(nf1, newEdge);
                _asDes.Add(nf2, newEdge);
                //
                Shape cb = itLCB.Value();
                Iterator itCB = new(cb);
                for (; itCB.More(); itCB.Next())
                {
                    Shape s = itCB.Value();
                    if (meConnection.Contains(s))
                    {
                        continue;
                    }
                    ref Shape ci = ref build.GetValueRef(s);
                    //
                    Compound newCI = new();
                    b.MakeCompound(ref newCI);
                    Explorer exp = new(ci, ShapeType.Edge);
                    for (; exp.More(); exp.Next())
                    {
                        Shape sx = exp.Current();
                        if (!sx.IsSame(e))
                        {
                            b.Add(newCI, sx);
                        }
                    }
                    b.Add(newCI, newEdge);
                    ci = newCI;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Computes the intersections with the not offset (context) faces.</summary>
    public void ContextIntByInt(IndexedSet<Shape> contextFaces, bool extentContext,
                                Dictionary<Shape, Offset> mapSF,
                                Analyse analyse, Dictionary<Shape, Shape> mes,
                                Dictionary<Shape, Shape> build,
                                NCollection_List<Shape> failed, ProgressRange range,
                                bool bIsPlanar = false)
    {
        HashSet<Shape> mv = new(ShapeHasher.Instance);
        Explorer exp = new();
        Face of = new(), nf = new(), wcf = new();
        Edge oe = new();
        Compound c = new();
        BrepBuilder b = new();
        NCollection_List<Shape>.Iterator it = new(), itF = new();
        int i, j, nb, nbVE;
        bool bEdge;

        nb = contextFaces.Count;
        for (i = 1; i <= nb; i++)
        {
            Face cf = ShapeCasts.Face(contextFaces[i - 1]);
            _touched.Add(cf);
            if (extentContext)
            {
                OffsetTool.EnLargeFace(cf, ref nf, false, false);
                mes[cf] = nf;
            }
        }
        Containment side = Out;

        using ProgressScope ps = new(range, "Intersecting with deepening faces", nb);
        for (i = 1; i <= nb; i++, ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Face cf = ShapeCasts.Face(contextFaces[i - 1]);
            if (extentContext)
            {
                wcf = ShapeCasts.Face(mes.GetValue(cf));
            }
            else
            {
                wcf = cf;
            }

            IndexedSet<Shape> vEmap = new(ShapeHasher.Instance);
            Exploration.MapShapes(cf.Oriented(Forward), ShapeType.Edge, vEmap);
            //
            if (bIsPlanar)
            {
                Exploration.MapShapes(cf.Oriented(Forward), ShapeType.Vertex, vEmap);
            }
            //
            nbVE = vEmap.Count;
            for (j = 1; j <= nbVE; ++j)
            {
                Shape s = vEmap[j - 1];
                //
                bEdge = (s.Type == ShapeType.Edge);
                //
                Edge E = new();
                NCollection_List<Shape> anc = new();
                //
                if (bEdge)
                {
                    // faces connected by the edge
                    //
                    E = ShapeCasts.Edge(s);
                    if (!analyse.HasAncestor(E))
                    {
                        //----------------------------------------------------------------
                        // the edges of faces of context that are not in the initial shape
                        // can appear in the result.
                        //----------------------------------------------------------------
                        if (!extentContext)
                        {
                            _asDes.Add(cf, E);
                            _newEdges.Add(E);
                        }
                        else
                        {
                            if (!mes.ContainsKey(E))
                            {
                                double f, l, tol;
                                Tool.Range(E, out f, out l);
                                tol = Tool.Tolerance(E);
                                ExtentEdge(cf, E, out Edge NE);
                                Exploration.Vertices(E, out Vertex v1, out Vertex v2);
                                NE.Orient(Forward);
                                _asDes.Add(NE, v1.Oriented(Reversed));
                                _asDes.Add(NE, v2.Oriented(Forward));
                                Shape localShape = v1.Oriented(Internal);
                                b.UpdateVertex(ShapeCasts.Vertex(localShape), f, NE, tol);
                                localShape = v2.Oriented(Internal);
                                b.UpdateVertex(ShapeCasts.Vertex(localShape), l, NE, tol);
                                //            B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),f,NE,Tol);
                                //            B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),l,NE,Tol);
                                NE.Orient(E.Orientation);
                                _asDes.Add(cf, NE);
                                _newEdges.Add(NE);
                                mes[E] = NE;
                            }
                            else
                            {
                                Shape NE = mes.GetValue(E);
                                Shape localShape = NE.Oriented(E.Orientation);
                                _asDes.Add(cf, localShape);
                                //            myAsDes->Add(CF,NE.Oriented(E.Orientation()));
                            }
                        }
                        continue;
                    }
                    anc.Assign(analyse.Ancestors(E));
                }
                else
                {
                    // faces connected by the vertex
                    //
                    if (!analyse.HasAncestor(s))
                    {
                        continue;
                    }
                    //
                    NCollection_List<Shape> le = analyse.Ancestors(s);
                    it.Initialize(le);
                    for (; it.More(); it.Next())
                    {
                        Edge e = ShapeCasts.Edge(it.Value());
                        //
                        if (Tool.Degenerated(e))
                        {
                            continue;
                        }
                        //
                        if (vEmap.Contains(e))
                        {
                            continue;
                        }
                        //
                        NCollection_List<Shape> lf = analyse.Ancestors(e);
                        itF.Initialize(lf);
                        for (; itF.More(); itF.Next())
                        {
                            Shape aF = itF.Value();
                            bool bAdd = true;
                            exp.Init(aF, ShapeType.Edge);
                            for (; exp.More() && bAdd; exp.Next())
                            {
                                Shape ef = exp.Current();
                                bAdd = !vEmap.Contains(ef);
                            }
                            if (bAdd)
                            {
                                anc.Append(aF);
                            }
                        }
                    }
                }
                //
                itF.Initialize(anc);
                for (; itF.More(); itF.Next())
                {
                    Face F = ShapeCasts.Face(itF.Value());
                    of = ShapeCasts.Face(mapSF.GetValue(F).InternalFace());
                    Shape localShape = mapSF.GetValue(F).InternalGenerated(E);
                    oe = ShapeCasts.Edge(localShape);
                    //      OE = TopoDS::Edge(MapSF(F).Generated(E));
                    if (!mes.ContainsKey(of))
                    {
                        OffsetTool.EnLargeFace(of, ref nf, true, true);
                        mes[of] = nf;
                    }
                    else
                    {
                        nf = ShapeCasts.Face(mes.GetValue(of));
                    }
                    if (!IsDone(nf, cf))
                    {
                        NCollection_List<Shape> lInt1 = new(), lInt2 = new();
                        NCollection_List<Shape> loe = new();
                        loe.Append(oe);
                        OffsetTool.Inter3D(wcf, nf, lInt1, lInt2, side, E, cf, F);
                        SetDone(nf, cf);
                        if (!lInt1.IsEmpty())
                        {
                            Store(cf, nf, lInt1, lInt2);
                            if ((lInt1.Extent() == 1) && !build.ContainsKey(s))
                            {
                                build[s] = lInt1.First();
                            }
                            else
                            {
                                b.MakeCompound(ref c);
                                if (build.ContainsKey(s))
                                {
                                    Shape se = build.GetValue(s);
                                    exp.Init(se, ShapeType.Edge);
                                    for (; exp.More(); exp.Next())
                                    {
                                        Shape ne = exp.Current();
                                        b.Add(c, ne);
                                    }
                                }
                                //
                                for (it.Initialize(lInt1); it.More(); it.Next())
                                {
                                    b.Add(c, it.Value());
                                }
                                build[s] = c;
                            }
                        }
                        else
                        {
                            failed.Append(s);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Computes the connections of the not offset (context) faces that have to be connected by arcs.</summary>
    public void ContextIntByArc(IndexedSet<Shape> contextFaces, bool inSide,
                                Analyse analyse, ShapeImage initOffsetFace,
                                ShapeImage initOffsetEdge, ProgressRange range)
    {
        NCollection_List<Shape> lInt1 = new(), lInt2 = new();
        HashSet<Shape> mv = new(ShapeHasher.Instance);
        Explorer exp = new();
        Face of1 = new(), of2 = new();
        Edge oe = new();
        BrepBuilder b = new();
        Edge nullEdge = new();
        Face nullFace = new();
        int j;

        for (j = 1; j <= contextFaces.Count; j++)
        {
            Face cf = ShapeCasts.Face(contextFaces[j - 1]);
            _touched.Add(cf);
        }

        using ProgressScope ps = new(range, "Intersecting with deepening faces", contextFaces.Count);
        for (j = 1; j <= contextFaces.Count; j++, ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Face cf = ShapeCasts.Face(contextFaces[j - 1]);
            for (exp.Init(cf.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                if (!analyse.HasAncestor(e))
                {
                    if (inSide)
                    {
                        _asDes.Add(cf, e);
                    }
                    else
                    {
                        Edge ne;
                        if (!initOffsetEdge.HasImage(e))
                        {
                            double f, l, tol;
                            Tool.Range(e, out f, out l);
                            tol = Tool.Tolerance(e);
                            ExtentEdge(cf, e, out ne);
                            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                            ne.Orient(Forward);
                            _asDes.Add(ne, v1.Oriented(Reversed));
                            _asDes.Add(ne, v2.Oriented(Forward));
                            Shape localShape = v1.Oriented(Internal);
                            b.UpdateVertex(ShapeCasts.Vertex(localShape), f, ne, tol);
                            localShape = v2.Oriented(Internal);
                            b.UpdateVertex(ShapeCasts.Vertex(localShape), l, ne, tol);
                            //            B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),f,NE,Tol);
                            //            B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),l,NE,Tol);
                            ne.Orient(e.Orientation);
                            _asDes.Add(cf, ne);
                            initOffsetEdge.Bind(e, ne);
                        }
                        else
                        {
                            ne = ShapeCasts.Edge(initOffsetEdge.Image(e).First());
                            _asDes.Add(cf, ne.Oriented(e.Orientation));
                        }
                    }
                    continue;
                }
                oe.Nullify();
                //---------------------------------------------------
                // OF1 parallel facee generated by the ancestor of E.
                //---------------------------------------------------
                Shape si = analyse.Ancestors(e).First();
                of1 = ShapeCasts.Face(initOffsetFace.Image(si).First());
                oe = ShapeCasts.Edge(initOffsetEdge.Image(e).First());

                {
                    // Check if OE has pcurve in CF

                    double f, l;

                    Curve2D? c1 = Tool.CurveOnSurface(oe, cf, out f, out l);
                    Curve2D? c2 = Tool.CurveOnSurface(oe, of1, out f, out l);

                    if (c1 is null || c2 is null)
                    {
                        continue;
                    }
                }

                //--------------------------------------------------
                // MAJ of OE on cap CF.
                //--------------------------------------------------
                //      NCollection_List<TopoDS_Shape> LOE; LOE.Append(OE);
                //      BRepOffset_Tool::TryProject(CF,OF1,LOE,LInt1,LInt2,mySide);
                //      LInt2.Clear();
                //      StoreInter3d(CF,OF1,myTouched,NewEdges,InterDone,myAsDes,
                //                   LInt1,LInt2);
                lInt1.Clear();
                lInt1.Append(oe);
                lInt2.Clear();
                Orientation ori1, ori2;
                OffsetTool.OrientSection(oe, cf, of1, out ori1, out ori2);
                //    if (mySide == TopAbs_OUT);
                ori1 = global::Kerf.Engine.Topology.TopAbs.Reverse(ori1);
                lInt1.ChangeFirst().Orient(ori1);
                Store(cf, of1, lInt1, lInt2);

                //------------------------------------------------------
                // Processing of offsets on the ancestors of vertices.
                //------------------------------------------------------
                Vertex[] v = [new(), new()];
                Exploration.Vertices(e, out v[0], out v[1]);
                for (int i = 0; i < 2; i++)
                {
                    if (!mv.Add(v[i]))
                    {
                        continue;
                    }
                    of1.Nullify();
                    NCollection_List<Shape> le = analyse.Ancestors(v[i]);
                    NCollection_List<Shape>.Iterator itLE = new(le);
                    for (; itLE.More(); itLE.Next())
                    {
                        Edge ev = ShapeCasts.Edge(itLE.Value());
                        if (initOffsetFace.HasImage(ev))
                        {
                            //-------------------------------------------------
                            // OF1 parallel face generated by an ancestor edge of V[i].
                            //-------------------------------------------------
                            of1 = ShapeCasts.Face(initOffsetFace.Image(ev).First());
                            oe = ShapeCasts.Edge(initOffsetEdge.Image(v[i]).First());

                            {
                                // Check if OE has pcurve in CF and OF1

                                double f, l;

                                Curve2D? c1 = Tool.CurveOnSurface(oe, cf, out f, out l);
                                Curve2D? c2 = Tool.CurveOnSurface(oe, of1, out f, out l);

                                if (c1 is null || c2 is null)
                                {
                                    continue;
                                }
                            }

                            //--------------------------------------------------
                            // MAj of OE on cap CF.
                            //--------------------------------------------------
                            //              LOE.Clear(); LOE.Append(OE);
                            //              BRepOffset_Tool::TryProject(CF,OF1,LOE,LInt1,LInt2,mySide);
                            //              LInt2.Clear();
                            //              StoreInter3d(CF,OF1,myTouched,NewEdges,InterDone,myAsDes,
                            //                           LInt1,LInt2);
                            lInt1.Clear();
                            lInt1.Append(oe);
                            lInt2.Clear();
                            Orientation o1, o2;
                            OffsetTool.OrientSection(oe, cf, of1, out o1, out o2);
                            //            if (mySide == TopAbs_OUT);
                            o1 = global::Kerf.Engine.Topology.TopAbs.Reverse(o1);
                            lInt1.ChangeFirst().Orient(o1);
                            Store(cf, of1, lInt1, lInt2);
                        }
                    }
                }
            }

            for (exp.Init(cf.Oriented(Forward), ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex v = ShapeCasts.Vertex(exp.Current());
                if (!analyse.HasAncestor(v))
                {
                    continue;
                }
                NCollection_List<Shape> le = analyse.Ancestors(v);
                NCollection_List<Shape>.Iterator itLE = new(le);
                for (; itLE.More(); itLE.Next())
                {
                    Edge ev = ShapeCasts.Edge(itLE.Value());
                    NCollection_List<Shape> lf = analyse.Ancestors(ev);
                    NCollection_List<Shape>.Iterator itLF = new(lf);
                    for (; itLF.More(); itLF.Next())
                    {
                        Face fev = ShapeCasts.Face(itLF.Value());
                        //-------------------------------------------------
                        // OF1 parallel face generated by uneFace ancestor of V[i].
                        //-------------------------------------------------
                        of1 = ShapeCasts.Face(initOffsetFace.Image(fev).First());
                        if (!IsDone(of1, cf))
                        {
                            //-------------------------------------------------------
                            // Find if one of edges of OF1 has no trace in CF.
                            //-------------------------------------------------------
                            NCollection_List<Shape> loe = new();
                            Explorer exp2 = new(of1.Oriented(Forward), ShapeType.Edge);
                            for (; exp2.More(); exp2.Next())
                            {
                                loe.Append(exp2.Current());
                            }
                            //-------------------------------------------------------
                            // If no trace try intersection.
                            //-------------------------------------------------------
                            if (!OffsetTool.TryProject(cf, of1, loe, lInt1, lInt2, _side, _tol)
                                || lInt1.IsEmpty())
                            {
                                OffsetTool.Inter3D(cf, of1, lInt1, lInt2, _side, nullEdge, nullFace, nullFace);
                            }
                            Store(cf, of1, lInt1, lInt2);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Marks the pair of faces as already intersected.</summary>
    public void SetDone(in Face f1, in Face f2)
    {
        if (!_done.ContainsKey(f1))
        {
            NCollection_List<Shape> empty = new();
            _done[f1] = empty;
        }
        _done.GetValue(f1).Append(f2);
        if (!_done.ContainsKey(f2))
        {
            NCollection_List<Shape> empty = new();
            _done[f2] = empty;
        }
        _done.GetValue(f2).Append(f1);
    }

    //=================================================================================================

    /// <summary>Whether the pair of faces has already been treated.</summary>
    public bool IsDone(in Face f1, in Face f2)
    {
        if (_done.ContainsKey(f1))
        {
            NCollection_List<Shape>.Iterator it = new(_done.GetValue(f1));
            for (; it.More(); it.Next())
            {
                if (it.Value().IsSame(f2))
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>The touched faces (the tool's own map).</summary>
    public IndexedSet<Shape> TouchedFaces() => _touched;

    /// <summary>The AsDes tool.</summary>
    public AsDes AsDes() => _asDes;

    /// <summary>The new edges (the tool's own map).</summary>
    public IndexedSet<Shape> NewEdges() => _newEdges;

    //=================================================================================================

    // Stores the intersection results into AsDes
    private void Store(in Face f1, in Face f2, NCollection_List<Shape> lInt1,
                       NCollection_List<Shape> lInt2)
    {
        if (!lInt1.IsEmpty())
        {
            _touched.Add(f1);
            _touched.Add(f2);
            _asDes.Add(f1, lInt1);
            _asDes.Add(f2, lInt2);
            NCollection_List<Shape>.Iterator it = new(lInt1);
            for (; it.More(); it.Next())
            {
                _newEdges.Add(it.Value());
            }
        }
        SetDone(f1, f2);
    }
}
