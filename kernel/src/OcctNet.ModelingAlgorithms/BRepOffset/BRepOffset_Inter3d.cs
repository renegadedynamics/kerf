// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Inter3d.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Inter3d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Fri Dec 26 12:20:14 2003 OCC4455

using static Occt.ChFiDS_TypeOfConcavity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

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
/// unfrozen: a frozen one throws <see cref="TopoDS_LockedShape"/> at the first write. The offsets of
/// <c>MapSF</c> are only read.
/// </remarks>
public sealed class BRepOffset_Inter3d
{
    private readonly BRepAlgo_AsDes myAsDes;
    private readonly NCollection_IndexedMap<TopoDS_Shape> myTouched = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myDone = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_IndexedMap<TopoDS_Shape> myNewEdges = new(TopTools_ShapeMapHasher.Instance);
    private readonly TopAbs_State mySide;
    private readonly double myTol;

    //=================================================================================================

    /// <summary>A tool storing its intersections into <paramref name="AsDes"/>.</summary>
    public BRepOffset_Inter3d(BRepAlgo_AsDes AsDes, TopAbs_State Side, double Tol)
    {
        myAsDes = AsDes;
        mySide = Side;
        myTol = Tol;
    }

    //=================================================================================================

    private static void ExtentEdge(in TopoDS_Face F, in TopoDS_Edge E, out TopoDS_Edge NE)
    {
        TopoDS_Shape aLocalShape = E.EmptyCopied();
        NE = TopoDS.Edge(aLocalShape);
        //  NE = TopoDS::Edge(E.EmptyCopied());

        // Enough for analytic edges, in general case reconstruct the
        // geometry of the edge recalculating the intersection of surfaces.

        NE.Orientation(TopAbs_FORWARD);
        double f, l;
        BRep_Tool.Range(E, out f, out l);
        double length = l - f;
        f -= 100 * length;
        l += 100 * length;

        BRep_Builder B = new();
        B.Range(NE, f, l);
        BRepAdaptor_Curve CE = new(E);
        TopoDS_Vertex V1 = new BRepLib_MakeVertex(CE.Value(f)).InternalVertex();
        TopoDS_Vertex V2 = new BRepLib_MakeVertex(CE.Value(l)).InternalVertex();
        B.Add(NE, V1.Oriented(TopAbs_FORWARD));
        B.Add(NE, V2.Oriented(TopAbs_REVERSED));
        NE.Orientation(E.Orientation());
    }

    //=================================================================================================

    /// <summary>Computes the intersections of the given faces among each other.</summary>
    public void CompletInt(NCollection_List<TopoDS_Shape> SetOfFaces, BRepAlgo_Image InitOffsetFace,
                           Message_ProgressRange theRange)
    {
        //---------------------------------------------------------------
        // Calculate the intersections of offset faces
        // Distinction of intersection between faces // tangents.
        //---------------------------------------------------------------

        // Prepare tools for sorting the bounding boxes
        BOPTools_BoxTree aBBTree = new();
        aBBTree.SetSize(SetOfFaces.Extent());
        //
        NCollection_IndexedDataMap<TopoDS_Shape, Bnd_Box> aMFaces = new(TopTools_ShapeMapHasher.Instance);
        // Construct bounding boxes for faces and add them to the tree
        NCollection_List<TopoDS_Shape>.Iterator aItL = new(SetOfFaces);
        for (; aItL.More(); aItL.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aItL.Value());
            //
            // compute bounding box
            Bnd_Box aBoxF = new();
            BRepBndLib.Add(aF, ref aBoxF);
            //
            int i = aMFaces.Add(aF, aBoxF);
            //
            aBBTree.Add(i, Bnd_Tools.Bnd2BVH(aBoxF));
        }

        // Build BVH
        aBBTree.Build();

        // Perform selection of the pairs
        BOPTools_BoxPairSelector aSelector = new();
        aSelector.SetBVHSets(aBBTree, aBBTree);
        aSelector.SetSame(true);
        aSelector.Select();
        aSelector.Sort();

        // Treat the selected pairs
        NCollection_LinearVector<BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs> aPairs = aSelector.Pairs();
        int aNbPairs = aPairs.Size();
        using Message_ProgressScope aPS = new(theRange, "Complete intersection", aNbPairs);
        for (int iPair = 0; iPair < aNbPairs; ++iPair, aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs aPair = aPairs.Value(iPair);

            TopoDS_Face aF1 = TopoDS.Face(aMFaces.FindKey(Min(aPair.ID1, aPair.ID2)));
            TopoDS_Face aF2 = TopoDS.Face(aMFaces.FindKey(Max(aPair.ID1, aPair.ID2)));

            // intersect faces
            FaceInter(aF1, aF2, InitOffsetFace);
        }
    }

    //=======================================================================
    // function : FaceInter
    // purpose  : Performs intersection of the given faces
    //=======================================================================

    /// <summary>Computes the intersection of a pair of faces.</summary>
    public void FaceInter(in TopoDS_Face F1, in TopoDS_Face F2, BRepAlgo_Image InitOffsetFace)
    {
        NCollection_List<TopoDS_Shape> LInt1 = new(), LInt2 = new();
        TopoDS_Edge NullEdge = new();
        TopoDS_Face NullFace = new();

        if (F1.IsSame(F2))
        {
            return;
        }
        if (IsDone(F1, F2))
        {
            return;
        }

        TopoDS_Shape InitF1 = InitOffsetFace.ImageFrom(F1);
        TopoDS_Shape InitF2 = InitOffsetFace.ImageFrom(F2);
        if (InitF1.IsSame(InitF2))
        {
            return;
        }

        bool InterPipes = (InitF2.ShapeType() == TopAbs_EDGE && InitF1.ShapeType() == TopAbs_EDGE);
        bool InterFaces = (InitF1.ShapeType() == TopAbs_FACE && InitF2.ShapeType() == TopAbs_FACE);
        NCollection_List<TopoDS_Shape> LE = new(), LV = new();
        LInt1.Clear();
        LInt2.Clear();
        if (BRepOffset_Tool.FindCommonShapes(F1, F2, LE, LV) || myAsDes.HasCommonDescendant(F1, F2, LE))
        {
            //-------------------------------------------------
            // F1 and F2 share shapes.
            //-------------------------------------------------
            if (LE.IsEmpty() && !LV.IsEmpty())
            {
                if (InterPipes)
                {
                    //----------------------
                    // tubes share a vertex.
                    //----------------------
                    TopoDS_Edge EE1 = TopoDS.Edge(InitF1);
                    TopoDS_Edge EE2 = TopoDS.Edge(InitF2);
                    TopoDS_Vertex[] VE1 = [new(), new()], VE2 = [new(), new()];
                    TopExp.Vertices(EE1, out VE1[0], out VE1[1]);
                    TopExp.Vertices(EE2, out VE2[0], out VE2[1]);
                    TopoDS_Vertex V = new();
                    for (int i = 0; i < 2; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            if (VE1[i].IsSame(VE2[j]))
                            {
                                V = VE1[i];
                            }
                        }
                    }
                    if (!InitOffsetFace.HasImage(V))
                    { // no sphere
                        BRepOffset_Tool.PipeInter(F1, F2, LInt1, LInt2, mySide);
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
                    if (InterFaces)
                    {
                        if (BRepOffset_Tool.FindCommonShapes(TopoDS.Face(InitF1), TopoDS.Face(InitF2), LE, LV))
                        {
                            if (!LE.IsEmpty())
                            {
                                BRepOffset_Tool.Inter3D(F1, F2, LInt1, LInt2, mySide, NullEdge, NullFace, NullFace);
                            }
                        }
                        else
                        {
                            BRepOffset_Tool.Inter3D(F1, F2, LInt1, LInt2, mySide, NullEdge, NullFace, NullFace);
                        }
                    }
                }
            }
        }
        else
        {
            if (InterPipes)
            {
                BRepOffset_Tool.PipeInter(F1, F2, LInt1, LInt2, mySide);
            }
            else
            {
                BRepOffset_Tool.Inter3D(F1, F2, LInt1, LInt2, mySide, NullEdge, NullFace, NullFace);
            }
        }
        Store(F1, F2, LInt1, LInt2);
    }

    //=================================================================================================

    /// <summary>Computes the connections of the offset faces that have to be connected by arcs.</summary>
    public void ConnexIntByArc(NCollection_List<TopoDS_Shape> SetOfFaces, in TopoDS_Shape ShapeInit,
                               BRepOffset_Analyse Analyse, BRepAlgo_Image InitOffsetFace,
                               Message_ProgressRange theRange)
    {
        ChFiDS_TypeOfConcavity OT = ChFiDS_Concave;
        if (mySide == TopAbs_OUT)
        {
            OT = ChFiDS_Convex;
        }
        TopExp_Explorer Exp = new(ShapeInit, TopAbs_EDGE);
        NCollection_List<TopoDS_Shape> LInt1 = new(), LInt2 = new();
        TopoDS_Face F1 = new(), F2 = new();
        TopoDS_Edge NullEdge = new();
        TopoDS_Face NullFace = new();
        using Message_ProgressScope aPSOuter = new(theRange, null, 2);
        using Message_ProgressScope aPSIntF = new(aPSOuter.Next(), "Intersecting offset faces", 1, true);
        //---------------------------------------------------------------------
        // etape 1 : Intersection of faces // corresponding to the initial faces
        //           separated by a concave edge if offset > 0, otherwise convex.
        //---------------------------------------------------------------------
        for (; Exp.More(); Exp.Next(), aPSIntF.Next())
        {
            if (!aPSIntF.More())
            {
                return;
            }
            TopoDS_Edge E = TopoDS.Edge(Exp.Current());
            NCollection_List<BRepOffset_Interval> L = Analyse.Type(E);
            if (!L.IsEmpty() && L.First().Type() == OT)
            {
                //-----------------------------------------------------------
                // edge is of the proper type , return adjacent faces.
                //-----------------------------------------------------------
                NCollection_List<TopoDS_Shape> Anc = Analyse.Ancestors(E);
                if (Anc.Extent() == 2)
                {

                    TopoDS_Face InitF1 = TopoDS.Face(Anc.First());
                    TopoDS_Face InitF2 = TopoDS.Face(Anc.Last());
                    F1 = TopoDS.Face(InitOffsetFace.Image(InitF1).First());
                    F2 = TopoDS.Face(InitOffsetFace.Image(InitF2).First());
                    if (!IsDone(F1, F2))
                    {
                        BRepOffset_Tool.Inter3D(F1, F2, LInt1, LInt2, mySide, E, InitF1, InitF2);
                        Store(F1, F2, LInt1, LInt2);
                    }
                }
            }
        }
        //---------------------------------------------------------------------
        // etape 2 : Intersections of tubes sharing a vertex without sphere with:
        //           - tubes on each other edge sharing the vertex
        //           - faces containing an edge connected to vertex that has no tubes.
        //---------------------------------------------------------------------
        TopoDS_Vertex[] V = [new(), new()];
        NCollection_List<TopoDS_Shape>.Iterator it = new();
        using Message_ProgressScope aPSIntT = new(aPSOuter.Next(), "Intersecting tubes", 1, true);
        for (Exp.Init(ShapeInit, TopAbs_EDGE); Exp.More(); Exp.Next(), aPSIntT.Next())
        {
            if (!aPSIntT.More())
            {
                return;
            }
            TopoDS_Edge E1 = TopoDS.Edge(Exp.Current());
            if (InitOffsetFace.HasImage(E1))
            {
                //---------------------------
                // E1 generated a tube.
                //---------------------------
                F1 = TopoDS.Face(InitOffsetFace.Image(E1).First());
                TopExp.Vertices(E1, out V[0], out V[1]);
                NCollection_List<TopoDS_Shape> AncE1 = Analyse.Ancestors(E1);

                for (int i = 0; i < 2; i++)
                {
                    if (!InitOffsetFace.HasImage(V[i]))
                    {
                        //-----------------------------
                        // the vertex has no sphere.
                        //-----------------------------
                        NCollection_List<TopoDS_Shape> Anc = Analyse.Ancestors(V[i]);
                        NCollection_List<TopoDS_Shape> TangOnV = new();
                        Analyse.TangentEdges(E1, V[i], TangOnV);
                        NCollection_Map<TopoDS_Shape> MTEV = new(TopTools_ShapeMapHasher.Instance);
                        for (it.Initialize(TangOnV); it.More(); it.Next())
                        {
                            MTEV.Add(it.Value());
                        }
                        for (it.Initialize(Anc); it.More(); it.Next())
                        {
                            TopoDS_Edge E2 = TopoDS.Edge(it.Value());
                            //  Modified by skv - Fri Jan 16 16:27:54 2004 OCC4455 Begin
                            //            if (E1.IsSame(E2) || MTEV.Contains(E2)) continue;
                            bool isToSkip = false;

                            if (!E1.IsSame(E2))
                            {
                                NCollection_List<BRepOffset_Interval> aL = Analyse.Type(E2);

                                isToSkip =
                                  (MTEV.Contains(E2) && (aL.IsEmpty() || (!aL.IsEmpty() && aL.First().Type() != OT)));
                            }

                            if (E1.IsSame(E2) || isToSkip)
                            {
                                continue;
                            }
                            //  Modified by skv - Fri Jan 16 16:27:54 2004 OCC4455 End
                            if (InitOffsetFace.HasImage(E2))
                            {
                                //-----------------------------
                                // E2 generated a tube.
                                //-----------------------------
                                F2 = TopoDS.Face(InitOffsetFace.Image(E2).First());
                                if (!IsDone(F1, F2))
                                {
                                    //---------------------------------------------------------------------
                                    // Intersection tube/tube if the edges are not tangent (AFINIR).
                                    //----------------------------------------------------------------------
                                    BRepOffset_Tool.PipeInter(F1, F2, LInt1, LInt2, mySide);
                                    Store(F1, F2, LInt1, LInt2);
                                }
                            }
                            else
                            {
                                //-------------------------------------------------------
                                // Intersection of the tube of E1 with faces //
                                // to face containing E2 if they are not tangent
                                // to the tube or if E2 is not a tangent edge.
                                //-------------------------------------------------------
                                NCollection_List<BRepOffset_Interval> L = Analyse.Type(E2);
                                if (!L.IsEmpty() && L.First().Type() == ChFiDS_Tangential)
                                {
                                    continue;
                                }
                                NCollection_List<TopoDS_Shape> AncE2 = Analyse.Ancestors(E2);
                                bool TangentFaces = false;
                                if (AncE2.Extent() == 2)
                                {
                                    TopoDS_Face InitF2 = TopoDS.Face(AncE2.First());
                                    TangentFaces = (InitF2.IsSame(AncE1.First()) || InitF2.IsSame(AncE1.Last()));
                                    if (!TangentFaces)
                                    {
                                        F2 = TopoDS.Face(InitOffsetFace.Image(InitF2).First());
                                        if (!IsDone(F1, F2))
                                        {
                                            BRepOffset_Tool.Inter3D(F1,
                                                                    F2,
                                                                    LInt1,
                                                                    LInt2,
                                                                    mySide,
                                                                    NullEdge,
                                                                    NullFace,
                                                                    NullFace);
                                            Store(F1, F2, LInt1, LInt2);
                                        }
                                    }
                                    InitF2 = TopoDS.Face(AncE2.Last());
                                    TangentFaces = (InitF2.IsSame(AncE1.First()) || InitF2.IsSame(AncE1.Last()));
                                    if (!TangentFaces)
                                    {
                                        F2 = TopoDS.Face(InitOffsetFace.Image(InitF2).First());
                                        if (!IsDone(F1, F2))
                                        {
                                            BRepOffset_Tool.Inter3D(F1,
                                                                    F2,
                                                                    LInt1,
                                                                    LInt2,
                                                                    mySide,
                                                                    NullEdge,
                                                                    NullFace,
                                                                    NullFace);
                                            Store(F1, F2, LInt1, LInt2);
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
    public void ConnexIntByInt(in TopoDS_Shape SI, NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> MapSF,
                               BRepOffset_Analyse Analyse, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MES,
                               NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Build,
                               NCollection_List<TopoDS_Shape> Failed, Message_ProgressRange theRange,
                               bool bIsPlanar = false)
    {
        NCollection_IndexedMap<TopoDS_Shape> VEmap = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Face F1 = new(), F2 = new(), OF1 = new(), OF2 = new(), NF1 = new(), NF2 = new();
        TopAbs_State CurSide = mySide;
        BRep_Builder B = new();
        bool bEdge;
        int i, aNb = 0;
        NCollection_List<TopoDS_Shape>.Iterator it = new(), it1 = new(), itF1 = new(), itF2 = new();
        //
        TopExp.MapShapes(SI, TopAbs_EDGE, VEmap);
        // Take the vertices for treatment
        using Message_ProgressScope aPSOuter = new(theRange, null, 10);
        if (bIsPlanar)
        {
            aNb = VEmap.Extent();
            for (i = 1; i <= aNb; ++i)
            {
                TopoDS_Edge aE = TopoDS.Edge(VEmap[i]);
                TopoDS_Shape aFGen = Analyse.Generated(aE);
                if (!aFGen.IsNull())
                {
                    TopExp.MapShapes(aFGen, TopAbs_EDGE, VEmap);
                }
            }

            // Add vertices for treatment
            TopExp.MapShapes(SI, TopAbs_VERTEX, VEmap);

            for (NCollection_List<TopoDS_Shape>.Iterator itNF = new(Analyse.NewFaces()); itNF.More(); itNF.Next())
            {
                TopExp.MapShapes(itNF.Value(), TopAbs_VERTEX, VEmap);
            }
        }
        //
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVLF1 = new(TopTools_ShapeMapHasher.Instance),
                                                                         aDMVLF2 = new(TopTools_ShapeMapHasher.Instance),
                                                                         aDMIntFF = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMIntE = new(TopTools_ShapeMapHasher.Instance);
        //
        if (bIsPlanar)
        {
            // Find internal edges in the faces to skip them while preparing faces
            // for intersection through vertices
            NCollection_DataMap<TopoDS_Shape, NCollection_Map<TopoDS_Shape>> aDMFEI = new(TopTools_ShapeMapHasher.Instance);
            {
                for (TopExp_Explorer expF = new(SI, TopAbs_FACE); expF.More(); expF.Next())
                {
                    TopoDS_Shape aFx = expF.Current();

                    NCollection_Map<TopoDS_Shape> aMEI = new(TopTools_ShapeMapHasher.Instance);
                    for (TopExp_Explorer expE = new(aFx, TopAbs_EDGE); expE.More(); expE.Next())
                    {
                        TopoDS_Shape aEx = expE.Current();
                        if (aEx.Orientation() != TopAbs_FORWARD && aEx.Orientation() != TopAbs_REVERSED)
                        {
                            aMEI.Add(aEx);
                        }
                    }
                    if (!aMEI.IsEmpty())
                    {
                        aDMFEI.Bind(aFx, aMEI);
                    }
                }
            }

            // Analyze faces connected through vertices
            for (i = aNb + 1, aNb = VEmap.Extent(); i <= aNb; ++i)
            {
                if (!aPSOuter.More())
                {
                    return;
                }
                TopoDS_Shape aS = VEmap[i];
                if (aS.ShapeType() != TopAbs_VERTEX)
                {
                    continue;
                }

                // Find faces connected to the vertex
                NCollection_List<TopoDS_Shape> aLF = new();
                {
                    NCollection_List<TopoDS_Shape> aLE = Analyse.Ancestors(aS);
                    for (NCollection_List<TopoDS_Shape>.Iterator itLE = new(aLE); itLE.More(); itLE.Next())
                    {
                        NCollection_List<TopoDS_Shape> aLEA = Analyse.Ancestors(itLE.Value());
                        for (NCollection_List<TopoDS_Shape>.Iterator itLEA = new(aLEA); itLEA.More(); itLEA.Next())
                        {
                            if (!aLF.Contains(itLEA.Value()))
                            {
                                aLF.Append(itLEA.Value());
                            }
                        }
                    }
                }

                if (aLF.Extent() < 2)
                {
                    continue;
                }

                // build lists of faces connected to the same vertex by looking for
                // the pairs in which the vertex is alone (not connected to shared edges)
                NCollection_List<TopoDS_Shape> aLF1 = new(), aLF2 = new();

                it.Initialize(aLF);
                for (; it.More(); it.Next())
                {
                    TopoDS_Shape aFV1 = it.Value();

                    // get edges of first face connected to current vertex
                    NCollection_Map<TopoDS_Shape> aME = new(TopTools_ShapeMapHasher.Instance);
                    NCollection_Map<TopoDS_Shape>? pF1Internal =
                      aDMFEI.Find(aFV1, out NCollection_Map<TopoDS_Shape> aF1Internal) ? aF1Internal : null;
                    NCollection_List<TopoDS_Shape>? pLE1 = Analyse.Descendants(aFV1);
                    if (pLE1 is null)
                    {
                        continue;
                    }
                    NCollection_List<TopoDS_Shape>.Iterator itLE1 = new(pLE1);
                    for (; itLE1.More(); itLE1.Next())
                    {
                        TopoDS_Shape aE = itLE1.Value();
                        if (pF1Internal is not null && pF1Internal.Contains(aE))
                        {
                            break;
                        }

                        for (TopoDS_Iterator aItV = new(aE); aItV.More(); aItV.Next())
                        {
                            if (aS.IsSame(aItV.Value()))
                            {
                                aME.Add(aE);
                                break;
                            }
                        }
                    }
                    if (itLE1.More())
                    {
                        continue;
                    }

                    // get to the next face in the list
                    it1.Reset(aLF, it.Index); // it1 = it
                    for (it1.Next(); it1.More(); it1.Next())
                    {
                        TopoDS_Face aFV2 = TopoDS.Face(it1.Value());

                        NCollection_Map<TopoDS_Shape>? pF2Internal =
                          aDMFEI.Find(aFV2, out NCollection_Map<TopoDS_Shape> aF2Internal) ? aF2Internal : null;

                        NCollection_List<TopoDS_Shape>? pLE2 = Analyse.Descendants(aFV2);
                        if (pLE2 is null)
                        {
                            continue;
                        }
                        NCollection_List<TopoDS_Shape>.Iterator itLE2 = new(pLE2);
                        for (; itLE2.More(); itLE2.Next())
                        {
                            TopoDS_Shape aEV2 = itLE2.Value();
                            if (!aME.Contains(aEV2))
                            {
                                continue;
                            }

                            if (pF2Internal is not null && pF2Internal.Contains(aEV2))
                            {
                                // Avoid intersection of faces connected by internal edge
                                break;
                            }

                            if (Analyse.HasAncestor(aEV2) && Analyse.Ancestors(aEV2).Extent() == 2)
                            {
                                // Faces will be intersected through the edge
                                break;
                            }
                        }

                        if (!itLE2.More())
                        {
                            aLF1.Append(aFV1);
                            aLF2.Append(aFV2);
                        }
                    }
                }
                //
                if (aLF1.Extent() != 0)
                {
                    aDMVLF1.Bind(aS, aLF1);
                    aDMVLF2.Bind(aS, aLF2);
                }
            }
        }
        //
        aNb = VEmap.Extent();
        using Message_ProgressScope aPSInter = new(aPSOuter.Next(8), "Intersecting offset faces", aNb);
        for (i = 1; i <= aNb; ++i, aPSInter.Next())
        {
            if (!aPSInter.More())
            {
                return;
            }
            TopoDS_Shape aS = VEmap[i];
            //
            TopoDS_Edge E = new();
            NCollection_List<TopoDS_Shape> aLF1 = new(), aLF2 = new();
            //
            bEdge = (aS.ShapeType() == TopAbs_EDGE);
            if (bEdge)
            {
                // faces connected by the edge
                E = TopoDS.Edge(aS);
                //
                NCollection_List<BRepOffset_Interval> L = Analyse.Type(E);
                if (L.IsEmpty())
                {
                    continue;
                }
                //
                ChFiDS_TypeOfConcavity OT = L.First().Type();
                if (OT != ChFiDS_Convex && OT != ChFiDS_Concave)
                {
                    continue;
                }
                //
                if (OT == ChFiDS_Concave)
                {
                    CurSide = TopAbs_IN;
                }
                else
                {
                    CurSide = TopAbs_OUT;
                }
                //-----------------------------------------------------------
                // edge is of the proper type, return adjacent faces.
                //-----------------------------------------------------------
                NCollection_List<TopoDS_Shape> Anc = Analyse.Ancestors(E);
                if (Anc.Extent() != 2)
                {
                    continue;
                }
                //
                F1 = TopoDS.Face(Anc.First());
                F2 = TopoDS.Face(Anc.Last());
                //
                aLF1.Append(F1);
                aLF2.Append(F2);
            }
            else
            {
                if (!aDMVLF1.IsBound(aS))
                {
                    continue;
                }
                //
                aLF1.Assign(aDMVLF1.Find(aS));
                aLF2.Assign(aDMVLF2.Find(aS));
                //
                CurSide = mySide;
            }
            //
            itF1.Initialize(aLF1);
            itF2.Initialize(aLF2);
            for (; itF1.More() && itF2.More(); itF1.Next(), itF2.Next())
            {
                F1 = TopoDS.Face(itF1.Value());
                F2 = TopoDS.Face(itF2.Value());
                //
                OF1 = TopoDS.Face(MapSF[F1].InternalFace());
                OF2 = TopoDS.Face(MapSF[F2].InternalFace());
                if (!MES.IsBound(OF1))
                {
                    bool enlargeU = true;
                    bool enlargeVfirst = true, enlargeVlast = true;
                    BRepOffset_Tool.CheckBounds(F1, Analyse, out enlargeU, out enlargeVfirst, out enlargeVlast);
                    BRepOffset_Tool.EnLargeFace(OF1, ref NF1, true, true, enlargeU, enlargeVfirst, enlargeVlast);
                    MES.Bind(OF1, NF1);
                }
                else
                {
                    NF1 = TopoDS.Face(MES[OF1]);
                }
                //
                if (!MES.IsBound(OF2))
                {
                    bool enlargeU = true;
                    bool enlargeVfirst = true, enlargeVlast = true;
                    BRepOffset_Tool.CheckBounds(F2, Analyse, out enlargeU, out enlargeVfirst, out enlargeVlast);
                    BRepOffset_Tool.EnLargeFace(OF2, ref NF2, true, true, enlargeU, enlargeVfirst, enlargeVlast);
                    MES.Bind(OF2, NF2);
                }
                else
                {
                    NF2 = TopoDS.Face(MES[OF2]);
                }
                //
                if (!IsDone(NF1, NF2))
                {
                    NCollection_List<TopoDS_Shape> LInt1 = new(), LInt2 = new();
                    BRepOffset_Tool.Inter3D(NF1, NF2, LInt1, LInt2, CurSide, E, F1, F2);
                    SetDone(NF1, NF2);
                    if (!LInt1.IsEmpty())
                    {
                        Store(NF1, NF2, LInt1, LInt2);
                        //
                        TopoDS_Compound C = new();
                        B.MakeCompound(ref C);
                        //
                        if (Build.IsBound(aS))
                        {
                            TopoDS_Shape aSE = Build[aS];
                            TopExp_Explorer aExp = new(aSE, TopAbs_EDGE);
                            for (; aExp.More(); aExp.Next())
                            {
                                TopoDS_Shape aNE = aExp.Current();
                                B.Add(C, aNE);
                            }
                        }
                        //
                        it.Initialize(LInt1);
                        for (; it.More(); it.Next())
                        {
                            TopoDS_Shape aNE = it.Value();
                            B.Add(C, aNE);
                            //
                            // keep connection from new edge to shape from which it was created
                            NCollection_List<TopoDS_Shape> pLS = aDMIntE[aDMIntE.Add(aNE, new NCollection_List<TopoDS_Shape>())];
                            pLS.Append(aS);
                            // keep connection to faces created the edge as well
                            NCollection_List<TopoDS_Shape> pLFF = aDMIntFF.Bound(aNE, new NCollection_List<TopoDS_Shape>());
                            pLFF.Append(F1);
                            pLFF.Append(F2);
                        }
                        //
                        Build.Bind(aS, C);
                    }
                    else
                    {
                        Failed.Append(aS);
                    }
                }
                else
                { // IsDone(NF1,NF2)
                    //  Modified by skv - Fri Dec 26 12:20:13 2003 OCC4455 Begin
                    NCollection_List<TopoDS_Shape> aLInt1 = myAsDes.Descendant(NF1);
                    NCollection_List<TopoDS_Shape> aLInt2 = myAsDes.Descendant(NF2);

                    if (!aLInt1.IsEmpty())
                    {
                        TopoDS_Compound C = new();
                        B.MakeCompound(ref C);
                        //
                        if (Build.IsBound(aS))
                        {
                            TopoDS_Shape aSE = Build[aS];
                            TopExp_Explorer aExp = new(aSE, TopAbs_EDGE);
                            for (; aExp.More(); aExp.Next())
                            {
                                TopoDS_Shape aNE = aExp.Current();
                                B.Add(C, aNE);
                            }
                        }
                        //
                        for (it.Initialize(aLInt1); it.More(); it.Next())
                        {
                            TopoDS_Shape anE1 = it.Value();
                            //
                            for (it1.Initialize(aLInt2); it1.More(); it1.Next())
                            {
                                TopoDS_Shape anE2 = it1.Value();
                                if (anE1.IsSame(anE2))
                                {
                                    B.Add(C, anE1);
                                    //
                                    if (aDMIntE.FindFromKey(anE1, out NCollection_List<TopoDS_Shape> pLS))
                                    {
                                        pLS.Append(aS);
                                    }
                                }
                            }
                        }
                        Build.Bind(aS, C);
                    }
                    else
                    {
                        Failed.Append(aS);
                    }
                }
            }
            //  Modified by skv - Fri Dec 26 12:20:14 2003 OCC4455 End
        }
        //
        // create unique intersection for each localized shared part
        aNb = aDMIntE.Extent();
        using Message_ProgressScope aPSPostTreat = new(aPSOuter.Next(2), "Creating unique intersection", aNb);
        for (i = 1; i <= aNb; ++i, aPSPostTreat.Next())
        {
            if (!aPSPostTreat.More())
            {
                return;
            }
            NCollection_List<TopoDS_Shape> aLS = aDMIntE[i];
            if (aLS.Extent() < 2)
            {
                continue;
            }
            //
            // intersection edge
            TopoDS_Edge aE = TopoDS.Edge(aDMIntE.FindKey(i));
            // faces created the edge
            NCollection_List<TopoDS_Shape> aLFF = aDMIntFF.Find(aE);
            TopoDS_Shape aF1 = aLFF.First();
            TopoDS_Shape aF2 = aLFF.Last();

            // Build really localized blocks from the original shapes in <aLS>:
            // 1. Find edges from original faces connected to two or more shapes in <aLS>;
            // 2. Make connexity blocks from edges in <aLS> and found connection edges;
            // 3. Check if the vertices from <aLS> are not connected by these connection edges:
            //    a. If so - add these vertices to Connexity Block containing the corresponding
            //       connexity edge;
            //    b. If not - add this vertex to list of connexity blocks
            // 4. Create unique intersection edge for each connexity block

            // list of vertices
            NCollection_List<TopoDS_Shape> aLV = new();
            // compound of edges to build connexity blocks
            TopoDS_Compound aCE = new();
            B.MakeCompound(ref aCE);
            NCollection_Map<TopoDS_Shape> aMS = new(TopTools_ShapeMapHasher.Instance);
            NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLS);
            for (; aItLS.More(); aItLS.Next())
            {
                TopoDS_Shape aS = aItLS.Value();
                aMS.Add(aS);
                if (aS.ShapeType() == TopAbs_EDGE)
                {
                    B.Add(aCE, aS);
                }
                else
                {
                    aLV.Append(aS);
                }
            }
            //
            // look for additional edges to connect the shared parts
            NCollection_Map<TopoDS_Shape> aMEConnection = new(TopTools_ShapeMapHasher.Instance);
            for (int j = 0; j < 2; ++j)
            {
                TopoDS_Shape aF = j == 0 ? aF1 : aF2;
                //
                TopExp_Explorer aExp = new(aF, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aEF = aExp.Current();
                    if (aMS.Contains(aEF) || aMEConnection.Contains(aEF))
                    {
                        continue;
                    }
                    //
                    TopExp.Vertices(TopoDS.Edge(aEF), out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
                    //
                    // find parts to which the edge is connected
                    int iCounter = 0;
                    aItLS.Initialize(aLS);
                    for (; aItLS.More(); aItLS.Next())
                    {
                        TopoDS_Shape aS = aItLS.Value();
                        // iterator is not suitable here, because aS may be a vertex
                        TopExp_Explorer aExpV = new(aS, TopAbs_VERTEX);
                        for (; aExpV.More(); aExpV.Next())
                        {
                            TopoDS_Shape aV = aExpV.Current();
                            if (aV.IsSame(aV1) || aV.IsSame(aV2))
                            {
                                ++iCounter;
                                break;
                            }
                        }
                    }
                    //
                    if (iCounter >= 2)
                    {
                        B.Add(aCE, aEF);
                        aMEConnection.Add(aEF);
                    }
                }
            }
            //
            NCollection_List<TopoDS_Shape> aLCBE = new();
            BOPTools_AlgoTools.MakeConnexityBlocks(aCE, TopAbs_VERTEX, TopAbs_EDGE, aLCBE);
            //
            // create connexity blocks for alone vertices
            NCollection_List<TopoDS_Shape> aLCBV = new();
            NCollection_List<TopoDS_Shape>.Iterator aItLV = new(aLV);
            for (; aItLV.More(); aItLV.Next())
            {
                TopoDS_Shape aV = aItLV.Value();
                // check if this vertex is contained in some connexity block of edges
                NCollection_List<TopoDS_Shape>.Iterator aItLCB1 = new(aLCBE);
                for (; aItLCB1.More(); aItLCB1.Next())
                {
                    ref TopoDS_Shape aCB = ref aItLCB1.ChangeValue();
                    TopExp_Explorer aExpV = new(aCB, TopAbs_VERTEX);
                    for (; aExpV.More(); aExpV.Next())
                    {
                        if (aV.IsSame(aExpV.Current()))
                        {
                            B.Add(aCB, aV);
                            break;
                        }
                    }
                    if (aExpV.More())
                    {
                        break;
                    }
                }
                //
                if (!aItLCB1.More())
                {
                    TopoDS_Compound aCV = new();
                    B.MakeCompound(ref aCV);
                    B.Add(aCV, aV);
                    aLCBV.Append(aCV);
                }
            }
            //
            aLCBE.Append(aLCBV);
            //
            if (aLCBE.Extent() == 1)
            {
                continue;
            }
            //
            TopoDS_Shape aNF1 = MES[MapSF[aF1].InternalFace()];
            TopoDS_Shape aNF2 = MES[MapSF[aF2].InternalFace()];
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLCB = new(aLCBE);
            for (aItLCB.Next(); aItLCB.More(); aItLCB.Next())
            {
                // make new edge with different tedge instance
                TopoDS_Edge aNewEdge;
                TopoDS_Vertex aV1, aV2;
                double aT1, aT2;
                //
                TopExp.Vertices(aE, out aV1, out aV2);
                BRep_Tool.Range(aE, out aT1, out aT2);
                //
                BOPTools_AlgoTools.MakeSplitEdge(aE, aV1, aT1, aV2, aT2, out aNewEdge);
                //
                myAsDes.Add(aNF1, aNewEdge);
                myAsDes.Add(aNF2, aNewEdge);
                //
                TopoDS_Shape aCB = aItLCB.Value();
                TopoDS_Iterator aItCB = new(aCB);
                for (; aItCB.More(); aItCB.Next())
                {
                    TopoDS_Shape aS = aItCB.Value();
                    if (aMEConnection.Contains(aS))
                    {
                        continue;
                    }
                    ref TopoDS_Shape aCI = ref Build.ChangeFind(aS);
                    //
                    TopoDS_Compound aNewCI = new();
                    B.MakeCompound(ref aNewCI);
                    TopExp_Explorer aExp = new(aCI, TopAbs_EDGE);
                    for (; aExp.More(); aExp.Next())
                    {
                        TopoDS_Shape aSx = aExp.Current();
                        if (!aSx.IsSame(aE))
                        {
                            B.Add(aNewCI, aSx);
                        }
                    }
                    B.Add(aNewCI, aNewEdge);
                    aCI = aNewCI;
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Computes the intersections with the not offset (context) faces.</summary>
    public void ContextIntByInt(NCollection_IndexedMap<TopoDS_Shape> ContextFaces, bool ExtentContext,
                                NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> MapSF,
                                BRepOffset_Analyse Analyse, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MES,
                                NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Build,
                                NCollection_List<TopoDS_Shape> Failed, Message_ProgressRange theRange,
                                bool bIsPlanar = false)
    {
        NCollection_Map<TopoDS_Shape> MV = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new();
        TopoDS_Face OF = new(), NF = new(), WCF = new();
        TopoDS_Edge OE = new();
        TopoDS_Compound C = new();
        BRep_Builder B = new();
        NCollection_List<TopoDS_Shape>.Iterator it = new(), itF = new();
        int i, j, aNb, aNbVE;
        bool bEdge;

        aNb = ContextFaces.Extent();
        for (i = 1; i <= aNb; i++)
        {
            TopoDS_Face CF = TopoDS.Face(ContextFaces[i]);
            myTouched.Add(CF);
            if (ExtentContext)
            {
                BRepOffset_Tool.EnLargeFace(CF, ref NF, false, false);
                MES.Bind(CF, NF);
            }
        }
        TopAbs_State Side = TopAbs_OUT;

        using Message_ProgressScope aPS = new(theRange, "Intersecting with deepening faces", aNb);
        for (i = 1; i <= aNb; i++, aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Face CF = TopoDS.Face(ContextFaces[i]);
            if (ExtentContext)
            {
                WCF = TopoDS.Face(MES[CF]);
            }
            else
            {
                WCF = CF;
            }

            NCollection_IndexedMap<TopoDS_Shape> VEmap = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(CF.Oriented(TopAbs_FORWARD), TopAbs_EDGE, VEmap);
            //
            if (bIsPlanar)
            {
                TopExp.MapShapes(CF.Oriented(TopAbs_FORWARD), TopAbs_VERTEX, VEmap);
            }
            //
            aNbVE = VEmap.Extent();
            for (j = 1; j <= aNbVE; ++j)
            {
                TopoDS_Shape aS = VEmap[j];
                //
                bEdge = (aS.ShapeType() == TopAbs_EDGE);
                //
                TopoDS_Edge E = new();
                NCollection_List<TopoDS_Shape> Anc = new();
                //
                if (bEdge)
                {
                    // faces connected by the edge
                    //
                    E = TopoDS.Edge(aS);
                    if (!Analyse.HasAncestor(E))
                    {
                        //----------------------------------------------------------------
                        // the edges of faces of context that are not in the initial shape
                        // can appear in the result.
                        //----------------------------------------------------------------
                        if (!ExtentContext)
                        {
                            myAsDes.Add(CF, E);
                            myNewEdges.Add(E);
                        }
                        else
                        {
                            if (!MES.IsBound(E))
                            {
                                double f, l, Tol;
                                BRep_Tool.Range(E, out f, out l);
                                Tol = BRep_Tool.Tolerance(E);
                                ExtentEdge(CF, E, out TopoDS_Edge NE);
                                TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                                NE.Orientation(TopAbs_FORWARD);
                                myAsDes.Add(NE, V1.Oriented(TopAbs_REVERSED));
                                myAsDes.Add(NE, V2.Oriented(TopAbs_FORWARD));
                                TopoDS_Shape aLocalShape = V1.Oriented(TopAbs_INTERNAL);
                                B.UpdateVertex(TopoDS.Vertex(aLocalShape), f, NE, Tol);
                                aLocalShape = V2.Oriented(TopAbs_INTERNAL);
                                B.UpdateVertex(TopoDS.Vertex(aLocalShape), l, NE, Tol);
                                //            B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),f,NE,Tol);
                                //            B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),l,NE,Tol);
                                NE.Orientation(E.Orientation());
                                myAsDes.Add(CF, NE);
                                myNewEdges.Add(NE);
                                MES.Bind(E, NE);
                            }
                            else
                            {
                                TopoDS_Shape NE = MES[E];
                                TopoDS_Shape aLocalShape = NE.Oriented(E.Orientation());
                                myAsDes.Add(CF, aLocalShape);
                                //            myAsDes->Add(CF,NE.Oriented(E.Orientation()));
                            }
                        }
                        continue;
                    }
                    Anc.Assign(Analyse.Ancestors(E));
                }
                else
                {
                    // faces connected by the vertex
                    //
                    if (!Analyse.HasAncestor(aS))
                    {
                        continue;
                    }
                    //
                    NCollection_List<TopoDS_Shape> aLE = Analyse.Ancestors(aS);
                    it.Initialize(aLE);
                    for (; it.More(); it.Next())
                    {
                        TopoDS_Edge aE = TopoDS.Edge(it.Value());
                        //
                        if (BRep_Tool.Degenerated(aE))
                        {
                            continue;
                        }
                        //
                        if (VEmap.Contains(aE))
                        {
                            continue;
                        }
                        //
                        NCollection_List<TopoDS_Shape> aLF = Analyse.Ancestors(aE);
                        itF.Initialize(aLF);
                        for (; itF.More(); itF.Next())
                        {
                            TopoDS_Shape aF = itF.Value();
                            bool bAdd = true;
                            exp.Init(aF, TopAbs_EDGE);
                            for (; exp.More() && bAdd; exp.Next())
                            {
                                TopoDS_Shape aEF = exp.Current();
                                bAdd = !VEmap.Contains(aEF);
                            }
                            if (bAdd)
                            {
                                Anc.Append(aF);
                            }
                        }
                    }
                }
                //
                itF.Initialize(Anc);
                for (; itF.More(); itF.Next())
                {
                    TopoDS_Face F = TopoDS.Face(itF.Value());
                    OF = TopoDS.Face(MapSF[F].InternalFace());
                    TopoDS_Shape aLocalShape = MapSF[F].InternalGenerated(E);
                    OE = TopoDS.Edge(aLocalShape);
                    //      OE = TopoDS::Edge(MapSF(F).Generated(E));
                    if (!MES.IsBound(OF))
                    {
                        BRepOffset_Tool.EnLargeFace(OF, ref NF, true, true);
                        MES.Bind(OF, NF);
                    }
                    else
                    {
                        NF = TopoDS.Face(MES[OF]);
                    }
                    if (!IsDone(NF, CF))
                    {
                        NCollection_List<TopoDS_Shape> LInt1 = new(), LInt2 = new();
                        NCollection_List<TopoDS_Shape> LOE = new();
                        LOE.Append(OE);
                        BRepOffset_Tool.Inter3D(WCF, NF, LInt1, LInt2, Side, E, CF, F);
                        SetDone(NF, CF);
                        if (!LInt1.IsEmpty())
                        {
                            Store(CF, NF, LInt1, LInt2);
                            if ((LInt1.Extent() == 1) && !Build.IsBound(aS))
                            {
                                Build.Bind(aS, LInt1.First());
                            }
                            else
                            {
                                B.MakeCompound(ref C);
                                if (Build.IsBound(aS))
                                {
                                    TopoDS_Shape aSE = Build[aS];
                                    exp.Init(aSE, TopAbs_EDGE);
                                    for (; exp.More(); exp.Next())
                                    {
                                        TopoDS_Shape aNE = exp.Current();
                                        B.Add(C, aNE);
                                    }
                                }
                                //
                                for (it.Initialize(LInt1); it.More(); it.Next())
                                {
                                    B.Add(C, it.Value());
                                }
                                Build.Bind(aS, C);
                            }
                        }
                        else
                        {
                            Failed.Append(aS);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Computes the connections of the not offset (context) faces that have to be connected by arcs.</summary>
    public void ContextIntByArc(NCollection_IndexedMap<TopoDS_Shape> ContextFaces, bool InSide,
                                BRepOffset_Analyse Analyse, BRepAlgo_Image InitOffsetFace,
                                BRepAlgo_Image InitOffsetEdge, Message_ProgressRange theRange)
    {
        NCollection_List<TopoDS_Shape> LInt1 = new(), LInt2 = new();
        NCollection_Map<TopoDS_Shape> MV = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer exp = new();
        TopoDS_Face OF1 = new(), OF2 = new();
        TopoDS_Edge OE = new();
        BRep_Builder B = new();
        TopoDS_Edge NullEdge = new();
        TopoDS_Face NullFace = new();
        int j;

        for (j = 1; j <= ContextFaces.Extent(); j++)
        {
            TopoDS_Face CF = TopoDS.Face(ContextFaces[j]);
            myTouched.Add(CF);
        }

        using Message_ProgressScope aPS = new(theRange, "Intersecting with deepening faces", ContextFaces.Extent());
        for (j = 1; j <= ContextFaces.Extent(); j++, aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Face CF = TopoDS.Face(ContextFaces[j]);
            for (exp.Init(CF.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(exp.Current());
                if (!Analyse.HasAncestor(E))
                {
                    if (InSide)
                    {
                        myAsDes.Add(CF, E);
                    }
                    else
                    {
                        TopoDS_Edge NE;
                        if (!InitOffsetEdge.HasImage(E))
                        {
                            double f, l, Tol;
                            BRep_Tool.Range(E, out f, out l);
                            Tol = BRep_Tool.Tolerance(E);
                            ExtentEdge(CF, E, out NE);
                            TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                            NE.Orientation(TopAbs_FORWARD);
                            myAsDes.Add(NE, V1.Oriented(TopAbs_REVERSED));
                            myAsDes.Add(NE, V2.Oriented(TopAbs_FORWARD));
                            TopoDS_Shape aLocalShape = V1.Oriented(TopAbs_INTERNAL);
                            B.UpdateVertex(TopoDS.Vertex(aLocalShape), f, NE, Tol);
                            aLocalShape = V2.Oriented(TopAbs_INTERNAL);
                            B.UpdateVertex(TopoDS.Vertex(aLocalShape), l, NE, Tol);
                            //            B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),f,NE,Tol);
                            //            B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),l,NE,Tol);
                            NE.Orientation(E.Orientation());
                            myAsDes.Add(CF, NE);
                            InitOffsetEdge.Bind(E, NE);
                        }
                        else
                        {
                            NE = TopoDS.Edge(InitOffsetEdge.Image(E).First());
                            myAsDes.Add(CF, NE.Oriented(E.Orientation()));
                        }
                    }
                    continue;
                }
                OE.Nullify();
                //---------------------------------------------------
                // OF1 parallel facee generated by the ancestor of E.
                //---------------------------------------------------
                TopoDS_Shape SI = Analyse.Ancestors(E).First();
                OF1 = TopoDS.Face(InitOffsetFace.Image(SI).First());
                OE = TopoDS.Edge(InitOffsetEdge.Image(E).First());

                {
                    // Check if OE has pcurve in CF

                    double f, l;

                    Geom2d_Curve? C1 = BRep_Tool.CurveOnSurface(OE, CF, out f, out l);
                    Geom2d_Curve? C2 = BRep_Tool.CurveOnSurface(OE, OF1, out f, out l);

                    if (C1 is null || C2 is null)
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
                LInt1.Clear();
                LInt1.Append(OE);
                LInt2.Clear();
                TopAbs_Orientation anOri1, anOri2;
                BRepOffset_Tool.OrientSection(OE, CF, OF1, out anOri1, out anOri2);
                //    if (mySide == TopAbs_OUT);
                anOri1 = TopAbs.Reverse(anOri1);
                LInt1.ChangeFirst().Orientation(anOri1);
                Store(CF, OF1, LInt1, LInt2);

                //------------------------------------------------------
                // Processing of offsets on the ancestors of vertices.
                //------------------------------------------------------
                TopoDS_Vertex[] V = [new(), new()];
                TopExp.Vertices(E, out V[0], out V[1]);
                for (int i = 0; i < 2; i++)
                {
                    if (!MV.Add(V[i]))
                    {
                        continue;
                    }
                    OF1.Nullify();
                    NCollection_List<TopoDS_Shape> LE = Analyse.Ancestors(V[i]);
                    NCollection_List<TopoDS_Shape>.Iterator itLE = new(LE);
                    for (; itLE.More(); itLE.Next())
                    {
                        TopoDS_Edge EV = TopoDS.Edge(itLE.Value());
                        if (InitOffsetFace.HasImage(EV))
                        {
                            //-------------------------------------------------
                            // OF1 parallel face generated by an ancestor edge of V[i].
                            //-------------------------------------------------
                            OF1 = TopoDS.Face(InitOffsetFace.Image(EV).First());
                            OE = TopoDS.Edge(InitOffsetEdge.Image(V[i]).First());

                            {
                                // Check if OE has pcurve in CF and OF1

                                double f, l;

                                Geom2d_Curve? C1 = BRep_Tool.CurveOnSurface(OE, CF, out f, out l);
                                Geom2d_Curve? C2 = BRep_Tool.CurveOnSurface(OE, OF1, out f, out l);

                                if (C1 is null || C2 is null)
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
                            LInt1.Clear();
                            LInt1.Append(OE);
                            LInt2.Clear();
                            TopAbs_Orientation O1, O2;
                            BRepOffset_Tool.OrientSection(OE, CF, OF1, out O1, out O2);
                            //            if (mySide == TopAbs_OUT);
                            O1 = TopAbs.Reverse(O1);
                            LInt1.ChangeFirst().Orientation(O1);
                            Store(CF, OF1, LInt1, LInt2);
                        }
                    }
                }
            }

            for (exp.Init(CF.Oriented(TopAbs_FORWARD), TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex V = TopoDS.Vertex(exp.Current());
                if (!Analyse.HasAncestor(V))
                {
                    continue;
                }
                NCollection_List<TopoDS_Shape> LE = Analyse.Ancestors(V);
                NCollection_List<TopoDS_Shape>.Iterator itLE = new(LE);
                for (; itLE.More(); itLE.Next())
                {
                    TopoDS_Edge EV = TopoDS.Edge(itLE.Value());
                    NCollection_List<TopoDS_Shape> LF = Analyse.Ancestors(EV);
                    NCollection_List<TopoDS_Shape>.Iterator itLF = new(LF);
                    for (; itLF.More(); itLF.Next())
                    {
                        TopoDS_Face FEV = TopoDS.Face(itLF.Value());
                        //-------------------------------------------------
                        // OF1 parallel face generated by uneFace ancestor of V[i].
                        //-------------------------------------------------
                        OF1 = TopoDS.Face(InitOffsetFace.Image(FEV).First());
                        if (!IsDone(OF1, CF))
                        {
                            //-------------------------------------------------------
                            // Find if one of edges of OF1 has no trace in CF.
                            //-------------------------------------------------------
                            NCollection_List<TopoDS_Shape> LOE = new();
                            TopExp_Explorer exp2 = new(OF1.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
                            for (; exp2.More(); exp2.Next())
                            {
                                LOE.Append(exp2.Current());
                            }
                            //-------------------------------------------------------
                            // If no trace try intersection.
                            //-------------------------------------------------------
                            if (!BRepOffset_Tool.TryProject(CF, OF1, LOE, LInt1, LInt2, mySide, myTol)
                                || LInt1.IsEmpty())
                            {
                                BRepOffset_Tool.Inter3D(CF, OF1, LInt1, LInt2, mySide, NullEdge, NullFace, NullFace);
                            }
                            Store(CF, OF1, LInt1, LInt2);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Marks the pair of faces as already intersected.</summary>
    public void SetDone(in TopoDS_Face F1, in TopoDS_Face F2)
    {
        if (!myDone.IsBound(F1))
        {
            NCollection_List<TopoDS_Shape> empty = new();
            myDone.Bind(F1, empty);
        }
        myDone[F1].Append(F2);
        if (!myDone.IsBound(F2))
        {
            NCollection_List<TopoDS_Shape> empty = new();
            myDone.Bind(F2, empty);
        }
        myDone[F2].Append(F1);
    }

    //=================================================================================================

    /// <summary>Whether the pair of faces has already been treated.</summary>
    public bool IsDone(in TopoDS_Face F1, in TopoDS_Face F2)
    {
        if (myDone.IsBound(F1))
        {
            NCollection_List<TopoDS_Shape>.Iterator it = new(myDone[F1]);
            for (; it.More(); it.Next())
            {
                if (it.Value().IsSame(F2))
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>The touched faces (the tool's own map).</summary>
    public NCollection_IndexedMap<TopoDS_Shape> TouchedFaces() => myTouched;

    /// <summary>The AsDes tool.</summary>
    public BRepAlgo_AsDes AsDes() => myAsDes;

    /// <summary>The new edges (the tool's own map).</summary>
    public NCollection_IndexedMap<TopoDS_Shape> NewEdges() => myNewEdges;

    //=================================================================================================

    // Stores the intersection results into AsDes
    private void Store(in TopoDS_Face F1, in TopoDS_Face F2, NCollection_List<TopoDS_Shape> LInt1,
                       NCollection_List<TopoDS_Shape> LInt2)
    {
        if (!LInt1.IsEmpty())
        {
            myTouched.Add(F1);
            myTouched.Add(F2);
            myAsDes.Add(F1, LInt1);
            myAsDes.Add(F2, LInt2);
            NCollection_List<TopoDS_Shape>.Iterator it = new(LInt1);
            for (; it.More(); it.Next())
            {
                myNewEdges.Add(it.Value());
            }
        }
        SetDone(F1, F2);
    }
}
