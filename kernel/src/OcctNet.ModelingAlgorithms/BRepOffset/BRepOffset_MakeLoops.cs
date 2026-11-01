// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeLoops.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeLoops.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Rebuilds the offset faces (and the faces of the context) from the edges stored for them in an AsDes, by cutting
/// those edges at their vertices (<see cref="BRepAlgo_Loop"/>), and records the new faces and edges in an image.
/// </summary>
/// <remarks>
/// A working object of the 3D offset. As in OCCT, it writes into the shapes it is given: <see cref="BRepAlgo_Loop"/>
/// writes into the edges; <see cref="BuildFaces"/> adds the reconstructed vertices' parameters on the edges kept
/// (<c>BRep_Builder.UpdateVertex</c>); and where the loops substitute a vertex by a close one, each method replaces it
/// in the edges of the new faces (<c>Free</c>, <c>Remove</c>, <c>Add</c>) and merges its tolerance and point
/// representations into the substitute's TShape. Those are the offset's own working shapes (intersection edges, the
/// images, and the faces of BRepOffset_MakeOffset's own copy of the caller's shape) and must be unfrozen: a frozen one
/// throws <see cref="TopoDS_LockedShape"/> or <see cref="Standard_ImmutableObject"/> at the first write.
/// </remarks>
public sealed class BRepOffset_MakeLoops
{
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myVerVerMap = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>An empty builder.</summary>
    public BRepOffset_MakeLoops()
    {
    }

    //=================================================================================================

    /// <summary>
    /// Rebuilds the faces <paramref name="LF"/> from their edges in <paramref name="AsDes"/>, binding the new faces and
    /// edges in <paramref name="Image"/>.
    /// </summary>
    public void Build(NCollection_List<TopoDS_Shape> LF, BRepAlgo_AsDes AsDes, BRepAlgo_Image Image,
                      BRepAlgo_Image theImageVV, Message_ProgressRange theRange)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new(LF);
        NCollection_List<TopoDS_Shape>.Iterator itl = new(), itLCE = new();
        BRepAlgo_Loop Loops = new();
        Loops.VerticesForSubstitute(myVerVerMap);
        Loops.SetImageVV(theImageVV);
        using Message_ProgressScope aPSOuter = new(theRange, null, 2);
        using Message_ProgressScope aPS1 = new(aPSOuter.Next(), "Init loops", LF.Extent());
        for (; it.More(); it.Next(), aPS1.Next())
        {
            if (!aPS1.More())
            {
                return;
            }
            TopoDS_Face F = TopoDS.Face(it.Value());
            //---------------------------
            // Initialization of Loops.
            //---------------------------
            Loops.Init(F);
            //-----------------------------
            // return edges of F.
            //-----------------------------
            NCollection_List<TopoDS_Shape> LE = AsDes.Descendant(F);
            NCollection_List<TopoDS_Shape> AddedEdges = new();

            for (itl.Initialize(LE); itl.More(); itl.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itl.Value());
                if (Image.HasImage(E))
                {
                    //-------------------------------------------
                    // E was already cut in another face.
                    // Return the cut edges reorientate them as E.
                    // See pb for the edges that have disappeared?
                    //-------------------------------------------
                    NCollection_List<TopoDS_Shape> LCE = Image.Image(E);
                    for (itLCE.Initialize(LCE); itLCE.More(); itLCE.Next())
                    {
                        TopoDS_Shape CE = itLCE.Value().Oriented(E.Orientation());
                        Loops.AddConstEdge(TopoDS.Edge(CE));
                    }
                }
                else
                {
                    Loops.AddEdge(E, AsDes.Descendant(E));
                    AddedEdges.Append(E);
                }
            }
            //------------------------
            // Unwind.
            //------------------------
            Loops.Perform();
            Loops.WiresToFaces();
            //------------------------
            // MAJ SD.
            //------------------------
            NCollection_List<TopoDS_Shape> NF = Loops.NewFaces();
            //-----------------------
            // F => New faces;
            //-----------------------
            Image.Bind(F, NF);

            NCollection_List<TopoDS_Shape>.Iterator itAdded = new();
            for (itAdded.Initialize(AddedEdges); itAdded.More(); itAdded.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itAdded.Value());
                //-----------------------
                //  E => New edges;
                //-----------------------
                NCollection_List<TopoDS_Shape> LoopNE = Loops.NewEdges(E);
                if (Image.HasImage(E))
                {
                    Image.Add(E, LoopNE);
                }
                else
                {
                    Image.Bind(E, LoopNE);
                }
            }
        }
        Loops.GetVerticesForSubstitute(myVerVerMap);
        if (myVerVerMap.IsEmpty())
        {
            return;
        }
        BRep_Builder BB = new();
        using Message_ProgressScope aPS2 = new(aPSOuter.Next(), "Building loops", LF.Extent());
        for (it.Initialize(LF); it.More(); it.Next(), aPS2.Next())
        {
            if (!aPS2.More())
            {
                return;
            }
            TopoDS_Shape F = it.Value();
            NCollection_List<TopoDS_Shape> LIF = new();
            Image.LastImage(F, LIF);
            for (itl.Initialize(LIF); itl.More(); itl.Next())
            {
                TopoDS_Shape IF = itl.Value();
                TopExp_Explorer EdExp = new(IF, TopAbs_EDGE);
                for (; EdExp.More(); EdExp.Next())
                {
                    TopoDS_Shape E = EdExp.Current();
                    NCollection_List<TopoDS_Shape> VList = new();
                    TopoDS_Iterator VerExp = new(E);
                    for (; VerExp.More(); VerExp.Next())
                    {
                        VList.Append(VerExp.Value());
                    }
                    NCollection_List<TopoDS_Shape>.Iterator itlv = new(VList);
                    for (; itlv.More(); itlv.Next())
                    {
                        TopoDS_Shape V = itlv.Value();
                        if (myVerVerMap.IsBound(V))
                        {
                            TopoDS_Shape NewV = myVerVerMap[V];
                            E.Free(true);
                            NewV.Orientation(V.Orientation());
                            BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
                            BRep_TVertex NewTV = (BRep_TVertex)NewV.TShape()!;
                            if (TV.Tolerance() > NewTV.Tolerance())
                            {
                                NewTV.Tolerance(TV.Tolerance());
                            }
                            NewTV.ChangePoints().Append(TV.ChangePoints()); // moves V's points, as OCCT's list Append does
                            AsDes.Replace(V, NewV);
                            BB.Remove(E, V);
                            BB.Add(E, NewV);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static bool IsBetweenCorks(in TopoDS_Shape E, BRepAlgo_AsDes AsDes, NCollection_List<TopoDS_Shape> LContext)
    {
        if (!AsDes.HasAscendant(E))
        {
            return true;
        }
        NCollection_List<TopoDS_Shape> LF = AsDes.Ascendant(E);
        NCollection_List<TopoDS_Shape>.Iterator it = new();
        for (it.Initialize(LF); it.More(); it.Next())
        {
            TopoDS_Shape S = it.Value();
            bool found = false;
            NCollection_List<TopoDS_Shape>.Iterator it2 = new();
            for (it2.Initialize(LContext); it2.More(); it2.Next())
            {
                if (S.IsSame(it2.Value()))
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    /// <summary>
    /// Rebuilds the context faces (caps) <paramref name="LContext"/> from their edges and the traces of the offsets in
    /// <paramref name="AsDes"/>, binding the new faces and edges in <paramref name="Image"/>.
    /// </summary>
    public void BuildOnContext(NCollection_List<TopoDS_Shape> LContext, BRepOffset_Analyse Analyse, BRepAlgo_AsDes AsDes,
                               BRepAlgo_Image Image, bool InSide, Message_ProgressRange theRange)
    {
        //-----------------------------------------
        // unwinding of caps.
        //-----------------------------------------
        NCollection_List<TopoDS_Shape>.Iterator it = new(LContext);
        NCollection_List<TopoDS_Shape>.Iterator itl = new(), itLCE = new();
        BRepAlgo_Loop Loops = new();
        Loops.VerticesForSubstitute(myVerVerMap);
        TopExp_Explorer exp = new();
        NCollection_Map<TopoDS_Shape> MapExtent = new(TopTools_ShapeMapHasher.Instance);

        using Message_ProgressScope aPS = new(theRange, "Building deepening faces", LContext.Extent());
        for (; it.More(); it.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Face F = TopoDS.Face(it.Value());
            NCollection_Map<TopoDS_Shape> MBound = new(TopTools_ShapeMapHasher.Instance);
            //-----------------------------------------------
            // Initialisation of Loops.
            // F is reversed it will be added in myOffC.
            // and myOffC will be reversed in the final result.
            //-----------------------------------------------
            TopoDS_Shape aReversedF = F.Reversed();
            if (InSide)
            {
                Loops.Init(TopoDS.Face(aReversedF));
                //    if (InSide) Loops.Init(TopoDS::Face(F.Reversed()));
            }
            else
            {
                Loops.Init(F);
            }
            //--------------------------------------------------------
            // return edges of F not modified by definition.
            //--------------------------------------------------------
            for (exp.Init(F.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge CE = TopoDS.Edge(exp.Current());
                MBound.Add(CE);
                if (Analyse.HasAncestor(CE))
                {
                    // the stop of cups except for the connectivity stops between caps.
                    //      if (!AsDes->HasAscendant(CE)) {
                    TopoDS_Shape aReversedE = CE.Reversed();
                    if (InSide)
                    {
                        Loops.AddConstEdge(CE);
                    }
                    else
                    {
                        Loops.AddConstEdge(TopoDS.Edge(aReversedE));
                    }
                    //	else        Loops.AddConstEdge(TopoDS::Edge(CE.Reversed()));
                }
            }
            //------------------------------------------------------
            // Trace of offsets + connectivity edge between caps.
            //------------------------------------------------------
            NCollection_List<TopoDS_Shape> LE = AsDes.Descendant(F);
            NCollection_List<TopoDS_Shape> AddedEdges = new();

            for (itl.Initialize(LE); itl.More(); itl.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itl.Value());
                if (Image.HasImage(E))
                {
                    //-------------------------------------------
                    // E was already cut in another face.
                    // Return cut edges and orientate them as E.
                    // See pb for the edges that have disappeared?
                    //-------------------------------------------
                    NCollection_List<TopoDS_Shape> LCE = Image.Image(E);
                    for (itLCE.Initialize(LCE); itLCE.More(); itLCE.Next())
                    {
                        TopoDS_Shape CE = itLCE.Value().Oriented(E.Orientation());
                        if (MapExtent.Contains(E))
                        {
                            Loops.AddConstEdge(TopoDS.Edge(CE));
                            continue;
                        }
                        if (!MBound.Contains(E))
                        {
                            CE.Reverse();
                        }
                        if (InSide)
                        {
                            Loops.AddConstEdge(TopoDS.Edge(CE));
                        }
                        else
                        {
                            TopoDS_Shape aReversedE = CE.Reversed();
                            Loops.AddConstEdge(TopoDS.Edge(aReversedE));
                        }
                        //	  else        Loops.AddConstEdge(TopoDS::Edge(CE.Reversed()));
                    }
                }
                else
                {
                    if (IsBetweenCorks(E, AsDes, LContext) && AsDes.HasDescendant(E))
                    {
                        // connection between 2 caps
                        MapExtent.Add(E);
                        NCollection_List<TopoDS_Shape> LV = new();
                        if (InSide)
                        {
                            for (itLCE.Initialize(AsDes.Descendant(E)); itLCE.More(); itLCE.Next())
                            {
                                LV.Append(itLCE.Value().Reversed());
                            }
                            Loops.AddEdge(E, LV);
                        }
                        else
                        {
                            Loops.AddEdge(E, AsDes.Descendant(E));
                        }
                        AddedEdges.Append(E);
                    }
                    else if (IsBetweenCorks(E, AsDes, LContext))
                    {
                        TopoDS_Shape aLocalShape = E.Reversed();
                        if (InSide)
                        {
                            Loops.AddConstEdge(E);
                        }
                        else
                        {
                            Loops.AddConstEdge(TopoDS.Edge(aLocalShape));
                        }
                        //	  if (InSide) Loops.AddConstEdge(TopoDS::Edge(E));
                        //	  else        Loops.AddConstEdge(TopoDS::Edge(E.Reversed()));
                    }
                    else
                    {
                        TopoDS_Shape aLocalShape = E.Reversed();
                        if (InSide)
                        {
                            Loops.AddConstEdge(TopoDS.Edge(aLocalShape));
                        }
                        else
                        {
                            Loops.AddConstEdge(E);
                        }
                        //	  if (InSide) Loops.AddConstEdge(TopoDS::Edge(E.Reversed()));
                        //	  else        Loops.AddConstEdge(TopoDS::Edge(E));
                    }
                }
            }
            //------------------------
            // Unwind.
            //------------------------
            Loops.Perform();
            Loops.WiresToFaces();
            //------------------------
            // MAJ SD.
            //------------------------
            NCollection_List<TopoDS_Shape> NF = Loops.NewFaces();
            //-----------------------
            // F => New faces;
            //-----------------------
            Image.Bind(F, NF);

            NCollection_List<TopoDS_Shape>.Iterator itAdded = new();
            for (itAdded.Initialize(AddedEdges); itAdded.More(); itAdded.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itAdded.Value());
                //-----------------------
                //  E => New edges;
                //-----------------------
                if (Image.HasImage(E))
                {
                    Image.Add(E, Loops.NewEdges(E));
                }
                else
                {
                    Image.Bind(E, Loops.NewEdges(E));
                }
            }
        }
        Loops.GetVerticesForSubstitute(myVerVerMap);
        if (myVerVerMap.IsEmpty())
        {
            return;
        }
        BRep_Builder BB = new();
        for (it.Initialize(LContext); it.More(); it.Next())
        {
            TopoDS_Shape F = it.Value();
            NCollection_List<TopoDS_Shape> LIF = new();
            Image.LastImage(F, LIF);
            for (itl.Initialize(LIF); itl.More(); itl.Next())
            {
                TopoDS_Shape IF = itl.Value();
                TopExp_Explorer EdExp = new(IF, TopAbs_EDGE);
                for (; EdExp.More(); EdExp.Next())
                {
                    TopoDS_Shape E = EdExp.Current();
                    NCollection_List<TopoDS_Shape> VList = new();
                    TopoDS_Iterator VerExp = new(E);
                    for (; VerExp.More(); VerExp.Next())
                    {
                        VList.Append(VerExp.Value());
                    }
                    NCollection_List<TopoDS_Shape>.Iterator itlv = new(VList);
                    for (; itlv.More(); itlv.Next())
                    {
                        TopoDS_Shape V = itlv.Value();
                        if (myVerVerMap.IsBound(V))
                        {
                            TopoDS_Shape NewV = myVerVerMap[V];
                            E.Free(true);
                            NewV.Orientation(V.Orientation());
                            BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
                            BRep_TVertex NewTV = (BRep_TVertex)NewV.TShape()!;
                            if (TV.Tolerance() > NewTV.Tolerance())
                            {
                                NewTV.Tolerance(TV.Tolerance());
                            }
                            NewTV.ChangePoints().Append(TV.ChangePoints()); // moves V's points, as OCCT's list Append does
                            AsDes.Replace(V, NewV);
                            BB.Remove(E, V);
                            BB.Add(E, NewV);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Rebuilds the faces <paramref name="LF"/> not yet rebuilt whose edges were cut, binding the new faces and edges
    /// in <paramref name="Image"/>.
    /// </summary>
    public void BuildFaces(NCollection_List<TopoDS_Shape> LF, BRepAlgo_AsDes AsDes, BRepAlgo_Image Image,
                           Message_ProgressRange theRange)
    {
        NCollection_List<TopoDS_Shape>.Iterator itr = new(), itl = new(), itLCE = new();
        bool ToRebuild;
        BRepAlgo_Loop Loops = new();
        Loops.VerticesForSubstitute(myVerVerMap);
        BRep_Builder B = new();

        //----------------------------------
        // Loop on all faces //.
        //----------------------------------
        using Message_ProgressScope aPS = new(theRange, "Building faces", LF.Extent());
        for (itr.Initialize(LF); itr.More(); itr.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            TopoDS_Face F = TopoDS.Face(itr.Value());
            Loops.Init(F);
            ToRebuild = false;
            NCollection_List<TopoDS_Shape> AddedEdges = new();

            if (!Image.HasImage(F))
            {
                //----------------------------------
                // Face F not yet reconstructed.
                //----------------------------------
                NCollection_List<TopoDS_Shape> LE = AsDes.Descendant(F);
                //----------------------------------------------------------------
                // first loop to find if the edges of the face were reconstructed.
                // - maj on map MONV. Some vertices on reconstructed edges
                // coincide geometrically with old but are not IsSame.
                //----------------------------------------------------------------
                NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MONV = new(TopTools_ShapeMapHasher.Instance);
                TopoDS_Vertex OV1, OV2, NV1, NV2;

                for (itl.Initialize(LE); itl.More(); itl.Next())
                {
                    TopoDS_Edge E = TopoDS.Edge(itl.Value());
                    if (Image.HasImage(E))
                    {
                        NCollection_List<TopoDS_Shape> LCE = Image.Image(E);
                        if (LCE.Extent() == 1 && LCE.First().IsSame(E))
                        {
                            TopoDS_Shape aLocalShape = LCE.First().Oriented(E.Orientation());
                            TopoDS_Edge CE = TopoDS.Edge(aLocalShape);
                            //	    TopoDS_Edge CE = TopoDS::Edge(LCE.First().Oriented(E.Orientation()));
                            Loops.AddConstEdge(CE);
                            continue;
                        }
                        //----------------------------------
                        // F should be reconstructed.
                        //----------------------------------
                        ToRebuild = true;
                        for (itLCE.Initialize(LCE); itLCE.More(); itLCE.Next())
                        {
                            TopoDS_Shape aLocalShape = itLCE.Value().Oriented(E.Orientation());
                            TopoDS_Edge CE = TopoDS.Edge(aLocalShape);
                            //	    TopoDS_Edge CE = TopoDS::Edge(itLCE.Value().Oriented(E.Orientation()));
                            TopExp.Vertices(E, out OV1, out OV2);
                            TopExp.Vertices(CE, out NV1, out NV2);
                            if (!OV1.IsSame(NV1))
                            {
                                MONV.Bind(OV1, NV1);
                            }
                            if (!OV2.IsSame(NV2))
                            {
                                MONV.Bind(OV2, NV2);
                            }
                            Loops.AddConstEdge(CE);
                        }
                    }
                }
                if (ToRebuild)
                {

                    //-----------------------------------------------------------
                    // Non-reconstructed edges on other faces are added.
                    // If their vertices were reconstructed they are reconstructed.
                    //-----------------------------------------------------------
                    for (itl.Initialize(LE); itl.More(); itl.Next())
                    {
                        double f, l;
                        TopoDS_Edge E = TopoDS.Edge(itl.Value());
                        BRep_Tool.Range(E, out f, out l);
                        if (!Image.HasImage(E))
                        {
                            TopExp.Vertices(E, out OV1, out OV2);
                            NCollection_List<TopoDS_Shape> LV = new();
                            if (MONV.IsBound(OV1))
                            {
                                TopoDS_Vertex VV = TopoDS.Vertex(MONV[OV1]);
                                VV.Orientation(TopAbs_FORWARD);
                                LV.Append(VV);
                                TopoDS_Shape aLocalShape = VV.Oriented(TopAbs_INTERNAL);
                                B.UpdateVertex(TopoDS.Vertex(aLocalShape), f, E, BRep_Tool.Tolerance(VV));
                            }
                            if (MONV.IsBound(OV2))
                            {
                                TopoDS_Vertex VV = TopoDS.Vertex(MONV[OV2]);
                                VV.Orientation(TopAbs_REVERSED);
                                LV.Append(VV);
                                TopoDS_Shape aLocalShape = VV.Oriented(TopAbs_INTERNAL);
                                B.UpdateVertex(TopoDS.Vertex(aLocalShape), l, E, BRep_Tool.Tolerance(VV));
                                //	      B.UpdateVertex(TopoDS::Vertex(VV.Oriented(TopAbs_INTERNAL)),
                                //			     l,E,BRep_Tool::Tolerance(VV));
                            }
                            if (LV.IsEmpty())
                            {
                                Loops.AddConstEdge(E);
                            }
                            else
                            {
                                Loops.AddEdge(E, LV);
                                AddedEdges.Append(E);
                            }
                        }
                    }
                }
            }
            if (ToRebuild)
            {
                //------------------------
                // Reconstruction.
                //------------------------
                Loops.Perform();
                Loops.WiresToFaces();
                //------------------------
                // MAJ SD.
                //------------------------
                NCollection_List<TopoDS_Shape> NF = Loops.NewFaces();
                //-----------------------
                // F => New faces;
                //-----------------------
                Image.Bind(F, NF);

                NCollection_List<TopoDS_Shape>.Iterator itAdded = new();
                for (itAdded.Initialize(AddedEdges); itAdded.More(); itAdded.Next())
                {
                    TopoDS_Edge E = TopoDS.Edge(itAdded.Value());
                    //-----------------------
                    //  E => New edges;
                    //-----------------------
                    if (Image.HasImage(E))
                    {
                        Image.Add(E, Loops.NewEdges(E));
                    }
                    else
                    {
                        Image.Bind(E, Loops.NewEdges(E));
                    }
                }
            }
        }
        Loops.GetVerticesForSubstitute(myVerVerMap);
        if (myVerVerMap.IsEmpty())
        {
            return;
        }
        BRep_Builder BB = new();
        for (itr.Initialize(LF); itr.More(); itr.Next())
        {
            TopoDS_Shape F = itr.Value();
            NCollection_List<TopoDS_Shape> LIF = new();
            Image.LastImage(F, LIF);
            for (itl.Initialize(LIF); itl.More(); itl.Next())
            {
                TopoDS_Shape IF = itl.Value();
                TopExp_Explorer EdExp = new(IF, TopAbs_EDGE);
                for (; EdExp.More(); EdExp.Next())
                {
                    TopoDS_Shape E = EdExp.Current();
                    NCollection_List<TopoDS_Shape> VList = new();
                    TopoDS_Iterator VerExp = new(E);
                    for (; VerExp.More(); VerExp.Next())
                    {
                        VList.Append(VerExp.Value());
                    }
                    NCollection_List<TopoDS_Shape>.Iterator itlv = new(VList);
                    for (; itlv.More(); itlv.Next())
                    {
                        TopoDS_Shape V = itlv.Value();
                        if (myVerVerMap.IsBound(V))
                        {
                            TopoDS_Shape NewV = myVerVerMap[V];
                            E.Free(true);
                            NewV.Orientation(V.Orientation());
                            BRep_TVertex TV = (BRep_TVertex)V.TShape()!;
                            BRep_TVertex NewTV = (BRep_TVertex)NewV.TShape()!;
                            if (TV.Tolerance() > NewTV.Tolerance())
                            {
                                NewTV.Tolerance(TV.Tolerance());
                            }
                            NewTV.ChangePoints().Append(TV.ChangePoints()); // moves V's points, as OCCT's list Append does
                            AsDes.Replace(V, NewV);
                            BB.Remove(E, V);
                            BB.Add(E, NewV);
                        }
                    }
                }
            }
        }
    }
}
