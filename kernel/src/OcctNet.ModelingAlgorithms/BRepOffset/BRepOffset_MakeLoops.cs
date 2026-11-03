// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeLoops.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeLoops.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
/// throws <see cref="LockedShapeException"/> or <see cref="ImmutableObjectException"/> at the first write.
/// </remarks>
public sealed class BRepOffset_MakeLoops
{
    private readonly NCollection_DataMap<Shape, Shape> myVerVerMap = new(ShapeHasher.Instance);

    /// <summary>An empty builder.</summary>
    public BRepOffset_MakeLoops()
    {
    }

    //=================================================================================================

    /// <summary>
    /// Rebuilds the faces <paramref name="LF"/> from their edges in <paramref name="AsDes"/>, binding the new faces and
    /// edges in <paramref name="Image"/>.
    /// </summary>
    public void Build(NCollection_List<Shape> LF, BRepAlgo_AsDes AsDes, BRepAlgo_Image Image,
                      BRepAlgo_Image theImageVV, ProgressRange theRange)
    {
        NCollection_List<Shape>.Iterator it = new(LF);
        NCollection_List<Shape>.Iterator itl = new(), itLCE = new();
        BRepAlgo_Loop Loops = new();
        Loops.VerticesForSubstitute(myVerVerMap);
        Loops.SetImageVV(theImageVV);
        using ProgressScope aPSOuter = new(theRange, null, 2);
        using ProgressScope aPS1 = new(aPSOuter.Next(), "Init loops", LF.Extent());
        for (; it.More(); it.Next(), aPS1.Next())
        {
            if (!aPS1.More())
            {
                return;
            }
            Face F = ShapeCasts.Face(it.Value());
            //---------------------------
            // Initialization of Loops.
            //---------------------------
            Loops.Init(F);
            //-----------------------------
            // return edges of F.
            //-----------------------------
            NCollection_List<Shape> LE = AsDes.Descendant(F);
            NCollection_List<Shape> AddedEdges = new();

            for (itl.Initialize(LE); itl.More(); itl.Next())
            {
                Edge E = ShapeCasts.Edge(itl.Value());
                if (Image.HasImage(E))
                {
                    //-------------------------------------------
                    // E was already cut in another face.
                    // Return the cut edges reorientate them as E.
                    // See pb for the edges that have disappeared?
                    //-------------------------------------------
                    NCollection_List<Shape> LCE = Image.Image(E);
                    for (itLCE.Initialize(LCE); itLCE.More(); itLCE.Next())
                    {
                        Shape CE = itLCE.Value().Oriented(E.Orientation());
                        Loops.AddConstEdge(ShapeCasts.Edge(CE));
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
            NCollection_List<Shape> NF = Loops.NewFaces();
            //-----------------------
            // F => New faces;
            //-----------------------
            Image.Bind(F, NF);

            NCollection_List<Shape>.Iterator itAdded = new();
            for (itAdded.Initialize(AddedEdges); itAdded.More(); itAdded.Next())
            {
                Edge E = ShapeCasts.Edge(itAdded.Value());
                //-----------------------
                //  E => New edges;
                //-----------------------
                NCollection_List<Shape> LoopNE = Loops.NewEdges(E);
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
        BrepBuilder BB = new();
        using ProgressScope aPS2 = new(aPSOuter.Next(), "Building loops", LF.Extent());
        for (it.Initialize(LF); it.More(); it.Next(), aPS2.Next())
        {
            if (!aPS2.More())
            {
                return;
            }
            Shape F = it.Value();
            NCollection_List<Shape> LIF = new();
            Image.LastImage(F, LIF);
            for (itl.Initialize(LIF); itl.More(); itl.Next())
            {
                Shape IF = itl.Value();
                Explorer EdExp = new(IF, ShapeType.Edge);
                for (; EdExp.More(); EdExp.Next())
                {
                    Shape E = EdExp.Current();
                    NCollection_List<Shape> VList = new();
                    Iterator VerExp = new(E);
                    for (; VerExp.More(); VerExp.Next())
                    {
                        VList.Append(VerExp.Value());
                    }
                    NCollection_List<Shape>.Iterator itlv = new(VList);
                    for (; itlv.More(); itlv.Next())
                    {
                        Shape V = itlv.Value();
                        if (myVerVerMap.IsBound(V))
                        {
                            Shape NewV = myVerVerMap[V];
                            E.Free(true);
                            NewV.Orientation(V.Orientation());
                            BrepTVertex TV = (BrepTVertex)V.TShape()!;
                            BrepTVertex NewTV = (BrepTVertex)NewV.TShape()!;
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

    private static bool IsBetweenCorks(in Shape E, BRepAlgo_AsDes AsDes, NCollection_List<Shape> LContext)
    {
        if (!AsDes.HasAscendant(E))
        {
            return true;
        }
        NCollection_List<Shape> LF = AsDes.Ascendant(E);
        NCollection_List<Shape>.Iterator it = new();
        for (it.Initialize(LF); it.More(); it.Next())
        {
            Shape S = it.Value();
            bool found = false;
            NCollection_List<Shape>.Iterator it2 = new();
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
    public void BuildOnContext(NCollection_List<Shape> LContext, BRepOffset_Analyse Analyse, BRepAlgo_AsDes AsDes,
                               BRepAlgo_Image Image, bool InSide, ProgressRange theRange)
    {
        //-----------------------------------------
        // unwinding of caps.
        //-----------------------------------------
        NCollection_List<Shape>.Iterator it = new(LContext);
        NCollection_List<Shape>.Iterator itl = new(), itLCE = new();
        BRepAlgo_Loop Loops = new();
        Loops.VerticesForSubstitute(myVerVerMap);
        Explorer exp = new();
        NCollection_Map<Shape> MapExtent = new(ShapeHasher.Instance);

        using ProgressScope aPS = new(theRange, "Building deepening faces", LContext.Extent());
        for (; it.More(); it.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            Face F = ShapeCasts.Face(it.Value());
            NCollection_Map<Shape> MBound = new(ShapeHasher.Instance);
            //-----------------------------------------------
            // Initialisation of Loops.
            // F is reversed it will be added in myOffC.
            // and myOffC will be reversed in the final result.
            //-----------------------------------------------
            Shape aReversedF = F.Reversed();
            if (InSide)
            {
                Loops.Init(ShapeCasts.Face(aReversedF));
                //    if (InSide) Loops.Init(TopoDS::Face(F.Reversed()));
            }
            else
            {
                Loops.Init(F);
            }
            //--------------------------------------------------------
            // return edges of F not modified by definition.
            //--------------------------------------------------------
            for (exp.Init(F.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge CE = ShapeCasts.Edge(exp.Current());
                MBound.Add(CE);
                if (Analyse.HasAncestor(CE))
                {
                    // the stop of cups except for the connectivity stops between caps.
                    //      if (!AsDes->HasAscendant(CE)) {
                    Shape aReversedE = CE.Reversed();
                    if (InSide)
                    {
                        Loops.AddConstEdge(CE);
                    }
                    else
                    {
                        Loops.AddConstEdge(ShapeCasts.Edge(aReversedE));
                    }
                    //	else        Loops.AddConstEdge(TopoDS::Edge(CE.Reversed()));
                }
            }
            //------------------------------------------------------
            // Trace of offsets + connectivity edge between caps.
            //------------------------------------------------------
            NCollection_List<Shape> LE = AsDes.Descendant(F);
            NCollection_List<Shape> AddedEdges = new();

            for (itl.Initialize(LE); itl.More(); itl.Next())
            {
                Edge E = ShapeCasts.Edge(itl.Value());
                if (Image.HasImage(E))
                {
                    //-------------------------------------------
                    // E was already cut in another face.
                    // Return cut edges and orientate them as E.
                    // See pb for the edges that have disappeared?
                    //-------------------------------------------
                    NCollection_List<Shape> LCE = Image.Image(E);
                    for (itLCE.Initialize(LCE); itLCE.More(); itLCE.Next())
                    {
                        Shape CE = itLCE.Value().Oriented(E.Orientation());
                        if (MapExtent.Contains(E))
                        {
                            Loops.AddConstEdge(ShapeCasts.Edge(CE));
                            continue;
                        }
                        if (!MBound.Contains(E))
                        {
                            CE.Reverse();
                        }
                        if (InSide)
                        {
                            Loops.AddConstEdge(ShapeCasts.Edge(CE));
                        }
                        else
                        {
                            Shape aReversedE = CE.Reversed();
                            Loops.AddConstEdge(ShapeCasts.Edge(aReversedE));
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
                        NCollection_List<Shape> LV = new();
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
                        Shape aLocalShape = E.Reversed();
                        if (InSide)
                        {
                            Loops.AddConstEdge(E);
                        }
                        else
                        {
                            Loops.AddConstEdge(ShapeCasts.Edge(aLocalShape));
                        }
                        //	  if (InSide) Loops.AddConstEdge(TopoDS::Edge(E));
                        //	  else        Loops.AddConstEdge(TopoDS::Edge(E.Reversed()));
                    }
                    else
                    {
                        Shape aLocalShape = E.Reversed();
                        if (InSide)
                        {
                            Loops.AddConstEdge(ShapeCasts.Edge(aLocalShape));
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
            NCollection_List<Shape> NF = Loops.NewFaces();
            //-----------------------
            // F => New faces;
            //-----------------------
            Image.Bind(F, NF);

            NCollection_List<Shape>.Iterator itAdded = new();
            for (itAdded.Initialize(AddedEdges); itAdded.More(); itAdded.Next())
            {
                Edge E = ShapeCasts.Edge(itAdded.Value());
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
        BrepBuilder BB = new();
        for (it.Initialize(LContext); it.More(); it.Next())
        {
            Shape F = it.Value();
            NCollection_List<Shape> LIF = new();
            Image.LastImage(F, LIF);
            for (itl.Initialize(LIF); itl.More(); itl.Next())
            {
                Shape IF = itl.Value();
                Explorer EdExp = new(IF, ShapeType.Edge);
                for (; EdExp.More(); EdExp.Next())
                {
                    Shape E = EdExp.Current();
                    NCollection_List<Shape> VList = new();
                    Iterator VerExp = new(E);
                    for (; VerExp.More(); VerExp.Next())
                    {
                        VList.Append(VerExp.Value());
                    }
                    NCollection_List<Shape>.Iterator itlv = new(VList);
                    for (; itlv.More(); itlv.Next())
                    {
                        Shape V = itlv.Value();
                        if (myVerVerMap.IsBound(V))
                        {
                            Shape NewV = myVerVerMap[V];
                            E.Free(true);
                            NewV.Orientation(V.Orientation());
                            BrepTVertex TV = (BrepTVertex)V.TShape()!;
                            BrepTVertex NewTV = (BrepTVertex)NewV.TShape()!;
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
    public void BuildFaces(NCollection_List<Shape> LF, BRepAlgo_AsDes AsDes, BRepAlgo_Image Image,
                           ProgressRange theRange)
    {
        NCollection_List<Shape>.Iterator itr = new(), itl = new(), itLCE = new();
        bool ToRebuild;
        BRepAlgo_Loop Loops = new();
        Loops.VerticesForSubstitute(myVerVerMap);
        BrepBuilder B = new();

        //----------------------------------
        // Loop on all faces //.
        //----------------------------------
        using ProgressScope aPS = new(theRange, "Building faces", LF.Extent());
        for (itr.Initialize(LF); itr.More(); itr.Next(), aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }
            Face F = ShapeCasts.Face(itr.Value());
            Loops.Init(F);
            ToRebuild = false;
            NCollection_List<Shape> AddedEdges = new();

            if (!Image.HasImage(F))
            {
                //----------------------------------
                // Face F not yet reconstructed.
                //----------------------------------
                NCollection_List<Shape> LE = AsDes.Descendant(F);
                //----------------------------------------------------------------
                // first loop to find if the edges of the face were reconstructed.
                // - maj on map MONV. Some vertices on reconstructed edges
                // coincide geometrically with old but are not IsSame.
                //----------------------------------------------------------------
                NCollection_DataMap<Shape, Shape> MONV = new(ShapeHasher.Instance);
                Vertex OV1, OV2, NV1, NV2;

                for (itl.Initialize(LE); itl.More(); itl.Next())
                {
                    Edge E = ShapeCasts.Edge(itl.Value());
                    if (Image.HasImage(E))
                    {
                        NCollection_List<Shape> LCE = Image.Image(E);
                        if (LCE.Extent() == 1 && LCE.First().IsSame(E))
                        {
                            Shape aLocalShape = LCE.First().Oriented(E.Orientation());
                            Edge CE = ShapeCasts.Edge(aLocalShape);
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
                            Shape aLocalShape = itLCE.Value().Oriented(E.Orientation());
                            Edge CE = ShapeCasts.Edge(aLocalShape);
                            //	    TopoDS_Edge CE = TopoDS::Edge(itLCE.Value().Oriented(E.Orientation()));
                            Exploration.Vertices(E, out OV1, out OV2);
                            Exploration.Vertices(CE, out NV1, out NV2);
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
                        Edge E = ShapeCasts.Edge(itl.Value());
                        Tool.Range(E, out f, out l);
                        if (!Image.HasImage(E))
                        {
                            Exploration.Vertices(E, out OV1, out OV2);
                            NCollection_List<Shape> LV = new();
                            if (MONV.IsBound(OV1))
                            {
                                Vertex VV = ShapeCasts.Vertex(MONV[OV1]);
                                VV.Orientation(Forward);
                                LV.Append(VV);
                                Shape aLocalShape = VV.Oriented(Internal);
                                B.UpdateVertex(ShapeCasts.Vertex(aLocalShape), f, E, Tool.Tolerance(VV));
                            }
                            if (MONV.IsBound(OV2))
                            {
                                Vertex VV = ShapeCasts.Vertex(MONV[OV2]);
                                VV.Orientation(Reversed);
                                LV.Append(VV);
                                Shape aLocalShape = VV.Oriented(Internal);
                                B.UpdateVertex(ShapeCasts.Vertex(aLocalShape), l, E, Tool.Tolerance(VV));
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
                NCollection_List<Shape> NF = Loops.NewFaces();
                //-----------------------
                // F => New faces;
                //-----------------------
                Image.Bind(F, NF);

                NCollection_List<Shape>.Iterator itAdded = new();
                for (itAdded.Initialize(AddedEdges); itAdded.More(); itAdded.Next())
                {
                    Edge E = ShapeCasts.Edge(itAdded.Value());
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
        BrepBuilder BB = new();
        for (itr.Initialize(LF); itr.More(); itr.Next())
        {
            Shape F = itr.Value();
            NCollection_List<Shape> LIF = new();
            Image.LastImage(F, LIF);
            for (itl.Initialize(LIF); itl.More(); itl.Next())
            {
                Shape IF = itl.Value();
                Explorer EdExp = new(IF, ShapeType.Edge);
                for (; EdExp.More(); EdExp.Next())
                {
                    Shape E = EdExp.Current();
                    NCollection_List<Shape> VList = new();
                    Iterator VerExp = new(E);
                    for (; VerExp.More(); VerExp.Next())
                    {
                        VList.Append(VerExp.Value());
                    }
                    NCollection_List<Shape>.Iterator itlv = new(VList);
                    for (; itlv.More(); itlv.Next())
                    {
                        Shape V = itlv.Value();
                        if (myVerVerMap.IsBound(V))
                        {
                            Shape NewV = myVerVerMap[V];
                            E.Free(true);
                            NewV.Orientation(V.Orientation());
                            BrepTVertex TV = (BrepTVertex)V.TShape()!;
                            BrepTVertex NewTV = (BrepTVertex)NewV.TShape()!;
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
