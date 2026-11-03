// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeLoops.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeLoops.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Offsets;

/// <summary>
/// Rebuilds the offset faces (and the faces of the context) from the edges stored for them in an AsDes, by cutting
/// those edges at their vertices (<see cref="Loop"/>), and records the new faces and edges in an image.
/// </summary>
/// <remarks>
/// A working object of the 3D offset. As in OCCT, it writes into the shapes it is given: <see cref="Loop"/>
/// writes into the edges; <see cref="BuildFaces"/> adds the reconstructed vertices' parameters on the edges kept
/// (<c>BRep_Builder.UpdateVertex</c>); and where the loops substitute a vertex by a close one, each method replaces it
/// in the edges of the new faces (<c>Free</c>, <c>Remove</c>, <c>Add</c>) and merges its tolerance and point
/// representations into the substitute's TShape. Those are the offset's own working shapes (intersection edges, the
/// images, and the faces of BRepOffset_MakeOffset's own copy of the caller's shape) and must be unfrozen: a frozen one
/// throws <see cref="LockedShapeException"/> or <see cref="ImmutableObjectException"/> at the first write.
/// </remarks>
public sealed class OffsetMakeLoops
{
    private readonly Dictionary<Shape, Shape> _verVerMap = new(ShapeHasher.Instance);

    /// <summary>An empty builder.</summary>
    public OffsetMakeLoops()
    {
    }

    //=================================================================================================

    /// <summary>
    /// Rebuilds the faces <paramref name="lf"/> from their edges in <paramref name="asDes"/>, binding the new faces and
    /// edges in <paramref name="image"/>.
    /// </summary>
    public void Build(NCollection_List<Shape> lf, AsDes asDes, ShapeImage image,
                      ShapeImage imageVV, ProgressRange range)
    {
        NCollection_List<Shape>.Iterator it = new(lf);
        NCollection_List<Shape>.Iterator itl = new(), itLCE = new();
        Loop loops = new();
        loops.VerticesForSubstitute(_verVerMap);
        loops.SetImageVV(imageVV);
        using ProgressScope psOuter = new(range, null, 2);
        using ProgressScope ps1 = new(psOuter.Next(), "Init loops", lf.Extent());
        for (; it.More(); it.Next(), ps1.Next())
        {
            if (!ps1.More())
            {
                return;
            }
            Face f = ShapeCasts.Face(it.Value());
            //---------------------------
            // Initialization of Loops.
            //---------------------------
            loops.Init(f);
            //-----------------------------
            // return edges of F.
            //-----------------------------
            NCollection_List<Shape> le = asDes.Descendant(f);
            NCollection_List<Shape> addedEdges = new();

            for (itl.Initialize(le); itl.More(); itl.Next())
            {
                Edge e = ShapeCasts.Edge(itl.Value());
                if (image.HasImage(e))
                {
                    //-------------------------------------------
                    // E was already cut in another face.
                    // Return the cut edges reorientate them as E.
                    // See pb for the edges that have disappeared?
                    //-------------------------------------------
                    NCollection_List<Shape> lce = image.Image(e);
                    for (itLCE.Initialize(lce); itLCE.More(); itLCE.Next())
                    {
                        Shape ce = itLCE.Value().Oriented(e.Orientation());
                        loops.AddConstEdge(ShapeCasts.Edge(ce));
                    }
                }
                else
                {
                    loops.AddEdge(e, asDes.Descendant(e));
                    addedEdges.Append(e);
                }
            }
            //------------------------
            // Unwind.
            //------------------------
            loops.Perform();
            loops.WiresToFaces();
            //------------------------
            // MAJ SD.
            //------------------------
            NCollection_List<Shape> nf = loops.NewFaces();
            //-----------------------
            // F => New faces;
            //-----------------------
            image.Bind(f, nf);

            NCollection_List<Shape>.Iterator itAdded = new();
            for (itAdded.Initialize(addedEdges); itAdded.More(); itAdded.Next())
            {
                Edge e = ShapeCasts.Edge(itAdded.Value());
                //-----------------------
                //  E => New edges;
                //-----------------------
                NCollection_List<Shape> loopNE = loops.NewEdges(e);
                if (image.HasImage(e))
                {
                    image.Add(e, loopNE);
                }
                else
                {
                    image.Bind(e, loopNE);
                }
            }
        }
        loops.GetVerticesForSubstitute(_verVerMap);
        if (_verVerMap.Count == 0)
        {
            return;
        }
        BrepBuilder bb = new();
        using ProgressScope ps2 = new(psOuter.Next(), "Building loops", lf.Extent());
        for (it.Initialize(lf); it.More(); it.Next(), ps2.Next())
        {
            if (!ps2.More())
            {
                return;
            }
            Shape f = it.Value();
            NCollection_List<Shape> lif = new();
            image.LastImage(f, lif);
            for (itl.Initialize(lif); itl.More(); itl.Next())
            {
                Shape IF = itl.Value();
                Explorer edExp = new(IF, ShapeType.Edge);
                for (; edExp.More(); edExp.Next())
                {
                    Shape e = edExp.Current();
                    NCollection_List<Shape> vList = new();
                    Iterator verExp = new(e);
                    for (; verExp.More(); verExp.Next())
                    {
                        vList.Append(verExp.Value());
                    }
                    NCollection_List<Shape>.Iterator itlv = new(vList);
                    for (; itlv.More(); itlv.Next())
                    {
                        Shape v = itlv.Value();
                        if (_verVerMap.ContainsKey(v))
                        {
                            Shape newV = _verVerMap.GetValue(v);
                            e.Free(true);
                            newV.Orientation(v.Orientation());
                            BrepTVertex tv = (BrepTVertex)v.TShape()!;
                            BrepTVertex newTV = (BrepTVertex)newV.TShape()!;
                            if (tv.Tolerance() > newTV.Tolerance())
                            {
                                newTV.Tolerance(tv.Tolerance());
                            }
                            newTV.ChangePoints().Append(tv.ChangePoints()); // moves V's points, as OCCT's list Append does
                            asDes.Replace(v, newV);
                            bb.Remove(e, v);
                            bb.Add(e, newV);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    private static bool IsBetweenCorks(in Shape e, AsDes asDes, NCollection_List<Shape> lContext)
    {
        if (!asDes.HasAscendant(e))
        {
            return true;
        }
        NCollection_List<Shape> lf = asDes.Ascendant(e);
        NCollection_List<Shape>.Iterator it = new();
        for (it.Initialize(lf); it.More(); it.Next())
        {
            Shape s = it.Value();
            bool found = false;
            NCollection_List<Shape>.Iterator it2 = new();
            for (it2.Initialize(lContext); it2.More(); it2.Next())
            {
                if (s.IsSame(it2.Value()))
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
    /// Rebuilds the context faces (caps) <paramref name="lContext"/> from their edges and the traces of the offsets in
    /// <paramref name="asDes"/>, binding the new faces and edges in <paramref name="image"/>.
    /// </summary>
    public void BuildOnContext(NCollection_List<Shape> lContext, Analyse analyse, AsDes asDes,
                               ShapeImage image, bool inSide, ProgressRange range)
    {
        //-----------------------------------------
        // unwinding of caps.
        //-----------------------------------------
        NCollection_List<Shape>.Iterator it = new(lContext);
        NCollection_List<Shape>.Iterator itl = new(), itLCE = new();
        Loop loops = new();
        loops.VerticesForSubstitute(_verVerMap);
        Explorer exp = new();
        HashSet<Shape> mapExtent = new(ShapeHasher.Instance);

        using ProgressScope ps = new(range, "Building deepening faces", lContext.Extent());
        for (; it.More(); it.Next(), ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Face f = ShapeCasts.Face(it.Value());
            HashSet<Shape> mBound = new(ShapeHasher.Instance);
            //-----------------------------------------------
            // Initialisation of Loops.
            // F is reversed it will be added in myOffC.
            // and myOffC will be reversed in the final result.
            //-----------------------------------------------
            Shape reversedF = f.Reversed();
            if (inSide)
            {
                loops.Init(ShapeCasts.Face(reversedF));
                //    if (InSide) Loops.Init(TopoDS::Face(F.Reversed()));
            }
            else
            {
                loops.Init(f);
            }
            //--------------------------------------------------------
            // return edges of F not modified by definition.
            //--------------------------------------------------------
            for (exp.Init(f.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge ce = ShapeCasts.Edge(exp.Current());
                mBound.Add(ce);
                if (analyse.HasAncestor(ce))
                {
                    // the stop of cups except for the connectivity stops between caps.
                    //      if (!AsDes->HasAscendant(CE)) {
                    Shape reversedE = ce.Reversed();
                    if (inSide)
                    {
                        loops.AddConstEdge(ce);
                    }
                    else
                    {
                        loops.AddConstEdge(ShapeCasts.Edge(reversedE));
                    }
                    //	else        Loops.AddConstEdge(TopoDS::Edge(CE.Reversed()));
                }
            }
            //------------------------------------------------------
            // Trace of offsets + connectivity edge between caps.
            //------------------------------------------------------
            NCollection_List<Shape> le = asDes.Descendant(f);
            NCollection_List<Shape> addedEdges = new();

            for (itl.Initialize(le); itl.More(); itl.Next())
            {
                Edge e = ShapeCasts.Edge(itl.Value());
                if (image.HasImage(e))
                {
                    //-------------------------------------------
                    // E was already cut in another face.
                    // Return cut edges and orientate them as E.
                    // See pb for the edges that have disappeared?
                    //-------------------------------------------
                    NCollection_List<Shape> lce = image.Image(e);
                    for (itLCE.Initialize(lce); itLCE.More(); itLCE.Next())
                    {
                        Shape ce = itLCE.Value().Oriented(e.Orientation());
                        if (mapExtent.Contains(e))
                        {
                            loops.AddConstEdge(ShapeCasts.Edge(ce));
                            continue;
                        }
                        if (!mBound.Contains(e))
                        {
                            ce.Reverse();
                        }
                        if (inSide)
                        {
                            loops.AddConstEdge(ShapeCasts.Edge(ce));
                        }
                        else
                        {
                            Shape reversedE = ce.Reversed();
                            loops.AddConstEdge(ShapeCasts.Edge(reversedE));
                        }
                        //	  else        Loops.AddConstEdge(TopoDS::Edge(CE.Reversed()));
                    }
                }
                else
                {
                    if (IsBetweenCorks(e, asDes, lContext) && asDes.HasDescendant(e))
                    {
                        // connection between 2 caps
                        mapExtent.Add(e);
                        NCollection_List<Shape> lv = new();
                        if (inSide)
                        {
                            for (itLCE.Initialize(asDes.Descendant(e)); itLCE.More(); itLCE.Next())
                            {
                                lv.Append(itLCE.Value().Reversed());
                            }
                            loops.AddEdge(e, lv);
                        }
                        else
                        {
                            loops.AddEdge(e, asDes.Descendant(e));
                        }
                        addedEdges.Append(e);
                    }
                    else if (IsBetweenCorks(e, asDes, lContext))
                    {
                        Shape localShape = e.Reversed();
                        if (inSide)
                        {
                            loops.AddConstEdge(e);
                        }
                        else
                        {
                            loops.AddConstEdge(ShapeCasts.Edge(localShape));
                        }
                        //	  if (InSide) Loops.AddConstEdge(TopoDS::Edge(E));
                        //	  else        Loops.AddConstEdge(TopoDS::Edge(E.Reversed()));
                    }
                    else
                    {
                        Shape localShape = e.Reversed();
                        if (inSide)
                        {
                            loops.AddConstEdge(ShapeCasts.Edge(localShape));
                        }
                        else
                        {
                            loops.AddConstEdge(e);
                        }
                        //	  if (InSide) Loops.AddConstEdge(TopoDS::Edge(E.Reversed()));
                        //	  else        Loops.AddConstEdge(TopoDS::Edge(E));
                    }
                }
            }
            //------------------------
            // Unwind.
            //------------------------
            loops.Perform();
            loops.WiresToFaces();
            //------------------------
            // MAJ SD.
            //------------------------
            NCollection_List<Shape> nf = loops.NewFaces();
            //-----------------------
            // F => New faces;
            //-----------------------
            image.Bind(f, nf);

            NCollection_List<Shape>.Iterator itAdded = new();
            for (itAdded.Initialize(addedEdges); itAdded.More(); itAdded.Next())
            {
                Edge e = ShapeCasts.Edge(itAdded.Value());
                //-----------------------
                //  E => New edges;
                //-----------------------
                if (image.HasImage(e))
                {
                    image.Add(e, loops.NewEdges(e));
                }
                else
                {
                    image.Bind(e, loops.NewEdges(e));
                }
            }
        }
        loops.GetVerticesForSubstitute(_verVerMap);
        if (_verVerMap.Count == 0)
        {
            return;
        }
        BrepBuilder bb = new();
        for (it.Initialize(lContext); it.More(); it.Next())
        {
            Shape f = it.Value();
            NCollection_List<Shape> lif = new();
            image.LastImage(f, lif);
            for (itl.Initialize(lif); itl.More(); itl.Next())
            {
                Shape IF = itl.Value();
                Explorer edExp = new(IF, ShapeType.Edge);
                for (; edExp.More(); edExp.Next())
                {
                    Shape e = edExp.Current();
                    NCollection_List<Shape> vList = new();
                    Iterator verExp = new(e);
                    for (; verExp.More(); verExp.Next())
                    {
                        vList.Append(verExp.Value());
                    }
                    NCollection_List<Shape>.Iterator itlv = new(vList);
                    for (; itlv.More(); itlv.Next())
                    {
                        Shape v = itlv.Value();
                        if (_verVerMap.ContainsKey(v))
                        {
                            Shape newV = _verVerMap.GetValue(v);
                            e.Free(true);
                            newV.Orientation(v.Orientation());
                            BrepTVertex tv = (BrepTVertex)v.TShape()!;
                            BrepTVertex newTV = (BrepTVertex)newV.TShape()!;
                            if (tv.Tolerance() > newTV.Tolerance())
                            {
                                newTV.Tolerance(tv.Tolerance());
                            }
                            newTV.ChangePoints().Append(tv.ChangePoints()); // moves V's points, as OCCT's list Append does
                            asDes.Replace(v, newV);
                            bb.Remove(e, v);
                            bb.Add(e, newV);
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// Rebuilds the faces <paramref name="lf"/> not yet rebuilt whose edges were cut, binding the new faces and edges
    /// in <paramref name="image"/>.
    /// </summary>
    public void BuildFaces(NCollection_List<Shape> lf, AsDes asDes, ShapeImage image,
                           ProgressRange range)
    {
        NCollection_List<Shape>.Iterator itr = new(), itl = new(), itLCE = new();
        bool toRebuild;
        Loop loops = new();
        loops.VerticesForSubstitute(_verVerMap);
        BrepBuilder b = new();

        //----------------------------------
        // Loop on all faces //.
        //----------------------------------
        using ProgressScope ps = new(range, "Building faces", lf.Extent());
        for (itr.Initialize(lf); itr.More(); itr.Next(), ps.Next())
        {
            if (!ps.More())
            {
                return;
            }
            Face F = ShapeCasts.Face(itr.Value());
            loops.Init(F);
            toRebuild = false;
            NCollection_List<Shape> addedEdges = new();

            if (!image.HasImage(F))
            {
                //----------------------------------
                // Face F not yet reconstructed.
                //----------------------------------
                NCollection_List<Shape> le = asDes.Descendant(F);
                //----------------------------------------------------------------
                // first loop to find if the edges of the face were reconstructed.
                // - maj on map MONV. Some vertices on reconstructed edges
                // coincide geometrically with old but are not IsSame.
                //----------------------------------------------------------------
                Dictionary<Shape, Shape> monv = new(ShapeHasher.Instance);
                Vertex ov1, ov2, nv1, nv2;

                for (itl.Initialize(le); itl.More(); itl.Next())
                {
                    Edge e = ShapeCasts.Edge(itl.Value());
                    if (image.HasImage(e))
                    {
                        NCollection_List<Shape> lce = image.Image(e);
                        if (lce.Extent() == 1 && lce.First().IsSame(e))
                        {
                            Shape localShape = lce.First().Oriented(e.Orientation());
                            Edge ce = ShapeCasts.Edge(localShape);
                            //	    TopoDS_Edge CE = TopoDS::Edge(LCE.First().Oriented(E.Orientation()));
                            loops.AddConstEdge(ce);
                            continue;
                        }
                        //----------------------------------
                        // F should be reconstructed.
                        //----------------------------------
                        toRebuild = true;
                        for (itLCE.Initialize(lce); itLCE.More(); itLCE.Next())
                        {
                            Shape localShape = itLCE.Value().Oriented(e.Orientation());
                            Edge ce = ShapeCasts.Edge(localShape);
                            //	    TopoDS_Edge CE = TopoDS::Edge(itLCE.Value().Oriented(E.Orientation()));
                            Exploration.Vertices(e, out ov1, out ov2);
                            Exploration.Vertices(ce, out nv1, out nv2);
                            if (!ov1.IsSame(nv1))
                            {
                                monv[ov1] = nv1;
                            }
                            if (!ov2.IsSame(nv2))
                            {
                                monv[ov2] = nv2;
                            }
                            loops.AddConstEdge(ce);
                        }
                    }
                }
                if (toRebuild)
                {

                    //-----------------------------------------------------------
                    // Non-reconstructed edges on other faces are added.
                    // If their vertices were reconstructed they are reconstructed.
                    //-----------------------------------------------------------
                    for (itl.Initialize(le); itl.More(); itl.Next())
                    {
                        double f, l;
                        Edge e = ShapeCasts.Edge(itl.Value());
                        Tool.Range(e, out f, out l);
                        if (!image.HasImage(e))
                        {
                            Exploration.Vertices(e, out ov1, out ov2);
                            NCollection_List<Shape> lv = new();
                            if (monv.ContainsKey(ov1))
                            {
                                Vertex vv = ShapeCasts.Vertex(monv.GetValue(ov1));
                                vv.Orientation(Forward);
                                lv.Append(vv);
                                Shape localShape = vv.Oriented(Internal);
                                b.UpdateVertex(ShapeCasts.Vertex(localShape), f, e, Tool.Tolerance(vv));
                            }
                            if (monv.ContainsKey(ov2))
                            {
                                Vertex vv = ShapeCasts.Vertex(monv.GetValue(ov2));
                                vv.Orientation(Reversed);
                                lv.Append(vv);
                                Shape localShape = vv.Oriented(Internal);
                                b.UpdateVertex(ShapeCasts.Vertex(localShape), l, e, Tool.Tolerance(vv));
                                //	      B.UpdateVertex(TopoDS::Vertex(VV.Oriented(TopAbs_INTERNAL)),
                                //			     l,E,BRep_Tool::Tolerance(VV));
                            }
                            if (lv.IsEmpty())
                            {
                                loops.AddConstEdge(e);
                            }
                            else
                            {
                                loops.AddEdge(e, lv);
                                addedEdges.Append(e);
                            }
                        }
                    }
                }
            }
            if (toRebuild)
            {
                //------------------------
                // Reconstruction.
                //------------------------
                loops.Perform();
                loops.WiresToFaces();
                //------------------------
                // MAJ SD.
                //------------------------
                NCollection_List<Shape> nf = loops.NewFaces();
                //-----------------------
                // F => New faces;
                //-----------------------
                image.Bind(F, nf);

                NCollection_List<Shape>.Iterator itAdded = new();
                for (itAdded.Initialize(addedEdges); itAdded.More(); itAdded.Next())
                {
                    Edge e = ShapeCasts.Edge(itAdded.Value());
                    //-----------------------
                    //  E => New edges;
                    //-----------------------
                    if (image.HasImage(e))
                    {
                        image.Add(e, loops.NewEdges(e));
                    }
                    else
                    {
                        image.Bind(e, loops.NewEdges(e));
                    }
                }
            }
        }
        loops.GetVerticesForSubstitute(_verVerMap);
        if (_verVerMap.Count == 0)
        {
            return;
        }
        BrepBuilder bb = new();
        for (itr.Initialize(lf); itr.More(); itr.Next())
        {
            Shape F = itr.Value();
            NCollection_List<Shape> lif = new();
            image.LastImage(F, lif);
            for (itl.Initialize(lif); itl.More(); itl.Next())
            {
                Shape IF = itl.Value();
                Explorer edExp = new(IF, ShapeType.Edge);
                for (; edExp.More(); edExp.Next())
                {
                    Shape e = edExp.Current();
                    NCollection_List<Shape> vList = new();
                    Iterator verExp = new(e);
                    for (; verExp.More(); verExp.Next())
                    {
                        vList.Append(verExp.Value());
                    }
                    NCollection_List<Shape>.Iterator itlv = new(vList);
                    for (; itlv.More(); itlv.Next())
                    {
                        Shape v = itlv.Value();
                        if (_verVerMap.ContainsKey(v))
                        {
                            Shape newV = _verVerMap.GetValue(v);
                            e.Free(true);
                            newV.Orientation(v.Orientation());
                            BrepTVertex tv = (BrepTVertex)v.TShape()!;
                            BrepTVertex newTV = (BrepTVertex)newV.TShape()!;
                            if (tv.Tolerance() > newTV.Tolerance())
                            {
                                newTV.Tolerance(tv.Tolerance());
                            }
                            newTV.ChangePoints().Append(tv.ChangePoints()); // moves V's points, as OCCT's list Append does
                            asDes.Replace(v, newV);
                            bb.Remove(e, v);
                            bb.Add(e, newV);
                        }
                    }
                }
            }
        }
    }
}
