// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Quilt.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Quilt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
/// edge bound with <see cref="Bind(in TopoDS_Edge, in TopoDS_Edge)"/> to a frozen edge (any shape a caller holds)
/// therefore makes <see cref="Add"/> throw <see cref="TopoDS_LockedShape"/>, as OCCT does for a locked one; ported code
/// passes edges it built itself.
/// </para>
/// </remarks>
public sealed class BRepTools_Quilt
{
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myBounds = new(TopTools_ShapeMapHasher.Instance);
    private bool hasCopy;

    /// <summary>An empty quilt.</summary>
    public BRepTools_Quilt()
    {
        hasCopy = false;
    }

    // test if the shape must be copied
    // i.e. it contains a bound subshape
    private static bool NeedCopied(in TopoDS_Shape theShape, NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myBounds)
    {
        bool IsCopied = false;
        var itv = new TopoDS_Iterator(theShape);
        for (; itv.More(); itv.Next())
        {
            if (myBounds.Contains(itv.Value()))
            {
                IsCopied = true;
                break;
            }
        }
        return IsCopied;
    }

    private static void CopyShape(in TopoDS_Edge E, NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> myBounds)
    {
        TopoDS_Edge NE = E;
        NE.EmptyCopy();
        NE.Orientation(TopAbs_FORWARD);
        BRep_Builder B = new();
        // add the edges
        var itv = new TopoDS_Iterator();
        itv.Initialize(E, false); // TCollection_DataMap
        for (; itv.More(); itv.Next())
        {
            TopoDS_Shape V = itv.Value();
            if (myBounds.Contains(V))
            {
                B.Add(NE, myBounds.FindFromKey(V).Oriented(V.Orientation()));
            }
            else
            {
                B.Add(NE, V);
            }
        }
        // set the 3d range
        BRep_Tool.Range(E, out double f, out double l);
        B.Range(NE, f, l);
        myBounds.Add(E, NE.Oriented(TopAbs_FORWARD));
    }

    /// <summary>
    /// Adds the faces of the shape: a face containing a bound edge or vertex is copied with the substitutions.
    /// </summary>
    public void Add(in TopoDS_Shape S)
    {
        //  Binds all the faces of S
        //   - to the face itself if it is not copied
        //   - to the copy if it is copied
        if (myBounds.Contains(S))
        {
            return;
        }

        BRep_Builder B = new();
        for (var wex = new TopExp_Explorer(S, TopAbs_WIRE, TopAbs_FACE); wex.More(); wex.Next())
        {
            myBounds.Add(wex.Current(), wex.Current());
        }

        for (var eex = new TopExp_Explorer(S, TopAbs_EDGE, TopAbs_WIRE); eex.More(); eex.Next())
        {
            myBounds.Add(eex.Current(), eex.Current());
        }

        for (var vex = new TopExp_Explorer(S, TopAbs_VERTEX, TopAbs_EDGE); vex.More(); vex.Next())
        {
            myBounds.Add(vex.Current(), vex.Current());
        }

        // explore the faces
        for (var fex = new TopExp_Explorer(S, TopAbs_FACE); fex.More(); fex.Next())
        {
            // explore the edges of the face and try to copy them
            // if one edge is bound the face must be copied

            bool copyFace = false;
            TopoDS_Face F = TopoDS.Face(fex.Current());

            if (hasCopy)
            { // if their is no binding, do not test for copy

                for (var fed = new TopExp_Explorer(F, TopAbs_EDGE); fed.More(); fed.Next())
                {
                    if (myBounds.Contains(fed.Current()))
                    {
                        copyFace = true;
                    }
                    else
                    {
                        // test if the edge must be copied
                        // i.e. it contains a bound vertex

                        bool copyEdge = NeedCopied(fed.Current(), myBounds);
                        TopoDS_Edge E = TopoDS.Edge(fed.Current());

                        if (copyEdge)
                        {
                            // copy of an edge

                            copyFace = true;
                            CopyShape(E, myBounds);
                        }
                    }
                }
            }

            // NF will be the copy of F or F itself
            TopoDS_Face NF = F;

            if (copyFace)
            {
                // copy of a face

                NF.EmptyCopy();
                NF.Orientation(TopAbs_FORWARD);

                for (var itw = new TopoDS_Iterator(F, false); itw.More(); itw.Next())
                {
                    TopoDS_Wire W = TopoDS.Wire(itw.Value());

                    TopoDS_Wire NW = new();
                    B.MakeWire(ref NW);
                    var ite = new TopoDS_Iterator(W, false);
                    double UFirst, ULast;

                    // Reconstruction of wires.

                    for (; ite.More(); ite.Next())
                    {
                        TopoDS_Edge E = TopoDS.Edge(ite.Value());
                        TopAbs_Orientation OE = E.Orientation();
                        if (myBounds.Contains(E))
                        {
                            TopoDS_Edge NE = TopoDS.Edge(myBounds.FindFromKey(E));
                            // pcurve.
                            if (NE.Orientation() == TopAbs_FORWARD)
                            {
                                B.UpdateEdge(NE,
                                             BRep_Tool.CurveOnSurface(E, F, out UFirst, out ULast),
                                             F,
                                             BRep_Tool.Tolerance(E));
                            }
                            else
                            {
                                // If NE is REVERSED
                                // => the 3D curves do not have the same orientation.
                                // (This is a convention, cf. BRepTools_Quilt.cdl and the Bind
                                // method.)
                                // => the PCurve of E on F must be reversed.

                                OE = TopAbs.Reverse(OE);
                                Geom2d_Curve CE = BRep_Tool.CurveOnSurface(E, F, out UFirst, out ULast)!;
                                Geom2d_Curve NCE = CE.Reversed();
                                B.UpdateEdge(NE, NCE, F, BRep_Tool.Tolerance(E));
                                double tmp = UFirst;
                                UFirst = CE.ReversedParameter(ULast);
                                ULast = CE.ReversedParameter(tmp);
                            }
                            // pcurve range
                            B.Range(NE, F, UFirst, ULast);

                            B.Add(NW, NE.Oriented(OE));
                        }
                        else
                        {
                            B.Add(NW, E);
                        }
                    }
                    NW.Orientation(W.Orientation());
                    B.Add(NF, NW);
                }
                NF.Orientation(F.Orientation());
            }

            // binds the face to itself or its copy
            myBounds.Add(F, NF);
        }
    }

    /// <summary>
    /// Binds <paramref name="Vnew"/> to be the new vertex instead of <paramref name="Vold"/>: the faces of the next
    /// added shape containing <paramref name="Vold"/> are copied with <paramref name="Vnew"/>.
    /// </summary>
    public void Bind(in TopoDS_Vertex Vold, in TopoDS_Vertex Vnew)
    {
        if (!myBounds.Contains(Vold))
        {
            myBounds.Add(Vold, Vnew);
        }
    }

    /// <summary>
    /// Binds <paramref name="Enew"/> (in a shape added before) to be the new edge instead of <paramref name="Eold"/>
    /// (in the next added shape): the faces containing <paramref name="Eold"/> are copied with
    /// <paramref name="Enew"/>, and the vertices of <paramref name="Eold"/> are bound to those of
    /// <paramref name="Enew"/> with the same orientation. Edges of different orientations have opposite curves: the
    /// pcurves of <paramref name="Eold"/> are reversed in the new faces.
    /// </summary>
    public void Bind(in TopoDS_Edge Eold, in TopoDS_Edge Enew)
    {
        if (!myBounds.Contains(Eold))
        {
            TopoDS_Edge ENew = Enew;
            if (IsCopied(Enew))
            {
                ENew = TopoDS.Edge(InternalCopy(Enew));
                ENew.Orientation(Enew.Orientation());
            }

            if (Eold.Orientation() != ENew.Orientation())
            {
                myBounds.Add(Eold.Oriented(TopAbs_FORWARD), ENew.Oriented(TopAbs_REVERSED));
            }
            else
            {
                myBounds.Add(Eold.Oriented(TopAbs_FORWARD), ENew.Oriented(TopAbs_FORWARD));
            }
            // if new binding bind also the vertices
            var itold = new TopoDS_Iterator(Eold);
            while (itold.More())
            {
                if (!myBounds.Contains(itold.Value()))
                {
                    // find the vertex of Enew with same orientation
                    TopAbs_Orientation anOrien = itold.Value().Orientation();
                    var itnew = new TopoDS_Iterator(ENew);
                    while (itnew.More())
                    {
                        if (itnew.Value().Orientation() == anOrien)
                        {
                            TopoDS_Vertex VNew = TopoDS.Vertex(itnew.Value());
                            if (IsCopied(VNew))
                            {
                                // if VNew has been copied take the copy
                                VNew = TopoDS.Vertex(InternalCopy(VNew));
                            }
                            myBounds.Add(itold.Value(), VNew);
                            break;
                        }
                        itnew.Next();
                    }
                }
                itold.Next();
            }
            hasCopy = true;
        }
    }

    /// <summary>True if the shape (a vertex, an edge or a face) has been copied.</summary>
    public bool IsCopied(in TopoDS_Shape S)
    {
        if (myBounds.Contains(S))
        {
            return !S.IsSame(myBounds.FindFromKey(S));
        }
        else
        {
            return false;
        }
    }

    /// <summary>The shape substituted for <paramref name="S"/> in the quilt, frozen.</summary>
    /// <exception cref="NoSuchObjectException">The shape hasn't been copied.</exception>
    public TopoDS_Shape Copy(in TopoDS_Shape S)
    {
        TopoDS_Shape aCopy = InternalCopy(S);
        aCopy.TShape()?.FreezeGraph();
        return aCopy;
    }

    /// <summary>The shape substituted for <paramref name="S"/>, as <see cref="Copy"/>, without freezing it (ported code).</summary>
    internal TopoDS_Shape InternalCopy(in TopoDS_Shape S)
    {
        Standard_NoSuchObject_Raise_if(!IsCopied(S), "BRepTools_Quilt::Copy");
        return myBounds.FindFromKey(S);
    }

    /// <summary>
    /// A compound of shells made from the current faces, each flagged closed or not, frozen. Shapes added without faces
    /// (free edges and vertices) are added to the compound too.
    /// </summary>
    public TopoDS_Shape Shells()
    {
        TopoDS_Shape aResult = InternalShells();
        aResult.TShape()!.FreezeGraph();
        return aResult;
    }

    /// <summary>The compound of shells, as <see cref="Shells"/>, without freezing it (ported code).</summary>
    internal TopoDS_Shape InternalShells()
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

        var M = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var MF = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        BRep_Builder B = new();
        TopoDS_Compound result = new();

        B.MakeCompound(ref result);

        var MapOtherShape = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance); // gka
        var EdgesFaces = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        // loop on the face in myBounds
        for (int ii = 1; ii <= myBounds.Extent(); ii++)
        {
            TopoDS_Shape Shape = myBounds.FindFromIndex(ii); // it.Value();
            if (Shape.ShapeType() == TopAbs_FACE)
            {
                for (var aExpEdg = new TopExp_Explorer(Shape, TopAbs_EDGE); aExpEdg.More(); aExpEdg.Next())
                { // gka
                    EdgesFaces.Add(aExpEdg.Current());
                }

                TopoDS_Shell SH = new();
                TopAbs_Orientation NewO;

                var itf1 = new TopExp_Explorer(Shape, TopAbs_EDGE);
                for (; itf1.More(); itf1.Next())
                {
                    TopoDS_Shape E = itf1.Current();
                    if (M.IsBound(E))
                    {
                        SH = TopoDS.Shell(M[E]);
                        if (SH.Orientation() == E.Orientation())
                        {
                            NewO = TopAbs.Reverse(Shape.Orientation());
                        }
                        else
                        {
                            NewO = Shape.Orientation();
                        }

                        MF.Bind(Shape, SH.Oriented(NewO));
                        break;
                    }
                }

                if (SH.IsNull())
                {
                    // Create a new shell, closed. Add it to the result.
                    B.MakeShell(ref SH);
                    SH.Closed(true);
                    B.Add(result, SH);
                    MF.Bind(Shape, SH.Oriented(Shape.Orientation()));
                }

                // Add the face to the shell
                SH.Free(true);
                TopoDS_Shape arefShape = SH.Oriented(TopAbs_FORWARD);
                B.Add(arefShape, Shape.Oriented(MF[Shape].Orientation()));

                var itf = new TopExp_Explorer(Shape.Oriented(TopAbs_FORWARD), TopAbs_EDGE);

                for (; itf.More(); itf.Next())
                {
                    TopoDS_Edge E = TopoDS.Edge(itf.Current());

                    if (M.IsBound(E))
                    {
                        TopoDS_Shape oldShell = M[E];
                        if (!oldShell.IsSame(SH))
                        {
                            // Fuse the old shell with the new one
                            // Compare the orientation of E in SH and in oldshell.
                            TopAbs_Orientation anOrien = E.Orientation();
                            if (MF[Shape].Orientation() == TopAbs_REVERSED)
                            {
                                anOrien = TopAbs.Reverse(anOrien);
                            }

                            bool Rev = (anOrien == oldShell.Orientation());
                            // if rev = True oldShell has to be reversed.

                            // Add the faces of oldShell in SH.
                            for (var its = new TopoDS_Iterator(oldShell); its.More(); its.Next())
                            {
                                TopoDS_Face Fo = TopoDS.Face(its.Value());
                                TopAbs_Orientation NewOFo;
                                // update the orientation of Fo in SH.
                                if (Rev)
                                {
                                    NewOFo = TopAbs.Reverse(MF[Fo].Orientation());
                                }
                                else
                                {
                                    NewOFo = MF[Fo].Orientation();
                                }

                                MF.Bind(Fo, SH.Oriented(NewOFo));
                                TopoDS_Shape arefShapeFo = SH.Oriented(TopAbs_FORWARD);
                                B.Add(arefShapeFo, Fo.Oriented(NewOFo));
                            }
                            // Rebind the free edges of the old shell to the new shell
                            // gka BUG 6491
                            var aexp = new TopExp_Explorer(SH, TopAbs_EDGE);
                            for (; aexp.More(); aexp.Next())
                            {
                                if (!M.IsBound(aexp.Current()))
                                {
                                    continue;
                                }
                                TopoDS_Shape ae = aexp.Current();
                                TopoDS_Shape @as = M.Find(ae);
                                if (@as.IsSame(oldShell))
                                {
                                    // update the orientation of free edges in SH.
                                    if (Rev)
                                    {
                                        NewO = TopAbs.Reverse(@as.Orientation());
                                    }
                                    else
                                    {
                                        NewO = @as.Orientation();
                                    }

                                    M.Bind(ae, SH.Oriented(NewO));
                                }
                            }
                            // remove the old shell from the result
                            B.Remove(result, oldShell.Oriented(TopAbs_FORWARD));
                        }
                        {
                            // Test if SH is always orientable.
                            TopAbs_Orientation anOrien = E.Orientation();
                            if (MF[Shape].Orientation() == TopAbs_REVERSED)
                            {
                                anOrien = TopAbs.Reverse(anOrien);
                            }

                            if (M[E].Orientation() == anOrien)
                            {
                                SH.Orientable(false);
                            }
                        }

                        // remove the edge from M (no more a free edge)
                        M.UnBind(E);
                    }
                    else
                    {
                        NewO = E.Orientation();
                        if (MF[Shape].Orientation() == TopAbs_REVERSED)
                        {
                            NewO = TopAbs.Reverse(NewO);
                        }
                        if (!E.IsNull())
                        {
                            M.Bind(E, SH.Oriented(NewO));
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
                MapOtherShape.Add(Shape);
            }
        }

        // Unclose all shells having free edges
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> it in M)
        {
            TopoDS_Shape S = it.Value;
            S.Closed(false);
        }

        // gka version for free edges
        foreach (TopoDS_Shape itother in MapOtherShape)
        {
            if (!EdgesFaces.Contains(itother) && myBounds.Contains(itother))
            {
                TopoDS_Shape aSh = myBounds.FindFromKey(itother);
                B.Add(result, aSh);
            }
        }
        return result;
    }
}
