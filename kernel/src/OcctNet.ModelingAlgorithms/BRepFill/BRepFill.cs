// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepExtrema_SupportType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Ruled faces and shells between edges and wires, and tools on wires used by the sweeps and lofts: the axis of a
/// profile on a planar spine, the reduced curvilinear abscissae of a wire's vertices and the cutting of a wire at
/// given abscissae.
/// </summary>
/// <remarks>
/// <para>
/// <see cref="Face"/> and <see cref="Shell"/> put the given edges into the new face (or shell) and give them pcurves
/// on it, as OCCT does: OCCT writes into the caller's edges. The port writes into copies of them when the inputs hold
/// frozen shapes (decision 5851): the result then holds changed copies of the edges and the caller's own vertices
/// wherever nothing changed them. Unfrozen inputs, which only ported code holds, are used in place, as in OCCT.
/// </para>
/// <para>The results handed out are frozen; ported callers that keep building use the <c>Internal</c> forms.</para>
/// </remarks>
public static class BRepFill
{
    private static void MakeWire(NCollection_Array1<TopoDS_Shape> Edges, int rangdeb, bool forward, ref TopoDS_Wire newwire)
    {
        BRep_Builder BW = new();
        int rang, nbEdges = Edges.Length();
        BW.MakeWire(ref newwire);
        if (forward)
        {
            for (rang = rangdeb; rang <= nbEdges; rang++)
            {
                BW.Add(newwire, TopoDS.Edge(Edges[rang]));
            }
            for (rang = 1; rang < rangdeb; rang++)
            {
                BW.Add(newwire, TopoDS.Edge(Edges[rang]));
            }
        }

        else
        {
            TopoDS_Edge E;
            for (rang = rangdeb; rang >= 1; rang--)
            {
                E = TopoDS.Edge(Edges[rang]);
                BW.Add(newwire, E.Reversed());
            }
            for (rang = nbEdges; rang > rangdeb; rang--)
            {
                E = TopoDS.Edge(Edges[rang]);
                BW.Add(newwire, E.Reversed());
            }
        }
        newwire.Orientation(TopAbs_FORWARD);
        newwire.Closed(true);
    }

    private static void CutEdge(in TopoDS_Edge CurrentEdge, double Param, out TopoDS_Edge E1, out TopoDS_Edge E2,
                                in TopoDS_Vertex VRef)
    {
        BRep_Builder B = new();
        Geom_Curve C = BRep_Tool.Curve(CurrentEdge, out double first, out double last)!;
        TopoDS_Vertex Vi = new();
        B.MakeVertex(ref Vi, C.Value(Param), Precision.Confusion());
        TopExp.Vertices(CurrentEdge, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
        if (VRef.IsSame(Vf))
        {
            E1 = new BRepLib_MakeEdge(C, Vf, Vi, first, Param).InternalEdge();
            E2 = new BRepLib_MakeEdge(C, Vi, Vl, Param, last).InternalEdge();
        }
        else
        {
            E2 = new BRepLib_MakeEdge(C, Vf, Vi, first, Param).InternalEdge();
            E1 = new BRepLib_MakeEdge(C, Vi, Vl, Param, last).InternalEdge();
        }
    }

    private static void TrimEdge(in TopoDS_Edge CurrentEdge,
                                 NCollection_Sequence<double> CutValues,
                                 double t0,
                                 double t1,
                                 bool SeqOrder,
                                 NCollection_Sequence<TopoDS_Shape> S)

    {
        S.Clear();
        int j, ndec = CutValues.Length();
        double m0, m1;
        Geom_Curve C = BRep_Tool.Curve(CurrentEdge, out double first, out double last)!;

        TopoDS_Vertex Vbid = new(), V0, V1;
        TopAbs_Orientation CurrentOrient = CurrentEdge.Orientation();
        TopExp.Vertices(CurrentEdge, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
        Vbid.Nullify();

        if (SeqOrder)
        {
            // from first to last
            m0 = first;
            V0 = Vf;
            for (j = 1; j <= ndec; j++)
            {
                // piece of edge
                m1 = (CutValues.Value(j) - t0) * (last - first) / (t1 - t0) + first;
                TopoDS_Edge CutE = new BRepLib_MakeEdge(C, V0, Vbid, m0, m1).InternalEdge();
                CutE.Orientation(CurrentOrient);
                S.Append(CutE);
                m0 = m1;
                V0 = TopExp.LastVertex(CutE);
                if (j == ndec)
                {
                    // last piece
                    TopoDS_Edge LastE = new BRepLib_MakeEdge(C, V0, Vl, m0, last).InternalEdge();
                    LastE.Orientation(CurrentOrient);
                    S.Append(LastE);
                }
            }
        }
        else
        {
            // from last to first
            m1 = last;
            V1 = Vl;
            for (j = ndec; j >= 1; j--)
            {
                // piece of edge
                m0 = (CutValues.Value(j) - t0) * (last - first) / (t1 - t0) + first;
                TopoDS_Edge CutE = new BRepLib_MakeEdge(C, Vbid, V1, m0, m1).InternalEdge();
                CutE.Orientation(CurrentOrient);
                S.Append(CutE);
                m1 = m0;
                V1 = TopExp.FirstVertex(CutE);
                if (j == 1)
                {
                    // last piece
                    TopoDS_Edge LastE = new BRepLib_MakeEdge(C, Vf, V1, first, m1).InternalEdge();
                    LastE.Orientation(CurrentOrient);
                    S.Append(LastE);
                }
            }
        }
    }

    // ----------------------------------------------------------------------------------------------------------------
    // Copies of frozen inputs (decision 5851)

    /// <summary>
    /// The two input shapes to write into: themselves when neither holds a
    /// frozen TShape, else their copies, made at once so that the sub-shapes they share stay shared.
    /// </summary>
    private sealed class WriteCopies
    {
        public readonly TopoDS_Shape Original = new();
        public readonly TopoDS_Shape Copy = new();
        public readonly Dictionary<TopoDS_TShape, TopoDS_TShape>? Copies;
        public readonly TopoDS_Shape S1, S2;

        public WriteCopies(in TopoDS_Shape theS1, in TopoDS_Shape theS2)
        {
            S1 = theS1;
            S2 = theS2;
            if (!BRepTools.HoldsFrozen(theS1) && !BRepTools.HoldsFrozen(theS2))
            {
                return;
            }

            BRep_Builder B = new();
            TopoDS_Compound aHolder = new();
            B.MakeCompound(ref aHolder);
            B.Add(aHolder, theS1);
            B.Add(aHolder, theS2);
            Original = aHolder;
            Copy = BRepTools.ExactCopyForWrite(aHolder, out _, out Copies);
            TopoDS_Iterator anIt = new(Copy, false, false);
            S1 = anIt.Value();
            anIt.Next();
            S2 = anIt.Value();
        }

        /// <summary>Gives back in <paramref name="theResult"/> the caller's sub-shapes the algorithm left unchanged.</summary>
        public TopoDS_Shape KeepUnchanged(in TopoDS_Shape theResult)
        {
            TopoDS_Shape aResult = theResult;
            if (Copies is not null)
            {
                BRepTools.KeepUnchangedIn(Original, Copy, Copies, ref aResult, out _);
            }
            return aResult;
        }
    }

    //=================================================================================================

    /// <summary>
    /// A ruled face between <paramref name="Edge1"/> and <paramref name="Edge2"/>, bounded by them and by the rulings
    /// through their ends (degenerated edges where those ends meet); frozen.
    /// </summary>
    public static TopoDS_Face Face(in TopoDS_Edge Edge1, in TopoDS_Edge Edge2)
    {
        TopoDS_Face aResult = InternalFace(Edge1, Edge2);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary><see cref="Face"/> without freezing the result, for ported callers that build on it.</summary>
    internal static TopoDS_Face InternalFace(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2)
    {
        WriteCopies aCopies = new(theEdge1, theEdge2);
        return TopoDS.Face(aCopies.KeepUnchanged(face(TopoDS.Edge(aCopies.S1), TopoDS.Edge(aCopies.S2))));
    }

    private static TopoDS_Face face(TopoDS_Edge Edge1, TopoDS_Edge Edge2)
    {
        TopoDS_Face Face = new();

        BRep_Builder B = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool BT;

        TopLoc_Location L = new();
        double Tol;

        Geom_Curve C1 = BRep_Tool.Curve(Edge1, out TopLoc_Location L1, out double f1, out double l1)!;
        Geom_Curve C2 = BRep_Tool.Curve(Edge2, out TopLoc_Location L2, out double f2, out double l2)!;

        // compute the location
        bool SameLoc = false;
        if (L1 == L2)
        {
            L = L1;
            L1 = L2 = new TopLoc_Location();
            SameLoc = true;
        }

        // transform and trim the curves

        TopoDS_Vertex V1f, V1l, V2f, V2l;

        // create a new Handle
        if (Math.Abs(f1 - C1.FirstParameter()) > Precision.PConfusion()
            || Math.Abs(l1 - C1.LastParameter()) > Precision.PConfusion())
        {
            C1 = new Geom_TrimmedCurve(C1, f1, l1);
        }
        else
        {
            C1 = (Geom_Curve)C1.Copy();
        }
        // eventually the curve is concerned
        if (!SameLoc)
        {
            C1.Transform(L1.Transformation());
        }
        // it is set in the proper direction and its vertices are taken
        if (Edge1.Orientation() == TopAbs_REVERSED)
        {
            TopExp.Vertices(Edge1, out V1l, out V1f);
            C1.Reverse();
        }
        else
        {
            TopExp.Vertices(Edge1, out V1f, out V1l);
        }

        // a new Handle is created
        if (Math.Abs(f2 - C2.FirstParameter()) > Precision.PConfusion()
            || Math.Abs(l2 - C2.LastParameter()) > Precision.PConfusion())
        {
            C2 = new Geom_TrimmedCurve(C2, f2, l2);
        }
        else
        {
            C2 = (Geom_Curve)C2.Copy();
        }
        // eventually the curve is concerned
        if (!SameLoc)
        {
            C2.Transform(L2.Transformation());
        }
        // it is set in the proper direction and its vertices are taken
        if (Edge2.Orientation() == TopAbs_REVERSED)
        {
            TopExp.Vertices(Edge2, out V2l, out V2f);
            C2.Reverse();
        }
        else
        {
            TopExp.Vertices(Edge2, out V2f, out V2l);
        }

        // Are they closed edges?
        bool Closed = V1f.IsSame(V1l) && V2f.IsSame(V2l);

        GeomFill_Generator Generator = new();
        Generator.AddCurve(C1);
        Generator.AddCurve(C2);
        Generator.Perform(Precision.PConfusion());

        Geom_Surface Surf = Generator.InternalSurface()!;
        Geom_Curve Iso;

        B.MakeFace(ref Face, Surf, Precision.Confusion());

        // make the missing edges
        Surf.Bounds(out f1, out l1, out f2, out l2);

        TopoDS_Edge Edge3 = new(), Edge4 = new();

        Iso = Surf.UIso(f1);
        Tol = Max(BRep_Tool.Tolerance(V1f), BRep_Tool.Tolerance(V2f));
        if (Iso.Value(f2).Distance(Iso.Value(l2)) > Tol)
        {
            B.MakeEdge(ref Edge3, Iso, Precision.Confusion());
        }
        else
        {
            B.MakeEdge(ref Edge3);
            B.Degenerated(Edge3, true);
        }
        V1f.Orientation(TopAbs_FORWARD);
        B.Add(Edge3, V1f);
        V2f.Orientation(TopAbs_REVERSED);
        B.Add(Edge3, V2f);
        B.Range(Edge3, f2, l2);

        if (Closed)
        {
            Edge4 = Edge3;
        }
        else
        {
            Iso = Surf.UIso(l1);
            Tol = Max(BRep_Tool.Tolerance(V1l), BRep_Tool.Tolerance(V2l));
            if (Iso.Value(l2).Distance(Iso.Value(f2)) > Tol)
            {
                B.MakeEdge(ref Edge4, Iso, Precision.Confusion());
            }
            else
            {
                B.MakeEdge(ref Edge4);
                B.Degenerated(Edge4, true);
            }
            V1l.Orientation(TopAbs_FORWARD);
            B.Add(Edge4, V1l);
            V2l.Orientation(TopAbs_REVERSED);
            B.Add(Edge4, V2l);
            B.Range(Edge4, f2, l2);
        }

        // make the wire

        TopoDS_Wire W = new();
        B.MakeWire(ref W);

        Edge3.Reverse();
        B.Add(W, Edge1);
        B.Add(W, Edge4);
        B.Add(W, Edge2.Reversed());
        B.Add(W, Edge3);
        W.Closed(true);

        B.Add(Face, W);

        // set the pcurves

        double T = Precision.Confusion();

        if (Edge1.Orientation() == TopAbs_REVERSED)
        {
            B.UpdateEdge(Edge1, new Geom2d_Line(new gp_Pnt2d(0, f2), new gp_Dir2d(gp_Dir2d.D.NX)), Face, T);
            B.Range(Edge1, Face, -l1, -f1);
        }
        else
        {
            B.UpdateEdge(Edge1, new Geom2d_Line(new gp_Pnt2d(0, f2), new gp_Dir2d(gp_Dir2d.D.X)), Face, T);
            B.Range(Edge1, Face, f1, l1);
        }

        if (Edge2.Orientation() == TopAbs_REVERSED)
        {
            B.UpdateEdge(Edge2, new Geom2d_Line(new gp_Pnt2d(0, l2), new gp_Dir2d(gp_Dir2d.D.NX)), Face, T);
            B.Range(Edge2, Face, -l1, -f1);
        }
        else
        {
            B.UpdateEdge(Edge2, new Geom2d_Line(new gp_Pnt2d(0, l2), new gp_Dir2d(gp_Dir2d.D.X)), Face, T);
            B.Range(Edge2, Face, f1, l1);
        }

        if (Closed)
        {
            B.UpdateEdge(Edge3,
                         new Geom2d_Line(new gp_Pnt2d(l1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                         new Geom2d_Line(new gp_Pnt2d(f1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                         Face,
                         T);
        }
        else
        {
            B.UpdateEdge(Edge3, new Geom2d_Line(new gp_Pnt2d(f1, 0), new gp_Dir2d(gp_Dir2d.D.Y)), Face, T);
            B.UpdateEdge(Edge4, new Geom2d_Line(new gp_Pnt2d(l1, 0), new gp_Dir2d(gp_Dir2d.D.Y)), Face, T);
        }

        // Set the non parameter flag;
        B.SameParameter(Edge1, false);
        B.SameParameter(Edge2, false);
        B.SameParameter(Edge3, false);
        B.SameParameter(Edge4, false);
        B.SameRange(Edge1, false);
        B.SameRange(Edge2, false);
        B.SameRange(Edge3, false);
        B.SameRange(Edge4, false);

        BRepLib.SameParameter(Face);

        if (SameLoc)
        {
            Face.Move(L);
        }
        return Face;
    }

    //=================================================================================================

    /// <summary>
    /// The ruled faces between the edges of <paramref name="Wire1"/> and <paramref name="Wire2"/>, taken pairwise in
    /// the order of their explorers (the wires should have the same number of edges), sewn into a shell; frozen.
    /// </summary>
    public static TopoDS_Shell Shell(in TopoDS_Wire Wire1, in TopoDS_Wire Wire2)
    {
        TopoDS_Shell aResult = InternalShell(Wire1, Wire2);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary><see cref="Shell"/> without freezing the result, for ported callers that build on it.</summary>
    internal static TopoDS_Shell InternalShell(in TopoDS_Wire theWire1, in TopoDS_Wire theWire2)
    {
        WriteCopies aCopies = new(theWire1, theWire2);
        return TopoDS.Shell(aCopies.KeepUnchanged(shell(TopoDS.Wire(aCopies.S1), TopoDS.Wire(aCopies.S2))));
    }

    private static TopoDS_Shell shell(in TopoDS_Wire Wire1, in TopoDS_Wire Wire2)
    {
        TopoDS_Shell Shell = new();
        TopoDS_Face Face = new();
        TopoDS_Edge Edge1, Edge2, Edge3 = new(), Edge4 = new(), Couture = new();

        BRep_Builder B = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool BT;
        B.MakeShell(ref Shell);

        TopExp_Explorer ex1 = new();
        TopExp_Explorer ex2 = new();

        bool Closed = Wire1.Closed() && Wire2.Closed();

        bool thefirst = true;

        ex1.Init(Wire1, TopAbs_EDGE);
        ex2.Init(Wire2, TopAbs_EDGE);

        while (ex1.More() && ex2.More())
        {

            Edge1 = TopoDS.Edge(ex1.Current());
            Edge2 = TopoDS.Edge(ex2.Current());

            bool Periodic = BRep_Tool.IsClosed(Edge1) && BRep_Tool.IsClosed(Edge2);

            ex1.Next();
            ex2.Next();

            TopLoc_Location L = new();
            double Tol;

            Geom_Curve C1 = BRep_Tool.Curve(Edge1, out TopLoc_Location L1, out double f1, out double l1)!;
            Geom_Curve C2 = BRep_Tool.Curve(Edge2, out TopLoc_Location L2, out double f2, out double l2)!;

            // compute the location
            bool SameLoc = false;
            if (L1 == L2)
            {
                L = L1;
                L1 = L2 = new TopLoc_Location();
                SameLoc = true;
            }

            // transform and trim the curves

            TopoDS_Vertex V1f, V1l, V2f, V2l;

            if (Math.Abs(f1 - C1.FirstParameter()) > Precision.PConfusion()
                || Math.Abs(l1 - C1.LastParameter()) > Precision.PConfusion())
            {
                C1 = new Geom_TrimmedCurve(C1, f1, l1);
            }
            else
            {
                C1 = (Geom_Curve)C1.Copy();
            }
            if (!SameLoc)
            {
                C1.Transform(L1.Transformation());
            }
            if (Edge1.Orientation() == TopAbs_REVERSED)
            {
                TopExp.Vertices(Edge1, out V1l, out V1f);
                C1.Reverse();
            }
            else
            {
                TopExp.Vertices(Edge1, out V1f, out V1l);
            }

            if (Math.Abs(f2 - C2.FirstParameter()) > Precision.PConfusion()
                || Math.Abs(l2 - C2.LastParameter()) > Precision.PConfusion())
            {
                C2 = new Geom_TrimmedCurve(C2, f2, l2);
            }
            else
            {
                C2 = (Geom_Curve)C2.Copy();
            }
            if (!SameLoc)
            {
                C2.Transform(L2.Transformation());
            }
            if (Edge2.Orientation() == TopAbs_REVERSED)
            {
                TopExp.Vertices(Edge2, out V2l, out V2f);
                C2.Reverse();
            }
            else
            {
                TopExp.Vertices(Edge2, out V2f, out V2l);
            }

            GeomFill_Generator Generator = new();
            Generator.AddCurve(C1);
            Generator.AddCurve(C2);
            Generator.Perform(Precision.PConfusion());

            Geom_Surface Surf = Generator.InternalSurface()!;
            Geom_Curve Iso;

            B.MakeFace(ref Face, Surf, Precision.Confusion());

            // make the missing edges
            Surf.Bounds(out f1, out l1, out f2, out l2);

            if (thefirst)
            {
                Iso = Surf.UIso(f1);
                //      Tol = std::max(BT.Tolerance(V1f), BT.Tolerance(V2f));
                Tol = Max(BRep_Tool.Tolerance(V1f), BRep_Tool.Tolerance(V2f));
                if (Iso.Value(f2).Distance(Iso.Value(l2)) > Tol)
                {
                    B.MakeEdge(ref Edge3, Iso, Precision.Confusion());
                }
                else
                {
                    B.MakeEdge(ref Edge3);
                    B.Degenerated(Edge3, true);
                }
                V1f.Orientation(TopAbs_FORWARD);
                B.Add(Edge3, V1f);
                V2f.Orientation(TopAbs_REVERSED);
                B.Add(Edge3, V2f);
                B.Range(Edge3, f2, l2);
                if (Closed)
                {
                    Couture = Edge3;
                }
                Edge3.Reverse();
                thefirst = false;
            }
            else
            {
                Edge3 = Edge4;
                Edge3.Reverse();
            }

            if (Closed && !ex1.More() && !ex2.More())
            {
                Edge4 = Couture;
            }
            else
            {
                Iso = Surf.UIso(l1);
                //      Tol = std::max(BT.Tolerance(V1l), BT.Tolerance(V2l));
                Tol = Max(BRep_Tool.Tolerance(V1l), BRep_Tool.Tolerance(V2l));
                if (Iso.Value(l2).Distance(Iso.Value(f2)) > Tol)
                {
                    B.MakeEdge(ref Edge4, Iso, Precision.Confusion());
                }
                else
                {
                    B.MakeEdge(ref Edge4);
                    B.Degenerated(Edge4, true);
                }
                V1l.Orientation(TopAbs_FORWARD);
                B.Add(Edge4, V1l);
                V2l.Orientation(TopAbs_REVERSED);
                B.Add(Edge4, V2l);
                B.Range(Edge4, f2, l2);
            }

            // make the wire

            TopoDS_Wire W = new();
            B.MakeWire(ref W);

            B.Add(W, Edge1);
            B.Add(W, Edge4);
            B.Add(W, Edge2.Reversed());
            B.Add(W, Edge3);
            W.Closed(true);

            B.Add(Face, W);

            if (SameLoc)
            {
                Face.Move(L);
            }

            B.Add(Shell, Face);

            // set the pcurves

            double T = Precision.Confusion();

            if (Edge1.Orientation() == TopAbs_REVERSED)
            {
                B.UpdateEdge(Edge1, new Geom2d_Line(new gp_Pnt2d(0, f2), new gp_Dir2d(gp_Dir2d.D.NX)), Face, T);
                B.Range(Edge1, Face, -l1, -f1);
            }
            else
            {
                B.UpdateEdge(Edge1, new Geom2d_Line(new gp_Pnt2d(0, f2), new gp_Dir2d(gp_Dir2d.D.X)), Face, T);
                B.Range(Edge1, Face, f1, l1);
            }

            if (Edge2.Orientation() == TopAbs_REVERSED)
            {
                B.UpdateEdge(Edge2, new Geom2d_Line(new gp_Pnt2d(0, l2), new gp_Dir2d(gp_Dir2d.D.NX)), Face, T);
                B.Range(Edge2, Face, -l1, -f1);
            }
            else
            {
                B.UpdateEdge(Edge2, new Geom2d_Line(new gp_Pnt2d(0, l2), new gp_Dir2d(gp_Dir2d.D.X)), Face, T);
                B.Range(Edge2, Face, f1, l1);
            }

            if (Periodic)
            {
                B.UpdateEdge(Edge3,
                             new Geom2d_Line(new gp_Pnt2d(l1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                             new Geom2d_Line(new gp_Pnt2d(f1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                             Face,
                             T);
            }
            else
            {
                B.UpdateEdge(Edge3, new Geom2d_Line(new gp_Pnt2d(f1, 0), new gp_Dir2d(gp_Dir2d.D.Y)), Face, T);
                B.UpdateEdge(Edge4, new Geom2d_Line(new gp_Pnt2d(l1, 0), new gp_Dir2d(gp_Dir2d.D.Y)), Face, T);
            }

            // Set the non parameter flag;
            B.SameParameter(Edge1, false);
            B.SameParameter(Edge2, false);
            B.SameParameter(Edge3, false);
            B.SameParameter(Edge4, false);
            B.SameRange(Edge1, false);
            B.SameRange(Edge2, false);
            B.SameRange(Edge3, false);
            B.SameRange(Edge4, false);
        }

        Shell.Closed(BRep_Tool.IsClosed(Shell));
        BRepLib.SameParameter(Shell);
        return Shell;
    }

    //=================================================================================================

    /// <summary>
    /// The axis of <paramref name="Profile"/> on the planar <paramref name="Spine"/> (a planar wire or face): its
    /// origin is the point of the spine nearest to the profile's vertices, its main direction the spine's normal and
    /// its X direction the spine's tangent there. <paramref name="ProfOnSpine"/> is true when the profile touches the
    /// spine within <paramref name="Tol"/>.
    /// </summary>
    /// <exception cref="Standard_NoSuchObject">The spine is a face that isn't planar.</exception>
    /// <exception cref="Standard_DomainError">No plane was found.</exception>
    public static void Axe(in TopoDS_Shape Spine, in TopoDS_Wire Profile, out gp_Ax3 AxeProf, out bool ProfOnSpine, double Tol)
    {
        gp_Pnt Loc = new(), Loc1, Loc2;
        gp_Vec Tang = new(), Tang1, Tang2, Normal;

        Geom_Surface? S = null;
        TopLoc_Location L = new();

        TopoDS_Face aFace = new();

        // normal to the Spine.
        if (Spine.ShapeType() == TopAbs_FACE)
        {
            aFace = TopoDS.Face(Spine);
            S = BRep_Tool.Surface(TopoDS.Face(Spine), out L);
            if (!(S is Geom_Plane))
            {
                BRepLib_FindSurface FS = new(TopoDS.Face(Spine), -1, true);
                if (FS.Found())
                {
                    S = FS.Surface();
                    L = FS.Location();
                }
                else
                {
                    throw new Standard_NoSuchObject("BRepFill_Evolved : The Face is not planar");
                }
            }
        }
        else if (Spine.ShapeType() == TopAbs_WIRE)
        {
            aFace = new BRepLib_MakeFace(TopoDS.Wire(Spine), true).InternalFace();
            S = BRep_Tool.Surface(aFace, out L);
        }

        if (S is null)
        {
            throw new Standard_DomainError("BRepFill_Evolved::Axe");
        }

        if (!L.IsIdentity())
        {
            S = (Geom_Surface)S.Transformed(L.Transformation());
        }

        Normal = new gp_Vec(((Geom_Plane)S).Pln().Axis().Direction());

        // Find vertex of the profile closest to the spine.
        double DistMin = Precision.Infinite();
        double Dist;
        //  double     Tol2 = Tol*Tol;
        double Tol2 = 1.0e-10;
        TopExp_Explorer PE = new(), SE = new();
        BRepExtrema_ExtPC BE = new();
        double Par = 0.0;
        //  double     D1,D2;
        gp_Pnt P1, P2;

        // First check if there is contact Vertex Vertex.
        bool IsOnVertex = false;
        SE.Init(aFace.Oriented(TopAbs_FORWARD), TopAbs_VERTEX);
        //  modified by NIZHNY-EAP Wed Feb 23 12:31:52 2000 ___BEGIN___
        //  for (;SE.More() && !IsOnVertex ; SE.Next()) {
        for (; SE.More(); SE.Next())
        {
            P1 = BRep_Tool.Pnt(TopoDS.Vertex(SE.Current()));

            PE.Init(Profile, TopAbs_VERTEX);
            for (; PE.More(); PE.Next())
            {
                P2 = BRep_Tool.Pnt(TopoDS.Vertex(PE.Current()));
                double DistP1P2 = P1.SquareDistance(P2);
                IsOnVertex = (DistP1P2 <= Tol2);
                if (IsOnVertex)
                {
                    break;
                }
            }
            // otherwise SE.Next() is done and VonF is wrong
            if (IsOnVertex)
            {
                break;
            }
            //  modified by NIZHNY-EAP Wed Jan 26 09:08:36 2000 ___END___
        }

        if (IsOnVertex)
        {
            // try to find on which edge which shared this vertex,
            // the profile must be considered.
            // E1, E2 : those two edges.
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(aFace.Oriented(TopAbs_FORWARD), TopAbs_VERTEX, TopAbs_EDGE, Map);

            TopoDS_Vertex VonF = TopoDS.Vertex(SE.Current());
            NCollection_List<TopoDS_Shape> List = Map.FindFromKey(VonF);
            TopoDS_Edge E1 = TopoDS.Edge(List.First());
            TopoDS_Edge E2 = TopoDS.Edge(List.Last());

            Geom_Curve CE1 = BRep_Tool.Curve(E1, out L, out _, out _)!;
            double Par1 = BRep_Tool.Parameter(VonF, E1, aFace);
            CE1.D1(Par1, out Loc1, out Tang1);
            if (!L.IsIdentity())
            {
                Tang1.Transform(L.Transformation());
                Loc1.Transform(L.Transformation());
            }
            if (E1.Orientation() == TopAbs_REVERSED)
            {
                Tang1.Reverse();
            }

            Geom_Curve CE2 = BRep_Tool.Curve(E2, out L, out _, out _)!;
            double Par2 = BRep_Tool.Parameter(VonF, E2, aFace);
            CE2.D1(Par2, out Loc2, out Tang2);
            if (!L.IsIdentity())
            {
                Tang2.Transform(L.Transformation());
                Loc2.Transform(L.Transformation());
            }
            if (E2.Orientation() == TopAbs_REVERSED)
            {
                Tang2.Reverse();
            }

            //  modified by NIZHNY-EAP Wed Feb  2 15:38:41 2000 ___BEGIN___
            Tang1.Normalize();
            Tang2.Normalize();
            double sca1 = 0.0, sca2 = 0.0;
            TopoDS_Edge E;
            for (PE.Init(Profile, TopAbs_EDGE); PE.More(); PE.Next())
            {
                E = TopoDS.Edge(PE.Current());
                TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                P1 = BRep_Tool.Pnt(V1);
                P2 = BRep_Tool.Pnt(V2);
                gp_Vec vec = new(P1, P2);
                sca1 += Math.Abs(Tang1.Dot(vec));
                sca2 += Math.Abs(Tang2.Dot(vec));
            }
            //  modified by NIZHNY-EAP Wed Feb  2 15:38:44 2000 ___END___

            if (Math.Abs(sca1) < Math.Abs(sca2))
            {
                Loc = Loc1;
                Tang = Tang1;
            }
            else
            {
                Loc = Loc2;
                Tang = Tang2;
            }
            DistMin = 0.0;
        }
        else
        {
            SE.Init(aFace.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
            for (; SE.More(); SE.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(SE.Current());
                BE.Initialize(E);
                for (PE.Init(Profile, TopAbs_VERTEX); PE.More(); PE.Next())
                {
                    Dist = Precision.Infinite();
                    TopoDS_Vertex V = TopoDS.Vertex(PE.Current());
                    BE.Perform(V);
                    if (BE.IsDone())
                    {
                        // extrema.
                        for (int i = 1; i <= BE.NbExt(); i++)
                        {
                            if (BE.IsMin(i))
                            {
                                Dist = Math.Sqrt(BE.SquareDistance(i));
                                Par = BE.Parameter(i);
                                break;
                            }
                        }
                    }
                    // save minimum.
                    if (Dist < DistMin)
                    {
                        DistMin = Dist;
                        BRepAdaptor_Curve BAC = new(E);
                        BAC.D1(Par, out Loc, out Tang);
                        if (E.Orientation() == TopAbs_REVERSED)
                        {
                            Tang.Reverse();
                        }
                    }
                }
            }
        }

        ProfOnSpine = (DistMin < Tol);
        // Construction AxeProf;
        gp_Ax3 A3 = new(Loc, new gp_Dir(Normal), new gp_Dir(Tang));
        AxeProf = A3;
    }

    //=======================================================================
    // function : SearchOrigin
    // purpose  : Cut and orientate a closed wire.
    //=======================================================================

    /// <summary>
    /// Cuts the closed wire <paramref name="W"/> at the point of it nearest to <paramref name="P"/> (a vertex, or a
    /// new vertex splitting an edge) and orients it along <paramref name="Dir"/>.
    /// </summary>
    /// <exception cref="Standard_NoSuchObject">The wire isn't closed.</exception>
    internal static void SearchOrigin(ref TopoDS_Wire W, in gp_Pnt P, in gp_Vec Dir, double Tol)
    {
        if (!W.Closed())
        {
            throw new Standard_NoSuchObject("BRepFill::SearchOrigin : the wire must be closed");
        }

        bool NewVertex = false;
        double theparam = 1.0e101, angle;
        TopoDS_Vertex V = new();
        TopoDS_Edge E = new(), Eref = new();
        BRep_Builder B = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool BT;

        W.Orientation(TopAbs_FORWARD); // to avoid composing the orientations

        // Calculate the distance
        B.MakeVertex(ref V, P, Tol);
        BRepExtrema_DistShapeShape DSS = new(V, W);
        if (DSS.IsDone())
        {
            int isol = 1;
            double dss = P.Distance(DSS.PointOnShape2(isol));
            for (int iss = 2; iss <= DSS.NbSolution(); iss++)
            {
                if (dss > P.Distance(DSS.PointOnShape2(iss)))
                {
                    dss = P.Distance(DSS.PointOnShape2(iss));
                    isol = iss;
                }
            }
            TopoDS_Shape supp = DSS.SupportOnShape2(isol);
            if (DSS.SupportTypeShape2(isol) == BRepExtrema_IsVertex)
            {
                V = TopoDS.Vertex(supp);
            }
            else
            {
                double d, dist;
                E = TopoDS.Edge(supp);
                TopExp.Vertices(E, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
                //      dist = P.Distance(BT.Pnt(Vf));
                dist = P.Distance(BRep_Tool.Pnt(Vf));
                if (dist < Tol)
                {
                    V = Vl;
                }
                //      d = P.Distance(BT.Pnt(Vl));
                d = P.Distance(BRep_Tool.Pnt(Vl));
                if ((d < Tol) && (d < dist))
                {
                    V = Vf;
                    dist = d;
                }
                NewVertex = (dist > Tol);
                if (NewVertex)
                {
                    DSS.ParOnEdgeS2(isol, out theparam);
                }
            }
        }

        int ii, rangdeb = 0, NbEdges = 0;
        bool forward;
        BRepTools_WireExplorer exp = new();

        // Calculate the number of edges
        for (exp.Init(W); exp.More(); exp.Next())
        {
            NbEdges++;
        }
        if (NewVertex)
        {
            NbEdges++;
            Eref = E;
        }

        // Construct the Table and calculate rangdeb
        NCollection_Array1<TopoDS_Shape> Edges = new(1, NbEdges);
        for (exp.Init(W), ii = 1; exp.More(); exp.Next(), ii++)
        {
            E = exp.Current();
            if (NewVertex && E.IsSame(Eref))
            {
                CutEdge(E, theparam, out TopoDS_Edge E1, out TopoDS_Edge E2, exp.CurrentVertex());
                Edges[ii] = E1;
                ii++;
                Edges[ii] = E2;
                rangdeb = ii;
            }
            else
            {
                Edges[ii] = E;
            }
            if (!NewVertex && V.IsSame(exp.CurrentVertex()))
            {
                rangdeb = ii;
            }
        }
        if (rangdeb == 0)
        {
            rangdeb = NbEdges;
        }

        // Calculate the direction of parsing
        E = TopoDS.Edge(Edges[rangdeb]);
        if (!NewVertex)
        {
            //    theparam = BT.Parameter(V, E);
            theparam = BRep_Tool.Parameter(V, E);
        }
        BRepAdaptor_Curve AC = new(E);
        AC.D1(theparam, out gp_Pnt _, out gp_Vec Ve);
        if (E.Orientation() == TopAbs_REVERSED)
        {
            Ve *= -1;
        }
        angle = Ve.Angle(Dir);
        if (angle > M_PI)
        {
            angle = 2 * M_PI - angle;
        }
        forward = (angle <= M_PI / 2);

        // Reconstruction
        MakeWire(Edges, rangdeb, forward, ref W);
        W.Closed(true);
    }

    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="ACR"/> (indexed from 0) with the reduced curvilinear abscissae of the ends of the edges of
    /// <paramref name="wire"/>, in the order of a wire explorer: ACR(i) is the length up to the end of edge i over the
    /// wire's length, and ACR(0) the wire's length (ACR(n) = 1 for a wire of no length).
    /// </summary>
    public static void ComputeACR(in TopoDS_Wire wire, NCollection_Array1<double> ACR)
    {
        // calculate the reduced curvilinear abscisses and the length of the wire
        BRepTools_WireExplorer anExp = new();
        int nbEdges = 0, i;

        // cumulated lengths
        ACR.Init(0);
        for (anExp.Init(wire); anExp.More(); anExp.Next())
        {
            nbEdges++;
            TopoDS_Edge Ecur = TopoDS.Edge(anExp.Current());
            ACR[nbEdges] = ACR[nbEdges - 1];
            if (!BRep_Tool.Degenerated(Ecur))
            {
                BRepAdaptor_Curve anEcur = new(Ecur);
                ACR[nbEdges] += GCPnts_AbscissaPoint.Length(anEcur);
            }
        }

        // total length of the wire
        ACR[0] = ACR[nbEdges];

        // reduced curvilinear abscisses
        if (ACR[0] > Precision.Confusion())
        {
            for (i = 1; i <= nbEdges; i++)
            {
                ACR[i] /= ACR[0];
            }
        }
        else
        {
            // punctual wire
            ACR[nbEdges] = 1;
        }
    }

    //=================================================================================================

    /// <summary>
    /// <paramref name="wire"/> with its edges cut at the reduced curvilinear abscissae <paramref name="ACRcuts"/>
    /// (those more than <paramref name="prec"/> from an edge's ends); frozen.
    /// </summary>
    public static TopoDS_Wire InsertACR(in TopoDS_Wire wire, NCollection_Array1<double> ACRcuts, double prec)
    {
        TopoDS_Wire aResult = InternalInsertACR(wire, ACRcuts, prec);
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary><see cref="InsertACR"/> without freezing the result, for ported callers that build on it.</summary>
    internal static TopoDS_Wire InternalInsertACR(in TopoDS_Wire wire, NCollection_Array1<double> ACRcuts, double prec)
    {
        // calculate ACR of the wire to be cut
        BRepTools_WireExplorer anExp = new();
        int nbEdges = 0;
        for (anExp.Init(wire); anExp.More(); anExp.Next())
        {
            nbEdges++;
        }
        NCollection_Array1<double> ACRwire = new(0, nbEdges);
        ComputeACR(wire, ACRwire);

        int i, j, nmax = ACRcuts.Length();
        NCollection_Array1<double> paradec = new(1, nmax);
        BRepLib_MakeWire MW = new();

        double t0, t1 = 0;
        nbEdges = 0;

        // processing edge by edge
        for (anExp.Init(wire); anExp.More(); anExp.Next())
        {
            nbEdges++;
            t0 = t1;
            t1 = ACRwire[nbEdges];

            // parameters of cut on this edge
            int ndec = 0;
            for (i = 1; i <= ACRcuts.Length(); i++)
            {
                if (t0 + prec < ACRcuts[i] && ACRcuts[i] < t1 - prec)
                {
                    ndec++;
                    paradec[ndec] = ACRcuts[i];
                }
            }

            TopoDS_Edge E = anExp.Current();
            TopoDS_Vertex V = anExp.CurrentVertex();

            if (ndec == 0 || BRep_Tool.Degenerated(E))
            {
                // copy the edge
                MW.Add(E);
            }
            else
            {
                // it is necessary to cut the edge
                // following the direction of parsing of the wire
                bool SO = (V.IsSame(TopExp.FirstVertex(E)));
                NCollection_Sequence<TopoDS_Shape> SE = new();
                SE.Clear();
                NCollection_Sequence<double> SR = new();
                SR.Clear();
                // the wire is always FORWARD
                // it is necessary to modify the parameter of cut6 if the edge is REVERSED
                if (E.Orientation() == TopAbs_FORWARD)
                {
                    for (j = 1; j <= ndec; j++)
                    {
                        SR.Append(paradec[j]);
                    }
                }
                else
                {
                    for (j = 1; j <= ndec; j++)
                    {
                        SR.Append(t0 + t1 - paradec[ndec + 1 - j]);
                    }
                }
                TrimEdge(E, SR, t0, t1, SO, SE);
                for (j = 1; j <= SE.Length(); j++)
                {
                    MW.Add(TopoDS.Edge(SE.Value(j)));
                }
            }
        }

        // result
        TopAbs_Orientation Orien = wire.Orientation();
        TopoDS_Shape aLocalShape = MW.InternalWire();
        aLocalShape.Orientation(Orien);
        TopoDS_Wire wres = TopoDS.Wire(aLocalShape);
        //  TopoDS_Wire wres = TopoDS::Wire(MW.Wire().Oriented(Orien));
        return wres;
    }
}
