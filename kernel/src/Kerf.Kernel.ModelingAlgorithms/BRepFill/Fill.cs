// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Measurement.Distance.SupportType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Ruled faces and shells between edges and wires, and tools on wires used by the sweeps and lofts: the axis of a
/// profile on a planar spine, the reduced curvilinear abscissae of a wire's vertices and the cutting of a wire at
/// given abscissae.
/// </summary>
/// <remarks>
/// <para>
/// <see cref="Face"/> and <see cref="Shell"/> put the given edges into the new face (or shell) and give them pcurves
/// on it, as OCCT does: OCCT writes into the caller's edges. The port writes into copies of them when the inputs hold
/// frozen shapes (decision 897): the result then holds changed copies of the edges and the caller's own vertices
/// wherever nothing changed them. Unfrozen inputs, which only ported code holds, are used in place, as in OCCT.
/// </para>
/// <para>The results handed out are frozen; ported callers that keep building use the <c>Internal</c> forms.</para>
/// </remarks>
internal static class Fill
{
    private static void MakeWire(NCollection_Array1<Shape> edges, int rangdeb, bool forward, ref Wire newwire)
    {
        BrepBuilder bw = new();
        int rang, nbEdges = edges.Length();
        bw.MakeWire(ref newwire);
        if (forward)
        {
            for (rang = rangdeb; rang <= nbEdges; rang++)
            {
                bw.Add(newwire, ShapeCasts.Edge(edges[rang]));
            }
            for (rang = 1; rang < rangdeb; rang++)
            {
                bw.Add(newwire, ShapeCasts.Edge(edges[rang]));
            }
        }

        else
        {
            Edge e;
            for (rang = rangdeb; rang >= 1; rang--)
            {
                e = ShapeCasts.Edge(edges[rang]);
                bw.Add(newwire, e.Reversed());
            }
            for (rang = nbEdges; rang > rangdeb; rang--)
            {
                e = ShapeCasts.Edge(edges[rang]);
                bw.Add(newwire, e.Reversed());
            }
        }
        newwire.Orient(Forward);
        newwire.Closed(true);
    }

    private static void CutEdge(in Edge currentEdge, double param, out Edge e1, out Edge e2,
                                in Vertex vRef)
    {
        BrepBuilder b = new();
        Curve3D c = Tool.Curve(currentEdge, out double first, out double last)!;
        Vertex vi = new();
        b.MakeVertex(ref vi, c.Value(param), global::Kerf.Engine.Foundation.Precision.Confusion());
        Exploration.Vertices(currentEdge, out Vertex vf, out Vertex vl);
        if (vRef.IsSame(vf))
        {
            e1 = new LibraryMakeEdge(c, vf, vi, first, param).InternalEdge();
            e2 = new LibraryMakeEdge(c, vi, vl, param, last).InternalEdge();
        }
        else
        {
            e2 = new LibraryMakeEdge(c, vf, vi, first, param).InternalEdge();
            e1 = new LibraryMakeEdge(c, vi, vl, param, last).InternalEdge();
        }
    }

    private static void TrimEdge(in Edge currentEdge,
                                 NCollection_Sequence<double> cutValues,
                                 double t0,
                                 double t1,
                                 bool seqOrder,
                                 NCollection_Sequence<Shape> s)

    {
        s.Clear();
        int j, ndec = cutValues.Length();
        double m0, m1;
        Curve3D c = Tool.Curve(currentEdge, out double first, out double last)!;

        Vertex vbid = new(), v0, v1;
        Orientation currentOrient = currentEdge.Orientation;
        Exploration.Vertices(currentEdge, out Vertex vf, out Vertex vl);
        vbid.Nullify();

        if (seqOrder)
        {
            // from first to last
            m0 = first;
            v0 = vf;
            for (j = 1; j <= ndec; j++)
            {
                // piece of edge
                m1 = (cutValues.Value(j) - t0) * (last - first) / (t1 - t0) + first;
                Edge cutE = new LibraryMakeEdge(c, v0, vbid, m0, m1).InternalEdge();
                cutE.Orient(currentOrient);
                s.Append(cutE);
                m0 = m1;
                v0 = Exploration.LastVertex(cutE);
                if (j == ndec)
                {
                    // last piece
                    Edge lastE = new LibraryMakeEdge(c, v0, vl, m0, last).InternalEdge();
                    lastE.Orient(currentOrient);
                    s.Append(lastE);
                }
            }
        }
        else
        {
            // from last to first
            m1 = last;
            v1 = vl;
            for (j = ndec; j >= 1; j--)
            {
                // piece of edge
                m0 = (cutValues.Value(j) - t0) * (last - first) / (t1 - t0) + first;
                Edge cutE = new LibraryMakeEdge(c, vbid, v1, m0, m1).InternalEdge();
                cutE.Orient(currentOrient);
                s.Append(cutE);
                m1 = m0;
                v1 = Exploration.FirstVertex(cutE);
                if (j == 1)
                {
                    // last piece
                    Edge lastE = new LibraryMakeEdge(c, vf, v1, first, m1).InternalEdge();
                    lastE.Orient(currentOrient);
                    s.Append(lastE);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>
    /// A ruled face between <paramref name="edge1"/> and <paramref name="edge2"/>, bounded by them and by the rulings
    /// through their ends (degenerated edges where those ends meet); frozen.
    /// </summary>
    public static Face Face(in Edge edge1, in Edge edge2)
    {
        Face result = InternalFace(edge1, edge2);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary><see cref="Face"/> without freezing the result, for ported callers that build on it.</summary>
    internal static Face InternalFace(in Edge edge1, in Edge edge2)
    {
        // Kerf: OCCT writes into the edges (pcurves on the face, vertex parameters): in a working overlay (unit 10H3),
        // so the face holds the caller's edges and vertices wherever it left them unchanged; inside an operation's
        // overlay, the writes are that operation's
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(new Shape());
        Face face = FaceCore(edge1, edge2);
        return overlay is null ? face : ShapeCasts.Face(overlay.Finish(face));
    }

    private static Face FaceCore(Edge edge1, Edge edge2)
    {
        Face face = new();

        BrepBuilder b = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool BT;

        Placement l = new();
        double tol;

        Curve3D c1 = Tool.Curve(edge1, out Placement L1, out double f1, out double l1)!;
        Curve3D c2 = Tool.Curve(edge2, out Placement L2, out double f2, out double l2)!;

        // compute the location
        bool sameLoc = false;
        if (L1.IsEqual(L2))
        {
            l = L1;
            L1 = L2 = new Placement();
            sameLoc = true;
        }

        // transform and trim the curves

        Vertex v1f, v1l, v2f, v2l;

        // create a new Handle
        if (Math.Abs(f1 - c1.FirstParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(l1 - c1.LastParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            c1 = new TrimmedCurve3D(c1, f1, l1);
        }
        else
        {
            c1 = (Curve3D)c1.Copy();
        }
        // eventually the curve is concerned
        if (!sameLoc)
        {
            c1.Transform(L1.Transformation);
        }
        // it is set in the proper direction and its vertices are taken
        if (edge1.Orientation == Reversed)
        {
            Exploration.Vertices(edge1, out v1l, out v1f);
            c1.Reverse();
        }
        else
        {
            Exploration.Vertices(edge1, out v1f, out v1l);
        }

        // a new Handle is created
        if (Math.Abs(f2 - c2.FirstParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(l2 - c2.LastParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            c2 = new TrimmedCurve3D(c2, f2, l2);
        }
        else
        {
            c2 = (Curve3D)c2.Copy();
        }
        // eventually the curve is concerned
        if (!sameLoc)
        {
            c2.Transform(L2.Transformation);
        }
        // it is set in the proper direction and its vertices are taken
        if (edge2.Orientation == Reversed)
        {
            Exploration.Vertices(edge2, out v2l, out v2f);
            c2.Reverse();
        }
        else
        {
            Exploration.Vertices(edge2, out v2f, out v2l);
        }

        // Are they closed edges?
        bool closed = v1f.IsSame(v1l) && v2f.IsSame(v2l);

        SurfaceGenerator generator = new();
        generator.AddCurve(c1);
        generator.AddCurve(c2);
        generator.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());

        Surface surf = generator.InternalSurface()!;
        Curve3D iso;

        b.MakeFace(ref face, surf, global::Kerf.Engine.Foundation.Precision.Confusion());

        // make the missing edges
        surf.ParameterBounds(out f1, out l1, out f2, out l2);

        Edge edge3 = new(), edge4 = new();

        iso = surf.UIso(f1);
        tol = Max(Tool.Tolerance(v1f), Tool.Tolerance(v2f));
        if (iso.Value(f2).DistanceTo(iso.Value(l2)) > tol)
        {
            b.MakeEdge(ref edge3, iso, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            b.MakeEdge(ref edge3);
            b.Degenerated(edge3, true);
        }
        v1f.Orient(Forward);
        b.Add(edge3, v1f);
        v2f.Orient(Reversed);
        b.Add(edge3, v2f);
        b.Range(edge3, f2, l2);

        if (closed)
        {
            edge4 = edge3;
        }
        else
        {
            iso = surf.UIso(l1);
            tol = Max(Tool.Tolerance(v1l), Tool.Tolerance(v2l));
            if (iso.Value(l2).DistanceTo(iso.Value(f2)) > tol)
            {
                b.MakeEdge(ref edge4, iso, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                b.MakeEdge(ref edge4);
                b.Degenerated(edge4, true);
            }
            v1l.Orient(Forward);
            b.Add(edge4, v1l);
            v2l.Orient(Reversed);
            b.Add(edge4, v2l);
            b.Range(edge4, f2, l2);
        }

        // make the wire

        Wire w = new();
        b.MakeWire(ref w);

        edge3.Reverse();
        b.Add(w, edge1);
        b.Add(w, edge4);
        b.Add(w, edge2.Reversed());
        b.Add(w, edge3);
        w.Closed(true);

        b.Add(face, w);

        // set the pcurves

        double t = global::Kerf.Engine.Foundation.Precision.Confusion();

        if (edge1.Orientation == Reversed)
        {
            b.UpdateEdge(edge1, new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.NX)), face, t);
            b.Range(edge1, face, -l1, -f1);
        }
        else
        {
            b.UpdateEdge(edge1, new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X)), face, t);
            b.Range(edge1, face, f1, l1);
        }

        if (edge2.Orientation == Reversed)
        {
            b.UpdateEdge(edge2, new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.NX)), face, t);
            b.Range(edge2, face, -l1, -f1);
        }
        else
        {
            b.UpdateEdge(edge2, new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X)), face, t);
            b.Range(edge2, face, f1, l1);
        }

        if (closed)
        {
            b.UpdateEdge(edge3,
                         new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)),
                         new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)),
                         face,
                         t);
        }
        else
        {
            b.UpdateEdge(edge3, new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)), face, t);
            b.UpdateEdge(edge4, new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)), face, t);
        }

        // Set the non parameter flag;
        b.SameParameter(edge1, false);
        b.SameParameter(edge2, false);
        b.SameParameter(edge3, false);
        b.SameParameter(edge4, false);
        b.SameRange(edge1, false);
        b.SameRange(edge2, false);
        b.SameRange(edge3, false);
        b.SameRange(edge4, false);

        BrepLibrary.SameParameter(face);

        if (sameLoc)
        {
            face.Move(l);
        }
        return face;
    }

    //=================================================================================================

    /// <summary>
    /// The ruled faces between the edges of <paramref name="wire1"/> and <paramref name="wire2"/>, taken pairwise in
    /// the order of their explorers (the wires should have the same number of edges), sewn into a shell; frozen.
    /// </summary>
    public static Shell Shell(in Wire wire1, in Wire wire2)
    {
        Shell result = InternalShell(wire1, wire2);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary><see cref="Shell"/> without freezing the result, for ported callers that build on it.</summary>
    internal static Shell InternalShell(in Wire wire1, in Wire wire2)
    {
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(new Shape()); // Kerf: as InternalFace
        Shell shell = ShellCore(wire1, wire2);
        return overlay is null ? shell : ShapeCasts.Shell(overlay.Finish(shell));
    }

    private static Shell ShellCore(in Wire wire1, in Wire wire2)
    {
        Shell shell = new();
        Face face = new();
        Edge edge1, edge2, edge3 = new(), edge4 = new(), couture = new();

        BrepBuilder b = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool BT;
        b.MakeShell(ref shell);

        Explorer ex1 = new();
        Explorer ex2 = new();

        bool closed = wire1.ClosedFlag && wire2.ClosedFlag;

        bool thefirst = true;

        ex1.Init(wire1, ShapeType.Edge);
        ex2.Init(wire2, ShapeType.Edge);

        while (ex1.More() && ex2.More())
        {

            edge1 = ShapeCasts.Edge(ex1.Current());
            edge2 = ShapeCasts.Edge(ex2.Current());

            bool periodic = Tool.IsClosed(edge1) && Tool.IsClosed(edge2);

            ex1.Next();
            ex2.Next();

            Placement l = new();
            double tol;

            Curve3D c1 = Tool.Curve(edge1, out Placement L1, out double f1, out double l1)!;
            Curve3D c2 = Tool.Curve(edge2, out Placement L2, out double f2, out double l2)!;

            // compute the location
            bool sameLoc = false;
            if (L1.IsEqual(L2))
            {
                l = L1;
                L1 = L2 = new Placement();
                sameLoc = true;
            }

            // transform and trim the curves

            Vertex v1f, v1l, v2f, v2l;

            if (Math.Abs(f1 - c1.FirstParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                || Math.Abs(l1 - c1.LastParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                c1 = new TrimmedCurve3D(c1, f1, l1);
            }
            else
            {
                c1 = (Curve3D)c1.Copy();
            }
            if (!sameLoc)
            {
                c1.Transform(L1.Transformation);
            }
            if (edge1.Orientation == Reversed)
            {
                Exploration.Vertices(edge1, out v1l, out v1f);
                c1.Reverse();
            }
            else
            {
                Exploration.Vertices(edge1, out v1f, out v1l);
            }

            if (Math.Abs(f2 - c2.FirstParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                || Math.Abs(l2 - c2.LastParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                c2 = new TrimmedCurve3D(c2, f2, l2);
            }
            else
            {
                c2 = (Curve3D)c2.Copy();
            }
            if (!sameLoc)
            {
                c2.Transform(L2.Transformation);
            }
            if (edge2.Orientation == Reversed)
            {
                Exploration.Vertices(edge2, out v2l, out v2f);
                c2.Reverse();
            }
            else
            {
                Exploration.Vertices(edge2, out v2f, out v2l);
            }

            SurfaceGenerator generator = new();
            generator.AddCurve(c1);
            generator.AddCurve(c2);
            generator.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());

            Surface surf = generator.InternalSurface()!;
            Curve3D iso;

            b.MakeFace(ref face, surf, global::Kerf.Engine.Foundation.Precision.Confusion());

            // make the missing edges
            surf.ParameterBounds(out f1, out l1, out f2, out l2);

            if (thefirst)
            {
                iso = surf.UIso(f1);
                //      Tol = std::max(BT.Tolerance(V1f), BT.Tolerance(V2f));
                tol = Max(Tool.Tolerance(v1f), Tool.Tolerance(v2f));
                if (iso.Value(f2).DistanceTo(iso.Value(l2)) > tol)
                {
                    b.MakeEdge(ref edge3, iso, global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                else
                {
                    b.MakeEdge(ref edge3);
                    b.Degenerated(edge3, true);
                }
                v1f.Orient(Forward);
                b.Add(edge3, v1f);
                v2f.Orient(Reversed);
                b.Add(edge3, v2f);
                b.Range(edge3, f2, l2);
                if (closed)
                {
                    couture = edge3;
                }
                edge3.Reverse();
                thefirst = false;
            }
            else
            {
                edge3 = edge4;
                edge3.Reverse();
            }

            if (closed && !ex1.More() && !ex2.More())
            {
                edge4 = couture;
            }
            else
            {
                iso = surf.UIso(l1);
                //      Tol = std::max(BT.Tolerance(V1l), BT.Tolerance(V2l));
                tol = Max(Tool.Tolerance(v1l), Tool.Tolerance(v2l));
                if (iso.Value(l2).DistanceTo(iso.Value(f2)) > tol)
                {
                    b.MakeEdge(ref edge4, iso, global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                else
                {
                    b.MakeEdge(ref edge4);
                    b.Degenerated(edge4, true);
                }
                v1l.Orient(Forward);
                b.Add(edge4, v1l);
                v2l.Orient(Reversed);
                b.Add(edge4, v2l);
                b.Range(edge4, f2, l2);
            }

            // make the wire

            Wire w = new();
            b.MakeWire(ref w);

            b.Add(w, edge1);
            b.Add(w, edge4);
            b.Add(w, edge2.Reversed());
            b.Add(w, edge3);
            w.Closed(true);

            b.Add(face, w);

            if (sameLoc)
            {
                face.Move(l);
            }

            b.Add(shell, face);

            // set the pcurves

            double t = global::Kerf.Engine.Foundation.Precision.Confusion();

            if (edge1.Orientation == Reversed)
            {
                b.UpdateEdge(edge1, new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.NX)), face, t);
                b.Range(edge1, face, -l1, -f1);
            }
            else
            {
                b.UpdateEdge(edge1, new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X)), face, t);
                b.Range(edge1, face, f1, l1);
            }

            if (edge2.Orientation == Reversed)
            {
                b.UpdateEdge(edge2, new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.NX)), face, t);
                b.Range(edge2, face, -l1, -f1);
            }
            else
            {
                b.UpdateEdge(edge2, new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X)), face, t);
                b.Range(edge2, face, f1, l1);
            }

            if (periodic)
            {
                b.UpdateEdge(edge3,
                             new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)),
                             new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)),
                             face,
                             t);
            }
            else
            {
                b.UpdateEdge(edge3, new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)), face, t);
                b.UpdateEdge(edge4, new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)), face, t);
            }

            // Set the non parameter flag;
            b.SameParameter(edge1, false);
            b.SameParameter(edge2, false);
            b.SameParameter(edge3, false);
            b.SameParameter(edge4, false);
            b.SameRange(edge1, false);
            b.SameRange(edge2, false);
            b.SameRange(edge3, false);
            b.SameRange(edge4, false);
        }

        shell.Closed(Tool.IsClosed(shell));
        BrepLibrary.SameParameter(shell);
        return shell;
    }

    //=================================================================================================

    /// <summary>
    /// The axis of <paramref name="profile"/> on the planar <paramref name="spine"/> (a planar wire or face): its
    /// origin is the point of the spine nearest to the profile's vertices, its main direction the spine's normal and
    /// its X direction the spine's tangent there. <paramref name="profOnSpine"/> is true when the profile touches the
    /// spine within <paramref name="tol"/>.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The spine is a face that isn't planar.</exception>
    /// <exception cref="DomainErrorException">No plane was found.</exception>
    public static void Axe(in Shape spine, in Wire profile, out CoordinateSystem axeProf, out bool profOnSpine, double tol)
    {
        Point3D loc = new(), loc1, loc2;
        Vector3D tang = new(), tang1, tang2, normal;

        Surface? s = null;
        Placement l = new();

        Face face = new();

        // normal to the Spine.
        if (spine.Type == global::Kerf.Topology.ShapeType.Face)
        {
            face = ShapeCasts.Face(spine);
            s = Tool.Surface(ShapeCasts.Face(spine), out l);
            if (!(s is PlaneSurface))
            {
                FindSurface fs = new(ShapeCasts.Face(spine), -1, true);
                if (fs.Found())
                {
                    s = fs.Surface();
                    l = fs.Location();
                }
                else
                {
                    throw new NoSuchObjectException("BRepFill_Evolved : The Face is not planar");
                }
            }
        }
        else if (spine.Type == ShapeType.Wire)
        {
            face = new LibraryMakeFace(ShapeCasts.Wire(spine), true).InternalFace();
            s = Tool.Surface(face, out l);
        }

        if (s is null)
        {
            throw new DomainErrorException("BRepFill_Evolved::Axe");
        }

        if (!l.IsIdentity)
        {
            s = (Surface)s.Transformed(l.Transformation);
        }

        normal = new Vector3D(((PlaneSurface)s).Plane.Axis.Direction);

        // Find vertex of the profile closest to the spine.
        double distMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        double dist;
        //  double     Tol2 = Tol*Tol;
        double tol2 = 1.0e-10;
        Explorer pe = new(), se = new();
        ExtPC be = new();
        double par = 0.0;
        //  double     D1,D2;
        Point3D p1, p2;

        // First check if there is contact Vertex Vertex.
        bool isOnVertex = false;
        se.Init(face.Oriented(Forward), ShapeType.Vertex);
        //  modified by NIZHNY-EAP Wed Feb 23 12:31:52 2000 ___BEGIN___
        //  for (;SE.More() && !IsOnVertex ; SE.Next()) {
        for (; se.More(); se.Next())
        {
            p1 = Tool.Pnt(ShapeCasts.Vertex(se.Current()));

            pe.Init(profile, ShapeType.Vertex);
            for (; pe.More(); pe.Next())
            {
                p2 = Tool.Pnt(ShapeCasts.Vertex(pe.Current()));
                double distP1P2 = p1.SquareDistanceTo(p2);
                isOnVertex = (distP1P2 <= tol2);
                if (isOnVertex)
                {
                    break;
                }
            }
            // otherwise SE.Next() is done and VonF is wrong
            if (isOnVertex)
            {
                break;
            }
            //  modified by NIZHNY-EAP Wed Jan 26 09:08:36 2000 ___END___
        }

        if (isOnVertex)
        {
            // try to find on which edge which shared this vertex,
            // the profile must be considered.
            // E1, E2 : those two edges.
            IndexedMap<Shape, NCollection_List<Shape>> map = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(face.Oriented(Forward), ShapeType.Vertex, ShapeType.Edge, map);

            Vertex vonF = ShapeCasts.Vertex(se.Current());
            NCollection_List<Shape> list = map.GetValue(vonF);
            Edge e1 = ShapeCasts.Edge(list.First());
            Edge e2 = ShapeCasts.Edge(list.Last());

            Curve3D ce1 = Tool.Curve(e1, out l, out _, out _)!;
            double par1 = Tool.Parameter(vonF, e1, face);
            ce1.D1(par1, out loc1, out tang1);
            if (!l.IsIdentity)
            {
                tang1 = tang1.Transformed(l.Transformation);
                loc1 = loc1.Transformed(l.Transformation);
            }
            if (e1.Orientation == Reversed)
            {
                tang1 = tang1.Reversed();
            }

            Curve3D ce2 = Tool.Curve(e2, out l, out _, out _)!;
            double par2 = Tool.Parameter(vonF, e2, face);
            ce2.D1(par2, out loc2, out tang2);
            if (!l.IsIdentity)
            {
                tang2 = tang2.Transformed(l.Transformation);
                loc2 = loc2.Transformed(l.Transformation);
            }
            if (e2.Orientation == Reversed)
            {
                tang2 = tang2.Reversed();
            }

            //  modified by NIZHNY-EAP Wed Feb  2 15:38:41 2000 ___BEGIN___
            tang1 = tang1.Normalized();
            tang2 = tang2.Normalized();
            double sca1 = 0.0, sca2 = 0.0;
            Edge e;
            for (pe.Init(profile, ShapeType.Edge); pe.More(); pe.Next())
            {
                e = ShapeCasts.Edge(pe.Current());
                Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                p1 = Tool.Pnt(v1);
                p2 = Tool.Pnt(v2);
                Vector3D vec = new(p1, p2);
                sca1 += Math.Abs(tang1.Dot(vec));
                sca2 += Math.Abs(tang2.Dot(vec));
            }
            //  modified by NIZHNY-EAP Wed Feb  2 15:38:44 2000 ___END___

            if (Math.Abs(sca1) < Math.Abs(sca2))
            {
                loc = loc1;
                tang = tang1;
            }
            else
            {
                loc = loc2;
                tang = tang2;
            }
            distMin = 0.0;
        }
        else
        {
            se.Init(face.Oriented(Forward), ShapeType.Edge);
            for (; se.More(); se.Next())
            {
                Edge e = ShapeCasts.Edge(se.Current());
                be.Initialize(e);
                for (pe.Init(profile, ShapeType.Vertex); pe.More(); pe.Next())
                {
                    dist = global::Kerf.Engine.Foundation.Precision.Infinite();
                    Vertex v = ShapeCasts.Vertex(pe.Current());
                    be.Perform(v);
                    if (be.IsDone)
                    {
                        // extrema.
                        for (int i = 1; i <= be.ExtCount; i++)
                        {
                            if (be.IsMin(i))
                            {
                                dist = Math.Sqrt(be.SquareDistance(i));
                                par = be.Parameter(i);
                                break;
                            }
                        }
                    }
                    // save minimum.
                    if (dist < distMin)
                    {
                        distMin = dist;
                        EdgeCurveAdaptor bac = new(e);
                        bac.D1(par, out loc, out tang);
                        if (e.Orientation == Reversed)
                        {
                            tang = tang.Reversed();
                        }
                    }
                }
            }
        }

        profOnSpine = (distMin < tol);
        // Construction AxeProf;
        CoordinateSystem a3 = new(loc, new Direction3D(normal), new Direction3D(tang));
        axeProf = a3;
    }

    //=======================================================================
    // function : SearchOrigin
    // purpose  : Cut and orientate a closed wire.
    //=======================================================================

    /// <summary>
    /// Cuts the closed wire <paramref name="w"/> at the point of it nearest to <paramref name="p"/> (a vertex, or a
    /// new vertex splitting an edge) and orients it along <paramref name="dir"/>.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The wire isn't closed.</exception>
    internal static void SearchOrigin(ref Wire w, in Point3D p, in Vector3D dir, double tol)
    {
        if (!w.ClosedFlag)
        {
            throw new NoSuchObjectException("BRepFill::SearchOrigin : the wire must be closed");
        }

        bool newVertex = false;
        double theparam = 1.0e101, angle;
        Vertex v = new();
        Edge e = new(), eref = new();
        BrepBuilder b = new();
        // Class BRep_Tool without fields and without Constructor :
        //  BRep_Tool BT;

        w.Orient(Forward); // to avoid composing the orientations

        // Calculate the distance
        b.MakeVertex(ref v, p, tol);
        DistanceShapeShape DSS = new(v, w);
        if (DSS.IsDone)
        {
            int isol = 1;
            double dss = p.DistanceTo(DSS.PointOnShape2(isol));
            for (int iss = 2; iss <= DSS.SolutionCount; iss++)
            {
                if (dss > p.DistanceTo(DSS.PointOnShape2(iss)))
                {
                    dss = p.DistanceTo(DSS.PointOnShape2(iss));
                    isol = iss;
                }
            }
            Shape supp = DSS.SupportOnShape2(isol);
            if (DSS.SupportTypeShape2(isol) == SupportType.IsVertex)
            {
                v = ShapeCasts.Vertex(supp);
            }
            else
            {
                double d, dist;
                e = ShapeCasts.Edge(supp);
                Exploration.Vertices(e, out Vertex vf, out Vertex vl);
                //      dist = P.Distance(BT.Pnt(Vf));
                dist = p.DistanceTo(Tool.Pnt(vf));
                if (dist < tol)
                {
                    v = vl;
                }
                //      d = P.Distance(BT.Pnt(Vl));
                d = p.DistanceTo(Tool.Pnt(vl));
                if ((d < tol) && (d < dist))
                {
                    v = vf;
                    dist = d;
                }
                newVertex = (dist > tol);
                if (newVertex)
                {
                    DSS.ParOnEdgeS2(isol, out theparam);
                }
            }
        }

        int ii, rangdeb = 0, nbEdges = 0;
        bool forward;
        WireExplorer exp = new();

        // Calculate the number of edges
        for (exp.Init(w); exp.More(); exp.Next())
        {
            nbEdges++;
        }
        if (newVertex)
        {
            nbEdges++;
            eref = e;
        }

        // Construct the Table and calculate rangdeb
        NCollection_Array1<Shape> edges = new(1, nbEdges);
        for (exp.Init(w), ii = 1; exp.More(); exp.Next(), ii++)
        {
            e = exp.Current();
            if (newVertex && e.IsSame(eref))
            {
                CutEdge(e, theparam, out Edge e1, out Edge e2, exp.CurrentVertex());
                edges[ii] = e1;
                ii++;
                edges[ii] = e2;
                rangdeb = ii;
            }
            else
            {
                edges[ii] = e;
            }
            if (!newVertex && v.IsSame(exp.CurrentVertex()))
            {
                rangdeb = ii;
            }
        }
        if (rangdeb == 0)
        {
            rangdeb = nbEdges;
        }

        // Calculate the direction of parsing
        e = ShapeCasts.Edge(edges[rangdeb]);
        if (!newVertex)
        {
            //    theparam = BT.Parameter(V, E);
            theparam = Tool.Parameter(v, e);
        }
        EdgeCurveAdaptor ac = new(e);
        ac.D1(theparam, out Point3D _, out Vector3D ve);
        if (e.Orientation == Reversed)
        {
            ve *= -1;
        }
        angle = ve.AngleTo(dir);
        if (angle > M_PI)
        {
            angle = 2 * M_PI - angle;
        }
        forward = (angle <= M_PI / 2);

        // Reconstruction
        MakeWire(edges, rangdeb, forward, ref w);
        w.Closed(true);
    }

    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="acr"/> (indexed from 0) with the reduced curvilinear abscissae of the ends of the edges of
    /// <paramref name="wire"/>, in the order of a wire explorer: ACR(i) is the length up to the end of edge i over the
    /// wire's length, and ACR(0) the wire's length (ACR(n) = 1 for a wire of no length).
    /// </summary>
    public static void ComputeACR(in Wire wire, NCollection_Array1<double> acr)
    {
        // calculate the reduced curvilinear abscisses and the length of the wire
        WireExplorer exp = new();
        int nbEdges = 0, i;

        // cumulated lengths
        acr.Init(0);
        for (exp.Init(wire); exp.More(); exp.Next())
        {
            nbEdges++;
            Edge Ecur = ShapeCasts.Edge(exp.Current());
            acr[nbEdges] = acr[nbEdges - 1];
            if (!Tool.Degenerated(Ecur))
            {
                EdgeCurveAdaptor ecur = new(Ecur);
                acr[nbEdges] += AbscissaPoint.Length(ecur);
            }
        }

        // total length of the wire
        acr[0] = acr[nbEdges];

        // reduced curvilinear abscisses
        if (acr[0] > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            for (i = 1; i <= nbEdges; i++)
            {
                acr[i] /= acr[0];
            }
        }
        else
        {
            // punctual wire
            acr[nbEdges] = 1;
        }
    }

    //=================================================================================================

    /// <summary>
    /// <paramref name="wire"/> with its edges cut at the reduced curvilinear abscissae <paramref name="acRcuts"/>
    /// (those more than <paramref name="prec"/> from an edge's ends); frozen.
    /// </summary>
    public static Wire InsertACR(in Wire wire, NCollection_Array1<double> acRcuts, double prec)
    {
        Wire result = InternalInsertACR(wire, acRcuts, prec);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary><see cref="InsertACR"/> without freezing the result, for ported callers that build on it.</summary>
    internal static Wire InternalInsertACR(in Wire wire, NCollection_Array1<double> acRcuts, double prec)
    {
        // calculate ACR of the wire to be cut
        WireExplorer exp = new();
        int nbEdges = 0;
        for (exp.Init(wire); exp.More(); exp.Next())
        {
            nbEdges++;
        }
        NCollection_Array1<double> acRwire = new(0, nbEdges);
        ComputeACR(wire, acRwire);

        int i, j, nmax = acRcuts.Length();
        NCollection_Array1<double> paradec = new(1, nmax);
        LibraryMakeWire mw = new();

        double t0, t1 = 0;
        nbEdges = 0;

        // processing edge by edge
        for (exp.Init(wire); exp.More(); exp.Next())
        {
            nbEdges++;
            t0 = t1;
            t1 = acRwire[nbEdges];

            // parameters of cut on this edge
            int ndec = 0;
            for (i = 1; i <= acRcuts.Length(); i++)
            {
                if (t0 + prec < acRcuts[i] && acRcuts[i] < t1 - prec)
                {
                    ndec++;
                    paradec[ndec] = acRcuts[i];
                }
            }

            Edge e = exp.Current();
            Vertex v = exp.CurrentVertex();

            if (ndec == 0 || Tool.Degenerated(e))
            {
                // copy the edge
                mw.Add(e);
            }
            else
            {
                // it is necessary to cut the edge
                // following the direction of parsing of the wire
                bool so = (v.IsSame(Exploration.FirstVertex(e)));
                NCollection_Sequence<Shape> se = new();
                se.Clear();
                NCollection_Sequence<double> sr = new();
                sr.Clear();
                // the wire is always FORWARD
                // it is necessary to modify the parameter of cut6 if the edge is REVERSED
                if (e.Orientation == Forward)
                {
                    for (j = 1; j <= ndec; j++)
                    {
                        sr.Append(paradec[j]);
                    }
                }
                else
                {
                    for (j = 1; j <= ndec; j++)
                    {
                        sr.Append(t0 + t1 - paradec[ndec + 1 - j]);
                    }
                }
                TrimEdge(e, sr, t0, t1, so, se);
                for (j = 1; j <= se.Length(); j++)
                {
                    mw.Add(ShapeCasts.Edge(se.Value(j)));
                }
            }
        }

        // result
        Orientation orien = wire.Orientation;
        Shape localShape = mw.InternalWire();
        localShape.Orient(orien);
        Wire wres = ShapeCasts.Wire(localShape);
        //  TopoDS_Wire wres = TopoDS::Wire(MW.Wire().Oriented(Orien));
        return wres;
    }
}
