// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Evolved.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Evolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Sweeping;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Offsets.Bisectors;
using Kerf.Engine.Offsets.MedialAxis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Engine.Offsets.MedialAxis.MedialSide;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Constructs an evolved volume from a spine (a planar wire or face) and a profile (a wire): the profile is swept
/// along the spine, its parallels built on the spine's bisecting locus.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Shape"/>, <see cref="Top"/>, <see cref="Bottom"/> and
/// <see cref="GeneratedShapes"/> publish: they freeze the result (design 4.2). The algorithm never writes the caller's
/// spine or profile: it moves its own handles on them, and builds every face, edge and vertex it changes.
/// </para>
/// <para>
/// The profile parts that are neither horizontal nor vertical (OCCT's <c>ElementaryPerform</c>) sweep each spine edge
/// with BRepFill_Pipe (BRepFill_Sweep, unit 5S2).
/// </para>
/// </remarks>
public sealed class Evolved
{
    private Face _spine = new();
    private Wire _profile = new();
    private Shape _shape = new();
    private bool _isDone;
#pragma warning disable CS0414 // OCCT sets the spine type without reading it
    private bool _spineType;
#pragma warning restore CS0414
    private global::Kerf.Geometry.JoinKind _joinType;
    private readonly NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> _map =
        new(ShapeHasher.Instance);
    private Shape _top = new();
    private Shape _bottom = new();

    private static double BRepFill_Confusion()
    {
        double tol = 1.0e-6;
        return tol;
    }

    private static NCollection_DataMap<Shape, NCollection_List<Shape>> NewProfMap() => new(ShapeHasher.Instance);

    private static void EdgeVertices(in Edge e, out Vertex v1, out Vertex v2)
    {
        if (e.Orientation() == Reversed)
        {
            Exploration.Vertices(e, out v2, out v1);
        }
        else
        {
            Exploration.Vertices(e, out v1, out v2);
        }
    }

    /// <summary>An empty algorithm.</summary>
    public Evolved()
    {
        _isDone = false;
        _spineType = true;
    }

    /// <summary>
    /// The evolved shape of <paramref name="profile"/> swept along <paramref name="spine"/>. <paramref name="axeProf"/>
    /// places the profile on the spine: it slides on the profile with its direction along the normal to the spine and
    /// its X direction along the spine's tangent.
    /// </summary>
    public Evolved(in Wire spine, in Wire profile, in CoordinateSystem axeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool solid = false)
    {
        _isDone = false;
        Perform(spine, profile, axeProf, Join, solid);
    }

    /// <summary>The evolved shape of <paramref name="profile"/> swept along the face <paramref name="spine"/>.</summary>
    public Evolved(in Face spine, in Wire profile, in CoordinateSystem axeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool solid = false)
    {
        _isDone = false;
        Perform(spine, profile, axeProf, Join, solid);
    }

    private static bool IsVertical(in Edge e)
    {
        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
        Point3D p1 = Tool.Pnt(v1);
        Point3D p2 = Tool.Pnt(v2);

        if (Math.Abs(p1.Y() - p2.Y()) < BRepFill_Confusion())
        {
            // It is a Line ?
            Curve3D? gc = Tool.Curve(e, out Placement _, out double _, out double _);
            if (gc!.GetType() == typeof(LineCurve3D))
            {
                return true;
            }
        }
        return false;
    }

    private static bool IsPlanar(in Edge e)
    {
        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
        Point3D p1 = Tool.Pnt(v1);
        Point3D p2 = Tool.Pnt(v2);

        if (Math.Abs(p1.Z() - p2.Z()) < BRepFill_Confusion())
        {
            // It is a Line ?
            Curve3D? gc = Tool.Curve(e, out Placement _, out double _, out double _);
            if (gc!.GetType() == typeof(LineCurve3D))
            {
                return true;
            }
        }
        return false;
    }

    // Determines the position of the profile relative to the plane XOZ:
    // 1 MAT_Left, 2 MAT_Left and planar, 3 MAT_Left and vertical,
    // 4 MAT_Right, 5 MAT_Right and planar, 6 MAT_Right and vertical.
    private static int Side(in Wire profil, double tol)
    {
        // Rem : it is enough to test the first edge of the Wire.
        //       ( Correctly cut in PrepareProfil)
        Explorer explo = new(profil, ShapeType.Edge);

        int theSide;
        Edge e = ShapeCasts.Edge(explo.Current());

        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
        Point3D p1 = Tool.Pnt(v1);
        Point3D p2 = Tool.Pnt(v2);

        if (p1.Y() < -tol || p2.Y() < -tol)
        {
            theSide = 4;
        }
        else
        {
            theSide = 1;
        }
        if (IsVertical(e))
        {
            theSide += 2;
        }
        else if (IsPlanar(e))
        {
            theSide++;
        }
        return theSide;
    }

    /// <summary>Builds the evolved shape of <paramref name="profile"/> swept along the wire <paramref name="spine"/>.</summary>
    public void Perform(in Wire spine, in Wire profile, in CoordinateSystem axeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool solid = false)
    {
        _spineType = false;
        Face face = new LibraryMakeFace(spine, true).InternalFace();
        PrivatePerform(face, profile, axeProf, Join, solid);
    }

    /// <summary>Builds the evolved shape of <paramref name="profile"/> swept along the face <paramref name="spine"/>.</summary>
    public void Perform(in Face spine, in Wire profile, in CoordinateSystem axeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool solid = false)
    {
        _spineType = true;
        PrivatePerform(spine, profile, axeProf, Join, solid);
    }

    // Kerf: OCCT writes into the spine's own edges and vertices: the parallels at distance 0 (BRepFill_OffsetWire's
    // Compute) and the uncut spine edges go into the result as they are, and Transfert, BRepLib.UpdateTolerances and the
    // quilt write them (SameRange, SameParameter, tolerances, pcurves). The algorithm runs on the caller's spine and
    // profile in a working overlay (unit 10H3, in place of decisions 642-644's copies): the result, Top, Bottom and the
    // generated lists hold the caller's own sub-shapes wherever the algorithm left them unchanged, and the generated
    // shapes are keyed by the caller's sub-shapes directly.
    private void PrivatePerform(in Face spine, in Wire profile, in CoordinateSystem axeProf, global::Kerf.Geometry.JoinKind Join, bool solid)
    {
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(new Shape());
        PrivatePerformInPlace(spine, profile, axeProf, Join, solid);
        if (overlay is null)
        {
            return;
        }

        List<Shape> others = [_top, _bottom];
        foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> bySpine in _map)
        {
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> byProf in bySpine.Value)
            {
                others.AddRange(byProf.Value);
            }
        }
        _shape = overlay.Finish(_shape, others);
        _top = overlay.Final(_top);
        _bottom = overlay.Final(_bottom);
        foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> bySpine in _map)
        {
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> byProf in bySpine.Value)
            {
                for (NCollection_List<Shape>.Iterator it = new(byProf.Value); it.More(); it.Next())
                {
                    it.ChangeValue() = overlay.Final(it.Value());
                }
            }
        }
    }

    private void PrivatePerformInPlace(in Face spine, in Wire profile, in CoordinateSystem axeProf, global::Kerf.Geometry.JoinKind Join, bool solid)
    {
        Shape localShape = spine.Oriented(Forward);
        _spine = ShapeCasts.Face(localShape);
        localShape = profile.Oriented(Forward);
        _profile = ShapeCasts.Wire(localShape);
        _joinType = Join;
        _map.Clear();

        if (_joinType > Arc)
        {
            throw new MethodNotImplementedException();
        }

        NCollection_List<Shape> workProf = new();
        Face workSpine = new();
        NCollection_List<Shape>.Iterator wpIte = new();

        //-------------------------------------------------------------------
        // Positioning of mySpine and myProfil in the workspace.
        //-------------------------------------------------------------------
        Placement lSpine = FindLocation(_spine);
        Transform3D t = new Transform3D().WithTransformation(axeProf);
        Placement lProfile = new(t);
        Placement initLS = _spine.Location();
        Placement initLP = _profile.Location();
        TransformInitWork(lSpine, lProfile);

        //------------------------------------------------------------------
        // projection of the profile and cut of the spine.
        //------------------------------------------------------------------
        NCollection_DataMap<Shape, Shape> mapProf = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> mapSpine = new(ShapeHasher.Instance);

        PrepareProfile(workProf, mapProf);
        PrepareSpine(ref workSpine, mapSpine);

        double tol = BRepFill_Confusion();
        bool yaLeft = false;
        bool yaRight = false;
        Wire sp;

        for (wpIte.Initialize(workProf); wpIte.More(); wpIte.Next())
        {
            sp = ShapeCasts.Wire(wpIte.Value());
            if (Side(sp, tol) < 4)
            {
                yaLeft = true;
            }
            else
            {
                yaRight = true;
            }
            if (yaLeft && yaRight)
            {
                break;
            }
        }

        Face face;
        BisectingLocus locus = new();

        //----------------------------------------------------------
        // Initialisation of cut volevo.
        // For each part of the profile create a volevo added to CutVevo
        //----------------------------------------------------------
        Evolved cutVevo = new();
        Wire wp = new();
        BrepBuilder bb = new();
        WireExplorer wExp = new();

        bb.MakeWire(ref wp);

        for (wpIte.Initialize(workProf); wpIte.More(); wpIte.Next())
        {
            for (wExp.Init(ShapeCasts.Wire(wpIte.Value())); wExp.More(); wExp.Next())
            {
                bb.Add(wp, wExp.Current());
            }
        }
        cutVevo.SetWork(workSpine, wp);

        Quilt glue = new();
        int cSide;

        //---------------------------------
        // Construction of vevos to the left.
        //---------------------------------
        if (yaLeft)
        {
            //-----------------------------------------------------
            // Calculate the map of bisector locations at the left.
            // and links Topology -> base elements of the map.
            //-----------------------------------------------------
            MedialAxisExplorer exp = new(workSpine);
            locus.Compute(exp, 1, MedialSide.Left);
            LinkTopoBilo link = new(exp, locus);

            for (wpIte.Initialize(workProf); wpIte.More(); wpIte.Next())
            {
                sp = ShapeCasts.Wire(wpIte.Value());
                cSide = Side(sp, tol);
                //-----------------------------------------------
                // Construction and adding of elementary volevo.
                //-----------------------------------------------
                Evolved vevo = new();
                if (cSide == 1)
                {
                    vevo.ElementaryPerform(workSpine, sp, locus, link, Join);
                }
                else if (cSide == 2)
                {
                    vevo.PlanarPerform(workSpine, sp, locus, link, Join);
                }
                else if (cSide == 3)
                {
                    vevo.VerticalPerform(workSpine, sp, locus, link, Join);
                }
                cutVevo.Add(vevo, sp, glue);
            }
        }

        //---------------------------------
        // Construction of vevos to the right.
        //---------------------------------
        if (yaRight)
        {
            //-----------------------------------
            // Decomposition of the face into wires.
            //-----------------------------------
            Explorer spineExp = new(workSpine, ShapeType.Wire);
            for (; spineExp.More(); spineExp.Next())
            {
                //----------------------------------------------
                // Calculate the map to the right of the current wire.
                //----------------------------------------------
                LibraryMakeFace b = new(new Plane3D(0.0, 0.0, 1.0, 0.0));
                Shape localShapeRev = spineExp.Current().Reversed();
                b.Add(ShapeCasts.Wire(localShapeRev));
                face = b.InternalFace();
                MedialAxisExplorer exp = new(face);
                locus.Compute(exp, 1, MedialSide.Left);
                LinkTopoBilo link = new(exp, locus);

                for (wpIte.Initialize(workProf); wpIte.More(); wpIte.Next())
                {
                    sp = ShapeCasts.Wire(wpIte.Value());
                    cSide = Side(sp, tol);
                    //-----------------------------------------------
                    // Construction and adding of an elementary volevo
                    //-----------------------------------------------
                    Evolved vevo = new();
                    if (cSide == 4)
                    {
                        vevo.ElementaryPerform(face, sp, locus, link, Join);
                    }
                    else if (cSide == 5)
                    {
                        vevo.PlanarPerform(face, sp, locus, link, Join);
                    }
                    else if (cSide == 6)
                    {
                        vevo.VerticalPerform(face, sp, locus, link, Join);
                    }
                    cutVevo.Add(vevo, sp, glue);
                }
            }
        }

        if (solid)
        {
            cutVevo.AddTopAndBottom(glue);
        }

        //-------------------------------------------------------------------------
        // Gluing of regularites on parallel edges generate4d by vertices of the
        // cut of the profile.
        //-------------------------------------------------------------------------
        cutVevo.ContinuityOnOffsetEdge(workProf);

        //-----------------------------------------------------------------
        // construction of the shape via the quilt, ie:
        // - sharing of topologies of elementary added volevos.
        // - Orientation of faces correspondingly to each other.
        //-----------------------------------------------------------------
        cutVevo.ChangeShape() = glue.InternalShells();
        //------------------------------------------------------------------------
        // Transfer of the map of generated elements and of the shape of Cutvevo
        // in myMap and repositioning in the initial space.
        //------------------------------------------------------------------------
        Transfert(cutVevo, mapProf, mapSpine, lSpine.Inverted(), initLS, initLP);

        // Orientation of the solid.
        if (solid)
        {
            MakeSolid();
        }

        //  modified by NIZHNY-EAP Mon Jan 24 11:26:48 2000 ___BEGIN___
        BrepLibrary.UpdateTolerances(_shape, false);
        //  modified by NIZHNY-EAP Mon Jan 24 11:26:50 2000 ___END___
        _isDone = true;
    }

    private static void IsInversed(in Shape s, in Edge e1, in Edge e2, bool[] inverse)
    {
        inverse[0] = inverse[1] = false;
        if (s.ShapeType() != ShapeType.Edge)
        {
            return;
        }

        Point3D p;
        Vector3D ds, dc1, dc2;
        EdgeCurveAdaptor cs = new(ShapeCasts.Edge(s));
        if (s.Orientation() == Forward)
        {
            cs.D1(cs.FirstParameter(), out p, out ds);
        }
        else
        {
            cs.D1(cs.LastParameter(), out p, out ds);
            ds = ds.Reversed();
        }

        if (!Tool.Degenerated(e1))
        {
            EdgeCurveAdaptor c1 = new(ShapeCasts.Edge(e1));
            if (e1.Orientation() == Forward)
            {
                c1.D1(c1.FirstParameter(), out p, out dc1);
            }
            else
            {
                c1.D1(c1.LastParameter(), out p, out dc1);
                dc1 = dc1.Reversed();
            }
            inverse[0] = (ds.Dot(dc1) < 0.0);
        }
        else
        {
            inverse[0] = true;
        }

        if (!Tool.Degenerated(e2))
        {
            EdgeCurveAdaptor c2 = new(ShapeCasts.Edge(e2));
            if (e2.Orientation() == Forward)
            {
                c2.D1(c2.FirstParameter(), out p, out dc2);
            }
            else
            {
                c2.D1(c2.LastParameter(), out p, out dc2);
                dc2 = dc2.Reversed();
            }
            inverse[1] = (ds.Dot(dc2) < 0.0);
        }
        else
        {
            inverse[1] = true;
        }
    }

    private void SetWork(in Face sp, in Wire pr)
    {
        _spine = sp;
        _profile = pr;
    }

    // Determines if the pipes were at the side of the concavity. In this case they can be closed.
    // WARNING: Not finished. Done only for circles.
    private static bool ConcaveSide(in Shape s, in Face F)
    {
        if (s.ShapeType() == ShapeType.Vertex)
        {
            return false;
        }

        if (s.ShapeType() == ShapeType.Edge)
        {
            Curve2D? g2d = Tool.CurveOnSurface(ShapeCasts.Edge(s), F, out double f, out double l);

            CurveAdaptor2D ac = new(g2d!, f, l);
            if (ac.GetType() == Circle)
            {
                bool direct = ac.Circle().IsDirect;
                if (s.Orientation() == Reversed)
                {
                    direct = (!direct);
                }
                return direct;
            }
        }
        return false;
    }

    private void ElementaryPerform(in Face sp, in Wire pr, BisectingLocus locus, LinkTopoBilo link,
                                   global::Kerf.Geometry.JoinKind Join)
    {
        Shape localShape = sp.Oriented(Forward);
        _spine = ShapeCasts.Face(localShape);
        _profile = pr;
        _map.Clear();

        BrepBuilder myBuilder = new();
        Compound compound = new();
        myBuilder.MakeCompound(ref compound);
        _shape = compound;

        //---------------------------------------------------------------------
        // MapNodeVertex : associate to each node of the map (key1) and
        //                 to each element of the profile (key2) a vertex (item).
        // MapBis        : a set of edges or vertexes (item) generated by
        //                 a bisectrice on a face or an edge (key) of
        //                 tubes or revolutions.
        // MapVerPar     : Map of parameters of vertices on parallel edges
        //                 the list contained in MapVerPar (E) corresponds
        //                 to parameters on E of vertices contained in  MapBis(E);
        // MapBS         : links BasicElt of the map => Topology of the spine.
        //---------------------------------------------------------------------

        // keyed by the medial axis's nodes (OCCT: by handle; only looked up, never iterated)
        NCollection_DataMap<MedialNode, NCollection_DataMap<Shape, Shape>> mapNodeVertex = new(ReferenceEqualityComparer.Instance);
        NCollection_DataMap<Shape, NCollection_Sequence<Shape>> mapBis = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_Sequence<double>> mapVerPar = new(ShapeHasher.Instance);

        // mark of the profile.
        CoordinateSystem axeRef = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));

        //---------------------------------------------------------------
        // Construction of revolutions and tubes.
        //---------------------------------------------------------------
        WireExplorer profExp = new();
        Explorer faceExp = new();
        WireExplorer wireExp = new();

        for (faceExp.Init(_spine, ShapeType.Wire); faceExp.More(); faceExp.Next())
        {
            for (wireExp.Init(ShapeCasts.Wire(faceExp.Current())); wireExp.More(); wireExp.Next())
            {
                Edge currentEdge0 = wireExp.Current();
                EdgeVertices(currentEdge0, out Vertex _, out Vertex vLast);

                for (link.Init(vLast); link.More(); link.Next())
                {
                    //----------------------------.
                    // Construction of a Revolution
                    //----------------------------.
                    MakeRevol(currentEdge0, vLast, axeRef);
                }

                for (link.Init(currentEdge0); link.More(); link.Next())
                {
                    //------------------------.
                    // Construction of a Tube
                    //-------------------------
                    MakePipe(currentEdge0, axeRef);
                }
            }
        }

        //---------------------------------------------------
        // Construction of edges associated to bissectrices.
        //---------------------------------------------------
        MedialArc currentArc;
        Curve2D bis;
        Curve2D? pCurve1, pCurve2;
        Curve3D? cBis;
        Edge currentEdge = new();
        Shape[] S = [new(), new()];
        Face[] f = [new(), new()];
        Edge[] e = [new(), new(), new(), new()];
        int k;

        for (int i = 1; i <= locus.Graph().NumberOfArcs(); i++)
        {
            currentArc = locus.Graph().Arc(i);
            SimpleExpression(locus.GeomBis(currentArc, out bool reverse), out bis);

            //------------------------------------------------------------------
            // Return elements of the spine corresponding to separate basicElts.
            //------------------------------------------------------------------
            S[0] = link.GeneratingShape(currentArc.FirstElement());
            S[1] = link.GeneratingShape(currentArc.SecondElement());

            bool concave0 = ConcaveSide(S[0], _spine);
            bool concave1 = ConcaveSide(S[1], _spine);

            NCollection_Sequence<Shape> vOnF = new(), vOnL = new();
            NCollection_Sequence<Point3D> parOnF = new(), parOnL = new();

            NCollection_DataMap<Shape, NCollection_Sequence<Shape>> mapSeqVer = new(ShapeHasher.Instance);
            NCollection_DataMap<Shape, NCollection_Sequence<Point3D>> mapSeqPar = new(ShapeHasher.Instance);

            for (profExp.Init(_profile); profExp.More(); profExp.Next())
            {
                //-----------------------------------------------
                // Return two faces separated by the bissectrice.
                //-----------------------------------------------
                f[0] = ShapeCasts.Face(_map[S[0]][profExp.Current()].First());
                f[1] = ShapeCasts.Face(_map[S[1]][profExp.Current()].First());

                //------------------------------------
                // Return parallel edges on each face.
                //------------------------------------
                EdgeVertices(profExp.Current(), out Vertex vf, out Vertex vl);

                e[0] = ShapeCasts.Edge(_map[S[0]][vf].First());
                e[1] = ShapeCasts.Edge(_map[S[0]][vl].First());
                e[2] = ShapeCasts.Edge(_map[S[1]][vf].First());
                e[3] = ShapeCasts.Edge(_map[S[1]][vl].First());

                bool[] inv0 = new bool[2];
                bool[] inv1 = new bool[2];

                inv0[0] = inv0[1] = inv1[0] = inv1[1] = false;
                if (concave0)
                {
                    IsInversed(S[0], e[0], e[1], inv0);
                }
                if (concave1)
                {
                    IsInversed(S[1], e[2], e[3], inv1);
                }

                //---------------------------------------------
                // Construction of geometries.
                //---------------------------------------------
                TrimSurfaceTool trim = new(bis, f[0], f[1], e[0], e[2], inv0[0], inv1[0]);
                //-----------------------------------------------------------
                // Construction of vertices corresponding to the node of the map
                //-----------------------------------------------------------
                Vertex vs = new(), ve = new();
                MedialNode node1, node2;

                if (reverse)
                {
                    node1 = currentArc.SecondNode();
                    node2 = currentArc.FirstNode();
                }
                else
                {
                    node1 = currentArc.FirstNode();
                    node2 = currentArc.SecondNode();
                }
                //--------------------------------------------------------
                // Particular case when the node is on a vertex of the spine.
                //--------------------------------------------------------
                if (node1.OnBasicElt())
                {
                    if (S[0].ShapeType() == ShapeType.Vertex)
                    {
                        node1 = currentArc.FirstElement().StartArc()!.FirstNode();
                    }
                    else if (S[1].ShapeType() == ShapeType.Vertex)
                    {
                        node1 = currentArc.SecondElement().StartArc()!.FirstNode();
                    }
                }
                // End of particular case.

                int startOnF = VertexFromNode(node1, ShapeCasts.Edge(profExp.Current()), vf, vl, mapNodeVertex, ref vs);

                int endOnF = VertexFromNode(node2, ShapeCasts.Edge(profExp.Current()), vf, vl, mapNodeVertex, ref ve);

                //-----------------------------------------------------------
                // Construction of vertices on edges parallel to the spine.
                //-----------------------------------------------------------
                if (!mapSeqVer.IsBound(vf))
                {
                    if (inv0[0] || inv1[0])
                    {
                        parOnF.Clear();
                        vOnF.Clear();
                    }
                    else
                    {
                        trim.IntersectWith(e[0], e[2], parOnF);
                        vOnF.Clear();
                        for (int s = 1; s <= parOnF.Length(); s++)
                        {
                            Vertex vc = new();
                            myBuilder.MakeVertex(ref vc);
                            vOnF.Append(vc);
                        }
                        if (startOnF == 1)
                        {
                            vOnF.SetValue(1, vs);
                        }
                        if (endOnF == 1)
                        {
                            vOnF.SetValue(parOnF.Length(), ve);
                        }
                    }
                }
                else
                {
                    // OCCT assigns copies
                    parOnF = new NCollection_Sequence<Point3D>(mapSeqPar[vf]);
                    vOnF = new NCollection_Sequence<Shape>(mapSeqVer[vf]);
                }

                if (!mapSeqVer.IsBound(vl))
                {
                    if (inv0[1] || inv1[1])
                    {
                        parOnL.Clear();
                        vOnL.Clear();
                    }
                    else
                    {
                        trim.IntersectWith(e[1], e[3], parOnL);
                        vOnL.Clear();
                        for (int s = 1; s <= parOnL.Length(); s++)
                        {
                            Vertex vc = new();
                            myBuilder.MakeVertex(ref vc);
                            vOnL.Append(vc);
                        }
                        if (startOnF == 3)
                        {
                            vOnL.SetValue(1, vs);
                        }
                        if (endOnF == 3)
                        {
                            vOnL.SetValue(parOnL.Length(), ve);
                        }
                    }
                }
                else
                {
                    parOnL = new NCollection_Sequence<Point3D>(mapSeqPar[vl]);
                    vOnL = new NCollection_Sequence<Shape>(mapSeqVer[vl]);
                }

                //------------------------------------------------------
                // Test if the Bissectrice is not projected on the face
                //------------------------------------------------------
                if ((startOnF == 0) && (endOnF == 0) && vOnL.IsEmpty() && vOnF.IsEmpty())
                {
                    // No trace of the bisectrice on the face.
                    continue;
                }

                if ((startOnF == 0) && (endOnF == 0) && (vOnL.Length() + vOnF.Length() == 1))
                {
                    // the first or last node of the arc is on the edge
                    // but the arc is not on the face.
                    continue;
                }

                //---------------------------------------------------------
                // determine the intervals of the bissectrice that are
                // projected on F[0] and F[1].
                //---------------------------------------------------------
                NCollection_Sequence<double> lastPar = new(), firstPar = new();
                NCollection_Sequence<Shape> firstV = new(), lastV = new();

                ComputeIntervals(vOnF, vOnL, parOnF, parOnL, trim, bis, vs, ve, firstPar, lastPar, firstV, lastV);

                for (int ti = 1; ti <= firstPar.Length(); ti++)
                {
                    Vertex v1 = ShapeCasts.Vertex(firstV.Value(ti));
                    Vertex v2 = ShapeCasts.Vertex(lastV.Value(ti));

                    trim.Project(firstPar.Value(ti), lastPar.Value(ti), out cBis, out pCurve1, out pCurve2, out Continuity continuity);

                    //-------------------------------------
                    // Coding of the edge.
                    //-------------------------------------
                    myBuilder.MakeEdge(ref currentEdge, cBis, BRepFill_Confusion());

                    myBuilder.UpdateVertex(v1, cBis!.Value(cBis.FirstParameter()), BRepFill_Confusion());
                    myBuilder.UpdateVertex(v2, cBis.Value(cBis.LastParameter()), BRepFill_Confusion());

                    myBuilder.Add(currentEdge, v1.Oriented(Forward));
                    myBuilder.Add(currentEdge, v2.Oriented(Reversed));

                    myBuilder.Range(currentEdge, cBis.FirstParameter(), cBis.LastParameter());
                    myBuilder.UpdateEdge(currentEdge, pCurve1, f[0], BRepFill_Confusion());
                    myBuilder.UpdateEdge(currentEdge, pCurve2, f[1], BRepFill_Confusion());

                    myBuilder.Continuity(currentEdge, f[0], f[1], continuity);

                    //-------------------------------------------
                    // Storage of the edge for each of faces.
                    //-------------------------------------------
                    for (k = 0; k <= 1; k++)
                    {
                        if (!mapBis.IsBound(f[k]))
                        {
                            mapBis.Bind(f[k], new NCollection_Sequence<Shape>());
                        }
                    }
                    //---------------------------------------------------------------
                    // orientation of the edge depends on the direction of the skin.
                    // skin => same orientation E[0] , inverted orientation E[2]
                    // if contreskin it is inverted.
                    //--------------------------------------------------------------
                    e[0].Orientation(BrepTools.OriEdgeInFace(e[0], f[0]));
                    e[2].Orientation(BrepTools.OriEdgeInFace(e[2], f[1]));

                    if (DistanceToOZ(vf) < DistanceToOZ(vl))
                    {
                        // Skin
                        mapBis[f[0]].Append(currentEdge.Oriented(e[0].Orientation()));
                        currentEdge.Orientation(global::Kerf.Engine.Topology.TopAbs.Complement(e[2].Orientation()));
                        mapBis[f[1]].Append(currentEdge);
                    }
                    else
                    {
                        // Contreskin
                        mapBis[f[1]].Append(currentEdge.Oriented(e[2].Orientation()));
                        currentEdge.Orientation(global::Kerf.Engine.Topology.TopAbs.Complement(e[0].Orientation()));
                        mapBis[f[0]].Append(currentEdge);
                    }
                }

                //----------------------------------------------
                // Storage of vertices on parallel edges.
                // fill MapBis and MapVerPar.
                // VOnF for E[0] and E[2].
                // VOnL for E[1] and E[3].
                //----------------------------------------------
                for (k = 0; k <= 2; k = k + 2)
                {
                    if (!mapSeqVer.IsBound(vf))
                    {
                        if (!vOnF.IsEmpty())
                        {
                            if (!mapBis.IsBound(e[k]))
                            {
                                mapBis.Bind(e[k], new NCollection_Sequence<Shape>());
                                mapVerPar.Bind(e[k], new NCollection_Sequence<double>());
                            }
                            for (int ii = 1; ii <= vOnF.Length(); ii++)
                            {
                                mapBis[e[k]].Append(vOnF.Value(ii));
                                if (k == 0)
                                {
                                    mapVerPar[e[k]].Append(parOnF.Value(ii).Y());
                                }
                                else
                                {
                                    mapVerPar[e[k]].Append(parOnF.Value(ii).Z());
                                }
                            }
                        }
                    }
                }

                for (k = 1; k <= 3; k = k + 2)
                {
                    if (!mapSeqVer.IsBound(vl))
                    {
                        if (!vOnL.IsEmpty())
                        {
                            if (!mapBis.IsBound(e[k]))
                            {
                                mapBis.Bind(e[k], new NCollection_Sequence<Shape>());
                                mapVerPar.Bind(e[k], new NCollection_Sequence<double>());
                            }
                            for (int ii = 1; ii <= vOnL.Length(); ii++)
                            {
                                mapBis[e[k]].Append(vOnL.Value(ii));
                                if (k == 1)
                                {
                                    mapVerPar[e[k]].Append(parOnL.Value(ii).Y());
                                }
                                else
                                {
                                    mapVerPar[e[k]].Append(parOnL.Value(ii).Z());
                                }
                            }
                        }
                    }
                }

                //----------------------------------------------------------------
                // Edge [1] of the current face will be Edge [0] of the next face.
                // => copy of VonL in VonF. To avoid creating the same vertices twice.
                //-----------------------------------------------------------------

                // OCCT binds copies
                mapSeqPar.Bind(vf, new NCollection_Sequence<Point3D>(parOnF));
                mapSeqVer.Bind(vf, new NCollection_Sequence<Shape>(vOnF));
                mapSeqPar.Bind(vl, new NCollection_Sequence<Point3D>(parOnL));
                mapSeqVer.Bind(vl, new NCollection_Sequence<Shape>(vOnL));
            }
        }

        //----------------------------------
        // Construction of parallel edges.
        //----------------------------------
        Shape currentProf = new(), precProf = new();
        Face currentFace;
        Shape currentSpine;
        Vertex vcf = new(), vcl = new();

        // OCCT iterates myMap in its hash order (TShape addresses); the port in insertion order (occt-issues.md)
        foreach (Shape spineKey in new List<Shape>(_map.Keys))
        {
            currentSpine = spineKey;

            for (profExp.Init(_profile); profExp.More(); profExp.Next())
            {
                currentProf = profExp.Current();
                EdgeVertices(ShapeCasts.Edge(currentProf), out vcf, out vcl);
                currentEdge = ShapeCasts.Edge(_map[currentSpine][vcf].First());

                //-------------------------------------------------------------
                // RQ : Current Edge is oriented relatively to the face (oriented forward)
                //     generated by edge CurrentProf .
                //-------------------------------------------------------------
                if (mapBis.IsBound(currentEdge))
                {
                    //--------------------------------------------------------
                    // Find if one of two faces connected to the edge
                    // belongs to volevo. The edges on this face serve
                    // to eliminate certain vertices that can appear twice
                    // on the parallel edge. These Vertices correspond to the
                    // nodes of the map.
                    //---------------------------------------------------------
                    Shape faceControle;
                    bool yaFace = true;

                    faceControle = _map[currentSpine][currentProf].First();
                    if (!mapBis.IsBound(faceControle))
                    {
                        yaFace = false;
                        if (!precProf.IsNull)
                        {
                            faceControle = _map[currentSpine][precProf].First();
                            if (mapBis.IsBound(faceControle))
                            {
                                yaFace = true;
                            }
                        }
                    }

                    if (yaFace)
                    {
                        //------------------------------------------------------------
                        // No connected face in the volevo => no parallel edge.
                        //------------------------------------------------------------
                        NCollection_Sequence<Shape> seqOfShape = new();
                        TrimEdge(currentEdge, mapBis[faceControle], mapBis[currentEdge], mapVerPar[currentEdge], seqOfShape);

                        for (k = 1; k <= seqOfShape.Length(); k++)
                        {
                            _map[currentSpine][vcf].Append(seqOfShape.Value(k));
                        }
                    }
                }
                precProf = currentProf;
            }

            //------------------------------------------------------------
            // Construction of the parallel edge from the last vertex of myProfile.
            //------------------------------------------------------------
            currentEdge = ShapeCasts.Edge(_map[currentSpine][vcl].First());

            if (mapBis.IsBound(currentEdge))
            {
                bool yaFace = true;
                Shape faceControle;

                faceControle = _map[currentSpine][currentProf].First();
                if (!mapBis.IsBound(faceControle))
                {
                    yaFace = false;
                }
                // the number of element of the list allows to know
                // if the edges have already been done (closed profile) .
                if (yaFace && _map[currentSpine][vcl].Extent() <= 1)
                {
                    NCollection_Sequence<Shape> seqOfShape = new();
                    TrimEdge(currentEdge, mapBis[faceControle], mapBis[currentEdge], mapVerPar[currentEdge], seqOfShape);

                    for (k = 1; k <= seqOfShape.Length(); k++)
                    {
                        _map[currentSpine][vcl].Append(seqOfShape.Value(k));
                    }
                }
            }
        }

        //-------------------------------------------------------------------
        // Cut faces by edges.
        //-------------------------------------------------------------------
        foreach (Shape spineKey in new List<Shape>(_map.Keys))
        {
            currentSpine = spineKey;

            for (profExp.Init(_profile); profExp.More(); profExp.Next())
            {
                currentProf = profExp.Current();
                currentFace = ShapeCasts.Face(_map[currentSpine][currentProf].First());
                _map[currentSpine][currentProf].Clear();

                if (mapBis.IsBound(currentFace))
                {
                    //----------------------------------------------------------
                    // If the face does not contain edges that can limit it
                    // it does not appear in volevo.
                    // cut of face by edges can generate many faces.
                    //
                    // Add edges generated on the edges parallel to the set
                    // of edges that limit the face.
                    //
                    //------------------------------------------------------------
                    EdgeVertices(ShapeCasts.Edge(currentProf), out vcf, out vcl);

                    NCollection_List<Shape>.Iterator itl = new();
                    NCollection_List<Shape> lf = _map[currentSpine][vcf];

                    Orientation ori = BrepTools.OriEdgeInFace(ShapeCasts.Edge(lf.First()), currentFace);
                    for (itl.Initialize(lf), itl.Next(); itl.More(); itl.Next())
                    {
                        Edge re = ShapeCasts.Edge(itl.Value());
                        mapBis[currentFace].Append(re.Oriented(ori));
                    }
                    NCollection_List<Shape> ll = _map[currentSpine][vcl];
                    ori = BrepTools.OriEdgeInFace(ShapeCasts.Edge(ll.First()), currentFace);
                    for (itl.Initialize(ll), itl.Next(); itl.More(); itl.Next())
                    {
                        Edge re = ShapeCasts.Edge(itl.Value());
                        mapBis[currentFace].Append(re.Oriented(ori));
                    }

                    // Cut of the face.
                    NCollection_Sequence<Shape> seqOfShape = new();

                    TrimFace(currentFace, mapBis[currentFace], seqOfShape);

                    for (int ii = 1; ii <= seqOfShape.Length(); ii++)
                    {
                        myBuilder.Add(_shape, seqOfShape.Value(ii));
                        _map[currentSpine][currentProf].Append(seqOfShape.Value(ii));
                    }
                }
            }
            //-----------------------------------------------------------------
            // Removal of first edge (edge of origin) from lists of myMap
            // corresponding to vertices of the profile.
            //-----------------------------------------------------------------
            Explorer explo = new(_profile, ShapeType.Vertex);
            HashSet<Shape> vmap = new(ShapeHasher.Instance);

            for (; explo.More(); explo.Next())
            {
                if (vmap.Add(explo.Current()))
                {
                    _map[currentSpine][explo.Current()].RemoveFirst();
                }
            }
        }
        _isDone = true;
    }

    private void PlanarPerform(in Face sp, in Wire pr, BisectingLocus locus, LinkTopoBilo link,
                               global::Kerf.Geometry.JoinKind Join)
    {
        Shape localShapeOriented = sp.Oriented(Forward);
        _spine = ShapeCasts.Face(localShapeOriented);
        _profile = pr;
        _map.Clear();

        BrepBuilder b = new();
        Compound compound = new();
        b.MakeCompound(ref compound);
        _shape = compound;

        WireExplorer profExp = new();
        Explorer exp = new();
        NCollection_DataMap<Shape, Shape> mapVP = new(ShapeHasher.Instance);
        OffsetWire paral = new();

        for (profExp.Init(_profile); profExp.More(); profExp.Next())
        {
            Edge e = profExp.Current();
            FaceRestrictor fr = new();
            OffsetAncestors offAnc = new();

            Vertex[] v = [new(), new()];
            EdgeVertices(e, out v[0], out v[1]);
            double alt = Altitud(v[0]);
            double[] offset = new double[2];
            offset[0] = DistanceToOZ(v[0]);
            offset[1] = DistanceToOZ(v[1]);
            bool isMinV1 = (offset[0] < offset[1]);

            for (int i = 0; i <= 1; i++)
            {
                if (!mapVP.IsBound(v[i]))
                {
                    //------------------------------------------------
                    // Calculate parallel lines corresponding to vertices.
                    //------------------------------------------------
                    paral.PerformWithBiLo(_spine, offset[i], locus, link, Join, alt);
                    offAnc.Perform(paral);
                    mapVP.Bind(v[i], paral.InternalShape());

                    //-----------------------------
                    // Update myMap (.)(V[i])
                    //-----------------------------
                    for (exp.Init(paral.InternalShape(), ShapeType.Edge); exp.More(); exp.Next())
                    {
                        Edge wc = ShapeCasts.Edge(exp.Current());
                        Shape gs = offAnc.Ancestor(wc);
                        if (!_map.IsBound(gs))
                        {
                            _map.Bind(gs, NewProfMap());
                        }
                        if (!_map[gs].IsBound(v[i]))
                        {
                            _map[gs].Bind(v[i], new NCollection_List<Shape>(paral.InternalGeneratedShapes(gs)));
                        }
                    }
                }
                Shape rest = mapVP[v[i]];

                bool toReverse = false;
                if ((isMinV1 && (i == 1)) || (!isMinV1 && (i == 0)))
                {
                    toReverse = true;
                }

                if (!rest.IsNull)
                {
                    if (rest.ShapeType() == ShapeType.Wire)
                    {
                        if (toReverse)
                        {
                            Shape localShape = rest.Reversed();
                            Wire wire = ShapeCasts.Wire(localShape);
                            fr.Add(wire);
                        }
                        else
                        {
                            fr.Add(ShapeCasts.Wire(rest));
                        }
                    }
                    else
                    {
                        for (exp.Init(rest, ShapeType.Wire); exp.More(); exp.Next())
                        {
                            Wire wCop = ShapeCasts.Wire(exp.Current());
                            if (toReverse)
                            {
                                Shape localShape = wCop.Reversed();
                                Wire bWire = ShapeCasts.Wire(localShape);
                                fr.Add(bWire);
                            }
                            else
                            {
                                fr.Add(wCop);
                            }
                        }
                    }
                }
            }

            //----------------------------------------------------
            // Construction of faces limited by parallels.
            // - set to the height of the support face.
            //----------------------------------------------------
            Transform3D t = new Transform3D().WithTranslation(new Vector3D(0, 0, alt));
            Placement lt = new(t);
            Shape localShape2 = _spine.Moved(lt);
            fr.Init(ShapeCasts.Face(localShape2));
            // the restrictor runs on the wires this algorithm made (5O1's notes)
            fr.InternalPerform();

            for (; fr.More(); fr.Next())
            {
                Face fc = fr.InternalCurrent();
                b.Add(_shape, fc);
                //---------------------------------------
                // Update myMap(.)(E)
                //---------------------------------------
                for (exp.Init(fc, ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge ce = ShapeCasts.Edge(exp.Current());
                    if (offAnc.HasAncestor(ce))
                    {
                        Shape initE = offAnc.Ancestor(ce);
                        if (!_map.IsBound(initE))
                        {
                            _map.Bind(initE, NewProfMap());
                        }
                        if (!_map[initE].IsBound(e))
                        {
                            _map[initE].Bind(e, new NCollection_List<Shape>());
                        }
                        _map[initE][e].Append(fc);
                    }
                }
            }
        } // End loop on profile.
    }

    private void VerticalPerform(in Face sp, in Wire pr, BisectingLocus locus, LinkTopoBilo link,
                                 global::Kerf.Geometry.JoinKind Join)
    {
        Shape localShape = sp.Oriented(Forward);
        _spine = ShapeCasts.Face(localShape);
        _profile = pr;
        _map.Clear();

        BrepBuilder b = new();
        Compound compound = new();
        b.MakeCompound(ref compound);
        _shape = compound;

        WireExplorer profExp = new();
        Explorer exp = new();
        OffsetWire paral = new();
        OffsetAncestors offAnc = new();

        bool first = true;
        Shape Base = new();

        for (profExp.Init(_profile); profExp.More(); profExp.Next())
        {
            Edge e = profExp.Current();
            EdgeVertices(e, out Vertex v1, out Vertex v2);
            double alt1 = Altitud(v1);
            double alt2 = Altitud(v2);

            if (first)
            {
                double offset = DistanceToOZ(v1);
                if (Math.Abs(offset) < BRepFill_Confusion())
                {
                    offset = 0.0;
                }
                paral.PerformWithBiLo(_spine, offset, locus, link, Join, alt1);
                offAnc.Perform(paral);
                Base = paral.InternalShape();

                // MAJ myMap
                for (exp.Init(Base, ShapeType.Edge); exp.More(); exp.Next())
                {
                    Edge edge = ShapeCasts.Edge(exp.Current());
                    Shape ae = offAnc.Ancestor(edge);
                    if (!_map.IsBound(ae))
                    {
                        _map.Bind(ae, NewProfMap());
                    }
                    if (!_map[ae].IsBound(v1))
                    {
                        _map[ae].Bind(v1, new NCollection_List<Shape>());
                    }
                    _map[ae][v1].Append(edge);
                }
                first = false;
            }

            Prism ps = new(Base, new Vector3D(0, 0, alt2 - alt1), false);

            Base = ps.LastShape();

            for (exp.Init(ps.Shape(), ShapeType.Face); exp.More(); exp.Next())
            {
                b.Add(_shape, exp.Current());
            }

            // MAJ myMap
            foreach (Shape key in new List<Shape>(_map.Keys))
            {
                NCollection_List<Shape> lof = _map[key][v1];
                NCollection_List<Shape>.Iterator itLOF = new(lof);
                if (!_map[key].IsBound(v2))
                {
                    _map[key].Bind(v2, new NCollection_List<Shape>());
                }

                if (!_map[key].IsBound(e))
                {
                    _map[key].Bind(e, new NCollection_List<Shape>());
                }

                for (; itLOF.More(); itLOF.Next())
                {
                    Shape os = itLOF.Value();
                    _map[key][v2].Append(ps.LastShape(os));
                    _map[key][e].Append(ps.Shape(os));
                }
            }
        }
    }

    // Order the sequence of points by growing x.
    private static void Bubble(NCollection_Sequence<double> seq)
    {
        bool invert = true;
        int nbPoints = seq.Length();

        while (invert)
        {
            invert = false;
            for (int i = 1; i < nbPoints; i++)
            {
                if (seq.Value(i + 1) < seq.Value(i))
                {
                    seq.Exchange(i, i + 1);
                    invert = true;
                }
            }
        }
    }

    // - Projection of the profile on the working plane.
    // - Cut of the profile at the extrema of distance from profile to axis Oz.
    // - Isolate vertical and horizontal parts.
    // - Reconstruction of wires starting from cut edges.
    // New wires stored in <WorkProf> are always at the same side of axis OZ or mixed with it.
    private void PrepareProfile(NCollection_List<Shape> workProf, NCollection_DataMap<Shape, Shape> mapProf)
    {
        // Supposedly the profile is located so that the only transformation
        // to be carried out is a projection on plane yOz.

        // initialise the projection Plane and the Line to evaluate the extrema.
        PlaneSurface plane = new(new CoordinateSystem(Constants.YOZ()));
        LineCurve2D line = new(Constants.OY2d());

        // Map initial vertex -> projected vertex.
        NCollection_DataMap<Shape, Shape> mapVerRefMoved = new(ShapeHasher.Instance);

        Wire w = new();
        BrepBuilder b = new();
        NCollection_List<Shape> wp = new();
        b.MakeWire(ref w);
        wp.Append(w);

        WireExplorer exp = new(_profile);

        while (exp.More())
        {
            NCollection_List<Shape> cuts = new();
            bool newWire = false;
            Edge e = ShapeCasts.Edge(exp.Current());

            // Cut of the edge.
            CutEdgeProf(e, plane, line, cuts, mapVerRefMoved);

            EdgeVertices(e, out Vertex _, out Vertex _);

            if (cuts.IsEmpty())
            {
                // Neither extrema nor intersections nor vertices on the axis.
                b.Add(w, e);
                mapProf.Bind(e, e);
            }
            else
            {
                while (!cuts.IsEmpty())
                {
                    Edge ne = ShapeCasts.Edge(cuts.First());
                    mapProf.Bind(ne, e);
                    EdgeVertices(ne, out Vertex v1, out Vertex v2);
                    if (!mapProf.IsBound(v1))
                    {
                        mapProf.Bind(v1, e);
                    }
                    if (!mapProf.IsBound(v2))
                    {
                        mapProf.Bind(v2, e);
                    }

                    b.Add(w, ne);
                    cuts.RemoveFirst();

                    if (DistanceToOZ(v2) < BRepFill_Confusion() && DistanceToOZ(v1) > BRepFill_Confusion())
                    {
                        // NE ends on axis OZ => new wire
                        if (cuts.IsEmpty())
                        {
                            // last part of the current edge
                            // If it is not the last edge of myProfile
                            // create a new wire.
                            newWire = true;
                        }
                        else
                        {
                            // New wire.
                            w = new Wire();
                            b.MakeWire(ref w);
                            wp.Append(w);
                        }
                    }
                }
            }
            exp.Next();
            if (exp.More() && newWire)
            {
                w = new Wire();
                b.MakeWire(ref w);
                wp.Append(w);
            }
        }

        // In the list of Wires, find edges generating plane or vertical vevo.
        NCollection_List<Shape>.Iterator ite = new();
        Wire curW, nw = new();
        Explorer ew = new();

        for (ite.Initialize(wp); ite.More(); ite.Next())
        {
            curW = ShapeCasts.Wire(ite.Value());
            bool yaModif = false;
            for (ew.Init(curW, ShapeType.Edge); ew.More(); ew.Next())
            {
                Edge ee = ShapeCasts.Edge(ew.Current());
                if (IsVertical(ee) || IsPlanar(ee))
                {
                    yaModif = true;
                    break;
                }
            }

            if (yaModif)
            {
                // Status = 0 for the beginning
                //          3 vertical
                //          2 horizontal
                //          1 other
                int status = 0;

                for (ew.Init(curW, ShapeType.Edge); ew.More(); ew.Next())
                {
                    Edge ee = ShapeCasts.Edge(ew.Current());
                    if (IsVertical(ee))
                    {
                        if (status != 3)
                        {
                            nw = new Wire();
                            b.MakeWire(ref nw);
                            workProf.Append(nw);
                            status = 3;
                        }
                    }
                    else if (IsPlanar(ee))
                    {
                        if (status != 2)
                        {
                            nw = new Wire();
                            b.MakeWire(ref nw);
                            workProf.Append(nw);
                            status = 2;
                        }
                    }
                    else if (status != 1)
                    {
                        nw = new Wire();
                        b.MakeWire(ref nw);
                        workProf.Append(nw);
                        status = 1;
                    }
                    b.Add(nw, ee);
                }
            }
            else
            {
                workProf.Append(curW);
            }
        }

        // connect vertices modified in MapProf;
        // OCCT iterates in hash order; binding each pair does not depend on the order
        foreach (KeyValuePair<Shape, Shape> gilbert in mapVerRefMoved)
        {
            mapProf.Bind(gilbert.Value, gilbert.Key);
        }
    }

    // Cut the spine's edges at the extrema of curvature and at the inflexion points.
    private void PrepareSpine(ref Face workSpine, NCollection_DataMap<Shape, Shape> mapSpine)
    {
        BrepBuilder b = new();
        NCollection_List<Shape> cuts = new();
        NCollection_List<Shape>.Iterator iteCuts = new();

        Surface? s = Tool.Surface(_spine, out Placement l);
        double tolF = Tool.Tolerance(_spine);
        b.MakeFace(ref workSpine, s, l, tolF);

        for (Iterator iteF = new(_spine); iteF.More(); iteF.Next())
        {
            Wire nw = new();
            b.MakeWire(ref nw);
            bool isClosed = iteF.Value().Closed();

            for (Iterator iteW = new(iteF.Value()); iteW.More(); iteW.Next())
            {
                Edge e = ShapeCasts.Edge(iteW.Value());
                EdgeVertices(e, out Vertex v1, out Vertex v2);
                mapSpine.Bind(v1, v1);
                mapSpine.Bind(v2, v2);
                cuts.Clear();

                // Cut
                CutEdge(e, _spine, cuts);

                if (cuts.IsEmpty())
                {
                    b.Add(nw, e);
                    mapSpine.Bind(e, e);
                }
                else
                {
                    for (iteCuts.Initialize(cuts); iteCuts.More(); iteCuts.Next())
                    {
                        Edge ne = ShapeCasts.Edge(iteCuts.Value());
                        b.Add(nw, ne);
                        mapSpine.Bind(ne, e);
                        EdgeVertices(ne, out v1, out v2);
                        if (!mapSpine.IsBound(v1))
                        {
                            mapSpine.Bind(v1, e);
                        }
                        if (!mapSpine.IsBound(v2))
                        {
                            mapSpine.Bind(v2, e);
                        }
                    }
                }
            }
            nw.Closed(isClosed);
            b.Add(workSpine, nw);
        }

        // Construct curves 3D of the spine
        BrepLibrary.BuildCurves3d(workSpine);
    }

    /// <summary>The top face (or compound of faces) of a solid result (frozen with the result).</summary>
    public Shape Top()
    {
        FreezeResult();
        return _top;
    }

    /// <summary>The bottom face (or compound of faces) of a solid result (frozen with the result).</summary>
    public Shape Bottom()
    {
        FreezeResult();
        return _bottom;
    }

    /// <summary>
    /// The shapes generated from a sub-shape <paramref name="spineShape"/> of the spine and a sub-shape
    /// <paramref name="profShape"/> of the profile (frozen, with the result).
    /// </summary>
    public ISchematiqList<Shape> GeneratedShapes(in Shape spineShape, in Shape profShape)
    {
        FreezeResult();
        NCollection_List<Shape> list = InternalGeneratedShapes(spineShape, profShape);
        if (list.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        var shapes = new Shape[list.Extent()];
        int i = 0;
        foreach (Shape s in list)
        {
            shapes[i] = s;
            shapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(shapes);
    }

    /// <summary>OCCT's <c>GeneratedShapes</c>: the algorithm's own list (shared: don't modify), not frozen.</summary>
    internal NCollection_List<Shape> InternalGeneratedShapes(in Shape spineShape, in Shape profShape)
    {
        if (_map.IsBound(spineShape) && _map[spineShape].IsBound(profShape))
        {
            return _map[spineShape][profShape];
        }
        else
        {
            return new NCollection_List<Shape>();
        }
    }

    private NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> Generated() => _map;

    private static Orientation Compare(in Edge e1, in Edge e2)
    {
        Orientation oo = Forward;
        Exploration.Vertices(e1, out Vertex v10, out Vertex _);
        Exploration.Vertices(e2, out Vertex v20, out Vertex v21);
        Point3D p1 = Tool.Pnt(v10);
        Point3D p2 = Tool.Pnt(v20);
        Point3D p3 = Tool.Pnt(v21);
        if (p1.Distance(p3) < p1.Distance(p2))
        {
            oo = Reversed;
        }

        return oo;
    }

    private void Add(Evolved vevo, in Wire prof, Quilt glue)
    {
        NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> mapVevo = vevo.Generated();
        Shape currentSpine, currentProf;

        if (vevo.InternalShape().IsNull)
        {
            return;
        }

        //-------------------------------------------------
        // Find wires common to <me> and <Vevo>.
        //-------------------------------------------------

        Explorer exProf = new();
        for (exProf.Init(prof, ShapeType.Vertex); exProf.More(); exProf.Next())
        {
            Shape vv = exProf.Current();
            //---------------------------------------------------------------
            // Parse edges generated by VV in myMap if they existent
            // and Bind in Glue
            //---------------------------------------------------------------

            //------------------------------------------------- -------------
            // Note: the curves of of reinforced edges are in the same direction
            //          if one remains on the same edge.
            //          if one passes from left to the right they are inverted.
            //------------------------------------------------- -------------
            Relative(_profile, prof, ShapeCasts.Vertex(vv), out bool commun);

            if (commun)
            {
                // OCCT iterates myMap in hash order; the bindings made do not depend on it
                foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in _map)
                {
                    Shape sp = iteS.Key;
                    if (iteS.Value.IsBound(vv) && mapVevo.IsBound(sp) && mapVevo[sp].IsBound(vv))
                    {
                        NCollection_List<Shape> myList = _map[sp][vv];
                        NCollection_List<Shape> vevoList = vevo.InternalGeneratedShapes(sp, vv);
                        NCollection_List<Shape>.Iterator myIte = new(myList);
                        NCollection_List<Shape>.Iterator vevoIte = new(vevoList);
                        for (; myIte.More(); myIte.Next(), vevoIte.Next())
                        {
                            Edge me = ShapeCasts.Edge(myIte.Value());
                            Edge ve = ShapeCasts.Edge(vevoIte.Value());
                            Orientation og = Compare(me, ve);
                            Shape localShape = ve.Oriented(Forward);
                            Shape localShape2 = me.Oriented(og);
                            glue.Bind(ShapeCasts.Edge(localShape), ShapeCasts.Edge(localShape2));
                        }
                    }
                }
            }
        }
        glue.Add(vevo.InternalShape());

        //----------------------------------------------------------
        // Add map of elements generate in Vevo in myMap.
        //----------------------------------------------------------
        foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in mapVevo)
        {
            currentSpine = iteS.Key;
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> iteP in iteS.Value)
            {
                currentProf = iteP.Key;
                if (!_map.IsBound(currentSpine))
                {
                    //------------------------------------------------
                    // The element of spine is not yet present .
                    // => previous profile not on the border.
                    //-------------------------------------------------
                    _map.Bind(currentSpine, NewProfMap());
                }
                if (!_map[currentSpine].IsBound(currentProf))
                {
                    _map[currentSpine].Bind(currentProf, new NCollection_List<Shape>());
                    NCollection_List<Shape> genShapes = iteP.Value;
                    NCollection_List<Shape>.Iterator itl = new(genShapes);
                    for (; itl.More(); itl.Next())
                    {
                        // during Glue.Add the shared shapes are recreated.
                        if (glue.IsCopied(itl.Value()))
                        {
                            _map[currentSpine][currentProf].Append(glue.InternalCopy(itl.Value()));
                        }
                        else
                        {
                            _map[currentSpine][currentProf].Append(itl.Value());
                        }
                    }
                }
            }
        }
    }

    private ref Shape ChangeShape() => ref _shape;

    private void Transfert(Evolved vevo,
                           NCollection_DataMap<Shape, Shape> mapProf,
                           NCollection_DataMap<Shape, Shape> mapSpine,
                           in Placement ls,
                           in Placement initLS,
                           in Placement initLP)
    {
        //----------------------------------------------------------------
        // Transfer the shape from Vevo in myShape and Reposition shapes.
        //----------------------------------------------------------------
        _shape = vevo.InternalShape();
        _spine.Location(initLS);
        _profile.Location(initLP);
        _shape.Move(ls);

        //
        // Expecting for better, the Same Parameter is forced here
        //  ( Pb Sameparameter between YaPlanar and Tuyaux
        //
        BrepBuilder b = new();
        Explorer ex = new(_shape, ShapeType.Edge);
        while (ex.More())
        {
            b.SameRange(ShapeCasts.Edge(ex.Current()), false);
            b.SameParameter(ShapeCasts.Edge(ex.Current()), false);
            BrepLibrary.SameParameter(ShapeCasts.Edge(ex.Current()));
            ex.Next();
        }

        //--------------------------------------------------------------
        // Transfer of myMap of Vevo into myMap.
        //--------------------------------------------------------------
        Shape initialSpine, initialProf;

        NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> mapVevo = vevo.Generated();

        // OCCT iterates MapVevo in hash order: the keys of myMap are bound in another order, which only changes
        // the order of later iterations over myMap (none in a published result)
        foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in mapVevo)
        {
            initialSpine = mapSpine[iteS.Key];
            initialSpine.Move(ls);

            foreach (KeyValuePair<Shape, NCollection_List<Shape>> iteP in iteS.Value)
            {
                initialProf = mapProf[iteP.Key];
                initialProf.Location(initLP);

                NCollection_List<Shape> genShapes = iteP.Value;

                NCollection_List<Shape>.Iterator itl = new();
                for (itl.Initialize(genShapes); itl.More(); itl.Next())
                {
                    itl.ChangeValue().Move(ls);
                }

                if (!_map.IsBound(initialSpine))
                {
                    _map.Bind(initialSpine, NewProfMap());
                }

                if (!_map[initialSpine].IsBound(initialProf))
                {
                    _map[initialSpine].Bind(initialProf, new NCollection_List<Shape>());
                }
                _map[initialSpine][initialProf].Append(genShapes);
            }
        }
        //--------------------------------------------------------------
        // Transfer of Top and Bottom of Vevo in myTop and myBottom.
        //--------------------------------------------------------------
        _top = vevo._top;
        _top.Move(ls);
        _bottom = vevo._bottom;
        _bottom.Move(ls);
    }

    /// <summary>True once the shape is built.</summary>
    public bool IsDone => _isDone;

    /// <summary>The generated shape (frozen).</summary>
    public Shape Shape()
    {
        FreezeResult();
        return _shape;
    }

    /// <summary>OCCT's <c>Shape</c>: the result, not frozen, for ported code that goes on building.</summary>
    internal Shape InternalShape() => _shape;

    private void FreezeResult()
    {
        _shape.TShape()?.FreezeGraph();
        _top.TShape()?.FreezeGraph();
        _bottom.TShape()?.FreezeGraph();
    }

    /// <summary>The join type of the parallels.</summary>
    public global::Kerf.Geometry.JoinKind JoinType() => _joinType;

    private void AddTopAndBottom(Quilt glue)
    {
        //  return first and last vertex of the profile.
        Vertex[] v = [new(), new()];
        Exploration.Vertices(_profile, out v[0], out v[1]);
        if (v[0].IsSame(v[1]))
        {
            return;
        }

        NCollection_List<Shape>.Iterator itL = new();
        bool toReverse = false;
        for (int i = 0; i <= 1; i++)
        {
            Loop loop = new();
            // Construction of supports.
            Plane3D s = new(0.0, 0.0, 1.0, -Altitud(v[i]));
            Face F = new LibraryMakeFace(s).InternalFace();
            loop.Init(F);

            Explorer expSpine = new(_spine, ShapeType.Edge);
            HashSet<Shape> view = new(ShapeHasher.Instance);

            for (; expSpine.More(); expSpine.Next())
            {
                Edge es = ShapeCasts.Edge(expSpine.Current());
                NCollection_List<Shape> L = InternalGeneratedShapes(es, v[i]);
                bool computeOrientation = false;

                for (itL.Initialize(L); itL.More(); itL.Next())
                {
                    Edge e = ShapeCasts.Edge(itL.Value());

                    if (!computeOrientation)
                    {
                        EdgeCurveAdaptor c1 = new(es);
                        EdgeCurveAdaptor c2 = new(e);
                        Tool.Range(e, out double f, out double l);
                        Tool.Range(es, out double fs, out double ls);
                        double u = 0.3 * f + 0.7 * l;
                        double us = 0.3 * fs + 0.7 * ls;
                        c1.D1(us, out Point3D _, out Vector3D v1);
                        c2.D1(u, out Point3D _, out Vector3D v2);
                        toReverse = (v1.Dot(v2) < 0.0);
                        computeOrientation = true;
                    }

                    Orientation or = es.Orientation();
                    if (toReverse)
                    {
                        or = global::Kerf.Engine.Topology.TopAbs.Reverse(or);
                    }
                    Shape localShape = e.Oriented(or);
                    loop.AddConstEdge(ShapeCasts.Edge(localShape));
                }
            }

            Point3D pv = Tool.Pnt(v[i]);
            bool isOut = pv.Y() < 0;

            for (expSpine.Init(_spine, ShapeType.Vertex); expSpine.More(); expSpine.Next())
            {
                Vertex es = ShapeCasts.Vertex(expSpine.Current());
                if (view.Add(es))
                {
                    NCollection_List<Shape> L = InternalGeneratedShapes(es, v[i]);
                    for (itL.Initialize(L); itL.More(); itL.Next())
                    {
                        Edge e = ShapeCasts.Edge(itL.Value());
                        if (!Tool.Degenerated(e))
                        {
                            // the center of circle (ie vertex) is IN the cap if vertex IsOut
                            //                                    OUT                   !IsOut
                            EdgeCurveAdaptor c = new(e);
                            Tool.Range(e, out double f, out double l);
                            double u = 0.3 * f + 0.7 * l;
                            Point3D p = Tool.Pnt(es);
                            c.D1(u, out Point3D pc, out Vector3D vc);
                            Vector3D ppc = new(p, pc);
                            Vector3D prod = ppc.Crossed(vc);
                            if (isOut)
                            {
                                toReverse = prod.Z() < 0.0;
                            }
                            else
                            {
                                toReverse = prod.Z() > 0.0;
                            }
                            Orientation or = Forward;
                            if (toReverse)
                            {
                                or = Reversed;
                            }
                            Shape localShape = e.Oriented(or);
                            loop.AddConstEdge(ShapeCasts.Edge(localShape));
                        }
                    }
                }
            }

            loop.Perform();
            loop.WiresToFaces();
            NCollection_List<Shape> lf = loop.NewFaces();
            NCollection_List<Shape>.Iterator iterL = new(lf);

            // Maj of myTop and myBottom for the history
            // and addition of constructed faces.
            Compound bouchon = new();
            BrepBuilder b = new();
            b.MakeCompound(ref bouchon);
            int j = 0;

            for (iterL.Initialize(lf); iterL.More(); iterL.Next())
            {
                j++;
                glue.Add(iterL.Value());
                if (j == 1 && i == 0)
                {
                    _top = iterL.Value();
                }
                if (j == 1 && i == 1)
                {
                    _bottom = iterL.Value();
                }
                b.Add(bouchon, iterL.Value());
            }
            if (i == 0 && j > 1)
            {
                _top = bouchon;
            }
            if (i == 1 && j > 1)
            {
                _bottom = bouchon;
            }
        }
    }

    private void MakeSolid()
    {
        Explorer exp = new(_shape, ShapeType.Shell);
        int ish = 0;
        Compound res = new();
        Solid sol = new();
        BrepBuilder b = new();
        b.MakeCompound(ref res);

        for (; exp.More(); exp.Next())
        {
            Shape sh = exp.Current();
            sol = new Solid();
            b.MakeSolid(ref sol);
            b.Add(sol, sh);
            SolidClassifier sc = new(sol);
            sc.PerformInfinitePoint(BRepFill_Confusion());
            if (sc.State() == In)
            {
                sol = new Solid();
                b.MakeSolid(ref sol);
                b.Add(sol, sh.Reversed());
            }
            b.Add(res, sol);
            ish++;
        }
        if (ish == 1)
        {
            _shape = sol;
        }
        else
        {
            _shape = res;
        }
    }

    private void MakePipe(in Edge se, in CoordinateSystem axeRef)
    {
        WireExplorer profExp = new();

        Transform3D trsf = new();
        if (Side(_profile, BRepFill_Confusion()) > 3)
        { // side right
            trsf = trsf.WithRotation(Constants.OZ(), Math.PI);
        }
        Placement dumLoc = new(trsf);
        Shape localShape = _profile.Moved(dumLoc);
        Wire dummyProf = PutProfilAt(ShapeCasts.Wire(localShape), axeRef, se, _spine, true);

        // Copy of the profile to avoid the accumulation of
        // locations on the Edges of myProfile!

        TrsfModification trsfMod = new(new Transform3D());
        Modifier modif = new(dummyProf, trsfMod);

        Wire genProf = ShapeCasts.Wire(modif.InternalModifiedShape(dummyProf));

        Pipe pipe = new(new LibraryMakeWire(se).InternalWire(), genProf);

        //---------------------------------------------
        // Arrangement of Tubes in myMap.
        //---------------------------------------------

        WireExplorer genProfExp = new();
        bool firstVertex = true;

        _map.Bind(se, NewProfMap());

        for (profExp.Init(_profile), genProfExp.Init(genProf); profExp.More(); profExp.Next(), genProfExp.Next())
        {
            EdgeVertices(profExp.Current(), out Vertex vf, out Vertex vl);
            EdgeVertices(genProfExp.Current(), out Vertex vfg, out Vertex vlg);

            if (firstVertex)
            {
                _map[se].Bind(vf, new NCollection_List<Shape>());
                _map[se][vf].Append(pipe.InternalEdge(se, vfg));
                firstVertex = false;
            }
            _map[se].Bind(vl, new NCollection_List<Shape>());
            _map[se][vl].Append(pipe.InternalEdge(se, vlg));
            _map[se].Bind(profExp.Current(), new NCollection_List<Shape>());
            _map[se][profExp.Current()].Append(pipe.InternalFace(se, genProfExp.Current()));
        }
    }

    private void MakeRevol(in Edge se, in Vertex vLast, in CoordinateSystem axeRef)
    {
        WireExplorer profExp = new();

        Transform3D trsf = new();
        if (Side(_profile, BRepFill_Confusion()) > 3)
        { // side right
            trsf = trsf.WithRotation(Constants.OZ(), Math.PI);
        }
        Placement dumLoc = new(trsf);
        Shape localShape = _profile.Moved(dumLoc);
        Wire genProf = PutProfilAt(ShapeCasts.Wire(localShape), axeRef, se, _spine, false);

        Axis3D axeRev = new(Tool.Pnt(vLast), Constants.DZ().Reversed());

        // Position of the sewing on the edge of the spine
        // so that the bissectrices didn't cross the sewings.
        Transform3D dummy = new Transform3D().WithRotation(axeRev, 1.5 * Math.PI);
        Placement dummyLoc = new(dummy);
        genProf.Move(dummyLoc);

        Revol rev = new(genProf, axeRev, true);

        //--------------------------------------------
        // Arrangement of revolutions in myMap.
        //---------------------------------------------
        WireExplorer genProfExp = new();
        bool firstVertex = true;

        _map.Bind(vLast, NewProfMap());

        for (profExp.Init(_profile), genProfExp.Init(genProf); profExp.More(); profExp.Next(), genProfExp.Next())
        {
            EdgeVertices(profExp.Current(), out Vertex vf, out Vertex vl);
            EdgeVertices(genProfExp.Current(), out Vertex vfg, out Vertex vlg);

            Orientation or = genProfExp.Current().Orientation();

            if (firstVertex)
            {
                _map[vLast].Bind(vf, new NCollection_List<Shape>());
                Shape rv0 = rev.Shape(vfg);
                Orientation oo0 = rv0.Orientation();
                _map[vLast][vf].Append(rv0.Oriented(oo0));
                firstVertex = false;
            }
            _map[vLast].Bind(profExp.Current(), new NCollection_List<Shape>());
            Shape rf = rev.Shape(genProfExp.Current());
            Orientation oo = global::Kerf.Engine.Topology.TopAbs.Compose(rf.Orientation(), or);

            _map[vLast][profExp.Current()].Append(rf.Oriented(oo));
            _map[vLast].Bind(vl, new NCollection_List<Shape>());
            Shape rv = rev.Shape(vlg);
            oo = rv.Orientation();
            _map[vLast][vl].Append(rv.Oriented(oo));
        }
    }

    private static Placement FindLocation(in Face face)
    {
        Surface s = Tool.Surface(face, out Placement l)!;

        if (s is not PlaneSurface)
        {
            FindSurface fs = new(face, -1, true);
            if (fs.Found())
            {
                s = fs.Surface()!;
                l = fs.Location();
            }
            else
            {
                throw new NoSuchObjectException("BRepFill_Evolved : The Face is not planar");
            }
        }

        if (!l.IsIdentity)
        {
            s = (Surface)s.Transformed(l.Transformation());
        }

        PlaneSurface p = (PlaneSurface)s;
        CoordinateSystem axis = p.Position();

        Transform3D t = new();
        CoordinateSystem axeRef = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        t = t.WithTransformation(axeRef, axis);

        return new Placement(t);
    }

    // Moves the handles mySpine and myProfile (the caller's shapes are not changed).
    private void TransformInitWork(in Placement ls, in Placement lp)
    {
        _spine.Move(ls);
        _profile.Move(lp);
    }

    // Coding of regularities on edges parallel to CutVevo common to left and right parts of volevo.
    private void ContinuityOnOffsetEdge(NCollection_List<Shape> workProf)
    {
        WireExplorer wExp = new();
        Vertex vf, vl, v;
        Edge precE, curE = new(), firstE;
        BrepBuilder b = new();

        wExp.Init(_profile);
        firstE = wExp.Current();
        precE = firstE;
        EdgeVertices(firstE, out vf, out v);
        if (wExp.More())
        {
            wExp.Next();
        }

        for (; wExp.More(); wExp.Next())
        {
            curE = wExp.Current();
            v = wExp.CurrentVertex();

            if (DistanceToOZ(v) <= BRepFill_Confusion())
            {
                // the regularities are already coded on the edges of elementary volevos
                double u1 = Tool.Parameter(v, curE);
                double u2 = Tool.Parameter(v, precE);
                EdgeCurveAdaptor curve1 = new(curE);
                EdgeCurveAdaptor curve2 = new(precE);
                Continuity continuity = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(curve1, curve2, u1, u2);

                if (continuity >= global::Kerf.Geometry.Continuity.G1)
                {
                    //-----------------------------------------------------
                    // Code continuity for all edges generated by V.
                    //-----------------------------------------------------
                    foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in _map)
                    {
                        Shape sp = iteS.Key;
                        if (_map[sp].IsBound(v) && _map[sp].IsBound(curE) && _map[sp].IsBound(precE))
                        {
                            if (!_map[sp][v].IsEmpty() && !_map[sp][curE].IsEmpty() && !_map[sp][precE].IsEmpty())
                            {
                                b.Continuity(ShapeCasts.Edge(_map[sp][v].First()),
                                             ShapeCasts.Face(_map[sp][curE].First()),
                                             ShapeCasts.Face(_map[sp][precE].First()),
                                             continuity);
                            }
                        }
                    }
                }
            }
            precE = curE;
        }

        EdgeVertices(precE, out v, out vl);

        if (vf.IsSame(vl))
        {
            // Closed profile.
            double u1 = Tool.Parameter(vf, curE);
            double u2 = Tool.Parameter(vf, firstE);
            EdgeCurveAdaptor curve1 = new(curE);
            EdgeCurveAdaptor curve2 = new(firstE);
            Continuity continuity = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(curve1, curve2, u1, u2);

            if (continuity >= global::Kerf.Geometry.Continuity.G1)
            {
                //---------------------------------------------
                // Code continuity for all edges generated by V.
                //---------------------------------------------
                foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in _map)
                {
                    Shape sp = iteS.Key;
                    if (_map[sp].IsBound(vf) && _map[sp].IsBound(curE) && _map[sp].IsBound(firstE))
                    {
                        if (!_map[sp][vf].IsEmpty() && !_map[sp][curE].IsEmpty() && !_map[sp][firstE].IsEmpty())
                        {
                            b.Continuity(ShapeCasts.Edge(_map[sp][vf].First()),
                                         ShapeCasts.Face(_map[sp][curE].First()),
                                         ShapeCasts.Face(_map[sp][firstE].First()),
                                         continuity);
                        }
                    }
                }
            }
        }
    }

    // Degenerated edges can be missing in some face: the missing degenerated edges have vertices corresponding to node
    // of the map. Now it is enough to compare points UV of vertices on edges with a certain tolerance.
    private static void AddDegeneratedEdge(in Face F, in Wire w)
    {
        Surface s = Tool.Surface(F, out Placement _)!;
        if (s.GetType() == typeof(TrimmedSurface))
        {
            Surface sb = ((TrimmedSurface)s).BasisSurface();
            if (sb.GetType() == typeof(PlaneSurface))
            {
                return;
            }
        }

        if (s.GetType() == typeof(PlaneSurface))
        {
            return;
        }

        BrepBuilder b = new();
        double tolConf = 1.0e-4;

        bool change = true;

        while (change)
        {
            change = false;
            WireExplorer we = new(w, F);
            Point2D pf = new(), prevP = new(), p1, p2 = new();
            Vertex vf = new(), v1, v2 = new();

            for (; we.More(); we.Next())
            {
                Edge ce = we.Current();
                EdgeVertices(ce, out v1, out v2);
                if (ce.Orientation() == Reversed)
                {
                    Tool.UVPoints(ce, F, out p2, out p1);
                }
                else
                {
                    Tool.UVPoints(ce, F, out p1, out p2);
                }
                if (vf.IsNull)
                {
                    vf = v1;
                    pf = p1;
                }
                else
                {
                    if (!p1.IsEqual(prevP, tolConf))
                    {
                        // degenerated edge to be inserted.
                        change = true;
                        Vector2D v = new(prevP, p1);
                        LineCurve2D c2d = new(prevP, new Direction2D(v));
                        double f = 0, l = prevP.Distance(p1);
                        Edge ne = new LibraryMakeEdge(c2d, s).InternalEdge();
                        b.Degenerated(ne, true);
                        b.Add(ne, v1.Oriented(Forward));
                        b.Add(ne, v1.Oriented(Reversed));
                        b.Range(ne, f, l);
                        b.Add(w, ne);
                        break;
                    }
                }
                prevP = p2;
            }
            if (!change && vf.IsSame(v2))
            { // closed
                if (!pf.IsEqual(p2, tolConf))
                {
                    // Degenerated edge to be inserted.
                    change = true;
                    Vector2D v = new(p2, pf);
                    LineCurve2D c2d = new(p2, new Direction2D(v));
                    double f = 0, l = p2.Distance(pf);
                    Edge ne = new LibraryMakeEdge(c2d, s).InternalEdge();
                    b.Degenerated(ne, true);
                    b.Add(ne, vf.Oriented(Forward));
                    b.Add(ne, vf.Oriented(Reversed));
                    b.Range(ne, f, l);
                    b.Add(w, ne);
                }
            }
        }
    }

    private static void TrimFace(in Face face, NCollection_Sequence<Shape> theEdges, NCollection_Sequence<Shape> s)
    {
        //--------------------------------------
        // Creation of wires limiting faces.
        //--------------------------------------
        BrepBuilder theBuilder = new();

        int nbEdges;
        bool newWire;
        bool addEdge;
        Wire goodWire;

        while (!theEdges.IsEmpty())
        {
            LibraryMakeWire mWire = new(ShapeCasts.Edge(theEdges.First()));
            goodWire = mWire.InternalWire();
            theEdges.Remove(1);
            nbEdges = theEdges.Length();
            newWire = false;

            while (!newWire)
            {
                addEdge = false;

                for (int i = 1; i <= nbEdges && !addEdge; i++)
                {
                    Edge e = ShapeCasts.Edge(theEdges.Value(i));
                    if (Tool.Degenerated(e))
                    {
                        theEdges.Remove(i);
                        addEdge = true;
                        nbEdges = theEdges.Length();
                        goodWire = mWire.InternalWire();
                    }
                    else
                    {
                        mWire.Add(e);
                        if (mWire.Error() == LibraryWireError.WireDone)
                        {
                            // the connection is successful
                            // it is removed from the sequence and one restarts from the beginning.
                            theEdges.Remove(i);
                            addEdge = true;
                            nbEdges = theEdges.Length();
                            goodWire = mWire.InternalWire();
                        }
                    }
                }
                newWire = (!addEdge);
            }
            Shape localShape = face.EmptyCopied();
            Face faceCut = ShapeCasts.Face(localShape);
            faceCut.Orientation(Forward);
            BrepTools.Update(faceCut);
            AddDegeneratedEdge(faceCut, goodWire);
            theBuilder.Add(faceCut, goodWire);
            faceCut.Orientation(face.Orientation());
            s.Append(faceCut);
        }
    }

    private static Wire PutProfilAt(in Wire profRef, in CoordinateSystem axeRef, in Edge e, in Face f, bool atStart)
    {
        Vector2D d1;
        Point2D p;
        Wire prof;

        Curve2D? c2d = Tool.CurveOnSurface(e, f, out double first, out double last);
        if (c2d is null)
        {
            throw new ConstructionErrorException("ConstructionError in PutProfilAt");
        }

        if (e.Orientation() == Reversed)
        {
            if (!atStart)
            {
                c2d.D1(first, out p, out d1);
            }
            else
            {
                c2d.D1(last, out p, out d1);
            }
            d1 = d1.Reversed();
        }
        else
        {
            if (!atStart)
            {
                c2d.D1(last, out p, out d1);
            }
            else
            {
                c2d.D1(first, out p, out d1);
            }
        }
        Point3D p3d = new(p.X(), p.Y(), 0.0);
        Vector3D v3d = new(d1.X(), d1.Y(), 0.0);

        CoordinateSystem ax = new(p3d, Constants.DZ(), new Direction3D(v3d));
        Transform3D trans = new Transform3D().WithTransformation(ax, axeRef);
        Shape localShape = profRef.Moved(new Placement(trans));
        prof = ShapeCasts.Wire(localShape);
        return prof;
    }

    private static void TrimEdge(in Edge edge,
                                 NCollection_Sequence<Shape> theEdgesControle,
                                 NCollection_Sequence<Shape> theVer,
                                 NCollection_Sequence<double> thePar,
                                 NCollection_Sequence<Shape> s)
    {
        bool change = true;
        BrepBuilder theBuilder = new();
        s.Clear();
        //------------------------------------------------------------
        // Parse two sequences depending on the parameter on the edge.
        //------------------------------------------------------------
        while (change)
        {
            change = false;
            for (int i = 1; i < thePar.Length(); i++)
            {
                if (thePar.Value(i) > thePar.Value(i + 1))
                {
                    thePar.Exchange(i, i + 1);
                    theVer.Exchange(i, i + 1);
                    change = true;
                }
            }
        }

        //----------------------------------------------------------
        // If a vertex is not in the proofing point, it is removed.
        //----------------------------------------------------------
        if (!Tool.Degenerated(edge))
        {
            for (int k = 1; k <= theVer.Length(); k++)
            {
                if (DoubleOrNotInFace(theEdgesControle, ShapeCasts.Vertex(theVer.Value(k))))
                {
                    theVer.Remove(k);
                    thePar.Remove(k);
                    k--;
                }
            }
        }

        //-------------------------------------------------------------------
        // Processing of double vertices for non-degenerated edges.
        // If a vertex_double appears twice in the edges of control,
        // the vertex is eliminated .
        // otherwise its only representation is preserved.
        //-------------------------------------------------------------------
        if (!Tool.Degenerated(edge))
        {
            for (int k = 1; k < theVer.Length(); k++)
            {
                if (theVer.Value(k).IsSame(theVer.Value(k + 1)))
                {
                    theVer.Remove(k + 1);
                    thePar.Remove(k + 1);
                    if (DoubleOrNotInFace(theEdgesControle, ShapeCasts.Vertex(theVer.Value(k))))
                    {
                        theVer.Remove(k);
                        thePar.Remove(k);
                    }
                    k--;
                }
            }
        }

        //-----------------------------------------------------------
        // Creation of edges.
        // the number of vertices should be even. The edges to be created leave
        // from a vertex with uneven index i to vertex i+1;
        //-----------------------------------------------------------
        for (int k = 1; k < theVer.Length(); k = k + 2)
        {
            Shape localShape = edge.EmptyCopied();
            Edge newEdge = ShapeCasts.Edge(localShape);

            if (newEdge.Orientation() == Reversed)
            {
                theBuilder.Add(newEdge, theVer.Value(k).Oriented(Reversed));
                theBuilder.Add(newEdge, theVer.Value(k + 1).Oriented(Forward));
            }
            else
            {
                theBuilder.Add(newEdge, theVer.Value(k).Oriented(Forward));
                theBuilder.Add(newEdge, theVer.Value(k + 1).Oriented(Reversed));
            }
            theBuilder.Range(newEdge, thePar.Value(k), thePar.Value(k + 1));
            //  modified by NIZHNY-EAP Wed Dec 22 12:09:48 1999 ___BEGIN___
            BrepLibrary.UpdateTolerances(newEdge, false);
            //  modified by NIZHNY-EAP Wed Dec 22 13:34:19 1999 ___END___
            s.Append(newEdge);
        }
    }

    private static void ComputeIntervals(NCollection_Sequence<Shape> vOnF,
                                         NCollection_Sequence<Shape> vOnL,
                                         NCollection_Sequence<Point3D> parOnF,
                                         NCollection_Sequence<Point3D> parOnL,
                                         TrimSurfaceTool trim,
                                         Curve2D bis,
                                         in Vertex vs,
                                         in Vertex ve,
                                         NCollection_Sequence<double> firstPar,
                                         NCollection_Sequence<double> lastPar,
                                         NCollection_Sequence<Shape> firstV,
                                         NCollection_Sequence<Shape> lastV)
    {
        int iOnF = 1, iOnL = 1;
        double u1 = 0.0, u2;
        Shape v1 = new(), v2;

        if (!vs.IsNull)
        {
            u1 = bis.FirstParameter();
            v1 = vs;
        }
        while (iOnF <= vOnF.Length() || iOnL <= vOnL.Length())
        {
            //---------------------------------------------------------
            // Return the smallest parameter on the bissectrice
            // corresponding to the current positions IOnF,IOnL.
            //---------------------------------------------------------
            if (iOnL > vOnL.Length() || (iOnF <= vOnF.Length() && parOnF.Value(iOnF).X() < parOnL.Value(iOnL).X()))
            {
                u2 = parOnF.Value(iOnF).X();
                v2 = vOnF.Value(iOnF);
                iOnF++;
            }
            else
            {
                u2 = parOnL.Value(iOnL).X();
                v2 = vOnL.Value(iOnL);
                iOnL++;
            }
            //---------------------------------------------------------------------
            // When V2 and V1 are different the medium point P of the
            // interval is tested compared to the face. If P is in the face the interval
            // is valid.
            //---------------------------------------------------------------------
            if (!v1.IsNull && !v2.IsSame(v1))
            {
                Point2D p = bis.Value((u2 + u1) * 0.5);
                if (trim.IsOnFace(p))
                {
                    firstPar.Append(u1);
                    lastPar.Append(u2);
                    firstV.Append(v1);
                    lastV.Append(v2);
                }
            }
            u1 = u2;
            v1 = v2;
        }

        if (!ve.IsNull)
        {
            u2 = bis.LastParameter();
            v2 = ve;
            if (!v2.IsSame(v1))
            {
                Point2D p = bis.Value((u2 + u1) * 0.5);
                if (trim.IsOnFace(p))
                {
                    firstPar.Append(u1);
                    lastPar.Append(u2);
                    firstV.Append(v1);
                    lastV.Append(v2);
                }
            }
        }
    }

    // Commun is true if two wires have V in common; returns FORWARD if the wires near the vertex are at the same side,
    // otherwise REVERSED.
    private static Orientation Relative(in Wire w1, in Wire w2, in Vertex v, out bool commun)
    {
        Explorer exp = new();
        Edge e1 = new(), e2 = new();

        for (exp.Init(w1, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
            if (v1.IsSame(v) || v2.IsSame(v))
            {
                e1 = e;
                break;
            }
        }
        for (exp.Init(w2, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
            if (v1.IsSame(v) || v2.IsSame(v))
            {
                e2 = e;
                break;
            }
        }

        if (e1.IsNull || e2.IsNull)
        {
            commun = false;
            return Forward;
        }
        commun = true;

        Wire ww1 = new LibraryMakeWire(e1).InternalWire();
        Wire ww2 = new LibraryMakeWire(e2).InternalWire();
        double tol = BRepFill_Confusion();
        if (Side(ww1, tol) < 4 && Side(ww2, tol) < 4)
        { // two to the left
            return Forward;
        }
        if (Side(ww1, tol) > 4 && Side(ww2, tol) > 4)
        { // two to the right
            return Forward;
        }

        return Reversed;
    }

    // The position of the point defined by d1 in the face defined by d2 d3:
    // 0 out of the face, 1 on the edge of d2, 2 inside the face, 3 on the edge of d3.
    private static int PosOnFace(double d1, double d2, double d3)
    {
        if (Math.Abs(d1 - d2) <= BRepFill_Confusion())
        {
            return 1;
        }
        if (Math.Abs(d1 - d3) <= BRepFill_Confusion())
        {
            return 3;
        }

        if (d2 < d3)
        {
            if (d1 > (d2 + BRepFill_Confusion()) && d1 < (d3 - BRepFill_Confusion()))
            {
                return 2;
            }
        }
        else
        {
            if (d1 > (d3 + BRepFill_Confusion()) && d1 < (d2 - BRepFill_Confusion()))
            {
                return 2;
            }
        }
        return 0;
    }

    // True if V appears zero or two times in the sequence of edges EC.
    private static bool DoubleOrNotInFace(NCollection_Sequence<Shape> ec, in Vertex v)
    {
        bool vu = false;

        for (int i = 1; i <= ec.Length(); i++)
        {
            Exploration.Vertices(ShapeCasts.Edge(ec.Value(i)), out Vertex v1, out Vertex v2);
            if (v1.IsSame(v))
            {
                if (vu)
                {
                    return true;
                }
                else
                {
                    vu = true;
                }
            }
            if (v2.IsSame(v))
            {
                if (vu)
                {
                    return true;
                }
                else
                {
                    vu = true;
                }
            }
        }
        return !vu;
    }

    private static double DistanceToOZ(in Vertex v)
    {
        Point3D pv3d = Tool.Pnt(v);
        return Math.Abs(pv3d.Y());
    }

    private static double Altitud(in Vertex v)
    {
        Point3D pv3d = Tool.Pnt(v);
        return pv3d.Z();
    }

    // OCCT hands on the locus's own trimmed curve (Bisector_Bisec::Value) unless its basis is a BisecAna; the port does
    // the same (BRepFill_TrimSurfaceTool may lengthen it in place, as natively).
    private static void SimpleExpression(Bisec b, out Curve2D bis)
    {
        bis = b.Value();

        if (bis.GetType() == typeof(TrimmedCurve2D))
        {
            TrimmedCurve2D trBis = (TrimmedCurve2D)bis;
            Curve2D basBis = trBis.ChangeBasisCurve();
            if (basBis.GetType() == typeof(BisecAna))
            {
                bis = ((BisecAna)basBis).Geom2dCurve();
                bis = new TrimmedCurve2D(bis, trBis.FirstParameter(), trBis.LastParameter());
            }
        }
    }

    // Projection and Cut of an edge at extrema of distance to axis OZ.
    private static void CutEdgeProf(in Edge e,
                                    PlaneSurface plane,
                                    LineCurve2D line,
                                    NCollection_List<Shape> cuts,
                                    NCollection_DataMap<Shape, Shape> mapVerRefMoved)
    {
        cuts.Clear();

        Curve3D c;
        TrimmedCurve3D ct;
        Curve2D c2d;

        // Return the curve associated to each Edge
        c = Tool.Curve(e, out Placement L, out double f, out double l)!;
        ct = new TrimmedCurve3D(c, f, l);
        ct.Transform(L.Transformation());

        // project it in the plane and return the associated PCurve
        Direction3D normal = plane.Pln().Axis().Direction();
        c = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(ct, plane, normal, false)!;
        c2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, plane)!;

        // Calculate the extrema with the straight line
        NCollection_Sequence<double> seq = new();

        double u1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double u2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        f = c2d.FirstParameter();
        l = c2d.LastParameter();

        Box2d b = new();
        CurveAdaptor2D ac2d = new(c2d);
        Add2dCurve.Add(ac2d, BRepFill_Confusion(), ref b);
        b.Get(out double _, out u1, out double _, out u2);

        //  modified by NIZHNY-EAP Wed Feb  2 16:32:37 2000 ___BEGIN___
        // no sense if C2 is normal to Line or really is a point
        if (u1 != u2)
        {
            ExtremaCurveCurve2D extrema = new(line, c2d, u1 - 1.0, u2 + 1.0, f, l);

            int nb = extrema.NbExtrema();
            for (int i = 1; i <= nb; i++)
            {
                extrema.Parameters(i, out u1, out u2);
                seq.Append(u2);
            }
        }
        //  modified by NIZHNY-EAP Wed Feb  2 16:33:05 2000 ___END___

        // On calcule les intersection avec Oy.
        CurveAdaptor2D aLine = new(line);
        double tol = global::Kerf.Engine.Foundation.Precision.Intersection();
        double tolC = 0.0;

        GInter intersector = new(aLine, ac2d, tolC, tol);
        int nbP = intersector.PointCount;

        for (int i = 1; i <= nbP; i++)
        {
            seq.Append(intersector.Point(i).ParamOnSecond());
        }

        // Compute the new edges.
        BrepBuilder builder = new();
        Vertex vv, vf = new(), vl = new();
        Exploration.Vertices(e, out Vertex vRf, out Vertex vRl);

        if (!mapVerRefMoved.IsBound(vRf))
        {
            builder.MakeVertex(ref vf, c.Value(f), Tool.Tolerance(vRf));
            mapVerRefMoved.Bind(vRf, vf);
        }
        else
        {
            vf = ShapeCasts.Vertex(mapVerRefMoved[vRf]);
        }

        if (!mapVerRefMoved.IsBound(vRl))
        {
            builder.MakeVertex(ref vl, c.Value(l), Tool.Tolerance(vRl));
            mapVerRefMoved.Bind(vRl, vl);
        }
        else
        {
            vl = ShapeCasts.Vertex(mapVerRefMoved[vRl]);
        }

        if (!seq.IsEmpty())
        {
            Bubble(seq);

            bool empty = false;

            double curParam = f;
            double param;

            while (!empty)
            {
                param = seq.First();
                seq.Remove(1);
                empty = seq.IsEmpty();
                if (Math.Abs(param - curParam) > BRepFill_Confusion() && Math.Abs(param - l) > BRepFill_Confusion())
                {
                    vv = new LibraryMakeVertex(c.Value(param)).InternalVertex();

                    Edge ee0 = new LibraryMakeEdge(c, vf, vv).InternalEdge();
                    ee0.Orientation(e.Orientation());
                    if (ee0.Orientation() == Forward)
                    {
                        cuts.Append(ee0);
                    }
                    else
                    {
                        cuts.Prepend(ee0);
                    }

                    // reinitialize
                    curParam = param;
                    vf = vv;
                }
            }
        }

        Edge ee = new LibraryMakeEdge(c, vf, vl).InternalEdge();
        ee.Orientation(e.Orientation());
        if (ee.Orientation() == Forward)
        {
            cuts.Append(ee);
        }
        else
        {
            cuts.Prepend(ee);
        }
    }

    // Cut an edge at the extrema of curves and at points of inflexion. Closed circles are also cut in two. If <Cuts> are
    // empty the edge is not modified. The first and the last vertex of the original edge belong to the first and last
    // parts respectively.
    private static void CutEdge(in Edge e, in Face F, NCollection_List<Shape> cuts)
    {
        cuts.Clear();
        CutCurve cuter = new();
        TrimmedCurve2D ct2d;

        Vertex vf, vl;
        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
        BrepBuilder b = new();

        Curve2D c2d = Tool.CurveOnSurface(e, F, out double f, out double l)!;
        ct2d = new TrimmedCurve2D(c2d, f, l);

        if (ct2d.ChangeBasisCurve() is CircleCurve2D && Tool.IsClosed(e))
        {
            //---------------------------
            // Cut closed circle.
            //---------------------------
            double m1 = (2 * f + l) / 3.0;
            double m2 = (f + 2 * l) / 3.0;
            Point2D p1 = ct2d.Value(m1);
            Point2D p2 = ct2d.Value(m2);

            Vertex vl1 = new LibraryMakeVertex(new Point3D(p1.X(), p1.Y(), 0.0)).InternalVertex();
            Vertex vl2 = new LibraryMakeVertex(new Point3D(p2.X(), p2.Y(), 0.0)).InternalVertex();
            Shape localShape1 = e.EmptyCopied();
            Shape localShape2 = e.EmptyCopied();
            Shape localShape3 = e.EmptyCopied();
            Edge fe = ShapeCasts.Edge(localShape1);
            Edge me = ShapeCasts.Edge(localShape2);
            Edge le = ShapeCasts.Edge(localShape3);

            fe.Orientation(Forward);
            me.Orientation(Forward);
            le.Orientation(Forward);

            b.Add(fe, v1);
            b.Add(fe, vl1.Oriented(Reversed));
            b.Range(fe, f, m1);

            b.Add(me, vl1.Oriented(Forward));
            b.Add(me, vl2.Oriented(Reversed));
            b.Range(me, m1, m2);

            b.Add(le, vl2.Oriented(Forward));
            b.Add(le, v2);
            b.Range(le, m2, l);

            cuts.Append(fe.Oriented(e.Orientation()));
            cuts.Append(me.Oriented(e.Orientation()));
            cuts.Append(le.Oriented(e.Orientation()));
            //--------
            // Return.
            //--------
            return;
        }

        //-------------------------
        // Cut of the curve.
        //-------------------------
        cuter.Perform(ct2d);

        if (cuter.UnModified())
        {
            //-----------------------------
            // edge not modified => return.
            //-----------------------------
            return;
        }
        else
        {
            //------------------------
            // Creation of cut edges.
            //------------------------
            vf = v1;

            for (int k = 1; k <= cuter.CurveCount; k++)
            {
                TrimmedCurve2D cc = cuter.Value(k);
                if (k == cuter.CurveCount)
                {
                    vl = v2;
                }
                else
                {
                    Point2D p = cc.Value(cc.LastParameter());
                    vl = new LibraryMakeVertex(new Point3D(p.X(), p.Y(), 0.0)).InternalVertex();
                }
                Shape localShape = e.EmptyCopied();
                Edge ne = ShapeCasts.Edge(localShape);
                ne.Orientation(Forward);
                b.Add(ne, vf.Oriented(Forward));
                b.Add(ne, vl.Oriented(Reversed));
                b.Range(ne, cc.FirstParameter(), cc.LastParameter());
                cuts.Append(ne.Oriented(e.Orientation()));
                vf = vl;
            }
        }
    }

    // Tests the position of aNode relative to the distances to OZ of the vertices VF and VL, and returns it; if it is not
    // 0, the vertex VN corresponding to aNode is created (once per node and shape).
    private static int VertexFromNode(MedialNode aNode,
                                      in Edge e,
                                      in Vertex vf,
                                      in Vertex vl,
                                      NCollection_DataMap<MedialNode, NCollection_DataMap<Shape, Shape>> mapNodeVertex,
                                      ref Vertex vn)
    {
        Shape shapeOnNode = new();
        int status = 0;
        BrepBuilder b = new();

        if (!aNode.Infinite())
        {
            status = PosOnFace(aNode.Distance(), DistanceToOZ(vf), DistanceToOZ(vl));
        }
        if (status == 2)
        {
            shapeOnNode = e;
        }
        else if (status == 1)
        {
            shapeOnNode = vf;
        }
        else if (status == 3)
        {
            shapeOnNode = vl;
        }

        if (!shapeOnNode.IsNull)
        {
            //-------------------------------------------------
            // the vertex will correspond to a node of the map
            //-------------------------------------------------
            if (mapNodeVertex.IsBound(aNode) && mapNodeVertex[aNode].IsBound(shapeOnNode))
            {
                vn = ShapeCasts.Vertex(mapNodeVertex[aNode][shapeOnNode]);
            }
            else
            {
                vn = new Vertex();
                b.MakeVertex(ref vn);
                if (!mapNodeVertex.IsBound(aNode))
                {
                    mapNodeVertex.Bind(aNode, new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance));
                }
                mapNodeVertex[aNode].Bind(shapeOnNode, vn);
            }
        }
        return status;
    }
}
