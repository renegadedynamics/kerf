// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Evolved.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Evolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Occt.MAT_Side;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

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
public sealed class BRepFill_Evolved
{
    private Face mySpine = new();
    private Wire myProfile = new();
    private Shape myShape = new();
    private bool myIsDone;
#pragma warning disable CS0414 // OCCT sets the spine type without reading it
    private bool mySpineType;
#pragma warning restore CS0414
    private global::Kerf.Geometry.JoinKind myJoinType;
    private readonly NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> myMap =
        new(ShapeHasher.Instance);
    private Shape myTop = new();
    private Shape myBottom = new();

    private static double BRepFill_Confusion()
    {
        double Tol = 1.0e-6;
        return Tol;
    }

    private static NCollection_DataMap<Shape, NCollection_List<Shape>> NewProfMap() => new(ShapeHasher.Instance);

    private static void EdgeVertices(in Edge E, out Vertex V1, out Vertex V2)
    {
        if (E.Orientation() == Reversed)
        {
            Exploration.Vertices(E, out V2, out V1);
        }
        else
        {
            Exploration.Vertices(E, out V1, out V2);
        }
    }

    /// <summary>An empty algorithm.</summary>
    public BRepFill_Evolved()
    {
        myIsDone = false;
        mySpineType = true;
    }

    /// <summary>
    /// The evolved shape of <paramref name="Profile"/> swept along <paramref name="Spine"/>. <paramref name="AxeProf"/>
    /// places the profile on the spine: it slides on the profile with its direction along the normal to the spine and
    /// its X direction along the spine's tangent.
    /// </summary>
    public BRepFill_Evolved(in Wire Spine, in Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool Solid = false)
    {
        myIsDone = false;
        Perform(Spine, Profile, AxeProf, Join, Solid);
    }

    /// <summary>The evolved shape of <paramref name="Profile"/> swept along the face <paramref name="Spine"/>.</summary>
    public BRepFill_Evolved(in Face Spine, in Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool Solid = false)
    {
        myIsDone = false;
        Perform(Spine, Profile, AxeProf, Join, Solid);
    }

    private static bool IsVertical(in Edge E)
    {
        Exploration.Vertices(E, out Vertex V1, out Vertex V2);
        Point3D P1 = Tool.Pnt(V1);
        Point3D P2 = Tool.Pnt(V2);

        if (Math.Abs(P1.Y() - P2.Y()) < BRepFill_Confusion())
        {
            // It is a Line ?
            Curve3D? GC = Tool.Curve(E, out Placement _, out double _, out double _);
            if (GC!.GetType() == typeof(LineCurve3D))
            {
                return true;
            }
        }
        return false;
    }

    private static bool IsPlanar(in Edge E)
    {
        Exploration.Vertices(E, out Vertex V1, out Vertex V2);
        Point3D P1 = Tool.Pnt(V1);
        Point3D P2 = Tool.Pnt(V2);

        if (Math.Abs(P1.Z() - P2.Z()) < BRepFill_Confusion())
        {
            // It is a Line ?
            Curve3D? GC = Tool.Curve(E, out Placement _, out double _, out double _);
            if (GC!.GetType() == typeof(LineCurve3D))
            {
                return true;
            }
        }
        return false;
    }

    // Determines the position of the profile relative to the plane XOZ:
    // 1 MAT_Left, 2 MAT_Left and planar, 3 MAT_Left and vertical,
    // 4 MAT_Right, 5 MAT_Right and planar, 6 MAT_Right and vertical.
    private static int Side(in Wire Profil, double Tol)
    {
        // Rem : it is enough to test the first edge of the Wire.
        //       ( Correctly cut in PrepareProfil)
        Explorer Explo = new(Profil, ShapeType.Edge);

        int TheSide;
        Edge E = ShapeCasts.Edge(Explo.Current());

        Exploration.Vertices(E, out Vertex V1, out Vertex V2);
        Point3D P1 = Tool.Pnt(V1);
        Point3D P2 = Tool.Pnt(V2);

        if (P1.Y() < -Tol || P2.Y() < -Tol)
        {
            TheSide = 4;
        }
        else
        {
            TheSide = 1;
        }
        if (IsVertical(E))
        {
            TheSide += 2;
        }
        else if (IsPlanar(E))
        {
            TheSide++;
        }
        return TheSide;
    }

    /// <summary>Builds the evolved shape of <paramref name="Profile"/> swept along the wire <paramref name="Spine"/>.</summary>
    public void Perform(in Wire Spine, in Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool Solid = false)
    {
        mySpineType = false;
        Face aFace = new BRepLib_MakeFace(Spine, true).InternalFace();
        PrivatePerform(aFace, Profile, AxeProf, Join, Solid);
    }

    /// <summary>Builds the evolved shape of <paramref name="Profile"/> swept along the face <paramref name="Spine"/>.</summary>
    public void Perform(in Face Spine, in Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool Solid = false)
    {
        mySpineType = true;
        PrivatePerform(Spine, Profile, AxeProf, Join, Solid);
    }

    // OCCT.NET: the copy at entry (decisions 642-644). OCCT writes into the spine's own edges and vertices: the
    // parallels at distance 0 (BRepFill_OffsetWire's Compute) and the uncut spine edges go into the result as they
    // are, and Transfert, BRepLib.UpdateTolerances and the quilt write them (SameRange, SameParameter, tolerances,
    // pcurves). A spine or profile holding frozen TShapes (a caller's) is copied exactly, the algorithm runs on the
    // copies, and afterwards the caller's own sub-shapes come back wherever the algorithm left their copies unchanged
    // (in the result, Top, Bottom and the generated lists), and the generated shapes are keyed by the caller's
    // sub-shapes.
    private void PrivatePerform(in Face Spine, in Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join, bool Solid)
    {
        CopyAtEntry aSpineEntry = new(Spine);
        CopyAtEntry aProfEntry = new(Profile);
        PrivatePerformInPlace(ShapeCasts.Face(aSpineEntry.Shape()), ShapeCasts.Wire(aProfEntry.Shape()), AxeProf, Join, Solid);
        if (!aSpineEntry.IsCopied && !aProfEntry.IsCopied)
        {
            return;
        }

        foreach (CopyAtEntry anEntry in new[] { aProfEntry, aSpineEntry })
        {
            List<Shape> anOthers = [myTop, myBottom];
            foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> aBySpine in myMap)
            {
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> aByProf in aBySpine.Value)
                {
                    anOthers.AddRange(aByProf.Value);
                }
            }
            myShape = anEntry.Finish(myShape, anOthers);
            myTop = anEntry.Final(myTop);
            myBottom = anEntry.Final(myBottom);
            foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> aBySpine in myMap)
            {
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> aByProf in aBySpine.Value)
                {
                    for (NCollection_List<Shape>.Iterator anIt = new(aByProf.Value); anIt.More(); anIt.Next())
                    {
                        anIt.ChangeValue() = anEntry.Final(anIt.Value());
                    }
                }
            }
        }

        // the keys: the caller's sub-shapes (spine first, then profile), in the same order
        List<KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>>> anEntries = [.. myMap];
        myMap.Clear();
        foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> aBySpine in anEntries)
        {
            NCollection_DataMap<Shape, NCollection_List<Shape>> aByProf = NewProfMap();
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> anItem in aBySpine.Value)
            {
                aByProf.Bind(aProfEntry.Original(anItem.Key), anItem.Value);
            }
            myMap.Bind(aSpineEntry.Original(aBySpine.Key), aByProf);
        }
        mySpine = ShapeCasts.Face(Spine.Oriented(Forward));
        myProfile = ShapeCasts.Wire(Profile.Oriented(Forward));
    }

    private void PrivatePerformInPlace(in Face Spine, in Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join, bool Solid)
    {
        Shape aLocalShape = Spine.Oriented(Forward);
        mySpine = ShapeCasts.Face(aLocalShape);
        aLocalShape = Profile.Oriented(Forward);
        myProfile = ShapeCasts.Wire(aLocalShape);
        myJoinType = Join;
        myMap.Clear();

        if (myJoinType > Arc)
        {
            throw new MethodNotImplementedException();
        }

        NCollection_List<Shape> WorkProf = new();
        Face WorkSpine = new();
        NCollection_List<Shape>.Iterator WPIte = new();

        //-------------------------------------------------------------------
        // Positioning of mySpine and myProfil in the workspace.
        //-------------------------------------------------------------------
        Placement LSpine = FindLocation(mySpine);
        Transform3D T = new();
        T.SetTransformation(AxeProf);
        Placement LProfile = new(T);
        Placement InitLS = mySpine.Location();
        Placement InitLP = myProfile.Location();
        TransformInitWork(LSpine, LProfile);

        //------------------------------------------------------------------
        // projection of the profile and cut of the spine.
        //------------------------------------------------------------------
        NCollection_DataMap<Shape, Shape> MapProf = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, Shape> MapSpine = new(ShapeHasher.Instance);

        PrepareProfile(WorkProf, MapProf);
        PrepareSpine(ref WorkSpine, MapSpine);

        double Tol = BRepFill_Confusion();
        bool YaLeft = false;
        bool YaRight = false;
        Wire SP;

        for (WPIte.Initialize(WorkProf); WPIte.More(); WPIte.Next())
        {
            SP = ShapeCasts.Wire(WPIte.Value());
            if (Side(SP, Tol) < 4)
            {
                YaLeft = true;
            }
            else
            {
                YaRight = true;
            }
            if (YaLeft && YaRight)
            {
                break;
            }
        }

        Face Face;
        BRepMAT2d_BisectingLocus Locus = new();

        //----------------------------------------------------------
        // Initialisation of cut volevo.
        // For each part of the profile create a volevo added to CutVevo
        //----------------------------------------------------------
        BRepFill_Evolved CutVevo = new();
        Wire WP = new();
        BrepBuilder BB = new();
        WireExplorer WExp = new();

        BB.MakeWire(ref WP);

        for (WPIte.Initialize(WorkProf); WPIte.More(); WPIte.Next())
        {
            for (WExp.Init(ShapeCasts.Wire(WPIte.Value())); WExp.More(); WExp.Next())
            {
                BB.Add(WP, WExp.Current());
            }
        }
        CutVevo.SetWork(WorkSpine, WP);

        Quilt Glue = new();
        int CSide;

        //---------------------------------
        // Construction of vevos to the left.
        //---------------------------------
        if (YaLeft)
        {
            //-----------------------------------------------------
            // Calculate the map of bisector locations at the left.
            // and links Topology -> base elements of the map.
            //-----------------------------------------------------
            BRepMAT2d_Explorer Exp = new(WorkSpine);
            Locus.Compute(Exp, 1, MAT_Left);
            BRepMAT2d_LinkTopoBilo Link = new(Exp, Locus);

            for (WPIte.Initialize(WorkProf); WPIte.More(); WPIte.Next())
            {
                SP = ShapeCasts.Wire(WPIte.Value());
                CSide = Side(SP, Tol);
                //-----------------------------------------------
                // Construction and adding of elementary volevo.
                //-----------------------------------------------
                BRepFill_Evolved Vevo = new();
                if (CSide == 1)
                {
                    Vevo.ElementaryPerform(WorkSpine, SP, Locus, Link, Join);
                }
                else if (CSide == 2)
                {
                    Vevo.PlanarPerform(WorkSpine, SP, Locus, Link, Join);
                }
                else if (CSide == 3)
                {
                    Vevo.VerticalPerform(WorkSpine, SP, Locus, Link, Join);
                }
                CutVevo.Add(Vevo, SP, Glue);
            }
        }

        //---------------------------------
        // Construction of vevos to the right.
        //---------------------------------
        if (YaRight)
        {
            //-----------------------------------
            // Decomposition of the face into wires.
            //-----------------------------------
            Explorer SpineExp = new(WorkSpine, ShapeType.Wire);
            for (; SpineExp.More(); SpineExp.Next())
            {
                //----------------------------------------------
                // Calculate the map to the right of the current wire.
                //----------------------------------------------
                BRepLib_MakeFace B = new(new Plane3D(0.0, 0.0, 1.0, 0.0));
                Shape aLocalShapeRev = SpineExp.Current().Reversed();
                B.Add(ShapeCasts.Wire(aLocalShapeRev));
                Face = B.InternalFace();
                BRepMAT2d_Explorer Exp = new(Face);
                Locus.Compute(Exp, 1, MAT_Left);
                BRepMAT2d_LinkTopoBilo Link = new(Exp, Locus);

                for (WPIte.Initialize(WorkProf); WPIte.More(); WPIte.Next())
                {
                    SP = ShapeCasts.Wire(WPIte.Value());
                    CSide = Side(SP, Tol);
                    //-----------------------------------------------
                    // Construction and adding of an elementary volevo
                    //-----------------------------------------------
                    BRepFill_Evolved Vevo = new();
                    if (CSide == 4)
                    {
                        Vevo.ElementaryPerform(Face, SP, Locus, Link, Join);
                    }
                    else if (CSide == 5)
                    {
                        Vevo.PlanarPerform(Face, SP, Locus, Link, Join);
                    }
                    else if (CSide == 6)
                    {
                        Vevo.VerticalPerform(Face, SP, Locus, Link, Join);
                    }
                    CutVevo.Add(Vevo, SP, Glue);
                }
            }
        }

        if (Solid)
        {
            CutVevo.AddTopAndBottom(Glue);
        }

        //-------------------------------------------------------------------------
        // Gluing of regularites on parallel edges generate4d by vertices of the
        // cut of the profile.
        //-------------------------------------------------------------------------
        CutVevo.ContinuityOnOffsetEdge(WorkProf);

        //-----------------------------------------------------------------
        // construction of the shape via the quilt, ie:
        // - sharing of topologies of elementary added volevos.
        // - Orientation of faces correspondingly to each other.
        //-----------------------------------------------------------------
        CutVevo.ChangeShape() = Glue.InternalShells();
        //------------------------------------------------------------------------
        // Transfer of the map of generated elements and of the shape of Cutvevo
        // in myMap and repositioning in the initial space.
        //------------------------------------------------------------------------
        Transfert(CutVevo, MapProf, MapSpine, LSpine.Inverted(), InitLS, InitLP);

        // Orientation of the solid.
        if (Solid)
        {
            MakeSolid();
        }

        //  modified by NIZHNY-EAP Mon Jan 24 11:26:48 2000 ___BEGIN___
        BRepLib.UpdateTolerances(myShape, false);
        //  modified by NIZHNY-EAP Mon Jan 24 11:26:50 2000 ___END___
        myIsDone = true;
    }

    private static void IsInversed(in Shape S, in Edge E1, in Edge E2, bool[] Inverse)
    {
        Inverse[0] = Inverse[1] = false;
        if (S.ShapeType() != ShapeType.Edge)
        {
            return;
        }

        Point3D P;
        Vector3D DS, DC1, DC2;
        EdgeCurveAdaptor CS = new(ShapeCasts.Edge(S));
        if (S.Orientation() == Forward)
        {
            CS.D1(CS.FirstParameter(), out P, out DS);
        }
        else
        {
            CS.D1(CS.LastParameter(), out P, out DS);
            DS.Reverse();
        }

        if (!Tool.Degenerated(E1))
        {
            EdgeCurveAdaptor C1 = new(ShapeCasts.Edge(E1));
            if (E1.Orientation() == Forward)
            {
                C1.D1(C1.FirstParameter(), out P, out DC1);
            }
            else
            {
                C1.D1(C1.LastParameter(), out P, out DC1);
                DC1.Reverse();
            }
            Inverse[0] = (DS.Dot(DC1) < 0.0);
        }
        else
        {
            Inverse[0] = true;
        }

        if (!Tool.Degenerated(E2))
        {
            EdgeCurveAdaptor C2 = new(ShapeCasts.Edge(E2));
            if (E2.Orientation() == Forward)
            {
                C2.D1(C2.FirstParameter(), out P, out DC2);
            }
            else
            {
                C2.D1(C2.LastParameter(), out P, out DC2);
                DC2.Reverse();
            }
            Inverse[1] = (DS.Dot(DC2) < 0.0);
        }
        else
        {
            Inverse[1] = true;
        }
    }

    private void SetWork(in Face Sp, in Wire Pr)
    {
        mySpine = Sp;
        myProfile = Pr;
    }

    // Determines if the pipes were at the side of the concavity. In this case they can be closed.
    // WARNING: Not finished. Done only for circles.
    private static bool ConcaveSide(in Shape S, in Face F)
    {
        if (S.ShapeType() == ShapeType.Vertex)
        {
            return false;
        }

        if (S.ShapeType() == ShapeType.Edge)
        {
            Curve2D? G2d = Tool.CurveOnSurface(ShapeCasts.Edge(S), F, out double f, out double l);

            CurveAdaptor2D AC = new(G2d!, f, l);
            if (AC.GetType() == Circle)
            {
                bool Direct = AC.Circle().IsDirect;
                if (S.Orientation() == Reversed)
                {
                    Direct = (!Direct);
                }
                return Direct;
            }
        }
        return false;
    }

    private void ElementaryPerform(in Face Sp, in Wire Pr, BRepMAT2d_BisectingLocus Locus, BRepMAT2d_LinkTopoBilo Link,
                                   global::Kerf.Geometry.JoinKind Join)
    {
        Shape aLocalShape = Sp.Oriented(Forward);
        mySpine = ShapeCasts.Face(aLocalShape);
        myProfile = Pr;
        myMap.Clear();

        BrepBuilder myBuilder = new();
        Compound aCompound = new();
        myBuilder.MakeCompound(ref aCompound);
        myShape = aCompound;

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
        NCollection_DataMap<MAT_Node, NCollection_DataMap<Shape, Shape>> MapNodeVertex = new(ReferenceEqualityComparer.Instance);
        NCollection_DataMap<Shape, NCollection_Sequence<Shape>> MapBis = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_Sequence<double>> MapVerPar = new(ShapeHasher.Instance);

        // mark of the profile.
        CoordinateSystem AxeRef = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));

        //---------------------------------------------------------------
        // Construction of revolutions and tubes.
        //---------------------------------------------------------------
        WireExplorer ProfExp = new();
        Explorer FaceExp = new();
        WireExplorer WireExp = new();

        for (FaceExp.Init(mySpine, ShapeType.Wire); FaceExp.More(); FaceExp.Next())
        {
            for (WireExp.Init(ShapeCasts.Wire(FaceExp.Current())); WireExp.More(); WireExp.Next())
            {
                Edge CurrentEdge0 = WireExp.Current();
                EdgeVertices(CurrentEdge0, out Vertex _, out Vertex VLast);

                for (Link.Init(VLast); Link.More(); Link.Next())
                {
                    //----------------------------.
                    // Construction of a Revolution
                    //----------------------------.
                    MakeRevol(CurrentEdge0, VLast, AxeRef);
                }

                for (Link.Init(CurrentEdge0); Link.More(); Link.Next())
                {
                    //------------------------.
                    // Construction of a Tube
                    //-------------------------
                    MakePipe(CurrentEdge0, AxeRef);
                }
            }
        }

        //---------------------------------------------------
        // Construction of edges associated to bissectrices.
        //---------------------------------------------------
        MAT_Arc CurrentArc;
        Curve2D Bis;
        Curve2D? PCurve1, PCurve2;
        Curve3D? CBis;
        Edge CurrentEdge = new();
        Shape[] S = [new(), new()];
        Face[] F = [new(), new()];
        Edge[] E = [new(), new(), new(), new()];
        int k;

        for (int i = 1; i <= Locus.Graph().NumberOfArcs(); i++)
        {
            CurrentArc = Locus.Graph().Arc(i);
            SimpleExpression(Locus.GeomBis(CurrentArc, out bool Reverse), out Bis);

            //------------------------------------------------------------------
            // Return elements of the spine corresponding to separate basicElts.
            //------------------------------------------------------------------
            S[0] = Link.GeneratingShape(CurrentArc.FirstElement());
            S[1] = Link.GeneratingShape(CurrentArc.SecondElement());

            bool Concave0 = ConcaveSide(S[0], mySpine);
            bool Concave1 = ConcaveSide(S[1], mySpine);

            NCollection_Sequence<Shape> VOnF = new(), VOnL = new();
            NCollection_Sequence<Point3D> ParOnF = new(), ParOnL = new();

            NCollection_DataMap<Shape, NCollection_Sequence<Shape>> MapSeqVer = new(ShapeHasher.Instance);
            NCollection_DataMap<Shape, NCollection_Sequence<Point3D>> MapSeqPar = new(ShapeHasher.Instance);

            for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
            {
                //-----------------------------------------------
                // Return two faces separated by the bissectrice.
                //-----------------------------------------------
                F[0] = ShapeCasts.Face(myMap[S[0]][ProfExp.Current()].First());
                F[1] = ShapeCasts.Face(myMap[S[1]][ProfExp.Current()].First());

                //------------------------------------
                // Return parallel edges on each face.
                //------------------------------------
                EdgeVertices(ProfExp.Current(), out Vertex VF, out Vertex VL);

                E[0] = ShapeCasts.Edge(myMap[S[0]][VF].First());
                E[1] = ShapeCasts.Edge(myMap[S[0]][VL].First());
                E[2] = ShapeCasts.Edge(myMap[S[1]][VF].First());
                E[3] = ShapeCasts.Edge(myMap[S[1]][VL].First());

                bool[] Inv0 = new bool[2];
                bool[] Inv1 = new bool[2];

                Inv0[0] = Inv0[1] = Inv1[0] = Inv1[1] = false;
                if (Concave0)
                {
                    IsInversed(S[0], E[0], E[1], Inv0);
                }
                if (Concave1)
                {
                    IsInversed(S[1], E[2], E[3], Inv1);
                }

                //---------------------------------------------
                // Construction of geometries.
                //---------------------------------------------
                BRepFill_TrimSurfaceTool Trim = new(Bis, F[0], F[1], E[0], E[2], Inv0[0], Inv1[0]);
                //-----------------------------------------------------------
                // Construction of vertices corresponding to the node of the map
                //-----------------------------------------------------------
                Vertex VS = new(), VE = new();
                MAT_Node Node1, Node2;

                if (Reverse)
                {
                    Node1 = CurrentArc.SecondNode();
                    Node2 = CurrentArc.FirstNode();
                }
                else
                {
                    Node1 = CurrentArc.FirstNode();
                    Node2 = CurrentArc.SecondNode();
                }
                //--------------------------------------------------------
                // Particular case when the node is on a vertex of the spine.
                //--------------------------------------------------------
                if (Node1.OnBasicElt())
                {
                    if (S[0].ShapeType() == ShapeType.Vertex)
                    {
                        Node1 = CurrentArc.FirstElement().StartArc()!.FirstNode();
                    }
                    else if (S[1].ShapeType() == ShapeType.Vertex)
                    {
                        Node1 = CurrentArc.SecondElement().StartArc()!.FirstNode();
                    }
                }
                // End of particular case.

                int StartOnF = VertexFromNode(Node1, ShapeCasts.Edge(ProfExp.Current()), VF, VL, MapNodeVertex, ref VS);

                int EndOnF = VertexFromNode(Node2, ShapeCasts.Edge(ProfExp.Current()), VF, VL, MapNodeVertex, ref VE);

                //-----------------------------------------------------------
                // Construction of vertices on edges parallel to the spine.
                //-----------------------------------------------------------
                if (!MapSeqVer.IsBound(VF))
                {
                    if (Inv0[0] || Inv1[0])
                    {
                        ParOnF.Clear();
                        VOnF.Clear();
                    }
                    else
                    {
                        Trim.IntersectWith(E[0], E[2], ParOnF);
                        VOnF.Clear();
                        for (int s = 1; s <= ParOnF.Length(); s++)
                        {
                            Vertex VC = new();
                            myBuilder.MakeVertex(ref VC);
                            VOnF.Append(VC);
                        }
                        if (StartOnF == 1)
                        {
                            VOnF.SetValue(1, VS);
                        }
                        if (EndOnF == 1)
                        {
                            VOnF.SetValue(ParOnF.Length(), VE);
                        }
                    }
                }
                else
                {
                    // OCCT assigns copies
                    ParOnF = new NCollection_Sequence<Point3D>(MapSeqPar[VF]);
                    VOnF = new NCollection_Sequence<Shape>(MapSeqVer[VF]);
                }

                if (!MapSeqVer.IsBound(VL))
                {
                    if (Inv0[1] || Inv1[1])
                    {
                        ParOnL.Clear();
                        VOnL.Clear();
                    }
                    else
                    {
                        Trim.IntersectWith(E[1], E[3], ParOnL);
                        VOnL.Clear();
                        for (int s = 1; s <= ParOnL.Length(); s++)
                        {
                            Vertex VC = new();
                            myBuilder.MakeVertex(ref VC);
                            VOnL.Append(VC);
                        }
                        if (StartOnF == 3)
                        {
                            VOnL.SetValue(1, VS);
                        }
                        if (EndOnF == 3)
                        {
                            VOnL.SetValue(ParOnL.Length(), VE);
                        }
                    }
                }
                else
                {
                    ParOnL = new NCollection_Sequence<Point3D>(MapSeqPar[VL]);
                    VOnL = new NCollection_Sequence<Shape>(MapSeqVer[VL]);
                }

                //------------------------------------------------------
                // Test if the Bissectrice is not projected on the face
                //------------------------------------------------------
                if ((StartOnF == 0) && (EndOnF == 0) && VOnL.IsEmpty() && VOnF.IsEmpty())
                {
                    // No trace of the bisectrice on the face.
                    continue;
                }

                if ((StartOnF == 0) && (EndOnF == 0) && (VOnL.Length() + VOnF.Length() == 1))
                {
                    // the first or last node of the arc is on the edge
                    // but the arc is not on the face.
                    continue;
                }

                //---------------------------------------------------------
                // determine the intervals of the bissectrice that are
                // projected on F[0] and F[1].
                //---------------------------------------------------------
                NCollection_Sequence<double> LastPar = new(), FirstPar = new();
                NCollection_Sequence<Shape> FirstV = new(), LastV = new();

                ComputeIntervals(VOnF, VOnL, ParOnF, ParOnL, Trim, Bis, VS, VE, FirstPar, LastPar, FirstV, LastV);

                for (int Ti = 1; Ti <= FirstPar.Length(); Ti++)
                {
                    Vertex V1 = ShapeCasts.Vertex(FirstV.Value(Ti));
                    Vertex V2 = ShapeCasts.Vertex(LastV.Value(Ti));

                    Trim.Project(FirstPar.Value(Ti), LastPar.Value(Ti), out CBis, out PCurve1, out PCurve2, out Continuity Continuity);

                    //-------------------------------------
                    // Coding of the edge.
                    //-------------------------------------
                    myBuilder.MakeEdge(ref CurrentEdge, CBis, BRepFill_Confusion());

                    myBuilder.UpdateVertex(V1, CBis!.Value(CBis.FirstParameter()), BRepFill_Confusion());
                    myBuilder.UpdateVertex(V2, CBis.Value(CBis.LastParameter()), BRepFill_Confusion());

                    myBuilder.Add(CurrentEdge, V1.Oriented(Forward));
                    myBuilder.Add(CurrentEdge, V2.Oriented(Reversed));

                    myBuilder.Range(CurrentEdge, CBis.FirstParameter(), CBis.LastParameter());
                    myBuilder.UpdateEdge(CurrentEdge, PCurve1, F[0], BRepFill_Confusion());
                    myBuilder.UpdateEdge(CurrentEdge, PCurve2, F[1], BRepFill_Confusion());

                    myBuilder.Continuity(CurrentEdge, F[0], F[1], Continuity);

                    //-------------------------------------------
                    // Storage of the edge for each of faces.
                    //-------------------------------------------
                    for (k = 0; k <= 1; k++)
                    {
                        if (!MapBis.IsBound(F[k]))
                        {
                            MapBis.Bind(F[k], new NCollection_Sequence<Shape>());
                        }
                    }
                    //---------------------------------------------------------------
                    // orientation of the edge depends on the direction of the skin.
                    // skin => same orientation E[0] , inverted orientation E[2]
                    // if contreskin it is inverted.
                    //--------------------------------------------------------------
                    E[0].Orientation(BrepTools.OriEdgeInFace(E[0], F[0]));
                    E[2].Orientation(BrepTools.OriEdgeInFace(E[2], F[1]));

                    if (DistanceToOZ(VF) < DistanceToOZ(VL))
                    {
                        // Skin
                        MapBis[F[0]].Append(CurrentEdge.Oriented(E[0].Orientation()));
                        CurrentEdge.Orientation(global::Kerf.Engine.Topology.TopAbs.Complement(E[2].Orientation()));
                        MapBis[F[1]].Append(CurrentEdge);
                    }
                    else
                    {
                        // Contreskin
                        MapBis[F[1]].Append(CurrentEdge.Oriented(E[2].Orientation()));
                        CurrentEdge.Orientation(global::Kerf.Engine.Topology.TopAbs.Complement(E[0].Orientation()));
                        MapBis[F[0]].Append(CurrentEdge);
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
                    if (!MapSeqVer.IsBound(VF))
                    {
                        if (!VOnF.IsEmpty())
                        {
                            if (!MapBis.IsBound(E[k]))
                            {
                                MapBis.Bind(E[k], new NCollection_Sequence<Shape>());
                                MapVerPar.Bind(E[k], new NCollection_Sequence<double>());
                            }
                            for (int ii = 1; ii <= VOnF.Length(); ii++)
                            {
                                MapBis[E[k]].Append(VOnF.Value(ii));
                                if (k == 0)
                                {
                                    MapVerPar[E[k]].Append(ParOnF.Value(ii).Y());
                                }
                                else
                                {
                                    MapVerPar[E[k]].Append(ParOnF.Value(ii).Z());
                                }
                            }
                        }
                    }
                }

                for (k = 1; k <= 3; k = k + 2)
                {
                    if (!MapSeqVer.IsBound(VL))
                    {
                        if (!VOnL.IsEmpty())
                        {
                            if (!MapBis.IsBound(E[k]))
                            {
                                MapBis.Bind(E[k], new NCollection_Sequence<Shape>());
                                MapVerPar.Bind(E[k], new NCollection_Sequence<double>());
                            }
                            for (int ii = 1; ii <= VOnL.Length(); ii++)
                            {
                                MapBis[E[k]].Append(VOnL.Value(ii));
                                if (k == 1)
                                {
                                    MapVerPar[E[k]].Append(ParOnL.Value(ii).Y());
                                }
                                else
                                {
                                    MapVerPar[E[k]].Append(ParOnL.Value(ii).Z());
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
                MapSeqPar.Bind(VF, new NCollection_Sequence<Point3D>(ParOnF));
                MapSeqVer.Bind(VF, new NCollection_Sequence<Shape>(VOnF));
                MapSeqPar.Bind(VL, new NCollection_Sequence<Point3D>(ParOnL));
                MapSeqVer.Bind(VL, new NCollection_Sequence<Shape>(VOnL));
            }
        }

        //----------------------------------
        // Construction of parallel edges.
        //----------------------------------
        Shape CurrentProf = new(), PrecProf = new();
        Face CurrentFace;
        Shape CurrentSpine;
        Vertex VCF = new(), VCL = new();

        // OCCT iterates myMap in its hash order (TShape addresses); the port in insertion order (occt-issues.md)
        foreach (Shape aSpineKey in new List<Shape>(myMap.Keys))
        {
            CurrentSpine = aSpineKey;

            for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
            {
                CurrentProf = ProfExp.Current();
                EdgeVertices(ShapeCasts.Edge(CurrentProf), out VCF, out VCL);
                CurrentEdge = ShapeCasts.Edge(myMap[CurrentSpine][VCF].First());

                //-------------------------------------------------------------
                // RQ : Current Edge is oriented relatively to the face (oriented forward)
                //     generated by edge CurrentProf .
                //-------------------------------------------------------------
                if (MapBis.IsBound(CurrentEdge))
                {
                    //--------------------------------------------------------
                    // Find if one of two faces connected to the edge
                    // belongs to volevo. The edges on this face serve
                    // to eliminate certain vertices that can appear twice
                    // on the parallel edge. These Vertices correspond to the
                    // nodes of the map.
                    //---------------------------------------------------------
                    Shape FaceControle;
                    bool YaFace = true;

                    FaceControle = myMap[CurrentSpine][CurrentProf].First();
                    if (!MapBis.IsBound(FaceControle))
                    {
                        YaFace = false;
                        if (!PrecProf.IsNull)
                        {
                            FaceControle = myMap[CurrentSpine][PrecProf].First();
                            if (MapBis.IsBound(FaceControle))
                            {
                                YaFace = true;
                            }
                        }
                    }

                    if (YaFace)
                    {
                        //------------------------------------------------------------
                        // No connected face in the volevo => no parallel edge.
                        //------------------------------------------------------------
                        NCollection_Sequence<Shape> aSeqOfShape = new();
                        TrimEdge(CurrentEdge, MapBis[FaceControle], MapBis[CurrentEdge], MapVerPar[CurrentEdge], aSeqOfShape);

                        for (k = 1; k <= aSeqOfShape.Length(); k++)
                        {
                            myMap[CurrentSpine][VCF].Append(aSeqOfShape.Value(k));
                        }
                    }
                }
                PrecProf = CurrentProf;
            }

            //------------------------------------------------------------
            // Construction of the parallel edge from the last vertex of myProfile.
            //------------------------------------------------------------
            CurrentEdge = ShapeCasts.Edge(myMap[CurrentSpine][VCL].First());

            if (MapBis.IsBound(CurrentEdge))
            {
                bool YaFace = true;
                Shape FaceControle;

                FaceControle = myMap[CurrentSpine][CurrentProf].First();
                if (!MapBis.IsBound(FaceControle))
                {
                    YaFace = false;
                }
                // the number of element of the list allows to know
                // if the edges have already been done (closed profile) .
                if (YaFace && myMap[CurrentSpine][VCL].Extent() <= 1)
                {
                    NCollection_Sequence<Shape> aSeqOfShape = new();
                    TrimEdge(CurrentEdge, MapBis[FaceControle], MapBis[CurrentEdge], MapVerPar[CurrentEdge], aSeqOfShape);

                    for (k = 1; k <= aSeqOfShape.Length(); k++)
                    {
                        myMap[CurrentSpine][VCL].Append(aSeqOfShape.Value(k));
                    }
                }
            }
        }

        //-------------------------------------------------------------------
        // Cut faces by edges.
        //-------------------------------------------------------------------
        foreach (Shape aSpineKey in new List<Shape>(myMap.Keys))
        {
            CurrentSpine = aSpineKey;

            for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
            {
                CurrentProf = ProfExp.Current();
                CurrentFace = ShapeCasts.Face(myMap[CurrentSpine][CurrentProf].First());
                myMap[CurrentSpine][CurrentProf].Clear();

                if (MapBis.IsBound(CurrentFace))
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
                    EdgeVertices(ShapeCasts.Edge(CurrentProf), out VCF, out VCL);

                    NCollection_List<Shape>.Iterator itl = new();
                    NCollection_List<Shape> LF = myMap[CurrentSpine][VCF];

                    Orientation Ori = BrepTools.OriEdgeInFace(ShapeCasts.Edge(LF.First()), CurrentFace);
                    for (itl.Initialize(LF), itl.Next(); itl.More(); itl.Next())
                    {
                        Edge RE = ShapeCasts.Edge(itl.Value());
                        MapBis[CurrentFace].Append(RE.Oriented(Ori));
                    }
                    NCollection_List<Shape> LL = myMap[CurrentSpine][VCL];
                    Ori = BrepTools.OriEdgeInFace(ShapeCasts.Edge(LL.First()), CurrentFace);
                    for (itl.Initialize(LL), itl.Next(); itl.More(); itl.Next())
                    {
                        Edge RE = ShapeCasts.Edge(itl.Value());
                        MapBis[CurrentFace].Append(RE.Oriented(Ori));
                    }

                    // Cut of the face.
                    NCollection_Sequence<Shape> aSeqOfShape = new();

                    TrimFace(CurrentFace, MapBis[CurrentFace], aSeqOfShape);

                    for (int ii = 1; ii <= aSeqOfShape.Length(); ii++)
                    {
                        myBuilder.Add(myShape, aSeqOfShape.Value(ii));
                        myMap[CurrentSpine][CurrentProf].Append(aSeqOfShape.Value(ii));
                    }
                }
            }
            //-----------------------------------------------------------------
            // Removal of first edge (edge of origin) from lists of myMap
            // corresponding to vertices of the profile.
            //-----------------------------------------------------------------
            Explorer Explo = new(myProfile, ShapeType.Vertex);
            NCollection_Map<Shape> vmap = new(ShapeHasher.Instance);

            for (; Explo.More(); Explo.Next())
            {
                if (vmap.Add(Explo.Current()))
                {
                    myMap[CurrentSpine][Explo.Current()].RemoveFirst();
                }
            }
        }
        myIsDone = true;
    }

    private void PlanarPerform(in Face Sp, in Wire Pr, BRepMAT2d_BisectingLocus Locus, BRepMAT2d_LinkTopoBilo Link,
                               global::Kerf.Geometry.JoinKind Join)
    {
        Shape aLocalShapeOriented = Sp.Oriented(Forward);
        mySpine = ShapeCasts.Face(aLocalShapeOriented);
        myProfile = Pr;
        myMap.Clear();

        BrepBuilder B = new();
        Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        myShape = aCompound;

        WireExplorer ProfExp = new();
        Explorer Exp = new();
        NCollection_DataMap<Shape, Shape> MapVP = new(ShapeHasher.Instance);
        BRepFill_OffsetWire Paral = new();

        for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
        {
            Edge E = ProfExp.Current();
            BRepAlgo_FaceRestrictor FR = new();
            BRepFill_OffsetAncestors OffAnc = new();

            Vertex[] V = [new(), new()];
            EdgeVertices(E, out V[0], out V[1]);
            double Alt = Altitud(V[0]);
            double[] Offset = new double[2];
            Offset[0] = DistanceToOZ(V[0]);
            Offset[1] = DistanceToOZ(V[1]);
            bool IsMinV1 = (Offset[0] < Offset[1]);

            for (int i = 0; i <= 1; i++)
            {
                if (!MapVP.IsBound(V[i]))
                {
                    //------------------------------------------------
                    // Calculate parallel lines corresponding to vertices.
                    //------------------------------------------------
                    Paral.PerformWithBiLo(mySpine, Offset[i], Locus, Link, Join, Alt);
                    OffAnc.Perform(Paral);
                    MapVP.Bind(V[i], Paral.InternalShape());

                    //-----------------------------
                    // Update myMap (.)(V[i])
                    //-----------------------------
                    for (Exp.Init(Paral.InternalShape(), ShapeType.Edge); Exp.More(); Exp.Next())
                    {
                        Edge WC = ShapeCasts.Edge(Exp.Current());
                        Shape GS = OffAnc.Ancestor(WC);
                        if (!myMap.IsBound(GS))
                        {
                            myMap.Bind(GS, NewProfMap());
                        }
                        if (!myMap[GS].IsBound(V[i]))
                        {
                            myMap[GS].Bind(V[i], new NCollection_List<Shape>(Paral.InternalGeneratedShapes(GS)));
                        }
                    }
                }
                Shape Rest = MapVP[V[i]];

                bool ToReverse = false;
                if ((IsMinV1 && (i == 1)) || (!IsMinV1 && (i == 0)))
                {
                    ToReverse = true;
                }

                if (!Rest.IsNull)
                {
                    if (Rest.ShapeType() == ShapeType.Wire)
                    {
                        if (ToReverse)
                        {
                            Shape aLocalShape = Rest.Reversed();
                            Wire aWire = ShapeCasts.Wire(aLocalShape);
                            FR.Add(aWire);
                        }
                        else
                        {
                            FR.Add(ShapeCasts.Wire(Rest));
                        }
                    }
                    else
                    {
                        for (Exp.Init(Rest, ShapeType.Wire); Exp.More(); Exp.Next())
                        {
                            Wire WCop = ShapeCasts.Wire(Exp.Current());
                            if (ToReverse)
                            {
                                Shape aLocalShape = WCop.Reversed();
                                Wire bWire = ShapeCasts.Wire(aLocalShape);
                                FR.Add(bWire);
                            }
                            else
                            {
                                FR.Add(WCop);
                            }
                        }
                    }
                }
            }

            //----------------------------------------------------
            // Construction of faces limited by parallels.
            // - set to the height of the support face.
            //----------------------------------------------------
            Transform3D T = new();
            T.SetTranslation(new Vector3D(0, 0, Alt));
            Placement LT = new(T);
            Shape aLocalShape2 = mySpine.Moved(LT);
            FR.Init(ShapeCasts.Face(aLocalShape2));
            // the restrictor runs on the wires this algorithm made (5O1's notes)
            FR.InternalPerform();

            for (; FR.More(); FR.Next())
            {
                Face Fc = FR.InternalCurrent();
                B.Add(myShape, Fc);
                //---------------------------------------
                // Update myMap(.)(E)
                //---------------------------------------
                for (Exp.Init(Fc, ShapeType.Edge); Exp.More(); Exp.Next())
                {
                    Edge CE = ShapeCasts.Edge(Exp.Current());
                    if (OffAnc.HasAncestor(CE))
                    {
                        Shape InitE = OffAnc.Ancestor(CE);
                        if (!myMap.IsBound(InitE))
                        {
                            myMap.Bind(InitE, NewProfMap());
                        }
                        if (!myMap[InitE].IsBound(E))
                        {
                            myMap[InitE].Bind(E, new NCollection_List<Shape>());
                        }
                        myMap[InitE][E].Append(Fc);
                    }
                }
            }
        } // End loop on profile.
    }

    private void VerticalPerform(in Face Sp, in Wire Pr, BRepMAT2d_BisectingLocus Locus, BRepMAT2d_LinkTopoBilo Link,
                                 global::Kerf.Geometry.JoinKind Join)
    {
        Shape aLocalShape = Sp.Oriented(Forward);
        mySpine = ShapeCasts.Face(aLocalShape);
        myProfile = Pr;
        myMap.Clear();

        BrepBuilder B = new();
        Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        myShape = aCompound;

        WireExplorer ProfExp = new();
        Explorer Exp = new();
        BRepFill_OffsetWire Paral = new();
        BRepFill_OffsetAncestors OffAnc = new();

        bool First = true;
        Shape Base = new();

        for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
        {
            Edge E = ProfExp.Current();
            EdgeVertices(E, out Vertex V1, out Vertex V2);
            double Alt1 = Altitud(V1);
            double Alt2 = Altitud(V2);

            if (First)
            {
                double Offset = DistanceToOZ(V1);
                if (Math.Abs(Offset) < BRepFill_Confusion())
                {
                    Offset = 0.0;
                }
                Paral.PerformWithBiLo(mySpine, Offset, Locus, Link, Join, Alt1);
                OffAnc.Perform(Paral);
                Base = Paral.InternalShape();

                // MAJ myMap
                for (Exp.Init(Base, ShapeType.Edge); Exp.More(); Exp.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(Exp.Current());
                    Shape AE = OffAnc.Ancestor(anEdge);
                    if (!myMap.IsBound(AE))
                    {
                        myMap.Bind(AE, NewProfMap());
                    }
                    if (!myMap[AE].IsBound(V1))
                    {
                        myMap[AE].Bind(V1, new NCollection_List<Shape>());
                    }
                    myMap[AE][V1].Append(anEdge);
                }
                First = false;
            }

            BRepSweep_Prism PS = new(Base, new Vector3D(0, 0, Alt2 - Alt1), false);

            Base = PS.LastShape();

            for (Exp.Init(PS.Shape(), ShapeType.Face); Exp.More(); Exp.Next())
            {
                B.Add(myShape, Exp.Current());
            }

            // MAJ myMap
            foreach (Shape aKey in new List<Shape>(myMap.Keys))
            {
                NCollection_List<Shape> LOF = myMap[aKey][V1];
                NCollection_List<Shape>.Iterator itLOF = new(LOF);
                if (!myMap[aKey].IsBound(V2))
                {
                    myMap[aKey].Bind(V2, new NCollection_List<Shape>());
                }

                if (!myMap[aKey].IsBound(E))
                {
                    myMap[aKey].Bind(E, new NCollection_List<Shape>());
                }

                for (; itLOF.More(); itLOF.Next())
                {
                    Shape OS = itLOF.Value();
                    myMap[aKey][V2].Append(PS.LastShape(OS));
                    myMap[aKey][E].Append(PS.Shape(OS));
                }
            }
        }
    }

    // Order the sequence of points by growing x.
    private static void Bubble(NCollection_Sequence<double> Seq)
    {
        bool Invert = true;
        int NbPoints = Seq.Length();

        while (Invert)
        {
            Invert = false;
            for (int i = 1; i < NbPoints; i++)
            {
                if (Seq.Value(i + 1) < Seq.Value(i))
                {
                    Seq.Exchange(i, i + 1);
                    Invert = true;
                }
            }
        }
    }

    // - Projection of the profile on the working plane.
    // - Cut of the profile at the extrema of distance from profile to axis Oz.
    // - Isolate vertical and horizontal parts.
    // - Reconstruction of wires starting from cut edges.
    // New wires stored in <WorkProf> are always at the same side of axis OZ or mixed with it.
    private void PrepareProfile(NCollection_List<Shape> WorkProf, NCollection_DataMap<Shape, Shape> MapProf)
    {
        // Supposedly the profile is located so that the only transformation
        // to be carried out is a projection on plane yOz.

        // initialise the projection Plane and the Line to evaluate the extrema.
        PlaneSurface Plane = new(new CoordinateSystem(Constants.YOZ()));
        LineCurve2D Line = new(Constants.OY2d());

        // Map initial vertex -> projected vertex.
        NCollection_DataMap<Shape, Shape> MapVerRefMoved = new(ShapeHasher.Instance);

        Wire W = new();
        BrepBuilder B = new();
        NCollection_List<Shape> WP = new();
        B.MakeWire(ref W);
        WP.Append(W);

        WireExplorer Exp = new(myProfile);

        while (Exp.More())
        {
            NCollection_List<Shape> Cuts = new();
            bool NewWire = false;
            Edge E = ShapeCasts.Edge(Exp.Current());

            // Cut of the edge.
            CutEdgeProf(E, Plane, Line, Cuts, MapVerRefMoved);

            EdgeVertices(E, out Vertex _, out Vertex _);

            if (Cuts.IsEmpty())
            {
                // Neither extrema nor intersections nor vertices on the axis.
                B.Add(W, E);
                MapProf.Bind(E, E);
            }
            else
            {
                while (!Cuts.IsEmpty())
                {
                    Edge NE = ShapeCasts.Edge(Cuts.First());
                    MapProf.Bind(NE, E);
                    EdgeVertices(NE, out Vertex V1, out Vertex V2);
                    if (!MapProf.IsBound(V1))
                    {
                        MapProf.Bind(V1, E);
                    }
                    if (!MapProf.IsBound(V2))
                    {
                        MapProf.Bind(V2, E);
                    }

                    B.Add(W, NE);
                    Cuts.RemoveFirst();

                    if (DistanceToOZ(V2) < BRepFill_Confusion() && DistanceToOZ(V1) > BRepFill_Confusion())
                    {
                        // NE ends on axis OZ => new wire
                        if (Cuts.IsEmpty())
                        {
                            // last part of the current edge
                            // If it is not the last edge of myProfile
                            // create a new wire.
                            NewWire = true;
                        }
                        else
                        {
                            // New wire.
                            W = new Wire();
                            B.MakeWire(ref W);
                            WP.Append(W);
                        }
                    }
                }
            }
            Exp.Next();
            if (Exp.More() && NewWire)
            {
                W = new Wire();
                B.MakeWire(ref W);
                WP.Append(W);
            }
        }

        // In the list of Wires, find edges generating plane or vertical vevo.
        NCollection_List<Shape>.Iterator ite = new();
        Wire CurW, NW = new();
        Explorer EW = new();

        for (ite.Initialize(WP); ite.More(); ite.Next())
        {
            CurW = ShapeCasts.Wire(ite.Value());
            bool YaModif = false;
            for (EW.Init(CurW, ShapeType.Edge); EW.More(); EW.Next())
            {
                Edge EE = ShapeCasts.Edge(EW.Current());
                if (IsVertical(EE) || IsPlanar(EE))
                {
                    YaModif = true;
                    break;
                }
            }

            if (YaModif)
            {
                // Status = 0 for the beginning
                //          3 vertical
                //          2 horizontal
                //          1 other
                int Status = 0;

                for (EW.Init(CurW, ShapeType.Edge); EW.More(); EW.Next())
                {
                    Edge EE = ShapeCasts.Edge(EW.Current());
                    if (IsVertical(EE))
                    {
                        if (Status != 3)
                        {
                            NW = new Wire();
                            B.MakeWire(ref NW);
                            WorkProf.Append(NW);
                            Status = 3;
                        }
                    }
                    else if (IsPlanar(EE))
                    {
                        if (Status != 2)
                        {
                            NW = new Wire();
                            B.MakeWire(ref NW);
                            WorkProf.Append(NW);
                            Status = 2;
                        }
                    }
                    else if (Status != 1)
                    {
                        NW = new Wire();
                        B.MakeWire(ref NW);
                        WorkProf.Append(NW);
                        Status = 1;
                    }
                    B.Add(NW, EE);
                }
            }
            else
            {
                WorkProf.Append(CurW);
            }
        }

        // connect vertices modified in MapProf;
        // OCCT iterates in hash order; binding each pair does not depend on the order
        foreach (KeyValuePair<Shape, Shape> gilbert in MapVerRefMoved)
        {
            MapProf.Bind(gilbert.Value, gilbert.Key);
        }
    }

    // Cut the spine's edges at the extrema of curvature and at the inflexion points.
    private void PrepareSpine(ref Face WorkSpine, NCollection_DataMap<Shape, Shape> MapSpine)
    {
        BrepBuilder B = new();
        NCollection_List<Shape> Cuts = new();
        NCollection_List<Shape>.Iterator IteCuts = new();

        Surface? S = Tool.Surface(mySpine, out Placement L);
        double TolF = Tool.Tolerance(mySpine);
        B.MakeFace(ref WorkSpine, S, L, TolF);

        for (Iterator IteF = new(mySpine); IteF.More(); IteF.Next())
        {
            Wire NW = new();
            B.MakeWire(ref NW);
            bool IsClosed = IteF.Value().Closed();

            for (Iterator IteW = new(IteF.Value()); IteW.More(); IteW.Next())
            {
                Edge E = ShapeCasts.Edge(IteW.Value());
                EdgeVertices(E, out Vertex V1, out Vertex V2);
                MapSpine.Bind(V1, V1);
                MapSpine.Bind(V2, V2);
                Cuts.Clear();

                // Cut
                CutEdge(E, mySpine, Cuts);

                if (Cuts.IsEmpty())
                {
                    B.Add(NW, E);
                    MapSpine.Bind(E, E);
                }
                else
                {
                    for (IteCuts.Initialize(Cuts); IteCuts.More(); IteCuts.Next())
                    {
                        Edge NE = ShapeCasts.Edge(IteCuts.Value());
                        B.Add(NW, NE);
                        MapSpine.Bind(NE, E);
                        EdgeVertices(NE, out V1, out V2);
                        if (!MapSpine.IsBound(V1))
                        {
                            MapSpine.Bind(V1, E);
                        }
                        if (!MapSpine.IsBound(V2))
                        {
                            MapSpine.Bind(V2, E);
                        }
                    }
                }
            }
            NW.Closed(IsClosed);
            B.Add(WorkSpine, NW);
        }

        // Construct curves 3D of the spine
        BRepLib.BuildCurves3d(WorkSpine);
    }

    /// <summary>The top face (or compound of faces) of a solid result (frozen with the result).</summary>
    public Shape Top()
    {
        FreezeResult();
        return myTop;
    }

    /// <summary>The bottom face (or compound of faces) of a solid result (frozen with the result).</summary>
    public Shape Bottom()
    {
        FreezeResult();
        return myBottom;
    }

    /// <summary>
    /// The shapes generated from a sub-shape <paramref name="SpineShape"/> of the spine and a sub-shape
    /// <paramref name="ProfShape"/> of the profile (frozen, with the result).
    /// </summary>
    public ISchematiqList<Shape> GeneratedShapes(in Shape SpineShape, in Shape ProfShape)
    {
        FreezeResult();
        NCollection_List<Shape> aList = InternalGeneratedShapes(SpineShape, ProfShape);
        if (aList.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        var aShapes = new Shape[aList.Extent()];
        int i = 0;
        foreach (Shape aS in aList)
        {
            aShapes[i] = aS;
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(aShapes);
    }

    /// <summary>OCCT's <c>GeneratedShapes</c>: the algorithm's own list (shared: don't modify), not frozen.</summary>
    internal NCollection_List<Shape> InternalGeneratedShapes(in Shape SpineShape, in Shape ProfShape)
    {
        if (myMap.IsBound(SpineShape) && myMap[SpineShape].IsBound(ProfShape))
        {
            return myMap[SpineShape][ProfShape];
        }
        else
        {
            return new NCollection_List<Shape>();
        }
    }

    private NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> Generated() => myMap;

    private static Orientation Compare(in Edge E1, in Edge E2)
    {
        Orientation OO = Forward;
        Exploration.Vertices(E1, out Vertex V10, out Vertex _);
        Exploration.Vertices(E2, out Vertex V20, out Vertex V21);
        Point3D P1 = Tool.Pnt(V10);
        Point3D P2 = Tool.Pnt(V20);
        Point3D P3 = Tool.Pnt(V21);
        if (P1.Distance(P3) < P1.Distance(P2))
        {
            OO = Reversed;
        }

        return OO;
    }

    private void Add(BRepFill_Evolved Vevo, in Wire Prof, Quilt Glue)
    {
        NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> MapVevo = Vevo.Generated();
        Shape CurrentSpine, CurrentProf;

        if (Vevo.InternalShape().IsNull)
        {
            return;
        }

        //-------------------------------------------------
        // Find wires common to <me> and <Vevo>.
        //-------------------------------------------------

        Explorer ExProf = new();
        for (ExProf.Init(Prof, ShapeType.Vertex); ExProf.More(); ExProf.Next())
        {
            Shape VV = ExProf.Current();
            //---------------------------------------------------------------
            // Parse edges generated by VV in myMap if they existent
            // and Bind in Glue
            //---------------------------------------------------------------

            //------------------------------------------------- -------------
            // Note: the curves of of reinforced edges are in the same direction
            //          if one remains on the same edge.
            //          if one passes from left to the right they are inverted.
            //------------------------------------------------- -------------
            Relative(myProfile, Prof, ShapeCasts.Vertex(VV), out bool Commun);

            if (Commun)
            {
                // OCCT iterates myMap in hash order; the bindings made do not depend on it
                foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in myMap)
                {
                    Shape SP = iteS.Key;
                    if (iteS.Value.IsBound(VV) && MapVevo.IsBound(SP) && MapVevo[SP].IsBound(VV))
                    {
                        NCollection_List<Shape> MyList = myMap[SP][VV];
                        NCollection_List<Shape> VevoList = Vevo.InternalGeneratedShapes(SP, VV);
                        NCollection_List<Shape>.Iterator MyIte = new(MyList);
                        NCollection_List<Shape>.Iterator VevoIte = new(VevoList);
                        for (; MyIte.More(); MyIte.Next(), VevoIte.Next())
                        {
                            Edge ME = ShapeCasts.Edge(MyIte.Value());
                            Edge VE = ShapeCasts.Edge(VevoIte.Value());
                            Orientation OG = Compare(ME, VE);
                            Shape aLocalShape = VE.Oriented(Forward);
                            Shape aLocalShape2 = ME.Oriented(OG);
                            Glue.Bind(ShapeCasts.Edge(aLocalShape), ShapeCasts.Edge(aLocalShape2));
                        }
                    }
                }
            }
        }
        Glue.Add(Vevo.InternalShape());

        //----------------------------------------------------------
        // Add map of elements generate in Vevo in myMap.
        //----------------------------------------------------------
        foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in MapVevo)
        {
            CurrentSpine = iteS.Key;
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> iteP in iteS.Value)
            {
                CurrentProf = iteP.Key;
                if (!myMap.IsBound(CurrentSpine))
                {
                    //------------------------------------------------
                    // The element of spine is not yet present .
                    // => previous profile not on the border.
                    //-------------------------------------------------
                    myMap.Bind(CurrentSpine, NewProfMap());
                }
                if (!myMap[CurrentSpine].IsBound(CurrentProf))
                {
                    myMap[CurrentSpine].Bind(CurrentProf, new NCollection_List<Shape>());
                    NCollection_List<Shape> GenShapes = iteP.Value;
                    NCollection_List<Shape>.Iterator itl = new(GenShapes);
                    for (; itl.More(); itl.Next())
                    {
                        // during Glue.Add the shared shapes are recreated.
                        if (Glue.IsCopied(itl.Value()))
                        {
                            myMap[CurrentSpine][CurrentProf].Append(Glue.InternalCopy(itl.Value()));
                        }
                        else
                        {
                            myMap[CurrentSpine][CurrentProf].Append(itl.Value());
                        }
                    }
                }
            }
        }
    }

    private ref Shape ChangeShape() => ref myShape;

    private void Transfert(BRepFill_Evolved Vevo,
                           NCollection_DataMap<Shape, Shape> MapProf,
                           NCollection_DataMap<Shape, Shape> MapSpine,
                           in Placement LS,
                           in Placement InitLS,
                           in Placement InitLP)
    {
        //----------------------------------------------------------------
        // Transfer the shape from Vevo in myShape and Reposition shapes.
        //----------------------------------------------------------------
        myShape = Vevo.InternalShape();
        mySpine.Location(InitLS);
        myProfile.Location(InitLP);
        myShape.Move(LS);

        //
        // Expecting for better, the Same Parameter is forced here
        //  ( Pb Sameparameter between YaPlanar and Tuyaux
        //
        BrepBuilder B = new();
        Explorer ex = new(myShape, ShapeType.Edge);
        while (ex.More())
        {
            B.SameRange(ShapeCasts.Edge(ex.Current()), false);
            B.SameParameter(ShapeCasts.Edge(ex.Current()), false);
            BRepLib.SameParameter(ShapeCasts.Edge(ex.Current()));
            ex.Next();
        }

        //--------------------------------------------------------------
        // Transfer of myMap of Vevo into myMap.
        //--------------------------------------------------------------
        Shape InitialSpine, InitialProf;

        NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> MapVevo = Vevo.Generated();

        // OCCT iterates MapVevo in hash order: the keys of myMap are bound in another order, which only changes
        // the order of later iterations over myMap (none in a published result)
        foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in MapVevo)
        {
            InitialSpine = MapSpine[iteS.Key];
            InitialSpine.Move(LS);

            foreach (KeyValuePair<Shape, NCollection_List<Shape>> iteP in iteS.Value)
            {
                InitialProf = MapProf[iteP.Key];
                InitialProf.Location(InitLP);

                NCollection_List<Shape> GenShapes = iteP.Value;

                NCollection_List<Shape>.Iterator itl = new();
                for (itl.Initialize(GenShapes); itl.More(); itl.Next())
                {
                    itl.ChangeValue().Move(LS);
                }

                if (!myMap.IsBound(InitialSpine))
                {
                    myMap.Bind(InitialSpine, NewProfMap());
                }

                if (!myMap[InitialSpine].IsBound(InitialProf))
                {
                    myMap[InitialSpine].Bind(InitialProf, new NCollection_List<Shape>());
                }
                myMap[InitialSpine][InitialProf].Append(GenShapes);
            }
        }
        //--------------------------------------------------------------
        // Transfer of Top and Bottom of Vevo in myTop and myBottom.
        //--------------------------------------------------------------
        myTop = Vevo.myTop;
        myTop.Move(LS);
        myBottom = Vevo.myBottom;
        myBottom.Move(LS);
    }

    /// <summary>True once the shape is built.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The generated shape (frozen).</summary>
    public Shape Shape()
    {
        FreezeResult();
        return myShape;
    }

    /// <summary>OCCT's <c>Shape</c>: the result, not frozen, for ported code that goes on building.</summary>
    internal Shape InternalShape() => myShape;

    private void FreezeResult()
    {
        myShape.TShape()?.FreezeGraph();
        myTop.TShape()?.FreezeGraph();
        myBottom.TShape()?.FreezeGraph();
    }

    /// <summary>The join type of the parallels.</summary>
    public global::Kerf.Geometry.JoinKind JoinType() => myJoinType;

    private void AddTopAndBottom(Quilt Glue)
    {
        //  return first and last vertex of the profile.
        Vertex[] V = [new(), new()];
        Exploration.Vertices(myProfile, out V[0], out V[1]);
        if (V[0].IsSame(V[1]))
        {
            return;
        }

        NCollection_List<Shape>.Iterator itL = new();
        bool ToReverse = false;
        for (int i = 0; i <= 1; i++)
        {
            BRepAlgo_Loop Loop = new();
            // Construction of supports.
            Plane3D S = new(0.0, 0.0, 1.0, -Altitud(V[i]));
            Face F = new BRepLib_MakeFace(S).InternalFace();
            Loop.Init(F);

            Explorer ExpSpine = new(mySpine, ShapeType.Edge);
            NCollection_Map<Shape> View = new(ShapeHasher.Instance);

            for (; ExpSpine.More(); ExpSpine.Next())
            {
                Edge ES = ShapeCasts.Edge(ExpSpine.Current());
                NCollection_List<Shape> L = InternalGeneratedShapes(ES, V[i]);
                bool ComputeOrientation = false;

                for (itL.Initialize(L); itL.More(); itL.Next())
                {
                    Edge E = ShapeCasts.Edge(itL.Value());

                    if (!ComputeOrientation)
                    {
                        EdgeCurveAdaptor C1 = new(ES);
                        EdgeCurveAdaptor C2 = new(E);
                        Tool.Range(E, out double f, out double l);
                        Tool.Range(ES, out double fs, out double ls);
                        double u = 0.3 * f + 0.7 * l;
                        double us = 0.3 * fs + 0.7 * ls;
                        C1.D1(us, out Point3D _, out Vector3D V1);
                        C2.D1(u, out Point3D _, out Vector3D V2);
                        ToReverse = (V1.Dot(V2) < 0.0);
                        ComputeOrientation = true;
                    }

                    Orientation Or = ES.Orientation();
                    if (ToReverse)
                    {
                        Or = global::Kerf.Engine.Topology.TopAbs.Reverse(Or);
                    }
                    Shape aLocalShape = E.Oriented(Or);
                    Loop.AddConstEdge(ShapeCasts.Edge(aLocalShape));
                }
            }

            Point3D PV = Tool.Pnt(V[i]);
            bool IsOut = PV.Y() < 0;

            for (ExpSpine.Init(mySpine, ShapeType.Vertex); ExpSpine.More(); ExpSpine.Next())
            {
                Vertex ES = ShapeCasts.Vertex(ExpSpine.Current());
                if (View.Add(ES))
                {
                    NCollection_List<Shape> L = InternalGeneratedShapes(ES, V[i]);
                    for (itL.Initialize(L); itL.More(); itL.Next())
                    {
                        Edge E = ShapeCasts.Edge(itL.Value());
                        if (!Tool.Degenerated(E))
                        {
                            // the center of circle (ie vertex) is IN the cap if vertex IsOut
                            //                                    OUT                   !IsOut
                            EdgeCurveAdaptor C = new(E);
                            Tool.Range(E, out double f, out double l);
                            double u = 0.3 * f + 0.7 * l;
                            Point3D P = Tool.Pnt(ES);
                            C.D1(u, out Point3D PC, out Vector3D VC);
                            Vector3D aPPC = new(P, PC);
                            Vector3D Prod = aPPC.Crossed(VC);
                            if (IsOut)
                            {
                                ToReverse = Prod.Z() < 0.0;
                            }
                            else
                            {
                                ToReverse = Prod.Z() > 0.0;
                            }
                            Orientation Or = Forward;
                            if (ToReverse)
                            {
                                Or = Reversed;
                            }
                            Shape aLocalShape = E.Oriented(Or);
                            Loop.AddConstEdge(ShapeCasts.Edge(aLocalShape));
                        }
                    }
                }
            }

            Loop.Perform();
            Loop.WiresToFaces();
            NCollection_List<Shape> LF = Loop.NewFaces();
            NCollection_List<Shape>.Iterator anIterL = new(LF);

            // Maj of myTop and myBottom for the history
            // and addition of constructed faces.
            Compound Bouchon = new();
            BrepBuilder B = new();
            B.MakeCompound(ref Bouchon);
            int j = 0;

            for (anIterL.Initialize(LF); anIterL.More(); anIterL.Next())
            {
                j++;
                Glue.Add(anIterL.Value());
                if (j == 1 && i == 0)
                {
                    myTop = anIterL.Value();
                }
                if (j == 1 && i == 1)
                {
                    myBottom = anIterL.Value();
                }
                B.Add(Bouchon, anIterL.Value());
            }
            if (i == 0 && j > 1)
            {
                myTop = Bouchon;
            }
            if (i == 1 && j > 1)
            {
                myBottom = Bouchon;
            }
        }
    }

    private void MakeSolid()
    {
        Explorer exp = new(myShape, ShapeType.Shell);
        int ish = 0;
        Compound Res = new();
        Solid Sol = new();
        BrepBuilder B = new();
        B.MakeCompound(ref Res);

        for (; exp.More(); exp.Next())
        {
            Shape Sh = exp.Current();
            Sol = new Solid();
            B.MakeSolid(ref Sol);
            B.Add(Sol, Sh);
            BRepClass3d_SolidClassifier SC = new(Sol);
            SC.PerformInfinitePoint(BRepFill_Confusion());
            if (SC.State() == In)
            {
                Sol = new Solid();
                B.MakeSolid(ref Sol);
                B.Add(Sol, Sh.Reversed());
            }
            B.Add(Res, Sol);
            ish++;
        }
        if (ish == 1)
        {
            myShape = Sol;
        }
        else
        {
            myShape = Res;
        }
    }

    private void MakePipe(in Edge SE, in CoordinateSystem AxeRef)
    {
        WireExplorer ProfExp = new();

        Transform3D trsf = new();
        if (Side(myProfile, BRepFill_Confusion()) > 3)
        { // side right
            trsf.SetRotation(Constants.OZ(), Math.PI);
        }
        Placement DumLoc = new(trsf);
        Shape aLocalShape = myProfile.Moved(DumLoc);
        Wire DummyProf = PutProfilAt(ShapeCasts.Wire(aLocalShape), AxeRef, SE, mySpine, true);

        // Copy of the profile to avoid the accumulation of
        // locations on the Edges of myProfile!

        TrsfModification TrsfMod = new(new Transform3D());
        Modifier Modif = new(DummyProf, TrsfMod);

        Wire GenProf = ShapeCasts.Wire(Modif.InternalModifiedShape(DummyProf));

        BRepFill_Pipe Pipe = new(new BRepLib_MakeWire(SE).InternalWire(), GenProf);

        //---------------------------------------------
        // Arrangement of Tubes in myMap.
        //---------------------------------------------

        WireExplorer GenProfExp = new();
        bool FirstVertex = true;

        myMap.Bind(SE, NewProfMap());

        for (ProfExp.Init(myProfile), GenProfExp.Init(GenProf); ProfExp.More(); ProfExp.Next(), GenProfExp.Next())
        {
            EdgeVertices(ProfExp.Current(), out Vertex VF, out Vertex VL);
            EdgeVertices(GenProfExp.Current(), out Vertex VFG, out Vertex VLG);

            if (FirstVertex)
            {
                myMap[SE].Bind(VF, new NCollection_List<Shape>());
                myMap[SE][VF].Append(Pipe.InternalEdge(SE, VFG));
                FirstVertex = false;
            }
            myMap[SE].Bind(VL, new NCollection_List<Shape>());
            myMap[SE][VL].Append(Pipe.InternalEdge(SE, VLG));
            myMap[SE].Bind(ProfExp.Current(), new NCollection_List<Shape>());
            myMap[SE][ProfExp.Current()].Append(Pipe.InternalFace(SE, GenProfExp.Current()));
        }
    }

    private void MakeRevol(in Edge SE, in Vertex VLast, in CoordinateSystem AxeRef)
    {
        WireExplorer ProfExp = new();

        Transform3D trsf = new();
        if (Side(myProfile, BRepFill_Confusion()) > 3)
        { // side right
            trsf.SetRotation(Constants.OZ(), Math.PI);
        }
        Placement DumLoc = new(trsf);
        Shape aLocalShape = myProfile.Moved(DumLoc);
        Wire GenProf = PutProfilAt(ShapeCasts.Wire(aLocalShape), AxeRef, SE, mySpine, false);

        Axis3D AxeRev = new(Tool.Pnt(VLast), Constants.DZ().Reversed());

        // Position of the sewing on the edge of the spine
        // so that the bissectrices didn't cross the sewings.
        Transform3D dummy = new();
        dummy.SetRotation(AxeRev, 1.5 * Math.PI);
        Placement DummyLoc = new(dummy);
        GenProf.Move(DummyLoc);

        BRepSweep_Revol Rev = new(GenProf, AxeRev, true);

        //--------------------------------------------
        // Arrangement of revolutions in myMap.
        //---------------------------------------------
        WireExplorer GenProfExp = new();
        bool FirstVertex = true;

        myMap.Bind(VLast, NewProfMap());

        for (ProfExp.Init(myProfile), GenProfExp.Init(GenProf); ProfExp.More(); ProfExp.Next(), GenProfExp.Next())
        {
            EdgeVertices(ProfExp.Current(), out Vertex VF, out Vertex VL);
            EdgeVertices(GenProfExp.Current(), out Vertex VFG, out Vertex VLG);

            Orientation Or = GenProfExp.Current().Orientation();

            if (FirstVertex)
            {
                myMap[VLast].Bind(VF, new NCollection_List<Shape>());
                Shape RV0 = Rev.Shape(VFG);
                Orientation OO0 = RV0.Orientation();
                myMap[VLast][VF].Append(RV0.Oriented(OO0));
                FirstVertex = false;
            }
            myMap[VLast].Bind(ProfExp.Current(), new NCollection_List<Shape>());
            Shape RF = Rev.Shape(GenProfExp.Current());
            Orientation OO = global::Kerf.Engine.Topology.TopAbs.Compose(RF.Orientation(), Or);

            myMap[VLast][ProfExp.Current()].Append(RF.Oriented(OO));
            myMap[VLast].Bind(VL, new NCollection_List<Shape>());
            Shape RV = Rev.Shape(VLG);
            OO = RV.Orientation();
            myMap[VLast][VL].Append(RV.Oriented(OO));
        }
    }

    private static Placement FindLocation(in Face Face)
    {
        Surface S = Tool.Surface(Face, out Placement L)!;

        if (S is not PlaneSurface)
        {
            BRepLib_FindSurface FS = new(Face, -1, true);
            if (FS.Found())
            {
                S = FS.Surface()!;
                L = FS.Location();
            }
            else
            {
                throw new NoSuchObjectException("BRepFill_Evolved : The Face is not planar");
            }
        }

        if (!L.IsIdentity)
        {
            S = (Surface)S.Transformed(L.Transformation());
        }

        PlaneSurface P = (PlaneSurface)S;
        CoordinateSystem Axis = P.Position();

        Transform3D T = new();
        CoordinateSystem AxeRef = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        T.SetTransformation(AxeRef, Axis);

        return new Placement(T);
    }

    // Moves the handles mySpine and myProfile (the caller's shapes are not changed).
    private void TransformInitWork(in Placement LS, in Placement LP)
    {
        mySpine.Move(LS);
        myProfile.Move(LP);
    }

    // Coding of regularities on edges parallel to CutVevo common to left and right parts of volevo.
    private void ContinuityOnOffsetEdge(NCollection_List<Shape> WorkProf)
    {
        WireExplorer WExp = new();
        Vertex VF, VL, V;
        Edge PrecE, CurE = new(), FirstE;
        BrepBuilder B = new();

        WExp.Init(myProfile);
        FirstE = WExp.Current();
        PrecE = FirstE;
        EdgeVertices(FirstE, out VF, out V);
        if (WExp.More())
        {
            WExp.Next();
        }

        for (; WExp.More(); WExp.Next())
        {
            CurE = WExp.Current();
            V = WExp.CurrentVertex();

            if (DistanceToOZ(V) <= BRepFill_Confusion())
            {
                // the regularities are already coded on the edges of elementary volevos
                double U1 = Tool.Parameter(V, CurE);
                double U2 = Tool.Parameter(V, PrecE);
                EdgeCurveAdaptor Curve1 = new(CurE);
                EdgeCurveAdaptor Curve2 = new(PrecE);
                Continuity Continuity = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(Curve1, Curve2, U1, U2);

                if (Continuity >= global::Kerf.Geometry.Continuity.G1)
                {
                    //-----------------------------------------------------
                    // Code continuity for all edges generated by V.
                    //-----------------------------------------------------
                    foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in myMap)
                    {
                        Shape SP = iteS.Key;
                        if (myMap[SP].IsBound(V) && myMap[SP].IsBound(CurE) && myMap[SP].IsBound(PrecE))
                        {
                            if (!myMap[SP][V].IsEmpty() && !myMap[SP][CurE].IsEmpty() && !myMap[SP][PrecE].IsEmpty())
                            {
                                B.Continuity(ShapeCasts.Edge(myMap[SP][V].First()),
                                             ShapeCasts.Face(myMap[SP][CurE].First()),
                                             ShapeCasts.Face(myMap[SP][PrecE].First()),
                                             Continuity);
                            }
                        }
                    }
                }
            }
            PrecE = CurE;
        }

        EdgeVertices(PrecE, out V, out VL);

        if (VF.IsSame(VL))
        {
            // Closed profile.
            double U1 = Tool.Parameter(VF, CurE);
            double U2 = Tool.Parameter(VF, FirstE);
            EdgeCurveAdaptor Curve1 = new(CurE);
            EdgeCurveAdaptor Curve2 = new(FirstE);
            Continuity Continuity = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(Curve1, Curve2, U1, U2);

            if (Continuity >= global::Kerf.Geometry.Continuity.G1)
            {
                //---------------------------------------------
                // Code continuity for all edges generated by V.
                //---------------------------------------------
                foreach (KeyValuePair<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>> iteS in myMap)
                {
                    Shape SP = iteS.Key;
                    if (myMap[SP].IsBound(VF) && myMap[SP].IsBound(CurE) && myMap[SP].IsBound(FirstE))
                    {
                        if (!myMap[SP][VF].IsEmpty() && !myMap[SP][CurE].IsEmpty() && !myMap[SP][FirstE].IsEmpty())
                        {
                            B.Continuity(ShapeCasts.Edge(myMap[SP][VF].First()),
                                         ShapeCasts.Face(myMap[SP][CurE].First()),
                                         ShapeCasts.Face(myMap[SP][FirstE].First()),
                                         Continuity);
                        }
                    }
                }
            }
        }
    }

    // Degenerated edges can be missing in some face: the missing degenerated edges have vertices corresponding to node
    // of the map. Now it is enough to compare points UV of vertices on edges with a certain tolerance.
    private static void AddDegeneratedEdge(in Face F, in Wire W)
    {
        Surface S = Tool.Surface(F, out Placement _)!;
        if (S.GetType() == typeof(TrimmedSurface))
        {
            Surface SB = ((TrimmedSurface)S).BasisSurface();
            if (SB.GetType() == typeof(PlaneSurface))
            {
                return;
            }
        }

        if (S.GetType() == typeof(PlaneSurface))
        {
            return;
        }

        BrepBuilder B = new();
        double TolConf = 1.0e-4;

        bool Change = true;

        while (Change)
        {
            Change = false;
            WireExplorer WE = new(W, F);
            Point2D PF = new(), PrevP = new(), P1, P2 = new();
            Vertex VF = new(), V1, V2 = new();

            for (; WE.More(); WE.Next())
            {
                Edge CE = WE.Current();
                EdgeVertices(CE, out V1, out V2);
                if (CE.Orientation() == Reversed)
                {
                    Tool.UVPoints(CE, F, out P2, out P1);
                }
                else
                {
                    Tool.UVPoints(CE, F, out P1, out P2);
                }
                if (VF.IsNull)
                {
                    VF = V1;
                    PF = P1;
                }
                else
                {
                    if (!P1.IsEqual(PrevP, TolConf))
                    {
                        // degenerated edge to be inserted.
                        Change = true;
                        Vector2D V = new(PrevP, P1);
                        LineCurve2D C2d = new(PrevP, new Direction2D(V));
                        double f = 0, l = PrevP.Distance(P1);
                        Edge NE = new BRepLib_MakeEdge(C2d, S).InternalEdge();
                        B.Degenerated(NE, true);
                        B.Add(NE, V1.Oriented(Forward));
                        B.Add(NE, V1.Oriented(Reversed));
                        B.Range(NE, f, l);
                        B.Add(W, NE);
                        break;
                    }
                }
                PrevP = P2;
            }
            if (!Change && VF.IsSame(V2))
            { // closed
                if (!PF.IsEqual(P2, TolConf))
                {
                    // Degenerated edge to be inserted.
                    Change = true;
                    Vector2D V = new(P2, PF);
                    LineCurve2D C2d = new(P2, new Direction2D(V));
                    double f = 0, l = P2.Distance(PF);
                    Edge NE = new BRepLib_MakeEdge(C2d, S).InternalEdge();
                    B.Degenerated(NE, true);
                    B.Add(NE, VF.Oriented(Forward));
                    B.Add(NE, VF.Oriented(Reversed));
                    B.Range(NE, f, l);
                    B.Add(W, NE);
                }
            }
        }
    }

    private static void TrimFace(in Face Face, NCollection_Sequence<Shape> TheEdges, NCollection_Sequence<Shape> S)
    {
        //--------------------------------------
        // Creation of wires limiting faces.
        //--------------------------------------
        BrepBuilder TheBuilder = new();

        int NbEdges;
        bool NewWire;
        bool AddEdge;
        Wire GoodWire;

        while (!TheEdges.IsEmpty())
        {
            BRepLib_MakeWire MWire = new(ShapeCasts.Edge(TheEdges.First()));
            GoodWire = MWire.InternalWire();
            TheEdges.Remove(1);
            NbEdges = TheEdges.Length();
            NewWire = false;

            while (!NewWire)
            {
                AddEdge = false;

                for (int i = 1; i <= NbEdges && !AddEdge; i++)
                {
                    Edge E = ShapeCasts.Edge(TheEdges.Value(i));
                    if (Tool.Degenerated(E))
                    {
                        TheEdges.Remove(i);
                        AddEdge = true;
                        NbEdges = TheEdges.Length();
                        GoodWire = MWire.InternalWire();
                    }
                    else
                    {
                        MWire.Add(E);
                        if (MWire.Error() == BRepLib_WireError.BRepLib_WireDone)
                        {
                            // the connection is successful
                            // it is removed from the sequence and one restarts from the beginning.
                            TheEdges.Remove(i);
                            AddEdge = true;
                            NbEdges = TheEdges.Length();
                            GoodWire = MWire.InternalWire();
                        }
                    }
                }
                NewWire = (!AddEdge);
            }
            Shape aLocalShape = Face.EmptyCopied();
            Face FaceCut = ShapeCasts.Face(aLocalShape);
            FaceCut.Orientation(Forward);
            BrepTools.Update(FaceCut);
            AddDegeneratedEdge(FaceCut, GoodWire);
            TheBuilder.Add(FaceCut, GoodWire);
            FaceCut.Orientation(Face.Orientation());
            S.Append(FaceCut);
        }
    }

    private static Wire PutProfilAt(in Wire ProfRef, in CoordinateSystem AxeRef, in Edge E, in Face F, bool AtStart)
    {
        Vector2D D1;
        Point2D P;
        Wire Prof;

        Curve2D? C2d = Tool.CurveOnSurface(E, F, out double First, out double Last);
        if (C2d is null)
        {
            throw new ConstructionErrorException("ConstructionError in PutProfilAt");
        }

        if (E.Orientation() == Reversed)
        {
            if (!AtStart)
            {
                C2d.D1(First, out P, out D1);
            }
            else
            {
                C2d.D1(Last, out P, out D1);
            }
            D1.Reverse();
        }
        else
        {
            if (!AtStart)
            {
                C2d.D1(Last, out P, out D1);
            }
            else
            {
                C2d.D1(First, out P, out D1);
            }
        }
        Point3D P3d = new(P.X(), P.Y(), 0.0);
        Vector3D V3d = new(D1.X(), D1.Y(), 0.0);

        CoordinateSystem Ax = new(P3d, Constants.DZ(), new Direction3D(V3d));
        Transform3D Trans = new();
        Trans.SetTransformation(Ax, AxeRef);
        Shape aLocalShape = ProfRef.Moved(new Placement(Trans));
        Prof = ShapeCasts.Wire(aLocalShape);
        return Prof;
    }

    private static void TrimEdge(in Edge Edge,
                                 NCollection_Sequence<Shape> TheEdgesControle,
                                 NCollection_Sequence<Shape> TheVer,
                                 NCollection_Sequence<double> ThePar,
                                 NCollection_Sequence<Shape> S)
    {
        bool Change = true;
        BrepBuilder TheBuilder = new();
        S.Clear();
        //------------------------------------------------------------
        // Parse two sequences depending on the parameter on the edge.
        //------------------------------------------------------------
        while (Change)
        {
            Change = false;
            for (int i = 1; i < ThePar.Length(); i++)
            {
                if (ThePar.Value(i) > ThePar.Value(i + 1))
                {
                    ThePar.Exchange(i, i + 1);
                    TheVer.Exchange(i, i + 1);
                    Change = true;
                }
            }
        }

        //----------------------------------------------------------
        // If a vertex is not in the proofing point, it is removed.
        //----------------------------------------------------------
        if (!Tool.Degenerated(Edge))
        {
            for (int k = 1; k <= TheVer.Length(); k++)
            {
                if (DoubleOrNotInFace(TheEdgesControle, ShapeCasts.Vertex(TheVer.Value(k))))
                {
                    TheVer.Remove(k);
                    ThePar.Remove(k);
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
        if (!Tool.Degenerated(Edge))
        {
            for (int k = 1; k < TheVer.Length(); k++)
            {
                if (TheVer.Value(k).IsSame(TheVer.Value(k + 1)))
                {
                    TheVer.Remove(k + 1);
                    ThePar.Remove(k + 1);
                    if (DoubleOrNotInFace(TheEdgesControle, ShapeCasts.Vertex(TheVer.Value(k))))
                    {
                        TheVer.Remove(k);
                        ThePar.Remove(k);
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
        for (int k = 1; k < TheVer.Length(); k = k + 2)
        {
            Shape aLocalShape = Edge.EmptyCopied();
            Edge NewEdge = ShapeCasts.Edge(aLocalShape);

            if (NewEdge.Orientation() == Reversed)
            {
                TheBuilder.Add(NewEdge, TheVer.Value(k).Oriented(Reversed));
                TheBuilder.Add(NewEdge, TheVer.Value(k + 1).Oriented(Forward));
            }
            else
            {
                TheBuilder.Add(NewEdge, TheVer.Value(k).Oriented(Forward));
                TheBuilder.Add(NewEdge, TheVer.Value(k + 1).Oriented(Reversed));
            }
            TheBuilder.Range(NewEdge, ThePar.Value(k), ThePar.Value(k + 1));
            //  modified by NIZHNY-EAP Wed Dec 22 12:09:48 1999 ___BEGIN___
            BRepLib.UpdateTolerances(NewEdge, false);
            //  modified by NIZHNY-EAP Wed Dec 22 13:34:19 1999 ___END___
            S.Append(NewEdge);
        }
    }

    private static void ComputeIntervals(NCollection_Sequence<Shape> VOnF,
                                         NCollection_Sequence<Shape> VOnL,
                                         NCollection_Sequence<Point3D> ParOnF,
                                         NCollection_Sequence<Point3D> ParOnL,
                                         BRepFill_TrimSurfaceTool Trim,
                                         Curve2D Bis,
                                         in Vertex VS,
                                         in Vertex VE,
                                         NCollection_Sequence<double> FirstPar,
                                         NCollection_Sequence<double> LastPar,
                                         NCollection_Sequence<Shape> FirstV,
                                         NCollection_Sequence<Shape> LastV)
    {
        int IOnF = 1, IOnL = 1;
        double U1 = 0.0, U2;
        Shape V1 = new(), V2;

        if (!VS.IsNull)
        {
            U1 = Bis.FirstParameter();
            V1 = VS;
        }
        while (IOnF <= VOnF.Length() || IOnL <= VOnL.Length())
        {
            //---------------------------------------------------------
            // Return the smallest parameter on the bissectrice
            // corresponding to the current positions IOnF,IOnL.
            //---------------------------------------------------------
            if (IOnL > VOnL.Length() || (IOnF <= VOnF.Length() && ParOnF.Value(IOnF).X() < ParOnL.Value(IOnL).X()))
            {
                U2 = ParOnF.Value(IOnF).X();
                V2 = VOnF.Value(IOnF);
                IOnF++;
            }
            else
            {
                U2 = ParOnL.Value(IOnL).X();
                V2 = VOnL.Value(IOnL);
                IOnL++;
            }
            //---------------------------------------------------------------------
            // When V2 and V1 are different the medium point P of the
            // interval is tested compared to the face. If P is in the face the interval
            // is valid.
            //---------------------------------------------------------------------
            if (!V1.IsNull && !V2.IsSame(V1))
            {
                Point2D P = Bis.Value((U2 + U1) * 0.5);
                if (Trim.IsOnFace(P))
                {
                    FirstPar.Append(U1);
                    LastPar.Append(U2);
                    FirstV.Append(V1);
                    LastV.Append(V2);
                }
            }
            U1 = U2;
            V1 = V2;
        }

        if (!VE.IsNull)
        {
            U2 = Bis.LastParameter();
            V2 = VE;
            if (!V2.IsSame(V1))
            {
                Point2D P = Bis.Value((U2 + U1) * 0.5);
                if (Trim.IsOnFace(P))
                {
                    FirstPar.Append(U1);
                    LastPar.Append(U2);
                    FirstV.Append(V1);
                    LastV.Append(V2);
                }
            }
        }
    }

    // Commun is true if two wires have V in common; returns FORWARD if the wires near the vertex are at the same side,
    // otherwise REVERSED.
    private static Orientation Relative(in Wire W1, in Wire W2, in Vertex V, out bool Commun)
    {
        Explorer Exp = new();
        Edge E1 = new(), E2 = new();

        for (Exp.Init(W1, ShapeType.Edge); Exp.More(); Exp.Next())
        {
            Edge E = ShapeCasts.Edge(Exp.Current());
            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            if (V1.IsSame(V) || V2.IsSame(V))
            {
                E1 = E;
                break;
            }
        }
        for (Exp.Init(W2, ShapeType.Edge); Exp.More(); Exp.Next())
        {
            Edge E = ShapeCasts.Edge(Exp.Current());
            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            if (V1.IsSame(V) || V2.IsSame(V))
            {
                E2 = E;
                break;
            }
        }

        if (E1.IsNull || E2.IsNull)
        {
            Commun = false;
            return Forward;
        }
        Commun = true;

        Wire WW1 = new BRepLib_MakeWire(E1).InternalWire();
        Wire WW2 = new BRepLib_MakeWire(E2).InternalWire();
        double Tol = BRepFill_Confusion();
        if (Side(WW1, Tol) < 4 && Side(WW2, Tol) < 4)
        { // two to the left
            return Forward;
        }
        if (Side(WW1, Tol) > 4 && Side(WW2, Tol) > 4)
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
    private static bool DoubleOrNotInFace(NCollection_Sequence<Shape> EC, in Vertex V)
    {
        bool Vu = false;

        for (int i = 1; i <= EC.Length(); i++)
        {
            Exploration.Vertices(ShapeCasts.Edge(EC.Value(i)), out Vertex V1, out Vertex V2);
            if (V1.IsSame(V))
            {
                if (Vu)
                {
                    return true;
                }
                else
                {
                    Vu = true;
                }
            }
            if (V2.IsSame(V))
            {
                if (Vu)
                {
                    return true;
                }
                else
                {
                    Vu = true;
                }
            }
        }
        return !Vu;
    }

    private static double DistanceToOZ(in Vertex V)
    {
        Point3D PV3d = Tool.Pnt(V);
        return Math.Abs(PV3d.Y());
    }

    private static double Altitud(in Vertex V)
    {
        Point3D PV3d = Tool.Pnt(V);
        return PV3d.Z();
    }

    // OCCT hands on the locus's own trimmed curve (Bisector_Bisec::Value) unless its basis is a BisecAna; the port does
    // the same (BRepFill_TrimSurfaceTool may lengthen it in place, as natively).
    private static void SimpleExpression(Bisector_Bisec B, out Curve2D Bis)
    {
        Bis = B.Value();

        if (Bis.GetType() == typeof(TrimmedCurve2D))
        {
            TrimmedCurve2D TrBis = (TrimmedCurve2D)Bis;
            Curve2D BasBis = TrBis.ChangeBasisCurve();
            if (BasBis.GetType() == typeof(Bisector_BisecAna))
            {
                Bis = ((Bisector_BisecAna)BasBis).Geom2dCurve();
                Bis = new TrimmedCurve2D(Bis, TrBis.FirstParameter(), TrBis.LastParameter());
            }
        }
    }

    // Projection and Cut of an edge at extrema of distance to axis OZ.
    private static void CutEdgeProf(in Edge E,
                                    PlaneSurface Plane,
                                    LineCurve2D Line,
                                    NCollection_List<Shape> Cuts,
                                    NCollection_DataMap<Shape, Shape> MapVerRefMoved)
    {
        Cuts.Clear();

        Curve3D C;
        TrimmedCurve3D CT;
        Curve2D C2d;

        // Return the curve associated to each Edge
        C = Tool.Curve(E, out Placement L, out double f, out double l)!;
        CT = new TrimmedCurve3D(C, f, l);
        CT.Transform(L.Transformation());

        // project it in the plane and return the associated PCurve
        Direction3D Normal = Plane.Pln().Axis().Direction();
        C = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(CT, Plane, Normal, false)!;
        C2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, Plane)!;

        // Calculate the extrema with the straight line
        NCollection_Sequence<double> Seq = new();

        double U1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double U2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        f = C2d.FirstParameter();
        l = C2d.LastParameter();

        Box2d B = new();
        CurveAdaptor2D AC2d = new(C2d);
        Add2dCurve.Add(AC2d, BRepFill_Confusion(), ref B);
        B.Get(out double _, out U1, out double _, out U2);

        //  modified by NIZHNY-EAP Wed Feb  2 16:32:37 2000 ___BEGIN___
        // no sense if C2 is normal to Line or really is a point
        if (U1 != U2)
        {
            Geom2dAPI_ExtremaCurveCurve Extrema = new(Line, C2d, U1 - 1.0, U2 + 1.0, f, l);

            int Nb = Extrema.NbExtrema();
            for (int i = 1; i <= Nb; i++)
            {
                Extrema.Parameters(i, out U1, out U2);
                Seq.Append(U2);
            }
        }
        //  modified by NIZHNY-EAP Wed Feb  2 16:33:05 2000 ___END___

        // On calcule les intersection avec Oy.
        CurveAdaptor2D ALine = new(Line);
        double Tol = global::Kerf.Engine.Foundation.Precision.Intersection();
        double TolC = 0.0;

        Geom2dInt_GInter Intersector = new(ALine, AC2d, TolC, Tol);
        int NbP = Intersector.NbPoints();

        for (int i = 1; i <= NbP; i++)
        {
            Seq.Append(Intersector.Point(i).ParamOnSecond());
        }

        // Compute the new edges.
        BrepBuilder Builder = new();
        Vertex VV, Vf = new(), Vl = new();
        Exploration.Vertices(E, out Vertex VRf, out Vertex VRl);

        if (!MapVerRefMoved.IsBound(VRf))
        {
            Builder.MakeVertex(ref Vf, C.Value(f), Tool.Tolerance(VRf));
            MapVerRefMoved.Bind(VRf, Vf);
        }
        else
        {
            Vf = ShapeCasts.Vertex(MapVerRefMoved[VRf]);
        }

        if (!MapVerRefMoved.IsBound(VRl))
        {
            Builder.MakeVertex(ref Vl, C.Value(l), Tool.Tolerance(VRl));
            MapVerRefMoved.Bind(VRl, Vl);
        }
        else
        {
            Vl = ShapeCasts.Vertex(MapVerRefMoved[VRl]);
        }

        if (!Seq.IsEmpty())
        {
            Bubble(Seq);

            bool Empty = false;

            double CurParam = f;
            double Param;

            while (!Empty)
            {
                Param = Seq.First();
                Seq.Remove(1);
                Empty = Seq.IsEmpty();
                if (Math.Abs(Param - CurParam) > BRepFill_Confusion() && Math.Abs(Param - l) > BRepFill_Confusion())
                {
                    VV = new BRepLib_MakeVertex(C.Value(Param)).InternalVertex();

                    Edge EE0 = new BRepLib_MakeEdge(C, Vf, VV).InternalEdge();
                    EE0.Orientation(E.Orientation());
                    if (EE0.Orientation() == Forward)
                    {
                        Cuts.Append(EE0);
                    }
                    else
                    {
                        Cuts.Prepend(EE0);
                    }

                    // reinitialize
                    CurParam = Param;
                    Vf = VV;
                }
            }
        }

        Edge EE = new BRepLib_MakeEdge(C, Vf, Vl).InternalEdge();
        EE.Orientation(E.Orientation());
        if (EE.Orientation() == Forward)
        {
            Cuts.Append(EE);
        }
        else
        {
            Cuts.Prepend(EE);
        }
    }

    // Cut an edge at the extrema of curves and at points of inflexion. Closed circles are also cut in two. If <Cuts> are
    // empty the edge is not modified. The first and the last vertex of the original edge belong to the first and last
    // parts respectively.
    private static void CutEdge(in Edge E, in Face F, NCollection_List<Shape> Cuts)
    {
        Cuts.Clear();
        MAT2d_CutCurve Cuter = new();
        TrimmedCurve2D CT2d;

        Vertex VF, VL;
        Exploration.Vertices(E, out Vertex V1, out Vertex V2);
        BrepBuilder B = new();

        Curve2D C2d = Tool.CurveOnSurface(E, F, out double f, out double l)!;
        CT2d = new TrimmedCurve2D(C2d, f, l);

        if (CT2d.ChangeBasisCurve() is CircleCurve2D && Tool.IsClosed(E))
        {
            //---------------------------
            // Cut closed circle.
            //---------------------------
            double m1 = (2 * f + l) / 3.0;
            double m2 = (f + 2 * l) / 3.0;
            Point2D P1 = CT2d.Value(m1);
            Point2D P2 = CT2d.Value(m2);

            Vertex VL1 = new BRepLib_MakeVertex(new Point3D(P1.X(), P1.Y(), 0.0)).InternalVertex();
            Vertex VL2 = new BRepLib_MakeVertex(new Point3D(P2.X(), P2.Y(), 0.0)).InternalVertex();
            Shape aLocalShape1 = E.EmptyCopied();
            Shape aLocalShape2 = E.EmptyCopied();
            Shape aLocalShape3 = E.EmptyCopied();
            Edge FE = ShapeCasts.Edge(aLocalShape1);
            Edge ME = ShapeCasts.Edge(aLocalShape2);
            Edge LE = ShapeCasts.Edge(aLocalShape3);

            FE.Orientation(Forward);
            ME.Orientation(Forward);
            LE.Orientation(Forward);

            B.Add(FE, V1);
            B.Add(FE, VL1.Oriented(Reversed));
            B.Range(FE, f, m1);

            B.Add(ME, VL1.Oriented(Forward));
            B.Add(ME, VL2.Oriented(Reversed));
            B.Range(ME, m1, m2);

            B.Add(LE, VL2.Oriented(Forward));
            B.Add(LE, V2);
            B.Range(LE, m2, l);

            Cuts.Append(FE.Oriented(E.Orientation()));
            Cuts.Append(ME.Oriented(E.Orientation()));
            Cuts.Append(LE.Oriented(E.Orientation()));
            //--------
            // Return.
            //--------
            return;
        }

        //-------------------------
        // Cut of the curve.
        //-------------------------
        Cuter.Perform(CT2d);

        if (Cuter.UnModified())
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
            VF = V1;

            for (int k = 1; k <= Cuter.NbCurves(); k++)
            {
                TrimmedCurve2D CC = Cuter.Value(k);
                if (k == Cuter.NbCurves())
                {
                    VL = V2;
                }
                else
                {
                    Point2D P = CC.Value(CC.LastParameter());
                    VL = new BRepLib_MakeVertex(new Point3D(P.X(), P.Y(), 0.0)).InternalVertex();
                }
                Shape aLocalShape = E.EmptyCopied();
                Edge NE = ShapeCasts.Edge(aLocalShape);
                NE.Orientation(Forward);
                B.Add(NE, VF.Oriented(Forward));
                B.Add(NE, VL.Oriented(Reversed));
                B.Range(NE, CC.FirstParameter(), CC.LastParameter());
                Cuts.Append(NE.Oriented(E.Orientation()));
                VF = VL;
            }
        }
    }

    // Tests the position of aNode relative to the distances to OZ of the vertices VF and VL, and returns it; if it is not
    // 0, the vertex VN corresponding to aNode is created (once per node and shape).
    private static int VertexFromNode(MAT_Node aNode,
                                      in Edge E,
                                      in Vertex VF,
                                      in Vertex VL,
                                      NCollection_DataMap<MAT_Node, NCollection_DataMap<Shape, Shape>> MapNodeVertex,
                                      ref Vertex VN)
    {
        Shape ShapeOnNode = new();
        int Status = 0;
        BrepBuilder B = new();

        if (!aNode.Infinite())
        {
            Status = PosOnFace(aNode.Distance(), DistanceToOZ(VF), DistanceToOZ(VL));
        }
        if (Status == 2)
        {
            ShapeOnNode = E;
        }
        else if (Status == 1)
        {
            ShapeOnNode = VF;
        }
        else if (Status == 3)
        {
            ShapeOnNode = VL;
        }

        if (!ShapeOnNode.IsNull)
        {
            //-------------------------------------------------
            // the vertex will correspond to a node of the map
            //-------------------------------------------------
            if (MapNodeVertex.IsBound(aNode) && MapNodeVertex[aNode].IsBound(ShapeOnNode))
            {
                VN = ShapeCasts.Vertex(MapNodeVertex[aNode][ShapeOnNode]);
            }
            else
            {
                VN = new Vertex();
                B.MakeVertex(ref VN);
                if (!MapNodeVertex.IsBound(aNode))
                {
                    MapNodeVertex.Bind(aNode, new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance));
                }
                MapNodeVertex[aNode].Bind(ShapeOnNode, VN);
            }
        }
        return Status;
    }
}
