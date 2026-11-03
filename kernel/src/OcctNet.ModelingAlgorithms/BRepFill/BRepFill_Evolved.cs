// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Evolved.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Evolved.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Topology;
using Schematiq.Collections;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Occt.MAT_Side;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

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
    private TopoDS_Face mySpine = new();
    private TopoDS_Wire myProfile = new();
    private TopoDS_Shape myShape = new();
    private bool myIsDone;
#pragma warning disable CS0414 // OCCT sets the spine type without reading it
    private bool mySpineType;
#pragma warning restore CS0414
    private global::Kerf.Geometry.JoinKind myJoinType;
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> myMap =
        new(TopTools_ShapeMapHasher.Instance);
    private TopoDS_Shape myTop = new();
    private TopoDS_Shape myBottom = new();

    private static double BRepFill_Confusion()
    {
        double Tol = 1.0e-6;
        return Tol;
    }

    private static NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> NewProfMap() => new(TopTools_ShapeMapHasher.Instance);

    private static void EdgeVertices(in TopoDS_Edge E, out TopoDS_Vertex V1, out TopoDS_Vertex V2)
    {
        if (E.Orientation() == TopAbs_REVERSED)
        {
            TopExp.Vertices(E, out V2, out V1);
        }
        else
        {
            TopExp.Vertices(E, out V1, out V2);
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
    public BRepFill_Evolved(in TopoDS_Wire Spine, in TopoDS_Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool Solid = false)
    {
        myIsDone = false;
        Perform(Spine, Profile, AxeProf, Join, Solid);
    }

    /// <summary>The evolved shape of <paramref name="Profile"/> swept along the face <paramref name="Spine"/>.</summary>
    public BRepFill_Evolved(in TopoDS_Face Spine, in TopoDS_Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool Solid = false)
    {
        myIsDone = false;
        Perform(Spine, Profile, AxeProf, Join, Solid);
    }

    private static bool IsVertical(in TopoDS_Edge E)
    {
        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        Point3D P1 = BRep_Tool.Pnt(V1);
        Point3D P2 = BRep_Tool.Pnt(V2);

        if (Math.Abs(P1.Y() - P2.Y()) < BRepFill_Confusion())
        {
            // It is a Line ?
            Geom_Curve? GC = BRep_Tool.Curve(E, out Placement _, out double _, out double _);
            if (GC!.GetType() == typeof(Geom_Line))
            {
                return true;
            }
        }
        return false;
    }

    private static bool IsPlanar(in TopoDS_Edge E)
    {
        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        Point3D P1 = BRep_Tool.Pnt(V1);
        Point3D P2 = BRep_Tool.Pnt(V2);

        if (Math.Abs(P1.Z() - P2.Z()) < BRepFill_Confusion())
        {
            // It is a Line ?
            Geom_Curve? GC = BRep_Tool.Curve(E, out Placement _, out double _, out double _);
            if (GC!.GetType() == typeof(Geom_Line))
            {
                return true;
            }
        }
        return false;
    }

    // Determines the position of the profile relative to the plane XOZ:
    // 1 MAT_Left, 2 MAT_Left and planar, 3 MAT_Left and vertical,
    // 4 MAT_Right, 5 MAT_Right and planar, 6 MAT_Right and vertical.
    private static int Side(in TopoDS_Wire Profil, double Tol)
    {
        // Rem : it is enough to test the first edge of the Wire.
        //       ( Correctly cut in PrepareProfil)
        TopExp_Explorer Explo = new(Profil, TopAbs_EDGE);

        int TheSide;
        TopoDS_Edge E = TopoDS.Edge(Explo.Current());

        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        Point3D P1 = BRep_Tool.Pnt(V1);
        Point3D P2 = BRep_Tool.Pnt(V2);

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
    public void Perform(in TopoDS_Wire Spine, in TopoDS_Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool Solid = false)
    {
        mySpineType = false;
        TopoDS_Face aFace = new BRepLib_MakeFace(Spine, true).InternalFace();
        PrivatePerform(aFace, Profile, AxeProf, Join, Solid);
    }

    /// <summary>Builds the evolved shape of <paramref name="Profile"/> swept along the face <paramref name="Spine"/>.</summary>
    public void Perform(in TopoDS_Face Spine, in TopoDS_Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join = Arc, bool Solid = false)
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
    private void PrivatePerform(in TopoDS_Face Spine, in TopoDS_Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join, bool Solid)
    {
        BRepTools_CopyAtEntry aSpineEntry = new(Spine);
        BRepTools_CopyAtEntry aProfEntry = new(Profile);
        PrivatePerformInPlace(TopoDS.Face(aSpineEntry.Shape()), TopoDS.Wire(aProfEntry.Shape()), AxeProf, Join, Solid);
        if (!aSpineEntry.IsCopied() && !aProfEntry.IsCopied())
        {
            return;
        }

        foreach (BRepTools_CopyAtEntry anEntry in new[] { aProfEntry, aSpineEntry })
        {
            List<TopoDS_Shape> anOthers = [myTop, myBottom];
            foreach (KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> aBySpine in myMap)
            {
                foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aByProf in aBySpine.Value)
                {
                    anOthers.AddRange(aByProf.Value);
                }
            }
            myShape = anEntry.Finish(myShape, anOthers);
            myTop = anEntry.Final(myTop);
            myBottom = anEntry.Final(myBottom);
            foreach (KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> aBySpine in myMap)
            {
                foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aByProf in aBySpine.Value)
                {
                    for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(aByProf.Value); anIt.More(); anIt.Next())
                    {
                        anIt.ChangeValue() = anEntry.Final(anIt.Value());
                    }
                }
            }
        }

        // the keys: the caller's sub-shapes (spine first, then profile), in the same order
        List<KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>>> anEntries = [.. myMap];
        myMap.Clear();
        foreach (KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> aBySpine in anEntries)
        {
            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aByProf = NewProfMap();
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in aBySpine.Value)
            {
                aByProf.Bind(aProfEntry.Original(anItem.Key), anItem.Value);
            }
            myMap.Bind(aSpineEntry.Original(aBySpine.Key), aByProf);
        }
        mySpine = TopoDS.Face(Spine.Oriented(TopAbs_FORWARD));
        myProfile = TopoDS.Wire(Profile.Oriented(TopAbs_FORWARD));
    }

    private void PrivatePerformInPlace(in TopoDS_Face Spine, in TopoDS_Wire Profile, in CoordinateSystem AxeProf, global::Kerf.Geometry.JoinKind Join, bool Solid)
    {
        TopoDS_Shape aLocalShape = Spine.Oriented(TopAbs_FORWARD);
        mySpine = TopoDS.Face(aLocalShape);
        aLocalShape = Profile.Oriented(TopAbs_FORWARD);
        myProfile = TopoDS.Wire(aLocalShape);
        myJoinType = Join;
        myMap.Clear();

        if (myJoinType > Arc)
        {
            throw new MethodNotImplementedException();
        }

        NCollection_List<TopoDS_Shape> WorkProf = new();
        TopoDS_Face WorkSpine = new();
        NCollection_List<TopoDS_Shape>.Iterator WPIte = new();

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
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapProf = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapSpine = new(TopTools_ShapeMapHasher.Instance);

        PrepareProfile(WorkProf, MapProf);
        PrepareSpine(ref WorkSpine, MapSpine);

        double Tol = BRepFill_Confusion();
        bool YaLeft = false;
        bool YaRight = false;
        TopoDS_Wire SP;

        for (WPIte.Initialize(WorkProf); WPIte.More(); WPIte.Next())
        {
            SP = TopoDS.Wire(WPIte.Value());
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

        TopoDS_Face Face;
        BRepMAT2d_BisectingLocus Locus = new();

        //----------------------------------------------------------
        // Initialisation of cut volevo.
        // For each part of the profile create a volevo added to CutVevo
        //----------------------------------------------------------
        BRepFill_Evolved CutVevo = new();
        TopoDS_Wire WP = new();
        BRep_Builder BB = new();
        BRepTools_WireExplorer WExp = new();

        BB.MakeWire(ref WP);

        for (WPIte.Initialize(WorkProf); WPIte.More(); WPIte.Next())
        {
            for (WExp.Init(TopoDS.Wire(WPIte.Value())); WExp.More(); WExp.Next())
            {
                BB.Add(WP, WExp.Current());
            }
        }
        CutVevo.SetWork(WorkSpine, WP);

        BRepTools_Quilt Glue = new();
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
                SP = TopoDS.Wire(WPIte.Value());
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
            TopExp_Explorer SpineExp = new(WorkSpine, TopAbs_WIRE);
            for (; SpineExp.More(); SpineExp.Next())
            {
                //----------------------------------------------
                // Calculate the map to the right of the current wire.
                //----------------------------------------------
                BRepLib_MakeFace B = new(new Plane3D(0.0, 0.0, 1.0, 0.0));
                TopoDS_Shape aLocalShapeRev = SpineExp.Current().Reversed();
                B.Add(TopoDS.Wire(aLocalShapeRev));
                Face = B.InternalFace();
                BRepMAT2d_Explorer Exp = new(Face);
                Locus.Compute(Exp, 1, MAT_Left);
                BRepMAT2d_LinkTopoBilo Link = new(Exp, Locus);

                for (WPIte.Initialize(WorkProf); WPIte.More(); WPIte.Next())
                {
                    SP = TopoDS.Wire(WPIte.Value());
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

    private static void IsInversed(in TopoDS_Shape S, in TopoDS_Edge E1, in TopoDS_Edge E2, bool[] Inverse)
    {
        Inverse[0] = Inverse[1] = false;
        if (S.ShapeType() != TopAbs_EDGE)
        {
            return;
        }

        Point3D P;
        Vector3D DS, DC1, DC2;
        BRepAdaptor_Curve CS = new(TopoDS.Edge(S));
        if (S.Orientation() == TopAbs_FORWARD)
        {
            CS.D1(CS.FirstParameter(), out P, out DS);
        }
        else
        {
            CS.D1(CS.LastParameter(), out P, out DS);
            DS.Reverse();
        }

        if (!BRep_Tool.Degenerated(E1))
        {
            BRepAdaptor_Curve C1 = new(TopoDS.Edge(E1));
            if (E1.Orientation() == TopAbs_FORWARD)
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

        if (!BRep_Tool.Degenerated(E2))
        {
            BRepAdaptor_Curve C2 = new(TopoDS.Edge(E2));
            if (E2.Orientation() == TopAbs_FORWARD)
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

    private void SetWork(in TopoDS_Face Sp, in TopoDS_Wire Pr)
    {
        mySpine = Sp;
        myProfile = Pr;
    }

    // Determines if the pipes were at the side of the concavity. In this case they can be closed.
    // WARNING: Not finished. Done only for circles.
    private static bool ConcaveSide(in TopoDS_Shape S, in TopoDS_Face F)
    {
        if (S.ShapeType() == TopAbs_VERTEX)
        {
            return false;
        }

        if (S.ShapeType() == TopAbs_EDGE)
        {
            Geom2d_Curve? G2d = BRep_Tool.CurveOnSurface(TopoDS.Edge(S), F, out double f, out double l);

            Geom2dAdaptor_Curve AC = new(G2d!, f, l);
            if (AC.GetType() == Circle)
            {
                bool Direct = AC.Circle().IsDirect;
                if (S.Orientation() == TopAbs_REVERSED)
                {
                    Direct = (!Direct);
                }
                return Direct;
            }
        }
        return false;
    }

    private void ElementaryPerform(in TopoDS_Face Sp, in TopoDS_Wire Pr, BRepMAT2d_BisectingLocus Locus, BRepMAT2d_LinkTopoBilo Link,
                                   global::Kerf.Geometry.JoinKind Join)
    {
        TopoDS_Shape aLocalShape = Sp.Oriented(TopAbs_FORWARD);
        mySpine = TopoDS.Face(aLocalShape);
        myProfile = Pr;
        myMap.Clear();

        BRep_Builder myBuilder = new();
        TopoDS_Compound aCompound = new();
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
        NCollection_DataMap<MAT_Node, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>> MapNodeVertex = new(ReferenceEqualityComparer.Instance);
        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<TopoDS_Shape>> MapBis = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<double>> MapVerPar = new(TopTools_ShapeMapHasher.Instance);

        // mark of the profile.
        CoordinateSystem AxeRef = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));

        //---------------------------------------------------------------
        // Construction of revolutions and tubes.
        //---------------------------------------------------------------
        BRepTools_WireExplorer ProfExp = new();
        TopExp_Explorer FaceExp = new();
        BRepTools_WireExplorer WireExp = new();

        for (FaceExp.Init(mySpine, TopAbs_WIRE); FaceExp.More(); FaceExp.Next())
        {
            for (WireExp.Init(TopoDS.Wire(FaceExp.Current())); WireExp.More(); WireExp.Next())
            {
                TopoDS_Edge CurrentEdge0 = WireExp.Current();
                EdgeVertices(CurrentEdge0, out TopoDS_Vertex _, out TopoDS_Vertex VLast);

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
        Geom2d_Curve Bis;
        Geom2d_Curve? PCurve1, PCurve2;
        Geom_Curve? CBis;
        TopoDS_Edge CurrentEdge = new();
        TopoDS_Shape[] S = [new(), new()];
        TopoDS_Face[] F = [new(), new()];
        TopoDS_Edge[] E = [new(), new(), new(), new()];
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

            NCollection_Sequence<TopoDS_Shape> VOnF = new(), VOnL = new();
            NCollection_Sequence<Point3D> ParOnF = new(), ParOnL = new();

            NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<TopoDS_Shape>> MapSeqVer = new(TopTools_ShapeMapHasher.Instance);
            NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<Point3D>> MapSeqPar = new(TopTools_ShapeMapHasher.Instance);

            for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
            {
                //-----------------------------------------------
                // Return two faces separated by the bissectrice.
                //-----------------------------------------------
                F[0] = TopoDS.Face(myMap[S[0]][ProfExp.Current()].First());
                F[1] = TopoDS.Face(myMap[S[1]][ProfExp.Current()].First());

                //------------------------------------
                // Return parallel edges on each face.
                //------------------------------------
                EdgeVertices(ProfExp.Current(), out TopoDS_Vertex VF, out TopoDS_Vertex VL);

                E[0] = TopoDS.Edge(myMap[S[0]][VF].First());
                E[1] = TopoDS.Edge(myMap[S[0]][VL].First());
                E[2] = TopoDS.Edge(myMap[S[1]][VF].First());
                E[3] = TopoDS.Edge(myMap[S[1]][VL].First());

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
                TopoDS_Vertex VS = new(), VE = new();
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
                    if (S[0].ShapeType() == TopAbs_VERTEX)
                    {
                        Node1 = CurrentArc.FirstElement().StartArc()!.FirstNode();
                    }
                    else if (S[1].ShapeType() == TopAbs_VERTEX)
                    {
                        Node1 = CurrentArc.SecondElement().StartArc()!.FirstNode();
                    }
                }
                // End of particular case.

                int StartOnF = VertexFromNode(Node1, TopoDS.Edge(ProfExp.Current()), VF, VL, MapNodeVertex, ref VS);

                int EndOnF = VertexFromNode(Node2, TopoDS.Edge(ProfExp.Current()), VF, VL, MapNodeVertex, ref VE);

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
                            TopoDS_Vertex VC = new();
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
                    VOnF = new NCollection_Sequence<TopoDS_Shape>(MapSeqVer[VF]);
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
                            TopoDS_Vertex VC = new();
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
                    VOnL = new NCollection_Sequence<TopoDS_Shape>(MapSeqVer[VL]);
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
                NCollection_Sequence<TopoDS_Shape> FirstV = new(), LastV = new();

                ComputeIntervals(VOnF, VOnL, ParOnF, ParOnL, Trim, Bis, VS, VE, FirstPar, LastPar, FirstV, LastV);

                for (int Ti = 1; Ti <= FirstPar.Length(); Ti++)
                {
                    TopoDS_Vertex V1 = TopoDS.Vertex(FirstV.Value(Ti));
                    TopoDS_Vertex V2 = TopoDS.Vertex(LastV.Value(Ti));

                    Trim.Project(FirstPar.Value(Ti), LastPar.Value(Ti), out CBis, out PCurve1, out PCurve2, out Continuity Continuity);

                    //-------------------------------------
                    // Coding of the edge.
                    //-------------------------------------
                    myBuilder.MakeEdge(ref CurrentEdge, CBis, BRepFill_Confusion());

                    myBuilder.UpdateVertex(V1, CBis!.Value(CBis.FirstParameter()), BRepFill_Confusion());
                    myBuilder.UpdateVertex(V2, CBis.Value(CBis.LastParameter()), BRepFill_Confusion());

                    myBuilder.Add(CurrentEdge, V1.Oriented(TopAbs_FORWARD));
                    myBuilder.Add(CurrentEdge, V2.Oriented(TopAbs_REVERSED));

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
                            MapBis.Bind(F[k], new NCollection_Sequence<TopoDS_Shape>());
                        }
                    }
                    //---------------------------------------------------------------
                    // orientation of the edge depends on the direction of the skin.
                    // skin => same orientation E[0] , inverted orientation E[2]
                    // if contreskin it is inverted.
                    //--------------------------------------------------------------
                    E[0].Orientation(BRepTools.OriEdgeInFace(E[0], F[0]));
                    E[2].Orientation(BRepTools.OriEdgeInFace(E[2], F[1]));

                    if (DistanceToOZ(VF) < DistanceToOZ(VL))
                    {
                        // Skin
                        MapBis[F[0]].Append(CurrentEdge.Oriented(E[0].Orientation()));
                        CurrentEdge.Orientation(TopAbs.Complement(E[2].Orientation()));
                        MapBis[F[1]].Append(CurrentEdge);
                    }
                    else
                    {
                        // Contreskin
                        MapBis[F[1]].Append(CurrentEdge.Oriented(E[2].Orientation()));
                        CurrentEdge.Orientation(TopAbs.Complement(E[0].Orientation()));
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
                                MapBis.Bind(E[k], new NCollection_Sequence<TopoDS_Shape>());
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
                                MapBis.Bind(E[k], new NCollection_Sequence<TopoDS_Shape>());
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
                MapSeqVer.Bind(VF, new NCollection_Sequence<TopoDS_Shape>(VOnF));
                MapSeqPar.Bind(VL, new NCollection_Sequence<Point3D>(ParOnL));
                MapSeqVer.Bind(VL, new NCollection_Sequence<TopoDS_Shape>(VOnL));
            }
        }

        //----------------------------------
        // Construction of parallel edges.
        //----------------------------------
        TopoDS_Shape CurrentProf = new(), PrecProf = new();
        TopoDS_Face CurrentFace;
        TopoDS_Shape CurrentSpine;
        TopoDS_Vertex VCF = new(), VCL = new();

        // OCCT iterates myMap in its hash order (TShape addresses); the port in insertion order (occt-issues.md)
        foreach (TopoDS_Shape aSpineKey in new List<TopoDS_Shape>(myMap.Keys))
        {
            CurrentSpine = aSpineKey;

            for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
            {
                CurrentProf = ProfExp.Current();
                EdgeVertices(TopoDS.Edge(CurrentProf), out VCF, out VCL);
                CurrentEdge = TopoDS.Edge(myMap[CurrentSpine][VCF].First());

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
                    TopoDS_Shape FaceControle;
                    bool YaFace = true;

                    FaceControle = myMap[CurrentSpine][CurrentProf].First();
                    if (!MapBis.IsBound(FaceControle))
                    {
                        YaFace = false;
                        if (!PrecProf.IsNull())
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
                        NCollection_Sequence<TopoDS_Shape> aSeqOfShape = new();
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
            CurrentEdge = TopoDS.Edge(myMap[CurrentSpine][VCL].First());

            if (MapBis.IsBound(CurrentEdge))
            {
                bool YaFace = true;
                TopoDS_Shape FaceControle;

                FaceControle = myMap[CurrentSpine][CurrentProf].First();
                if (!MapBis.IsBound(FaceControle))
                {
                    YaFace = false;
                }
                // the number of element of the list allows to know
                // if the edges have already been done (closed profile) .
                if (YaFace && myMap[CurrentSpine][VCL].Extent() <= 1)
                {
                    NCollection_Sequence<TopoDS_Shape> aSeqOfShape = new();
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
        foreach (TopoDS_Shape aSpineKey in new List<TopoDS_Shape>(myMap.Keys))
        {
            CurrentSpine = aSpineKey;

            for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
            {
                CurrentProf = ProfExp.Current();
                CurrentFace = TopoDS.Face(myMap[CurrentSpine][CurrentProf].First());
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
                    EdgeVertices(TopoDS.Edge(CurrentProf), out VCF, out VCL);

                    NCollection_List<TopoDS_Shape>.Iterator itl = new();
                    NCollection_List<TopoDS_Shape> LF = myMap[CurrentSpine][VCF];

                    TopAbs_Orientation Ori = BRepTools.OriEdgeInFace(TopoDS.Edge(LF.First()), CurrentFace);
                    for (itl.Initialize(LF), itl.Next(); itl.More(); itl.Next())
                    {
                        TopoDS_Edge RE = TopoDS.Edge(itl.Value());
                        MapBis[CurrentFace].Append(RE.Oriented(Ori));
                    }
                    NCollection_List<TopoDS_Shape> LL = myMap[CurrentSpine][VCL];
                    Ori = BRepTools.OriEdgeInFace(TopoDS.Edge(LL.First()), CurrentFace);
                    for (itl.Initialize(LL), itl.Next(); itl.More(); itl.Next())
                    {
                        TopoDS_Edge RE = TopoDS.Edge(itl.Value());
                        MapBis[CurrentFace].Append(RE.Oriented(Ori));
                    }

                    // Cut of the face.
                    NCollection_Sequence<TopoDS_Shape> aSeqOfShape = new();

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
            TopExp_Explorer Explo = new(myProfile, TopAbs_VERTEX);
            NCollection_Map<TopoDS_Shape> vmap = new(TopTools_ShapeMapHasher.Instance);

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

    private void PlanarPerform(in TopoDS_Face Sp, in TopoDS_Wire Pr, BRepMAT2d_BisectingLocus Locus, BRepMAT2d_LinkTopoBilo Link,
                               global::Kerf.Geometry.JoinKind Join)
    {
        TopoDS_Shape aLocalShapeOriented = Sp.Oriented(TopAbs_FORWARD);
        mySpine = TopoDS.Face(aLocalShapeOriented);
        myProfile = Pr;
        myMap.Clear();

        BRep_Builder B = new();
        TopoDS_Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        myShape = aCompound;

        BRepTools_WireExplorer ProfExp = new();
        TopExp_Explorer Exp = new();
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapVP = new(TopTools_ShapeMapHasher.Instance);
        BRepFill_OffsetWire Paral = new();

        for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
        {
            TopoDS_Edge E = ProfExp.Current();
            BRepAlgo_FaceRestrictor FR = new();
            BRepFill_OffsetAncestors OffAnc = new();

            TopoDS_Vertex[] V = [new(), new()];
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
                    for (Exp.Init(Paral.InternalShape(), TopAbs_EDGE); Exp.More(); Exp.Next())
                    {
                        TopoDS_Edge WC = TopoDS.Edge(Exp.Current());
                        TopoDS_Shape GS = OffAnc.Ancestor(WC);
                        if (!myMap.IsBound(GS))
                        {
                            myMap.Bind(GS, NewProfMap());
                        }
                        if (!myMap[GS].IsBound(V[i]))
                        {
                            myMap[GS].Bind(V[i], new NCollection_List<TopoDS_Shape>(Paral.InternalGeneratedShapes(GS)));
                        }
                    }
                }
                TopoDS_Shape Rest = MapVP[V[i]];

                bool ToReverse = false;
                if ((IsMinV1 && (i == 1)) || (!IsMinV1 && (i == 0)))
                {
                    ToReverse = true;
                }

                if (!Rest.IsNull())
                {
                    if (Rest.ShapeType() == TopAbs_WIRE)
                    {
                        if (ToReverse)
                        {
                            TopoDS_Shape aLocalShape = Rest.Reversed();
                            TopoDS_Wire aWire = TopoDS.Wire(aLocalShape);
                            FR.Add(aWire);
                        }
                        else
                        {
                            FR.Add(TopoDS.Wire(Rest));
                        }
                    }
                    else
                    {
                        for (Exp.Init(Rest, TopAbs_WIRE); Exp.More(); Exp.Next())
                        {
                            TopoDS_Wire WCop = TopoDS.Wire(Exp.Current());
                            if (ToReverse)
                            {
                                TopoDS_Shape aLocalShape = WCop.Reversed();
                                TopoDS_Wire bWire = TopoDS.Wire(aLocalShape);
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
            TopoDS_Shape aLocalShape2 = mySpine.Moved(LT);
            FR.Init(TopoDS.Face(aLocalShape2));
            // the restrictor runs on the wires this algorithm made (5O1's notes)
            FR.InternalPerform();

            for (; FR.More(); FR.Next())
            {
                TopoDS_Face Fc = FR.InternalCurrent();
                B.Add(myShape, Fc);
                //---------------------------------------
                // Update myMap(.)(E)
                //---------------------------------------
                for (Exp.Init(Fc, TopAbs_EDGE); Exp.More(); Exp.Next())
                {
                    TopoDS_Edge CE = TopoDS.Edge(Exp.Current());
                    if (OffAnc.HasAncestor(CE))
                    {
                        TopoDS_Shape InitE = OffAnc.Ancestor(CE);
                        if (!myMap.IsBound(InitE))
                        {
                            myMap.Bind(InitE, NewProfMap());
                        }
                        if (!myMap[InitE].IsBound(E))
                        {
                            myMap[InitE].Bind(E, new NCollection_List<TopoDS_Shape>());
                        }
                        myMap[InitE][E].Append(Fc);
                    }
                }
            }
        } // End loop on profile.
    }

    private void VerticalPerform(in TopoDS_Face Sp, in TopoDS_Wire Pr, BRepMAT2d_BisectingLocus Locus, BRepMAT2d_LinkTopoBilo Link,
                                 global::Kerf.Geometry.JoinKind Join)
    {
        TopoDS_Shape aLocalShape = Sp.Oriented(TopAbs_FORWARD);
        mySpine = TopoDS.Face(aLocalShape);
        myProfile = Pr;
        myMap.Clear();

        BRep_Builder B = new();
        TopoDS_Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        myShape = aCompound;

        BRepTools_WireExplorer ProfExp = new();
        TopExp_Explorer Exp = new();
        BRepFill_OffsetWire Paral = new();
        BRepFill_OffsetAncestors OffAnc = new();

        bool First = true;
        TopoDS_Shape Base = new();

        for (ProfExp.Init(myProfile); ProfExp.More(); ProfExp.Next())
        {
            TopoDS_Edge E = ProfExp.Current();
            EdgeVertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
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
                for (Exp.Init(Base, TopAbs_EDGE); Exp.More(); Exp.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(Exp.Current());
                    TopoDS_Shape AE = OffAnc.Ancestor(anEdge);
                    if (!myMap.IsBound(AE))
                    {
                        myMap.Bind(AE, NewProfMap());
                    }
                    if (!myMap[AE].IsBound(V1))
                    {
                        myMap[AE].Bind(V1, new NCollection_List<TopoDS_Shape>());
                    }
                    myMap[AE][V1].Append(anEdge);
                }
                First = false;
            }

            BRepSweep_Prism PS = new(Base, new Vector3D(0, 0, Alt2 - Alt1), false);

            Base = PS.LastShape();

            for (Exp.Init(PS.Shape(), TopAbs_FACE); Exp.More(); Exp.Next())
            {
                B.Add(myShape, Exp.Current());
            }

            // MAJ myMap
            foreach (TopoDS_Shape aKey in new List<TopoDS_Shape>(myMap.Keys))
            {
                NCollection_List<TopoDS_Shape> LOF = myMap[aKey][V1];
                NCollection_List<TopoDS_Shape>.Iterator itLOF = new(LOF);
                if (!myMap[aKey].IsBound(V2))
                {
                    myMap[aKey].Bind(V2, new NCollection_List<TopoDS_Shape>());
                }

                if (!myMap[aKey].IsBound(E))
                {
                    myMap[aKey].Bind(E, new NCollection_List<TopoDS_Shape>());
                }

                for (; itLOF.More(); itLOF.Next())
                {
                    TopoDS_Shape OS = itLOF.Value();
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
    private void PrepareProfile(NCollection_List<TopoDS_Shape> WorkProf, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapProf)
    {
        // Supposedly the profile is located so that the only transformation
        // to be carried out is a projection on plane yOz.

        // initialise the projection Plane and the Line to evaluate the extrema.
        Geom_Plane Plane = new(new CoordinateSystem(Constants.YOZ()));
        Geom2d_Line Line = new(Constants.OY2d());

        // Map initial vertex -> projected vertex.
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapVerRefMoved = new(TopTools_ShapeMapHasher.Instance);

        TopoDS_Wire W = new();
        BRep_Builder B = new();
        NCollection_List<TopoDS_Shape> WP = new();
        B.MakeWire(ref W);
        WP.Append(W);

        BRepTools_WireExplorer Exp = new(myProfile);

        while (Exp.More())
        {
            NCollection_List<TopoDS_Shape> Cuts = new();
            bool NewWire = false;
            TopoDS_Edge E = TopoDS.Edge(Exp.Current());

            // Cut of the edge.
            CutEdgeProf(E, Plane, Line, Cuts, MapVerRefMoved);

            EdgeVertices(E, out TopoDS_Vertex _, out TopoDS_Vertex _);

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
                    TopoDS_Edge NE = TopoDS.Edge(Cuts.First());
                    MapProf.Bind(NE, E);
                    EdgeVertices(NE, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
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
                            W = new TopoDS_Wire();
                            B.MakeWire(ref W);
                            WP.Append(W);
                        }
                    }
                }
            }
            Exp.Next();
            if (Exp.More() && NewWire)
            {
                W = new TopoDS_Wire();
                B.MakeWire(ref W);
                WP.Append(W);
            }
        }

        // In the list of Wires, find edges generating plane or vertical vevo.
        NCollection_List<TopoDS_Shape>.Iterator ite = new();
        TopoDS_Wire CurW, NW = new();
        TopExp_Explorer EW = new();

        for (ite.Initialize(WP); ite.More(); ite.Next())
        {
            CurW = TopoDS.Wire(ite.Value());
            bool YaModif = false;
            for (EW.Init(CurW, TopAbs_EDGE); EW.More(); EW.Next())
            {
                TopoDS_Edge EE = TopoDS.Edge(EW.Current());
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

                for (EW.Init(CurW, TopAbs_EDGE); EW.More(); EW.Next())
                {
                    TopoDS_Edge EE = TopoDS.Edge(EW.Current());
                    if (IsVertical(EE))
                    {
                        if (Status != 3)
                        {
                            NW = new TopoDS_Wire();
                            B.MakeWire(ref NW);
                            WorkProf.Append(NW);
                            Status = 3;
                        }
                    }
                    else if (IsPlanar(EE))
                    {
                        if (Status != 2)
                        {
                            NW = new TopoDS_Wire();
                            B.MakeWire(ref NW);
                            WorkProf.Append(NW);
                            Status = 2;
                        }
                    }
                    else if (Status != 1)
                    {
                        NW = new TopoDS_Wire();
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
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> gilbert in MapVerRefMoved)
        {
            MapProf.Bind(gilbert.Value, gilbert.Key);
        }
    }

    // Cut the spine's edges at the extrema of curvature and at the inflexion points.
    private void PrepareSpine(ref TopoDS_Face WorkSpine, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapSpine)
    {
        BRep_Builder B = new();
        NCollection_List<TopoDS_Shape> Cuts = new();
        NCollection_List<TopoDS_Shape>.Iterator IteCuts = new();

        Geom_Surface? S = BRep_Tool.Surface(mySpine, out Placement L);
        double TolF = BRep_Tool.Tolerance(mySpine);
        B.MakeFace(ref WorkSpine, S, L, TolF);

        for (TopoDS_Iterator IteF = new(mySpine); IteF.More(); IteF.Next())
        {
            TopoDS_Wire NW = new();
            B.MakeWire(ref NW);
            bool IsClosed = IteF.Value().Closed();

            for (TopoDS_Iterator IteW = new(IteF.Value()); IteW.More(); IteW.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(IteW.Value());
                EdgeVertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
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
                        TopoDS_Edge NE = TopoDS.Edge(IteCuts.Value());
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
    public TopoDS_Shape Top()
    {
        FreezeResult();
        return myTop;
    }

    /// <summary>The bottom face (or compound of faces) of a solid result (frozen with the result).</summary>
    public TopoDS_Shape Bottom()
    {
        FreezeResult();
        return myBottom;
    }

    /// <summary>
    /// The shapes generated from a sub-shape <paramref name="SpineShape"/> of the spine and a sub-shape
    /// <paramref name="ProfShape"/> of the profile (frozen, with the result).
    /// </summary>
    public ISchematiqList<TopoDS_Shape> GeneratedShapes(in TopoDS_Shape SpineShape, in TopoDS_Shape ProfShape)
    {
        FreezeResult();
        NCollection_List<TopoDS_Shape> aList = InternalGeneratedShapes(SpineShape, ProfShape);
        if (aList.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        var aShapes = new TopoDS_Shape[aList.Extent()];
        int i = 0;
        foreach (TopoDS_Shape aS in aList)
        {
            aShapes[i] = aS;
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(aShapes);
    }

    /// <summary>OCCT's <c>GeneratedShapes</c>: the algorithm's own list (shared: don't modify), not frozen.</summary>
    internal NCollection_List<TopoDS_Shape> InternalGeneratedShapes(in TopoDS_Shape SpineShape, in TopoDS_Shape ProfShape)
    {
        if (myMap.IsBound(SpineShape) && myMap[SpineShape].IsBound(ProfShape))
        {
            return myMap[SpineShape][ProfShape];
        }
        else
        {
            return new NCollection_List<TopoDS_Shape>();
        }
    }

    private NCollection_DataMap<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> Generated() => myMap;

    private static TopAbs_Orientation Compare(in TopoDS_Edge E1, in TopoDS_Edge E2)
    {
        TopAbs_Orientation OO = TopAbs_FORWARD;
        TopExp.Vertices(E1, out TopoDS_Vertex V10, out TopoDS_Vertex _);
        TopExp.Vertices(E2, out TopoDS_Vertex V20, out TopoDS_Vertex V21);
        Point3D P1 = BRep_Tool.Pnt(V10);
        Point3D P2 = BRep_Tool.Pnt(V20);
        Point3D P3 = BRep_Tool.Pnt(V21);
        if (P1.Distance(P3) < P1.Distance(P2))
        {
            OO = TopAbs_REVERSED;
        }

        return OO;
    }

    private void Add(BRepFill_Evolved Vevo, in TopoDS_Wire Prof, BRepTools_Quilt Glue)
    {
        NCollection_DataMap<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> MapVevo = Vevo.Generated();
        TopoDS_Shape CurrentSpine, CurrentProf;

        if (Vevo.InternalShape().IsNull())
        {
            return;
        }

        //-------------------------------------------------
        // Find wires common to <me> and <Vevo>.
        //-------------------------------------------------

        TopExp_Explorer ExProf = new();
        for (ExProf.Init(Prof, TopAbs_VERTEX); ExProf.More(); ExProf.Next())
        {
            TopoDS_Shape VV = ExProf.Current();
            //---------------------------------------------------------------
            // Parse edges generated by VV in myMap if they existent
            // and Bind in Glue
            //---------------------------------------------------------------

            //------------------------------------------------- -------------
            // Note: the curves of of reinforced edges are in the same direction
            //          if one remains on the same edge.
            //          if one passes from left to the right they are inverted.
            //------------------------------------------------- -------------
            Relative(myProfile, Prof, TopoDS.Vertex(VV), out bool Commun);

            if (Commun)
            {
                // OCCT iterates myMap in hash order; the bindings made do not depend on it
                foreach (KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> iteS in myMap)
                {
                    TopoDS_Shape SP = iteS.Key;
                    if (iteS.Value.IsBound(VV) && MapVevo.IsBound(SP) && MapVevo[SP].IsBound(VV))
                    {
                        NCollection_List<TopoDS_Shape> MyList = myMap[SP][VV];
                        NCollection_List<TopoDS_Shape> VevoList = Vevo.InternalGeneratedShapes(SP, VV);
                        NCollection_List<TopoDS_Shape>.Iterator MyIte = new(MyList);
                        NCollection_List<TopoDS_Shape>.Iterator VevoIte = new(VevoList);
                        for (; MyIte.More(); MyIte.Next(), VevoIte.Next())
                        {
                            TopoDS_Edge ME = TopoDS.Edge(MyIte.Value());
                            TopoDS_Edge VE = TopoDS.Edge(VevoIte.Value());
                            TopAbs_Orientation OG = Compare(ME, VE);
                            TopoDS_Shape aLocalShape = VE.Oriented(TopAbs_FORWARD);
                            TopoDS_Shape aLocalShape2 = ME.Oriented(OG);
                            Glue.Bind(TopoDS.Edge(aLocalShape), TopoDS.Edge(aLocalShape2));
                        }
                    }
                }
            }
        }
        Glue.Add(Vevo.InternalShape());

        //----------------------------------------------------------
        // Add map of elements generate in Vevo in myMap.
        //----------------------------------------------------------
        foreach (KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> iteS in MapVevo)
        {
            CurrentSpine = iteS.Key;
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> iteP in iteS.Value)
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
                    myMap[CurrentSpine].Bind(CurrentProf, new NCollection_List<TopoDS_Shape>());
                    NCollection_List<TopoDS_Shape> GenShapes = iteP.Value;
                    NCollection_List<TopoDS_Shape>.Iterator itl = new(GenShapes);
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

    private ref TopoDS_Shape ChangeShape() => ref myShape;

    private void Transfert(BRepFill_Evolved Vevo,
                           NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapProf,
                           NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapSpine,
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
        BRep_Builder B = new();
        TopExp_Explorer ex = new(myShape, TopAbs_EDGE);
        while (ex.More())
        {
            B.SameRange(TopoDS.Edge(ex.Current()), false);
            B.SameParameter(TopoDS.Edge(ex.Current()), false);
            BRepLib.SameParameter(TopoDS.Edge(ex.Current()));
            ex.Next();
        }

        //--------------------------------------------------------------
        // Transfer of myMap of Vevo into myMap.
        //--------------------------------------------------------------
        TopoDS_Shape InitialSpine, InitialProf;

        NCollection_DataMap<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> MapVevo = Vevo.Generated();

        // OCCT iterates MapVevo in hash order: the keys of myMap are bound in another order, which only changes
        // the order of later iterations over myMap (none in a published result)
        foreach (KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> iteS in MapVevo)
        {
            InitialSpine = MapSpine[iteS.Key];
            InitialSpine.Move(LS);

            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> iteP in iteS.Value)
            {
                InitialProf = MapProf[iteP.Key];
                InitialProf.Location(InitLP);

                NCollection_List<TopoDS_Shape> GenShapes = iteP.Value;

                NCollection_List<TopoDS_Shape>.Iterator itl = new();
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
                    myMap[InitialSpine].Bind(InitialProf, new NCollection_List<TopoDS_Shape>());
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
    public TopoDS_Shape Shape()
    {
        FreezeResult();
        return myShape;
    }

    /// <summary>OCCT's <c>Shape</c>: the result, not frozen, for ported code that goes on building.</summary>
    internal TopoDS_Shape InternalShape() => myShape;

    private void FreezeResult()
    {
        myShape.TShape()?.FreezeGraph();
        myTop.TShape()?.FreezeGraph();
        myBottom.TShape()?.FreezeGraph();
    }

    /// <summary>The join type of the parallels.</summary>
    public global::Kerf.Geometry.JoinKind JoinType() => myJoinType;

    private void AddTopAndBottom(BRepTools_Quilt Glue)
    {
        //  return first and last vertex of the profile.
        TopoDS_Vertex[] V = [new(), new()];
        TopExp.Vertices(myProfile, out V[0], out V[1]);
        if (V[0].IsSame(V[1]))
        {
            return;
        }

        NCollection_List<TopoDS_Shape>.Iterator itL = new();
        bool ToReverse = false;
        for (int i = 0; i <= 1; i++)
        {
            BRepAlgo_Loop Loop = new();
            // Construction of supports.
            Plane3D S = new(0.0, 0.0, 1.0, -Altitud(V[i]));
            TopoDS_Face F = new BRepLib_MakeFace(S).InternalFace();
            Loop.Init(F);

            TopExp_Explorer ExpSpine = new(mySpine, TopAbs_EDGE);
            NCollection_Map<TopoDS_Shape> View = new(TopTools_ShapeMapHasher.Instance);

            for (; ExpSpine.More(); ExpSpine.Next())
            {
                TopoDS_Edge ES = TopoDS.Edge(ExpSpine.Current());
                NCollection_List<TopoDS_Shape> L = InternalGeneratedShapes(ES, V[i]);
                bool ComputeOrientation = false;

                for (itL.Initialize(L); itL.More(); itL.Next())
                {
                    TopoDS_Edge E = TopoDS.Edge(itL.Value());

                    if (!ComputeOrientation)
                    {
                        BRepAdaptor_Curve C1 = new(ES);
                        BRepAdaptor_Curve C2 = new(E);
                        BRep_Tool.Range(E, out double f, out double l);
                        BRep_Tool.Range(ES, out double fs, out double ls);
                        double u = 0.3 * f + 0.7 * l;
                        double us = 0.3 * fs + 0.7 * ls;
                        C1.D1(us, out Point3D _, out Vector3D V1);
                        C2.D1(u, out Point3D _, out Vector3D V2);
                        ToReverse = (V1.Dot(V2) < 0.0);
                        ComputeOrientation = true;
                    }

                    TopAbs_Orientation Or = ES.Orientation();
                    if (ToReverse)
                    {
                        Or = TopAbs.Reverse(Or);
                    }
                    TopoDS_Shape aLocalShape = E.Oriented(Or);
                    Loop.AddConstEdge(TopoDS.Edge(aLocalShape));
                }
            }

            Point3D PV = BRep_Tool.Pnt(V[i]);
            bool IsOut = PV.Y() < 0;

            for (ExpSpine.Init(mySpine, TopAbs_VERTEX); ExpSpine.More(); ExpSpine.Next())
            {
                TopoDS_Vertex ES = TopoDS.Vertex(ExpSpine.Current());
                if (View.Add(ES))
                {
                    NCollection_List<TopoDS_Shape> L = InternalGeneratedShapes(ES, V[i]);
                    for (itL.Initialize(L); itL.More(); itL.Next())
                    {
                        TopoDS_Edge E = TopoDS.Edge(itL.Value());
                        if (!BRep_Tool.Degenerated(E))
                        {
                            // the center of circle (ie vertex) is IN the cap if vertex IsOut
                            //                                    OUT                   !IsOut
                            BRepAdaptor_Curve C = new(E);
                            BRep_Tool.Range(E, out double f, out double l);
                            double u = 0.3 * f + 0.7 * l;
                            Point3D P = BRep_Tool.Pnt(ES);
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
                            TopAbs_Orientation Or = TopAbs_FORWARD;
                            if (ToReverse)
                            {
                                Or = TopAbs_REVERSED;
                            }
                            TopoDS_Shape aLocalShape = E.Oriented(Or);
                            Loop.AddConstEdge(TopoDS.Edge(aLocalShape));
                        }
                    }
                }
            }

            Loop.Perform();
            Loop.WiresToFaces();
            NCollection_List<TopoDS_Shape> LF = Loop.NewFaces();
            NCollection_List<TopoDS_Shape>.Iterator anIterL = new(LF);

            // Maj of myTop and myBottom for the history
            // and addition of constructed faces.
            TopoDS_Compound Bouchon = new();
            BRep_Builder B = new();
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
        TopExp_Explorer exp = new(myShape, TopAbs_SHELL);
        int ish = 0;
        TopoDS_Compound Res = new();
        TopoDS_Solid Sol = new();
        BRep_Builder B = new();
        B.MakeCompound(ref Res);

        for (; exp.More(); exp.Next())
        {
            TopoDS_Shape Sh = exp.Current();
            Sol = new TopoDS_Solid();
            B.MakeSolid(ref Sol);
            B.Add(Sol, Sh);
            BRepClass3d_SolidClassifier SC = new(Sol);
            SC.PerformInfinitePoint(BRepFill_Confusion());
            if (SC.State() == TopAbs_IN)
            {
                Sol = new TopoDS_Solid();
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

    private void MakePipe(in TopoDS_Edge SE, in CoordinateSystem AxeRef)
    {
        BRepTools_WireExplorer ProfExp = new();

        Transform3D trsf = new();
        if (Side(myProfile, BRepFill_Confusion()) > 3)
        { // side right
            trsf.SetRotation(Constants.OZ(), Math.PI);
        }
        Placement DumLoc = new(trsf);
        TopoDS_Shape aLocalShape = myProfile.Moved(DumLoc);
        TopoDS_Wire DummyProf = PutProfilAt(TopoDS.Wire(aLocalShape), AxeRef, SE, mySpine, true);

        // Copy of the profile to avoid the accumulation of
        // locations on the Edges of myProfile!

        BRepTools_TrsfModification TrsfMod = new(new Transform3D());
        BRepTools_Modifier Modif = new(DummyProf, TrsfMod);

        TopoDS_Wire GenProf = TopoDS.Wire(Modif.InternalModifiedShape(DummyProf));

        BRepFill_Pipe Pipe = new(new BRepLib_MakeWire(SE).InternalWire(), GenProf);

        //---------------------------------------------
        // Arrangement of Tubes in myMap.
        //---------------------------------------------

        BRepTools_WireExplorer GenProfExp = new();
        bool FirstVertex = true;

        myMap.Bind(SE, NewProfMap());

        for (ProfExp.Init(myProfile), GenProfExp.Init(GenProf); ProfExp.More(); ProfExp.Next(), GenProfExp.Next())
        {
            EdgeVertices(ProfExp.Current(), out TopoDS_Vertex VF, out TopoDS_Vertex VL);
            EdgeVertices(GenProfExp.Current(), out TopoDS_Vertex VFG, out TopoDS_Vertex VLG);

            if (FirstVertex)
            {
                myMap[SE].Bind(VF, new NCollection_List<TopoDS_Shape>());
                myMap[SE][VF].Append(Pipe.InternalEdge(SE, VFG));
                FirstVertex = false;
            }
            myMap[SE].Bind(VL, new NCollection_List<TopoDS_Shape>());
            myMap[SE][VL].Append(Pipe.InternalEdge(SE, VLG));
            myMap[SE].Bind(ProfExp.Current(), new NCollection_List<TopoDS_Shape>());
            myMap[SE][ProfExp.Current()].Append(Pipe.InternalFace(SE, GenProfExp.Current()));
        }
    }

    private void MakeRevol(in TopoDS_Edge SE, in TopoDS_Vertex VLast, in CoordinateSystem AxeRef)
    {
        BRepTools_WireExplorer ProfExp = new();

        Transform3D trsf = new();
        if (Side(myProfile, BRepFill_Confusion()) > 3)
        { // side right
            trsf.SetRotation(Constants.OZ(), Math.PI);
        }
        Placement DumLoc = new(trsf);
        TopoDS_Shape aLocalShape = myProfile.Moved(DumLoc);
        TopoDS_Wire GenProf = PutProfilAt(TopoDS.Wire(aLocalShape), AxeRef, SE, mySpine, false);

        Axis3D AxeRev = new(BRep_Tool.Pnt(VLast), Constants.DZ().Reversed());

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
        BRepTools_WireExplorer GenProfExp = new();
        bool FirstVertex = true;

        myMap.Bind(VLast, NewProfMap());

        for (ProfExp.Init(myProfile), GenProfExp.Init(GenProf); ProfExp.More(); ProfExp.Next(), GenProfExp.Next())
        {
            EdgeVertices(ProfExp.Current(), out TopoDS_Vertex VF, out TopoDS_Vertex VL);
            EdgeVertices(GenProfExp.Current(), out TopoDS_Vertex VFG, out TopoDS_Vertex VLG);

            TopAbs_Orientation Or = GenProfExp.Current().Orientation();

            if (FirstVertex)
            {
                myMap[VLast].Bind(VF, new NCollection_List<TopoDS_Shape>());
                TopoDS_Shape RV0 = Rev.Shape(VFG);
                TopAbs_Orientation OO0 = RV0.Orientation();
                myMap[VLast][VF].Append(RV0.Oriented(OO0));
                FirstVertex = false;
            }
            myMap[VLast].Bind(ProfExp.Current(), new NCollection_List<TopoDS_Shape>());
            TopoDS_Shape RF = Rev.Shape(GenProfExp.Current());
            TopAbs_Orientation OO = TopAbs.Compose(RF.Orientation(), Or);

            myMap[VLast][ProfExp.Current()].Append(RF.Oriented(OO));
            myMap[VLast].Bind(VL, new NCollection_List<TopoDS_Shape>());
            TopoDS_Shape RV = Rev.Shape(VLG);
            OO = RV.Orientation();
            myMap[VLast][VL].Append(RV.Oriented(OO));
        }
    }

    private static Placement FindLocation(in TopoDS_Face Face)
    {
        Geom_Surface S = BRep_Tool.Surface(Face, out Placement L)!;

        if (S is not Geom_Plane)
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
            S = (Geom_Surface)S.Transformed(L.Transformation());
        }

        Geom_Plane P = (Geom_Plane)S;
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
    private void ContinuityOnOffsetEdge(NCollection_List<TopoDS_Shape> WorkProf)
    {
        BRepTools_WireExplorer WExp = new();
        TopoDS_Vertex VF, VL, V;
        TopoDS_Edge PrecE, CurE = new(), FirstE;
        BRep_Builder B = new();

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
                double U1 = BRep_Tool.Parameter(V, CurE);
                double U2 = BRep_Tool.Parameter(V, PrecE);
                BRepAdaptor_Curve Curve1 = new(CurE);
                BRepAdaptor_Curve Curve2 = new(PrecE);
                Continuity Continuity = BRepLProp.Continuity(Curve1, Curve2, U1, U2);

                if (Continuity >= global::Kerf.Geometry.Continuity.G1)
                {
                    //-----------------------------------------------------
                    // Code continuity for all edges generated by V.
                    //-----------------------------------------------------
                    foreach (KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> iteS in myMap)
                    {
                        TopoDS_Shape SP = iteS.Key;
                        if (myMap[SP].IsBound(V) && myMap[SP].IsBound(CurE) && myMap[SP].IsBound(PrecE))
                        {
                            if (!myMap[SP][V].IsEmpty() && !myMap[SP][CurE].IsEmpty() && !myMap[SP][PrecE].IsEmpty())
                            {
                                B.Continuity(TopoDS.Edge(myMap[SP][V].First()),
                                             TopoDS.Face(myMap[SP][CurE].First()),
                                             TopoDS.Face(myMap[SP][PrecE].First()),
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
            double U1 = BRep_Tool.Parameter(VF, CurE);
            double U2 = BRep_Tool.Parameter(VF, FirstE);
            BRepAdaptor_Curve Curve1 = new(CurE);
            BRepAdaptor_Curve Curve2 = new(FirstE);
            Continuity Continuity = BRepLProp.Continuity(Curve1, Curve2, U1, U2);

            if (Continuity >= global::Kerf.Geometry.Continuity.G1)
            {
                //---------------------------------------------
                // Code continuity for all edges generated by V.
                //---------------------------------------------
                foreach (KeyValuePair<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>> iteS in myMap)
                {
                    TopoDS_Shape SP = iteS.Key;
                    if (myMap[SP].IsBound(VF) && myMap[SP].IsBound(CurE) && myMap[SP].IsBound(FirstE))
                    {
                        if (!myMap[SP][VF].IsEmpty() && !myMap[SP][CurE].IsEmpty() && !myMap[SP][FirstE].IsEmpty())
                        {
                            B.Continuity(TopoDS.Edge(myMap[SP][VF].First()),
                                         TopoDS.Face(myMap[SP][CurE].First()),
                                         TopoDS.Face(myMap[SP][FirstE].First()),
                                         Continuity);
                        }
                    }
                }
            }
        }
    }

    // Degenerated edges can be missing in some face: the missing degenerated edges have vertices corresponding to node
    // of the map. Now it is enough to compare points UV of vertices on edges with a certain tolerance.
    private static void AddDegeneratedEdge(in TopoDS_Face F, in TopoDS_Wire W)
    {
        Geom_Surface S = BRep_Tool.Surface(F, out Placement _)!;
        if (S.GetType() == typeof(Geom_RectangularTrimmedSurface))
        {
            Geom_Surface SB = ((Geom_RectangularTrimmedSurface)S).BasisSurface();
            if (SB.GetType() == typeof(Geom_Plane))
            {
                return;
            }
        }

        if (S.GetType() == typeof(Geom_Plane))
        {
            return;
        }

        BRep_Builder B = new();
        double TolConf = 1.0e-4;

        bool Change = true;

        while (Change)
        {
            Change = false;
            BRepTools_WireExplorer WE = new(W, F);
            Point2D PF = new(), PrevP = new(), P1, P2 = new();
            TopoDS_Vertex VF = new(), V1, V2 = new();

            for (; WE.More(); WE.Next())
            {
                TopoDS_Edge CE = WE.Current();
                EdgeVertices(CE, out V1, out V2);
                if (CE.Orientation() == TopAbs_REVERSED)
                {
                    BRep_Tool.UVPoints(CE, F, out P2, out P1);
                }
                else
                {
                    BRep_Tool.UVPoints(CE, F, out P1, out P2);
                }
                if (VF.IsNull())
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
                        Geom2d_Line C2d = new(PrevP, new Direction2D(V));
                        double f = 0, l = PrevP.Distance(P1);
                        TopoDS_Edge NE = new BRepLib_MakeEdge(C2d, S).InternalEdge();
                        B.Degenerated(NE, true);
                        B.Add(NE, V1.Oriented(TopAbs_FORWARD));
                        B.Add(NE, V1.Oriented(TopAbs_REVERSED));
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
                    Geom2d_Line C2d = new(P2, new Direction2D(V));
                    double f = 0, l = P2.Distance(PF);
                    TopoDS_Edge NE = new BRepLib_MakeEdge(C2d, S).InternalEdge();
                    B.Degenerated(NE, true);
                    B.Add(NE, VF.Oriented(TopAbs_FORWARD));
                    B.Add(NE, VF.Oriented(TopAbs_REVERSED));
                    B.Range(NE, f, l);
                    B.Add(W, NE);
                }
            }
        }
    }

    private static void TrimFace(in TopoDS_Face Face, NCollection_Sequence<TopoDS_Shape> TheEdges, NCollection_Sequence<TopoDS_Shape> S)
    {
        //--------------------------------------
        // Creation of wires limiting faces.
        //--------------------------------------
        BRep_Builder TheBuilder = new();

        int NbEdges;
        bool NewWire;
        bool AddEdge;
        TopoDS_Wire GoodWire;

        while (!TheEdges.IsEmpty())
        {
            BRepLib_MakeWire MWire = new(TopoDS.Edge(TheEdges.First()));
            GoodWire = MWire.InternalWire();
            TheEdges.Remove(1);
            NbEdges = TheEdges.Length();
            NewWire = false;

            while (!NewWire)
            {
                AddEdge = false;

                for (int i = 1; i <= NbEdges && !AddEdge; i++)
                {
                    TopoDS_Edge E = TopoDS.Edge(TheEdges.Value(i));
                    if (BRep_Tool.Degenerated(E))
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
            TopoDS_Shape aLocalShape = Face.EmptyCopied();
            TopoDS_Face FaceCut = TopoDS.Face(aLocalShape);
            FaceCut.Orientation(TopAbs_FORWARD);
            BRepTools.Update(FaceCut);
            AddDegeneratedEdge(FaceCut, GoodWire);
            TheBuilder.Add(FaceCut, GoodWire);
            FaceCut.Orientation(Face.Orientation());
            S.Append(FaceCut);
        }
    }

    private static TopoDS_Wire PutProfilAt(in TopoDS_Wire ProfRef, in CoordinateSystem AxeRef, in TopoDS_Edge E, in TopoDS_Face F, bool AtStart)
    {
        Vector2D D1;
        Point2D P;
        TopoDS_Wire Prof;

        Geom2d_Curve? C2d = BRep_Tool.CurveOnSurface(E, F, out double First, out double Last);
        if (C2d is null)
        {
            throw new ConstructionErrorException("ConstructionError in PutProfilAt");
        }

        if (E.Orientation() == TopAbs_REVERSED)
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
        TopoDS_Shape aLocalShape = ProfRef.Moved(new Placement(Trans));
        Prof = TopoDS.Wire(aLocalShape);
        return Prof;
    }

    private static void TrimEdge(in TopoDS_Edge Edge,
                                 NCollection_Sequence<TopoDS_Shape> TheEdgesControle,
                                 NCollection_Sequence<TopoDS_Shape> TheVer,
                                 NCollection_Sequence<double> ThePar,
                                 NCollection_Sequence<TopoDS_Shape> S)
    {
        bool Change = true;
        BRep_Builder TheBuilder = new();
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
        if (!BRep_Tool.Degenerated(Edge))
        {
            for (int k = 1; k <= TheVer.Length(); k++)
            {
                if (DoubleOrNotInFace(TheEdgesControle, TopoDS.Vertex(TheVer.Value(k))))
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
        if (!BRep_Tool.Degenerated(Edge))
        {
            for (int k = 1; k < TheVer.Length(); k++)
            {
                if (TheVer.Value(k).IsSame(TheVer.Value(k + 1)))
                {
                    TheVer.Remove(k + 1);
                    ThePar.Remove(k + 1);
                    if (DoubleOrNotInFace(TheEdgesControle, TopoDS.Vertex(TheVer.Value(k))))
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
            TopoDS_Shape aLocalShape = Edge.EmptyCopied();
            TopoDS_Edge NewEdge = TopoDS.Edge(aLocalShape);

            if (NewEdge.Orientation() == TopAbs_REVERSED)
            {
                TheBuilder.Add(NewEdge, TheVer.Value(k).Oriented(TopAbs_REVERSED));
                TheBuilder.Add(NewEdge, TheVer.Value(k + 1).Oriented(TopAbs_FORWARD));
            }
            else
            {
                TheBuilder.Add(NewEdge, TheVer.Value(k).Oriented(TopAbs_FORWARD));
                TheBuilder.Add(NewEdge, TheVer.Value(k + 1).Oriented(TopAbs_REVERSED));
            }
            TheBuilder.Range(NewEdge, ThePar.Value(k), ThePar.Value(k + 1));
            //  modified by NIZHNY-EAP Wed Dec 22 12:09:48 1999 ___BEGIN___
            BRepLib.UpdateTolerances(NewEdge, false);
            //  modified by NIZHNY-EAP Wed Dec 22 13:34:19 1999 ___END___
            S.Append(NewEdge);
        }
    }

    private static void ComputeIntervals(NCollection_Sequence<TopoDS_Shape> VOnF,
                                         NCollection_Sequence<TopoDS_Shape> VOnL,
                                         NCollection_Sequence<Point3D> ParOnF,
                                         NCollection_Sequence<Point3D> ParOnL,
                                         BRepFill_TrimSurfaceTool Trim,
                                         Geom2d_Curve Bis,
                                         in TopoDS_Vertex VS,
                                         in TopoDS_Vertex VE,
                                         NCollection_Sequence<double> FirstPar,
                                         NCollection_Sequence<double> LastPar,
                                         NCollection_Sequence<TopoDS_Shape> FirstV,
                                         NCollection_Sequence<TopoDS_Shape> LastV)
    {
        int IOnF = 1, IOnL = 1;
        double U1 = 0.0, U2;
        TopoDS_Shape V1 = new(), V2;

        if (!VS.IsNull())
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
            if (!V1.IsNull() && !V2.IsSame(V1))
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

        if (!VE.IsNull())
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
    private static TopAbs_Orientation Relative(in TopoDS_Wire W1, in TopoDS_Wire W2, in TopoDS_Vertex V, out bool Commun)
    {
        TopExp_Explorer Exp = new();
        TopoDS_Edge E1 = new(), E2 = new();

        for (Exp.Init(W1, TopAbs_EDGE); Exp.More(); Exp.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(Exp.Current());
            TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            if (V1.IsSame(V) || V2.IsSame(V))
            {
                E1 = E;
                break;
            }
        }
        for (Exp.Init(W2, TopAbs_EDGE); Exp.More(); Exp.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(Exp.Current());
            TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            if (V1.IsSame(V) || V2.IsSame(V))
            {
                E2 = E;
                break;
            }
        }

        if (E1.IsNull() || E2.IsNull())
        {
            Commun = false;
            return TopAbs_FORWARD;
        }
        Commun = true;

        TopoDS_Wire WW1 = new BRepLib_MakeWire(E1).InternalWire();
        TopoDS_Wire WW2 = new BRepLib_MakeWire(E2).InternalWire();
        double Tol = BRepFill_Confusion();
        if (Side(WW1, Tol) < 4 && Side(WW2, Tol) < 4)
        { // two to the left
            return TopAbs_FORWARD;
        }
        if (Side(WW1, Tol) > 4 && Side(WW2, Tol) > 4)
        { // two to the right
            return TopAbs_FORWARD;
        }

        return TopAbs_REVERSED;
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
    private static bool DoubleOrNotInFace(NCollection_Sequence<TopoDS_Shape> EC, in TopoDS_Vertex V)
    {
        bool Vu = false;

        for (int i = 1; i <= EC.Length(); i++)
        {
            TopExp.Vertices(TopoDS.Edge(EC.Value(i)), out TopoDS_Vertex V1, out TopoDS_Vertex V2);
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

    private static double DistanceToOZ(in TopoDS_Vertex V)
    {
        Point3D PV3d = BRep_Tool.Pnt(V);
        return Math.Abs(PV3d.Y());
    }

    private static double Altitud(in TopoDS_Vertex V)
    {
        Point3D PV3d = BRep_Tool.Pnt(V);
        return PV3d.Z();
    }

    // OCCT hands on the locus's own trimmed curve (Bisector_Bisec::Value) unless its basis is a BisecAna; the port does
    // the same (BRepFill_TrimSurfaceTool may lengthen it in place, as natively).
    private static void SimpleExpression(Bisector_Bisec B, out Geom2d_Curve Bis)
    {
        Bis = B.Value();

        if (Bis.GetType() == typeof(Geom2d_TrimmedCurve))
        {
            Geom2d_TrimmedCurve TrBis = (Geom2d_TrimmedCurve)Bis;
            Geom2d_Curve BasBis = TrBis.ChangeBasisCurve();
            if (BasBis.GetType() == typeof(Bisector_BisecAna))
            {
                Bis = ((Bisector_BisecAna)BasBis).Geom2dCurve();
                Bis = new Geom2d_TrimmedCurve(Bis, TrBis.FirstParameter(), TrBis.LastParameter());
            }
        }
    }

    // Projection and Cut of an edge at extrema of distance to axis OZ.
    private static void CutEdgeProf(in TopoDS_Edge E,
                                    Geom_Plane Plane,
                                    Geom2d_Line Line,
                                    NCollection_List<TopoDS_Shape> Cuts,
                                    NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapVerRefMoved)
    {
        Cuts.Clear();

        Geom_Curve C;
        Geom_TrimmedCurve CT;
        Geom2d_Curve C2d;

        // Return the curve associated to each Edge
        C = BRep_Tool.Curve(E, out Placement L, out double f, out double l)!;
        CT = new Geom_TrimmedCurve(C, f, l);
        CT.Transform(L.Transformation());

        // project it in the plane and return the associated PCurve
        Direction3D Normal = Plane.Pln().Axis().Direction();
        C = GeomProjLib.ProjectOnPlane(CT, Plane, Normal, false)!;
        C2d = GeomProjLib.Curve2d(C, Plane)!;

        // Calculate the extrema with the straight line
        NCollection_Sequence<double> Seq = new();

        double U1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double U2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        f = C2d.FirstParameter();
        l = C2d.LastParameter();

        Box2d B = new();
        Geom2dAdaptor_Curve AC2d = new(C2d);
        BndLib_Add2dCurve.Add(AC2d, BRepFill_Confusion(), ref B);
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
        Geom2dAdaptor_Curve ALine = new(Line);
        double Tol = global::Kerf.Engine.Foundation.Precision.Intersection();
        double TolC = 0.0;

        Geom2dInt_GInter Intersector = new(ALine, AC2d, TolC, Tol);
        int NbP = Intersector.NbPoints();

        for (int i = 1; i <= NbP; i++)
        {
            Seq.Append(Intersector.Point(i).ParamOnSecond());
        }

        // Compute the new edges.
        BRep_Builder Builder = new();
        TopoDS_Vertex VV, Vf = new(), Vl = new();
        TopExp.Vertices(E, out TopoDS_Vertex VRf, out TopoDS_Vertex VRl);

        if (!MapVerRefMoved.IsBound(VRf))
        {
            Builder.MakeVertex(ref Vf, C.Value(f), BRep_Tool.Tolerance(VRf));
            MapVerRefMoved.Bind(VRf, Vf);
        }
        else
        {
            Vf = TopoDS.Vertex(MapVerRefMoved[VRf]);
        }

        if (!MapVerRefMoved.IsBound(VRl))
        {
            Builder.MakeVertex(ref Vl, C.Value(l), BRep_Tool.Tolerance(VRl));
            MapVerRefMoved.Bind(VRl, Vl);
        }
        else
        {
            Vl = TopoDS.Vertex(MapVerRefMoved[VRl]);
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

                    TopoDS_Edge EE0 = new BRepLib_MakeEdge(C, Vf, VV).InternalEdge();
                    EE0.Orientation(E.Orientation());
                    if (EE0.Orientation() == TopAbs_FORWARD)
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

        TopoDS_Edge EE = new BRepLib_MakeEdge(C, Vf, Vl).InternalEdge();
        EE.Orientation(E.Orientation());
        if (EE.Orientation() == TopAbs_FORWARD)
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
    private static void CutEdge(in TopoDS_Edge E, in TopoDS_Face F, NCollection_List<TopoDS_Shape> Cuts)
    {
        Cuts.Clear();
        MAT2d_CutCurve Cuter = new();
        Geom2d_TrimmedCurve CT2d;

        TopoDS_Vertex VF, VL;
        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        BRep_Builder B = new();

        Geom2d_Curve C2d = BRep_Tool.CurveOnSurface(E, F, out double f, out double l)!;
        CT2d = new Geom2d_TrimmedCurve(C2d, f, l);

        if (CT2d.ChangeBasisCurve() is Geom2d_Circle && BRep_Tool.IsClosed(E))
        {
            //---------------------------
            // Cut closed circle.
            //---------------------------
            double m1 = (2 * f + l) / 3.0;
            double m2 = (f + 2 * l) / 3.0;
            Point2D P1 = CT2d.Value(m1);
            Point2D P2 = CT2d.Value(m2);

            TopoDS_Vertex VL1 = new BRepLib_MakeVertex(new Point3D(P1.X(), P1.Y(), 0.0)).InternalVertex();
            TopoDS_Vertex VL2 = new BRepLib_MakeVertex(new Point3D(P2.X(), P2.Y(), 0.0)).InternalVertex();
            TopoDS_Shape aLocalShape1 = E.EmptyCopied();
            TopoDS_Shape aLocalShape2 = E.EmptyCopied();
            TopoDS_Shape aLocalShape3 = E.EmptyCopied();
            TopoDS_Edge FE = TopoDS.Edge(aLocalShape1);
            TopoDS_Edge ME = TopoDS.Edge(aLocalShape2);
            TopoDS_Edge LE = TopoDS.Edge(aLocalShape3);

            FE.Orientation(TopAbs_FORWARD);
            ME.Orientation(TopAbs_FORWARD);
            LE.Orientation(TopAbs_FORWARD);

            B.Add(FE, V1);
            B.Add(FE, VL1.Oriented(TopAbs_REVERSED));
            B.Range(FE, f, m1);

            B.Add(ME, VL1.Oriented(TopAbs_FORWARD));
            B.Add(ME, VL2.Oriented(TopAbs_REVERSED));
            B.Range(ME, m1, m2);

            B.Add(LE, VL2.Oriented(TopAbs_FORWARD));
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
                Geom2d_TrimmedCurve CC = Cuter.Value(k);
                if (k == Cuter.NbCurves())
                {
                    VL = V2;
                }
                else
                {
                    Point2D P = CC.Value(CC.LastParameter());
                    VL = new BRepLib_MakeVertex(new Point3D(P.X(), P.Y(), 0.0)).InternalVertex();
                }
                TopoDS_Shape aLocalShape = E.EmptyCopied();
                TopoDS_Edge NE = TopoDS.Edge(aLocalShape);
                NE.Orientation(TopAbs_FORWARD);
                B.Add(NE, VF.Oriented(TopAbs_FORWARD));
                B.Add(NE, VL.Oriented(TopAbs_REVERSED));
                B.Range(NE, CC.FirstParameter(), CC.LastParameter());
                Cuts.Append(NE.Oriented(E.Orientation()));
                VF = VL;
            }
        }
    }

    // Tests the position of aNode relative to the distances to OZ of the vertices VF and VL, and returns it; if it is not
    // 0, the vertex VN corresponding to aNode is created (once per node and shape).
    private static int VertexFromNode(MAT_Node aNode,
                                      in TopoDS_Edge E,
                                      in TopoDS_Vertex VF,
                                      in TopoDS_Vertex VL,
                                      NCollection_DataMap<MAT_Node, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>> MapNodeVertex,
                                      ref TopoDS_Vertex VN)
    {
        TopoDS_Shape ShapeOnNode = new();
        int Status = 0;
        BRep_Builder B = new();

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

        if (!ShapeOnNode.IsNull())
        {
            //-------------------------------------------------
            // the vertex will correspond to a node of the map
            //-------------------------------------------------
            if (MapNodeVertex.IsBound(aNode) && MapNodeVertex[aNode].IsBound(ShapeOnNode))
            {
                VN = TopoDS.Vertex(MapNodeVertex[aNode][ShapeOnNode]);
            }
            else
            {
                VN = new TopoDS_Vertex();
                B.MakeVertex(ref VN);
                if (!MapNodeVertex.IsBound(aNode))
                {
                    MapNodeVertex.Bind(aNode, new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance));
                }
                MapNodeVertex[aNode].Bind(ShapeOnNode, VN);
            }
        }
        return Status;
    }
}
