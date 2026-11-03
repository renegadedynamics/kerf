// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetWire.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
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

namespace Occt;

/// <summary>
/// Constructs a planar parallel (offset) of the wires of a planar face, through the medial axis of the face: the
/// parallels of its edges and arcs around its vertices, trimmed where they meet on the bisectors.
/// </summary>
/// <remarks>
/// <para>A working object (decision 21), as OCCT's. <see cref="Shape"/>, <see cref="Spine"/> and
/// <see cref="GeneratedShapes"/> are publication points: they freeze what they return (and the whole result with a
/// generated shape). <see cref="InternalShape"/>, <see cref="InternalSpine"/> and <see cref="InternalGeneratedShapes"/>
/// don't, for ported callers that go on building on the result (BRepOffsetAPI_MakeOffset, BRepFill_Evolved).</para>
/// <para>The algorithm never writes into the face it is given: it works on BRepMAT2d_Explorer's new face and on a work
/// face of cut edges (copies), and builds the parallels from new edges and vertices. A zero offset gives the spine's
/// own wires back, moved by the altitude.</para>
/// </remarks>
public sealed class BRepFill_OffsetWire
{
    private Face mySpine = new();
    private Face myWorkSpine = new();
    private double myOffset;
    private bool myIsOpenResult;
    private Shape myShape = new();
    private bool myIsDone;
    private global::Kerf.Geometry.JoinKind myJoinType;
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myMap = new(EqualityComparer<Shape>.Default);
    private readonly BRepMAT2d_BisectingLocus myBilo = new();
    private readonly BRepMAT2d_LinkTopoBilo myLink = new();
    private readonly NCollection_DataMap<Shape, Shape> myMapSpine = new(ShapeHasher.Instance);
    // OCCT leaves myCallGen uninitialised until Perform; PerformWithBiLo alone reads it in GeneratedShapes, where an
    // empty myMapSpine makes its value irrelevant (occt-issues.md, 5O2)
    private bool myCallGen;

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

    // A face bounded by one closed circle (or any single edge for an open result): its parallel is computed directly.
    private static bool KPartCircle(in Face mySpine,
                                    double myOffset,
                                    bool IsOpenResult,
                                    double Alt,
                                    ref Shape myShape,
                                    NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myMap,
                                    ref bool myIsDone)
    {
        Edge E = new();
        for (Explorer anEdgeIter = new(mySpine, ShapeType.Edge); anEdgeIter.More(); anEdgeIter.Next())
        {
            if (!E.IsNull)
            {
                return false;
            }
            E = ShapeCasts.Edge(anEdgeIter.Current());
        }
        if (E.IsNull)
        {
            return false;
        }

        Curve3D? C = Tool.Curve(E, out Placement L, out double f, out double l);
        if (C is null)
        {
            return false;
        }

        if (C is TrimmedCurve3D Ct)
        {
            C = Ct.BasisCurve();
        }

        if ((C is CircleCurve3D && Tool.IsClosed(E)) || // closed circle
            IsOpenResult)
        {
            double anOffset = myOffset;

            Curve2D aPCurve = Tool.CurveOnSurface(E, mySpine, out f, out l)!;
            CurveAdaptor2D AHC = new(aPCurve, f, l);
            Curve2D OC;
            if (AHC.GetType() == Line)
            {
                if (E.Orientation() == Forward)
                {
                    anOffset *= -1;
                }
                OffsetCurveAdaptor2D Off = new(AHC, anOffset);
                OC = new LineCurve2D(Off.Line());
            }
            else if (AHC.GetType() == Circle)
            {
                if (E.Orientation() == Forward)
                {
                    anOffset *= -1;
                }
                if (!Tool.IsClosed(E))
                {
                    anOffset *= -1;
                }
                Circle2D theCirc = AHC.Circle();
                if (anOffset > 0.0 || Math.Abs(anOffset) < theCirc.Radius())
                {
                    OC = new CircleCurve2D(theCirc.Position(), theCirc.Radius() + anOffset);
                }
                else
                {
                    myIsDone = false;
                    return false;
                }
            }
            else
            {
                if (E.Orientation() == Forward)
                {
                    anOffset *= -1;
                }
                TrimmedCurve2D G2dT = new(aPCurve, f, l);
                OC = new OffsetCurve2D(G2dT, anOffset);
            }
            Surface aSurf = Tool.Surface(mySpine)!;
            PlaneSurface? aPlane = aSurf as PlaneSurface;
            myShape = new BRepLib_MakeEdge(OC, aPlane!, f, l).InternalEdge();
            BRepLib.BuildCurve3d(ShapeCasts.Edge(myShape));

            myShape.Orientation(E.Orientation());
            myShape.Location(L);
            if (Math.Abs(Alt) > Constants.Resolution())
            {
                FaceSurfaceAdaptor S = new(mySpine, false);
                Axis3D Nor = S.Plane().Axis();
                Transform3D T = new();
                Vector3D Trans = new(Nor.Direction());
                Trans = Alt * Trans;
                T.SetTranslation(Trans);
                myShape.Move(new Placement(T));
            }

            NCollection_List<Shape> LL = new();
            LL.Append(myShape);
            myMap.Add(E, LL);

            Edge myEdge = ShapeCasts.Edge(myShape);
            myShape = new BRepLib_MakeWire(myEdge).InternalWire();

            myIsDone = true;
            return true;
        }

        return false;
    }

    /// <summary>An empty algorithm.</summary>
    public BRepFill_OffsetWire()
    {
        myIsOpenResult = false;
        myIsDone = false;
    }

    /// <summary>
    /// Prepares the offset of the wires of <paramref name="Spine"/> (a planar face) with the join type
    /// <paramref name="Join"/>; <paramref name="IsOpenResult"/> asks for open parallels of an open wire.
    /// </summary>
    public BRepFill_OffsetWire(in Face Spine, global::Kerf.Geometry.JoinKind Join = Arc, bool IsOpenResult = false)
    {
        Init(Spine, Join, IsOpenResult);
    }

    /// <summary>
    /// Prepares the offset of the wires of <paramref name="Spine"/>: cuts its edges for the bisectors and computes its
    /// medial axis on the left.
    /// </summary>
    public void Init(in Face Spine, global::Kerf.Geometry.JoinKind Join = Arc, bool IsOpenResult = false)
    {
        myIsDone = false;
        Shape aLocalShape = Spine.Oriented(Forward);
        mySpine = ShapeCasts.Face(aLocalShape);
        myJoinType = Join;
        myIsOpenResult = IsOpenResult;

        myMap.Clear();
        myMapSpine.Clear();

        //------------------------------------------------------------------
        // cut the spine for bissectors.
        //------------------------------------------------------------------
        BRepMAT2d_Explorer Exp = new();
        Exp.Perform(mySpine);
        mySpine = ShapeCasts.Face(Exp.ModifiedShape(mySpine));
        PrepareSpine();

        Shape aShape = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMap = new(EqualityComparer<Shape>.Default);
        bool Done = false;
        if (KPartCircle(myWorkSpine, 1.0, myIsOpenResult, 0.0, ref aShape, aMap, ref Done))
        {
            return;
        }

        //-----------------------------------------------------
        // Calculate the map of bissectors to the left.
        // and Links Topology -> base elements of the map.
        //-----------------------------------------------------

        Exp.Perform(myWorkSpine);
        myBilo.Compute(Exp, 1, MAT_Left, myJoinType, myIsOpenResult);
        myLink.Perform(Exp, myBilo);
    }

    /// <summary>True if the last computation succeeded.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>The face whose wires are offset (BRepMAT2d_Explorer's face; frozen).</summary>
    public Face Spine()
    {
        mySpine.TShape()?.FreezeGraph();
        return mySpine;
    }

    /// <summary><see cref="Spine"/> without freezing, for ported callers.</summary>
    internal Face InternalSpine() => mySpine;

    /// <summary>The parallel: a wire, or a compound of wires (frozen).</summary>
    public Shape Shape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal Shape InternalShape() => myShape;

    /// <summary>
    /// The shapes of the parallel generated by <paramref name="SpineShape"/>, an edge or a vertex of the spine (frozen,
    /// with the whole result).
    /// </summary>
    public ISchematiqList<Shape> GeneratedShapes(in Shape SpineShape)
    {
        NCollection_List<Shape> aList = InternalGeneratedShapes(SpineShape);
        myShape.TShape()?.FreezeGraph();
        if (aList.IsEmpty())
        {
            return PackedList<Shape>.Empty;
        }
        var aShapes = new Shape[aList.Extent()];
        int i = 0;
        foreach (Shape aS in aList)
        {
            aS.TShape()?.FreezeGraph();
            aShapes[i++] = aS;
        }
        return PackedList.Own(aShapes);
    }

    /// <summary>
    /// <see cref="GeneratedShapes"/> without freezing: the algorithm's own list (an empty new list if there is none).
    /// </summary>
    internal NCollection_List<Shape> InternalGeneratedShapes(in Shape SpineShape)
    {
        if (!myCallGen)
        {
            if (!myMapSpine.IsEmpty())
            {
                // myMapSpine can be empty if passed by PerformWithBilo.
                // OCCT iterates the map in address order; the port in insertion order (occt-issues.md, 5O2)
                foreach (KeyValuePair<Shape, Shape> it in myMapSpine)
                {
                    if (myMap.Contains(it.Key))
                    {
                        if (!myMap.Contains(it.Value))
                        {
                            NCollection_List<Shape> L = new();
                            myMap.Add(it.Value, L);
                        }
                        if (!it.Value.IsSame(it.Key))
                        {
                            myMap.ChangeFromKey(it.Value).Append(myMap.ChangeFromKey(it.Key));
                            myMap.RemoveKey(it.Key);
                        }
                    }
                    if (myMap.Contains(it.Key.Reversed()))
                    {
                        if (!myMap.Contains(it.Value.Reversed()))
                        {
                            NCollection_List<Shape> L = new();
                            myMap.Add(it.Value.Reversed(), L);
                        }
                        if (!it.Value.IsSame(it.Key))
                        {
                            myMap.ChangeFromKey(it.Value.Reversed())
                                 .Append(myMap.ChangeFromKey(it.Key.Reversed()));
                            myMap.RemoveKey(it.Key.Reversed());
                        }
                    }
                }
            }
            myCallGen = true;
        }

        if (myMap.Contains(SpineShape))
        {
            return myMap.FindFromKey(SpineShape);
        }
        else
        {
            // OCCT returns a static empty list
            return new NCollection_List<Shape>();
        }
    }

    /// <summary>The join type.</summary>
    public global::Kerf.Geometry.JoinKind JoinType() => myJoinType;

    /// <summary>
    /// Computes the parallel at the distance <paramref name="Offset"/>, at the altitude <paramref name="Alt"/> along
    /// the normal of the spine's plane.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A closed result is expected and a parallel wire isn't closed.</exception>
    public void Perform(double Offset, double Alt = 0.0)
    {
        try
        {
            myCallGen = false;
            if (KPartCircle(myWorkSpine, Offset, myIsOpenResult, Alt, ref myShape, myMap, ref myIsDone))
            {
                return;
            }

            Face oldWorkSpain = myWorkSpine;

            NCollection_List<Shape> BadEdges = new();
            CheckBadEdges(myWorkSpine, Offset, myBilo, myLink, BadEdges);

            if (!BadEdges.IsEmpty())
            {
                // Modification of myWorkSpine;
                Substitution aSubst = new();
                NCollection_List<Shape>.Iterator it = new(BadEdges);
                NCollection_List<Shape> aL = new();
                double aDefl = .01 * Math.Abs(Offset);
                NCollection_Sequence<double> Parameters = new();
                NCollection_Sequence<Point3D> Points = new();

                for (; it.More(); it.Next())
                {
                    aL.Clear();
                    Parameters.Clear();
                    Points.Clear();
                    Shape anE = it.Value();

                    Exploration.Vertices(ShapeCasts.Edge(anE), out Vertex Vf, out Vertex Vl);

                    Curve3D? G3d = Tool.Curve(ShapeCasts.Edge(anE), out double f, out double l);
                    CurveAdaptor AC = new(G3d!, f, l);

                    PerformCurve(Parameters, Points, AC, aDefl, f, l, global::Kerf.Engine.Foundation.Precision.Confusion(), 2);

                    int NPnts = Points.Length();
                    if (NPnts > 2)
                    {
                        Vertex FV = Vf;
                        Vertex LV;
                        Edge newE;
                        int np;
                        for (np = 2; np < NPnts; np++)
                        {
                            Point3D LP = Points[np];
                            LV = new BRepLib_MakeVertex(LP).InternalVertex();
                            newE = new BRepLib_MakeEdge(FV, LV).InternalEdge();
                            aL.Append(newE);
                            FV = LV;
                        }
                        LV = Vl;
                        newE = new BRepLib_MakeEdge(FV, LV).InternalEdge();
                        aL.Append(newE);
                    }
                    else
                    {
                        Edge newE = new BRepLib_MakeEdge(Vf, Vl).InternalEdge();
                        aL.Append(newE);
                    }
                    // Update myMapSpine
                    if (myMapSpine.IsBound(anE))
                    {
                        NCollection_List<Shape>.Iterator newit = new(aL);
                        for (; newit.More(); newit.Next())
                        {
                            Edge NewEdge = ShapeCasts.Edge(newit.Value());
                            myMapSpine.Bind(NewEdge, myMapSpine[anE]);
                            EdgeVertices(NewEdge, out Vertex NewV1, out Vertex NewV2);
                            if (!myMapSpine.IsBound(NewV1))
                            {
                                myMapSpine.Bind(NewV1, myMapSpine[anE]);
                            }
                            if (!myMapSpine.IsBound(NewV2))
                            {
                                myMapSpine.Bind(NewV2, myMapSpine[anE]);
                            }
                        }
                        myMapSpine.UnBind(anE);
                    }
                    ///////////////////
                    aSubst.Substitute(anE, aL);
                }

                NCollection_DataMap<Shape, NCollection_List<Shape>> wwmap = new(ShapeHasher.Instance);
                Iterator itws = new(myWorkSpine);
                for (; itws.More(); itws.Next())
                {
                    Shape aWire = itws.Value();
                    aSubst.Build(aWire);
                    if (aSubst.IsCopied(aWire))
                    {
                        Wire NewWire = ShapeCasts.Wire(aSubst.InternalCopy(aWire).First());
                        NewWire.Closed(aWire.Closed());
                        NCollection_List<Shape> Lw = new();
                        Lw.Append(NewWire);
                        wwmap.Bind(aWire, Lw);
                    }
                }
                aSubst.Clear();
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> itmap in wwmap)
                {
                    aSubst.Substitute(itmap.Key, itmap.Value);
                }

                aSubst.Build(myWorkSpine);

                if (aSubst.IsCopied(myWorkSpine))
                {
                    myWorkSpine = ShapeCasts.Face(aSubst.InternalCopy(myWorkSpine).First());

                    BRepMAT2d_Explorer newExp = new();
                    newExp.Perform(myWorkSpine);
                    BRepMAT2d_BisectingLocus newBilo = new();
                    BRepMAT2d_LinkTopoBilo newLink = new();
                    newBilo.Compute(newExp, 1, MAT_Left, myJoinType, myIsOpenResult);

                    if (!newBilo.IsDone())
                    {
                        myShape.Nullify();
                        myIsDone = false;
                        return;
                    }

                    newLink.Perform(newExp, newBilo);
                    PerformWithBiLo(myWorkSpine, Offset, newBilo, newLink, myJoinType, Alt);
                    myWorkSpine = oldWorkSpain;
                }
                else
                {
                    PerformWithBiLo(myWorkSpine, Offset, myBilo, myLink, myJoinType, Alt);
                }
            }
            else
            {
                PerformWithBiLo(myWorkSpine, Offset, myBilo, myLink, myJoinType, Alt);
            }
        }
        catch (Exception anException) when (anException is FailureException or NullReferenceException
                                            && anException is not (ImmutableObjectException or LockedShapeException))
        {
            // OCC_CATCH_SIGNALS turns OCCT's access violations (a null bisector, occt-issues.md) into failures caught
            // here: the port's NullReferenceException (decision 917)
            myShape.Nullify();
            myIsDone = false;

            return;
        }

        if (!myIsOpenResult)
        {
            Explorer anExp = new(myShape, ShapeType.Wire);

            for (; anExp.More(); anExp.Next())
            {
                Shape aWire = anExp.Current();

                if (!aWire.Closed())
                {
                    myShape.Nullify();
                    myIsDone = false;
                    throw new ConstructionErrorException("Offset wire is not closed.");
                }
            }
        }
    }

    // The parallel at distance 0: the wires of the spine, moved by the altitude.
    private static void Compute(in Face Spine,
                                ref Shape aShape,
                                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Map,
                                double Alt)
    {
        BrepBuilder B = new();
        Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        aShape = aCompound;
        double ALT = Alt;
        if (Spine.Orientation() == Reversed)
        {
            ALT = -Alt;
        }
        Transform3D T = new();
        T.SetTranslation(new Vector3D(0.0, 0.0, ALT));
        Placement L = new(T);

        for (Explorer exp = new(Spine, ShapeType.Wire); exp.More(); exp.Next())
        {
            Wire CurW = ShapeCasts.Wire(exp.Current());
            Shape aLocalShape = CurW.Moved(L);
            Wire NewW = ShapeCasts.Wire(aLocalShape);
            B.Add(aShape, NewW);
            // update Map.
            Iterator it1 = new(CurW);
            Iterator it2 = new(NewW);
            for (; it1.More(); it1.Next(), it2.Next())
            {
                NCollection_List<Shape> List = new();
                List.Append(it2.Value());
                Map.Add(it1.Value(), List);
            }
        }
    }

    /// <summary>
    /// Computes the parallel at the distance <paramref name="Offset"/> of the face <paramref name="Spine"/> from its
    /// bisecting locus <paramref name="Locus"/> and links <paramref name="Link"/>, already computed.
    /// </summary>
    public void PerformWithBiLo(in Face Spine,
                                double Offset,
                                BRepMAT2d_BisectingLocus Locus,
                                BRepMAT2d_LinkTopoBilo Link,
                                global::Kerf.Geometry.JoinKind Join = Arc,
                                double Alt = 0.0)
    {
        myIsDone = false;
        Shape aLocalShapeWork = Spine.Oriented(Forward);
        myWorkSpine = ShapeCasts.Face(aLocalShapeWork);
        myJoinType = Join;
        myOffset = Offset;
        myShape.Nullify();

        if (mySpine.IsNull)
        {
            Shape aLocalShape = Spine.Oriented(Forward);
            mySpine = ShapeCasts.Face(aLocalShape);
        }
        myMap.Clear();

        if (Math.Abs(myOffset) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Compute(mySpine, ref myShape, myMap, Alt);
            myIsDone = true;
            return;
        }

        //********************************
        // Calculate for a non null offset
        //********************************
        if (KPartCircle(myWorkSpine, Offset, myIsOpenResult, Alt, ref myShape, myMap, ref myIsDone))
        {
            return;
        }

        BrepBuilder myBuilder = new();

        //---------------------------------------------------------------------
        // MapNodeVertex : associate to each node of the map (key1) and to
        //                 each element of the profile (key2) a vertex (item).
        // MapBis        : all edges or vertices (item) generated by
        //                 a bisectrice on a face or an edge (key) of revolution tubes.
        // MapVerPar     : Map of parameters of vertices on parallel edges
        //                 the list contained in MapVerPar (E) corresponds to
        //                 parameters on E of vertices contained in MapBis(E);
        //---------------------------------------------------------------------

        // OCCT.NET: keyed by the node's identity, as OCCT's handle; only looked up
        NCollection_DataMap<MAT_Node, Shape> MapNodeVertex = new(ReferenceEqualityComparer.Instance);
        NCollection_DataMap<Shape, NCollection_Sequence<Shape>> MapBis = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, NCollection_Sequence<double>> MapVerPar = new(ShapeHasher.Instance);

        // OCCT.NET: OCCT down-casts (a null plane on another surface, dereferenced below: an access violation natively)
        PlaneSurface RefPlane = (Tool.Surface(myWorkSpine) as PlaneSurface)!;
        if (Math.Abs(Alt) > Constants.Resolution())
        {
            double anAlt = Alt;
            if (myWorkSpine.Orientation() == Reversed)
            {
                anAlt = -Alt;
            }
            RefPlane = (PlaneSurface)RefPlane.Translated(anAlt * new Vector3D(RefPlane.Axis().Direction()));
        }

        //---------------------------------------------------------------
        // Construction of Circles and OffsetCurves
        //---------------------------------------------------------------

        Vertex[] Ends = [new(), new()];
        if (myIsOpenResult)
        {
            Wire theWire;
            Iterator iter = new(mySpine);
            theWire = ShapeCasts.Wire(iter.Value());
            Exploration.Vertices(theWire, out Ends[0], out Ends[1]);
        }

        if (Locus.NumberOfContours() == 0)
        {
            return;
        }

        for (int ic = 1; ic <= Locus.NumberOfContours(); ic++)
        {
            Shape PE = Link.GeneratingShape(Locus.BasicElt(ic, Locus.NumberOfElts(ic)));
            for (int ie = 1; ie <= Locus.NumberOfElts(ic); ie++)
            {
                Shape SE = Link.GeneratingShape(Locus.BasicElt(ic, ie));
                if (SE.ShapeType() == ShapeType.Vertex)
                {
                    if (!SE.IsSame(Ends[0]) && !SE.IsSame(Ends[1]))
                    {
                        MakeCircle(ShapeCasts.Edge(PE), ShapeCasts.Vertex(SE), myWorkSpine, myOffset, myMap, RefPlane);
                    }
                }
                else
                {
                    MakeOffset(ShapeCasts.Edge(SE),
                               myWorkSpine,
                               myOffset,
                               myMap,
                               RefPlane,
                               myIsOpenResult,
                               myJoinType,
                               Ends);
                    PE = SE;
                }
            }
        }

        // Remove possible hanging arcs on vertices
        if (myIsOpenResult && myJoinType == Arc)
        {
            if (!myMap.IsEmpty() && myMap.FindKey(1).ShapeType() == ShapeType.Vertex)
            {
                myMap.RemoveFromIndex(1);
            }
            if (!myMap.IsEmpty() && myMap.FindKey(myMap.Extent()).ShapeType() == ShapeType.Vertex)
            {
                myMap.RemoveLast();
            }
        }

        //---------------------------------------------------
        // Construction of offset vertices.
        //---------------------------------------------------
        NCollection_DataMap<Shape, NCollection_List<Shape>> Detromp = new(EqualityComparer<Shape>.Default);
        MAT_Arc CurrentArc;
        Edge CurrentEdge;
        Shape[] S = [new(), new()];
        Edge[] E = [new(), new()];
        int j, k;

        for (int i = 1; i <= Locus.Graph().NumberOfArcs(); i++)
        {
            CurrentArc = Locus.Graph().Arc(i);
            Bisector_Bisec Bisec = Locus.GeomBis(CurrentArc, out bool Reverse);

            //-------------------------------------------------------------------
            // Return elements of the spine corresponding to separate basicElts.
            //-------------------------------------------------------------------
            S[0] = Link.GeneratingShape(CurrentArc.FirstElement());
            S[1] = Link.GeneratingShape(CurrentArc.SecondElement());

            NCollection_Sequence<Shape> Vertices = new();
            NCollection_Sequence<Point3D> Params = new();

            //-----------------------------------------------------------
            // Return parallel edges on each face.
            // If no offset generated => move to the next bissectrice.
            //--------------------------------------------------------------
            if (myMap.Contains(S[0]) && myMap.Contains(S[1]))
            {
                E[0] = ShapeCasts.Edge(myMap.FindFromKey(S[0]).First());
                E[1] = ShapeCasts.Edge(myMap.FindFromKey(S[1]).First());
            }
            else
            {
                continue;
            }

            //-----------------------------------------------------------
            // Construction of vertices corresponding to the node of the map.
            // if they are on the offset.
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

            bool StartOnEdge = false, EndOnEdge = false;

            if (!Node1.Infinite())
            {
                Point2D aLocalPnt2d = Locus.GeomElt(Node1);
                StartOnEdge = VertexFromNode(Node1, myOffset, aLocalPnt2d, MapNodeVertex, ref VS);
            }
            if (!Node2.Infinite())
            {
                Point2D aLocalPnt2d = Locus.GeomElt(Node2);
                EndOnEdge = VertexFromNode(Node2, myOffset, aLocalPnt2d, MapNodeVertex, ref VE);
            }

            if (myJoinType == Intersection)
            {
                StartOnEdge = EndOnEdge = false;
            }

            //---------------------------------------------
            // Construction of geometries.
            //---------------------------------------------
            BRepFill_TrimEdgeTool Trim = new(Bisec,
                                             Locus.GeomElt(CurrentArc.FirstElement()),
                                             Locus.GeomElt(CurrentArc.SecondElement()),
                                             myOffset);

            //-----------------------------------------------------------
            // Construction of vertices on edges parallel to the spine.
            //-----------------------------------------------------------

            Trim.IntersectWith(E[0], E[1], S[0], S[1], Ends[0], Ends[1], myJoinType, myIsOpenResult, Params);

            for (int s = 1; s <= Params.Length(); s++)
            {
                Vertex VC = new();
                myBuilder.MakeVertex(ref VC);
                Point2D P2 = Bisec.Value().Value(Params.Value(s).X());
                Point3D PVC = new(P2.X(), P2.Y(), 0.0);

                myBuilder.UpdateVertex(VC, PVC, global::Kerf.Engine.Foundation.Precision.Confusion());
                Vertices.Append(VC);
            }
            if (StartOnEdge)
            {
                bool Start = true;
                Trim.AddOrConfuse(Start, E[0], E[1], Params);
                if (Params.Length() == Vertices.Length() && Params.Length() != 0)
                {
                    Vertices.SetValue(1, VS);
                }
                else
                {
                    // the point was not found by IntersectWith
                    Vertices.Prepend(VS);
                }
            }
            if (EndOnEdge)
            {
                bool Start = false;
                Trim.AddOrConfuse(Start, E[0], E[1], Params);
                if (Params.Length() == Vertices.Length() && Params.Length() != 0)
                {
                    Vertices.SetValue(Params.Length(), VE);
                }
                else
                {
                    // the point was not found by IntersectWith
                    Vertices.Append(VE);
                }
            }

            //------------------------------------------------------------
            // Update Detromp.
            // Detromp allows to remove vertices on the offset
            // corresponding to tangency zones
            // Detromp ranks the vertices that limit
            // the parts of the bissectrices located between the spine and the
            // offset.
            //------------------------------------------------------------
            if (!Detromp.IsBound(S[0]))
            {
                Detromp.Bind(S[0], new NCollection_List<Shape>());
            }
            if (!Detromp.IsBound(S[1]))
            {
                Detromp.Bind(S[1], new NCollection_List<Shape>());
            }

            UpdateDetromp(Detromp, S[0], S[1], Vertices, Params, Bisec, StartOnEdge, EndOnEdge, Trim);
            //----------------------------------------------
            // Storage of vertices on parallel edges.
            // fill MapBis and MapVerPar.
            //----------------------------------------------
            if (!Vertices.IsEmpty() && Params.Length() == Vertices.Length())
            {
                for (k = 0; k <= 1; k++)
                {
                    if (!MapBis.IsBound(E[k]))
                    {
                        MapBis.Bind(E[k], new NCollection_Sequence<Shape>());
                        MapVerPar.Bind(E[k], new NCollection_Sequence<double>());
                    }
                    for (int ii = 1; ii <= Vertices.Length(); ii++)
                    {
                        MapBis[E[k]].Append(Vertices.Value(ii));
                        if (k == 0)
                        {
                            MapVerPar[E[k]].Append(Params.Value(ii).Y());
                        }
                        else
                        {
                            MapVerPar[E[k]].Append(Params.Value(ii).Z());
                        }
                    }
                }
            }
            else
            {
                //------------------------------------------------------------
                // FOR COMPLETE CIRCLES. the parallel line can be contained
                // in the zone without intersection with the border
                // no intersection
                // if myoffset is < distance of nodes the parallel can be valid.
                //-------------------------------------------------------------
                for (k = 0; k <= 1; k++)
                {
                    if (!MapBis.IsBound(E[k]))
                    {
                        if (Node1.Distance() > myOffset && Node2.Distance() > myOffset)
                        {
                            MapBis.Bind(E[k], new NCollection_Sequence<Shape>());
                            MapVerPar.Bind(E[k], new NCollection_Sequence<double>());
                        }
                    }
                }
            }
        }

        //----------------------------------
        // Construction of parallel edges.
        //----------------------------------
        NCollection_IndexedDataMap<Shape, Shape> MapVV = new(ShapeHasher.Instance);

        Shape CurrentSpine;

        for (j = 1; j <= myMap.Extent(); j++)
        {
            CurrentSpine = myMap.FindKey(j);
            CurrentEdge = ShapeCasts.Edge(myMap[j].First());

            myMap[j].Clear();
            if (MapBis.IsBound(CurrentEdge))
            {
                NCollection_Sequence<Shape> aSeqOfShape = new();
                if (!MapBis[CurrentEdge].IsEmpty())
                {
                    int IndOfE = 0;
                    if (myIsOpenResult)
                    {
                        if (j == 1)
                        {
                            IndOfE = 1;
                        }
                        else if (j == myMap.Extent())
                        {
                            IndOfE = -1;
                        }
                    }
                    TrimEdge(CurrentEdge,
                             CurrentSpine,
                             mySpine,
                             Detromp[CurrentSpine],
                             MapBis[CurrentEdge],
                             MapVerPar[CurrentEdge],
                             aSeqOfShape,
                             MapVV,
                             IndOfE);
                    for (k = 1; k <= aSeqOfShape.Length(); k++)
                    {
                        myMap[j].Append(aSeqOfShape.Value(k));
                    }
                }
                else
                {
                    //-----------------
                    // Complete circles
                    //-----------------
                    myMap[j].Append(CurrentEdge);
                }
            }
        }

        int ind;
        for (ind = 1; ind <= MapVV.Extent(); ind++)
        {
            Vertex OldV = ShapeCasts.Vertex(MapVV.FindKey(ind));
            Vertex NewV = ShapeCasts.Vertex(MapVV[ind]);
            Point3D P1 = Tool.Pnt(OldV);
            Point3D P2 = Tool.Pnt(NewV);
            myBuilder.UpdateVertex(NewV, P1.Distance(P2));
            NCollection_List<Shape> LV = new();
            LV.Append(NewV.Oriented(Forward));
            Substitution aSubst = new();
            aSubst.Substitute(OldV, LV);
            for (j = 1; j <= myMap.Extent(); j++)
            {
                NCollection_List<Shape>.Iterator itl = new(myMap[j]);
                for (; itl.More(); itl.Next())
                {
                    aSubst.Build(itl.Value());
                    if (aSubst.IsCopied(itl.Value()))
                    {
                        NCollection_List<Shape> listSh = aSubst.InternalCopy(itl.Value());
                        Orientation SaveOr = itl.Value().Orientation();
                        itl.ChangeValue() = listSh.First();
                        itl.ChangeValue().Orientation(SaveOr);
                    }
                }
            }
        }

        //----------------------------------
        // Construction of offset wires.
        //----------------------------------
        MakeWires();

        // Update vertices ( Constructed in the plane Z = 0) !!!
        NCollection_Map<Shape> MapVertex = new(ShapeHasher.Instance);
        for (Explorer exp = new(myShape, ShapeType.Vertex); exp.More(); exp.Next())
        {
            Vertex V = ShapeCasts.Vertex(exp.Current());
            if (MapVertex.Add(V))
            {
                Point3D P = Tool.Pnt(V);
                P = RefPlane.Value(P.X(), P.Y());
                myBuilder.UpdateVertex(V, P, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        // Construction of curves 3d.
        BRepLib.BuildCurves3d(myShape);
        MapVertex.Clear();
        Explorer Explo = new(myShape, ShapeType.Edge);
        for (; Explo.More(); Explo.Next())
        {
            Edge anEdge = ShapeCasts.Edge(Explo.Current());
            Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
            BrepTVertex TV1 = (BrepTVertex)V1.TShape()!;
            BrepTVertex TV2 = (BrepTVertex)V2.TShape()!;

            Curve3D theCurve = Tool.Curve(anEdge, out Placement loc, out double f, out double l)!;
            theCurve = (Curve3D)theCurve.Copy();
            theCurve.Transform(loc.Transformation());
            Point3D f3d = theCurve.Value(f);
            Point3D l3d = theCurve.Value(l);

            double dist1, dist2;
            dist1 = f3d.Distance(TV1.Pnt());
            dist2 = l3d.Distance(TV2.Pnt());
            if (!MapVertex.Contains(V1))
            {
                TV1.Pnt(f3d);
                MapVertex.Add(V1);
            }
            else
            {
                TV1.UpdateTolerance(1.5 * dist1);
            }
            if (!MapVertex.Contains(V2))
            {
                TV2.Pnt(l3d);
                MapVertex.Add(V2);
            }
            else
            {
                TV2.UpdateTolerance(1.5 * dist2);
            }
        }

        if (!myIsOpenResult)
        {
            FixHoles();
        }

        myIsDone = true;
    }

    // OCCT's private Generated() (the map of generated shapes) is never called: not ported.
    // OCCT's private Add(const BRepFill_OffsetWire&) is declared but never defined: not ported.

    // The work face: the spine's edges, cut at the extrema of curvature and inflexions (MAT2d_CutCurve).
    private void PrepareSpine()
    {
        BrepBuilder B = new();
        NCollection_List<Shape> Cuts = new();
        NCollection_List<Shape>.Iterator IteCuts = new();
        Vertex V1, V2;

        myWorkSpine.Nullify();
        myMapSpine.Clear();

        Surface? S = Tool.Surface(mySpine, out Placement L);
        double TolF = Tool.Tolerance(mySpine);
        Face aWorkSpine = new();
        B.MakeFace(ref aWorkSpine, S, L, TolF);
        myWorkSpine = aWorkSpine;

        for (Iterator IteF = new(mySpine); IteF.More(); IteF.Next())
        {
            Wire NW = new();
            B.MakeWire(ref NW);

            int ForcedCut = 0;
            int nbResEdges = -1;
            NCollection_IndexedMap<Shape> EdgeMap = new(ShapeHasher.Instance);

            Exploration.MapShapes(IteF.Value(), ShapeType.Edge, EdgeMap);
            int nbEdges = EdgeMap.Extent();

            if (nbEdges == 1 && !myIsOpenResult)
            { // in case of open wire there's no need to do it
                ForcedCut = 2;
            }

            for (Iterator IteW = new(IteF.Value()); IteW.More(); IteW.Next())
            {
                Edge E = ShapeCasts.Edge(IteW.Value());
                EdgeVertices(E, out V1, out V2);
                myMapSpine.Bind(V1, V1);
                myMapSpine.Bind(V2, V2);
                Cuts.Clear();

                // Cut
                Shape aLocalShape = E.Oriented(Forward);
                BrepTEdge TEdge = (BrepTEdge)E.TShape()!;
                int aNumCurvesInEdge = TEdge.Curves().Extent();
                if (nbEdges == 2 && nbResEdges == 0 && aNumCurvesInEdge > 1)
                {
                    ForcedCut = 1;
                }
                nbResEdges = CutEdge(ShapeCasts.Edge(aLocalShape), mySpine, ForcedCut, Cuts);

                if (Cuts.IsEmpty())
                {
                    B.Add(NW, E);
                    myMapSpine.Bind(E, E);
                }
                else
                {
                    for (IteCuts.Initialize(Cuts); IteCuts.More(); IteCuts.Next())
                    {
                        Edge NE = ShapeCasts.Edge(IteCuts.Value());
                        NE.Orientation(E.Orientation());
                        B.Add(NW, NE);
                        myMapSpine.Bind(NE, E);
                        EdgeVertices(NE, out V1, out V2);
                        if (!myMapSpine.IsBound(V1))
                        {
                            myMapSpine.Bind(V1, E);
                        }
                        if (!myMapSpine.IsBound(V2))
                        {
                            myMapSpine.Bind(V2, E);
                        }
                    }
                }
            }
            Exploration.Vertices(NW, out Vertex aV1, out Vertex aV2);

            NW.Closed(aV1.IsSame(aV2));

            B.Add(myWorkSpine, NW);
        }
    }

    // For each interval on bissectrice defined by parameters test if the medium point is at a distance > offset; in
    // this case vertices corresponding to the extremities of the interval are ranked in the proofing.
    // => If the same vertex appears in the proofing, the border of the zone of proximity is tangent to the offset.
    private void UpdateDetromp(NCollection_DataMap<Shape, NCollection_List<Shape>> Detromp,
                               in Shape Shape1,
                               in Shape Shape2,
                               NCollection_Sequence<Shape> Vertices,
                               NCollection_Sequence<Point3D> Params,
                               Bisector_Bisec Bisec,
                               bool SOnE,
                               bool EOnE,
                               BRepFill_TrimEdgeTool Trim)
    {
        int ii = 1;

        if (myJoinType == Intersection)
        {
            for (; ii <= Vertices.Length(); ii++)
            {
                Vertex aVertex = ShapeCasts.Vertex(Vertices.Value(ii));
                Detromp[Shape1].Append(aVertex);
                Detromp[Shape2].Append(aVertex);
            }
        }
        else // myJoinType == GeomAbs_Arc
        {
            double U1, U2;
            Vertex V1 = new(), V2;

            Curve2D Bis = Bisec.Value();
            bool ForceAdd = false;
            if (Bis is TrimmedCurve2D aTC && aTC.ChangeBasisCurve().IsPeriodic)
            {
                Point2D Pf = Bis.Value(Bis.FirstParameter());
                Point2D Pl = Bis.Value(Bis.LastParameter());
                ForceAdd = Pf.Distance(Pl) <= global::Kerf.Engine.Foundation.Precision.Confusion();
            }

            U1 = Bis.FirstParameter();

            if (SOnE)
            {
                // the first point of the bissectrice is on the offset
                V1 = ShapeCasts.Vertex(Vertices.Value(ii));
                ii++;
            }

            while (ii <= Vertices.Length() && ii <= Params.Length())
            {
                U2 = Params.Value(ii).X();
                V2 = ShapeCasts.Vertex(Vertices.Value(ii));

                Point2D P = Bis.Value((U2 + U1) * 0.5);
                if (!Trim.IsInside(P) || ForceAdd)
                {
                    if (!V1.IsNull)
                    {
                        Detromp[Shape1].Append(V1);
                        Detromp[Shape2].Append(V1);
                    }
                    Detromp[Shape1].Append(V2);
                    Detromp[Shape2].Append(V2);
                }
                U1 = U2;
                V1 = V2;
                ii++;
            }

            // test medium point between the last parameter and the end of the bissectrice.
            U2 = Bis.LastParameter();
            if (!EOnE)
            {
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(U2))
                {
                    Point2D P = Bis.Value((U2 + U1) * 0.5);
                    if (!Trim.IsInside(P) || ForceAdd)
                    {
                        if (!V1.IsNull)
                        {
                            Detromp[Shape1].Append(V1);
                            Detromp[Shape2].Append(V1);
                        }
                    }
                }
                else
                {
                    if (!V1.IsNull)
                    {
                        Detromp[Shape1].Append(V1);
                        Detromp[Shape2].Append(V1);
                    }
                }
            }
        } // end of else (myJoinType==GeomAbs_Arc)
    }

    // Chains the trimmed parallel edges into wires.
    private void MakeWires()
    {
        //--------------------------------------------------------
        // creation of a single list of created parallel edges.
        //--------------------------------------------------------
        NCollection_List<Shape> TheWires = new();
        NCollection_List<Shape>.Iterator itl = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> MVE = new(ShapeHasher.Instance);
        Vertex V1, V2, VF, CV;
        int i;

        for (i = 1; i <= myMap.Extent(); i++)
        {
            for (itl.Initialize(myMap[i]); itl.More(); itl.Next())
            {
                Edge E = ShapeCasts.Edge(itl.Value());
                Exploration.Vertices(E, out V1, out V2);
                if (V1.IsSame(V2) && IsSmallClosedEdge(E, V1))
                {
                    continue; // remove small closed edges
                }
                if (!CheckSmallParamOnEdge(E))
                {
                    continue;
                }
                if (!MVE.Contains(V1))
                {
                    NCollection_List<Shape> empty = new();
                    MVE.Add(V1, empty);
                }
                MVE.ChangeFromKey(V1).Append(E);
                if (!MVE.Contains(V2))
                {
                    NCollection_List<Shape> empty = new();
                    MVE.Add(V2, empty);
                }
                MVE.ChangeFromKey(V2).Append(E);
            }
        }

        //--------------------------------------
        // Creation of parallel wires.
        //--------------------------------------
        BrepBuilder B = new();

        Wire NW = new();
        bool End;
        Edge CE;

        while (!MVE.IsEmpty())
        {
            B.MakeWire(ref NW);

            for (i = 1; i <= MVE.Extent(); i++)
            {
                if (MVE[i].Extent() == 1)
                {
                    break;
                }
            }

            if (i > MVE.Extent())
            {
                i = 1;
            }

            CV = VF = ShapeCasts.Vertex(MVE.FindKey(i));
            CE = ShapeCasts.Edge(MVE[i].First());
            End = false;
            MVE.ChangeFromKey(CV).RemoveFirst();

            if (myIsOpenResult && MVE.FindFromKey(CV).IsEmpty())
            {
                MVE.RemoveKey(CV);
            }

            bool isClosed = false;

            while (!End)
            {
                //-------------------------------
                // Construction of a wire.
                //-------------------------------
                Exploration.Vertices(CE, out V1, out V2);
                if (!CV.IsSame(V1))
                {
                    CV = V1;
                }
                else
                {
                    CV = V2;
                }

                B.Add(NW, CE);

                if (VF.IsSame(CV) || !MVE.Contains(CV))
                {
                    isClosed = VF.IsSame(CV);
                    End = true;
                    MVE.RemoveKey(VF);
                }

                if (!End)
                {
                    for (itl.Initialize(MVE.FindFromKey(CV)); itl.More(); itl.Next())
                    {
                        if (itl.Value().IsSame(CE))
                        {
                            MVE.ChangeFromKey(CV).Remove(itl);
                            break;
                        }
                    }
                    if (!MVE.FindFromKey(CV).IsEmpty())
                    {
                        CE = ShapeCasts.Edge(MVE.FindFromKey(CV).First());
                        MVE.ChangeFromKey(CV).RemoveFirst();
                    }
                    else if (myIsOpenResult)
                    { // CV was a vertex with one edge
                        End = true;
                    }

                    if (MVE.FindFromKey(CV).IsEmpty())
                    {
                        MVE.RemoveKey(CV);
                    }
                }
            }
            NW.Closed(isClosed);
            TheWires.Append(NW);
        }

        // update myShape :
        //      -- if only one wire : myShape is a Wire
        //      -- if several wires : myShape is a Compound.
        if (TheWires.Extent() == 1)
        {
            myShape = TheWires.First();
        }
        else
        {
            Compound R = new();
            B.MakeCompound(ref R);
            NCollection_List<Shape>.Iterator it = new(TheWires);
            for (; it.More(); it.Next())
            {
                B.Add(R, it.Value());
            }
            myShape = R;
        }
    }

    // Closes the parallel wires that MakeWires left open, joining close ends.
    private void FixHoles()
    {
        NCollection_Sequence<Shape> ClosedWires = new(), UnclosedWires = new(), IsolatedWires = new();

        double MaxTol = 0.0;
        BrepBuilder BB = new();

        Explorer Explo = new(mySpine, ShapeType.Vertex);
        for (; Explo.More(); Explo.Next())
        {
            Vertex aVertex = ShapeCasts.Vertex(Explo.Current());
            double Tol = Tool.Tolerance(aVertex);
            if (Tol > MaxTol)
            {
                MaxTol = Tol;
            }
        }
        MaxTol *= 100.0;

        Explo.Init(myShape, ShapeType.Wire);
        for (; Explo.More(); Explo.Next())
        {
            Shape aWire = Explo.Current();
            // Remove duplicated edges
            NCollection_DataMap<Shape, NCollection_List<Shape>> EEmap = new(ShapeHasher.Instance);
            Iterator it = new(aWire);
            for (; it.More(); it.Next())
            {
                Shape anEdge = it.Value();
                if (!EEmap.IsBound(anEdge))
                {
                    NCollection_List<Shape> LE = new();
                    EEmap.Bind(anEdge, LE);
                }
                else
                {
                    EEmap[anEdge].Append(anEdge);
                }
            }
            aWire.Free(true);
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> mapit in EEmap)
            {
                NCollection_List<Shape> LE = mapit.Value;
                NCollection_List<Shape>.Iterator itl = new(LE);
                for (; itl.More(); itl.Next())
                {
                    BB.Remove(aWire, itl.Value());
                }
            }
            // Sorting
            if (aWire.Closed())
            {
                ClosedWires.Append(aWire);
            }
            else
            {
                UnclosedWires.Append(aWire);
            }
        }

        while (!UnclosedWires.IsEmpty())
        {
            Wire Base = ShapeCasts.Wire(UnclosedWires[1]);
            Exploration.Vertices(Base, out Vertex Vf, out Vertex Vl);
            if (Vf.IsNull || Vl.IsNull)
            {
                throw new FailureException("BRepFill_OffsetWire::FixHoles(): Wrong wire.");
            }
            Point3D Pf, Pl;
            Pf = Tool.Pnt(Vf);
            Pl = Tool.Pnt(Vl);
            double DistF = RealLast(), DistL = RealLast();
            int IndexF = 0, IndexL = 0;
            bool IsFirstF = false, IsFirstL = false;
            for (int i = 2; i <= UnclosedWires.Length(); i++)
            {
                Wire aWire = ShapeCasts.Wire(UnclosedWires[i]);
                Exploration.Vertices(aWire, out Vertex V1, out Vertex V2);

                if (V1.IsNull || V2.IsNull)
                {
                    throw new FailureException("BRepFill_OffsetWire::FixHoles(): Wrong wire.");
                }

                Point3D P1, P2;
                P1 = Tool.Pnt(V1);
                P2 = Tool.Pnt(V2);
                double dist = Pf.Distance(P1);
                if (dist < DistF)
                {
                    DistF = dist;
                    IndexF = i;
                    IsFirstF = true;
                }
                dist = Pf.Distance(P2);
                if (dist < DistF)
                {
                    DistF = dist;
                    IndexF = i;
                    IsFirstF = false;
                }
                dist = Pl.Distance(P1);
                if (dist < DistL)
                {
                    DistL = dist;
                    IndexL = i;
                    IsFirstL = true;
                }
                dist = Pl.Distance(P2);
                if (dist < DistL)
                {
                    DistL = dist;
                    IndexL = i;
                    IsFirstL = false;
                }
            }
            if (DistF > MaxTol)
            {
                IndexF = 0;
            }
            if (DistL > MaxTol)
            {
                IndexL = 0;
            }
            Wire theWire;
            Edge theEdge;
            Vertex theVertex;
            double CommonTol;
            bool TryToClose = true;
            if (DistF <= MaxTol && DistL <= MaxTol && IndexF == IndexL && IsFirstF == IsFirstL)
            {
                if (DistF < DistL)
                {
                    DistL = RealLast();
                    IndexL++;
                }
                else
                {
                    DistF = RealLast();
                    IndexF++;
                }
                TryToClose = false;
            }
            if (DistF <= MaxTol)
            {
                theWire = ShapeCasts.Wire(UnclosedWires[IndexF]);
                Exploration.Vertices(theWire, out Vertex V1, out Vertex V2);
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(theWire, ShapeType.Vertex, ShapeType.Edge, VEmap);
                theEdge = IsFirstF ? ShapeCasts.Edge(VEmap.FindFromKey(V1).First())
                                   : ShapeCasts.Edge(VEmap.FindFromKey(V2).First());
                Iterator it = new(theWire);
                for (; it.More(); it.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(it.Value());
                    if (IsFirstF)
                    {
                        anEdge.Reverse();
                    }
                    if (!anEdge.IsSame(theEdge))
                    {
                        BB.Add(Base, anEdge);
                    }
                }
                theVertex = IsFirstF ? V1 : V2;
                CommonTol = Max(Tool.Tolerance(Vf), Tool.Tolerance(theVertex));
                if (DistF <= CommonTol)
                {
                    theEdge.Free(true);
                    Vf.Orientation(theVertex.Orientation());
                    BB.Remove(theEdge, theVertex);
                    BB.Add(theEdge, Vf);
                    BB.UpdateVertex(Vf, CommonTol);
                    if (IsFirstF)
                    {
                        theEdge.Reverse();
                    }
                    BB.Add(Base, theEdge);
                }
                else
                {
                    if (IsFirstF)
                    {
                        theEdge.Reverse();
                    }
                    BB.Add(Base, theEdge);
                    // Creating new edge from theVertex to Vf
                    Edge NewEdge = new BRepLib_MakeEdge(theVertex, Vf).InternalEdge();
                    BB.Add(Base, NewEdge);
                }
            }
            if (DistL <= MaxTol && IndexL != IndexF)
            {
                theWire = ShapeCasts.Wire(UnclosedWires[IndexL]);
                Exploration.Vertices(theWire, out Vertex V1, out Vertex V2);
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(theWire, ShapeType.Vertex, ShapeType.Edge, VEmap);
                theEdge = IsFirstL ? ShapeCasts.Edge(VEmap.FindFromKey(V1).First())
                                   : ShapeCasts.Edge(VEmap.FindFromKey(V2).First());
                Iterator it = new(theWire);
                for (; it.More(); it.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(it.Value());
                    if (!IsFirstL)
                    {
                        anEdge.Reverse();
                    }
                    if (!anEdge.IsSame(theEdge))
                    {
                        BB.Add(Base, anEdge);
                    }
                }
                theVertex = IsFirstL ? V1 : V2;
                CommonTol = Max(Tool.Tolerance(Vl), Tool.Tolerance(theVertex));
                if (DistL <= CommonTol)
                {
                    theEdge.Free(true);
                    Vl.Orientation(theVertex.Orientation());
                    BB.Remove(theEdge, theVertex);
                    BB.Add(theEdge, Vl);
                    BB.UpdateVertex(Vl, CommonTol);
                    if (!IsFirstL)
                    {
                        theEdge.Reverse();
                    }
                    BB.Add(Base, theEdge);
                }
                else
                {
                    if (!IsFirstL)
                    {
                        theEdge.Reverse();
                    }
                    BB.Add(Base, theEdge);
                    // Creating new edge from Vl to theVertex
                    Edge NewEdge = new BRepLib_MakeEdge(Vl, theVertex).InternalEdge();
                    BB.Add(Base, NewEdge);
                }
            }
            // Check if it is possible to close resulting wire
            if (TryToClose)
            {
                Exploration.Vertices(Base, out Vf, out Vl);
                CommonTol = Max(Tool.Tolerance(Vf), Tool.Tolerance(Vl));
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(Base, ShapeType.Vertex, ShapeType.Edge, VEmap);
                Edge Efirst, Elast;
                Efirst = ShapeCasts.Edge(VEmap.FindFromKey(Vf).First());
                Elast = ShapeCasts.Edge(VEmap.FindFromKey(Vl).First());
                Pf = Tool.Pnt(Vf);
                Pl = Tool.Pnt(Vl);
                double Dist = Pf.Distance(Pl);
                if (Dist <= CommonTol)
                {
                    Elast.Free(true);
                    Vf.Orientation(Vl.Orientation());
                    BB.Remove(Elast, Vl);
                    BB.Add(Elast, Vf);
                    BB.UpdateVertex(Vf, CommonTol);
                    Base.Closed(true);
                }
                else if (Dist <= MaxTol)
                {
                    // Creating new edge from Vl to Vf
                    Edge NewEdge = new BRepLib_MakeEdge(Vf, Vl).InternalEdge();
                    BB.Add(Base, NewEdge);
                    Base.Closed(true);
                }
                _ = Efirst;
            }
            // Updating sequences ClosedWires and UnclosedWires
            if (DistF <= MaxTol)
            {
                UnclosedWires.Remove(IndexF);
            }
            if (DistL <= MaxTol && IndexL != IndexF)
            {
                if (DistF <= MaxTol && IndexL > IndexF)
                {
                    IndexL--;
                }
                UnclosedWires.Remove(IndexL);
            }
            if (Base.Closed())
            {
                ClosedWires.Append(Base);
                UnclosedWires.Remove(1);
            }
            else if (DistF > MaxTol && DistL > MaxTol)
            {
                IsolatedWires.Append(Base);
                UnclosedWires.Remove(1);
            }
        }

        // Updating myShape
        if (ClosedWires.Length() + IsolatedWires.Length() == 1)
        {
            if (!ClosedWires.IsEmpty())
            {
                myShape = ClosedWires.First();
            }
            else
            {
                myShape = IsolatedWires.First();
            }
        }
        else
        {
            Compound R = new();
            BB.MakeCompound(ref R);
            int i;
            for (i = 1; i <= ClosedWires.Length(); i++)
            {
                BB.Add(R, ClosedWires[i]);
            }
            for (i = 1; i <= IsolatedWires.Length(); i++)
            {
                BB.Add(R, IsolatedWires[i]);
            }
            myShape = R;
        }
    }

    // Cut edge at the extrema of curvatures and points of inflexion. So, closed circles are cut in two. If Cuts is
    // empty, the edge is not modified. The first and the last vertex of the initial edge belong to the first and the
    // last parts respectively.
    private static int CutEdge(in Edge E, in Face F, int ForceCut, NCollection_List<Shape> Cuts)
    {
        Cuts.Clear();
        MAT2d_CutCurve Cuter = new();
        NCollection_Sequence<Curve2D> theCurves = new();
        double f, l;
        Curve2D C2d;
        TrimmedCurve2D CT2d;
        double aTol = Tool.Tolerance(E);
        Curve3D? aC;

        Vertex VF, VL;
        Exploration.Vertices(E, out Vertex V1, out Vertex V2);
        BrepBuilder B = new();

        C2d = Tool.CurveOnSurface(E, F, out f, out l)!;
        aC = Tool.Curve(E, out f, out l);
        CT2d = new TrimmedCurve2D(C2d, f, l);

        if (CT2d.ChangeBasisCurve() is CircleCurve2D && Math.Abs(f - l) >= Math.PI)
        {
            return 0;
        }

        //-------------------------
        // Cut curve.
        //-------------------------
        Cuter.Perform(CT2d);

        if (ForceCut == 0)
        {
            if (Cuter.UnModified())
            {
                //-----------------------------
                // edge not modified => return.
                //-----------------------------
                return 0;
            }
            else
            {
                for (int k = 1; k <= Cuter.NbCurves(); k++)
                {
                    theCurves.Append(Cuter.Value(k));
                }
            }
        }
        else if (ForceCut == 1)
        {
            if (Cuter.UnModified())
            {
                CutCurve(CT2d, 2, theCurves);
            }
            else
            {
                for (int k = 1; k <= Cuter.NbCurves(); k++)
                {
                    theCurves.Append(Cuter.Value(k));
                }
            }
        }
        else if (ForceCut == 2)
        {
            if (Cuter.UnModified())
            {
                CutCurve(CT2d, 3, theCurves);
            }
            else
            {
                if (Cuter.NbCurves() == 2)
                {
                    TrimmedCurve2D CC = Cuter.Value(1);

                    if (CC.LastParameter() > (l + f) / 2.0)
                    {
                        CutCurve(CC, 2, theCurves);
                        theCurves.Append(Cuter.Value(2));
                    }
                    else
                    {
                        theCurves.Append(CC);
                        CutCurve(Cuter.Value(2), 2, theCurves);
                    }
                }
                else
                {
                    for (int k = 1; k <= Cuter.NbCurves(); k++)
                    {
                        theCurves.Append(Cuter.Value(k));
                    }
                }
            }
        }

        //--------------------------------------
        // Creation of cut edges.
        //--------------------------------------
        VF = V1;

        for (int k = 1; k <= theCurves.Length(); k++)
        {
            TrimmedCurve2D CC = (TrimmedCurve2D)theCurves.Value(k);

            if (k == theCurves.Length())
            {
                VL = V2;
            }
            else
            {
                Point3D P = aC!.Value(CC.LastParameter());

                VL = new BRepLib_MakeVertex(P).InternalVertex();
                B.UpdateVertex(VL, aTol);
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

        return theCurves.Length();
    }

    // Cuts the curve in nbParts parts of equal parameter range (parts too small are merged).
    private static void CutCurve(TrimmedCurve2D C, int nbParts, NCollection_Sequence<Curve2D> theCurves)
    {
        TrimmedCurve2D TrimC;
        double UF, UL, UC;
        double Step;
        Point2D PF, PL, PC;
        double PTol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 10;
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion() * 10;
        bool YaCut = false;

        UF = C.FirstParameter();
        UL = C.LastParameter();
        PF = C.Value(UF);
        PL = C.Value(UL);

        Step = (UL - UF) / nbParts;

        for (int i = 1; i < nbParts; i++)
        {
            UC = UF + i * Step;
            PC = C.Value(UC);

            if (UC - UF > PTol && PC.Distance(PF) > Tol)
            {
                if (UL - UC < PTol || PL.Distance(PC) < Tol)
                {
                    continue;
                }

                TrimC = new TrimmedCurve2D(C, UF, UC);
                theCurves.Append(TrimC);
                UF = UC;
                PF = PC;
                YaCut = true;
            }
        }
        if (YaCut)
        {
            TrimC = new TrimmedCurve2D(C, UF, UL);
            theCurves.Append(TrimC);
        }
        else
        {
            theCurves.Append(C);
        }
    }

    // The arc of the offset around the vertex V, which follows the edge E in the contour.
    private static void MakeCircle(in Edge E,
                                   in Vertex V,
                                   in Face F,
                                   double Offset,
                                   NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Map,
                                   PlaneSurface RefPlane)
    {
        // evaluate the Axis of the Circle.
        Curve2D GC = Tool.CurveOnSurface(E, F, out double f, out double l)!;
        Vector2D DX;
        Point2D P;

        if (E.Orientation() == Forward)
        {
            GC.D1(l, out P, out DX);
            DX.Reverse();
        }
        else
        {
            GC.D1(f, out P, out DX);
        }

        Axis2D Axis = new(P, new Direction2D(DX));
        CircleCurve2D Circ = new(Axis, Math.Abs(Offset), Offset < 0.0);

        // Bind the edges in my Map.
        Edge OE = new BRepLib_MakeEdge(Circ, RefPlane).InternalEdge();
        NCollection_List<Shape> LL = new();

        LL.Append(OE);
        Map.Add(V, LL);
    }

    // The parallel of the edge E, extended at its ends for the trimming.
    private static void MakeOffset(in Edge E,
                                   in Face F,
                                   double Offset,
                                   NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Map,
                                   PlaneSurface RefPlane,
                                   bool IsOpenResult,
                                   global::Kerf.Geometry.JoinKind theJoinType,
                                   Vertex[] Ends)
    {
        double f, l;
        double anOffset = Offset;

        if (E.Orientation() == Forward)
        {
            anOffset *= -1;
        }

        Curve2D G2d = Tool.CurveOnSurface(E, F, out f, out l)!;
        Curve2D? G2dOC = null;

        bool ToExtendFirstPar = true;
        bool ToExtendLastPar = true;
        if (IsOpenResult)
        {
            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            if (V1.IsSame(Ends[0]) || V1.IsSame(Ends[1]))
            {
                ToExtendFirstPar = false;
            }
            if (V2.IsSame(Ends[0]) || V2.IsSame(Ends[1]))
            {
                ToExtendLastPar = false;
            }
        }

        CurveAdaptor2D AC = new(G2d, f, l);
        if (AC.GetType() == Circle)
        {
            // if the offset is greater otr equal to the radius and the side of the
            // concavity of the circle => edge null.
            Circle2D C1 = AC.Circle();
            Frame2D axes = C1.Axis();
            Direction2D Xd = axes.XDirection();
            Direction2D Yd = axes.YDirection();
            double Crossed = Xd.X() * Yd.Y() - Xd.Y() * Yd.X();
            double Signe = (Crossed > 0.0) ? -1.0 : 1.0;

            if (anOffset * Signe < AC.Circle().Radius() - global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                CurveAdaptor2D AHC = new(G2d);
                OffsetCurveAdaptor2D Off = new(AHC, anOffset);
                CircleCurve2D CC = new(Off.Circle());

                double Delta = 2 * Math.PI - l + f;
                if (theJoinType == Arc)
                {
                    if (ToExtendFirstPar)
                    {
                        f -= 0.2 * Delta;
                    }
                    if (ToExtendLastPar)
                    {
                        l += 0.2 * Delta;
                    }
                }
                else // GeomAbs_Intersection
                {
                    if (ToExtendFirstPar && ToExtendLastPar)
                    {
                        double old_l = l;
                        f = old_l + Delta / 2.0;
                        l = f + 2 * Math.PI;
                    }
                    else if (ToExtendFirstPar)
                    {
                        f = l;
                        l = f + 2 * Math.PI;
                    }
                    else if (ToExtendLastPar)
                    {
                        l = f + 2 * Math.PI;
                    }
                }
                G2dOC = new TrimmedCurve2D(CC, f, l);
            }
        }
        else if (AC.GetType() == Line)
        {
            CurveAdaptor2D AHC = new(G2d);
            OffsetCurveAdaptor2D Off = new(AHC, anOffset);
            LineCurve2D CC = new(Off.Line());
            double Delta = l - f;
            if (ToExtendFirstPar)
            {
                if (theJoinType == Arc)
                {
                    f -= Delta;
                }
                else
                { // GeomAbs_Intersection
                    f = -global::Kerf.Engine.Foundation.Precision.Infinite();
                }
            }
            if (ToExtendLastPar)
            {
                if (theJoinType == Arc)
                {
                    l += Delta;
                }
                else
                { // GeomAbs_Intersection
                    l = global::Kerf.Engine.Foundation.Precision.Infinite();
                }
            }
            G2dOC = new TrimmedCurve2D(CC, f, l);
        }
        else
        {
            TrimmedCurve2D G2dT = new(G2d, f, l);
            G2dOC = new OffsetCurve2D(G2dT, anOffset);
        }

        // Bind the edges in my Map.
        if (G2dOC is not null)
        {
            Edge OE = new BRepLib_MakeEdge(G2dOC, RefPlane).InternalEdge();
            OE.Orientation(E.Orientation());
            NCollection_List<Shape> LL = new();
            LL.Append(OE);
            Map.Add(E, LL);
        }
    }

    // The vertex of a node of the locus that lies on the offset (one per node).
    private static bool VertexFromNode(MAT_Node aNode,
                                       double Offset,
                                       in Point2D PN,
                                       NCollection_DataMap<MAT_Node, Shape> MapNodeVertex,
                                       ref Vertex VN)
    {
        bool Status;
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        BrepBuilder B = new();

        if (!aNode.Infinite() && Math.Abs(aNode.Distance() - Offset) < Tol)
        {
            //------------------------------------------------
            // the Node gives a vertex on the offset
            //------------------------------------------------
            if (MapNodeVertex.IsBound(aNode))
            {
                VN = ShapeCasts.Vertex(MapNodeVertex[aNode]);
            }
            else
            {
                Point3D P = new(PN.X(), PN.Y(), 0.0);
                B.MakeVertex(ref VN);
                B.UpdateVertex(VN, P, global::Kerf.Engine.Foundation.Precision.Confusion());
                MapNodeVertex.Bind(aNode, VN);
            }
            Status = true;
        }
        else
        {
            Status = false;
        }

        return Status;
    }

    private static void StoreInMap(in Shape V1, in Shape V2, NCollection_IndexedDataMap<Shape, Shape> MapVV)
    {
        Shape NewV = V2;
        int i;

        if (MapVV.Contains(V2))
        {
            NewV = MapVV.FindFromKey(V2);
        }

        if (MapVV.Contains(V1))
        {
            MapVV.ChangeFromKey(V1) = NewV;
        }

        for (i = 1; i <= MapVV.Extent(); i++)
        {
            if (MapVV[i].IsSame(V1))
            {
                MapVV[i] = NewV;
            }
        }

        MapVV.Add(V1, NewV);
    }

    // The parts of the parallel E between its vertices on the bisectors that are kept (the proofing Detromp).
    private static void TrimEdge(in Edge E,
                                 in Shape ProE,
                                 in Face AllSpine,
                                 NCollection_List<Shape> Detromp,
                                 NCollection_Sequence<Shape> TheVer,
                                 NCollection_Sequence<double> ThePar,
                                 NCollection_Sequence<Shape> S,
                                 NCollection_IndexedDataMap<Shape, Shape> MapVV,
                                 int IndOfE)
    {
        bool Change = true;
        BrepBuilder TheBuilder = new();
        S.Clear();

        //-----------------------------------------------------------
        // Parse two sequences depending on the parameter on the edge.
        //-----------------------------------------------------------
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
        // If a vertex is not in the proofing, it is eliminated.
        //----------------------------------------------------------
        if (!Tool.Degenerated(E))
        {
            for (int k = 1; k <= TheVer.Length(); k++)
            {
                if (DoubleOrNotInside(Detromp, ShapeCasts.Vertex(TheVer.Value(k))))
                {
                    TheVer.Remove(k);
                    ThePar.Remove(k);
                    k--;
                }
            }
        }

        //----------------------------------------------------------
        // If a vertex_double appears twice in the proofing
        // the vertex is removed.
        // otherwise preserve only one of its representations.
        //----------------------------------------------------------
        if (!Tool.Degenerated(E))
        {
            double aParTol = 2.0 * global::Kerf.Engine.Foundation.Precision.PConfusion();
            for (int k = 1; k < TheVer.Length(); k++)
            {
                if (TheVer.Value(k).IsSame(TheVer.Value(k + 1))
                    || Math.Abs(ThePar.Value(k) - ThePar.Value(k + 1)) <= aParTol)
                {
                    if (k + 1 == TheVer.Length())
                    {
                        StoreInMap(TheVer[k], TheVer[k + 1], MapVV);
                        TheVer.Remove(k);
                        ThePar.Remove(k);
                    }
                    else
                    {
                        StoreInMap(TheVer[k + 1], TheVer[k], MapVV);
                        TheVer.Remove(k + 1);
                        ThePar.Remove(k + 1);
                    }
                    k--;
                }
            }
        }
        //-----------------------------------------------------------
        // Creation of edges.
        // the number of vertices should be even. The created edges
        // go from a vertex with uneven index i to vertex i+1;
        //-----------------------------------------------------------
        if (IndOfE == 1 || IndOfE == -1) // open result and extreme edges of result
        {
            Shape aLocalShape = E.EmptyCopied();
            Edge NewEdge = ShapeCasts.Edge(aLocalShape);
            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            Tool.CurveOnSurface(E, out Curve2D? PCurve, out Surface? _, out Placement _, out double fpar, out double lpar);

            bool ToTrimAsOrigin = IsInnerEdge(ProE, AllSpine, out double TrPar1, out double TrPar2);

            if (IndOfE == 1) // first edge of open wire
            {
                if (NewEdge.Orientation() == Forward)
                {
                    if (ToTrimAsOrigin)
                    {
                        fpar = TrPar1;
                        Point2D TrPnt2d = PCurve!.Value(fpar);
                        Point3D TrPnt = new(TrPnt2d.X(), TrPnt2d.Y(), 0.0);
                        V1 = new BRepLib_MakeVertex(TrPnt).InternalVertex();
                    }
                    TheBuilder.Add(NewEdge, V1.Oriented(Forward));
                    TheBuilder.Add(NewEdge, TheVer.First().Oriented(Reversed));
                    TheBuilder.Range(NewEdge, fpar, ThePar.First());
                }
                else
                {
                    if (ToTrimAsOrigin)
                    {
                        lpar = TrPar2;
                        Point2D TrPnt2d = PCurve!.Value(lpar);
                        Point3D TrPnt = new(TrPnt2d.X(), TrPnt2d.Y(), 0.0);
                        V2 = new BRepLib_MakeVertex(TrPnt).InternalVertex();
                    }
                    TheBuilder.Add(NewEdge, TheVer.First().Oriented(Reversed));
                    TheBuilder.Add(NewEdge, V2.Oriented(Forward));
                    TheBuilder.Range(NewEdge, ThePar.First(), lpar);
                }
            }
            else // last edge of open wire
            {
                if (NewEdge.Orientation() == Forward)
                {
                    if (ToTrimAsOrigin)
                    {
                        lpar = TrPar2;
                        Point2D TrPnt2d = PCurve!.Value(lpar);
                        Point3D TrPnt = new(TrPnt2d.X(), TrPnt2d.Y(), 0.0);
                        V2 = new BRepLib_MakeVertex(TrPnt).InternalVertex();
                    }
                    TheBuilder.Add(NewEdge, TheVer.First().Oriented(Forward));
                    TheBuilder.Add(NewEdge, V2.Oriented(Reversed));
                    TheBuilder.Range(NewEdge, ThePar.First(), lpar);
                }
                else
                {
                    if (ToTrimAsOrigin)
                    {
                        fpar = TrPar1;
                        Point2D TrPnt2d = PCurve!.Value(fpar);
                        Point3D TrPnt = new(TrPnt2d.X(), TrPnt2d.Y(), 0.0);
                        V1 = new BRepLib_MakeVertex(TrPnt).InternalVertex();
                    }
                    TheBuilder.Add(NewEdge, V1.Oriented(Reversed));
                    TheBuilder.Add(NewEdge, TheVer.First().Oriented(Forward));
                    TheBuilder.Range(NewEdge, fpar, ThePar.First());
                }
            }
            S.Append(NewEdge);
        }
        else
        {
            for (int k = 1; k < TheVer.Length(); k = k + 2)
            {
                Shape aLocalShape = E.EmptyCopied();
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

                S.Append(NewEdge);
            }
        }
    }

    private static bool IsInnerEdge(in Shape ProE, in Face AllSpine, out double TrPar1, out double TrPar2)
    {
        TrPar1 = TrPar2 = 0.0;
        if (ProE.ShapeType() != ShapeType.Edge)
        {
            return false;
        }

        Edge anEdge = ShapeCasts.Edge(ProE);

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> VEmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(AllSpine, ShapeType.Vertex, ShapeType.Edge, VEmap);
        for (int i = 1; i <= VEmap.Extent(); i++)
        {
            NCollection_List<Shape> LE = VEmap[i];
            if (LE.Extent() == 1 && anEdge.IsSame(LE.First()))
            {
                return false;
            }
        }

        Tool.Range(anEdge, out TrPar1, out TrPar2);
        return true;
    }

    // True if V appears twice in LV or is not inside.
    private static bool DoubleOrNotInside(NCollection_List<Shape> LV, in Vertex V)
    {
        bool Vu = false;
        NCollection_List<Shape>.Iterator it = new(LV);

        for (; it.More(); it.Next())
        {
            if (V.IsSame(it.Value()))
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

    private static bool IsSmallClosedEdge(in Edge anEdge, in Vertex aVertex)
    {
        Point3D PV = Tool.Pnt(aVertex);
        Point2D PV2d = new(), Pfirst, Plast, Pmid;
        PV2d.SetCoord(PV.X(), PV.Y());

        Curve2D PCurve;
        CurveRepresentation CurveRep = ((BrepTEdge)anEdge.TShape()!).Curves().First();
        PCurve = CurveRep.PCurve()!;

        double fpar = ((GCurve)CurveRep).First();
        double lpar = ((GCurve)CurveRep).Last();
        Pfirst = PCurve.Value(fpar);
        Plast = PCurve.Value(lpar);
        Pmid = PCurve.Value((fpar + lpar) * 0.5);

        double theTol = Tool.Tolerance(aVertex);
        theTol *= 1.5;

        double dist1 = Pfirst.Distance(PV2d);
        double dist2 = Plast.Distance(PV2d);
        double dist3 = Pmid.Distance(PV2d);

        return dist1 <= theTol && dist2 <= theTol && dist3 <= theTol;
    }

    // The edges, other than lines and circles, whose curvature at an end is at least 1/Offset on the side of the offset.
    private static void CheckBadEdges(in Face Spine,
                                      double Offset,
                                      BRepMAT2d_BisectingLocus Locus,
                                      BRepMAT2d_LinkTopoBilo Link,
                                      NCollection_List<Shape> BadEdges)
    {
        Face F = ShapeCasts.Face(Spine.Oriented(Forward));
        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double LimCurv = 1.0 / Offset;

        NCollection_Map<Shape> aMap = new(ShapeHasher.Instance);

        for (int ic = 1; ic <= Locus.NumberOfContours(); ic++)
        {
            for (int ie = 1; ie <= Locus.NumberOfElts(ic); ie++)
            {
                Shape SE = Link.GeneratingShape(Locus.BasicElt(ic, ie));
                if (SE.ShapeType() == ShapeType.Edge)
                {
                    if (aMap.Contains(SE))
                    {
                        continue;
                    }

                    Edge E = ShapeCasts.Edge(SE);

                    Curve2D G2d = Tool.CurveOnSurface(E, F, out double f, out double l)!;

                    CurveAdaptor2D AC = new(G2d, f, l);
                    CurveKind aCType = AC.GetType();

                    if (aCType != Line && aCType != Circle)
                    {
                        bool reverse = false;
                        if (E.Orientation() == Forward)
                        {
                            reverse = true;
                        }

                        Point2D P, Pc;
                        Direction2D N;

                        CLProps2d aCLProps = new(G2d, 2, eps);

                        aCLProps.SetParameter(f);
                        if (!aCLProps.IsTangentDefined())
                        {
                            BadEdges.Append(SE);
                            aMap.Add(SE);
                            continue;
                        }

                        P = aCLProps.Value();
                        double Crv = aCLProps.Curvature();

                        if (Crv >= eps)
                        {
                            aCLProps.Tangent(out N);
                            double x = N.Y(), y = -N.X();
                            N.SetCoord(x, y);
                            if (reverse)
                            {
                                N.Reverse();
                            }
                            aCLProps.CentreOfCurvature(out Pc);
                            Vector2D Dir = new(P, Pc);
                            if (N.Dot(new Direction2D(Dir)) > 0.0) // C++ converts the vector to a direction implicitly
                            {
                                if (LimCurv <= Crv + eps)
                                {
                                    BadEdges.Append(SE);
                                    aMap.Add(SE);
                                    continue;
                                }
                            }
                        }

                        aCLProps.SetParameter(l);
                        if (!aCLProps.IsTangentDefined())
                        {
                            BadEdges.Append(SE);
                            aMap.Add(SE);
                            continue;
                        }

                        P = aCLProps.Value();
                        Crv = aCLProps.Curvature();

                        if (Crv >= eps)
                        {
                            aCLProps.Tangent(out N);
                            double x = N.Y(), y = -N.X();
                            N.SetCoord(x, y);
                            if (reverse)
                            {
                                N.Reverse();
                            }
                            aCLProps.CentreOfCurvature(out Pc);
                            Vector2D Dir = new(P, Pc);
                            if (N.Dot(new Direction2D(Dir)) > 0.0) // C++ converts the vector to a direction implicitly
                            {
                                if (LimCurv <= Crv + eps)
                                {
                                    BadEdges.Append(SE);
                                    aMap.Add(SE);
                                    continue;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> Parameters,
                                     NCollection_Sequence<Point3D> Points,
                                     CurveAdaptorBase C,
                                     double Deflection,
                                     double U1,
                                     double U2,
                                     double EPSILON,
                                     int Nbmin)
    {
        double UU1 = Min(U1, U2);
        double UU2 = Max(U1, U2);

        C.D1(UU1, out Point3D Pdeb, out Vector3D Ddeb);
        Parameters.Append(UU1);
        Points.Append(Pdeb);

        C.D1(UU2, out Point3D Pfin, out Vector3D Dfin);

        double aDelta = UU2 - UU1;
        double aDist = Pdeb.Distance(Pfin);

        if ((aDelta / aDist) > 5.0e-14)
        {
            QuasiFleche(C, Deflection * Deflection, UU1, Pdeb, Ddeb, UU2, Pfin, Dfin, Nbmin, EPSILON * EPSILON, Parameters, Points);
        }

        return true;
    }

    private static void QuasiFleche(CurveAdaptorBase C,
                                    double Deflection2,
                                    double Udeb,
                                    in Point3D Pdeb,
                                    in Vector3D Vdeb,
                                    double Ufin,
                                    in Point3D Pfin,
                                    in Vector3D Vfin,
                                    int Nbmin,
                                    double Eps,
                                    NCollection_Sequence<double> Parameters,
                                    NCollection_Sequence<Point3D> Points)
    {
        int Ptslength = Points.Length();
        double Udelta = Ufin - Udeb;
        Point3D Pdelta;
        Vector3D Vdelta;
        if (Nbmin > 2)
        {
            Udelta /= (Nbmin - 1);
            C.D1(Udeb + Udelta, out Pdelta, out Vdelta);
        }
        else
        {
            Pdelta = Pfin;
            Vdelta = Vfin;
        }

        double Norme = new Vector3D(Pdeb, Pdelta).SquareMagnitude();
        double theFleche = 0;
        bool flecheok = false;
        if (Norme > Eps)
        {
            // Evaluation of the arrow by interpolation. See IntWalk_IWalking::TestDeflection
            double N1 = Vdeb.SquareMagnitude();
            double N2 = Vdelta.SquareMagnitude();
            if (N1 > Eps && N2 > Eps)
            {
                double Normediff = (Vdeb.Normalized().XYZ() - Vdelta.Normalized().XYZ()).SquareModulus();
                if (Normediff > Eps)
                {
                    theFleche = Normediff * Norme / 64.0;
                    flecheok = true;
                }
            }
        }
        if (!flecheok)
        {
            Point3D Pmid = new((Pdeb.XYZ() + Pdelta.XYZ()) / 2.0);
            Point3D Pverif = C.Value(Udeb + Udelta / 2.0);
            theFleche = Pmid.SquareDistance(Pverif);
        }

        if (theFleche < Deflection2)
        {
            Parameters.Append(Udeb + Udelta);
            Points.Append(Pdelta);
        }
        else
        {
            QuasiFleche(C, Deflection2, Udeb, Pdeb, Vdeb, Udeb + Udelta, Pdelta, Vdelta, 3, Eps, Parameters, Points);
        }

        if (Nbmin > 2)
        {
            QuasiFleche(C, Deflection2, Udeb + Udelta, Pdelta, Vdelta, Ufin, Pfin, Vfin,
                        Nbmin - (Points.Length() - Ptslength), Eps, Parameters, Points);
        }
    }

    private static bool CheckSmallParamOnEdge(in Edge anEdge)
    {
        NCollection_List<CurveRepresentation> aList = ((BrepTEdge)anEdge.TShape()!).Curves();
        if (!aList.IsEmpty())
        {
            CurveRepresentation CRep = ((BrepTEdge)anEdge.TShape()!).Curves().First();
            double f = ((GCurve)CRep).First();
            double l = ((GCurve)CRep).Last();
            if (Math.Abs(l - f) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }
        }
        return true;
    }
}
