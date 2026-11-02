// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetWire.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_JoinType;
using static Occt.MAT_Side;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

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
    private TopoDS_Face mySpine = new();
    private TopoDS_Face myWorkSpine = new();
    private double myOffset;
    private bool myIsOpenResult;
    private TopoDS_Shape myShape = new();
    private bool myIsDone;
    private GeomAbs_JoinType myJoinType;
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(EqualityComparer<TopoDS_Shape>.Default);
    private readonly BRepMAT2d_BisectingLocus myBilo = new();
    private readonly BRepMAT2d_LinkTopoBilo myLink = new();
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMapSpine = new(TopTools_ShapeMapHasher.Instance);
    // OCCT leaves myCallGen uninitialised until Perform; PerformWithBiLo alone reads it in GeneratedShapes, where an
    // empty myMapSpine makes its value irrelevant (occt-issues.md, 5O2)
    private bool myCallGen;

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

    // A face bounded by one closed circle (or any single edge for an open result): its parallel is computed directly.
    private static bool KPartCircle(in TopoDS_Face mySpine,
                                    double myOffset,
                                    bool IsOpenResult,
                                    double Alt,
                                    ref TopoDS_Shape myShape,
                                    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap,
                                    ref bool myIsDone)
    {
        TopoDS_Edge E = new();
        for (TopExp_Explorer anEdgeIter = new(mySpine, TopAbs_EDGE); anEdgeIter.More(); anEdgeIter.Next())
        {
            if (!E.IsNull())
            {
                return false;
            }
            E = TopoDS.Edge(anEdgeIter.Current());
        }
        if (E.IsNull())
        {
            return false;
        }

        Geom_Curve? C = BRep_Tool.Curve(E, out TopLoc_Location L, out double f, out double l);
        if (C is null)
        {
            return false;
        }

        if (C is Geom_TrimmedCurve Ct)
        {
            C = Ct.BasisCurve();
        }

        if ((C is Geom_Circle && BRep_Tool.IsClosed(E)) || // closed circle
            IsOpenResult)
        {
            double anOffset = myOffset;

            Geom2d_Curve aPCurve = BRep_Tool.CurveOnSurface(E, mySpine, out f, out l)!;
            Geom2dAdaptor_Curve AHC = new(aPCurve, f, l);
            Geom2d_Curve OC;
            if (AHC.GetType() == GeomAbs_Line)
            {
                if (E.Orientation() == TopAbs_FORWARD)
                {
                    anOffset *= -1;
                }
                Adaptor2d_OffsetCurve Off = new(AHC, anOffset);
                OC = new Geom2d_Line(Off.Line());
            }
            else if (AHC.GetType() == GeomAbs_Circle)
            {
                if (E.Orientation() == TopAbs_FORWARD)
                {
                    anOffset *= -1;
                }
                if (!BRep_Tool.IsClosed(E))
                {
                    anOffset *= -1;
                }
                gp_Circ2d theCirc = AHC.Circle();
                if (anOffset > 0.0 || Math.Abs(anOffset) < theCirc.Radius())
                {
                    OC = new Geom2d_Circle(theCirc.Position(), theCirc.Radius() + anOffset);
                }
                else
                {
                    myIsDone = false;
                    return false;
                }
            }
            else
            {
                if (E.Orientation() == TopAbs_FORWARD)
                {
                    anOffset *= -1;
                }
                Geom2d_TrimmedCurve G2dT = new(aPCurve, f, l);
                OC = new Geom2d_OffsetCurve(G2dT, anOffset);
            }
            Geom_Surface aSurf = BRep_Tool.Surface(mySpine)!;
            Geom_Plane? aPlane = aSurf as Geom_Plane;
            myShape = new BRepLib_MakeEdge(OC, aPlane!, f, l).InternalEdge();
            BRepLib.BuildCurve3d(TopoDS.Edge(myShape));

            myShape.Orientation(E.Orientation());
            myShape.Location(L);
            if (Math.Abs(Alt) > gp.Resolution())
            {
                BRepAdaptor_Surface S = new(mySpine, false);
                gp_Ax1 Nor = S.Plane().Axis();
                gp_Trsf T = new();
                gp_Vec Trans = new(Nor.Direction());
                Trans = Alt * Trans;
                T.SetTranslation(Trans);
                myShape.Move(new TopLoc_Location(T));
            }

            NCollection_List<TopoDS_Shape> LL = new();
            LL.Append(myShape);
            myMap.Add(E, LL);

            TopoDS_Edge myEdge = TopoDS.Edge(myShape);
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
    public BRepFill_OffsetWire(in TopoDS_Face Spine, GeomAbs_JoinType Join = GeomAbs_Arc, bool IsOpenResult = false)
    {
        Init(Spine, Join, IsOpenResult);
    }

    /// <summary>
    /// Prepares the offset of the wires of <paramref name="Spine"/>: cuts its edges for the bisectors and computes its
    /// medial axis on the left.
    /// </summary>
    public void Init(in TopoDS_Face Spine, GeomAbs_JoinType Join = GeomAbs_Arc, bool IsOpenResult = false)
    {
        myIsDone = false;
        TopoDS_Shape aLocalShape = Spine.Oriented(TopAbs_FORWARD);
        mySpine = TopoDS.Face(aLocalShape);
        myJoinType = Join;
        myIsOpenResult = IsOpenResult;

        myMap.Clear();
        myMapSpine.Clear();

        //------------------------------------------------------------------
        // cut the spine for bissectors.
        //------------------------------------------------------------------
        BRepMAT2d_Explorer Exp = new();
        Exp.Perform(mySpine);
        mySpine = TopoDS.Face(Exp.ModifiedShape(mySpine));
        PrepareSpine();

        TopoDS_Shape aShape = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMap = new(EqualityComparer<TopoDS_Shape>.Default);
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
    public TopoDS_Face Spine()
    {
        mySpine.TShape()?.FreezeGraph();
        return mySpine;
    }

    /// <summary><see cref="Spine"/> without freezing, for ported callers.</summary>
    internal TopoDS_Face InternalSpine() => mySpine;

    /// <summary>The parallel: a wire, or a compound of wires (frozen).</summary>
    public TopoDS_Shape Shape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal TopoDS_Shape InternalShape() => myShape;

    /// <summary>
    /// The shapes of the parallel generated by <paramref name="SpineShape"/>, an edge or a vertex of the spine (frozen,
    /// with the whole result).
    /// </summary>
    public ISchematiqList<TopoDS_Shape> GeneratedShapes(in TopoDS_Shape SpineShape)
    {
        NCollection_List<TopoDS_Shape> aList = InternalGeneratedShapes(SpineShape);
        myShape.TShape()?.FreezeGraph();
        if (aList.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        var aShapes = new TopoDS_Shape[aList.Extent()];
        int i = 0;
        foreach (TopoDS_Shape aS in aList)
        {
            aS.TShape()?.FreezeGraph();
            aShapes[i++] = aS;
        }
        return PackedList.Own(aShapes);
    }

    /// <summary>
    /// <see cref="GeneratedShapes"/> without freezing: the algorithm's own list (an empty new list if there is none).
    /// </summary>
    internal NCollection_List<TopoDS_Shape> InternalGeneratedShapes(in TopoDS_Shape SpineShape)
    {
        if (!myCallGen)
        {
            if (!myMapSpine.IsEmpty())
            {
                // myMapSpine can be empty if passed by PerformWithBilo.
                // OCCT iterates the map in address order; the port in insertion order (occt-issues.md, 5O2)
                foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> it in myMapSpine)
                {
                    if (myMap.Contains(it.Key))
                    {
                        if (!myMap.Contains(it.Value))
                        {
                            NCollection_List<TopoDS_Shape> L = new();
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
                            NCollection_List<TopoDS_Shape> L = new();
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
            return new NCollection_List<TopoDS_Shape>();
        }
    }

    /// <summary>The join type.</summary>
    public GeomAbs_JoinType JoinType() => myJoinType;

    /// <summary>
    /// Computes the parallel at the distance <paramref name="Offset"/>, at the altitude <paramref name="Alt"/> along
    /// the normal of the spine's plane.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">A closed result is expected and a parallel wire isn't closed.</exception>
    public void Perform(double Offset, double Alt = 0.0)
    {
        try
        {
            myCallGen = false;
            if (KPartCircle(myWorkSpine, Offset, myIsOpenResult, Alt, ref myShape, myMap, ref myIsDone))
            {
                return;
            }

            TopoDS_Face oldWorkSpain = myWorkSpine;

            NCollection_List<TopoDS_Shape> BadEdges = new();
            CheckBadEdges(myWorkSpine, Offset, myBilo, myLink, BadEdges);

            if (!BadEdges.IsEmpty())
            {
                // Modification of myWorkSpine;
                BRepTools_Substitution aSubst = new();
                NCollection_List<TopoDS_Shape>.Iterator it = new(BadEdges);
                NCollection_List<TopoDS_Shape> aL = new();
                double aDefl = .01 * Math.Abs(Offset);
                NCollection_Sequence<double> Parameters = new();
                NCollection_Sequence<gp_Pnt> Points = new();

                for (; it.More(); it.Next())
                {
                    aL.Clear();
                    Parameters.Clear();
                    Points.Clear();
                    TopoDS_Shape anE = it.Value();

                    TopExp.Vertices(TopoDS.Edge(anE), out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);

                    Geom_Curve? G3d = BRep_Tool.Curve(TopoDS.Edge(anE), out double f, out double l);
                    GeomAdaptor_Curve AC = new(G3d!, f, l);

                    PerformCurve(Parameters, Points, AC, aDefl, f, l, Precision.Confusion(), 2);

                    int NPnts = Points.Length();
                    if (NPnts > 2)
                    {
                        TopoDS_Vertex FV = Vf;
                        TopoDS_Vertex LV;
                        TopoDS_Edge newE;
                        int np;
                        for (np = 2; np < NPnts; np++)
                        {
                            gp_Pnt LP = Points[np];
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
                        TopoDS_Edge newE = new BRepLib_MakeEdge(Vf, Vl).InternalEdge();
                        aL.Append(newE);
                    }
                    // Update myMapSpine
                    if (myMapSpine.IsBound(anE))
                    {
                        NCollection_List<TopoDS_Shape>.Iterator newit = new(aL);
                        for (; newit.More(); newit.Next())
                        {
                            TopoDS_Edge NewEdge = TopoDS.Edge(newit.Value());
                            myMapSpine.Bind(NewEdge, myMapSpine[anE]);
                            EdgeVertices(NewEdge, out TopoDS_Vertex NewV1, out TopoDS_Vertex NewV2);
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

                NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> wwmap = new(TopTools_ShapeMapHasher.Instance);
                TopoDS_Iterator itws = new(myWorkSpine);
                for (; itws.More(); itws.Next())
                {
                    TopoDS_Shape aWire = itws.Value();
                    aSubst.Build(aWire);
                    if (aSubst.IsCopied(aWire))
                    {
                        TopoDS_Wire NewWire = TopoDS.Wire(aSubst.InternalCopy(aWire).First());
                        NewWire.Closed(aWire.Closed());
                        NCollection_List<TopoDS_Shape> Lw = new();
                        Lw.Append(NewWire);
                        wwmap.Bind(aWire, Lw);
                    }
                }
                aSubst.Clear();
                foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itmap in wwmap)
                {
                    aSubst.Substitute(itmap.Key, itmap.Value);
                }

                aSubst.Build(myWorkSpine);

                if (aSubst.IsCopied(myWorkSpine))
                {
                    myWorkSpine = TopoDS.Face(aSubst.InternalCopy(myWorkSpine).First());

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
        catch (Exception anException) when (anException is Standard_Failure or NullReferenceException
                                            && anException is not (Standard_ImmutableObject or TopoDS_LockedShape))
        {
            // OCC_CATCH_SIGNALS turns OCCT's access violations (a null bisector, occt-issues.md) into failures caught
            // here: the port's NullReferenceException (decision 917)
            myShape.Nullify();
            myIsDone = false;

            return;
        }

        if (!myIsOpenResult)
        {
            TopExp_Explorer anExp = new(myShape, TopAbs_WIRE);

            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Shape aWire = anExp.Current();

                if (!aWire.Closed())
                {
                    myShape.Nullify();
                    myIsDone = false;
                    throw new Standard_ConstructionError("Offset wire is not closed.");
                }
            }
        }
    }

    // The parallel at distance 0: the wires of the spine, moved by the altitude.
    private static void Compute(in TopoDS_Face Spine,
                                ref TopoDS_Shape aShape,
                                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map,
                                double Alt)
    {
        BRep_Builder B = new();
        TopoDS_Compound aCompound = new();
        B.MakeCompound(ref aCompound);
        aShape = aCompound;
        double ALT = Alt;
        if (Spine.Orientation() == TopAbs_REVERSED)
        {
            ALT = -Alt;
        }
        gp_Trsf T = new();
        T.SetTranslation(new gp_Vec(0.0, 0.0, ALT));
        TopLoc_Location L = new(T);

        for (TopExp_Explorer exp = new(Spine, TopAbs_WIRE); exp.More(); exp.Next())
        {
            TopoDS_Wire CurW = TopoDS.Wire(exp.Current());
            TopoDS_Shape aLocalShape = CurW.Moved(L);
            TopoDS_Wire NewW = TopoDS.Wire(aLocalShape);
            B.Add(aShape, NewW);
            // update Map.
            TopoDS_Iterator it1 = new(CurW);
            TopoDS_Iterator it2 = new(NewW);
            for (; it1.More(); it1.Next(), it2.Next())
            {
                NCollection_List<TopoDS_Shape> List = new();
                List.Append(it2.Value());
                Map.Add(it1.Value(), List);
            }
        }
    }

    /// <summary>
    /// Computes the parallel at the distance <paramref name="Offset"/> of the face <paramref name="Spine"/> from its
    /// bisecting locus <paramref name="Locus"/> and links <paramref name="Link"/>, already computed.
    /// </summary>
    public void PerformWithBiLo(in TopoDS_Face Spine,
                                double Offset,
                                BRepMAT2d_BisectingLocus Locus,
                                BRepMAT2d_LinkTopoBilo Link,
                                GeomAbs_JoinType Join = GeomAbs_Arc,
                                double Alt = 0.0)
    {
        myIsDone = false;
        TopoDS_Shape aLocalShapeWork = Spine.Oriented(TopAbs_FORWARD);
        myWorkSpine = TopoDS.Face(aLocalShapeWork);
        myJoinType = Join;
        myOffset = Offset;
        myShape.Nullify();

        if (mySpine.IsNull())
        {
            TopoDS_Shape aLocalShape = Spine.Oriented(TopAbs_FORWARD);
            mySpine = TopoDS.Face(aLocalShape);
        }
        myMap.Clear();

        if (Math.Abs(myOffset) < Precision.Confusion())
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

        BRep_Builder myBuilder = new();

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
        NCollection_DataMap<MAT_Node, TopoDS_Shape> MapNodeVertex = new(ReferenceEqualityComparer.Instance);
        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<TopoDS_Shape>> MapBis = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, NCollection_Sequence<double>> MapVerPar = new(TopTools_ShapeMapHasher.Instance);

        // OCCT.NET: OCCT down-casts (a null plane on another surface, dereferenced below: an access violation natively)
        Geom_Plane RefPlane = (BRep_Tool.Surface(myWorkSpine) as Geom_Plane)!;
        if (Math.Abs(Alt) > gp.Resolution())
        {
            double anAlt = Alt;
            if (myWorkSpine.Orientation() == TopAbs_REVERSED)
            {
                anAlt = -Alt;
            }
            RefPlane = (Geom_Plane)RefPlane.Translated(anAlt * new gp_Vec(RefPlane.Axis().Direction()));
        }

        //---------------------------------------------------------------
        // Construction of Circles and OffsetCurves
        //---------------------------------------------------------------

        TopoDS_Vertex[] Ends = [new(), new()];
        if (myIsOpenResult)
        {
            TopoDS_Wire theWire;
            TopoDS_Iterator iter = new(mySpine);
            theWire = TopoDS.Wire(iter.Value());
            TopExp.Vertices(theWire, out Ends[0], out Ends[1]);
        }

        if (Locus.NumberOfContours() == 0)
        {
            return;
        }

        for (int ic = 1; ic <= Locus.NumberOfContours(); ic++)
        {
            TopoDS_Shape PE = Link.GeneratingShape(Locus.BasicElt(ic, Locus.NumberOfElts(ic)));
            for (int ie = 1; ie <= Locus.NumberOfElts(ic); ie++)
            {
                TopoDS_Shape SE = Link.GeneratingShape(Locus.BasicElt(ic, ie));
                if (SE.ShapeType() == TopAbs_VERTEX)
                {
                    if (!SE.IsSame(Ends[0]) && !SE.IsSame(Ends[1]))
                    {
                        MakeCircle(TopoDS.Edge(PE), TopoDS.Vertex(SE), myWorkSpine, myOffset, myMap, RefPlane);
                    }
                }
                else
                {
                    MakeOffset(TopoDS.Edge(SE),
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
        if (myIsOpenResult && myJoinType == GeomAbs_Arc)
        {
            if (!myMap.IsEmpty() && myMap.FindKey(1).ShapeType() == TopAbs_VERTEX)
            {
                myMap.RemoveFromIndex(1);
            }
            if (!myMap.IsEmpty() && myMap.FindKey(myMap.Extent()).ShapeType() == TopAbs_VERTEX)
            {
                myMap.RemoveLast();
            }
        }

        //---------------------------------------------------
        // Construction of offset vertices.
        //---------------------------------------------------
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Detromp = new(EqualityComparer<TopoDS_Shape>.Default);
        MAT_Arc CurrentArc;
        TopoDS_Edge CurrentEdge;
        TopoDS_Shape[] S = [new(), new()];
        TopoDS_Edge[] E = [new(), new()];
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

            NCollection_Sequence<TopoDS_Shape> Vertices = new();
            NCollection_Sequence<gp_Pnt> Params = new();

            //-----------------------------------------------------------
            // Return parallel edges on each face.
            // If no offset generated => move to the next bissectrice.
            //--------------------------------------------------------------
            if (myMap.Contains(S[0]) && myMap.Contains(S[1]))
            {
                E[0] = TopoDS.Edge(myMap.FindFromKey(S[0]).First());
                E[1] = TopoDS.Edge(myMap.FindFromKey(S[1]).First());
            }
            else
            {
                continue;
            }

            //-----------------------------------------------------------
            // Construction of vertices corresponding to the node of the map.
            // if they are on the offset.
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

            bool StartOnEdge = false, EndOnEdge = false;

            if (!Node1.Infinite())
            {
                gp_Pnt2d aLocalPnt2d = Locus.GeomElt(Node1);
                StartOnEdge = VertexFromNode(Node1, myOffset, aLocalPnt2d, MapNodeVertex, ref VS);
            }
            if (!Node2.Infinite())
            {
                gp_Pnt2d aLocalPnt2d = Locus.GeomElt(Node2);
                EndOnEdge = VertexFromNode(Node2, myOffset, aLocalPnt2d, MapNodeVertex, ref VE);
            }

            if (myJoinType == GeomAbs_Intersection)
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
                TopoDS_Vertex VC = new();
                myBuilder.MakeVertex(ref VC);
                gp_Pnt2d P2 = Bisec.Value().Value(Params.Value(s).X());
                gp_Pnt PVC = new(P2.X(), P2.Y(), 0.0);

                myBuilder.UpdateVertex(VC, PVC, Precision.Confusion());
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
                Detromp.Bind(S[0], new NCollection_List<TopoDS_Shape>());
            }
            if (!Detromp.IsBound(S[1]))
            {
                Detromp.Bind(S[1], new NCollection_List<TopoDS_Shape>());
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
                        MapBis.Bind(E[k], new NCollection_Sequence<TopoDS_Shape>());
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
                            MapBis.Bind(E[k], new NCollection_Sequence<TopoDS_Shape>());
                            MapVerPar.Bind(E[k], new NCollection_Sequence<double>());
                        }
                    }
                }
            }
        }

        //----------------------------------
        // Construction of parallel edges.
        //----------------------------------
        NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> MapVV = new(TopTools_ShapeMapHasher.Instance);

        TopoDS_Shape CurrentSpine;

        for (j = 1; j <= myMap.Extent(); j++)
        {
            CurrentSpine = myMap.FindKey(j);
            CurrentEdge = TopoDS.Edge(myMap[j].First());

            myMap[j].Clear();
            if (MapBis.IsBound(CurrentEdge))
            {
                NCollection_Sequence<TopoDS_Shape> aSeqOfShape = new();
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
            TopoDS_Vertex OldV = TopoDS.Vertex(MapVV.FindKey(ind));
            TopoDS_Vertex NewV = TopoDS.Vertex(MapVV[ind]);
            gp_Pnt P1 = BRep_Tool.Pnt(OldV);
            gp_Pnt P2 = BRep_Tool.Pnt(NewV);
            myBuilder.UpdateVertex(NewV, P1.Distance(P2));
            NCollection_List<TopoDS_Shape> LV = new();
            LV.Append(NewV.Oriented(TopAbs_FORWARD));
            BRepTools_Substitution aSubst = new();
            aSubst.Substitute(OldV, LV);
            for (j = 1; j <= myMap.Extent(); j++)
            {
                NCollection_List<TopoDS_Shape>.Iterator itl = new(myMap[j]);
                for (; itl.More(); itl.Next())
                {
                    aSubst.Build(itl.Value());
                    if (aSubst.IsCopied(itl.Value()))
                    {
                        NCollection_List<TopoDS_Shape> listSh = aSubst.InternalCopy(itl.Value());
                        TopAbs_Orientation SaveOr = itl.Value().Orientation();
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
        NCollection_Map<TopoDS_Shape> MapVertex = new(TopTools_ShapeMapHasher.Instance);
        for (TopExp_Explorer exp = new(myShape, TopAbs_VERTEX); exp.More(); exp.Next())
        {
            TopoDS_Vertex V = TopoDS.Vertex(exp.Current());
            if (MapVertex.Add(V))
            {
                gp_Pnt P = BRep_Tool.Pnt(V);
                P = RefPlane.Value(P.X(), P.Y());
                myBuilder.UpdateVertex(V, P, Precision.Confusion());
            }
        }

        // Construction of curves 3d.
        BRepLib.BuildCurves3d(myShape);
        MapVertex.Clear();
        TopExp_Explorer Explo = new(myShape, TopAbs_EDGE);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(Explo.Current());
            TopExp.Vertices(anEdge, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            BRep_TVertex TV1 = (BRep_TVertex)V1.TShape()!;
            BRep_TVertex TV2 = (BRep_TVertex)V2.TShape()!;

            Geom_Curve theCurve = BRep_Tool.Curve(anEdge, out TopLoc_Location loc, out double f, out double l)!;
            theCurve = (Geom_Curve)theCurve.Copy();
            theCurve.Transform(loc.Transformation());
            gp_Pnt f3d = theCurve.Value(f);
            gp_Pnt l3d = theCurve.Value(l);

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
        BRep_Builder B = new();
        NCollection_List<TopoDS_Shape> Cuts = new();
        NCollection_List<TopoDS_Shape>.Iterator IteCuts = new();
        TopoDS_Vertex V1, V2;

        myWorkSpine.Nullify();
        myMapSpine.Clear();

        Geom_Surface? S = BRep_Tool.Surface(mySpine, out TopLoc_Location L);
        double TolF = BRep_Tool.Tolerance(mySpine);
        TopoDS_Face aWorkSpine = new();
        B.MakeFace(ref aWorkSpine, S, L, TolF);
        myWorkSpine = aWorkSpine;

        for (TopoDS_Iterator IteF = new(mySpine); IteF.More(); IteF.Next())
        {
            TopoDS_Wire NW = new();
            B.MakeWire(ref NW);

            int ForcedCut = 0;
            int nbResEdges = -1;
            NCollection_IndexedMap<TopoDS_Shape> EdgeMap = new(TopTools_ShapeMapHasher.Instance);

            TopExp.MapShapes(IteF.Value(), TopAbs_EDGE, EdgeMap);
            int nbEdges = EdgeMap.Extent();

            if (nbEdges == 1 && !myIsOpenResult)
            { // in case of open wire there's no need to do it
                ForcedCut = 2;
            }

            for (TopoDS_Iterator IteW = new(IteF.Value()); IteW.More(); IteW.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(IteW.Value());
                EdgeVertices(E, out V1, out V2);
                myMapSpine.Bind(V1, V1);
                myMapSpine.Bind(V2, V2);
                Cuts.Clear();

                // Cut
                TopoDS_Shape aLocalShape = E.Oriented(TopAbs_FORWARD);
                BRep_TEdge TEdge = (BRep_TEdge)E.TShape()!;
                int aNumCurvesInEdge = TEdge.Curves().Extent();
                if (nbEdges == 2 && nbResEdges == 0 && aNumCurvesInEdge > 1)
                {
                    ForcedCut = 1;
                }
                nbResEdges = CutEdge(TopoDS.Edge(aLocalShape), mySpine, ForcedCut, Cuts);

                if (Cuts.IsEmpty())
                {
                    B.Add(NW, E);
                    myMapSpine.Bind(E, E);
                }
                else
                {
                    for (IteCuts.Initialize(Cuts); IteCuts.More(); IteCuts.Next())
                    {
                        TopoDS_Edge NE = TopoDS.Edge(IteCuts.Value());
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
            TopExp.Vertices(NW, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);

            NW.Closed(aV1.IsSame(aV2));

            B.Add(myWorkSpine, NW);
        }
    }

    // For each interval on bissectrice defined by parameters test if the medium point is at a distance > offset; in
    // this case vertices corresponding to the extremities of the interval are ranked in the proofing.
    // => If the same vertex appears in the proofing, the border of the zone of proximity is tangent to the offset.
    private void UpdateDetromp(NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Detromp,
                               in TopoDS_Shape Shape1,
                               in TopoDS_Shape Shape2,
                               NCollection_Sequence<TopoDS_Shape> Vertices,
                               NCollection_Sequence<gp_Pnt> Params,
                               Bisector_Bisec Bisec,
                               bool SOnE,
                               bool EOnE,
                               BRepFill_TrimEdgeTool Trim)
    {
        int ii = 1;

        if (myJoinType == GeomAbs_Intersection)
        {
            for (; ii <= Vertices.Length(); ii++)
            {
                TopoDS_Vertex aVertex = TopoDS.Vertex(Vertices.Value(ii));
                Detromp[Shape1].Append(aVertex);
                Detromp[Shape2].Append(aVertex);
            }
        }
        else // myJoinType == GeomAbs_Arc
        {
            double U1, U2;
            TopoDS_Vertex V1 = new(), V2;

            Geom2d_Curve Bis = Bisec.Value();
            bool ForceAdd = false;
            if (Bis is Geom2d_TrimmedCurve aTC && aTC.ChangeBasisCurve().IsPeriodic())
            {
                gp_Pnt2d Pf = Bis.Value(Bis.FirstParameter());
                gp_Pnt2d Pl = Bis.Value(Bis.LastParameter());
                ForceAdd = Pf.Distance(Pl) <= Precision.Confusion();
            }

            U1 = Bis.FirstParameter();

            if (SOnE)
            {
                // the first point of the bissectrice is on the offset
                V1 = TopoDS.Vertex(Vertices.Value(ii));
                ii++;
            }

            while (ii <= Vertices.Length() && ii <= Params.Length())
            {
                U2 = Params.Value(ii).X();
                V2 = TopoDS.Vertex(Vertices.Value(ii));

                gp_Pnt2d P = Bis.Value((U2 + U1) * 0.5);
                if (!Trim.IsInside(P) || ForceAdd)
                {
                    if (!V1.IsNull())
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
                if (!Precision.IsInfinite(U2))
                {
                    gp_Pnt2d P = Bis.Value((U2 + U1) * 0.5);
                    if (!Trim.IsInside(P) || ForceAdd)
                    {
                        if (!V1.IsNull())
                        {
                            Detromp[Shape1].Append(V1);
                            Detromp[Shape2].Append(V1);
                        }
                    }
                }
                else
                {
                    if (!V1.IsNull())
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
        NCollection_List<TopoDS_Shape> TheWires = new();
        NCollection_List<TopoDS_Shape>.Iterator itl = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> MVE = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Vertex V1, V2, VF, CV;
        int i;

        for (i = 1; i <= myMap.Extent(); i++)
        {
            for (itl.Initialize(myMap[i]); itl.More(); itl.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(itl.Value());
                TopExp.Vertices(E, out V1, out V2);
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
                    NCollection_List<TopoDS_Shape> empty = new();
                    MVE.Add(V1, empty);
                }
                MVE.ChangeFromKey(V1).Append(E);
                if (!MVE.Contains(V2))
                {
                    NCollection_List<TopoDS_Shape> empty = new();
                    MVE.Add(V2, empty);
                }
                MVE.ChangeFromKey(V2).Append(E);
            }
        }

        //--------------------------------------
        // Creation of parallel wires.
        //--------------------------------------
        BRep_Builder B = new();

        TopoDS_Wire NW = new();
        bool End;
        TopoDS_Edge CE;

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

            CV = VF = TopoDS.Vertex(MVE.FindKey(i));
            CE = TopoDS.Edge(MVE[i].First());
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
                TopExp.Vertices(CE, out V1, out V2);
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
                        CE = TopoDS.Edge(MVE.FindFromKey(CV).First());
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
            TopoDS_Compound R = new();
            B.MakeCompound(ref R);
            NCollection_List<TopoDS_Shape>.Iterator it = new(TheWires);
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
        NCollection_Sequence<TopoDS_Shape> ClosedWires = new(), UnclosedWires = new(), IsolatedWires = new();

        double MaxTol = 0.0;
        BRep_Builder BB = new();

        TopExp_Explorer Explo = new(mySpine, TopAbs_VERTEX);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Vertex aVertex = TopoDS.Vertex(Explo.Current());
            double Tol = BRep_Tool.Tolerance(aVertex);
            if (Tol > MaxTol)
            {
                MaxTol = Tol;
            }
        }
        MaxTol *= 100.0;

        Explo.Init(myShape, TopAbs_WIRE);
        for (; Explo.More(); Explo.Next())
        {
            TopoDS_Shape aWire = Explo.Current();
            // Remove duplicated edges
            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> EEmap = new(TopTools_ShapeMapHasher.Instance);
            TopoDS_Iterator it = new(aWire);
            for (; it.More(); it.Next())
            {
                TopoDS_Shape anEdge = it.Value();
                if (!EEmap.IsBound(anEdge))
                {
                    NCollection_List<TopoDS_Shape> LE = new();
                    EEmap.Bind(anEdge, LE);
                }
                else
                {
                    EEmap[anEdge].Append(anEdge);
                }
            }
            aWire.Free(true);
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapit in EEmap)
            {
                NCollection_List<TopoDS_Shape> LE = mapit.Value;
                NCollection_List<TopoDS_Shape>.Iterator itl = new(LE);
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
            TopoDS_Wire Base = TopoDS.Wire(UnclosedWires[1]);
            TopExp.Vertices(Base, out TopoDS_Vertex Vf, out TopoDS_Vertex Vl);
            if (Vf.IsNull() || Vl.IsNull())
            {
                throw new Standard_Failure("BRepFill_OffsetWire::FixHoles(): Wrong wire.");
            }
            gp_Pnt Pf, Pl;
            Pf = BRep_Tool.Pnt(Vf);
            Pl = BRep_Tool.Pnt(Vl);
            double DistF = RealLast(), DistL = RealLast();
            int IndexF = 0, IndexL = 0;
            bool IsFirstF = false, IsFirstL = false;
            for (int i = 2; i <= UnclosedWires.Length(); i++)
            {
                TopoDS_Wire aWire = TopoDS.Wire(UnclosedWires[i]);
                TopExp.Vertices(aWire, out TopoDS_Vertex V1, out TopoDS_Vertex V2);

                if (V1.IsNull() || V2.IsNull())
                {
                    throw new Standard_Failure("BRepFill_OffsetWire::FixHoles(): Wrong wire.");
                }

                gp_Pnt P1, P2;
                P1 = BRep_Tool.Pnt(V1);
                P2 = BRep_Tool.Pnt(V2);
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
            TopoDS_Wire theWire;
            TopoDS_Edge theEdge;
            TopoDS_Vertex theVertex;
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
                theWire = TopoDS.Wire(UnclosedWires[IndexF]);
                TopExp.Vertices(theWire, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VEmap = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapesAndAncestors(theWire, TopAbs_VERTEX, TopAbs_EDGE, VEmap);
                theEdge = IsFirstF ? TopoDS.Edge(VEmap.FindFromKey(V1).First())
                                   : TopoDS.Edge(VEmap.FindFromKey(V2).First());
                TopoDS_Iterator it = new(theWire);
                for (; it.More(); it.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(it.Value());
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
                CommonTol = Max(BRep_Tool.Tolerance(Vf), BRep_Tool.Tolerance(theVertex));
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
                    TopoDS_Edge NewEdge = new BRepLib_MakeEdge(theVertex, Vf).InternalEdge();
                    BB.Add(Base, NewEdge);
                }
            }
            if (DistL <= MaxTol && IndexL != IndexF)
            {
                theWire = TopoDS.Wire(UnclosedWires[IndexL]);
                TopExp.Vertices(theWire, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VEmap = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapesAndAncestors(theWire, TopAbs_VERTEX, TopAbs_EDGE, VEmap);
                theEdge = IsFirstL ? TopoDS.Edge(VEmap.FindFromKey(V1).First())
                                   : TopoDS.Edge(VEmap.FindFromKey(V2).First());
                TopoDS_Iterator it = new(theWire);
                for (; it.More(); it.Next())
                {
                    TopoDS_Edge anEdge = TopoDS.Edge(it.Value());
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
                CommonTol = Max(BRep_Tool.Tolerance(Vl), BRep_Tool.Tolerance(theVertex));
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
                    TopoDS_Edge NewEdge = new BRepLib_MakeEdge(Vl, theVertex).InternalEdge();
                    BB.Add(Base, NewEdge);
                }
            }
            // Check if it is possible to close resulting wire
            if (TryToClose)
            {
                TopExp.Vertices(Base, out Vf, out Vl);
                CommonTol = Max(BRep_Tool.Tolerance(Vf), BRep_Tool.Tolerance(Vl));
                NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VEmap = new(TopTools_ShapeMapHasher.Instance);
                TopExp.MapShapesAndAncestors(Base, TopAbs_VERTEX, TopAbs_EDGE, VEmap);
                TopoDS_Edge Efirst, Elast;
                Efirst = TopoDS.Edge(VEmap.FindFromKey(Vf).First());
                Elast = TopoDS.Edge(VEmap.FindFromKey(Vl).First());
                Pf = BRep_Tool.Pnt(Vf);
                Pl = BRep_Tool.Pnt(Vl);
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
                    TopoDS_Edge NewEdge = new BRepLib_MakeEdge(Vf, Vl).InternalEdge();
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
            TopoDS_Compound R = new();
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
    private static int CutEdge(in TopoDS_Edge E, in TopoDS_Face F, int ForceCut, NCollection_List<TopoDS_Shape> Cuts)
    {
        Cuts.Clear();
        MAT2d_CutCurve Cuter = new();
        NCollection_Sequence<Geom2d_Curve> theCurves = new();
        double f, l;
        Geom2d_Curve C2d;
        Geom2d_TrimmedCurve CT2d;
        double aTol = BRep_Tool.Tolerance(E);
        Geom_Curve? aC;

        TopoDS_Vertex VF, VL;
        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
        BRep_Builder B = new();

        C2d = BRep_Tool.CurveOnSurface(E, F, out f, out l)!;
        aC = BRep_Tool.Curve(E, out f, out l);
        CT2d = new Geom2d_TrimmedCurve(C2d, f, l);

        if (CT2d.ChangeBasisCurve() is Geom2d_Circle && Math.Abs(f - l) >= Math.PI)
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
                    Geom2d_TrimmedCurve CC = Cuter.Value(1);

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
            Geom2d_TrimmedCurve CC = (Geom2d_TrimmedCurve)theCurves.Value(k);

            if (k == theCurves.Length())
            {
                VL = V2;
            }
            else
            {
                gp_Pnt P = aC!.Value(CC.LastParameter());

                VL = new BRepLib_MakeVertex(P).InternalVertex();
                B.UpdateVertex(VL, aTol);
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

        return theCurves.Length();
    }

    // Cuts the curve in nbParts parts of equal parameter range (parts too small are merged).
    private static void CutCurve(Geom2d_TrimmedCurve C, int nbParts, NCollection_Sequence<Geom2d_Curve> theCurves)
    {
        Geom2d_TrimmedCurve TrimC;
        double UF, UL, UC;
        double Step;
        gp_Pnt2d PF, PL, PC;
        double PTol = Precision.PConfusion() * 10;
        double Tol = Precision.Confusion() * 10;
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

                TrimC = new Geom2d_TrimmedCurve(C, UF, UC);
                theCurves.Append(TrimC);
                UF = UC;
                PF = PC;
                YaCut = true;
            }
        }
        if (YaCut)
        {
            TrimC = new Geom2d_TrimmedCurve(C, UF, UL);
            theCurves.Append(TrimC);
        }
        else
        {
            theCurves.Append(C);
        }
    }

    // The arc of the offset around the vertex V, which follows the edge E in the contour.
    private static void MakeCircle(in TopoDS_Edge E,
                                   in TopoDS_Vertex V,
                                   in TopoDS_Face F,
                                   double Offset,
                                   NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map,
                                   Geom_Plane RefPlane)
    {
        // evaluate the Axis of the Circle.
        Geom2d_Curve GC = BRep_Tool.CurveOnSurface(E, F, out double f, out double l)!;
        gp_Vec2d DX;
        gp_Pnt2d P;

        if (E.Orientation() == TopAbs_FORWARD)
        {
            GC.D1(l, out P, out DX);
            DX.Reverse();
        }
        else
        {
            GC.D1(f, out P, out DX);
        }

        gp_Ax2d Axis = new(P, new gp_Dir2d(DX));
        Geom2d_Circle Circ = new(Axis, Math.Abs(Offset), Offset < 0.0);

        // Bind the edges in my Map.
        TopoDS_Edge OE = new BRepLib_MakeEdge(Circ, RefPlane).InternalEdge();
        NCollection_List<TopoDS_Shape> LL = new();

        LL.Append(OE);
        Map.Add(V, LL);
    }

    // The parallel of the edge E, extended at its ends for the trimming.
    private static void MakeOffset(in TopoDS_Edge E,
                                   in TopoDS_Face F,
                                   double Offset,
                                   NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map,
                                   Geom_Plane RefPlane,
                                   bool IsOpenResult,
                                   GeomAbs_JoinType theJoinType,
                                   TopoDS_Vertex[] Ends)
    {
        double f, l;
        double anOffset = Offset;

        if (E.Orientation() == TopAbs_FORWARD)
        {
            anOffset *= -1;
        }

        Geom2d_Curve G2d = BRep_Tool.CurveOnSurface(E, F, out f, out l)!;
        Geom2d_Curve? G2dOC = null;

        bool ToExtendFirstPar = true;
        bool ToExtendLastPar = true;
        if (IsOpenResult)
        {
            TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            if (V1.IsSame(Ends[0]) || V1.IsSame(Ends[1]))
            {
                ToExtendFirstPar = false;
            }
            if (V2.IsSame(Ends[0]) || V2.IsSame(Ends[1]))
            {
                ToExtendLastPar = false;
            }
        }

        Geom2dAdaptor_Curve AC = new(G2d, f, l);
        if (AC.GetType() == GeomAbs_Circle)
        {
            // if the offset is greater otr equal to the radius and the side of the
            // concavity of the circle => edge null.
            gp_Circ2d C1 = AC.Circle();
            gp_Ax22d axes = C1.Axis();
            gp_Dir2d Xd = axes.XDirection();
            gp_Dir2d Yd = axes.YDirection();
            double Crossed = Xd.X() * Yd.Y() - Xd.Y() * Yd.X();
            double Signe = (Crossed > 0.0) ? -1.0 : 1.0;

            if (anOffset * Signe < AC.Circle().Radius() - Precision.Confusion())
            {
                Geom2dAdaptor_Curve AHC = new(G2d);
                Adaptor2d_OffsetCurve Off = new(AHC, anOffset);
                Geom2d_Circle CC = new(Off.Circle());

                double Delta = 2 * Math.PI - l + f;
                if (theJoinType == GeomAbs_Arc)
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
                G2dOC = new Geom2d_TrimmedCurve(CC, f, l);
            }
        }
        else if (AC.GetType() == GeomAbs_Line)
        {
            Geom2dAdaptor_Curve AHC = new(G2d);
            Adaptor2d_OffsetCurve Off = new(AHC, anOffset);
            Geom2d_Line CC = new(Off.Line());
            double Delta = l - f;
            if (ToExtendFirstPar)
            {
                if (theJoinType == GeomAbs_Arc)
                {
                    f -= Delta;
                }
                else
                { // GeomAbs_Intersection
                    f = -Precision.Infinite();
                }
            }
            if (ToExtendLastPar)
            {
                if (theJoinType == GeomAbs_Arc)
                {
                    l += Delta;
                }
                else
                { // GeomAbs_Intersection
                    l = Precision.Infinite();
                }
            }
            G2dOC = new Geom2d_TrimmedCurve(CC, f, l);
        }
        else
        {
            Geom2d_TrimmedCurve G2dT = new(G2d, f, l);
            G2dOC = new Geom2d_OffsetCurve(G2dT, anOffset);
        }

        // Bind the edges in my Map.
        if (G2dOC is not null)
        {
            TopoDS_Edge OE = new BRepLib_MakeEdge(G2dOC, RefPlane).InternalEdge();
            OE.Orientation(E.Orientation());
            NCollection_List<TopoDS_Shape> LL = new();
            LL.Append(OE);
            Map.Add(E, LL);
        }
    }

    // The vertex of a node of the locus that lies on the offset (one per node).
    private static bool VertexFromNode(MAT_Node aNode,
                                       double Offset,
                                       in gp_Pnt2d PN,
                                       NCollection_DataMap<MAT_Node, TopoDS_Shape> MapNodeVertex,
                                       ref TopoDS_Vertex VN)
    {
        bool Status;
        double Tol = Precision.Confusion();
        BRep_Builder B = new();

        if (!aNode.Infinite() && Math.Abs(aNode.Distance() - Offset) < Tol)
        {
            //------------------------------------------------
            // the Node gives a vertex on the offset
            //------------------------------------------------
            if (MapNodeVertex.IsBound(aNode))
            {
                VN = TopoDS.Vertex(MapNodeVertex[aNode]);
            }
            else
            {
                gp_Pnt P = new(PN.X(), PN.Y(), 0.0);
                B.MakeVertex(ref VN);
                B.UpdateVertex(VN, P, Precision.Confusion());
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

    private static void StoreInMap(in TopoDS_Shape V1, in TopoDS_Shape V2, NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> MapVV)
    {
        TopoDS_Shape NewV = V2;
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
    private static void TrimEdge(in TopoDS_Edge E,
                                 in TopoDS_Shape ProE,
                                 in TopoDS_Face AllSpine,
                                 NCollection_List<TopoDS_Shape> Detromp,
                                 NCollection_Sequence<TopoDS_Shape> TheVer,
                                 NCollection_Sequence<double> ThePar,
                                 NCollection_Sequence<TopoDS_Shape> S,
                                 NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> MapVV,
                                 int IndOfE)
    {
        bool Change = true;
        BRep_Builder TheBuilder = new();
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
        if (!BRep_Tool.Degenerated(E))
        {
            for (int k = 1; k <= TheVer.Length(); k++)
            {
                if (DoubleOrNotInside(Detromp, TopoDS.Vertex(TheVer.Value(k))))
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
        if (!BRep_Tool.Degenerated(E))
        {
            double aParTol = 2.0 * Precision.PConfusion();
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
            TopoDS_Shape aLocalShape = E.EmptyCopied();
            TopoDS_Edge NewEdge = TopoDS.Edge(aLocalShape);
            TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            BRep_Tool.CurveOnSurface(E, out Geom2d_Curve? PCurve, out Geom_Surface? _, out TopLoc_Location _, out double fpar, out double lpar);

            bool ToTrimAsOrigin = IsInnerEdge(ProE, AllSpine, out double TrPar1, out double TrPar2);

            if (IndOfE == 1) // first edge of open wire
            {
                if (NewEdge.Orientation() == TopAbs_FORWARD)
                {
                    if (ToTrimAsOrigin)
                    {
                        fpar = TrPar1;
                        gp_Pnt2d TrPnt2d = PCurve!.Value(fpar);
                        gp_Pnt TrPnt = new(TrPnt2d.X(), TrPnt2d.Y(), 0.0);
                        V1 = new BRepLib_MakeVertex(TrPnt).InternalVertex();
                    }
                    TheBuilder.Add(NewEdge, V1.Oriented(TopAbs_FORWARD));
                    TheBuilder.Add(NewEdge, TheVer.First().Oriented(TopAbs_REVERSED));
                    TheBuilder.Range(NewEdge, fpar, ThePar.First());
                }
                else
                {
                    if (ToTrimAsOrigin)
                    {
                        lpar = TrPar2;
                        gp_Pnt2d TrPnt2d = PCurve!.Value(lpar);
                        gp_Pnt TrPnt = new(TrPnt2d.X(), TrPnt2d.Y(), 0.0);
                        V2 = new BRepLib_MakeVertex(TrPnt).InternalVertex();
                    }
                    TheBuilder.Add(NewEdge, TheVer.First().Oriented(TopAbs_REVERSED));
                    TheBuilder.Add(NewEdge, V2.Oriented(TopAbs_FORWARD));
                    TheBuilder.Range(NewEdge, ThePar.First(), lpar);
                }
            }
            else // last edge of open wire
            {
                if (NewEdge.Orientation() == TopAbs_FORWARD)
                {
                    if (ToTrimAsOrigin)
                    {
                        lpar = TrPar2;
                        gp_Pnt2d TrPnt2d = PCurve!.Value(lpar);
                        gp_Pnt TrPnt = new(TrPnt2d.X(), TrPnt2d.Y(), 0.0);
                        V2 = new BRepLib_MakeVertex(TrPnt).InternalVertex();
                    }
                    TheBuilder.Add(NewEdge, TheVer.First().Oriented(TopAbs_FORWARD));
                    TheBuilder.Add(NewEdge, V2.Oriented(TopAbs_REVERSED));
                    TheBuilder.Range(NewEdge, ThePar.First(), lpar);
                }
                else
                {
                    if (ToTrimAsOrigin)
                    {
                        fpar = TrPar1;
                        gp_Pnt2d TrPnt2d = PCurve!.Value(fpar);
                        gp_Pnt TrPnt = new(TrPnt2d.X(), TrPnt2d.Y(), 0.0);
                        V1 = new BRepLib_MakeVertex(TrPnt).InternalVertex();
                    }
                    TheBuilder.Add(NewEdge, V1.Oriented(TopAbs_REVERSED));
                    TheBuilder.Add(NewEdge, TheVer.First().Oriented(TopAbs_FORWARD));
                    TheBuilder.Range(NewEdge, fpar, ThePar.First());
                }
            }
            S.Append(NewEdge);
        }
        else
        {
            for (int k = 1; k < TheVer.Length(); k = k + 2)
            {
                TopoDS_Shape aLocalShape = E.EmptyCopied();
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

                S.Append(NewEdge);
            }
        }
    }

    private static bool IsInnerEdge(in TopoDS_Shape ProE, in TopoDS_Face AllSpine, out double TrPar1, out double TrPar2)
    {
        TrPar1 = TrPar2 = 0.0;
        if (ProE.ShapeType() != TopAbs_EDGE)
        {
            return false;
        }

        TopoDS_Edge anEdge = TopoDS.Edge(ProE);

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VEmap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(AllSpine, TopAbs_VERTEX, TopAbs_EDGE, VEmap);
        for (int i = 1; i <= VEmap.Extent(); i++)
        {
            NCollection_List<TopoDS_Shape> LE = VEmap[i];
            if (LE.Extent() == 1 && anEdge.IsSame(LE.First()))
            {
                return false;
            }
        }

        BRep_Tool.Range(anEdge, out TrPar1, out TrPar2);
        return true;
    }

    // True if V appears twice in LV or is not inside.
    private static bool DoubleOrNotInside(NCollection_List<TopoDS_Shape> LV, in TopoDS_Vertex V)
    {
        bool Vu = false;
        NCollection_List<TopoDS_Shape>.Iterator it = new(LV);

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

    private static bool IsSmallClosedEdge(in TopoDS_Edge anEdge, in TopoDS_Vertex aVertex)
    {
        gp_Pnt PV = BRep_Tool.Pnt(aVertex);
        gp_Pnt2d PV2d = new(), Pfirst, Plast, Pmid;
        PV2d.SetCoord(PV.X(), PV.Y());

        Geom2d_Curve PCurve;
        BRep_CurveRepresentation CurveRep = ((BRep_TEdge)anEdge.TShape()!).Curves().First();
        PCurve = CurveRep.PCurve()!;

        double fpar = ((BRep_GCurve)CurveRep).First();
        double lpar = ((BRep_GCurve)CurveRep).Last();
        Pfirst = PCurve.Value(fpar);
        Plast = PCurve.Value(lpar);
        Pmid = PCurve.Value((fpar + lpar) * 0.5);

        double theTol = BRep_Tool.Tolerance(aVertex);
        theTol *= 1.5;

        double dist1 = Pfirst.Distance(PV2d);
        double dist2 = Plast.Distance(PV2d);
        double dist3 = Pmid.Distance(PV2d);

        return dist1 <= theTol && dist2 <= theTol && dist3 <= theTol;
    }

    // The edges, other than lines and circles, whose curvature at an end is at least 1/Offset on the side of the offset.
    private static void CheckBadEdges(in TopoDS_Face Spine,
                                      double Offset,
                                      BRepMAT2d_BisectingLocus Locus,
                                      BRepMAT2d_LinkTopoBilo Link,
                                      NCollection_List<TopoDS_Shape> BadEdges)
    {
        TopoDS_Face F = TopoDS.Face(Spine.Oriented(TopAbs_FORWARD));
        double eps = Precision.Confusion();
        double LimCurv = 1.0 / Offset;

        NCollection_Map<TopoDS_Shape> aMap = new(TopTools_ShapeMapHasher.Instance);

        for (int ic = 1; ic <= Locus.NumberOfContours(); ic++)
        {
            for (int ie = 1; ie <= Locus.NumberOfElts(ic); ie++)
            {
                TopoDS_Shape SE = Link.GeneratingShape(Locus.BasicElt(ic, ie));
                if (SE.ShapeType() == TopAbs_EDGE)
                {
                    if (aMap.Contains(SE))
                    {
                        continue;
                    }

                    TopoDS_Edge E = TopoDS.Edge(SE);

                    Geom2d_Curve G2d = BRep_Tool.CurveOnSurface(E, F, out double f, out double l)!;

                    Geom2dAdaptor_Curve AC = new(G2d, f, l);
                    GeomAbs_CurveType aCType = AC.GetType();

                    if (aCType != GeomAbs_Line && aCType != GeomAbs_Circle)
                    {
                        bool reverse = false;
                        if (E.Orientation() == TopAbs_FORWARD)
                        {
                            reverse = true;
                        }

                        gp_Pnt2d P, Pc;
                        gp_Dir2d N;

                        GeomLProp_CLProps2d aCLProps = new(G2d, 2, eps);

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
                            gp_Vec2d Dir = new(P, Pc);
                            if (N.Dot(new gp_Dir2d(Dir)) > 0.0) // C++ converts the vector to a direction implicitly
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
                            gp_Vec2d Dir = new(P, Pc);
                            if (N.Dot(new gp_Dir2d(Dir)) > 0.0) // C++ converts the vector to a direction implicitly
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
                                     NCollection_Sequence<gp_Pnt> Points,
                                     Adaptor3d_Curve C,
                                     double Deflection,
                                     double U1,
                                     double U2,
                                     double EPSILON,
                                     int Nbmin)
    {
        double UU1 = Min(U1, U2);
        double UU2 = Max(U1, U2);

        C.D1(UU1, out gp_Pnt Pdeb, out gp_Vec Ddeb);
        Parameters.Append(UU1);
        Points.Append(Pdeb);

        C.D1(UU2, out gp_Pnt Pfin, out gp_Vec Dfin);

        double aDelta = UU2 - UU1;
        double aDist = Pdeb.Distance(Pfin);

        if ((aDelta / aDist) > 5.0e-14)
        {
            QuasiFleche(C, Deflection * Deflection, UU1, Pdeb, Ddeb, UU2, Pfin, Dfin, Nbmin, EPSILON * EPSILON, Parameters, Points);
        }

        return true;
    }

    private static void QuasiFleche(Adaptor3d_Curve C,
                                    double Deflection2,
                                    double Udeb,
                                    in gp_Pnt Pdeb,
                                    in gp_Vec Vdeb,
                                    double Ufin,
                                    in gp_Pnt Pfin,
                                    in gp_Vec Vfin,
                                    int Nbmin,
                                    double Eps,
                                    NCollection_Sequence<double> Parameters,
                                    NCollection_Sequence<gp_Pnt> Points)
    {
        int Ptslength = Points.Length();
        double Udelta = Ufin - Udeb;
        gp_Pnt Pdelta;
        gp_Vec Vdelta;
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

        double Norme = new gp_Vec(Pdeb, Pdelta).SquareMagnitude();
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
            gp_Pnt Pmid = new((Pdeb.XYZ() + Pdelta.XYZ()) / 2.0);
            gp_Pnt Pverif = C.Value(Udeb + Udelta / 2.0);
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

    private static bool CheckSmallParamOnEdge(in TopoDS_Edge anEdge)
    {
        NCollection_List<BRep_CurveRepresentation> aList = ((BRep_TEdge)anEdge.TShape()!).Curves();
        if (!aList.IsEmpty())
        {
            BRep_CurveRepresentation CRep = ((BRep_TEdge)anEdge.TShape()!).Curves().First();
            double f = ((BRep_GCurve)CRep).First();
            double l = ((BRep_GCurve)CRep).Last();
            if (Math.Abs(l - f) < Precision.PConfusion())
            {
                return false;
            }
        }
        return true;
    }
}
