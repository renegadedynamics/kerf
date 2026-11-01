// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Provides a tool to cut edges with vertices and faces with wires, and rebuilds the shape containing the edges and
/// the faces.
/// </summary>
/// <remarks>
/// <para>
/// As in OCCT, it writes into what it is given: the point representations of the split edges' vertices and of the
/// added vertices, the continuities of the added wires' edges. So the shape, and each vertex, edge and wire given, is
/// worked on as a copy when frozen, all sharing their copies; the first <see cref="DescendantShapes"/> ends the
/// splitting and gives back the caller's sub-shapes it left unchanged (decision 1013). The shapes it hands out are
/// frozen; ported callers (LocOpe_Spliter) read the <c>Internal</c> forms, which share the spliter's working shapes.
/// </para>
/// <para>
/// <see cref="LeftOf"/> before the splitting ends answers with the working faces, which later additions may still
/// change (they are not frozen).
/// </para>
/// </remarks>
public sealed class LocOpe_SplitShape
{
    private bool myDone;
    private TopoDS_Shape myShape = new();
    // OCCT iterates this map keyed by the shapes' addresses (CanSplit, which only tests); the port in insertion order
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_Map<TopoDS_Shape> myDblE = new(TopTools_ShapeMapHasher.Instance);
    private NCollection_List<TopoDS_Shape> myLeft = new();
    private LocOpe_WorkingShapes myWork = new();
    private bool myOwnsWork = true;

    /// <summary>Empty constructor.</summary>
    public LocOpe_SplitShape()
    {
        myDone = false;
    }

    /// <summary>Creates the process with the shape <paramref name="S"/>.</summary>
    public LocOpe_SplitShape(in TopoDS_Shape S)
    {
        Init(S);
    }

    /// <summary>For LocOpe_Spliter: the process on its working shape, sharing its working shapes (it ends them).</summary>
    internal LocOpe_SplitShape(in TopoDS_Shape S, LocOpe_WorkingShapes theWork)
    {
        myWork = theWork;
        myOwnsWork = false;
        initWorking(theWork.Working(S));
    }

    /// <summary>Initializes the process on the shape <paramref name="S"/>.</summary>
    public void Init(in TopoDS_Shape S)
    {
        if (myOwnsWork)
        {
            myWork = new LocOpe_WorkingShapes();
        }
        initWorking(myWork.Working(S));
    }

    private void initWorking(in TopoDS_Shape S)
    {
        myDone = false;
        myShape = S;
        myDblE.Clear();
        myMap.Clear();
        Put(myShape);
    }

    /// <summary>Tests if it is possible to split the edge <paramref name="E"/>.</summary>
    public bool CanSplit(in TopoDS_Edge E) => InternalCanSplit(TopoDS.Edge(myWork.Copied(E)));

    private bool InternalCanSplit(in TopoDS_Edge E)
    {
        if (myDone)
        {
            return false;
        }
        if (myMap.IsEmpty())
        {
            return false;
        }

        if (!myMap.IsBound(E))
        {
            return false;
        }

        // On verifie que l`edge n`appartient pas a un wire deja reconstruit
        TopExp_Explorer exp = new();
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itm in myMap)
        {
            if (itm.Key.ShapeType() == TopAbs_WIRE && !itm.Value.IsEmpty())
            {
                for (exp.Init(itm.Key, TopAbs_EDGE); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(E))
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    /// <summary>Adds the vertex <paramref name="V"/> on the edge <paramref name="E"/>, at parameter <paramref name="P"/>.</summary>
    public void Add(in TopoDS_Vertex V, double P, in TopoDS_Edge E)
    {
        if (myDone)
        {
            return;
        }
        InternalAdd(TopoDS.Vertex(myWork.Working(V)), P, TopoDS.Edge(myWork.Working(E)));
    }

    // Add(V, P, E) on working shapes
    internal void InternalAdd(in TopoDS_Vertex V, double P, in TopoDS_Edge E)
    {
        if (!InternalCanSplit(E))
        {
            return;
        }

        BRep_Builder B = new();
        NCollection_List<TopoDS_Shape> le = myMap[E];
        if (le.IsEmpty())
        {
            le.Append(E);
        }
        NCollection_List<TopoDS_Shape>.Iterator itl = new(le);
        double f, l;
        NCollection_List<TopoDS_Shape> aNewList = new();
        for (; itl.More(); itl.Next())
        {
            TopoDS_Edge edg1 = TopoDS.Edge(itl.Value());
            BRep_Tool.Range(edg1, out f, out l);
            if (P > f + Precision.PConfusion() && P < l - Precision.PConfusion())
            {
                break;
            }
            aNewList.Append(edg1);
        }
        if (!itl.More())
        {
            return;
        }
        TopoDS_Edge edg = TopoDS.Edge(itl.Value());
        le.Remove(itl);
        if (V.Orientation() == TopAbs_FORWARD || V.Orientation() == TopAbs_REVERSED)
        {
            edg.Orientation(TopAbs_FORWARD);
            TopExp.Vertices(edg, out TopoDS_Vertex aCurV1, out TopoDS_Vertex aCurV2);
            double aPar1 = BRep_Tool.Parameter(aCurV1, edg);

            double aPar2 = BRep_Tool.Parameter(aCurV2, edg);

            TopoDS_Shape aLocalShape = edg.EmptyCopied();
            TopoDS_Edge E1 = TopoDS.Edge(aLocalShape);
            aLocalShape = edg.EmptyCopied();
            TopoDS_Edge E2 = TopoDS.Edge(aLocalShape);
            E1.Orientation(TopAbs_FORWARD);
            E2.Orientation(TopAbs_FORWARD);
            TopoDS_Vertex newVtx = V;
            double aTolSplitV = BRep_Tool.Tolerance(V);

            aCurV1.Orientation(TopAbs_FORWARD);

            B.Add(E1, aCurV1);
            // for degenerated edges tolerance of vertices should be set to maximal value
            // from tolerance of the vertex of the edge and tolerance of splitting vertex
            double aTolV1 = (BRep_Tool.Degenerated(edg) ? Max(BRep_Tool.Tolerance(aCurV1), aTolSplitV)
                                                         : BRep_Tool.Tolerance(aCurV1));

            B.UpdateVertex(aCurV1, aPar1, E1, aTolV1);
            newVtx.Orientation(TopAbs_REVERSED);
            B.Add(E1, newVtx);
            B.UpdateVertex(newVtx, P, E1, BRep_Tool.Tolerance(V));
            newVtx.Orientation(TopAbs_FORWARD);
            B.Add(E2, newVtx);
            B.UpdateVertex(newVtx, P, E2, BRep_Tool.Tolerance(V));

            aCurV2.Orientation(TopAbs_REVERSED);
            B.Add(E2, aCurV2);

            // for degenerated edges tolerance of vertices should be set to maximal value
            // from tolerance of the vertex of the edge and tolerance of splitting vertex
            double aTolV2 = (BRep_Tool.Degenerated(edg) ? aTolV1 : BRep_Tool.Tolerance(aCurV2));
            B.UpdateVertex(aCurV2, aPar2, E2, aTolV2);

            aNewList.Append(E1);
            aNewList.Append(E2);
            for (; itl.More(); itl.Next())
            {
                TopoDS_Edge edg1 = TopoDS.Edge(itl.Value());
                aNewList.Append(edg1);
            }
            myMap.UnBind(E);
            myMap.Bind(E, aNewList);
        }
        else
        {
            TopoDS_Shape aLocalShape = edg.EmptyCopied();
            TopoDS_Edge E1 = TopoDS.Edge(aLocalShape);

            TopExp_Explorer exp = new();
            for (exp.Init(edg, TopAbs_VERTEX); exp.More(); exp.Next())
            {
                TopoDS_Vertex vtx = TopoDS.Vertex(exp.Current());
                f = BRep_Tool.Parameter(vtx, edg);
                B.Add(E1, vtx);
                B.UpdateVertex(vtx, f, E1, BRep_Tool.Tolerance(vtx));
            }
            B.Add(E1, V);
            B.UpdateVertex(V, P, E1, BRep_Tool.Tolerance(V));
            le.Append(E1);
        }
    }

    /// <summary>Adds the list of wires <paramref name="Lwires"/> on the face <paramref name="F"/>.</summary>
    public bool Add(NCollection_List<TopoDS_Shape> Lwires, in TopoDS_Face F)
    {
        if (myDone)
        {
            return false;
        }
        NCollection_List<TopoDS_Shape> aWires = new();
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(Lwires); anIt.More(); anIt.Next())
        {
            aWires.Append(myWork.Working(anIt.Value()));
        }
        return InternalAdd(aWires, TopoDS.Face(myWork.Working(F)));
    }

    // Add(Lwires, F) on working shapes
    internal bool InternalAdd(NCollection_List<TopoDS_Shape> Lwires, in TopoDS_Face F)
    {
        if (myDone)
        {
            return false;
        }

        NCollection_List<TopoDS_Shape> lf = myMap[F];
        if (lf.IsEmpty())
        {
            Rebuild(F);
        }

        // On cherche la face descendante de F qui continent le wire
        lf = myMap[F];
        NCollection_List<TopoDS_Shape>.Iterator itl = new(lf);

        BRepTools.Update(F);

        NCollection_List<TopoDS_Shape> aLInside = new();
        for (; itl.More(); itl.Next())
        {
            TopoDS_Face fac = TopoDS.Face(itl.Value());
            NCollection_List<TopoDS_Shape>.Iterator itwires = new(Lwires);
            for (; itwires.More(); itwires.Next())
            {
                TopoDS_Wire aWire = TopoDS.Wire(itwires.Value());
                if (IsInside(fac, aWire))
                {
                    aLInside.Append(aWire);
                }
            }
            if (aLInside.Extent() != 0)
            {
                break;
            }
        }
        if (aLInside.Extent() == 0 || !itl.More())
        {
            return false;
        }

        TopoDS_Face FaceRef = TopoDS.Face(itl.Value());
        FaceRef.Orientation(TopAbs_FORWARD);
        lf.Remove(itl);

        NCollection_List<TopoDS_Shape> NewWires = new();

        NCollection_DataMap<TopoDS_Shape, int> SectionsTimes = new(TopTools_ShapeMapHasher.Instance);
        for (itl.Initialize(aLInside); itl.More(); itl.Next())
        {
            SectionsTimes.Bind(itl.Value(), 2);
        }

        NCollection_List<TopoDS_Shape> BreakVertices = new();
        NCollection_List<TopoDS_Shape> BreakOnWires = new();

        // OCCT iterates this map keyed by the vertices' addresses to count a wire's entries, which isn't order dependent
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> VerWireMap = new(TopTools_ShapeMapHasher.Instance);
        int i;
        TopExp_Explorer ExploF = new(), ExploW = new();
        for (itl.Initialize(aLInside); itl.More(); itl.Next())
        {
            TopoDS_Wire aSection = TopoDS.Wire(itl.Value());
            TopoDS_Vertex[] Ver = new TopoDS_Vertex[2];
            TopExp.Vertices(aSection, out Ver[0], out Ver[1]);
            for (i = 0; i < 2; i++)
            {
                if (VerWireMap.IsBound(Ver[i]))
                {
                    continue;
                }
                for (ExploF.Init(FaceRef, TopAbs_WIRE); ExploF.More(); ExploF.Next())
                {
                    TopoDS_Shape aWire = ExploF.Current();
                    TopoDS_Shape aVer = new();
                    for (ExploW.Init(aWire, TopAbs_VERTEX); ExploW.More(); ExploW.Next())
                    {
                        aVer = ExploW.Current();
                        if (aVer.IsSame(Ver[i]))
                        {
                            break;
                        }
                    }
                    if (aVer.IsSame(Ver[i]))
                    {
                        VerWireMap.Bind(aVer, aWire);
                        break;
                    }
                }
            }
        }

        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> VerSecMap = new(TopTools_ShapeMapHasher.Instance);
        for (itl.Initialize(aLInside); itl.More(); itl.Next())
        {
            TopoDS_Wire aWire = TopoDS.Wire(itl.Value());
            TopExp.Vertices(aWire, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            NCollection_List<TopoDS_Shape> LW1 = new(), LW2 = new();
            if (!VerSecMap.IsBound(V1))
            {
                VerSecMap.Bind(V1, LW1);
            }
            VerSecMap[V1].Append(aWire);
            if (!VerSecMap.IsBound(V2))
            {
                VerSecMap.Bind(V2, LW2);
            }
            VerSecMap[V2].Append(aWire);
        }

        TopoDS_Wire outerW = BRepTools.OuterWire(FaceRef);
        TopoDS_Wire CurWire = outerW;
        BRepLib_MakeWire MW;
        MW = new BRepLib_MakeWire();
        BRepTools_WireExplorer wexp = new(CurWire, FaceRef);
        for (;;)
        {
            TopoDS_Vertex theStartVertex = wexp.CurrentVertex(), CurVertex = new();
            TopoDS_Edge CurEdge = wexp.Current();
            TopoDS_Edge LastEdge = CurEdge;
            MW.Add(CurEdge);
            TopoDS_Wire aSectionWire = new();
            TopoDS_Vertex aBreakVertex;
            wexp.Next();
            if (!wexp.More())
            {
                wexp.Init(CurWire, FaceRef);
            }
            for (;;)
            {
                if (MW.InternalWire().Closed())
                {
                    break;
                }
                CurVertex = wexp.CurrentVertex();
                if (VerSecMap.IsBound(CurVertex))
                {
                    TopoDS_Shape aLocalWire = ChooseDirection(LastEdge, CurVertex, FaceRef, VerSecMap[CurVertex]);
                    aSectionWire = TopoDS.Wire(aLocalWire);
                    break;
                }
                CurEdge = wexp.Current();
                MW.Add(CurEdge);
                LastEdge = CurEdge;
                wexp.Next();
                if (!wexp.More())
                {
                    wexp.Init(CurWire, FaceRef);
                }
            }
            if (MW.InternalWire().Closed())
            {
                NewWires.Append(MW.InternalWire());
                theStartVertex = TopoDS.Vertex(BreakVertices.First());
                BreakVertices.RemoveFirst();
                CurWire = TopoDS.Wire(BreakOnWires.First());
                BreakOnWires.RemoveFirst();
                wexp.Init(CurWire, FaceRef);
                while (!wexp.CurrentVertex().IsSame(theStartVertex))
                {
                    wexp.Next();
                }
                MW = new BRepLib_MakeWire();
                continue;
            }
            aBreakVertex = CurVertex;
            BreakVertices.Append(aBreakVertex);
            BreakOnWires.Append(CurWire);
            for (;;)
            {
                MW.Add(aSectionWire);
                (SectionsTimes[aSectionWire])--;
                if (SectionsTimes[aSectionWire] == 0)
                {
                    SectionsTimes.UnBind(aSectionWire);
                }
                if (MW.InternalWire().Closed())
                {
                    NewWires.Append(MW.InternalWire());
                    if (SectionsTimes.IsEmpty())
                    {
                        break;
                    }
                    theStartVertex = TopoDS.Vertex(BreakVertices.First());
                    BreakVertices.RemoveFirst();
                    CurWire = TopoDS.Wire(BreakOnWires.First());
                    BreakOnWires.RemoveFirst();
                    wexp.Init(CurWire, FaceRef);
                    while (!wexp.CurrentVertex().IsSame(theStartVertex))
                    {
                        wexp.Next();
                    }
                    MW = new BRepLib_MakeWire();
                    break;
                }
                else
                {
                    TopExp.Vertices(aSectionWire, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    TopoDS_Vertex aStartVertex = (V1.IsSame(aBreakVertex)) ? V2 : V1;
                    CurWire = TopoDS.Wire(VerWireMap[aStartVertex]);

                    wexp.Init(CurWire, FaceRef);
                    while (!wexp.CurrentVertex().IsSame(aStartVertex))
                    {
                        wexp.Next();
                    }

                    NCollection_List<TopoDS_Shape> Lsections = VerSecMap[aStartVertex];
                    if (Lsections.Extent() == 1)
                    {
                        break;
                    }

                    // else: choose the way
                    TopoDS_Wire NextSectionWire =
                        TopoDS.Wire((aSectionWire.IsSame(Lsections.First())) ? Lsections.Last() : Lsections.First());

                    int Times = 0;
                    foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> itVW in VerWireMap)
                    {
                        if (itVW.Value.IsSame(CurWire))
                        {
                            Times++;
                        }
                    }
                    if (Times == 1) // it is inner touching wire
                    {
                        // InnerTouchingWiresOnVertex.Bind(aWire, aStartVertex);
                    }
                    else
                    {
                        // we have to choose the direction
                        TopoDS_Edge aStartEdge = wexp.Current();
                        NCollection_List<TopoDS_Shape> Ldirs = new();
                        Ldirs.Append(aStartEdge);
                        Ldirs.Append(NextSectionWire);
                        TopoDS_Shape theDirection = ChooseDirection(aSectionWire, aStartVertex, FaceRef, Ldirs);
                        if (theDirection.IsSame(aStartEdge))
                        {
                            break;
                        }
                    }
                    aSectionWire = NextSectionWire;
                    aBreakVertex = aStartVertex;
                } // end of else (MW is not closed)
            } // end of for (;;) (loop on section wires)
            if (SectionsTimes.IsEmpty())
            {
                break;
            }
        } // end of global for (;;)

        NCollection_List<TopoDS_Shape> NewFaces = new();
        BRep_Builder BB = new();
        for (itl.Initialize(NewWires); itl.More(); itl.Next())
        {
            TopoDS_Shape aLocalFace = FaceRef.EmptyCopied();
            TopoDS_Face aNewFace = TopoDS.Face(aLocalFace);
            aNewFace.Orientation(TopAbs_FORWARD);
            BB.Add(aNewFace, itl.Value());
            NewFaces.Append(aNewFace);
        }

        // Inserting holes
        NCollection_List<TopoDS_Shape> Holes = new();
        for (ExploF.Init(FaceRef, TopAbs_WIRE); ExploF.More(); ExploF.Next())
        {
            TopoDS_Shape aWire = ExploF.Current();
            ExploW.Init(aWire, TopAbs_EDGE);
            TopoDS_Shape anEdge = ExploW.Current();
            bool found = false;
            for (itl.Initialize(NewWires); itl.More(); itl.Next())
            {
                TopoDS_Shape aNewWire = itl.Value();
                for (ExploW.Init(aNewWire, TopAbs_EDGE); ExploW.More(); ExploW.Next())
                {
                    if (anEdge.IsSame(ExploW.Current()))
                    {
                        found = true;
                        break;
                    }
                }
                if (found)
                {
                    break;
                }
            }
            if (!found)
            {
                Holes.Append(aWire);
            }
        }
        NCollection_List<TopoDS_Shape>.Iterator itlNewF = new();
        for (itl.Initialize(Holes); itl.More(); itl.Next())
        {
            TopoDS_Wire aHole = TopoDS.Wire(itl.Value());
            for (itlNewF.Initialize(NewFaces); itlNewF.More(); itlNewF.Next())
            {
                TopoDS_Face aNewFace = TopoDS.Face(itlNewF.Value());
                if (IsInside(aNewFace, aHole))
                {
                    BB.Add(aNewFace, aHole);
                    break;
                }
            }
        }

        // Update "myMap"
        lf.Append(NewFaces);

        // Update of descendants of wires
        for (ExploF.Init(F, TopAbs_WIRE); ExploF.More(); ExploF.Next())
        {
            NCollection_List<TopoDS_Shape> ls = myMap[ExploF.Current()];
            ls.Clear();
        }
        ///////////////////

        // JAG 10.11.95 Codage des regularites
        for (itl.Initialize(aLInside); itl.More(); itl.Next())
        {
            for (ExploW.Init(itl.Value(), TopAbs_EDGE); ExploW.More(); ExploW.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(ExploW.Current());
                if (!BRep_Tool.HasContinuity(edg, F, F))
                {
                    BB.Continuity(edg, F, F, GeomAbs_CN);
                }
            }
        }
        return true;
    }

    /// <summary>Adds the wire <paramref name="W"/> on the face <paramref name="F"/>.</summary>
    public bool Add(in TopoDS_Wire W, in TopoDS_Face F)
    {
        if (myDone)
        {
            return false;
        }
        return InternalAdd(TopoDS.Wire(myWork.Working(W)), TopoDS.Face(myWork.Working(F)));
    }

    // Add(W, F) on working shapes
    internal bool InternalAdd(in TopoDS_Wire W, in TopoDS_Face F)
    {
        if (myDone)
        {
            return false;
        }

        TopExp_Explorer exp = new();
        NCollection_List<TopoDS_Shape> lf = myMap[F];
        if (lf.IsEmpty())
        {
            Rebuild(F);
        }
        try
        {
            if (!LocOpe.Closed(W, F))
            {
                if (!AddOpenWire(W, F))
                {
                    return false;
                }
            }
            else
            {
                if (!AddClosedWire(W, F))
                {
                    return false;
                }
            }
        }
        catch (Exception e) when (e is Standard_Failure or NullReferenceException && e is not (Standard_ImmutableObject or TopoDS_LockedShape))
        {
            // OCCT_DEBUG trace left out: "SpliShape internal problem detected, some faces may be lost"
            return false;
        }
        // JAG 10.11.95 Codage des regularites
        BRep_Builder B = new();
        for (exp.Init(W, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            if (!BRep_Tool.HasContinuity(edg, F, F))
            {
                B.Continuity(edg, F, F, GeomAbs_CN);
            }
        }
        return true;
    }

    private bool AddClosedWire(in TopoDS_Wire W, in TopoDS_Face F)
    {
        TopExp_Explorer exp = new();

        // On cherche la face descendante de F qui continent le wire
        NCollection_List<TopoDS_Shape> lf = myMap[F];
        NCollection_List<TopoDS_Shape>.Iterator itl = new(lf);
        for (; itl.More(); itl.Next())
        {
            TopoDS_Face fac = TopoDS.Face(itl.Value());

            if (IsInside(fac, W))
            {
                break;
            }
        }
        if (!itl.More())
        {
            return false;
        }

        BRep_Builder B = new();

        TopAbs_Orientation orWire = W.Orientation();
        TopoDS_Shape aLocalFace = F.EmptyCopied();
        TopoDS_Face newFace = TopoDS.Face(aLocalFace);
        newFace.Orientation(TopAbs_FORWARD);
        B.Add(newFace, W);
        BRepTopAdaptor_FClass2d classif = new(newFace, Precision.PConfusion());
        if (classif.PerformInfinitePoint() == TopAbs_IN)
        {
            // le wire donne defini un trou
            aLocalFace = F.EmptyCopied();
            newFace = TopoDS.Face(aLocalFace);
            newFace.Orientation(TopAbs_FORWARD);
            orWire = TopAbs.Reverse(orWire);
            B.Add(newFace, W.Oriented(orWire));
        }

        TopoDS_Face FaceRef = TopoDS.Face(itl.Value());
        FaceRef.Orientation(TopAbs_FORWARD);
        lf.Remove(itl);

        aLocalFace = FaceRef.EmptyCopied();
        TopoDS_Face newRef = TopoDS.Face(aLocalFace);
        newRef.Orientation(TopAbs_FORWARD);

        // On suppose que les edges du wire ont des courbes 2d.
        // Comme on ne change pas de surface de base, pas besoin d`UpdateEdge.

        for (exp.Init(FaceRef.Oriented(TopAbs_FORWARD), TopAbs_WIRE); exp.More(); exp.Next())
        {
            TopoDS_Wire wir = TopoDS.Wire(exp.Current());
            if (IsInside(F, wir, W))
            {
                B.Add(newFace, wir);
            }
            else
            {
                B.Add(newRef, wir);
            }
        }
        B.Add(newRef, W.Oriented(TopAbs.Reverse(orWire)));
        lf.Append(newRef);
        lf.Append(newFace);
        return true;
    }

    private static bool checkOverlapping(in TopoDS_Edge theEdge1, in TopoDS_Edge theEdge2, in TopoDS_Face theFace)
    {
        BRepAdaptor_Surface anAdS = new(theFace, false);

        double MaxTol = (BRep_Tool.Tolerance(theEdge1) + BRep_Tool.Tolerance(theEdge2));

        double aMaxTol2d = Max(anAdS.UResolution(MaxTol), anAdS.VResolution(MaxTol));
        double aTolAng = Math.PI / 180.0;
        Geom2dAPI_ProjectPointOnCurve proj = new();

        Geom2d_Curve aCrv1 = BRep_Tool.CurveOnSurface(theEdge1, theFace, out double aF1, out double aL1)!;
        Geom2d_Curve aCrv2 = BRep_Tool.CurveOnSurface(theEdge2, theFace, out double aF2, out double aL2)!;
        int j = 1, nbP = 4;
        double aStep = (aL2 - aF2) / nbP;
        for (; j < nbP; j++)
        {
            double par2 = aF2 + aStep * j;
            aCrv2.D1(par2, out gp_Pnt2d aP2d, out gp_Vec2d aV2);

            proj.Init(aP2d, aCrv1, aF1, aL1);
            // check intermediate points
            if (proj.NbPoints() == 0 || proj.LowerDistance() > aMaxTol2d)
            {
                return false;
            }
            double par1 = proj.LowerDistanceParameter();
            aCrv1.D1(par1, out gp_Pnt2d aP2d1, out gp_Vec2d aV1);

            if (!aV1.IsParallel(aV2, aTolAng))
            {
                return false;
            }
        }
        return true;
    }

    private bool AddOpenWire(in TopoDS_Wire W, in TopoDS_Face F)
    {
        // On cherche la face descendante de F qui continent le wire
        NCollection_List<TopoDS_Shape> lf = myMap[F];
        NCollection_List<TopoDS_Shape>.Iterator itl = new(lf);
        TopoDS_Vertex Vfirst, Vlast;

        BRepTools.Update(F);

        double tolf, toll, tol1;

        TopoDS_Shape aLocalShape = W.Oriented(TopAbs_FORWARD);
        TopExp.Vertices(TopoDS.Wire(aLocalShape), out Vfirst, out Vlast);

        tolf = BRep_Tool.Tolerance(Vfirst);
        toll = BRep_Tool.Tolerance(Vlast);
        tol1 = Max(tolf, toll);

        TopExp_Explorer exp = new(), exp2 = new();

        TopoDS_Wire wfirst = new(), wlast = new();
        for (; itl.More(); itl.Next())
        {
            TopoDS_Face fac = TopoDS.Face(itl.Value());
            if (!IsInside(fac, W))
            {
                continue;
            }

            fac.Orientation(TopAbs_FORWARD);
            bool ffound = false;
            bool lfound = false;
            for (exp.Init(fac, TopAbs_WIRE); exp.More(); exp.Next())
            {
                TopoDS_Wire wir = TopoDS.Wire(exp.Current());
                for (exp2.Init(wir, TopAbs_VERTEX); exp2.More(); exp2.Next())
                {
                    if (!ffound && exp2.Current().IsSame(Vfirst))
                    {
                        ffound = true;
                        wfirst = wir;
                    }
                    else if (!lfound && exp2.Current().IsSame(Vlast))
                    {
                        lfound = true;
                        wlast = wir;
                    }
                    if (ffound && lfound)
                    {
                        break;
                    }
                }
                if (exp2.More())
                {
                    break;
                }
            }
            if (exp.More())
            {
                break;
            }
        }
        if (!itl.More())
        {
            return false;
        }

        TopoDS_Face FaceRef = TopoDS.Face(itl.Value());
        FaceRef.Orientation(TopAbs_FORWARD);
        lf.Remove(itl);
        BRep_Builder B = new();

        BRepAdaptor_Surface BAS = new(FaceRef, false);

        bool IsPeriodic = BAS.IsUPeriodic() || BAS.IsVPeriodic();

        tol1 = Max(BAS.UResolution(tol1), BAS.VResolution(tol1));

        if (wfirst.IsSame(wlast))
        {
            // on cree 2 faces en remplacement de itl.Value()
            // Essai JAG
            NCollection_List<TopoDS_Shape> WiresFirst = new();
            for (exp.Init(wfirst, TopAbs_EDGE); exp.More(); exp.Next())
            {
                if (BRep_Tool.IsClosed(TopoDS.Edge(exp.Current()), FaceRef))
                {
                    myDblE.Add(exp.Current());
                }
                WiresFirst.Append(exp.Current());
            }

            TopAbs_Orientation orient;
            TopoDS_Wire newW1 = new(), newW2 = new();
            B.MakeWire(ref newW1);
            newW1.Orientation(TopAbs_FORWARD);
            B.MakeWire(ref newW2);
            newW2.Orientation(TopAbs_FORWARD);

            int nbE = 0;
            for (exp.Init(W.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                nbE++;
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                orient = edg.Orientation();
                WiresFirst.Append(edg);
                WiresFirst.Append(edg.Oriented(TopAbs.Reverse(orient)));
                myDblE.Add(edg);
            }

            NCollection_IndexedMap<TopoDS_Shape> PossE = new(TopTools_ShapeMapHasher.Instance);
            // OCCT's default hasher: the shapes' TShapes, locations and orientations
            NCollection_Map<TopoDS_Shape> MapE = new(EqualityComparer<TopoDS_Shape>.Default);
            TopoDS_Vertex vdeb, vfin;
            int nbPoss;

            // On recherche l`edge contenant Vlast
            TopoDS_Edge LastEdge = new();
            gp_Pnt2d pfirst, plast;
            gp_Vec2d dlast;
            Geom2d_Curve C2d;
            double f, l;

            for (exp.Init(W.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                for (exp2.Init(edg, TopAbs_VERTEX); exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(Vfirst))
                    {
                        break;
                    }
                }
                if (exp2.More())
                {
                    LastEdge = edg;
                    LastEdge.Orientation(edg.Orientation());
                    break;
                }
            }

            TopoDS_Shape aLocalFace = FaceRef.Oriented(wfirst.Orientation());
            C2d = BRep_Tool.CurveOnSurface(LastEdge, TopoDS.Face(aLocalFace), out f, out l)!;

            if (LastEdge.Orientation() == TopAbs_FORWARD)
            {
                pfirst = C2d.Value(f);
            }
            else
            {
                pfirst = C2d.Value(l);
            }

            for (exp.Init(W.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                if (nbE > 1 && edg.IsSame(LastEdge))
                {
                    continue;
                }
                for (exp2.Init(edg, TopAbs_VERTEX); exp2.More(); exp2.Next())
                {
                    if (exp2.Current().IsSame(Vlast))
                    {
                        break;
                    }
                }
                if (exp2.More())
                {
                    LastEdge = edg;
                    LastEdge.Orientation(edg.Orientation());
                    break;
                }
            }
            aLocalFace = FaceRef.Oriented(wfirst.Orientation());
            GetDirection(LastEdge, TopoDS.Face(aLocalFace), out plast, out dlast, false);

            bool cond = true;

            if (IsPeriodic)
            {
                cond = !(Vfirst.IsSame(Vlast) && SameUV(pfirst, plast, BAS));
            }
            else
            {
                cond = !(Vfirst.IsSame(Vlast));
            }

            while (cond)
            {
                PossE.Clear();

                // On enchaine par la fin
                NCollection_List<TopoDS_Shape>.Iterator lexp1 = new(WiresFirst);
                for (; lexp1.More(); lexp1.Next())
                {
                    TopoDS_Edge edg = TopoDS.Edge(lexp1.Value());

                    orient = edg.Orientation();
                    TopExp.Vertices(edg, out vdeb, out vfin);

                    if ((orient == TopAbs_FORWARD && Vlast.IsSame(vdeb))
                        || (orient == TopAbs_REVERSED && Vlast.IsSame(vfin)))
                    {
                        PossE.Add(edg);
                    }
                }
                nbPoss = PossE.Extent();
                if (nbPoss == 0)
                {
                    break;
                }

                TopoDS_Edge aNextEdge = new();
                if (nbPoss == 1)
                {
                    aNextEdge = TopoDS.Edge(PossE.FindKey(1));
                    TopoDS_Shape aLocalFaceTemp = FaceRef.Oriented(wfirst.Orientation());
                    GetDirection(aNextEdge, TopoDS.Face(aLocalFaceTemp), out plast, out dlast, false);
                }
                else if (nbPoss > 1)
                {
                    // Faire choix en U,V...
                    TopoDS_Shape aLocalFaceTemp = FaceRef.Oriented(wfirst.Orientation());

                    if (!ChoixUV(LastEdge, TopoDS.Face(aLocalFaceTemp), PossE, ref aNextEdge, ref plast, ref dlast))
                    {
                        return false;
                    }
                }

                if (nbPoss >= 1)
                {
                    if (aNextEdge.IsNull())
                    {
                        // loop is not closed. Split is not possible
                        return false;
                    }

                    if (MapE.Contains(aNextEdge))
                    {
                        break;
                    }
                    B.Add(newW1, aNextEdge);
                    MapE.Add(aNextEdge);
                    LastEdge = aNextEdge;

                    if (LastEdge.Orientation() == TopAbs_FORWARD)
                    {
                        Vlast = TopExp.LastVertex(LastEdge);
                    }
                    else
                    {
                        Vlast = TopExp.FirstVertex(LastEdge);
                    }

                    toll = BRep_Tool.Tolerance(Vlast);
                    tol1 = Max(tolf, toll);
                }
                // MODIFICATION PIERRE SMEYERS : si pas de possibilite, on sort avec erreur
                else
                {
                    // OCCT writes "erreur Spliter : pas de chainage du wire" to std::cout (left out)
                    return false;
                }
                // fin MODIF.

                tol1 = Max(BAS.UResolution(tol1), BAS.VResolution(tol1));
            }

            int nbAddBound = 0;
            NCollection_List<TopoDS_Shape>.Iterator lexp = new(WiresFirst);
            TopoDS_Shape anE1 = new(), anE2 = new();
            for (; lexp.More(); lexp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(lexp.Value());
                if (!MapE.Contains(edg))
                {
                    B.Add(newW2, edg);
                    MapE.Add(edg);
                    nbAddBound++;
                    if (anE1.IsNull())
                    {
                        anE1 = edg;
                    }
                    else
                    {
                        anE2 = edg;
                    }
                }
            }
            // check overlapping edges for second face
            if (nbAddBound < 2)
            {
                return false;
            }
            if (nbAddBound == 2 && !anE1.IsNull() && !anE2.IsNull())
            {
                if (checkOverlapping(TopoDS.Edge(anE1), TopoDS.Edge(anE2), FaceRef))
                {
                    return false;
                }
            }

            nbAddBound = 0;

            TopoDS_Shape anE11 = new(), anE12 = new();
            TopoDS_Iterator anItE = new(newW1, false);
            for (; anItE.More(); anItE.Next())
            {
                if (anItE.Value().ShapeType() != TopAbs_EDGE)
                {
                    continue;
                }
                nbAddBound++;
                if (anE11.IsNull())
                {
                    anE11 = anItE.Value();
                }
                else
                {
                    anE12 = anItE.Value();
                }
            }
            // check overlapping edges for first face
            if (nbAddBound < 2)
            {
                return false;
            }
            if (nbAddBound == 2 && !anE11.IsNull() && !anE12.IsNull())
            {
                if (checkOverlapping(TopoDS.Edge(anE11), TopoDS.Edge(anE12), FaceRef))
                {
                    return false;
                }
            }

            TopoDS_Face newF1, newF2;
            aLocalFace = FaceRef.EmptyCopied();
            newF1 = TopoDS.Face(aLocalFace);
            newF1.Orientation(TopAbs_FORWARD);
            aLocalFace = FaceRef.EmptyCopied();
            newF2 = TopoDS.Face(aLocalFace);
            newF2.Orientation(TopAbs_FORWARD);

            // modifs JAG 97.05.28

            B.Add(newF1, newW1);
            B.Add(newF2, newW2);
            for (exp.Init(FaceRef.Oriented(TopAbs_FORWARD), TopAbs_WIRE); exp.More(); exp.Next())
            {
                TopoDS_Wire wir = TopoDS.Wire(exp.Current());
                if (!wir.IsSame(wfirst))
                {
                    if (IsInside(newF1, wir))
                    {
                        B.Add(newF1, wir);
                    }
                    else if (IsInside(newF2, wir))
                    {
                        B.Add(newF2, wir);
                    }
                    else
                    {
                        // Ce wire est ni dans newF2 ni dans newF1
                        // Peut etre faut il construire une troisieme face
                        // OCCT writes "WARNING: LocOpe_SPlitShape : Ce wire est ni dans newF2 ni dans newF1" to
                        // std::cout (left out)
                    }
                }
            }
            lf.Append(newF1);
            lf.Append(newF2);

            // Mise a jour des descendants des wires
            for (exp.Init(F, TopAbs_WIRE); exp.More(); exp.Next())
            {
                NCollection_List<TopoDS_Shape> ls = myMap[exp.Current()];
                itl.Initialize(ls);
                for (; itl.More(); itl.Next())
                {
                    if (itl.Value().IsSame(wfirst))
                    {
                        break;
                    }
                }
                if (itl.More())
                { // on a trouve le wire
                    ls.Remove(itl);
                    ls.Append(newW1);
                    ls.Append(newW2);
                }
            }
        }
        else
        {
            // on ne cree qu`une seule face
            TopoDS_Wire outerW = BRepTools.OuterWire(FaceRef);
            TopoDS_Wire newWire = new();
            TopoDS_Face newFace;
            B.MakeWire(ref newWire);
            newWire.Orientation(TopAbs_FORWARD);
            TopAbs_Orientation orient, orRelat;

            if (wfirst.Orientation() == wlast.Orientation())
            {
                orRelat = TopAbs_FORWARD;
            }
            else
            {
                orRelat = TopAbs_REVERSED;
            }

            if (wlast.IsSame(outerW))
            {
                wlast = wfirst;
                wfirst = outerW;
            }

            // Edges de wfirst
            for (exp.Init(wfirst.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                B.Add(newWire, TopoDS.Edge(exp.Current()));
            }

            // Edges de wlast
            for (exp.Init(wlast.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                orient = TopAbs.Compose(edg.Orientation(), orRelat);
                B.Add(newWire, edg.Oriented(orient));
            }

            // Edges du wire ajoute, et dans les 2 sens
            for (exp.Init(W.Oriented(TopAbs_FORWARD), TopAbs_EDGE); exp.More(); exp.Next())
            {
                TopoDS_Edge edg = TopoDS.Edge(exp.Current());
                orient = edg.Orientation();
                B.Add(newWire, edg.Oriented(orient));
                B.Add(newWire, edg.Oriented(TopAbs.Reverse(orient)));
                myDblE.Add(edg.Oriented(orient));
            }

            // on refait une face

            TopoDS_Shape aLocalFace = FaceRef.EmptyCopied();
            newFace = TopoDS.Face(aLocalFace);
            FaceRef.Orientation(TopAbs_FORWARD);
            for (exp.Init(FaceRef.Oriented(TopAbs_FORWARD), TopAbs_WIRE); exp.More(); exp.Next())
            {
                TopoDS_Wire wir = TopoDS.Wire(exp.Current());
                if (wir.IsSame(wfirst))
                {
                    B.Add(newFace, newWire.Oriented(wir.Orientation()));
                }
                else if (!wir.IsSame(wlast))
                {
                    B.Add(newFace, wir);
                }
            }
            lf.Append(newFace);

            // Mise a jour des descendants des wires
            for (exp.Init(F, TopAbs_WIRE); exp.More(); exp.Next())
            {
                NCollection_List<TopoDS_Shape> ls = myMap[exp.Current()];
                itl.Initialize(ls);
                bool touch = false;
                while (itl.More())
                {
                    if (itl.Value().IsSame(wfirst) || itl.Value().IsSame(wlast))
                    {
                        ls.Remove(itl);
                        touch = true;
                    }
                    else
                    {
                        itl.Next();
                    }
                }
                if (touch)
                {
                    ls.Append(newWire);
                }
            }
        }
        _ = tol1;
        return true;
    }

    /// <summary>
    /// Returns the "left" part defined by the wire <paramref name="W"/> on the face <paramref name="F"/>: a list of
    /// faces. <paramref name="F"/> is considered with its topological orientation in the original shape,
    /// <paramref name="W"/> with its orientation. Frozen once the splitting has ended.
    /// </summary>
    public NCollection_List<TopoDS_Shape> LeftOf(in TopoDS_Wire W, in TopoDS_Face F)
    {
        NCollection_List<TopoDS_Shape> aLeft = InternalLeftOf(TopoDS.Wire(myWork.Copied(W)), TopoDS.Face(myWork.Copied(F)));
        return myDone && myOwnsWork ? LocOpe_WorkingShapes.Published(aLeft) : aLeft;
    }

    // LeftOf on working shapes
    internal NCollection_List<TopoDS_Shape> InternalLeftOf(in TopoDS_Wire W, in TopoDS_Face F)
    {
        if (myShape.IsNull())
        {
            throw new Standard_NoSuchObject();
        }

        TopExp_Explorer exp = new(), expw = new(), expf = new();
        exp.Init(myShape, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(F))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new Standard_NoSuchObject();
        }
        myLeft = new NCollection_List<TopoDS_Shape>(); // OCCT clears the list it returned before

        TopoDS_Face theFace = TopoDS.Face(exp.Current());
        TopAbs_Orientation orFace = theFace.Orientation();
        NCollection_List<TopoDS_Shape>.Iterator itl = new(), itl2 = new();

        for (expw.Init(W, TopAbs_EDGE); expw.More(); expw.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(expw.Current());
            for (itl.Initialize(myMap[theFace]); itl.More(); itl.Next())
            {
                TopoDS_Face fac = TopoDS.Face(itl.Value());
                fac.Orientation(orFace);
                for (expf.Init(fac, TopAbs_EDGE); expf.More(); expf.Next())
                {
                    TopoDS_Edge edgbis = TopoDS.Edge(expf.Current());
                    if (edgbis.IsSame(edg) && edgbis.Orientation() == edg.Orientation())
                    {
                        for (itl2.Initialize(myLeft); itl2.More(); itl2.Next())
                        {
                            if (itl2.Value().IsSame(fac))
                            {
                                break;
                            }
                        }
                        if (!itl2.More())
                        { // la face n`est pas deja presente
                            myLeft.Append(fac);
                        }
                        break;
                    }
                }
                if (expf.More())
                { // face found
                    break;
                }
            }
        }
        return myLeft;
    }

    /// <summary>
    /// Returns the list of descendant shapes of <paramref name="S"/> (frozen). The first call ends the splitting.
    /// </summary>
    public NCollection_List<TopoDS_Shape> DescendantShapes(in TopoDS_Shape S)
    {
        NCollection_List<TopoDS_Shape> aList = InternalDescendantShapes(myWork.Copied(S));
        return myOwnsWork ? LocOpe_WorkingShapes.Published(aList) : aList;
    }

    // DescendantShapes on working shapes; ends the splitting, and gives back the caller's unchanged sub-shapes when
    // the working shapes are this process's own
    internal NCollection_List<TopoDS_Shape> InternalDescendantShapes(in TopoDS_Shape S)
    {
        if (!myDone)
        {
            Rebuild(myShape);
            myDone = true;
            if (myOwnsWork && myWork.IsCopied())
            {
                var aShapes = new List<TopoDS_Shape>();
                foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anItem in myMap)
                {
                    aShapes.AddRange(anItem.Value);
                }
                myWork.Finish(aShapes);
                foreach (TopoDS_Shape aKey in myMap.Keys.ToList())
                {
                    myMap[aKey] = myWork.Final(myMap[aKey]);
                }
            }
        }
        // OCCT_DEBUG trace left out ("Le shape comporte des faces invalides" when myDblE isn't empty)
        return myMap[S];
    }

    /// <summary>Returns the "original" shape (the caller's).</summary>
    public TopoDS_Shape Shape() => myWork.Original(myShape);

    private void Put(in TopoDS_Shape S)
    {
        if (!myMap.IsBound(S))
        {
            NCollection_List<TopoDS_Shape> thelist = new();
            myMap.Bind(S, thelist);
            if (S.ShapeType() != TopAbs_VERTEX)
            {
                for (TopoDS_Iterator theIterator = new(S); theIterator.More(); theIterator.Next())
                {
                    Put(theIterator.Value());
                }
            }
            else
            {
                myMap[S].Append(S);
            }
        }
    }

    private static void updateToleraces(in TopoDS_Face theFace,
                                        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMap)
    {
        TopExp_Explorer aExpE = new(theFace, TopAbs_EDGE);
        for (; aExpE.More(); aExpE.Next())
        {
            if (!theMap.IsBound(aExpE.Current()))
            {
                continue;
            }
            NCollection_List<TopoDS_Shape> lEdges = theMap[aExpE.Current()];
            if (lEdges.Extent() <= 1)
            {
                continue;
            }

            NCollection_List<TopoDS_Shape>.Iterator itrE = new(lEdges);
            ShapeAnalysis_Edge aSae = new();

            for (; itrE.More(); itrE.Next())
            {
                TopoDS_Edge aCurE = TopoDS.Edge(itrE.Value());
                double amaxdev = 0.0;
                if (aSae.CheckSameParameter(aCurE, theFace, ref amaxdev))
                {
                    BRep_Builder aB = new();
                    aB.UpdateEdge(aCurE, amaxdev);
                }
            }
        }
    }

    private bool Rebuild(in TopoDS_Shape S)
    {
        if (S.ShapeType() == TopAbs_FACE)
        {
            updateToleraces(TopoDS.Face(S), myMap);
        }
        NCollection_List<TopoDS_Shape>.Iterator itr = new(myMap[S]);
        if (itr.More())
        {
            return !itr.Value().IsSame(S);
        }
        bool rebuild = false;
        TopoDS_Iterator it = new();
        for (it.Initialize(S); it.More(); it.Next())
        {
            rebuild = Rebuild(it.Value()) || rebuild;
        }

        if (rebuild)
        {
            BRep_Builder B = new();
            TopoDS_Shape result = S.EmptyCopied();
            TopAbs_Orientation orient;
            for (it.Initialize(S); it.More(); it.Next())
            {
                orient = it.Value().Orientation();
                for (itr.Initialize(myMap[it.Value()]); itr.More(); itr.Next())
                {
                    B.Add(result, itr.Value().Oriented(orient));
                }
            }
            // Assign "Closed" flag for Wires and Shells only
            if (result.ShapeType() == TopAbs_WIRE || result.ShapeType() == TopAbs_SHELL)
            {
                result.Closed(BRep_Tool.IsClosed(result));
            }
            myMap[S].Append(result);
        }
        else
        {
            myMap[S].Append(S);
        }
        return rebuild;
    }

    private static bool SameUV(in gp_Pnt2d P1, in gp_Pnt2d P2, BRepAdaptor_Surface theBAS)
    {
        bool isSame = true;
        if (theBAS.IsUPeriodic())
        {
            isSame = (Math.Abs(P1.X() - P2.X()) < theBAS.UPeriod() * 0.5);
        }
        if (theBAS.IsVPeriodic())
        {
            isSame = (isSame && (Math.Abs(P1.Y() - P2.Y()) < theBAS.VPeriod() * 0.5));
        }
        return isSame;
    }

    private static bool IsInside(in TopoDS_Face F, in TopoDS_Wire W1, in TopoDS_Wire W2)
    {
        // Attention, c`est tres boeuf !!!!
        BRep_Builder B = new();
        TopoDS_Shape aLocalShape = F.EmptyCopied();
        TopoDS_Face newFace = TopoDS.Face(aLocalShape);

        newFace.Orientation(TopAbs_FORWARD);
        B.Add(newFace, W2);
        BRepTopAdaptor_FClass2d classif = new(newFace, Precision.PConfusion());
        bool Reversed = false;
        if (classif.PerformInfinitePoint() == TopAbs_IN)
        {
            // le wire donne defini un trou
            Reversed = true;
        }

        TopExp_Explorer exp = new(W1, TopAbs_EDGE);
        TopoDS_Edge edg = TopoDS.Edge(exp.Current());
        TopExp_Explorer exp2 = new(edg, TopAbs_VERTEX);
        TopoDS_Vertex vtx = TopoDS.Vertex(exp2.Current());
        double prm = BRep_Tool.Parameter(vtx, edg);
        Geom2d_Curve? C2d = BRep_Tool.CurveOnSurface(edg, F, out _, out _);
        if (C2d is null)
        {
            // OCCT_DEBUG trace left out ("Edge is not on surface")
            return false;
        }
        gp_Pnt2d pt2d = C2d.Value(prm);
        if (!Reversed)
        {
            return (classif.Perform(pt2d) == TopAbs_IN);
        }
        else
        {
            return (classif.Perform(pt2d) == TopAbs_OUT);
        }
    }

    private static bool IsInside(in TopoDS_Face F, in TopoDS_Wire W)
    {
        // Attention, c`est tres boeuf !!!!
        TopExp_Explorer exp = new(W, TopAbs_EDGE);
        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge edg = TopoDS.Edge(exp.Current());
            double prm;
            Geom2d_Curve C2d = BRep_Tool.CurveOnSurface(edg, F, out double f, out double l)!;
            if (!Precision.IsNegativeInfinite(f) && !Precision.IsPositiveInfinite(l))
            {
                prm = (f + l) / 2.0;
            }
            else
            {
                if (Precision.IsNegativeInfinite(f) && Precision.IsPositiveInfinite(l))
                {
                    prm = 0.0;
                }
                else if (Precision.IsNegativeInfinite(f))
                {
                    prm = l - 1.0;
                }
                else
                {
                    prm = f + 1.0;
                }
            }

            gp_Pnt2d pt2d = C2d.Value(prm);
            BRepTopAdaptor_FClass2d classif = new(F, Precision.PConfusion());
            TopAbs_State stat = classif.Perform(pt2d);
            if (stat == TopAbs_OUT)
            {
                return false;
            }

            if (stat == TopAbs_ON)
            {
                int nbPnt = 10;
                int nbOut = 0, nbIn = 0, nbOn = 0;
                int j = 1;
                for (; j <= nbPnt; j++)
                {
                    // check neighbouring point
                    prm = f + (l - f) / nbPnt * (j - 1);
                    pt2d = C2d.Value(prm);
                    stat = classif.Perform(pt2d);
                    if (stat == TopAbs_OUT)
                    {
                        nbOut++;
                    }
                    else if (stat == TopAbs_IN)
                    {
                        nbIn++;
                    }
                    else
                    {
                        nbOn++;
                    }
                }
                if (nbOut > nbIn + nbOn)
                {
                    return false;
                }
            }
        }
        return true;
    }

    private static void GetDirection(in TopoDS_Edge theEdge, in TopoDS_Face theFace, out gp_Pnt2d thePnt, out gp_Vec2d theDir,
                                     bool isFirstEnd)
    {
        Geom2d_Curve aC2d = BRep_Tool.CurveOnSurface(theEdge, theFace, out double aFirst, out double aLast)!;

        TopAbs_Orientation anOr = theEdge.Orientation();
        bool takeFirst = ((anOr == TopAbs_FORWARD && isFirstEnd) || (anOr == TopAbs_REVERSED && !isFirstEnd));

        double dpar = (aLast - aFirst) * 0.01;
        if (takeFirst)
        {
            aC2d.D0(aFirst, out thePnt);
            gp_Pnt2d aNextPnt = aC2d.Value(aFirst + dpar);
            theDir = new gp_Vec2d(thePnt, aNextPnt);
        }

        else
        {
            aC2d.D0(aLast, out thePnt);
            gp_Pnt2d aPrevPnt = aC2d.Value(aLast - dpar);
            theDir = new gp_Vec2d(aPrevPnt, thePnt);
        }
        if (anOr == TopAbs_REVERSED)
        {
            theDir.Reverse();
        }
    }

    private static bool ChoixUV(in TopoDS_Edge Last,
                                in TopoDS_Face F,
                                NCollection_IndexedMap<TopoDS_Shape> Poss,
                                ref TopoDS_Edge theResEdge,
                                ref gp_Pnt2d plst,
                                ref gp_Vec2d dlst)
    {
        BRepAdaptor_Surface surf = new(F, false); // no restriction
        surf.D0(plst.X(), plst.Y(), out gp_Pnt aPlst);

        gp_Dir2d ref2d = new(dlst);

        int index = 0, imin = 0;
        double angmax = -Math.PI, ang;

        for (index = 1; index <= Poss.Extent(); index++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(Poss.FindKey(index));

            GetDirection(anEdge, F, out gp_Pnt2d p2d, out gp_Vec2d v2d, true);
            if (!SameUV(plst, p2d, surf))
            {
                continue;
            }

            surf.D0(p2d.X(), p2d.Y(), out gp_Pnt aPCur);

            if (!Last.IsSame(anEdge))
            {
                ang = ref2d.Angle(new gp_Dir2d(v2d));
            }
            else
            {
                ang = -Math.PI;
            }

            if ((ang > angmax))
            {
                imin = index;
                angmax = ang;
            }
        }

        if (imin != 0)
        {
            theResEdge = TopoDS.Edge(Poss.FindKey(imin));
            GetDirection(theResEdge, F, out plst, out dlst, false);
        }

        _ = aPlst;
        return imin != 0;
    }

    private static TopoDS_Shape ChooseDirection(in TopoDS_Shape RefDir,
                                                in TopoDS_Vertex RefVertex,
                                                in TopoDS_Face theFace,
                                                NCollection_List<TopoDS_Shape> Ldirs)
    {
        TopExp_Explorer Explo = new(RefDir, TopAbs_EDGE);
        TopoDS_Edge RefEdge = new();
        TopoDS_Vertex V1, V2;
        TopAbs_Orientation anOr = TopAbs_FORWARD;
        for (; Explo.More(); Explo.Next())
        {
            RefEdge = TopoDS.Edge(Explo.Current());
            TopExp.Vertices(RefEdge, out V1, out V2);
            if (V1.IsSame(RefVertex))
            {
                anOr = TopAbs_REVERSED;
                break;
            }
            else if (V2.IsSame(RefVertex))
            {
                anOr = TopAbs_FORWARD;
                break;
            }
        }

        Geom2d_Curve RefCurve = BRep_Tool.CurveOnSurface(RefEdge, theFace, out double RefFirst, out double RefLast)!;

        double RefPar = (anOr == TopAbs_FORWARD) ? RefLast : RefFirst;
        RefCurve.D1(RefPar, out gp_Pnt2d RefPnt, out gp_Vec2d RefVec);
        if (anOr == TopAbs_FORWARD)
        {
            RefVec.Reverse();
        }

        double aFirst, aLast, aPar;
        double MinAngle = RealLast(), anAngle;
        TopoDS_Shape TargetDir = new();
        NCollection_List<TopoDS_Shape>.Iterator itl = new(Ldirs);
        for (; itl.More(); itl.Next())
        {
            TopoDS_Shape aShape = itl.Value();
            TopoDS_Edge anEdge = new();
            for (Explo.Init(aShape, TopAbs_EDGE); Explo.More(); Explo.Next())
            {
                anEdge = TopoDS.Edge(Explo.Current());
                TopExp.Vertices(anEdge, out V1, out V2);
                if (V1.IsSame(RefVertex))
                {
                    anOr = TopAbs_FORWARD;
                    break;
                }
                else if (V2.IsSame(RefVertex))
                {
                    anOr = TopAbs_REVERSED;
                    break;
                }
            }
            Geom2d_Curve aCurve = BRep_Tool.CurveOnSurface(anEdge, theFace, out aFirst, out aLast)!;
            aPar = (anOr == TopAbs_FORWARD) ? aFirst : aLast;
            aCurve.D1(aPar, out RefPnt, out gp_Vec2d aVec);
            if (anOr == TopAbs_REVERSED)
            {
                aVec.Reverse();
            }
            anAngle = aVec.Angle(RefVec);
            if (anAngle < 0.0)
            {
                anAngle += 2.0 * Math.PI;
            }

            if (anAngle < MinAngle)
            {
                MinAngle = anAngle;
                TargetDir = aShape;
            }
        }

        return TargetDir;
    }
}
