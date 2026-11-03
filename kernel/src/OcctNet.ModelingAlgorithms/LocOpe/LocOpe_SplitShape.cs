// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_SplitShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    private Shape myShape = new();
    // OCCT iterates this map keyed by the shapes' addresses (CanSplit, which only tests); the port in insertion order
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myMap = new(ShapeHasher.Instance);
    private readonly NCollection_Map<Shape> myDblE = new(ShapeHasher.Instance);
    private NCollection_List<Shape> myLeft = new();
    private LocOpe_WorkingShapes myWork = new();
    private bool myOwnsWork = true;

    /// <summary>Empty constructor.</summary>
    public LocOpe_SplitShape()
    {
        myDone = false;
    }

    /// <summary>Creates the process with the shape <paramref name="S"/>.</summary>
    public LocOpe_SplitShape(in Shape S)
    {
        Init(S);
    }

    /// <summary>For LocOpe_Spliter: the process on its working shape, sharing its working shapes (it ends them).</summary>
    internal LocOpe_SplitShape(in Shape S, LocOpe_WorkingShapes theWork)
    {
        myWork = theWork;
        myOwnsWork = false;
        initWorking(theWork.Working(S));
    }

    /// <summary>Initializes the process on the shape <paramref name="S"/>.</summary>
    public void Init(in Shape S)
    {
        if (myOwnsWork)
        {
            myWork = new LocOpe_WorkingShapes();
        }
        initWorking(myWork.Working(S));
    }

    private void initWorking(in Shape S)
    {
        myDone = false;
        myShape = S;
        myDblE.Clear();
        myMap.Clear();
        Put(myShape);
    }

    /// <summary>Tests if it is possible to split the edge <paramref name="E"/>.</summary>
    public bool CanSplit(in Edge E) => InternalCanSplit(ShapeCasts.Edge(myWork.Copied(E)));

    private bool InternalCanSplit(in Edge E)
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
        Explorer exp = new();
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itm in myMap)
        {
            if (itm.Key.ShapeType() == ShapeType.Wire && !itm.Value.IsEmpty())
            {
                for (exp.Init(itm.Key, ShapeType.Edge); exp.More(); exp.Next())
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
    public void Add(in Vertex V, double P, in Edge E)
    {
        if (myDone)
        {
            return;
        }
        InternalAdd(ShapeCasts.Vertex(myWork.Working(V)), P, ShapeCasts.Edge(myWork.Working(E)));
    }

    // Add(V, P, E) on working shapes
    internal void InternalAdd(in Vertex V, double P, in Edge E)
    {
        if (!InternalCanSplit(E))
        {
            return;
        }

        BrepBuilder B = new();
        NCollection_List<Shape> le = myMap[E];
        if (le.IsEmpty())
        {
            le.Append(E);
        }
        NCollection_List<Shape>.Iterator itl = new(le);
        double f, l;
        NCollection_List<Shape> aNewList = new();
        for (; itl.More(); itl.Next())
        {
            Edge edg1 = ShapeCasts.Edge(itl.Value());
            Tool.Range(edg1, out f, out l);
            if (P > f + global::Kerf.Engine.Foundation.Precision.PConfusion() && P < l - global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                break;
            }
            aNewList.Append(edg1);
        }
        if (!itl.More())
        {
            return;
        }
        Edge edg = ShapeCasts.Edge(itl.Value());
        le.Remove(itl);
        if (V.Orientation() == Forward || V.Orientation() == Reversed)
        {
            edg.Orientation(Forward);
            Exploration.Vertices(edg, out Vertex aCurV1, out Vertex aCurV2);
            double aPar1 = Tool.Parameter(aCurV1, edg);

            double aPar2 = Tool.Parameter(aCurV2, edg);

            Shape aLocalShape = edg.EmptyCopied();
            Edge E1 = ShapeCasts.Edge(aLocalShape);
            aLocalShape = edg.EmptyCopied();
            Edge E2 = ShapeCasts.Edge(aLocalShape);
            E1.Orientation(Forward);
            E2.Orientation(Forward);
            Vertex newVtx = V;
            double aTolSplitV = Tool.Tolerance(V);

            aCurV1.Orientation(Forward);

            B.Add(E1, aCurV1);
            // for degenerated edges tolerance of vertices should be set to maximal value
            // from tolerance of the vertex of the edge and tolerance of splitting vertex
            double aTolV1 = (Tool.Degenerated(edg) ? Max(Tool.Tolerance(aCurV1), aTolSplitV)
                                                         : Tool.Tolerance(aCurV1));

            B.UpdateVertex(aCurV1, aPar1, E1, aTolV1);
            newVtx.Orientation(Reversed);
            B.Add(E1, newVtx);
            B.UpdateVertex(newVtx, P, E1, Tool.Tolerance(V));
            newVtx.Orientation(Forward);
            B.Add(E2, newVtx);
            B.UpdateVertex(newVtx, P, E2, Tool.Tolerance(V));

            aCurV2.Orientation(Reversed);
            B.Add(E2, aCurV2);

            // for degenerated edges tolerance of vertices should be set to maximal value
            // from tolerance of the vertex of the edge and tolerance of splitting vertex
            double aTolV2 = (Tool.Degenerated(edg) ? aTolV1 : Tool.Tolerance(aCurV2));
            B.UpdateVertex(aCurV2, aPar2, E2, aTolV2);

            aNewList.Append(E1);
            aNewList.Append(E2);
            for (; itl.More(); itl.Next())
            {
                Edge edg1 = ShapeCasts.Edge(itl.Value());
                aNewList.Append(edg1);
            }
            myMap.UnBind(E);
            myMap.Bind(E, aNewList);
        }
        else
        {
            Shape aLocalShape = edg.EmptyCopied();
            Edge E1 = ShapeCasts.Edge(aLocalShape);

            Explorer exp = new();
            for (exp.Init(edg, ShapeType.Vertex); exp.More(); exp.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(exp.Current());
                f = Tool.Parameter(vtx, edg);
                B.Add(E1, vtx);
                B.UpdateVertex(vtx, f, E1, Tool.Tolerance(vtx));
            }
            B.Add(E1, V);
            B.UpdateVertex(V, P, E1, Tool.Tolerance(V));
            le.Append(E1);
        }
    }

    /// <summary>Adds the list of wires <paramref name="Lwires"/> on the face <paramref name="F"/>.</summary>
    public bool Add(NCollection_List<Shape> Lwires, in Face F)
    {
        if (myDone)
        {
            return false;
        }
        NCollection_List<Shape> aWires = new();
        for (NCollection_List<Shape>.Iterator anIt = new(Lwires); anIt.More(); anIt.Next())
        {
            aWires.Append(myWork.Working(anIt.Value()));
        }
        return InternalAdd(aWires, ShapeCasts.Face(myWork.Working(F)));
    }

    // Add(Lwires, F) on working shapes
    internal bool InternalAdd(NCollection_List<Shape> Lwires, in Face F)
    {
        if (myDone)
        {
            return false;
        }

        NCollection_List<Shape> lf = myMap[F];
        if (lf.IsEmpty())
        {
            Rebuild(F);
        }

        // On cherche la face descendante de F qui continent le wire
        lf = myMap[F];
        NCollection_List<Shape>.Iterator itl = new(lf);

        BrepTools.Update(F);

        NCollection_List<Shape> aLInside = new();
        for (; itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());
            NCollection_List<Shape>.Iterator itwires = new(Lwires);
            for (; itwires.More(); itwires.Next())
            {
                Wire aWire = ShapeCasts.Wire(itwires.Value());
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

        Face FaceRef = ShapeCasts.Face(itl.Value());
        FaceRef.Orientation(Forward);
        lf.Remove(itl);

        NCollection_List<Shape> NewWires = new();

        NCollection_DataMap<Shape, int> SectionsTimes = new(ShapeHasher.Instance);
        for (itl.Initialize(aLInside); itl.More(); itl.Next())
        {
            SectionsTimes.Bind(itl.Value(), 2);
        }

        NCollection_List<Shape> BreakVertices = new();
        NCollection_List<Shape> BreakOnWires = new();

        // OCCT iterates this map keyed by the vertices' addresses to count a wire's entries, which isn't order dependent
        NCollection_DataMap<Shape, Shape> VerWireMap = new(ShapeHasher.Instance);
        int i;
        Explorer ExploF = new(), ExploW = new();
        for (itl.Initialize(aLInside); itl.More(); itl.Next())
        {
            Wire aSection = ShapeCasts.Wire(itl.Value());
            Vertex[] Ver = new Vertex[2];
            Exploration.Vertices(aSection, out Ver[0], out Ver[1]);
            for (i = 0; i < 2; i++)
            {
                if (VerWireMap.IsBound(Ver[i]))
                {
                    continue;
                }
                for (ExploF.Init(FaceRef, ShapeType.Wire); ExploF.More(); ExploF.Next())
                {
                    Shape aWire = ExploF.Current();
                    Shape aVer = new();
                    for (ExploW.Init(aWire, ShapeType.Vertex); ExploW.More(); ExploW.Next())
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

        NCollection_DataMap<Shape, NCollection_List<Shape>> VerSecMap = new(ShapeHasher.Instance);
        for (itl.Initialize(aLInside); itl.More(); itl.Next())
        {
            Wire aWire = ShapeCasts.Wire(itl.Value());
            Exploration.Vertices(aWire, out Vertex V1, out Vertex V2);
            NCollection_List<Shape> LW1 = new(), LW2 = new();
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

        Wire outerW = BrepTools.OuterWire(FaceRef);
        Wire CurWire = outerW;
        BRepLib_MakeWire MW;
        MW = new BRepLib_MakeWire();
        WireExplorer wexp = new(CurWire, FaceRef);
        for (;;)
        {
            Vertex theStartVertex = wexp.CurrentVertex(), CurVertex = new();
            Edge CurEdge = wexp.Current();
            Edge LastEdge = CurEdge;
            MW.Add(CurEdge);
            Wire aSectionWire = new();
            Vertex aBreakVertex;
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
                    Shape aLocalWire = ChooseDirection(LastEdge, CurVertex, FaceRef, VerSecMap[CurVertex]);
                    aSectionWire = ShapeCasts.Wire(aLocalWire);
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
                theStartVertex = ShapeCasts.Vertex(BreakVertices.First());
                BreakVertices.RemoveFirst();
                CurWire = ShapeCasts.Wire(BreakOnWires.First());
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
                    theStartVertex = ShapeCasts.Vertex(BreakVertices.First());
                    BreakVertices.RemoveFirst();
                    CurWire = ShapeCasts.Wire(BreakOnWires.First());
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
                    Exploration.Vertices(aSectionWire, out Vertex V1, out Vertex V2);
                    Vertex aStartVertex = (V1.IsSame(aBreakVertex)) ? V2 : V1;
                    CurWire = ShapeCasts.Wire(VerWireMap[aStartVertex]);

                    wexp.Init(CurWire, FaceRef);
                    while (!wexp.CurrentVertex().IsSame(aStartVertex))
                    {
                        wexp.Next();
                    }

                    NCollection_List<Shape> Lsections = VerSecMap[aStartVertex];
                    if (Lsections.Extent() == 1)
                    {
                        break;
                    }

                    // else: choose the way
                    Wire NextSectionWire =
                        ShapeCasts.Wire((aSectionWire.IsSame(Lsections.First())) ? Lsections.Last() : Lsections.First());

                    int Times = 0;
                    foreach (KeyValuePair<Shape, Shape> itVW in VerWireMap)
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
                        Edge aStartEdge = wexp.Current();
                        NCollection_List<Shape> Ldirs = new();
                        Ldirs.Append(aStartEdge);
                        Ldirs.Append(NextSectionWire);
                        Shape theDirection = ChooseDirection(aSectionWire, aStartVertex, FaceRef, Ldirs);
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

        NCollection_List<Shape> NewFaces = new();
        BrepBuilder BB = new();
        for (itl.Initialize(NewWires); itl.More(); itl.Next())
        {
            Shape aLocalFace = FaceRef.EmptyCopied();
            Face aNewFace = ShapeCasts.Face(aLocalFace);
            aNewFace.Orientation(Forward);
            BB.Add(aNewFace, itl.Value());
            NewFaces.Append(aNewFace);
        }

        // Inserting holes
        NCollection_List<Shape> Holes = new();
        for (ExploF.Init(FaceRef, ShapeType.Wire); ExploF.More(); ExploF.Next())
        {
            Shape aWire = ExploF.Current();
            ExploW.Init(aWire, ShapeType.Edge);
            Shape anEdge = ExploW.Current();
            bool found = false;
            for (itl.Initialize(NewWires); itl.More(); itl.Next())
            {
                Shape aNewWire = itl.Value();
                for (ExploW.Init(aNewWire, ShapeType.Edge); ExploW.More(); ExploW.Next())
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
        NCollection_List<Shape>.Iterator itlNewF = new();
        for (itl.Initialize(Holes); itl.More(); itl.Next())
        {
            Wire aHole = ShapeCasts.Wire(itl.Value());
            for (itlNewF.Initialize(NewFaces); itlNewF.More(); itlNewF.Next())
            {
                Face aNewFace = ShapeCasts.Face(itlNewF.Value());
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
        for (ExploF.Init(F, ShapeType.Wire); ExploF.More(); ExploF.Next())
        {
            NCollection_List<Shape> ls = myMap[ExploF.Current()];
            ls.Clear();
        }
        ///////////////////

        // JAG 10.11.95 Codage des regularites
        for (itl.Initialize(aLInside); itl.More(); itl.Next())
        {
            for (ExploW.Init(itl.Value(), ShapeType.Edge); ExploW.More(); ExploW.Next())
            {
                Edge edg = ShapeCasts.Edge(ExploW.Current());
                if (!Tool.HasContinuity(edg, F, F))
                {
                    BB.Continuity(edg, F, F, CN);
                }
            }
        }
        return true;
    }

    /// <summary>Adds the wire <paramref name="W"/> on the face <paramref name="F"/>.</summary>
    public bool Add(in Wire W, in Face F)
    {
        if (myDone)
        {
            return false;
        }
        return InternalAdd(ShapeCasts.Wire(myWork.Working(W)), ShapeCasts.Face(myWork.Working(F)));
    }

    // Add(W, F) on working shapes
    internal bool InternalAdd(in Wire W, in Face F)
    {
        if (myDone)
        {
            return false;
        }

        Explorer exp = new();
        NCollection_List<Shape> lf = myMap[F];
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
        catch (Exception e) when (e is FailureException or NullReferenceException && e is not (ImmutableObjectException or LockedShapeException))
        {
            // OCCT_DEBUG trace left out: "SpliShape internal problem detected, some faces may be lost"
            return false;
        }
        // JAG 10.11.95 Codage des regularites
        BrepBuilder B = new();
        for (exp.Init(W, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            if (!Tool.HasContinuity(edg, F, F))
            {
                B.Continuity(edg, F, F, CN);
            }
        }
        return true;
    }

    private bool AddClosedWire(in Wire W, in Face F)
    {
        Explorer exp = new();

        // On cherche la face descendante de F qui continent le wire
        NCollection_List<Shape> lf = myMap[F];
        NCollection_List<Shape>.Iterator itl = new(lf);
        for (; itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());

            if (IsInside(fac, W))
            {
                break;
            }
        }
        if (!itl.More())
        {
            return false;
        }

        BrepBuilder B = new();

        Orientation orWire = W.Orientation();
        Shape aLocalFace = F.EmptyCopied();
        Face newFace = ShapeCasts.Face(aLocalFace);
        newFace.Orientation(Forward);
        B.Add(newFace, W);
        BRepTopAdaptor_FClass2d classif = new(newFace, global::Kerf.Engine.Foundation.Precision.PConfusion());
        if (classif.PerformInfinitePoint() == In)
        {
            // le wire donne defini un trou
            aLocalFace = F.EmptyCopied();
            newFace = ShapeCasts.Face(aLocalFace);
            newFace.Orientation(Forward);
            orWire = global::Kerf.Engine.Topology.TopAbs.Reverse(orWire);
            B.Add(newFace, W.Oriented(orWire));
        }

        Face FaceRef = ShapeCasts.Face(itl.Value());
        FaceRef.Orientation(Forward);
        lf.Remove(itl);

        aLocalFace = FaceRef.EmptyCopied();
        Face newRef = ShapeCasts.Face(aLocalFace);
        newRef.Orientation(Forward);

        // On suppose que les edges du wire ont des courbes 2d.
        // Comme on ne change pas de surface de base, pas besoin d`UpdateEdge.

        for (exp.Init(FaceRef.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
        {
            Wire wir = ShapeCasts.Wire(exp.Current());
            if (IsInside(F, wir, W))
            {
                B.Add(newFace, wir);
            }
            else
            {
                B.Add(newRef, wir);
            }
        }
        B.Add(newRef, W.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(orWire)));
        lf.Append(newRef);
        lf.Append(newFace);
        return true;
    }

    private static bool checkOverlapping(in Edge theEdge1, in Edge theEdge2, in Face theFace)
    {
        FaceSurfaceAdaptor anAdS = new(theFace, false);

        double MaxTol = (Tool.Tolerance(theEdge1) + Tool.Tolerance(theEdge2));

        double aMaxTol2d = Max(anAdS.UResolution(MaxTol), anAdS.VResolution(MaxTol));
        double aTolAng = Math.PI / 180.0;
        Geom2dAPI_ProjectPointOnCurve proj = new();

        Curve2D aCrv1 = Tool.CurveOnSurface(theEdge1, theFace, out double aF1, out double aL1)!;
        Curve2D aCrv2 = Tool.CurveOnSurface(theEdge2, theFace, out double aF2, out double aL2)!;
        int j = 1, nbP = 4;
        double aStep = (aL2 - aF2) / nbP;
        for (; j < nbP; j++)
        {
            double par2 = aF2 + aStep * j;
            aCrv2.D1(par2, out Point2D aP2d, out Vector2D aV2);

            proj.Init(aP2d, aCrv1, aF1, aL1);
            // check intermediate points
            if (proj.NbPoints() == 0 || proj.LowerDistance() > aMaxTol2d)
            {
                return false;
            }
            double par1 = proj.LowerDistanceParameter();
            aCrv1.D1(par1, out Point2D aP2d1, out Vector2D aV1);

            if (!aV1.IsParallel(aV2, aTolAng))
            {
                return false;
            }
        }
        return true;
    }

    private bool AddOpenWire(in Wire W, in Face F)
    {
        // On cherche la face descendante de F qui continent le wire
        NCollection_List<Shape> lf = myMap[F];
        NCollection_List<Shape>.Iterator itl = new(lf);
        Vertex Vfirst, Vlast;

        BrepTools.Update(F);

        double tolf, toll, tol1;

        Shape aLocalShape = W.Oriented(Forward);
        Exploration.Vertices(ShapeCasts.Wire(aLocalShape), out Vfirst, out Vlast);

        tolf = Tool.Tolerance(Vfirst);
        toll = Tool.Tolerance(Vlast);
        tol1 = Max(tolf, toll);

        Explorer exp = new(), exp2 = new();

        Wire wfirst = new(), wlast = new();
        for (; itl.More(); itl.Next())
        {
            Face fac = ShapeCasts.Face(itl.Value());
            if (!IsInside(fac, W))
            {
                continue;
            }

            fac.Orientation(Forward);
            bool ffound = false;
            bool lfound = false;
            for (exp.Init(fac, ShapeType.Wire); exp.More(); exp.Next())
            {
                Wire wir = ShapeCasts.Wire(exp.Current());
                for (exp2.Init(wir, ShapeType.Vertex); exp2.More(); exp2.Next())
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

        Face FaceRef = ShapeCasts.Face(itl.Value());
        FaceRef.Orientation(Forward);
        lf.Remove(itl);
        BrepBuilder B = new();

        FaceSurfaceAdaptor BAS = new(FaceRef, false);

        bool IsPeriodic = BAS.IsUPeriodic || BAS.IsVPeriodic;

        tol1 = Max(BAS.UResolution(tol1), BAS.VResolution(tol1));

        if (wfirst.IsSame(wlast))
        {
            // on cree 2 faces en remplacement de itl.Value()
            // Essai JAG
            NCollection_List<Shape> WiresFirst = new();
            for (exp.Init(wfirst, ShapeType.Edge); exp.More(); exp.Next())
            {
                if (Tool.IsClosed(ShapeCasts.Edge(exp.Current()), FaceRef))
                {
                    myDblE.Add(exp.Current());
                }
                WiresFirst.Append(exp.Current());
            }

            Orientation orient;
            Wire newW1 = new(), newW2 = new();
            B.MakeWire(ref newW1);
            newW1.Orientation(Forward);
            B.MakeWire(ref newW2);
            newW2.Orientation(Forward);

            int nbE = 0;
            for (exp.Init(W.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                nbE++;
                Edge edg = ShapeCasts.Edge(exp.Current());
                orient = edg.Orientation();
                WiresFirst.Append(edg);
                WiresFirst.Append(edg.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(orient)));
                myDblE.Add(edg);
            }

            NCollection_IndexedMap<Shape> PossE = new(ShapeHasher.Instance);
            // OCCT's default hasher: the shapes' TShapes, locations and orientations
            NCollection_Map<Shape> MapE = new(EqualityComparer<Shape>.Default);
            Vertex vdeb, vfin;
            int nbPoss;

            // On recherche l`edge contenant Vlast
            Edge LastEdge = new();
            Point2D pfirst, plast;
            Vector2D dlast;
            Curve2D C2d;
            double f, l;

            for (exp.Init(W.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                for (exp2.Init(edg, ShapeType.Vertex); exp2.More(); exp2.Next())
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

            Shape aLocalFace = FaceRef.Oriented(wfirst.Orientation());
            C2d = Tool.CurveOnSurface(LastEdge, ShapeCasts.Face(aLocalFace), out f, out l)!;

            if (LastEdge.Orientation() == Forward)
            {
                pfirst = C2d.Value(f);
            }
            else
            {
                pfirst = C2d.Value(l);
            }

            for (exp.Init(W.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                if (nbE > 1 && edg.IsSame(LastEdge))
                {
                    continue;
                }
                for (exp2.Init(edg, ShapeType.Vertex); exp2.More(); exp2.Next())
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
            GetDirection(LastEdge, ShapeCasts.Face(aLocalFace), out plast, out dlast, false);

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
                NCollection_List<Shape>.Iterator lexp1 = new(WiresFirst);
                for (; lexp1.More(); lexp1.Next())
                {
                    Edge edg = ShapeCasts.Edge(lexp1.Value());

                    orient = edg.Orientation();
                    Exploration.Vertices(edg, out vdeb, out vfin);

                    if ((orient == Forward && Vlast.IsSame(vdeb))
                        || (orient == Reversed && Vlast.IsSame(vfin)))
                    {
                        PossE.Add(edg);
                    }
                }
                nbPoss = PossE.Extent();
                if (nbPoss == 0)
                {
                    break;
                }

                Edge aNextEdge = new();
                if (nbPoss == 1)
                {
                    aNextEdge = ShapeCasts.Edge(PossE.FindKey(1));
                    Shape aLocalFaceTemp = FaceRef.Oriented(wfirst.Orientation());
                    GetDirection(aNextEdge, ShapeCasts.Face(aLocalFaceTemp), out plast, out dlast, false);
                }
                else if (nbPoss > 1)
                {
                    // Faire choix en U,V...
                    Shape aLocalFaceTemp = FaceRef.Oriented(wfirst.Orientation());

                    if (!ChoixUV(LastEdge, ShapeCasts.Face(aLocalFaceTemp), PossE, ref aNextEdge, ref plast, ref dlast))
                    {
                        return false;
                    }
                }

                if (nbPoss >= 1)
                {
                    if (aNextEdge.IsNull)
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

                    if (LastEdge.Orientation() == Forward)
                    {
                        Vlast = Exploration.LastVertex(LastEdge);
                    }
                    else
                    {
                        Vlast = Exploration.FirstVertex(LastEdge);
                    }

                    toll = Tool.Tolerance(Vlast);
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
            NCollection_List<Shape>.Iterator lexp = new(WiresFirst);
            Shape anE1 = new(), anE2 = new();
            for (; lexp.More(); lexp.Next())
            {
                Edge edg = ShapeCasts.Edge(lexp.Value());
                if (!MapE.Contains(edg))
                {
                    B.Add(newW2, edg);
                    MapE.Add(edg);
                    nbAddBound++;
                    if (anE1.IsNull)
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
            if (nbAddBound == 2 && !anE1.IsNull && !anE2.IsNull)
            {
                if (checkOverlapping(ShapeCasts.Edge(anE1), ShapeCasts.Edge(anE2), FaceRef))
                {
                    return false;
                }
            }

            nbAddBound = 0;

            Shape anE11 = new(), anE12 = new();
            Iterator anItE = new(newW1, false);
            for (; anItE.More(); anItE.Next())
            {
                if (anItE.Value().ShapeType() != ShapeType.Edge)
                {
                    continue;
                }
                nbAddBound++;
                if (anE11.IsNull)
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
            if (nbAddBound == 2 && !anE11.IsNull && !anE12.IsNull)
            {
                if (checkOverlapping(ShapeCasts.Edge(anE11), ShapeCasts.Edge(anE12), FaceRef))
                {
                    return false;
                }
            }

            Face newF1, newF2;
            aLocalFace = FaceRef.EmptyCopied();
            newF1 = ShapeCasts.Face(aLocalFace);
            newF1.Orientation(Forward);
            aLocalFace = FaceRef.EmptyCopied();
            newF2 = ShapeCasts.Face(aLocalFace);
            newF2.Orientation(Forward);

            // modifs JAG 97.05.28

            B.Add(newF1, newW1);
            B.Add(newF2, newW2);
            for (exp.Init(FaceRef.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
            {
                Wire wir = ShapeCasts.Wire(exp.Current());
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
            for (exp.Init(F, ShapeType.Wire); exp.More(); exp.Next())
            {
                NCollection_List<Shape> ls = myMap[exp.Current()];
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
            Wire outerW = BrepTools.OuterWire(FaceRef);
            Wire newWire = new();
            Face newFace;
            B.MakeWire(ref newWire);
            newWire.Orientation(Forward);
            Orientation orient, orRelat;

            if (wfirst.Orientation() == wlast.Orientation())
            {
                orRelat = Forward;
            }
            else
            {
                orRelat = Reversed;
            }

            if (wlast.IsSame(outerW))
            {
                wlast = wfirst;
                wfirst = outerW;
            }

            // Edges de wfirst
            for (exp.Init(wfirst.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                B.Add(newWire, ShapeCasts.Edge(exp.Current()));
            }

            // Edges de wlast
            for (exp.Init(wlast.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                orient = global::Kerf.Engine.Topology.TopAbs.Compose(edg.Orientation(), orRelat);
                B.Add(newWire, edg.Oriented(orient));
            }

            // Edges du wire ajoute, et dans les 2 sens
            for (exp.Init(W.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edg = ShapeCasts.Edge(exp.Current());
                orient = edg.Orientation();
                B.Add(newWire, edg.Oriented(orient));
                B.Add(newWire, edg.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(orient)));
                myDblE.Add(edg.Oriented(orient));
            }

            // on refait une face

            Shape aLocalFace = FaceRef.EmptyCopied();
            newFace = ShapeCasts.Face(aLocalFace);
            FaceRef.Orientation(Forward);
            for (exp.Init(FaceRef.Oriented(Forward), ShapeType.Wire); exp.More(); exp.Next())
            {
                Wire wir = ShapeCasts.Wire(exp.Current());
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
            for (exp.Init(F, ShapeType.Wire); exp.More(); exp.Next())
            {
                NCollection_List<Shape> ls = myMap[exp.Current()];
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
    public NCollection_List<Shape> LeftOf(in Wire W, in Face F)
    {
        NCollection_List<Shape> aLeft = InternalLeftOf(ShapeCasts.Wire(myWork.Copied(W)), ShapeCasts.Face(myWork.Copied(F)));
        return myDone && myOwnsWork ? LocOpe_WorkingShapes.Published(aLeft) : aLeft;
    }

    // LeftOf on working shapes
    internal NCollection_List<Shape> InternalLeftOf(in Wire W, in Face F)
    {
        if (myShape.IsNull)
        {
            throw new NoSuchObjectException();
        }

        Explorer exp = new(), expw = new(), expf = new();
        exp.Init(myShape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(F))
            {
                break;
            }
        }
        if (!exp.More())
        {
            throw new NoSuchObjectException();
        }
        myLeft = new NCollection_List<Shape>(); // OCCT clears the list it returned before

        Face theFace = ShapeCasts.Face(exp.Current());
        Orientation orFace = theFace.Orientation();
        NCollection_List<Shape>.Iterator itl = new(), itl2 = new();

        for (expw.Init(W, ShapeType.Edge); expw.More(); expw.Next())
        {
            Edge edg = ShapeCasts.Edge(expw.Current());
            for (itl.Initialize(myMap[theFace]); itl.More(); itl.Next())
            {
                Face fac = ShapeCasts.Face(itl.Value());
                fac.Orientation(orFace);
                for (expf.Init(fac, ShapeType.Edge); expf.More(); expf.Next())
                {
                    Edge edgbis = ShapeCasts.Edge(expf.Current());
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
    public NCollection_List<Shape> DescendantShapes(in Shape S)
    {
        NCollection_List<Shape> aList = InternalDescendantShapes(myWork.Copied(S));
        return myOwnsWork ? LocOpe_WorkingShapes.Published(aList) : aList;
    }

    // DescendantShapes on working shapes; ends the splitting, and gives back the caller's unchanged sub-shapes when
    // the working shapes are this process's own
    internal NCollection_List<Shape> InternalDescendantShapes(in Shape S)
    {
        if (!myDone)
        {
            Rebuild(myShape);
            myDone = true;
            if (myOwnsWork && myWork.IsCopied())
            {
                var aShapes = new List<Shape>();
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> anItem in myMap)
                {
                    aShapes.AddRange(anItem.Value);
                }
                myWork.Finish(aShapes);
                foreach (Shape aKey in myMap.Keys.ToList())
                {
                    myMap[aKey] = myWork.Final(myMap[aKey]);
                }
            }
        }
        // OCCT_DEBUG trace left out ("Le shape comporte des faces invalides" when myDblE isn't empty)
        return myMap[S];
    }

    /// <summary>Returns the "original" shape (the caller's).</summary>
    public Shape Shape() => myWork.Original(myShape);

    private void Put(in Shape S)
    {
        if (!myMap.IsBound(S))
        {
            NCollection_List<Shape> thelist = new();
            myMap.Bind(S, thelist);
            if (S.ShapeType() != ShapeType.Vertex)
            {
                for (Iterator theIterator = new(S); theIterator.More(); theIterator.Next())
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

    private static void updateToleraces(in Face theFace,
                                        NCollection_DataMap<Shape, NCollection_List<Shape>> theMap)
    {
        Explorer aExpE = new(theFace, ShapeType.Edge);
        for (; aExpE.More(); aExpE.Next())
        {
            if (!theMap.IsBound(aExpE.Current()))
            {
                continue;
            }
            NCollection_List<Shape> lEdges = theMap[aExpE.Current()];
            if (lEdges.Extent() <= 1)
            {
                continue;
            }

            NCollection_List<Shape>.Iterator itrE = new(lEdges);
            ShapeAnalysis_Edge aSae = new();

            for (; itrE.More(); itrE.Next())
            {
                Edge aCurE = ShapeCasts.Edge(itrE.Value());
                double amaxdev = 0.0;
                if (aSae.CheckSameParameter(aCurE, theFace, ref amaxdev))
                {
                    BrepBuilder aB = new();
                    aB.UpdateEdge(aCurE, amaxdev);
                }
            }
        }
    }

    private bool Rebuild(in Shape S)
    {
        if (S.ShapeType() == ShapeType.Face)
        {
            updateToleraces(ShapeCasts.Face(S), myMap);
        }
        NCollection_List<Shape>.Iterator itr = new(myMap[S]);
        if (itr.More())
        {
            return !itr.Value().IsSame(S);
        }
        bool rebuild = false;
        Iterator it = new();
        for (it.Initialize(S); it.More(); it.Next())
        {
            rebuild = Rebuild(it.Value()) || rebuild;
        }

        if (rebuild)
        {
            BrepBuilder B = new();
            Shape result = S.EmptyCopied();
            Orientation orient;
            for (it.Initialize(S); it.More(); it.Next())
            {
                orient = it.Value().Orientation();
                for (itr.Initialize(myMap[it.Value()]); itr.More(); itr.Next())
                {
                    B.Add(result, itr.Value().Oriented(orient));
                }
            }
            // Assign "Closed" flag for Wires and Shells only
            if (result.ShapeType() == ShapeType.Wire || result.ShapeType() == ShapeType.Shell)
            {
                result.Closed(Tool.IsClosed(result));
            }
            myMap[S].Append(result);
        }
        else
        {
            myMap[S].Append(S);
        }
        return rebuild;
    }

    private static bool SameUV(in Point2D P1, in Point2D P2, FaceSurfaceAdaptor theBAS)
    {
        bool isSame = true;
        if (theBAS.IsUPeriodic)
        {
            isSame = (Math.Abs(P1.X() - P2.X()) < theBAS.UPeriod() * 0.5);
        }
        if (theBAS.IsVPeriodic)
        {
            isSame = (isSame && (Math.Abs(P1.Y() - P2.Y()) < theBAS.VPeriod() * 0.5));
        }
        return isSame;
    }

    private static bool IsInside(in Face F, in Wire W1, in Wire W2)
    {
        // Attention, c`est tres boeuf !!!!
        BrepBuilder B = new();
        Shape aLocalShape = F.EmptyCopied();
        Face newFace = ShapeCasts.Face(aLocalShape);

        newFace.Orientation(Forward);
        B.Add(newFace, W2);
        BRepTopAdaptor_FClass2d classif = new(newFace, global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool Reversed = false;
        if (classif.PerformInfinitePoint() == In)
        {
            // le wire donne defini un trou
            Reversed = true;
        }

        Explorer exp = new(W1, ShapeType.Edge);
        Edge edg = ShapeCasts.Edge(exp.Current());
        Explorer exp2 = new(edg, ShapeType.Vertex);
        Vertex vtx = ShapeCasts.Vertex(exp2.Current());
        double prm = Tool.Parameter(vtx, edg);
        Curve2D? C2d = Tool.CurveOnSurface(edg, F, out _, out _);
        if (C2d is null)
        {
            // OCCT_DEBUG trace left out ("Edge is not on surface")
            return false;
        }
        Point2D pt2d = C2d.Value(prm);
        if (!Reversed)
        {
            return (classif.Perform(pt2d) == In);
        }
        else
        {
            return (classif.Perform(pt2d) == Out);
        }
    }

    private static bool IsInside(in Face F, in Wire W)
    {
        // Attention, c`est tres boeuf !!!!
        Explorer exp = new(W, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge edg = ShapeCasts.Edge(exp.Current());
            double prm;
            Curve2D C2d = Tool.CurveOnSurface(edg, F, out double f, out double l)!;
            if (!global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f) && !global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
            {
                prm = (f + l) / 2.0;
            }
            else
            {
                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f) && global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(l))
                {
                    prm = 0.0;
                }
                else if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(f))
                {
                    prm = l - 1.0;
                }
                else
                {
                    prm = f + 1.0;
                }
            }

            Point2D pt2d = C2d.Value(prm);
            BRepTopAdaptor_FClass2d classif = new(F, global::Kerf.Engine.Foundation.Precision.PConfusion());
            Containment stat = classif.Perform(pt2d);
            if (stat == Out)
            {
                return false;
            }

            if (stat == On)
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
                    if (stat == Out)
                    {
                        nbOut++;
                    }
                    else if (stat == In)
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

    private static void GetDirection(in Edge theEdge, in Face theFace, out Point2D thePnt, out Vector2D theDir,
                                     bool isFirstEnd)
    {
        Curve2D aC2d = Tool.CurveOnSurface(theEdge, theFace, out double aFirst, out double aLast)!;

        Orientation anOr = theEdge.Orientation();
        bool takeFirst = ((anOr == Forward && isFirstEnd) || (anOr == Reversed && !isFirstEnd));

        double dpar = (aLast - aFirst) * 0.01;
        if (takeFirst)
        {
            aC2d.D0(aFirst, out thePnt);
            Point2D aNextPnt = aC2d.Value(aFirst + dpar);
            theDir = new Vector2D(thePnt, aNextPnt);
        }

        else
        {
            aC2d.D0(aLast, out thePnt);
            Point2D aPrevPnt = aC2d.Value(aLast - dpar);
            theDir = new Vector2D(aPrevPnt, thePnt);
        }
        if (anOr == Reversed)
        {
            theDir.Reverse();
        }
    }

    private static bool ChoixUV(in Edge Last,
                                in Face F,
                                NCollection_IndexedMap<Shape> Poss,
                                ref Edge theResEdge,
                                ref Point2D plst,
                                ref Vector2D dlst)
    {
        FaceSurfaceAdaptor surf = new(F, false); // no restriction
        surf.D0(plst.X(), plst.Y(), out Point3D aPlst);

        Direction2D ref2d = new(dlst);

        int index = 0, imin = 0;
        double angmax = -Math.PI, ang;

        for (index = 1; index <= Poss.Extent(); index++)
        {
            Edge anEdge = ShapeCasts.Edge(Poss.FindKey(index));

            GetDirection(anEdge, F, out Point2D p2d, out Vector2D v2d, true);
            if (!SameUV(plst, p2d, surf))
            {
                continue;
            }

            surf.D0(p2d.X(), p2d.Y(), out Point3D aPCur);

            if (!Last.IsSame(anEdge))
            {
                ang = ref2d.Angle(new Direction2D(v2d));
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
            theResEdge = ShapeCasts.Edge(Poss.FindKey(imin));
            GetDirection(theResEdge, F, out plst, out dlst, false);
        }

        _ = aPlst;
        return imin != 0;
    }

    private static Shape ChooseDirection(in Shape RefDir,
                                                in Vertex RefVertex,
                                                in Face theFace,
                                                NCollection_List<Shape> Ldirs)
    {
        Explorer Explo = new(RefDir, ShapeType.Edge);
        Edge RefEdge = new();
        Vertex V1, V2;
        Orientation anOr = Forward;
        for (; Explo.More(); Explo.Next())
        {
            RefEdge = ShapeCasts.Edge(Explo.Current());
            Exploration.Vertices(RefEdge, out V1, out V2);
            if (V1.IsSame(RefVertex))
            {
                anOr = Reversed;
                break;
            }
            else if (V2.IsSame(RefVertex))
            {
                anOr = Forward;
                break;
            }
        }

        Curve2D RefCurve = Tool.CurveOnSurface(RefEdge, theFace, out double RefFirst, out double RefLast)!;

        double RefPar = (anOr == Forward) ? RefLast : RefFirst;
        RefCurve.D1(RefPar, out Point2D RefPnt, out Vector2D RefVec);
        if (anOr == Forward)
        {
            RefVec.Reverse();
        }

        double aFirst, aLast, aPar;
        double MinAngle = RealLast(), anAngle;
        Shape TargetDir = new();
        NCollection_List<Shape>.Iterator itl = new(Ldirs);
        for (; itl.More(); itl.Next())
        {
            Shape aShape = itl.Value();
            Edge anEdge = new();
            for (Explo.Init(aShape, ShapeType.Edge); Explo.More(); Explo.Next())
            {
                anEdge = ShapeCasts.Edge(Explo.Current());
                Exploration.Vertices(anEdge, out V1, out V2);
                if (V1.IsSame(RefVertex))
                {
                    anOr = Forward;
                    break;
                }
                else if (V2.IsSame(RefVertex))
                {
                    anOr = Reversed;
                    break;
                }
            }
            Curve2D aCurve = Tool.CurveOnSurface(anEdge, theFace, out aFirst, out aLast)!;
            aPar = (anOr == Forward) ? aFirst : aLast;
            aCurve.D1(aPar, out RefPnt, out Vector2D aVec);
            if (anOr == Reversed)
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
