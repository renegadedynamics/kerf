// OCCT.NET: the fillets' topological reconstruction on the Boolean engine (unit 7R, option C of
// docs/phase7-plan.md 2.1, decision 992). Not a port: it answers for TopOpeBRepBuild_HBuilder, which OCCT.NET does
// not port, what ChFi3d_Builder asks of it (ChFi3d_Reconstruction), reading the same data structure.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>
/// The fillets' reconstruction: builds from the legacy data structure ChFi3d filled what OCCT's
/// <c>TopOpeBRepBuild_HBuilder</c> builds (the edges of its curves, the faces of its surfaces, the input's faces and
/// edges cut, the solids merged) with the Boolean engine's builders, and answers the same queries.
/// </summary>
/// <remarks>
/// <para>
/// <b>What the data structure says</b> (decision 1069): each curve has its points (CPIs: a vertex or a new point, the
/// parameter, a transition whose orientation for the state IN is FORWARD at the start of the kept part and REVERSED
/// at its end); each face of the input cut by a fillet has SCIs (the curve, its pcurve on the face, a transition); each
/// fillet surface has the SCIs of its boundary curves; each edge of the input a fillet cuts has CPIs at its points
/// (the states before and after them); the solid has SSIs on the fillet surfaces. The transitions' orientations for
/// the state IN orient each new edge in the faces it bounds, and each new face in the solid, as the HBuilder orients
/// them (<c>TopOpeBRepBuild_Builder::AddIntersectionEdges</c>, <c>BuildFaces</c>, <c>SplitSolid</c>).
/// </para>
/// <para>
/// <b>The steps</b> (decisions 1110 to 1115). <see cref="Perform"/>: (1) the vertices of the points and the edges of
/// the curves, each between a FORWARD and the next REVERSED point, with its pcurves on the faces and surfaces whose
/// SCIs name it (two on a seam); the edges of the input cut at their points, each part with its state; (2) the faces
/// of the surfaces, by <see cref="BOPAlgo_BuilderFace"/> from their edges. <see cref="MergeSolid"/>: (3) each face of
/// the solid with curves or cut edges is rebuilt by <see cref="BOPAlgo_BuilderFace"/> from the edges it keeps: its new
/// edges oriented by their transitions, the parts IN of its cut edges, its other edges whole (those the fillet removes
/// are left dangling and dropped), the wires the fillet does not touch classified apart, degenerated edges cut where a
/// fillet reaches them; (4) the pieces, the faces left whole and the fillet faces (oriented by the SSIs) are assembled
/// into shells by connexity, and a solid. The queries (5) answer from the maps those steps fill.
/// </para>
/// <para>
/// <b>Shapes.</b> The input is the fillet builder's working copy (decision 1087): the reconstruction writes only into
/// the shapes it makes (vertices of points, edges, parts and pieces, faces, shells, solids), and reuses the input's
/// faces and edges left whole and its vertices. Nothing is frozen here: ChFi3d's <c>Compute</c> raises the tolerances
/// of vertices, makes the new faces same-parameter and codes the regularities on the result afterwards; the builder
/// freezes once at the end.
/// </para>
/// </remarks>
internal sealed partial class ChFi3d_BOPReconstruction : ChFi3d_Reconstruction
{
    /// <summary>The build tool OCCT gives its HBuilder (kept for the record: the edges are built here).</summary>
    private readonly TopOpeBRepDS_BuildTool myBuildTool;

    private TopOpeBRepDS_HDataStructure? myHDS;
    private IntTools_Context myContext = new();
    private readonly BrepBuilder myBB = new();

    /// <summary>The vertices made for the data structure's points.</summary>
    private readonly Dictionary<int, Vertex> myPointVertices = new();

    /// <summary>What <see cref="NewEdges"/> and <see cref="NewFaces"/> give: by curve and by surface.</summary>
    private readonly Dictionary<int, NCollection_List<Shape>> myNewEdges = new();
    private readonly Dictionary<int, NCollection_List<Shape>> myNewFaces = new();

    /// <summary>The parts of each input edge cut at points, forward, with their states.</summary>
    private readonly Dictionary<Shape, List<(Edge Part, Containment State)>> myEdgeParts =
        new(ShapeHasher.Instance);

    /// <summary>The parts kept (state IN) of the edges and faces cut: <see cref="Splits"/>.</summary>
    private readonly Dictionary<Shape, NCollection_List<Shape>> mySplitIN =
        new(ShapeHasher.Instance);

    /// <summary>What <see cref="MergeSolid"/> made of each solid or shell: <see cref="Merged"/>.</summary>
    private readonly Dictionary<Shape, NCollection_List<Shape>> myMergedIN =
        new(ShapeHasher.Instance);

    /// <summary>The SCIs of each curve, on faces (support kind SURFACE, a shape index) and surfaces.</summary>
    private readonly Dictionary<int, List<CurveSupport>> myCurveSupports = new();

    private static readonly NCollection_List<Shape> theEmptyList = new();

    /// <summary>A curve's SCI: on a face of the input (<see cref="Face"/> not null) or on a fillet surface.</summary>
    private readonly record struct CurveSupport(Face Face, int Surface, Curve2D PCurve,
                                                Orientation Orientation);

    /// <summary>A reconstruction for the fillet builder (<see cref="ChFi3d_Reconstruction.Create"/>).</summary>
    internal ChFi3d_BOPReconstruction(TopOpeBRepDS_BuildTool BT) => myBuildTool = BT;

    //=================================================================================================
    // ChFi3d_Reconstruction

    /// <inheritdoc/>
    public override void Perform(TopOpeBRepDS_HDataStructure HDS)
    {
        Clear();
        myHDS = HDS;
        CollectCurveSupports();
        BindPointsAtEdgeEnds();
        BuildEdges();
        SplitEdges();
        BuildFaces();
    }

    /// <inheritdoc/>
    public override void MergeSolid(in Shape S, Containment TB)
    {
        if (TB != In || myHDS is null || myMergedIN.ContainsKey(S))
        {
            return;
        }
        NCollection_List<Shape> aMerged = new();
        MergeShape(S, aMerged);
        myMergedIN[S] = aMerged;
    }

    /// <inheritdoc/>
    public override bool IsSplit(in Shape S, Containment ToBuild) =>
        ToBuild == In && mySplitIN.TryGetValue(S, out NCollection_List<Shape>? L) && !L.IsEmpty();

    /// <inheritdoc/>
    public override NCollection_List<Shape> Splits(in Shape S, Containment ToBuild) =>
        ToBuild == In && mySplitIN.TryGetValue(S, out NCollection_List<Shape>? L) ? L : theEmptyList;

    /// <inheritdoc/>
    public override NCollection_List<Shape> Merged(in Shape S, Containment ToBuild) =>
        ToBuild == In && myMergedIN.TryGetValue(S, out NCollection_List<Shape>? L) ? L : theEmptyList;

    /// <inheritdoc/>
    public override NCollection_List<Shape> NewFaces(int I) =>
        myNewFaces.TryGetValue(I, out NCollection_List<Shape>? L) ? L : theEmptyList;

    /// <inheritdoc/>
    public override NCollection_List<Shape> NewEdges(int I) =>
        myNewEdges.TryGetValue(I, out NCollection_List<Shape>? L) ? L : theEmptyList;

    /// <inheritdoc/>
    public override TopOpeBRepDS_HDataStructure? DataStructure() => myHDS;

    public override string ToString() => "ChFi3d_BOPReconstruction, " + myBuildTool;

    //=================================================================================================
    // Perform

    private TopOpeBRepDS_DataStructure DS => myHDS!.DS();

    private void Clear()
    {
        myHDS = null;
        myContext = new IntTools_Context();
        myPointVertices.Clear();
        myNewEdges.Clear();
        myNewFaces.Clear();
        myEdgeParts.Clear();
        mySplitIN.Clear();
        myMergedIN.Clear();
        myCurveSupports.Clear();
    }

    /// <summary>The SCIs naming each curve: on the input's faces (shape interferences) and on the surfaces.</summary>
    private void CollectCurveSupports()
    {
        TopOpeBRepDS_DataStructure aDS = DS;
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            Shape S = aDS.Shape(i);
            if (S.IsNull || S.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            Face F = ShapeCasts.Face(S);
            foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(i))
            {
                if (I is TopOpeBRepDS_SurfaceCurveInterference SCI && SCI.GeometryType() == TopOpeBRepDS_CURVE
                    && SCI.PCurve() is Curve2D PC)
                {
                    Orientation O = SCI.Transition().Orientation(In);
                    AddCurveSupport(SCI.Geometry(), new CurveSupport(F, 0, PC, O));
                }
            }
        }
        for (int iS = 1; iS <= aDS.NbSurfaces(); iS++)
        {
            foreach (TopOpeBRepDS_Interference I in aDS.SurfaceInterferences(iS))
            {
                if (I is TopOpeBRepDS_SurfaceCurveInterference SCI && SCI.GeometryType() == TopOpeBRepDS_CURVE
                    && SCI.PCurve() is Curve2D PC)
                {
                    Orientation O = SCI.Transition().Orientation(In);
                    AddCurveSupport(SCI.Geometry(), new CurveSupport(new Face(), iS, PC, O));
                }
            }
        }
    }

    private void AddCurveSupport(int theCurve, CurveSupport theSupport)
    {
        if (!myCurveSupports.TryGetValue(theCurve, out List<CurveSupport>? L))
        {
            myCurveSupports[theCurve] = L = [];
        }
        L.Add(theSupport);
    }

    /// <summary>The vertex of a point of the data structure: the input's vertex, or one made for the point.</summary>
    private Vertex VertexOf(TopOpeBRepDS_Kind theKind, int theIndex)
    {
        if (theKind == TopOpeBRepDS_VERTEX)
        {
            return ShapeCasts.Vertex(DS.Shape(theIndex));
        }
        if (!myPointVertices.TryGetValue(theIndex, out Vertex V))
        {
            TopOpeBRepDS_Point P = DS.Point(theIndex);
            V = new Vertex();
            myBB.MakeVertex(ref V, P.Point(), Math.Max(P.Tolerance(), global::Kerf.Engine.Foundation.Precision.Confusion()));
            myPointVertices[theIndex] = V;
        }
        return V;
    }

    /// <summary>Step 1: the edges of the curves, between their points.</summary>
    private void BuildEdges()
    {
        TopOpeBRepDS_DataStructure aDS = DS;
        for (int iC = 1; iC <= aDS.NbCurves(); iC++)
        {
            List<(double T, Vertex V, Orientation O)> aPaves = [];
            foreach (TopOpeBRepDS_Interference I in aDS.CurveInterferences(iC))
            {
                if (I is TopOpeBRepDS_CurvePointInterference CPI
                    && (CPI.GeometryType() == TopOpeBRepDS_POINT || CPI.GeometryType() == TopOpeBRepDS_VERTEX))
                {
                    aPaves.Add((CPI.Parameter(), VertexOf(CPI.GeometryType(), CPI.Geometry()),
                                CPI.Transition().Orientation(In)));
                }
            }
            if (aPaves.Count < 2)
            {
                continue; // a curve without points makes no edge (OCCT's BuildEdges: no pave)
            }
            aPaves.Sort((a, b) => a.T.CompareTo(b.T));
            NCollection_List<Shape> aLE = new();
            for (int k = 0; k + 1 < aPaves.Count; k++)
            {
                if (aPaves[k].O == Reversed || aPaves[k + 1].O == Forward)
                {
                    continue;
                }
                if (aPaves[k + 1].T - aPaves[k].T <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    continue; // two points at one parameter: no edge, as the HBuilder's pave set (decision 1159)
                }
                Edge E = MakeCurveEdge(iC, aPaves[k].V, aPaves[k].T, aPaves[k + 1].V, aPaves[k + 1].T);
                aLE.Append(E);
            }
            if (!aLE.IsEmpty())
            {
                myNewEdges[iC] = aLE;
            }
        }
    }

    /// <summary>An edge on a curve between two of its points, with the pcurves of its SCIs.</summary>
    private Edge MakeCurveEdge(int iC, Vertex V1, double T1, Vertex V2, double T2)
    {
        TopOpeBRepDS_Curve C = DS.Curve(iC);
        double aTol = Math.Max(C.Tolerance(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Edge E = new();
        Curve3D? C3d = C.Curve();
        if (C3d is null)
        {
            myBB.MakeEdge(ref E);
            myBB.UpdateEdge(E, aTol);
            myBB.Degenerated(E, true);
        }
        else
        {
            myBB.MakeEdge(ref E, C3d, aTol);
        }
        myBB.Add(E, V1.Oriented(Forward));
        myBB.Add(E, V2.Oriented(Reversed));

        if (myCurveSupports.TryGetValue(iC, out List<CurveSupport>? aSupports))
        {
            // the pcurves, by face or surface: two on the same one make a seam (the FORWARD one first)
            HashSet<int> aDone = [];
            for (int k = 0; k < aSupports.Count; k++)
            {
                if (aDone.Contains(k))
                {
                    continue;
                }
                CurveSupport s = aSupports[k];
                int aTwin = -1;
                for (int j = k + 1; j < aSupports.Count && aTwin < 0; j++)
                {
                    if (SameSupport(s, aSupports[j]))
                    {
                        aTwin = j;
                    }
                }
                GetSupport(s, out Surface aSurf, out Placement aLoc, out double aSurfTol);
                aTol = Math.Max(aTol, aSurfTol);
                if (aTwin < 0)
                {
                    myBB.UpdateEdge(E, s.PCurve, aSurf, aLoc, aTol);
                }
                else
                {
                    aDone.Add(aTwin);
                    CurveSupport t = aSupports[aTwin];
                    (Curve2D aF, Curve2D aR) =
                        s.Orientation == Reversed ? (t.PCurve, s.PCurve) : (s.PCurve, t.PCurve);
                    myBB.UpdateEdge(E, aF, aR, aSurf, aLoc, aTol);
                }
            }
        }
        myBB.Range(E, T1, T2);
        myBB.UpdateEdge(E, aTol);
        return E;
    }

    private static bool SameSupport(in CurveSupport A, in CurveSupport B) =>
        A.Face.IsNull ? B.Face.IsNull && A.Surface == B.Surface : !B.Face.IsNull && A.Face.IsSame(B.Face);

    private void GetSupport(in CurveSupport theS, out Surface theSurf, out Placement theLoc,
                            out double theTol)
    {
        if (!theS.Face.IsNull)
        {
            theSurf = Tool.Surface(theS.Face, out theLoc)!;
            theTol = 0.0;
        }
        else
        {
            TopOpeBRepDS_Surface S = DS.Surface(theS.Surface);
            theSurf = S.Surface()!;
            theLoc = new Placement();
            theTol = S.Tolerance(); // OCCT's BuildFaces raises the edges to the surface's tolerance
        }
    }

    /// <summary>Step 1: the input's edges cut at their points, each part with its state.</summary>
    private void SplitEdges()
    {
        TopOpeBRepDS_DataStructure aDS = DS;
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            Shape S = aDS.Shape(i);
            if (S.IsNull || S.ShapeType() != ShapeType.Edge)
            {
                continue;
            }
            Edge E = ShapeCasts.Edge(S.Oriented(Forward));
            if (Tool.Degenerated(E))
            {
                continue;
            }
            List<(double T, Vertex V, Containment Before, Containment After)> aCuts = [];
            bool aOnlyPointsAtEnds = true;
            foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(i))
            {
                if (I is TopOpeBRepDS_CurvePointInterference CPI
                    && (CPI.GeometryType() == TopOpeBRepDS_POINT || CPI.GeometryType() == TopOpeBRepDS_VERTEX))
                {
                    TopOpeBRepDS_Transition T = CPI.Transition();
                    aCuts.Add((CPI.Parameter(), VertexOf(CPI.GeometryType(), CPI.Geometry()), T.Before(), T.After()));
                    aOnlyPointsAtEnds &= CPI.GeometryType() == TopOpeBRepDS_POINT && AtEnd(E, CPI.Parameter());
                }
            }
            if (aCuts.Count == 0 || aOnlyPointsAtEnds)
            {
                // points only where the edge's vertices are (a fillet ending at a vertex the data structure gives as a
                // point): they cut nothing, and the edge is left whole (decision 1157)
                continue;
            }
            SplitEdge(E, aCuts);
        }
    }

    private static bool AtEnd(Edge E, double T)
    {
        Tool.Range(E, out double f, out double l);
        return Math.Abs(T - f) <= global::Kerf.Engine.Foundation.Precision.PConfusion() || Math.Abs(T - l) <= global::Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>
    /// The points the data structure puts on an edge at one of its ends are its vertex there: the curves ending at
    /// them then end on the edge's vertex, as natively (decision 1157).
    /// </summary>
    private void BindPointsAtEdgeEnds()
    {
        TopOpeBRepDS_DataStructure aDS = DS;
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            Shape S = aDS.Shape(i);
            if (S.IsNull || S.ShapeType() != ShapeType.Edge)
            {
                continue;
            }
            Edge E = ShapeCasts.Edge(S.Oriented(Forward));
            if (Tool.Degenerated(E))
            {
                continue;
            }
            Tool.Range(E, out double f, out double l);
            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(i))
            {
                if (I is not TopOpeBRepDS_CurvePointInterference CPI || CPI.GeometryType() != TopOpeBRepDS_POINT
                    || myPointVertices.ContainsKey(CPI.Geometry()))
                {
                    continue;
                }
                Vertex V = Math.Abs(CPI.Parameter() - f) <= global::Kerf.Engine.Foundation.Precision.PConfusion() ? V1
                                  : Math.Abs(CPI.Parameter() - l) <= global::Kerf.Engine.Foundation.Precision.PConfusion() ? V2 : new Vertex();
                if (!V.IsNull)
                {
                    myPointVertices[CPI.Geometry()] = V;
                }
            }
        }
    }

    private void SplitEdge(Edge E,
                           List<(double T, Vertex V, Containment Before, Containment After)> theCuts)
    {
        Exploration.Vertices(E, out Vertex aV1, out Vertex aV2);
        Tool.Range(E, out double f, out double l);
        theCuts.Sort((a, b) => a.T.CompareTo(b.T));
        double aTolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();

        // a point beyond an end, where the part IN runs to it: the fillet's section overflows the edge, and the part
        // kept is extended to the point along the edge's curve, as the HBuilder's pave set does (decision 1160)
        (double T, Vertex V) aStart = (f, aV1), anEnd = (l, aV2);
        Curve3D? aC3d = Tool.Curve(E, out double _, out double _);
        foreach ((double T, Vertex V, Containment Before, Containment After) c in theCuts)
        {
            if (aC3d is null)
            {
                break;
            }
            if (c.T > l + aTolPar && c.Before == In && c.T <= aC3d.LastParameter())
            {
                anEnd = (c.T, c.V);
            }
            else if (c.T < f - aTolPar && c.After == In && c.T >= aC3d.FirstParameter() && aStart.T == f)
            {
                aStart = (c.T, c.V);
            }
        }
        bool anExtended = aStart.T != f || anEnd.T != l;

        // the paves: the edge's ends and its points inside, each with the state of the part that follows
        List<(double T, Vertex V)> aPaves = [aStart];
        List<Containment> aStates = [];
        Containment aFirst = theCuts[0].Before;
        aStates.Add(aFirst);
        foreach ((double T, Vertex V, Containment Before, Containment After) c in theCuts)
        {
            if (c.T <= f + aTolPar)
            {
                aStates[0] = c.After; // a point at the start only gives the state of the first part
                continue;
            }
            if (c.T >= l - aTolPar)
            {
                continue;
            }
            if (c.T - aPaves[^1].T <= aTolPar)
            {
                aStates[^1] = c.After;
                continue;
            }
            aPaves.Add((c.T, c.V));
            aStates.Add(c.After);
        }
        aPaves.Add(anEnd);
        if (aPaves.Count == 2 && !anExtended)
        {
            // not cut: one part, kept when its state is IN
            List<(Edge, Containment)> aWhole = [(E, aStates[0])];
            myEdgeParts[E] = aWhole;
            NCollection_List<Shape> aKeep = new();
            if (aStates[0] != Out)
            {
                aKeep.Append(E);
            }
            mySplitIN[E] = aKeep;
            return;
        }
        List<(Edge Part, Containment State)> aParts = [];
        NCollection_List<Shape> aIN = new();
        for (int k = 0; k + 1 < aPaves.Count; k++)
        {
            BOPTools_AlgoTools.MakeSplitEdge(E, aPaves[k].V, aPaves[k].T, aPaves[k + 1].V, aPaves[k + 1].T,
                                             out Edge aPart);
            aParts.Add((aPart, aStates[k]));
            if (aStates[k] != Out)
            {
                aIN.Append(aPart);
            }
        }
        myEdgeParts[E] = aParts;
        mySplitIN[E] = aIN;
    }

    /// <summary>Step 2: the faces of the surfaces, from their edges oriented by the SCIs.</summary>
    private void BuildFaces()
    {
        TopOpeBRepDS_DataStructure aDS = DS;
        for (int iS = 1; iS <= aDS.NbSurfaces(); iS++)
        {
            TopOpeBRepDS_Surface S = aDS.Surface(iS);
            if (S.Surface() is not Surface aSurf)
            {
                continue;
            }
            NCollection_List<Shape> aLE = new();
            foreach (TopOpeBRepDS_Interference I in aDS.SurfaceInterferences(iS))
            {
                if (I is not TopOpeBRepDS_SurfaceCurveInterference SCI || SCI.GeometryType() != TopOpeBRepDS_CURVE)
                {
                    continue;
                }
                Orientation O = SCI.Transition().Orientation(In);
                if (O != Forward && O != Reversed)
                {
                    continue;
                }
                foreach (Shape E in NewEdges(SCI.Geometry()))
                {
                    aLE.Append(E.Oriented(O));
                }
            }
            if (aLE.IsEmpty())
            {
                continue;
            }
            Face F = new();
            myBB.MakeFace(ref F, aSurf, Math.Max(S.Tolerance(), global::Kerf.Engine.Foundation.Precision.Confusion()));
            NCollection_List<Shape> aFaces = BuildAreas(F, aLE, out NCollection_List<Shape> aLoops);
            if (aFaces.IsEmpty())
            {
                // the SCIs' edges close a loop the face builder takes for a hole (a fillet surface whose normal runs
                // against its transitions): the HBuilder makes the face on it all the same (decision 1158)
                foreach (Shape W in aLoops)
                {
                    Face FW = ShapeCasts.Face(F.EmptyCopied());
                    myBB.Add(FW, W);
                    aFaces.Append(FW);
                }
            }
            if (!aFaces.IsEmpty())
            {
                myNewFaces[iS] = aFaces;
            }
        }
    }

    /// <summary>The faces <see cref="BOPAlgo_BuilderFace"/> builds on a forward face from edges.</summary>
    private NCollection_List<Shape> BuildAreas(Face theFace, NCollection_List<Shape> theEdges) =>
        BuildAreas(theFace, theEdges, out _);

    private NCollection_List<Shape> BuildAreas(Face theFace, NCollection_List<Shape> theEdges,
                                                      out NCollection_List<Shape> theLoops)
    {
        BOPAlgo_BuilderFace aBF = new();
        aBF.SetFace(theFace);
        aBF.SetShapes(theEdges);
        aBF.SetContext(myContext);
        aBF.SetRunParallel(false);
        aBF.SetAvoidInternalShapes(true); // the edges the fillet leaves dangling are dropped
        aBF.Perform();
        theLoops = new NCollection_List<Shape>(aBF.Loops());
        return new NCollection_List<Shape>(aBF.Areas());
    }

    //=================================================================================================
    // MergeSolid

    /// <summary>Steps 3 and 4: the solid's (or shell's) faces split, fillet faces added, shells assembled.</summary>
    private void MergeShape(Shape S, NCollection_List<Shape> theMerged)
    {
        TopOpeBRepDS_DataStructure aDS = DS;
        NCollection_List<Shape> aFaces = new();
        List<Shape> aWhole = [];
        // the shell of the input each face kept comes from, for the shells' history
        Dictionary<Shape, Shape> anOrigin = new(ShapeHasher.Instance);
        bool aChanged = false; // a solid the fillets do not touch is not split (natively it is then "deleted")
        for (Explorer exs = new(S, ShapeType.Shell); exs.More(); exs.Next())
        {
            for (Explorer exp = new(exs.Current(), ShapeType.Face); exp.More(); exp.Next())
            {
                Face F = ShapeCasts.Face(exp.Current());
                NCollection_List<Shape>? aPieces = SplitFace(F);
                if (aPieces is null)
                {
                    aWhole.Add(F);
                    anOrigin[F] = exs.Current();
                    continue;
                }
                aChanged = true;
                foreach (Shape P in aPieces)
                {
                    aFaces.Append(P.Oriented(F.Orientation()));
                    anOrigin[P] = exs.Current();
                }
            }
        }
        foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(S))
        {
            if (I is not TopOpeBRepDS_SolidSurfaceInterference || I.GeometryType() != TopOpeBRepDS_SURFACE)
            {
                continue;
            }
            Orientation O = I.Transition().Orientation(In);
            if (O != Forward && O != Reversed)
            {
                continue;
            }
            foreach (Shape F in NewFaces(I.Geometry()))
            {
                aFaces.Append(F.Oriented(O));
                aChanged = true;
            }
        }
        foreach (Shape F in KeptWholeFaces(S, aWhole, aFaces))
        {
            aFaces.Append(F);
        }

        // the shells: the faces connected by their edges (a solid's outer shell and its voids' apart)
        List<Shell> aShells = [];
        foreach (List<Shape> aBlock in ConnectedBlocks(aFaces))
        {
            Shell aShell = new();
            myBB.MakeShell(ref aShell);
            foreach (Shape F in aBlock)
            {
                myBB.Add(aShell, F);
            }
            aShell.Closed(Tool.IsClosed(aShell));
            aShells.Add(aShell);
            // the input's shells this one comes from (decision 1161: the HBuilder splits solids and shells too)
            HashSet<Shape> aFrom = new(ShapeHasher.Instance);
            foreach (Shape F in aBlock)
            {
                if (aChanged && anOrigin.TryGetValue(F, out Shape anOld) && aFrom.Add(anOld))
                {
                    if (!mySplitIN.TryGetValue(anOld, out NCollection_List<Shape>? L))
                    {
                        mySplitIN[anOld] = L = new NCollection_List<Shape>();
                    }
                    L.Append(aShell);
                }
            }
        }
        if (S.ShapeType() == ShapeType.Shell)
        {
            foreach (Shell aShell in aShells)
            {
                theMerged.Append(aShell);
            }
            return;
        }
        Solid aSolid = new();
        myBB.MakeSolid(ref aSolid);
        foreach (Shell aShell in aShells)
        {
            myBB.Add(aSolid, aShell);
        }
        theMerged.Append(aSolid);
        // BRepFilletAPI's Modified and IsDeleted read the solid's splits: native's give the solid made of it
        if (aChanged)
        {
            NCollection_List<Shape> aSolidSplits = new();
            aSolidSplits.Append(aSolid);
            mySplitIN[S] = aSolidSplits;
        }
    }

    /// <summary>
    /// The faces left whole that stay: a face is dropped when one of its edges has lost a face it bounded (the
    /// pieces around dropped it: the face lies where a fillet swallowed it), until none is (decision 1114).
    /// </summary>
    private static List<Shape> KeptWholeFaces(Shape S, List<Shape> theWhole,
                                                     NCollection_List<Shape> theOthers)
    {
        Dictionary<Shape, int> aBefore = EdgeUses(new[] { S });
        List<Shape> aKept = [.. theWhole];
        bool aChanged = true;
        while (aChanged && aKept.Count > 0)
        {
            aChanged = false;
            Dictionary<Shape, int> aNow = EdgeUses(aKept.Concat(theOthers));
            for (int i = aKept.Count - 1; i >= 0; i--)
            {
                for (Explorer exp = new(aKept[i], ShapeType.Edge); exp.More(); exp.Next())
                {
                    Shape E = exp.Current();
                    if (!Tool.Degenerated(ShapeCasts.Edge(E))
                        && aNow.GetValueOrDefault(E) < aBefore.GetValueOrDefault(E))
                    {
                        aKept.RemoveAt(i);
                        aChanged = true;
                        break;
                    }
                }
            }
        }
        return aKept;
    }

    /// <summary>How many faces of the shapes each edge bounds (a seam counts once).</summary>
    private static Dictionary<Shape, int> EdgeUses(IEnumerable<Shape> theShapes)
    {
        Dictionary<Shape, int> aUses = new(ShapeHasher.Instance);
        foreach (Shape S in theShapes)
        {
            for (Explorer expf = new(S, ShapeType.Face); expf.More(); expf.Next())
            {
                HashSet<Shape> aSeen = new(ShapeHasher.Instance);
                for (Explorer exp = new(expf.Current(), ShapeType.Edge); exp.More(); exp.Next())
                {
                    if (aSeen.Add(exp.Current()))
                    {
                        aUses[exp.Current()] = aUses.GetValueOrDefault(exp.Current()) + 1;
                    }
                }
            }
        }
        return aUses;
    }

    /// <summary>The faces in blocks connected by shared edges, in the faces' order.</summary>
    private static List<List<Shape>> ConnectedBlocks(NCollection_List<Shape> theFaces)
    {
        List<Shape> aFaces = [.. theFaces];
        int[] aParent = Enumerable.Range(0, aFaces.Count).ToArray();
        int Root(int i)
        {
            while (aParent[i] != i)
            {
                i = aParent[i] = aParent[aParent[i]];
            }
            return i;
        }
        Dictionary<Shape, int> aFirstFace = new(ShapeHasher.Instance);
        for (int i = 0; i < aFaces.Count; i++)
        {
            for (Explorer exp = new(aFaces[i], ShapeType.Edge); exp.More(); exp.Next())
            {
                if (Tool.Degenerated(ShapeCasts.Edge(exp.Current())))
                {
                    continue;
                }
                if (aFirstFace.TryGetValue(exp.Current(), out int j))
                {
                    aParent[Root(i)] = Root(j);
                }
                else
                {
                    aFirstFace[exp.Current()] = i;
                }
            }
        }
        Dictionary<int, List<Shape>> aBlocks = new();
        List<List<Shape>> aOrder = [];
        for (int i = 0; i < aFaces.Count; i++)
        {
            int r = Root(i);
            if (!aBlocks.TryGetValue(r, out List<Shape>? B))
            {
                aBlocks[r] = B = [];
                aOrder.Add(B);
            }
            B.Add(aFaces[i]);
        }
        return aOrder;
    }

    /// <summary>
    /// Step 3: the pieces of a face with curves or cut edges (forward), or null if it has neither; recorded as the
    /// face's <see cref="Splits"/>.
    /// </summary>
    /// <remarks>
    /// As the HBuilder's face builder, the pieces are built from the kept edges only: the face's new edges oriented by
    /// their transitions, the parts of its cut edges in the state IN, and its edges not cut. Edges not cut that the
    /// fillet removes (the edge filleted, those of a face it swallows) are left dangling by the parts dropped, and
    /// <see cref="BOPAlgo_BuilderFace"/> drops them. A wire of the face that touches neither (no edge cut, no vertex
    /// on a kept edge) is classified (decision 1112): an inner wire is kept if it lies inside a piece built without it
    /// (a hole far from the fillet), dropped otherwise (the rim of a hole filleted); the outer wire is kept when the
    /// fillet's loops are only holes (a hole's rim filleted, a concave fillet around a window), classified as the inner
    /// ones otherwise (a face whose every edge is filleted). A concave fillet's new edges lie outside the face: the
    /// face grows to them, as natively.
    /// </remarks>
    private NCollection_List<Shape>? SplitFace(Face F)
    {
        if (mySplitIN.TryGetValue(F, out NCollection_List<Shape>? aDone))
        {
            return aDone;
        }
        TopOpeBRepDS_DataStructure aDS = DS;
        Face FF = ShapeCasts.Face(F.Oriented(Forward));

        // the face's new edges, oriented by their transitions (both ways on a seam of the face)
        NCollection_List<Shape> aLE = new();
        NCollection_Map<Shape> aKeptVertices = new(ShapeHasher.Instance);
        HashSet<(TShape, Orientation)> aAdded = [];
        if (aDS.HasShape(F))
        {
            foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(F))
            {
                if (I is not TopOpeBRepDS_SurfaceCurveInterference SCI || SCI.GeometryType() != TopOpeBRepDS_CURVE)
                {
                    continue;
                }
                Orientation O = SCI.Transition().Orientation(In);
                if (O != Forward && O != Reversed)
                {
                    continue;
                }
                foreach (Shape E in NewEdges(SCI.Geometry()))
                {
                    if (aAdded.Add((E.TShape()!, O)))
                    {
                        aLE.Append(E.Oriented(O));
                        AddVertices(E, aKeptVertices);
                    }
                }
            }
        }
        bool aHasCurves = !aLE.IsEmpty();

        // the face's edges: the kept parts of the cut ones, the others whole
        bool aHasCuts = false;
        NCollection_List<Shape> aWhole = new();
        for (Explorer exp = new(FF, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());
            if (myEdgeParts.TryGetValue(E, out List<(Edge Part, Containment State)>? aParts))
            {
                aHasCuts = true;
                foreach ((Edge aPart, Containment aState) in aParts)
                {
                    if (aState != Out)
                    {
                        aLE.Append(aPart.Oriented(E.Orientation()));
                        AddVertices(aPart, aKeptVertices);
                    }
                }
            }
            else if (Tool.Degenerated(E) && SplitDegenerated(FF, E, aLE) is List<Edge> aDegParts)
            {
                foreach (Edge aPart in aDegParts)
                {
                    aLE.Append(aPart);
                }
            }
            else
            {
                aWhole.Append(E);
            }
        }
        if (!aHasCurves && !aHasCuts)
        {
            return null;
        }

        // the wires the fillet does not touch are classified afterwards
        Wire anOuter = BrepTools.OuterWire(FF);
        List<NCollection_List<Shape>> aIsolated = [];
        NCollection_List<Shape>? anOuterIsolated = null;
        NCollection_Map<Shape> aIsolatedEdges = new(ShapeHasher.Instance);
        for (Iterator itw = new(FF); itw.More(); itw.Next())
        {
            Shape W = itw.Value();
            if (W.ShapeType() != ShapeType.Wire)
            {
                continue;
            }
            bool aTouched = false;
            NCollection_List<Shape> aWE = new();
            for (Explorer exp = new(W, ShapeType.Edge); exp.More() && !aTouched; exp.Next())
            {
                Shape E = exp.Current();
                aTouched = myEdgeParts.ContainsKey(E);
                for (Explorer exv = new(E, ShapeType.Vertex); exv.More() && !aTouched; exv.Next())
                {
                    aTouched = aKeptVertices.Contains(exv.Current());
                }
                aWE.Append(E);
            }
            if (aTouched)
            {
                continue;
            }
            foreach (Shape E in aWE)
            {
                aIsolatedEdges.Add(E);
            }
            if (W.IsSame(anOuter))
            {
                anOuterIsolated = aWE;
            }
            else
            {
                aIsolated.Add(aWE);
            }
        }
        foreach (Shape E in WholeEdgesKept(aWhole, aIsolatedEdges, aKeptVertices))
        {
            aLE.Append(E);
        }

        NCollection_List<Shape> aPieces = BuildAreas(FF, aLE);
        if (anOuterIsolated is not null)
        {
            // the outer wire, not touched: it bounds what the face keeps when the fillet's loops are holes (a hole's
            // rim filleted, a concave fillet around a hole), and is dropped when a loop the fillet made replaces it
            if (aPieces.IsEmpty())
            {
                foreach (Shape E in anOuterIsolated)
                {
                    aLE.Append(E);
                }
                aPieces = BuildAreas(FF, aLE);
            }
            else
            {
                aIsolated.Add(anOuterIsolated);
            }
        }
        if (aIsolated.Count > 0)
        {
            // an inner wire the fillet does not touch is kept if it lies in a piece (a hole of it), dropped if it
            // lies where the fillet removed the face (the rim of a hole filleted)
            bool aAny = false;
            foreach (NCollection_List<Shape> aWE in aIsolated)
            {
                if (IsInside(aWE, aPieces))
                {
                    aAny = true;
                    foreach (Shape E in aWE)
                    {
                        aLE.Append(E);
                    }
                }
            }
            if (aAny)
            {
                aPieces = BuildAreas(FF, aLE);
            }
        }
        mySplitIN[F] = aPieces;
        return aPieces;
    }

    /// <summary>
    /// The parts of a degenerated edge of the face (forward) cut where new edges reach its vertex in the face's
    /// parametric space (a fillet ending at a cone's apex), oriented as the edge; null if none reaches it inside.
    /// </summary>
    /// <remarks>
    /// All the parts are given to <see cref="BOPAlgo_BuilderFace"/>: its wire splitter follows the parametric space
    /// at the degenerated vertex, so the part under the fillet is left over and dropped (decision 1113).
    /// </remarks>
    private static List<Edge>? SplitDegenerated(Face FF, Edge E,
                                                       NCollection_List<Shape> theNew)
    {
        Vertex V = Exploration.FirstVertex(ShapeCasts.Edge(E.Oriented(Forward)));
        Curve2D? aDC = Tool.CurveOnSurface(E, FF, out double f, out double l);
        if (V.IsNull || aDC is null)
        {
            return null;
        }
        double aTolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();
        List<double> aParams = [];
        foreach (Shape S in theNew)
        {
            Edge NE = ShapeCasts.Edge(S.Oriented(Forward));
            Exploration.Vertices(NE, out Vertex V1, out Vertex V2);
            bool a1 = V1.IsSame(V), a2 = V2.IsSame(V);
            if (!a1 && !a2)
            {
                continue;
            }
            Curve2D? aNC = Tool.CurveOnSurface(NE, FF, out double nf, out double nl);
            if (aNC is null)
            {
                continue;
            }
            foreach (double t in a1 && a2 ? new[] { nf, nl } : new[] { a1 ? nf : nl })
            {
                Geom2dAPI_ProjectPointOnCurve aProj = new(aNC.Value(t), aDC, f, l);
                if (aProj.NbPoints() == 0)
                {
                    continue;
                }
                double u = aProj.LowerDistanceParameter();
                if (u > f + aTolPar && u < l - aTolPar && aParams.TrueForAll(x => Math.Abs(x - u) > aTolPar))
                {
                    aParams.Add(u);
                }
            }
        }
        if (aParams.Count == 0)
        {
            return null;
        }
        aParams.Sort();
        aParams.Insert(0, f);
        aParams.Add(l);
        Edge EF = ShapeCasts.Edge(E.Oriented(Forward));
        List<Edge> aParts = [];
        for (int k = 0; k + 1 < aParams.Count; k++)
        {
            BOPTools_AlgoTools.MakeSplitEdge(EF, V, aParams[k], V, aParams[k + 1], out Edge aPart);
            aParts.Add(ShapeCasts.Edge(aPart.Oriented(E.Orientation())));
        }
        return aParts;
    }

    /// <summary>Whether a point inside an edge of the wire is inside one of the faces.</summary>
    private bool IsInside(NCollection_List<Shape> theWire, NCollection_List<Shape> theFaces)
    {
        foreach (Shape S in theWire)
        {
            Edge E = ShapeCasts.Edge(S);
            if (Tool.Degenerated(E))
            {
                continue;
            }
            foreach (Shape P in theFaces)
            {
                Face F = ShapeCasts.Face(P);
                Curve2D? C2d = Tool.CurveOnSurface(E, F, out double f, out double l);
                if (C2d is null)
                {
                    continue;
                }
                Point2D aP2d = C2d.Value(BOPTools_AlgoTools2D.IntermediatePoint(f, l));
                if (myContext.IsPointInFace(F, aP2d))
                {
                    return true;
                }
            }
            return false;
        }
        return false;
    }

    /// <summary>
    /// The face's edges left whole, in touched wires, that it keeps: those connected through edges left whole to a
    /// vertex of a kept edge (decision 1155). A chain of them between two parts dropped lies under the fillet: alone
    /// it dangles and the face builder drops it, but round a seam two such chains close a loop (the rim of a hole that
    /// crosses a cylinder's seam) that it would build into a band.
    /// </summary>
    private static List<Shape> WholeEdgesKept(NCollection_List<Shape> theWhole,
                                                     NCollection_Map<Shape> theIsolated,
                                                     NCollection_Map<Shape> theKeptVertices)
    {
        List<Shape> aEdges = [.. theWhole.Where(E => !theIsolated.Contains(E))];
        int[] aParent = Enumerable.Range(0, aEdges.Count).ToArray();
        int Root(int i)
        {
            while (aParent[i] != i)
            {
                i = aParent[i] = aParent[aParent[i]];
            }
            return i;
        }
        Dictionary<Shape, int> aFirst = new(ShapeHasher.Instance);
        for (int i = 0; i < aEdges.Count; i++)
        {
            for (Explorer exv = new(aEdges[i], ShapeType.Vertex); exv.More(); exv.Next())
            {
                if (aFirst.TryGetValue(exv.Current(), out int j))
                {
                    aParent[Root(i)] = Root(j);
                }
                else
                {
                    aFirst[exv.Current()] = i;
                }
            }
        }
        HashSet<int> aAnchored = [];
        foreach ((Shape V, int i) in aFirst)
        {
            if (theKeptVertices.Contains(V))
            {
                aAnchored.Add(Root(i));
            }
        }
        return [.. aEdges.Where((E, i) => aAnchored.Contains(Root(i)))];
    }

    private static void AddVertices(in Shape E, NCollection_Map<Shape> theMap)
    {
        for (Explorer exv = new(E, ShapeType.Vertex); exv.More(); exv.Next())
        {
            theMap.Add(exv.Current());
        }
    }
}
