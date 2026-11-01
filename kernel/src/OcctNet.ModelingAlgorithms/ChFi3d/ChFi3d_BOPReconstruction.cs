// OCCT.NET: the fillets' topological reconstruction on the Boolean engine (unit 7R, option C of
// docs/phase7-plan.md 2.1, decision 992). Not a port: it answers for TopOpeBRepBuild_HBuilder, which OCCT.NET does
// not port, what ChFi3d_Builder asks of it (IChFi3d_Reconstruction), reading the same data structure.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;
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
/// <b>The steps</b> (decisions 10701 to 10706). <see cref="Perform"/>: (1) the vertices of the points and the edges of
/// the curves, each between a FORWARD and the next REVERSED point, with its pcurves on the faces and surfaces whose
/// SCIs name it (two on a seam); the edges of the input cut at their points, each part with its state; (2) the faces
/// of the surfaces, by <see cref="BOPAlgo_BuilderFace"/> from their edges. <see cref="MergeSolid"/>: (3) each face of
/// the solid with curves or cut edges is split by <see cref="BOPAlgo_BuilderFace"/> from its edges (cut ones replaced
/// by their parts) and its new edges in both orientations; a piece is kept when it holds its new edges in the
/// orientation the transitions give, dropped when it holds them reversed (the strip under the fillet); (4) the kept
/// pieces, the faces left whole and the fillet faces (oriented by the SSIs) are assembled into shells and a solid.
/// The queries (5) answer from the maps those steps fill.
/// </para>
/// <para>
/// <b>Shapes.</b> The input is the fillet builder's working copy (decision 10203): the reconstruction writes only into
/// the shapes it makes (vertices of points, edges, parts and pieces, faces, shells, solids), and reuses the input's
/// faces and edges left whole and its vertices. Nothing is frozen here: ChFi3d's <c>Compute</c> raises the tolerances
/// of vertices, makes the new faces same-parameter and codes the regularities on the result afterwards; the builder
/// freezes once at the end.
/// </para>
/// </remarks>
internal sealed partial class ChFi3d_BOPReconstruction : IChFi3d_Reconstruction
{
    /// <summary>The build tool OCCT gives its HBuilder (kept for the record: the edges are built here).</summary>
    private readonly TopOpeBRepDS_BuildTool myBuildTool;

    private TopOpeBRepDS_HDataStructure? myHDS;
    private IntTools_Context myContext = new();
    private readonly BRep_Builder myBB = new();

    /// <summary>The vertices made for the data structure's points.</summary>
    private readonly Dictionary<int, TopoDS_Vertex> myPointVertices = new();

    /// <summary>The edges of each curve (<see cref="NewEdges"/>) and the faces of each surface (<see cref="NewFaces"/>).</summary>
    private readonly Dictionary<int, NCollection_List<TopoDS_Shape>> myNewEdges = new();
    private readonly Dictionary<int, NCollection_List<TopoDS_Shape>> myNewFaces = new();

    /// <summary>The curve each new edge was made from.</summary>
    private readonly Dictionary<TopoDS_TShape, int> myCurveOfEdge = new(ReferenceEqualityComparer.Instance);

    /// <summary>The parts of each input edge cut at points, forward, with their states.</summary>
    private readonly Dictionary<TopoDS_Shape, List<(TopoDS_Edge Part, TopAbs_State State)>> myEdgeParts =
        new(TopTools_ShapeMapHasher.Instance);

    /// <summary>The parts kept (state IN) of the edges and faces cut: <see cref="Splits"/>.</summary>
    private readonly Dictionary<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySplitIN =
        new(TopTools_ShapeMapHasher.Instance);

    /// <summary>What <see cref="MergeSolid"/> made of each solid or shell: <see cref="Merged"/>.</summary>
    private readonly Dictionary<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMergedIN =
        new(TopTools_ShapeMapHasher.Instance);

    /// <summary>The SCIs of each curve, on faces (support kind SURFACE, a shape index) and surfaces.</summary>
    private readonly Dictionary<int, List<CurveSupport>> myCurveSupports = new();

    private static readonly NCollection_List<TopoDS_Shape> theEmptyList = new();

    internal static Action<string>? Trace; // p7r-DEBUG

    /// <summary>A curve's SCI: on a face of the input (<see cref="Face"/> not null) or on a fillet surface.</summary>
    private readonly record struct CurveSupport(TopoDS_Face Face, int Surface, Geom2d_Curve PCurve,
                                                TopAbs_Orientation Orientation);

    /// <summary>A reconstruction for the fillet builder (<see cref="ChFi3d_Reconstruction.Create"/>).</summary>
    internal ChFi3d_BOPReconstruction(TopOpeBRepDS_BuildTool BT) => myBuildTool = BT;

    //=================================================================================================
    // IChFi3d_Reconstruction

    /// <inheritdoc/>
    public void Perform(TopOpeBRepDS_HDataStructure HDS)
    {
        Clear();
        myHDS = HDS;
        CollectCurveSupports();
        BuildEdges();
        SplitEdges();
        BuildFaces();
    }

    /// <inheritdoc/>
    public void MergeSolid(in TopoDS_Shape S, TopAbs_State ToBuild)
    {
        if (ToBuild != TopAbs_IN || myHDS is null || myMergedIN.ContainsKey(S))
        {
            return;
        }
        NCollection_List<TopoDS_Shape> aMerged = new();
        MergeShape(S, aMerged);
        myMergedIN[S] = aMerged;
    }

    /// <inheritdoc/>
    public bool IsSplit(in TopoDS_Shape S, TopAbs_State ToBuild) =>
        ToBuild == TopAbs_IN && mySplitIN.TryGetValue(S, out NCollection_List<TopoDS_Shape>? L) && !L.IsEmpty();

    /// <inheritdoc/>
    public NCollection_List<TopoDS_Shape> Splits(in TopoDS_Shape S, TopAbs_State ToBuild) =>
        ToBuild == TopAbs_IN && mySplitIN.TryGetValue(S, out NCollection_List<TopoDS_Shape>? L) ? L : theEmptyList;

    /// <inheritdoc/>
    public NCollection_List<TopoDS_Shape> Merged(in TopoDS_Shape S, TopAbs_State ToBuild) =>
        ToBuild == TopAbs_IN && myMergedIN.TryGetValue(S, out NCollection_List<TopoDS_Shape>? L) ? L : theEmptyList;

    /// <inheritdoc/>
    public NCollection_List<TopoDS_Shape> NewFaces(int I) =>
        myNewFaces.TryGetValue(I, out NCollection_List<TopoDS_Shape>? L) ? L : theEmptyList;

    /// <inheritdoc/>
    public NCollection_List<TopoDS_Shape> NewEdges(int I) =>
        myNewEdges.TryGetValue(I, out NCollection_List<TopoDS_Shape>? L) ? L : theEmptyList;

    /// <inheritdoc/>
    public TopOpeBRepDS_HDataStructure? DataStructure() => myHDS;

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
        myCurveOfEdge.Clear();
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
            TopoDS_Shape S = aDS.Shape(i);
            if (S.IsNull() || S.ShapeType() != TopAbs_FACE)
            {
                continue;
            }
            TopoDS_Face F = TopoDS.Face(S);
            foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(i))
            {
                if (I is TopOpeBRepDS_SurfaceCurveInterference SCI && SCI.GeometryType() == TopOpeBRepDS_CURVE
                    && SCI.PCurve() is Geom2d_Curve PC)
                {
                    AddCurveSupport(SCI.Geometry(), new CurveSupport(F, 0, PC, SCI.Transition().Orientation(TopAbs_IN)));
                }
            }
        }
        for (int iS = 1; iS <= aDS.NbSurfaces(); iS++)
        {
            foreach (TopOpeBRepDS_Interference I in aDS.SurfaceInterferences(iS))
            {
                if (I is TopOpeBRepDS_SurfaceCurveInterference SCI && SCI.GeometryType() == TopOpeBRepDS_CURVE
                    && SCI.PCurve() is Geom2d_Curve PC)
                {
                    AddCurveSupport(SCI.Geometry(), new CurveSupport(new TopoDS_Face(), iS, PC, SCI.Transition().Orientation(TopAbs_IN)));
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
    private TopoDS_Vertex VertexOf(TopOpeBRepDS_Kind theKind, int theIndex)
    {
        if (theKind == TopOpeBRepDS_VERTEX)
        {
            return TopoDS.Vertex(DS.Shape(theIndex));
        }
        if (!myPointVertices.TryGetValue(theIndex, out TopoDS_Vertex V))
        {
            TopOpeBRepDS_Point P = DS.Point(theIndex);
            V = new TopoDS_Vertex();
            myBB.MakeVertex(ref V, P.Point(), Math.Max(P.Tolerance(), Precision.Confusion()));
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
            List<(double T, TopoDS_Vertex V, TopAbs_Orientation O)> aPaves = [];
            foreach (TopOpeBRepDS_Interference I in aDS.CurveInterferences(iC))
            {
                if (I is TopOpeBRepDS_CurvePointInterference CPI
                    && (CPI.GeometryType() == TopOpeBRepDS_POINT || CPI.GeometryType() == TopOpeBRepDS_VERTEX))
                {
                    aPaves.Add((CPI.Parameter(), VertexOf(CPI.GeometryType(), CPI.Geometry()),
                                CPI.Transition().Orientation(TopAbs_IN)));
                }
            }
            if (aPaves.Count < 2)
            {
                continue; // a curve without points makes no edge (OCCT's BuildEdges: no pave)
            }
            aPaves.Sort((a, b) => a.T.CompareTo(b.T));
            NCollection_List<TopoDS_Shape> aLE = new();
            for (int k = 0; k + 1 < aPaves.Count; k++)
            {
                if (aPaves[k].O == TopAbs_REVERSED || aPaves[k + 1].O == TopAbs_FORWARD)
                {
                    continue;
                }
                TopoDS_Edge E = MakeCurveEdge(iC, aPaves[k].V, aPaves[k].T, aPaves[k + 1].V, aPaves[k + 1].T);
                aLE.Append(E);
                myCurveOfEdge[E.TShape()!] = iC;
            }
            if (!aLE.IsEmpty())
            {
                myNewEdges[iC] = aLE;
            }
        }
    }

    /// <summary>An edge on a curve between two of its points, with the pcurves of its SCIs.</summary>
    private TopoDS_Edge MakeCurveEdge(int iC, TopoDS_Vertex V1, double T1, TopoDS_Vertex V2, double T2)
    {
        TopOpeBRepDS_Curve C = DS.Curve(iC);
        double aTol = Math.Max(C.Tolerance(), Precision.Confusion());
        TopoDS_Edge E = new();
        Geom_Curve? C3d = C.Curve();
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
        myBB.Add(E, V1.Oriented(TopAbs_FORWARD));
        myBB.Add(E, V2.Oriented(TopAbs_REVERSED));

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
                GetSupport(s, out Geom_Surface aSurf, out TopLoc_Location aLoc, out double aSurfTol);
                aTol = Math.Max(aTol, aSurfTol);
                if (aTwin < 0)
                {
                    myBB.UpdateEdge(E, s.PCurve, aSurf, aLoc, aTol);
                }
                else
                {
                    aDone.Add(aTwin);
                    CurveSupport t = aSupports[aTwin];
                    (Geom2d_Curve aF, Geom2d_Curve aR) = s.Orientation == TopAbs_REVERSED ? (t.PCurve, s.PCurve) : (s.PCurve, t.PCurve);
                    myBB.UpdateEdge(E, aF, aR, aSurf, aLoc, aTol);
                }
            }
        }
        myBB.Range(E, T1, T2);
        myBB.UpdateEdge(E, aTol);
        return E;
    }

    private static bool SameSupport(in CurveSupport A, in CurveSupport B) =>
        A.Face.IsNull() ? B.Face.IsNull() && A.Surface == B.Surface : !B.Face.IsNull() && A.Face.IsSame(B.Face);

    private void GetSupport(in CurveSupport theS, out Geom_Surface theSurf, out TopLoc_Location theLoc, out double theTol)
    {
        if (!theS.Face.IsNull())
        {
            theSurf = BRep_Tool.Surface(theS.Face, out theLoc)!;
            theTol = 0.0;
        }
        else
        {
            TopOpeBRepDS_Surface S = DS.Surface(theS.Surface);
            theSurf = S.Surface()!;
            theLoc = new TopLoc_Location();
            theTol = S.Tolerance(); // OCCT's BuildFaces raises the edges to the surface's tolerance
        }
    }

    /// <summary>Step 1: the input's edges cut at their points, each part with its state.</summary>
    private void SplitEdges()
    {
        TopOpeBRepDS_DataStructure aDS = DS;
        for (int i = 1; i <= aDS.NbShapes(); i++)
        {
            TopoDS_Shape S = aDS.Shape(i);
            if (S.IsNull() || S.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }
            List<(double T, TopoDS_Vertex V, TopAbs_State Before, TopAbs_State After)> aCuts = [];
            foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(i))
            {
                if (I is TopOpeBRepDS_CurvePointInterference CPI
                    && (CPI.GeometryType() == TopOpeBRepDS_POINT || CPI.GeometryType() == TopOpeBRepDS_VERTEX))
                {
                    TopOpeBRepDS_Transition T = CPI.Transition();
                    aCuts.Add((CPI.Parameter(), VertexOf(CPI.GeometryType(), CPI.Geometry()), T.Before(), T.After()));
                }
            }
            if (aCuts.Count == 0)
            {
                continue;
            }
            TopoDS_Edge E = TopoDS.Edge(S.Oriented(TopAbs_FORWARD));
            if (BRep_Tool.Degenerated(E))
            {
                continue;
            }
            SplitEdge(E, aCuts);
        }
    }

    private void SplitEdge(TopoDS_Edge E, List<(double T, TopoDS_Vertex V, TopAbs_State Before, TopAbs_State After)> theCuts)
    {
        TopExp.Vertices(E, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        BRep_Tool.Range(E, out double f, out double l);
        theCuts.Sort((a, b) => a.T.CompareTo(b.T));
        double aTolPar = Precision.PConfusion();

        // the paves: the edge's ends and its points inside, each with the state of the part that follows
        List<(double T, TopoDS_Vertex V)> aPaves = [(f, aV1)];
        List<TopAbs_State> aStates = [];
        TopAbs_State aFirst = theCuts[0].Before;
        aStates.Add(aFirst);
        foreach ((double T, TopoDS_Vertex V, TopAbs_State Before, TopAbs_State After) c in theCuts)
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
        aPaves.Add((l, aV2));
        if (aPaves.Count == 2)
        {
            // not cut: one part, kept when its state is IN
            List<(TopoDS_Edge, TopAbs_State)> aWhole = [(E, aStates[0])];
            myEdgeParts[E] = aWhole;
            NCollection_List<TopoDS_Shape> aKeep = new();
            if (aStates[0] != TopAbs_OUT)
            {
                aKeep.Append(E);
            }
            mySplitIN[E] = aKeep;
            return;
        }
        List<(TopoDS_Edge Part, TopAbs_State State)> aParts = [];
        NCollection_List<TopoDS_Shape> aIN = new();
        for (int k = 0; k + 1 < aPaves.Count; k++)
        {
            BOPTools_AlgoTools.MakeSplitEdge(E, aPaves[k].V, aPaves[k].T, aPaves[k + 1].V, aPaves[k + 1].T,
                                             out TopoDS_Edge aPart);
            aParts.Add((aPart, aStates[k]));
            if (aStates[k] != TopAbs_OUT)
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
            if (S.Surface() is not Geom_Surface aSurf)
            {
                continue;
            }
            NCollection_List<TopoDS_Shape> aLE = new();
            foreach (TopOpeBRepDS_Interference I in aDS.SurfaceInterferences(iS))
            {
                if (I is not TopOpeBRepDS_SurfaceCurveInterference SCI || SCI.GeometryType() != TopOpeBRepDS_CURVE)
                {
                    continue;
                }
                TopAbs_Orientation O = SCI.Transition().Orientation(TopAbs_IN);
                if (O != TopAbs_FORWARD && O != TopAbs_REVERSED)
                {
                    continue;
                }
                foreach (TopoDS_Shape E in NewEdges(SCI.Geometry()))
                {
                    aLE.Append(E.Oriented(O));
                }
            }
            if (aLE.IsEmpty())
            {
                continue;
            }
            TopoDS_Face F = new();
            myBB.MakeFace(ref F, aSurf, Math.Max(S.Tolerance(), Precision.Confusion()));
            NCollection_List<TopoDS_Shape> aFaces = BuildAreas(F, aLE);
            if (!aFaces.IsEmpty())
            {
                myNewFaces[iS] = aFaces;
            }
        }
    }

    /// <summary>The faces <see cref="BOPAlgo_BuilderFace"/> builds on a forward face from edges.</summary>
    private NCollection_List<TopoDS_Shape> BuildAreas(TopoDS_Face theFace, NCollection_List<TopoDS_Shape> theEdges)
    {
        BOPAlgo_BuilderFace aBF = new();
        aBF.SetFace(theFace);
        aBF.SetShapes(theEdges);
        aBF.SetContext(myContext);
        aBF.SetRunParallel(false);
        aBF.SetAvoidInternalShapes(true); // the edges the fillet leaves dangling are dropped
        aBF.Perform();
        return new NCollection_List<TopoDS_Shape>(aBF.Areas());
    }

    //=================================================================================================
    // MergeSolid

    /// <summary>Steps 3 and 4: the solid's (or shell's) faces split, the fillet faces added, the shells assembled.</summary>
    private void MergeShape(TopoDS_Shape S, NCollection_List<TopoDS_Shape> theMerged)
    {
        TopOpeBRepDS_DataStructure aDS = DS;
        NCollection_List<TopoDS_Shape> aFaces = new();
        for (TopExp_Explorer exp = new(S, TopAbs_FACE); exp.More(); exp.Next())
        {
            TopoDS_Face F = TopoDS.Face(exp.Current());
            NCollection_List<TopoDS_Shape>? aPieces = SplitFace(F);
            if (aPieces is null)
            {
                aFaces.Append(F);
                continue;
            }
            foreach (TopoDS_Shape P in aPieces)
            {
                aFaces.Append(P.Oriented(F.Orientation()));
            }
        }
        foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(S))
        {
            if (I is not TopOpeBRepDS_SolidSurfaceInterference || I.GeometryType() != TopOpeBRepDS_SURFACE)
            {
                continue;
            }
            TopAbs_Orientation O = I.Transition().Orientation(TopAbs_IN);
            if (O != TopAbs_FORWARD && O != TopAbs_REVERSED)
            {
                continue;
            }
            foreach (TopoDS_Shape F in NewFaces(I.Geometry()))
            {
                aFaces.Append(F.Oriented(O));
            }
        }

        TopoDS_Shell aShell = new();
        myBB.MakeShell(ref aShell);
        foreach (TopoDS_Shape F in aFaces)
        {
            myBB.Add(aShell, F);
        }
        aShell.Closed(BRep_Tool.IsClosed(aShell));
        if (S.ShapeType() == TopAbs_SHELL)
        {
            theMerged.Append(aShell);
            return;
        }
        TopoDS_Solid aSolid = new();
        myBB.MakeSolid(ref aSolid);
        myBB.Add(aSolid, aShell);
        theMerged.Append(aSolid);
    }


    /// <summary>
    /// Step 3: the pieces of a face with curves or cut edges (forward), or null if it has neither; recorded as the
    /// face's <see cref="Splits"/>.
    /// </summary>
    /// <remarks>
    /// As the HBuilder's face builder, the pieces are built from the kept edges only: the face's new edges oriented by
    /// their transitions, the parts of its cut edges in the state IN, and its edges not cut. Edges not cut that the
    /// fillet removes (the edge filleted, those of a face it swallows) are left dangling by the parts dropped, and
    /// <see cref="BOPAlgo_BuilderFace"/> drops them; a whole wire of the face that touches neither (a hole's wire) is
    /// kept only if it lies inside a piece built without it (decision 10703). A concave fillet's new edges lie outside
    /// the face: the face grows to them, as natively.
    /// </remarks>
    private NCollection_List<TopoDS_Shape>? SplitFace(TopoDS_Face F)
    {
        if (mySplitIN.TryGetValue(F, out NCollection_List<TopoDS_Shape>? aDone))
        {
            return aDone;
        }
        TopOpeBRepDS_DataStructure aDS = DS;
        TopoDS_Face FF = TopoDS.Face(F.Oriented(TopAbs_FORWARD));

        // the face's new edges, oriented by their transitions (both ways on a seam of the face)
        NCollection_List<TopoDS_Shape> aLE = new();
        NCollection_Map<TopoDS_Shape> aKeptVertices = new(TopTools_ShapeMapHasher.Instance);
        HashSet<(TopoDS_TShape, TopAbs_Orientation)> aAdded = [];
        if (aDS.HasShape(F))
        {
            foreach (TopOpeBRepDS_Interference I in aDS.ShapeInterferences(F))
            {
                if (I is not TopOpeBRepDS_SurfaceCurveInterference SCI || SCI.GeometryType() != TopOpeBRepDS_CURVE)
                {
                    continue;
                }
                TopAbs_Orientation O = SCI.Transition().Orientation(TopAbs_IN);
                if (O != TopAbs_FORWARD && O != TopAbs_REVERSED)
                {
                    continue;
                }
                foreach (TopoDS_Shape E in NewEdges(SCI.Geometry()))
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
        NCollection_List<TopoDS_Shape> aWhole = new();
        for (TopExp_Explorer exp = new(FF, TopAbs_EDGE); exp.More(); exp.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(exp.Current());
            if (myEdgeParts.TryGetValue(E, out List<(TopoDS_Edge Part, TopAbs_State State)>? aParts))
            {
                aHasCuts = true;
                foreach ((TopoDS_Edge aPart, TopAbs_State aState) in aParts)
                {
                    if (aState != TopAbs_OUT)
                    {
                        aLE.Append(aPart.Oriented(E.Orientation()));
                        AddVertices(aPart, aKeptVertices);
                    }
                }
            }
            else if (BRep_Tool.Degenerated(E) && SplitDegenerated(FF, E, aLE) is List<TopoDS_Edge> aDegParts)
            {
                foreach (TopoDS_Edge aPart in aDegParts)
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
        TopoDS_Wire anOuter = BRepTools.OuterWire(FF);
        List<NCollection_List<TopoDS_Shape>> aIsolated = [];
        NCollection_List<TopoDS_Shape>? anOuterIsolated = null;
        NCollection_Map<TopoDS_Shape> aIsolatedEdges = new(TopTools_ShapeMapHasher.Instance);
        for (TopoDS_Iterator itw = new(FF); itw.More(); itw.Next())
        {
            TopoDS_Shape W = itw.Value();
            if (W.ShapeType() != TopAbs_WIRE)
            {
                continue;
            }
            bool aTouched = false;
            NCollection_List<TopoDS_Shape> aWE = new();
            for (TopExp_Explorer exp = new(W, TopAbs_EDGE); exp.More() && !aTouched; exp.Next())
            {
                TopoDS_Shape E = exp.Current();
                aTouched = myEdgeParts.ContainsKey(E);
                for (TopExp_Explorer exv = new(E, TopAbs_VERTEX); exv.More() && !aTouched; exv.Next())
                {
                    aTouched = aKeptVertices.Contains(exv.Current());
                }
                aWE.Append(E);
            }
            if (aTouched)
            {
                continue;
            }
            foreach (TopoDS_Shape E in aWE)
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
        foreach (TopoDS_Shape E in aWhole)
        {
            if (!aIsolatedEdges.Contains(E))
            {
                aLE.Append(E);
            }
        }

        NCollection_List<TopoDS_Shape> aPieces = BuildAreas(FF, aLE);
        if (anOuterIsolated is not null)
        {
            // the outer wire, not touched: it bounds what the face keeps when the fillet's loops are holes (a hole's
            // rim filleted, a concave fillet around a hole), and is dropped when a loop the fillet made replaces it
            if (aPieces.IsEmpty())
            {
                foreach (TopoDS_Shape E in anOuterIsolated)
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
            foreach (NCollection_List<TopoDS_Shape> aWE in aIsolated)
            {
                if (IsInside(aWE, aPieces))
                {
                    aAny = true;
                    foreach (TopoDS_Shape E in aWE)
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
        Trace?.Invoke($"face {DS.Shape(F)}: edges {aLE.Extent()} isolated {aIsolated.Count} pieces {aPieces.Extent()}");
        mySplitIN[F] = aPieces;
        return aPieces;
    }

    /// <summary>
    /// The parts of a degenerated edge of the face (forward) cut where new edges reach its vertex in the face's
    /// parametric space (a fillet ending at a cone's apex), oriented as the edge; null if none reaches it inside.
    /// </summary>
    /// <remarks>
    /// All the parts are given to <see cref="BOPAlgo_BuilderFace"/>: its wire splitter follows the parametric space
    /// at the degenerated vertex, so the part under the fillet is left over and dropped (decision 10704).
    /// </remarks>
    private static List<TopoDS_Edge>? SplitDegenerated(TopoDS_Face FF, TopoDS_Edge E, NCollection_List<TopoDS_Shape> theNew)
    {
        TopoDS_Vertex V = TopExp.FirstVertex(TopoDS.Edge(E.Oriented(TopAbs_FORWARD)));
        Geom2d_Curve? aDC = BRep_Tool.CurveOnSurface(E, FF, out double f, out double l);
        if (V.IsNull() || aDC is null)
        {
            return null;
        }
        double aTolPar = Precision.PConfusion();
        List<double> aParams = [];
        foreach (TopoDS_Shape S in theNew)
        {
            TopoDS_Edge NE = TopoDS.Edge(S.Oriented(TopAbs_FORWARD));
            TopExp.Vertices(NE, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            bool a1 = V1.IsSame(V), a2 = V2.IsSame(V);
            if (!a1 && !a2)
            {
                continue;
            }
            Geom2d_Curve? aNC = BRep_Tool.CurveOnSurface(NE, FF, out double nf, out double nl);
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
        TopoDS_Edge EF = TopoDS.Edge(E.Oriented(TopAbs_FORWARD));
        List<TopoDS_Edge> aParts = [];
        for (int k = 0; k + 1 < aParams.Count; k++)
        {
            BOPTools_AlgoTools.MakeSplitEdge(EF, V, aParams[k], V, aParams[k + 1], out TopoDS_Edge aPart);
            aParts.Add(TopoDS.Edge(aPart.Oriented(E.Orientation())));
        }
        return aParts;
    }

    /// <summary>Whether a point inside an edge of the wire is inside one of the faces.</summary>
    private bool IsInside(NCollection_List<TopoDS_Shape> theWire, NCollection_List<TopoDS_Shape> theFaces)
    {
        foreach (TopoDS_Shape S in theWire)
        {
            TopoDS_Edge E = TopoDS.Edge(S);
            if (BRep_Tool.Degenerated(E))
            {
                continue;
            }
            foreach (TopoDS_Shape P in theFaces)
            {
                TopoDS_Face F = TopoDS.Face(P);
                Geom2d_Curve? C2d = BRep_Tool.CurveOnSurface(E, F, out double f, out double l);
                if (C2d is null)
                {
                    continue;
                }
                gp_Pnt2d aP2d = C2d.Value(BOPTools_AlgoTools2D.IntermediatePoint(f, l));
                if (myContext.IsPointInFace(F, aP2d))
                {
                    return true;
                }
            }
            return false;
        }
        return false;
    }

    private static void AddEdges(in TopoDS_Shape S, NCollection_Map<TopoDS_Shape> theMap)
    {
        for (TopExp_Explorer exp = new(S, TopAbs_EDGE); exp.More(); exp.Next())
        {
            theMap.Add(exp.Current());
        }
    }

    private static void AddVertices(in TopoDS_Shape E, NCollection_Map<TopoDS_Shape> theMap)
    {
        for (TopExp_Explorer exv = new(E, TopAbs_VERTEX); exv.More(); exv.Next())
        {
            theMap.Add(exv.Current());
        }
    }
}
