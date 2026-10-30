// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_FaceBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_FaceBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds the faces of a <see cref="TopOpeBRepBuild_WireEdgeSet"/>: its wires and the blocks of connex edges are
/// grouped into areas (<see cref="TopOpeBRepBuild_FaceAreaBuilder"/> with a
/// <see cref="TopOpeBRepBuild_WireEdgeClassifier"/>); the result is an iteration on faces, on their wires (old wires
/// kept as they are, or new ones), and on the edges of the new wires.
/// </summary>
/// <remarks>
/// A working object. <see cref="CorrectGclosedWire"/> writes into the edges (their vertices), as in OCCT; the edges
/// must then be unfrozen. OCCT's <c>myMOSI</c> map is never used and is left out.
/// </remarks>
public sealed class TopOpeBRepBuild_FaceBuilder
{
    private const int ISUNKNOWN = -1;
    private const int ISVERTEX = 0;
    private const int GCLOSEDW = 1;
    private const int UNCLOSEDW = 2;
    private const int CLOSEDW = 10;

    private TopoDS_Face myFace = new();
    private readonly TopOpeBRepBuild_LoopSet myLoopSet = new();
    private TopOpeBRepBuild_BlockIterator myBlockIterator = new();
    private readonly TopOpeBRepBuild_BlockBuilder myBlockBuilder = new();
    private readonly TopOpeBRepBuild_FaceAreaBuilder myFaceAreaBuilder = new();

    /// <summary>An empty face builder.</summary>
    public TopOpeBRepBuild_FaceBuilder()
    {
    }

    /// <summary>
    /// The faces of <paramref name="WES"/> on the face <paramref name="F"/>; with <paramref name="ForceClass"/>, the
    /// wires are classified as the blocks are.
    /// </summary>
    public TopOpeBRepBuild_FaceBuilder(TopOpeBRepBuild_WireEdgeSet WES, in TopoDS_Shape F, bool ForceClass = false)
    {
        InitFaceBuilder(WES, F, ForceClass);
    }

    /// <summary>Builds the faces of <paramref name="WES"/> on the face <paramref name="F"/>.</summary>
    public void InitFaceBuilder(TopOpeBRepBuild_WireEdgeSet WES, in TopoDS_Shape F, bool ForceClass)
    {
        myFace = TopoDS.Face(F);
        MakeLoops(WES);
        TopOpeBRepBuild_BlockBuilder BB = myBlockBuilder;
        TopOpeBRepBuild_WireEdgeClassifier WEC = new(F, BB);
        TopOpeBRepBuild_LoopSet LS = myLoopSet;
        myFaceAreaBuilder.InitFaceAreaBuilder(LS, WEC, ForceClass);
    }

    // The vertices of W (not INTERNAL) on one edge only (not INTERNAL or EXTERNAL), with that edge
    private static void FUN_DetectVerticesOn1Edge(in TopoDS_Shape W,
                                                  NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVon1E)
    {
        // Fills up map <mapVon1E>, with vertices of W, connected to only one edge
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapVedges =
            new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(W, TopAbs_VERTEX, TopAbs_EDGE, mapVedges);
        int nV = mapVedges.Extent();

        for (int i = 1; i <= nV; i++)
        {
            TopoDS_Shape V = mapVedges.FindKey(i);
            if (V.Orientation() == TopAbs_INTERNAL)
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> loE = mapVedges.FindFromIndex(i);
            if (loE.Extent() < 2)
            {
                // Keep INTERNAL edges
                TopoDS_Shape E = loE.First();
                TopAbs_Orientation oriE = E.Orientation();
                if ((oriE == TopAbs_INTERNAL) || (oriE == TopAbs_EXTERNAL))
                {
                    continue;
                }
                // ajout du 080399
                mapVon1E.Add(V, E);
            }
        }
    }

    // What the vertices on one edge make of the wire: ISVERTEX, CLOSEDW, GCLOSEDW (closed geometrically: the vertices
    // pair up by position, in mapVV) or UNCLOSEDW
    private static int FUN_AnalyzemapVon1E(NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVon1E,
                                           NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVV)
    {
        int res = ISUNKNOWN;

        int nV = mapVon1E.Extent();
        if (nV == 0)
        {
            res = CLOSEDW;
        }
        else if (nV == 1)
        {
            TopoDS_Shape E = mapVon1E.FindFromIndex(1);
            bool Eclosed = BRep_Tool.IsClosed(E);
            bool dgE = BRep_Tool.Degenerated(TopoDS.Edge(E));
            if (dgE)
            {
                res = ISVERTEX;
            }
            else if (Eclosed)
            {
                res = CLOSEDW;
            }
            else
            {
                res = UNCLOSEDW;
            }
        }
        else
        {
            // Finding among all vertices,
            // couple of vertices falling on same geometry.
            // Filling up map <mapVV>, with (vi,vj) and (vj,vi).
            double tol = Precision.Confusion();
            for (int i = 1; i <= nV; i++)
            {
                TopoDS_Vertex vi = TopoDS.Vertex(mapVon1E.FindKey(i));
                gp_Pnt pi = BRep_Tool.Pnt(vi);
                for (int j = i + 1; j <= nV; j++)
                {
                    TopoDS_Vertex vj = TopoDS.Vertex(mapVon1E.FindKey(j));
                    gp_Pnt pj = BRep_Tool.Pnt(vj);
                    bool same = pi.IsEqual(pj, tol);
                    if (same)
                    {
                        mapVV.Add(vi, vj);
                        mapVV.Add(vj, vi);
                        break;
                    }
                } // j
            } // i
            int nVV = mapVV.Extent();
            if (nVV == nV)
            {
                res = GCLOSEDW;
            }
            else
            {
                res = UNCLOSEDW;
            }
        }
        return res;
    } // FUN_AnalyzemapVon1E

    /// <summary>
    /// Invalidates the edges of the new wires that are not closed even geometrically; fills
    /// <paramref name="mapVVsameG"/> and <paramref name="mapVon1Edge"/> with the vertices of the wires closed only
    /// geometrically (see <see cref="CorrectGclosedWire"/>).
    /// </summary>
    public void DetectUnclosedWire(NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVVsameG,
                                   NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVon1Edge)
    {
        // wire unclosed : has vertices connected to only one edge.
        // edges of wire unclosed : (only one ancestor edge per vertex)
        //   - non INTERNAL or EXTERNAL
        //   - non degenerated
        //   - non closed
        // Fills up the maps <mapVVsameG>, <mapVon1Edge> with the vertices of
        // the 3d-closed but topologically unclosed wires.

        mapVVsameG.Clear();
        mapVon1Edge.Clear();

        // Wire unclosed : edges have vertices connected to only one edge.
        InitFace();
        for (; MoreFace(); NextFace())
        {
            InitWire();
            for (; MoreWire(); NextWire())
            {
                bool isold = IsOldWire();
                if (isold)
                {
                    continue;
                }

                TopoDS_Compound cmp = new();
                BRep_Builder BB = new();
                BB.MakeCompound(ref cmp);
                InitEdge();
                for (; MoreEdge(); NextEdge())
                {
                    AddEdgeWire(Edge(), cmp);
                }
                TopoDS_Shape W = cmp;
                NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVon1E = new(TopTools_ShapeMapHasher.Instance);
                FUN_DetectVerticesOn1Edge(W, mapVon1E);

                NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVV = new(TopTools_ShapeMapHasher.Instance);
                int res = FUN_AnalyzemapVon1E(mapVon1E, mapVV);
                if (res == ISVERTEX)
                {
                    continue;
                }
                else if (res == CLOSEDW)
                {
                    continue;
                }
                else if (res == GCLOSEDW)
                {
                    int i;
                    for (i = 1; i <= mapVV.Extent(); i++)
                    {
                        mapVVsameG.Add(mapVV.FindKey(i), mapVV.FindFromIndex(i));
                    }
                    for (i = 1; i <= mapVon1E.Extent(); i++)
                    {
                        mapVon1Edge.Add(mapVon1E.FindKey(i), mapVon1E.FindFromIndex(i));
                    }
                }
                else if (res == UNCLOSEDW)
                {
                    TopExp_Explorer ex = new();
                    for (ex.Init(W, TopAbs_EDGE); ex.More(); ex.Next())
                    {
                        int I = myBlockBuilder.Element(ex.Current());
                        myBlockBuilder.SetValid(I, false);
                    }
                }
            } // MoreWire
        } // MoreFace
    } // DetectUnclosedWire

    /// <summary>
    /// Makes the wires closed only geometrically closed topologically: on the edge of each vertex of
    /// <paramref name="mapVVref"/>, replaces it by its reference vertex. Writes into the edges.
    /// </summary>
    public void CorrectGclosedWire(NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVVref,
                                   NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> mapVon1Edge)
    {
        // prequesitory : edges described by <mapVon1Edge> are not shared
        int nVV = mapVVref.Extent();
        for (int i = 1; i <= nVV; i++)
        {
            TopoDS_Vertex V = TopoDS.Vertex(mapVVref.FindKey(i));
            TopoDS_Vertex Vref = TopoDS.Vertex(mapVVref.FindFromIndex(i));

            if (V.IsSame(Vref))
            {
                continue;
            }

            TopoDS_Edge E = TopoDS.Edge(mapVon1Edge.FindFromKey(V));
            double paronE = BRep_Tool.Parameter(V, E);

            BRep_Builder BB = new();
            E.Free(true);
            BB.Remove(E, V);
            TopoDS_Shape aLocalShape = Vref.Oriented(V.Orientation());
            TopoDS_Vertex newVref = TopoDS.Vertex(aLocalShape);
            BB.Add(E, newVref);
            TopOpeBRepDS_BuildTool BT = new();
            BT.Parameter(E, newVref, paronE);
        }
    }

    /// <summary>
    /// Invalidates the edges that appear twice (FORWARD and REVERSED) at a vertex of the new wires shared by no other
    /// edge; <paramref name="MapE"/> receives them.
    /// </summary>
    public void DetectPseudoInternalEdge(NCollection_IndexedMap<TopoDS_Shape> MapE)
    {
        TopoDS_Compound cmp = new();
        BRep_Builder BB = new();
        BB.MakeCompound(ref cmp);

        // Wire unclosed : edges have vertices connected to only one edge.
        InitFace();
        for (; MoreFace(); NextFace())
        {
            InitWire();
            for (; MoreWire(); NextWire())
            {
                bool isold = IsOldWire();
                if (isold)
                {
                    continue;
                }
                InitEdge();
                for (; MoreEdge(); NextEdge())
                {
                    AddEdgeWire(Edge(), cmp);
                }
            } // MoreWire
        } // MoreFace

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mapVOE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(cmp, TopAbs_VERTEX, TopAbs_EDGE, mapVOE);
        int nv = mapVOE.Extent();

        MapE.Clear();
        for (int i = 1; i <= nv; i++)
        {
            NCollection_List<TopoDS_Shape> le = mapVOE.FindFromIndex(i);
            int ne = le.Extent();
            if (ne == 2)
            {
                NCollection_List<TopoDS_Shape>.Iterator ile = new(le);
                TopoDS_Shape e1 = ile.Value();
                ile.Next();
                TopoDS_Shape e2 = ile.Value();
                bool same = e1.IsSame(e2);
                TopAbs_Orientation o1 = e1.Orientation();
                TopAbs_Orientation o2 = e2.Orientation();
                bool o1co2 = o1 == TopAbs.Complement(o2);

                if (same && o1co2)
                {
                    MapE.Add(e1);

                    int ie1 = myBlockBuilder.Element(e1);
                    myBlockBuilder.SetValid(ie1, false);

                    int ie2 = myBlockBuilder.Element(e2);
                    myBlockBuilder.SetValid(ie2, false);
                }
            }
        }
    }

    /// <summary>The face the faces are built on.</summary>
    public TopoDS_Shape Face() => myFace;

    /// <summary>Starts the iteration on the faces; their number.</summary>
    public int InitFace()
    {
        int n = myFaceAreaBuilder.InitArea();
        return n;
    }

    /// <summary>Whether there is a current face.</summary>
    public bool MoreFace()
    {
        bool b = myFaceAreaBuilder.MoreArea();
        return b;
    }

    /// <summary>Moves to the next face.</summary>
    public void NextFace()
    {
        myFaceAreaBuilder.NextArea();
    }

    /// <summary>Starts the iteration on the wires of the current face; their number.</summary>
    public int InitWire()
    {
        int n = myFaceAreaBuilder.InitLoop();
        return n;
    }

    /// <summary>Whether there is a current wire.</summary>
    public bool MoreWire()
    {
        bool b = myFaceAreaBuilder.MoreLoop();
        return b;
    }

    /// <summary>Moves to the next wire.</summary>
    public void NextWire()
    {
        myFaceAreaBuilder.NextLoop();
    }

    /// <summary>Whether the current wire is an old one, kept as it is.</summary>
    public bool IsOldWire()
    {
        TopOpeBRepBuild_Loop L = myFaceAreaBuilder.Loop();
        bool b = L.IsShape();
        return b;
    }

    /// <summary>The current wire, when it is an old one.</summary>
    public TopoDS_Shape OldWire()
    {
        TopOpeBRepBuild_Loop L = myFaceAreaBuilder.Loop();
        TopoDS_Shape B = L.Shape();
        return B;
    }

    /// <summary>Moves the edge iterator to the next valid element.</summary>
    public void FindNextValidElement()
    {
        // prerequisites : myBlockIterator.Initialize
        myFaceAreaBuilder.Loop();
        bool found = false;

        while (myBlockIterator.More())
        {
            int i = myBlockIterator.Value();
            found = myBlockBuilder.ElementIsValid(i);
            if (found)
            {
                break;
            }
            else
            {
                myBlockIterator.Next();
            }
        }
    }

    /// <summary>Starts the iteration on the edges of the current (new) wire; their number.</summary>
    /// <exception cref="Standard_DomainError">The current wire is an old one.</exception>
    public int InitEdge()
    {
        TopOpeBRepBuild_Loop L = myFaceAreaBuilder.Loop();
        if (L.IsShape())
        {
            throw new Standard_DomainError("TopOpeBRepBuild_FaceBuilder:InitEdge");
        }
        else
        {
            myBlockIterator = L.BlockIterator();
            myBlockIterator.Initialize();
            FindNextValidElement();
        }
        int n = myBlockIterator.Extent();
        return n;
    }

    /// <summary>Whether there is a current edge.</summary>
    public bool MoreEdge()
    {
        bool b = myBlockIterator.More();
        return b;
    }

    /// <summary>Moves to the next valid edge.</summary>
    public void NextEdge()
    {
        myBlockIterator.Next();
        FindNextValidElement();
    }

    /// <summary>The current edge of the current new wire.</summary>
    /// <exception cref="Standard_Failure">There is none, or it isn't valid.</exception>
    public TopoDS_Shape Edge()
    {
        if (!myBlockIterator.More())
        {
            throw new Standard_Failure("OutOfRange");
        }

        int i = myBlockIterator.Value();
        bool isvalid = myBlockBuilder.ElementIsValid(i);
        if (!isvalid)
        {
            throw new Standard_Failure("Edge not Valid");
        }

        TopoDS_Shape E = myBlockBuilder.Element(i);
        return E;
    }

    /// <summary>0: OCCT's release build has the edge connexity management disabled.</summary>
    public int EdgeConnexity(in TopoDS_Shape E) => 0;

    /// <summary>Adds the edge <paramref name="E"/> to <paramref name="W"/>; 1.</summary>
    public int AddEdgeWire(in TopoDS_Shape E, in TopoDS_Shape W)
    {
        int nadd = 0;
        BRep_Builder BB = new();
        BB.Add(W, E);
        nadd++;
        return nadd;
    }

    private void MakeLoops(TopOpeBRepBuild_ShapeSet SS)
    {
        TopOpeBRepBuild_BlockBuilder BB = myBlockBuilder;
        NCollection_List<TopOpeBRepBuild_Loop> LL = myLoopSet.ChangeListOfLoop();

        // Build blocks on elements of SS
        BB.MakeBlock(SS);

        // make list of loop (LL) of the LoopSet
        // - on shapes of the ShapeSet (SS)
        // - on blocks of the BlockBuilder (BB)

        // Add shapes of SS as shape loops
        LL.Clear();
        for (SS.InitShapes(); SS.MoreShapes(); SS.NextShape())
        {
            TopoDS_Shape S = SS.Shape();
            TopOpeBRepBuild_Loop ShapeLoop = new(S);
            LL.Append(ShapeLoop);
        }

        // Add blocks of BB as block loops
        for (BB.InitBlock(); BB.MoreBlock(); BB.NextBlock())
        {
            TopOpeBRepBuild_BlockIterator BI = BB.BlockIterator();
            TopOpeBRepBuild_Loop BlockLoop = new(BI);
            LL.Append(BlockLoop);
        }
    }
}
