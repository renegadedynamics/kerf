// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_FaceBuilder.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_FaceBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>
/// Builds the faces of a <see cref="LegacyWireEdgeSet"/>: its wires and the blocks of connex edges are
/// grouped into areas (<see cref="FaceAreaBuilder"/> with a
/// <see cref="WireEdgeClassifier"/>); the result is an iteration on faces, on their wires (old wires
/// kept as they are, or new ones), and on the edges of the new wires.
/// </summary>
/// <remarks>
/// A working object. <see cref="CorrectGclosedWire"/> writes into the edges (their vertices), as in OCCT; the edges
/// must then be unfrozen. OCCT's <c>myMOSI</c> map is never used and is left out.
/// </remarks>
public sealed class BuildFaceBuilder
{
    private const int Isunknown = -1;
    private const int Isvertex = 0;
    private const int Gclosedw = 1;
    private const int Unclosedw = 2;
    private const int Closedw = 10;

    private Face _face = new();
    private readonly LoopSet _loopSet = new();
    private BlockIterator _blockIterator = new();
    private readonly BlockBuilder _blockBuilder = new();
    private readonly FaceAreaBuilder _faceAreaBuilder = new();

    /// <summary>An empty face builder.</summary>
    public BuildFaceBuilder()
    {
    }

    /// <summary>
    /// The faces of <paramref name="wes"/> on the face <paramref name="f"/>; with <paramref name="forceClass"/>, the
    /// wires are classified as the blocks are.
    /// </summary>
    public BuildFaceBuilder(LegacyWireEdgeSet wes, in Shape f, bool forceClass = false)
    {
        InitFaceBuilder(wes, f, forceClass);
    }

    /// <summary>Builds the faces of <paramref name="wes"/> on the face <paramref name="f"/>.</summary>
    public void InitFaceBuilder(LegacyWireEdgeSet wes, in Shape f, bool forceClass)
    {
        _face = ShapeCasts.Face(f);
        MakeLoops(wes);
        BlockBuilder bb = _blockBuilder;
        WireEdgeClassifier wec = new(f, bb);
        LoopSet ls = _loopSet;
        _faceAreaBuilder.InitFaceAreaBuilder(ls, wec, forceClass);
    }

    // The vertices of W (not INTERNAL) on one edge only (not INTERNAL or EXTERNAL), with that edge
    private static void FUN_DetectVerticesOn1Edge(in Shape w,
                                                  NCollection_IndexedDataMap<Shape, Shape> mapVon1E)
    {
        // Fills up map <mapVon1E>, with vertices of W, connected to only one edge
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapVedges =
            new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(w, ShapeType.Vertex, global::Kerf.Topology.ShapeType.Edge, mapVedges);
        int nV = mapVedges.Extent();

        for (int i = 1; i <= nV; i++)
        {
            Shape v = mapVedges.FindKey(i);
            if (v.Orientation() == Internal)
            {
                continue;
            }

            NCollection_List<Shape> loE = mapVedges.FindFromIndex(i);
            if (loE.Extent() < 2)
            {
                // Keep INTERNAL edges
                Shape e = loE.First();
                Orientation oriE = e.Orientation();
                if ((oriE == Internal) || (oriE == External))
                {
                    continue;
                }
                // ajout du 080399
                mapVon1E.Add(v, e);
            }
        }
    }

    // What the vertices on one edge make of the wire: ISVERTEX, CLOSEDW, GCLOSEDW (closed geometrically: the vertices
    // pair up by position, in mapVV) or UNCLOSEDW
    private static int FUN_AnalyzemapVon1E(NCollection_IndexedDataMap<Shape, Shape> mapVon1E,
                                           NCollection_IndexedDataMap<Shape, Shape> mapVV)
    {
        int res = Isunknown;

        int nV = mapVon1E.Extent();
        if (nV == 0)
        {
            res = Closedw;
        }
        else if (nV == 1)
        {
            Shape e = mapVon1E.FindFromIndex(1);
            bool eclosed = Tool.IsClosed(e);
            bool dgE = Tool.Degenerated(ShapeCasts.Edge(e));
            if (dgE)
            {
                res = Isvertex;
            }
            else if (eclosed)
            {
                res = Closedw;
            }
            else
            {
                res = Unclosedw;
            }
        }
        else
        {
            // Finding among all vertices,
            // couple of vertices falling on same geometry.
            // Filling up map <mapVV>, with (vi,vj) and (vj,vi).
            double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
            for (int i = 1; i <= nV; i++)
            {
                Vertex vi = ShapeCasts.Vertex(mapVon1E.FindKey(i));
                Point3D pi = Tool.Pnt(vi);
                for (int j = i + 1; j <= nV; j++)
                {
                    Vertex vj = ShapeCasts.Vertex(mapVon1E.FindKey(j));
                    Point3D pj = Tool.Pnt(vj);
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
                res = Gclosedw;
            }
            else
            {
                res = Unclosedw;
            }
        }
        return res;
    } // FUN_AnalyzemapVon1E

    /// <summary>
    /// Invalidates the edges of the new wires that are not closed even geometrically; fills
    /// <paramref name="mapVVsameG"/> and <paramref name="mapVon1Edge"/> with the vertices of the wires closed only
    /// geometrically (see <see cref="CorrectGclosedWire"/>).
    /// </summary>
    public void DetectUnclosedWire(NCollection_IndexedDataMap<Shape, Shape> mapVVsameG,
                                   NCollection_IndexedDataMap<Shape, Shape> mapVon1Edge)
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

                Compound cmp = new();
                BrepBuilder bb = new();
                bb.MakeCompound(ref cmp);
                InitEdge();
                for (; MoreEdge(); NextEdge())
                {
                    AddEdgeWire(Edge(), cmp);
                }
                Shape w = cmp;
                NCollection_IndexedDataMap<Shape, Shape> mapVon1E = new(ShapeHasher.Instance);
                FUN_DetectVerticesOn1Edge(w, mapVon1E);

                NCollection_IndexedDataMap<Shape, Shape> mapVV = new(ShapeHasher.Instance);
                int res = FUN_AnalyzemapVon1E(mapVon1E, mapVV);
                if (res == Isvertex)
                {
                    continue;
                }
                else if (res == Closedw)
                {
                    continue;
                }
                else if (res == Gclosedw)
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
                else if (res == Unclosedw)
                {
                    Explorer ex = new();
                    for (ex.Init(w, global::Kerf.Topology.ShapeType.Edge); ex.More(); ex.Next())
                    {
                        int I = _blockBuilder.Element(ex.Current());
                        _blockBuilder.SetValid(I, false);
                    }
                }
            } // MoreWire
        } // MoreFace
    } // DetectUnclosedWire

    /// <summary>
    /// Makes the wires closed only geometrically closed topologically: on the edge of each vertex of
    /// <paramref name="mapVVref"/>, replaces it by its reference vertex. Writes into the edges.
    /// </summary>
    public void CorrectGclosedWire(NCollection_IndexedDataMap<Shape, Shape> mapVVref,
                                   NCollection_IndexedDataMap<Shape, Shape> mapVon1Edge)
    {
        // prequesitory : edges described by <mapVon1Edge> are not shared
        int nVV = mapVVref.Extent();
        for (int i = 1; i <= nVV; i++)
        {
            Vertex v = ShapeCasts.Vertex(mapVVref.FindKey(i));
            Vertex vref = ShapeCasts.Vertex(mapVVref.FindFromIndex(i));

            if (v.IsSame(vref))
            {
                continue;
            }

            Edge e = ShapeCasts.Edge(mapVon1Edge.FindFromKey(v));
            double paronE = Tool.Parameter(v, e);

            BrepBuilder bb = new();
            e.Free(true);
            bb.Remove(e, v);
            Shape localShape = vref.Oriented(v.Orientation());
            Vertex newVref = ShapeCasts.Vertex(localShape);
            bb.Add(e, newVref);
            DsBuildTool bt = new();
            bt.Parameter(e, newVref, paronE);
        }
    }

    /// <summary>
    /// Invalidates the edges that appear twice (FORWARD and REVERSED) at a vertex of the new wires shared by no other
    /// edge; <paramref name="mapE"/> receives them.
    /// </summary>
    public void DetectPseudoInternalEdge(NCollection_IndexedMap<Shape> mapE)
    {
        Compound cmp = new();
        BrepBuilder bb = new();
        bb.MakeCompound(ref cmp);

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

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapVOE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(cmp, ShapeType.Vertex, global::Kerf.Topology.ShapeType.Edge, mapVOE);
        int nv = mapVOE.Extent();

        mapE.Clear();
        for (int i = 1; i <= nv; i++)
        {
            NCollection_List<Shape> le = mapVOE.FindFromIndex(i);
            int ne = le.Extent();
            if (ne == 2)
            {
                NCollection_List<Shape>.Iterator ile = new(le);
                Shape e1 = ile.Value();
                ile.Next();
                Shape e2 = ile.Value();
                bool same = e1.IsSame(e2);
                Orientation o1 = e1.Orientation();
                Orientation o2 = e2.Orientation();
                bool o1co2 = o1 == global::Kerf.Engine.Topology.TopAbs.Complement(o2);

                if (same && o1co2)
                {
                    mapE.Add(e1);

                    int ie1 = _blockBuilder.Element(e1);
                    _blockBuilder.SetValid(ie1, false);

                    int ie2 = _blockBuilder.Element(e2);
                    _blockBuilder.SetValid(ie2, false);
                }
            }
        }
    }

    /// <summary>The face the faces are built on.</summary>
    public Shape Face() => _face;

    /// <summary>Starts the iteration on the faces; their number.</summary>
    public int InitFace()
    {
        int n = _faceAreaBuilder.InitArea();
        return n;
    }

    /// <summary>Whether there is a current face.</summary>
    public bool MoreFace()
    {
        bool b = _faceAreaBuilder.MoreArea();
        return b;
    }

    /// <summary>Moves to the next face.</summary>
    public void NextFace()
    {
        _faceAreaBuilder.NextArea();
    }

    /// <summary>Starts the iteration on the wires of the current face; their number.</summary>
    public int InitWire()
    {
        int n = _faceAreaBuilder.InitLoop();
        return n;
    }

    /// <summary>Whether there is a current wire.</summary>
    public bool MoreWire()
    {
        bool b = _faceAreaBuilder.MoreLoop();
        return b;
    }

    /// <summary>Moves to the next wire.</summary>
    public void NextWire()
    {
        _faceAreaBuilder.NextLoop();
    }

    /// <summary>Whether the current wire is an old one, kept as it is.</summary>
    public bool IsOldWire()
    {
        BuildLoop l = _faceAreaBuilder.Loop();
        bool b = l.IsShape;
        return b;
    }

    /// <summary>The current wire, when it is an old one.</summary>
    public Shape OldWire()
    {
        BuildLoop l = _faceAreaBuilder.Loop();
        Shape b = l.Shape();
        return b;
    }

    /// <summary>Moves the edge iterator to the next valid element.</summary>
    public void FindNextValidElement()
    {
        // prerequisites : myBlockIterator.Initialize
        _faceAreaBuilder.Loop();
        bool found = false;

        while (_blockIterator.More())
        {
            int i = _blockIterator.Value();
            found = _blockBuilder.ElementIsValid(i);
            if (found)
            {
                break;
            }
            else
            {
                _blockIterator.Next();
            }
        }
    }

    /// <summary>Starts the iteration on the edges of the current (new) wire; their number.</summary>
    /// <exception cref="DomainErrorException">The current wire is an old one.</exception>
    public int InitEdge()
    {
        BuildLoop l = _faceAreaBuilder.Loop();
        if (l.IsShape)
        {
            throw new DomainErrorException("TopOpeBRepBuild_FaceBuilder:InitEdge");
        }
        else
        {
            _blockIterator = l.BlockIterator();
            _blockIterator.Initialize();
            FindNextValidElement();
        }
        int n = _blockIterator.Extent();
        return n;
    }

    /// <summary>Whether there is a current edge.</summary>
    public bool MoreEdge()
    {
        bool b = _blockIterator.More();
        return b;
    }

    /// <summary>Moves to the next valid edge.</summary>
    public void NextEdge()
    {
        _blockIterator.Next();
        FindNextValidElement();
    }

    /// <summary>The current edge of the current new wire.</summary>
    /// <exception cref="FailureException">There is none, or it isn't valid.</exception>
    public Shape Edge()
    {
        if (!_blockIterator.More())
        {
            throw new FailureException("OutOfRange");
        }

        int i = _blockIterator.Value();
        bool isvalid = _blockBuilder.ElementIsValid(i);
        if (!isvalid)
        {
            throw new FailureException("Edge not Valid");
        }

        Shape e = _blockBuilder.Element(i);
        return e;
    }

    /// <summary>0: OCCT's release build has the edge connexity management disabled.</summary>
    public int EdgeConnexity(in Shape e) => 0;

    /// <summary>Adds the edge <paramref name="e"/> to <paramref name="w"/>; 1.</summary>
    public int AddEdgeWire(in Shape e, in Shape w)
    {
        int nadd = 0;
        BrepBuilder bb = new();
        bb.Add(w, e);
        nadd++;
        return nadd;
    }

    private void MakeLoops(BuildShapeSet ss)
    {
        BlockBuilder bb = _blockBuilder;
        NCollection_List<BuildLoop> ll = _loopSet.ChangeListOfLoop();

        // Build blocks on elements of SS
        bb.MakeBlock(ss);

        // make list of loop (LL) of the LoopSet
        // - on shapes of the ShapeSet (SS)
        // - on blocks of the BlockBuilder (BB)

        // Add shapes of SS as shape loops
        ll.Clear();
        for (ss.InitShapes(); ss.MoreShapes(); ss.NextShape())
        {
            Shape s = ss.Shape();
            BuildLoop shapeLoop = new(s);
            ll.Append(shapeLoop);
        }

        // Add blocks of BB as block loops
        for (bb.InitBlock(); bb.MoreBlock(); bb.NextBlock())
        {
            BlockIterator bi = bb.BlockIterator();
            BuildLoop blockLoop = new(bi);
            ll.Append(blockLoop);
        }
    }
}
