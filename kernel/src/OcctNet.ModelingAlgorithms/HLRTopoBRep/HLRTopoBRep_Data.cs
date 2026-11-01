// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_Data.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_Data.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_Data.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The topological data of the outlining: the splits of the edges, the internal outlines, outlines and isolines of
/// the faces, the outline and internal vertices, the vertices inserted on each edge (sorted by parameter), and the
/// map from the new shapes to the shapes they come from.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (HLRTopoBRep_OutLiner holds one by value). OCCT's maps keyed by shapes iterate in hash
/// order; the port's iterate in insertion order (decision 10604): the edges with inserted vertices are visited in the
/// order their first vertex was inserted, which changes only the order the splits are built in.
/// </remarks>
public sealed class HLRTopoBRep_Data
{
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myOldS = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> mySplE =
        new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, HLRTopoBRep_FaceData> myData =
        new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_Map<TopoDS_Shape> myOutV = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_Map<TopoDS_Shape> myIntV = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<HLRTopoBRep_VData>> myEdgesVertices =
        new(TopTools_ShapeMapHasher.Instance);

    // OCCT's map iterator on myEdgesVertices: the keys, in the map's order, and the current position
    private readonly List<TopoDS_Shape> myEKeys = [];
    private int myEIndex;
    private readonly NCollection_List<HLRTopoBRep_VData>.Iterator myVIterator = new();
    private NCollection_List<HLRTopoBRep_VData>? myVList;

    public HLRTopoBRep_Data()
    {
        Clear();
    }

    /// <summary>Clears every map.</summary>
    public void Clear()
    {
        myOldS.Clear();
        mySplE.Clear();
        myData.Clear();
        myOutV.Clear();
        myIntV.Clear();
        myEdgesVertices.Clear();
    }

    /// <summary>Does nothing, as OCCT's.</summary>
    public void Clean()
    {
    }

    /// <summary>True if the edge has splits.</summary>
    public bool EdgeHasSplE(in TopoDS_Edge E)
    {
        if (!mySplE.IsBound(E))
        {
            return false;
        }
        return !mySplE[E].IsEmpty();
    }

    /// <summary>True if the face has internal outlines.</summary>
    public bool FaceHasIntL(in TopoDS_Face F)
    {
        if (!myData.IsBound(F))
        {
            return false;
        }
        return !myData[F].FaceIntL().IsEmpty();
    }

    /// <summary>True if the face has outlines on its boundary.</summary>
    public bool FaceHasOutL(in TopoDS_Face F)
    {
        if (!myData.IsBound(F))
        {
            return false;
        }
        return !myData[F].FaceOutL().IsEmpty();
    }

    /// <summary>True if the face has isolines.</summary>
    public bool FaceHasIsoL(in TopoDS_Face F)
    {
        if (!myData.IsBound(F))
        {
            return false;
        }
        return !myData[F].FaceIsoL().IsEmpty();
    }

    /// <summary>True if E2 is E1 or one of its splits.</summary>
    public bool IsSplEEdgeEdge(in TopoDS_Edge E1, in TopoDS_Edge E2)
    {
        bool found = false;
        if (EdgeHasSplE(E1))
        {
            NCollection_List<TopoDS_Shape>.Iterator itS = new();
            for (itS.Initialize(EdgeSplE(E1)); itS.More() && !found; itS.Next())
            {
                found = itS.Value().IsSame(E2);
            }
        }
        else
        {
            found = E1.IsSame(E2);
        }
        return found;
    }

    /// <summary>True if E is (a split of) an internal outline of F.</summary>
    public bool IsIntLFaceEdge(in TopoDS_Face F, in TopoDS_Edge E)
    {
        bool found = false;
        if (FaceHasIntL(F))
        {
            NCollection_List<TopoDS_Shape>.Iterator itE = new();
            for (itE.Initialize(FaceIntL(F)); itE.More() && !found; itE.Next())
            {
                found = IsSplEEdgeEdge(TopoDS.Edge(itE.Value()), E);
            }
        }
        return found;
    }

    /// <summary>True if E is (a split of) an outline of F.</summary>
    public bool IsOutLFaceEdge(in TopoDS_Face F, in TopoDS_Edge E)
    {
        bool found = false;
        if (FaceHasOutL(F))
        {
            NCollection_List<TopoDS_Shape>.Iterator itE = new();
            for (itE.Initialize(FaceOutL(F)); itE.More() && !found; itE.Next())
            {
                found = IsSplEEdgeEdge(TopoDS.Edge(itE.Value()), E);
            }
        }
        return found;
    }

    /// <summary>True if E is (a split of) an isoline of F.</summary>
    public bool IsIsoLFaceEdge(in TopoDS_Face F, in TopoDS_Edge E)
    {
        bool found = false;
        if (FaceHasIsoL(F))
        {
            NCollection_List<TopoDS_Shape>.Iterator itE = new();
            for (itE.Initialize(FaceIsoL(F)); itE.More() && !found; itE.Next())
            {
                found = IsSplEEdgeEdge(TopoDS.Edge(itE.Value()), E);
            }
        }
        return found;
    }

    /// <summary>The shape a new shape comes from (itself if none).</summary>
    public TopoDS_Shape NewSOldS(in TopoDS_Shape NewS)
    {
        if (myOldS.IsBound(NewS))
        {
            return myOldS[NewS];
        }
        else
        {
            return NewS;
        }
    }

    public NCollection_List<TopoDS_Shape> EdgeSplE(in TopoDS_Edge E) => mySplE.Find(E);

    public NCollection_List<TopoDS_Shape> FaceIntL(in TopoDS_Face F) => myData.Find(F).FaceIntL();

    public NCollection_List<TopoDS_Shape> FaceOutL(in TopoDS_Face F) => myData.Find(F).FaceOutL();

    public NCollection_List<TopoDS_Shape> FaceIsoL(in TopoDS_Face F) => myData.Find(F).FaceIsoL();

    public bool IsOutV(in TopoDS_Vertex V) => myOutV.Contains(V);

    public bool IsIntV(in TopoDS_Vertex V) => myIntV.Contains(V);

    /// <summary>Records that NewS comes from OldS (the first record is kept).</summary>
    public void AddOldS(in TopoDS_Shape NewS, in TopoDS_Shape OldS)
    {
        if (!myOldS.IsBound(NewS))
        {
            myOldS.Bind(NewS, OldS);
        }
    }

    /// <summary>The splits of E, created empty if needed.</summary>
    public NCollection_List<TopoDS_Shape> AddSplE(in TopoDS_Edge E)
    {
        if (!mySplE.IsBound(E))
        {
            NCollection_List<TopoDS_Shape> empty = new();
            mySplE.Bind(E, empty);
        }
        return mySplE[E];
    }

    /// <summary>The internal outlines of F, created empty if needed.</summary>
    public NCollection_List<TopoDS_Shape> AddIntL(in TopoDS_Face F)
    {
        if (!myData.IsBound(F))
        {
            HLRTopoBRep_FaceData theData = new();
            myData.Bind(F, theData);
        }
        return myData[F].AddIntL();
    }

    /// <summary>The outlines of F, created empty if needed.</summary>
    public NCollection_List<TopoDS_Shape> AddOutL(in TopoDS_Face F)
    {
        if (!myData.IsBound(F))
        {
            HLRTopoBRep_FaceData theData = new();
            myData.Bind(F, theData);
        }
        return myData[F].AddOutL();
    }

    /// <summary>The isolines of F, created empty if needed.</summary>
    public NCollection_List<TopoDS_Shape> AddIsoL(in TopoDS_Face F)
    {
        if (!myData.IsBound(F))
        {
            HLRTopoBRep_FaceData theData = new();
            myData.Bind(F, theData);
        }
        return myData[F].AddIsoL();
    }

    public void AddOutV(in TopoDS_Vertex V) => myOutV.Add(V);

    public void AddIntV(in TopoDS_Vertex V) => myIntV.Add(V);

    /// <summary>Starts the iteration on the edges with inserted vertices.</summary>
    public void InitEdge()
    {
        myEKeys.Clear();
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<HLRTopoBRep_VData>> anItem in myEdgesVertices)
        {
            myEKeys.Add(anItem.Key);
        }
        myEIndex = 0;

        while (MoreEdge() && myEdgesVertices[myEKeys[myEIndex]].IsEmpty())
        {
            myEIndex++;
        }
    }

    public bool MoreEdge() => myEIndex < myEKeys.Count;

    public void NextEdge()
    {
        myEIndex++;

        while (MoreEdge() && myEdgesVertices[myEKeys[myEIndex]].IsEmpty())
        {
            myEIndex++;
        }
    }

    public TopoDS_Edge Edge() => TopoDS.Edge(myEKeys[myEIndex]);

    /// <summary>Starts the iteration on the vertices inserted on E (creating the empty list if needed).</summary>
    public void InitVertex(in TopoDS_Edge E)
    {
        if (!myEdgesVertices.IsBound(E))
        {
            NCollection_List<HLRTopoBRep_VData> empty = new();
            myEdgesVertices.Bind(E, empty);
        }
        NCollection_List<HLRTopoBRep_VData> L = myEdgesVertices[E];
        myVList = L;
        myVIterator.Initialize(L);
    }

    public bool MoreVertex() => myVIterator.More();

    public void NextVertex() => myVIterator.Next();

    public TopoDS_Vertex Vertex() => TopoDS.Vertex(myVIterator.Value().Vertex());

    public double Parameter() => myVIterator.Value().Parameter();

    /// <summary>Inserts the vertex V of parameter P before the current one.</summary>
    public void InsertBefore(in TopoDS_Vertex V, double P)
    {
        HLRTopoBRep_VData VD = new(P, V);
        myVList!.InsertBefore(VD, myVIterator);
    }

    /// <summary>Appends the vertex V of parameter P.</summary>
    public void Append(in TopoDS_Vertex V, double P)
    {
        HLRTopoBRep_VData VD = new(P, V);
        myVList!.Append(VD);
    }
}
