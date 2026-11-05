// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_Data.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_Data.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_Data.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.HiddenLines.Topology;

/// <summary>
/// The topological data of the outlining: the splits of the edges, the internal outlines, outlines and isolines of
/// the faces, the outline and internal vertices, the vertices inserted on each edge (sorted by parameter), and the
/// map from the new shapes to the shapes they come from.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (HLRTopoBRep_OutLiner holds one by value). OCCT's maps keyed by shapes iterate in hash
/// order; the port's iterate in insertion order (decision 1094): the edges with inserted vertices are visited in the
/// order their first vertex was inserted, which changes only the order the splits are built in.
/// </remarks>
internal sealed class TopologyData
{
    private readonly Dictionary<Shape, Shape> _oldS = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NCollection_List<Shape>> _splE =
        new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, TopologyFaceData> _data =
        new(ShapeHasher.Instance);
    private readonly HashSet<Shape> _outV = new(ShapeHasher.Instance);
    private readonly HashSet<Shape> _intV = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NCollection_List<VData>> _edgesVertices =
        new(ShapeHasher.Instance);

    // OCCT's map iterator on myEdgesVertices: the keys, in the map's order, and the current position
    private readonly List<Shape> _eKeys = [];
    private int _eIndex;
    private readonly NCollection_List<VData>.Iterator _vIterator = new();
    private NCollection_List<VData>? _vList;

    public TopologyData()
    {
        Clear();
    }

    /// <summary>Clears every map.</summary>
    public void Clear()
    {
        _oldS.Clear();
        _splE.Clear();
        _data.Clear();
        _outV.Clear();
        _intV.Clear();
        _edgesVertices.Clear();
    }

    /// <summary>Does nothing, as OCCT's.</summary>
    public void Clean()
    {
    }

    /// <summary>True if the edge has splits.</summary>
    public bool EdgeHasSplE(in Edge e)
    {
        if (!_splE.ContainsKey(e))
        {
            return false;
        }
        return !_splE.GetValue(e).IsEmpty();
    }

    /// <summary>True if the face has internal outlines.</summary>
    public bool FaceHasIntL(in Face f)
    {
        if (!_data.ContainsKey(f))
        {
            return false;
        }
        return !_data.GetValue(f).FaceIntL().IsEmpty();
    }

    /// <summary>True if the face has outlines on its boundary.</summary>
    public bool FaceHasOutL(in Face f)
    {
        if (!_data.ContainsKey(f))
        {
            return false;
        }
        return !_data.GetValue(f).FaceOutL().IsEmpty();
    }

    /// <summary>True if the face has isolines.</summary>
    public bool FaceHasIsoL(in Face f)
    {
        if (!_data.ContainsKey(f))
        {
            return false;
        }
        return !_data.GetValue(f).FaceIsoL().IsEmpty();
    }

    /// <summary>True if E2 is E1 or one of its splits.</summary>
    public bool IsSplEEdgeEdge(in Edge e1, in Edge e2)
    {
        bool found = false;
        if (EdgeHasSplE(e1))
        {
            NCollection_List<Shape>.Iterator itS = new();
            for (itS.Initialize(EdgeSplE(e1)); itS.More() && !found; itS.Next())
            {
                found = itS.Value().IsSame(e2);
            }
        }
        else
        {
            found = e1.IsSame(e2);
        }
        return found;
    }

    /// <summary>True if E is (a split of) an internal outline of F.</summary>
    public bool IsIntLFaceEdge(in Face f, in Edge e)
    {
        bool found = false;
        if (FaceHasIntL(f))
        {
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(FaceIntL(f)); itE.More() && !found; itE.Next())
            {
                found = IsSplEEdgeEdge(ShapeCasts.Edge(itE.Value()), e);
            }
        }
        return found;
    }

    /// <summary>True if E is (a split of) an outline of F.</summary>
    public bool IsOutLFaceEdge(in Face f, in Edge e)
    {
        bool found = false;
        if (FaceHasOutL(f))
        {
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(FaceOutL(f)); itE.More() && !found; itE.Next())
            {
                found = IsSplEEdgeEdge(ShapeCasts.Edge(itE.Value()), e);
            }
        }
        return found;
    }

    /// <summary>True if E is (a split of) an isoline of F.</summary>
    public bool IsIsoLFaceEdge(in Face f, in Edge e)
    {
        bool found = false;
        if (FaceHasIsoL(f))
        {
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(FaceIsoL(f)); itE.More() && !found; itE.Next())
            {
                found = IsSplEEdgeEdge(ShapeCasts.Edge(itE.Value()), e);
            }
        }
        return found;
    }

    /// <summary>The shape a new shape comes from (itself if none).</summary>
    public Shape NewSOldS(in Shape newS)
    {
        if (_oldS.ContainsKey(newS))
        {
            return _oldS.GetValue(newS);
        }
        else
        {
            return newS;
        }
    }

    public NCollection_List<Shape> EdgeSplE(in Edge e) => _splE.GetValue(e);

    public NCollection_List<Shape> FaceIntL(in Face f) => _data.GetValue(f).FaceIntL();

    public NCollection_List<Shape> FaceOutL(in Face f) => _data.GetValue(f).FaceOutL();

    public NCollection_List<Shape> FaceIsoL(in Face f) => _data.GetValue(f).FaceIsoL();

    public bool IsOutV(in Vertex v) => _outV.Contains(v);

    public bool IsIntV(in Vertex v) => _intV.Contains(v);

    /// <summary>Records that NewS comes from OldS (the first record is kept).</summary>
    public void AddOldS(in Shape newS, in Shape oldS)
    {
        if (!_oldS.ContainsKey(newS))
        {
            _oldS[newS] = oldS;
        }
    }

    /// <summary>The splits of E, created empty if needed.</summary>
    public NCollection_List<Shape> AddSplE(in Edge e)
    {
        if (!_splE.ContainsKey(e))
        {
            NCollection_List<Shape> empty = new();
            _splE[e] = empty;
        }
        return _splE.GetValue(e);
    }

    /// <summary>The internal outlines of F, created empty if needed.</summary>
    public NCollection_List<Shape> AddIntL(in Face f)
    {
        if (!_data.ContainsKey(f))
        {
            TopologyFaceData theData = new();
            _data[f] = theData;
        }
        return _data.GetValue(f).AddIntL();
    }

    /// <summary>The outlines of F, created empty if needed.</summary>
    public NCollection_List<Shape> AddOutL(in Face f)
    {
        if (!_data.ContainsKey(f))
        {
            TopologyFaceData theData = new();
            _data[f] = theData;
        }
        return _data.GetValue(f).AddOutL();
    }

    /// <summary>The isolines of F, created empty if needed.</summary>
    public NCollection_List<Shape> AddIsoL(in Face f)
    {
        if (!_data.ContainsKey(f))
        {
            TopologyFaceData theData = new();
            _data[f] = theData;
        }
        return _data.GetValue(f).AddIsoL();
    }

    public void AddOutV(in Vertex v) => _outV.Add(v);

    public void AddIntV(in Vertex v) => _intV.Add(v);

    /// <summary>Starts the iteration on the edges with inserted vertices.</summary>
    public void InitEdge()
    {
        _eKeys.Clear();
        foreach (KeyValuePair<Shape, NCollection_List<VData>> item in _edgesVertices)
        {
            _eKeys.Add(item.Key);
        }
        _eIndex = 0;

        while (MoreEdge() && _edgesVertices.GetValue(_eKeys[_eIndex]).IsEmpty())
        {
            _eIndex++;
        }
    }

    public bool MoreEdge() => _eIndex < _eKeys.Count;

    public void NextEdge()
    {
        _eIndex++;

        while (MoreEdge() && _edgesVertices.GetValue(_eKeys[_eIndex]).IsEmpty())
        {
            _eIndex++;
        }
    }

    public Edge Edge() => ShapeCasts.Edge(_eKeys[_eIndex]);

    /// <summary>Starts the iteration on the vertices inserted on E (creating the empty list if needed).</summary>
    public void InitVertex(in Edge e)
    {
        if (!_edgesVertices.ContainsKey(e))
        {
            NCollection_List<VData> empty = new();
            _edgesVertices[e] = empty;
        }
        NCollection_List<VData> l = _edgesVertices.GetValue(e);
        _vList = l;
        _vIterator.Initialize(l);
    }

    public bool MoreVertex() => _vIterator.More();

    public void NextVertex() => _vIterator.Next();

    public Vertex Vertex() => ShapeCasts.Vertex(_vIterator.Value().Vertex());

    public double Parameter() => _vIterator.Value().Parameter();

    /// <summary>Inserts the vertex V of parameter P before the current one.</summary>
    public void InsertBefore(in Vertex v, double p)
    {
        VData vd = new(p, v);
        _vList!.InsertBefore(vd, _vIterator);
    }

    /// <summary>Appends the vertex V of parameter P.</summary>
    public void Append(in Vertex v, double p)
    {
        VData vd = new(p, v);
        _vList!.Append(vd);
    }
}
