// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Tool.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>
/// The working data of a translation from STEP topology to a BRep: the shapes made from the topological items (bound
/// in the transient process too), the vertices and edges of faceted breps keyed by their points, and statistics on
/// the continuity of the geometry.
/// </summary>
/// <remarks>
/// A working object (OCCT's value class, decision 1185). It holds the translation's shapes as they are being built
/// (unfrozen); the maps are keyed by reference and never iterated.
/// </remarks>
public sealed class ImportTool
{
    private Dictionary<StepTopologicalRepresentationItem, Shape> _dataMap = new();
    private Dictionary<StepCartesianPoint, Vertex> _vertexMap = new();
    private Dictionary<PointPair, Edge> _edgeMap = new();
    private bool _computePC;
    private TransientProcess? _transProc;
    private int _nbC0Surf;
    private int _nbC1Surf;
    private int _nbC2Surf;
    private int _nbC0Cur2;
    private int _nbC1Cur2;
    private int _nbC2Cur2;
    private int _nbC0Cur3;
    private int _nbC1Cur3;
    private int _nbC2Cur3;
    // OCCT's maps accept a null handle as a key (a file's missing reference); .NET's dictionaries don't
    private bool _hasNullItem;
    private Shape _nullItemShape;

    public ImportTool()
    {
        _computePC = false;
    }

    /// <summary>A tool with a copy of the map and the transient process.</summary>
    public ImportTool(Dictionary<StepTopologicalRepresentationItem, Shape> map,
                             TransientProcess? tp)
    {
        Init(map, tp);
    }

    public void Init(Dictionary<StepTopologicalRepresentationItem, Shape> map, TransientProcess? tp)
    {
        _computePC = false;

        _dataMap = new Dictionary<StepTopologicalRepresentationItem, Shape>(map,map.Comparer);
        _vertexMap = new Dictionary<StepCartesianPoint, Vertex>();
        _edgeMap = new Dictionary<PointPair, Edge>();
        _transProc = tp;
        _hasNullItem = false;
        _nullItemShape = new Shape();

        _nbC0Surf = _nbC1Surf = _nbC2Surf = 0;
        _nbC0Cur2 = _nbC1Cur2 = _nbC2Cur2 = 0;
        _nbC0Cur3 = _nbC1Cur3 = _nbC2Cur3 = 0;
    }

    /// <summary>True if the item is bound in the map.</summary>
    public bool IsBound(StepTopologicalRepresentationItem? tri) => tri is null ? _hasNullItem : _dataMap.ContainsKey(tri);

    /// <summary>Binds the item with a shape in the map and, as its shape result, in the transient process.</summary>
    public void Bind(StepTopologicalRepresentationItem? tri, in Shape s)
    {
        if (tri is null)
        {
            _hasNullItem = true;
            _nullItemShape = s;
        }
        else
        {
            _dataMap[tri] = s;
        }

        global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.SetShapeResult(_transProc, tri, s);
    }

    /// <summary>The shape bound to the item.</summary>
    /// <exception cref="NoSuchObjectException">The item is not bound.</exception>
    public Shape Find(StepTopologicalRepresentationItem? tri)
    {
        if (tri is null)
        {
            return _hasNullItem ? _nullItemShape : throw new NoSuchObjectException("NCollection_DataMap::Find");
        }

        return _dataMap.GetValue(tri);
    }

    public void ClearEdgeMap() => _edgeMap.Clear();

    public bool IsEdgeBound(in PointPair pp) => _edgeMap.ContainsKey(pp);

    public void BindEdge(in PointPair pp, in Edge e) => _edgeMap[pp] = e;

    public Edge FindEdge(in PointPair pp) => _edgeMap.GetValue(pp);

    public void ClearVertexMap() => _vertexMap.Clear();

    public bool IsVertexBound(StepCartesianPoint? pg) => pg is not null && _vertexMap.ContainsKey(pg);

    public void BindVertex(StepCartesianPoint? p, in Vertex v)
    {
        _vertexMap[p!] = v;
        // OCCT_DEBUG: TransferBRep::SetShapeResult(myTransProc, P, V) left out
    }

    public Vertex FindVertex(StepCartesianPoint? p) => _vertexMap.GetValue(p!);

    public void ComputePCurve(bool b) => _computePC = b;

    public bool ComputePCurve() => _computePC;

    public TransientProcess? TransientProcess() => _transProc;

    // AddStatistics

    public void AddContinuity(Surface geomSurf)
    {
        switch (geomSurf.Continuity())
        {
            case C0:
                _nbC0Surf++;
                break;
            case C1:
                _nbC1Surf++;
                break;
            default:
                _nbC2Surf++;
                break;
        }
    }

    public void AddContinuity(Curve3D geomCurve)
    {
        switch (geomCurve.Continuity())
        {
            case C0:
                _nbC0Cur3++;
                break;
            case C1:
                _nbC1Cur3++;
                break;
            default:
                _nbC2Cur3++;
                break;
        }
    }

    public void AddContinuity(Curve2D geomCur2d)
    {
        switch (geomCur2d.Continuity())
        {
            case C0:
                _nbC0Cur2++;
                break;
            case C1:
                _nbC1Cur2++;
                break;
            default:
                _nbC2Cur2++;
                break;
        }
    }

    // Statistics

    public int C0Surf() => _nbC0Surf;

    public int C1Surf() => _nbC1Surf;

    public int C2Surf() => _nbC2Surf;

    public int C0Cur2() => _nbC0Cur2;

    public int C1Cur2() => _nbC1Cur2;

    public int C2Cur2() => _nbC2Cur2;

    public int C0Cur3() => _nbC0Cur3;

    public int C1Cur3() => _nbC1Cur3;

    public int C2Cur3() => _nbC2Cur3;
}
