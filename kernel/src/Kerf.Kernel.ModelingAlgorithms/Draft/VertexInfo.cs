// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_VertexInfo.hxx
// Port of: src/ModelingAlgorithms/TKOffset/Draft/Draft_VertexInfo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Features.Draft;

/// <summary>What the draft modification knows of a vertex: its new point and its parameters on its edges.</summary>
/// <remarks>
/// A value type in C++; a class in the port (decision 1033), as <see cref="DraftEdgeInfo"/>. OCCT's two lists and
/// their iterator are two lists and an index.
/// </remarks>
internal sealed class VertexInfo
{
    private Point3D _geom = new();
    private readonly List<Shape> _edges = [];
    private readonly List<double> _params = [];
    private int _itEd; // OCCT's NCollection_List<TopoDS_Shape>::Iterator

    public VertexInfo()
    {
    }

    public void Add(in Edge e)
    {
        for (_itEd = 0; _itEd < _edges.Count; _itEd++)
        {
            if (e.IsSame(_edges[_itEd]))
            {
                break;
            }
        }
        if (_itEd >= _edges.Count)
        {
            _edges.Add(e);
            _params.Add(RealLast());
        }
    }

    public Point3D Geometry() => _geom;

    public ref Point3D ChangeGeometry() => ref _geom;

    /// <exception cref="DomainErrorException">The edge isn't one of the vertex's.</exception>
    public double Parameter(in Edge e) => ChangeParameter(e);

    /// <exception cref="DomainErrorException">The edge isn't one of the vertex's.</exception>
    public ref double ChangeParameter(in Edge e)
    {
        for (_itEd = 0; _itEd < _edges.Count; _itEd++)
        {
            if (_edges[_itEd].IsSame(e))
            {
                return ref CollectionsMarshal.AsSpan(_params)[_itEd];
            }
        }
        throw new DomainErrorException();
    }

    public void InitEdgeIterator() => _itEd = 0;

    public Edge Edge() => ShapeCasts.Edge(_edges[_itEd]);

    public void NextEdge() => _itEd++;

    public bool MoreEdge() => _itEd < _edges.Count;
}
