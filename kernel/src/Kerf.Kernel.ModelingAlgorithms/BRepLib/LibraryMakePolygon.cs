// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakePolygon.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakePolygon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Makes a polygonal wire from points or vertices, adding straight edges one at a time; <see cref="Close"/> closes
/// it. A point equal to the first one (within the vertices' tolerances) closes the polygon; a point that can't make
/// an edge with the previous one (the same point) is skipped.
/// </summary>
/// <remarks>
/// <see cref="Wire"/>, <see cref="Edge"/>, <see cref="FirstVertex"/> and <see cref="LastVertex"/> freeze the wire
/// (design 4.2); ported code reads the internal forms. Adding after a public read goes on with a new wire holding the
/// same edges, so the wire handed out doesn't change.
/// </remarks>
internal class LibraryMakePolygon : LibraryMakeShape
{
    private Vertex _firstVertex = new();
    private Vertex _lastVertex = new();
    private Edge _edge = new();

    /// <summary>Creates an empty polygon.</summary>
    public LibraryMakePolygon()
    {
    }

    public LibraryMakePolygon(in Point3D p1, in Point3D p2)
    {
        Add(p1);
        Add(p2);
    }

    public LibraryMakePolygon(in Point3D p1, in Point3D p2, in Point3D p3, bool cl = false)
    {
        Add(p1);
        Add(p2);
        Add(p3);
        if (cl)
        {
            Close();
        }
    }

    public LibraryMakePolygon(in Point3D p1, in Point3D p2, in Point3D p3, in Point3D p4, bool cl = false)
    {
        Add(p1);
        Add(p2);
        Add(p3);
        Add(p4);
        if (cl)
        {
            Close();
        }
    }

    public LibraryMakePolygon(in Vertex v1, in Vertex v2)
    {
        Add(v1);
        Add(v2);
    }

    public LibraryMakePolygon(in Vertex v1, in Vertex v2, in Vertex v3, bool cl = false)
    {
        Add(v1);
        Add(v2);
        Add(v3);
        if (cl)
        {
            Close();
        }
    }

    public LibraryMakePolygon(in Vertex v1, in Vertex v2, in Vertex v3, in Vertex v4,
                               bool cl = false)
    {
        Add(v1);
        Add(v2);
        Add(v3);
        Add(v4);
        if (cl)
        {
            Close();
        }
    }

    public void Add(in Point3D p)
    {
        var b = new BrepBuilder();
        Vertex v = new();
        b.MakeVertex(ref v, p, global::Kerf.Engine.Foundation.Precision.Confusion());
        Add(v);
    }

    public void Add(in Vertex v)
    {
        WorkingOverlay? overlay = BeginResultChange(); // Kerf: a published polygon changes into a new one
        try
        {
            AddCore(v);
        }
        finally
        {
            EndChange(overlay);
        }
    }

    private void AddCore(in Vertex v)
    {
        if (_firstVertex.IsNull)
        {
            _firstVertex = v;
        }
        else
        {
            _edge.Nullify();
            var b = new BrepBuilder();
            Vertex last;
            bool second = _lastVertex.IsNull;
            if (second)
            {
                last = _firstVertex;
                _lastVertex = v;
                Wire w = new();
                b.MakeWire(ref w);
                _shape = w;
                _shape.Closed(false);
                _shape.Orientable(true);
            }
            else
            {
                last = _lastVertex;
                if (BrepTools.Compare(v, _firstVertex))
                {
                    _lastVertex = _firstVertex;
                    _shape.Closed(true);
                }
                else
                {
                    _lastVertex = v;
                }
            }
            var me = new LibraryMakeEdge(last, _lastVertex);
            if (me.IsDone)
            {
                _edge = me.InternalEdge();
                b.Add(_shape, _edge);
                Done();
            }
            else
            {
                // restore the previous last vertex
                if (second)
                {
                    _lastVertex.Nullify();
                }
                else
                {
                    _lastVertex = last;
                }
            }
        }
    }

    /// <summary>Returns true if the last vertex added was not rejected.</summary>
    public bool Added()
    {
        return !_edge.IsNull;
    }

    /// <summary>Closes the polygon with an edge from the last vertex to the first, unless it is already closed.</summary>
    public void Close()
    {
        if (_firstVertex.IsNull || _lastVertex.IsNull)
        {
            return;
        }

        // check not already closed
        if (_shape.ClosedFlag)
        {
            return;
        }

        WorkingOverlay? overlay = BeginResultChange(); // Kerf: a published polygon changes into a new one

        // build the last edge
        var b = new BrepBuilder();
        _edge.Nullify();
        var me = new LibraryMakeEdge(_lastVertex, _firstVertex);
        if (me.IsDone)
        {
            _edge = me.InternalEdge();
            b.Add(_shape, _edge);
            _shape.Closed(true);
        }
        EndChange(overlay);
    }

    // Kerf: the end of a change of a published polygon; its first and last vertices and last edge as they stand in it
    private void EndChange(WorkingOverlay? overlay)
    {
        if (overlay is null)
        {
            return;
        }
        try
        {
            _shape = overlay.Finish(_shape, [_firstVertex, _lastVertex, _edge]);
            _firstVertex = ShapeCasts.Vertex(overlay.Final(_firstVertex));
            _lastVertex = ShapeCasts.Vertex(overlay.Final(_lastVertex));
            _edge = ShapeCasts.Edge(overlay.Final(_edge));
        }
        finally
        {
            overlay.Dispose();
        }
    }

    /// <summary>The first vertex (frozen with the wire). May be null.</summary>
    public Vertex FirstVertex() => ShapeCasts.Vertex(Published(_firstVertex));

    /// <summary><see cref="FirstVertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalFirstVertex() => _firstVertex;

    /// <summary>The last vertex (frozen with the wire). May be null.</summary>
    public Vertex LastVertex() => ShapeCasts.Vertex(Published(_lastVertex));

    /// <summary><see cref="LastVertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalLastVertex() => _lastVertex;

    /// <summary>The last edge added (frozen with the wire); null if the last vertex was rejected.</summary>
    public Edge Edge() => ShapeCasts.Edge(Published(_edge));

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge() => _edge;

    /// <summary>The wire (frozen).</summary>
    public Wire Wire()
    {
        return ShapeCasts.Wire(Shape());
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal Wire InternalWire()
    {
        return ShapeCasts.Wire(InternalShape());
    }

    /// <summary>The last edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Edge(LibraryMakePolygon maker) => maker.Edge();

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Wire(LibraryMakePolygon maker) => maker.Wire();
}
