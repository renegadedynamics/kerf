// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakePolygon.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakePolygon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds a polygonal wire from points or vertices: a sequence of connected rectilinear edges. A point identical to the previous one adds no edge (see <see cref="Added"/>).</summary>
/// <remarks>
/// A wrapper of <see cref="LibraryMakePolygon"/>. The result is BRepLib's, read without freezing it; <see cref="MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
internal class MakePolygon : MakeShape
{
    private readonly LibraryMakePolygon _makePolygon;

    // OCCT's "if (myMakePolygon.IsDone()) { Done(); myShape = myMakePolygon.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (_makePolygon.IsDone)
        {
            Done();
            _shape = _makePolygon.InternalShape();
        }
    }

    public MakePolygon()
    {
        _makePolygon = new LibraryMakePolygon();
    }

    public MakePolygon(in Point3D p1, in Point3D p2)
    {
        _makePolygon = new LibraryMakePolygon(p1, p2);
        TakeResult();
    }

    public MakePolygon(in Point3D p1, in Point3D p2, in Point3D p3, bool cl = false)
    {
        _makePolygon = new LibraryMakePolygon(p1, p2, p3, cl);
        TakeResult();
    }

    public MakePolygon(in Point3D p1, in Point3D p2, in Point3D p3, in Point3D p4, bool cl = false)
    {
        _makePolygon = new LibraryMakePolygon(p1, p2, p3, p4, cl);
        TakeResult();
    }

    public MakePolygon(in Vertex v1, in Vertex v2)
    {
        _makePolygon = new LibraryMakePolygon(v1, v2);
        TakeResult();
    }

    public MakePolygon(in Vertex v1, in Vertex v2, in Vertex v3, bool cl = false)
    {
        _makePolygon = new LibraryMakePolygon(v1, v2, v3, cl);
        TakeResult();
    }

    public MakePolygon(in Vertex v1, in Vertex v2, in Vertex v3, in Vertex v4, bool cl = false)
    {
        _makePolygon = new LibraryMakePolygon(v1, v2, v3, v4, cl);
        TakeResult();
    }



    /// <summary>Adds the point <paramref name="p"/> at the end of the polygon.</summary>
    public void Add(in Point3D p)
    {
        _makePolygon.Add(p);
        if (_makePolygon.IsDone)
        {
            Done();
            if (!_makePolygon.InternalLastVertex().IsNull) // OCCT: LastVertex()
            {
                _shape = _makePolygon.InternalShape();
            }
        }
    }

    /// <summary>Adds the vertex <paramref name="v"/> at the end of the polygon.</summary>
    public void Add(in Vertex v)
    {
        _makePolygon.Add(v);
        if (_makePolygon.IsDone)
        {
            Done();
            _shape = _makePolygon.InternalShape();
        }
    }

    /// <summary>True if the last vertex added made an edge (it isn't at the previous one).</summary>
    public bool Added()
    {
        return _makePolygon.Added();
    }

    /// <summary>Closes the polygon with an edge from its last vertex to its first.</summary>
    public void Close()
    {
        _makePolygon.Close();
        _shape = _makePolygon.InternalShape();
    }

    /// <summary>The first vertex of the polygon (frozen with it).</summary>
    public Vertex FirstVertex()
    {
        return _makePolygon.FirstVertex();
    }

    /// <summary><see cref="FirstVertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalFirstVertex()
    {
        return _makePolygon.InternalFirstVertex();
    }

    /// <summary>The last vertex of the polygon (frozen with it).</summary>
    public Vertex LastVertex()
    {
        return _makePolygon.LastVertex();
    }

    /// <summary><see cref="LastVertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalLastVertex()
    {
        return _makePolygon.InternalLastVertex();
    }

    public override bool IsDone
    {
        get
        {
            return _makePolygon.IsDone;
        }
    }

    /// <summary>The last edge built (frozen with the polygon).</summary>
    public Edge Edge()
    {
        return _makePolygon.Edge();
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge()
    {
        return _makePolygon.InternalEdge();
    }

    /// <summary>The wire (frozen).</summary>
    public Wire Wire()
    {
        return _makePolygon.Wire();
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal Wire InternalWire()
    {
        return _makePolygon.InternalWire();
    }

    /// <summary>The last edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Edge(MakePolygon maker) => maker.Edge();

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Wire(MakePolygon maker) => maker.Wire();
}
