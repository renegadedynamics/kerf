// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeEdge2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeEdge2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds an edge on a 2D curve (gp or Geom2d), in the default plane (<c>BRepBuilderAPI.Plane()</c>), bounded by parameters, points or vertices.</summary>
/// <remarks>
/// A wrapper of <see cref="LibraryMakeEdge2d"/>. The result is BRepLib's, read without freezing it; <see cref="MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
internal class MakeEdge2d : MakeShape
{
    private readonly LibraryMakeEdge2d _makeEdge2d;

    // OCCT's "if (myMakeEdge2d.IsDone()) { Done(); myShape = myMakeEdge2d.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (_makeEdge2d.IsDone)
        {
            Done();
            _shape = _makeEdge2d.InternalShape();
        }
    }

    public MakeEdge2d(in Vertex v1, in Vertex v2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(v1, v2);
        TakeResult();
    }

    public MakeEdge2d(in Point2D p1, in Point2D p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Line2D l)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l);
        TakeResult();
    }

    public MakeEdge2d(in Line2D l, double p1, double p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Line2D l, in Point2D p1, in Point2D p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Line2D l, in Vertex v1, in Vertex v2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l, v1, v2);
        TakeResult();
    }

    public MakeEdge2d(in Circle2D c)
    {
        _makeEdge2d = new LibraryMakeEdge2d(c);
        TakeResult();
    }

    public MakeEdge2d(in Circle2D c, double p1, double p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(c, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Circle2D c, in Point2D p1, in Point2D p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(c, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Circle2D c, in Vertex v1, in Vertex v2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(c, v1, v2);
        TakeResult();
    }

    public MakeEdge2d(in Ellipse2D e)
    {
        _makeEdge2d = new LibraryMakeEdge2d(e);
        TakeResult();
    }

    public MakeEdge2d(in Ellipse2D e, double p1, double p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(e, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Ellipse2D e, in Point2D p1, in Point2D p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(e, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Ellipse2D e, in Vertex v1, in Vertex v2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(e, v1, v2);
        TakeResult();
    }

    public MakeEdge2d(in Hyperbola2D h)
    {
        _makeEdge2d = new LibraryMakeEdge2d(h);
        TakeResult();
    }

    public MakeEdge2d(in Hyperbola2D h, double p1, double p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(h, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Hyperbola2D h, in Point2D p1, in Point2D p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(h, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Hyperbola2D h, in Vertex v1, in Vertex v2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(h, v1, v2);
        TakeResult();
    }

    public MakeEdge2d(in Parabola2D p)
    {
        _makeEdge2d = new LibraryMakeEdge2d(p);
        TakeResult();
    }

    public MakeEdge2d(in Parabola2D p, double p1, double p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(p, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Parabola2D p, in Point2D p1, in Point2D p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(p, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(in Parabola2D p, in Vertex v1, in Vertex v2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(p, v1, v2);
        TakeResult();
    }

    public MakeEdge2d(Curve2D l)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l);
        TakeResult();
    }

    public MakeEdge2d(Curve2D l, double p1, double p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(Curve2D l, in Point2D p1, in Point2D p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(Curve2D l, in Vertex v1, in Vertex v2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l, v1, v2);
        TakeResult();
    }

    public MakeEdge2d(Curve2D l, in Point2D P1, in Point2D P2, double p1, double p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l, P1, P2, p1, p2);
        TakeResult();
    }

    public MakeEdge2d(Curve2D l, in Vertex v1, in Vertex v2, double p1, double p2)
    {
        _makeEdge2d = new LibraryMakeEdge2d(l, v1, v2, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D c)
    {
        _makeEdge2d.Init(c);
        TakeResult();
    }

    public void Init(Curve2D c, double p1, double p2)
    {
        _makeEdge2d.Init(c, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D c, in Point2D p1, in Point2D p2)
    {
        _makeEdge2d.Init(c, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D c, in Vertex v1, in Vertex v2)
    {
        _makeEdge2d.Init(c, v1, v2);
        TakeResult();
    }

    public void Init(Curve2D c, in Point2D P1, in Point2D P2, double p1, double p2)
    {
        _makeEdge2d.Init(c, P1, P2, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D cc, in Vertex vv1, in Vertex vv2, double pp1, double pp2)
    {
        _makeEdge2d.Init(cc, vv1, vv2, pp1, pp2);
        TakeResult();
    }


    public override bool IsDone
    {
        get
        {
            return _makeEdge2d.IsDone;
        }
    }

    /// <summary>The construction status.</summary>
    public EdgeError Error()
    {
        switch (_makeEdge2d.Error())
        {
            case LibraryEdgeError.EdgeDone:
                return EdgeError.EdgeDone;
            case LibraryEdgeError.PointProjectionFailed:
                return EdgeError.PointProjectionFailed;
            case LibraryEdgeError.ParameterOutOfRange:
                return EdgeError.ParameterOutOfRange;
            case LibraryEdgeError.DifferentPointsOnClosedCurve:
                return EdgeError.DifferentPointsOnClosedCurve;
            case LibraryEdgeError.PointWithInfiniteParameter:
                return EdgeError.PointWithInfiniteParameter;
            case LibraryEdgeError.DifferentsPointAndParameter:
                return EdgeError.DifferentsPointAndParameter;
            case LibraryEdgeError.LineThroughIdenticPoints:
                return EdgeError.LineThroughIdenticPoints;
        }

        // portage WNT
        return EdgeError.EdgeDone;
    }

    /// <summary>The edge (frozen).</summary>
    public Edge Edge()
    {
        return _makeEdge2d.Edge();
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge()
    {
        return _makeEdge2d.InternalEdge();
    }

    /// <summary>The first vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex1()
    {
        return _makeEdge2d.Vertex1();
    }

    /// <summary><see cref="Vertex1"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex1()
    {
        return _makeEdge2d.InternalVertex1();
    }

    /// <summary>The second vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex2()
    {
        return _makeEdge2d.Vertex2();
    }

    /// <summary><see cref="Vertex2"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex2()
    {
        return _makeEdge2d.InternalVertex2();
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Edge(MakeEdge2d maker) => maker.Edge();
}
