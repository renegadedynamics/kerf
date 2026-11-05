// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Api.EdgeError;
using static Kerf.Engine.Construction.Library.LibraryEdgeError;

namespace Kerf.Engine.Construction.Api;

/// <summary>Builds an edge on a curve (gp or Geom) or a curve on a surface, bounded by parameters, points or vertices.</summary>
/// <remarks>
/// A wrapper of <see cref="LibraryMakeEdge"/>. The result is BRepLib's, read without freezing it; <see cref="MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
internal class MakeEdge : MakeShape
{
    private readonly LibraryMakeEdge _makeEdge;

    // OCCT's "if (myMakeEdge.IsDone()) { Done(); myShape = myMakeEdge.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (_makeEdge.IsDone)
        {
            Done();
            _shape = _makeEdge.InternalShape();
        }
    }

    public MakeEdge()
    {
        _makeEdge = new LibraryMakeEdge();
    }

    public MakeEdge(in Vertex v1, in Vertex v2)
    {
        _makeEdge = new LibraryMakeEdge(v1, v2);
        TakeResult();
    }

    public MakeEdge(in Point3D p1, in Point3D p2)
    {
        _makeEdge = new LibraryMakeEdge(p1, p2);
        TakeResult();
    }

    public MakeEdge(in Line3D l)
    {
        _makeEdge = new LibraryMakeEdge(l);
        TakeResult();
    }

    public MakeEdge(in Line3D l, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(l, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Line3D l, in Point3D p1, in Point3D p2)
    {
        _makeEdge = new LibraryMakeEdge(l, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Line3D l, in Vertex v1, in Vertex v2)
    {
        _makeEdge = new LibraryMakeEdge(l, v1, v2);
        TakeResult();
    }

    public MakeEdge(in Circle3D c)
    {
        _makeEdge = new LibraryMakeEdge(c);
        TakeResult();
    }

    public MakeEdge(in Circle3D c, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(c, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Circle3D c, in Point3D p1, in Point3D p2)
    {
        _makeEdge = new LibraryMakeEdge(c, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Circle3D c, in Vertex v1, in Vertex v2)
    {
        _makeEdge = new LibraryMakeEdge(c, v1, v2);
        TakeResult();
    }

    public MakeEdge(in Ellipse3D e)
    {
        _makeEdge = new LibraryMakeEdge(e);
        TakeResult();
    }

    public MakeEdge(in Ellipse3D e, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(e, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Ellipse3D e, in Point3D p1, in Point3D p2)
    {
        _makeEdge = new LibraryMakeEdge(e, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Ellipse3D e, in Vertex v1, in Vertex v2)
    {
        _makeEdge = new LibraryMakeEdge(e, v1, v2);
        TakeResult();
    }

    public MakeEdge(in Hyperbola3D h)
    {
        _makeEdge = new LibraryMakeEdge(h);
        TakeResult();
    }

    public MakeEdge(in Hyperbola3D h, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(h, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Hyperbola3D h, in Point3D p1, in Point3D p2)
    {
        _makeEdge = new LibraryMakeEdge(h, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Hyperbola3D h, in Vertex v1, in Vertex v2)
    {
        _makeEdge = new LibraryMakeEdge(h, v1, v2);
        TakeResult();
    }

    public MakeEdge(in Parabola3D p)
    {
        _makeEdge = new LibraryMakeEdge(p);
        TakeResult();
    }

    public MakeEdge(in Parabola3D p, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(p, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Parabola3D p, in Point3D p1, in Point3D p2)
    {
        _makeEdge = new LibraryMakeEdge(p, p1, p2);
        TakeResult();
    }

    public MakeEdge(in Parabola3D p, in Vertex v1, in Vertex v2)
    {
        _makeEdge = new LibraryMakeEdge(p, v1, v2);
        TakeResult();
    }

    public MakeEdge(Curve3D l)
    {
        _makeEdge = new LibraryMakeEdge(l);
        TakeResult();
    }

    public MakeEdge(Curve3D l, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(l, p1, p2);
        TakeResult();
    }

    public MakeEdge(Curve3D l, in Point3D p1, in Point3D p2)
    {
        _makeEdge = new LibraryMakeEdge(l, p1, p2);
        TakeResult();
    }

    public MakeEdge(Curve3D l, in Vertex v1, in Vertex v2)
    {
        _makeEdge = new LibraryMakeEdge(l, v1, v2);
        TakeResult();
    }

    public MakeEdge(Curve3D l, in Point3D P1, in Point3D P2, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(l, P1, P2, p1, p2);
        TakeResult();
    }

    public MakeEdge(Curve3D l, in Vertex v1, in Vertex v2, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(l, v1, v2, p1, p2);
        TakeResult();
    }

    public MakeEdge(Curve2D l, Surface s)
    {
        _makeEdge = new LibraryMakeEdge(l, s);
        TakeResult();
    }

    public MakeEdge(Curve2D l, Surface s, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(l, s, p1, p2);
        TakeResult();
    }

    public MakeEdge(Curve2D l, Surface s, in Point3D p1, in Point3D p2)
    {
        _makeEdge = new LibraryMakeEdge(l, s, p1, p2);
        TakeResult();
    }

    public MakeEdge(Curve2D l, Surface s, in Vertex v1, in Vertex v2)
    {
        _makeEdge = new LibraryMakeEdge(l, s, v1, v2);
        TakeResult();
    }

    public MakeEdge(Curve2D l, Surface s, in Point3D P1, in Point3D P2, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(l, s, P1, P2, p1, p2);
        TakeResult();
    }

    public MakeEdge(Curve2D l, Surface s, in Vertex v1, in Vertex v2, double p1, double p2)
    {
        _makeEdge = new LibraryMakeEdge(l, s, v1, v2, p1, p2);
        TakeResult();
    }

    public void Init(Curve3D c)
    {
        _makeEdge.Init(c);
        TakeResult();
    }

    public void Init(Curve3D c, double p1, double p2)
    {
        _makeEdge.Init(c, p1, p2);
        TakeResult();
    }

    public void Init(Curve3D c, in Point3D p1, in Point3D p2)
    {
        _makeEdge.Init(c, p1, p2);
        TakeResult();
    }

    public void Init(Curve3D c, in Vertex v1, in Vertex v2)
    {
        _makeEdge.Init(c, v1, v2);
        TakeResult();
    }

    public void Init(Curve3D c, in Point3D P1, in Point3D P2, double p1, double p2)
    {
        _makeEdge.Init(c, P1, P2, p1, p2);
        TakeResult();
    }

    public void Init(Curve3D cc, in Vertex vv1, in Vertex vv2, double pp1, double pp2)
    {
        _makeEdge.Init(cc, vv1, vv2, pp1, pp2);
        TakeResult();
    }

    public void Init(Curve2D c, Surface s)
    {
        _makeEdge.Init(c, s);
        TakeResult();
    }

    public void Init(Curve2D c, Surface s, double p1, double p2)
    {
        _makeEdge.Init(c, s, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D c, Surface s, in Point3D p1, in Point3D p2)
    {
        _makeEdge.Init(c, s, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D c, Surface s, in Vertex v1, in Vertex v2)
    {
        _makeEdge.Init(c, s, v1, v2);
        TakeResult();
    }

    public void Init(Curve2D c, Surface s, in Point3D P1, in Point3D P2, double p1, double p2)
    {
        _makeEdge.Init(c, s, P1, P2, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D cc, Surface s, in Vertex vv1, in Vertex vv2, double pp1, double pp2)
    {
        _makeEdge.Init(cc, s, vv1, vv2, pp1, pp2);
        TakeResult();
    }


    public override bool IsDone
    {
        get
        {
            return _makeEdge.IsDone;
        }
    }

    /// <summary>The construction status.</summary>
    public EdgeError Error()
    {
        switch (_makeEdge.Error())
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
        return _makeEdge.Edge();
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge()
    {
        return _makeEdge.InternalEdge();
    }

    /// <summary>The first vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex1()
    {
        return _makeEdge.Vertex1();
    }

    /// <summary><see cref="Vertex1"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex1()
    {
        return _makeEdge.InternalVertex1();
    }

    /// <summary>The second vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex2()
    {
        return _makeEdge.Vertex2();
    }

    /// <summary><see cref="Vertex2"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex2()
    {
        return _makeEdge.InternalVertex2();
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Edge(MakeEdge maker) => maker.Edge();
}
