// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeEdge2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeEdge2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepBuilderAPI_EdgeError;
using static Occt.BRepLib_EdgeError;

namespace Occt;

/// <summary>Builds an edge on a 2D curve (gp or Geom2d), in the default plane (<c>BRepBuilderAPI.Plane()</c>), bounded by parameters, points or vertices.</summary>
/// <remarks>
/// A wrapper of <see cref="BRepLib_MakeEdge2d"/>. The result is BRepLib's, read without freezing it; <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
public class BRepBuilderAPI_MakeEdge2d : BRepBuilderAPI_MakeShape
{
    private readonly BRepLib_MakeEdge2d myMakeEdge2d;

    // OCCT's "if (myMakeEdge2d.IsDone()) { Done(); myShape = myMakeEdge2d.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (myMakeEdge2d.IsDone())
        {
            Done();
            myShape = myMakeEdge2d.InternalShape();
        }
    }

    public BRepBuilderAPI_MakeEdge2d(in Vertex V1, in Vertex V2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Point2D P1, in Point2D P2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Line2D L)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Line2D L, double p1, double p2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Line2D L, in Point2D P1, in Point2D P2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Line2D L, in Vertex V1, in Vertex V2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Circle2D C)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(C);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Circle2D C, double p1, double p2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(C, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Circle2D C, in Point2D P1, in Point2D P2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(C, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Circle2D C, in Vertex V1, in Vertex V2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(C, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Ellipse2D E)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(E);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Ellipse2D E, double p1, double p2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(E, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Ellipse2D E, in Point2D P1, in Point2D P2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(E, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Ellipse2D E, in Vertex V1, in Vertex V2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(E, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Hyperbola2D H)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(H);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Hyperbola2D H, double p1, double p2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(H, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Hyperbola2D H, in Point2D P1, in Point2D P2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(H, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Hyperbola2D H, in Vertex V1, in Vertex V2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(H, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Parabola2D P)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(P);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Parabola2D P, double p1, double p2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(P, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Parabola2D P, in Point2D P1, in Point2D P2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(P, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(in Parabola2D P, in Vertex V1, in Vertex V2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(P, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(Curve2D L)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(Curve2D L, double p1, double p2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(Curve2D L, in Point2D P1, in Point2D P2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(Curve2D L, in Vertex V1, in Vertex V2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(Curve2D L, in Point2D P1, in Point2D P2, double p1, double p2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L, P1, P2, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge2d(Curve2D L, in Vertex V1, in Vertex V2, double p1, double p2)
    {
        myMakeEdge2d = new BRepLib_MakeEdge2d(L, V1, V2, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D C)
    {
        myMakeEdge2d.Init(C);
        TakeResult();
    }

    public void Init(Curve2D C, double p1, double p2)
    {
        myMakeEdge2d.Init(C, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D C, in Point2D P1, in Point2D P2)
    {
        myMakeEdge2d.Init(C, P1, P2);
        TakeResult();
    }

    public void Init(Curve2D C, in Vertex V1, in Vertex V2)
    {
        myMakeEdge2d.Init(C, V1, V2);
        TakeResult();
    }

    public void Init(Curve2D C, in Point2D P1, in Point2D P2, double p1, double p2)
    {
        myMakeEdge2d.Init(C, P1, P2, p1, p2);
        TakeResult();
    }

    public void Init(Curve2D CC, in Vertex VV1, in Vertex VV2, double pp1, double pp2)
    {
        myMakeEdge2d.Init(CC, VV1, VV2, pp1, pp2);
        TakeResult();
    }


    public override bool IsDone()
    {
        return myMakeEdge2d.IsDone();
    }

    /// <summary>The construction status.</summary>
    public BRepBuilderAPI_EdgeError Error()
    {
        switch (myMakeEdge2d.Error())
        {
            case BRepLib_EdgeDone:
                return BRepBuilderAPI_EdgeDone;
            case BRepLib_PointProjectionFailed:
                return BRepBuilderAPI_PointProjectionFailed;
            case BRepLib_ParameterOutOfRange:
                return BRepBuilderAPI_ParameterOutOfRange;
            case BRepLib_DifferentPointsOnClosedCurve:
                return BRepBuilderAPI_DifferentPointsOnClosedCurve;
            case BRepLib_PointWithInfiniteParameter:
                return BRepBuilderAPI_PointWithInfiniteParameter;
            case BRepLib_DifferentsPointAndParameter:
                return BRepBuilderAPI_DifferentsPointAndParameter;
            case BRepLib_LineThroughIdenticPoints:
                return BRepBuilderAPI_LineThroughIdenticPoints;
        }

        // portage WNT
        return BRepBuilderAPI_EdgeDone;
    }

    /// <summary>The edge (frozen).</summary>
    public Edge Edge()
    {
        return myMakeEdge2d.Edge();
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge()
    {
        return myMakeEdge2d.InternalEdge();
    }

    /// <summary>The first vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex1()
    {
        return myMakeEdge2d.Vertex1();
    }

    /// <summary><see cref="Vertex1"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex1()
    {
        return myMakeEdge2d.InternalVertex1();
    }

    /// <summary>The second vertex of the edge (frozen with the edge). May be null.</summary>
    public Vertex Vertex2()
    {
        return myMakeEdge2d.Vertex2();
    }

    /// <summary><see cref="Vertex2"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex2()
    {
        return myMakeEdge2d.InternalVertex2();
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Edge(BRepBuilderAPI_MakeEdge2d theMaker) => theMaker.Edge();
}
