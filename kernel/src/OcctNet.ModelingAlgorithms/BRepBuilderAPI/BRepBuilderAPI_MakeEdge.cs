// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeEdge.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakeEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepBuilderAPI_EdgeError;
using static Occt.BRepLib_EdgeError;

namespace Occt;

/// <summary>Builds an edge on a curve (gp or Geom) or a curve on a surface, bounded by parameters, points or vertices.</summary>
/// <remarks>
/// A wrapper of <see cref="BRepLib_MakeEdge"/>. The result is BRepLib's, read without freezing it; <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
public class BRepBuilderAPI_MakeEdge : BRepBuilderAPI_MakeShape
{
    private readonly BRepLib_MakeEdge myMakeEdge;

    // OCCT's "if (myMakeEdge.IsDone()) { Done(); myShape = myMakeEdge.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (myMakeEdge.IsDone())
        {
            Done();
            myShape = myMakeEdge.InternalShape();
        }
    }

    public BRepBuilderAPI_MakeEdge()
    {
        myMakeEdge = new BRepLib_MakeEdge();
    }

    public BRepBuilderAPI_MakeEdge(in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge = new BRepLib_MakeEdge(V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge = new BRepLib_MakeEdge(P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Lin L)
    {
        myMakeEdge = new BRepLib_MakeEdge(L);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Lin L, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Lin L, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Lin L, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Circ C)
    {
        myMakeEdge = new BRepLib_MakeEdge(C);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Circ C, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(C, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Circ C, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge = new BRepLib_MakeEdge(C, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Circ C, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge = new BRepLib_MakeEdge(C, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Elips E)
    {
        myMakeEdge = new BRepLib_MakeEdge(E);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Elips E, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(E, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Elips E, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge = new BRepLib_MakeEdge(E, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Elips E, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge = new BRepLib_MakeEdge(E, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Hypr H)
    {
        myMakeEdge = new BRepLib_MakeEdge(H);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Hypr H, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(H, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Hypr H, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge = new BRepLib_MakeEdge(H, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Hypr H, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge = new BRepLib_MakeEdge(H, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Parab P)
    {
        myMakeEdge = new BRepLib_MakeEdge(P);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Parab P, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(P, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Parab P, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge = new BRepLib_MakeEdge(P, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(in gp_Parab P, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge = new BRepLib_MakeEdge(P, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom_Curve L)
    {
        myMakeEdge = new BRepLib_MakeEdge(L);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom_Curve L, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom_Curve L, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom_Curve L, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom_Curve L, in gp_Pnt P1, in gp_Pnt P2, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, P1, P2, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom_Curve L, in TopoDS_Vertex V1, in TopoDS_Vertex V2, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, V1, V2, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom2d_Curve L, Geom_Surface S)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, S);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom2d_Curve L, Geom_Surface S, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, S, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom2d_Curve L, Geom_Surface S, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, S, P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom2d_Curve L, Geom_Surface S, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, S, V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom2d_Curve L, Geom_Surface S, in gp_Pnt P1, in gp_Pnt P2, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, S, P1, P2, p1, p2);
        TakeResult();
    }

    public BRepBuilderAPI_MakeEdge(Geom2d_Curve L, Geom_Surface S, in TopoDS_Vertex V1, in TopoDS_Vertex V2, double p1, double p2)
    {
        myMakeEdge = new BRepLib_MakeEdge(L, S, V1, V2, p1, p2);
        TakeResult();
    }

    public void Init(Geom_Curve C)
    {
        myMakeEdge.Init(C);
        TakeResult();
    }

    public void Init(Geom_Curve C, double p1, double p2)
    {
        myMakeEdge.Init(C, p1, p2);
        TakeResult();
    }

    public void Init(Geom_Curve C, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge.Init(C, P1, P2);
        TakeResult();
    }

    public void Init(Geom_Curve C, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge.Init(C, V1, V2);
        TakeResult();
    }

    public void Init(Geom_Curve C, in gp_Pnt P1, in gp_Pnt P2, double p1, double p2)
    {
        myMakeEdge.Init(C, P1, P2, p1, p2);
        TakeResult();
    }

    public void Init(Geom_Curve CC, in TopoDS_Vertex VV1, in TopoDS_Vertex VV2, double pp1, double pp2)
    {
        myMakeEdge.Init(CC, VV1, VV2, pp1, pp2);
        TakeResult();
    }

    public void Init(Geom2d_Curve C, Geom_Surface S)
    {
        myMakeEdge.Init(C, S);
        TakeResult();
    }

    public void Init(Geom2d_Curve C, Geom_Surface S, double p1, double p2)
    {
        myMakeEdge.Init(C, S, p1, p2);
        TakeResult();
    }

    public void Init(Geom2d_Curve C, Geom_Surface S, in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakeEdge.Init(C, S, P1, P2);
        TakeResult();
    }

    public void Init(Geom2d_Curve C, Geom_Surface S, in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakeEdge.Init(C, S, V1, V2);
        TakeResult();
    }

    public void Init(Geom2d_Curve C, Geom_Surface S, in gp_Pnt P1, in gp_Pnt P2, double p1, double p2)
    {
        myMakeEdge.Init(C, S, P1, P2, p1, p2);
        TakeResult();
    }

    public void Init(Geom2d_Curve CC, Geom_Surface S, in TopoDS_Vertex VV1, in TopoDS_Vertex VV2, double pp1, double pp2)
    {
        myMakeEdge.Init(CC, S, VV1, VV2, pp1, pp2);
        TakeResult();
    }


    public override bool IsDone()
    {
        return myMakeEdge.IsDone();
    }

    /// <summary>The construction status.</summary>
    public BRepBuilderAPI_EdgeError Error()
    {
        switch (myMakeEdge.Error())
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
    public TopoDS_Edge Edge()
    {
        return myMakeEdge.Edge();
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal TopoDS_Edge InternalEdge()
    {
        return myMakeEdge.InternalEdge();
    }

    /// <summary>The first vertex of the edge (frozen with the edge). May be null.</summary>
    public TopoDS_Vertex Vertex1()
    {
        return myMakeEdge.Vertex1();
    }

    /// <summary><see cref="Vertex1"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex1()
    {
        return myMakeEdge.InternalVertex1();
    }

    /// <summary>The second vertex of the edge (frozen with the edge). May be null.</summary>
    public TopoDS_Vertex Vertex2()
    {
        return myMakeEdge.Vertex2();
    }

    /// <summary><see cref="Vertex2"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalVertex2()
    {
        return myMakeEdge.InternalVertex2();
    }

    /// <summary>The edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Edge(BRepBuilderAPI_MakeEdge theMaker) => theMaker.Edge();
}
