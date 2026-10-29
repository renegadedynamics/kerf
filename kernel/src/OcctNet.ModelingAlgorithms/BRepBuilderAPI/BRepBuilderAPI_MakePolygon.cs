// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakePolygon.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepBuilderAPI/BRepBuilderAPI_MakePolygon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Builds a polygonal wire from points or vertices: a sequence of connected rectilinear edges. A point identical to the previous one adds no edge (see <see cref="Added"/>).</summary>
/// <remarks>
/// A wrapper of <see cref="BRepLib_MakePolygon"/>. The result is BRepLib's, read without freezing it; <see cref="BRepBuilderAPI_MakeShape.Shape"/>
/// and the typed accessors freeze it (design 4.2), and the internal accessors don't, for ported code.
/// </remarks>
public class BRepBuilderAPI_MakePolygon : BRepBuilderAPI_MakeShape
{
    private readonly BRepLib_MakePolygon myMakePolygon;

    // OCCT's "if (myMakePolygon.IsDone()) { Done(); myShape = myMakePolygon.Shape(); }" after each construction.
    private void TakeResult()
    {
        if (myMakePolygon.IsDone())
        {
            Done();
            myShape = myMakePolygon.InternalShape();
        }
    }

    public BRepBuilderAPI_MakePolygon()
    {
        myMakePolygon = new BRepLib_MakePolygon();
    }

    public BRepBuilderAPI_MakePolygon(in gp_Pnt P1, in gp_Pnt P2)
    {
        myMakePolygon = new BRepLib_MakePolygon(P1, P2);
        TakeResult();
    }

    public BRepBuilderAPI_MakePolygon(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3, bool Cl = false)
    {
        myMakePolygon = new BRepLib_MakePolygon(P1, P2, P3, Cl);
        TakeResult();
    }

    public BRepBuilderAPI_MakePolygon(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3, in gp_Pnt P4, bool Cl = false)
    {
        myMakePolygon = new BRepLib_MakePolygon(P1, P2, P3, P4, Cl);
        TakeResult();
    }

    public BRepBuilderAPI_MakePolygon(in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        myMakePolygon = new BRepLib_MakePolygon(V1, V2);
        TakeResult();
    }

    public BRepBuilderAPI_MakePolygon(in TopoDS_Vertex V1, in TopoDS_Vertex V2, in TopoDS_Vertex V3, bool Cl = false)
    {
        myMakePolygon = new BRepLib_MakePolygon(V1, V2, V3, Cl);
        TakeResult();
    }

    public BRepBuilderAPI_MakePolygon(in TopoDS_Vertex V1, in TopoDS_Vertex V2, in TopoDS_Vertex V3, in TopoDS_Vertex V4, bool Cl = false)
    {
        myMakePolygon = new BRepLib_MakePolygon(V1, V2, V3, V4, Cl);
        TakeResult();
    }



    /// <summary>Adds the point <paramref name="P"/> at the end of the polygon.</summary>
    public void Add(in gp_Pnt P)
    {
        myMakePolygon.Add(P);
        if (myMakePolygon.IsDone())
        {
            Done();
            if (!myMakePolygon.InternalLastVertex().IsNull()) // OCCT: LastVertex()
            {
                myShape = myMakePolygon.InternalShape();
            }
        }
    }

    /// <summary>Adds the vertex <paramref name="V"/> at the end of the polygon.</summary>
    public void Add(in TopoDS_Vertex V)
    {
        myMakePolygon.Add(V);
        if (myMakePolygon.IsDone())
        {
            Done();
            myShape = myMakePolygon.InternalShape();
        }
    }

    /// <summary>True if the last vertex added made an edge (it isn't at the previous one).</summary>
    public bool Added()
    {
        return myMakePolygon.Added();
    }

    /// <summary>Closes the polygon with an edge from its last vertex to its first.</summary>
    public void Close()
    {
        myMakePolygon.Close();
        myShape = myMakePolygon.InternalShape();
    }

    /// <summary>The first vertex of the polygon (frozen with it).</summary>
    public TopoDS_Vertex FirstVertex()
    {
        return myMakePolygon.FirstVertex();
    }

    /// <summary><see cref="FirstVertex"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalFirstVertex()
    {
        return myMakePolygon.InternalFirstVertex();
    }

    /// <summary>The last vertex of the polygon (frozen with it).</summary>
    public TopoDS_Vertex LastVertex()
    {
        return myMakePolygon.LastVertex();
    }

    /// <summary><see cref="LastVertex"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalLastVertex()
    {
        return myMakePolygon.InternalLastVertex();
    }

    public override bool IsDone()
    {
        return myMakePolygon.IsDone();
    }

    /// <summary>The last edge built (frozen with the polygon).</summary>
    public TopoDS_Edge Edge()
    {
        return myMakePolygon.Edge();
    }

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal TopoDS_Edge InternalEdge()
    {
        return myMakePolygon.InternalEdge();
    }

    /// <summary>The wire (frozen).</summary>
    public TopoDS_Wire Wire()
    {
        return myMakePolygon.Wire();
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal TopoDS_Wire InternalWire()
    {
        return myMakePolygon.InternalWire();
    }

    /// <summary>The last edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Edge(BRepBuilderAPI_MakePolygon theMaker) => theMaker.Edge();

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Wire(BRepBuilderAPI_MakePolygon theMaker) => theMaker.Wire();
}
