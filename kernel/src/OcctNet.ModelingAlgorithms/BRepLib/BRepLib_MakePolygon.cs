// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakePolygon.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakePolygon.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class BRepLib_MakePolygon : BRepLib_MakeShape
{
    private TopoDS_Vertex myFirstVertex = new();
    private TopoDS_Vertex myLastVertex = new();
    private TopoDS_Edge myEdge = new();

    /// <summary>Creates an empty polygon.</summary>
    public BRepLib_MakePolygon()
    {
    }

    public BRepLib_MakePolygon(in gp_Pnt P1, in gp_Pnt P2)
    {
        Add(P1);
        Add(P2);
    }

    public BRepLib_MakePolygon(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3, bool Cl = false)
    {
        Add(P1);
        Add(P2);
        Add(P3);
        if (Cl)
        {
            Close();
        }
    }

    public BRepLib_MakePolygon(in gp_Pnt P1, in gp_Pnt P2, in gp_Pnt P3, in gp_Pnt P4, bool Cl = false)
    {
        Add(P1);
        Add(P2);
        Add(P3);
        Add(P4);
        if (Cl)
        {
            Close();
        }
    }

    public BRepLib_MakePolygon(in TopoDS_Vertex V1, in TopoDS_Vertex V2)
    {
        Add(V1);
        Add(V2);
    }

    public BRepLib_MakePolygon(in TopoDS_Vertex V1, in TopoDS_Vertex V2, in TopoDS_Vertex V3, bool Cl = false)
    {
        Add(V1);
        Add(V2);
        Add(V3);
        if (Cl)
        {
            Close();
        }
    }

    public BRepLib_MakePolygon(in TopoDS_Vertex V1, in TopoDS_Vertex V2, in TopoDS_Vertex V3, in TopoDS_Vertex V4,
                               bool Cl = false)
    {
        Add(V1);
        Add(V2);
        Add(V3);
        Add(V4);
        if (Cl)
        {
            Close();
        }
    }

    public void Add(in gp_Pnt P)
    {
        var B = new BRep_Builder();
        TopoDS_Vertex V = new();
        B.MakeVertex(ref V, P, Precision.Confusion());
        Add(V);
    }

    public void Add(in TopoDS_Vertex V)
    {
        if (myFirstVertex.IsNull())
        {
            myFirstVertex = V;
        }
        else
        {
            myEdge.Nullify();
            var B = new BRep_Builder();
            TopoDS_Vertex last;
            bool second = myLastVertex.IsNull();
            if (second)
            {
                last = myFirstVertex;
                myLastVertex = V;
                TopoDS_Wire W = new();
                B.MakeWire(ref W);
                myShape = W;
                myShape.Closed(false);
                myShape.Orientable(true);
            }
            else
            {
                UnfreezeResult(); // OCCT.NET: frozen
                last = myLastVertex;
                if (BRepTools.Compare(V, myFirstVertex))
                {
                    myLastVertex = myFirstVertex;
                    myShape.Closed(true);
                }
                else
                {
                    myLastVertex = V;
                }
            }
            var ME = new BRepLib_MakeEdge(last, myLastVertex);
            if (ME.IsDone())
            {
                myEdge = ME.InternalEdge();
                B.Add(myShape, myEdge);
                Done();
            }
            else
            {
                // restore the previous last vertex
                if (second)
                {
                    myLastVertex.Nullify();
                }
                else
                {
                    myLastVertex = last;
                }
            }
        }
    }

    /// <summary>Returns true if the last vertex added was not rejected.</summary>
    public bool Added()
    {
        return !myEdge.IsNull();
    }

    /// <summary>Closes the polygon with an edge from the last vertex to the first, unless it is already closed.</summary>
    public void Close()
    {
        if (myFirstVertex.IsNull() || myLastVertex.IsNull())
        {
            return;
        }

        // check not already closed
        if (myShape.Closed())
        {
            return;
        }

        UnfreezeResult(); // OCCT.NET: frozen

        // build the last edge
        var B = new BRep_Builder();
        myEdge.Nullify();
        var ME = new BRepLib_MakeEdge(myLastVertex, myFirstVertex);
        if (ME.IsDone())
        {
            myEdge = ME.InternalEdge();
            B.Add(myShape, myEdge);
            myShape.Closed(true);
        }
    }

    /// <summary>The first vertex (frozen with the wire). May be null.</summary>
    public TopoDS_Vertex FirstVertex() => TopoDS.Vertex(Published(myFirstVertex));

    /// <summary><see cref="FirstVertex"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalFirstVertex() => myFirstVertex;

    /// <summary>The last vertex (frozen with the wire). May be null.</summary>
    public TopoDS_Vertex LastVertex() => TopoDS.Vertex(Published(myLastVertex));

    /// <summary><see cref="LastVertex"/> without freezing, for ported code.</summary>
    internal TopoDS_Vertex InternalLastVertex() => myLastVertex;

    /// <summary>The last edge added (frozen with the wire); null if the last vertex was rejected.</summary>
    public TopoDS_Edge Edge() => TopoDS.Edge(Published(myEdge));

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal TopoDS_Edge InternalEdge() => myEdge;

    /// <summary>The wire (frozen).</summary>
    public TopoDS_Wire Wire()
    {
        return TopoDS.Wire(Shape());
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal TopoDS_Wire InternalWire()
    {
        return TopoDS.Wire(InternalShape());
    }

    /// <summary>The last edge (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Edge(BRepLib_MakePolygon theMaker) => theMaker.Edge();

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator TopoDS_Wire(BRepLib_MakePolygon theMaker) => theMaker.Wire();
}
