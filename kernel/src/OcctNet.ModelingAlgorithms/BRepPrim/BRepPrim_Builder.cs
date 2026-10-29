// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.lxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>Implements the building methods of the primitives over a <see cref="BRep_Builder"/>.</summary>
/// <remarks>Internal, as BRepPrim (decision 3201): it builds unfrozen shapes for the primitives' algorithms.</remarks>
internal sealed class BRepPrim_Builder
{
    private readonly BRep_Builder myBuilder;

    /// <summary>Creates an empty, useless Builder. Necessary for compilation.</summary>
    public BRepPrim_Builder()
    {
        myBuilder = new BRep_Builder();
    }

    /// <summary>Creates from a Builder.</summary>
    public BRepPrim_Builder(BRep_Builder B)
    {
        myBuilder = B;
    }

    public BRep_Builder Builder() => myBuilder;

    /// <summary>Makes an empty Shell.</summary>
    public void MakeShell(ref TopoDS_Shell S)
    {
        myBuilder.MakeShell(ref S);
        S.Closed(true);
    }

    /// <summary>Returns in <paramref name="F"/> a Face built with the plane equation <paramref name="P"/>.</summary>
    public void MakeFace(ref TopoDS_Face F, in gp_Pln P)
    {
        myBuilder.MakeFace(ref F, new Geom_Plane(P), Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="W"/> an empty Wire.</summary>
    public void MakeWire(ref TopoDS_Wire W)
    {
        myBuilder.MakeWire(ref W);
    }

    /// <summary>Returns in <paramref name="E"/> a degenerated edge.</summary>
    public void MakeDegeneratedEdge(ref TopoDS_Edge E)
    {
        myBuilder.MakeEdge(ref E);
        myBuilder.Degenerated(E, true);
    }

    /// <summary>Returns in <paramref name="E"/> an Edge built with the line equation <paramref name="L"/>.</summary>
    public void MakeEdge(ref TopoDS_Edge E, in gp_Lin L)
    {
        myBuilder.MakeEdge(ref E, new Geom_Line(L), Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="E"/> an Edge built with the circle equation <paramref name="C"/>.</summary>
    public void MakeEdge(ref TopoDS_Edge E, in gp_Circ C)
    {
        myBuilder.MakeEdge(ref E, new Geom_Circle(C), Precision.Confusion());
    }

    /// <summary>Sets the line <paramref name="L"/> as the pcurve of <paramref name="E"/> on the surface of <paramref name="F"/>.</summary>
    public void SetPCurve(in TopoDS_Edge E, in TopoDS_Face F, in gp_Lin2d L)
    {
        myBuilder.UpdateEdge(E, new Geom2d_Line(L), F, Precision.Confusion());
    }

    /// <summary>
    /// Sets the lines <paramref name="L1"/>, <paramref name="L2"/> as the pcurves of <paramref name="E"/> on the closed
    /// surface of <paramref name="F"/>.
    /// </summary>
    public void SetPCurve(in TopoDS_Edge E, in TopoDS_Face F, in gp_Lin2d L1, in gp_Lin2d L2)
    {
        TopoDS_Shape aLocalShape = E.Oriented(TopAbs_FORWARD);
        myBuilder.UpdateEdge(TopoDS.Edge(aLocalShape),
                             new Geom2d_Line(L1),
                             new Geom2d_Line(L2),
                             F,
                             Precision.Confusion());
        myBuilder.Continuity(E, F, F, GeomAbs_CN);
    }

    /// <summary>Sets the circle <paramref name="C"/> as the pcurve of <paramref name="E"/> on the surface of <paramref name="F"/>.</summary>
    public void SetPCurve(in TopoDS_Edge E, in TopoDS_Face F, in gp_Circ2d C)
    {
        myBuilder.UpdateEdge(E, new Geom2d_Circle(C), F, Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="V"/> a Vertex built with the point <paramref name="P"/>.</summary>
    public void MakeVertex(ref TopoDS_Vertex V, in gp_Pnt P)
    {
        myBuilder.MakeVertex(ref V, P, Precision.Confusion());
    }

    /// <summary>Reverses the Face <paramref name="F"/>.</summary>
    public void ReverseFace(ref TopoDS_Face F)
    {
        F.Reverse();
    }

    /// <summary>
    /// Adds the Vertex <paramref name="V"/> at the parameter <paramref name="P"/> in the Edge <paramref name="E"/>,
    /// reversed if not <paramref name="direct"/>.
    /// </summary>
    public void AddEdgeVertex(in TopoDS_Edge E, in TopoDS_Vertex V, double P, bool direct)
    {
        TopoDS_Vertex VV = V;
        if (!direct)
        {
            VV.Reverse();
        }
        myBuilder.Add(E, VV);
        myBuilder.UpdateVertex(VV, P, E, Precision.Confusion());
    }

    /// <summary>
    /// Adds the Vertex <paramref name="V"/> at both ends <paramref name="P1"/>, <paramref name="P2"/> of the closed Edge
    /// <paramref name="E"/>.
    /// </summary>
    public void AddEdgeVertex(in TopoDS_Edge E, in TopoDS_Vertex V, double P1, double P2)
    {
        TopoDS_Vertex VV = V;
        VV.Orientation(TopAbs_FORWARD);
        myBuilder.Add(E, VV);
        VV.Orientation(TopAbs_REVERSED);
        myBuilder.Add(E, VV);
        myBuilder.Range(E, P1, P2);
    }

    /// <summary><paramref name="P1"/>, <paramref name="P2"/> are the parameters of the vertex on the closed edge.</summary>
    public void SetParameters(in TopoDS_Edge E, in TopoDS_Vertex V, double P1, double P2)
    {
        myBuilder.Range(E, P1, P2);
    }

    /// <summary>Adds the Edge <paramref name="E"/> in the Wire <paramref name="W"/>, reversed if not <paramref name="direct"/>.</summary>
    public void AddWireEdge(in TopoDS_Wire W, in TopoDS_Edge E, bool direct)
    {
        TopoDS_Edge EE = E;
        if (!direct)
        {
            EE.Reverse();
        }
        myBuilder.Add(W, EE);
    }

    /// <summary>Adds the Wire <paramref name="W"/> in the Face <paramref name="F"/>.</summary>
    public void AddFaceWire(in TopoDS_Face F, in TopoDS_Wire W)
    {
        myBuilder.Add(F, W);
    }

    /// <summary>Adds the Face <paramref name="F"/> in the Shell <paramref name="S"/>.</summary>
    public void AddShellFace(in TopoDS_Shell S, in TopoDS_Face F)
    {
        myBuilder.Add(S, F);
    }

    /// <summary>Called once an edge is completed.</summary>
    public void CompleteEdge(in TopoDS_Edge E)
    {
        BRepTools.Update(E);
    }

    /// <summary>Called once a wire is completed.</summary>
    public void CompleteWire(in TopoDS_Wire W)
    {
        W.Closed(BRep_Tool.IsClosed(W));
        BRepTools.Update(W);
    }

    /// <summary>Called once a face is completed.</summary>
    public void CompleteFace(in TopoDS_Face F)
    {
        BRepTools.Update(F);
    }

    /// <summary>Called once a shell is completed.</summary>
    public void CompleteShell(in TopoDS_Shell S)
    {
        S.Closed(BRep_Tool.IsClosed(S));
        BRepTools.Update(S);
    }
}
