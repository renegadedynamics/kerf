// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.lxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>Implements the building methods of the primitives over a <see cref="BrepBuilder"/>.</summary>
/// <remarks>Internal, as BRepPrim (decision 465): it builds unfrozen shapes for the primitives' algorithms.</remarks>
internal sealed class BRepPrim_Builder
{
    private readonly BrepBuilder myBuilder;

    /// <summary>Creates an empty, useless Builder. Necessary for compilation.</summary>
    public BRepPrim_Builder()
    {
        myBuilder = new BrepBuilder();
    }

    /// <summary>Creates from a Builder.</summary>
    public BRepPrim_Builder(BrepBuilder B)
    {
        myBuilder = B;
    }

    public BrepBuilder Builder() => myBuilder;

    /// <summary>Makes an empty Shell.</summary>
    public void MakeShell(ref Shell S)
    {
        myBuilder.MakeShell(ref S);
        S.Closed(true);
    }

    /// <summary>Returns in <paramref name="F"/> a Face built with the plane equation <paramref name="P"/>.</summary>
    public void MakeFace(ref Face F, in Plane3D P)
    {
        myBuilder.MakeFace(ref F, new PlaneSurface(P), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="W"/> an empty Wire.</summary>
    public void MakeWire(ref Wire W)
    {
        myBuilder.MakeWire(ref W);
    }

    /// <summary>Returns in <paramref name="E"/> a degenerated edge.</summary>
    public void MakeDegeneratedEdge(ref Edge E)
    {
        myBuilder.MakeEdge(ref E);
        myBuilder.Degenerated(E, true);
    }

    /// <summary>Returns in <paramref name="E"/> an Edge built with the line equation <paramref name="L"/>.</summary>
    public void MakeEdge(ref Edge E, in Line3D L)
    {
        myBuilder.MakeEdge(ref E, new LineCurve3D(L), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="E"/> an Edge built with the circle equation <paramref name="C"/>.</summary>
    public void MakeEdge(ref Edge E, in Circle3D C)
    {
        myBuilder.MakeEdge(ref E, new CircleCurve3D(C), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Sets the line <paramref name="L"/> as the pcurve of <paramref name="E"/> on the surface of <paramref name="F"/>.</summary>
    public void SetPCurve(in Edge E, in Face F, in Line2D L)
    {
        myBuilder.UpdateEdge(E, new LineCurve2D(L), F, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>
    /// Sets the lines <paramref name="L1"/>, <paramref name="L2"/> as the pcurves of <paramref name="E"/> on the closed
    /// surface of <paramref name="F"/>.
    /// </summary>
    public void SetPCurve(in Edge E, in Face F, in Line2D L1, in Line2D L2)
    {
        Shape aLocalShape = E.Oriented(Forward);
        myBuilder.UpdateEdge(ShapeCasts.Edge(aLocalShape),
                             new LineCurve2D(L1),
                             new LineCurve2D(L2),
                             F,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
        myBuilder.Continuity(E, F, F, CN);
    }

    /// <summary>Sets the circle <paramref name="C"/> as the pcurve of <paramref name="E"/> on the surface of <paramref name="F"/>.</summary>
    public void SetPCurve(in Edge E, in Face F, in Circle2D C)
    {
        myBuilder.UpdateEdge(E, new CircleCurve2D(C), F, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="V"/> a Vertex built with the point <paramref name="P"/>.</summary>
    public void MakeVertex(ref Vertex V, in Point3D P)
    {
        myBuilder.MakeVertex(ref V, P, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Reverses the Face <paramref name="F"/>.</summary>
    public void ReverseFace(ref Face F)
    {
        F.Reverse();
    }

    /// <summary>
    /// Adds the Vertex <paramref name="V"/> at the parameter <paramref name="P"/> in the Edge <paramref name="E"/>,
    /// reversed if not <paramref name="direct"/>.
    /// </summary>
    public void AddEdgeVertex(in Edge E, in Vertex V, double P, bool direct)
    {
        Vertex VV = V;
        if (!direct)
        {
            VV.Reverse();
        }
        myBuilder.Add(E, VV);
        myBuilder.UpdateVertex(VV, P, E, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>
    /// Adds the Vertex <paramref name="V"/> at both ends <paramref name="P1"/>, <paramref name="P2"/> of the closed Edge
    /// <paramref name="E"/>.
    /// </summary>
    public void AddEdgeVertex(in Edge E, in Vertex V, double P1, double P2)
    {
        Vertex VV = V;
        VV.Orientation(Forward);
        myBuilder.Add(E, VV);
        VV.Orientation(Reversed);
        myBuilder.Add(E, VV);
        myBuilder.Range(E, P1, P2);
    }

    /// <summary><paramref name="P1"/>, <paramref name="P2"/> are the parameters of the vertex on the closed edge.</summary>
    public void SetParameters(in Edge E, in Vertex V, double P1, double P2)
    {
        myBuilder.Range(E, P1, P2);
    }

    /// <summary>Adds the Edge <paramref name="E"/> in the Wire <paramref name="W"/>, reversed if not <paramref name="direct"/>.</summary>
    public void AddWireEdge(in Wire W, in Edge E, bool direct)
    {
        Edge EE = E;
        if (!direct)
        {
            EE.Reverse();
        }
        myBuilder.Add(W, EE);
    }

    /// <summary>Adds the Wire <paramref name="W"/> in the Face <paramref name="F"/>.</summary>
    public void AddFaceWire(in Face F, in Wire W)
    {
        myBuilder.Add(F, W);
    }

    /// <summary>Adds the Face <paramref name="F"/> in the Shell <paramref name="S"/>.</summary>
    public void AddShellFace(in Shell S, in Face F)
    {
        myBuilder.Add(S, F);
    }

    /// <summary>Called once an edge is completed.</summary>
    public void CompleteEdge(in Edge E)
    {
        BrepTools.Update(E);
    }

    /// <summary>Called once a wire is completed.</summary>
    public void CompleteWire(in Wire W)
    {
        W.Closed(Tool.IsClosed(W));
        BrepTools.Update(W);
    }

    /// <summary>Called once a face is completed.</summary>
    public void CompleteFace(in Face F)
    {
        BrepTools.Update(F);
    }

    /// <summary>Called once a shell is completed.</summary>
    public void CompleteShell(in Shell S)
    {
        S.Closed(Tool.IsClosed(S));
        BrepTools.Update(S);
    }
}
