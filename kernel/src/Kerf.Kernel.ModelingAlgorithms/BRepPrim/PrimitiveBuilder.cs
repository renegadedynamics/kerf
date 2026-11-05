// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.lxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepPrim/BRepPrim_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Primitives;

/// <summary>Implements the building methods of the primitives over a <see cref="BrepBuilder"/>.</summary>
/// <remarks>Internal, as BRepPrim (decision 465): it builds unfrozen shapes for the primitives' algorithms.</remarks>
internal sealed class PrimitiveBuilder
{
    private readonly BrepBuilder _builder;

    /// <summary>Creates an empty, useless Builder. Necessary for compilation.</summary>
    public PrimitiveBuilder()
    {
        _builder = new BrepBuilder();
    }

    /// <summary>Creates from a Builder.</summary>
    public PrimitiveBuilder(BrepBuilder b)
    {
        _builder = b;
    }

    public BrepBuilder Builder() => _builder;

    /// <summary>Makes an empty Shell.</summary>
    public void MakeShell(ref Shell s)
    {
        _builder.MakeShell(ref s);
        s.Closed(true);
    }

    /// <summary>Returns in <paramref name="f"/> a Face built with the plane equation <paramref name="p"/>.</summary>
    public void MakeFace(ref Face f, in Plane3D p)
    {
        _builder.MakeFace(ref f, new PlaneSurface(p), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="w"/> an empty Wire.</summary>
    public void MakeWire(ref Wire w)
    {
        _builder.MakeWire(ref w);
    }

    /// <summary>Returns in <paramref name="e"/> a degenerated edge.</summary>
    public void MakeDegeneratedEdge(ref Edge e)
    {
        _builder.MakeEdge(ref e);
        _builder.Degenerated(e, true);
    }

    /// <summary>Returns in <paramref name="e"/> an Edge built with the line equation <paramref name="l"/>.</summary>
    public void MakeEdge(ref Edge e, in Line3D l)
    {
        _builder.MakeEdge(ref e, new LineCurve3D(l), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="e"/> an Edge built with the circle equation <paramref name="c"/>.</summary>
    public void MakeEdge(ref Edge e, in Circle3D c)
    {
        _builder.MakeEdge(ref e, new CircleCurve3D(c), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Sets the line <paramref name="l"/> as the pcurve of <paramref name="e"/> on the surface of <paramref name="f"/>.</summary>
    public void SetPCurve(in Edge e, in Face f, in Line2D l)
    {
        _builder.UpdateEdge(e, new LineCurve2D(l), f, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>
    /// Sets the lines <paramref name="l1"/>, <paramref name="l2"/> as the pcurves of <paramref name="e"/> on the closed
    /// surface of <paramref name="f"/>.
    /// </summary>
    public void SetPCurve(in Edge e, in Face f, in Line2D l1, in Line2D l2)
    {
        Shape localShape = e.Oriented(Forward);
        _builder.UpdateEdge(ShapeCasts.Edge(localShape),
                             new LineCurve2D(l1),
                             new LineCurve2D(l2),
                             f,
                             global::Kerf.Engine.Foundation.Precision.Confusion());
        _builder.Continuity(e, f, f, CN);
    }

    /// <summary>Sets the circle <paramref name="c"/> as the pcurve of <paramref name="e"/> on the surface of <paramref name="f"/>.</summary>
    public void SetPCurve(in Edge e, in Face f, in Circle2D c)
    {
        _builder.UpdateEdge(e, new CircleCurve2D(c), f, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Returns in <paramref name="v"/> a Vertex built with the point <paramref name="p"/>.</summary>
    public void MakeVertex(ref Vertex v, in Point3D p)
    {
        _builder.MakeVertex(ref v, p, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Reverses the Face <paramref name="f"/>.</summary>
    public void ReverseFace(ref Face f)
    {
        f.Reverse();
    }

    /// <summary>
    /// Adds the Vertex <paramref name="v"/> at the parameter <paramref name="p"/> in the Edge <paramref name="e"/>,
    /// reversed if not <paramref name="direct"/>.
    /// </summary>
    public void AddEdgeVertex(in Edge e, in Vertex v, double p, bool direct)
    {
        Vertex vv = v;
        if (!direct)
        {
            vv.Reverse();
        }
        _builder.Add(e, vv);
        _builder.UpdateVertex(vv, p, e, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>
    /// Adds the Vertex <paramref name="v"/> at both ends <paramref name="p1"/>, <paramref name="p2"/> of the closed Edge
    /// <paramref name="e"/>.
    /// </summary>
    public void AddEdgeVertex(in Edge e, in Vertex v, double p1, double p2)
    {
        Vertex vv = v;
        vv.Orient(Forward);
        _builder.Add(e, vv);
        vv.Orient(Reversed);
        _builder.Add(e, vv);
        _builder.Range(e, p1, p2);
    }

    /// <summary><paramref name="p1"/>, <paramref name="p2"/> are the parameters of the vertex on the closed edge.</summary>
    public void SetParameters(in Edge e, in Vertex v, double p1, double p2)
    {
        _builder.Range(e, p1, p2);
    }

    /// <summary>Adds the Edge <paramref name="e"/> in the Wire <paramref name="w"/>, reversed if not <paramref name="direct"/>.</summary>
    public void AddWireEdge(in Wire w, in Edge e, bool direct)
    {
        Edge ee = e;
        if (!direct)
        {
            ee.Reverse();
        }
        _builder.Add(w, ee);
    }

    /// <summary>Adds the Wire <paramref name="w"/> in the Face <paramref name="f"/>.</summary>
    public void AddFaceWire(in Face f, in Wire w)
    {
        _builder.Add(f, w);
    }

    /// <summary>Adds the Face <paramref name="f"/> in the Shell <paramref name="s"/>.</summary>
    public void AddShellFace(in Shell s, in Face f)
    {
        _builder.Add(s, f);
    }

    /// <summary>Called once an edge is completed.</summary>
    public void CompleteEdge(in Edge e)
    {
        BrepTools.Update(e);
    }

    /// <summary>Called once a wire is completed.</summary>
    public void CompleteWire(in Wire w)
    {
        w.Closed(Tool.IsClosed(w));
        BrepTools.Update(w);
    }

    /// <summary>Called once a face is completed.</summary>
    public void CompleteFace(in Face f)
    {
        BrepTools.Update(f);
    }

    /// <summary>Called once a shell is completed.</summary>
    public void CompleteShell(in Shell s)
    {
        s.Closed(Tool.IsClosed(s));
        BrepTools.Update(s);
    }
}
