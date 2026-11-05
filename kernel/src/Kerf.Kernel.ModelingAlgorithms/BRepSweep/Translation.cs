// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Translation.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Translation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>The geometry of a linear sweep (a prism): lines, extruded surfaces and translated caps.</summary>
internal sealed class Translation : Trsf
{
    private readonly Vector3D _vec;
    private readonly bool _canonize;

    private static void SetThePCurve(BrepBuilder b, in Edge e, in Face F, Orientation o, Curve2D c)
    {
        // check if there is already a pcurve on non planar faces
        double f, l;
        Curve2D? oc = null;
        var gp = Tool.Surface(F, out Placement _) as PlaneSurface;
        if (gp is null)
        {
            oc = Tool.CurveOnSurface(e, F, out f, out l);
        }
        if (oc is null)
        {
            b.UpdateEdge(e, c, F, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            if (o == Reversed)
            {
                b.UpdateEdge(e, oc, c, F, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                b.UpdateEdge(e, c, oc, F, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
    }

    /// <summary>
    /// The translation of <paramref name="s"/> along the directing edge <paramref name="n"/> by
    /// <paramref name="l"/> (the translation of <paramref name="v"/>); with <paramref name="c"/> the generating shape
    /// is copied; with <paramref name="canonize"/> extrusions of lines and circles are planes and cylinders.
    /// </summary>
    public Translation(in Shape s, in NumShape n, in Placement l, in Vector3D v, bool c, bool canonize = true)
        : base(new BrepBuilder(), s, n, l, c)
    {
        _vec = v;
        _canonize = canonize;
        Standard_ConstructionError_Raise_if(v.Magnitude < global::Kerf.Engine.Foundation.Precision.Confusion(), "BRepSweep_Translation::Constructor");
        Init();
    }

    public override Shape MakeEmptyVertex(in Shape aGenV, in NumShape aDirV)
    {
        // Only called when the option of construction is with copy.
        Standard_ConstructionError_Raise_if(!_copy, "BRepSweep_Translation::MakeEmptyVertex");
        Point3D p = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        if (aDirV.Index() == 2)
        {
            p = p.Transformed(_location.Transformation);
        }
        Vertex v = new();
        ////// modified by jgv, 5.10.01, for buc61008 //////
        // myBuilder.Builder().MakeVertex(V,P,Precision::Confusion());
        _builder.Builder().MakeVertex(ref v, p, Tool.Tolerance(ShapeCasts.Vertex(aGenV)));
        ////////////////////////////////////////////////////
        return v;
    }

    public override Shape MakeEmptyDirectingEdge(in Shape aGenV, in NumShape aDirE)
    {
        Point3D p = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        var l = new Line3D(p, new Direction3D(_vec));
        var gl = new LineCurve3D(l);
        Edge e = new();
        _builder.Builder().MakeEdge(ref e, gl, Tool.Tolerance(ShapeCasts.Vertex(aGenV)));
        return e;
    }

    public override Shape MakeEmptyGeneratingEdge(in Shape aGenE, in NumShape aDirV)
    {
        // Call only in case of construction with copy.
        Standard_ConstructionError_Raise_if(!_copy, "BRepSweep_Translation::MakeEmptyVertex");
        Edge newE = new();
        if (Tool.Degenerated(ShapeCasts.Edge(aGenE)))
        {
            _builder.Builder().MakeEdge(ref newE);
            _builder.Builder().UpdateEdge(newE, Tool.Tolerance(ShapeCasts.Edge(aGenE)));
            _builder.Builder().Degenerated(newE, true);
        }
        else
        {
            Curve3D? c = Tool.Curve(ShapeCasts.Edge(aGenE), out Placement l, out double first, out double last);
            if (c is not null)
            {
                c = (Curve3D)c.Copy();
                c.Transform(l.Transformation);
                if (aDirV.Index() == 2)
                {
                    c.Transform(_location.Transformation);
                }
            }
            _builder.Builder().MakeEdge(ref newE, c, Tool.Tolerance(ShapeCasts.Edge(aGenE)));
        }
        return newE;
    }

    public override void SetParameters(in Shape aNewFace, ref Shape aNewVertex, in Shape aGenF,
                                       in Shape aGenV, in NumShape aDirV)
    {
        // Glue the parameter of vertices directly included in cap faces.
        Point2D pnt2d = Tool.Parameters(ShapeCasts.Vertex(aGenV), ShapeCasts.Face(aGenF));
        _builder.Builder().UpdateVertex(ShapeCasts.Vertex(aNewVertex), pnt2d.X, pnt2d.Y, ShapeCasts.Face(aNewFace),
                                         global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetDirectingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenV,
                                               in NumShape aDirE, in NumShape aDirV)
    {
        double param = 0;
        if (aDirV.Index() == 2)
        {
            param = _vec.Magnitude;
        }
        _builder.Builder().UpdateVertex(ShapeCasts.Vertex(aNewVertex), param, ShapeCasts.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetGeneratingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenE,
                                                in Shape aGenV, in NumShape aDirV)
    {
        Vertex vbid = ShapeCasts.Vertex(aNewVertex);
        vbid.Orient(aGenV.Orientation);
        _builder.Builder().UpdateVertex(vbid, Tool.Parameter(ShapeCasts.Vertex(aGenV), ShapeCasts.Edge(aGenE)),
                                         ShapeCasts.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override Shape MakeEmptyFace(in Shape aGenS, in NumShape aDirS)
    {
        double toler;
        Face f = new();
        Surface? s;
        if (_dirShapeTool.Type(aDirS) == ShapeType.Edge)
        {
            Curve3D c = Tool.Curve(ShapeCasts.Edge(aGenS), out Placement l, out double first, out double last)!;
            toler = Tool.Tolerance(ShapeCasts.Edge(aGenS));
            Transform3D tr = l.Transformation;
            c = (Curve3D)c.Copy();
            // extruded surfaces are inverted correspondingly to the topology, so reverse.
            c.Transform(tr);
            var d = new Direction3D(_vec);
            d = d.Reversed();
            if (_canonize)
            {
                var hc = new CurveAdaptor(c, first, last);
                var AS = new SurfaceOfLinearExtrusion(hc, d);
                switch (AS.GetType())
                {
                    case Plane:
                        s = new PlaneSurface(AS.Plane());
                        break;
                    case SurfaceKind.Cylinder:
                        s = new CylindricalSurface(AS.Cylinder());
                        break;
                    default:
                        s = new ExtrusionSurface(c, d);
                        break;
                }
            }
            else
            {
                s = new ExtrusionSurface(c, d);
            }
        }
        else
        {
            s = Tool.Surface(ShapeCasts.Face(aGenS), out Placement l)!;
            toler = Tool.Tolerance(ShapeCasts.Face(aGenS));
            Transform3D tr = l.Transformation;
            s = (Surface)s.Copy();
            s.Transform(tr);
            if (aDirS.Index() == 2)
            {
                s.Translate(_vec);
            }
        }
        _builder.Builder().MakeFace(ref f, s, toler);
        return f;
    }

    public override void SetPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenF,
                                   in Shape aGenE, in NumShape aDirV, Orientation orien)
    {
        // Set on edges of cap faces the same pcurves as
        // edges of the generating face.
        bool isclosed = Tool.IsClosed(ShapeCasts.Edge(aGenE), ShapeCasts.Face(aGenF));
        if (isclosed)
        {
            Edge e = ShapeCasts.Edge(aGenE.Oriented(Forward));
            Curve2D? c1 = Tool.CurveOnSurface(e, ShapeCasts.Face(aGenF), out _, out _);
            e.Reverse();
            Curve2D? c2 = Tool.CurveOnSurface(e, ShapeCasts.Face(aGenF), out _, out _);
            _builder.Builder().UpdateEdge(ShapeCasts.Edge(aNewEdge), c1, c2, ShapeCasts.Face(aNewFace), global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
        else
        {
            _builder.Builder().UpdateEdge(ShapeCasts.Edge(aNewEdge),
                                           Tool.CurveOnSurface(ShapeCasts.Edge(aGenE), ShapeCasts.Face(aGenF), out _, out _),
                                           ShapeCasts.Face(aNewFace), global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
    }

    public override void SetGeneratingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                             in NumShape aDirE, in NumShape aDirV, Orientation orien)
    {
        var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewFace), out Placement _)!);
        var l = new Line2D();
        Edge newOrientedEdge = ShapeCasts.Edge(aNewEdge);
        newOrientedEdge.Orient(orien);
        if (AS.GetType() == Plane)
        {
            // nothing is done JAG (OCCT's commented-out computation of the line on the plane)
        }
        else
        {
            double v = 0;
            if (aDirV.Index() == 2)
            {
                v = -_vec.Magnitude;
            }
            l = l.WithLocation(new Point2D(0, v));
            l = l.WithDirection(new Direction2D(Direction2D.D.X));
            //  }
            var gl = new LineCurve2D(l);
            SetThePCurve(_builder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien, gl);
        }
    }

    public override void SetDirectingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                            in Shape aGenV, in NumShape aDirE, Orientation orien)
    {
        var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewFace), out Placement _)!);
        var l = new Line2D();
        if (AS.GetType() != Plane)
        {
            l = l.WithLocation(new Point2D(Tool.Parameter(ShapeCasts.Vertex(aGenV), ShapeCasts.Edge(aGenE)), 0));
            l = l.WithDirection(new Direction2D(Direction2D.D.NY));
            // JAG: OCCT's commented-out computation on planes
            var gl = new LineCurve2D(l);
            SetThePCurve(_builder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien, gl);
        }
    }

    public override Orientation DirectSolid(in Shape aGenS, in NumShape aDirS)
    {
        // compare the face normal and the direction
        var surf = new FaceSurfaceAdaptor(ShapeCasts.Face(aGenS));
        surf.D1((surf.FirstUParameter() + surf.LastUParameter()) / 2.0,
                (surf.FirstVParameter() + surf.LastVParameter()) / 2.0, out Point3D _, out Vector3D du, out Vector3D dv);
        double x = _vec.DotCross(du, dv);
        Orientation orient = (x > 0) ? Reversed : Forward;
        return orient;
    }

    public override bool GGDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Shape aSubGenS, in NumShape aDirS)
    {
        return true;
    }

    public override bool GDDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in NumShape aDirS, in NumShape aSubDirS)
    {
        return true;
    }

    public override bool SeparatedWires(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                        in Shape aSubGenS, in NumShape aDirS)
    {
        return false;
    }

    public override bool HasShape(in Shape aGenS, in NumShape aDirS)
    {
        if (_dirShapeTool.Type(aDirS) == ShapeType.Edge)
        {
            if (_genShapeTool.Type(aGenS) == ShapeType.Edge)
            {
                Edge e = ShapeCasts.Edge(aGenS);
                // check if the edge is degenerated
                if (Tool.Degenerated(e))
                {
                    return false;
                }
                // check if the edge is a sewing edge
                //  modified by NIZHNY-EAP Fri Dec 24 11:13:09 1999 ___BEGIN___
                var faceExp = new Explorer(_genShape, ShapeType.Face);
                for (; faceExp.More(); faceExp.Next())
                {
                    Face f = ShapeCasts.Face(faceExp.Current());
                    if (BrepTools.IsReallyClosed(e, f))
                    {
                        return false;
                    }
                }
                //  modified by NIZHNY-EAP Fri Dec 24 11:13:21 1999 ___END___
            }
        }
        return true;
    }

    public override bool IsInvariant(in Shape aGenS)
    {
        return false;
    }

    /// <summary>The vector of the translation.</summary>
    public Vector3D Vec() => _vec;
}
