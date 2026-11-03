// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Translation.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Translation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The geometry of a linear sweep (a prism): lines, extruded surfaces and translated caps.</summary>
internal sealed class BRepSweep_Translation : BRepSweep_Trsf
{
    private readonly Vector3D myVec;
    private readonly bool myCanonize;

    private static void SetThePCurve(BrepBuilder B, in Edge E, in Face F, Orientation O, Curve2D C)
    {
        // check if there is already a pcurve on non planar faces
        double f, l;
        Curve2D? OC = null;
        var GP = Tool.Surface(F, out Placement _) as PlaneSurface;
        if (GP is null)
        {
            OC = Tool.CurveOnSurface(E, F, out f, out l);
        }
        if (OC is null)
        {
            B.UpdateEdge(E, C, F, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            if (O == Reversed)
            {
                B.UpdateEdge(E, OC, C, F, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                B.UpdateEdge(E, C, OC, F, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
    }

    /// <summary>
    /// The translation of <paramref name="S"/> along the directing edge <paramref name="N"/> by
    /// <paramref name="L"/> (the translation of <paramref name="V"/>); with <paramref name="C"/> the generating shape
    /// is copied; with <paramref name="Canonize"/> extrusions of lines and circles are planes and cylinders.
    /// </summary>
    public BRepSweep_Translation(in Shape S, in Sweep_NumShape N, in Placement L, in Vector3D V, bool C, bool Canonize = true)
        : base(new BrepBuilder(), S, N, L, C)
    {
        myVec = V;
        myCanonize = Canonize;
        Standard_ConstructionError_Raise_if(V.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion(), "BRepSweep_Translation::Constructor");
        Init();
    }

    public override Shape MakeEmptyVertex(in Shape aGenV, in Sweep_NumShape aDirV)
    {
        // Only called when the option of construction is with copy.
        Standard_ConstructionError_Raise_if(!myCopy, "BRepSweep_Translation::MakeEmptyVertex");
        Point3D P = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        if (aDirV.Index() == 2)
        {
            P.Transform(myLocation.Transformation());
        }
        Vertex V = new();
        ////// modified by jgv, 5.10.01, for buc61008 //////
        // myBuilder.Builder().MakeVertex(V,P,Precision::Confusion());
        myBuilder.Builder().MakeVertex(ref V, P, Tool.Tolerance(ShapeCasts.Vertex(aGenV)));
        ////////////////////////////////////////////////////
        return V;
    }

    public override Shape MakeEmptyDirectingEdge(in Shape aGenV, in Sweep_NumShape aDirE)
    {
        Point3D P = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        var L = new Line3D(P, new Direction3D(myVec));
        var GL = new LineCurve3D(L);
        Edge E = new();
        myBuilder.Builder().MakeEdge(ref E, GL, Tool.Tolerance(ShapeCasts.Vertex(aGenV)));
        return E;
    }

    public override Shape MakeEmptyGeneratingEdge(in Shape aGenE, in Sweep_NumShape aDirV)
    {
        // Call only in case of construction with copy.
        Standard_ConstructionError_Raise_if(!myCopy, "BRepSweep_Translation::MakeEmptyVertex");
        Edge newE = new();
        if (Tool.Degenerated(ShapeCasts.Edge(aGenE)))
        {
            myBuilder.Builder().MakeEdge(ref newE);
            myBuilder.Builder().UpdateEdge(newE, Tool.Tolerance(ShapeCasts.Edge(aGenE)));
            myBuilder.Builder().Degenerated(newE, true);
        }
        else
        {
            Curve3D? C = Tool.Curve(ShapeCasts.Edge(aGenE), out Placement L, out double First, out double Last);
            if (C is not null)
            {
                C = (Curve3D)C.Copy();
                C.Transform(L.Transformation());
                if (aDirV.Index() == 2)
                {
                    C.Transform(myLocation.Transformation());
                }
            }
            myBuilder.Builder().MakeEdge(ref newE, C, Tool.Tolerance(ShapeCasts.Edge(aGenE)));
        }
        return newE;
    }

    public override void SetParameters(in Shape aNewFace, ref Shape aNewVertex, in Shape aGenF,
                                       in Shape aGenV, in Sweep_NumShape aDirV)
    {
        // Glue the parameter of vertices directly included in cap faces.
        Point2D pnt2d = Tool.Parameters(ShapeCasts.Vertex(aGenV), ShapeCasts.Face(aGenF));
        myBuilder.Builder().UpdateVertex(ShapeCasts.Vertex(aNewVertex), pnt2d.X(), pnt2d.Y(), ShapeCasts.Face(aNewFace),
                                         global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetDirectingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenV,
                                               in Sweep_NumShape aDirE, in Sweep_NumShape aDirV)
    {
        double param = 0;
        if (aDirV.Index() == 2)
        {
            param = myVec.Magnitude();
        }
        // OCCT.NET: frozen. Built on demand by the history after the result is published, the edge holds its vertices
        myBuilder.Builder().UpdateVertexOfFrozen(ShapeCasts.Vertex(aNewVertex), param, ShapeCasts.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetGeneratingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenE,
                                                in Shape aGenV, in Sweep_NumShape aDirV)
    {
        Vertex vbid = ShapeCasts.Vertex(aNewVertex);
        vbid.Orientation(aGenV.Orientation());
        myBuilder.Builder().UpdateVertex(vbid, Tool.Parameter(ShapeCasts.Vertex(aGenV), ShapeCasts.Edge(aGenE)),
                                         ShapeCasts.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override Shape MakeEmptyFace(in Shape aGenS, in Sweep_NumShape aDirS)
    {
        double toler;
        Face F = new();
        Surface? S;
        if (myDirShapeTool.Type(aDirS) == ShapeType.Edge)
        {
            Curve3D C = Tool.Curve(ShapeCasts.Edge(aGenS), out Placement L, out double First, out double Last)!;
            toler = Tool.Tolerance(ShapeCasts.Edge(aGenS));
            Transform3D Tr = L.Transformation();
            C = (Curve3D)C.Copy();
            // extruded surfaces are inverted correspondingly to the topology, so reverse.
            C.Transform(Tr);
            var D = new Direction3D(myVec);
            D.Reverse();
            if (myCanonize)
            {
                var HC = new CurveAdaptor(C, First, Last);
                var AS = new SurfaceOfLinearExtrusion(HC, D);
                switch (AS.GetType())
                {
                    case Plane:
                        S = new PlaneSurface(AS.Plane());
                        break;
                    case SurfaceKind.Cylinder:
                        S = new CylindricalSurface(AS.Cylinder());
                        break;
                    default:
                        S = new ExtrusionSurface(C, D);
                        break;
                }
            }
            else
            {
                S = new ExtrusionSurface(C, D);
            }
        }
        else
        {
            S = Tool.Surface(ShapeCasts.Face(aGenS), out Placement L)!;
            toler = Tool.Tolerance(ShapeCasts.Face(aGenS));
            Transform3D Tr = L.Transformation();
            S = (Surface)S.Copy();
            S.Transform(Tr);
            if (aDirS.Index() == 2)
            {
                S.Translate(myVec);
            }
        }
        myBuilder.Builder().MakeFace(ref F, S, toler);
        return F;
    }

    public override void SetPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenF,
                                   in Shape aGenE, in Sweep_NumShape aDirV, Orientation orien)
    {
        // Set on edges of cap faces the same pcurves as
        // edges of the generating face.
        bool isclosed = Tool.IsClosed(ShapeCasts.Edge(aGenE), ShapeCasts.Face(aGenF));
        if (isclosed)
        {
            Edge anE = ShapeCasts.Edge(aGenE.Oriented(Forward));
            Curve2D? aC1 = Tool.CurveOnSurface(anE, ShapeCasts.Face(aGenF), out _, out _);
            anE.Reverse();
            Curve2D? aC2 = Tool.CurveOnSurface(anE, ShapeCasts.Face(aGenF), out _, out _);
            myBuilder.Builder().UpdateEdge(ShapeCasts.Edge(aNewEdge), aC1, aC2, ShapeCasts.Face(aNewFace), global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
        else
        {
            myBuilder.Builder().UpdateEdge(ShapeCasts.Edge(aNewEdge),
                                           Tool.CurveOnSurface(ShapeCasts.Edge(aGenE), ShapeCasts.Face(aGenF), out _, out _),
                                           ShapeCasts.Face(aNewFace), global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
    }

    public override void SetGeneratingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                             in Sweep_NumShape aDirE, in Sweep_NumShape aDirV, Orientation orien)
    {
        var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewFace), out Placement _)!);
        var L = new Line2D();
        Edge aNewOrientedEdge = ShapeCasts.Edge(aNewEdge);
        aNewOrientedEdge.Orientation(orien);
        if (AS.GetType() == Plane)
        {
            // nothing is done JAG (OCCT's commented-out computation of the line on the plane)
        }
        else
        {
            double v = 0;
            if (aDirV.Index() == 2)
            {
                v = -myVec.Magnitude();
            }
            L.SetLocation(new Point2D(0, v));
            L.SetDirection(new Direction2D(Direction2D.D.X));
            //  }
            var GL = new LineCurve2D(L);
            SetThePCurve(myBuilder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien, GL);
        }
    }

    public override void SetDirectingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                            in Shape aGenV, in Sweep_NumShape aDirE, Orientation orien)
    {
        var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewFace), out Placement _)!);
        var L = new Line2D();
        if (AS.GetType() != Plane)
        {
            L.SetLocation(new Point2D(Tool.Parameter(ShapeCasts.Vertex(aGenV), ShapeCasts.Edge(aGenE)), 0));
            L.SetDirection(new Direction2D(Direction2D.D.NY));
            // JAG: OCCT's commented-out computation on planes
            var GL = new LineCurve2D(L);
            SetThePCurve(myBuilder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien, GL);
        }
    }

    public override Orientation DirectSolid(in Shape aGenS, in Sweep_NumShape aDirS)
    {
        // compare the face normal and the direction
        var surf = new FaceSurfaceAdaptor(ShapeCasts.Face(aGenS));
        surf.D1((surf.FirstUParameter() + surf.LastUParameter()) / 2.0,
                (surf.FirstVParameter() + surf.LastVParameter()) / 2.0, out Point3D _, out Vector3D du, out Vector3D dv);
        double x = myVec.DotCross(du, dv);
        Orientation orient = (x > 0) ? Reversed : Forward;
        return orient;
    }

    public override bool GGDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Shape aSubGenS, in Sweep_NumShape aDirS)
    {
        return true;
    }

    public override bool GDDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Sweep_NumShape aDirS, in Sweep_NumShape aSubDirS)
    {
        return true;
    }

    public override bool SeparatedWires(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                        in Shape aSubGenS, in Sweep_NumShape aDirS)
    {
        return false;
    }

    public override bool HasShape(in Shape aGenS, in Sweep_NumShape aDirS)
    {
        if (myDirShapeTool.Type(aDirS) == ShapeType.Edge)
        {
            if (myGenShapeTool.Type(aGenS) == ShapeType.Edge)
            {
                Edge E = ShapeCasts.Edge(aGenS);
                // check if the edge is degenerated
                if (Tool.Degenerated(E))
                {
                    return false;
                }
                // check if the edge is a sewing edge
                //  modified by NIZHNY-EAP Fri Dec 24 11:13:09 1999 ___BEGIN___
                var FaceExp = new Explorer(myGenShape, ShapeType.Face);
                for (; FaceExp.More(); FaceExp.Next())
                {
                    Face F = ShapeCasts.Face(FaceExp.Current());
                    if (BrepTools.IsReallyClosed(E, F))
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
    public Vector3D Vec() => myVec;
}
