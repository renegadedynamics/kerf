// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Translation.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Translation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The geometry of a linear sweep (a prism): lines, extruded surfaces and translated caps.</summary>
internal sealed class BRepSweep_Translation : BRepSweep_Trsf
{
    private readonly Vector3D myVec;
    private readonly bool myCanonize;

    private static void SetThePCurve(BRep_Builder B, in TopoDS_Edge E, in TopoDS_Face F, TopAbs_Orientation O, Geom2d_Curve C)
    {
        // check if there is already a pcurve on non planar faces
        double f, l;
        Geom2d_Curve? OC = null;
        var GP = BRep_Tool.Surface(F, out Placement _) as Geom_Plane;
        if (GP is null)
        {
            OC = BRep_Tool.CurveOnSurface(E, F, out f, out l);
        }
        if (OC is null)
        {
            B.UpdateEdge(E, C, F, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            if (O == TopAbs_REVERSED)
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
    public BRepSweep_Translation(in TopoDS_Shape S, in Sweep_NumShape N, in Placement L, in Vector3D V, bool C, bool Canonize = true)
        : base(new BRep_Builder(), S, N, L, C)
    {
        myVec = V;
        myCanonize = Canonize;
        Standard_ConstructionError_Raise_if(V.Magnitude() < global::Kerf.Engine.Foundation.Precision.Confusion(), "BRepSweep_Translation::Constructor");
        Init();
    }

    public override TopoDS_Shape MakeEmptyVertex(in TopoDS_Shape aGenV, in Sweep_NumShape aDirV)
    {
        // Only called when the option of construction is with copy.
        Standard_ConstructionError_Raise_if(!myCopy, "BRepSweep_Translation::MakeEmptyVertex");
        Point3D P = BRep_Tool.Pnt(TopoDS.Vertex(aGenV));
        if (aDirV.Index() == 2)
        {
            P.Transform(myLocation.Transformation());
        }
        TopoDS_Vertex V = new();
        ////// modified by jgv, 5.10.01, for buc61008 //////
        // myBuilder.Builder().MakeVertex(V,P,Precision::Confusion());
        myBuilder.Builder().MakeVertex(ref V, P, BRep_Tool.Tolerance(TopoDS.Vertex(aGenV)));
        ////////////////////////////////////////////////////
        return V;
    }

    public override TopoDS_Shape MakeEmptyDirectingEdge(in TopoDS_Shape aGenV, in Sweep_NumShape aDirE)
    {
        Point3D P = BRep_Tool.Pnt(TopoDS.Vertex(aGenV));
        var L = new Line3D(P, new Direction3D(myVec));
        var GL = new Geom_Line(L);
        TopoDS_Edge E = new();
        myBuilder.Builder().MakeEdge(ref E, GL, BRep_Tool.Tolerance(TopoDS.Vertex(aGenV)));
        return E;
    }

    public override TopoDS_Shape MakeEmptyGeneratingEdge(in TopoDS_Shape aGenE, in Sweep_NumShape aDirV)
    {
        // Call only in case of construction with copy.
        Standard_ConstructionError_Raise_if(!myCopy, "BRepSweep_Translation::MakeEmptyVertex");
        TopoDS_Edge newE = new();
        if (BRep_Tool.Degenerated(TopoDS.Edge(aGenE)))
        {
            myBuilder.Builder().MakeEdge(ref newE);
            myBuilder.Builder().UpdateEdge(newE, BRep_Tool.Tolerance(TopoDS.Edge(aGenE)));
            myBuilder.Builder().Degenerated(newE, true);
        }
        else
        {
            Geom_Curve? C = BRep_Tool.Curve(TopoDS.Edge(aGenE), out Placement L, out double First, out double Last);
            if (C is not null)
            {
                C = (Geom_Curve)C.Copy();
                C.Transform(L.Transformation());
                if (aDirV.Index() == 2)
                {
                    C.Transform(myLocation.Transformation());
                }
            }
            myBuilder.Builder().MakeEdge(ref newE, C, BRep_Tool.Tolerance(TopoDS.Edge(aGenE)));
        }
        return newE;
    }

    public override void SetParameters(in TopoDS_Shape aNewFace, ref TopoDS_Shape aNewVertex, in TopoDS_Shape aGenF,
                                       in TopoDS_Shape aGenV, in Sweep_NumShape aDirV)
    {
        // Glue the parameter of vertices directly included in cap faces.
        Point2D pnt2d = BRep_Tool.Parameters(TopoDS.Vertex(aGenV), TopoDS.Face(aGenF));
        myBuilder.Builder().UpdateVertex(TopoDS.Vertex(aNewVertex), pnt2d.X(), pnt2d.Y(), TopoDS.Face(aNewFace),
                                         global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetDirectingParameter(in TopoDS_Shape aNewEdge, ref TopoDS_Shape aNewVertex, in TopoDS_Shape aGenV,
                                               in Sweep_NumShape aDirE, in Sweep_NumShape aDirV)
    {
        double param = 0;
        if (aDirV.Index() == 2)
        {
            param = myVec.Magnitude();
        }
        // OCCT.NET: frozen. Built on demand by the history after the result is published, the edge holds its vertices
        myBuilder.Builder().UpdateVertexOfFrozen(TopoDS.Vertex(aNewVertex), param, TopoDS.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetGeneratingParameter(in TopoDS_Shape aNewEdge, ref TopoDS_Shape aNewVertex, in TopoDS_Shape aGenE,
                                                in TopoDS_Shape aGenV, in Sweep_NumShape aDirV)
    {
        TopoDS_Vertex vbid = TopoDS.Vertex(aNewVertex);
        vbid.Orientation(aGenV.Orientation());
        myBuilder.Builder().UpdateVertex(vbid, BRep_Tool.Parameter(TopoDS.Vertex(aGenV), TopoDS.Edge(aGenE)),
                                         TopoDS.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override TopoDS_Shape MakeEmptyFace(in TopoDS_Shape aGenS, in Sweep_NumShape aDirS)
    {
        double toler;
        TopoDS_Face F = new();
        Geom_Surface? S;
        if (myDirShapeTool.Type(aDirS) == TopAbs_EDGE)
        {
            Geom_Curve C = BRep_Tool.Curve(TopoDS.Edge(aGenS), out Placement L, out double First, out double Last)!;
            toler = BRep_Tool.Tolerance(TopoDS.Edge(aGenS));
            Transform3D Tr = L.Transformation();
            C = (Geom_Curve)C.Copy();
            // extruded surfaces are inverted correspondingly to the topology, so reverse.
            C.Transform(Tr);
            var D = new Direction3D(myVec);
            D.Reverse();
            if (myCanonize)
            {
                var HC = new GeomAdaptor_Curve(C, First, Last);
                var AS = new GeomAdaptor_SurfaceOfLinearExtrusion(HC, D);
                switch (AS.GetType())
                {
                    case Plane:
                        S = new Geom_Plane(AS.Plane());
                        break;
                    case SurfaceKind.Cylinder:
                        S = new Geom_CylindricalSurface(AS.Cylinder());
                        break;
                    default:
                        S = new Geom_SurfaceOfLinearExtrusion(C, D);
                        break;
                }
            }
            else
            {
                S = new Geom_SurfaceOfLinearExtrusion(C, D);
            }
        }
        else
        {
            S = BRep_Tool.Surface(TopoDS.Face(aGenS), out Placement L)!;
            toler = BRep_Tool.Tolerance(TopoDS.Face(aGenS));
            Transform3D Tr = L.Transformation();
            S = (Geom_Surface)S.Copy();
            S.Transform(Tr);
            if (aDirS.Index() == 2)
            {
                S.Translate(myVec);
            }
        }
        myBuilder.Builder().MakeFace(ref F, S, toler);
        return F;
    }

    public override void SetPCurve(in TopoDS_Shape aNewFace, ref TopoDS_Shape aNewEdge, in TopoDS_Shape aGenF,
                                   in TopoDS_Shape aGenE, in Sweep_NumShape aDirV, TopAbs_Orientation orien)
    {
        // Set on edges of cap faces the same pcurves as
        // edges of the generating face.
        bool isclosed = BRep_Tool.IsClosed(TopoDS.Edge(aGenE), TopoDS.Face(aGenF));
        if (isclosed)
        {
            TopoDS_Edge anE = TopoDS.Edge(aGenE.Oriented(TopAbs_FORWARD));
            Geom2d_Curve? aC1 = BRep_Tool.CurveOnSurface(anE, TopoDS.Face(aGenF), out _, out _);
            anE.Reverse();
            Geom2d_Curve? aC2 = BRep_Tool.CurveOnSurface(anE, TopoDS.Face(aGenF), out _, out _);
            myBuilder.Builder().UpdateEdge(TopoDS.Edge(aNewEdge), aC1, aC2, TopoDS.Face(aNewFace), global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
        else
        {
            myBuilder.Builder().UpdateEdge(TopoDS.Edge(aNewEdge),
                                           BRep_Tool.CurveOnSurface(TopoDS.Edge(aGenE), TopoDS.Face(aGenF), out _, out _),
                                           TopoDS.Face(aNewFace), global::Kerf.Engine.Foundation.Precision.PConfusion());
        }
    }

    public override void SetGeneratingPCurve(in TopoDS_Shape aNewFace, ref TopoDS_Shape aNewEdge, in TopoDS_Shape aGenE,
                                             in Sweep_NumShape aDirE, in Sweep_NumShape aDirV, TopAbs_Orientation orien)
    {
        var AS = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(aNewFace), out Placement _)!);
        var L = new Line2D();
        TopoDS_Edge aNewOrientedEdge = TopoDS.Edge(aNewEdge);
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
            var GL = new Geom2d_Line(L);
            SetThePCurve(myBuilder.Builder(), TopoDS.Edge(aNewEdge), TopoDS.Face(aNewFace), orien, GL);
        }
    }

    public override void SetDirectingPCurve(in TopoDS_Shape aNewFace, ref TopoDS_Shape aNewEdge, in TopoDS_Shape aGenE,
                                            in TopoDS_Shape aGenV, in Sweep_NumShape aDirE, TopAbs_Orientation orien)
    {
        var AS = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(aNewFace), out Placement _)!);
        var L = new Line2D();
        if (AS.GetType() != Plane)
        {
            L.SetLocation(new Point2D(BRep_Tool.Parameter(TopoDS.Vertex(aGenV), TopoDS.Edge(aGenE)), 0));
            L.SetDirection(new Direction2D(Direction2D.D.NY));
            // JAG: OCCT's commented-out computation on planes
            var GL = new Geom2d_Line(L);
            SetThePCurve(myBuilder.Builder(), TopoDS.Edge(aNewEdge), TopoDS.Face(aNewFace), orien, GL);
        }
    }

    public override TopAbs_Orientation DirectSolid(in TopoDS_Shape aGenS, in Sweep_NumShape aDirS)
    {
        // compare the face normal and the direction
        var surf = new BRepAdaptor_Surface(TopoDS.Face(aGenS));
        surf.D1((surf.FirstUParameter() + surf.LastUParameter()) / 2.0,
                (surf.FirstVParameter() + surf.LastVParameter()) / 2.0, out Point3D _, out Vector3D du, out Vector3D dv);
        double x = myVec.DotCross(du, dv);
        TopAbs_Orientation orient = (x > 0) ? TopAbs_REVERSED : TopAbs_FORWARD;
        return orient;
    }

    public override bool GGDShapeIsToAdd(in TopoDS_Shape aNewShape, in TopoDS_Shape aNewSubShape, in TopoDS_Shape aGenS,
                                         in TopoDS_Shape aSubGenS, in Sweep_NumShape aDirS)
    {
        return true;
    }

    public override bool GDDShapeIsToAdd(in TopoDS_Shape aNewShape, in TopoDS_Shape aNewSubShape, in TopoDS_Shape aGenS,
                                         in Sweep_NumShape aDirS, in Sweep_NumShape aSubDirS)
    {
        return true;
    }

    public override bool SeparatedWires(in TopoDS_Shape aNewShape, in TopoDS_Shape aNewSubShape, in TopoDS_Shape aGenS,
                                        in TopoDS_Shape aSubGenS, in Sweep_NumShape aDirS)
    {
        return false;
    }

    public override bool HasShape(in TopoDS_Shape aGenS, in Sweep_NumShape aDirS)
    {
        if (myDirShapeTool.Type(aDirS) == TopAbs_EDGE)
        {
            if (myGenShapeTool.Type(aGenS) == TopAbs_EDGE)
            {
                TopoDS_Edge E = TopoDS.Edge(aGenS);
                // check if the edge is degenerated
                if (BRep_Tool.Degenerated(E))
                {
                    return false;
                }
                // check if the edge is a sewing edge
                //  modified by NIZHNY-EAP Fri Dec 24 11:13:09 1999 ___BEGIN___
                var FaceExp = new TopExp_Explorer(myGenShape, TopAbs_FACE);
                for (; FaceExp.More(); FaceExp.Next())
                {
                    TopoDS_Face F = TopoDS.Face(FaceExp.Current());
                    if (BRepTools.IsReallyClosed(E, F))
                    {
                        return false;
                    }
                }
                //  modified by NIZHNY-EAP Fri Dec 24 11:13:21 1999 ___END___
            }
        }
        return true;
    }

    public override bool IsInvariant(in TopoDS_Shape aGenS)
    {
        return false;
    }

    /// <summary>The vector of the translation.</summary>
    public Vector3D Vec() => myVec;
}
