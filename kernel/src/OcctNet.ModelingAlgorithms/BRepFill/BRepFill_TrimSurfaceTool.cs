// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimSurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimSurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Computes the intersection of two neighbouring faces of an evolved shape along a bisector of the spine:
/// parameters of the intersections with edges, and the projection of the bisector onto both faces.
/// </summary>
/// <remarks>
/// A working object of BRepFill_Evolved (decision 21). As OCCT's, <see cref="IntersectWith"/> can lengthen the
/// bisector it was given in place (by the intersection tolerance at each end) when no intersection is found: the
/// bisector must be a Geom2d_TrimmedCurve the caller owns, unfrozen (BRepFill_Evolved's copies are).
/// </remarks>
public sealed class BRepFill_TrimSurfaceTool
{
    private readonly TopoDS_Face myFace1;
    private readonly TopoDS_Face myFace2;
    private readonly TopoDS_Edge myEdge1;
    private readonly TopoDS_Edge myEdge2;
    private readonly bool myInv1;
    private readonly bool myInv2;
    private readonly Geom2d_Curve myBis;

    /// <summary>
    /// Initialisation with two neighbour faces. Edge1 and Edge2 are the parallel edges corresponding to the minimum
    /// iso on Face1 and Face2 (Edge1 is Umin or Vmin on Face1); Inv1 and Inv2 say whether they are reversed.
    /// </summary>
    public BRepFill_TrimSurfaceTool(Geom2d_Curve Bis,
                                    in TopoDS_Face Face1,
                                    in TopoDS_Face Face2,
                                    in TopoDS_Edge Edge1,
                                    in TopoDS_Edge Edge2,
                                    bool Inv1,
                                    bool Inv2)
    {
        myFace1 = Face1;
        myFace2 = Face2;
        myEdge1 = Edge1;
        myEdge2 = Edge2;
        myInv1 = Inv1;
        myInv2 = Inv2;
        myBis = Bis;
    }

    // Order the sequence of points by increasing x.
    private static void Bubble(NCollection_Sequence<gp_Pnt> Seq)
    {
        bool Invert = true;
        int NbPoints = Seq.Length();
        while (Invert)
        {
            Invert = false;
            for (int i = 1; i < NbPoints; i++)
            {
                gp_Pnt P1 = Seq.Value(i);
                gp_Pnt P2 = Seq.Value(i + 1);
                if (P2.X() < P1.X())
                {
                    Seq.Exchange(i, i + 1);
                    Invert = true;
                }
            }
        }
    }

    private static double EvalPhase(in TopoDS_Edge Edge, in TopoDS_Face Face, GeomAdaptor_Surface GAS, in gp_Ax3 Axis)
    {
        gp_Pnt2d PF1, PF2;
        double VDeg;
        double V = 0.0;
        BRep_Tool.UVPoints(Edge, Face, out gp_Pnt2d PE1, out gp_Pnt2d _);
        VDeg = PE1.Y();
        TopExp_Explorer Exp = new(Face, TopAbs_EDGE);
        for (; Exp.More(); Exp.Next())
        {
            if (!TopoDS.Edge(Exp.Current()).IsSame(Edge))
            {
                BRep_Tool.UVPoints(TopoDS.Edge(Exp.Current()), Face, out PF1, out PF2);
                V = (Math.Abs(PF1.Y() - VDeg) > Math.Abs(PF2.Y() - VDeg)) ? PF1.Y() : PF2.Y();
                break;
            }
        }
        gp_Pnt P = GAS.Value(0.0, V);

        if (new gp_Vec(Axis.Location(), P).Dot(new gp_Vec(Axis.XDirection())) < 0.0)
        {
            return Math.PI;
        }
        else
        {
            return 0.0;
        }
    }

    private static void EvalParameters(in TopoDS_Edge Edge, in TopoDS_Face Face, Geom2d_Curve Bis, NCollection_Sequence<gp_Pnt> Seq)
    {
        bool Degener = BRep_Tool.Degenerated(Edge);
        // return curves 3d associated to edges.
        Geom_TrimmedCurve CT;
        Geom_Plane Plane = new(0, 0, 1, 0);

        Geom2dInt_GInter Intersector;

        int NbPoints, NbSegments;
        double U1, U2;
        gp_Pnt P;

        //  modified by NIZHNY-EAP Wed Dec 22 15:00:51 1999 ___BEGIN___
        double Tol = 1.0e-6; // BRepFill_Precision();
        double TolC = 0.0;

        if (!Degener)
        {
            Geom_Curve C = BRep_Tool.Curve(Edge, out TopLoc_Location L, out double f, out double l)!;
            CT = new Geom_TrimmedCurve(C, f, l);
            CT.Transform(L.Transformation());
            // projection of 3d curves in the plane xOy
            Geom2d_Curve C2d = GeomProjLib.Curve2d(CT, Plane)!;

            Geom2dAdaptor_Curve AC = new(C2d);
            Geom2dAdaptor_Curve ABis = new(Bis);

            Intersector = new Geom2dInt_GInter(ABis, AC, TolC, Tol);

            if (!Intersector.IsDone())
            {
                throw new StdFail_NotDone("BRepFill_TrimSurfaceTool::IntersectWith");
            }

            NbPoints = Intersector.NbPoints();

            if (NbPoints < 1)
            {
                // try to elongate curves and enlarge tolerance

                // don't do it rightaway from the beginning in order not to get
                // extra solutions those would cause *Exception*: incoherent intersection

                GeomAbs_CurveType CType = AC.GetType(), BisType = ABis.GetType();
                bool canElongateC = CType != GeomAbs_BezierCurve && CType != GeomAbs_BSplineCurve
                                    && CType != GeomAbs_OffsetCurve && CType != GeomAbs_OtherCurve;
                bool canElongateBis = BisType != GeomAbs_BezierCurve && BisType != GeomAbs_BSplineCurve
                                      && BisType != GeomAbs_OffsetCurve && BisType != GeomAbs_OtherCurve;

                // OCCT: the down-casts are not checked (a curve that isn't trimmed crashes natively)
                Geom2d_TrimmedCurve TBis = (Bis as Geom2d_TrimmedCurve)!;
                Geom2d_TrimmedCurve TC2d = (C2d as Geom2d_TrimmedCurve)!;

                if (canElongateC)
                {
                    TC2d.SetTrim(TC2d.FirstParameter() - Tol, TC2d.LastParameter() + Tol);
                    AC.Load(TC2d);
                }
                if (canElongateBis)
                {
                    // OCCT: lengthens the caller's bisector in place, so that later calls see it lengthened
                    TBis.SetTrim(TBis.FirstParameter() - Tol, TBis.LastParameter() + Tol);
                    ABis.Load(TBis);
                }
                Intersector = new Geom2dInt_GInter(ABis, AC, TolC, Tol * 10);

                if (!Intersector.IsDone())
                {
                    throw new StdFail_NotDone("BRepFill_TrimSurfaceTool::IntersectWith");
                }

                NbPoints = Intersector.NbPoints();
            }
            //  modified by NIZHNY-EAP Wed Dec 22 15:00:56 1999 ___END___
            if (NbPoints > 0)
            {
                for (int i = 1; i <= NbPoints; i++)
                {
                    U1 = Intersector.Point(i).ParamOnFirst();
                    U2 = Intersector.Point(i).ParamOnSecond();
                    P = new gp_Pnt(U1, U2, 0.0);
                    Seq.Append(P);
                }
            }

            NbSegments = Intersector.NbSegments();

            if (NbSegments > 0)
            {
                IntRes2d_IntersectionSegment Seg;
                for (int i = 1; i <= NbSegments; i++)
                {
                    Seg = Intersector.Segment(i);
                    U1 = Seg.FirstPoint().ParamOnFirst();
                    U1 += Seg.LastPoint().ParamOnFirst();
                    U1 /= 2.0;
                    U2 = Seg.FirstPoint().ParamOnSecond();
                    U2 += Seg.LastPoint().ParamOnSecond();
                    U2 /= 2.0;
                    P = new gp_Pnt(U1, U2, 0.0);
                    Seq.Append(P);
                }
            }
            // Order the sequence by increasing parameter on the bissectrice.
            Bubble(Seq);

            //  modified by NIZHNY-EAP Fri Dec 24 18:47:24 1999 ___BEGIN___
            // Remove double points
            gp_Pnt P1, P2;
            for (int i = 1; i < NbPoints; i++)
            {
                P1 = Seq.Value(i);
                P2 = Seq.Value(i + 1);
                if (P2.X() - P1.X() < Tol)
                {
                    Seq.Remove(i--);
                    NbPoints--;
                }
            }
            //  modified by NIZHNY-EAP Fri Dec 24 18:47:28 1999 ___END___
        }
        else
        {
            // the edge is degenerated : the point and it is found if it is
            // on the bissectrice.

            gp_Pnt P3d = BRep_Tool.Pnt(TopExp.FirstVertex(Edge));
            gp_Pnt2d P2d = new(P3d.X(), P3d.Y());

            double UBis = Bis.FirstParameter();
            gp_Pnt2d PBis = Bis.Value(UBis);

            //  modified by NIZHNY-EAP Wed Jan 12 11:41:30 2000 ___BEGIN___
            // inside gp_Pnt2d::Distance
            // Infinite * Infinite => Exception: DefaultNumericError
            // Case encounered: UBis < Precision::Infinite()
            // but PBis.X() > Precision::Infinite()
            if (Precision.IsPositiveInfinite(Math.Abs(PBis.X()))
                || Precision.IsPositiveInfinite(Math.Abs(PBis.Y())) || PBis.Distance(P2d) > Tol)
            {
                //  modified by NIZHNY-EAP Wed Jan 12 11:41:40 2000 ___END___
                UBis = Bis.LastParameter();
                if (UBis >= Precision.Infinite())
                {
                    return;
                }
                PBis = Bis.Value(UBis);
                if (PBis.Distance(P2d) > Tol)
                {
                    return;
                }
            }

            // evaluate parameter intersection.
            Geom_Surface GS = BRep_Tool.Surface(Face)!;
            GeomAdaptor_Surface GAS = new(GS);

            gp_Ax3 Axis;
            double Phase = 0.0;

            switch (GAS.GetType())
            {
                case GeomAbs_Sphere:
                    Axis = GAS.Sphere().Position();
                    break;
                case GeomAbs_Cone:
                    {
                        //----------------------------------------------------------
                        // if myFace1 is not at the same side of the apex as the point
                        // of parameter 0 0 on the cone => phase = M_PI.
                        //----------------------------------------------------------
                        Axis = GAS.Cone().Position();
                        Phase = EvalPhase(Edge, Face, GAS, Axis);
                        break;
                    }
                case GeomAbs_Torus:
                    Axis = GAS.Torus().Position();
                    break;
                case GeomAbs_Cylinder:
                    Axis = GAS.Cylinder().Position();
                    break;
                case GeomAbs_SurfaceOfRevolution:
                    {
                        //----------------------------------------------------------
                        // if myFace1 is not at the same side of the apex as the point
                        // of parameter 0 0 on the cone => phase = M_PI.
                        //----------------------------------------------------------
                        Geom_SurfaceOfRevolution GSRev = (Geom_SurfaceOfRevolution)GS;
                        GeomAdaptor_Curve HC = new(GSRev.BasisCurve());
                        GeomAdaptor_SurfaceOfRevolution ASRev = new(HC, GAS.AxeOfRevolution());
                        Axis = ASRev.Axis();
                        Phase = EvalPhase(Edge, Face, GAS, Axis);
                        break;
                    }
                default:
                    throw new Standard_NotImplemented(" BRepFill_TrimSurfaceTool");
            }

            gp_Vec2d D12d = Bis.DN(UBis, 1);
            gp_Vec D1 = new(D12d.X(), D12d.Y(), 0.0);

            double U = Axis.XDirection().AngleWithRef(new gp_Dir(D1), Axis.XDirection() ^ Axis.YDirection());
            U += Phase;
            if (U < 0.0)
            {
                U += 2 * Math.PI;
            }

            P = new gp_Pnt(Bis.FirstParameter(), U, 0.0);
            Seq.Append(P);
        }
    }

    /// <summary>
    /// The intersections of the bisector with EdgeOnF1 and EdgeOnF2: each point is (parameter on the bisector,
    /// parameter on EdgeOnF1, parameter on EdgeOnF2), by increasing parameter on the bisector.
    /// </summary>
    /// <exception cref="StdFail_NotDone">The two edges give different numbers of intersections.</exception>
    public void IntersectWith(in TopoDS_Edge EdgeOnF1, in TopoDS_Edge EdgeOnF2, NCollection_Sequence<gp_Pnt> Points)
    {
        Points.Clear();
        NCollection_Sequence<gp_Pnt> Points2 = new();

        EvalParameters(EdgeOnF1, myFace1, myBis, Points);
        EvalParameters(EdgeOnF2, myFace2, myBis, Points2);

        StdFail_NotDone_Raise_if(Points.Length() != Points2.Length(),
                                 "BRepFill_TrimSurfaceTool::IntersectWith: incoherent intersection");

        gp_Pnt PSeq;
        int NbPoints = Points.Length();
        for (int i = 1; i <= NbPoints; i++)
        {
            PSeq = Points[i];
            PSeq.SetZ((Points2.Value(i)).Y());
            Points.SetValue(i, PSeq);
        }
    }

    /// <summary>True if the vertical line through <paramref name="Point"/> meets one of the two faces.</summary>
    public bool IsOnFace(in gp_Pnt2d Point)
    {
        gp_Pnt P = new(Point.X(), Point.Y(), 0.0);
        gp_Lin Line = new(P, gp.DZ());

        BRepIntCurveSurface_Inter Inter = new();

        // eval if is on face 1
        //  modified by NIZHNY-EAP Fri Jan 21 09:49:09 2000 ___BEGIN___
        Inter.Init(myFace1, Line, 1e-6); // Precision::PConfusion());
        if (Inter.More())
        {
            return true;
        }

        // eval if is on face 2
        Inter.Init(myFace2, Line, 1e-6); // Precision::PConfusion());
        return Inter.More();
        //  modified by NIZHNY-EAP Fri Jan 21 09:49:14 2000 ___END___
    }

    /// <summary>The parameter of the projection of <paramref name="Point"/> on the edge's curve projected on xOy.</summary>
    public double ProjOn(in gp_Pnt2d Point, in TopoDS_Edge Edge)
    {
        Geom_Curve C1 = BRep_Tool.Curve(Edge, out TopLoc_Location L, out double f, out double l)!;
        Geom_TrimmedCurve CT = new(C1, f, l);
        CT.Transform(L.Transformation());

        // projection of curves 3d in the plane xOy
        Geom_Plane Plane = new(0, 0, 1, 0);
        Geom2d_Curve C2d = GeomProjLib.Curve2d(CT, Plane)!;

        // evaluate the projection of the point on the curve.
        Geom2dAPI_ProjectPointOnCurve Projector = new(Point, C2d);

        double U = Projector.LowerDistanceParameter();
        return U;
    }

    /// <summary>
    /// The 3D curve and the pcurves on both faces of the bisector between <paramref name="U1"/> and
    /// <paramref name="U2"/>, exact in the particular cases, approximated otherwise; and its continuity.
    /// </summary>
    public void Project(double U1,
                        double U2,
                        out Geom_Curve? Curve,
                        out Geom2d_Curve? PCurve1,
                        out Geom2d_Curve? PCurve2,
                        out GeomAbs_Shape Cont)
    {
        Geom2d_TrimmedCurve CT = new(myBis, U1, U2);
        BRepFill_MultiLine ML = new(myFace1, myFace2, myEdge1, myEdge2, myInv1, myInv2, CT);

        Cont = ML.Continuity();

        if (ML.IsParticularCase())
        {
            ML.Curves(out Curve, out PCurve1, out PCurve2);
        }
        else
        {
            BRepFill_ApproxSeewing AppSeew = new(ML);

            Curve = AppSeew.Curve();
            PCurve1 = AppSeew.CurveOnF1();
            PCurve2 = AppSeew.CurveOnF2();
        }
    }
}
