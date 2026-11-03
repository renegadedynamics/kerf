// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimSurfaceTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimSurfaceTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.ShapeType;

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
    private readonly Face myFace1;
    private readonly Face myFace2;
    private readonly Edge myEdge1;
    private readonly Edge myEdge2;
    private readonly bool myInv1;
    private readonly bool myInv2;
    private readonly Curve2D myBis;

    /// <summary>
    /// Initialisation with two neighbour faces. Edge1 and Edge2 are the parallel edges corresponding to the minimum
    /// iso on Face1 and Face2 (Edge1 is Umin or Vmin on Face1); Inv1 and Inv2 say whether they are reversed.
    /// </summary>
    public BRepFill_TrimSurfaceTool(Curve2D Bis,
                                    in Face Face1,
                                    in Face Face2,
                                    in Edge Edge1,
                                    in Edge Edge2,
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
    private static void Bubble(NCollection_Sequence<Point3D> Seq)
    {
        bool Invert = true;
        int NbPoints = Seq.Length();
        while (Invert)
        {
            Invert = false;
            for (int i = 1; i < NbPoints; i++)
            {
                Point3D P1 = Seq.Value(i);
                Point3D P2 = Seq.Value(i + 1);
                if (P2.X() < P1.X())
                {
                    Seq.Exchange(i, i + 1);
                    Invert = true;
                }
            }
        }
    }

    private static double EvalPhase(in Edge Edge, in Face Face, SurfaceAdaptor GAS, in CoordinateSystem Axis)
    {
        Point2D PF1, PF2;
        double VDeg;
        double V = 0.0;
        Tool.UVPoints(Edge, Face, out Point2D PE1, out Point2D _);
        VDeg = PE1.Y();
        Explorer Exp = new(Face, global::Kerf.Topology.ShapeType.Edge);
        for (; Exp.More(); Exp.Next())
        {
            if (!ShapeCasts.Edge(Exp.Current()).IsSame(Edge))
            {
                Tool.UVPoints(ShapeCasts.Edge(Exp.Current()), Face, out PF1, out PF2);
                V = (Math.Abs(PF1.Y() - VDeg) > Math.Abs(PF2.Y() - VDeg)) ? PF1.Y() : PF2.Y();
                break;
            }
        }
        Point3D P = GAS.Value(0.0, V);

        if (new Vector3D(Axis.Location(), P).Dot(new Vector3D(Axis.XDirection())) < 0.0)
        {
            return Math.PI;
        }
        else
        {
            return 0.0;
        }
    }

    private static void EvalParameters(in Edge Edge, in Face Face, Curve2D Bis, NCollection_Sequence<Point3D> Seq)
    {
        bool Degener = Tool.Degenerated(Edge);
        // return curves 3d associated to edges.
        TrimmedCurve3D CT;
        PlaneSurface Plane = new(0, 0, 1, 0);

        Geom2dInt_GInter Intersector;

        int NbPoints, NbSegments;
        double U1, U2;
        Point3D P;

        //  modified by NIZHNY-EAP Wed Dec 22 15:00:51 1999 ___BEGIN___
        double Tol = 1.0e-6; // BRepFill_Precision();
        double TolC = 0.0;

        if (!Degener)
        {
            Curve3D C = Tool.Curve(Edge, out Placement L, out double f, out double l)!;
            CT = new TrimmedCurve3D(C, f, l);
            CT.Transform(L.Transformation());
            // projection of 3d curves in the plane xOy
            Curve2D C2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(CT, Plane)!;

            CurveAdaptor2D AC = new(C2d);
            CurveAdaptor2D ABis = new(Bis);

            Intersector = new Geom2dInt_GInter(ABis, AC, TolC, Tol);

            if (!Intersector.IsDone())
            {
                throw new NotDoneException("BRepFill_TrimSurfaceTool::IntersectWith");
            }

            NbPoints = Intersector.NbPoints();

            if (NbPoints < 1)
            {
                // try to elongate curves and enlarge tolerance

                // don't do it rightaway from the beginning in order not to get
                // extra solutions those would cause *Exception*: incoherent intersection

                CurveKind CType = AC.GetType(), BisType = ABis.GetType();
                bool canElongateC = CType != BezierCurve && CType != BSplineCurve
                                    && CType != OffsetCurve && CType != OtherCurve;
                bool canElongateBis = BisType != BezierCurve && BisType != BSplineCurve
                                      && BisType != OffsetCurve && BisType != OtherCurve;

                // OCCT: the down-casts are not checked (a curve that isn't trimmed crashes natively)
                TrimmedCurve2D TBis = (Bis as TrimmedCurve2D)!;
                TrimmedCurve2D TC2d = (C2d as TrimmedCurve2D)!;

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
                    throw new NotDoneException("BRepFill_TrimSurfaceTool::IntersectWith");
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
                    P = new Point3D(U1, U2, 0.0);
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
                    P = new Point3D(U1, U2, 0.0);
                    Seq.Append(P);
                }
            }
            // Order the sequence by increasing parameter on the bissectrice.
            Bubble(Seq);

            //  modified by NIZHNY-EAP Fri Dec 24 18:47:24 1999 ___BEGIN___
            // Remove double points
            Point3D P1, P2;
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

            Point3D P3d = Tool.Pnt(Exploration.FirstVertex(Edge));
            Point2D P2d = new(P3d.X(), P3d.Y());

            double UBis = Bis.FirstParameter();
            Point2D PBis = Bis.Value(UBis);

            //  modified by NIZHNY-EAP Wed Jan 12 11:41:30 2000 ___BEGIN___
            // inside gp_Pnt2d::Distance
            // Infinite * Infinite => Exception: DefaultNumericError
            // Case encounered: UBis < Precision::Infinite()
            // but PBis.X() > Precision::Infinite()
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(Math.Abs(PBis.X()))
                || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(Math.Abs(PBis.Y())) || PBis.Distance(P2d) > Tol)
            {
                //  modified by NIZHNY-EAP Wed Jan 12 11:41:40 2000 ___END___
                UBis = Bis.LastParameter();
                if (UBis >= global::Kerf.Engine.Foundation.Precision.Infinite())
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
            Surface GS = Tool.Surface(Face)!;
            SurfaceAdaptor GAS = new(GS);

            CoordinateSystem Axis;
            double Phase = 0.0;

            switch (GAS.GetType())
            {
                case SurfaceKind.Sphere:
                    Axis = GAS.Sphere().Position();
                    break;
                case SurfaceKind.Cone:
                    {
                        //----------------------------------------------------------
                        // if myFace1 is not at the same side of the apex as the point
                        // of parameter 0 0 on the cone => phase = M_PI.
                        //----------------------------------------------------------
                        Axis = GAS.Cone().Position();
                        Phase = EvalPhase(Edge, Face, GAS, Axis);
                        break;
                    }
                case SurfaceKind.Torus:
                    Axis = GAS.Torus().Position();
                    break;
                case SurfaceKind.Cylinder:
                    Axis = GAS.Cylinder().Position();
                    break;
                case SurfaceOfRevolution:
                    {
                        //----------------------------------------------------------
                        // if myFace1 is not at the same side of the apex as the point
                        // of parameter 0 0 on the cone => phase = M_PI.
                        //----------------------------------------------------------
                        RevolutionSurface GSRev = (RevolutionSurface)GS;
                        CurveAdaptor HC = new(GSRev.BasisCurve());
                        RevolutionSurfaceAdaptor ASRev = new(HC, GAS.AxeOfRevolution());
                        Axis = ASRev.Axis();
                        Phase = EvalPhase(Edge, Face, GAS, Axis);
                        break;
                    }
                default:
                    throw new MethodNotImplementedException(" BRepFill_TrimSurfaceTool");
            }

            Vector2D D12d = Bis.DN(UBis, 1);
            Vector3D D1 = new(D12d.X(), D12d.Y(), 0.0);

            double U = Axis.XDirection().AngleWithRef(new Direction3D(D1), Axis.XDirection() ^ Axis.YDirection());
            U += Phase;
            if (U < 0.0)
            {
                U += 2 * Math.PI;
            }

            P = new Point3D(Bis.FirstParameter(), U, 0.0);
            Seq.Append(P);
        }
    }

    /// <summary>
    /// The intersections of the bisector with EdgeOnF1 and EdgeOnF2: each point is (parameter on the bisector,
    /// parameter on EdgeOnF1, parameter on EdgeOnF2), by increasing parameter on the bisector.
    /// </summary>
    /// <exception cref="NotDoneException">The two edges give different numbers of intersections.</exception>
    public void IntersectWith(in Edge EdgeOnF1, in Edge EdgeOnF2, NCollection_Sequence<Point3D> Points)
    {
        Points.Clear();
        NCollection_Sequence<Point3D> Points2 = new();

        EvalParameters(EdgeOnF1, myFace1, myBis, Points);
        EvalParameters(EdgeOnF2, myFace2, myBis, Points2);

        StdFail_NotDone_Raise_if(Points.Length() != Points2.Length(),
                                 "BRepFill_TrimSurfaceTool::IntersectWith: incoherent intersection");

        Point3D PSeq;
        int NbPoints = Points.Length();
        for (int i = 1; i <= NbPoints; i++)
        {
            PSeq = Points[i];
            PSeq.SetZ((Points2.Value(i)).Y());
            Points.SetValue(i, PSeq);
        }
    }

    /// <summary>True if the vertical line through <paramref name="Point"/> meets one of the two faces.</summary>
    public bool IsOnFace(in Point2D Point)
    {
        Point3D P = new(Point.X(), Point.Y(), 0.0);
        Line3D Line = new(P, Constants.DZ());

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
    public double ProjOn(in Point2D Point, in Edge Edge)
    {
        Curve3D C1 = Tool.Curve(Edge, out Placement L, out double f, out double l)!;
        TrimmedCurve3D CT = new(C1, f, l);
        CT.Transform(L.Transformation());

        // projection of curves 3d in the plane xOy
        PlaneSurface Plane = new(0, 0, 1, 0);
        Curve2D C2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(CT, Plane)!;

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
                        out Curve3D? Curve,
                        out Curve2D? PCurve1,
                        out Curve2D? PCurve2,
                        out Continuity Cont)
    {
        TrimmedCurve2D CT = new(myBis, U1, U2);
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
