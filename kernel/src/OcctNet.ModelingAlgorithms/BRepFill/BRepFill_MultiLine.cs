// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_MultiLine.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_MultiLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The continuous MultiLine BRepFill_ComputeCLine approximates: a bisector curve seen as a 3D curve on the first face
/// and a 2D curve on each of two faces (BRepFill_TrimSurfaceTool's intersection of two faces of an evolved shape).
/// </summary>
/// <remarks>A working object of BRepFill_TrimSurfaceTool (decision 21).</remarks>
public sealed class BRepFill_MultiLine : Function
{
    private readonly Face myFace1 = new();
    private readonly Face myFace2 = new();
    private readonly CurveAdaptor2D myU1 = new();
    private readonly CurveAdaptor2D myV1 = new();
    private readonly CurveAdaptor2D myU2 = new();
    private readonly CurveAdaptor2D myV2 = new();
    private readonly bool myIsoU1;
    private readonly bool myIsoU2;
    private readonly CurveAdaptor2D myBis = new();
    private readonly int myKPart;
    private readonly Continuity myCont;

    private static bool isIsoU(in Face Face, in Edge Edge)
    {
        Curve2D? C = Tool.CurveOnSurface(Edge, Face, out double f, out double _);
        if (C is null)
        {
            throw new ConstructionErrorException("BRepFill_MultiLine : Edge without PCurve");
        }

        Direction2D D = new(C.DN(f, 1));

        return Math.Abs(D.Dot(Constants.DX2d())) < Math.Abs(D.Dot(Constants.DY2d()));
    }

    /// <summary>An empty MultiLine (one 3D point and two 2D points).</summary>
    public BRepFill_MultiLine()
    {
        _nbPnt2d = 2;
        _nbPnt = 1;
    }

    /// <summary>The bisector <paramref name="Bissec"/> between Edge1 of Face1 and Edge2 of Face2.</summary>
    public BRepFill_MultiLine(in Face Face1,
                              in Face Face2,
                              in Edge Edge1,
                              in Edge Edge2,
                              bool Inv1,
                              bool Inv2,
                              Curve2D Bissec)
    {
        myFace1 = Face1;
        myFace2 = Face2;
        myBis = new CurveAdaptor2D(Bissec);
        myKPart = 0;
        //
        const double mult = 5.0;
        double eps = mult * global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        _nbPnt2d = 2;
        _nbPnt = 1;

        // eval if myedges are IsoU or not
        myIsoU1 = isIsoU(Face1, Edge1);
        myIsoU2 = isIsoU(Face2, Edge2);

        // eval myU1, myV1, myU2, myV2;
        PlaneSurface RefPlane;
        PlaneSurface BasisPlane = new(0.0, 0.0, 1.0, 0.0);
        Placement L;

        Explorer Exp = new();
        double Umin = 0.0, Vmin = 0.0, Umax = 0.0, Vmax = 0.0, U, V;
        Point2D P1, P2;
        Vector3D DZ;
        Point3D P;

        // Result on Face1
        bool First = true;
        for (Exp.Init(myFace1, ShapeType.Edge); Exp.More(); Exp.Next())
        {
            Edge CurEdge = ShapeCasts.Edge(Exp.Current());
            Tool.UVPoints(CurEdge, myFace1, out P1, out P2);
            if (First)
            {
                First = false;
                Umin = Min(P1.X(), P2.X());
                Umax = Max(P1.X(), P2.X());

                Vmin = Min(P1.Y(), P2.Y());
                Vmax = Max(P1.Y(), P2.Y());
            }
            else
            {
                U = Min(P1.X(), P2.X());
                Umin = Min(Umin, U);
                U = Max(P1.X(), P2.X());
                Umax = Max(Umax, U);

                V = Min(P1.Y(), P2.Y());
                Vmin = Min(Vmin, V);
                V = Max(P1.Y(), P2.Y());
                Vmax = Max(Vmax, V);
            }
        }

        // return isos in their domain of restriction.
        Curve3D UU1, UU2, VV1, VV2;
        Surface S = Tool.Surface(myFace1, out L)!;
        if (!L.IsIdentity)
        {
            S = (Surface)S.Transformed(L.Transformation());
        }

        if (myIsoU1)
        {
            if (!Tool.Degenerated(Edge1) && !Inv1)
            {
                UU1 = S.UIso(Umin);
                CurveAdaptor Dummy = new(UU1);
                if (Dummy.GetType() == Circle && Dummy.Circle().Radius() < eps)
                {
                    UU1 = S.UIso(Umax);
                }
            }
            else
            {
                UU1 = S.UIso(Umax);
                CurveAdaptor Dummy = new(UU1);
                if (Dummy.GetType() == Circle && Dummy.Circle().Radius() < eps)
                {
                    UU1 = S.UIso(Umin);
                }
            }
            VV1 = S.VIso(Vmin);
        }
        else
        {
            if (!Tool.Degenerated(Edge1) && !Inv1)
            {
                UU1 = S.VIso(Vmin);
                CurveAdaptor Dummy = new(UU1);
                if (Dummy.GetType() == Circle && Dummy.Circle().Radius() < eps)
                {
                    UU1 = S.VIso(Vmax);
                }
            }
            else
            {
                UU1 = S.VIso(Vmax);
                CurveAdaptor Dummy = new(UU1);
                if (Dummy.GetType() == Circle && Dummy.Circle().Radius() < eps)
                {
                    UU1 = S.VIso(Vmin);
                }
            }
            VV1 = S.UIso(Umin);
        }

        if (myIsoU1)
        {
            double dummyUmin = Umin, dummyUmax = Umax;
            Umin = Vmin;
            Umax = Vmax;
            Vmin = dummyUmin;
            Vmax = dummyUmax;
        }

        // try duplication
        SurfaceAdaptor GAS1 = new(S);
        SurfaceKind Type1 = GAS1.GetType();

        if (UU1.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(UU1.FirstParameter(), UU1.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref Umin, ref Umax);
        }
        if (VV1.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(VV1.FirstParameter(), VV1.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref Vmin, ref Vmax);
        }
        // end try duplication

        myU1 = new CurveAdaptor2D(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(UU1, BasisPlane)!, Umin, Umax);

        UU1.D1(Umin, out P, out DZ);
        RefPlane = new PlaneSurface(new CoordinateSystem(P, new Direction3D(DZ), Constants.DZ()));

        myV1 = new CurveAdaptor2D(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(VV1, RefPlane)!, Vmin, Vmax);

        First = true;
        for (Exp.Init(myFace2, ShapeType.Edge); Exp.More(); Exp.Next())
        {
            Edge CurEdge = ShapeCasts.Edge(Exp.Current());
            Tool.UVPoints(CurEdge, myFace2, out P1, out P2);
            if (First)
            {
                First = false;
                Umin = Min(P1.X(), P2.X());
                Umax = Max(P1.X(), P2.X());

                Vmin = Min(P1.Y(), P2.Y());
                Vmax = Max(P1.Y(), P2.Y());
            }
            else
            {
                U = Min(P1.X(), P2.X());
                Umin = Min(Umin, U);
                U = Max(P1.X(), P2.X());
                Umax = Max(Umax, U);

                V = Min(P1.Y(), P2.Y());
                Vmin = Min(Vmin, V);
                V = Max(P1.Y(), P2.Y());
                Vmax = Max(Vmax, V);
            }
        }

        // return isos in their domain of restriction.
        S = Tool.Surface(myFace2, out L)!;

        if (!L.IsIdentity)
        {
            S = (Surface)S.Transformed(L.Transformation());
        }

        if (myIsoU2)
        {
            if (!Tool.Degenerated(Edge2) && !Inv2)
            {
                UU2 = S.UIso(Umin);
                CurveAdaptor Dummy = new(UU2);
                if (Dummy.GetType() == Circle && Dummy.Circle().Radius() < eps)
                {
                    UU2 = S.UIso(Umax);
                }
            }
            else
            {
                UU2 = S.UIso(Umax);
                CurveAdaptor Dummy = new(UU2);
                if (Dummy.GetType() == Circle && Dummy.Circle().Radius() < eps)
                {
                    UU2 = S.UIso(Umin);
                }
            }
            VV2 = S.VIso(Vmin);
        }
        else
        {
            if (!Tool.Degenerated(Edge2) && !Inv2)
            {
                UU2 = S.VIso(Vmin);
                CurveAdaptor Dummy = new(UU2);
                if (Dummy.GetType() == Circle && Dummy.Circle().Radius() < eps)
                {
                    UU2 = S.VIso(Vmax);
                }
            }
            else
            {
                UU2 = S.VIso(Vmax);
                CurveAdaptor Dummy = new(UU2);
                if (Dummy.GetType() == Circle && Dummy.Circle().Radius() < eps)
                {
                    UU2 = S.VIso(Vmin);
                }
            }
            VV2 = S.UIso(Umin);
        }

        if (myIsoU2)
        {
            double dummyUmin = Umin, dummyUmax = Umax;
            Umin = Vmin;
            Umax = Vmax;
            Vmin = dummyUmin;
            Vmax = dummyUmax;
        }

        // try duplication
        SurfaceAdaptor GAS2 = new(S);
        SurfaceKind Type2 = GAS2.GetType();

        if (UU2.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(UU2.FirstParameter(), UU2.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref Umin, ref Umax);
        }
        if (VV2.IsPeriodic)
        {
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(VV2.FirstParameter(), VV2.LastParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion(), ref Vmin, ref Vmax);
        }
        // end try duplication

        myU2 = new CurveAdaptor2D(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(UU2, BasisPlane)!, Umin, Umax);

        UU2.D1(Umin, out P, out DZ);
        RefPlane = new PlaneSurface(new CoordinateSystem(P, new Direction3D(DZ), Constants.DZ()));
        myV2 = new CurveAdaptor2D(global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(VV2, RefPlane)!, Vmin, Vmax);

        // eval if in a particular case.
        // Particular case if :
        //     1) - Straight Bissectrice
        //        - Bissectrice orthogonal to the base element.
        //        ==> Iso on 2 faces.
        //     2) - Straight Bissectrice
        //        - 2 surfaces are planes.
        myCont = C0;

        if (myBis.GetType() == Line)
        {
            double DeltaU = myBis.LastParameter() - myBis.FirstParameter();
            Point2D aPnt1 = ValueOnF1(myBis.FirstParameter() + 0.1 * DeltaU);
            Point2D aPnt2 = ValueOnF1(myBis.FirstParameter() + 0.9 * DeltaU);
            if (myIsoU1)
            {
                if (Math.Abs(aPnt1.Y() - aPnt2.Y()) < eps)
                {
                    myKPart = 1;
                }
            }
            else
            {
                if (Math.Abs(aPnt1.X() - aPnt2.X()) < eps)
                {
                    myKPart = 1;
                }
            }

            if (myKPart == 1)
            {
                myCont = G1;
            }

            if ((Type1 == Plane) && (Type2 == Plane))
            {
                myKPart = 2;
            }
        }
    }

    /// <summary>True if the bisector is a line that gives isos on both faces, or lies between two planes.</summary>
    public bool IsParticularCase() => myKPart != 0;

    /// <summary>The continuity of the line.</summary>
    public Continuity Continuity() => myCont;

    /// <summary>In a particular case, the 3D curve and the two pcurves, built exactly.</summary>
    public void Curves(out Curve3D? Curve, out Curve2D? PCurve1, out Curve2D? PCurve2)
    {
        Curve = null;
        PCurve1 = null;
        PCurve2 = null;
        if (myKPart == 1)
        {
            Point2D P1, P2, PMil;
            double f, l;

            P1 = ValueOnF1(myBis.FirstParameter());
            P2 = ValueOnF1(myBis.LastParameter());

            // find value of the with medium point
            // the ends can be degenerated points.

            PMil = ValueOnF1(0.5 * (myBis.FirstParameter() + myBis.LastParameter()));

            Surface S = Tool.Surface(myFace1, out Placement L)!;
            if (!L.IsIdentity)
            {
                S = (Surface)S.Transformed(L.Transformation());
            }

            bool Sens;
            if (!myIsoU1)
            {
                Curve = S.UIso(PMil.X());
                Sens = P1.Y() < P2.Y();
                if (Sens)
                {
                    Curve = new TrimmedCurve3D(Curve, P1.Y(), P2.Y(), Sens);
                }
                else
                {
                    Curve = new TrimmedCurve3D(Curve, P2.Y(), P1.Y(), Sens);
                }

                f = Curve.FirstParameter();
                l = Curve.LastParameter();
                if (Sens)
                {
                    PCurve1 = new LineCurve2D(new Point2D(PMil.X(), P1.Y() - f), Constants.DY2d());
                }
                else
                {
                    PCurve1 = new LineCurve2D(new Point2D(PMil.X(), P1.Y() + f), Constants.DY2d().Reversed());
                }
                PCurve1 = new TrimmedCurve2D(PCurve1, f, l);
            }
            else
            {
                Curve = S.VIso(PMil.Y());
                Sens = P1.X() < P2.X();
                if (Sens)
                {
                    Curve = new TrimmedCurve3D(Curve, P1.X(), P2.X(), Sens);
                }
                else
                {
                    Curve = new TrimmedCurve3D(Curve, P2.X(), P1.X(), Sens);
                }

                f = Curve.FirstParameter();
                l = Curve.LastParameter();
                if (Sens)
                {
                    PCurve1 = new LineCurve2D(new Point2D(P1.X() - f, PMil.Y()), Constants.DX2d());
                }
                else
                {
                    PCurve1 = new LineCurve2D(new Point2D(P1.X() + f, PMil.Y()), Constants.DX2d().Reversed());
                }
                PCurve1 = new TrimmedCurve2D(PCurve1, f, l);
            }

            P1 = ValueOnF2(myBis.FirstParameter());
            P2 = ValueOnF2(myBis.LastParameter());
            PMil = ValueOnF2(0.5 * (myBis.FirstParameter() + myBis.LastParameter()));

            if (!myIsoU2)
            {
                Sens = P1.Y() < P2.Y();
                if (Sens)
                {
                    PCurve2 = new LineCurve2D(new Point2D(PMil.X(), (P1.Y() - f)), Constants.DY2d());
                }
                else
                {
                    PCurve2 = new LineCurve2D(new Point2D(PMil.X(), (P1.Y() + f)), Constants.DY2d().Reversed());
                }
            }
            else
            {
                Sens = P1.X() < P2.X();
                if (Sens)
                {
                    PCurve2 = new LineCurve2D(new Point2D(P1.X() - f, PMil.Y()), Constants.DX2d());
                }
                else
                {
                    PCurve2 = new LineCurve2D(new Point2D(P1.X() + f, PMil.Y()), Constants.DX2d().Reversed());
                }
            }
            PCurve2 = new TrimmedCurve2D(PCurve2, f, l);
        }
        else if (myKPart == 2)
        {
            Surface S = Tool.Surface(myFace1, out Placement L)!;
            if (!L.IsIdentity)
            {
                S = (Surface)S.Transformed(L.Transformation());
            }

            if (S is TrimmedSurface aRTS1)
            {
                S = aRTS1.BasisSurface();
            }

            PlaneSurface Plane = (PlaneSurface)S;
            // eval the 3d curve corresponding to the bissectrice.
            Point2D P = myBis.Line().Location();
            Direction2D D = myBis.Line().Direction();
            LineCurve3D Line = new(new Point3D(P.X(), P.Y(), 0.0), new Direction3D(D.X(), D.Y(), 0.0));
            TrimmedCurve3D TLine = new(Line, myBis.FirstParameter(), myBis.LastParameter());
            Curve = global::Kerf.Engine.Geometry.Projection.GeomProjLib.ProjectOnPlane(TLine, Plane, Constants.DZ(), false);

            // eval PCurve1
            PCurve1 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(Curve!, Plane);

            // eval PCurve2
            S = Tool.Surface(myFace2, out L)!;
            if (!L.IsIdentity)
            {
                S = (Surface)S.Transformed(L.Transformation());
            }
            if (S is TrimmedSurface aRTS2)
            {
                S = aRTS2.BasisSurface();
            }
            Plane = (PlaneSurface)S;
            PCurve2 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(Curve!, Plane);
        }
    }

    /// <inheritdoc/>
    public override double FirstParameter() => myBis.FirstParameter();

    /// <inheritdoc/>
    public override double LastParameter() => myBis.LastParameter();

    /// <summary>The 3D point at <paramref name="U"/> (on the first face).</summary>
    public Point3D Value(double U)
    {
        Surface S = Tool.Surface(myFace1, out Placement L)!;

        Point2D P2d = ValueOnF1(U);

        Point3D P3d = S.Value(P2d.X(), P2d.Y());
        P3d.Transform(L.Transformation());

        return P3d;
    }

    private static Point2D ValueOnFace(double U,
                                        CurveAdaptor2D TheBis,
                                        CurveAdaptor2D TheU,
                                        CurveAdaptor2D TheV,
                                        bool IsIsoU)
    {
        Point2D P = TheBis.Value(U);

        Geom2dAPI_ProjectPointOnCurve Ext = new(P, TheU.Curve()!, TheU.FirstParameter(), TheU.LastParameter());
        //
        const double mult = 5.0;
        double eps = mult * global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        double UU = 0.0, Dist = global::Kerf.Engine.Foundation.Precision.Infinite(), D1, D2;

        if (Ext.NbPoints() != 0)
        {
            UU = Ext.LowerDistanceParameter();
            Dist = Ext.LowerDistance();
        }
        // Control with `ends`
        D1 = P.Distance(TheU.Value(TheU.FirstParameter()));
        D2 = P.Distance(TheU.Value(TheU.LastParameter()));

        if (D1 < Dist || D2 < Dist || Math.Abs(D1 - Dist) < eps || Math.Abs(D2 - Dist) < eps)
        {
            if (Math.Abs(D1 - D2) < eps)
            {
                if (TheU.GetType() == Circle)
                {
                    Vector2D Axis = new(TheU.Circle().XAxis().Direction());
                    Vector2D D12d = TheBis.DN(TheBis.FirstParameter(), 1);
                    double Ang = Axis.Angle(D12d);
                    if (!TheU.Circle().IsDirect)
                    {
                        Ang = -Ang;
                    }
                    UU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(Ang, TheU.FirstParameter(), TheU.FirstParameter() + 2 * Math.PI);
                    Dist = TheU.Circle().Radius();
                }
                else
                {
                    throw new ConstructionErrorException("BRepFill_MultiLine: ValueOnFace");
                }
            }
            else if (D1 < D2)
            {
                Dist = D1;
                UU = TheU.FirstParameter();
            }
            else
            {
                Dist = D2;
                UU = TheU.LastParameter();
            }
        }

        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double VV;

        Point2D PF = TheV.Value(TheV.FirstParameter());
        Point2D PL = TheV.Value(TheV.LastParameter());

        if (Math.Abs(Dist - Math.Abs(PF.Y())) < Tol)
        {
            VV = TheV.FirstParameter();
        }
        else if (Math.Abs(Dist - Math.Abs(PL.Y())) < Tol)
        {
            VV = TheV.LastParameter();
        }
        else
        {
            // test if the curve is at the side `negative Y`.
            if (Min(PF.Y(), PL.Y()) < -Tol)
            {
                Dist = -Dist;
            }

            LineCurve2D Line = new(new Point2D(0.0, Dist), Constants.DX2d());

            CurveAdaptor2D Cu1 = TheV;
            CurveAdaptor2D Cu2 = new(Line);

            double TolConf = 0.0;

            Geom2dInt_GInter Intersector = new(Cu1, Cu2, TolConf, Tol);

            if (!Intersector.IsDone())
            {
                return new Point2D(0.0, 0.0);
            }
            else
            {
                if (Intersector.NbPoints() > 0)
                {
                    VV = Intersector.Point(1).ParamOnFirst();
                }
                else if (Intersector.NbSegments() > 0)
                {
                    IntRes2d_IntersectionSegment Seg = Intersector.Segment(1);
                    double VS1 = Seg.FirstPoint().ParamOnFirst();
                    double VS2 = Seg.LastPoint().ParamOnFirst();
                    Point2D PS1 = TheV.Value(VS1);
                    Point2D PS2 = TheV.Value(VS2);
                    double Alp = (Dist - PS1.Y()) / (PS2.Y() - PS1.Y());
                    VV = Alp * (VS2 - VS1) + VS1;
                }
                else
                {
                    if (Math.Abs(Dist - PL.Y()) < Math.Abs(Dist - PF.Y()))
                    {
                        VV = TheV.LastParameter();
                    }
                    else
                    {
                        VV = TheV.FirstParameter();
                    }
                }
            }
        }

        if (IsIsoU)
        {
            return new Point2D(VV, UU);
        }
        else
        {
            return new Point2D(UU, VV);
        }
    }

    /// <summary>The point at <paramref name="U"/> on the first face's parameter plane.</summary>
    public Point2D ValueOnF1(double U) => ValueOnFace(U, myBis, myU1, myV1, myIsoU1);

    /// <summary>The point at <paramref name="U"/> on the second face's parameter plane.</summary>
    public Point2D ValueOnF2(double U) => ValueOnFace(U, myBis, myU2, myV2, myIsoU2);

    /// <summary>The 3D point and both 2D points at <paramref name="U"/>.</summary>
    public void Value3dOnF1OnF2(double U, out Point3D P3d, out Point2D PF1, out Point2D PF2)
    {
        PF1 = ValueOnFace(U, myBis, myU1, myV1, myIsoU1);
        PF2 = ValueOnFace(U, myBis, myU2, myV2, myIsoU2);

        Surface S = Tool.Surface(myFace1, out Placement L)!;
        P3d = S.Value(PF1.X(), PF1.Y());
        P3d.Transform(L.Transformation());
    }

    /// <inheritdoc/>
    public override bool Value(double theT, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
    {
        thePnt[1] = Value(theT);
        thePnt2d[1] = ValueOnF1(theT);
        thePnt2d[2] = ValueOnF2(theT);
        return true;
    }

    /// <inheritdoc/>
    public override bool D1(double theT, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec) => false;
}
