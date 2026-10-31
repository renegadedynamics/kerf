// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_MultiLine.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_MultiLine.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The continuous MultiLine BRepFill_ComputeCLine approximates: a bisector curve seen as a 3D curve on the first face
/// and a 2D curve on each of two faces (BRepFill_TrimSurfaceTool's intersection of two faces of an evolved shape).
/// </summary>
/// <remarks>A working object of BRepFill_TrimSurfaceTool (decision 21).</remarks>
public sealed class BRepFill_MultiLine : AppCont_Function
{
    private readonly TopoDS_Face myFace1 = new();
    private readonly TopoDS_Face myFace2 = new();
    private readonly Geom2dAdaptor_Curve myU1 = new();
    private readonly Geom2dAdaptor_Curve myV1 = new();
    private readonly Geom2dAdaptor_Curve myU2 = new();
    private readonly Geom2dAdaptor_Curve myV2 = new();
    private readonly bool myIsoU1;
    private readonly bool myIsoU2;
    private readonly Geom2dAdaptor_Curve myBis = new();
    private readonly int myKPart;
    private readonly GeomAbs_Shape myCont;

    private static bool isIsoU(in TopoDS_Face Face, in TopoDS_Edge Edge)
    {
        Geom2d_Curve? C = BRep_Tool.CurveOnSurface(Edge, Face, out double f, out double _);
        if (C is null)
        {
            throw new Standard_ConstructionError("BRepFill_MultiLine : Edge without PCurve");
        }

        gp_Dir2d D = new(C.DN(f, 1));

        return Math.Abs(D.Dot(gp.DX2d())) < Math.Abs(D.Dot(gp.DY2d()));
    }

    /// <summary>An empty MultiLine (one 3D point and two 2D points).</summary>
    public BRepFill_MultiLine()
    {
        myNbPnt2d = 2;
        myNbPnt = 1;
    }

    /// <summary>The bisector <paramref name="Bissec"/> between Edge1 of Face1 and Edge2 of Face2.</summary>
    public BRepFill_MultiLine(in TopoDS_Face Face1,
                              in TopoDS_Face Face2,
                              in TopoDS_Edge Edge1,
                              in TopoDS_Edge Edge2,
                              bool Inv1,
                              bool Inv2,
                              Geom2d_Curve Bissec)
    {
        myFace1 = Face1;
        myFace2 = Face2;
        myBis = new Geom2dAdaptor_Curve(Bissec);
        myKPart = 0;
        //
        const double mult = 5.0;
        double eps = mult * Precision.Confusion();
        //
        myNbPnt2d = 2;
        myNbPnt = 1;

        // eval if myedges are IsoU or not
        myIsoU1 = isIsoU(Face1, Edge1);
        myIsoU2 = isIsoU(Face2, Edge2);

        // eval myU1, myV1, myU2, myV2;
        Geom_Plane RefPlane;
        Geom_Plane BasisPlane = new(0.0, 0.0, 1.0, 0.0);
        TopLoc_Location L;

        TopExp_Explorer Exp = new();
        double Umin = 0.0, Vmin = 0.0, Umax = 0.0, Vmax = 0.0, U, V;
        gp_Pnt2d P1, P2;
        gp_Vec DZ;
        gp_Pnt P;

        // Result on Face1
        bool First = true;
        for (Exp.Init(myFace1, TopAbs_EDGE); Exp.More(); Exp.Next())
        {
            TopoDS_Edge CurEdge = TopoDS.Edge(Exp.Current());
            BRep_Tool.UVPoints(CurEdge, myFace1, out P1, out P2);
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
        Geom_Curve UU1, UU2, VV1, VV2;
        Geom_Surface S = BRep_Tool.Surface(myFace1, out L)!;
        if (!L.IsIdentity())
        {
            S = (Geom_Surface)S.Transformed(L.Transformation());
        }

        if (myIsoU1)
        {
            if (!BRep_Tool.Degenerated(Edge1) && !Inv1)
            {
                UU1 = S.UIso(Umin);
                GeomAdaptor_Curve Dummy = new(UU1);
                if (Dummy.GetType() == GeomAbs_Circle && Dummy.Circle().Radius() < eps)
                {
                    UU1 = S.UIso(Umax);
                }
            }
            else
            {
                UU1 = S.UIso(Umax);
                GeomAdaptor_Curve Dummy = new(UU1);
                if (Dummy.GetType() == GeomAbs_Circle && Dummy.Circle().Radius() < eps)
                {
                    UU1 = S.UIso(Umin);
                }
            }
            VV1 = S.VIso(Vmin);
        }
        else
        {
            if (!BRep_Tool.Degenerated(Edge1) && !Inv1)
            {
                UU1 = S.VIso(Vmin);
                GeomAdaptor_Curve Dummy = new(UU1);
                if (Dummy.GetType() == GeomAbs_Circle && Dummy.Circle().Radius() < eps)
                {
                    UU1 = S.VIso(Vmax);
                }
            }
            else
            {
                UU1 = S.VIso(Vmax);
                GeomAdaptor_Curve Dummy = new(UU1);
                if (Dummy.GetType() == GeomAbs_Circle && Dummy.Circle().Radius() < eps)
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
        GeomAdaptor_Surface GAS1 = new(S);
        GeomAbs_SurfaceType Type1 = GAS1.GetType();

        if (UU1.IsPeriodic())
        {
            ElCLib.AdjustPeriodic(UU1.FirstParameter(), UU1.LastParameter(), Precision.PConfusion(), ref Umin, ref Umax);
        }
        if (VV1.IsPeriodic())
        {
            ElCLib.AdjustPeriodic(VV1.FirstParameter(), VV1.LastParameter(), Precision.PConfusion(), ref Vmin, ref Vmax);
        }
        // end try duplication

        myU1 = new Geom2dAdaptor_Curve(GeomProjLib.Curve2d(UU1, BasisPlane)!, Umin, Umax);

        UU1.D1(Umin, out P, out DZ);
        RefPlane = new Geom_Plane(new gp_Ax3(P, new gp_Dir(DZ), gp.DZ()));

        myV1 = new Geom2dAdaptor_Curve(GeomProjLib.Curve2d(VV1, RefPlane)!, Vmin, Vmax);

        First = true;
        for (Exp.Init(myFace2, TopAbs_EDGE); Exp.More(); Exp.Next())
        {
            TopoDS_Edge CurEdge = TopoDS.Edge(Exp.Current());
            BRep_Tool.UVPoints(CurEdge, myFace2, out P1, out P2);
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
        S = BRep_Tool.Surface(myFace2, out L)!;

        if (!L.IsIdentity())
        {
            S = (Geom_Surface)S.Transformed(L.Transformation());
        }

        if (myIsoU2)
        {
            if (!BRep_Tool.Degenerated(Edge2) && !Inv2)
            {
                UU2 = S.UIso(Umin);
                GeomAdaptor_Curve Dummy = new(UU2);
                if (Dummy.GetType() == GeomAbs_Circle && Dummy.Circle().Radius() < eps)
                {
                    UU2 = S.UIso(Umax);
                }
            }
            else
            {
                UU2 = S.UIso(Umax);
                GeomAdaptor_Curve Dummy = new(UU2);
                if (Dummy.GetType() == GeomAbs_Circle && Dummy.Circle().Radius() < eps)
                {
                    UU2 = S.UIso(Umin);
                }
            }
            VV2 = S.VIso(Vmin);
        }
        else
        {
            if (!BRep_Tool.Degenerated(Edge2) && !Inv2)
            {
                UU2 = S.VIso(Vmin);
                GeomAdaptor_Curve Dummy = new(UU2);
                if (Dummy.GetType() == GeomAbs_Circle && Dummy.Circle().Radius() < eps)
                {
                    UU2 = S.VIso(Vmax);
                }
            }
            else
            {
                UU2 = S.VIso(Vmax);
                GeomAdaptor_Curve Dummy = new(UU2);
                if (Dummy.GetType() == GeomAbs_Circle && Dummy.Circle().Radius() < eps)
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
        GeomAdaptor_Surface GAS2 = new(S);
        GeomAbs_SurfaceType Type2 = GAS2.GetType();

        if (UU2.IsPeriodic())
        {
            ElCLib.AdjustPeriodic(UU2.FirstParameter(), UU2.LastParameter(), Precision.PConfusion(), ref Umin, ref Umax);
        }
        if (VV2.IsPeriodic())
        {
            ElCLib.AdjustPeriodic(VV2.FirstParameter(), VV2.LastParameter(), Precision.PConfusion(), ref Vmin, ref Vmax);
        }
        // end try duplication

        myU2 = new Geom2dAdaptor_Curve(GeomProjLib.Curve2d(UU2, BasisPlane)!, Umin, Umax);

        UU2.D1(Umin, out P, out DZ);
        RefPlane = new Geom_Plane(new gp_Ax3(P, new gp_Dir(DZ), gp.DZ()));
        myV2 = new Geom2dAdaptor_Curve(GeomProjLib.Curve2d(VV2, RefPlane)!, Vmin, Vmax);

        // eval if in a particular case.
        // Particular case if :
        //     1) - Straight Bissectrice
        //        - Bissectrice orthogonal to the base element.
        //        ==> Iso on 2 faces.
        //     2) - Straight Bissectrice
        //        - 2 surfaces are planes.
        myCont = GeomAbs_C0;

        if (myBis.GetType() == GeomAbs_Line)
        {
            double DeltaU = myBis.LastParameter() - myBis.FirstParameter();
            gp_Pnt2d aPnt1 = ValueOnF1(myBis.FirstParameter() + 0.1 * DeltaU);
            gp_Pnt2d aPnt2 = ValueOnF1(myBis.FirstParameter() + 0.9 * DeltaU);
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
                myCont = GeomAbs_G1;
            }

            if ((Type1 == GeomAbs_Plane) && (Type2 == GeomAbs_Plane))
            {
                myKPart = 2;
            }
        }
    }

    /// <summary>True if the bisector is a line that gives isos on both faces, or lies between two planes.</summary>
    public bool IsParticularCase() => myKPart != 0;

    /// <summary>The continuity of the line.</summary>
    public GeomAbs_Shape Continuity() => myCont;

    /// <summary>In a particular case, the 3D curve and the two pcurves, built exactly.</summary>
    public void Curves(out Geom_Curve? Curve, out Geom2d_Curve? PCurve1, out Geom2d_Curve? PCurve2)
    {
        Curve = null;
        PCurve1 = null;
        PCurve2 = null;
        if (myKPart == 1)
        {
            gp_Pnt2d P1, P2, PMil;
            double f, l;

            P1 = ValueOnF1(myBis.FirstParameter());
            P2 = ValueOnF1(myBis.LastParameter());

            // find value of the with medium point
            // the ends can be degenerated points.

            PMil = ValueOnF1(0.5 * (myBis.FirstParameter() + myBis.LastParameter()));

            Geom_Surface S = BRep_Tool.Surface(myFace1, out TopLoc_Location L)!;
            if (!L.IsIdentity())
            {
                S = (Geom_Surface)S.Transformed(L.Transformation());
            }

            bool Sens;
            if (!myIsoU1)
            {
                Curve = S.UIso(PMil.X());
                Sens = P1.Y() < P2.Y();
                if (Sens)
                {
                    Curve = new Geom_TrimmedCurve(Curve, P1.Y(), P2.Y(), Sens);
                }
                else
                {
                    Curve = new Geom_TrimmedCurve(Curve, P2.Y(), P1.Y(), Sens);
                }

                f = Curve.FirstParameter();
                l = Curve.LastParameter();
                if (Sens)
                {
                    PCurve1 = new Geom2d_Line(new gp_Pnt2d(PMil.X(), P1.Y() - f), gp.DY2d());
                }
                else
                {
                    PCurve1 = new Geom2d_Line(new gp_Pnt2d(PMil.X(), P1.Y() + f), gp.DY2d().Reversed());
                }
                PCurve1 = new Geom2d_TrimmedCurve(PCurve1, f, l);
            }
            else
            {
                Curve = S.VIso(PMil.Y());
                Sens = P1.X() < P2.X();
                if (Sens)
                {
                    Curve = new Geom_TrimmedCurve(Curve, P1.X(), P2.X(), Sens);
                }
                else
                {
                    Curve = new Geom_TrimmedCurve(Curve, P2.X(), P1.X(), Sens);
                }

                f = Curve.FirstParameter();
                l = Curve.LastParameter();
                if (Sens)
                {
                    PCurve1 = new Geom2d_Line(new gp_Pnt2d(P1.X() - f, PMil.Y()), gp.DX2d());
                }
                else
                {
                    PCurve1 = new Geom2d_Line(new gp_Pnt2d(P1.X() + f, PMil.Y()), gp.DX2d().Reversed());
                }
                PCurve1 = new Geom2d_TrimmedCurve(PCurve1, f, l);
            }

            P1 = ValueOnF2(myBis.FirstParameter());
            P2 = ValueOnF2(myBis.LastParameter());
            PMil = ValueOnF2(0.5 * (myBis.FirstParameter() + myBis.LastParameter()));

            if (!myIsoU2)
            {
                Sens = P1.Y() < P2.Y();
                if (Sens)
                {
                    PCurve2 = new Geom2d_Line(new gp_Pnt2d(PMil.X(), (P1.Y() - f)), gp.DY2d());
                }
                else
                {
                    PCurve2 = new Geom2d_Line(new gp_Pnt2d(PMil.X(), (P1.Y() + f)), gp.DY2d().Reversed());
                }
            }
            else
            {
                Sens = P1.X() < P2.X();
                if (Sens)
                {
                    PCurve2 = new Geom2d_Line(new gp_Pnt2d(P1.X() - f, PMil.Y()), gp.DX2d());
                }
                else
                {
                    PCurve2 = new Geom2d_Line(new gp_Pnt2d(P1.X() + f, PMil.Y()), gp.DX2d().Reversed());
                }
            }
            PCurve2 = new Geom2d_TrimmedCurve(PCurve2, f, l);
        }
        else if (myKPart == 2)
        {
            Geom_Surface S = BRep_Tool.Surface(myFace1, out TopLoc_Location L)!;
            if (!L.IsIdentity())
            {
                S = (Geom_Surface)S.Transformed(L.Transformation());
            }

            if (S is Geom_RectangularTrimmedSurface aRTS1)
            {
                S = aRTS1.BasisSurface();
            }

            Geom_Plane Plane = (Geom_Plane)S;
            // eval the 3d curve corresponding to the bissectrice.
            gp_Pnt2d P = myBis.Line().Location();
            gp_Dir2d D = myBis.Line().Direction();
            Geom_Line Line = new(new gp_Pnt(P.X(), P.Y(), 0.0), new gp_Dir(D.X(), D.Y(), 0.0));
            Geom_TrimmedCurve TLine = new(Line, myBis.FirstParameter(), myBis.LastParameter());
            Curve = GeomProjLib.ProjectOnPlane(TLine, Plane, gp.DZ(), false);

            // eval PCurve1
            PCurve1 = GeomProjLib.Curve2d(Curve!, Plane);

            // eval PCurve2
            S = BRep_Tool.Surface(myFace2, out L)!;
            if (!L.IsIdentity())
            {
                S = (Geom_Surface)S.Transformed(L.Transformation());
            }
            if (S is Geom_RectangularTrimmedSurface aRTS2)
            {
                S = aRTS2.BasisSurface();
            }
            Plane = (Geom_Plane)S;
            PCurve2 = GeomProjLib.Curve2d(Curve!, Plane);
        }
    }

    /// <inheritdoc/>
    public override double FirstParameter() => myBis.FirstParameter();

    /// <inheritdoc/>
    public override double LastParameter() => myBis.LastParameter();

    /// <summary>The 3D point at <paramref name="U"/> (on the first face).</summary>
    public gp_Pnt Value(double U)
    {
        Geom_Surface S = BRep_Tool.Surface(myFace1, out TopLoc_Location L)!;

        gp_Pnt2d P2d = ValueOnF1(U);

        gp_Pnt P3d = S.Value(P2d.X(), P2d.Y());
        P3d.Transform(L.Transformation());

        return P3d;
    }

    private static gp_Pnt2d ValueOnFace(double U,
                                        Geom2dAdaptor_Curve TheBis,
                                        Geom2dAdaptor_Curve TheU,
                                        Geom2dAdaptor_Curve TheV,
                                        bool IsIsoU)
    {
        gp_Pnt2d P = TheBis.Value(U);

        Geom2dAPI_ProjectPointOnCurve Ext = new(P, TheU.Curve()!, TheU.FirstParameter(), TheU.LastParameter());
        //
        const double mult = 5.0;
        double eps = mult * Precision.Confusion();
        //
        double UU = 0.0, Dist = Precision.Infinite(), D1, D2;

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
                if (TheU.GetType() == GeomAbs_Circle)
                {
                    gp_Vec2d Axis = new(TheU.Circle().XAxis().Direction());
                    gp_Vec2d D12d = TheBis.DN(TheBis.FirstParameter(), 1);
                    double Ang = Axis.Angle(D12d);
                    if (!TheU.Circle().IsDirect())
                    {
                        Ang = -Ang;
                    }
                    UU = ElCLib.InPeriod(Ang, TheU.FirstParameter(), TheU.FirstParameter() + 2 * Math.PI);
                    Dist = TheU.Circle().Radius();
                }
                else
                {
                    throw new Standard_ConstructionError("BRepFill_MultiLine: ValueOnFace");
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

        double Tol = Precision.Confusion();
        double VV;

        gp_Pnt2d PF = TheV.Value(TheV.FirstParameter());
        gp_Pnt2d PL = TheV.Value(TheV.LastParameter());

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

            Geom2d_Line Line = new(new gp_Pnt2d(0.0, Dist), gp.DX2d());

            Geom2dAdaptor_Curve Cu1 = TheV;
            Geom2dAdaptor_Curve Cu2 = new(Line);

            double TolConf = 0.0;

            Geom2dInt_GInter Intersector = new(Cu1, Cu2, TolConf, Tol);

            if (!Intersector.IsDone())
            {
                return new gp_Pnt2d(0.0, 0.0);
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
                    gp_Pnt2d PS1 = TheV.Value(VS1);
                    gp_Pnt2d PS2 = TheV.Value(VS2);
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
            return new gp_Pnt2d(VV, UU);
        }
        else
        {
            return new gp_Pnt2d(UU, VV);
        }
    }

    /// <summary>The point at <paramref name="U"/> on the first face's parameter plane.</summary>
    public gp_Pnt2d ValueOnF1(double U) => ValueOnFace(U, myBis, myU1, myV1, myIsoU1);

    /// <summary>The point at <paramref name="U"/> on the second face's parameter plane.</summary>
    public gp_Pnt2d ValueOnF2(double U) => ValueOnFace(U, myBis, myU2, myV2, myIsoU2);

    /// <summary>The 3D point and both 2D points at <paramref name="U"/>.</summary>
    public void Value3dOnF1OnF2(double U, out gp_Pnt P3d, out gp_Pnt2d PF1, out gp_Pnt2d PF2)
    {
        PF1 = ValueOnFace(U, myBis, myU1, myV1, myIsoU1);
        PF2 = ValueOnFace(U, myBis, myU2, myV2, myIsoU2);

        Geom_Surface S = BRep_Tool.Surface(myFace1, out TopLoc_Location L)!;
        P3d = S.Value(PF1.X(), PF1.Y());
        P3d.Transform(L.Transformation());
    }

    /// <inheritdoc/>
    public override bool Value(double theT, NCollection_Array1<gp_Pnt2d> thePnt2d, NCollection_Array1<gp_Pnt> thePnt)
    {
        thePnt[1] = Value(theT);
        thePnt2d[1] = ValueOnF1(theT);
        thePnt2d[2] = ValueOnF2(theT);
        return true;
    }

    /// <inheritdoc/>
    public override bool D1(double theT, NCollection_Array1<gp_Vec2d> theVec2d, NCollection_Array1<gp_Vec> theVec) => false;
}
