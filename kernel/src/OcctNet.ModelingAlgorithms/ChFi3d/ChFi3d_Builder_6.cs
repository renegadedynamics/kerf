// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_Builder_6;
using static Occt.ChFiDS_ErrorStatus;
using static Occt.ChFiDS_State;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;
public partial class ChFi3d_Builder
{

    // modif : jlr branchement F(t) pour Edge/Face
    //  Modified by skv - Wed Jun  9 17:16:26 2004 OCC5898
    //  modified by Edward AGAPOV (eap) Fri Feb  8 2002 (bug occ67 == BUC61052)
    //  ComputeData(), case where BRepBlend_Walking::Continu() can't get up to Target




    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=======================================================================
    // function : CompBlendPoint
    // purpose  : create BlendPoint corresponding to a tangency on Vertex
    // pmn : 15/10/1997 : returns false, if there is no pcurve
    //=======================================================================

    //=======================================================================
    // function :  UpdateLine
    // purpose  : Updates extremities after a partial invalidation
    //=======================================================================

    //=======================================================================
    // function : CompleteData
    // purpose  : Calculates curves and CommonPoints from the data
    //           calculated by filling.
    //=======================================================================

    internal bool CompleteData(ChFiDS_SurfData? Data, Geom_Surface? Surfcoin, Adaptor3d_Surface? S1, Geom2d_Curve? PC1, Adaptor3d_Surface? S2, Geom2d_Curve? PC2, TopAbs_Orientation Or, bool On1, bool Gd1, bool Gd2, bool Gf1, bool Gf2)
    {
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        Data.ChangeSurf(DStr.AddSurface(new TopOpeBRepDS_Surface(Surfcoin, tolapp3d)));

        double UFirst = 0.0, ULast = 0.0, VFirst = 0.0, VLast = 0.0;
        Surfcoin.Bounds(out UFirst, out ULast, out VFirst, out VLast);
        if (!Gd1)
        {
            Data.ChangeVertexFirstOnS1().SetPoint(Surfcoin.Value(UFirst, VFirst));
        }
        if (!Gd2)
        {
            Data.ChangeVertexFirstOnS2().SetPoint(Surfcoin.Value(UFirst, VLast));
        }
        if (!Gf1)
        {
            Data.ChangeVertexLastOnS1().SetPoint(Surfcoin.Value(ULast, VFirst));
        }
        if (!Gf2)
        {
            Data.ChangeVertexLastOnS2().SetPoint(Surfcoin.Value(ULast, VLast));
        }

        // calculate curves side S1
        Geom_Curve? Crv3d1 = null;
        if (PC1 is not null)
        {
            Crv3d1 = Surfcoin.VIso(VFirst);
        }
        gp_Pnt2d pd1 = new(UFirst, VFirst), pf1 = new(ULast, VFirst);
        gp_Lin2d lfil1 = new(pd1, new gp_Dir2d(new gp_Vec2d(pd1, pf1)));
        Geom2d_Curve? PCurveOnSurf = new Geom2d_Line(lfil1);
        TopAbs_Orientation tra1 = TopAbs_FORWARD, orsurf = Or;
        double x = 0.0, y = 0.0, w = 0.5 * (UFirst + ULast);
        gp_Pnt p = new();
        gp_Vec du = new(), dv = new();
        Geom2d_Curve? c2dtrim = null;
        double tolreached = 1.0e-5;
        if (PC1 is not null)
        {
            GeomAdaptor_Curve? hcS1 = new GeomAdaptor_Curve(Crv3d1);
            c2dtrim = new Geom2d_TrimmedCurve(PC1, UFirst, ULast);
            ChFi3d_SameParameter(hcS1, ref c2dtrim, S1, tolapp3d, out tolreached);
            c2dtrim.Value(w).Coord(out x, out y);
            S1.D1(x, y, out p, out du, out dv);
            gp_Vec nf = du.Crossed(dv);
            Surfcoin.D1(w, VFirst, out p, out du, out dv);
            gp_Vec ns = du.Crossed(dv);
            if (nf.Dot(ns) > 0.0)
            {
                tra1 = TopAbs_REVERSED;
            }
            else if (On1)
            {
                orsurf = TopAbs.Reverse(orsurf);
            }
        }
        int Index1OfCurve = DStr.AddCurve(new TopOpeBRepDS_Curve(Crv3d1, tolreached));
        ref ChFiDS_FaceInterference Fint1 = ref Data.ChangeInterferenceOnS1();
        Fint1.SetFirstParameter(UFirst);
        Fint1.SetLastParameter(ULast);
        Fint1.SetInterference(Index1OfCurve, tra1, c2dtrim, PCurveOnSurf);
        // calculate curves side S2
        Geom_Curve? Crv3d2 = null;
        if (PC2 is not null)
        {
            Crv3d2 = Surfcoin.VIso(VLast);
        }
        gp_Pnt2d pd2 = new(UFirst, VLast), pf2 = new(ULast, VLast);
        gp_Lin2d lfil2 = new(pd2, new gp_Dir2d(new gp_Vec2d(pd2, pf2)));
        PCurveOnSurf = new Geom2d_Line(lfil2);
        TopAbs_Orientation tra2 = TopAbs_FORWARD;
        if (PC2 is not null)
        {
            GeomAdaptor_Curve? hcS2 = new GeomAdaptor_Curve(Crv3d2);
            c2dtrim = new Geom2d_TrimmedCurve(PC2, UFirst, ULast);
            ChFi3d_SameParameter(hcS2, ref c2dtrim, S2, tolapp3d, out tolreached);
            c2dtrim.Value(w).Coord(out x, out y);
            S2.D1(x, y, out p, out du, out dv);
            gp_Vec np = du.Crossed(dv);
            Surfcoin.D1(w, VLast, out p, out du, out dv);
            gp_Vec ns = du.Crossed(dv);
            if (np.Dot(ns) < 0.0)
            {
                tra2 = TopAbs_REVERSED;
                if (!On1)
                {
                    orsurf = TopAbs.Reverse(orsurf);
                }
            }
        }
        int Index2OfCurve = DStr.AddCurve(new TopOpeBRepDS_Curve(Crv3d2, tolreached));
        ref ChFiDS_FaceInterference Fint2 = ref Data.ChangeInterferenceOnS2();
        Fint2.SetFirstParameter(UFirst);
        Fint2.SetLastParameter(ULast);
        Fint2.SetInterference(Index2OfCurve, tra2, c2dtrim, PCurveOnSurf);
        Data.ChangeOrientation() = orsurf;
        return true;
    }

    //=======================================================================
    // function : CompleteData
    // purpose  : Calculates the surface of curves and eventually
    //           CommonPoints from the data calculated in ComputeData.
    //
    //  11/08/1996 : Use of F(t)
    //
    //=======================================================================

    internal bool CompleteData(ChFiDS_SurfData? Data, Blend_Function Func, BRepBlend_Line? lin, Adaptor3d_Surface? S1, Adaptor3d_Surface? S2, TopAbs_Orientation Or1, bool Gd1, bool Gd2, bool Gf1, bool Gf2, bool Reversed = false)
    {
        BRepBlend_AppFunc? TheFunc = new BRepBlend_AppFunc(lin, Func, tolapp3d, 1.0e-5);

        int Degmax = 20, Segmax = 5000;
        BRepBlend_AppSurface approx = new(TheFunc, lin.Point(1).Parameter(), lin.Point(lin.NbPoints()).Parameter(),
                                          tolapp3d, 1.0e-5, // tolapp2d, tolerance max
                                          tolappangle,      // Contact G1
                                          myConti, Degmax, Segmax);
        if (!approx.IsDone())
        {
            return false;
        }
        return StoreData(Data, approx, lin, S1, S2, Or1, Gd1, Gd2, Gf1, Gf2, Reversed);
    }

    //=======================================================================
    // function : CompleteData
    // purpose  : New overload for functions surf/rst
    // jlr le 28/07/97 branchement F(t)
    //=======================================================================

    internal bool CompleteData(ChFiDS_SurfData? Data, Blend_SurfRstFunction Func, BRepBlend_Line? lin, Adaptor3d_Surface? S1, Adaptor3d_Surface? S2, TopAbs_Orientation Or, bool Reversed = false)
    {
        BRepBlend_AppFuncRst? TheFunc = new BRepBlend_AppFuncRst(lin, Func, tolapp3d, 1.0e-5);
        BRepBlend_AppSurface approx = new(TheFunc, lin.Point(1).Parameter(), lin.Point(lin.NbPoints()).Parameter(),
                                          tolapp3d, 1.0e-5, // tolapp2d, tolerance max
                                          tolappangle,      // Contact G1
                                          myConti);
        if (!approx.IsDone())
        {
            return false;
        }

        return StoreData(Data, approx, lin, S1, S2, Or, false, false, false, false, Reversed);
    }

    //=======================================================================
    // function : CompleteData
    // purpose  : New overload for functions rst/rst
    // jlr le 28/07/97 branchement F(t)
    //=======================================================================

    internal bool CompleteData(ChFiDS_SurfData? Data, Blend_RstRstFunction Func, BRepBlend_Line? lin, Adaptor3d_Surface? S1, Adaptor3d_Surface? S2, TopAbs_Orientation Or)
    {
        BRepBlend_AppFuncRstRst? TheFunc = new BRepBlend_AppFuncRstRst(lin, Func, tolapp3d, 1.0e-5);
        BRepBlend_AppSurface approx = new(TheFunc, lin.Point(1).Parameter(), lin.Point(lin.NbPoints()).Parameter(),
                                          tolapp3d, 1.0e-5, // tolapp2d, tolerance max
                                          tolappangle,      // Contact G1
                                          myConti);
        if (!approx.IsDone())
        {
            return false;
        }

        return StoreData(Data, approx, lin, S1, S2, Or, false, false, false, false);
    }

    //=======================================================================
    // function : StoreData
    // purpose  : Copy of an approximation result in SurfData.
    //=======================================================================

    internal bool StoreData(ChFiDS_SurfData? Data, AppBlend_Approx approx, BRepBlend_Line? lin, Adaptor3d_Surface? S1, Adaptor3d_Surface? S2, TopAbs_Orientation Or1, bool Gd1, bool Gd2, bool Gf1, bool Gf2, bool Reversed = false)
    {
        // Small control tools.
        // OCCT keeps a thread_local scratch adaptor; it is loaded before every use and never read across calls,
        // so it is a local here (plan 2.3).
        GeomAdaptor_Curve checkcurve = new GeomAdaptor_Curve();
        GeomAdaptor_Curve chc = checkcurve;
        double tolget3d = 0.0, tolget2d = 0.0, tolaux = 0.0, tolC1 = 0.0, tolcheck = 0.0;
        double tolC2 = 0.0;
        approx.TolReached(out tolget3d, out tolget2d);
        tolaux = approx.TolCurveOnSurf(1);
        tolC1 = tolget3d + tolaux;
        if (S2 is not null)
        {
            tolaux = approx.TolCurveOnSurf(2);
            tolC2 = tolget3d + tolaux;
        }

        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        // By default parametric space is created using a square surface
        // to be able to parameterize in U by # R*teta // a revoir lbo 29/08/97
        NCollection_Array1<double> ku = approx.SurfUKnots();
        NCollection_Array1<double> kv = approx.SurfVKnots();
        double larg = (kv[kv.Upper()] - kv[kv.Lower()]);
        NCollection_Array1<double> kku = ku; // OCCT casts the constness away: the same array
        BSplCLib.Reparametrize(0.0, larg, kku);
        Geom_BSplineSurface? Surf = new Geom_BSplineSurface(approx.SurfPoles(), approx.SurfWeights(), kku, kv,
                                                            approx.SurfUMults(), approx.SurfVMults(),
                                                            approx.UDegree(), approx.VDegree());
        // extension of the surface

        double length1 = 0.0, length2 = 0.0;
        length1 = Data.FirstExtensionValue();
        length2 = Data.LastExtensionValue();

        Geom_BoundedSurface? aBndSurf = Surf;
        bool ext1 = false, ext2 = false;
        double eps = Max(tolget3d, 2.0 * Precision.Confusion());
        if (length1 > eps)
        {
            gp_Pnt P11 = new(), P21 = new();
            P11 = Surf.Pole(1, 1);
            P21 = Surf.Pole(Surf.NbUPoles(), 1);
            if (P11.Distance(P21) > eps)
            {
                // to avoid extending surface with singular boundary
                GeomLib.ExtendSurfByLength(ref aBndSurf, length1, 1, false, false);
                ext1 = true;
            }
        }
        if (length2 > eps)
        {
            gp_Pnt P12 = new(), P22 = new();
            P12 = Surf.Pole(1, Surf.NbVPoles());
            P22 = Surf.Pole(Surf.NbUPoles(), Surf.NbVPoles());
            if (P12.Distance(P22) > eps)
            {
                // to avoid extending surface with singular boundary
                GeomLib.ExtendSurfByLength(ref aBndSurf, length2, 1, false, true);
                ext2 = true;
            }
        }
        Surf = (aBndSurf as Geom_BSplineSurface);

        // Correction of surface on extremities
        if (!ext1)
        {
            gp_Pnt P11 = new(), P21 = new();
            P11 = lin.StartPointOnFirst().Value();
            P21 = lin.StartPointOnSecond().Value();
            Surf.SetPole(1, 1, P11);
            Surf.SetPole(Surf.NbUPoles(), 1, P21);
        }
        if (!ext2)
        {
            gp_Pnt P12 = new(), P22 = new();
            P12 = lin.EndPointOnFirst().Value();
            P22 = lin.EndPointOnSecond().Value();
            Surf.SetPole(1, Surf.NbVPoles(), P12);
            Surf.SetPole(Surf.NbUPoles(), Surf.NbVPoles(), P22);
        }

        Data.ChangeSurf(DStr.AddSurface(new TopOpeBRepDS_Surface(Surf, tolget3d)));

        double UFirst = 0.0, ULast = 0.0, VFirst = 0.0, VLast = 0.0, pppdeb = 0.0, pppfin = 0.0;
        Surf.Bounds(out UFirst, out ULast, out VFirst, out VLast);
        BRepAdaptor_Curve2d brc = new();
        BRepAdaptor_Curve CArc = new();
        BRepAdaptor_Surface? BS1 = (S1 as BRepAdaptor_Surface);
        BRepAdaptor_Surface? BS2 = (S2 as BRepAdaptor_Surface);
        Geom2dAPI_ProjectPointOnCurve projector = new();

        double Uon1 = UFirst, Uon2 = ULast;
        int ion1 = 1, ion2 = 2;
        if (Reversed)
        {
            Uon1 = ULast;
            Uon2 = UFirst;
            ion1 = 2;
            ion2 = 1;
        }

        // The SurfData is filled in what concerns S1,
        Geom_Curve? Crv3d1 = Surf.UIso(Uon1);
        gp_Pnt2d pori1 = new(Uon1, 0.0);
        gp_Lin2d lfil1 = new(pori1, gp.DY2d());
        Geom2d_Curve? PCurveOnSurf = new Geom2d_Line(lfil1);
        Geom2d_Curve? PCurveOnFace = null;
        PCurveOnFace = new Geom2d_BSplineCurve(approx.Curve2dPoles(ion1), approx.Curves2dKnots(), approx.Curves2dMults(),
                                               approx.Curves2dDegree());

        double par1 = PCurveOnFace.FirstParameter();
        double par2 = PCurveOnFace.LastParameter();
        chc.Load(Crv3d1, par1, par2);

        if (!ChFi3d_CheckSameParameter(checkcurve, PCurveOnFace, S1, tolC1, out tolcheck))
        {
            tolC1 = tolcheck;
        }
        int Index1OfCurve = DStr.AddCurve(new TopOpeBRepDS_Curve(Crv3d1, tolC1));

        double uarc = 0.0, utg = 0.0;
        if (Gd1)
        {
            TopoDS_Face forwfac = BS1.Face();
            forwfac.Orientation(TopAbs_FORWARD);
            brc.Initialize(Data.VertexFirstOnS1().Arc(), forwfac);
            ref ChFiDS_CommonPoint V = ref Data.ChangeVertexFirstOnS1();
            CArc.Initialize(V.Arc());
            CompParam(brc, PCurveOnFace, ref uarc, ref utg, V.ParameterOnArc(), V.Parameter());
            tolcheck = CArc.Value(uarc).Distance(V.Point());
            V.SetArc(tolC1 + tolcheck, V.Arc(), uarc, V.TransitionOnArc());
            pppdeb = utg;
        }
        else
        {
            pppdeb = VFirst;
        }
        if (Gf1)
        {
            TopoDS_Face forwfac = BS1.Face();
            forwfac.Orientation(TopAbs_FORWARD);
            ref ChFiDS_CommonPoint V = ref Data.ChangeVertexLastOnS1();
            brc.Initialize(V.Arc(), forwfac);
            CArc.Initialize(V.Arc());
            CompParam(brc, PCurveOnFace, ref uarc, ref utg, V.ParameterOnArc(), V.Parameter());
            tolcheck = CArc.Value(uarc).Distance(V.Point());
            V.SetArc(tolC1 + tolcheck, V.Arc(), uarc, V.TransitionOnArc());
            pppfin = utg;
        }
        else
        {
            pppfin = VLast;
        }
        ref ChFiDS_FaceInterference Fint1 = ref Data.ChangeInterferenceOnS1();
        Fint1.SetFirstParameter(pppdeb);
        Fint1.SetLastParameter(pppfin);
        TopAbs_Orientation TraOn1 = default;
        if (Reversed)
        {
            TraOn1 = ChFi3d_TrsfTrans(lin.TransitionOnS2());
        }
        else
        {
            TraOn1 = ChFi3d_TrsfTrans(lin.TransitionOnS1());
        }
        Fint1.SetInterference(Index1OfCurve, TraOn1, PCurveOnFace, PCurveOnSurf);

        // SurfData is filled in what concerns S2,
        Geom_Curve? Crv3d2 = Surf.UIso(Uon2);
        gp_Pnt2d pori2 = new(Uon2, 0.0);
        gp_Lin2d lfil2 = new(pori2, gp.DY2d());
        PCurveOnSurf = new Geom2d_Line(lfil2);
        if (S2 is not null)
        {
            PCurveOnFace = new Geom2d_BSplineCurve(approx.Curve2dPoles(ion2), approx.Curves2dKnots(),
                                                   approx.Curves2dMults(), approx.Curves2dDegree());
            chc.Load(Crv3d2, par1, par2);
            if (!ChFi3d_CheckSameParameter(checkcurve, PCurveOnFace, S2, tolC2, out tolcheck))
            {
                tolC2 = tolcheck;
            }
        }
        int Index2OfCurve = DStr.AddCurve(new TopOpeBRepDS_Curve(Crv3d2, tolC2));
        if (Gd2)
        {
            TopoDS_Face forwfac = BS2.Face();
            forwfac.Orientation(TopAbs_FORWARD);
            brc.Initialize(Data.VertexFirstOnS2().Arc(), forwfac);
            ref ChFiDS_CommonPoint V = ref Data.ChangeVertexFirstOnS2();
            CArc.Initialize(V.Arc());
            CompParam(brc, PCurveOnFace, ref uarc, ref utg, V.ParameterOnArc(), V.Parameter());
            tolcheck = CArc.Value(uarc).Distance(V.Point());
            V.SetArc(tolC2 + tolcheck, V.Arc(), uarc, V.TransitionOnArc());
            pppdeb = utg;
        }
        else
        {
            pppdeb = VFirst;
        }
        if (Gf2)
        {
            TopoDS_Face forwfac = BS2.Face();
            forwfac.Orientation(TopAbs_FORWARD);
            brc.Initialize(Data.VertexLastOnS2().Arc(), forwfac);
            ref ChFiDS_CommonPoint V = ref Data.ChangeVertexLastOnS2();
            CArc.Initialize(V.Arc());
            CompParam(brc, PCurveOnFace, ref uarc, ref utg, V.ParameterOnArc(), V.Parameter());
            tolcheck = CArc.Value(uarc).Distance(V.Point());
            V.SetArc(tolC2 + tolcheck, V.Arc(), uarc, V.TransitionOnArc());
            pppfin = utg;
        }
        else
        {
            pppfin = VLast;
        }
        ref ChFiDS_FaceInterference Fint2 = ref Data.ChangeInterferenceOnS2();
        Fint2.SetFirstParameter(pppdeb);
        Fint2.SetLastParameter(pppfin);
        if (S2 is not null)
        {
            TopAbs_Orientation TraOn2 = default;
            if (Reversed)
            {
                TraOn2 = ChFi3d_TrsfTrans(lin.TransitionOnS1());
            }
            else
            {
                TraOn2 = ChFi3d_TrsfTrans(lin.TransitionOnS2());
            }
            Fint2.SetInterference(Index2OfCurve, TraOn2, PCurveOnFace, PCurveOnSurf);
        }
        else
        {
            Geom2d_Curve? bidpc = null;
            Fint2.SetInterference(Index2OfCurve, TopAbs_FORWARD, bidpc, PCurveOnSurf);
        }

        // the orientation of the fillet in relation to the faces is evaluated,

        Adaptor3d_Surface? Sref = S1;
        PCurveOnFace = Fint1.PCurveOnFace();
        if (Reversed)
        {
            Sref = S2;
            PCurveOnFace = Fint2.PCurveOnFace();
        }

        //  Modified by skv - Wed Jun  9 17:16:26 2004 OCC5898 Begin
        //   gp_Pnt2d PUV = PCurveOnFace->Value((VFirst+VLast)/2.);
        //   gp_Pnt P;
        //   gp_Vec Du1,Du2,Dv1,Dv2;
        //   Sref->D1(PUV.X(),PUV.Y(),P,Du1,Dv1);
        //   Du1.Cross(Dv1);
        //   if (Or1 == TopAbs_REVERSED) Du1.Reverse();
        //   Surf->D1(UFirst,(VFirst+VLast)/2.,P,Du2,Dv2);
        //   Du2.Cross(Dv2);
        //   if (Du1.Dot(Du2)>0) Data->ChangeOrientation() = TopAbs_FORWARD;
        //   else Data->ChangeOrientation() = TopAbs_REVERSED;

        double aDelta = VLast - VFirst;
        int aDenom = 2;

        for (;;)
        {
            double aDeltav = aDelta / aDenom;
            double aParam = VFirst + aDeltav;
            gp_Pnt2d PUV = PCurveOnFace.Value(aParam);
            gp_Pnt P = new();
            gp_Vec Du1 = new(), Du2 = new(), Dv1 = new(), Dv2 = new();

            Sref.D1(PUV.X(), PUV.Y(), out P, out Du1, out Dv1);
            Du1.Cross(Dv1);

            if (Or1 == TopAbs_REVERSED)
            {
                Du1.Reverse();
            }

            Surf.D1(UFirst, aParam, out P, out Du2, out Dv2);
            Du2.Cross(Dv2);

            if (Du1.Magnitude() <= tolget3d || Du2.Magnitude() <= tolget3d)
            {
                aDenom++;

                if (Math.Abs(aDeltav) <= tolget2d)
                {
                    return false;
                }

                continue;
            }

            if (Du1.Dot(Du2) > 0)
            {
                Data.ChangeOrientation() = TopAbs_FORWARD;
            }
            else
            {
                Data.ChangeOrientation() = TopAbs_REVERSED;
            }

            break;
        }
        //  Modified by skv - Wed Jun  9 17:16:26 2004 OCC5898 End

        if (!Gd1 && S1 is not null)
        {
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref Data.ChangeVertex(true, ion1),
                                                        tolC1);
        }
        if (!Gf1 && S1 is not null)
        {
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref Data.ChangeVertex(false, ion1),
                                                        tolC1);
        }
        if (!Gd2 && S2 is not null)
        {
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref Data.ChangeVertex(true, ion2),
                                                        tolC2);
        }
        if (!Gf2 && S2 is not null)
        {
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref Data.ChangeVertex(false, ion2),
                                                        tolC2);
        }
        // Parameters on ElSpine
        int nbp = lin.NbPoints();
        Data.FirstSpineParam(lin.Point(1).Parameter());
        Data.LastSpineParam(lin.Point(nbp).Parameter());
        return true;
    }

    //=======================================================================
    // function : ComputeData
    // purpose  : Head of the path edge/face for the bypass of obstacle.
    //=======================================================================

    internal bool ComputeData(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor3d_TopolTool? I1, Adaptor3d_Surface? S2, Adaptor2d_Curve2d? PC2, Adaptor3d_TopolTool? I2, ref bool Decroch, Blend_SurfRstFunction Func, Blend_FuncInv FInv, Blend_SurfPointFuncInv FInvP, Blend_SurfCurvFuncInv FInvC, double PFirst, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, math_Vector Soldep, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst)
    {
        BRepBlend_SurfRstLineBuilder TheWalk = new(S1, I1, S2, PC2, I2);

        Data.FirstExtensionValue(0);
        Data.LastExtensionValue(0);

        bool reverse = (!Forward || Inside);
        double SpFirst = HGuide.FirstParameter();
        double SpLast = HGuide.LastParameter();
        double Target = SpLast;
        if (reverse)
        {
            Target = SpFirst;
        }
        double Targetsov = Target;

        double MS = MaxStep;
        int again = 0;
        int nbptmin = 3;// jlr
        int Nbpnt = 1;
        // the initial solution is reframed if necessary.
        math_Vector ParSol = new(1, 3);
        double NewFirst = PFirst;
        if (RecP || RecS || RecRst)
        {
            if (!TheWalk.PerformFirstSection(Func, FInv, FInvP, FInvC, PFirst, Target, Soldep, tolapp3d, tolapp2d, TolGuide,
                                                                              RecRst, RecP, RecS, ref NewFirst, ParSol))
            {
                return false;
            }
        }
        else
        {
            ParSol.Initialized(Soldep); // a copy of the values, as math_Vector::operator=
        }

        while (again < 2)
        {
            TheWalk.Perform(Func, FInv, FInvP, FInvC, NewFirst, Last, MS, tolapp3d, tolapp2d, TolGuide, ParSol, Fleche,
                                            Appro);

            if (!TheWalk.IsDone())
            {
                return false;
            }

            if (reverse)
            {
                if (!TheWalk.Complete(Func, FInv, FInvP, FInvC, SpLast))
                {
                }
            }

            Lin = TheWalk.Line();
            Nbpnt = Lin.NbPoints();
            if (Nbpnt <= 1 && again == 0)
            {
                again++;
                MS = MS / 50.0;
                Target = Targetsov;
            }
            else if (Nbpnt <= nbptmin && again == 0)
            {
                again++;
                double u1 = Lin.Point(1).Parameter();
                double u2 = Lin.Point(Nbpnt).Parameter();
                MS = (u2 - u1) / (nbptmin + 1.0);
                //      std::cout << " MS : " << MS << " u1 : " << u1 << " u2 : " << u2 << " nbptmin : " <<
                //      nbptmin << std::endl;
                Target = Targetsov;
            }
            else if (Nbpnt <= nbptmin)
            {
                return false;
            }
            else
            {
                again = 2;
            }
        }
        if (Forward)
        {
            Decroch = TheWalk.DecrochEnd();
        }
        else
        {
            Decroch = TheWalk.DecrochStart();
        }
        Last = Lin.Point(Nbpnt).Parameter();
        First = Lin.Point(1).Parameter();
        return true;
    }

    //=======================================================================
    // function : ComputeData
    // purpose  : Heading of the path edge/edge for the bypass of obstacle.
    //=======================================================================

    internal bool ComputeData(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor2d_Curve2d? PC1, Adaptor3d_TopolTool? I1, ref bool Decroch1, Adaptor3d_Surface? S2, Adaptor2d_Curve2d? PC2, Adaptor3d_TopolTool? I2, ref bool Decroch2, Blend_RstRstFunction Func, Blend_SurfCurvFuncInv FInv1, Blend_CurvPointFuncInv FInvP1, Blend_SurfCurvFuncInv FInv2, Blend_CurvPointFuncInv FInvP2, double PFirst, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, math_Vector Soldep, bool Inside, bool Appro, bool Forward, bool RecP1, bool RecRst1, bool RecP2, bool RecRst2)
    {
        BRepBlend_RstRstLineBuilder TheWalk = new(S1, PC1, I1, S2, PC2, I2);

        Data.FirstExtensionValue(0);
        Data.LastExtensionValue(0);

        bool reverse = (!Forward || Inside);
        double SpFirst = HGuide.FirstParameter();
        double SpLast = HGuide.LastParameter();
        double Target = SpLast;
        if (reverse)
        {
            Target = SpFirst;
        }
        double Targetsov = Target;

        double MS = MaxStep;
        int again = 0;
        int nbptmin = 3;// jlr
        int Nbpnt = 0;
        // the initial solution is reframed if necessary.
        math_Vector ParSol = new(1, 2);
        double NewFirst = PFirst;
        if (RecP1 || RecRst1 || RecP2 || RecRst2)
        {
            if (!TheWalk.PerformFirstSection(Func, FInv1, FInvP1, FInv2, FInvP2, PFirst, Target, Soldep, tolapp3d, TolGuide,
                                                                              RecRst1, RecP1, RecRst2, RecP2, ref NewFirst, ParSol))
            {
                return false;
            }
        }
        else
        {
            ParSol.Initialized(Soldep); // a copy of the values, as math_Vector::operator=
        }

        while (again < 2)
        {
            TheWalk.Perform(Func, FInv1, FInvP1, FInv2, FInvP2, NewFirst, Last, MS, tolapp3d, TolGuide, ParSol, Fleche,
                                            Appro);

            if (!TheWalk.IsDone())
            {
                return false;
            }

            if (reverse)
            {
                if (!TheWalk.Complete(Func, FInv1, FInvP1, FInv2, FInvP2, SpLast))
                {
                }
            }

            Lin = TheWalk.Line();
            Nbpnt = Lin.NbPoints();
            if (Nbpnt <= 1 && again == 0)
            {
                again++;
                MS = MS / 50.0;
                Target = Targetsov;
            }
            else if (Nbpnt <= nbptmin && again == 0)
            {
                again++;
                double u1 = Lin.Point(1).Parameter();
                double u2 = Lin.Point(Nbpnt).Parameter();
                MS = (u2 - u1) / (nbptmin + 1);
                Target = Targetsov;
            }
            else if (Nbpnt <= nbptmin)
            {
                return false;
            }
            else
            {
                again = 2;
            }
        }
        if (Forward)
        {
            Decroch1 = TheWalk.Decroch1End();
            Decroch2 = TheWalk.Decroch2End();
        }
        else
        {
            Decroch1 = TheWalk.Decroch1Start();
            Decroch2 = TheWalk.Decroch2Start();
        }
        Last = Lin.Point(Nbpnt).Parameter();
        First = Lin.Point(1).Parameter();
        return true;
    }

    //=======================================================================
    // function : SimulData
    // purpose  : Heading of the path edge/face for the bypass of obstacle in simulation mode.
    //=======================================================================

    internal bool SimulData(ChFiDS_SurfData? _unused1, ChFiDS_ElSpine? HGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor3d_TopolTool? I1, Adaptor3d_Surface? S2, Adaptor2d_Curve2d? PC2, Adaptor3d_TopolTool? I2, ref bool Decroch, Blend_SurfRstFunction Func, Blend_FuncInv FInv, Blend_SurfPointFuncInv FInvP, Blend_SurfCurvFuncInv FInvC, double PFirst, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, math_Vector Soldep, int NbSecMin, bool Inside, bool Appro, bool Forward, bool RecP, bool RecS, bool RecRst)
    {
        BRepBlend_SurfRstLineBuilder TheWalk = new(S1, I1, S2, PC2, I2);

        bool reverse = (!Forward || Inside);
        double SpFirst = HGuide.FirstParameter();
        double SpLast = HGuide.LastParameter();
        double Target = SpLast;
        if (reverse)
        {
            Target = SpFirst;
        }
        double Targetsov = Target;

        double MS = MaxStep;
        int again = 0;
        int Nbpnt = 0;
        // the starting solution is reframed if needed.
        math_Vector ParSol = new(1, 3);
        double NewFirst = PFirst;
        if (RecP || RecS || RecRst)
        {
            if (!TheWalk.PerformFirstSection(Func, FInv, FInvP, FInvC, PFirst, Target, Soldep, tolapp3d, tolapp2d, TolGuide,
                                                                              RecRst, RecP, RecS, ref NewFirst, ParSol))
            {
                return false;
            }
        }
        else
        {
            ParSol.Initialized(Soldep); // a copy of the values, as math_Vector::operator=
        }

        while (again < 2)
        {
            TheWalk.Perform(Func, FInv, FInvP, FInvC, NewFirst, Last, MS, tolapp3d, tolapp2d, TolGuide, ParSol, Fleche,
                                            Appro);
            if (!TheWalk.IsDone())
            {
                return false;
            }
            if (reverse)
            {
                if (!TheWalk.Complete(Func, FInv, FInvP, FInvC, SpLast))
                {
                }
            }
            Lin = TheWalk.Line();
            Nbpnt = Lin.NbPoints();
            if (Nbpnt <= 1 && again == 0)
            {
                again++;
                MS = MS / 50.0;
                Target = Targetsov;
            }
            else if (Nbpnt <= NbSecMin && again == 0)
            {
                again++;
                double u1 = Lin.Point(1).Parameter();
                double u2 = Lin.Point(Nbpnt).Parameter();
                MS = (u2 - u1) / (NbSecMin + 1);
                Target = Targetsov;
            }
            else if (Nbpnt <= NbSecMin)
            {
                return false;
            }
            else
            {
                again = 2;
            }
        }
        if (Forward)
        {
            Decroch = TheWalk.DecrochEnd();
        }
        else
        {
            Decroch = TheWalk.DecrochStart();
        }
        Last = Lin.Point(Nbpnt).Parameter();
        First = Lin.Point(1).Parameter();
        return true;
    }

    //=======================================================================
    // function : SimulData
    // purpose  : Heading of path edge/edge for the bypass
    //           of obstacle in simulation mode.
    //=======================================================================

    internal bool SimulData(ChFiDS_SurfData? _unused2, ChFiDS_ElSpine? HGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor2d_Curve2d? PC1, Adaptor3d_TopolTool? I1, ref bool Decroch1, Adaptor3d_Surface? S2, Adaptor2d_Curve2d? PC2, Adaptor3d_TopolTool? I2, ref bool Decroch2, Blend_RstRstFunction Func, Blend_SurfCurvFuncInv FInv1, Blend_CurvPointFuncInv FInvP1, Blend_SurfCurvFuncInv FInv2, Blend_CurvPointFuncInv FInvP2, double PFirst, double MaxStep, double Fleche, double TolGuide, ref double First, ref double Last, math_Vector Soldep, int NbSecMin, bool Inside, bool Appro, bool Forward, bool RecP1, bool RecRst1, bool RecP2, bool RecRst2)
    {
        BRepBlend_RstRstLineBuilder TheWalk = new(S1, PC1, I1, S2, PC2, I2);

        bool reverse = (!Forward || Inside);
        double SpFirst = HGuide.FirstParameter();
        double SpLast = HGuide.LastParameter();
        double Target = SpLast;
        if (reverse)
        {
            Target = SpFirst;
        }
        double Targetsov = Target;

        double MS = MaxStep;
        int again = 0;
        int Nbpnt = 0;
        // The initial solution is reframed if necessary.
        math_Vector ParSol = new(1, 2);
        double NewFirst = PFirst;
        if (RecP1 || RecRst1 || RecP2 || RecRst2)
        {
            if (!TheWalk.PerformFirstSection(Func, FInv1, FInvP1, FInv2, FInvP2, PFirst, Target, Soldep, tolapp3d, TolGuide,
                                                                              RecRst1, RecP1, RecRst2, RecP2, ref NewFirst, ParSol))
            {
                return false;
            }
        }
        else
        {
            ParSol.Initialized(Soldep); // a copy of the values, as math_Vector::operator=
        }

        while (again < 2)
        {
            TheWalk.Perform(Func, FInv1, FInvP1, FInv2, FInvP2, NewFirst, Last, MS, tolapp3d, TolGuide, ParSol, Fleche,
                                            Appro);
            if (!TheWalk.IsDone())
            {
                return false;
            }
            if (reverse)
            {
                if (!TheWalk.Complete(Func, FInv1, FInvP1, FInv2, FInvP2, SpLast))
                {
                }
            }
            Lin = TheWalk.Line();
            Nbpnt = Lin.NbPoints();
            if (Nbpnt <= 1 && again == 0)
            {
                again++;
                MS = MS / 50.0;
                Target = Targetsov;
            }
            else if (Nbpnt <= NbSecMin && again == 0)
            {
                again++;
                double u1 = Lin.Point(1).Parameter();
                double u2 = Lin.Point(Nbpnt).Parameter();
                MS = (u2 - u1) / (NbSecMin + 1);
                Target = Targetsov;
            }
            else if (Nbpnt <= NbSecMin)
            {
                return false;
            }
            else
            {
                again = 2;
            }
        }
        if (Forward)
        {
            Decroch1 = TheWalk.Decroch1End();
            Decroch2 = TheWalk.Decroch2End();
        }
        else
        {
            Decroch1 = TheWalk.Decroch1Start();
            Decroch2 = TheWalk.Decroch2Start();
        }

        Last = Lin.Point(Nbpnt).Parameter();
        First = Lin.Point(1).Parameter();
        return true;
    }

    //=======================================================================
    // function : ComputeData
    // purpose  : Construction of elementary fillet by path.
    //
    //=======================================================================

    internal bool ComputeData(ChFiDS_SurfData? Data, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor3d_TopolTool? I1, Adaptor3d_Surface? S2, Adaptor3d_TopolTool? I2, Blend_Function Func, Blend_FuncInv FInv, double PFirst, double MaxStep, double Fleche, double tolguide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, math_Vector Soldep, ref int intf, ref int intl, ref bool Gd1, ref bool Gd2, ref bool Gf1, ref bool Gf2, bool RecOnS1 = false, bool RecOnS2 = false)
    {
        // Get offset guide if exists
        ChFiDS_ElSpine? OffsetHGuide = null;
        if (Spine is not null && Spine.Mode() == ChFiDS_ChamfMode.ChFiDS_ConstThroatWithPenetrationChamfer)
        {
            NCollection_List<ChFiDS_ElSpine> ll = Spine.ChangeElSpines();
            NCollection_List<ChFiDS_ElSpine> ll_offset = Spine.ChangeOffsetElSpines();
            NCollection_List<ChFiDS_ElSpine>.Iterator ILES = new(ll), ILES_offset = new(ll_offset);
            for (; ILES.More(); ILES.Next(), ILES_offset.Next())
            {
                ChFiDS_ElSpine? aHElSpine = ILES.Value();
                if (aHElSpine == HGuide)
                {
                    OffsetHGuide = ILES_offset.Value();
                }
            }
        }

        // The extrensions are created in case of output of two domains
        // directly and not by path ( too hasardous ).
        Data.FirstExtensionValue(0);
        Data.LastExtensionValue(0);

        // The eventual faces are restored to test the jump of edge.
        TopoDS_Face F1 = new(), F2 = new();
        BRepAdaptor_Surface? HS = (S1 as BRepAdaptor_Surface);
        if (HS is not null)
        {
            F1 = HS.Face();
        }
        HS = (S2 as BRepAdaptor_Surface);
        if (HS is not null)
        {
            F2 = HS.Face();
        }

        // Path framing variables
        double TolGuide = tolguide;
        int nbptmin = 4;

        BRepBlend_Walking TheWalk = new(S1, S2, I1, I2, HGuide);

        // Start of removal, 2D path controls
        // that qui s'accomodent mal des surfaces a parametrages non homogenes
        // en u et en v are extinguished.
        TheWalk.Check2d(false);

        double MS = MaxStep;
        int Nbpnt = 0;
        double SpFirst = HGuide.FirstParameter();
        double SpLast = HGuide.LastParameter();

        // When the start point is inside, the path goes first to the left
        // to determine the Last for the periodicals.
        bool reverse = (!Forward || Inside);
        double Target = 0.0;
        if (reverse)
        {
            Target = SpFirst;
            if (intf == 0)
            {
                Target = Last;
            }
        }
        else
        {
            Target = SpLast + Math.Abs(SpLast);
            if (intl == 0)
            {
                Target = Last;
            }
        }

        // In case if the singularity is pre-determined,
        // the path is indicated.
        if (Spine is not null)
        {
            if (Spine.IsTangencyExtremity(true))
            {
                TopoDS_Vertex V = Spine.FirstVertex();
                TopoDS_Edge E = Spine.Edges(1);
                double param = Spine.FirstParameter();
                Blend_Point BP = new();
                if (CompBlendPoint(V, E, param, F1, F2, ref BP))
                {
                    math_Vector vec = new(1, 4);
                    BP.ParametersOnS1(out vec[1], out vec[2]);
                    BP.ParametersOnS2(out vec[3], out vec[4]);
                    Func.Set(param);
                    if (Func.IsSolution(vec, tolapp3d))
                    {
                        TheWalk.AddSingularPoint(BP);
                    }
                }
            }
            if (Spine.IsTangencyExtremity(false))
            {
                TopoDS_Vertex V = Spine.LastVertex();
                TopoDS_Edge E = Spine.Edges(Spine.NbEdges());
                double param = Spine.LastParameter();
                Blend_Point BP = new();
                if (CompBlendPoint(V, E, param, F1, F2, ref BP))
                {
                    math_Vector vec = new(1, 4);
                    BP.ParametersOnS1(out vec[1], out vec[2]);
                    BP.ParametersOnS2(out vec[3], out vec[4]);
                    Func.Set(param);
                    if (Func.IsSolution(vec, tolapp3d))
                    {
                        TheWalk.AddSingularPoint(BP);
                    }
                }
            }
        }

        // The starting solution is reframed if necessary.
        //**********************************************//
        math_Vector ParSol = new(1, 4);
        double NewFirst = PFirst;
        if (RecOnS1 || RecOnS2)
        {
            if (!TheWalk.PerformFirstSection(Func, FInv, PFirst, Target, Soldep, tolapp3d, TolGuide, RecOnS1, RecOnS2,
                                                                              ref NewFirst, ParSol))
            {
                return false;
            }
        }
        else
        {
            ParSol.Initialized(Soldep); // a copy of the values, as math_Vector::operator=
        }

        // First the valid part is calculate, without caring for the extensions.
        //******************************************************************//
        int again = 0;
        bool tchernobyl = false;
        double u1sov = 0.0, u2sov = 0.0;
        TopoDS_Face bif = new();
        // Max step is relevant, but too great, the vector is required to detect
        // the twists.
        if ((Math.Abs(Last - First) <= MS * 5.0)
                && (Math.Abs(Last - First) >= 0.01 * Math.Abs(NewFirst - Target)))
        {
            MS = Math.Abs(Last - First) * 0.2;
        }

        while (again < 3)
        {
            // Path.
            if (again == 0 && (MS < 5 * TolGuide))
            {
                MS = 5 * TolGuide;
            }
            else
            {
                if (5 * TolGuide > MS)
                {
                    TolGuide = MS / 5;
                }
            }
            TheWalk.Perform(Func, FInv, NewFirst, Target, MS, tolapp3d, TolGuide, ParSol, Fleche, Appro);
            if (!TheWalk.IsDone())
            {
                return false;
            }
            Lin = TheWalk.Line();
            if (HGuide.IsPeriodic() && Inside)
            {
                SpFirst = Lin.Point(1).Parameter();
                SpLast = SpFirst + HGuide.Period();
                HGuide.FirstParameter(SpFirst);
                HGuide.LastParameter(SpLast);
                HGuide.SetOrigin(SpFirst);
                if (OffsetHGuide is not null)
                {
                    OffsetHGuide.FirstParameter(SpFirst);
                    OffsetHGuide.LastParameter(SpLast);
                    OffsetHGuide.SetOrigin(SpFirst);
                }
            }
            bool complmnt = true;
            if (Inside)
            {
                complmnt = TheWalk.Complete(Func, FInv, SpLast);
            }
            if (!complmnt)
            {
                return false;
            }

            // The result is controlled using two criterions :
            //- if there is enough points,
            //- if one has gone far enough.
            Nbpnt = Lin.NbPoints();
            if (Nbpnt == 0)
            {
                return false;
            }
            double fpointpar = Lin.Point(1).Parameter();
            double lpointpar = Lin.Point(Nbpnt).Parameter();

            double factor = 1.0 / (nbptmin + 1);
            bool okdeb = (Forward && !Inside);
            bool okfin = (!Forward && !Inside);
            if (!okdeb)
            {
                int narc1d = Lin.StartPointOnFirst().NbPointOnRst();
                int narc2d = Lin.StartPointOnSecond().NbPointOnRst();
                okdeb = (narc1d > 0 || narc2d > 0 || (fpointpar - First) < 10 * TolGuide);
            }
            if (!okfin)
            {
                int narc1d = Lin.EndPointOnFirst().NbPointOnRst();
                int narc2d = Lin.EndPointOnSecond().NbPointOnRst();
                okfin = (narc1d > 0 || narc2d > 0 || (Last - lpointpar) < 10 * TolGuide);
            }
            if (!okdeb || !okfin || Nbpnt == 1)
            {
                // It drags, the controls are extended, it is  expected to evaluate a
                // satisfactory maximum step. If it already done, quit.
                if (tchernobyl)
                {
                    return false;
                }
                tchernobyl = true;
                TheWalk.Check(false);
                if (Nbpnt == 1)
                {
                    MS *= 0.01;
                }
                else
                {
                    MS = (lpointpar - fpointpar) / Nbpnt; // EvalStep(Lin);
                }
            }
            else if (Nbpnt < nbptmin)
            {
                if (again == 0)
                {
                    u1sov = fpointpar;
                    u2sov = lpointpar;
                    MS = (lpointpar - fpointpar) * factor;
                }
                else if (again == 1)
                {
                    if (Math.Abs(fpointpar - u1sov) >= TolGuide || Math.Abs(lpointpar - u2sov) >= TolGuide)
                    {
                        MS = (lpointpar - fpointpar) * factor;
                    }
                    else
                    {
                        return false;
                    }
                }
                again++;
            }
            else
            {
                again = 3;
            }
        }

        if (TheWalk.TwistOnS1())
        {
            Data.TwistOnS1(true);
        }
        if (TheWalk.TwistOnS2())
        {
            Data.TwistOnS2(true);
        }

        // Here there is a more or less presentable result
        // however it covers a the minimum zone.
        // The extensions are targeted.
        //*****************************//

        Gd1 = Gd2 = Gf1 = Gf2 = false;

        bool unseulsuffitdeb = (intf >= 2);
        bool unseulsuffitfin = (intl >= 2);
        bool noproldeb = (intf >= 3);
        bool noprolfin = (intl >= 3);

        double Rab = 0.03 * (SpLast - SpFirst);

        bool debarc1 = false, debarc2 = false;
        bool debcas1 = false, debcas2 = false;
        bool debobst1 = false, debobst2 = false;

        bool finarc1 = false, finarc2 = false;
        bool fincas1 = false, fincas2 = false;
        bool finobst1 = false, finobst2 = false;

        int narc1 = 0, narc2 = 0;

        bool backwContinueFailed = false;// eap
        if (reverse && intf != 0)
        {
            narc1 = Lin.StartPointOnFirst().NbPointOnRst();
            narc2 = Lin.StartPointOnSecond().NbPointOnRst();
            if (narc1 != 0)
            {
                ChFi3d_FilCommonPoint(Lin.StartPointOnFirst(), Lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS1(),
                                                            tolapp3d);
                debarc1 = true;
                if (!SearchFace(Spine, Data.VertexFirstOnS1(), F1, ref bif))
                {
                    // It is checked if there is not an obstacle.
                    debcas1 = true;
                    if (Spine is not null)
                    {
                        if (Spine.IsPeriodic())
                        {
                            debobst1 = true;
                        }
                        else
                        {
                            debobst1 = IsObst(Data.VertexFirstOnS1(), Spine.FirstVertex(), myVEMap);
                        }
                    }
                }
            }
            if (narc2 != 0)
            {
                ChFi3d_FilCommonPoint(Lin.StartPointOnSecond(), Lin.TransitionOnS2(), true, ref Data.ChangeVertexFirstOnS2(),
                                                            tolapp3d);
                debarc2 = true;
                if (!SearchFace(Spine, Data.VertexFirstOnS2(), F2, ref bif))
                {
                    // It is checked if it is not an obstacle.
                    debcas2 = true;
                    if (Spine is not null)
                    {
                        if (Spine.IsPeriodic())
                        {
                            debobst2 = true;
                        }
                        else
                        {
                            debobst2 = IsObst(Data.VertexFirstOnS2(), Spine.FirstVertex(), myVEMap);
                        }
                    }
                }
            }
            bool oncontinue = !noproldeb && (narc1 != 0 || narc2 != 0);
            if (debobst1 || debobst2)
            {
                oncontinue = false;
            }
            else if (debcas1 && debcas2)
            {
                oncontinue = false;
            }
            else if ((!debcas1 && debarc1) || (!debcas2 && debarc2))
            {
                oncontinue = false;
            }

            if (oncontinue)
            {
                TheWalk.ClassificationOnS1(!debarc1);
                TheWalk.ClassificationOnS2(!debarc2);
                TheWalk.Check2d(true); // It should be strict (PMN)
                TheWalk.Continu(Func, FInv, Target);
                TheWalk.ClassificationOnS1(true);
                TheWalk.ClassificationOnS2(true);
                TheWalk.Check2d(false);
                narc1 = Lin.StartPointOnFirst().NbPointOnRst();
                narc2 = Lin.StartPointOnSecond().NbPointOnRst();
                //  modified by eap Fri Feb  8 11:43:48 2002 ___BEGIN___
                if (!debarc1)
                {
                    if (narc1 == 0)
                    {
                        backwContinueFailed = Lin.StartPointOnFirst().ParameterOnGuide() > Target;
                    }
                    else
                    {
                        ChFi3d_FilCommonPoint(Lin.StartPointOnFirst(), Lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS1(),
                                                                    tolapp3d);
                        debarc1 = true;
                        if (!SearchFace(Spine, Data.VertexFirstOnS1(), F1, ref bif))
                        {
                            // It is checked if it is not an obstacle.
                            debcas1 = true;
                            // 	    if(!Spine.IsNull()) {
                            // 	      if(Spine->IsPeriodic()){
                            // 	        debobst1 = 1;
                            // 	      }
                            // 	      else{
                            // 		debobst1 = IsObst(Data->VertexFirstOnS1(),
                            // 				  Spine->FirstVertex(),myVEMap);
                            // 	      }
                            // 	    }
                        }
                    }
                }
                if (!debarc2)
                {
                    if (narc2 == 0)
                    {
                        backwContinueFailed = Lin.StartPointOnSecond().ParameterOnGuide() > Target;
                    }
                    else
                    {
                        ChFi3d_FilCommonPoint(Lin.StartPointOnSecond(), Lin.TransitionOnS2(), true,
                                                                    ref Data.ChangeVertexFirstOnS2(), tolapp3d);
                        debarc2 = true;
                        if (!SearchFace(Spine, Data.VertexFirstOnS2(), F2, ref bif))
                        {
                            // It is checked if it is not an obstacle.
                            debcas2 = true;
                            //             if(!Spine.IsNull()){
                            //               if(Spine->IsPeriodic()){
                            //                 debobst2 = 1;
                            //               }
                            //               else{
                            //                 debobst2 = IsObst(Data->VertexFirstOnS2(),
                            //                                   Spine->FirstVertex(),myVEMap);
                            //               }
                            //             }
                        }
                    }
                }
                if (backwContinueFailed)
                {
                    // if we leave backwContinueFailed as is, we will stop in this direction
                    // but we are to continue if there are no more faces on the side with arc
                    // check this condition
                    ChFiDS_CommonPoint aCP = debarc1 ? Data.VertexFirstOnS1() : Data.VertexFirstOnS2();
                    if (aCP.IsOnArc() && bif.IsNull())
                    {
                        backwContinueFailed = false;
                    }
                }
            }
        }
        bool forwContinueFailed = false;
        //  modified by eap Fri Feb  8 11:44:11 2002 ___END___
        if (Forward && intl != 0)
        {
            Target = SpLast;
            narc1 = Lin.EndPointOnFirst().NbPointOnRst();
            narc2 = Lin.EndPointOnSecond().NbPointOnRst();
            if (narc1 != 0)
            {
                ChFi3d_FilCommonPoint(Lin.EndPointOnFirst(), Lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS1(),
                                                            tolapp3d);
                finarc1 = true;
                if (!SearchFace(Spine, Data.VertexLastOnS1(), F1, ref bif))
                {
                    // It is checked if it is not an obstacle.
                    fincas1 = true;
                    if (Spine is not null)
                    {
                        finobst1 = IsObst(Data.VertexLastOnS1(), Spine.LastVertex(), myVEMap);
                    }
                }
            }
            if (narc2 != 0)
            {
                ChFi3d_FilCommonPoint(Lin.EndPointOnSecond(), Lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS2(),
                                                            tolapp3d);
                finarc2 = true;
                if (!SearchFace(Spine, Data.VertexLastOnS2(), F2, ref bif))
                {
                    // It is checked if it is not an obstacle.
                    fincas2 = true;
                    if (Spine is not null)
                    {
                        finobst2 = IsObst(Data.VertexLastOnS2(), Spine.LastVertex(), myVEMap);
                    }
                }
            }
            bool oncontinue = !noprolfin && (narc1 != 0 || narc2 != 0);
            if (finobst1 || finobst2)
            {
                oncontinue = false;
            }
            else if (fincas1 && fincas2)
            {
                oncontinue = false;
            }
            else if ((!fincas1 && finarc1) || (!fincas2 && finarc2))
            {
                oncontinue = false;
            }

            if (oncontinue)
            {
                TheWalk.ClassificationOnS1(!finarc1);
                TheWalk.ClassificationOnS2(!finarc2);
                TheWalk.Check2d(true); // It should be strict (PMN)
                TheWalk.Continu(Func, FInv, Target);
                TheWalk.ClassificationOnS1(true);
                TheWalk.ClassificationOnS2(true);
                TheWalk.Check2d(false);
                narc1 = Lin.EndPointOnFirst().NbPointOnRst();
                narc2 = Lin.EndPointOnSecond().NbPointOnRst();
                //  modified by eap Fri Feb  8 11:44:57 2002 ___BEGIN___
                if (!finarc1)
                {
                    if (narc1 == 0)
                    {
                        forwContinueFailed = Lin.EndPointOnFirst().ParameterOnGuide() < Target;
                    }
                    else
                    {
                        ChFi3d_FilCommonPoint(Lin.EndPointOnFirst(), Lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS1(),
                                                                    tolapp3d);
                        finarc1 = true;
                        if (!SearchFace(Spine, Data.VertexLastOnS1(), F1, ref bif))
                        {
                            // It is checked if it is not an obstacle.
                            fincas1 = true;
                            // 	    if(!Spine.IsNull()){
                            // 	      finobst1 = IsObst(Data->VertexLastOnS1(),
                            // 				Spine->LastVertex(),myVEMap);
                            // 	    }
                        }
                    }
                }
                if (!finarc2)
                {
                    if (narc2 == 0)
                    {
                        forwContinueFailed = Lin.EndPointOnSecond().ParameterOnGuide() < Target;
                    }
                    else
                    {
                        ChFi3d_FilCommonPoint(Lin.EndPointOnSecond(), Lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS2(),
                                                                    tolapp3d);
                        finarc2 = true;
                        if (!SearchFace(Spine, Data.VertexLastOnS2(), F2, ref bif))
                        {
                            // On regarde si ce n'est pas un obstacle.
                            fincas2 = true;
                            // 	    if(!Spine.IsNull()){
                            // 	      finobst2 = IsObst(Data->VertexLastOnS2(),
                            // 				Spine->LastVertex(),myVEMap);
                            // 	    }
                        }
                    }
                }
                if (forwContinueFailed)
                {
                    // if we leave forwContinueFailed as is, we will stop in this direction
                    // but we are to continue if there are no more faces on the side with arc
                    // check this condition
                    ChFiDS_CommonPoint aCP = finarc1 ? Data.VertexLastOnS1() : Data.VertexLastOnS2();
                    if (aCP.IsOnArc() && bif.IsNull())
                    {
                        forwContinueFailed = false;
                    }
                }
                //  modified by eap Fri Feb  8 11:45:10 2002 ___END___
            }
        }
        Nbpnt = Lin.NbPoints();
        First = Lin.Point(1).Parameter();
        Last = Lin.Point(Nbpnt).Parameter();

        // ============= INVALIDATION EVENTUELLE =============
        // ------ Preparation des prolongement par plan tangent -----
        if (reverse && intf != 0)
        {
            Gd1 = debcas1 /* && !debobst1*/; // skv(occ67)
            Gd2 = debcas2 /* && !debobst2*/; // skv(occ67)
            if ((debarc1 ^ debarc2) && !unseulsuffitdeb && (First != SpFirst))
            {
                // Case of incomplete path, of course this ends badly :
                // the result is truncated instead of exit.
                double sortie = 0.0;
                int ind = 0;
                if (debarc1)
                {
                    sortie = Data.VertexFirstOnS1().Parameter();
                }
                else
                {
                    sortie = Data.VertexFirstOnS2().Parameter();
                }
                if (sortie - First > tolesp)
                {
                    ind = SearchIndex(sortie, Lin);
                    if (Lin.Point(ind).Parameter() == sortie)
                    {
                        ind--;
                    }
                    if (ind >= 1)
                    {
                        Lin.Remove(1, ind);
                        UpdateLine(Lin, true);
                    }
                    Nbpnt = Lin.NbPoints();
                    First = Lin.Point(1).Parameter();
                }
            }
            else if ((intf >= 5) && !debarc1 && !debarc2 && (First != SpFirst))
            {
                double sortie = (2 * First + Last) / 3;
                int ind = 0;
                if (sortie - First > tolesp)
                {
                    ind = SearchIndex(sortie, Lin);
                    if (Lin.Point(ind).Parameter() == sortie)
                    {
                        ind--;
                    }
                    if (Nbpnt - ind < 3)
                    {
                        ind = Nbpnt - 3;
                    }
                    if (ind >= 1)
                    {
                        Lin.Remove(1, ind);
                        UpdateLine(Lin, true);
                    }
                    Nbpnt = Lin.NbPoints();
                    First = Lin.Point(1).Parameter();
                }
            }
            if (Gd1 && Gd2)
            {
                Target = Min((Lin.Point(1).Parameter() - Rab), First);
                Target = Max(Target, SpFirst);
                Data.FirstExtensionValue(Math.Abs(Lin.Point(1).Parameter() - Target));
            }
            if (intf != 0 && !unseulsuffitdeb)
            {
                intf = ((Gd1 && Gd2) //;
                        || backwContinueFailed) ? 1 : 0; // eap
            }
            else if (intf != 0 && unseulsuffitdeb && (intf < 5))
            {
                intf = (Gd1 || Gd2) ? 1 : 0;
                // It is checked if there is no new face.
                if (intf != 0 && ((!debcas1 && debarc1) || (!debcas2 && debarc2)))
                {
                    intf = 0;
                }
            }
            else if (intf < 5)
            {
                intf = 0;
            }
        }

        if (Forward && intl != 0)
        {
            Gf1 = fincas1 /* && !finobst1*/; // skv(occ67)
            Gf2 = fincas2 /* && !finobst2*/; // skv(occ67)
            if ((finarc1 ^ finarc2) && !unseulsuffitfin && (Last != SpLast))
            {
                // Case of incomplete path, of course, this ends badly :
                // the result is truncated instead of exit.
                double sortie = 0.0;
                int ind = 0;
                if (finarc1)
                {
                    sortie = Data.VertexLastOnS1().Parameter();
                }
                else
                {
                    sortie = Data.VertexLastOnS2().Parameter();
                }
                if (Last - sortie > tolesp)
                {
                    ind = SearchIndex(sortie, Lin);
                    if (Lin.Point(ind).Parameter() == sortie)
                    {
                        ind++;
                    }
                    if (ind <= Nbpnt)
                    {
                        Lin.Remove(ind, Nbpnt);
                        UpdateLine(Lin, false);
                    }
                    Nbpnt = Lin.NbPoints();
                    Last = Lin.Point(Nbpnt).Parameter();
                }
            }
            else if ((intl >= 5) && !finarc1 && !finarc2 && (Last != SpLast))
            {
                // The same in case when the entire "Lin" is an extension
                double sortie = (First + 2 * Last) / 3;
                int ind = 0;
                if (Last - sortie > tolesp)
                {
                    ind = SearchIndex(sortie, Lin);
                    if (Lin.Point(ind).Parameter() == sortie)
                    {
                        ind++;
                    }
                    if (ind < 3)
                    {
                        ind = 3;
                    }
                    if (ind <= Nbpnt)
                    {
                        Lin.Remove(ind, Nbpnt);
                        UpdateLine(Lin, false);
                    }
                    Nbpnt = Lin.NbPoints();
                    Last = Lin.Point(Nbpnt).Parameter();
                }
            }
            if (Gf1 && Gf2)
            {
                Target = Max((Lin.Point(Nbpnt).Parameter() + Rab), Last);
                Target = Min(Target, SpLast);
                Data.LastExtensionValue(Math.Abs(Target - Lin.Point(Nbpnt).Parameter()));
            }

            if (intl != 0 && !unseulsuffitfin)
            {
                intl = ((Gf1 && Gf2) //;
                        || forwContinueFailed) ? 1 : 0; // eap
            }
            else if (intl != 0 && unseulsuffitfin && (intl < 5))
            {
                intl = (Gf1 || Gf2) ? 1 : 0; // It is checked if there is no new face.
                if (intl != 0 && ((!fincas1 && finarc1) || (!fincas2 && finarc2)))
                {
                    intl = 0;
                }
            }
            else if (intl < 5)
            {
                intl = 0;
            }
        }
        return true;
    }

    //=================================================================================================

    internal bool SimulData(ChFiDS_SurfData? _unused3, ChFiDS_ElSpine? HGuide, ChFiDS_ElSpine? AdditionalHGuide, ref BRepBlend_Line? Lin, Adaptor3d_Surface? S1, Adaptor3d_TopolTool? I1, Adaptor3d_Surface? S2, Adaptor3d_TopolTool? I2, Blend_Function Func, Blend_FuncInv FInv, double PFirst, double MaxStep, double Fleche, double tolguide, ref double First, ref double Last, bool Inside, bool Appro, bool Forward, math_Vector Soldep, int NbSecMin, bool RecOnS1 = false, bool RecOnS2 = false)
    {
        BRepBlend_Walking TheWalk = new(S1, S2, I1, I2, HGuide);
        TheWalk.Check2d(false);

        double MS = MaxStep;
        double TolGuide = tolguide;
        int Nbpnt = 0;
        double SpFirst = HGuide.FirstParameter();
        double SpLast = HGuide.LastParameter();
        bool reverse = (!Forward || Inside);
        double Target = 0.0;
        if (reverse)
        {
            Target = SpFirst;
        }
        else
        {
            Target = SpLast;
        }

        double Targetsov = Target;
        double u1sov = 0.0, u2sov = 0.0;
        // on recadre la solution de depart a la demande.
        math_Vector ParSol = new(1, 4);
        double NewFirst = PFirst;
        if (RecOnS1 || RecOnS2)
        {
            if (!TheWalk.PerformFirstSection(Func, FInv, PFirst, Target, Soldep, tolapp3d, TolGuide, RecOnS1, RecOnS2,
                                                                              ref NewFirst, ParSol))
            {
                return false;
            }
        }
        else
        {
            ParSol.Initialized(Soldep); // a copy of the values, as math_Vector::operator=
        }
        int again = 0;
        while (again < 3)
        {
            // When the start point is inside, the path goes first to the left
            // to determine the Last for the periodicals.
            if (again == 0 && (MS < 5 * TolGuide))
            {
                MS = 5 * TolGuide;
            }
            else
            {
                if (5 * TolGuide > MS)
                {
                    TolGuide = MS / 5;
                }
            }

            TheWalk.Perform(Func, FInv, NewFirst, Target, MS, tolapp3d, TolGuide, ParSol, Fleche, Appro);

            if (!TheWalk.IsDone())
            {
                return false;
            }
            Lin = TheWalk.Line();
            if (reverse)
            {
                if (HGuide.IsPeriodic())
                {
                    SpFirst = Lin.Point(1).Parameter();
                    SpLast = SpFirst + HGuide.Period();
                    HGuide.FirstParameter(SpFirst);
                    HGuide.LastParameter(SpLast);
                    if (AdditionalHGuide is not null)
                    {
                        AdditionalHGuide.FirstParameter(SpFirst);
                        AdditionalHGuide.LastParameter(SpLast);
                    }
                }
                bool complmnt = true;
                if (Inside)
                {
                    complmnt = TheWalk.Complete(Func, FInv, SpLast);
                }
                if (!complmnt)
                {
                    return false;
                }
            }
            Nbpnt = Lin.NbPoints();
            double factor = 1.0 / (NbSecMin + 1);
            if (Nbpnt == 0)
            {
                return false;
            }
            else if (Nbpnt == 1 && again == 0)
            {
                again++;
                MS *= 0.01;
                Target = Targetsov;
                u1sov = u2sov = Lin.Point(1).Parameter();
            }
            else if (Nbpnt < NbSecMin && again == 0)
            {
                again++;
                double u1 = u1sov = Lin.Point(1).Parameter();
                double u2 = u2sov = Lin.Point(Nbpnt).Parameter();
                MS = (u2 - u1) * factor;
                Target = Targetsov;
            }
            else if (Nbpnt < NbSecMin && again == 1)
            {
                double u1 = Lin.Point(1).Parameter();
                double u2 = Lin.Point(Nbpnt).Parameter();
                if (Math.Abs(u1 - u1sov) >= TolGuide || Math.Abs(u2 - u2sov) >= TolGuide)
                {
                    again++;
                    MS /= 100;
                    Target = Targetsov;
                }
                else
                {
                    return false;
                }
            }
            else if (Nbpnt < NbSecMin)
            {
                return false;
            }
            else
            {
                again = 3;
            }
        }
        First = Lin.Point(1).Parameter();
        Last = Lin.Point(Nbpnt).Parameter();
        return true;
    }

}

internal static class ChFi3d_Builder_6
{
    internal static int SearchIndex(double Value, BRepBlend_Line? Lin)
    {
        int NbPnt = Lin.NbPoints(), Ind;

        for (Ind = 1; (Ind < NbPnt) && (Lin.Point(Ind).Parameter() < Value);)
        {
            Ind++;
        }
        return Ind;
    }

    internal static int nbedconnex(NCollection_List<TopoDS_Shape> L)
    {
        int nb = 0, i = 0;
        NCollection_List<TopoDS_Shape>.Iterator It1 = new(L);
        for (; It1.More(); It1.Next(), i++)
        {
            TopoDS_Shape curs = It1.Value();
            bool dejavu = false;
            NCollection_List<TopoDS_Shape>.Iterator It2 = new(L);
            for (int j = 0; j < i && It2.More(); j++, It2.Next())
            {
                if (curs.IsSame(It2.Value()))
                {
                    dejavu = true;
                    break;
                }
            }
            if (!dejavu)
            {
                nb++;
            }
        }
        return nb;
    }

    internal static bool IsVois(in TopoDS_Edge E, in TopoDS_Vertex Vref, ChFiDS_Map VEMap, NCollection_Map<TopoDS_Shape> DONE, int prof, int profmax)
    {
        if (prof > profmax)
        {
            return false;
        }
        if (DONE.Contains(E))
        {
            return false;
        }
        TopoDS_Vertex V1 = new(), V2 = new();
        TopExp.Vertices(E, out V1, out V2);
        if (Vref.IsSame(V1) || Vref.IsSame(V2))
        {
            return true;
        }
        DONE.Add(E);
        NCollection_List<TopoDS_Shape> L1 = VEMap[V1];
        int i1 = nbedconnex(L1);
        NCollection_List<TopoDS_Shape>.Iterator It1 = new(L1);
        for (; It1.More(); It1.Next())
        {
            TopoDS_Edge curE = TopoDS.Edge(It1.Value());
            if (i1 <= 2)
            {
                if (IsVois(curE, Vref, VEMap, DONE, prof, profmax))
                {
                    return true;
                }
            }
            else if (IsVois(curE, Vref, VEMap, DONE, prof + 1, profmax))
            {
                return true;
            }
        }
        NCollection_List<TopoDS_Shape> L2 = VEMap[V2];
        NCollection_List<TopoDS_Shape>.Iterator It2 = new(L2);
        for (; It2.More(); It2.Next())
        {
            TopoDS_Edge curE = TopoDS.Edge(It2.Value());
            if (i1 <= 2)
            {
                if (IsVois(curE, Vref, VEMap, DONE, prof, profmax))
                {
                    return true;
                }
            }
            else if (IsVois(curE, Vref, VEMap, DONE, prof + 1, profmax))
            {
                return true;
            }
        }
        return false;
    }

    internal static bool IsObst(in ChFiDS_CommonPoint CP, in TopoDS_Vertex Vref, ChFiDS_Map VEMap)
    {
        if (!CP.IsOnArc())
        {
            return false;
        }
        TopoDS_Edge E = CP.Arc();
        NCollection_Map<TopoDS_Shape> DONE = new(TopTools_ShapeMapHasher.Instance);
        int prof = 4;
        return !IsVois(E, Vref, VEMap, DONE, 0, prof);
    }

    internal static void CompParam(Geom2dAdaptor_Curve Carc, Geom2d_Curve? Ctg, ref double parc, ref double ptg, double prefarc, double preftg)
    {
        bool found = false;
        //(1) It is checked if the provided parameters are good
        //    if pcurves have the same parameters as the spine.
        gp_Pnt2d point = Carc.Value(prefarc);
        double distini = point.Distance(Ctg.Value(preftg));
        if (distini <= Precision.PConfusion())
        {
            parc = prefarc;
            ptg = preftg;
            found = true;
        }
        else
        {
            //(2) Intersection
            IntRes2d_IntersectionPoint int2d = new();
            Geom2dInt_GInter Intersection = new();
            int nbpt = 0, nbseg = 0;
            Intersection.Perform(new Geom2dAdaptor_Curve(Ctg), Carc, Precision.PIntersection(), Precision.PIntersection());

            double dist = Precision.Infinite(), p1, p2;
            if (Intersection.IsDone())
            {
                if (!Intersection.IsEmpty())
                {
                    nbseg = Intersection.NbSegments();
                    if (nbseg > 0)
                    {
                    }
                    nbpt = Intersection.NbPoints();
                    for (int i = 1; i <= nbpt; i++)
                    {
                        int2d = Intersection.Point(i);
                        p1 = int2d.ParamOnFirst();
                        p2 = int2d.ParamOnSecond();
                        if (Math.Abs(prefarc - p2) < dist)
                        {
                            ptg = p1;
                            parc = p2;
                            dist = Math.Abs(prefarc - p2);
                            found = true;
                        }
                    }
                }
            }
        }

        if (!found)
        {
            // (3) Projection...
            parc = prefarc;
            Geom2dAPI_ProjectPointOnCurve projector = new(point, Ctg);

            if (projector.NbPoints() == 0)
            {
                // This happens in some cases when there is a vertex
                // at the end of spine...
                ptg = preftg;
            }
            else
            {
                // It is checked if everything was calculated correctly (EDC402 C2)
                if (projector.LowerDistance() < distini)
                {
                    ptg = projector.LowerDistanceParameter();
                }
                else
                {
                    ptg = preftg;
                }
            }
        }
    }

    internal static bool CompBlendPoint(in TopoDS_Vertex V, in TopoDS_Edge E, double W, in TopoDS_Face F1, in TopoDS_Face F2, ref Blend_Point BP)
    {
        gp_Pnt2d P1 = new(), P2 = new();
        gp_Pnt P3d = new();
        double param = 0.0, f = 0.0, l = 0.0;
        Geom2d_Curve? pc = null;

        P3d = BRep_Tool.Pnt(V);
        param = BRep_Tool.Parameter(V, E, F1);
        pc = BRep_Tool.CurveOnSurface(E, F1, out f, out l);
        if (pc is null)
        {
            return false;
        }
        P1 = pc.Value(param);
        param = BRep_Tool.Parameter(V, E, F2);
        pc = BRep_Tool.CurveOnSurface(E, F2, out f, out l);
        if (pc is null)
        {
            return false;
        }
        P2 = pc.Value(param);
        BP.SetValue(P3d, P3d, W, P1.X(), P1.Y(), P2.X(), P2.Y());
        return true;
    }

    internal static void UpdateLine(BRepBlend_Line? Line, bool isfirst)
    {
        double tguide = 0.0, U = 0.0, V = 0.0;
        if (isfirst)
        {
            Blend_Point BP = Line.Point(1);
            tguide = BP.Parameter();
            if (Line.StartPointOnFirst().ParameterOnGuide() < tguide)
            {
                BRepBlend_Extremity BE = new();
                BP.ParametersOnS1(out U, out V);
                BE.SetValue(BP.PointOnS1(), U, V, Precision.Confusion());
                Line.SetStartPoints(BE, Line.StartPointOnSecond());
            }
            if (Line.StartPointOnSecond().ParameterOnGuide() < tguide)
            {
                BRepBlend_Extremity BE = new();
                BP.ParametersOnS2(out U, out V);
                BE.SetValue(BP.PointOnS2(), U, V, Precision.Confusion());
                Line.SetStartPoints(Line.StartPointOnFirst(), BE);
            }
        }
        else
        {
            Blend_Point BP = Line.Point(Line.NbPoints());
            tguide = BP.Parameter();
            if (Line.EndPointOnFirst().ParameterOnGuide() > tguide)
            {
                BRepBlend_Extremity BE = new();
                BP.ParametersOnS1(out U, out V);
                BE.SetValue(BP.PointOnS1(), U, V, Precision.Confusion());
                Line.SetEndPoints(BE, Line.EndPointOnSecond());
            }
            if (Line.EndPointOnSecond().ParameterOnGuide() > tguide)
            {
                BRepBlend_Extremity BE = new();
                BP.ParametersOnS2(out U, out V);
                BE.SetValue(BP.PointOnS2(), U, V, Precision.Confusion());
                Line.SetEndPoints(Line.EndPointOnFirst(), BE);
            }
        }
    }


}
