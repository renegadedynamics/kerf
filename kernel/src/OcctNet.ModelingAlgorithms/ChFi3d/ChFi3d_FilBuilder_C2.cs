// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder_C2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_FilBuilder_C2;
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
public partial class ChFi3d_FilBuilder
{



    //=======================================================================
    // function : ToricRotule
    // purpose  : Test if it is a particular case of torus routine.
    //           Three planes with two constant incident fillets
    //           of the same radius and the third face perpendicular to
    //           two others are required.
    //=======================================================================

    //=================================================================================================

    internal override void PerformTwoCorner(int Index)
    {

        done = false;
        TopoDS_Vertex Vtx = myVDataMap.FindKey(Index);
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        It.Initialize(myVDataMap[Index]);
        ChFiDS_Stripe? st1 = null, st2 = null;
        int Sens1 = 0, Sens2 = 0;
        int Isd1 = 0, Isd2 = 0, i1 = 0, i2 = 0;
        ChFiDS_SurfData? sd1 = null, sd2 = null;
        NCollection_Sequence<ChFiDS_SurfData> SeqFil1 = new(), SeqFil2 = new();
        Geom_Surface? surf1 = null, surf2 = null;
        bool OkinterCC = false, Okvisavis = false, SameSide = false;
        int IFaCo1 = 0, IFaCo2 = 0;
        double UIntPC1 = 0.0, UIntPC2 = 0.0;
        TopoDS_Face FaCo = new();
        TopoDS_Edge E1 = new(), E2 = new(), E = new();
        TopoDS_Vertex V1 = new(), V2 = new();
        //  gp_Pnt P1,P2;
        int nbsurf1 = 0, nbsurf2 = 0, deb1 = 0, fin1 = 0, deb2 = 0, fin2 = 0;
        double parE1 = 0.0, parE2 = 0.0;
        // Necessary information on fillets is extracted
        //------------------------------------------------------

        // the first
        //----------

        st1 = It.Value();
        Isd1 = ChFi3d_IndexOfSurfData(Vtx, st1, out Sens1);

        // the second
        //----------
        It.Next();
        st2 = It.Value();
        if (st2 == st1)
        {
            Sens2 = -1;
            Isd2 = st2.SetOfSurfData().Length();
        }
        else
        {
            Isd2 = ChFi3d_IndexOfSurfData(Vtx, st2, out Sens2);
        }

        // If two edges to rounded are tangent GeomPlate is called

        if (Sens1 == 1)
        {
            E1 = st1.Spine().Edges(1);
        }
        else
        {
            E1 = st1.Spine().Edges(st1.Spine().NbEdges());
        }

        if (Sens2 == 1)
        {
            E2 = st2.Spine().Edges(1);
        }
        else
        {
            E2 = st2.Spine().Edges(st2.Spine().NbEdges());
        }

        BRepAdaptor_Curve BCurv1 = new(E1);
        BRepAdaptor_Curve BCurv2 = new(E2);
        parE1 = BRep_Tool.Parameter(Vtx, E1);
        parE2 = BRep_Tool.Parameter(Vtx, E2);
        BRepLProp_CLProps CL1 = new(BCurv1, parE1, 1, 1.0e-4);
        BRepLProp_CLProps CL2 = new(BCurv2, parE2, 1, 1.0e-4);
        gp_Dir dir1 = new(), dir2 = new();
        CL1.Tangent(out dir1);
        CL2.Tangent(out dir2);
        if (Sens1 == -1)
        {
            dir1.Reverse();
        }
        if (Sens2 == -1)
        {
            dir2.Reverse();
        }
        double ang1 = 0.0;
        ang1 = Math.Abs(dir1.Angle(dir2));
        if (ang1 < M_PI / 180.0)
        {
            PerformMoreThreeCorner(Index, 2);
            done = true;
            return;
        }

        OkinterCC = ChFi3d_IsInFront(DStr, st1, st2, Isd1, Isd2, Sens1, Sens2, ref UIntPC1, ref UIntPC2, ref FaCo,
                                                                  ref SameSide, ref IFaCo1, ref IFaCo2, ref Okvisavis, Vtx, true);

        bool trouve = false;
        if (!Okvisavis)
        {

            // one is not limited to the first or the last surfdata
            // to find the opposing data
            nbsurf1 = st1.SetOfSurfData().Length();
            nbsurf2 = st2.SetOfSurfData().Length();
            deb1 = 1;
            deb2 = 1;
            fin1 = 1;
            fin2 = 1;
            if (nbsurf1 != 1)
            {
                if (Sens1 == 1)
                {
                    deb1 = 1;
                    fin1 = 2;
                }
                else
                {
                    deb1 = nbsurf1 - 1;
                    fin1 = nbsurf1;
                }
            }
            if (nbsurf2 != 1)
            {
                if (Sens2 == 1)
                {
                    deb2 = 1;
                    fin2 = 2;
                }
                else
                {
                    deb2 = nbsurf2 - 1;
                    fin2 = nbsurf2;
                }
            }

            for (i1 = deb1; i1 <= fin1 && !trouve; i1++)
            {
                Isd1 = i1;
                for (i2 = deb2; i2 <= fin2 && !trouve; i2++)
                {
                    Isd2 = i2;

                    OkinterCC = ChFi3d_IsInFront(DStr, st1, st2, Isd1, Isd2, Sens1, Sens2, ref UIntPC1, ref UIntPC2, ref FaCo,
                                                                              ref SameSide, ref IFaCo1, ref IFaCo2, ref Okvisavis, Vtx, true);
                    trouve = Okvisavis;
                }
            }
            if (!trouve)
            {
                PerformMoreThreeCorner(Index, 2);
                done = true;
                return;
            }
            else
            {
                if (Sens1 == 1 && Isd1 != 1)
                {
                    RemoveSD(st1, 1, 1);
                }
                if (Sens1 != 1 && Isd1 != nbsurf1)
                {
                    RemoveSD(st1, fin1, fin1);
                }
                if (Sens2 == 1 && Isd2 != 1)
                {
                    RemoveSD(st2, 1, 1);
                }
                if (Sens2 != 1 && Isd2 != nbsurf2)
                {
                    RemoveSD(st2, fin2, fin2);
                }
            }
            Isd1 = ChFi3d_IndexOfSurfData(Vtx, st1, out Sens1);
            Isd2 = ChFi3d_IndexOfSurfData(Vtx, st2, out Sens2);
        }
        // throw StdFail_NotDone("TwoCorner : no common face");
        int IFaArc1 = 3 - IFaCo1, IFaArc2 = 3 - IFaCo2;
        SeqFil1 = st1.ChangeSetOfSurfData(); // OCCT copies the sequence; nothing writes into the copy
        SeqFil2 = st2.ChangeSetOfSurfData();
        sd1 = SeqFil1.ChangeValue(Isd1);
        surf1 = DStr.Surface(sd1.Surf()).Surface();
        sd2 = SeqFil2.ChangeValue(Isd2);
        surf2 = DStr.Surface(sd2.Surf()).Surface();
        TopAbs_Orientation OFaCo = FaCo.Orientation();
        // The concavities are analyzed and the opposite face and the
        // eventual intersection of 2 pcurves on this face are found.

        ChFiDS_State Stat1 = default, Stat2 = default;
        bool isfirst1 = (Sens1 == 1);
        bool isfirst2 = (Sens2 == 1);
        Stat1 = st1.Spine().Status(isfirst1);
        Stat2 = st2.Spine().Status(isfirst2);
        bool c1biseau = (Stat1 == ChFiDS_AllSame);
        bool c1rotule = (Stat1 == ChFiDS_OnSame && Stat2 == ChFiDS_OnSame);

        // It is checked if the fillets have a commonpoint on a common arc.
        // This edge is the pivot of the bevel or the knee.

        ref ChFiDS_CommonPoint CP1 = ref sd1.ChangeVertex(isfirst1, IFaArc1);
        ref ChFiDS_CommonPoint CP2 = ref sd2.ChangeVertex(isfirst2, IFaArc2);

        bool resetcp1 = false;
        bool resetcp2 = false;

        TopoDS_Edge pivot = new();
        bool yapiv = false;
        if (CP1.IsOnArc())
        {
            pivot = CP1.Arc();
        }
        else
        {
            PerformMoreThreeCorner(Index, 2);
            done = true;
            return;
        }
        if (CP1.IsOnArc() && CP2.IsOnArc())
        {
            yapiv = (pivot.IsSame(CP2.Arc()));
        }
        BRepAdaptor_Curve? Hpivot = null;
        bool sameparam = false;
        double parCP1 = 0.0, parCP2 = 0.0;
        if (yapiv)
        {
            Hpivot = new BRepAdaptor_Curve(pivot);
            parCP1 = CP1.ParameterOnArc();
            parCP2 = CP2.ParameterOnArc();
            gp_Pnt tst1 = Hpivot.Value(parCP1);
            gp_Pnt tst2 = Hpivot.Value(parCP2);
            sameparam = tst1.Distance(tst2) <= tolapp3d;
        }
        BRepAdaptor_Surface? HFaCo = new BRepAdaptor_Surface();
        BRepAdaptor_Surface? HFaPiv = null;
        BRepAdaptor_Surface? HBRS1 = new BRepAdaptor_Surface();
        BRepAdaptor_Surface? HBRS2 = new BRepAdaptor_Surface();

        BRepAdaptor_Surface BRS1 = HBRS1;
        BRepAdaptor_Surface BRS2 = HBRS2;
        BRepAdaptor_Surface BRFaCo = HFaCo;
        BRFaCo.Initialize(FaCo);

        TopoDS_Face FF1 = new(), FF2 = new(), F = new(), FaPiv = new();
        TopAbs_Orientation pctrans = TopAbs_FORWARD;
        Geom2d_BSplineCurve? PCurveOnPiv = null;
        FF1 = TopoDS.Face(DStr.Shape(sd1.Index(IFaArc1)));
        FF2 = TopoDS.Face(DStr.Shape(sd2.Index(IFaArc2)));
        if (FF1.IsNull() || FF2.IsNull())
        {
            PerformMoreThreeCorner(Index, 2);
            done = true;
            return;
        }
        BRS1.Initialize(FF1);
        BRS2.Initialize(FF2);

        if (yapiv)
        {
            NCollection_List<TopoDS_Shape>.Iterator Kt = new();
            bool ok1 = false, ok2 = false;
            for (Kt.Initialize(myEFMap[pivot]); Kt.More(); Kt.Next())
            {
                F = TopoDS.Face(Kt.Value());
                if (!ok1 && FF1.IsSame(F))
                {
                    ok1 = true;
                }
                if (!ok2 && FF2.IsSame(F))
                {
                    ok2 = true;
                }
            }
            if (!ok1 || !ok2)
            {
                PerformMoreThreeCorner(Index, 2);
                done = true;
                return;
            }
        }


        // bevel
        //------
        ChFiDS_CommonPoint cp11 = new(), cp12 = new(), cp21 = new(), cp22 = new();
        ChFiDS_FaceInterference intf11 = new(), intf12 = new(), intf21 = new(), intf22 = new();

        if (c1biseau)
        {

            done = PerformTwoCornerbyInter(Index);


            if (!done)
            {
                PerformMoreThreeCorner(Index, 2);
                done = true;
                return;
            }
        }
        else if (c1rotule)
        { // save.
            cp11 = sd1.Vertex(isfirst1, 1);
            cp12 = sd1.Vertex(isfirst1, 2);
            cp21 = sd2.Vertex(isfirst2, 1);
            cp22 = sd2.Vertex(isfirst2, 2);
            intf11 = sd1.InterferenceOnS1();
            intf12 = sd1.InterferenceOnS2();
            intf21 = sd2.InterferenceOnS1();
            intf22 = sd2.InterferenceOnS2();

            done = PerformTwoCornerbyInter(Index);

            if (!done)
            {
                // restore
                sd1.ChangeVertex(isfirst1, 1) = cp11;
                sd1.ChangeVertex(isfirst1, 2) = cp12;
                sd2.ChangeVertex(isfirst2, 1) = cp21;
                sd2.ChangeVertex(isfirst2, 2) = cp22;
                sd1.ChangeInterferenceOnS1() = intf11;
                sd1.ChangeInterferenceOnS2() = intf12;
                sd2.ChangeInterferenceOnS1() = intf21;
                sd2.ChangeInterferenceOnS2() = intf22;
                done = false;
            }
        }

        if (!c1biseau && !done)
        {
            // new cornerdata is created
            //-------------------------------
            ChFiDS_Stripe? corner = new ChFiDS_Stripe();
            ref NCollection_HSequence<ChFiDS_SurfData>? cornerset = ref 
                corner.ChangeSetOfSurfData();
            cornerset = new NCollection_HSequence<ChFiDS_SurfData>();
            ChFiDS_SurfData? coin = new ChFiDS_SurfData();
            cornerset.Append(coin);

            if (SameSide)
            {
                if (ToricRotule(BRFaCo, BRS1, BRS2, st1, st2))
                {
                    // Direct construction.
                    // ---------------------

                    int bid = 0;
                    TopAbs_Orientation ori = OFaCo;
                    TopAbs_Orientation oriS = st1.Orientation(IFaCo1);
                    TopAbs_Orientation OFF1 = FF1.Orientation();
                    TopAbs_Orientation oriSFF1 = st1.Orientation(IFaArc1);
                    bid = 1;
                    bid = ChFi3d.NextSide(ref ori, ref OFF1, oriS, oriSFF1, bid);
                    TopAbs_Orientation op1 = TopAbs_FORWARD, op2 = TopAbs_FORWARD;
                    if (yapiv)
                    {
                        bid = ChFi3d.ConcaveSide(BRS1, BRS2, pivot, out op1, out op2);
                    }
                    op1 = TopAbs.Reverse(op1);
                    op2 = TopAbs.Reverse(op2);
                    double radius = (st1.Spine() as ChFiDS_FilSpine).Radius();
                    done = ChFiKPart_ComputeData.ComputeCorner(DStr, coin, HFaCo, HBRS1, HBRS2, OFaCo, ori, op1, op2,
                                                                                                            radius);
                }
                else
                {
                    // Construction by filling remplissage
                    // ----------------------------
                    double uPCArc1 = 0.0, uPCArc2 = 0.0;
                    gp_Pnt2d p2da1 = new(), p2df1 = new(), p2da2 = new(), p2df2 = new(), p2dfac1 = new(), p2dfac2 = new();
                    gp_Vec2d v2dfac1 = new(), v2dfac2 = new();
                    GeomFill_Boundary? B1 = null, B2 = null, Bpiv = null, Bfac = null;
                    uPCArc1 = sd1.Interference(IFaArc1).Parameter(isfirst1);
                    p2da1 = sd1.Interference(IFaArc1).PCurveOnSurf().Value(uPCArc1);
                    p2df1 = sd1.Interference(IFaCo1).PCurveOnSurf().Value(uPCArc1);
                    sd1.Interference(IFaCo1).PCurveOnFace().D1(uPCArc1, out p2dfac1, out v2dfac1);
                    uPCArc2 = sd2.Interference(IFaArc2).Parameter(isfirst2);
                    p2da2 = sd2.Interference(IFaArc2).PCurveOnSurf().Value(uPCArc2);
                    p2df2 = sd2.Interference(IFaCo2).PCurveOnSurf().Value(uPCArc2);
                    sd2.Interference(IFaCo2).PCurveOnFace().D1(uPCArc2, out p2dfac2, out v2dfac2);
                    B1 = ChFi3d_mkbound(surf1, p2df1, p2da1, tolapp3d, 2.0e-4);
                    B2 = ChFi3d_mkbound(surf2, p2df2, p2da2, tolapp3d, 2.0e-4);
                    Geom2d_Curve? PCurveOnFace = null;
                    Bfac = ChFi3d_mkbound(HFaCo, out PCurveOnFace, Sens1, p2dfac1, v2dfac1, Sens2, p2dfac2, v2dfac2, tolapp3d,
                                                                2.0e-4);
                    GeomFill_ConstrainedFilling fil = new(8, 20);
                    if (sameparam)
                    {
                        fil.Init(Bfac, B2, B1, true);
                    }
                    else
                    {
                        Adaptor3d_Curve? HPivTrim = 
                            Hpivot.Trim(Min(parCP1, parCP2), Max(parCP1, parCP2), tolesp);
                        Bpiv = new GeomFill_SimpleBound(HPivTrim, tolapp3d, 2.0e-4);
                        fil.Init(Bfac, B2, Bpiv, B1, true);
                        BRepAdaptor_Curve2d pcpivot = new();
                        gp_Vec dArc = new(), dcf = new();
                        gp_Pnt bidon = new();
                        Hpivot.D1(parCP1, out bidon, out dArc);
                        double fb1 = 0.0, lb1 = 0.0;
                        B1.Bounds(out fb1, out lb1);
                        B1.D1(lb1, out bidon, out dcf);
                        bool pivotverslebas = dArc.Dot(dcf) <= 0.0;
                        bool pcfalenvers = (parCP1 > parCP2);
                        if ((pivotverslebas && !pcfalenvers) || (!pivotverslebas && pcfalenvers))
                        {
                            FaPiv = FF2;
                            HFaPiv = HBRS2;
                            resetcp2 = true;
                        }
                        else
                        {
                            FaPiv = FF1;
                            HFaPiv = HBRS1;
                            resetcp1 = true;
                        }
                        FaPiv.Orientation(TopAbs_FORWARD);
                        pcpivot.Initialize(pivot, FaPiv);
                        TopExp_Explorer Expl = new();
                        for (Expl.Init(FaPiv, TopAbs_EDGE); Expl.More(); Expl.Next())
                        {
                            if (Expl.Current().IsSame(pivot))
                            {
                                pctrans = Expl.Current().Orientation();
                                break;
                            }
                        }
                        if (pcpivot.GetType() != GeomAbs_BSplineCurve)
                        {
                            Geom2d_TrimmedCurve? trc = 
                                new Geom2d_TrimmedCurve(pcpivot.Curve(), Min(parCP1, parCP2), Max(parCP1, parCP2));
                            PCurveOnPiv = Geom2dConvert.CurveToBSplineCurve(trc);
                        }
                        else
                        {
                            PCurveOnPiv =
                                Geom2dConvert.SplitBSplineCurve((pcpivot.Curve() as Geom2d_BSplineCurve), Min(parCP1, parCP2),
                                                                                                  Max(parCP1, parCP2), tol2d);
                        }
                        PCurveOnPiv = Writable(PCurveOnPiv);
                        NCollection_Array1<double> kk = new(PCurveOnPiv.InternalKnots());
                        BSplCLib.Reparametrize(0.0, 1.0, kk);
                        PCurveOnPiv.SetKnots(kk);
                        if (pcfalenvers)
                        {
                            PCurveOnPiv.Reverse();
                            pctrans = TopAbs.Reverse(pctrans);
                        }
                    }
                    Geom_Surface? Surfcoin = fil.InternalSurface();
                    done = CompleteData(coin, Surfcoin, HFaCo, PCurveOnFace, HFaPiv, PCurveOnPiv,
                                                            OFaCo, true, false, false, false, false);
                }
                if (done)
                {
                    // Update 3 CornerData and the DS
                    // ----------------------------------------
                    if (resetcp1)
                    {
                        gp_Pnt pjyl = CP1.Point();
                        double tolsav = CP1.Tolerance();
                        CP1.Reset();
                        CP1.SetPoint(pjyl);
                        CP1.SetTolerance(tolsav);
                    }
                    else if (resetcp2)
                    {
                        gp_Pnt pjyl = CP2.Point();
                        double tolsav = CP2.Tolerance();
                        CP2.Reset();
                        CP2.SetPoint(pjyl);
                        CP2.SetTolerance(tolsav);
                    }
                    double P1deb = 0.0, P2deb = 0.0, P1fin = 0.0, P2fin = 0.0;
                    int If1 = 0, If2 = 0, Il1 = 0, Il2 = 0, Icf = 0, Icl = 0;
                    ChFiDS_CommonPoint Pf1 = coin.VertexFirstOnS1();
                    ref ChFiDS_CommonPoint Pf2 = ref coin.ChangeVertexFirstOnS2();
                    ChFiDS_CommonPoint Pl1 = coin.VertexLastOnS1();
                    ref ChFiDS_CommonPoint Pl2 = ref coin.ChangeVertexLastOnS2();
                    Pf2 = CP1;
                    Pl2 = CP2;

                    // the corner to start,
                    // -----------------------
                    ChFiDS_Regul regdeb = new(), regfin = new();
                    If1 = ChFi3d_IndexPointInDS(Pf1, DStr);
                    If2 = ChFi3d_IndexPointInDS(Pf2, DStr);
                    Il1 = ChFi3d_IndexPointInDS(Pl1, DStr);
                    if (sameparam)
                    {
                        Il2 = If2;
                    }
                    else
                    {
                        Il2 = ChFi3d_IndexPointInDS(Pl2, DStr);
                    }

                    gp_Pnt2d pp1 = new(), pp2 = new();
                    pp1 =
                        coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().FirstParameter());
                    pp2 =
                        coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().FirstParameter());
                    Geom_Curve? C3d = null;
                    double tolreached = 0.0;
                    ChFi3d_ComputeArete(Pf1, pp1, Pf2, pp2, DStr.Surface(coin.Surf()).Surface(), ref C3d,
                                                            ref corner.ChangeFirstPCurve(), ref P1deb, ref P2deb, tolapp3d, tol2d, out tolreached, 0);
                    double par1 = sd1.Interference(IFaArc1).Parameter(isfirst1);
                    pp1 = sd1.Interference(IFaCo1).PCurveOnSurf().Value(par1);
                    pp2 = sd1.Interference(IFaArc1).PCurveOnSurf().Value(par1);
                    double tolr1 = 0.0;
                    ChFi3d_ComputePCurv(C3d, pp1, pp2, out st1.ChangePCurve(isfirst1), DStr.Surface(sd1.Surf()).Surface(), P1deb,
                                                            P2deb, tolapp3d, out tolr1);
                    tolreached = Max(tolreached, tolr1);
                    TopOpeBRepDS_Curve Tcurv1 = new(C3d, tolreached);
                    Icf = DStr.AddCurve(Tcurv1);
                    regdeb.SetCurve(Icf);
                    regdeb.SetS1(coin.Surf(), false);
                    regdeb.SetS2(sd1.Surf(), false);
                    myRegul.Append(regdeb);
                    corner.ChangeFirstCurve(Icf);
                    corner.ChangeFirstParameters(P1deb, P2deb);
                    corner.ChangeIndexFirstPointOnS1(If1);
                    corner.ChangeIndexFirstPointOnS2(If2);

                    pp1 =
                        coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().LastParameter());
                    pp2 =
                        coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().LastParameter());
                    ChFi3d_ComputeArete(Pl1, pp1, Pl2, pp2, DStr.Surface(coin.Surf()).Surface(), ref C3d,
                                                            ref corner.ChangeLastPCurve(), ref P1fin, ref P2fin, tolapp3d, tol2d, out tolreached, 0);
                    double par2 = sd2.Interference(IFaArc2).Parameter(isfirst2);
                    pp1 = sd2.Interference(IFaCo2).PCurveOnSurf().Value(par2);
                    pp2 = sd2.Interference(IFaArc2).PCurveOnSurf().Value(par2);
                    double tolr2 = 0.0;
                    ChFi3d_ComputePCurv(C3d, pp1, pp2, out st2.ChangePCurve(isfirst2), DStr.Surface(sd2.Surf()).Surface(), P1deb,
                                                            P2deb, tolapp3d, out tolr2);
                    tolreached = Max(tolreached, tolr2);
                    TopOpeBRepDS_Curve Tcurv2 = new(C3d, tolreached);
                    Icl = DStr.AddCurve(Tcurv2);
                    regfin.SetCurve(Icl);
                    regfin.SetS1(coin.Surf(), false);
                    regfin.SetS2(sd2.Surf(), false);
                    myRegul.Append(regfin);
                    corner.ChangeLastCurve(Icl);
                    corner.ChangeLastParameters(P1fin, P2fin);
                    corner.ChangeIndexLastPointOnS1(Il1);
                    corner.ChangeIndexLastPointOnS2(Il2);

                    coin.ChangeIndexOfS1(DStr.AddShape(FaCo));
                    if (sameparam)
                    {
                        coin.ChangeIndexOfS2(0);
                    }
                    else
                    {
                        coin.ChangeIndexOfS2(DStr.AddShape(FaPiv));
                        coin.ChangeInterferenceOnS2().SetTransition(pctrans);
                    }
                    corner.SetSolidIndex(st1.SolidIndex());

                    // then the starting Stripe,
                    // ------------------------
                    st1.SetCurve(Icf, isfirst1);
                    st1.SetIndexPoint(If1, isfirst1, IFaCo1);
                    st1.SetIndexPoint(If2, isfirst1, IFaArc1);
                    st1.SetParameters(isfirst1, P1deb, P2deb);
                    sd1.ChangeVertex(isfirst1, IFaCo1) = Pf1;
                    sd1.ChangeVertex(isfirst1, IFaArc1) = Pf2;
                    sd1.ChangeInterference(IFaCo1).SetParameter(par1, isfirst1);
                    if (IFaCo1 == 2)
                    {
                        st1.SetOrientation(TopAbs_REVERSED, isfirst1);
                    }

                    // then the end Stripe,
                    // -------------------------
                    st2.SetCurve(Icl, isfirst2);
                    st2.SetIndexPoint(Il1, isfirst2, IFaCo2);
                    st2.SetIndexPoint(Il2, isfirst2, IFaArc2);
                    st2.SetParameters(isfirst2, P1fin, P2fin);
                    sd2.ChangeVertex(isfirst2, IFaCo2) = Pl1;
                    sd2.ChangeVertex(isfirst2, IFaArc2) = Pl2;
                    sd2.ChangeInterference(IFaCo2).SetParameter(par2, isfirst2);
                    if (IFaCo2 == 2)
                    {
                        st2.SetOrientation(TopAbs_REVERSED, isfirst2);
                    }
                }
            }
            else
            {
                // it is necessary to make difference with
                if (!OkinterCC)
                {
                    throw new Standard_Failure("TwoCorner : No intersection pc pc");
                }
                ChFiDS_Stripe? stsam = null, stdif = null;
                ChFiDS_SurfData? sdsam = null, sddif = null;
                double uintpcsam = 0.0, uintpcdif = 0.0;
                int ifacosam = 0, ifacodif = 0, ifaopsam = 0, ifaopdif = 0;
                bool isfirstsam = false, isfirstdif = false;
                if (Stat1 == ChFiDS_OnSame && Stat2 == ChFiDS_OnDiff)
                {
                    stsam = st1;
                    sdsam = sd1;
                    uintpcsam = UIntPC1;
                    ifacosam = IFaCo1;
                    ifaopsam = IFaArc1;
                    isfirstsam = isfirst1;
                    stdif = st2;
                    sddif = sd2;
                    uintpcdif = UIntPC2;
                    ifacodif = IFaCo2;
                    ifaopdif = IFaArc2;
                    isfirstdif = isfirst2;
                }
                else if (Stat1 == ChFiDS_OnDiff && Stat2 == ChFiDS_OnSame)
                {
                    stsam = st2;
                    sdsam = sd2;
                    uintpcsam = UIntPC2;
                    ifacosam = IFaCo2;
                    ifaopsam = IFaArc2;
                    isfirstsam = isfirst2;
                    stdif = st1;
                    sddif = sd1;
                    uintpcdif = UIntPC1;
                    ifacodif = IFaCo1;
                    ifaopdif = IFaArc1;
                    isfirstdif = isfirst1;
                }
                else
                {
                    throw new Standard_Failure("TwoCorner : Config unknown");
                }
                // It is checked if surface ondiff has a point on arc from the side opposed
                // to the common face and if this arc is connected to the base face
                // opposed to common face of the surface onsame.
                ref ChFiDS_CommonPoint cpopdif = ref sddif.ChangeVertex(isfirstdif, ifaopdif);
                if (!cpopdif.IsOnArc())
                {
                    throw new Standard_Failure("TwoCorner : No point on restriction on surface OnDiff");
                }
                TopoDS_Edge Arcopdif = cpopdif.Arc();
                TopoDS_Face Fopsam = TopoDS.Face(DStr.Shape(sdsam.Index(ifaopsam)));
                TopExp_Explorer ex = new();
                for (ex.Init(Fopsam, TopAbs_EDGE); ex.More(); ex.Next())
                {
                    if (ex.Current().IsSame(Arcopdif))
                    {
                        break;
                    }
                    else if (!ex.More())
                    {
                        throw new Standard_Failure("TwoCorner : No common face to loop the contour");
                    }
                }
                GeomFill_Boundary? Bsam = null, Bdif = null, Bfac = null;
                gp_Pnt2d ppopsam = sdsam.Interference(ifaopsam).PCurveOnSurf().Value(uintpcsam);
                gp_Pnt2d ppcosam = sdsam.Interference(ifacosam).PCurveOnSurf().Value(uintpcsam);
                Geom_Surface? surfsam = DStr.Surface(sdsam.Surf()).Surface();
                GeomAdaptor_Surface? Hsurfsam = new GeomAdaptor_Surface(surfsam);
                Geom2d_Curve? pcsurfsam = null;
                Bsam = ChFi3d_mkbound(Hsurfsam, out pcsurfsam, ppopsam, ppcosam, tolapp3d, 2.0e-4);
                double upcopdif = sddif.Interference(ifaopdif).Parameter(isfirstdif);
                gp_Pnt2d ppopdif = sddif.Interference(ifaopdif).PCurveOnSurf().Value(upcopdif);
                gp_Pnt2d ppcodif = sddif.Interference(ifacodif).PCurveOnSurf().Value(uintpcdif);
                Geom_Surface? surfdif = DStr.Surface(sddif.Surf()).Surface();
                GeomAdaptor_Surface? Hsurfdif = new GeomAdaptor_Surface(surfdif);
                Geom2d_Curve? pcsurfdif = null;
                Bdif = ChFi3d_mkbound(Hsurfdif, out pcsurfdif, ppcodif, ppopdif, tolapp3d, 2.0e-4);
                gp_Pnt2d ppfacsam = new(), ppfacdif = new();
                gp_Pnt PPfacsam = new(), PPfacdif = new();
                gp_Vec VVfacsam = new(), VVfacdif = new();
                sdsam.Interference(ifaopsam).PCurveOnFace().D0(uintpcsam, out ppfacsam);
                Geom_Curve? curvopsam = 
                    DStr.Curve(sdsam.Interference(ifaopsam).LineIndex()).Curve();
                curvopsam.D1(uintpcsam, out PPfacsam, out VVfacsam);
                BRepAdaptor_Curve2d PCArcFac = new(Arcopdif, Fopsam);
                PCArcFac.D0(cpopdif.ParameterOnArc(), out ppfacdif);
                // jgv for OCC26173
                BRepAdaptor_Surface SurFopsam = new(Fopsam);
                if (SurFopsam.IsUClosed())
                {
                    double Uperiod = SurFopsam.LastUParameter() - SurFopsam.FirstUParameter();
                    if (Math.Abs(ppfacsam.X() - ppfacdif.X()) > Uperiod / 2)
                    {
                        if (ppfacdif.X() < ppfacsam.X())
                        {
                            ppfacdif.SetX(ppfacdif.X() + Uperiod);
                        }
                        else
                        {
                            ppfacdif.SetX(ppfacdif.X() - Uperiod);
                        }
                    }
                }
                //////////////////
                BRepAdaptor_Curve CArcFac = new(Arcopdif);
                CArcFac.D1(cpopdif.ParameterOnArc(), out PPfacdif, out VVfacdif);
                BRepAdaptor_Surface? HBRFopsam = new BRepAdaptor_Surface();
                BRepAdaptor_Surface BRFopsam = HBRFopsam;
                BRFopsam.Initialize(Fopsam, false);
                Geom2d_Curve? pcFopsam = 
                    ChFi3d_BuildPCurve(HBRFopsam, ppfacsam, VVfacsam, ppfacdif, VVfacdif, true);
                Bfac = ChFi3d_mkbound(HBRFopsam, pcFopsam, tolapp3d, 2.0e-4);
                GeomFill_ConstrainedFilling fil = new(8, 20);
                fil.Init(Bsam, Bdif, Bfac, true);
                Geom_Surface? Surfcoin = fil.InternalSurface();
                TopAbs_Orientation Osurfsam = sdsam.Orientation();
                Geom2d_Curve? pcnul = null;
                done = CompleteData(coin, Surfcoin, Hsurfsam, pcsurfsam, HBRFopsam, pcnul, Osurfsam, true, false, false, false,
                                                        false);
                if (!done)
                {
                    throw new Standard_Failure("concavites inverted : fail");
                }
                // Update 3 CornerData and the DS
                // ----------------------------------------
                // the corner to start,
                // -----------------------
                double P1deb = 0.0, P2deb = 0.0, P1fin = 0.0, P2fin = 0.0;
                int If1 = 0, If2 = 0, Il1 = 0, Il2 = 0, Icf = 0, Icl = 0;
                ChFiDS_CommonPoint Pf1 = coin.VertexFirstOnS1();
                ref ChFiDS_CommonPoint Pf2 = ref coin.ChangeVertexFirstOnS2();
                ChFiDS_CommonPoint Pl1 = coin.VertexLastOnS1();
                ref ChFiDS_CommonPoint Pl2 = ref coin.ChangeVertexLastOnS2();
                Pf2 = Pl2 = cpopdif;

                ChFiDS_Regul regdeb = new(), regfin = new();
                If1 = ChFi3d_IndexPointInDS(Pf1, DStr);
                If2 = ChFi3d_IndexPointInDS(Pf2, DStr);
                Il1 = ChFi3d_IndexPointInDS(Pl1, DStr);
                Il2 = If2;

                gp_Pnt2d pp1 = new(), pp2 = new();
                pp1 =
                    coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().FirstParameter());
                pp2 =
                    coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().FirstParameter());
                Geom_Curve? C3d = null;
                double tolreached = 0.0;
                ChFi3d_ComputeArete(Pf1, pp1, Pf2, pp2, DStr.Surface(coin.Surf()).Surface(), ref C3d,
                                                        ref corner.ChangeFirstPCurve(), ref P1deb, ref P2deb, tolapp3d, tol2d, out tolreached, 0);
                double tolr1 = 0.0;
                GeomAdaptor_Curve? HC3d = new GeomAdaptor_Curve(C3d);
                ChFi3d_SameParameter(HC3d, ref pcFopsam, HBRFopsam, tolapp3d, out tolr1);
                tolreached = Max(tolreached, tolr1);
                TopOpeBRepDS_Curve Tcurv1 = new(C3d, tolreached);
                Icf = DStr.AddCurve(Tcurv1);
                // place the pcurve on face in the DS
                TopAbs_Orientation OpcFopsam = sdsam.Interference(ifaopsam).Transition();
                int IFopsam = sdsam.Index(ifaopsam);
                if (isfirstsam)
                {
                    OpcFopsam = TopAbs.Reverse(OpcFopsam);
                }
                TopOpeBRepDS_SurfaceCurveInterference? interf = 
                    ChFi3d_FilCurveInDS(Icf, IFopsam, pcFopsam, OpcFopsam);
                DStr.ChangeShapeInterferences(IFopsam).Append(interf);

                regdeb.SetCurve(Icf);
                regdeb.SetS1(coin.Surf(), false);
                regdeb.SetS2(IFopsam, true);
                myRegul.Append(regdeb);
                corner.ChangeFirstCurve(Icf);
                corner.ChangeFirstParameters(P1deb, P2deb);
                corner.ChangeIndexFirstPointOnS1(If1);
                corner.ChangeIndexFirstPointOnS2(If2);

                pp1 =
                    coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().LastParameter());
                pp2 =
                    coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().LastParameter());
                ChFi3d_ComputeArete(Pl1, pp1, Pl2, pp2, DStr.Surface(coin.Surf()).Surface(), ref C3d,
                                                        ref corner.ChangeLastPCurve(), ref P1fin, ref P2fin, tolapp3d, tol2d, out tolreached, 0);
                double tolr2 = 0.0;
                HC3d.Load(C3d);
                ChFi3d_SameParameter(HC3d, ref pcsurfdif, Hsurfdif, tolapp3d, out tolr2);
                tolreached = Max(tolreached, tolr2);
                TopOpeBRepDS_Curve Tcurv2 = new(C3d, tolreached);
                Icl = DStr.AddCurve(Tcurv2);
                regfin.SetCurve(Icl);
                regfin.SetS1(coin.Surf(), false);
                regfin.SetS2(sddif.Surf(), false);
                myRegul.Append(regfin);
                corner.ChangeLastCurve(Icl);
                corner.ChangeLastParameters(P1fin, P2fin);
                corner.ChangeIndexLastPointOnS1(Il1);
                corner.ChangeIndexLastPointOnS2(Il2);

                coin.ChangeIndexOfS1(-sdsam.Surf());
                coin.ChangeIndexOfS2(0);

                corner.SetSolidIndex(stsam.SolidIndex());

                // then Stripe OnSame
                // ---------------------
                ChFiDS_FaceInterference intcoin1 = coin.InterferenceOnS1();
                stsam.SetCurve(intcoin1.LineIndex(), isfirstsam);
                stsam.InDS(isfirstsam); // filDS already works from the corner.
                stsam.ChangePCurve(isfirstsam) = coin.InterferenceOnS1().PCurveOnFace();
                stsam.SetIndexPoint(If1, isfirstsam, ifaopsam);
                stsam.SetIndexPoint(Il1, isfirstsam, ifacosam);
                stsam.SetParameters(isfirstsam, intcoin1.FirstParameter(), intcoin1.LastParameter());
                sdsam.ChangeVertex(isfirstsam, ifaopsam) = Pf1;
                sdsam.ChangeVertex(isfirstsam, ifacosam) = Pl1;
                sdsam.ChangeInterferenceOnS1().SetParameter(uintpcsam, isfirstsam);
                sdsam.ChangeInterferenceOnS2().SetParameter(uintpcsam, isfirstsam);
                if (ifaopsam == 2)
                {
                    stsam.SetOrientation(TopAbs_REVERSED, isfirstsam);
                }

                // then Stripe OnDiff
                // ---------------------
                stdif.SetCurve(Icl, isfirstdif);
                stdif.ChangePCurve(isfirstdif) = pcsurfdif;
                stdif.SetIndexPoint(Il2, isfirstdif, ifaopdif);
                stdif.SetIndexPoint(Il1, isfirstdif, ifacodif);
                stdif.SetParameters(isfirstdif, P1fin, P2fin);
                sddif.ChangeVertex(isfirstdif, ifaopdif) = Pl2;
                sddif.ChangeVertex(isfirstdif, ifacodif) = Pl1;
                sddif.ChangeInterference(ifacodif).SetParameter(uintpcdif, isfirstdif);
                if (ifaopdif == 1)
                {
                    stdif.SetOrientation(TopAbs_REVERSED, isfirstdif);
                }
            }
            if (!myEVIMap.IsBound(Vtx))
            {
                NCollection_List<int> li = new();
                myEVIMap.Bind(Vtx, li);
            }
            myEVIMap.ChangeFind(Vtx).Append(coin.Surf());
            myListStripe.Append(corner);
        }
    }

}

internal static class ChFi3d_FilBuilder_C2
{
    internal static bool ToricRotule(BRepAdaptor_Surface fac, BRepAdaptor_Surface s1, BRepAdaptor_Surface s2, ChFiDS_Stripe? c1, ChFiDS_Stripe? c2)
    {
        double tolesp = 1.0e-7;

        ChFiDS_FilSpine? sp1 = (c1.Spine() as ChFiDS_FilSpine);
        ChFiDS_FilSpine? sp2 = (c2.Spine() as ChFiDS_FilSpine);
        if (sp1 is null || sp2 is null)
        {
            return false;
        }
        if (!sp1.IsConstant() || !sp2.IsConstant())
        {
            return false;
        }
        if ((fac.GetType() != GeomAbs_Plane) || (s1.GetType() != GeomAbs_Plane)
                || (s2.GetType() != GeomAbs_Plane))
        {
            return false;
        }
        gp_Dir df = fac.Plane().Position().Direction();
        gp_Dir ds1 = s1.Plane().Position().Direction();
        gp_Dir ds2 = s2.Plane().Position().Direction();
        if (Math.Abs(df.Dot(ds1)) >= tolesp || Math.Abs(df.Dot(ds2)) >= tolesp)
        {
            return false;
        }
        double r1 = sp1.Radius();
        double r2 = sp2.Radius();
        return Math.Abs(r1 - r2) < tolesp;
    }

    // OCCT.NET: OCCT changes this curve in place; one frozen elsewhere is changed on a copy (decisions 974, 1088).
    internal static T Writable<T>(T theCurve) where T : Geom2d_Geometry =>
        theCurve.IsFrozen() ? (T)theCurve.Copy() : theCurve;


    internal static void RemoveSD(ChFiDS_Stripe? Stripe, int num1, int num2)
    {
        NCollection_Sequence<ChFiDS_SurfData> Seq = 
            Stripe.ChangeSetOfSurfData();
        if (Seq.IsEmpty())
        {
            return;
        }
        if (num1 == num2)
        {
            Seq.Remove(num1);
        }
        else
        {
            Seq.Remove(num1, num2);
        }
    }


}
