// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder_C2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Properties;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.FilBuilder_C2;
using static Kerf.Engine.Fillets.Data.ErrorStatus;
using static Kerf.Engine.Fillets.Data.State;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Fillets;
internal partial class FilBuilder
{



    //=======================================================================
    // function : ToricRotule
    // purpose  : Test if it is a particular case of torus routine.
    //           Three planes with two constant incident fillets
    //           of the same radius and the third face perpendicular to
    //           two others are required.
    //=======================================================================

    //=================================================================================================

    internal override void PerformTwoCorner(int index)
    {

        _done = false;
        Vertex vtx = _vDataMap.FindKey(index);
        DsDataStructure dStr = _ds.ChangeDS();
        NCollection_List<Stripe>.Iterator it = new();
        it.Initialize(_vDataMap[index]);
        Stripe? st1 = null, st2 = null;
        int sens1 = 0, sens2 = 0;
        int isd1 = 0, isd2 = 0, i1 = 0, i2 = 0;
        SurfData? sd1 = null, sd2 = null;
        NCollection_Sequence<SurfData> seqFil1 = new(), seqFil2 = new();
        Surface? surf1 = null, surf2 = null;
        bool okinterCC = false, okvisavis = false, sameSide = false;
        int iFaCo1 = 0, iFaCo2 = 0;
        double uIntPC1 = 0.0, uIntPC2 = 0.0;
        Face faCo = new();
        Edge e1 = new(), e2 = new(), e = new();
        Vertex v1 = new(), v2 = new();
        //  gp_Pnt P1,P2;
        int nbsurf1 = 0, nbsurf2 = 0, deb1 = 0, fin1 = 0, deb2 = 0, fin2 = 0;
        double parE1 = 0.0, parE2 = 0.0;
        // Necessary information on fillets is extracted
        //------------------------------------------------------

        // the first
        //----------

        st1 = it.Value();
        isd1 = ChFi3d_IndexOfSurfData(vtx, st1, out sens1);

        // the second
        //----------
        it.Next();
        st2 = it.Value();
        if (st2 == st1)
        {
            sens2 = -1;
            isd2 = st2.SetOfSurfData().Length();
        }
        else
        {
            isd2 = ChFi3d_IndexOfSurfData(vtx, st2, out sens2);
        }

        // If two edges to rounded are tangent GeomPlate is called

        if (sens1 == 1)
        {
            e1 = st1.Spine().Edges(1);
        }
        else
        {
            e1 = st1.Spine().Edges(st1.Spine().EdgeCount);
        }

        if (sens2 == 1)
        {
            e2 = st2.Spine().Edges(1);
        }
        else
        {
            e2 = st2.Spine().Edges(st2.Spine().EdgeCount);
        }

        EdgeCurveAdaptor bCurv1 = new(e1);
        EdgeCurveAdaptor bCurv2 = new(e2);
        parE1 = Tool.Parameter(vtx, e1);
        parE2 = Tool.Parameter(vtx, e2);
        CLProps cl1 = new(bCurv1, parE1, 1, 1.0e-4);
        CLProps cl2 = new(bCurv2, parE2, 1, 1.0e-4);
        Direction3D dir1 = new(), dir2 = new();
        cl1.Tangent(out dir1);
        cl2.Tangent(out dir2);
        if (sens1 == -1)
        {
            dir1 = dir1.Reversed();
        }
        if (sens2 == -1)
        {
            dir2 = dir2.Reversed();
        }
        double ang1 = 0.0;
        ang1 = Math.Abs(dir1.AngleTo(dir2));
        if (ang1 < M_PI / 180.0)
        {
            PerformMoreThreeCorner(index, 2);
            _done = true;
            return;
        }

        okinterCC = ChFi3d_IsInFront(dStr, st1, st2, isd1, isd2, sens1, sens2, ref uIntPC1, ref uIntPC2, ref faCo,
                                                                  ref sameSide, ref iFaCo1, ref iFaCo2, ref okvisavis, vtx, true);

        bool trouve = false;
        if (!okvisavis)
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
                if (sens1 == 1)
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
                if (sens2 == 1)
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
                isd1 = i1;
                for (i2 = deb2; i2 <= fin2 && !trouve; i2++)
                {
                    isd2 = i2;

                    okinterCC = ChFi3d_IsInFront(dStr, st1, st2, isd1, isd2, sens1, sens2, ref uIntPC1, ref uIntPC2, ref faCo,
                                                                              ref sameSide, ref iFaCo1, ref iFaCo2, ref okvisavis, vtx, true);
                    trouve = okvisavis;
                }
            }
            if (!trouve)
            {
                PerformMoreThreeCorner(index, 2);
                _done = true;
                return;
            }
            else
            {
                if (sens1 == 1 && isd1 != 1)
                {
                    RemoveSD(st1, 1, 1);
                }
                if (sens1 != 1 && isd1 != nbsurf1)
                {
                    RemoveSD(st1, fin1, fin1);
                }
                if (sens2 == 1 && isd2 != 1)
                {
                    RemoveSD(st2, 1, 1);
                }
                if (sens2 != 1 && isd2 != nbsurf2)
                {
                    RemoveSD(st2, fin2, fin2);
                }
            }
            isd1 = ChFi3d_IndexOfSurfData(vtx, st1, out sens1);
            isd2 = ChFi3d_IndexOfSurfData(vtx, st2, out sens2);
        }
        // throw StdFail_NotDone("TwoCorner : no common face");
        int iFaArc1 = 3 - iFaCo1, iFaArc2 = 3 - iFaCo2;
        seqFil1 = st1.ChangeSetOfSurfData(); // OCCT copies the sequence; nothing writes into the copy
        seqFil2 = st2.ChangeSetOfSurfData();
        sd1 = seqFil1.ChangeValue(isd1);
        surf1 = dStr.Surface(sd1.Surf()).Surface();
        sd2 = seqFil2.ChangeValue(isd2);
        surf2 = dStr.Surface(sd2.Surf()).Surface();
        Orientation oFaCo = faCo.Orientation;
        // The concavities are analyzed and the opposite face and the
        // eventual intersection of 2 pcurves on this face are found.

        State stat1 = default, stat2 = default;
        bool isfirst1 = (sens1 == 1);
        bool isfirst2 = (sens2 == 1);
        stat1 = st1.Spine().Status(isfirst1);
        stat2 = st2.Spine().Status(isfirst2);
        bool c1biseau = (stat1 == State.AllSame);
        bool c1rotule = (stat1 == State.OnSame && stat2 == State.OnSame);

        // It is checked if the fillets have a commonpoint on a common arc.
        // This edge is the pivot of the bevel or the knee.

        ref CommonPoint cp1 = ref sd1.ChangeVertex(isfirst1, iFaArc1);
        ref CommonPoint cp2 = ref sd2.ChangeVertex(isfirst2, iFaArc2);

        bool resetcp1 = false;
        bool resetcp2 = false;

        Edge pivot = new();
        bool yapiv = false;
        if (cp1.IsOnArc)
        {
            pivot = cp1.Arc();
        }
        else
        {
            PerformMoreThreeCorner(index, 2);
            _done = true;
            return;
        }
        if (cp1.IsOnArc && cp2.IsOnArc)
        {
            yapiv = (pivot.IsSame(cp2.Arc()));
        }
        EdgeCurveAdaptor? hpivot = null;
        bool sameparam = false;
        double parCP1 = 0.0, parCP2 = 0.0;
        if (yapiv)
        {
            hpivot = new EdgeCurveAdaptor(pivot);
            parCP1 = cp1.ParameterOnArc();
            parCP2 = cp2.ParameterOnArc();
            Point3D tst1 = hpivot.Value(parCP1);
            Point3D tst2 = hpivot.Value(parCP2);
            sameparam = tst1.DistanceTo(tst2) <= _tolapp3d;
        }
        FaceSurfaceAdaptor? hFaCo = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hFaPiv = null;
        FaceSurfaceAdaptor? hbrs1 = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hbrs2 = new FaceSurfaceAdaptor();

        FaceSurfaceAdaptor brs1 = hbrs1;
        FaceSurfaceAdaptor brs2 = hbrs2;
        FaceSurfaceAdaptor brFaCo = hFaCo;
        brFaCo.Initialize(faCo);

        Face ff1 = new(), ff2 = new(), f = new(), faPiv = new();
        Orientation pctrans = Forward;
        BSplineCurve2D? pCurveOnPiv = null;
        ff1 = ShapeCasts.Face(dStr.Shape(sd1.Index(iFaArc1)));
        ff2 = ShapeCasts.Face(dStr.Shape(sd2.Index(iFaArc2)));
        if (ff1.IsNull || ff2.IsNull)
        {
            PerformMoreThreeCorner(index, 2);
            _done = true;
            return;
        }
        brs1.Initialize(ff1);
        brs2.Initialize(ff2);

        if (yapiv)
        {
            NCollection_List<Shape>.Iterator kt = new();
            bool ok1 = false, ok2 = false;
            for (kt.Initialize(_efMap[pivot]); kt.More(); kt.Next())
            {
                f = ShapeCasts.Face(kt.Value());
                if (!ok1 && ff1.IsSame(f))
                {
                    ok1 = true;
                }
                if (!ok2 && ff2.IsSame(f))
                {
                    ok2 = true;
                }
            }
            if (!ok1 || !ok2)
            {
                PerformMoreThreeCorner(index, 2);
                _done = true;
                return;
            }
        }


        // bevel
        //------
        CommonPoint cp11 = new(), cp12 = new(), cp21 = new(), cp22 = new();
        FaceInterference intf11 = new(), intf12 = new(), intf21 = new(), intf22 = new();

        if (c1biseau)
        {

            _done = PerformTwoCornerbyInter(index);


            if (!_done)
            {
                PerformMoreThreeCorner(index, 2);
                _done = true;
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

            _done = PerformTwoCornerbyInter(index);

            if (!_done)
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
                _done = false;
            }
        }

        if (!c1biseau && !_done)
        {
            // new cornerdata is created
            //-------------------------------
            Stripe? corner = new Stripe();
            ref NCollection_HSequence<SurfData>? cornerset = ref 
                corner.ChangeSetOfSurfData();
            cornerset = new NCollection_HSequence<SurfData>();
            SurfData? coin = new SurfData();
            cornerset.Append(coin);

            if (sameSide)
            {
                if (ToricRotule(brFaCo, brs1, brs2, st1, st2))
                {
                    // Direct construction.
                    // ---------------------

                    int bid = 0;
                    Orientation ori = oFaCo;
                    Orientation oriS = st1.Orientation(iFaCo1);
                    Orientation off1 = ff1.Orientation;
                    Orientation oriSFF1 = st1.Orientation(iFaArc1);
                    bid = 1;
                    bid = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref ori, ref off1, oriS, oriSFF1, bid);
                    Orientation op1 = Forward, op2 = Forward;
                    if (yapiv)
                    {
                        bid = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(brs1, brs2, pivot, out op1, out op2);
                    }
                    op1 = global::Kerf.Engine.Topology.TopAbs.Reverse(op1);
                    op2 = global::Kerf.Engine.Topology.TopAbs.Reverse(op2);
                    double radius = (st1.Spine() as FilSpine).Radius();
                    _done = global::Kerf.Engine.Fillets.Analytic.ComputeData.ComputeCorner(dStr, coin, hFaCo, hbrs1, hbrs2, oFaCo, ori, op1, op2,
                                                                                                            radius);
                }
                else
                {
                    // Construction by filling remplissage
                    // ----------------------------
                    double uPCArc1 = 0.0, uPCArc2 = 0.0;
                    Point2D p2da1 = new(), p2df1 = new(), p2da2 = new(), p2df2 = new(), p2dfac1 = new(), p2dfac2 = new();
                    Vector2D v2dfac1 = new(), v2dfac2 = new();
                    Boundary? b1 = null, b2 = null, bpiv = null, bfac = null;
                    uPCArc1 = sd1.Interference(iFaArc1).Parameter(isfirst1);
                    p2da1 = sd1.Interference(iFaArc1).PCurveOnSurf().Value(uPCArc1);
                    p2df1 = sd1.Interference(iFaCo1).PCurveOnSurf().Value(uPCArc1);
                    sd1.Interference(iFaCo1).PCurveOnFace().D1(uPCArc1, out p2dfac1, out v2dfac1);
                    uPCArc2 = sd2.Interference(iFaArc2).Parameter(isfirst2);
                    p2da2 = sd2.Interference(iFaArc2).PCurveOnSurf().Value(uPCArc2);
                    p2df2 = sd2.Interference(iFaCo2).PCurveOnSurf().Value(uPCArc2);
                    sd2.Interference(iFaCo2).PCurveOnFace().D1(uPCArc2, out p2dfac2, out v2dfac2);
                    b1 = ChFi3d_mkbound(surf1, p2df1, p2da1, _tolapp3d, 2.0e-4);
                    b2 = ChFi3d_mkbound(surf2, p2df2, p2da2, _tolapp3d, 2.0e-4);
                    Curve2D? pCurveOnFace = null;
                    bfac = ChFi3d_mkbound(hFaCo, out pCurveOnFace, sens1, p2dfac1, v2dfac1, sens2, p2dfac2, v2dfac2, _tolapp3d,
                                                                2.0e-4);
                    ConstrainedFilling fil = new(8, 20);
                    if (sameparam)
                    {
                        fil.Init(bfac, b2, b1, true);
                    }
                    else
                    {
                        CurveAdaptorBase? hPivTrim = 
                            hpivot.Trim(Min(parCP1, parCP2), Max(parCP1, parCP2), _tolesp);
                        bpiv = new SimpleBound(hPivTrim, _tolapp3d, 2.0e-4);
                        fil.Init(bfac, b2, bpiv, b1, true);
                        EdgeCurve2DAdaptor pcpivot = new();
                        Vector3D dArc = new(), dcf = new();
                        Point3D bidon = new();
                        hpivot.D1(parCP1, out bidon, out dArc);
                        double fb1 = 0.0, lb1 = 0.0;
                        b1.Bounds(out fb1, out lb1);
                        b1.D1(lb1, out bidon, out dcf);
                        bool pivotverslebas = dArc.Dot(dcf) <= 0.0;
                        bool pcfalenvers = (parCP1 > parCP2);
                        if ((pivotverslebas && !pcfalenvers) || (!pivotverslebas && pcfalenvers))
                        {
                            faPiv = ff2;
                            hFaPiv = hbrs2;
                            resetcp2 = true;
                        }
                        else
                        {
                            faPiv = ff1;
                            hFaPiv = hbrs1;
                            resetcp1 = true;
                        }
                        faPiv.Orient(Forward);
                        pcpivot.Initialize(pivot, faPiv);
                        Explorer expl = new();
                        for (expl.Init(faPiv, ShapeType.Edge); expl.More(); expl.Next())
                        {
                            if (expl.Current().IsSame(pivot))
                            {
                                pctrans = expl.Current().Orientation;
                                break;
                            }
                        }
                        if (pcpivot.GetType() != BSplineCurve)
                        {
                            TrimmedCurve2D? trc = 
                                new TrimmedCurve2D(pcpivot.Curve(), Min(parCP1, parCP2), Max(parCP1, parCP2));
                            pCurveOnPiv = Geom2dConvert2D.CurveToBSplineCurve(trc);
                        }
                        else
                        {
                            pCurveOnPiv =
                                Geom2dConvert2D.SplitBSplineCurve((pcpivot.Curve() as BSplineCurve2D), Min(parCP1, parCP2),
                                                                                                  Max(parCP1, parCP2), _tol2d);
                        }
                        pCurveOnPiv = Writable(pCurveOnPiv);
                        NCollection_Array1<double> kk = new(pCurveOnPiv.InternalKnots());
                        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, 1.0, kk);
                        pCurveOnPiv.SetKnots(kk);
                        if (pcfalenvers)
                        {
                            pCurveOnPiv.Reverse();
                            pctrans = global::Kerf.Engine.Topology.TopAbs.Reverse(pctrans);
                        }
                    }
                    Surface? surfcoin = fil.InternalSurface();
                    _done = CompleteData(coin, surfcoin, hFaCo, pCurveOnFace, hFaPiv, pCurveOnPiv,
                                                            oFaCo, true, false, false, false, false);
                }
                if (_done)
                {
                    // Update 3 CornerData and the DS
                    // ----------------------------------------
                    if (resetcp1)
                    {
                        Point3D pjyl = cp1.Point();
                        double tolsav = cp1.Tolerance();
                        cp1.Reset();
                        cp1.SetPoint(pjyl);
                        cp1.SetTolerance(tolsav);
                    }
                    else if (resetcp2)
                    {
                        Point3D pjyl = cp2.Point();
                        double tolsav = cp2.Tolerance();
                        cp2.Reset();
                        cp2.SetPoint(pjyl);
                        cp2.SetTolerance(tolsav);
                    }
                    double p1deb = 0.0, p2deb = 0.0, p1fin = 0.0, p2fin = 0.0;
                    int if1 = 0, if2 = 0, il1 = 0, il2 = 0, icf = 0, icl = 0;
                    CommonPoint pf1 = coin.VertexFirstOnS1();
                    ref CommonPoint pf2 = ref coin.ChangeVertexFirstOnS2();
                    CommonPoint pl1 = coin.VertexLastOnS1();
                    ref CommonPoint pl2 = ref coin.ChangeVertexLastOnS2();
                    pf2 = cp1;
                    pl2 = cp2;

                    // the corner to start,
                    // -----------------------
                    Regul regdeb = new(), regfin = new();
                    if1 = ChFi3d_IndexPointInDS(pf1, dStr);
                    if2 = ChFi3d_IndexPointInDS(pf2, dStr);
                    il1 = ChFi3d_IndexPointInDS(pl1, dStr);
                    if (sameparam)
                    {
                        il2 = if2;
                    }
                    else
                    {
                        il2 = ChFi3d_IndexPointInDS(pl2, dStr);
                    }

                    Point2D pp1 = new(), pp2 = new();
                    pp1 =
                        coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().FirstParameter());
                    pp2 =
                        coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().FirstParameter());
                    Curve3D? c3d = null;
                    double tolreached = 0.0;
                    ChFi3d_ComputeArete(pf1, pp1, pf2, pp2, dStr.Surface(coin.Surf()).Surface(), ref c3d,
                                                            ref corner.ChangeFirstPCurve(), ref p1deb, ref p2deb, _tolapp3d, _tol2d, out tolreached, 0);
                    double par1 = sd1.Interference(iFaArc1).Parameter(isfirst1);
                    pp1 = sd1.Interference(iFaCo1).PCurveOnSurf().Value(par1);
                    pp2 = sd1.Interference(iFaArc1).PCurveOnSurf().Value(par1);
                    double tolr1 = 0.0;
                    ChFi3d_ComputePCurv(c3d, pp1, pp2, out st1.ChangePCurve(isfirst1), dStr.Surface(sd1.Surf()).Surface(), p1deb,
                                                            p2deb, _tolapp3d, out tolr1);
                    tolreached = Max(tolreached, tolr1);
                    DsCurve tcurv1 = new(c3d, tolreached);
                    icf = dStr.AddCurve(tcurv1);
                    regdeb.SetCurve(icf);
                    regdeb.SetS1(coin.Surf(), false);
                    regdeb.SetS2(sd1.Surf(), false);
                    _regul.Append(regdeb);
                    corner.ChangeFirstCurve(icf);
                    corner.ChangeFirstParameters(p1deb, p2deb);
                    corner.ChangeIndexFirstPointOnS1(if1);
                    corner.ChangeIndexFirstPointOnS2(if2);

                    pp1 =
                        coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().LastParameter());
                    pp2 =
                        coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().LastParameter());
                    ChFi3d_ComputeArete(pl1, pp1, pl2, pp2, dStr.Surface(coin.Surf()).Surface(), ref c3d,
                                                            ref corner.ChangeLastPCurve(), ref p1fin, ref p2fin, _tolapp3d, _tol2d, out tolreached, 0);
                    double par2 = sd2.Interference(iFaArc2).Parameter(isfirst2);
                    pp1 = sd2.Interference(iFaCo2).PCurveOnSurf().Value(par2);
                    pp2 = sd2.Interference(iFaArc2).PCurveOnSurf().Value(par2);
                    double tolr2 = 0.0;
                    ChFi3d_ComputePCurv(c3d, pp1, pp2, out st2.ChangePCurve(isfirst2), dStr.Surface(sd2.Surf()).Surface(), p1deb,
                                                            p2deb, _tolapp3d, out tolr2);
                    tolreached = Max(tolreached, tolr2);
                    DsCurve tcurv2 = new(c3d, tolreached);
                    icl = dStr.AddCurve(tcurv2);
                    regfin.SetCurve(icl);
                    regfin.SetS1(coin.Surf(), false);
                    regfin.SetS2(sd2.Surf(), false);
                    _regul.Append(regfin);
                    corner.ChangeLastCurve(icl);
                    corner.ChangeLastParameters(p1fin, p2fin);
                    corner.ChangeIndexLastPointOnS1(il1);
                    corner.ChangeIndexLastPointOnS2(il2);

                    coin.ChangeIndexOfS1(dStr.AddShape(faCo));
                    if (sameparam)
                    {
                        coin.ChangeIndexOfS2(0);
                    }
                    else
                    {
                        coin.ChangeIndexOfS2(dStr.AddShape(faPiv));
                        coin.ChangeInterferenceOnS2().SetTransition(pctrans);
                    }
                    corner.SetSolidIndex(st1.SolidIndex());

                    // then the starting Stripe,
                    // ------------------------
                    st1.SetCurve(icf, isfirst1);
                    st1.SetIndexPoint(if1, isfirst1, iFaCo1);
                    st1.SetIndexPoint(if2, isfirst1, iFaArc1);
                    st1.SetParameters(isfirst1, p1deb, p2deb);
                    sd1.ChangeVertex(isfirst1, iFaCo1) = pf1;
                    sd1.ChangeVertex(isfirst1, iFaArc1) = pf2;
                    sd1.ChangeInterference(iFaCo1).SetParameter(par1, isfirst1);
                    if (iFaCo1 == 2)
                    {
                        st1.SetOrientation(Reversed, isfirst1);
                    }

                    // then the end Stripe,
                    // -------------------------
                    st2.SetCurve(icl, isfirst2);
                    st2.SetIndexPoint(il1, isfirst2, iFaCo2);
                    st2.SetIndexPoint(il2, isfirst2, iFaArc2);
                    st2.SetParameters(isfirst2, p1fin, p2fin);
                    sd2.ChangeVertex(isfirst2, iFaCo2) = pl1;
                    sd2.ChangeVertex(isfirst2, iFaArc2) = pl2;
                    sd2.ChangeInterference(iFaCo2).SetParameter(par2, isfirst2);
                    if (iFaCo2 == 2)
                    {
                        st2.SetOrientation(Reversed, isfirst2);
                    }
                }
            }
            else
            {
                // it is necessary to make difference with
                if (!okinterCC)
                {
                    throw new FailureException("TwoCorner : No intersection pc pc");
                }
                Stripe? stsam = null, stdif = null;
                SurfData? sdsam = null, sddif = null;
                double uintpcsam = 0.0, uintpcdif = 0.0;
                int ifacosam = 0, ifacodif = 0, ifaopsam = 0, ifaopdif = 0;
                bool isfirstsam = false, isfirstdif = false;
                if (stat1 == State.OnSame && stat2 == State.OnDiff)
                {
                    stsam = st1;
                    sdsam = sd1;
                    uintpcsam = uIntPC1;
                    ifacosam = iFaCo1;
                    ifaopsam = iFaArc1;
                    isfirstsam = isfirst1;
                    stdif = st2;
                    sddif = sd2;
                    uintpcdif = uIntPC2;
                    ifacodif = iFaCo2;
                    ifaopdif = iFaArc2;
                    isfirstdif = isfirst2;
                }
                else if (stat1 == State.OnDiff && stat2 == State.OnSame)
                {
                    stsam = st2;
                    sdsam = sd2;
                    uintpcsam = uIntPC2;
                    ifacosam = iFaCo2;
                    ifaopsam = iFaArc2;
                    isfirstsam = isfirst2;
                    stdif = st1;
                    sddif = sd1;
                    uintpcdif = uIntPC1;
                    ifacodif = iFaCo1;
                    ifaopdif = iFaArc1;
                    isfirstdif = isfirst1;
                }
                else
                {
                    throw new FailureException("TwoCorner : Config unknown");
                }
                // It is checked if surface ondiff has a point on arc from the side opposed
                // to the common face and if this arc is connected to the base face
                // opposed to common face of the surface onsame.
                ref CommonPoint cpopdif = ref sddif.ChangeVertex(isfirstdif, ifaopdif);
                if (!cpopdif.IsOnArc)
                {
                    throw new FailureException("TwoCorner : No point on restriction on surface OnDiff");
                }
                Edge arcopdif = cpopdif.Arc();
                Face fopsam = ShapeCasts.Face(dStr.Shape(sdsam.Index(ifaopsam)));
                Explorer ex = new();
                for (ex.Init(fopsam, ShapeType.Edge); ex.More(); ex.Next())
                {
                    if (ex.Current().IsSame(arcopdif))
                    {
                        break;
                    }
                    else if (!ex.More())
                    {
                        throw new FailureException("TwoCorner : No common face to loop the contour");
                    }
                }
                Boundary? bsam = null, bdif = null, bfac = null;
                Point2D ppopsam = sdsam.Interference(ifaopsam).PCurveOnSurf().Value(uintpcsam);
                Point2D ppcosam = sdsam.Interference(ifacosam).PCurveOnSurf().Value(uintpcsam);
                Surface? surfsam = dStr.Surface(sdsam.Surf()).Surface();
                SurfaceAdaptor? hsurfsam = new SurfaceAdaptor(surfsam);
                Curve2D? pcsurfsam = null;
                bsam = ChFi3d_mkbound(hsurfsam, out pcsurfsam, ppopsam, ppcosam, _tolapp3d, 2.0e-4);
                double upcopdif = sddif.Interference(ifaopdif).Parameter(isfirstdif);
                Point2D ppopdif = sddif.Interference(ifaopdif).PCurveOnSurf().Value(upcopdif);
                Point2D ppcodif = sddif.Interference(ifacodif).PCurveOnSurf().Value(uintpcdif);
                Surface? surfdif = dStr.Surface(sddif.Surf()).Surface();
                SurfaceAdaptor? hsurfdif = new SurfaceAdaptor(surfdif);
                Curve2D? pcsurfdif = null;
                bdif = ChFi3d_mkbound(hsurfdif, out pcsurfdif, ppcodif, ppopdif, _tolapp3d, 2.0e-4);
                Point2D ppfacsam = new(), ppfacdif = new();
                Point3D pPfacsam = new(), pPfacdif = new();
                Vector3D vVfacsam = new(), vVfacdif = new();
                sdsam.Interference(ifaopsam).PCurveOnFace().D0(uintpcsam, out ppfacsam);
                Curve3D? curvopsam = 
                    dStr.Curve(sdsam.Interference(ifaopsam).LineIndex()).Curve();
                curvopsam.D1(uintpcsam, out pPfacsam, out vVfacsam);
                EdgeCurve2DAdaptor pcArcFac = new(arcopdif, fopsam);
                pcArcFac.D0(cpopdif.ParameterOnArc(), out ppfacdif);
                // jgv for OCC26173
                FaceSurfaceAdaptor surFopsam = new(fopsam);
                if (surFopsam.IsUClosed)
                {
                    double uperiod = surFopsam.LastUParameter() - surFopsam.FirstUParameter();
                    if (Math.Abs(ppfacsam.X - ppfacdif.X) > uperiod / 2)
                    {
                        if (ppfacdif.X < ppfacsam.X)
                        {
                            ppfacdif = ppfacdif.WithX(ppfacdif.X + uperiod);
                        }
                        else
                        {
                            ppfacdif = ppfacdif.WithX(ppfacdif.X - uperiod);
                        }
                    }
                }
                //////////////////
                EdgeCurveAdaptor cArcFac = new(arcopdif);
                cArcFac.D1(cpopdif.ParameterOnArc(), out pPfacdif, out vVfacdif);
                FaceSurfaceAdaptor? hbrFopsam = new FaceSurfaceAdaptor();
                FaceSurfaceAdaptor brFopsam = hbrFopsam;
                brFopsam.Initialize(fopsam, false);
                Curve2D? pcFopsam = 
                    ChFi3d_BuildPCurve(hbrFopsam, ppfacsam, vVfacsam, ppfacdif, vVfacdif, true);
                bfac = ChFi3d_mkbound(hbrFopsam, pcFopsam, _tolapp3d, 2.0e-4);
                ConstrainedFilling fil = new(8, 20);
                fil.Init(bsam, bdif, bfac, true);
                Surface? surfcoin = fil.InternalSurface();
                Orientation osurfsam = sdsam.Orientation();
                Curve2D? pcnul = null;
                _done = CompleteData(coin, surfcoin, hsurfsam, pcsurfsam, hbrFopsam, pcnul, osurfsam, true, false, false, false,
                                                        false);
                if (!_done)
                {
                    throw new FailureException("concavites inverted : fail");
                }
                // Update 3 CornerData and the DS
                // ----------------------------------------
                // the corner to start,
                // -----------------------
                double p1deb = 0.0, p2deb = 0.0, p1fin = 0.0, p2fin = 0.0;
                int if1 = 0, if2 = 0, il1 = 0, il2 = 0, icf = 0, icl = 0;
                CommonPoint pf1 = coin.VertexFirstOnS1();
                ref CommonPoint pf2 = ref coin.ChangeVertexFirstOnS2();
                CommonPoint pl1 = coin.VertexLastOnS1();
                ref CommonPoint pl2 = ref coin.ChangeVertexLastOnS2();
                pf2 = pl2 = cpopdif;

                Regul regdeb = new(), regfin = new();
                if1 = ChFi3d_IndexPointInDS(pf1, dStr);
                if2 = ChFi3d_IndexPointInDS(pf2, dStr);
                il1 = ChFi3d_IndexPointInDS(pl1, dStr);
                il2 = if2;

                Point2D pp1 = new(), pp2 = new();
                pp1 =
                    coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().FirstParameter());
                pp2 =
                    coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().FirstParameter());
                Curve3D? c3d = null;
                double tolreached = 0.0;
                ChFi3d_ComputeArete(pf1, pp1, pf2, pp2, dStr.Surface(coin.Surf()).Surface(), ref c3d,
                                                        ref corner.ChangeFirstPCurve(), ref p1deb, ref p2deb, _tolapp3d, _tol2d, out tolreached, 0);
                double tolr1 = 0.0;
                CurveAdaptor? hc3d = new CurveAdaptor(c3d);
                ChFi3d_SameParameter(hc3d, ref pcFopsam, hbrFopsam, _tolapp3d, out tolr1);
                tolreached = Max(tolreached, tolr1);
                DsCurve tcurv1 = new(c3d, tolreached);
                icf = dStr.AddCurve(tcurv1);
                // place the pcurve on face in the DS
                Orientation opcFopsam = sdsam.Interference(ifaopsam).Transition();
                int iFopsam = sdsam.Index(ifaopsam);
                if (isfirstsam)
                {
                    opcFopsam = global::Kerf.Engine.Topology.TopAbs.Reverse(opcFopsam);
                }
                DsSurfaceCurveInterference? interf = 
                    ChFi3d_FilCurveInDS(icf, iFopsam, pcFopsam, opcFopsam);
                dStr.ChangeShapeInterferences(iFopsam).Append(interf);

                regdeb.SetCurve(icf);
                regdeb.SetS1(coin.Surf(), false);
                regdeb.SetS2(iFopsam, true);
                _regul.Append(regdeb);
                corner.ChangeFirstCurve(icf);
                corner.ChangeFirstParameters(p1deb, p2deb);
                corner.ChangeIndexFirstPointOnS1(if1);
                corner.ChangeIndexFirstPointOnS2(if2);

                pp1 =
                    coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().LastParameter());
                pp2 =
                    coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().LastParameter());
                ChFi3d_ComputeArete(pl1, pp1, pl2, pp2, dStr.Surface(coin.Surf()).Surface(), ref c3d,
                                                        ref corner.ChangeLastPCurve(), ref p1fin, ref p2fin, _tolapp3d, _tol2d, out tolreached, 0);
                double tolr2 = 0.0;
                hc3d.Load(c3d);
                ChFi3d_SameParameter(hc3d, ref pcsurfdif, hsurfdif, _tolapp3d, out tolr2);
                tolreached = Max(tolreached, tolr2);
                DsCurve tcurv2 = new(c3d, tolreached);
                icl = dStr.AddCurve(tcurv2);
                regfin.SetCurve(icl);
                regfin.SetS1(coin.Surf(), false);
                regfin.SetS2(sddif.Surf(), false);
                _regul.Append(regfin);
                corner.ChangeLastCurve(icl);
                corner.ChangeLastParameters(p1fin, p2fin);
                corner.ChangeIndexLastPointOnS1(il1);
                corner.ChangeIndexLastPointOnS2(il2);

                coin.ChangeIndexOfS1(-sdsam.Surf());
                coin.ChangeIndexOfS2(0);

                corner.SetSolidIndex(stsam.SolidIndex());

                // then Stripe OnSame
                // ---------------------
                FaceInterference intcoin1 = coin.InterferenceOnS1();
                stsam.SetCurve(intcoin1.LineIndex(), isfirstsam);
                stsam.InDS(isfirstsam); // filDS already works from the corner.
                stsam.ChangePCurve(isfirstsam) = coin.InterferenceOnS1().PCurveOnFace();
                stsam.SetIndexPoint(if1, isfirstsam, ifaopsam);
                stsam.SetIndexPoint(il1, isfirstsam, ifacosam);
                stsam.SetParameters(isfirstsam, intcoin1.FirstParameter(), intcoin1.LastParameter());
                sdsam.ChangeVertex(isfirstsam, ifaopsam) = pf1;
                sdsam.ChangeVertex(isfirstsam, ifacosam) = pl1;
                sdsam.ChangeInterferenceOnS1().SetParameter(uintpcsam, isfirstsam);
                sdsam.ChangeInterferenceOnS2().SetParameter(uintpcsam, isfirstsam);
                if (ifaopsam == 2)
                {
                    stsam.SetOrientation(Reversed, isfirstsam);
                }

                // then Stripe OnDiff
                // ---------------------
                stdif.SetCurve(icl, isfirstdif);
                stdif.ChangePCurve(isfirstdif) = pcsurfdif;
                stdif.SetIndexPoint(il2, isfirstdif, ifaopdif);
                stdif.SetIndexPoint(il1, isfirstdif, ifacodif);
                stdif.SetParameters(isfirstdif, p1fin, p2fin);
                sddif.ChangeVertex(isfirstdif, ifaopdif) = pl2;
                sddif.ChangeVertex(isfirstdif, ifacodif) = pl1;
                sddif.ChangeInterference(ifacodif).SetParameter(uintpcdif, isfirstdif);
                if (ifaopdif == 1)
                {
                    stdif.SetOrientation(Reversed, isfirstdif);
                }
            }
            if (!_eviMap.ContainsKey(vtx))
            {
                NCollection_List<int> li = new();
                _eviMap[vtx] = li;
            }
            _eviMap.GetValue(vtx).Append(coin.Surf());
            _listStripe.Append(corner);
        }
    }

}

internal static class FilBuilder_C2
{
    internal static bool ToricRotule(FaceSurfaceAdaptor fac, FaceSurfaceAdaptor s1, FaceSurfaceAdaptor s2, Stripe? c1, Stripe? c2)
    {
        double tolesp = 1.0e-7;

        FilSpine? sp1 = (c1.Spine() as FilSpine);
        FilSpine? sp2 = (c2.Spine() as FilSpine);
        if (sp1 is null || sp2 is null)
        {
            return false;
        }
        if (!sp1.IsConstant() || !sp2.IsConstant())
        {
            return false;
        }
        if ((fac.GetType() != Plane) || (s1.GetType() != Plane)
                || (s2.GetType() != Plane))
        {
            return false;
        }
        Direction3D df = fac.Plane().Position.Direction;
        Direction3D ds1 = s1.Plane().Position.Direction;
        Direction3D ds2 = s2.Plane().Position.Direction;
        if (Math.Abs(df.Dot(ds1)) >= tolesp || Math.Abs(df.Dot(ds2)) >= tolesp)
        {
            return false;
        }
        double r1 = sp1.Radius();
        double r2 = sp2.Radius();
        return Math.Abs(r1 - r2) < tolesp;
    }

    // Kerf: OCCT changes this curve in place; one frozen elsewhere is changed on a copy (decisions 974, 1088).
    internal static T Writable<T>(T curve) where T : Geometry2D =>
        curve.IsFrozen ? (T)curve.Copy() : curve;


    internal static void RemoveSD(Stripe? stripe, int num1, int num2)
    {
        NCollection_Sequence<SurfData> seq = 
            stripe.ChangeSetOfSurfData();
        if (seq.IsEmpty())
        {
            return;
        }
        if (num1 == num2)
        {
            seq.Remove(num1);
        }
        else
        {
            seq.Remove(num1, num2);
        }
    }


}
