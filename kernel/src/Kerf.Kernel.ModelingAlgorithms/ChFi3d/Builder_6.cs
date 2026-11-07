// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_6.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.Builder_6;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets;
internal partial class FilletBuilderBase
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

    internal bool CompleteData(SurfData? data, Surface? surfcoin, SurfaceAdaptorBase? s1, Curve2D? pc1, SurfaceAdaptorBase? s2, Curve2D? pc2, Orientation or, bool on1, bool gd1, bool gd2, bool gf1, bool gf2)
    {
        DsDataStructure dStr = _ds.ChangeDS();
        data.ChangeSurf(dStr.AddSurface(new DsSurface(surfcoin, _tolapp3d)));

        double uFirst = 0.0, uLast = 0.0, vFirst = 0.0, vLast = 0.0;
        surfcoin.ParameterBounds(out uFirst, out uLast, out vFirst, out vLast);
        if (!gd1)
        {
            data.ChangeVertexFirstOnS1().SetPoint(surfcoin.Value(uFirst, vFirst));
        }
        if (!gd2)
        {
            data.ChangeVertexFirstOnS2().SetPoint(surfcoin.Value(uFirst, vLast));
        }
        if (!gf1)
        {
            data.ChangeVertexLastOnS1().SetPoint(surfcoin.Value(uLast, vFirst));
        }
        if (!gf2)
        {
            data.ChangeVertexLastOnS2().SetPoint(surfcoin.Value(uLast, vLast));
        }

        // calculate curves side S1
        Curve3D? crv3d1 = null;
        if (pc1 is not null)
        {
            crv3d1 = surfcoin.VIso(vFirst);
        }
        Point2D pd1 = new(uFirst, vFirst), pf1 = new(uLast, vFirst);
        Line2D lfil1 = new(pd1, new Direction2D(new Vector2D(pd1, pf1)));
        Curve2D? pCurveOnSurf = new LineCurve2D(lfil1);
        Orientation tra1 = Forward, orsurf = or;
        double x = 0.0, y = 0.0, w = 0.5 * (uFirst + uLast);
        Point3D p = new();
        Vector3D du = new(), dv = new();
        Curve2D? c2dtrim = null;
        double tolreached = 1.0e-5;
        if (pc1 is not null)
        {
            CurveAdaptor? hcS1 = new CurveAdaptor(crv3d1);
            c2dtrim = new TrimmedCurve2D(pc1, uFirst, uLast);
            ChFi3d_SameParameter(hcS1, ref c2dtrim, s1, _tolapp3d, out tolreached);
            c2dtrim.Value(w).Coord(out x, out y);
            s1.D1(x, y, out p, out du, out dv);
            Vector3D nf = du.Crossed(dv);
            surfcoin.D1(w, vFirst, out p, out du, out dv);
            Vector3D ns = du.Crossed(dv);
            if (nf.Dot(ns) > 0.0)
            {
                tra1 = Reversed;
            }
            else if (on1)
            {
                orsurf = global::Kerf.Engine.Topology.TopAbs.Reverse(orsurf);
            }
        }
        int index1OfCurve = dStr.AddCurve(new DsCurve(crv3d1, tolreached));
        ref FaceInterference fint1 = ref data.ChangeInterferenceOnS1();
        fint1.SetFirstParameter(uFirst);
        fint1.SetLastParameter(uLast);
        fint1.SetInterference(index1OfCurve, tra1, c2dtrim, pCurveOnSurf);
        // calculate curves side S2
        Curve3D? crv3d2 = null;
        if (pc2 is not null)
        {
            crv3d2 = surfcoin.VIso(vLast);
        }
        Point2D pd2 = new(uFirst, vLast), pf2 = new(uLast, vLast);
        Line2D lfil2 = new(pd2, new Direction2D(new Vector2D(pd2, pf2)));
        pCurveOnSurf = new LineCurve2D(lfil2);
        Orientation tra2 = Forward;
        if (pc2 is not null)
        {
            CurveAdaptor? hcS2 = new CurveAdaptor(crv3d2);
            c2dtrim = new TrimmedCurve2D(pc2, uFirst, uLast);
            ChFi3d_SameParameter(hcS2, ref c2dtrim, s2, _tolapp3d, out tolreached);
            c2dtrim.Value(w).Coord(out x, out y);
            s2.D1(x, y, out p, out du, out dv);
            Vector3D np = du.Crossed(dv);
            surfcoin.D1(w, vLast, out p, out du, out dv);
            Vector3D ns = du.Crossed(dv);
            if (np.Dot(ns) < 0.0)
            {
                tra2 = Reversed;
                if (!on1)
                {
                    orsurf = global::Kerf.Engine.Topology.TopAbs.Reverse(orsurf);
                }
            }
        }
        int index2OfCurve = dStr.AddCurve(new DsCurve(crv3d2, tolreached));
        ref FaceInterference fint2 = ref data.ChangeInterferenceOnS2();
        fint2.SetFirstParameter(uFirst);
        fint2.SetLastParameter(uLast);
        fint2.SetInterference(index2OfCurve, tra2, c2dtrim, pCurveOnSurf);
        data.ChangeOrientation() = orsurf;
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

    internal bool CompleteData(SurfData? data, BlendFunction func, global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, SurfaceAdaptorBase? s2, Orientation or1, bool gd1, bool gd2, bool gf1, bool gf2, bool reversed = false)
    {
        AppFunc? theFunc = new AppFunc(lin, func, _tolapp3d, 1.0e-5);

        int degmax = 20, segmax = 5000;
        AppSurface approx = new(theFunc, lin.Point(1).Parameter(), lin.Point(lin.NbPoints()).Parameter(),
                                          _tolapp3d, 1.0e-5, // tolapp2d, tolerance max
                                          _tolappangle,      // Contact G1
                                          _conti, degmax, segmax);
        if (!approx.IsDone)
        {
            return false;
        }
        return StoreData(data, approx, lin, s1, s2, or1, gd1, gd2, gf1, gf2, reversed);
    }

    //=======================================================================
    // function : CompleteData
    // purpose  : New overload for functions surf/rst
    // jlr le 28/07/97 branchement F(t)
    //=======================================================================

    internal bool CompleteData(SurfData? data, SurfRstFunction func, global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, SurfaceAdaptorBase? s2, Orientation or, bool reversed = false)
    {
        AppFuncRst? theFunc = new AppFuncRst(lin, func, _tolapp3d, 1.0e-5);
        AppSurface approx = new(theFunc, lin.Point(1).Parameter(), lin.Point(lin.NbPoints()).Parameter(),
                                          _tolapp3d, 1.0e-5, // tolapp2d, tolerance max
                                          _tolappangle,      // Contact G1
                                          _conti);
        if (!approx.IsDone)
        {
            return false;
        }

        return StoreData(data, approx, lin, s1, s2, or, false, false, false, false, reversed);
    }

    //=======================================================================
    // function : CompleteData
    // purpose  : New overload for functions rst/rst
    // jlr le 28/07/97 branchement F(t)
    //=======================================================================

    internal bool CompleteData(SurfData? data, RstRstFunction func, global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, SurfaceAdaptorBase? s2, Orientation or)
    {
        AppFuncRstRst? theFunc = new AppFuncRstRst(lin, func, _tolapp3d, 1.0e-5);
        AppSurface approx = new(theFunc, lin.Point(1).Parameter(), lin.Point(lin.NbPoints()).Parameter(),
                                          _tolapp3d, 1.0e-5, // tolapp2d, tolerance max
                                          _tolappangle,      // Contact G1
                                          _conti);
        if (!approx.IsDone)
        {
            return false;
        }

        return StoreData(data, approx, lin, s1, s2, or, false, false, false, false);
    }

    //=======================================================================
    // function : StoreData
    // purpose  : Copy of an approximation result in SurfData.
    //=======================================================================

    internal bool StoreData(SurfData? data, Approx approx, global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, SurfaceAdaptorBase? s2, Orientation or1, bool gd1, bool gd2, bool gf1, bool gf2, bool reversed = false)
    {
        // Small control tools.
        // OCCT keeps a thread_local scratch adaptor; it is loaded before every use and never read across calls,
        // so it is a local here (plan 2.3).
        CurveAdaptor checkcurve = new CurveAdaptor();
        CurveAdaptor chc = checkcurve;
        double tolget3d = 0.0, tolget2d = 0.0, tolaux = 0.0, tolC1 = 0.0, tolcheck = 0.0;
        double tolC2 = 0.0;
        approx.TolReached(out tolget3d, out tolget2d);
        tolaux = approx.TolCurveOnSurf(1);
        tolC1 = tolget3d + tolaux;
        if (s2 is not null)
        {
            tolaux = approx.TolCurveOnSurf(2);
            tolC2 = tolget3d + tolaux;
        }

        DsDataStructure dStr = _ds.ChangeDS();
        // By default parametric space is created using a square surface
        // to be able to parameterize in U by # R*teta // a revoir lbo 29/08/97
        NCollection_Array1<double> ku = approx.SurfUKnots();
        NCollection_Array1<double> kv = approx.SurfVKnots();
        double larg = (kv[kv.Upper()] - kv[kv.Lower()]);
        NCollection_Array1<double> kku = ku; // OCCT casts the constness away: the same array
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(0.0, larg, kku);
        global::Kerf.Geometry.BSplineSurface? surf = new global::Kerf.Geometry.BSplineSurface(approx.SurfPoles(), approx.SurfWeights(), kku, kv,
                                                            approx.SurfUMults(), approx.SurfVMults(),
                                                            approx.UDegree(), approx.VDegree());
        // extension of the surface

        double length1 = 0.0, length2 = 0.0;
        length1 = data.FirstExtensionValue();
        length2 = data.LastExtensionValue();

        BoundedSurface? bndSurf = surf;
        bool ext1 = false, ext2 = false;
        double eps = Max(tolget3d, 2.0 * global::Kerf.Engine.Foundation.Precision.Confusion());
        if (length1 > eps)
        {
            Point3D p11 = new(), p21 = new();
            p11 = surf.Pole(1, 1);
            p21 = surf.Pole(surf.UPoleCount, 1);
            if (p11.DistanceTo(p21) > eps)
            {
                // to avoid extending surface with singular boundary
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref bndSurf, length1, 1, false, false);
                ext1 = true;
            }
        }
        if (length2 > eps)
        {
            Point3D p12 = new(), p22 = new();
            p12 = surf.Pole(1, surf.VPoleCount);
            p22 = surf.Pole(surf.UPoleCount, surf.VPoleCount);
            if (p12.DistanceTo(p22) > eps)
            {
                // to avoid extending surface with singular boundary
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref bndSurf, length2, 1, false, true);
                ext2 = true;
            }
        }
        surf = (bndSurf as global::Kerf.Geometry.BSplineSurface);

        // Correction of surface on extremities
        if (!ext1)
        {
            Point3D p11 = new(), p21 = new();
            p11 = lin.StartPointOnFirst().Value();
            p21 = lin.StartPointOnSecond().Value();
            surf.SetPole(1, 1, p11);
            surf.SetPole(surf.UPoleCount, 1, p21);
        }
        if (!ext2)
        {
            Point3D p12 = new(), p22 = new();
            p12 = lin.EndPointOnFirst().Value();
            p22 = lin.EndPointOnSecond().Value();
            surf.SetPole(1, surf.VPoleCount, p12);
            surf.SetPole(surf.UPoleCount, surf.VPoleCount, p22);
        }

        data.ChangeSurf(dStr.AddSurface(new DsSurface(surf, tolget3d)));

        double uFirst = 0.0, uLast = 0.0, vFirst = 0.0, vLast = 0.0, pppdeb = 0.0, pppfin = 0.0;
        surf.ParameterBounds(out uFirst, out uLast, out vFirst, out vLast);
        EdgeCurve2DAdaptor brc = new();
        EdgeCurveAdaptor cArc = new();
        FaceSurfaceAdaptor? bs1 = (s1 as FaceSurfaceAdaptor);
        FaceSurfaceAdaptor? bs2 = (s2 as FaceSurfaceAdaptor);
        ProjectPointOnCurve2D projector = new();

        double uon1 = uFirst, uon2 = uLast;
        int ion1 = 1, ion2 = 2;
        if (reversed)
        {
            uon1 = uLast;
            uon2 = uFirst;
            ion1 = 2;
            ion2 = 1;
        }

        // The SurfData is filled in what concerns S1,
        Curve3D? crv3d1 = surf.UIso(uon1);
        Point2D pori1 = new(uon1, 0.0);
        Line2D lfil1 = new(pori1, Constants.DY2d());
        Curve2D? pCurveOnSurf = new LineCurve2D(lfil1);
        Curve2D? pCurveOnFace = null;
        pCurveOnFace = new BSplineCurve2D(approx.Curve2dPoles(ion1), approx.Curves2dKnots(), approx.Curves2dMults(),
                                               approx.Curves2dDegree());

        double par1 = pCurveOnFace.FirstParameter;
        double par2 = pCurveOnFace.LastParameter;
        chc.Load(crv3d1, par1, par2);

        if (!ChFi3d_CheckSameParameter(checkcurve, pCurveOnFace, s1, tolC1, out tolcheck))
        {
            tolC1 = tolcheck;
        }
        int index1OfCurve = dStr.AddCurve(new DsCurve(crv3d1, tolC1));

        double uarc = 0.0, utg = 0.0;
        if (gd1)
        {
            Face forwfac = bs1.Face();
            forwfac.Orient(Forward);
            brc.Initialize(data.VertexFirstOnS1().Arc(), forwfac);
            ref CommonPoint v = ref data.ChangeVertexFirstOnS1();
            cArc.Initialize(v.Arc());
            CompParam(brc, pCurveOnFace, ref uarc, ref utg, v.ParameterOnArc(), v.Parameter());
            tolcheck = cArc.Value(uarc).DistanceTo(v.Point());
            v.SetArc(tolC1 + tolcheck, v.Arc(), uarc, v.TransitionOnArc());
            pppdeb = utg;
        }
        else
        {
            pppdeb = vFirst;
        }
        if (gf1)
        {
            Face forwfac = bs1.Face();
            forwfac.Orient(Forward);
            ref CommonPoint v = ref data.ChangeVertexLastOnS1();
            brc.Initialize(v.Arc(), forwfac);
            cArc.Initialize(v.Arc());
            CompParam(brc, pCurveOnFace, ref uarc, ref utg, v.ParameterOnArc(), v.Parameter());
            tolcheck = cArc.Value(uarc).DistanceTo(v.Point());
            v.SetArc(tolC1 + tolcheck, v.Arc(), uarc, v.TransitionOnArc());
            pppfin = utg;
        }
        else
        {
            pppfin = vLast;
        }
        ref FaceInterference fint1 = ref data.ChangeInterferenceOnS1();
        fint1.SetFirstParameter(pppdeb);
        fint1.SetLastParameter(pppfin);
        Orientation traOn1 = default;
        if (reversed)
        {
            traOn1 = ChFi3d_TrsfTrans(lin.TransitionOnS2());
        }
        else
        {
            traOn1 = ChFi3d_TrsfTrans(lin.TransitionOnS1());
        }
        fint1.SetInterference(index1OfCurve, traOn1, pCurveOnFace, pCurveOnSurf);

        // SurfData is filled in what concerns S2,
        Curve3D? crv3d2 = surf.UIso(uon2);
        Point2D pori2 = new(uon2, 0.0);
        Line2D lfil2 = new(pori2, Constants.DY2d());
        pCurveOnSurf = new LineCurve2D(lfil2);
        if (s2 is not null)
        {
            pCurveOnFace = new BSplineCurve2D(approx.Curve2dPoles(ion2), approx.Curves2dKnots(),
                                                   approx.Curves2dMults(), approx.Curves2dDegree());
            chc.Load(crv3d2, par1, par2);
            if (!ChFi3d_CheckSameParameter(checkcurve, pCurveOnFace, s2, tolC2, out tolcheck))
            {
                tolC2 = tolcheck;
            }
        }
        int index2OfCurve = dStr.AddCurve(new DsCurve(crv3d2, tolC2));
        if (gd2)
        {
            Face forwfac = bs2.Face();
            forwfac.Orient(Forward);
            brc.Initialize(data.VertexFirstOnS2().Arc(), forwfac);
            ref CommonPoint v = ref data.ChangeVertexFirstOnS2();
            cArc.Initialize(v.Arc());
            CompParam(brc, pCurveOnFace, ref uarc, ref utg, v.ParameterOnArc(), v.Parameter());
            tolcheck = cArc.Value(uarc).DistanceTo(v.Point());
            v.SetArc(tolC2 + tolcheck, v.Arc(), uarc, v.TransitionOnArc());
            pppdeb = utg;
        }
        else
        {
            pppdeb = vFirst;
        }
        if (gf2)
        {
            Face forwfac = bs2.Face();
            forwfac.Orient(Forward);
            brc.Initialize(data.VertexLastOnS2().Arc(), forwfac);
            ref CommonPoint v = ref data.ChangeVertexLastOnS2();
            cArc.Initialize(v.Arc());
            CompParam(brc, pCurveOnFace, ref uarc, ref utg, v.ParameterOnArc(), v.Parameter());
            tolcheck = cArc.Value(uarc).DistanceTo(v.Point());
            v.SetArc(tolC2 + tolcheck, v.Arc(), uarc, v.TransitionOnArc());
            pppfin = utg;
        }
        else
        {
            pppfin = vLast;
        }
        ref FaceInterference fint2 = ref data.ChangeInterferenceOnS2();
        fint2.SetFirstParameter(pppdeb);
        fint2.SetLastParameter(pppfin);
        if (s2 is not null)
        {
            Orientation traOn2 = default;
            if (reversed)
            {
                traOn2 = ChFi3d_TrsfTrans(lin.TransitionOnS1());
            }
            else
            {
                traOn2 = ChFi3d_TrsfTrans(lin.TransitionOnS2());
            }
            fint2.SetInterference(index2OfCurve, traOn2, pCurveOnFace, pCurveOnSurf);
        }
        else
        {
            Curve2D? bidpc = null;
            fint2.SetInterference(index2OfCurve, Forward, bidpc, pCurveOnSurf);
        }

        // the orientation of the fillet in relation to the faces is evaluated,

        SurfaceAdaptorBase? sref = s1;
        pCurveOnFace = fint1.PCurveOnFace();
        if (reversed)
        {
            sref = s2;
            pCurveOnFace = fint2.PCurveOnFace();
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

        double delta = vLast - vFirst;
        int denom = 2;

        for (;;)
        {
            double deltav = delta / denom;
            double param = vFirst + deltav;
            Point2D puv = pCurveOnFace.Value(param);
            Point3D p = new();
            Vector3D du1 = new(), du2 = new(), dv1 = new(), dv2 = new();

            sref.D1(puv.X, puv.Y, out p, out du1, out dv1);
            du1 = du1.Crossed(dv1);

            if (or1 == global::Kerf.Topology.Orientation.Reversed)
            {
                du1 = du1.Reversed();
            }

            surf.D1(uFirst, param, out p, out du2, out dv2);
            du2 = du2.Crossed(dv2);

            if (du1.Magnitude <= tolget3d || du2.Magnitude <= tolget3d)
            {
                denom++;

                // Kerf: OCCT #1495: with degenerate normals all along, reaching the tolerance takes more steps than an int
                // holds, each with two evaluations. A thousand points without a normal say the same (decision 21106)
                if (Math.Abs(deltav) <= tolget2d || denom > 1000)
                {
                    return false;
                }

                continue;
            }

            if (du1.Dot(du2) > 0)
            {
                data.ChangeOrientation() = Forward;
            }
            else
            {
                data.ChangeOrientation() = global::Kerf.Topology.Orientation.Reversed;
            }

            break;
        }
        //  Modified by skv - Wed Jun  9 17:16:26 2004 OCC5898 End

        if (!gd1 && s1 is not null)
        {
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertex(true, ion1),
                                                        tolC1);
        }
        if (!gf1 && s1 is not null)
        {
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertex(false, ion1),
                                                        tolC1);
        }
        if (!gd2 && s2 is not null)
        {
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertex(true, ion2),
                                                        tolC2);
        }
        if (!gf2 && s2 is not null)
        {
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertex(false, ion2),
                                                        tolC2);
        }
        // Parameters on ElSpine
        int nbp = lin.NbPoints();
        data.FirstSpineParam(lin.Point(1).Parameter());
        data.LastSpineParam(lin.Point(nbp).Parameter());
        return true;
    }

    //=======================================================================
    // function : ComputeData
    // purpose  : Head of the path edge/face for the bypass of obstacle.
    //=======================================================================

    internal bool ComputeData(SurfData? data, ElSpine? hGuide, ref global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, TopolTool? i1, SurfaceAdaptorBase? s2, Curve2DAdaptorBase? pc2, TopolTool? i2, ref bool decroch, SurfRstFunction func, FuncInv fInv, SurfPointFuncInv fInvP, SurfCurvFuncInv fInvC, double pFirst, double maxStep, double fleche, double tolGuide, ref double first, ref double last, math_Vector soldep, bool inside, bool appro, bool forward, bool recP, bool recS, bool recRst)
    {
        SurfRstLineBuilder theWalk = new(s1, i1, s2, pc2, i2);

        data.FirstExtensionValue(0);
        data.LastExtensionValue(0);

        bool reverse = (!forward || inside);
        double spFirst = hGuide.FirstParameter();
        double spLast = hGuide.LastParameter();
        double target = spLast;
        if (reverse)
        {
            target = spFirst;
        }
        double targetsov = target;

        double ms = maxStep;
        int again = 0;
        int nbptmin = 3;// jlr
        int nbpnt = 1;
        // the initial solution is reframed if necessary.
        math_Vector parSol = new(1, 3);
        double newFirst = pFirst;
        if (recP || recS || recRst)
        {
            if (!theWalk.PerformFirstSection(func, fInv, fInvP, fInvC, pFirst, target, soldep, _tolapp3d, _tolapp2d, tolGuide,
                                                                              recRst, recP, recS, ref newFirst, parSol))
            {
                return false;
            }
        }
        else
        {
            parSol.Initialized(soldep); // a copy of the values, as math_Vector::operator=
        }

        while (again < 2)
        {
            theWalk.Perform(func, fInv, fInvP, fInvC, newFirst, last, ms, _tolapp3d, _tolapp2d, tolGuide, parSol, fleche,
                                            appro);

            if (!theWalk.IsDone)
            {
                return false;
            }

            if (reverse)
            {
                if (!theWalk.Complete(func, fInv, fInvP, fInvC, spLast))
                {
                }
            }

            lin = theWalk.Line();
            nbpnt = lin.NbPoints();
            if (nbpnt <= 1 && again == 0)
            {
                again++;
                ms = ms / 50.0;
                target = targetsov;
            }
            else if (nbpnt <= nbptmin && again == 0)
            {
                again++;
                double u1 = lin.Point(1).Parameter();
                double u2 = lin.Point(nbpnt).Parameter();
                ms = (u2 - u1) / (nbptmin + 1.0);
                //      std::cout << " MS : " << MS << " u1 : " << u1 << " u2 : " << u2 << " nbptmin : " <<
                //      nbptmin << std::endl;
                target = targetsov;
            }
            else if (nbpnt <= nbptmin)
            {
                return false;
            }
            else
            {
                again = 2;
            }
        }
        if (forward)
        {
            decroch = theWalk.DecrochEnd();
        }
        else
        {
            decroch = theWalk.DecrochStart();
        }
        last = lin.Point(nbpnt).Parameter();
        first = lin.Point(1).Parameter();
        return true;
    }

    //=======================================================================
    // function : ComputeData
    // purpose  : Heading of the path edge/edge for the bypass of obstacle.
    //=======================================================================

    internal bool ComputeData(SurfData? data, ElSpine? hGuide, ref global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, Curve2DAdaptorBase? pc1, TopolTool? i1, ref bool decroch1, SurfaceAdaptorBase? s2, Curve2DAdaptorBase? pc2, TopolTool? i2, ref bool decroch2, RstRstFunction func, SurfCurvFuncInv fInv1, CurvPointFuncInv fInvP1, SurfCurvFuncInv fInv2, CurvPointFuncInv fInvP2, double pFirst, double maxStep, double fleche, double tolGuide, ref double first, ref double last, math_Vector soldep, bool inside, bool appro, bool forward, bool recP1, bool recRst1, bool recP2, bool recRst2)
    {
        RstRstLineBuilder theWalk = new(s1, pc1, i1, s2, pc2, i2);

        data.FirstExtensionValue(0);
        data.LastExtensionValue(0);

        bool reverse = (!forward || inside);
        double spFirst = hGuide.FirstParameter();
        double spLast = hGuide.LastParameter();
        double target = spLast;
        if (reverse)
        {
            target = spFirst;
        }
        double targetsov = target;

        double ms = maxStep;
        int again = 0;
        int nbptmin = 3;// jlr
        int nbpnt = 0;
        // the initial solution is reframed if necessary.
        math_Vector parSol = new(1, 2);
        double newFirst = pFirst;
        if (recP1 || recRst1 || recP2 || recRst2)
        {
            if (!theWalk.PerformFirstSection(func, fInv1, fInvP1, fInv2, fInvP2, pFirst, target, soldep, _tolapp3d, tolGuide,
                                                                              recRst1, recP1, recRst2, recP2, ref newFirst, parSol))
            {
                return false;
            }
        }
        else
        {
            parSol.Initialized(soldep); // a copy of the values, as math_Vector::operator=
        }

        while (again < 2)
        {
            theWalk.Perform(func, fInv1, fInvP1, fInv2, fInvP2, newFirst, last, ms, _tolapp3d, tolGuide, parSol, fleche,
                                            appro);

            if (!theWalk.IsDone)
            {
                return false;
            }

            if (reverse)
            {
                if (!theWalk.Complete(func, fInv1, fInvP1, fInv2, fInvP2, spLast))
                {
                }
            }

            lin = theWalk.Line();
            nbpnt = lin.NbPoints();
            if (nbpnt <= 1 && again == 0)
            {
                again++;
                ms = ms / 50.0;
                target = targetsov;
            }
            else if (nbpnt <= nbptmin && again == 0)
            {
                again++;
                double u1 = lin.Point(1).Parameter();
                double u2 = lin.Point(nbpnt).Parameter();
                ms = (u2 - u1) / (nbptmin + 1);
                target = targetsov;
            }
            else if (nbpnt <= nbptmin)
            {
                return false;
            }
            else
            {
                again = 2;
            }
        }
        if (forward)
        {
            decroch1 = theWalk.Decroch1End();
            decroch2 = theWalk.Decroch2End();
        }
        else
        {
            decroch1 = theWalk.Decroch1Start();
            decroch2 = theWalk.Decroch2Start();
        }
        last = lin.Point(nbpnt).Parameter();
        first = lin.Point(1).Parameter();
        return true;
    }

    //=======================================================================
    // function : SimulData
    // purpose  : Heading of the path edge/face for the bypass of obstacle in simulation mode.
    //=======================================================================

    internal bool SimulData(SurfData? _unused1, ElSpine? hGuide, ref global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, TopolTool? i1, SurfaceAdaptorBase? s2, Curve2DAdaptorBase? pc2, TopolTool? i2, ref bool decroch, SurfRstFunction func, FuncInv fInv, SurfPointFuncInv fInvP, SurfCurvFuncInv fInvC, double pFirst, double maxStep, double fleche, double tolGuide, ref double first, ref double last, math_Vector soldep, int nbSecMin, bool inside, bool appro, bool forward, bool recP, bool recS, bool recRst)
    {
        SurfRstLineBuilder theWalk = new(s1, i1, s2, pc2, i2);

        bool reverse = (!forward || inside);
        double spFirst = hGuide.FirstParameter();
        double spLast = hGuide.LastParameter();
        double target = spLast;
        if (reverse)
        {
            target = spFirst;
        }
        double targetsov = target;

        double ms = maxStep;
        int again = 0;
        int nbpnt = 0;
        // the starting solution is reframed if needed.
        math_Vector parSol = new(1, 3);
        double newFirst = pFirst;
        if (recP || recS || recRst)
        {
            if (!theWalk.PerformFirstSection(func, fInv, fInvP, fInvC, pFirst, target, soldep, _tolapp3d, _tolapp2d, tolGuide,
                                                                              recRst, recP, recS, ref newFirst, parSol))
            {
                return false;
            }
        }
        else
        {
            parSol.Initialized(soldep); // a copy of the values, as math_Vector::operator=
        }

        while (again < 2)
        {
            theWalk.Perform(func, fInv, fInvP, fInvC, newFirst, last, ms, _tolapp3d, _tolapp2d, tolGuide, parSol, fleche,
                                            appro);
            if (!theWalk.IsDone)
            {
                return false;
            }
            if (reverse)
            {
                if (!theWalk.Complete(func, fInv, fInvP, fInvC, spLast))
                {
                }
            }
            lin = theWalk.Line();
            nbpnt = lin.NbPoints();
            if (nbpnt <= 1 && again == 0)
            {
                again++;
                ms = ms / 50.0;
                target = targetsov;
            }
            else if (nbpnt <= nbSecMin && again == 0)
            {
                again++;
                double u1 = lin.Point(1).Parameter();
                double u2 = lin.Point(nbpnt).Parameter();
                ms = (u2 - u1) / (nbSecMin + 1);
                target = targetsov;
            }
            else if (nbpnt <= nbSecMin)
            {
                return false;
            }
            else
            {
                again = 2;
            }
        }
        if (forward)
        {
            decroch = theWalk.DecrochEnd();
        }
        else
        {
            decroch = theWalk.DecrochStart();
        }
        last = lin.Point(nbpnt).Parameter();
        first = lin.Point(1).Parameter();
        return true;
    }

    //=======================================================================
    // function : SimulData
    // purpose  : Heading of path edge/edge for the bypass
    //           of obstacle in simulation mode.
    //=======================================================================

    internal bool SimulData(SurfData? _unused2, ElSpine? hGuide, ref global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, Curve2DAdaptorBase? pc1, TopolTool? i1, ref bool decroch1, SurfaceAdaptorBase? s2, Curve2DAdaptorBase? pc2, TopolTool? i2, ref bool decroch2, RstRstFunction func, SurfCurvFuncInv fInv1, CurvPointFuncInv fInvP1, SurfCurvFuncInv fInv2, CurvPointFuncInv fInvP2, double pFirst, double maxStep, double fleche, double tolGuide, ref double first, ref double last, math_Vector soldep, int nbSecMin, bool inside, bool appro, bool forward, bool recP1, bool recRst1, bool recP2, bool recRst2)
    {
        RstRstLineBuilder theWalk = new(s1, pc1, i1, s2, pc2, i2);

        bool reverse = (!forward || inside);
        double spFirst = hGuide.FirstParameter();
        double spLast = hGuide.LastParameter();
        double target = spLast;
        if (reverse)
        {
            target = spFirst;
        }
        double targetsov = target;

        double ms = maxStep;
        int again = 0;
        int nbpnt = 0;
        // The initial solution is reframed if necessary.
        math_Vector parSol = new(1, 2);
        double newFirst = pFirst;
        if (recP1 || recRst1 || recP2 || recRst2)
        {
            if (!theWalk.PerformFirstSection(func, fInv1, fInvP1, fInv2, fInvP2, pFirst, target, soldep, _tolapp3d, tolGuide,
                                                                              recRst1, recP1, recRst2, recP2, ref newFirst, parSol))
            {
                return false;
            }
        }
        else
        {
            parSol.Initialized(soldep); // a copy of the values, as math_Vector::operator=
        }

        while (again < 2)
        {
            theWalk.Perform(func, fInv1, fInvP1, fInv2, fInvP2, newFirst, last, ms, _tolapp3d, tolGuide, parSol, fleche,
                                            appro);
            if (!theWalk.IsDone)
            {
                return false;
            }
            if (reverse)
            {
                if (!theWalk.Complete(func, fInv1, fInvP1, fInv2, fInvP2, spLast))
                {
                }
            }
            lin = theWalk.Line();
            nbpnt = lin.NbPoints();
            if (nbpnt <= 1 && again == 0)
            {
                again++;
                ms = ms / 50.0;
                target = targetsov;
            }
            else if (nbpnt <= nbSecMin && again == 0)
            {
                again++;
                double u1 = lin.Point(1).Parameter();
                double u2 = lin.Point(nbpnt).Parameter();
                ms = (u2 - u1) / (nbSecMin + 1);
                target = targetsov;
            }
            else if (nbpnt <= nbSecMin)
            {
                return false;
            }
            else
            {
                again = 2;
            }
        }
        if (forward)
        {
            decroch1 = theWalk.Decroch1End();
            decroch2 = theWalk.Decroch2End();
        }
        else
        {
            decroch1 = theWalk.Decroch1Start();
            decroch2 = theWalk.Decroch2Start();
        }

        last = lin.Point(nbpnt).Parameter();
        first = lin.Point(1).Parameter();
        return true;
    }

    //=======================================================================
    // function : ComputeData
    // purpose  : Construction of elementary fillet by path.
    //
    //=======================================================================

    internal bool ComputeData(SurfData? data, ElSpine? hGuide, Spine? spine, ref global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, TopolTool? i1, SurfaceAdaptorBase? s2, TopolTool? i2, BlendFunction func, FuncInv fInv, double pFirst, double maxStep, double fleche, double tolguide, ref double first, ref double last, bool inside, bool appro, bool forward, math_Vector soldep, ref int intf, ref int intl, ref bool gd1, ref bool gd2, ref bool gf1, ref bool gf2, bool recOnS1 = false, bool recOnS2 = false)
    {
        // Get offset guide if exists
        ElSpine? offsetHGuide = null;
        if (spine is not null && spine.Mode() == ChamfMode.ConstThroatWithPenetrationChamfer)
        {
            NCollection_List<ElSpine> ll = spine.ChangeElSpines();
            NCollection_List<ElSpine> ll_offset = spine.ChangeOffsetElSpines();
            NCollection_List<ElSpine>.Iterator iles = new(ll), ILES_offset = new(ll_offset);
            for (; iles.More(); iles.Next(), ILES_offset.Next())
            {
                ElSpine? hElSpine = iles.Value();
                if (hElSpine == hGuide)
                {
                    offsetHGuide = ILES_offset.Value();
                }
            }
        }

        // The extrensions are created in case of output of two domains
        // directly and not by path ( too hasardous ).
        data.FirstExtensionValue(0);
        data.LastExtensionValue(0);

        // The eventual faces are restored to test the jump of edge.
        Face f1 = new(), f2 = new();
        FaceSurfaceAdaptor? hs = (s1 as FaceSurfaceAdaptor);
        if (hs is not null)
        {
            f1 = hs.Face();
        }
        hs = (s2 as FaceSurfaceAdaptor);
        if (hs is not null)
        {
            f2 = hs.Face();
        }

        // Path framing variables
        double tolGuide = tolguide;
        int nbptmin = 4;

        Walking theWalk = new(s1, s2, i1, i2, hGuide);

        // Start of removal, 2D path controls
        // that qui s'accomodent mal des surfaces a parametrages non homogenes
        // en u et en v are extinguished.
        theWalk.Check2d(false);

        double ms = maxStep;
        int nbpnt = 0;
        double spFirst = hGuide.FirstParameter();
        double spLast = hGuide.LastParameter();

        // When the start point is inside, the path goes first to the left
        // to determine the Last for the periodicals.
        bool reverse = (!forward || inside);
        double target = 0.0;
        if (reverse)
        {
            target = spFirst;
            if (intf == 0)
            {
                target = last;
            }
        }
        else
        {
            target = spLast + Math.Abs(spLast);
            if (intl == 0)
            {
                target = last;
            }
        }

        // In case if the singularity is pre-determined,
        // the path is indicated.
        if (spine is not null)
        {
            if (spine.IsTangencyExtremity(true))
            {
                Vertex v = spine.FirstVertex();
                Edge e = spine.Edges(1);
                double param = spine.FirstParameter();
                BlendPoint bp = new();
                if (CompBlendPoint(v, e, param, f1, f2, ref bp))
                {
                    math_Vector vec = new(1, 4);
                    bp.ParametersOnS1(out vec[1], out vec[2]);
                    bp.ParametersOnS2(out vec[3], out vec[4]);
                    func.Set(param);
                    if (func.IsSolution(vec, _tolapp3d))
                    {
                        theWalk.AddSingularPoint(bp);
                    }
                }
            }
            if (spine.IsTangencyExtremity(false))
            {
                Vertex v = spine.LastVertex();
                Edge e = spine.Edges(spine.EdgeCount);
                double param = spine.LastParameter();
                BlendPoint bp = new();
                if (CompBlendPoint(v, e, param, f1, f2, ref bp))
                {
                    math_Vector vec = new(1, 4);
                    bp.ParametersOnS1(out vec[1], out vec[2]);
                    bp.ParametersOnS2(out vec[3], out vec[4]);
                    func.Set(param);
                    if (func.IsSolution(vec, _tolapp3d))
                    {
                        theWalk.AddSingularPoint(bp);
                    }
                }
            }
        }

        // The starting solution is reframed if necessary.
        //**********************************************//
        math_Vector parSol = new(1, 4);
        double newFirst = pFirst;
        if (recOnS1 || recOnS2)
        {
            if (!theWalk.PerformFirstSection(func, fInv, pFirst, target, soldep, _tolapp3d, tolGuide, recOnS1, recOnS2,
                                                                              ref newFirst, parSol))
            {
                return false;
            }
        }
        else
        {
            parSol.Initialized(soldep); // a copy of the values, as math_Vector::operator=
        }

        // First the valid part is calculate, without caring for the extensions.
        //******************************************************************//
        int again = 0;
        bool tchernobyl = false;
        double u1sov = 0.0, u2sov = 0.0;
        Face bif = new();
        // Max step is relevant, but too great, the vector is required to detect
        // the twists.
        if ((Math.Abs(last - first) <= ms * 5.0)
                && (Math.Abs(last - first) >= 0.01 * Math.Abs(newFirst - target)))
        {
            ms = Math.Abs(last - first) * 0.2;
        }

        while (again < 3)
        {
            // Path.
            if (again == 0 && (ms < 5 * tolGuide))
            {
                ms = 5 * tolGuide;
            }
            else
            {
                if (5 * tolGuide > ms)
                {
                    tolGuide = ms / 5;
                }
            }
            theWalk.Perform(func, fInv, newFirst, target, ms, _tolapp3d, tolGuide, parSol, fleche, appro);
            if (!theWalk.IsDone)
            {
                return false;
            }
            lin = theWalk.Line();
            if (hGuide.IsPeriodic && inside)
            {
                spFirst = lin.Point(1).Parameter();
                spLast = spFirst + hGuide.Period();
                hGuide.FirstParameter(spFirst);
                hGuide.LastParameter(spLast);
                hGuide.SetOrigin(spFirst);
                if (offsetHGuide is not null)
                {
                    offsetHGuide.FirstParameter(spFirst);
                    offsetHGuide.LastParameter(spLast);
                    offsetHGuide.SetOrigin(spFirst);
                }
            }
            bool complmnt = true;
            if (inside)
            {
                complmnt = theWalk.Complete(func, fInv, spLast);
            }
            if (!complmnt)
            {
                return false;
            }

            // The result is controlled using two criterions :
            //- if there is enough points,
            //- if one has gone far enough.
            nbpnt = lin.NbPoints();
            if (nbpnt == 0)
            {
                return false;
            }
            double fpointpar = lin.Point(1).Parameter();
            double lpointpar = lin.Point(nbpnt).Parameter();

            double factor = 1.0 / (nbptmin + 1);
            bool okdeb = (forward && !inside);
            bool okfin = (!forward && !inside);
            if (!okdeb)
            {
                int narc1d = lin.StartPointOnFirst().PointOnRstCount;
                int narc2d = lin.StartPointOnSecond().PointOnRstCount;
                okdeb = (narc1d > 0 || narc2d > 0 || (fpointpar - first) < 10 * tolGuide);
            }
            if (!okfin)
            {
                int narc1d = lin.EndPointOnFirst().PointOnRstCount;
                int narc2d = lin.EndPointOnSecond().PointOnRstCount;
                okfin = (narc1d > 0 || narc2d > 0 || (last - lpointpar) < 10 * tolGuide);
            }
            if (!okdeb || !okfin || nbpnt == 1)
            {
                // It drags, the controls are extended, it is  expected to evaluate a
                // satisfactory maximum step. If it already done, quit.
                if (tchernobyl)
                {
                    return false;
                }
                tchernobyl = true;
                theWalk.Check(false);
                if (nbpnt == 1)
                {
                    ms *= 0.01;
                }
                else
                {
                    ms = (lpointpar - fpointpar) / nbpnt; // EvalStep(Lin);
                }
            }
            else if (nbpnt < nbptmin)
            {
                if (again == 0)
                {
                    u1sov = fpointpar;
                    u2sov = lpointpar;
                    ms = (lpointpar - fpointpar) * factor;
                }
                else if (again == 1)
                {
                    if (Math.Abs(fpointpar - u1sov) >= tolGuide || Math.Abs(lpointpar - u2sov) >= tolGuide)
                    {
                        ms = (lpointpar - fpointpar) * factor;
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

        if (theWalk.TwistOnS1())
        {
            data.TwistOnS1(true);
        }
        if (theWalk.TwistOnS2())
        {
            data.TwistOnS2(true);
        }

        // Here there is a more or less presentable result
        // however it covers a the minimum zone.
        // The extensions are targeted.
        //*****************************//

        gd1 = gd2 = gf1 = gf2 = false;

        bool unseulsuffitdeb = (intf >= 2);
        bool unseulsuffitfin = (intl >= 2);
        bool noproldeb = (intf >= 3);
        bool noprolfin = (intl >= 3);

        double rab = 0.03 * (spLast - spFirst);

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
            narc1 = lin.StartPointOnFirst().PointOnRstCount;
            narc2 = lin.StartPointOnSecond().PointOnRstCount;
            if (narc1 != 0)
            {
                ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS1(),
                                                            _tolapp3d);
                debarc1 = true;
                if (!SearchFace(spine, data.VertexFirstOnS1(), f1, ref bif))
                {
                    // It is checked if there is not an obstacle.
                    debcas1 = true;
                    if (spine is not null)
                    {
                        if (spine.IsPeriodic)
                        {
                            debobst1 = true;
                        }
                        else
                        {
                            debobst1 = IsObst(data.VertexFirstOnS1(), spine.FirstVertex(), _veMap);
                        }
                    }
                }
            }
            if (narc2 != 0)
            {
                ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertexFirstOnS2(),
                                                            _tolapp3d);
                debarc2 = true;
                if (!SearchFace(spine, data.VertexFirstOnS2(), f2, ref bif))
                {
                    // It is checked if it is not an obstacle.
                    debcas2 = true;
                    if (spine is not null)
                    {
                        if (spine.IsPeriodic)
                        {
                            debobst2 = true;
                        }
                        else
                        {
                            debobst2 = IsObst(data.VertexFirstOnS2(), spine.FirstVertex(), _veMap);
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
                theWalk.ClassificationOnS1(!debarc1);
                theWalk.ClassificationOnS2(!debarc2);
                theWalk.Check2d(true); // It should be strict (PMN)
                theWalk.Continu(func, fInv, target);
                theWalk.ClassificationOnS1(true);
                theWalk.ClassificationOnS2(true);
                theWalk.Check2d(false);
                narc1 = lin.StartPointOnFirst().PointOnRstCount;
                narc2 = lin.StartPointOnSecond().PointOnRstCount;
                //  modified by eap Fri Feb  8 11:43:48 2002 ___BEGIN___
                if (!debarc1)
                {
                    if (narc1 == 0)
                    {
                        backwContinueFailed = lin.StartPointOnFirst().ParameterOnGuide() > target;
                    }
                    else
                    {
                        ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS1(),
                                                                    _tolapp3d);
                        debarc1 = true;
                        if (!SearchFace(spine, data.VertexFirstOnS1(), f1, ref bif))
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
                        backwContinueFailed = lin.StartPointOnSecond().ParameterOnGuide() > target;
                    }
                    else
                    {
                        ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true,
                                                                    ref data.ChangeVertexFirstOnS2(), _tolapp3d);
                        debarc2 = true;
                        if (!SearchFace(spine, data.VertexFirstOnS2(), f2, ref bif))
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
                    CommonPoint cp = debarc1 ? data.VertexFirstOnS1() : data.VertexFirstOnS2();
                    if (cp.IsOnArc && bif.IsNull)
                    {
                        backwContinueFailed = false;
                    }
                }
            }
        }
        bool forwContinueFailed = false;
        //  modified by eap Fri Feb  8 11:44:11 2002 ___END___
        if (forward && intl != 0)
        {
            target = spLast;
            narc1 = lin.EndPointOnFirst().PointOnRstCount;
            narc2 = lin.EndPointOnSecond().PointOnRstCount;
            if (narc1 != 0)
            {
                ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS1(),
                                                            _tolapp3d);
                finarc1 = true;
                if (!SearchFace(spine, data.VertexLastOnS1(), f1, ref bif))
                {
                    // It is checked if it is not an obstacle.
                    fincas1 = true;
                    if (spine is not null)
                    {
                        finobst1 = IsObst(data.VertexLastOnS1(), spine.LastVertex(), _veMap);
                    }
                }
            }
            if (narc2 != 0)
            {
                ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS2(),
                                                            _tolapp3d);
                finarc2 = true;
                if (!SearchFace(spine, data.VertexLastOnS2(), f2, ref bif))
                {
                    // It is checked if it is not an obstacle.
                    fincas2 = true;
                    if (spine is not null)
                    {
                        finobst2 = IsObst(data.VertexLastOnS2(), spine.LastVertex(), _veMap);
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
                theWalk.ClassificationOnS1(!finarc1);
                theWalk.ClassificationOnS2(!finarc2);
                theWalk.Check2d(true); // It should be strict (PMN)
                theWalk.Continu(func, fInv, target);
                theWalk.ClassificationOnS1(true);
                theWalk.ClassificationOnS2(true);
                theWalk.Check2d(false);
                narc1 = lin.EndPointOnFirst().PointOnRstCount;
                narc2 = lin.EndPointOnSecond().PointOnRstCount;
                //  modified by eap Fri Feb  8 11:44:57 2002 ___BEGIN___
                if (!finarc1)
                {
                    if (narc1 == 0)
                    {
                        forwContinueFailed = lin.EndPointOnFirst().ParameterOnGuide() < target;
                    }
                    else
                    {
                        ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS1(),
                                                                    _tolapp3d);
                        finarc1 = true;
                        if (!SearchFace(spine, data.VertexLastOnS1(), f1, ref bif))
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
                        forwContinueFailed = lin.EndPointOnSecond().ParameterOnGuide() < target;
                    }
                    else
                    {
                        ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS2(),
                                                                    _tolapp3d);
                        finarc2 = true;
                        if (!SearchFace(spine, data.VertexLastOnS2(), f2, ref bif))
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
                    CommonPoint cp = finarc1 ? data.VertexLastOnS1() : data.VertexLastOnS2();
                    if (cp.IsOnArc && bif.IsNull)
                    {
                        forwContinueFailed = false;
                    }
                }
                //  modified by eap Fri Feb  8 11:45:10 2002 ___END___
            }
        }
        nbpnt = lin.NbPoints();
        first = lin.Point(1).Parameter();
        last = lin.Point(nbpnt).Parameter();

        // ============= INVALIDATION EVENTUELLE =============
        // ------ Preparation des prolongement par plan tangent -----
        if (reverse && intf != 0)
        {
            gd1 = debcas1 /* && !debobst1*/; // skv(occ67)
            gd2 = debcas2 /* && !debobst2*/; // skv(occ67)
            if ((debarc1 ^ debarc2) && !unseulsuffitdeb && (first != spFirst))
            {
                // Case of incomplete path, of course this ends badly :
                // the result is truncated instead of exit.
                double sortie = 0.0;
                int ind = 0;
                if (debarc1)
                {
                    sortie = data.VertexFirstOnS1().Parameter();
                }
                else
                {
                    sortie = data.VertexFirstOnS2().Parameter();
                }
                if (sortie - first > _tolesp)
                {
                    ind = SearchIndex(sortie, lin);
                    if (lin.Point(ind).Parameter() == sortie)
                    {
                        ind--;
                    }
                    if (ind >= 1)
                    {
                        lin.Remove(1, ind);
                        UpdateLine(lin, true);
                    }
                    nbpnt = lin.NbPoints();
                    first = lin.Point(1).Parameter();
                }
            }
            else if ((intf >= 5) && !debarc1 && !debarc2 && (first != spFirst))
            {
                double sortie = (2 * first + last) / 3;
                int ind = 0;
                if (sortie - first > _tolesp)
                {
                    ind = SearchIndex(sortie, lin);
                    if (lin.Point(ind).Parameter() == sortie)
                    {
                        ind--;
                    }
                    if (nbpnt - ind < 3)
                    {
                        ind = nbpnt - 3;
                    }
                    if (ind >= 1)
                    {
                        lin.Remove(1, ind);
                        UpdateLine(lin, true);
                    }
                    nbpnt = lin.NbPoints();
                    first = lin.Point(1).Parameter();
                }
            }
            if (gd1 && gd2)
            {
                target = Min((lin.Point(1).Parameter() - rab), first);
                target = Max(target, spFirst);
                data.FirstExtensionValue(Math.Abs(lin.Point(1).Parameter() - target));
            }
            if (intf != 0 && !unseulsuffitdeb)
            {
                intf = ((gd1 && gd2) //;
                        || backwContinueFailed) ? 1 : 0; // eap
            }
            else if (intf != 0 && unseulsuffitdeb && (intf < 5))
            {
                intf = (gd1 || gd2) ? 1 : 0;
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

        if (forward && intl != 0)
        {
            gf1 = fincas1 /* && !finobst1*/; // skv(occ67)
            gf2 = fincas2 /* && !finobst2*/; // skv(occ67)
            if ((finarc1 ^ finarc2) && !unseulsuffitfin && (last != spLast))
            {
                // Case of incomplete path, of course, this ends badly :
                // the result is truncated instead of exit.
                double sortie = 0.0;
                int ind = 0;
                if (finarc1)
                {
                    sortie = data.VertexLastOnS1().Parameter();
                }
                else
                {
                    sortie = data.VertexLastOnS2().Parameter();
                }
                if (last - sortie > _tolesp)
                {
                    ind = SearchIndex(sortie, lin);
                    if (lin.Point(ind).Parameter() == sortie)
                    {
                        ind++;
                    }
                    if (ind <= nbpnt)
                    {
                        lin.Remove(ind, nbpnt);
                        UpdateLine(lin, false);
                    }
                    nbpnt = lin.NbPoints();
                    last = lin.Point(nbpnt).Parameter();
                }
            }
            else if ((intl >= 5) && !finarc1 && !finarc2 && (last != spLast))
            {
                // The same in case when the entire "Lin" is an extension
                double sortie = (first + 2 * last) / 3;
                int ind = 0;
                if (last - sortie > _tolesp)
                {
                    ind = SearchIndex(sortie, lin);
                    if (lin.Point(ind).Parameter() == sortie)
                    {
                        ind++;
                    }
                    if (ind < 3)
                    {
                        ind = 3;
                    }
                    if (ind <= nbpnt)
                    {
                        lin.Remove(ind, nbpnt);
                        UpdateLine(lin, false);
                    }
                    nbpnt = lin.NbPoints();
                    last = lin.Point(nbpnt).Parameter();
                }
            }
            if (gf1 && gf2)
            {
                target = Max((lin.Point(nbpnt).Parameter() + rab), last);
                target = Min(target, spLast);
                data.LastExtensionValue(Math.Abs(target - lin.Point(nbpnt).Parameter()));
            }

            if (intl != 0 && !unseulsuffitfin)
            {
                intl = ((gf1 && gf2) //;
                        || forwContinueFailed) ? 1 : 0; // eap
            }
            else if (intl != 0 && unseulsuffitfin && (intl < 5))
            {
                intl = (gf1 || gf2) ? 1 : 0; // It is checked if there is no new face.
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

    internal bool SimulData(SurfData? _unused3, ElSpine? hGuide, ElSpine? additionalHGuide, ref global::Kerf.Engine.Fillets.Blend.Line? lin, SurfaceAdaptorBase? s1, TopolTool? i1, SurfaceAdaptorBase? s2, TopolTool? i2, BlendFunction func, FuncInv fInv, double pFirst, double maxStep, double fleche, double tolguide, ref double first, ref double last, bool inside, bool appro, bool forward, math_Vector soldep, int nbSecMin, bool recOnS1 = false, bool recOnS2 = false)
    {
        Walking theWalk = new(s1, s2, i1, i2, hGuide);
        theWalk.Check2d(false);

        double ms = maxStep;
        double tolGuide = tolguide;
        int nbpnt = 0;
        double spFirst = hGuide.FirstParameter();
        double spLast = hGuide.LastParameter();
        bool reverse = (!forward || inside);
        double target = 0.0;
        if (reverse)
        {
            target = spFirst;
        }
        else
        {
            target = spLast;
        }

        double targetsov = target;
        double u1sov = 0.0, u2sov = 0.0;
        // on recadre la solution de depart a la demande.
        math_Vector parSol = new(1, 4);
        double newFirst = pFirst;
        if (recOnS1 || recOnS2)
        {
            if (!theWalk.PerformFirstSection(func, fInv, pFirst, target, soldep, _tolapp3d, tolGuide, recOnS1, recOnS2,
                                                                              ref newFirst, parSol))
            {
                return false;
            }
        }
        else
        {
            parSol.Initialized(soldep); // a copy of the values, as math_Vector::operator=
        }
        int again = 0;
        while (again < 3)
        {
            // When the start point is inside, the path goes first to the left
            // to determine the Last for the periodicals.
            if (again == 0 && (ms < 5 * tolGuide))
            {
                ms = 5 * tolGuide;
            }
            else
            {
                if (5 * tolGuide > ms)
                {
                    tolGuide = ms / 5;
                }
            }

            theWalk.Perform(func, fInv, newFirst, target, ms, _tolapp3d, tolGuide, parSol, fleche, appro);

            if (!theWalk.IsDone)
            {
                return false;
            }
            lin = theWalk.Line();
            if (reverse)
            {
                if (hGuide.IsPeriodic)
                {
                    spFirst = lin.Point(1).Parameter();
                    spLast = spFirst + hGuide.Period();
                    hGuide.FirstParameter(spFirst);
                    hGuide.LastParameter(spLast);
                    if (additionalHGuide is not null)
                    {
                        additionalHGuide.FirstParameter(spFirst);
                        additionalHGuide.LastParameter(spLast);
                    }
                }
                bool complmnt = true;
                if (inside)
                {
                    complmnt = theWalk.Complete(func, fInv, spLast);
                }
                if (!complmnt)
                {
                    return false;
                }
            }
            nbpnt = lin.NbPoints();
            double factor = 1.0 / (nbSecMin + 1);
            if (nbpnt == 0)
            {
                return false;
            }
            else if (nbpnt == 1 && again == 0)
            {
                again++;
                ms *= 0.01;
                target = targetsov;
                u1sov = u2sov = lin.Point(1).Parameter();
            }
            else if (nbpnt < nbSecMin && again == 0)
            {
                again++;
                double u1 = u1sov = lin.Point(1).Parameter();
                double u2 = u2sov = lin.Point(nbpnt).Parameter();
                ms = (u2 - u1) * factor;
                target = targetsov;
            }
            else if (nbpnt < nbSecMin && again == 1)
            {
                double u1 = lin.Point(1).Parameter();
                double u2 = lin.Point(nbpnt).Parameter();
                if (Math.Abs(u1 - u1sov) >= tolGuide || Math.Abs(u2 - u2sov) >= tolGuide)
                {
                    again++;
                    ms /= 100;
                    target = targetsov;
                }
                else
                {
                    return false;
                }
            }
            else if (nbpnt < nbSecMin)
            {
                return false;
            }
            else
            {
                again = 3;
            }
        }
        first = lin.Point(1).Parameter();
        last = lin.Point(nbpnt).Parameter();
        return true;
    }

}

internal static class Builder_6
{
    internal static int SearchIndex(double value, global::Kerf.Engine.Fillets.Blend.Line? lin)
    {
        int nbPnt = lin.NbPoints(), ind;

        for (ind = 1; (ind < nbPnt) && (lin.Point(ind).Parameter() < value);)
        {
            ind++;
        }
        return ind;
    }

    internal static int Nbedconnex(NCollection_List<Shape> l)
    {
        int nb = 0, i = 0;
        NCollection_List<Shape>.Iterator it1 = new(l);
        for (; it1.More(); it1.Next(), i++)
        {
            Shape curs = it1.Value();
            bool dejavu = false;
            NCollection_List<Shape>.Iterator it2 = new(l);
            for (int j = 0; j < i && it2.More(); j++, it2.Next())
            {
                if (curs.IsSame(it2.Value()))
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

    internal static bool IsVois(in Edge e, in Vertex vref, Map veMap, HashSet<Shape> done, int prof, int profmax)
    {
        if (prof > profmax)
        {
            return false;
        }
        if (done.Contains(e))
        {
            return false;
        }
        Vertex v1 = new(), v2 = new();
        Exploration.Vertices(e, out v1, out v2);
        if (vref.IsSame(v1) || vref.IsSame(v2))
        {
            return true;
        }
        done.Add(e);
        NCollection_List<Shape> l1 = veMap[v1];
        int i1 = Nbedconnex(l1);
        NCollection_List<Shape>.Iterator it1 = new(l1);
        for (; it1.More(); it1.Next())
        {
            Edge curE = ShapeCasts.Edge(it1.Value());
            if (i1 <= 2)
            {
                if (IsVois(curE, vref, veMap, done, prof, profmax))
                {
                    return true;
                }
            }
            else if (IsVois(curE, vref, veMap, done, prof + 1, profmax))
            {
                return true;
            }
        }
        NCollection_List<Shape> l2 = veMap[v2];
        NCollection_List<Shape>.Iterator it2 = new(l2);
        for (; it2.More(); it2.Next())
        {
            Edge curE = ShapeCasts.Edge(it2.Value());
            if (i1 <= 2)
            {
                if (IsVois(curE, vref, veMap, done, prof, profmax))
                {
                    return true;
                }
            }
            else if (IsVois(curE, vref, veMap, done, prof + 1, profmax))
            {
                return true;
            }
        }
        return false;
    }

    internal static bool IsObst(in CommonPoint cp, in Vertex vref, Map veMap)
    {
        if (!cp.IsOnArc)
        {
            return false;
        }
        Edge e = cp.Arc();
        HashSet<Shape> done = new(ShapeHasher.Instance);
        int prof = 4;
        return !IsVois(e, vref, veMap, done, 0, prof);
    }

    internal static void CompParam(CurveAdaptor2D carc, Curve2D? ctg, ref double parc, ref double ptg, double prefarc, double preftg)
    {
        bool found = false;
        //(1) It is checked if the provided parameters are good
        //    if pcurves have the same parameters as the spine.
        Point2D point = carc.Value(prefarc);
        double distini = point.DistanceTo(ctg.Value(preftg));
        if (distini <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            parc = prefarc;
            ptg = preftg;
            found = true;
        }
        else
        {
            //(2) Intersection
            IntersectionPoint2D int2d = new();
            GInter intersection = new();
            int nbpt = 0, nbseg = 0;
            intersection.Perform(new CurveAdaptor2D(ctg), carc, global::Kerf.Engine.Foundation.Precision.PIntersection(), global::Kerf.Engine.Foundation.Precision.PIntersection());

            double dist = global::Kerf.Engine.Foundation.Precision.Infinite(), p1, p2;
            if (intersection.IsDone)
            {
                if (!intersection.IsEmpty)
                {
                    nbseg = intersection.SegmentCount;
                    if (nbseg > 0)
                    {
                    }
                    nbpt = intersection.PointCount;
                    for (int i = 1; i <= nbpt; i++)
                    {
                        int2d = intersection.Point(i);
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
            ProjectPointOnCurve2D projector = new(point, ctg);

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

    internal static bool CompBlendPoint(in Vertex v, in Edge e, double w, in Face f1, in Face f2, ref BlendPoint bp)
    {
        Point2D p1 = new(), p2 = new();
        Point3D p3d = new();
        double param = 0.0, f = 0.0, l = 0.0;
        Curve2D? pc = null;

        p3d = Tool.Pnt(v);
        param = Tool.Parameter(v, e, f1);
        pc = Tool.CurveOnSurface(e, f1, out f, out l);
        if (pc is null)
        {
            return false;
        }
        p1 = pc.Value(param);
        param = Tool.Parameter(v, e, f2);
        pc = Tool.CurveOnSurface(e, f2, out f, out l);
        if (pc is null)
        {
            return false;
        }
        p2 = pc.Value(param);
        bp.SetValue(p3d, p3d, w, p1.X, p1.Y, p2.X, p2.Y);
        return true;
    }

    internal static void UpdateLine(global::Kerf.Engine.Fillets.Blend.Line? line, bool isfirst)
    {
        double tguide = 0.0, u = 0.0, v = 0.0;
        if (isfirst)
        {
            BlendPoint bp = line.Point(1);
            tguide = bp.Parameter();
            if (line.StartPointOnFirst().ParameterOnGuide() < tguide)
            {
                Extremity be = new();
                bp.ParametersOnS1(out u, out v);
                be.SetValue(bp.PointOnS1(), u, v, global::Kerf.Engine.Foundation.Precision.Confusion());
                line.SetStartPoints(be, line.StartPointOnSecond());
            }
            if (line.StartPointOnSecond().ParameterOnGuide() < tguide)
            {
                Extremity be = new();
                bp.ParametersOnS2(out u, out v);
                be.SetValue(bp.PointOnS2(), u, v, global::Kerf.Engine.Foundation.Precision.Confusion());
                line.SetStartPoints(line.StartPointOnFirst(), be);
            }
        }
        else
        {
            BlendPoint bp = line.Point(line.NbPoints());
            tguide = bp.Parameter();
            if (line.EndPointOnFirst().ParameterOnGuide() > tguide)
            {
                Extremity be = new();
                bp.ParametersOnS1(out u, out v);
                be.SetValue(bp.PointOnS1(), u, v, global::Kerf.Engine.Foundation.Precision.Confusion());
                line.SetEndPoints(be, line.EndPointOnSecond());
            }
            if (line.EndPointOnSecond().ParameterOnGuide() > tguide)
            {
                Extremity be = new();
                bp.ParametersOnS2(out u, out v);
                be.SetValue(bp.PointOnS2(), u, v, global::Kerf.Engine.Foundation.Precision.Confusion());
                line.SetEndPoints(line.EndPointOnFirst(), be);
            }
        }
    }


}
