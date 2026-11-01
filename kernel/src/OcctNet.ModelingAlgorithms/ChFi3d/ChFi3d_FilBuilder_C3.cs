// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder_C3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_FilBuilder_C3;
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



    //=================================================================================================

    //=================================================================================================

    //=======================================================================
    // function : ToricCorner
    // purpose  : Test if this is a particular case of a torus corner
    //           (or spherical limited by isos).
    //=======================================================================

    //=======================================================================
    // function : PerformThreeCorner
    // purpose  : Calculate fillet on a top with three edges
    //           incident carrying each edge.
    //=======================================================================

    internal override void PerformThreeCorner(int Jndex)
    {


        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        TopoDS_Vertex Vtx = myVDataMap.FindKey(Jndex);
        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        int[] Index = new int[3];
        int pivot = 0, deb = 0, fin = 0, ii = 0, jj = 0, kk = 0;
        // double R = 0.;
        bool pivdif = true;
        bool c1pointu = false;
        bool c1toric = false;
        bool c1spheric = false;
        ChFiDS_Stripe?[] CD = new ChFiDS_Stripe?[3];
        TopoDS_Face[] face = [new(), new(), new()];
        int[,] jf = new int[3, 3];
        bool[] sameside = new bool[3], oksea = new bool[3];
        for (int g = 0; g <= 2; g++)
        {
            oksea[g] = false;
        }
        int[,] i = new int[3, 3];
        int[] sens = new int[3];
        double[,] p = new double[3, 3];

        bool filling = false;

        for (It.Initialize(myVDataMap[Jndex]), ii = 0; It.More() && ii < 3; It.Next(), ii++)
        {
            Index[ii] = ChFi3d_IndexOfSurfData(Vtx, It.Value(), out sens[ii]);
            CD[ii] = It.Value();
        }
        // It is checked if one of CD is not present twice in which
        // case it is necessary to modify the return of IndexOfSurfData
        // that takes the first solution.
        if (CD[0] == CD[1])
        {
            Index[1] = CD[1].SetOfSurfData().Length();
            sens[1] = -1;
        }
        else if (CD[1] == CD[2])
        {
            Index[2] = CD[2].SetOfSurfData().Length();
            sens[2] = -1;
        }
        else if (CD[0] == CD[2])
        {
            Index[2] = CD[2].SetOfSurfData().Length();
            sens[2] = -1;
        }
        oksea[2] = SearchFD(DStr, CD[0], CD[1], sens[0], sens[1], ref i[0, 1], ref i[1, 0], ref p[0, 1], ref p[1, 0], Index[0],
                            Index[1], ref face[2], ref sameside[2], ref jf[0, 1], ref jf[1, 0]);
        oksea[1] = SearchFD(DStr, CD[0], CD[2], sens[0], sens[2], ref i[0, 2], ref i[2, 0], ref p[0, 2], ref p[2, 0], Index[0],
                            Index[2], ref face[1], ref sameside[1], ref jf[0, 2], ref jf[2, 0]);
        oksea[0] = SearchFD(DStr, CD[1], CD[2], sens[1], sens[2], ref i[1, 2], ref i[2, 1], ref p[1, 2], ref p[2, 1], Index[1],
                            Index[2], ref face[0], ref sameside[0], ref jf[1, 2], ref jf[2, 1]);
        //
        // Analyze concavities of 3 fillets :
        //        - 2 concavities identic and 1 inverted.
        //        - 3 concavities identic
        //
        if (oksea[2] && oksea[1] && !sameside[2] && !sameside[1])
        {
            pivot = 0;
            deb = 1;
            fin = 2;
        }
        else if (oksea[2] && oksea[0] && !sameside[2] && !sameside[0])
        {
            pivot = 1;
            deb = 2;
            fin = 0;
        }
        else if (oksea[1] && oksea[0] && !sameside[1] && !sameside[0])
        {
            pivot = 2;
            deb = 0;
            fin = 1;
        }
        else if (oksea[0] && oksea[1] && oksea[2])
        {
            // 3 concavities identic.
            pivot = SearchPivot(sens, p, tol2d);
            if (pivot < 0)
            {
                PerformMoreThreeCorner(Jndex, 3);
                return;
            }
            else
            {
                deb = (pivot + 1) % 3;
                fin = (pivot + 2) % 3;
            }
            pivdif = false;
            if (Math.Abs(p[0, 1] - p[0, 2]) <= tol2d && Math.Abs(p[1, 0] - p[1, 2]) <= tol2d
                    && Math.Abs(p[2, 0] - p[2, 1]) <= tol2d)
            {
                c1pointu = true;
            }
        }
        else
        {
            PerformMoreThreeCorner(Jndex, 3);
            return;
        }
        int ifacdeb = 0, ifacfin = 0;
        ifacdeb = CD[deb].ChangeSetOfSurfData().Value(i[deb, pivot]).Index(3 - jf[deb, pivot]);
        ifacfin = CD[fin].ChangeSetOfSurfData().Value(i[fin, pivot]).Index(3 - jf[fin, pivot]);
        if (ifacfin != ifacdeb)
        {
            PerformMoreThreeCorner(Jndex, 3);
            return;
        }
        if (i[pivot, deb] != i[pivot, fin])
        {
            PerformMoreThreeCorner(Jndex, 3);
            return;
        }

        double Rdeb = 0.0, Rfin = 0.0, Rdp = 0.0, Rfp = 0.0;
        gp_Pnt Pdeb = new(), Pfin = new(), Pdp = new(), Pfp = new();
        gp_Vec Vdeb = new(), Vfin = new(), Vdp = new(), Vfp = new();
        if (c1pointu)
        {
            gp_Pnt pbid = new();
            gp_Vec[] qv = [new(), new(), new()];
            double[] qr = new double[3];
            for (ii = 0; ii <= 2; ii++)
            {
                jj = (ii + 1) % 3;
                kk = (ii + 2) % 3;
                ChFi3d_ExtrSpineCarac(DStr, CD[jj], i[jj, ii], p[jj, ii], 1, sens[jj], out pbid, out qv[jj], out qr[jj]);
            }
            for (ii = 0; ii <= 2 && !c1toric; ii++)
            {
                jj = (ii + 1) % 3;
                kk = (ii + 2) % 3;
                if (ToricCorner(face[ii], qr[jj], qr[kk], qv[ii]))
                {
                    c1toric = true;
                    pivot = ii;
                    deb = jj;
                    fin = kk;
                }
            }
            if (!c1toric)
            {
                c1spheric = (Math.Abs(qr[0] - qr[1]) < tolapp3d && Math.Abs(qr[0] - qr[2]) < tolapp3d);
            }
        }

        //  Previously to avoid loops the points were always located
        //  inside, which could impede the construction of the
        //  guideline of the corner which now is a circle.
        //  int jjjd = jf[deb, fin], jjjf = jf[fin, deb];
        //  if (pivdif) jjjd = jf[deb, pivot], jjjf = jf[fin, pivot];
        int jjjd = jf[deb, pivot], jjjf = jf[fin, pivot];
        ChFi3d_ExtrSpineCarac(DStr, CD[deb], i[deb, pivot], p[deb, pivot], jjjd, sens[deb], out Pdeb, out Vdeb, out Rdeb);
        ChFi3d_ExtrSpineCarac(DStr, CD[fin], i[fin, pivot], p[fin, pivot], jjjf, sens[fin], out Pfin, out Vfin, out Rfin);
        ChFi3d_ExtrSpineCarac(DStr, CD[pivot], i[pivot, deb], p[pivot, deb], 0, sens[pivot], out Pdp, out Vdp, out Rdp);
        ChFi3d_ExtrSpineCarac(DStr, CD[pivot], i[pivot, fin], p[pivot, fin], 0, sens[pivot], out Pfp, out Vfp, out Rfp);
        // in cas of allsame it is checked that points on the face are not
        // too close, which can stop the work.
        if (!pivdif)
        {
            gp_Pnt ptestdeb = new(), ptestfin = new();
            gp_Vec bidvec = new();
            double bidr = 0.0;
            ChFi3d_ExtrSpineCarac(DStr, CD[deb], i[deb, pivot], p[deb, pivot], jf[deb, fin], sens[deb], out ptestdeb,
                                                        out bidvec, out bidr);
            ChFi3d_ExtrSpineCarac(DStr, CD[fin], i[fin, pivot], p[fin, pivot], jf[fin, deb], sens[fin], out ptestfin,
                                                        out bidvec, out bidr);
            double distest = ptestdeb.Distance(ptestfin);
            if (distest < (Rdp + Rfp) * 0.05)
            {
                filling = true;
            }
            if (distest < (Rdp + Rfp) * 0.005)
            {
                c1pointu = true;
            }
        }

        if (!c1pointu)
        {
            if (!pivdif)
            {
                c1pointu = (Math.Abs(p[deb, pivot] - p[deb, fin]) <= tol2d
                                        && Math.Abs(p[fin, pivot] - p[fin, deb]) <= tol2d);
            }
            if (Math.Abs(p[pivot, deb] - p[pivot, fin]) <= tol2d)
            {
                c1toric = ToricCorner(face[pivot], Rdeb, Rfin, Vdp);
            }
        }
        // there is a pivot, the start and the end CD (finally !?!) :
        // -------------------------------------------------------------
        // the criterions determining if the corner is a torus or a sphere
        // are based only on the configuration of sections at end and the
        // nature of faces, it is necessary to make tests to
        // determine if a more detailed analysis of incident fillets
        // is necessare to provide tangency between them and
        // the corner (in particular in the case with variable radius).

        // OCCT holds references to the sequences' handles, never assigned through: plain locals
        ChFiDS_SurfData? fddeb = CD[deb].ChangeSetOfSurfData().ChangeValue(i[deb, pivot]);
        ChFiDS_SurfData? fdfin = CD[fin].ChangeSetOfSurfData().ChangeValue(i[fin, pivot]);
        ChFiDS_SurfData? fdpiv = CD[pivot].ChangeSetOfSurfData().ChangeValue(i[pivot, deb]);

        // HSurfaces and other suitable tools are constructed.
        // ----------------------------------------------------------

        TopAbs_Orientation OFac = face[pivot].Orientation();
        BRepAdaptor_Surface? Fac = new BRepAdaptor_Surface(face[pivot]);
        gp_Pnt2d ppp1 = new(), ppp2 = new();
        ChFiDS_FaceInterference bid1 = 
            CD[pivot].SetOfSurfData().Value(i[pivot, deb]).InterferenceOnS1();
        ppp1 = bid1.PCurveOnSurf().Value(bid1.FirstParameter());
        ChFiDS_FaceInterference bid2 = 
            CD[pivot].SetOfSurfData().Value(i[pivot, deb]).InterferenceOnS2();
        ppp2 = bid2.PCurveOnSurf().Value(bid2.LastParameter());
        double uu1 = ppp1.X(), uu2 = ppp2.X(), vv1 = ppp1.Y(), vv2 = ppp2.Y();
        GeomAdaptor_Surface gasurf = new((DStr.Surface(CD[pivot].SetOfSurfData().Value(i[pivot, deb]).Surf())).Surface(),
                                         uu1, uu2, vv1, vv2);
        GeomAbs_SurfaceType styp = gasurf.GetType();
        if (styp == GeomAbs_Cylinder)
        {
            double h = vv2 - vv1;
            vv1 -= 0.5 * h;
            vv2 += 0.5 * h;
            gasurf.Load((DStr.Surface(CD[pivot].SetOfSurfData().Value(i[pivot, deb]).Surf())).Surface(), uu1, uu2, vv1, vv2);
        }
        else if (styp == GeomAbs_Torus)
        {
            double h = uu2 - uu1;
            uu1 -= 0.1 * h;
            uu2 += 0.1 * h;
            gasurf.Load((DStr.Surface(CD[pivot].SetOfSurfData().Value(i[pivot, deb]).Surf())).Surface(), uu1, uu2, vv1, vv2);
        }
        else if (styp == GeomAbs_BezierSurface || styp == GeomAbs_BSplineSurface)
        {
            gasurf.Load((DStr.Surface(CD[pivot].SetOfSurfData().Value(i[pivot, deb]).Surf())).Surface());
        }

        GeomAdaptor_Surface? Surf = gasurf; // OCCT copies gasurf into a handle; gasurf is not used again
        //  occ::handle<BRepTopAdaptor_TopolTool> IFac = new BRepTopAdaptor_TopolTool(Fac);
        // Try to not classify on the face for cases of reentering fillets which naturally depass
        // the border.
        GeomAdaptor_Surface? bidsurf = new GeomAdaptor_Surface(Fac.GeomSurfaceOriginal());
        Adaptor3d_TopolTool? IFac = new Adaptor3d_TopolTool(bidsurf);
        // end of the attempt.
        Adaptor3d_TopolTool? ISurf = new Adaptor3d_TopolTool(Surf);
        ChFiDS_Stripe? corner = new ChFiDS_Stripe();
        ref NCollection_HSequence<ChFiDS_SurfData>? cornerset = ref 
            corner.ChangeSetOfSurfData();
        cornerset = new NCollection_HSequence<ChFiDS_SurfData>();
        ChFiDS_SurfData? coin = new ChFiDS_SurfData();
        cornerset.Append(coin);
        TopAbs_Orientation o1 = default, o2 = default, os1 = default, os2 = default, oo1 = default, oo2 = default;
        int choix = CD[deb].Choix();
        o1 = face[pivot].Orientation();
        o2 = fdpiv.Orientation();
        oo1 = o1;
        oo2 = o2;
        os1 = CD[deb].OrientationOnFace1();
        os2 = CD[deb].OrientationOnFace2();
        if (jf[deb, fin] == 1)
        {
            choix = ChFi3d.NextSide(ref o1, ref o2, os1, os2, choix);
            if (sens[deb] == 1)
            {
                if (choix % 2 == 1)
                {
                    choix++;
                }
                else
                {
                    choix--;
                }
            }
        }
        else
        {
            choix = ChFi3d.NextSide(ref o2, ref o1, os1, os2, -choix);
            if (sens[deb] == -1)
            {
                if (choix % 2 == 1)
                {
                    choix++;
                }
                else
                {
                    choix--;
                }
            }
        }

        gp_Pnt2d pfac1 = new(), pfac2 = new(), psurf1 = new(), psurf2 = new();
        gp_Vec2d vfac1 = new(), vfac2 = new();
        CD[deb]
            .SetOfSurfData()
            .Value(i[deb, pivot])
            .Interference(jf[deb, fin])
            .PCurveOnFace()
            .D1(p[deb, pivot], out pfac1, out vfac1);
        CD[fin]
            .SetOfSurfData()
            .Value(i[fin, pivot])
            .Interference(jf[fin, deb])
            .PCurveOnFace()
            .D1(p[fin, pivot], out pfac2, out vfac2);
        psurf1 = CD[pivot]
                              .SetOfSurfData()
                              .Value(i[pivot, deb])
                              .Interference(jf[pivot, deb])
                              .PCurveOnSurf()
                              .Value(p[pivot, deb]);
        psurf2 = CD[pivot]
                              .SetOfSurfData()
                              .Value(i[pivot, fin])
                              .Interference(jf[pivot, fin])
                              .PCurveOnSurf()
                              .Value(p[pivot, fin]);

        done = false;


        if (c1toric)
        {


            // Direct Construction.
            // ---------------------
            done = ChFiKPart_ComputeData.ComputeCorner(DStr, coin, Fac, Surf, oo1, oo2, o1, o2, Rdeb, Rdp, pfac1, pfac2,
                                                                                                    psurf1, psurf2);

        }
        else if (c1spheric)
        {


            done = ChFiKPart_ComputeData.ComputeCorner(DStr, coin, Fac, Surf, oo1, oo2, o1, o2, Rdp, pfac1, psurf1, psurf2);

        }
        else if (c1pointu)
        {
            filling = true;
        }
        if (!done)
        {
            if (!filling)
            {


                // Calculate a guideline,
                //------------------------------
                // Numerous problems with loops and half-turns connected to
                // the curvature of the guideline !!!!!!
                // FOR CIRCLE.
                // If the nature of guideline is changed it is necessary to
                // reset points Pdeb and Pfin at the inside (see the
                // comments about it in the calculation of Pdeb and Pfin).

                double radpondere = (Rdp + Rfp) / 2.0;
                double locfleche = fleche;

                double WFirst = 0.0, WLast = 0.0;
                Geom_Curve? spinecoin = 
                    ChFi3d_CircularSpine(ref WFirst, ref WLast, Pdeb, Vdeb, Pfin, Vfin, radpondere);
                if (spinecoin is null)
                {
                    // This is a bad case when the intersection of
                    // section planes is done out of the sector.
                    spinecoin = ChFi3d_Spine(Pdeb, ref Vdeb, Pfin, ref Vfin, radpondere);
                    WFirst = 0.0;
                    WLast = 1.0;
                }
                else
                {
                    locfleche = radpondere * (WLast - WFirst) * fleche;
                }
                double pasmax = (WLast - WFirst) * 0.05;
                ChFiDS_ElSpine? cornerspine = new ChFiDS_ElSpine();
                cornerspine.SetCurve(spinecoin);
                cornerspine.FirstParameter(WFirst - pasmax);
                cornerspine.LastParameter(WLast + pasmax);
                // Just to confuse Compute that should not require this
                // in this exact case ...
                ChFiDS_Spine? NullSpine = null;
                // The fillet is calculated - from beginning to end
                //                       - from the face to the surface
                //
                math_Vector Soldep = new(1, 4);
                Soldep[1] = pfac1.X();
                Soldep[2] = pfac1.Y();
                Soldep[3] = psurf1.X();
                Soldep[4] = psurf1.Y();

                bool Gd1 = false, Gd2 = false, Gf1 = false, Gf2 = false;
                BRepBlend_Line? lin = null;
                double ffi = WFirst, lla = WLast + pasmax;

                if (Math.Abs(Rdeb - Rfin) <= tolapp3d)
                {

                    BlendFunc_ConstRad func = new(Fac, Surf, cornerspine);
                    BlendFunc_ConstRadInv finv = new(Fac, Surf, cornerspine);
                    func.Set(Rdeb, choix);
                    func.Set(myShape);
                    finv.Set(Rdeb, choix);
                    double TolGuide = cornerspine.Resolution(tolapp3d);

                    int intf = 3, intl = 3;
                    done = ComputeData(coin, cornerspine, NullSpine, ref lin, Fac, IFac, Surf, ISurf, func, finv, ffi, pasmax, locfleche,
                                       TolGuide, ref ffi, ref lla, false, false, true, Soldep, ref intf, ref intl, ref Gd1,
                                       ref Gd2, ref Gf1, ref Gf2, false, true);
                    if (done && Gf2)
                    {
                        done = CompleteData(coin, func, lin, Fac, Surf, OFac, Gd1, false, Gf1, false);
                        filling = !done;
                    }
                    else
                    {
                        filling = true;
                    }
                }
                else
                {
                    Law_S? law = new Law_S();
                    law.Set(WFirst, Rdeb, WLast, Rfin);
                    BlendFunc_EvolRad func = new(Fac, Surf, cornerspine, law);
                    BlendFunc_EvolRadInv finv = new(Fac, Surf, cornerspine, law);
                    func.Set(choix);
                    func.Set(myShape);
                    finv.Set(choix);
                    double TolGuide = cornerspine.Resolution(tolapp3d);
                    int intf = 3, intl = 3;
                    done = ComputeData(coin, cornerspine, NullSpine, ref lin, Fac, IFac, Surf, ISurf, func, finv, ffi, pasmax, locfleche,
                                       TolGuide, ref ffi, ref lla, false, false, true, Soldep, ref intf, ref intl, ref Gd1,
                                       ref Gd2, ref Gf1, ref Gf2, false, true);
                    if (done && Gf2)
                    {
                        done = CompleteData(coin, func, lin, Fac, Surf, OFac, Gd1, false, Gf1, false);
                        filling = !done;
                    }
                    else
                    {
                        filling = true;
                    }
                }

            }

            if (filling)
            {


                // the contour to be fillet consists of straight lines uv in beginning and end
                // of two pcurves (only one if c1pointu) calculated as possible
                // on piv and the opposite face.
                GeomFill_Boundary? Bdeb = null, Bfin = null, Bpiv = null, Bfac = null;
                Geom2d_Curve? PCurveOnFace = null;
                if (!c1pointu)
                {
                    Bfac = ChFi3d_mkbound(Fac, out PCurveOnFace, sens[deb], pfac1, vfac1, sens[fin], pfac2, vfac2, tolapp3d, 2.0e-4);
                }
                int kkk = 0;
                gp_Pnt ppbid = new();
                gp_Vec vp1 = new(), vp2 = new();
                kkk =
                    CD[deb].SetOfSurfData().Value(i[deb, pivot]).Interference(jf[deb, pivot]).LineIndex();
                DStr.Curve(kkk).Curve().D1(p[deb, pivot], out ppbid, out vp1);
                kkk =
                    CD[fin].SetOfSurfData().Value(i[fin, pivot]).Interference(jf[fin, pivot]).LineIndex();
                DStr.Curve(kkk).Curve().D1(p[fin, pivot], out ppbid, out vp2);
                Geom2d_Curve? PCurveOnPiv = null;
                //      Bpiv = ChFi3d_mkbound(Surf,PCurveOnPiv,sens[deb],psurf1,vp1,
                //			    sens[fin],psurf2,vp2,tolesp,2.e-4);
                Bpiv = ChFi3d_mkbound(Surf, out PCurveOnPiv, psurf1, psurf2, tolapp3d, 2.0e-4, false);
                double pardeb2 = p[deb, pivot];
                double parfin2 = p[fin, pivot];
                if (c1pointu)
                {
                    pardeb2 = p[deb, fin];
                    parfin2 = p[fin, deb];
                }
                gp_Pnt2d pdeb1 = CD[deb]
                                                      .SetOfSurfData()
                                                      .Value(i[deb, pivot])
                                                      .Interference(jf[deb, pivot])
                                                      .PCurveOnSurf()
                                                      .Value(p[deb, pivot]);
                gp_Pnt2d pdeb2 = CD[deb]
                                                      .SetOfSurfData()
                                                      .Value(i[deb, pivot])
                                                      .Interference(jf[deb, fin])
                                                      .PCurveOnSurf()
                                                      .Value(pardeb2);
                gp_Pnt2d pfin1 = CD[fin]
                                                      .SetOfSurfData()
                                                      .Value(i[fin, pivot])
                                                      .Interference(jf[fin, pivot])
                                                      .PCurveOnSurf()
                                                      .Value(p[fin, pivot]);
                gp_Pnt2d pfin2 = CD[fin]
                                                      .SetOfSurfData()
                                                      .Value(i[fin, pivot])
                                                      .Interference(jf[fin, deb])
                                                      .PCurveOnSurf()
                                                      .Value(parfin2);
                Geom_Surface? sdeb = 
                    DStr.Surface(CD[deb].SetOfSurfData().Value(i[deb, pivot]).Surf()).Surface();
                Geom_Surface? sfin = 
                    DStr.Surface(CD[fin].SetOfSurfData().Value(i[fin, pivot]).Surf()).Surface();

                Bdeb = ChFi3d_mkbound(sdeb, pdeb1, pdeb2, tolapp3d, 2.0e-4);
                Bfin = ChFi3d_mkbound(sfin, pfin1, pfin2, tolapp3d, 2.0e-4);

                GeomFill_ConstrainedFilling fil = new(11, 20);
                if (c1pointu)
                {
                    fil.Init(Bpiv, Bfin, Bdeb, true);
                }
                else
                {
                    fil.Init(Bpiv, Bfin, Bfac, Bdeb, true);
                }

                Geom_Surface? Surfcoin = fil.InternalSurface();
                Surfcoin.VReverse(); // revert to direction face surface;
                done = CompleteData(coin, Surfcoin, Fac, PCurveOnFace, Surf, PCurveOnPiv, fdpiv.Orientation(), false, false,
                                                        false, false, false);

            }
        }
        double P1deb = 0.0, P2deb = 0.0, P1fin = 0.0, P2fin = 0.0;
        if (!c1pointu)
        {
            p[deb, fin] = p[deb, pivot];
            p[fin, deb] = p[fin, pivot];
        }

        if (done)
        {
            // Update of 4 Stripes and the DS
            // -------------------------------------


            gp_Pnt2d pp1 = new(), pp2 = new();
            double parpp1 = 0.0, parpp2 = 0.0;
            int If1 = 0, If2 = 0, Il1 = 0, Il2 = 0, Icf = 0, Icl = 0;
            ChFiDS_CommonPoint Pf1 = coin.VertexFirstOnS1();
            ChFiDS_CommonPoint Pf2 = coin.VertexFirstOnS2();
            ref ChFiDS_CommonPoint Pl1 = ref coin.ChangeVertexLastOnS1();
            if (c1pointu)
            {
                Pl1 = coin.ChangeVertexFirstOnS1(); // OCCT: assigns through the reference, LastOnS1 = FirstOnS1
            }
            ChFiDS_CommonPoint Pl2 = coin.VertexLastOnS2();

            // OCCT's boxes bf1, bl1, bf2, bl2 and the pointers to them: an array and indices (pbl1 may alias pbf1)
            Bnd_Box[] bx = [new(), new(), new(), new()];
            int pbf1 = 0, pbl1 = 1, pbf2 = 2, pbl2 = 3;
            if (c1pointu)
            {
                pbl1 = pbf1;
            }
            bx[pbf1].Add(Pf1.Point());
            bx[pbf2].Add(Pf2.Point());
            bx[pbl1].Add(Pl1.Point());
            bx[pbl2].Add(Pl2.Point());

            // the start corner,
            // -----------------------
            ChFiDS_Regul regdeb = new(), regfin = new();
            If1 = ChFi3d_IndexPointInDS(Pf1, DStr);
            If2 = ChFi3d_IndexPointInDS(Pf2, DStr);
            if (c1pointu)
            {
                Il1 = If1;
            }
            else
            {
                Il1 = ChFi3d_IndexPointInDS(Pl1, DStr);
            }
            Il2 = ChFi3d_IndexPointInDS(Pl2, DStr);
            pp1 = coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().FirstParameter());
            pp2 = coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().FirstParameter());
            if (c1pointu)
            {
                coin.ChangeIndexOfS1(0);
            }
            else
            {
                coin.ChangeIndexOfS1(DStr.AddShape(face[pivot]));
            }
            coin.ChangeIndexOfS2(-fdpiv.Surf());

            Geom_Curve? C3d = null;
            double tolreached = 0.0;
            ChFi3d_ComputeArete(Pf1, pp1, Pf2, pp2, DStr.Surface(coin.Surf()).Surface(), ref C3d,
                                                    ref corner.ChangeFirstPCurve(), ref P1deb, ref P2deb, tolapp3d, tol2d, out tolreached, 0);
            TopOpeBRepDS_Curve Tcurv1 = new(C3d, tolreached);
            Icf = DStr.AddCurve(Tcurv1);
            regdeb.SetCurve(Icf);
            regdeb.SetS1(coin.Surf(), false);
            regdeb.SetS2(fddeb.Surf(), false);
            myRegul.Append(regdeb);
            corner.ChangeFirstCurve(Icf);
            corner.ChangeFirstParameters(P1deb, P2deb);
            corner.ChangeIndexFirstPointOnS1(If1);
            corner.ChangeIndexFirstPointOnS2(If2);
            ChFi3d_EnlargeBox(DStr, corner, coin, ref bx[pbf1], ref bx[pbf2], true);

            pp1 = coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().LastParameter());
            pp2 = coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().LastParameter());
            ChFi3d_ComputeArete(Pl1, pp1, Pl2, pp2, DStr.Surface(coin.Surf()).Surface(), ref C3d,
                                                    ref corner.ChangeLastPCurve(), ref P1fin, ref P2fin, tolapp3d, tol2d, out tolreached, 0);
            TopOpeBRepDS_Curve Tcurv2 = new(C3d, tolreached);
            Icl = DStr.AddCurve(Tcurv2);
            regfin.SetCurve(Icl);
            regfin.SetS1(coin.Surf(), false);
            regfin.SetS2(fdfin.Surf(), false);
            myRegul.Append(regfin);
            corner.ChangeLastCurve(Icl);
            corner.ChangeLastParameters(P1fin, P2fin);
            corner.ChangeIndexLastPointOnS1(Il1);
            corner.ChangeIndexLastPointOnS2(Il2);
            ChFi3d_EnlargeBox(DStr, corner, coin, ref bx[pbl1], ref bx[pbl2], false);

            // then CornerData of the beginning,
            // --------------------------------
            bool isfirst = (sens[deb] == 1), rev = (jf[deb, fin] == 2);
            int isurf1 = 1, isurf2 = 2;
            parpp1 = p[deb, fin];
            parpp2 = p[deb, pivot];
            if (rev)
            {
                isurf1 = 2;
                isurf2 = 1;
                parpp1 = p[deb, pivot];
                parpp2 = p[deb, fin];
                CD[deb].SetOrientation(TopAbs_REVERSED, isfirst);
            }
            pp1 = fddeb.InterferenceOnS1().PCurveOnSurf().Value(parpp1);
            pp2 = fddeb.InterferenceOnS2().PCurveOnSurf().Value(parpp2);
            CD[deb].SetCurve(Icf, isfirst);
            CD[deb].SetIndexPoint(If1, isfirst, isurf1);
            CD[deb].SetIndexPoint(If2, isfirst, isurf2);
            CD[deb].SetParameters(isfirst, P1deb, P2deb);
            fddeb.ChangeVertex(isfirst, isurf1) = Pf1;
            fddeb.ChangeVertex(isfirst, isurf2) = Pf2;
            fddeb.ChangeInterferenceOnS1().SetParameter(parpp1, isfirst);
            fddeb.ChangeInterferenceOnS2().SetParameter(parpp2, isfirst);
            ref TopOpeBRepDS_Curve tcdeb = ref DStr.ChangeCurve(Icf);
            Geom_Curve? crefdeb = tcdeb.Curve();
            double tolrdeb = 0.0;
            ChFi3d_ComputePCurv(crefdeb, pp1, pp2, out CD[deb].ChangePCurve(isfirst), DStr.Surface(fddeb.Surf()).Surface(),
                                                    P1deb, P2deb, tolapp3d, out tolrdeb, rev);
            tcdeb.Tolerance(Max(tolrdeb, tcdeb.Tolerance()));
            if (rev)
            {
                ChFi3d_EnlargeBox(DStr, CD[deb], fddeb, ref bx[pbf2], ref bx[pbf1], isfirst);
            }
            else
            {
                ChFi3d_EnlargeBox(DStr, CD[deb], fddeb, ref bx[pbf1], ref bx[pbf2], isfirst);
            }

            // then the end CornerData,
            // ------------------------
            isfirst = (sens[fin] == 1);
            rev = (jf[fin, deb] == 2);
            isurf1 = 1;
            isurf2 = 2;
            parpp1 = p[fin, deb];
            parpp2 = p[fin, pivot];
            if (rev)
            {
                isurf1 = 2;
                isurf2 = 1;
                parpp1 = p[fin, pivot];
                parpp2 = p[fin, deb];
                CD[fin].SetOrientation(TopAbs_REVERSED, isfirst);
            }
            pp1 = fdfin.InterferenceOnS1().PCurveOnSurf().Value(parpp1);
            pp2 = fdfin.InterferenceOnS2().PCurveOnSurf().Value(parpp2);
            CD[fin].SetCurve(Icl, isfirst);
            CD[fin].SetIndexPoint(Il1, isfirst, isurf1);
            CD[fin].SetIndexPoint(Il2, isfirst, isurf2);
            CD[fin].SetParameters(isfirst, P1fin, P2fin);
            fdfin.ChangeVertex(isfirst, isurf1) = Pl1;
            fdfin.ChangeVertex(isfirst, isurf2) = Pl2;
            fdfin.ChangeInterferenceOnS1().SetParameter(parpp1, isfirst);
            fdfin.ChangeInterferenceOnS2().SetParameter(parpp2, isfirst);
            ref TopOpeBRepDS_Curve tcfin = ref DStr.ChangeCurve(Icl);
            Geom_Curve? creffin = tcfin.Curve();
            double tolrfin = 0.0;
            ChFi3d_ComputePCurv(creffin, pp1, pp2, out CD[fin].ChangePCurve(isfirst), DStr.Surface(fdfin.Surf()).Surface(),
                                                    P1fin, P2fin, tolapp3d, out tolrfin, rev);
            tcfin.Tolerance(Max(tolrfin, tcfin.Tolerance()));
            if (rev)
            {
                ChFi3d_EnlargeBox(DStr, CD[fin], fdfin, ref bx[pbl2], ref bx[pbl1], isfirst);
            }
            else
            {
                ChFi3d_EnlargeBox(DStr, CD[fin], fdfin, ref bx[pbl1], ref bx[pbl2], isfirst);
            }

            // anf finally the pivot.
            // ------------------
            ref ChFiDS_FaceInterference fi = ref coin.ChangeInterferenceOnS2();
            isfirst = (sens[pivot] == 1);
            rev = (jf[pivot, deb] == 2);
            isurf1 = 1;
            isurf2 = 2;
            if (rev)
            {
                isurf1 = 2;
                isurf2 = 1;
                CD[pivot].SetOrientation(TopAbs_REVERSED, isfirst);
            }
            int ICcoinpiv = fi.LineIndex();
            ref TopOpeBRepDS_Curve TCcoinpiv = ref DStr.ChangeCurve(ICcoinpiv);
            CD[pivot].SetCurve(ICcoinpiv, isfirst);
            Geom_Curve? Ccoinpiv = DStr.Curve(ICcoinpiv).Curve();
            ref Geom2d_Curve? C2dOnPiv = ref fi.ChangePCurveOnFace();
            Geom_Surface? Spiv = DStr.Surface(fdpiv.Surf()).Surface();
            double tolr = 0.0;
            ChFi3d_SameParameter(Ccoinpiv, ref C2dOnPiv, Spiv, fi.FirstParameter(), fi.LastParameter(), tolapp3d, out tolr);
            TCcoinpiv.Tolerance(Max(TCcoinpiv.Tolerance(), tolr));
            CD[pivot].ChangePCurve(isfirst) = C2dOnPiv;
            CD[pivot].SetIndexPoint(If2, isfirst, isurf1);
            CD[pivot].SetIndexPoint(Il2, isfirst, isurf2);
            CD[pivot].SetParameters(isfirst, fi.FirstParameter(), fi.LastParameter());
            fdpiv.ChangeVertex(isfirst, isurf1) = Pf2;
            fdpiv.ChangeVertex(isfirst, isurf2) = Pl2;
            fdpiv.ChangeInterference(isurf1).SetParameter(p[pivot, deb], isfirst);
            fdpiv.ChangeInterference(isurf2).SetParameter(p[pivot, fin], isfirst);
            CD[pivot].InDS(isfirst); // filDS already does it from the corner.
            if (rev)
            {
                ChFi3d_EnlargeBox(DStr, CD[pivot], fdpiv, ref bx[pbl2], ref bx[pbf2], isfirst);
            }
            else
            {
                ChFi3d_EnlargeBox(DStr, CD[pivot], fdpiv, ref bx[pbf2], ref bx[pbl2], isfirst);
            }

            // To end the tolerances of points are rescaled.
            ChFi3d_SetPointTolerance(DStr, bx[pbf1], If1);
            ChFi3d_SetPointTolerance(DStr, bx[pbf2], If2);
            ChFi3d_SetPointTolerance(DStr, bx[pbl1], Il1);
            ChFi3d_SetPointTolerance(DStr, bx[pbl2], Il2);
        }

        // The data corners are truncated and index is updated.
        //----------------------------------------------------

        if (i[deb, pivot] < Index[deb])
        {
            CD[deb].ChangeSetOfSurfData().Remove(i[deb, pivot] + 1, Index[deb]);
            Index[deb] = i[deb, pivot];
        }
        else if (i[deb, pivot] > Index[deb])
        {
            CD[deb].ChangeSetOfSurfData().Remove(Index[deb], i[deb, pivot] - 1);
            i[deb, pivot] = Index[deb];
        }
        if (i[fin, pivot] < Index[fin])
        {
            CD[fin].ChangeSetOfSurfData().Remove(i[fin, pivot] + 1, Index[fin]);
            Index[fin] = i[fin, pivot];
        }
        else if (i[fin, pivot] > Index[fin])
        {
            CD[fin].ChangeSetOfSurfData().Remove(Index[fin], i[fin, pivot] - 1);
            i[fin, pivot] = Index[fin];
        }
        // it is necessary to take into account mutant corners.
        if (i[pivot, deb] < Index[pivot])
        {
            CD[pivot].ChangeSetOfSurfData().Remove(i[pivot, deb] + 1, Index[pivot]);
            Index[pivot] = i[pivot, deb];
        }
        else if (i[pivot, deb] > Index[pivot])
        {
            CD[pivot].ChangeSetOfSurfData().Remove(Index[pivot], i[pivot, deb] - 1);
            i[pivot, deb] = Index[pivot];
        }
        if (!myEVIMap.IsBound(Vtx))
        {
            NCollection_List<int> li = new();
            myEVIMap.Bind(Vtx, li);
        }
        myEVIMap.ChangeFind(Vtx).Append(coin.Surf());
        corner.SetSolidIndex(CD[pivot].SolidIndex());
        myListStripe.Append(corner);

    }

}

internal static class ChFi3d_FilBuilder_C3
{
    internal static int SearchPivot(int[] s, double[,] u, double t)

    {
        bool bondeb = false, bonfin = false;
        for (int i = 0; i <= 2; i++)
        {
            if (s[(i + 1) % 3] == 1)
            {
                bondeb = (u[(i + 1) % 3, i] - u[(i + 1) % 3, (i + 2) % 3] >= -t);
            }
            else
            {
                bondeb = (u[(i + 1) % 3, i] - u[(i + 1) % 3, (i + 2) % 3] <= t);
            }
            if (s[(i + 2) % 3] == 1)
            {
                bonfin = (u[(i + 2) % 3, i] - u[(i + 2) % 3, (i + 1) % 3] >= -t);
            }
            else
            {
                bonfin = (u[(i + 2) % 3, i] - u[(i + 2) % 3, (i + 1) % 3] <= t);
            }
            if (bondeb && bonfin)
            {
                return i;
            }
        }
        return -1;
    }

    internal static bool SearchFD(TopOpeBRepDS_DataStructure DStr, ChFiDS_Stripe? cd1, ChFiDS_Stripe? cd2, int sens1, int sens2, ref int i1, ref int i2, ref double p1, ref double p2, int ind1, int ind2, ref TopoDS_Face face, ref bool sameside, ref int jf1, ref int jf2)
    {
        bool found = false;
        int id1 = ind1, id2 = ind2;
        int if1 = ind1, if2 = ind2;
        int l1 = cd1.SetOfSurfData().Length();
        int l2 = cd2.SetOfSurfData().Length();
        int i = 0;
        bool fini1 = false, fini2 = false;
        bool visavis = false;
        TopoDS_Vertex Vtx = new();
        while (!found)
        {
            for (i = id1; (i * sens1) <= (if1 * sens1) && !found && !fini2; i = i + sens1)
            {
                if (ChFi3d_IsInFront(DStr, cd1, cd2, i,
                                                          if2, sens1, sens2, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2, ref visavis,
                                                          Vtx, false, false))
                {
                    i1 = i;
                    i2 = if2;
                    found = true;
                }
            }
            if (!fini1)
            {
                if1 = if1 + sens1;
                if (if1 < 1 || if1 > l1)
                {
                    if1 = if1 - sens1;
                    fini1 = true;
                }
            }

            for (i = id2; (i * sens2) <= (if2 * sens2) && !found && !fini1; i = i + sens2)
            {
                if (ChFi3d_IsInFront(DStr, cd1, cd2,
                                                          if1, i, sens1, sens2, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2,
                                                          ref visavis, Vtx, false, false))
                {
                    i1 = if1;
                    i2 = i;
                    found = true;
                }
            }
            if (!fini2)
            {
                if2 = if2 + sens2;
                if (if2 < 1 || if2 > l2)
                {
                    if2 = if2 - sens2;
                    fini2 = true;
                }
            }
            if (fini1 && fini2)
            {
                break;
            }
        }
        return found;
    }

    internal static bool ToricCorner(in TopoDS_Face F, double rd, double rf, in gp_Vec v)
    {
        if (Math.Abs(rd - rf) > Precision.Confusion())
        {
            return false;
        }
        BRepAdaptor_Surface bs = new(F);
        if (bs.GetType() != GeomAbs_Plane)
        {
            return false;
        }
        double scal1 = Math.Abs(bs.Plane().Position().XDirection().Dot(new gp_Dir(v)));
        double scal2 = Math.Abs(bs.Plane().Position().YDirection().Dot(new gp_Dir(v)));
        return (scal1 <= Precision.Confusion() && scal2 <= Precision.Confusion());
    }


}
