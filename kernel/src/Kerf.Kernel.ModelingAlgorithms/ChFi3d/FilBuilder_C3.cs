// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_FilBuilder_C3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.FilBuilder_C3;
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
public partial class FilBuilder
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

    internal override void PerformThreeCorner(int jndex)
    {


        DsDataStructure dStr = _ds.ChangeDS();
        Vertex vtx = _vDataMap.FindKey(jndex);
        NCollection_List<Stripe>.Iterator it = new();
        int[] index = new int[3];
        int pivot = 0, deb = 0, fin = 0, ii = 0, jj = 0, kk = 0;
        // double R = 0.;
        bool pivdif = true;
        bool c1pointu = false;
        bool c1toric = false;
        bool c1spheric = false;
        Stripe?[] cd = new Stripe?[3];
        Face[] face = [new(), new(), new()];
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

        for (it.Initialize(_vDataMap[jndex]), ii = 0; it.More() && ii < 3; it.Next(), ii++)
        {
            index[ii] = ChFi3d_IndexOfSurfData(vtx, it.Value(), out sens[ii]);
            cd[ii] = it.Value();
        }
        // It is checked if one of CD is not present twice in which
        // case it is necessary to modify the return of IndexOfSurfData
        // that takes the first solution.
        if (cd[0] == cd[1])
        {
            index[1] = cd[1].SetOfSurfData().Length();
            sens[1] = -1;
        }
        else if (cd[1] == cd[2])
        {
            index[2] = cd[2].SetOfSurfData().Length();
            sens[2] = -1;
        }
        else if (cd[0] == cd[2])
        {
            index[2] = cd[2].SetOfSurfData().Length();
            sens[2] = -1;
        }
        oksea[2] = SearchFD(dStr, cd[0], cd[1], sens[0], sens[1], ref i[0, 1], ref i[1, 0], ref p[0, 1], ref p[1, 0], index[0],
                            index[1], ref face[2], ref sameside[2], ref jf[0, 1], ref jf[1, 0]);
        oksea[1] = SearchFD(dStr, cd[0], cd[2], sens[0], sens[2], ref i[0, 2], ref i[2, 0], ref p[0, 2], ref p[2, 0], index[0],
                            index[2], ref face[1], ref sameside[1], ref jf[0, 2], ref jf[2, 0]);
        oksea[0] = SearchFD(dStr, cd[1], cd[2], sens[1], sens[2], ref i[1, 2], ref i[2, 1], ref p[1, 2], ref p[2, 1], index[1],
                            index[2], ref face[0], ref sameside[0], ref jf[1, 2], ref jf[2, 1]);
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
            pivot = SearchPivot(sens, p, _tol2d);
            if (pivot < 0)
            {
                PerformMoreThreeCorner(jndex, 3);
                return;
            }
            else
            {
                deb = (pivot + 1) % 3;
                fin = (pivot + 2) % 3;
            }
            pivdif = false;
            if (Math.Abs(p[0, 1] - p[0, 2]) <= _tol2d && Math.Abs(p[1, 0] - p[1, 2]) <= _tol2d
                    && Math.Abs(p[2, 0] - p[2, 1]) <= _tol2d)
            {
                c1pointu = true;
            }
        }
        else
        {
            PerformMoreThreeCorner(jndex, 3);
            return;
        }
        int ifacdeb = 0, ifacfin = 0;
        ifacdeb = cd[deb].ChangeSetOfSurfData().Value(i[deb, pivot]).Index(3 - jf[deb, pivot]);
        ifacfin = cd[fin].ChangeSetOfSurfData().Value(i[fin, pivot]).Index(3 - jf[fin, pivot]);
        if (ifacfin != ifacdeb)
        {
            PerformMoreThreeCorner(jndex, 3);
            return;
        }
        if (i[pivot, deb] != i[pivot, fin])
        {
            PerformMoreThreeCorner(jndex, 3);
            return;
        }

        double rdeb = 0.0, rfin = 0.0, rdp = 0.0, rfp = 0.0;
        Point3D pdeb = new(), pfin = new(), pdp = new(), pfp = new();
        Vector3D vdeb = new(), vfin = new(), vdp = new(), vfp = new();
        if (c1pointu)
        {
            Point3D pbid = new();
            Vector3D[] qv = [new(), new(), new()];
            double[] qr = new double[3];
            for (ii = 0; ii <= 2; ii++)
            {
                jj = (ii + 1) % 3;
                kk = (ii + 2) % 3;
                ChFi3d_ExtrSpineCarac(dStr, cd[jj], i[jj, ii], p[jj, ii], 1, sens[jj], out pbid, out qv[jj], out qr[jj]);
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
                c1spheric = (Math.Abs(qr[0] - qr[1]) < _tolapp3d && Math.Abs(qr[0] - qr[2]) < _tolapp3d);
            }
        }

        //  Previously to avoid loops the points were always located
        //  inside, which could impede the construction of the
        //  guideline of the corner which now is a circle.
        //  int jjjd = jf[deb, fin], jjjf = jf[fin, deb];
        //  if (pivdif) jjjd = jf[deb, pivot], jjjf = jf[fin, pivot];
        int jjjd = jf[deb, pivot], jjjf = jf[fin, pivot];
        ChFi3d_ExtrSpineCarac(dStr, cd[deb], i[deb, pivot], p[deb, pivot], jjjd, sens[deb], out pdeb, out vdeb, out rdeb);
        ChFi3d_ExtrSpineCarac(dStr, cd[fin], i[fin, pivot], p[fin, pivot], jjjf, sens[fin], out pfin, out vfin, out rfin);
        ChFi3d_ExtrSpineCarac(dStr, cd[pivot], i[pivot, deb], p[pivot, deb], 0, sens[pivot], out pdp, out vdp, out rdp);
        ChFi3d_ExtrSpineCarac(dStr, cd[pivot], i[pivot, fin], p[pivot, fin], 0, sens[pivot], out pfp, out vfp, out rfp);
        // in cas of allsame it is checked that points on the face are not
        // too close, which can stop the work.
        if (!pivdif)
        {
            Point3D ptestdeb = new(), ptestfin = new();
            Vector3D bidvec = new();
            double bidr = 0.0;
            ChFi3d_ExtrSpineCarac(dStr, cd[deb], i[deb, pivot], p[deb, pivot], jf[deb, fin], sens[deb], out ptestdeb,
                                                        out bidvec, out bidr);
            ChFi3d_ExtrSpineCarac(dStr, cd[fin], i[fin, pivot], p[fin, pivot], jf[fin, deb], sens[fin], out ptestfin,
                                                        out bidvec, out bidr);
            double distest = ptestdeb.Distance(ptestfin);
            if (distest < (rdp + rfp) * 0.05)
            {
                filling = true;
            }
            if (distest < (rdp + rfp) * 0.005)
            {
                c1pointu = true;
            }
        }

        if (!c1pointu)
        {
            if (!pivdif)
            {
                c1pointu = (Math.Abs(p[deb, pivot] - p[deb, fin]) <= _tol2d
                                        && Math.Abs(p[fin, pivot] - p[fin, deb]) <= _tol2d);
            }
            if (Math.Abs(p[pivot, deb] - p[pivot, fin]) <= _tol2d)
            {
                c1toric = ToricCorner(face[pivot], rdeb, rfin, vdp);
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
        SurfData? fddeb = cd[deb].ChangeSetOfSurfData().ChangeValue(i[deb, pivot]);
        SurfData? fdfin = cd[fin].ChangeSetOfSurfData().ChangeValue(i[fin, pivot]);
        SurfData? fdpiv = cd[pivot].ChangeSetOfSurfData().ChangeValue(i[pivot, deb]);

        // HSurfaces and other suitable tools are constructed.
        // ----------------------------------------------------------

        Orientation oFac = face[pivot].Orientation();
        FaceSurfaceAdaptor? fac = new FaceSurfaceAdaptor(face[pivot]);
        Point2D ppp1 = new(), ppp2 = new();
        FaceInterference bid1 = 
            cd[pivot].SetOfSurfData().Value(i[pivot, deb]).InterferenceOnS1();
        ppp1 = bid1.PCurveOnSurf().Value(bid1.FirstParameter());
        FaceInterference bid2 = 
            cd[pivot].SetOfSurfData().Value(i[pivot, deb]).InterferenceOnS2();
        ppp2 = bid2.PCurveOnSurf().Value(bid2.LastParameter());
        double uu1 = ppp1.X(), uu2 = ppp2.X(), vv1 = ppp1.Y(), vv2 = ppp2.Y();
        SurfaceAdaptor gasurf = new((dStr.Surface(cd[pivot].SetOfSurfData().Value(i[pivot, deb]).Surf())).Surface(),
                                         uu1, uu2, vv1, vv2);
        SurfaceKind styp = gasurf.GetType();
        if (styp == SurfaceKind.Cylinder)
        {
            double h = vv2 - vv1;
            vv1 -= 0.5 * h;
            vv2 += 0.5 * h;
            gasurf.Load((dStr.Surface(cd[pivot].SetOfSurfData().Value(i[pivot, deb]).Surf())).Surface(), uu1, uu2, vv1, vv2);
        }
        else if (styp == SurfaceKind.Torus)
        {
            double h = uu2 - uu1;
            uu1 -= 0.1 * h;
            uu2 += 0.1 * h;
            gasurf.Load((dStr.Surface(cd[pivot].SetOfSurfData().Value(i[pivot, deb]).Surf())).Surface(), uu1, uu2, vv1, vv2);
        }
        else if (styp == SurfaceKind.BezierSurface || styp == SurfaceKind.BSplineSurface)
        {
            gasurf.Load((dStr.Surface(cd[pivot].SetOfSurfData().Value(i[pivot, deb]).Surf())).Surface());
        }

        SurfaceAdaptor? surf = gasurf; // OCCT copies gasurf into a handle; gasurf is not used again
        //  occ::handle<BRepTopAdaptor_TopolTool> IFac = new BRepTopAdaptor_TopolTool(Fac);
        // Try to not classify on the face for cases of reentering fillets which naturally depass
        // the border.
        SurfaceAdaptor? bidsurf = new SurfaceAdaptor(fac.GeomSurfaceOriginal());
        TopolTool? iFac = new TopolTool(bidsurf);
        // end of the attempt.
        TopolTool? iSurf = new TopolTool(surf);
        Stripe? corner = new Stripe();
        ref NCollection_HSequence<SurfData>? cornerset = ref 
            corner.ChangeSetOfSurfData();
        cornerset = new NCollection_HSequence<SurfData>();
        SurfData? coin = new SurfData();
        cornerset.Append(coin);
        Orientation o1 = default, o2 = default, os1 = default, os2 = default, oo1 = default, oo2 = default;
        int choix = cd[deb].Choix();
        o1 = face[pivot].Orientation();
        o2 = fdpiv.Orientation();
        oo1 = o1;
        oo2 = o2;
        os1 = cd[deb].OrientationOnFace1();
        os2 = cd[deb].OrientationOnFace2();
        if (jf[deb, fin] == 1)
        {
            choix = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref o1, ref o2, os1, os2, choix);
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
            choix = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref o2, ref o1, os1, os2, -choix);
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

        Point2D pfac1 = new(), pfac2 = new(), psurf1 = new(), psurf2 = new();
        Vector2D vfac1 = new(), vfac2 = new();
        cd[deb]
            .SetOfSurfData()
            .Value(i[deb, pivot])
            .Interference(jf[deb, fin])
            .PCurveOnFace()
            .D1(p[deb, pivot], out pfac1, out vfac1);
        cd[fin]
            .SetOfSurfData()
            .Value(i[fin, pivot])
            .Interference(jf[fin, deb])
            .PCurveOnFace()
            .D1(p[fin, pivot], out pfac2, out vfac2);
        psurf1 = cd[pivot]
                              .SetOfSurfData()
                              .Value(i[pivot, deb])
                              .Interference(jf[pivot, deb])
                              .PCurveOnSurf()
                              .Value(p[pivot, deb]);
        psurf2 = cd[pivot]
                              .SetOfSurfData()
                              .Value(i[pivot, fin])
                              .Interference(jf[pivot, fin])
                              .PCurveOnSurf()
                              .Value(p[pivot, fin]);

        _done = false;


        if (c1toric)
        {


            // Direct Construction.
            // ---------------------
            _done = global::Kerf.Engine.Fillets.Analytic.ComputeData.ComputeCorner(dStr, coin, fac, surf, oo1, oo2, o1, o2, rdeb, rdp, pfac1, pfac2,
                                                                                                    psurf1, psurf2);

        }
        else if (c1spheric)
        {


            _done = global::Kerf.Engine.Fillets.Analytic.ComputeData.ComputeCorner(dStr, coin, fac, surf, oo1, oo2, o1, o2, rdp, pfac1, psurf1, psurf2);

        }
        else if (c1pointu)
        {
            filling = true;
        }
        if (!_done)
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

                double radpondere = (rdp + rfp) / 2.0;
                double locfleche = _fleche;

                double wFirst = 0.0, wLast = 0.0;
                Curve3D? spinecoin = 
                    ChFi3d_CircularSpine(ref wFirst, ref wLast, pdeb, vdeb, pfin, vfin, radpondere);
                if (spinecoin is null)
                {
                    // This is a bad case when the intersection of
                    // section planes is done out of the sector.
                    spinecoin = ChFi3d_Spine(pdeb, ref vdeb, pfin, ref vfin, radpondere);
                    wFirst = 0.0;
                    wLast = 1.0;
                }
                else
                {
                    locfleche = radpondere * (wLast - wFirst) * _fleche;
                }
                double pasmax = (wLast - wFirst) * 0.05;
                ElSpine? cornerspine = new ElSpine();
                cornerspine.SetCurve(spinecoin);
                cornerspine.FirstParameter(wFirst - pasmax);
                cornerspine.LastParameter(wLast + pasmax);
                // Just to confuse Compute that should not require this
                // in this exact case ...
                Spine? nullSpine = null;
                // The fillet is calculated - from beginning to end
                //                       - from the face to the surface
                //
                math_Vector soldep = new(1, 4);
                soldep[1] = pfac1.X();
                soldep[2] = pfac1.Y();
                soldep[3] = psurf1.X();
                soldep[4] = psurf1.Y();

                bool gd1 = false, gd2 = false, gf1 = false, gf2 = false;
                global::Kerf.Engine.Fillets.Blend.Line? lin = null;
                double ffi = wFirst, lla = wLast + pasmax;

                if (Math.Abs(rdeb - rfin) <= _tolapp3d)
                {

                    ConstRad func = new(fac, surf, cornerspine);
                    ConstRadInv finv = new(fac, surf, cornerspine);
                    func.Set(rdeb, choix);
                    func.Set(_shape);
                    finv.Set(rdeb, choix);
                    double tolGuide = cornerspine.Resolution(_tolapp3d);

                    int intf = 3, intl = 3;
                    _done = ComputeData(coin, cornerspine, nullSpine, ref lin, fac, iFac, surf, iSurf, func, finv, ffi, pasmax, locfleche,
                                       tolGuide, ref ffi, ref lla, false, false, true, soldep, ref intf, ref intl, ref gd1,
                                       ref gd2, ref gf1, ref gf2, false, true);
                    if (_done && gf2)
                    {
                        _done = CompleteData(coin, func, lin, fac, surf, oFac, gd1, false, gf1, false);
                        filling = !_done;
                    }
                    else
                    {
                        filling = true;
                    }
                }
                else
                {
                    S? law = new S();
                    law.Set(wFirst, rdeb, wLast, rfin);
                    EvolRad func = new(fac, surf, cornerspine, law);
                    EvolRadInv finv = new(fac, surf, cornerspine, law);
                    func.Set(choix);
                    func.Set(_shape);
                    finv.Set(choix);
                    double tolGuide = cornerspine.Resolution(_tolapp3d);
                    int intf = 3, intl = 3;
                    _done = ComputeData(coin, cornerspine, nullSpine, ref lin, fac, iFac, surf, iSurf, func, finv, ffi, pasmax, locfleche,
                                       tolGuide, ref ffi, ref lla, false, false, true, soldep, ref intf, ref intl, ref gd1,
                                       ref gd2, ref gf1, ref gf2, false, true);
                    if (_done && gf2)
                    {
                        _done = CompleteData(coin, func, lin, fac, surf, oFac, gd1, false, gf1, false);
                        filling = !_done;
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
                Boundary? bdeb = null, bfin = null, bpiv = null, bfac = null;
                Curve2D? pCurveOnFace = null;
                if (!c1pointu)
                {
                    bfac = ChFi3d_mkbound(fac, out pCurveOnFace, sens[deb], pfac1, vfac1, sens[fin], pfac2, vfac2, _tolapp3d, 2.0e-4);
                }
                int kkk = 0;
                Point3D ppbid = new();
                Vector3D vp1 = new(), vp2 = new();
                kkk =
                    cd[deb].SetOfSurfData().Value(i[deb, pivot]).Interference(jf[deb, pivot]).LineIndex();
                dStr.Curve(kkk).Curve().D1(p[deb, pivot], out ppbid, out vp1);
                kkk =
                    cd[fin].SetOfSurfData().Value(i[fin, pivot]).Interference(jf[fin, pivot]).LineIndex();
                dStr.Curve(kkk).Curve().D1(p[fin, pivot], out ppbid, out vp2);
                Curve2D? pCurveOnPiv = null;
                //      Bpiv = ChFi3d_mkbound(Surf,PCurveOnPiv,sens[deb],psurf1,vp1,
                //			    sens[fin],psurf2,vp2,tolesp,2.e-4);
                bpiv = ChFi3d_mkbound(surf, out pCurveOnPiv, psurf1, psurf2, _tolapp3d, 2.0e-4, false);
                double pardeb2 = p[deb, pivot];
                double parfin2 = p[fin, pivot];
                if (c1pointu)
                {
                    pardeb2 = p[deb, fin];
                    parfin2 = p[fin, deb];
                }
                Point2D pdeb1 = cd[deb]
                                                      .SetOfSurfData()
                                                      .Value(i[deb, pivot])
                                                      .Interference(jf[deb, pivot])
                                                      .PCurveOnSurf()
                                                      .Value(p[deb, pivot]);
                Point2D pdeb2 = cd[deb]
                                                      .SetOfSurfData()
                                                      .Value(i[deb, pivot])
                                                      .Interference(jf[deb, fin])
                                                      .PCurveOnSurf()
                                                      .Value(pardeb2);
                Point2D pfin1 = cd[fin]
                                                      .SetOfSurfData()
                                                      .Value(i[fin, pivot])
                                                      .Interference(jf[fin, pivot])
                                                      .PCurveOnSurf()
                                                      .Value(p[fin, pivot]);
                Point2D pfin2 = cd[fin]
                                                      .SetOfSurfData()
                                                      .Value(i[fin, pivot])
                                                      .Interference(jf[fin, deb])
                                                      .PCurveOnSurf()
                                                      .Value(parfin2);
                Surface? sdeb = 
                    dStr.Surface(cd[deb].SetOfSurfData().Value(i[deb, pivot]).Surf()).Surface();
                Surface? sfin = 
                    dStr.Surface(cd[fin].SetOfSurfData().Value(i[fin, pivot]).Surf()).Surface();

                bdeb = ChFi3d_mkbound(sdeb, pdeb1, pdeb2, _tolapp3d, 2.0e-4);
                bfin = ChFi3d_mkbound(sfin, pfin1, pfin2, _tolapp3d, 2.0e-4);

                ConstrainedFilling fil = new(11, 20);
                if (c1pointu)
                {
                    fil.Init(bpiv, bfin, bdeb, true);
                }
                else
                {
                    fil.Init(bpiv, bfin, bfac, bdeb, true);
                }

                Surface? surfcoin = fil.InternalSurface();
                surfcoin.VReverse(); // revert to direction face surface;
                _done = CompleteData(coin, surfcoin, fac, pCurveOnFace, surf, pCurveOnPiv, fdpiv.Orientation(), false, false,
                                                        false, false, false);

            }
        }
        double p1deb = 0.0, p2deb = 0.0, p1fin = 0.0, p2fin = 0.0;
        if (!c1pointu)
        {
            p[deb, fin] = p[deb, pivot];
            p[fin, deb] = p[fin, pivot];
        }

        if (_done)
        {
            // Update of 4 Stripes and the DS
            // -------------------------------------


            Point2D pp1 = new(), pp2 = new();
            double parpp1 = 0.0, parpp2 = 0.0;
            int if1 = 0, if2 = 0, il1 = 0, il2 = 0, icf = 0, icl = 0;
            CommonPoint pf1 = coin.VertexFirstOnS1();
            CommonPoint pf2 = coin.VertexFirstOnS2();
            ref CommonPoint pl1 = ref coin.ChangeVertexLastOnS1();
            if (c1pointu)
            {
                pl1 = coin.ChangeVertexFirstOnS1(); // OCCT: assigns through the reference, LastOnS1 = FirstOnS1
            }
            CommonPoint pl2 = coin.VertexLastOnS2();

            // OCCT's boxes bf1, bl1, bf2, bl2 and the pointers to them: an array and indices (pbl1 may alias pbf1)
            Box[] bx = [new(), new(), new(), new()];
            int pbf1 = 0, pbl1 = 1, pbf2 = 2, pbl2 = 3;
            if (c1pointu)
            {
                pbl1 = pbf1;
            }
            bx[pbf1].Add(pf1.Point());
            bx[pbf2].Add(pf2.Point());
            bx[pbl1].Add(pl1.Point());
            bx[pbl2].Add(pl2.Point());

            // the start corner,
            // -----------------------
            Regul regdeb = new(), regfin = new();
            if1 = ChFi3d_IndexPointInDS(pf1, dStr);
            if2 = ChFi3d_IndexPointInDS(pf2, dStr);
            if (c1pointu)
            {
                il1 = if1;
            }
            else
            {
                il1 = ChFi3d_IndexPointInDS(pl1, dStr);
            }
            il2 = ChFi3d_IndexPointInDS(pl2, dStr);
            pp1 = coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().FirstParameter());
            pp2 = coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().FirstParameter());
            if (c1pointu)
            {
                coin.ChangeIndexOfS1(0);
            }
            else
            {
                coin.ChangeIndexOfS1(dStr.AddShape(face[pivot]));
            }
            coin.ChangeIndexOfS2(-fdpiv.Surf());

            Curve3D? c3d = null;
            double tolreached = 0.0;
            ChFi3d_ComputeArete(pf1, pp1, pf2, pp2, dStr.Surface(coin.Surf()).Surface(), ref c3d,
                                                    ref corner.ChangeFirstPCurve(), ref p1deb, ref p2deb, _tolapp3d, _tol2d, out tolreached, 0);
            DsCurve tcurv1 = new(c3d, tolreached);
            icf = dStr.AddCurve(tcurv1);
            regdeb.SetCurve(icf);
            regdeb.SetS1(coin.Surf(), false);
            regdeb.SetS2(fddeb.Surf(), false);
            _regul.Append(regdeb);
            corner.ChangeFirstCurve(icf);
            corner.ChangeFirstParameters(p1deb, p2deb);
            corner.ChangeIndexFirstPointOnS1(if1);
            corner.ChangeIndexFirstPointOnS2(if2);
            ChFi3d_EnlargeBox(dStr, corner, coin, ref bx[pbf1], ref bx[pbf2], true);

            pp1 = coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().LastParameter());
            pp2 = coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().LastParameter());
            ChFi3d_ComputeArete(pl1, pp1, pl2, pp2, dStr.Surface(coin.Surf()).Surface(), ref c3d,
                                                    ref corner.ChangeLastPCurve(), ref p1fin, ref p2fin, _tolapp3d, _tol2d, out tolreached, 0);
            DsCurve tcurv2 = new(c3d, tolreached);
            icl = dStr.AddCurve(tcurv2);
            regfin.SetCurve(icl);
            regfin.SetS1(coin.Surf(), false);
            regfin.SetS2(fdfin.Surf(), false);
            _regul.Append(regfin);
            corner.ChangeLastCurve(icl);
            corner.ChangeLastParameters(p1fin, p2fin);
            corner.ChangeIndexLastPointOnS1(il1);
            corner.ChangeIndexLastPointOnS2(il2);
            ChFi3d_EnlargeBox(dStr, corner, coin, ref bx[pbl1], ref bx[pbl2], false);

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
                cd[deb].SetOrientation(Reversed, isfirst);
            }
            pp1 = fddeb.InterferenceOnS1().PCurveOnSurf().Value(parpp1);
            pp2 = fddeb.InterferenceOnS2().PCurveOnSurf().Value(parpp2);
            cd[deb].SetCurve(icf, isfirst);
            cd[deb].SetIndexPoint(if1, isfirst, isurf1);
            cd[deb].SetIndexPoint(if2, isfirst, isurf2);
            cd[deb].SetParameters(isfirst, p1deb, p2deb);
            fddeb.ChangeVertex(isfirst, isurf1) = pf1;
            fddeb.ChangeVertex(isfirst, isurf2) = pf2;
            fddeb.ChangeInterferenceOnS1().SetParameter(parpp1, isfirst);
            fddeb.ChangeInterferenceOnS2().SetParameter(parpp2, isfirst);
            ref DsCurve tcdeb = ref dStr.ChangeCurve(icf);
            Curve3D? crefdeb = tcdeb.Curve();
            double tolrdeb = 0.0;
            ChFi3d_ComputePCurv(crefdeb, pp1, pp2, out cd[deb].ChangePCurve(isfirst), dStr.Surface(fddeb.Surf()).Surface(),
                                                    p1deb, p2deb, _tolapp3d, out tolrdeb, rev);
            tcdeb.Tolerance(Max(tolrdeb, tcdeb.Tolerance()));
            if (rev)
            {
                ChFi3d_EnlargeBox(dStr, cd[deb], fddeb, ref bx[pbf2], ref bx[pbf1], isfirst);
            }
            else
            {
                ChFi3d_EnlargeBox(dStr, cd[deb], fddeb, ref bx[pbf1], ref bx[pbf2], isfirst);
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
                cd[fin].SetOrientation(Reversed, isfirst);
            }
            pp1 = fdfin.InterferenceOnS1().PCurveOnSurf().Value(parpp1);
            pp2 = fdfin.InterferenceOnS2().PCurveOnSurf().Value(parpp2);
            cd[fin].SetCurve(icl, isfirst);
            cd[fin].SetIndexPoint(il1, isfirst, isurf1);
            cd[fin].SetIndexPoint(il2, isfirst, isurf2);
            cd[fin].SetParameters(isfirst, p1fin, p2fin);
            fdfin.ChangeVertex(isfirst, isurf1) = pl1;
            fdfin.ChangeVertex(isfirst, isurf2) = pl2;
            fdfin.ChangeInterferenceOnS1().SetParameter(parpp1, isfirst);
            fdfin.ChangeInterferenceOnS2().SetParameter(parpp2, isfirst);
            ref DsCurve tcfin = ref dStr.ChangeCurve(icl);
            Curve3D? creffin = tcfin.Curve();
            double tolrfin = 0.0;
            ChFi3d_ComputePCurv(creffin, pp1, pp2, out cd[fin].ChangePCurve(isfirst), dStr.Surface(fdfin.Surf()).Surface(),
                                                    p1fin, p2fin, _tolapp3d, out tolrfin, rev);
            tcfin.Tolerance(Max(tolrfin, tcfin.Tolerance()));
            if (rev)
            {
                ChFi3d_EnlargeBox(dStr, cd[fin], fdfin, ref bx[pbl2], ref bx[pbl1], isfirst);
            }
            else
            {
                ChFi3d_EnlargeBox(dStr, cd[fin], fdfin, ref bx[pbl1], ref bx[pbl2], isfirst);
            }

            // anf finally the pivot.
            // ------------------
            ref FaceInterference fi = ref coin.ChangeInterferenceOnS2();
            isfirst = (sens[pivot] == 1);
            rev = (jf[pivot, deb] == 2);
            isurf1 = 1;
            isurf2 = 2;
            if (rev)
            {
                isurf1 = 2;
                isurf2 = 1;
                cd[pivot].SetOrientation(Reversed, isfirst);
            }
            int iCcoinpiv = fi.LineIndex();
            ref DsCurve tCcoinpiv = ref dStr.ChangeCurve(iCcoinpiv);
            cd[pivot].SetCurve(iCcoinpiv, isfirst);
            Curve3D? ccoinpiv = dStr.Curve(iCcoinpiv).Curve();
            ref Curve2D? c2dOnPiv = ref fi.ChangePCurveOnFace();
            Surface? spiv = dStr.Surface(fdpiv.Surf()).Surface();
            double tolr = 0.0;
            ChFi3d_SameParameter(ccoinpiv, ref c2dOnPiv, spiv, fi.FirstParameter(), fi.LastParameter(), _tolapp3d, out tolr);
            tCcoinpiv.Tolerance(Max(tCcoinpiv.Tolerance(), tolr));
            cd[pivot].ChangePCurve(isfirst) = c2dOnPiv;
            cd[pivot].SetIndexPoint(if2, isfirst, isurf1);
            cd[pivot].SetIndexPoint(il2, isfirst, isurf2);
            cd[pivot].SetParameters(isfirst, fi.FirstParameter(), fi.LastParameter());
            fdpiv.ChangeVertex(isfirst, isurf1) = pf2;
            fdpiv.ChangeVertex(isfirst, isurf2) = pl2;
            fdpiv.ChangeInterference(isurf1).SetParameter(p[pivot, deb], isfirst);
            fdpiv.ChangeInterference(isurf2).SetParameter(p[pivot, fin], isfirst);
            cd[pivot].InDS(isfirst); // filDS already does it from the corner.
            if (rev)
            {
                ChFi3d_EnlargeBox(dStr, cd[pivot], fdpiv, ref bx[pbl2], ref bx[pbf2], isfirst);
            }
            else
            {
                ChFi3d_EnlargeBox(dStr, cd[pivot], fdpiv, ref bx[pbf2], ref bx[pbl2], isfirst);
            }

            // To end the tolerances of points are rescaled.
            ChFi3d_SetPointTolerance(dStr, bx[pbf1], if1);
            ChFi3d_SetPointTolerance(dStr, bx[pbf2], if2);
            ChFi3d_SetPointTolerance(dStr, bx[pbl1], il1);
            ChFi3d_SetPointTolerance(dStr, bx[pbl2], il2);
        }

        // The data corners are truncated and index is updated.
        //----------------------------------------------------

        if (i[deb, pivot] < index[deb])
        {
            cd[deb].ChangeSetOfSurfData().Remove(i[deb, pivot] + 1, index[deb]);
            index[deb] = i[deb, pivot];
        }
        else if (i[deb, pivot] > index[deb])
        {
            cd[deb].ChangeSetOfSurfData().Remove(index[deb], i[deb, pivot] - 1);
            i[deb, pivot] = index[deb];
        }
        if (i[fin, pivot] < index[fin])
        {
            cd[fin].ChangeSetOfSurfData().Remove(i[fin, pivot] + 1, index[fin]);
            index[fin] = i[fin, pivot];
        }
        else if (i[fin, pivot] > index[fin])
        {
            cd[fin].ChangeSetOfSurfData().Remove(index[fin], i[fin, pivot] - 1);
            i[fin, pivot] = index[fin];
        }
        // it is necessary to take into account mutant corners.
        if (i[pivot, deb] < index[pivot])
        {
            cd[pivot].ChangeSetOfSurfData().Remove(i[pivot, deb] + 1, index[pivot]);
            index[pivot] = i[pivot, deb];
        }
        else if (i[pivot, deb] > index[pivot])
        {
            cd[pivot].ChangeSetOfSurfData().Remove(index[pivot], i[pivot, deb] - 1);
            i[pivot, deb] = index[pivot];
        }
        if (!_eviMap.ContainsKey(vtx))
        {
            NCollection_List<int> li = new();
            _eviMap[vtx] = li;
        }
        _eviMap.GetValue(vtx).Append(coin.Surf());
        corner.SetSolidIndex(cd[pivot].SolidIndex());
        _listStripe.Append(corner);

    }

}

internal static class FilBuilder_C3
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

    internal static bool SearchFD(DsDataStructure dStr, Stripe? cd1, Stripe? cd2, int sens1, int sens2, ref int i1, ref int i2, ref double p1, ref double p2, int ind1, int ind2, ref Face face, ref bool sameside, ref int jf1, ref int jf2)
    {
        bool found = false;
        int id1 = ind1, id2 = ind2;
        int if1 = ind1, if2 = ind2;
        int l1 = cd1.SetOfSurfData().Length();
        int l2 = cd2.SetOfSurfData().Length();
        int i = 0;
        bool fini1 = false, fini2 = false;
        bool visavis = false;
        Vertex vtx = new();
        while (!found)
        {
            for (i = id1; (i * sens1) <= (if1 * sens1) && !found && !fini2; i = i + sens1)
            {
                if (ChFi3d_IsInFront(dStr, cd1, cd2, i,
                                                          if2, sens1, sens2, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2, ref visavis,
                                                          vtx, false, false))
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
                if (ChFi3d_IsInFront(dStr, cd1, cd2,
                                                          if1, i, sens1, sens2, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2,
                                                          ref visavis, vtx, false, false))
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

    internal static bool ToricCorner(in Face f, double rd, double rf, in Vector3D v)
    {
        if (Math.Abs(rd - rf) > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false;
        }
        FaceSurfaceAdaptor bs = new(f);
        if (bs.GetType() != Plane)
        {
            return false;
        }
        double scal1 = Math.Abs(bs.Plane().Position().XDirection().Dot(new Direction3D(v)));
        double scal2 = Math.Abs(bs.Plane().Position().YDirection().Dot(new Direction3D(v)));
        return (scal1 <= global::Kerf.Engine.Foundation.Precision.Confusion() && scal2 <= global::Kerf.Engine.Foundation.Precision.Confusion());
    }


}
