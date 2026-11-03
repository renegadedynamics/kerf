// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_ChBuilder_C3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFcts;
using static Kerf.Engine.Fillets.ChBuilder_C3;
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
public partial class ChBuilder
{


    //=======================================================================
    // function : CoPlanar
    // purpose  : Sert a savoir si 4 points sont coplanaires, pour cela on calcul
    //           la distance de PntD par rapport au plan passant par les trois
    //           points PntA, PntB, PntC
    //=======================================================================
    //=======================================================================
    // function : BoundSurf
    // purpose  : computes a GeomAdaptor_Surface from the surface and trims
    //           it to allow the intersection computation
    //=======================================================================

    //=======================================================================
    // function : ComputeIntersection
    // purpose  : compute the 3d curve <gc> and the pcurves <pc1> and <pc2>
    //           of the intersection between one of the 3 SurfData <SD> and
    //           the SurfData of the corner <SDCoin>. Here we know the
    //           extremities of the intersection <pdeb> and <pfin>, and
    //           their parameters <p2dfin>, <p2ddeb> on <SD>.
    //           <ptcoindeb> contains the intersection 2d point on the corner
    //            which corresponds to the point <pdeb>
    //           <derudeb> and <dervdeb> are the derivative vectors on the
    //            SurfData <SD> at the point <ptdeb>
    //=======================================================================

    //======================================================================
    // function : PerformThreeCorner
    // purpose  : compute the intersection of three chamfers on a same
    //            vertex of index <Jndex> in myVDataMap
    //======================================================================

    internal override void PerformThreeCorner(int jndex)
    {

        // modifier pour le passer en option dans le cdl!!!!!!!!!!!!
        bool issmooth = false;

        DsDataStructure dStr = _ds.ChangeDS();
        Vertex vtx = _vDataMap.FindKey(jndex);
        NCollection_List<Stripe>.Iterator it = new();
        //  int Index[3],pivot,deb,fin,ii,jj,kk;
        int[] index = new int[3];
        int pivot = 0, deb = 0, fin = 0, ii = 0;
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

        bool c1triangle = false;

        for (it.Initialize(_vDataMap[jndex]), ii = 0; it.More() && ii < 3; it.Next(), ii++)
        {
            index[ii] = ChFi3d_IndexOfSurfData(vtx, it.Value(), out sens[ii]);
            cd[ii] = it.Value();
        }
        // On verifie que l une des CD ne figure pas deux fois, au quel cas
        // il faut modifier le retour de IndexOfSurfData qui prend la
        // premiere des solutions.
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
        oksea[2] = ChFi3d_SearchFD(dStr, cd[0], cd[1], sens[0], sens[1], ref i[0, 1], ref i[1, 0], ref p[0, 1], ref p[1, 0],
                                                              index[0], index[1], ref face[2], ref sameside[2], ref jf[0, 1], ref jf[1, 0]);
        oksea[1] = ChFi3d_SearchFD(dStr, cd[0], cd[2], sens[0], sens[2], ref i[0, 2], ref i[2, 0], ref p[0, 2], ref p[2, 0],
                                                              index[0], index[2], ref face[1], ref sameside[1], ref jf[0, 2], ref jf[2, 0]);
        oksea[0] = ChFi3d_SearchFD(dStr, cd[1], cd[2], sens[1], sens[2], ref i[1, 2], ref i[2, 1], ref p[1, 2], ref p[2, 1],
                                                              index[1], index[2], ref face[0], ref sameside[0], ref jf[1, 2], ref jf[2, 1]);
        //
        // Analyse des concavites des 3 chanfreins :
        //        - 2 concavites identiques et 1 inverse.
        //        - 3 concavites identiques
        //
        bool cornerAllSame = false;
        bool okinter = true;
        bool visavis = false;

        if (oksea[2] && oksea[1] && !sameside[2] && !sameside[1])
        {
            pivot = 0;
            deb = 1;
            fin = 2;
            // on calcule l'intersection des pcurves sans les restreindre a leur common point
            if (!oksea[0])
            {
                okinter = ChFi3d_IsInFront(dStr, cd[1], cd[2], i[1, 2], i[2, 1], sens[1], sens[2], ref p[1, 2], ref p[2, 1],
                                                                      ref face[0], ref sameside[0], ref jf[1, 2], ref jf[2, 1], ref visavis, vtx, false,
                                                                      true);
            }
        }
        else if (oksea[2] && oksea[0] && !sameside[2] && !sameside[0])
        {
            pivot = 1;
            deb = 2;
            fin = 0;
            if (!oksea[1])
            {
                okinter = ChFi3d_IsInFront(dStr, cd[0], cd[2], i[0, 2], i[2, 0], sens[0], sens[2], ref p[0, 2], ref p[2, 0],
                                                                      ref face[1], ref sameside[1], ref jf[0, 2], ref jf[2, 0], ref visavis, vtx, false,
                                                                      true);
            }
        }
        else if (oksea[1] && oksea[0] && !sameside[1] && !sameside[0])
        {
            pivot = 2;
            deb = 0;
            fin = 1;
            if (!oksea[2])
            {
                okinter = ChFi3d_IsInFront(dStr, cd[0], cd[1], i[0, 1], i[1, 0], sens[0], sens[1], ref p[0, 1], ref p[1, 0],
                                                                      ref face[2], ref sameside[2], ref jf[0, 1], ref jf[1, 0], ref visavis, vtx, false,
                                                                      true);
            }
        }
        else if (oksea[0] && oksea[1] && oksea[2])
        {
            // 3 concavites identiques.
            pivot = ChFi3d_SearchPivot(sens, p, _tol2d);
            if (pivot < 0)
            {
                // on prend un pivot au hasard!!!!!!!!!!!!!!!
                pivot = 0;
            }
            deb = (pivot + 1) % 3;
            fin = (pivot + 2) % 3;
            cornerAllSame = true;
        }
        else
        {
            throw new FailureException("FD en vis a vis non trouvees");
        }
        if (!okinter)
        {
            throw new FailureException("Echec intersection PCurves OnCommonFace");
        }

        // on a le pivot, le CD deb et le CD fin (enfin on espere !?!) :
        // -------------------------------------------------------------

        /* Remarque Importante : dans le cas ou les indices des Surf data
              du pivot sur lesquelles ont ete trouvees les intersections de pcurves
              ne sont pas egaux, il va y avoir changement de Surf data lors du
              cheminement et creations de Surf data mutantes a 3 ou 5 cotes!!!
              NON TRAITE !!!!!! (pour l instant)*/
        if (i[pivot, deb] != i[pivot, fin])
        {
            throw new MethodNotImplementedException("coin mutant non programme");
        }
        /* Autre Remarque : dans le cas ou les indices des Surf data
              du deb = new(de la fin) sur lesquelles ont ete trouvees les intersections
              de pcurves ne sont pas egaux, il va y avoir changement de face lors du
              cheminement NON GERE !!!!!! (pour l instant). Prevoir un
              PerformSetOfSurf adapte.*/
        if (oksea[pivot] && (i[deb, pivot] != i[deb, fin] || i[fin, pivot] != i[fin, deb]))
        {
            throw new MethodNotImplementedException("coin sur plusieurs faces non programme");
        }

        // OCCT holds references to the sequences' handles, never assigned through: plain locals
        SurfData? fddeb = cd[deb].ChangeSetOfSurfData().ChangeValue(i[deb, pivot]);
        SurfData? fdfin = cd[fin].ChangeSetOfSurfData().ChangeValue(i[fin, pivot]);
        SurfData? fdpiv = cd[pivot].ChangeSetOfSurfData().ChangeValue(i[pivot, deb]);

        // On construit les HSurfaces et autres outils qui vont bien.
        // ----------------------------------------------------------

        FaceSurfaceAdaptor? fac = new FaceSurfaceAdaptor(face[pivot]);
        SurfaceAdaptor? bidsurf = (SurfaceAdaptor)fac.AdaptorSurfaceOriginal().ShallowCopy(); // OCCT's copy
        TopolTool? iFac = new TopolTool(bidsurf);

        SurfaceAdaptor? surf = 
            ChFi3d_BoundSurf(dStr, fdpiv, jf[pivot, deb], jf[pivot, fin]);
        TopolTool? iSurf = new TopolTool(surf);

        // Creation of a new Stripe for the corner
        Stripe? corner = new Stripe();
        ref NCollection_HSequence<SurfData>? cornerset = ref 
            corner.ChangeSetOfSurfData();
        cornerset = new NCollection_HSequence<SurfData>();
        SurfData? coin = new SurfData();
        cornerset.Append(coin);

        // Pour plus de surete, on verifie les intersections des pcurves des chanfreins sur leur
        // face commune
        SurfaceAdaptor? hSdeb = new SurfaceAdaptor(dStr.Surface(fddeb.Surf()).Surface());
        SurfaceAdaptor? hSfin = new SurfaceAdaptor(dStr.Surface(fdfin.Surf()).Surface());
        SurfaceAdaptor? hSpiv = new SurfaceAdaptor(dStr.Surface(fdpiv.Surf()).Surface());

        Point2D[] p2d = [new(), new(), new(), new()];
        Point3D[] p3d = [new(), new(), new(), new()];
        Point3D pSom = new();

        ChFi3d_ComputesIntPC(fdpiv.Interference(jf[pivot, deb]), fddeb.Interference(jf[deb, pivot]), hSpiv, hSdeb,
                                                  ref p[pivot, deb], ref p[deb, pivot], ref p3d[fin]);
        ChFi3d_ComputesIntPC(fdpiv.Interference(jf[pivot, fin]), fdfin.Interference(jf[fin, pivot]), hSpiv, hSfin,
                                                  ref p[pivot, fin], ref p[fin, pivot], ref p3d[deb]);
        ChFi3d_ComputesIntPC(fddeb.Interference(jf[deb, fin]), fdfin.Interference(jf[fin, deb]), hSdeb, hSfin,
                                                  ref p[deb, fin], ref p[fin, deb], ref pSom);

        // On determine les extremites du coin
        //------------------------------------
        // c1triangle : on n'a besoin que des 3 points intersection des 3 chanfreins
        // sinon : on a les 2 points intersection de fdpiv avec fddeb et fdfin, et on
        //         cree 2 autres points sur la face commune a l'aide des deux premiers

        // p2d[deb] et p2d[fin] sur la surface du chanfrein fdpiv.
        // p2d[piv], p2d[3] (confondus si c1triangle) sur la face en bout du chanfrein de fdpiv
        // p2d[piv](resp.vp2d[3]) est sur la Uiso de fddeb(resp. fdfin) passant par p2d[deb]
        // (resp. p2d[fin])

        //  if (CornerAllSame)
        //    c1triangle = (std::abs(p[deb, pivot]-p[deb, fin])<tolesp &&
        //		  std::abs(p[fin, pivot]-p[fin, deb])<tolesp);

        Vector2D tg3 = new(), tgpiv = new();

        //  if (c1triangle)
        //    p2d[pivot] = fddeb->Interference(jf[deb, fin]).PCurveOnFace()->Value(p[deb, pivot]);
        //  else {
        if (issmooth)
        {
            fddeb.Interference(jf[deb, fin]).PCurveOnFace().D1(p[deb, pivot], out p2d[pivot], out tgpiv);
            fdfin.Interference(jf[fin, deb]).PCurveOnFace().D1(p[fin, pivot], out p2d[3], out tg3);
        }
        else
        {
            p2d[pivot] = fddeb.Interference(jf[deb, fin]).PCurveOnFace().Value(p[deb, pivot]);
            p2d[3] = fdfin.Interference(jf[fin, deb]).PCurveOnFace().Value(p[fin, pivot]);
        }
        //  }
        p2d[fin] = fdpiv.Interference(jf[pivot, deb]).PCurveOnSurf().Value(p[pivot, deb]);
        p2d[deb] = fdpiv.Interference(jf[pivot, fin]).PCurveOnSurf().Value(p[pivot, fin]);

        //  gp_Pnt pnt;
        Vector3D deru = new(), derv = new();

        //  p3d[fin] = HSpiv->Value(p2d[fin].X(),p2d[fin].Y());
        //  p3d[deb] = HSpiv->Value(p2d[deb].X(),p2d[deb].Y());
        fac.D1(p2d[pivot].X(), p2d[pivot].Y(), out p3d[pivot], out deru, out derv);
        Vector3D norpl = deru.Crossed(derv);
        //  if (!c1triangle)
        p3d[3] = fac.Value(p2d[3].X(), p2d[3].Y());

        double distMin = (p3d[3]).Distance(p3d[fin]);
        double distTmp = (p3d[pivot]).Distance(p3d[deb]);
        double distDebFin = (p3d[pivot]).Distance(p3d[3]);

        if (distTmp > distMin)
        {
            distMin = distTmp;
        }

        // on elargi la notion de triangle pour eviter de creer
        // des surfaces ecraser avec deux coins proches
        // attention ceci entraine un effet de seuil
        if (cornerAllSame)
        {
            c1triangle = (distDebFin < 0.3 * distMin);
        }

        if (c1triangle)
        {
            p3d[pivot] = pSom;
        }

        // on calcule la surface portant le coin
        //--------------------------------------
        // Si c1triangle ou les 4 points p3d sont coplanaires, alors
        // le chanfrein est porte par le plan passant par les 3  premiers p3d.
        // Sinon, on construit le chanfrein par la methode GeomFill_ConstrainedFilling
        bool c1plan = c1triangle;
        Vector3D v1 = new(p3d[pivot], p3d[deb]);
        Vector3D v2 = new(p3d[pivot], p3d[fin]);
        Vector3D nor = v1.Crossed(v2);

        _done = false;

        int icf = 0, icl = 0;
        Curve2D? debpc1 = null, finpc1 = null;

        if (!c1triangle)
        {
            c1plan = CoPlanar(p3d[0], p3d[1], p3d[2], p3d[3]);
        }

        if (c1plan)
        {
            // c1plan
            //-------

            // on construit le plan
            Direction3D ndir = new(nor);
            //    gp_Dir xdir(gp_Vec(p3d[fin],p3d[deb]));
            Direction3D xdir = new Direction3D(new Vector3D(p3d[fin], p3d[deb]));
            CoordinateSystem planAx3 = new(p3d[pivot], ndir, xdir);
            if (planAx3.YDirection().Dot(new Direction3D(v1)) <= 0.0)
            {
                planAx3 = planAx3.YReversed();
            }
            PlaneSurface? gpl = new PlaneSurface(planAx3);
            coin.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gpl, dStr));

            // on oriente coin
            Vector3D norface = norpl;
            if (face[pivot].Orientation() == Reversed)
            {
                norface = norface.Reversed();
            }
            Vector3D norcoin = new(gpl.Pln().Position().XDirection().Crossed(gpl.Pln().Position().YDirection()));
            if (norcoin.Dot(norface) <= 0.0)
            {
                coin.ChangeOrientation() = Reversed;
            }
            else
            {
                coin.ChangeOrientation() = Forward;
            }

            // on calcule les intersections
            Curve3D? gcpiv = null, gcdeb = null, gcfin = null;
            TrimmedCurve3D? gcface = null;
            Curve2D? pivpc1 = null, pivpc2 = null, debpc2 = null, finpc2 = null, facepc1 = null, facepc2 = null;
            Point2D ptbid = new();

            // intersection coin-pivot
            double tolrcoinpiv = 0.0;
            if (!ComputeIntersection(dStr, fdpiv, coin, p3d[fin], p2d[fin], p3d[deb], p2d[deb], ref gcpiv, ref pivpc1, ref pivpc2,
                                     ref deru, ref derv, ref ptbid, _tolapp3d, _tol2d, ref tolrcoinpiv))
            {
                throw new NotDoneException("echec calcul intersection coin-pivot");
            }
            Vector3D norpiv = deru.Crossed(derv);

            // intersection coin-deb
            double tolrcoindeb = 0.0;
            Point2D p2d1 = new(), p2d2 = new();
            if (c1triangle)
            {
                p2d1 = fddeb.Interference(jf[deb, fin]).PCurveOnSurf().Value(p[deb, fin]);
            }
            else
            {
                p2d1 = fddeb.Interference(jf[deb, fin]).PCurveOnSurf().Value(p[deb, pivot]);
            }

            p2d2 = fddeb.Interference(jf[deb, pivot]).PCurveOnSurf().Value(p[deb, pivot]);

            if (!ComputeIntersection(dStr, fddeb, coin, p3d[pivot], p2d1, p3d[fin], p2d2, ref gcdeb, ref debpc1,
                                     ref debpc2, ref deru, ref derv, ref ptbid, _tolapp3d, _tol2d, ref tolrcoindeb))
            {
                throw new NotDoneException("echec calcul intersection coin-deb");
            }
            icf = dStr.AddCurve(new DsCurve(gcdeb, tolrcoindeb));

            // intersection coin-fin
            double tolrcoinfin = 0.0;
            Point3D p3dface = new();
            if (c1triangle)
            {
                p3dface = p3d[pivot];
                p2d1 = fdfin.Interference(jf[fin, deb]).PCurveOnSurf().Value(p[fin, deb]);
            }
            else
            {
                p3dface = p3d[3];
                p2d1 = fdfin.Interference(jf[fin, deb]).PCurveOnSurf().Value(p[fin, pivot]);
            }
            p2d2 = fdfin.Interference(jf[fin, pivot]).PCurveOnSurf().Value(p[fin, pivot]);
            if (!ComputeIntersection(dStr, fdfin, coin, p3dface, p2d1, p3d[deb], p2d2, ref gcfin, ref finpc1,
                                     ref finpc2, ref deru, ref derv, ref ptbid, _tolapp3d, _tol2d, ref tolrcoinfin))
            {
                throw new NotDoneException("echec calcul intersection coin-face");
            }
            icl = dStr.AddCurve(new DsCurve(gcfin, tolrcoinfin));

            //! c1triangle: intersection coin-face[pivot]
            if (!c1triangle)
            {
                IntSS inter = new();
                FaceSurfaceAdaptor facebid = new(face[pivot]);
                Surface? surfbid = facebid.GeomSurfaceTransformed();
                inter.Perform(gpl, surfbid, global::Kerf.Engine.Foundation.Precision.Intersection());
                if (inter.IsDone)
                {
                    int nbl = inter.LineCount;
                    if (nbl > 1)
                    {
                    }
                    else if (nbl == 1)
                    {
                        ChFi3d_TrimCurve(inter.Line(1), p3d[pivot], p3dface, out gcface);

                        CurveAdaptor? gac = new CurveAdaptor();
                        gac.Load(gcface);
                        SurfaceAdaptor? gas = new SurfaceAdaptor();
                        gas.Load(gpl);
                        FaceSurfaceAdaptor? gaf = new FaceSurfaceAdaptor();
                        gaf.Initialize(face[pivot]);

                        double tolr = 0.0;
                        ChFi3d_ProjectPCurv(gac, gaf, ref facepc1, _tolapp3d, ref tolr);
                        ChFi3d_ProjectPCurv(gac, gas, ref facepc2, _tolapp3d, ref tolr);
                    }
                }
            }

            // on remplit les donnees du coin oriente face-pivot
            Orientation trans = default;

            // avec les CommonPoints
            coin.ChangeVertexFirstOnS1().SetPoint(p3d[pivot]);
            coin.ChangeVertexFirstOnS2().SetPoint(p3d[fin]);
            if (c1triangle)
            {
                coin.ChangeVertexLastOnS1().SetPoint(p3d[pivot]);
            }
            else
            {
                coin.ChangeVertexLastOnS1().SetPoint(p3d[3]);
            }
            coin.ChangeVertexLastOnS2().SetPoint(p3d[deb]);

            // avec les FaceInterference
            //    int Igcpiv,Igcdeb,Igcfin,Igcface;
            int igcpiv = 0, igcface = 0;
            ref FaceInterference fi1 = ref coin.ChangeInterferenceOnS1();
            ref FaceInterference fi2 = ref coin.ChangeInterferenceOnS2();

            // sur face[pivot]
            if (norcoin.Dot(norpl) <= 0.0)
            {
                trans = Forward;
            }
            else
            {
                trans = Reversed;
            }
            Curve2D? bidpc = null;
            if (c1triangle)
            {
                fi1.SetInterference(0, trans, bidpc, bidpc);
            }
            else
            {
                igcface = ChFiKPart_IndexCurveInDS(gcface, dStr);
                fi1.SetInterference(igcface, trans, facepc1, facepc2);
                fi1.SetFirstParameter(gcface.FirstParameter());
                fi1.SetLastParameter(gcface.LastParameter());
            }
            // sur le pivot
            if (norcoin.Dot(norpiv) <= 0.0)
            {
                trans = Reversed;
            }
            else
            {
                trans = Forward;
            }
            igcpiv = ChFiKPart_IndexCurveInDS(gcpiv, dStr);
            fi2.SetInterference(igcpiv, trans, pivpc1, pivpc2);
            fi2.SetFirstParameter(gcpiv.FirstParameter());
            fi2.SetLastParameter(gcpiv.LastParameter());

            _done = true;
        }
        else
        {
            // !c1plan
            //--------

            Surface? surfcoin = null;
            Curve2D? pCurveOnFace = null, pCurveOnPiv = null;

            // le contour a remplir est constitue de courbes isos sur deb et fin
            // de deux pcurves calculees sur piv et la face opposee.
            Boundary? bdeb = null, bfin = null, bpiv = null, bfac = null;
            int ind1 = fddeb.Interference(jf[deb, pivot]).LineIndex();
            int ind2 = fdfin.Interference(jf[fin, pivot]).LineIndex();
            Point3D pfin = new(), pdeb = new();
            Vector3D vpfin = new(), vpdeb = new();

            dStr.Curve(ind1).Curve().D1(p[deb, pivot], out pfin, out vpfin);
            dStr.Curve(ind2).Curve().D1(p[fin, pivot], out pdeb, out vpdeb);

            if (issmooth)
            {
                // les bords de coin sont des lignes courbes qui suivent les
                // tangentes donnees
                bfac = ChFi3d_mkbound(fac, out pCurveOnFace, sens[deb], p2d[pivot], tgpiv, sens[fin], p2d[3], tg3, _tolapp3d,
                                                            2.0e-4);
                bpiv = ChFi3d_mkbound(surf, out pCurveOnPiv, sens[deb], p2d[fin], ref vpfin, sens[fin], p2d[deb], ref vpdeb, _tolapp3d,
                                                            2.0e-4);
            }
            else
            {
                // les bords de coin sont des segments
                //      Bfac = ChFi3d_mkbound(Fac,PCurveOnFace,p2d[pivot],
                //			    p2d[3],tolesp,2.e-4);
                bfac = ChFi3d_mkbound(fac, out pCurveOnFace, p2d[pivot], p2d[3], _tolapp3d, 2.0e-4);
                bpiv = ChFi3d_mkbound(surf, out pCurveOnPiv, p2d[fin], p2d[deb], _tolapp3d, 2.0e-4);
            }

            Point2D pdeb1 = fddeb.Interference(jf[deb, pivot]).PCurveOnSurf().Value(p[deb, pivot]);
            Point2D pdeb2 = fddeb.Interference(jf[deb, fin]).PCurveOnSurf().Value(p[deb, pivot]);
            Point2D pfin1 = fdfin.Interference(jf[fin, pivot]).PCurveOnSurf().Value(p[fin, pivot]);
            Point2D pfin2 = fdfin.Interference(jf[fin, deb]).PCurveOnSurf().Value(p[fin, pivot]);

            if (issmooth)
            {
                // il faut homogeneiser, mettre les bords "BoundWithSurf"
                bdeb = ChFi3d_mkbound(dStr.Surface(fddeb.Surf()).Surface(), pdeb1, pdeb2, _tolapp3d, 2.0e-4);
                bfin = ChFi3d_mkbound(dStr.Surface(fdfin.Surf()).Surface(), pfin1, pfin2, _tolapp3d, 2.0e-4);
            }
            else
            {
                // ou les 4 bords de type "FreeBoundary"
                bdeb =
                    ChFi3d_mkbound(dStr.Surface(fddeb.Surf()).Surface(), pdeb1, pdeb2, _tolapp3d, 2.0e-4, true);
                bfin =
                    ChFi3d_mkbound(dStr.Surface(fdfin.Surf()).Surface(), pfin1, pfin2, _tolapp3d, 2.0e-4, true);
            }
            ConstrainedFilling fil = new(8, 20);
            fil.Init(bpiv, bfin, bfac, bdeb);

            surfcoin = fil.InternalSurface();
            // on se ramene au sens face surf: S1 = face, S2 = surf
            surfcoin.VReverse();

            _done = CompleteData(coin, surfcoin, fac, pCurveOnFace, surf, pCurveOnPiv, fdpiv.Orientation(), false, false,
                                                    false, false, false);
        }
        double p1deb = 0.0, p2deb = 0.0, p1fin = 0.0, p2fin = 0.0;

        if (_done)
        {
            int if1 = 0, if2 = 0, il1 = 0, il2 = 0;

            // Mise a jour des 4 Stripes et de la DS
            // -------------------------------------

            CommonPoint pf1 = coin.VertexFirstOnS1();
            CommonPoint pf2 = coin.VertexFirstOnS2();
            ref CommonPoint pl1 = ref coin.ChangeVertexLastOnS1();
            if (c1triangle)
            {
                pl1 = coin.ChangeVertexFirstOnS1();
            }
            CommonPoint pl2 = coin.VertexLastOnS2();

            // le coin pour commencer,
            // -----------------------
            Regul regdeb = new(), regfin = new();
            if1 = ChFi3d_IndexPointInDS(pf1, dStr);
            if2 = ChFi3d_IndexPointInDS(pf2, dStr);
            if (c1triangle)
            {
                il1 = if1;
            }
            else
            {
                il1 = ChFi3d_IndexPointInDS(pl1, dStr);
            }
            il2 = ChFi3d_IndexPointInDS(pl2, dStr);

            coin.ChangeIndexOfS1(dStr.AddShape(face[pivot]));
            coin.ChangeIndexOfS2(-fdpiv.Surf());

            // first points
            Point2D pp1 = new(), pp2 = new();
            if (c1plan)
            {
                p1deb = dStr.Curve(icf).Curve().FirstParameter();
                p2deb = dStr.Curve(icf).Curve().LastParameter();
            }
            else
            {
                pp1 =
                    coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().FirstParameter());
                pp2 =
                    coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().FirstParameter());
                Curve3D? c3d = null;
                double tolreached = 0.0;
                ChFi3d_ComputeArete(pf1, pp1, pf2, pp2, dStr.Surface(coin.Surf()).Surface(), ref c3d,
                                                        ref corner.ChangeFirstPCurve(), ref p1deb, ref p2deb, _tolapp3d, _tol2d, out tolreached, 0);
                DsCurve tcurv = new(c3d, tolreached);
                icf = dStr.AddCurve(tcurv);
            }

            regdeb.SetCurve(icf);
            regdeb.SetS1(coin.Surf(), false);
            regdeb.SetS2(fddeb.Surf(), false);
            _regul.Append(regdeb);
            corner.ChangeFirstCurve(icf);
            corner.ChangeFirstParameters(p1deb, p2deb);
            corner.ChangeIndexFirstPointOnS1(if1);
            corner.ChangeIndexFirstPointOnS2(if2);

            // last points
            if (c1plan)
            {
                p1fin = dStr.Curve(icl).Curve().FirstParameter();
                p2fin = dStr.Curve(icl).Curve().LastParameter();
            }
            else
            {
                pp1 =
                    coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().LastParameter());
                pp2 =
                    coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().LastParameter());
                Curve3D? c3d = null;
                double tolreached = 0.0;
                ChFi3d_ComputeArete(pl1, pp1, pl2, pp2, dStr.Surface(coin.Surf()).Surface(), ref c3d,
                                                        ref corner.ChangeLastPCurve(), ref p1fin, ref p2fin, _tolapp3d, _tol2d, out tolreached, 0);
                DsCurve tcurv = new(c3d, tolreached);
                icl = dStr.AddCurve(tcurv);
            }
            regfin.SetCurve(icl);
            regfin.SetS1(coin.Surf(), false);
            regfin.SetS2(fdfin.Surf(), false);
            _regul.Append(regfin);
            corner.ChangeLastCurve(icl);
            corner.ChangeLastParameters(p1fin, p2fin);
            corner.ChangeIndexLastPointOnS1(il1);
            corner.ChangeIndexLastPointOnS2(il2);

            // puis la CornerData du debut,
            // ----------------------------
            bool isfirst = (sens[deb] == 1), rev = (jf[deb, fin] == 2);
            int isurf1 = 1, isurf2 = 2;
            double par = p[deb, pivot], par2 = p[deb, pivot];
            if (c1triangle)
            {
                par2 = p[deb, fin];
            }
            if (rev)
            {
                isurf1 = 2;
                isurf2 = 1;
                cd[deb].SetOrientation(Reversed, isfirst);
            }
            cd[deb].SetCurve(icf, isfirst);
            cd[deb].SetIndexPoint(if1, isfirst, isurf1);
            cd[deb].SetIndexPoint(if2, isfirst, isurf2);
            cd[deb].SetParameters(isfirst, p1deb, p2deb);
            fddeb.ChangeVertex(isfirst, isurf1) = pf1;
            fddeb.ChangeVertex(isfirst, isurf2) = pf2;
            fddeb.ChangeInterference(isurf1).SetParameter(par2, isfirst);
            fddeb.ChangeInterference(isurf2).SetParameter(par, isfirst);
            if (c1plan)
            {
                cd[deb].ChangePCurve(isfirst) = debpc1;
            }
            else
            {
                pp1 = fddeb.InterferenceOnS1().PCurveOnSurf().Value(par);
                pp2 = fddeb.InterferenceOnS2().PCurveOnSurf().Value(par);
                ChFi3d_ComputePCurv(pp1, pp2, out cd[deb].ChangePCurve(isfirst), p1deb, p2deb, rev);
            }

            // puis la CornerData de la fin,
            // -----------------------------
            isfirst = (sens[fin] == 1);
            rev = (jf[fin, deb] == 2);
            isurf1 = 1;
            isurf2 = 2;
            par = p[fin, pivot];
            par2 = p[fin, pivot];
            if (c1triangle)
            {
                par2 = p[fin, deb];
            }
            if (rev)
            {
                isurf1 = 2;
                isurf2 = 1;
                cd[fin].SetOrientation(Reversed, isfirst);
            }
            cd[fin].SetCurve(icl, isfirst);
            cd[fin].SetIndexPoint(il1, isfirst, isurf1);
            cd[fin].SetIndexPoint(il2, isfirst, isurf2);
            cd[fin].SetParameters(isfirst, p1fin, p2fin);
            fdfin.ChangeVertex(isfirst, isurf1) = pl1;
            fdfin.ChangeVertex(isfirst, isurf2) = pl2;
            fdfin.ChangeInterference(isurf1).SetParameter(par2, isfirst);
            fdfin.ChangeInterference(isurf2).SetParameter(par, isfirst);
            if (c1plan)
            {
                cd[fin].ChangePCurve(isfirst) = finpc1;
            }
            else
            {
                pp1 = fdfin.InterferenceOnS1().PCurveOnSurf().Value(par);
                pp2 = fdfin.InterferenceOnS2().PCurveOnSurf().Value(par);
                ChFi3d_ComputePCurv(pp1, pp2, out cd[fin].ChangePCurve(isfirst), p1fin, p2fin, rev);
            }

            // et enfin le pivot.
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
            cd[pivot].SetCurve(fi.LineIndex(), isfirst);
            cd[pivot].ChangePCurve(isfirst) = fi.PCurveOnFace();
            cd[pivot].SetIndexPoint(if2, isfirst, isurf1);
            cd[pivot].SetIndexPoint(il2, isfirst, isurf2);
            cd[pivot].SetParameters(isfirst, fi.FirstParameter(), fi.LastParameter());
            fdpiv.ChangeVertex(isfirst, isurf1) = pf2;
            fdpiv.ChangeVertex(isfirst, isurf2) = pl2;
            fdpiv.ChangeInterference(isurf1).SetParameter(p[pivot, deb], isfirst);
            fdpiv.ChangeInterference(isurf2).SetParameter(p[pivot, fin], isfirst);
            cd[pivot].InDS(isfirst); // filDS fait deja le boulot depuis le coin.
        }

        // On tronque les corners data et met a jour les index.
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
        // il faudra ici tenir compte des coins mutants.
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

internal static class ChBuilder_C3
{
    internal static bool CoPlanar(in Point3D pntA, in Point3D pntB, in Point3D pntC, in Point3D pntD)
    {
        Vector3D vecAB = new(pntA, pntB);
        Vector3D vecAC = new(pntA, pntC);
        Vector3D vecAD = new(pntA, pntD);

        double nor2AB = vecAB.SquareMagnitude();
        double nor2AC = vecAC.SquareMagnitude();
        double proABAC = vecAB.Dot(vecAC);

        double alpha = nor2AB * nor2AC - proABAC * proABAC;

        if (alpha < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return true;
        }

        double proABAD = vecAB.Dot(vecAD);
        double proACAD = vecAC.Dot(vecAD);
        double alpha1 = proABAD * nor2AC - proABAC * proACAD;
        double alpha2 = proACAD * nor2AB - proABAC * proABAD;
        Vector3D vecDABC = alpha1 * vecAB + alpha2 * vecAC - alpha * vecAD;

        return (vecDABC.Magnitude() / alpha) < global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    internal static SurfaceAdaptor? BoundSurf(Surface? s, in Point2D pdeb, in Point2D pfin)
    {
        SurfaceAdaptor? hs = new SurfaceAdaptor();
        SurfaceAdaptor gas = hs;
        gas.Load(s);

        double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
        double uuu1 = 0.0, uuu2 = 0.0, vvv1 = 0.0, vvv2 = 0.0;
        s.Bounds(out uuu1, out uuu2, out vvv1, out vvv2);
        ChFi3d_Boite(pdeb, pfin, out uu1, out uu2, out vv1, out vv2);
        double step = Max((uu2 - uu1), (vv2 - vv1));
        step *= 0.2;
        uuu1 = Max((uu1 - step), uuu1);
        uuu2 = Min((uu2 + step), uuu2);
        vvv1 = Max((vv1 - step), vvv1);
        vvv2 = Min((vv2 + step), vvv2);
        gas.Load(s, uuu1, uuu2, vvv1, vvv2);
        return hs;
    }

    internal static bool ComputeIntersection(DsDataStructure dStr, SurfData? sd, SurfData? sdCoin, in Point3D pdeb, in Point2D p2ddeb, in Point3D pfin, in Point2D p2dfin, ref Curve3D? gc, ref Curve2D? pc1, ref Curve2D? pc2, ref Vector3D derudeb,
                                             ref Vector3D dervdeb, ref Point2D ptcoindeb, double tol3d, double tol2d,
                                             ref double tolreached)
    {
        //  gp_Pnt2d UVf1,UVf2,UVl1,UVl2;

        // take the surface of the pivot SurfData and trim it to allow
        // the intersection computation if it's an analytic surface
        SurfaceAdaptor? hs1 = null;
        hs1 = ChFi3d_BoundSurf(dStr, sd, 1, 2);

        Surface? gpl = dStr.Surface(sdCoin.Surf()).Surface();
        Surface? gSD = dStr.Surface(sd.Surf()).Surface();

        // compute pardeb
        NCollection_Array1<double> pardeb = new(1, 4), parfin = new(1, 4);
        double u = 0.0, v = 0.0;
        Point3D pbidon = new();
        u = p2ddeb.X();
        v = p2ddeb.Y();
        gSD.D1(u, v, out pbidon, out derudeb, out dervdeb);

        pardeb[1] = u;
        pardeb[2] = v;
        //  gp_Pnt2d pd2(u,v);

        ChFi3d_Parameters(gpl, pdeb, out u, out v);
        pardeb[3] = u;
        pardeb[4] = v;
        ptcoindeb = ptcoindeb.WithCoord(u, v);

        // compute parfin
        u = p2dfin.X();
        v = p2dfin.Y();
        parfin[1] = u;
        parfin[2] = v;
        //  gp_Pnt2d pf2(u,v);

        ChFi3d_Parameters(gpl, pfin, out u, out v);
        parfin[3] = u;
        parfin[4] = v;
        Point2D cpf2 = new(u, v);

        // Trims the chamfer surface to allow the intersection computation
        // and computes a GeomAdaptor_Surface for using the ComputeCurves
        // function
        SurfaceAdaptor? hs2 = null;
        hs2 = BoundSurf(gpl, ptcoindeb, cpf2);

        // compute the intersection curves and pcurves
        return ChFi3d_ComputeCurves(hs1, hs2, pardeb, parfin, ref gc, ref pc1, ref pc2, tol3d, tol2d, out tolreached);
    }


}
