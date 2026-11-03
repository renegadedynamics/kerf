// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_ChBuilder_C3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFi3d_Builder_0;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.ChFi3d_ChBuilder_C3;
using static Occt.ChFiDS_ErrorStatus;
using static Occt.ChFiDS_State;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;
public partial class ChFi3d_ChBuilder
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

    internal override void PerformThreeCorner(int Jndex)
    {

        // modifier pour le passer en option dans le cdl!!!!!!!!!!!!
        bool issmooth = false;

        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        Vertex Vtx = myVDataMap.FindKey(Jndex);
        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        //  int Index[3],pivot,deb,fin,ii,jj,kk;
        int[] Index = new int[3];
        int pivot = 0, deb = 0, fin = 0, ii = 0;
        ChFiDS_Stripe?[] CD = new ChFiDS_Stripe?[3];
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

        for (It.Initialize(myVDataMap[Jndex]), ii = 0; It.More() && ii < 3; It.Next(), ii++)
        {
            Index[ii] = ChFi3d_IndexOfSurfData(Vtx, It.Value(), out sens[ii]);
            CD[ii] = It.Value();
        }
        // On verifie que l une des CD ne figure pas deux fois, au quel cas
        // il faut modifier le retour de IndexOfSurfData qui prend la
        // premiere des solutions.
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
        oksea[2] = ChFi3d_SearchFD(DStr, CD[0], CD[1], sens[0], sens[1], ref i[0, 1], ref i[1, 0], ref p[0, 1], ref p[1, 0],
                                                              Index[0], Index[1], ref face[2], ref sameside[2], ref jf[0, 1], ref jf[1, 0]);
        oksea[1] = ChFi3d_SearchFD(DStr, CD[0], CD[2], sens[0], sens[2], ref i[0, 2], ref i[2, 0], ref p[0, 2], ref p[2, 0],
                                                              Index[0], Index[2], ref face[1], ref sameside[1], ref jf[0, 2], ref jf[2, 0]);
        oksea[0] = ChFi3d_SearchFD(DStr, CD[1], CD[2], sens[1], sens[2], ref i[1, 2], ref i[2, 1], ref p[1, 2], ref p[2, 1],
                                                              Index[1], Index[2], ref face[0], ref sameside[0], ref jf[1, 2], ref jf[2, 1]);
        //
        // Analyse des concavites des 3 chanfreins :
        //        - 2 concavites identiques et 1 inverse.
        //        - 3 concavites identiques
        //
        bool CornerAllSame = false;
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
                okinter = ChFi3d_IsInFront(DStr, CD[1], CD[2], i[1, 2], i[2, 1], sens[1], sens[2], ref p[1, 2], ref p[2, 1],
                                                                      ref face[0], ref sameside[0], ref jf[1, 2], ref jf[2, 1], ref visavis, Vtx, false,
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
                okinter = ChFi3d_IsInFront(DStr, CD[0], CD[2], i[0, 2], i[2, 0], sens[0], sens[2], ref p[0, 2], ref p[2, 0],
                                                                      ref face[1], ref sameside[1], ref jf[0, 2], ref jf[2, 0], ref visavis, Vtx, false,
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
                okinter = ChFi3d_IsInFront(DStr, CD[0], CD[1], i[0, 1], i[1, 0], sens[0], sens[1], ref p[0, 1], ref p[1, 0],
                                                                      ref face[2], ref sameside[2], ref jf[0, 1], ref jf[1, 0], ref visavis, Vtx, false,
                                                                      true);
            }
        }
        else if (oksea[0] && oksea[1] && oksea[2])
        {
            // 3 concavites identiques.
            pivot = ChFi3d_SearchPivot(sens, p, tol2d);
            if (pivot < 0)
            {
                // on prend un pivot au hasard!!!!!!!!!!!!!!!
                pivot = 0;
            }
            deb = (pivot + 1) % 3;
            fin = (pivot + 2) % 3;
            CornerAllSame = true;
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
        ChFiDS_SurfData? fddeb = CD[deb].ChangeSetOfSurfData().ChangeValue(i[deb, pivot]);
        ChFiDS_SurfData? fdfin = CD[fin].ChangeSetOfSurfData().ChangeValue(i[fin, pivot]);
        ChFiDS_SurfData? fdpiv = CD[pivot].ChangeSetOfSurfData().ChangeValue(i[pivot, deb]);

        // On construit les HSurfaces et autres outils qui vont bien.
        // ----------------------------------------------------------

        FaceSurfaceAdaptor? Fac = new FaceSurfaceAdaptor(face[pivot]);
        SurfaceAdaptor? bidsurf = (SurfaceAdaptor)Fac.AdaptorSurfaceOriginal().ShallowCopy(); // OCCT's copy
        TopolTool? IFac = new TopolTool(bidsurf);

        SurfaceAdaptor? Surf = 
            ChFi3d_BoundSurf(DStr, fdpiv, jf[pivot, deb], jf[pivot, fin]);
        TopolTool? ISurf = new TopolTool(Surf);

        // Creation of a new Stripe for the corner
        ChFiDS_Stripe? corner = new ChFiDS_Stripe();
        ref NCollection_HSequence<ChFiDS_SurfData>? cornerset = ref 
            corner.ChangeSetOfSurfData();
        cornerset = new NCollection_HSequence<ChFiDS_SurfData>();
        ChFiDS_SurfData? coin = new ChFiDS_SurfData();
        cornerset.Append(coin);

        // Pour plus de surete, on verifie les intersections des pcurves des chanfreins sur leur
        // face commune
        SurfaceAdaptor? HSdeb = new SurfaceAdaptor(DStr.Surface(fddeb.Surf()).Surface());
        SurfaceAdaptor? HSfin = new SurfaceAdaptor(DStr.Surface(fdfin.Surf()).Surface());
        SurfaceAdaptor? HSpiv = new SurfaceAdaptor(DStr.Surface(fdpiv.Surf()).Surface());

        Point2D[] p2d = [new(), new(), new(), new()];
        Point3D[] p3d = [new(), new(), new(), new()];
        Point3D PSom = new();

        ChFi3d_ComputesIntPC(fdpiv.Interference(jf[pivot, deb]), fddeb.Interference(jf[deb, pivot]), HSpiv, HSdeb,
                                                  ref p[pivot, deb], ref p[deb, pivot], ref p3d[fin]);
        ChFi3d_ComputesIntPC(fdpiv.Interference(jf[pivot, fin]), fdfin.Interference(jf[fin, pivot]), HSpiv, HSfin,
                                                  ref p[pivot, fin], ref p[fin, pivot], ref p3d[deb]);
        ChFi3d_ComputesIntPC(fddeb.Interference(jf[deb, fin]), fdfin.Interference(jf[fin, deb]), HSdeb, HSfin,
                                                  ref p[deb, fin], ref p[fin, deb], ref PSom);

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

        Vector2D Tg3 = new(), Tgpiv = new();

        //  if (c1triangle)
        //    p2d[pivot] = fddeb->Interference(jf[deb, fin]).PCurveOnFace()->Value(p[deb, pivot]);
        //  else {
        if (issmooth)
        {
            fddeb.Interference(jf[deb, fin]).PCurveOnFace().D1(p[deb, pivot], out p2d[pivot], out Tgpiv);
            fdfin.Interference(jf[fin, deb]).PCurveOnFace().D1(p[fin, pivot], out p2d[3], out Tg3);
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
        Fac.D1(p2d[pivot].X(), p2d[pivot].Y(), out p3d[pivot], out deru, out derv);
        Vector3D norpl = deru.Crossed(derv);
        //  if (!c1triangle)
        p3d[3] = Fac.Value(p2d[3].X(), p2d[3].Y());

        double DistMin = (p3d[3]).Distance(p3d[fin]);
        double DistTmp = (p3d[pivot]).Distance(p3d[deb]);
        double DistDebFin = (p3d[pivot]).Distance(p3d[3]);

        if (DistTmp > DistMin)
        {
            DistMin = DistTmp;
        }

        // on elargi la notion de triangle pour eviter de creer
        // des surfaces ecraser avec deux coins proches
        // attention ceci entraine un effet de seuil
        if (CornerAllSame)
        {
            c1triangle = (DistDebFin < 0.3 * DistMin);
        }

        if (c1triangle)
        {
            p3d[pivot] = PSom;
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

        done = false;

        int Icf = 0, Icl = 0;
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
                planAx3.YReverse();
            }
            PlaneSurface? gpl = new PlaneSurface(planAx3);
            coin.ChangeSurf(ChFiKPart_IndexSurfaceInDS(gpl, DStr));

            // on oriente coin
            Vector3D norface = norpl;
            if (face[pivot].Orientation() == Reversed)
            {
                norface.Reverse();
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
            if (!ComputeIntersection(DStr, fdpiv, coin, p3d[fin], p2d[fin], p3d[deb], p2d[deb], ref gcpiv, ref pivpc1, ref pivpc2,
                                     ref deru, ref derv, ref ptbid, tolapp3d, tol2d, ref tolrcoinpiv))
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

            if (!ComputeIntersection(DStr, fddeb, coin, p3d[pivot], p2d1, p3d[fin], p2d2, ref gcdeb, ref debpc1,
                                     ref debpc2, ref deru, ref derv, ref ptbid, tolapp3d, tol2d, ref tolrcoindeb))
            {
                throw new NotDoneException("echec calcul intersection coin-deb");
            }
            Icf = DStr.AddCurve(new TopOpeBRepDS_Curve(gcdeb, tolrcoindeb));

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
            if (!ComputeIntersection(DStr, fdfin, coin, p3dface, p2d1, p3d[deb], p2d2, ref gcfin, ref finpc1,
                                     ref finpc2, ref deru, ref derv, ref ptbid, tolapp3d, tol2d, ref tolrcoinfin))
            {
                throw new NotDoneException("echec calcul intersection coin-face");
            }
            Icl = DStr.AddCurve(new TopOpeBRepDS_Curve(gcfin, tolrcoinfin));

            //! c1triangle: intersection coin-face[pivot]
            if (!c1triangle)
            {
                GeomInt_IntSS inter = new();
                FaceSurfaceAdaptor facebid = new(face[pivot]);
                Surface? surfbid = facebid.GeomSurfaceTransformed();
                inter.Perform(gpl, surfbid, global::Kerf.Engine.Foundation.Precision.Intersection());
                if (inter.IsDone())
                {
                    int nbl = inter.NbLines();
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
                        ChFi3d_ProjectPCurv(gac, gaf, ref facepc1, tolapp3d, ref tolr);
                        ChFi3d_ProjectPCurv(gac, gas, ref facepc2, tolapp3d, ref tolr);
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
            int Igcpiv = 0, Igcface = 0;
            ref ChFiDS_FaceInterference fi1 = ref coin.ChangeInterferenceOnS1();
            ref ChFiDS_FaceInterference fi2 = ref coin.ChangeInterferenceOnS2();

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
                Igcface = ChFiKPart_IndexCurveInDS(gcface, DStr);
                fi1.SetInterference(Igcface, trans, facepc1, facepc2);
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
            Igcpiv = ChFiKPart_IndexCurveInDS(gcpiv, DStr);
            fi2.SetInterference(Igcpiv, trans, pivpc1, pivpc2);
            fi2.SetFirstParameter(gcpiv.FirstParameter());
            fi2.SetLastParameter(gcpiv.LastParameter());

            done = true;
        }
        else
        {
            // !c1plan
            //--------

            Surface? Surfcoin = null;
            Curve2D? PCurveOnFace = null, PCurveOnPiv = null;

            // le contour a remplir est constitue de courbes isos sur deb et fin
            // de deux pcurves calculees sur piv et la face opposee.
            GeomFill_Boundary? Bdeb = null, Bfin = null, Bpiv = null, Bfac = null;
            int ind1 = fddeb.Interference(jf[deb, pivot]).LineIndex();
            int ind2 = fdfin.Interference(jf[fin, pivot]).LineIndex();
            Point3D Pfin = new(), Pdeb = new();
            Vector3D vpfin = new(), vpdeb = new();

            DStr.Curve(ind1).Curve().D1(p[deb, pivot], out Pfin, out vpfin);
            DStr.Curve(ind2).Curve().D1(p[fin, pivot], out Pdeb, out vpdeb);

            if (issmooth)
            {
                // les bords de coin sont des lignes courbes qui suivent les
                // tangentes donnees
                Bfac = ChFi3d_mkbound(Fac, out PCurveOnFace, sens[deb], p2d[pivot], Tgpiv, sens[fin], p2d[3], Tg3, tolapp3d,
                                                            2.0e-4);
                Bpiv = ChFi3d_mkbound(Surf, out PCurveOnPiv, sens[deb], p2d[fin], ref vpfin, sens[fin], p2d[deb], ref vpdeb, tolapp3d,
                                                            2.0e-4);
            }
            else
            {
                // les bords de coin sont des segments
                //      Bfac = ChFi3d_mkbound(Fac,PCurveOnFace,p2d[pivot],
                //			    p2d[3],tolesp,2.e-4);
                Bfac = ChFi3d_mkbound(Fac, out PCurveOnFace, p2d[pivot], p2d[3], tolapp3d, 2.0e-4);
                Bpiv = ChFi3d_mkbound(Surf, out PCurveOnPiv, p2d[fin], p2d[deb], tolapp3d, 2.0e-4);
            }

            Point2D pdeb1 = fddeb.Interference(jf[deb, pivot]).PCurveOnSurf().Value(p[deb, pivot]);
            Point2D pdeb2 = fddeb.Interference(jf[deb, fin]).PCurveOnSurf().Value(p[deb, pivot]);
            Point2D pfin1 = fdfin.Interference(jf[fin, pivot]).PCurveOnSurf().Value(p[fin, pivot]);
            Point2D pfin2 = fdfin.Interference(jf[fin, deb]).PCurveOnSurf().Value(p[fin, pivot]);

            if (issmooth)
            {
                // il faut homogeneiser, mettre les bords "BoundWithSurf"
                Bdeb = ChFi3d_mkbound(DStr.Surface(fddeb.Surf()).Surface(), pdeb1, pdeb2, tolapp3d, 2.0e-4);
                Bfin = ChFi3d_mkbound(DStr.Surface(fdfin.Surf()).Surface(), pfin1, pfin2, tolapp3d, 2.0e-4);
            }
            else
            {
                // ou les 4 bords de type "FreeBoundary"
                Bdeb =
                    ChFi3d_mkbound(DStr.Surface(fddeb.Surf()).Surface(), pdeb1, pdeb2, tolapp3d, 2.0e-4, true);
                Bfin =
                    ChFi3d_mkbound(DStr.Surface(fdfin.Surf()).Surface(), pfin1, pfin2, tolapp3d, 2.0e-4, true);
            }
            GeomFill_ConstrainedFilling fil = new(8, 20);
            fil.Init(Bpiv, Bfin, Bfac, Bdeb);

            Surfcoin = fil.InternalSurface();
            // on se ramene au sens face surf: S1 = face, S2 = surf
            Surfcoin.VReverse();

            done = CompleteData(coin, Surfcoin, Fac, PCurveOnFace, Surf, PCurveOnPiv, fdpiv.Orientation(), false, false,
                                                    false, false, false);
        }
        double P1deb = 0.0, P2deb = 0.0, P1fin = 0.0, P2fin = 0.0;

        if (done)
        {
            int If1 = 0, If2 = 0, Il1 = 0, Il2 = 0;

            // Mise a jour des 4 Stripes et de la DS
            // -------------------------------------

            ChFiDS_CommonPoint Pf1 = coin.VertexFirstOnS1();
            ChFiDS_CommonPoint Pf2 = coin.VertexFirstOnS2();
            ref ChFiDS_CommonPoint Pl1 = ref coin.ChangeVertexLastOnS1();
            if (c1triangle)
            {
                Pl1 = coin.ChangeVertexFirstOnS1();
            }
            ChFiDS_CommonPoint Pl2 = coin.VertexLastOnS2();

            // le coin pour commencer,
            // -----------------------
            ChFiDS_Regul regdeb = new(), regfin = new();
            If1 = ChFi3d_IndexPointInDS(Pf1, DStr);
            If2 = ChFi3d_IndexPointInDS(Pf2, DStr);
            if (c1triangle)
            {
                Il1 = If1;
            }
            else
            {
                Il1 = ChFi3d_IndexPointInDS(Pl1, DStr);
            }
            Il2 = ChFi3d_IndexPointInDS(Pl2, DStr);

            coin.ChangeIndexOfS1(DStr.AddShape(face[pivot]));
            coin.ChangeIndexOfS2(-fdpiv.Surf());

            // first points
            Point2D pp1 = new(), pp2 = new();
            if (c1plan)
            {
                P1deb = DStr.Curve(Icf).Curve().FirstParameter();
                P2deb = DStr.Curve(Icf).Curve().LastParameter();
            }
            else
            {
                pp1 =
                    coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().FirstParameter());
                pp2 =
                    coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().FirstParameter());
                Curve3D? C3d = null;
                double tolreached = 0.0;
                ChFi3d_ComputeArete(Pf1, pp1, Pf2, pp2, DStr.Surface(coin.Surf()).Surface(), ref C3d,
                                                        ref corner.ChangeFirstPCurve(), ref P1deb, ref P2deb, tolapp3d, tol2d, out tolreached, 0);
                TopOpeBRepDS_Curve Tcurv = new(C3d, tolreached);
                Icf = DStr.AddCurve(Tcurv);
            }

            regdeb.SetCurve(Icf);
            regdeb.SetS1(coin.Surf(), false);
            regdeb.SetS2(fddeb.Surf(), false);
            myRegul.Append(regdeb);
            corner.ChangeFirstCurve(Icf);
            corner.ChangeFirstParameters(P1deb, P2deb);
            corner.ChangeIndexFirstPointOnS1(If1);
            corner.ChangeIndexFirstPointOnS2(If2);

            // last points
            if (c1plan)
            {
                P1fin = DStr.Curve(Icl).Curve().FirstParameter();
                P2fin = DStr.Curve(Icl).Curve().LastParameter();
            }
            else
            {
                pp1 =
                    coin.InterferenceOnS1().PCurveOnSurf().Value(coin.InterferenceOnS1().LastParameter());
                pp2 =
                    coin.InterferenceOnS2().PCurveOnSurf().Value(coin.InterferenceOnS2().LastParameter());
                Curve3D? C3d = null;
                double tolreached = 0.0;
                ChFi3d_ComputeArete(Pl1, pp1, Pl2, pp2, DStr.Surface(coin.Surf()).Surface(), ref C3d,
                                                        ref corner.ChangeLastPCurve(), ref P1fin, ref P2fin, tolapp3d, tol2d, out tolreached, 0);
                TopOpeBRepDS_Curve Tcurv = new(C3d, tolreached);
                Icl = DStr.AddCurve(Tcurv);
            }
            regfin.SetCurve(Icl);
            regfin.SetS1(coin.Surf(), false);
            regfin.SetS2(fdfin.Surf(), false);
            myRegul.Append(regfin);
            corner.ChangeLastCurve(Icl);
            corner.ChangeLastParameters(P1fin, P2fin);
            corner.ChangeIndexLastPointOnS1(Il1);
            corner.ChangeIndexLastPointOnS2(Il2);

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
                CD[deb].SetOrientation(Reversed, isfirst);
            }
            CD[deb].SetCurve(Icf, isfirst);
            CD[deb].SetIndexPoint(If1, isfirst, isurf1);
            CD[deb].SetIndexPoint(If2, isfirst, isurf2);
            CD[deb].SetParameters(isfirst, P1deb, P2deb);
            fddeb.ChangeVertex(isfirst, isurf1) = Pf1;
            fddeb.ChangeVertex(isfirst, isurf2) = Pf2;
            fddeb.ChangeInterference(isurf1).SetParameter(par2, isfirst);
            fddeb.ChangeInterference(isurf2).SetParameter(par, isfirst);
            if (c1plan)
            {
                CD[deb].ChangePCurve(isfirst) = debpc1;
            }
            else
            {
                pp1 = fddeb.InterferenceOnS1().PCurveOnSurf().Value(par);
                pp2 = fddeb.InterferenceOnS2().PCurveOnSurf().Value(par);
                ChFi3d_ComputePCurv(pp1, pp2, out CD[deb].ChangePCurve(isfirst), P1deb, P2deb, rev);
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
                CD[fin].SetOrientation(Reversed, isfirst);
            }
            CD[fin].SetCurve(Icl, isfirst);
            CD[fin].SetIndexPoint(Il1, isfirst, isurf1);
            CD[fin].SetIndexPoint(Il2, isfirst, isurf2);
            CD[fin].SetParameters(isfirst, P1fin, P2fin);
            fdfin.ChangeVertex(isfirst, isurf1) = Pl1;
            fdfin.ChangeVertex(isfirst, isurf2) = Pl2;
            fdfin.ChangeInterference(isurf1).SetParameter(par2, isfirst);
            fdfin.ChangeInterference(isurf2).SetParameter(par, isfirst);
            if (c1plan)
            {
                CD[fin].ChangePCurve(isfirst) = finpc1;
            }
            else
            {
                pp1 = fdfin.InterferenceOnS1().PCurveOnSurf().Value(par);
                pp2 = fdfin.InterferenceOnS2().PCurveOnSurf().Value(par);
                ChFi3d_ComputePCurv(pp1, pp2, out CD[fin].ChangePCurve(isfirst), P1fin, P2fin, rev);
            }

            // et enfin le pivot.
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
                CD[pivot].SetOrientation(Reversed, isfirst);
            }
            CD[pivot].SetCurve(fi.LineIndex(), isfirst);
            CD[pivot].ChangePCurve(isfirst) = fi.PCurveOnFace();
            CD[pivot].SetIndexPoint(If2, isfirst, isurf1);
            CD[pivot].SetIndexPoint(Il2, isfirst, isurf2);
            CD[pivot].SetParameters(isfirst, fi.FirstParameter(), fi.LastParameter());
            fdpiv.ChangeVertex(isfirst, isurf1) = Pf2;
            fdpiv.ChangeVertex(isfirst, isurf2) = Pl2;
            fdpiv.ChangeInterference(isurf1).SetParameter(p[pivot, deb], isfirst);
            fdpiv.ChangeInterference(isurf2).SetParameter(p[pivot, fin], isfirst);
            CD[pivot].InDS(isfirst); // filDS fait deja le boulot depuis le coin.
        }

        // On tronque les corners data et met a jour les index.
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
        // il faudra ici tenir compte des coins mutants.
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

internal static class ChFi3d_ChBuilder_C3
{
    internal static bool CoPlanar(in Point3D PntA, in Point3D PntB, in Point3D PntC, in Point3D PntD)
    {
        Vector3D vecAB = new(PntA, PntB);
        Vector3D vecAC = new(PntA, PntC);
        Vector3D vecAD = new(PntA, PntD);

        double nor2AB = vecAB.SquareMagnitude();
        double nor2AC = vecAC.SquareMagnitude();
        double ProABAC = vecAB.Dot(vecAC);

        double Alpha = nor2AB * nor2AC - ProABAC * ProABAC;

        if (Alpha < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return true;
        }

        double ProABAD = vecAB.Dot(vecAD);
        double ProACAD = vecAC.Dot(vecAD);
        double Alpha1 = ProABAD * nor2AC - ProABAC * ProACAD;
        double Alpha2 = ProACAD * nor2AB - ProABAC * ProABAD;
        Vector3D vecDABC = Alpha1 * vecAB + Alpha2 * vecAC - Alpha * vecAD;

        return (vecDABC.Magnitude() / Alpha) < global::Kerf.Engine.Foundation.Precision.Confusion();
    }

    internal static SurfaceAdaptor? BoundSurf(Surface? S, in Point2D Pdeb, in Point2D Pfin)
    {
        SurfaceAdaptor? HS = new SurfaceAdaptor();
        SurfaceAdaptor GAS = HS;
        GAS.Load(S);

        double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
        double uuu1 = 0.0, uuu2 = 0.0, vvv1 = 0.0, vvv2 = 0.0;
        S.Bounds(out uuu1, out uuu2, out vvv1, out vvv2);
        ChFi3d_Boite(Pdeb, Pfin, out uu1, out uu2, out vv1, out vv2);
        double Step = Max((uu2 - uu1), (vv2 - vv1));
        Step *= 0.2;
        uuu1 = Max((uu1 - Step), uuu1);
        uuu2 = Min((uu2 + Step), uuu2);
        vvv1 = Max((vv1 - Step), vvv1);
        vvv2 = Min((vv2 + Step), vvv2);
        GAS.Load(S, uuu1, uuu2, vvv1, vvv2);
        return HS;
    }

    internal static bool ComputeIntersection(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData? SD, ChFiDS_SurfData? SDCoin, in Point3D pdeb, in Point2D p2ddeb, in Point3D pfin, in Point2D p2dfin, ref Curve3D? gc, ref Curve2D? pc1, ref Curve2D? pc2, ref Vector3D derudeb,
                                             ref Vector3D dervdeb, ref Point2D ptcoindeb, double tol3d, double tol2d,
                                             ref double tolreached)
    {
        //  gp_Pnt2d UVf1,UVf2,UVl1,UVl2;

        // take the surface of the pivot SurfData and trim it to allow
        // the intersection computation if it's an analytic surface
        SurfaceAdaptor? HS1 = null;
        HS1 = ChFi3d_BoundSurf(DStr, SD, 1, 2);

        Surface? gpl = DStr.Surface(SDCoin.Surf()).Surface();
        Surface? gSD = DStr.Surface(SD.Surf()).Surface();

        // compute pardeb
        NCollection_Array1<double> Pardeb = new(1, 4), Parfin = new(1, 4);
        double u = 0.0, v = 0.0;
        Point3D Pbidon = new();
        u = p2ddeb.X();
        v = p2ddeb.Y();
        gSD.D1(u, v, out Pbidon, out derudeb, out dervdeb);

        Pardeb[1] = u;
        Pardeb[2] = v;
        //  gp_Pnt2d pd2(u,v);

        ChFi3d_Parameters(gpl, pdeb, out u, out v);
        Pardeb[3] = u;
        Pardeb[4] = v;
        ptcoindeb.SetCoord(u, v);

        // compute parfin
        u = p2dfin.X();
        v = p2dfin.Y();
        Parfin[1] = u;
        Parfin[2] = v;
        //  gp_Pnt2d pf2(u,v);

        ChFi3d_Parameters(gpl, pfin, out u, out v);
        Parfin[3] = u;
        Parfin[4] = v;
        Point2D cpf2 = new(u, v);

        // Trims the chamfer surface to allow the intersection computation
        // and computes a GeomAdaptor_Surface for using the ComputeCurves
        // function
        SurfaceAdaptor? HS2 = null;
        HS2 = BoundSurf(gpl, ptcoindeb, cpf2);

        // compute the intersection curves and pcurves
        return ChFi3d_ComputeCurves(HS1, HS2, Pardeb, Parfin, ref gc, ref pc1, ref pc2, tol3d, tol2d, out tolreached);
    }


}
