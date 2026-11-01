// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_CnCrn.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_Builder_CnCrn.ChFi3d_SurfType;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Occt.ChFi3d_Builder_CnCrn;
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

    // Modified by  MPS (14-04-97)  traitement des cas  ou il n'y a pas
    //                              d'intersection entre les stripes
    //  Modified by  MPS (16-06-97) : on tient compte du fait que GeomPlate
    //                                rend les courbes 2d dans meme ordre que les
    //                                courbes frontieres passees en entree
    //  Modified by JLR (20-08-97) mise en place des nouveaux constructeurs de GeomPlate
    //  Modified by MPS (03-11-97) on ne cree pas un  batten lorsque le rapport
    //  entre les deux resolutions sur la surface est trop grand (PRO10649)
    //  Modified by MPS (05-12-97) on ne tient pas compte des aretes degenerees
    //                             lors du calcul du nombre d'aretes.
    //  Modified by JCT (08-12-97) traitement des aretes vives consecutives ou non
    //                             (grille EDC412 sauf D2, L1, L2, L3)
    //  Modified by JCT (11-12-97) pb osf avec indpoint + orientation de plate
    //                             ( --> D2, L1, L2, L3 valides mais laids)
    //  Modified by MPS (24-02-98)  traitement des aretes de regularite
    //  Modified by MPS (01-06-98)  traitement des aretes de couture
    //  Modified by MPS (01-12-98)  traitement des bords libres
    //  Modified by MPS (01-02-99)  traitement des aretes de regularite
    //                              consecutives
    // Traitement des coins


    // performances

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=======================================================================
    // function : cherche_edge1
    // purpose  : find common edge of faces F1 and F2
    //=======================================================================

    //=======================================================================
    // function : CurveHermite
    // purpose  : calculate a  curve 3d using polynoms of Hermite.
    //           the edge is a regular edge. Curve 3D is constructed
    //           between edges icmoins and icplus.
    //=======================================================================

    //=======================================================================
    // function : CalculDroite
    // purpose  : calculate a 2D straight line passing through point p2d1 and direction xdir ydir
    //=======================================================================

    //=======================================================================
    // function : CalculBatten
    // purpose  : calcule a batten between curves 2d  curv2d1 and curv2d2 at points p2d1 and p2d2
    //=======================================================================

    //=======================================================================
    // function : OrientationIcNonVive
    // purpose  : calculate the orientation of the curve between ic and icplus knowing that ic
    //           is not a living edge.
    //=======================================================================

    //=======================================================================
    // function : OrientationIcplusNonVive
    // purpose  : calculate the orientation of the curve between ic and icplus knowing that icplus
    //           is not a living edge;
    //=======================================================================

    //=======================================================================
    // function : OrientationAreteViveConsecutive
    // purpose  : calculate the orientation of the curve between edges ic and icplus
    //           where ic and icplus are consecutively living
    //=======================================================================

    //=======================================================================
    // function : PerformTwoCornerSameExt
    // purpose  : calculate intersection between two stripes stripe1 and stripe2
    //=======================================================================

    //=======================================================================
    // function : CpOnEdge
    // purpose  : determine if surfdata num has a common point on Eadj1 or Eadj2
    //=======================================================================

    //=======================================================================
    // function : RemoveSurfData
    // purpose  : for each stripe removal of unused surfdatas
    //=======================================================================

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=======================================================================
    // function : PlateOrientation
    // purpose  : Define Plate orientation compared to <theRefDir> previewing
    //           that Plate surface can have a sharp angle with adjacent
    //           filet (bug occ266: 2 chamfs, OnSame and OnDiff) and
    //           can be even twisted (grid tests cfi900 B1)
    //=======================================================================

    //=======================================================================
    // function : PerformMoreThreeCorner
    // purpose  : Process case of a top with n edges.
    //=======================================================================

    internal void PerformMoreThreeCorner(int Jndex, int nconges)
    {
    //    ========================================
    //             Initialisations
    //     ========================================
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        TopoDS_Vertex V1 = myVDataMap.FindKey(Jndex);
        int nedge = 0;
        bool bordlibre = false;
        TopoDS_Edge edgelibre1 = new(), edgelibre2 = new();
        //  NCollection_List<TopoDS_Shape>::Iterator ItE;
        nedge = ChFi3d_NbNotDegeneratedEdges(V1, myVEMap);
        ChFi3d_ChercheBordsLibres(myVEMap, V1, out bordlibre, ref edgelibre1, ref edgelibre2);
        bool droit = false;
        if (bordlibre)
        {
            nedge = (nedge - 2) / 2 + 2;
            double angedg = Math.Abs(ChFi3d_AngleEdge(V1, edgelibre1, edgelibre2));
            droit = Math.Abs(angedg - M_PI) < 0.01;
        }
        else
        {
            nedge = nedge / 2;
        }
        int size = nedge * 2;
        NCollection_Array1<ChFiDS_Stripe> CD = new(0, size);
        NCollection_Array1<int> jf = new(0, size);
        NCollection_Array1<int> Index = new(0, size);
        NCollection_Array1<int> Indice = new(0, size);
        NCollection_Array1<int> sens = new(0, size);
        NCollection_Array1<int> indcurve3d = new(0, size);
        NCollection_Array2<int> numfa = new(0, size, 0, size);
        NCollection_Array1<int> Order = new(0, size);
        NCollection_Array2<int> i = new(0, size, 0, size);
        NCollection_Array2<int> indpoint = new(0, size, 0, 1);
        NCollection_Array1<bool> oksea = new(0, size);
        NCollection_Array1<bool> sharp = new(0, size);
        NCollection_Array1<bool> regul = new(0, size);
        NCollection_Array2<double> p = new(0, size, 0, size);
        NCollection_Array1<double> errapp = new(0, size);
        NCollection_Array1<TopoDS_Shape> Evive = new(0, size);
        NCollection_Array2<TopoDS_Shape> Fvive = new(0, size, 0, size);
        NCollection_Array1<bool> ponctuel = new(0, size);
        NCollection_Array1<bool> samedge = new(0, size);
        NCollection_Array1<bool> moresurf = new(0, size);
        NCollection_Array1<bool> libre = new(0, size);
        NCollection_Array1<bool> tangentregul = new(0, size);
        NCollection_Array1<bool> isG1 = new(0, size);
        //  for(int ind=0;ind<=size;ind++){
        int ind = 0;
        for (ind = 0; ind <= size; ind++)
        {
            indpoint.SetValue(ind, 0, 0);
            indpoint.SetValue(ind, 1, 0);
            Indice.SetValue(ind, 0);
            oksea.SetValue(ind, false);
            sharp.SetValue(ind, false);
            regul.SetValue(ind, false);
            ponctuel.SetValue(ind, false);
            samedge.SetValue(ind, false);
            moresurf.SetValue(ind, false);
            libre.SetValue(ind, false);
            tangentregul.SetValue(ind, false);
            isG1.SetValue(ind, false);
        }
        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        ChFiDS_Stripe? cdbid = null, cnext = null; // OCCT's unused cd2 left out
        TopoDS_Face face = new();
        int jfp = 0, ii;
        int ic = 0, icplus = 0, icmoins = 0, icplus2 = 0, sense = 0, index = 0, indice = 0, isurf1 = 0, isurf2 = 0;
        int n3d = 0, IVtx = 0, nb;
        bool sameside = false, trouve = false, isfirst = false;
        double pardeb = 0.0, parfin = 0.0, xdir = 0.0, ydir = 0.0;
        double tolapp = 1.0e-4, maxapp = 0.0, maxapp1 = 0.0, avedev;
        TopOpeBRepDS_CurvePointInterference? Interfp1 = null, Interfp2 = null;
        TopOpeBRepDS_SurfaceCurveInterference? Interfc = null;
        Geom_Curve? Curv3d = null;
        ChFiDS_Regul regular = new();
        NCollection_Sequence<TopoDS_Shape> Fproj = new();
        int num = 0;
        TopoDS_Edge Ecur = new();
        NCollection_List<TopoDS_Shape>.Iterator ItF = new();
        TopoDS_Face F1 = new(), F2 = new();
        gp_Vec SumFaceNormalAtV1 = new(0, 0, 0); // is used to define Plate orientation

        // it is determined if there is a sewing edge
        bool couture = false;
        TopoDS_Face facecouture = new();
        TopoDS_Edge edgecouture = new();
        for (ItF.Initialize(myVFMap[V1]); ItF.More() && !couture; ItF.Next())
        {
            TopoDS_Face fcur = TopoDS.Face(ItF.Value());
            ChFi3d_CoutureOnVertex(fcur, V1, out couture, ref edgecouture);
            if (couture)
            {
                facecouture = fcur;
            }
        }

        // unused surfdata are removed
        RemoveSurfData(myVDataMap, myEFMap, edgecouture, facecouture, V1);

        // parse edges and faces
        trouve = false;
        TopoDS_Edge Enext = new();
        TopoDS_Vertex VV = new();
        TopoDS_Face Fcur = new(), Fnext = new();
        for (It.Initialize(myVDataMap[Jndex]); It.More() && !trouve; It.Next())
        {
            cnext = It.Value();
            CD.SetValue(0, cnext);
            Index.SetValue(0, ChFi3d_IndexOfSurfData(V1, cnext, out sense));
            sens.SetValue(0, sense);
            numfa.SetValue(0, 1, SurfIndex(CD, 0, Index.Value(0), FACE2));
            numfa.SetValue(1, 0, numfa.Value(0, 1));
            Fcur = TopoDS.Face(DStr.Shape(numfa.Value(0, 1)));
            Fvive.SetValue(0, 1, Fcur);
            Fvive.SetValue(1, 0, Fcur);
            jf.SetValue(0, 2);
            if (sens.Value(0) == 1)
            {
                Ecur = CD.Value(0).Spine().Edges(1);
            }
            else
            {
                Ecur = CD.Value(0).Spine().Edges(CD.Value(0).Spine().NbEdges());
            }
            Evive.SetValue(0, Ecur);
            ChFi3d_cherche_edge(V1, Evive, Fcur, ref Enext, ref VV);
            trouve = !Enext.IsNull();
        }
        // find sum of all face normals at V1
        SummarizeNormal(V1, Fcur, Ecur, ref SumFaceNormalAtV1);

        int nbcouture = 0;
        for (ii = 1; ii < nedge; ii++)
        {
            if (Fcur.IsSame(facecouture) && nbcouture == 0)
            {
                Enext = edgecouture;
                nbcouture++;
            }
            else
            {
                ChFi3d_cherche_edge(V1, Evive, Fcur, ref Enext, ref VV);
            }
            if (Enext.IsNull())
            {
                throw new Standard_ConstructionError("PerformMoreThreeCorner: pb in the parsing of edges and faces");
            }
            if (Enext.IsSame(edgelibre1) || Enext.IsSame(edgelibre2))
            {
                CD.SetValue(ii, cdbid);
                Index.SetValue(ii, 0);
                sens.SetValue(ii, -1);
                TopoDS_Vertex Vref = new();
                Vref = TopExp.FirstVertex(Enext);
                if (Vref.IsSame(V1))
                {
                    sens.SetValue(ii, 1);
                }
                sharp.SetValue(ii, true);
                Evive.SetValue(ii, Enext);
                jf.SetValue(ii, 0);
                Indices(nedge, ii, ref icplus, ref icmoins);
                Fvive.SetValue(ii, icplus, Fcur);
                Fvive.SetValue(icplus, ii, Fcur);
                numfa.SetValue(ii, icplus, DStr.AddShape(Fcur));
                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                ii++;
                if (Enext.IsSame(edgelibre1))
                {
                    Ecur = edgelibre2;
                }
                else
                {
                    Ecur = edgelibre1;
                }
                ChFi3d_edge_common_faces(myEFMap[Ecur], out Fcur, out Fcur);
                Indices(nedge, ii, ref icplus, ref icmoins);
                Fvive.SetValue(ii, icplus, Fcur);
                Fvive.SetValue(icplus, ii, Fcur);
                numfa.SetValue(ii, icplus, DStr.AddShape(Fcur));
                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                CD.SetValue(ii, cdbid);
                Index.SetValue(ii, 0);
                sens.SetValue(ii, -1);
                Vref = TopExp.FirstVertex(Ecur);
                if (Vref.IsSame(V1))
                {
                    sens.SetValue(ii, 1);
                }
                sharp.SetValue(ii, true);
                Evive.SetValue(ii, Ecur);
                jf.SetValue(ii, 0);
            }
            else
            {
                // it is found if Enext is in the map of stripes
                TopoDS_Edge EE = new();
                /*bool */ trouve = false;
                for (It.Initialize(myVDataMap[Jndex]); It.More() && !trouve; It.Next())
                {
                    index = ChFi3d_IndexOfSurfData(V1, It.Value(), out sense);
                    if (sense == 1)
                    {
                        EE = It.Value().Spine().Edges(1);
                    }
                    else
                    {
                        EE = It.Value().Spine().Edges(It.Value().Spine().NbEdges());
                    }
                    if (Enext.IsSame(EE))
                    {
                        cnext = It.Value();
                        trouve = true;
                    }
                }
                if (trouve)
                {
                    CD.SetValue(ii, cnext);
                    Index.SetValue(ii, index);
                    sens.SetValue(ii, sense);
                    sharp.SetValue(ii, false);
                    Evive.SetValue(ii, Enext);
                }
                else
                {
                    //      edge ii is alive
                    CD.SetValue(ii, cdbid);
                    Index.SetValue(ii, 0);
                    sens.SetValue(ii, -1);
                    TopoDS_Vertex Vref = new();
                    Vref = TopExp.FirstVertex(Enext);
                    if (Vref.IsSame(V1))
                    {
                        sens.SetValue(ii, 1);
                    }
                    sharp.SetValue(ii, true);
                    Evive.SetValue(ii, Enext);
                    jf.SetValue(ii, 0);
                }
                // Face Fnext!=Fcur containing Enext
                Fnext = Fcur;
                ChFi3d_cherche_face1(myEFMap[Enext], Fcur, ref Fnext);
                Indices(nedge, ii, ref icplus, ref icmoins);
                Fvive.SetValue(ii, icplus, Fnext);
                Fvive.SetValue(icplus, ii, Fnext);
                numfa.SetValue(ii, icplus, DStr.AddShape(Fnext));
                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                int numface1 = 0, numface2 = 0;
                if (trouve)
                {
                    // it is checked if numfa corresponds to IndexOfS1 or IndexOfS2
                    // jf is updated is consequently updated
                    // if it is not the case among the previous faces are found
                    // those which correspond to  IndexOfs1 IndexOfS2  and
                    // numfa and Fvive are reupdated (cts16288)
                    numface2 = SurfIndex(CD, ii, Index.Value(ii), FACE2);
                    if (numface2 == numfa.Value(ii, icplus))
                    {
                        jf.SetValue(ii, 2);
                    }
                    else
                    {
                        numface1 = SurfIndex(CD, ii, Index.Value(ii), FACE1);
                        if (numface1 == numfa.Value(ii, icplus))
                        {
                            jf.SetValue(ii, 1);
                        }
                        else
                        {
                            if (numface1 == numfa.Value(icmoins, ii))
                            {
                                jf.SetValue(ii, 2);
                                Fvive.SetValue(ii, icplus, TopoDS.Face(DStr.Shape(numface2)));
                                Fvive.SetValue(icplus, ii, TopoDS.Face(DStr.Shape(numface2)));
                                numfa.SetValue(ii, icplus, DStr.AddShape(TopoDS.Face(DStr.Shape(numface2))));
                                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                            }
                            if (numface2 == numfa.Value(icmoins, ii))
                            {
                                jf.SetValue(ii, 1);
                                Fvive.SetValue(ii, icplus, TopoDS.Face(DStr.Shape(numface1)));
                                Fvive.SetValue(icplus, ii, TopoDS.Face(DStr.Shape(numface1)));
                                numfa.SetValue(ii, icplus, DStr.AddShape(TopoDS.Face(DStr.Shape(numface1))));
                                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                            }
                        }
                    }
                }
                Ecur = Enext;
                Fcur = Fnext;
                // find sum of all face normales at V1
                SummarizeNormal(V1, Fcur, Ecur, ref SumFaceNormalAtV1);
            }
        }
        // mise a jour du tableau regul
        for (ic = 0; ic < nedge; ic++)
        {
            if (sharp.Value(ic))
            {
                Ecur = TopoDS.Edge(Evive.Value(ic));
                if (!Ecur.IsSame(edgecouture))
                {
                    ChFi3d_edge_common_faces(myEFMap[Ecur], out F1, out F2);
                    //  Modified by Sergey KHROMOV - Fri Dec 21 18:11:02 2001 Begin
                    // 	regul.SetValue(ic,BRep_Tool::Continuity(TopoDS::Edge(Evive.Value(ic)),F1,F2)
                    // 		     !=GeomAbs_C0);
                    regul.SetValue(ic, ChFi3d.IsTangentFaces(TopoDS.Edge(Evive.Value(ic)), F1, F2));
                    //  Modified by Sergey KHROMOV - Fri Dec 21 18:11:07 2001 End
                }
            }
        }
        // it is checked if a regular edge is not tangent to another edge
        // in case if it is not considered regular (cts60072)
        for (ic = 0; ic < nedge; ic++)
        {
            if (regul.Value(ic))
            {
                trouve = false;
                TopoDS_Edge ereg = TopoDS.Edge(Evive.Value(ic));
                for (ind = 0; ind < nedge && !trouve; ind++)
                {
                    if (ind != ic)
                    {
                        TopoDS_Edge ecur = TopoDS.Edge(Evive.Value(ind));
                        double ang = Math.Abs(ChFi3d_AngleEdge(V1, ecur, ereg));
                        if (ang < 0.01 || Math.Abs(ang - M_PI) < 0.01)
                        {
                            regul.SetValue(ic, false);
                            tangentregul.SetValue(ic, true);
                            trouve = true;
                        }
                    }
                }
            }
        }

        // variable deuxconges allows detecting cases when there is a top with
        // n edges and two fillets on two tangent edges that are not free borders
        // the connecting curves start from the fillet and end on top

        bool deuxconges = false, deuxcgnontg = false;
        deuxconges = false;
        trouve = false;
        if (nconges == 2)
        {
            TopoDS_Edge E1 = new(), E2 = new();
            for (ic = 0; ic < nedge && !trouve; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (!sharp.Value(ic) && !sharp.Value(icplus))
                {
                    E1 = TopoDS.Edge(Evive.Value(ic));
                    E2 = TopoDS.Edge(Evive.Value(icplus));
                    deuxconges = (Math.Abs(ChFi3d_AngleEdge(V1, E1, E2)) < 0.01);
                    trouve = deuxconges;
                }
            }
        }

        // variable deuxconges is used in the special case when there are
        // two fillets and if two other living edges are tangent (cts60072)
        if (nconges == 2 && nedge == 4)
        {
            TopoDS_Edge E1 = new(), E2 = new();
            for (ic = 0; ic < nedge && !deuxconges; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (sharp.Value(ic) && sharp.Value(icplus))
                {
                    E1 = TopoDS.Edge(Evive.Value(ic));
                    E2 = TopoDS.Edge(Evive.Value(icplus));
                    if (!E1.IsSame(edgelibre1) && !E1.IsSame(edgelibre2) && !E2.IsSame(edgelibre1)
                            && !E2.IsSame(edgelibre2))
                    {
                        double ang = Math.Abs(ChFi3d_AngleEdge(V1, E1, E2));
                        deuxconges = (ang < 0.01 || Math.Abs(ang - M_PI) < 0.01);
                    }
                }
            }
        }

        deuxcgnontg = nconges == 2 && nedge == 3 && !deuxconges; // pro12305

        if (deuxconges)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                regul.SetValue(ic, false);
            }
        }

        // Detect case of 3 edges & 2 conges: OnSame + OnDiff
        // (eap, Arp 9 2002, occ266)
        bool isOnSameDiff = false;
        if (deuxcgnontg)
        {
            bool isOnSame = false, isOnDiff = false;
            for (ic = 0; ic < nedge; ic++)
            {
                if (sharp.Value(ic))
                {
                    continue;
                }
                ChFiDS_State stat = default;
                if (sens[ic] == 1)
                {
                    stat = CD.Value(ic).Spine().FirstStatus();
                }
                else
                {
                    stat = CD.Value(ic).Spine().LastStatus();
                }

                if (stat == ChFiDS_OnSame)
                {
                    isOnSame = true;
                }
                else if (stat == ChFiDS_OnDiff)
                {
                    isOnDiff = true;
                }
            }
            isOnSameDiff = isOnSame && isOnDiff;
        }
        if (isOnSameDiff)
        {
            PerformOneCorner(Jndex, true);
        }

        // if the commonpoint is on an edge that does not have a
        // vertex at the extremity, Evive is found anew
        // Fvive is found anew if it does not correspond
        // to two faces adjacent to Evive (cts16288)

        if (!deuxconges && !isOnSameDiff)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                if (sharp.Value(ic))
                {
                    Indices(nedge, ic, ref icplus, ref icmoins);
                    TopoDS_Edge Arc = TopoDS.Edge(Evive.Value(ic));
                    ChFiDS_CommonPoint cp1 = new(), cp2 = new();
                    double angedg = M_PI;
                    TopoDS_Vertex Vcom = new();
                    if (!sharp.Value(icplus))
                    {
                        isfirst = (sens.Value(icplus) == 1);
                        jfp = 3 - jf.Value(icplus);
                        cp1 = CD.Value(icplus)
                                        .SetOfSurfData()
                                        .Value(Index.Value(icplus))
                                        .ChangeVertex(isfirst, jfp);
                        if (cp1.IsOnArc())
                        {
                            ChFi3d_cherche_vertex(Arc, cp1.Arc(), ref Vcom, out trouve);
                            if (trouve)
                            {
                                angedg = Math.Abs(ChFi3d_AngleEdge(Vcom, Arc, cp1.Arc()));
                            }
                            if (!cp1.Arc().IsSame(Arc) && Math.Abs(angedg - M_PI) < 0.01)
                            {
                                Evive.SetValue(ic, cp1.Arc());
                                ChFi3d_edge_common_faces(myEFMap[cp1.Arc()], out F1, out F2);
                                if (!Fvive.Value(ic, icplus).IsSame(F1) && !Fvive.Value(ic, icplus).IsSame(F2))
                                {
                                    if (Fvive.Value(ic, icmoins).IsSame(F2))
                                    {
                                        Fvive.SetValue(ic, icplus, F1);
                                        Fvive.SetValue(icplus, ic, F1);
                                        numfa.SetValue(ic, icplus, DStr.AddShape(F1));
                                        numfa.SetValue(icplus, ic, DStr.AddShape(F1));
                                    }
                                    else
                                    {
                                        Fvive.SetValue(ic, icplus, F2);
                                        Fvive.SetValue(icplus, ic, F2);
                                        numfa.SetValue(ic, icplus, DStr.AddShape(F2));
                                        numfa.SetValue(icplus, ic, DStr.AddShape(F2));
                                    }
                                }
                                samedge.SetValue(ic, true);
                                p.SetValue(ic, icplus, cp1.ParameterOnArc());
                                p.SetValue(ic, icmoins, cp1.ParameterOnArc());
                                i.SetValue(ic, icplus, 1);
                            }
                        }
                    }
                    if (!sharp.Value(icmoins))
                    {
                        isfirst = (sens.Value(icmoins) == 1);
                        cp2 = CD.Value(icmoins)
                                        .SetOfSurfData()
                                        .Value(Index.Value(icmoins))
                                        .ChangeVertex(isfirst, jf.Value(icmoins));
                        if (cp2.IsOnArc())
                        {
                            angedg = M_PI;
                            ChFi3d_cherche_vertex(Arc, cp2.Arc(), ref Vcom, out trouve);
                            if (trouve)
                            {
                                angedg = Math.Abs(ChFi3d_AngleEdge(Vcom, Arc, cp2.Arc()));
                            }
                            if (!cp2.Arc().IsSame(Arc) && Math.Abs(angedg - M_PI) < 0.01)
                            {
                                Evive.SetValue(ic, cp2.Arc());
                                ChFi3d_edge_common_faces(myEFMap[cp2.Arc()], out F1, out F2);
                                if (!Fvive.Value(ic, icmoins).IsSame(F1) && !Fvive.Value(ic, icmoins).IsSame(F2))
                                {
                                    if (Fvive.Value(ic, icplus).IsSame(F2))
                                    {
                                        Fvive.SetValue(ic, icmoins, F1);
                                        numfa.SetValue(ic, icmoins, DStr.AddShape(F1));
                                        Fvive.SetValue(icmoins, ic, F1);
                                        numfa.SetValue(icmoins, ic, DStr.AddShape(F1));
                                    }
                                    else
                                    {
                                        Fvive.SetValue(ic, icmoins, F2);
                                        numfa.SetValue(ic, icmoins, DStr.AddShape(F2));
                                        Fvive.SetValue(icmoins, ic, F2);
                                        numfa.SetValue(icmoins, ic, DStr.AddShape(F2));
                                    }
                                }
                                samedge.SetValue(ic, true);
                                p.SetValue(ic, icmoins, cp2.ParameterOnArc());
                                p.SetValue(ic, icplus, cp2.ParameterOnArc());
                                i.SetValue(ic, icmoins, 1);
                            }
                        }
                    }
                }
            }
        }

        // the first free edge is restored if it exists
        trouve = false;
        for (ic = 0; ic < nedge && !trouve; ic++)
        {
            TopoDS_Edge ecom = new();
            ecom = TopoDS.Edge(Evive.Value(ic));
            if (ecom.IsSame(edgelibre1) || ecom.IsSame(edgelibre2))
            {
                libre.SetValue(ic, true);
                trouve = true;
            }
        }

        // determine the minimum recoil distance that can't be exceeded
        bool distmini = false;
        gp_Pnt som = BRep_Tool.Pnt(V1), pic;
        gp_Pnt2d p2 = new();
        TopoDS_Edge edgemin = new();
        TopoDS_Vertex V = new(), V2 = new();
        double dst = 0.0, distmin = 0.0;
        distmin = 1.0e30;
        for (ic = 0; ic < nedge; ic++)
        {
            if (sharp.Value(ic))
            {
                edgemin = TopoDS.Edge(Evive.Value(ic));
            }
            else
            {
                if (sens.Value(ic) == 1)
                {
                    edgemin = CD.Value(ic).Spine().Edges(1);
                }
                else
                {
                    edgemin = CD.Value(ic).Spine().Edges(CD.Value(ic).Spine().NbEdges());
                }
            }
            V = TopExp.FirstVertex(edgemin);
            V2 = TopExp.LastVertex(edgemin);
            dst = (BRep_Tool.Pnt(V)).Distance(BRep_Tool.Pnt(V2)) / 1.5;
            if (dst < distmin)
            {
                distmin = dst;
            }
        }

        //  calculate intersections between stripes and determine the parameters on each pcurve
        bool inters = true;
        for (ic = 0; ic < nedge; ic++)
        {
            Indices(nedge, ic, ref icplus, ref icmoins);
            if (sharp.Value(ic) || sharp.Value(icplus))
            {
                oksea.SetValue(ic, false);
            }
            else
            {
                int jf1 = 0;
                int i1 = 0, i2 = 0;
                double pa1 = 0.0, pa2 = 0.0;
                bool ok = false;
                ChFiDS_Stripe? strip = null;
                double angedg = 0.0;
                int iface = 0;
                // if two edges are tangent the intersection is not attempted (cts60046)
                angedg = Math.Abs(ChFi3d_AngleEdge(V1, TopoDS.Edge(Evive.Value(ic)), TopoDS.Edge(Evive.Value(icplus))));
                if (Math.Abs(angedg - M_PI) > 0.01)
                {
                    ok = ChFi3d_SearchFD(DStr, CD.Value(ic), CD.Value(icplus), sens.Value(ic), sens.Value(icplus), ref i1, ref i2,
                                                              ref pa1, ref pa2, Index.Value(ic), Index.Value(icplus), ref face, ref sameside, ref jf1,
                                                              ref jfp);
                }
                else
                {
                    ok = false;
                }
                // if there is an intersection it is checked if surfdata with the intersection
                // corresponds to the first or the last
                // if this is not the case, the surfdata are removed from SD

                if (ok)
                {
                    if (i1 != Index.Value(ic))
                    {
                        int ideb = 0, ifin = 0;
                        strip = CD.Value(ic);
                        if (sens.Value(ic) == 1)
                        {
                            ideb = Index.Value(ic);
                            ifin = i1 - 1;
                        }
                        else
                        {
                            ifin = Index.Value(ic);
                            ideb = i1 + 1;
                        }
                        if (i1 < Index.Value(ic))
                        {
                            for (nb = Index.Value(ic); nb >= i1; nb--)
                            {
                                if ((3 - jf1) == 1)
                                {
                                    iface = SurfIndex(CD, ic, nb, FACE1);
                                }
                                else
                                {
                                    iface = SurfIndex(CD, ic, nb, FACE2);
                                }
                                Fproj.Append(TopoDS.Face(myDS.Shape(iface)));
                            }
                        }
                        if (i1 > Index.Value(ic))
                        {
                            for (nb = Index.Value(ic); nb <= i1; nb++)
                            {
                                if ((3 - jf1) == 1)
                                {
                                    iface = SurfIndex(CD, ic, nb, FACE1);
                                }
                                else
                                {
                                    iface = SurfIndex(CD, ic, nb, FACE2);
                                }
                                Fproj.Append(TopoDS.Face(myDS.Shape(iface)));
                            }
                        }
                        strip = CD.Value(ic);
                        RemoveSD(strip, ideb, ifin);
                        num = ChFi3d_IndexOfSurfData(V1, CD.Value(ic), out sense);
                        Index.SetValue(ic, num);
                        i1 = num;
                    }
                    if (i2 != Index.Value(icplus))
                    {
                        int ideb = 0, ifin = 0;
                        strip = CD.Value(icplus);
                        if (sens.Value(icplus) == 1)
                        {
                            ideb = Index.Value(icplus);
                            ifin = i2 - 1;
                        }
                        else
                        {
                            ifin = Index.Value(icplus);
                            ideb = i2 + 1;
                        }

                        if (i2 < Index.Value(icplus))
                        {
                            for (nb = i2; nb <= Index.Value(icplus); nb++)
                            {
                                if ((3 - jfp) == 1)
                                {
                                    iface = SurfIndex(CD, icplus, nb, FACE1);
                                }
                                else
                                {
                                    iface = SurfIndex(CD, icplus, nb, FACE2);
                                }
                                Fproj.Append(TopoDS.Face(myDS.Shape(iface)));
                            }
                        }
                        if (i2 > Index.Value(icplus))
                        {
                            for (nb = i2; nb >= Index.Value(icplus); nb--)
                            {
                                if ((3 - jfp) == 1)
                                {
                                    iface = SurfIndex(CD, icplus, nb, FACE1);
                                }
                                else
                                {
                                    iface = SurfIndex(CD, icplus, nb, FACE2);
                                }
                                Fproj.Append(TopoDS.Face(myDS.Shape(iface)));
                            }
                        }
                        RemoveSD(strip, ideb, ifin);
                        num = ChFi3d_IndexOfSurfData(V1, CD.Value(icplus), out sense);
                        Index.SetValue(icplus, num);
                        i2 = num;
                    }
                    Calcul_P2dOnSurf(CD.Value(ic), jf1, i1, pa1, ref p2);
                    indice = SurfIndex(CD, ic, i1, ChFiSURFACE);
                    DStr.Surface(indice).Surface().D0(p2.X(), p2.Y(), out pic);
                    if (pic.Distance(som) > distmin)
                    {
                        distmini = true;
                    }
                    jf.SetValue(ic, jf1);
                    i.SetValue(ic, icplus, i1);
                    i.SetValue(icplus, ic, i2);
                    p.SetValue(ic, icplus, pa1);
                    p.SetValue(icplus, ic, pa2);
                }
                oksea.SetValue(ic, ok);
            }
            if (!oksea.Value(ic))
            {
                inters = false;
            }
        }

        // case if there are only intersections
        // the parametres on Pcurves are the extremities of the stripe
        double para = 0.0;
        if (!inters)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                Indices(nedge, icplus, ref icplus2, ref ic);
                if (!oksea.Value(ic))
                {
                    if (sharp.Value(ic))
                    {
                        if (!samedge.Value(ic))
                        {
                            para = BRep_Tool.Parameter(V1, TopoDS.Edge(Evive.Value(ic)));
                            p.SetValue(ic, icplus, para);
                            i.SetValue(ic, icplus, 1);
                        }
                    }
                    else
                    {
                        isfirst = (sens.Value(ic) == 1);
                        i.SetValue(ic, icplus, ChFi3d_IndexOfSurfData(V1, CD.Value(ic), out sense));
                        if (oksea.Value(icmoins))
                        {
                            para = p.Value(ic, icmoins);
                            p.SetValue(ic, icplus, para);
                        }
                        else
                        {
                            Calcul_Param(CD.Value(ic), jf.Value(ic), i.Value(ic, icplus), isfirst, ref para);
                            p.SetValue(ic, icplus, para);
                        }
                    }
                    if (sharp.Value(icplus))
                    {
                        if (!samedge.Value(icplus))
                        {
                            para = BRep_Tool.Parameter(V1, TopoDS.Edge(Evive.Value(icplus)));
                            p.SetValue(icplus, ic, para);
                            i.SetValue(icplus, ic, 1);
                        }
                    }
                    else
                    {
                        isfirst = (sens.Value(icplus) == 1);
                        i.SetValue(icplus, ic, ChFi3d_IndexOfSurfData(V1, CD.Value(icplus), out sense));
                        if (oksea.Value(icplus))
                        {
                            para = p.Value(icplus, icplus2);
                            p.SetValue(icplus, ic, para);
                        }
                        else
                        {
                            jfp = 3 - jf.Value(icplus);
                            Calcul_Param(CD.Value(icplus), jfp, i.Value(icplus, ic), isfirst, ref para);
                            p.SetValue(icplus, ic, para);
                        }
                    }
                }
            }

            //  calculate max distance to the top at each point
            NCollection_Array1<double> dist1 = new(0, size);
            NCollection_Array1<double> dist2 = new(0, size);
            double distance = 0.0;
            gp_Pnt sommet_ = BRep_Tool.Pnt(V1); // OCCT's sommet, which C# cannot redeclare in this scope
            if (!deuxconges)
            {
                for (ic = 0; ic < nedge; ic++)
                {
                    Indices(nedge, ic, ref icplus, ref icmoins);
                    if (sharp.Value(ic))
                    {
                        dist1.SetValue(ic, 0);
                        dist2.SetValue(ic, 0);
                    }
                    else
                    {
                        jfp = 3 - jf.Value(ic);
                        Calcul_P2dOnSurf(CD.Value(ic), jfp, i.Value(ic, icmoins), p.Value(ic, icmoins), ref p2);
                        indice = SurfIndex(CD, ic, i.Value(ic, icmoins), ChFiSURFACE);
                        DStr.Surface(indice).Surface().D0(p2.X(), p2.Y(), out pic);
                        dist1.SetValue(ic, sommet_.Distance(pic));
                        if (dist1.Value(ic) > distance)
                        {
                            distance = dist1.Value(ic);
                        }

                        Calcul_P2dOnSurf(CD.Value(ic), jf.Value(ic), i.Value(ic, icplus), p.Value(ic, icplus), ref p2);
                        indice = SurfIndex(CD, ic, i.Value(ic, icplus), ChFiSURFACE);
                        DStr.Surface(indice).Surface().D0(p2.X(), p2.Y(), out pic);
                        dist2.SetValue(ic, sommet_.Distance(pic));
                        if (dist2.Value(ic) > distance)
                        {
                            distance = dist2.Value(ic);
                        }
                    }
                }
            }

            //  offset of parameters and removal of intersection points
            //  too close to the top

            double ec = 0.0, dist = 0.0;
            if (!deuxconges && !deuxcgnontg)
            {
                for (ic = 0; ic < nedge; ic++)
                {
                    Indices(nedge, ic, ref icplus, ref icmoins);
                    if (sharp.Value(ic))
                    {
                        BRepAdaptor_Curve C = new(TopoDS.Edge(Evive.Value(ic)));
                        // to pass from 3D distance to a parametric distance
                        if (!tangentregul[ic])
                        {
                            ec = distance * 100 * C.Resolution(0.01);
                        }
                        else
                        {
                            ec = 0.0;
                        }
                        if (TopExp.FirstVertex(TopoDS.Edge(Evive.Value(ic))).IsSame(V1))
                        {
                            para = p.Value(ic, icmoins) + ec;
                            p.SetValue(ic, icmoins, para);
                        }
                        else
                        {
                            para = p.Value(ic, icmoins) - ec;
                            p.SetValue(ic, icmoins, para);
                        }
                        // it is necessary to be on to remain on the edge
                        p.SetValue(ic, icplus, p.Value(ic, icmoins));
                    }
                    else if (!distmini)
                    {
                        dist = dist1.Value(ic);
                        if ((!oksea.Value(icmoins)) || (oksea.Value(icmoins) && (distance > 1.3 * dist)))
                        {
                            ec = distance - dist;
                            if (oksea.Value(icmoins))
                            {
                                oksea.SetValue(icmoins, false);
                                inters = false;
                            }
                            if (sens.Value(ic) == 1)
                            {
                                para = p.Value(ic, icmoins) + ec;
                                p.SetValue(ic, icmoins, para);
                            }
                            else
                            {
                                para = p.Value(ic, icmoins) - ec;
                                p.SetValue(ic, icmoins, para);
                            }
                        }
                        dist = dist2.Value(ic);
                        if ((!oksea.Value(ic)) || (oksea.Value(ic) && (distance > 1.3 * dist)))
                        {
                            if (oksea.Value(ic))
                            {
                                oksea.SetValue(ic, false);
                                inters = false;
                            }
                            if (nconges != 1)
                            {
                                double parold = 0.0, parnew = 0.0;
                                parold = p.Value(ic, icplus);
                                parnew = p.Value(ic, icmoins);
                                if (sens.Value(ic) == 1)
                                {
                                    if (parnew > parold)
                                    {
                                        p.SetValue(ic, icplus, p.Value(ic, icmoins));
                                    }
                                }
                                else
                                {
                                    if (parnew < parold)
                                    {
                                        p.SetValue(ic, icplus, p.Value(ic, icmoins));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // it is attempted to limit the edge by a commonpoint
        //

        double tolcp = 0;
        gp_Pnt PE = new(), sommet = BRep_Tool.Pnt(V1);
        if (!deuxconges)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                if (sharp.Value(ic))
                {
                    Indices(nedge, ic, ref icplus, ref icmoins);
                    BRepAdaptor_Curve C = new(TopoDS.Edge(Evive.Value(ic)));
                    PE = C.Value(p.Value(ic, icplus));
                    double d1 = 0.0, d2 = 0.0, dS = PE.Distance(sommet);
                    ChFiDS_CommonPoint cp1 = new(), cp2 = new();
                    if (!sharp.Value(icplus))
                    {
                        isfirst = (sens.Value(icplus) == 1);
                        jfp = 3 - jf.Value(icplus);
                        cp1 = CD.Value(icplus)
                                        .SetOfSurfData()
                                        .Value(i.Value(icplus, ic))
                                        .ChangeVertex(isfirst, jfp);
                        d1 = cp1.Point().Distance(sommet);
                    }
                    if (!sharp.Value(icmoins))
                    {
                        isfirst = (sens.Value(icmoins) == 1);
                        cp2 = CD.Value(icmoins)
                                        .SetOfSurfData()
                                        .Value(i.Value(icmoins, ic))
                                        .ChangeVertex(isfirst, jf.Value(icmoins));
                        d2 = cp2.Point().Distance(sommet);
                    }
                    bool samecompoint = false;
                    if (!sharp.Value(icmoins) && !sharp.Value(icplus))
                    {
                        samecompoint = cp1.Point().Distance(cp2.Point()) < tolapp;
                    }
                    if ((dS < d1 || dS < d2) && !samecompoint)
                    {
                        // step back till Common Points
                        // without leaving the Edge ??
                        if (d2 < d1 && cp1.IsOnArc())
                        {
                            // cp1 is chosen
                            p.SetValue(ic, icmoins, cp1.ParameterOnArc());
                            p.SetValue(ic, icplus, p.Value(ic, icmoins));
                            isfirst = (sens.Value(icplus) == 1);
                            jfp = 3 - jf.Value(icplus);
                            Calcul_Param(CD.Value(icplus), jfp, i.Value(icplus, ic), isfirst, ref para);
                            p.SetValue(icplus, ic, para);
                            if (cp1.Tolerance() > tolcp && cp1.Tolerance() < 1)
                            {
                                tolcp = cp1.Tolerance();
                            }
                        }
                        else if (cp2.IsOnArc())
                        {
                            // cp2 is chosen
                            p.SetValue(ic, icmoins, cp2.ParameterOnArc());
                            p.SetValue(ic, icplus, p.Value(ic, icmoins));
                            isfirst = (sens.Value(icmoins) == 1);
                            Calcul_Param(CD.Value(icmoins), jf.Value(icmoins), i.Value(icmoins, ic), isfirst, ref para);
                            p.SetValue(icmoins, ic, para);
                            if (cp2.Tolerance() > tolcp && cp2.Tolerance() < 1)
                            {
                                tolcp = cp2.Tolerance();
                            }
                        }
                    }
                    else
                    {
                        // step back till Common Point only if it is very close
                        if (!sharp.Value(icplus))
                        {
                            if ((cp1.Point().Distance(PE) < cp1.Tolerance() || samecompoint || nconges == 1)
                                    && cp1.IsOnArc())
                            {
                                // it is very close to cp1
                                p.SetValue(ic, icmoins, cp1.ParameterOnArc());
                                ponctuel.SetValue(ic, true);
                                p.SetValue(ic, icplus, p.Value(ic, icmoins));
                                isfirst = (sens.Value(icplus) == 1);
                                jfp = 3 - jf.Value(icplus);
                                Calcul_Param(CD.Value(icplus), jfp, i.Value(icplus, ic), isfirst, ref para);
                                p.SetValue(icplus, ic, para);
                                if (cp1.Tolerance() > tolcp && cp1.Tolerance() < 1)
                                {
                                    tolcp = cp1.Tolerance();
                                }
                            }
                        }
                        if (!sharp.Value(icmoins))
                        {
                            if ((cp2.Point().Distance(PE) < cp2.Tolerance() || samecompoint || nconges == 1)
                                    && cp2.IsOnArc())
                            {
                                // it is very close to cp2
                                ponctuel.SetValue(icmoins, true);
                                p.SetValue(ic, icmoins, cp2.ParameterOnArc());
                                p.SetValue(ic, icplus, p.Value(ic, icmoins));
                                isfirst = (sens.Value(icmoins) == 1);
                                Calcul_Param(CD.Value(icmoins), jf.Value(icmoins), i.Value(icmoins, ic), isfirst, ref para);
                                p.SetValue(icmoins, ic, para);
                                if (cp2.Tolerance() > tolcp && cp2.Tolerance() < 1)
                                {
                                    tolcp = cp2.Tolerance();
                                }
                            }
                        }
                    }
                }
            }
        }

        // in case of a free border the parameter corresponding
        // to the common point on the free edge is chosen.

        for (ic = 0; ic < nedge; ic++)
        {
            if (TopoDS.Edge(Evive.Value(ic)).IsSame(edgelibre1)
                    || TopoDS.Edge(Evive.Value(ic)).IsSame(edgelibre2))
            {
                int indic = 0;
                ChFiDS_CommonPoint CP1 = new();
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (libre.Value(ic))
                {
                    indic = icmoins;
                }
                else
                {
                    indic = icplus;
                }
                if (!sharp[indic])
                {
                    isfirst = sens.Value(indic) == 1;
                    CP1 = CD.Value(indic).SetOfSurfData().Value(Index.Value(indic)).ChangeVertex(isfirst, 1);
                    /*bool*/ trouve = false;
                    if (CP1.IsOnArc())
                    {
                        if (CP1.Arc().IsSame(TopoDS.Edge(Evive.Value(ic))))
                        {
                            p.SetValue(ic, icmoins, CP1.ParameterOnArc());
                            p.SetValue(ic, icplus, CP1.ParameterOnArc());
                            trouve = true;
                        }
                    }
                    if (!trouve)
                    {
                        CP1 =
                            CD.Value(indic).SetOfSurfData().Value(Index.Value(indic)).ChangeVertex(isfirst, 2);
                        if (CP1.IsOnArc())
                        {
                            if (CP1.Arc().IsSame(TopoDS.Edge(Evive.Value(ic))))
                            {
                                p.SetValue(ic, icmoins, CP1.ParameterOnArc());
                                p.SetValue(ic, icplus, CP1.ParameterOnArc());
                            }
                        }
                    }
                }
            }
        }

        // if ic is a regular edge, one finds edge indfin which is not
        // a regular edge, and construtc a curve 3d
        // between edges (or stripes ) icmoins and indfin.
        // Then this courbe3d is projected on all faces (nbface) that
        // separate icmoins and indfin
        int nbface = 0;
        double error = 0.0;
        NCollection_Array1<Geom2d_Curve> proj2d1 = new(0, size);
        NCollection_Array1<Geom2d_Curve> proj2d2 = new(0, size);
        NCollection_Array1<Geom_Curve> cproj1 = new(0, size);
        NCollection_Array1<Geom_Curve> cproj2 = new(0, size);
        if (!deuxconges)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                int ilin = 0;
                NCollection_Sequence<Geom_Curve> cr = new();
                NCollection_Sequence<Geom2d_Curve> pr = new();
                NCollection_Sequence<TopoDS_Shape> Lface = new();
                NCollection_Sequence<TopoDS_Shape> Ledge = new();
                Lface.Clear();
                if (regul.Value(ic))
                {
                    Indices(nedge, ic, ref icplus, ref icmoins);
                    Indices(nedge, icplus, ref icplus2, ref ic);
                    int indfin = 0, indfinmoins = 0, indfinplus = 0;
                    indfin = icplus;
                    trouve = false;
                    ii = icplus;
                    while (!trouve)
                    {
                        if (!regul.Value(ii))
                        {
                            indfin = ii;
                            trouve = true;
                        }
                        if (ii == nedge - 1)
                        {
                            ii = 0;
                        }
                        else
                        {
                            ii++;
                        }
                    }
                    Indices(nedge, indfin, ref indfinplus, ref indfinmoins);
                    if (!sharp.Value(icmoins))
                    {
                        if (jf.Value(icmoins) == 1)
                        {
                            ilin = SurfIndex(CD, icmoins, i.Value(icmoins, ic), FACE1);
                        }
                        else
                        {
                            ilin = SurfIndex(CD, icmoins, i.Value(icmoins, ic), FACE2);
                        }
                        Lface.Append(TopoDS.Face(DStr.Shape(ilin)));
                    }
                    else
                    {
                        Lface.Append(Fvive[ic, icmoins]);
                    }
                    if (indfin > icmoins)
                    {
                        nbface = indfin - icmoins;
                    }
                    else
                    {
                        nbface = nedge - (icmoins - indfin);
                    }
                    NCollection_Sequence<TopoDS_Shape> Epj = new();
                    NCollection_Sequence<double> seqpr = new();
                    ii = ic;
                    for (int nf = 1; nf <= nbface - 1; nf++)
                    {
                        int iimoins = 0, iiplus = 0;
                        Indices(nedge, ii, ref iiplus, ref iimoins);
                        Ledge.Append(TopoDS.Edge(Evive.Value(ii)));
                        seqpr.Append(p.Value(ii, iiplus));
                        if (nf != nbface - 1)
                        {
                            Lface.Append(Fvive[ii, iiplus]);
                        }
                        if (ii == nedge - 1)
                        {
                            ii = 0;
                        }
                        else
                        {
                            ii++;
                        }
                    }
                    if (!sharp.Value(indfin))
                    {
                        jfp = 3 - jf.Value(indfin);
                        if (jfp == 1)
                        {
                            ilin = SurfIndex(CD, indfin, i.Value(indfin, indfinmoins), FACE1);
                        }
                        else
                        {
                            ilin = SurfIndex(CD, indfin, i.Value(indfin, indfinmoins), FACE2);
                        }
                        Lface.Append(TopoDS.Face(DStr.Shape(ilin)));
                    }
                    else
                    {
                        Lface.Append(Fvive[indfin, indfinmoins]);
                    }
                    CurveHermite(DStr, CD.Value(icmoins), jf.Value(icmoins), i.Value(icmoins, ic), p.Value(icmoins, ic),
                                              sens.Value(icmoins), sharp.Value(icmoins), TopoDS.Edge(Evive.Value(icmoins)), CD.Value(indfin),
                                              jf.Value(indfin), i.Value(indfin, indfinmoins), p.Value(indfin, indfinmoins), sens.Value(indfin),
                                              sharp.Value(indfin), TopoDS.Edge(Evive.Value(indfin)), nbface, Ledge, Lface, pr, cr, Epj, seqpr,
                                              ref error);
                    ii = ic;
                    for (ind = 1; ind <= nbface - 1; ind++)
                    {
                        int iimoins = 0, iiplus = 0;
                        Indices(nedge, ii, ref iiplus, ref iimoins);
                        p.SetValue(ii, iiplus, seqpr.Value(ind));
                        p.SetValue(ii, iimoins, seqpr.Value(ind));
                        proj2d1.SetValue(ii, pr.Value(ind));
                        proj2d2.SetValue(ii, pr.Value(ind + 1));
                        cproj1.SetValue(ii, cr.Value(ind));
                        cproj2.SetValue(ii, cr.Value(ind + 1));
                        if (ii == nedge - 1)
                        {
                            ii = 0;
                        }
                        else
                        {
                            ii++;
                        }
                    }
                    if (!sharp.Value(icmoins) && !sharp.Value(indfin))
                    {
                        ii = icmoins;
                        while (ii != indfin)
                        {
                            isG1.SetValue(ii, true);
                            if (ii == nedge - 1)
                            {
                                ii = 0;
                            }
                            else
                            {
                                ii++;
                            }
                        }
                    }
                    ic = ic + nbface - 1;
                }
            }
        }

        // case when the connecting curve between ic and icplus crosses many faces

        NCollection_Sequence<TopoDS_Shape> Ecom = new();
        NCollection_Sequence<TopoDS_Shape> Eproj = new();
        NCollection_Sequence<double> parcom = new();
        if (!deuxconges)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                int iface1 = 0, iface2 = 0;
                TopoDS_Face face1 = new(), face2 = new();
                TopoDS_Edge edge = new();
                NCollection_Sequence<Geom_Curve> cr = new();
                NCollection_Sequence<Geom2d_Curve> pr = new();
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (!oksea.Value(ic))
                {
                    iface1 = numfa.Value(ic, icplus);
                    iface2 = numfa.Value(icplus, ic);
                    if (!sharp.Value(ic))
                    {
                        if (jf.Value(ic) == 1)
                        {
                            iface1 = SurfIndex(CD, ic, i.Value(ic, icplus), FACE1);
                        }
                        else
                        {
                            iface1 = SurfIndex(CD, ic, i.Value(ic, icplus), FACE2);
                        }
                    }
                    face1 = TopoDS.Face(myDS.Shape(iface1));

                    if (!sharp.Value(icplus))
                    {
                        if (jf.Value(icplus) == 1)
                        {
                            iface2 = SurfIndex(CD, icplus, i.Value(icplus, ic), FACE2);
                        }
                        else
                        {
                            iface2 = SurfIndex(CD, icplus, i.Value(icplus, ic), FACE1);
                        }
                    }
                    face2 = TopoDS.Face(myDS.Shape(iface2));
                    if (!face1.IsSame(face2))
                    {
                        if (Fproj.Length() == 0)
                        {
                            Fproj.Append(face1);
                            Fproj.Append(face2);
                        }
                        moresurf.SetValue(ic, true);
                        nbface = Fproj.Length();
                        if (!TopoDS.Face(Fproj.Value(nbface)).IsSame(face2))
                        {
                            Fproj.Remove(nbface);
                            Fproj.Append(face2);
                        }
                        if (!TopoDS.Face(Fproj.Value(1)).IsSame(face1))
                        {
                            Fproj.Remove(1);
                            Fproj.Prepend(face1);
                        }
                        for (nb = 1; nb <= nbface - 1; nb++)
                        {
                            cherche_edge1(TopoDS.Face(Fproj.Value(nb)), TopoDS.Face(Fproj.Value(nb + 1)), ref edge);
                            Ecom.Append(edge);
                            para = BRep_Tool.Parameter(TopExp.FirstVertex(edge), edge);
                            parcom.Append(para);
                        }
                        CurveHermite(DStr, CD.Value(ic), jf.Value(ic), i.Value(ic, icplus), p.Value(ic, icplus), sens.Value(ic),
                                                  sharp.Value(ic), TopoDS.Edge(Evive.Value(ic)), CD.Value(icplus), jf.Value(icplus),
                                                  i.Value(icplus, ic), p.Value(icplus, ic), sens.Value(icplus), sharp.Value(icplus),
                                                  TopoDS.Edge(Evive.Value(icplus)), nbface, Ecom, Fproj, pr, cr, Eproj, parcom, ref error);
                        Ecom.Append(Ecom.Value(nbface - 1));
                        parcom.Append(parcom.Value(nbface - 1));
                    }
                }
            }
        }

        // case when two fillets have the same commonpoints
        // one continues then by intersection
        // it is checked if the extremities of the intersection coincide with commonpoints

        bool intersection = false, introuve = false;
        if (nconges == 2 && !deuxconges)
        {
            gp_Pnt P1 = new(), P2 = new(), P3 = new(), P4 = new();
            int ic1 = 0, ic2 = 0;
            trouve = false;
            for (ic = 0; ic < nedge && !trouve; ic++)
            {
                if (!sharp.Value(ic))
                {
                    ic1 = ic;
                    trouve = true;
                }
            }
            for (ic = 0; ic < nedge; ic++)
            {
                if (!sharp.Value(ic) && ic != ic1)
                {
                    ic2 = ic;
                }
            }
            jfp = 3 - jf.Value(ic1);
            Indices(nedge, ic1, ref icplus, ref icmoins);
            Calcul_P2dOnSurf(CD.Value(ic1), jfp, i.Value(ic1, icmoins), p.Value(ic1, icmoins), ref p2);
            indice = SurfIndex(CD, ic1, i.Value(ic1, icmoins), ChFiSURFACE);
            DStr.Surface(indice).Surface().D0(p2.X(), p2.Y(), out P1);

            Calcul_P2dOnSurf(CD.Value(ic1), jf.Value(ic1), i.Value(ic1, icplus), p.Value(ic1, icplus), ref p2);
            indice = SurfIndex(CD, ic1, i.Value(ic1, icplus), ChFiSURFACE);
            DStr.Surface(indice).Surface().D0(p2.X(), p2.Y(), out P2);

            jfp = 3 - jf.Value(ic2);
            Indices(nedge, ic2, ref icplus, ref icmoins);
            Calcul_P2dOnSurf(CD.Value(ic2), jfp, i.Value(ic2, icmoins), p.Value(ic2, icmoins), ref p2);
            indice = SurfIndex(CD, ic2, i.Value(ic2, icmoins), ChFiSURFACE);
            DStr.Surface(indice).Surface().D0(p2.X(), p2.Y(), out P3);

            Calcul_P2dOnSurf(CD.Value(ic2), jf.Value(ic2), i.Value(ic2, icplus), p.Value(ic2, icplus), ref p2);
            indice = SurfIndex(CD, ic2, i.Value(ic2, icplus), ChFiSURFACE);
            DStr.Surface(indice).Surface().D0(p2.X(), p2.Y(), out P4);
            intersection = (P1.Distance(P4) <= 1.0e-7 || P1.Distance(P3) <= 1.0e-7)
                                          && (P2.Distance(P4) <= 1.0e-7 || P2.Distance(P3) <= 1.0e-7);
            if (intersection)
            {
                PerformTwoCornerSameExt(DStr, CD.Value(ic1), Index.Value(ic1), sens.Value(ic1), CD.Value(ic2), Index.Value(ic2),
                                                                sens.Value(ic2), ref introuve);
                if (introuve)
                {
                    return;
                }
            }
        }

        // declaration for plate
        // GeomPlate_BuildPlateSurface PSurf(3,10,3,tol2d,tolesp,angular);
        //
        // Sence of Plate parameters and their preferable values :
        // degree is total order of ordinary or mixed derivatives:
        // dS/dU, dS/dV have degree 1, d2S/dU2, d2S/dV2, d2S/(dUdV) have degree 2
        // nbiter - number of iterations, when surface from previous iteration uses as initial surface for
        // next one practically this process does not converge, using "bad" initial surface leads to much
        // more "bad" solution. constr is order of constraint: 0 - G0, 1 - G1 ... Using constraint order >
        // 0 very often causes unpredictable undulations of solution
        int degree = 3, nbcurvpnt = 10, nbiter = 1;
        int constr = 1;// G1
        GeomPlate_BuildPlateSurface PSurf = new(degree, nbcurvpnt, nbiter, tol2d, tolapp3d, angular);
        // calculation of curves on surface for each stripe
        for (ic = 0; ic < nedge; ic++)
        {
            gp_Pnt2d p2d1 = new(), p2d2 = new();
            if (!sharp.Value(ic))
            {
                n3d++;
                Indices(nedge, ic, ref icplus, ref icmoins);
                jfp = 3 - jf.Value(ic);
                Calcul_P2dOnSurf(CD.Value(ic), jfp, i.Value(ic, icmoins), p.Value(ic, icmoins), ref p2d1);
                Calcul_P2dOnSurf(CD.Value(ic), jf.Value(ic), i.Value(ic, icplus), p.Value(ic, icplus), ref p2d2);
                //      if (i[ic][icplus]!=  i[ic][icmoins]) std::cout<<"probleme surface"<<std::endl;
                indice = SurfIndex(CD, ic, i.Value(ic, icplus), ChFiSURFACE);
                GeomAdaptor_Surface? Asurf = 
                    new GeomAdaptor_Surface(DStr.Surface(indice).Surface());
                // calculation of curve 2d
                xdir = p2d2.X() - p2d1.X();
                ydir = p2d2.Y() - p2d1.Y();
                double l0 = Math.Sqrt(xdir * xdir + ydir * ydir);
                gp_Dir2d dir = new(xdir, ydir);
                Geom2d_Line? l = new Geom2d_Line(p2d1, dir);
                Geom2d_Curve? pcurve = new Geom2d_TrimmedCurve(l, 0, l0);
                Geom2dAdaptor_Curve? Acurv = new Geom2dAdaptor_Curve(pcurve);
                Adaptor3d_CurveOnSurface CurvOnS = new(Acurv, Asurf);
                Adaptor3d_CurveOnSurface? HCons = (Adaptor3d_CurveOnSurface)CurvOnS.ShallowCopy(); // OCCT copy-constructs a handle
                // Order.SetValue(ic,1);
                Order.SetValue(ic, constr);
                GeomPlate_CurveConstraint? Cont = 
                    new GeomPlate_CurveConstraint(HCons, Order.Value(ic), nbcurvpnt, tolapp3d, angular, 0.1);
                PSurf.Add(Cont);

                // calculate indexes of points and of the curve for the DS
                isfirst = (sens.Value(ic) == 1);
                GeomLib.BuildCurve3d(tolapp, CurvOnS, CurvOnS.FirstParameter(), CurvOnS.LastParameter(), ref Curv3d, out maxapp,
                                                            out avedev);
                TopOpeBRepDS_Curve tcurv3d = new(Curv3d, maxapp);
                indcurve3d.SetValue(n3d, DStr.AddCurve(tcurv3d));
                gp_Pnt point1 = new(), point2 = new();
                point1 = CurvOnS.Value(CurvOnS.FirstParameter());
                point2 = CurvOnS.Value(CurvOnS.LastParameter());

                TopOpeBRepDS_Point tpoint1 = new(point1, maxapp);
                TopOpeBRepDS_Point tpoint2 = new(point2, maxapp);
                errapp.SetValue(ic, maxapp);
                if (ic == 0)
                {
                    // it is necessary to create two points
                    indpoint.SetValue(ic, 0, DStr.AddPoint(tpoint1));
                    indpoint.SetValue(ic, 1, DStr.AddPoint(tpoint2));
                }
                else
                {
                    // probably the points are already on the fillet
                    // (previous intersection...)
                    trouve = false;
                    for (ii = 0; ii < ic && (!trouve); ii++)
                    {
                        if (!sharp.Value(ii))
                        {
                            ref TopOpeBRepDS_Point tpt = ref DStr.ChangePoint(indpoint.Value(ii, 1));
                            if (point1.Distance(tpt.Point()) < 1.0e-4)
                            {
                                trouve = true;
                            }
                        }
                    }
                    if (trouve)
                    {
                        indpoint.SetValue(ic, 0, indpoint.Value(ii - 1, 1));
                    }
                    else
                    {
                        indpoint.SetValue(ic, 0, DStr.AddPoint(tpoint1));
                    }

                    trouve = false;
                    for (ii = 0; ii < ic && (!trouve); ii++)
                    {
                        if (!sharp.Value(ii))
                        {
                            ref TopOpeBRepDS_Point tpt = ref DStr.ChangePoint(indpoint.Value(ii, 0));
                            if (point2.Distance(tpt.Point()) < 1.0e-4)
                            {
                                trouve = true;
                            }
                        }
                    }
                    if (trouve)
                    {
                        indpoint.SetValue(ic, 1, indpoint.Value(ii - 1, 0));
                    }
                    else
                    {
                        indpoint.SetValue(ic, 1, DStr.AddPoint(tpoint2));
                    }
                }

                //   update of the stripe
                isurf1 = 3 - jf.Value(ic);
                isurf2 = jf.Value(ic);
                if (isurf1 == 2)
                {
                    CD.Value(ic).SetOrientation(TopAbs_REVERSED, isfirst);
                }
                CD.Value(ic).SetCurve(indcurve3d.Value(n3d), isfirst);
                CD.Value(ic).SetIndexPoint(indpoint.Value(ic, 0), isfirst, isurf1);
                CD.Value(ic).SetIndexPoint(indpoint.Value(ic, 1), isfirst, isurf2);
                CD.Value(ic).SetParameters(isfirst, pcurve.FirstParameter(), pcurve.LastParameter());
                ChFiDS_CommonPoint cp1 = new();
                ChFiDS_CommonPoint cp2 = new();
                cp1.SetPoint(point1);
                cp2.SetPoint(point2);
                CD.Value(ic).SetOfSurfData().Value(i.Value(ic, icmoins)).ChangeVertex(isfirst, isurf1) =
                    cp1;
                CD.Value(ic).SetOfSurfData().Value(i.Value(ic, icmoins)).ChangeVertex(isfirst, isurf2) =
                    cp2;
                CD.Value(ic)
                    .SetOfSurfData()
                    .Value(i.Value(ic, icmoins))
                    .ChangeInterference(isurf1)
                    .SetParameter(p.Value(ic, icmoins), isfirst);
                CD.Value(ic)
                    .SetOfSurfData()
                    .Value(i.Value(ic, icmoins))
                    .ChangeInterference(isurf2)
                    .SetParameter(p.Value(ic, icplus), isfirst);
                CD.Value(ic).ChangePCurve(isfirst) = pcurve;
            }
        }

        // calculate the indices of points for living edges
        for (ic = 0; ic < nedge; ic++)
        {
            if (sharp.Value(ic))
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                BRepAdaptor_Curve C = new(TopoDS.Edge(Evive.Value(ic)));
                /*gp_Pnt*/ PE = C.Value(p.Value(ic, icplus));
                TopOpeBRepDS_Point TPE = new(PE, BRep_Tool.Tolerance(TopoDS.Edge(Evive.Value(ic))));
                ChFiDS_CommonPoint cp = new();
                if (deuxconges)
                {
                    IVtx = DStr.AddShape(V1);
                    indpoint.SetValue(ic, 0, IVtx);
                    indpoint.SetValue(ic, 1, IVtx);
                }
                if (!sharp.Value(icplus))
                {
                    isfirst = (sens.Value(icplus) == 1);
                    jfp = 3 - jf.Value(icplus);
                    cp =
                        CD.Value(icplus).SetOfSurfData().Value(i.Value(icplus, ic)).ChangeVertex(isfirst, jfp);
                    if (cp.Point().Distance(PE) <= Max(1.0e-4, tolcp))
                    {
                        // edge was limited by the 1st CommonPoint of CD[icplus]
                        indpoint.SetValue(ic, 0, indpoint.Value(icplus, 0));
                        indpoint.SetValue(ic, 1, indpoint.Value(icplus, 0));
                    }
                }
                if (!sharp.Value(icmoins))
                {
                    isfirst = (sens.Value(icmoins) == 1);
                    cp = CD.Value(icmoins)
                                  .SetOfSurfData()
                                  .Value(i.Value(icmoins, ic))
                                  .ChangeVertex(isfirst, jf.Value(icmoins));
                    if (cp.Point().Distance(PE) <= Max(1.0e-4, tolcp))
                    {
                        // edge was limited by the 2nd CommonPoint of CD[icmoins]
                        if (indpoint.Value(ic, 0) == 0)
                        {
                            indpoint.SetValue(ic, 0, indpoint.Value(icmoins, 1));
                            indpoint.SetValue(ic, 1, indpoint.Value(icmoins, 1));
                        }
                    }
                }
                if (indpoint.Value(ic, 0) == 0)
                {
                    indpoint.SetValue(ic, 0, DStr.AddPoint(TPE));
                    indpoint.SetValue(ic, 1, indpoint.Value(ic, 0));
                }
            }
        }

        // calculation of intermediary curves connecting two stripes in case if
        // there is no intersection. The curve is a straight line, projection or batten

        bool raccordbatten = false;
        if (!inters)
        {

            for (ic = 0; ic < nedge; ic++)
            {

                if (!oksea.Value(ic) && !moresurf.Value(ic) && !libre.Value(ic))
                {
                    Indices(nedge, ic, ref icplus, ref icmoins);
                    raccordbatten = false;
                    if (!regul.Value(ic))
                    {
                        raccordbatten = true;
                        if (regul.Value(icplus))
                        {
                            raccordbatten = false;
                        }
                    }
                    n3d++;
                    gp_Pnt2d p2d1 = new(), p2d2 = new();
                    Geom2d_Curve? curv2d1 = null, curv2d2 = null;
                    Geom2d_Curve? pcurve = null;
                    Geom_Curve? curveint = null;
                    GeomAdaptor_Surface? Asurf = null;
                    double u1bid = 0.0, u2bid = 0.0;

                    // return the 1st curve 2d
                    // and the 1st connection point
                    if (sharp.Value(ic))
                    {
                        curv2d1 = BRep_Tool.CurveOnSurface(TopoDS.Edge(Evive.Value(ic)), TopoDS.Face(Fvive.Value(ic, icplus)),
                                                                                                out u1bid, out u2bid);
                    }
                    else
                    {
                        Calcul_C2dOnFace(CD.Value(ic), jf.Value(ic), i.Value(ic, icplus), ref curv2d1);
                    }

                    if (curv2d1 is null)
                    {
                        throw new Standard_ConstructionError("Failed to get p-curve of edge");
                    }
                    p2d1 = curv2d1.Value(p.Value(ic, icplus));

                    // recuperation de la deuxieme courbe 2d
                    // et du deuxieme point de raccordement
                    if (sharp.Value(icplus))
                    {
                        curv2d2 = BRep_Tool.CurveOnSurface(TopoDS.Edge(Evive.Value(icplus)), TopoDS.Face(Fvive.Value(ic, icplus)),
                                                                                                out u1bid, out u2bid);
                    }
                    else
                    {
                        jfp = 3 - jf.Value(icplus);
                        Calcul_C2dOnFace(CD.Value(icplus), jfp, i.Value(icplus, ic), ref curv2d2);
                    }
                    if (curv2d2 is null)
                    {
                        throw new Standard_ConstructionError("Failed to get p-curve of edge");
                    }
                    p2d2 = curv2d2.Value(p.Value(icplus, ic));

                    Asurf = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(Fvive.Value(ic, icplus))));
                    double tolu = 0.0, tolv = 0.0, ratio = 0.0;
                    tolu = Asurf.UResolution(1.0e-3);
                    tolv = Asurf.VResolution(1.0e-3);
                    if (tolu > tolv)
                    {
                        ratio = tolu / tolv;
                    }
                    else
                    {
                        ratio = tolv / tolu;
                    }

                    // in case of a sewing edge the parameters are reframed
                    if (couture)
                    {
                        bool PI1 = false, PI2 = false;
                        double xx = 0.0;
                        PI1 = 0 <= p2d1.X() && p2d1.X() <= M_PI;
                        PI2 = 0 <= p2d2.X() && p2d2.X() <= M_PI;

                        if (Evive.Value(ic).IsSame(edgecouture))
                        {
                            xx = p2d1.X();
                            if (PI2 && !PI1)
                            {
                                xx = xx - 2 * M_PI;
                            }
                            if (!PI2 && PI1)
                            {
                                xx = xx + 2 * M_PI;
                            }
                            p2d1.SetX(xx);
                        }
                        if (Evive.Value(icplus).IsSame(edgecouture))
                        {
                            xx = p2d2.X();
                            if (PI2 && !PI1)
                            {
                                xx = xx + 2 * M_PI;
                            }
                            if (!PI2 && PI1)
                            {
                                xx = xx - 2 * M_PI;
                            }
                            p2d2.SetX(xx);
                        }
                    }
                    xdir = p2d2.X() - p2d1.X();
                    ydir = p2d2.Y() - p2d1.Y();

                    double l0 = Math.Sqrt(xdir * xdir + ydir * ydir);
                    if (l0 < 1.0e-7 || ponctuel.Value(ic))
                    {
                        // unused connection
                        n3d--;
                        ponctuel.SetValue(ic, true);
                        if (!deuxconges)
                        {
                            if (sharp.Value(icplus) && indpoint.Value(icplus, 0) == 0)
                            {
                                indpoint.SetValue(icplus, 0, indpoint.Value(ic, 1));
                                indpoint.SetValue(icplus, 1, indpoint.Value(ic, 1));
                            }
                            if (sharp.Value(ic) && indpoint.Value(ic, 0) == 0)
                            {
                                indpoint.SetValue(ic, 0, indpoint.Value(icmoins, 1));
                                indpoint.SetValue(ic, 1, indpoint.Value(icmoins, 1));
                            }
                        }
                    }
                    else
                    { // the connection is a straight line, projection or batten
                        if (ratio > 10 && nconges == 1)
                        {
                            raccordbatten = true;
                        }
                        if (ratio > 10 && raccordbatten)
                        {
                            CalculDroite(p2d1, xdir, ydir, ref pcurve);
                            raccordbatten = false;
                        }
                        else if (!raccordbatten)
                        { // the projected curves are returned
                            if (regul.Value(ic))
                            {
                                if (cproj2.Value(ic) is null)
                                {
                                    raccordbatten = true;
                                }
                                else
                                {
                                    pcurve = proj2d2.Value(ic);
                                    curveint = cproj2.Value(ic);
                                    maxapp1 = 1.0e-6;
                                }
                            }
                            else
                            {
                                if (cproj1.Value(ic + 1) is null)
                                {
                                    raccordbatten = true;
                                }
                                else
                                {
                                    pcurve = proj2d1.Value(ic + 1);
                                    curveint = cproj1.Value(ic + 1);
                                    maxapp1 = 1.0e-6;
                                }
                            }
                        }
                        bool contraint1 = true, contraint2 = true;
                        if (raccordbatten)
                        {
                            bool inverseic = false, inverseicplus = false;
                            if (sharp.Value(ic))
                            {
                                inverseic = TopExp.FirstVertex(TopoDS.Edge(Evive.Value(ic))).IsSame(V1);
                            }
                            else
                            {
                                inverseic = sens.Value(ic) == 1;
                            }
                            if (sharp.Value(icplus))
                            {
                                inverseicplus = TopExp.FirstVertex(TopoDS.Edge(Evive.Value(icplus))).IsSame(V1);
                            }
                            else
                            {
                                inverseicplus = sens.Value(icplus) == 1;
                            }
                            if (TopoDS.Edge(Evive.Value(ic)).IsSame(edgelibre1)
                                    || TopoDS.Edge(Evive.Value(ic)).IsSame(edgelibre2))
                            {
                                contraint1 = false;
                            }
                            if (TopoDS.Edge(Evive.Value(icplus)).IsSame(edgelibre1)
                                    || TopoDS.Edge(Evive.Value(icplus)).IsSame(edgelibre2))
                            {
                                contraint2 = false;
                            }
                            CalculBatten(Asurf, TopoDS.Face(Fvive[ic, icplus]), xdir, ydir, p2d1, p2d2, contraint1, contraint2,
                                                      curv2d1, curv2d2, p.Value(ic, icplus), p.Value(icplus, ic), inverseic, inverseicplus,
                                                      ref pcurve);
                        }

                        // construction of borders for Plate
                        Geom2dAdaptor_Curve? Acurv = new Geom2dAdaptor_Curve(pcurve);
                        Adaptor3d_CurveOnSurface CurvOnS = new(Acurv, Asurf);
                        Adaptor3d_CurveOnSurface? HCons = (Adaptor3d_CurveOnSurface)CurvOnS.ShallowCopy(); // OCCT copy-constructs a handle

                        // constraints G1 are set if edges ic and icplus are not both alive

                        Order.SetValue(n3d, 0);
                        if (!sharp.Value(ic) && !sharp.Value(icplus))
                        {
                            Order.SetValue(n3d, 1);
                        }
                        if (!contraint1 && !sharp.Value(icplus))
                        {
                            Order.SetValue(n3d, 1);
                        }
                        if (!contraint2 && !sharp.Value(ic))
                        {
                            Order.SetValue(n3d, 1);
                        }
                        if (tangentregul[ic] || tangentregul[icplus])
                        {
                            Order.SetValue(n3d, 1);
                        }
                        if (isG1.Value(ic))
                        {
                            Order.SetValue(n3d, 1);
                        }
                        GeomPlate_CurveConstraint? Cont = 
                            new GeomPlate_CurveConstraint(HCons, Order.Value(n3d), 10, tolapp3d, angular, 0.1);
                        PSurf.Add(Cont);

                        // calculation of curve 3d if it is not a projection
                        if (curveint is null)
                        {
                            GeomLib.BuildCurve3d(tolapp, CurvOnS, CurvOnS.FirstParameter(), CurvOnS.LastParameter(), ref Curv3d,
                                                                        out maxapp1, out avedev);
                            pardeb = CurvOnS.FirstParameter();
                            parfin = CurvOnS.LastParameter();
                            curveint = new Geom_TrimmedCurve(Curv3d, pardeb, parfin);
                        }

                        // storage in the DS
                        TopOpeBRepDS_Curve tcurv3d = new(curveint, maxapp1);
                        indcurve3d.SetValue(n3d, DStr.AddCurve(tcurv3d));
                        pardeb = curveint.FirstParameter();
                        parfin = curveint.LastParameter();
                        if (sharp.Value(icplus) && indpoint.Value(icplus, 0) == 0)
                        {
                            // it is necessary to initialize indpoint[icplus][0] and indpoint[icplus][1]
                            gp_Pnt point2 = new();
                            point2 = curveint.Value(parfin);
                            TopOpeBRepDS_Point tpoint2 = new(point2, maxapp);
                            indpoint.SetValue(icplus, 0, DStr.AddPoint(tpoint2));
                            indpoint.SetValue(icplus, 1, indpoint.Value(icplus, 0));
                        }
                        bool IsVt1 = false;
                        bool IsVt2 = false;
                        if (deuxconges)
                        {
                            IsVt1 = sharp.Value(ic);
                            IsVt2 = sharp.Value(icplus);
                        }
                        Interfp1 = ChFi3d_FilPointInDS(TopAbs_FORWARD, indcurve3d.Value(n3d), indpoint.Value(ic, 1), pardeb, IsVt1);
                        Interfp2 = ChFi3d_FilPointInDS(TopAbs_REVERSED, indcurve3d.Value(n3d), indpoint[icplus, 0], parfin, IsVt2);
                        DStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(Interfp1);
                        DStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(Interfp2);
                        if (!IsVt1)
                        {
                            ref TopOpeBRepDS_Point tpt1 = ref DStr.ChangePoint(indpoint[ic, 1]);
                            tpt1.Tolerance(tpt1.Tolerance() + maxapp1);
                        }
                        if (!IsVt2)
                        {
                            ref TopOpeBRepDS_Point tpt2 = ref DStr.ChangePoint(indpoint[icplus, 0]);
                            tpt2.Tolerance(tpt2.Tolerance() + maxapp1);
                        }

                        // calculate orientation of the curve
                        TopAbs_Orientation orinterf = default;
                        if (!sharp.Value(ic))
                        {
                            OrientationIcNonVive(CD.Value(ic), jf.Value(ic), i.Value(ic, icplus), sens.Value(ic), ref orinterf);
                        }
                        else if (!sharp.Value(icplus))
                        {
                            OrientationIcplusNonVive(CD.Value(icplus), jf.Value(icplus), i.Value(icplus, ic), sens.Value(icplus),
                                                                              ref orinterf);
                        }
                        else
                        {
                            OrientationAreteViveConsecutive(Fvive.Value(ic, icplus), Evive.Value(ic), V1, ref orinterf);
                        }
                        Interfc =
                            ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), numfa.Value(ic, icplus), pcurve, orinterf);
                        DStr.ChangeShapeInterferences(numfa.Value(ic, icplus)).Append(Interfc);
                    }
                } // end of processing by edge
            } // end of the loop on edges
        } // end of processing for intermediary curves

        //  storage in the DS of curves projected on several faces
        for (ic = 0; ic < nedge; ic++)
        {
            if (moresurf.Value(ic))
            {
                TopoDS_Vertex Vf = new(), Vl = new();
                gp_Pnt Pf = new(), Pl = new(), P1 = new(), P2 = new(), Pcom = new();
                ind = 0; // must be initialized because of possible use, see L2249
                double up1 = 0.0, up2 = 0.0;
                TopAbs_Orientation orvt = default;
                TopAbs_Orientation oredge = TopAbs_FORWARD;
                int indpoint1 = 0, indpoint2 = 0;
                Indices(nedge, ic, ref icplus, ref icmoins);
                Geom2d_Curve? proj = null, proj2d = null;
                Geom_Curve? projc = null, cproj = null;
                ref TopOpeBRepDS_Point tpt1 = ref DStr.ChangePoint(indpoint[ic, 1]);
                ref TopOpeBRepDS_Point tpt2 = ref DStr.ChangePoint(indpoint[icplus, 0]);
                tpt1.Tolerance(tpt1.Tolerance() + error);
                tpt2.Tolerance(tpt1.Tolerance() + error); // OCCT: tpt1's tolerance given to tpt2
                for (nb = 1; nb <= nbface; nb++)
                {
                    orvt = TopAbs_REVERSED;
                    Vf = TopExp.FirstVertex(TopoDS.Edge(Ecom.Value(nb)));
                    Vl = TopExp.LastVertex(TopoDS.Edge(Ecom.Value(nb)));
                    Pf = BRep_Tool.Pnt(Vf);
                    Pl = BRep_Tool.Pnt(Vl);
                    para = parcom.Value(nb);
                    Pcom = BRep_Tool.Curve(TopoDS.Edge(Ecom.Value(nb)), out up1, out up2).Value(para);
                    if (Pf.Distance(BRep_Tool.Pnt(V1)) < Pl.Distance(BRep_Tool.Pnt(V1)))
                    {
                        orvt = TopAbs_FORWARD;
                    }
                    if (!Eproj.Value(nb).IsNull())
                    {
                        n3d++;
                        proj = BRep_Tool.CurveOnSurface(TopoDS.Edge(Eproj.Value(nb)), TopoDS.Face(Fproj.Value(nb)), out up1,
                                                                                          out up2);
                        if (proj is null)
                        {
                            throw new Standard_ConstructionError("Failed to get p-curve of edge");
                        }
                        proj2d = new Geom2d_TrimmedCurve(proj, up1, up2);
                        projc = BRep_Tool.Curve(TopoDS.Edge(Eproj.Value(nb)), out up1, out up2);
                        if (projc is null)
                        {
                            throw new Standard_ConstructionError("Failed to get 3D curve of edge");
                        }
                        cproj = new Geom_TrimmedCurve(projc, up1, up2);
                        pardeb = cproj.FirstParameter();
                        parfin = cproj.LastParameter();
                        P1 = cproj.Value(pardeb);
                        P2 = cproj.Value(parfin);
                        if (P1.Distance(tpt1.Point()) < 1.0e-3)
                        {
                            indpoint1 = indpoint[ic, 1];
                        }
                        else
                        {
                            indpoint1 = ind;
                        }
                        if (P2.Distance(tpt2.Point()) < 1.0e-3)
                        {
                            indpoint2 = indpoint[icplus, 0];
                        }
                        else
                        {
                            TopOpeBRepDS_Point tpoint2 = new(P2, error);
                            indpoint2 = DStr.AddPoint(tpoint2);
                            ind = indpoint2;
                        }
                        GeomAdaptor_Surface? Asurf = null;
                        Asurf = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(Fproj.Value(nb))));
                        Geom2dAdaptor_Curve? Acurv = new Geom2dAdaptor_Curve(proj2d);
                        Adaptor3d_CurveOnSurface CurvOnS = new(Acurv, Asurf);
                        Adaptor3d_CurveOnSurface? HCons = (Adaptor3d_CurveOnSurface)CurvOnS.ShallowCopy(); // OCCT copy-constructs a handle
                        Order.SetValue(n3d, 1);
                        GeomPlate_CurveConstraint? Cont = 
                            new GeomPlate_CurveConstraint(HCons, Order.Value(n3d), 10, tolapp3d, angular, 0.1);
                        PSurf.Add(Cont);
                        TopOpeBRepDS_Curve tcurv3d = new(cproj, error);
                        indcurve3d.SetValue(n3d, DStr.AddCurve(tcurv3d));
                        Interfp1 = ChFi3d_FilPointInDS(TopAbs_FORWARD, indcurve3d.Value(n3d), indpoint1, pardeb);
                        Interfp2 = ChFi3d_FilPointInDS(TopAbs_REVERSED, indcurve3d.Value(n3d), indpoint2, parfin);
                        DStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(Interfp1);
                        DStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(Interfp2);
                        num = DStr.AddShape(Fproj.Value(nb));
                        TopExp_Explorer ex = new();
                        for (ex.Init(Fproj.Value(nb).Oriented(TopAbs_FORWARD), TopAbs_EDGE); ex.More(); ex.Next())
                        {
                            if (Ecom.Value(nb).IsSame(ex.Current()))
                            {
                                oredge = ex.Current().Orientation();
                                break;
                            }
                        }

                        // calculation of the orientation
                        TopAbs_Orientation orinterf = default;
                        if (P1.Distance(Pcom) > 1.0e-4)
                        {
                            if (orvt == TopAbs_FORWARD)
                            {
                                orinterf = oredge;
                            }
                            else
                            {
                                orinterf = TopAbs.Reverse(oredge);
                            }
                        }
                        else
                        {
                            if (orvt == TopAbs_FORWARD)
                            {
                                orinterf = TopAbs.Reverse(oredge);
                            }
                            else
                            {
                                orinterf = oredge;
                            }
                        }
                        Interfc = ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), num, proj2d, orinterf);
                        DStr.ChangeShapeInterferences(num).Append(Interfc);
                    }
                    indice = ind;
                    if (nb != nbface)
                    {
                        if (Eproj.Value(nb).IsNull())
                        {
                            indice = indpoint[ic, 1];
                        }
                        if (Eproj.Value(nb + 1).IsNull())
                        {
                            indice = indpoint[icplus, 0];
                        }
                        Indice.SetValue(n3d, indice);
                        int Iarc1 = DStr.AddShape(TopoDS.Edge(Ecom.Value(nb)));
                        Interfp1 = ChFi3d_FilPointInDS(orvt, Iarc1, indice, parcom.Value(nb));
                        DStr.ChangeShapeInterferences(Iarc1).Append(Interfp1);
                    }
                }
            }
        }

        // case when two free borders are tangent
        if (droit)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                Geom_Curve? curve = null, ctrim = null; // OCCT's unused rcurve and rcurve2d left out
                Geom2d_Curve? curve2d = null, ctrim2d = null;
                double ufirst = 0.0, ulast = 0.0;
                Indices(nedge, ic, ref icplus, ref icmoins);
                int indpoint1 = 0, indpoint2 = 0;
                bool isvt1 = false, isvt2 = false;
                TopoDS_Edge ecur = TopoDS.Edge(Evive.Value(ic));
                if (ecur.IsSame(edgelibre1) || ecur.IsSame(edgelibre2))
                {
                    n3d++;
                    curve2d = BRep_Tool.CurveOnSurface(TopoDS.Edge(Evive.Value(ic)), TopoDS.Face(Fvive.Value(ic, icplus)),
                                                                                            out ufirst, out ulast);
                    curve = BRep_Tool.Curve(TopoDS.Edge(Evive.Value(ic)), out ufirst, out ulast);
                    if (TopExp.FirstVertex((TopoDS.Edge(Evive.Value(ic)))).IsSame(V1))
                    {
                        ctrim = new Geom_TrimmedCurve(curve, ufirst, p.Value(ic, icmoins));
                        ctrim2d = new Geom2d_TrimmedCurve(curve2d, ufirst, p.Value(ic, icmoins));
                        indpoint1 = DStr.AddShape(V1);
                        isvt1 = true;
                        indpoint2 = indpoint[ic, 1];
                    }
                    else
                    {
                        ctrim = new Geom_TrimmedCurve(curve, p.Value(ic, icmoins), ulast);
                        ctrim2d = new Geom2d_TrimmedCurve(curve2d, p.Value(ic, icmoins), ulast);
                        indpoint2 = DStr.AddShape(V1);
                        isvt2 = true;
                        indpoint1 = indpoint[ic, 1];
                    }
                    if (libre.Value(ic))
                    {
                        if (TopExp.FirstVertex((TopoDS.Edge(Evive.Value(ic)))).IsSame(V1))
                        {
                            ctrim.Reverse();
                            ctrim2d.Reverse();
                            indpoint2 = DStr.AddShape(V1);
                            isvt2 = true;
                            isvt1 = false;
                            indpoint1 = indpoint[ic, 1];
                        }
                    }
                    else
                    {
                        if (TopExp.LastVertex((TopoDS.Edge(Evive.Value(ic)))).IsSame(V1))
                        {
                            ctrim.Reverse();
                            ctrim2d.Reverse();
                            indpoint1 = DStr.AddShape(V1);
                            isvt1 = true;
                            isvt2 = false;
                            indpoint2 = indpoint[ic, 1];
                        }
                    }
                    ufirst = ctrim.FirstParameter();
                    ulast = ctrim.LastParameter();
                    GeomAdaptor_Surface? Asurf = null;
                    Asurf = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(Fvive.Value(ic, icplus))));
                    Geom2dAdaptor_Curve? Acurv = new Geom2dAdaptor_Curve(ctrim2d);
                    Adaptor3d_CurveOnSurface CurvOnS = new(Acurv, Asurf);
                    Adaptor3d_CurveOnSurface? HCons = (Adaptor3d_CurveOnSurface)CurvOnS.ShallowCopy(); // OCCT copy-constructs a handle
                    Order.SetValue(n3d, 0);
                    GeomPlate_CurveConstraint? Cont = 
                        new GeomPlate_CurveConstraint(HCons, Order.Value(n3d), 10, tolapp3d, angular, 0.1);
                    PSurf.Add(Cont);
                    TopOpeBRepDS_Curve tcurv3d = new(ctrim, 1.0e-4);
                    indcurve3d.SetValue(n3d, DStr.AddCurve(tcurv3d));
                    Interfp1 =
                        ChFi3d_FilPointInDS(TopAbs_FORWARD, indcurve3d.Value(n3d), indpoint1, ufirst, isvt1);
                    Interfp2 =
                        ChFi3d_FilPointInDS(TopAbs_REVERSED, indcurve3d.Value(n3d), indpoint2, ulast, isvt2);
                    DStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(Interfp1);
                    DStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(Interfp2);
                }
            }
        }


        PSurf.Perform();


        // call of approx

        if (PSurf.IsDone())
        {
            int nbcarreau = 9;
            int degmax = 8;
            double seuil = 0.0;
            GeomPlate_Surface? gpPlate = PSurf.Surface();

            NCollection_Sequence<gp_XY> S2d = new();
            NCollection_Sequence<gp_XYZ> S3d = new();
            S2d.Clear();
            S3d.Clear();
            PSurf.Disc2dContour(4, S2d);
            PSurf.Disc3dContour(4, 0, S3d);
            seuil = Max(tolapp, 10 * PSurf.G0Error());
            GeomPlate_PlateG0Criterion critere = new(S2d, S3d, seuil);
            GeomPlate_MakeApprox Mapp = new(gpPlate, critere, tolapp, nbcarreau, degmax);
            Geom_Surface? Surf = Mapp.Surface();
            double coef = 1.1, apperror;
            apperror = Mapp.CriterionError() * coef;


            //  Storage of the surface plate and corresponding curves in the DS

            TopAbs_Orientation orplate = default, orsurfdata = default, orpcurve = default, orien = default;
            //     gp_Vec n1,n2,du,dv,du1,dv1;
            //     gp_Pnt pp,pp1;
            //     double tpar;
            //     gp_Pnt2d uv;
            //     double scal;

            TopOpeBRepDS_Surface Tsurf = new(Surf, Mapp.ApproxError());
            int Isurf = DStr.AddSurface(Tsurf);
            // lbo : historique QDF.
            if (!myEVIMap.IsBound(V1))
            {
                NCollection_List<int> li = new();
                myEVIMap.Bind(V1, li);
            }
            myEVIMap.ChangeFind(V1).Append(Isurf);

            int SolInd = CD.Value(0).SolidIndex();
            NCollection_List<TopOpeBRepDS_Interference> SolidInterfs = 
                DStr.ChangeShapeInterferences(SolInd);

            // in case when one rereads at top, it is necessary that
            // alive edges that arrive at the top should be removed from the DS.
            // For this they are stored in the DS with their inverted orientation
            int nbedge = 0;
            TopExp_Explorer ex = new();
            if (deuxconges)
            {
                for (ic = 0; ic < nedge; ic++)
                {
                    if (!sharp.Value(ic))
                    {
                        nbedge = CD.Value(ic).Spine().NbEdges();
                        TopoDS_Edge Arcspine = new();
                        if (sens.Value(ic) == 1)
                        {
                            Arcspine = CD.Value(ic).Spine().Edges(1);
                        }
                        else
                        {
                            Arcspine = CD.Value(ic).Spine().Edges(nbedge);
                        }
                        int IArcspine = DStr.AddShape(Arcspine);
                        TopAbs_Orientation OVtx = TopAbs_FORWARD;
                        for (ex.Init(Arcspine.Oriented(TopAbs_FORWARD), TopAbs_VERTEX); ex.More(); ex.Next())
                        {
                            if (V1.IsSame(ex.Current()))
                            {
                                OVtx = ex.Current().Orientation();
                                break;
                            }
                        }
                        OVtx = TopAbs.Reverse(OVtx);
                        double parVtx = BRep_Tool.Parameter(V1, Arcspine);
                        TopOpeBRepDS_CurvePointInterference? interfv = 
                            ChFi3d_FilVertexInDS(OVtx, IArcspine, IVtx, parVtx);
                        DStr.ChangeShapeInterferences(IArcspine).Append(interfv);
                    }
                }
            }

            // calculate orientation of Plate orplate corresponding to surfdata
            // calculation corresponding to the first stripe
            Indices(nedge, 0, ref icplus, ref icmoins);
            isfirst = (sens.Value(0) == 1);
            ChFiDS_SurfData? Fd = 
                CD.Value(0).SetOfSurfData().Value(i.Value(0, icmoins));
            indice = Fd.Surf();
            //    Handle (Geom_Surface) surfdata  = DStr.Surface(indice).Surface();
            //     tpar= (CD.Value(0)->PCurve(isfirst)->FirstParameter()+
            // 	 CD.Value(0)->PCurve(isfirst)->LastParameter())/2 ;
            //     CD.Value(0)->PCurve(isfirst)->D0(tpar,uv);
            //     surfdata->D1(uv.X(),uv.Y(),pp,du,dv);
            //     tpar=(PSurf.Curves2d()->Value(1)->FirstParameter()+
            // 	  PSurf.Curves2d()->Value(1)->LastParameter())/2;
            //     (PSurf.Curves2d())->Value(1)->D0(tpar,uv);
            //     Surf-> D1(uv.X(),uv.Y(),pp1,du1,dv1);
            //     n1=du.Crossed(dv);
            //     n2=du1.Crossed(dv1);
            //     scal= n1.Dot(n2);
            orsurfdata = Fd.Orientation();
            //     if (scal>0) orplate=orsurfdata;
            //     else  orplate=TopAbs::Reverse(orsurfdata);
            orplate = PlateOrientation(Surf, PSurf.Curves2d(), SumFaceNormalAtV1);

            //  creation of solidinterderence for Plate
            TopOpeBRepDS_SolidSurfaceInterference? SSI = 
                new TopOpeBRepDS_SolidSurfaceInterference(new TopOpeBRepDS_Transition(orplate), TopOpeBRepDS_SOLID, SolInd,
                                                                                                    TopOpeBRepDS_SURFACE, Isurf);
            SolidInterfs.Append(SSI);

            // calculate orientation orien of pcurves of Plate
            // the curves from ic to icplus the pcurves of Plate
            // all have the same orientation
            int Ishape1 = 0, Ishape2 = 0;
            TopAbs_Orientation trafil1 = TopAbs_FORWARD, trafil2 = TopAbs_FORWARD;
            Ishape1 = Fd.IndexOfS1();
            Ishape2 = Fd.IndexOfS2();
            ChFiDS_FaceInterference Fi1 = Fd.InterferenceOnS1();
            ChFiDS_FaceInterference Fi2 = Fd.InterferenceOnS2();
            if (Ishape1 != 0)
            {
                if (Ishape1 > 0)
                {
                    trafil1 = DStr.Shape(Ishape1).Orientation();
                }
                trafil1 = TopAbs.Compose(trafil1, Fd.Orientation());
                trafil1 = TopAbs.Compose(TopAbs.Reverse(Fi1.Transition()), trafil1);
                trafil2 = TopAbs.Reverse(trafil1);
            }
            else
            {
                if (Ishape2 > 0)
                {
                    trafil2 = DStr.Shape(Ishape2).Orientation();
                }
                trafil2 = TopAbs.Compose(trafil2, Fd.Orientation());
                trafil2 = TopAbs.Compose(TopAbs.Reverse(Fi2.Transition()), trafil2);
                trafil1 = TopAbs.Reverse(trafil2);
            }
            if (isfirst)
            {
                orpcurve = TopAbs.Reverse(trafil1);
                orpcurve = TopAbs.Compose(orpcurve, CD.Value(0).FirstPCurveOrientation());
            }
            else
            {
                orpcurve = trafil1;
                orpcurve = TopAbs.Compose(orpcurve, CD.Value(0).LastPCurveOrientation());
            }
            if (orsurfdata == orplate)
            {
                orien = TopAbs.Reverse(orpcurve);
            }
            else
            {
                orien = orpcurve;
            }

            if (!droit)
            {
                for (ic = 0; ic <= nedge; ic++)
                {
                    if (libre.Value(ic))
                    {
                        int icplus21 = 0;
                        Indices(nedge, ic, ref icplus, ref icmoins);
                        Indices(nedge, icplus, ref icplus21, ref ic);
                        gp_Pnt2d UV1 = new(), UV2 = new();
                        Geom_Curve? C3d = null;
                        Geom2d_Curve? C2d = null; // OCCT's unused curv2d left out
                        gp_Pnt ptic = new(), pticplus = new();
                        BRepAdaptor_Curve BCurv1 = new(TopoDS.Edge(Evive.Value(ic)));
                        BRepAdaptor_Curve BCurv2 = new(TopoDS.Edge(Evive.Value(icplus)));
                        double par1 = p.Value(ic, icplus);
                        double par2 = p.Value(icplus, ic);
                        BCurv1.D0(par1, out ptic);
                        BCurv2.D0(par2, out pticplus);
                        ParametrePlate(n3d, PSurf, Surf, ptic, apperror, ref UV1);
                        ParametrePlate(n3d, PSurf, Surf, pticplus, apperror, ref UV2);
                        double to3d = 1.0e-3, to2d = 1.0e-6, tolreached;
                        ChFiDS_CommonPoint CP1 = new(), CP2 = new();
                        CP1.SetArc(1.0e-3, TopoDS.Edge(Evive.Value(ic)), par1, TopAbs_FORWARD);
                        CP1.SetPoint(ptic);
                        CP2.SetArc(1.0e-3, TopoDS.Edge(Evive.Value(icplus)), par2, TopAbs_FORWARD);
                        CP2.SetPoint(pticplus);
                        double param1 = 0.0, param2 = 0.0;
                        ChFi3d_ComputeArete(CP1, UV1, CP2, UV2, Surf, ref C3d, ref C2d, ref param1, ref param2, to3d, to2d,
                                                                out tolreached, 0);
                        TopOpeBRepDS_Curve tcurv3d = new(C3d, tolreached);
                        int ind1 = 0, ind2 = 0;
                        ind1 = indpoint[ic, 0];
                        ind2 = indpoint[icplus, 0];
                        int indcurv = DStr.AddCurve(tcurv3d);
                        Interfp1 = ChFi3d_FilPointInDS(TopAbs_FORWARD, indcurv, ind1, param1);
                        Interfp2 = ChFi3d_FilPointInDS(TopAbs_REVERSED, indcurv, ind2, param2);
                        DStr.ChangeCurveInterferences(indcurv).Append(Interfp1);
                        DStr.ChangeCurveInterferences(indcurv).Append(Interfp2);
                        Interfc = ChFi3d_FilCurveInDS(indcurv, Isurf, C2d, orien);
                        DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc);
                    }
                }
            }

            //  stockage des courbes relatives aux stripes
            n3d = 0;
            for (ic = 0; ic < nedge; ic++)
            {
                if (!sharp.Value(ic))
                {
                    n3d++;
                    Indices(nedge, ic, ref icplus, ref icmoins);

                    isfirst = (sens.Value(ic) == 1);
                    //   calculate curves interference relative to stripes

                    apperror = Mapp.CriterionError() * coef;
                    pardeb = CD.Value(ic).PCurve(isfirst).FirstParameter();
                    parfin = CD.Value(ic).PCurve(isfirst).LastParameter();

                    Interfp1 =
                        ChFi3d_FilPointInDS(TopAbs_FORWARD, indcurve3d.Value(n3d), indpoint.Value(ic, 0), pardeb);
                    Interfp2 = ChFi3d_FilPointInDS(TopAbs_REVERSED, indcurve3d.Value(n3d), indpoint.Value(ic, 1), parfin);
                    DStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(Interfp1);
                    DStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(Interfp2);
                    ref TopOpeBRepDS_Curve tcourb = ref DStr.ChangeCurve(indcurve3d.Value(n3d));

                    tcourb.Tolerance(errapp.Value(ic) + apperror);
                    ref TopOpeBRepDS_Point tpt1 = ref DStr.ChangePoint(indpoint[ic, 0]);
                    ref TopOpeBRepDS_Point tpt2 = ref DStr.ChangePoint(indpoint[ic, 1]);
                    tpt1.Tolerance(tpt1.Tolerance() + apperror);
                    tpt2.Tolerance(tpt2.Tolerance() + apperror);

                    // calculate surfaceinterference
                    Interfc =
                        ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), Isurf, PSurf.Curves2d().Value(n3d), orien);
                    DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc);
                    regular.SetCurve(indcurve3d.Value(n3d));
                    regular.SetS1(Isurf, false);
                    indice = CD.Value(ic).SetOfSurfData().Value(i.Value(ic, icmoins)).Surf();
                    regular.SetS2(indice, false);
                    myRegul.Append(regular);
                }
            }

            // storage of connection curves

            for (ic = 0; ic < nedge; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (!oksea.Value(ic))
                {
                    if (sharp.Value(ic) && !deuxconges)
                    {
                        // limitation of the alive edge
                        TopAbs_Orientation ori = default;
                        gp_Pnt Pf = new(), Pl = new(), sommet1 = new();
                        TopoDS_Vertex Vd = TopExp.FirstVertex(TopoDS.Edge(Evive.Value(ic)));
                        TopoDS_Vertex Vf = TopExp.LastVertex(TopoDS.Edge(Evive.Value(ic)));
                        Pf = BRep_Tool.Pnt(Vd);
                        Pl = BRep_Tool.Pnt(Vf);
                        sommet1 = BRep_Tool.Pnt(V1);
                        if (Pf.Distance(sommet1) < Pl.Distance(sommet1))
                        {
                            ori = TopAbs_FORWARD;
                        }
                        else
                        {
                            ori = TopAbs_REVERSED;
                        }
                        int Iarc1 = DStr.AddShape(TopoDS.Edge(Evive.Value(ic)));
                        Interfp1 = ChFi3d_FilPointInDS(ori, Iarc1, indpoint[ic, 1], p.Value(ic, icplus));
                        DStr.ChangeShapeInterferences(TopoDS.Edge(Evive.Value(ic))).Append(Interfp1);
                    }

                    if (!ponctuel.Value(ic) && !libre.Value(ic))
                    {
                        // actual connection
                        if (!moresurf.Value(ic))
                        {
                            n3d++;
                            ref TopOpeBRepDS_Curve tcourb1 = ref DStr.ChangeCurve(indcurve3d.Value(n3d));
                            tcourb1.Tolerance(tcourb1.Tolerance() + apperror);
                            if (!deuxconges)
                            {
                                ref TopOpeBRepDS_Point tpt11 = ref DStr.ChangePoint(indpoint[ic, 1]);
                                ref TopOpeBRepDS_Point tpt21 = ref DStr.ChangePoint(indpoint[icplus, 0]);
                                tpt11.Tolerance(tpt11.Tolerance() + apperror);
                                tpt21.Tolerance(tpt21.Tolerance() + apperror);
                            }
                            Interfc = ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), Isurf, PSurf.Curves2d().Value(n3d), orien);
                            DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc);
                            if (Order.Value(n3d) == 1)
                            {
                                regular.SetCurve(indcurve3d.Value(n3d));
                                regular.SetS1(Isurf, false);
                                regular.SetS2(numfa.Value(ic, icplus));
                                myRegul.Append(regular);
                            }
                        }
                    }
                }
            }

            // storage of curves projected on several faces
            for (ic = 0; ic < nedge; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (moresurf[ic])
                {
                    for (nb = 1; nb <= nbface; nb++)
                    {
                        if (!Eproj.Value(nb).IsNull())
                        {
                            n3d++;
                            ref TopOpeBRepDS_Curve tcourb1 = ref DStr.ChangeCurve(indcurve3d.Value(n3d));
                            tcourb1.Tolerance(tcourb1.Tolerance() + apperror);
                            if (Indice.Value(n3d) != 0)
                            {
                                ref TopOpeBRepDS_Point tpt11 = ref DStr.ChangePoint(Indice.Value(n3d));
                                tpt11.Tolerance(tpt11.Tolerance() + apperror);
                            }
                            Interfc = ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), Isurf, PSurf.Curves2d().Value(n3d), orien);
                            DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc);
                            if (Order.Value(n3d) == 1)
                            {
                                regular.SetCurve(indcurve3d.Value(n3d));
                                regular.SetS1(Isurf, false);
                                regular.SetS2(DStr.AddShape(TopoDS.Face(Fproj.Value(nb))));
                                myRegul.Append(regular);
                            }
                        }
                    }
                }
            }

            // storage of curves in case of tangent free borders
            if (droit)
            {
                for (ic = 0; ic < nedge; ic++)
                {
                    Indices(nedge, ic, ref icplus, ref icmoins);
                    TopoDS_Edge ecom = new();
                    ecom = TopoDS.Edge(Evive.Value(ic));
                    if (ecom.IsSame(edgelibre1) || ecom.IsSame(edgelibre2))
                    {
                        n3d++;
                        ref TopOpeBRepDS_Curve tcourb1 = ref DStr.ChangeCurve(indcurve3d.Value(n3d));
                        tcourb1.Tolerance(tcourb1.Tolerance() + apperror);
                        Interfc =
                            ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), Isurf, PSurf.Curves2d().Value(n3d), orien);
                        DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc);
                    }
                }
            }
        }
        else
        { // there is only one partial result
            done = false;
            hasresult = true;
            for (ic = 0; ic < nedge; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (!oksea.Value(ic))
                {
                    if (sharp.Value(ic) && !deuxconges)
                    {
                        // limitation of the alive edge
                        TopAbs_Orientation ori = default;
                        gp_Pnt Pf = new(), Pl = new(), sommet1 = new();
                        TopoDS_Vertex Vd = TopExp.FirstVertex(TopoDS.Edge(Evive.Value(ic)));
                        TopoDS_Vertex Vf = TopExp.LastVertex(TopoDS.Edge(Evive.Value(ic)));
                        Pf = BRep_Tool.Pnt(Vd);
                        Pl = BRep_Tool.Pnt(Vf);
                        sommet1 = BRep_Tool.Pnt(V1);
                        if (Pf.Distance(sommet1) < Pl.Distance(sommet1))
                        {
                            ori = TopAbs_FORWARD;
                        }
                        else
                        {
                            ori = TopAbs_REVERSED;
                        }
                        int Iarc1 = DStr.AddShape(TopoDS.Edge(Evive.Value(ic)));
                        Interfp1 = ChFi3d_FilPointInDS(ori, Iarc1, indpoint[ic, 1], p.Value(ic, icplus));
                        DStr.ChangeShapeInterferences(TopoDS.Edge(Evive.Value(ic))).Append(Interfp1);
                    }
                }
            }
        }
    }

}

internal static class ChFi3d_Builder_CnCrn
{
    internal static void Indices(int n, int ic, ref int icplus, ref int icmoins)
    {
        if (ic == (n - 1))
        {
            icplus = 0;
        }
        else
        {
            icplus = ic + 1;
        }
        if (ic == 0)
        {
            icmoins = n - 1;
        }
        else
        {
            icmoins = ic - 1;
        }
    }

    internal static void Calcul_Param(ChFiDS_Stripe? stripe, int jfposit, int indice, bool isfirst, ref double param)
    {
        if (jfposit == 2)
        {
            param = stripe.SetOfSurfData().Value(indice).InterferenceOnS2().Parameter(isfirst);
        }
        else
        {
            param = stripe.SetOfSurfData().Value(indice).InterferenceOnS1().Parameter(isfirst);
        }
    }

    internal static void Calcul_P2dOnSurf(ChFiDS_Stripe? stripe, int jfposit, int indice, double param, ref gp_Pnt2d p2)
    {
        if (jfposit == 1)
        {
            stripe.SetOfSurfData().Value(indice).InterferenceOnS1().PCurveOnSurf().D0(param, out p2);
        }
        else
        {
            stripe.SetOfSurfData().Value(indice).InterferenceOnS2().PCurveOnSurf().D0(param, out p2);
        }
    }

    internal static void Calcul_C2dOnFace(ChFiDS_Stripe? stripe, int jfposit, int indice, ref Geom2d_Curve? c2d)
    {
        if (jfposit == 1)
        {
            c2d = stripe.SetOfSurfData().Value(indice).InterferenceOnS1().PCurveOnFace();
        }
        else
        {
            c2d = stripe.SetOfSurfData().Value(indice).InterferenceOnS2().PCurveOnFace();
        }
    }

    internal static void Calcul_Orientation(ChFiDS_Stripe? stripe, int jfposit, int indice, ref TopAbs_Orientation orient)
    {
        if (jfposit == 1)
        {
            orient = stripe.SetOfSurfData().Value(indice).InterferenceOnS1().Transition();
        }
        else
        {
            orient = stripe.SetOfSurfData().Value(indice).InterferenceOnS2().Transition();
        }
    }

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

    internal static void cherche_edge1(in TopoDS_Face F1, in TopoDS_Face F2, ref TopoDS_Edge Edge)
    {
        int i = 0, j = 0;
        TopoDS_Edge Ecur1 = new(), Ecur2 = new();
        bool trouve = false;
        NCollection_IndexedMap<TopoDS_Shape> MapE1 = new(TopTools_ShapeMapHasher.Instance),
                                             MapE2 = new(TopTools_ShapeMapHasher.Instance);

        TopExp.MapShapes(F1, TopAbs_EDGE, MapE1);
        TopExp.MapShapes(F2, TopAbs_EDGE, MapE2);
        for (i = 1; i <= MapE1.Extent() && !trouve; i++)
        {
            TopoDS_Shape aLocalShape = MapE1[i];
            Ecur1 = TopoDS.Edge(aLocalShape);
            //  Ecur1=TopoDS::Edge(TopoDS_Shape (MapE1[i]));
            for (j = 1; j <= MapE2.Extent() && !trouve; j++)
            {
                aLocalShape = MapE2[j];
                Ecur2 = TopoDS.Edge(aLocalShape);
                //              Ecur2=TopoDS::Edge(TopoDS_Shape (MapE2[j]));
                if (Ecur2.IsSame(Ecur1))
                {
                    Edge = Ecur1;
                    trouve = true;
                }
            }
        }
        if (Edge.IsNull())
        {
            throw new Standard_ConstructionError("Failed to find edge");
        }
    }

    internal static void CurveHermite(TopOpeBRepDS_DataStructure DStr, ChFiDS_Stripe? CDicmoins, int jficmoins, int icmoins, double picmoins, int sensicmoins, bool sharpicmoins, in TopoDS_Edge Eviveicmoins, ChFiDS_Stripe? CDicplus, int jficplus, int icplus, double picplus, int sensicplus, bool sharpicplus, in TopoDS_Edge Eviveicplus, int nbface, NCollection_Sequence<TopoDS_Shape> Ecom, NCollection_Sequence<TopoDS_Shape> Face, NCollection_Sequence<Geom2d_Curve> proj2d, NCollection_Sequence<Geom_Curve> cproj, NCollection_Sequence<TopoDS_Shape> Eproj, NCollection_Sequence<double> param, ref double error)
    {
        gp_Pnt p01 = new(), p02 = new();
        gp_Vec d11 = new(), d12 = new();
        int ii = 0, jj = 0;
        double up1 = 0.0, up2 = 0.0;
        int ilin = 0, jfp = 0;
        Geom_Curve? c1 = null, c2 = null;
        if (sharpicmoins)
        {
            c1 = BRep_Tool.Curve(Eviveicmoins, out up1, out up2);
        }
        else
        {
            if (jficmoins == 1)
            {
                ilin = CDicmoins.SetOfSurfData().Value(icmoins).InterferenceOnS1().LineIndex();
            }
            else
            {
                ilin = CDicmoins.SetOfSurfData().Value(icmoins).InterferenceOnS2().LineIndex();
            }
            c1 = DStr.Curve(ilin).Curve();
        }
        if (sharpicplus)
        {
            c2 = BRep_Tool.Curve(Eviveicplus, out up1, out up2);
        }
        else
        {
            jfp = 3 - jficplus;
            if (jfp == 1)
            {
                ilin = CDicplus.SetOfSurfData().Value(icplus).InterferenceOnS1().LineIndex();
            }
            else
            {
                ilin = CDicplus.SetOfSurfData().Value(icplus).InterferenceOnS2().LineIndex();
            }
            c2 = DStr.Curve(ilin).Curve();
        }
        if (c1 is null)
        {
            throw new Standard_ConstructionError("Failed to get 3D curve of edge");
        }
        if (c2 is null)
        {
            throw new Standard_ConstructionError("Failed to get 3D curve of edge");
        }
        c1.D1(picmoins, out p01, out d11);
        c2.D1(picplus, out p02, out d12);
        int size = 4;
        math_Matrix MatCoefs = new(1, size, 1, size);
        NCollection_Array1<gp_XYZ> Cont = new(1, size);
        PLib.HermiteCoefficients(0, 1, 1, 1, MatCoefs);
        double L1 = p01.Distance(p02);
        double lambda = ((double)1) / Max(d11.Magnitude() / L1, 1.0e-6);
        Cont[1] = p01.XYZ();
        if (sensicmoins == 1)
        {
            Cont[2] = d11.XYZ() * (-lambda);
        }
        else
        {
            Cont[2] = d11.XYZ() * (lambda);
        }
        lambda = ((double)1) / Max(d12.Magnitude() / L1, 1.0e-6);
        Cont[3] = p02.XYZ();
        if (sensicplus == 1)
        {
            Cont[4] = d12.XYZ() * (lambda);
        }
        else
        {
            Cont[4] = d12.XYZ() * (-lambda);
        }
        NCollection_Array1<gp_Pnt> ExtrapPoles = new(1, size);
        NCollection_Array1<gp_Pnt> ExtraCoeffs = new(1, size);
        gp_Pnt p0 = new(0, 0, 0);
        ExtraCoeffs.Init(p0);
        for (ii = 1; ii <= size; ii++)
        {
            for (jj = 1; jj <= size; jj++)
            {
                ExtraCoeffs[jj].ChangeCoord() += MatCoefs[ii, jj] * Cont[ii];
            }
        }
        PLib.CoefficientsPoles(ExtraCoeffs, PLib.NoWeights(), ExtrapPoles, PLib.NoWeights());
        Geom_BezierCurve? Bezier = new Geom_BezierCurve(ExtrapPoles);
        BRepLib_MakeEdge Bedge = new(Bezier);
        TopoDS_Edge edg = Bedge.Edge();
        TopoDS_Face F = new();
        error = 1.0e-30;
        int nb = 0;
        for (nb = 1; nb <= nbface; nb++)
        {
            F = TopoDS.Face(Face.Value(nb));
            NCollection_IndexedMap<TopoDS_Shape> MapE1 = new(TopTools_ShapeMapHasher.Instance);
            TopoDS_Edge E1 = new();
            Geom2d_Curve? proj1 = null;
            Geom_Curve? proj1c = null; // OCCT's unused proj2c left out
            BRepAlgo_NormalProjection OrtProj = new();
            OrtProj.Init(F);
            OrtProj.Add(edg);
            OrtProj.SetParams(1.0e-4, 1.0e-4, GeomAbs_C1, 14, 16);
            OrtProj.Build();
            if (OrtProj.IsDone())
            {
                TopExp.MapShapes(OrtProj.Projection(), TopAbs_EDGE, MapE1);
                if (MapE1.Extent() != 0)
                {
                    if (MapE1.Extent() != 1)
                    {
                        BRepLib_MakeFace Bface = new(BRep_Tool.Surface(F), Precision.Confusion());
                        F = Bface.Face();
                        OrtProj.Init(F);
                        OrtProj.Build();
                        MapE1.Clear();
                        if (OrtProj.IsDone())
                        {
                            TopExp.MapShapes(OrtProj.Projection(), TopAbs_EDGE, MapE1);
                        }
                    }
                    if (MapE1.Extent() != 0)
                    {
                        bool trouve = false;
                        for (int ind = 1; ind <= MapE1.Extent() && !trouve; ind++)
                        {
                            TopoDS_Shape aLocalShape = MapE1[ind];
                            E1 = TopoDS.Edge(aLocalShape);
                            //           E1=TopoDS::Edge( TopoDS_Shape (MapE1[ind]));
                            if (!BRep_Tool.Degenerated(E1))
                            {
                                trouve = true;
                            }
                        }
                        Eproj.Append(E1);
                        proj1 = BRep_Tool.CurveOnSurface(E1, F, out up1, out up2);
                        if (proj1 is null)
                        {
                            throw new Standard_ConstructionError("Failed to get p-curve of edge");
                        }
                        proj2d.Append(new Geom2d_TrimmedCurve(proj1, up1, up2));
                        proj1c = BRep_Tool.Curve(E1, out up1, out up2);
                        if (proj1c is null)
                        {
                            throw new Standard_ConstructionError("Failed to get 3D curve of edge");
                        }
                        cproj.Append(new Geom_TrimmedCurve(proj1c, up1, up2));
                        if (error > BRep_Tool.Tolerance(E1))
                        {
                            error = BRep_Tool.Tolerance(E1);
                        }
                    }
                    else
                    {
                        Eproj.Append(E1);
                        proj2d.Append(proj1);
                        cproj.Append(proj1c);
                    }
                }
                else
                {
                    Eproj.Append(E1);
                    proj2d.Append(proj1);
                    cproj.Append(proj1c);
                }
            }
        }
        for (nb = 1; nb <= nbface - 1; nb++)
        {
            BRepAdaptor_Curve C = new(TopoDS.Edge(Ecom.Value(nb)));
            C.D0(param.Value(nb), out p02);
            GeomAdaptor_Curve L = new(Bezier);
            Extrema_ExtCC ext = new(C, L);
            if (ext.IsDone())
            {
                if (!ext.IsParallel() && ext.NbExt() != 0)
                {
                    Extrema_POnCurv POnC = new(), POnL = new();
                    ext.Points(1, out POnC, out POnL);
                    if (POnC.Value().Distance(POnL.Value()) < Precision.Confusion())
                    {
                        param.ChangeValue(nb) = POnC.Parameter();
                    }
                    else
                    {
                        if (cproj.Value(nb) is not null)
                        {
                            cproj.Value(nb).D0(cproj.Value(nb).LastParameter(), out p01);
                        }
                        else if (cproj.Value(nb + 1) is not null)
                        {
                            cproj.Value(nb + 1).D0(cproj.Value(nb + 1).FirstParameter(), out p01);
                        }
                    }
                }
            }
            if (!ext.IsDone() || ext.NbExt() == 0)
            {
                if (cproj.Value(nb) is not null)
                {
                    cproj.Value(nb).D0(cproj.Value(nb).LastParameter(), out p01);
                }
                else if (cproj.Value(nb + 1) is not null)
                {
                    cproj.Value(nb + 1).D0(cproj.Value(nb + 1).FirstParameter(), out p01);
                }
                if (p01.Distance(p02) > 1.0e-4)
                {
                    Extrema_ExtPC ext1 = new(p01, C);
                    if (ext1.IsDone())
                    {
                        if (ext1.NbExt() != 0)
                        {
                            Extrema_POnCurv POnC = ext1.Point(1);
                            param.ChangeValue(nb) = POnC.Parameter();
                        }
                    }
                }
            }
        }
    }

    internal static void CalculDroite(in gp_Pnt2d p2d1, double xdir, double ydir, ref Geom2d_Curve? pcurve)
    {
        gp_Dir2d dir1 = new(xdir, ydir);
        Geom2d_Line? l = new Geom2d_Line(p2d1, dir1);
        double l0 = Math.Sqrt(xdir * xdir + ydir * ydir);
        pcurve = new Geom2d_TrimmedCurve(l, 0, l0);
    }

    internal static void CalculBatten(GeomAdaptor_Surface? ASurf, in TopoDS_Face Face, double xdir, double ydir, in gp_Pnt2d p2d1, in gp_Pnt2d p2d2, bool contraint1, bool contraint2, Geom2d_Curve? curv2d1, Geom2d_Curve? curv2d2, double picicplus, double picplusic, bool inverseic, bool inverseicplus, ref Geom2d_Curve? pcurve)
    {
        bool isplane = false;
        bool anglebig = false;
        isplane = ASurf.GetType() == GeomAbs_Plane;
        gp_Dir2d dir1 = new(xdir, ydir);
        GeomLProp_CLProps2d CL1 = new(curv2d1, picicplus, 1, 1.0e-4);
        GeomLProp_CLProps2d CL2 = new(curv2d2, picplusic, 1, 1.0e-4);
        gp_Dir2d dir3 = new(), dir4 = new();
        CL1.Tangent(out dir3);
        CL2.Tangent(out dir4);
        if (inverseic)
        {
            dir3.Reverse();
        }
        if (inverseicplus)
        {
            dir4.Reverse();
        }
        double h = p2d2.Distance(p2d1) / 20;
        FairCurve_Batten Bat = new(p2d1, p2d2, h);
        Bat.SetFreeSliding(true);
        double ang1 = 0.0, ang2 = 0.0;
        ang1 = dir1.Angle(dir3);
        if (dir1.Angle(dir4) > 0)
        {
            ang2 = M_PI - dir1.Angle(dir4);
        }
        else
        {
            ang2 = -M_PI - dir1.Angle(dir4);
        }
        if (contraint1 && contraint2)
        {
            anglebig = (Math.Abs(ang1) > 1.2) || (Math.Abs(ang2) > 1.2);
        }
        else if (contraint1)
        {
            anglebig = Math.Abs(ang1) > 1.2;
        }
        else if (contraint2)
        {
            anglebig = Math.Abs(ang2) > 1.2;
        }
        if (isplane && (Math.Abs(ang1) > M_PI / 2 || Math.Abs(ang2) > M_PI / 2))
        {
            isplane = false;
        }
        if (anglebig && !isplane)
        {
            CalculDroite(p2d1, xdir, ydir, ref pcurve);
        }
        else
        {
            if (contraint1)
            {
                Bat.SetAngle1(ang1);
            }
            else
            {
                Bat.SetConstraintOrder1(0);
            }
            if (contraint2)
            {
                Bat.SetAngle2(ang2);
            }
            else
            {
                Bat.SetConstraintOrder2(0);
            }
            FairCurve_AnalysisCode Iana = default;
            bool Ok = false;
            Ok = Bat.Compute(out Iana, 25, 1.0e-2);
            if (Ok)
            {
                pcurve = Bat.Curve();
                double umin = 0.0, vmin = 0.0, umax = 0.0, vmax = 0.0;
                BRepTools.UVBounds(Face, out umin, out umax, out vmin, out vmax);
                Bnd_Box2d bf = new(), bc = new();
                Geom2dAdaptor_Curve acur = new(pcurve);
                BndLib_Add2dCurve.Add(acur, 0, ref bc);
                bf.Update(umin, vmin, umax, vmax);
                double uminc = 0.0, vminc = 0.0, umaxc = 0.0, vmaxc = 0.0;
                bc.Get(out uminc, out vminc, out umaxc, out vmaxc);
                if (uminc < umin - 1.0e-7)
                {
                    Ok = false;
                }
                if (umaxc > umax + 1.0e-7)
                {
                    Ok = false;
                }
                if (vminc < vmin - 1.0e-7)
                {
                    Ok = false;
                }
                if (vmaxc > vmax + 1.0e-7)
                {
                    Ok = false;
                }
            }
            if (!Ok)
            {
                CalculDroite(p2d1, xdir, ydir, ref pcurve);
            }
        }
    }

    internal static void OrientationIcNonVive(ChFiDS_Stripe? CDic, int jfic, int icicplus, int sensic, ref TopAbs_Orientation orien)
    {
        TopAbs_Orientation orinterf = default;
        Calcul_Orientation(CDic, jfic, icicplus, ref orinterf);
        if (sensic != 1)
        {
            if (orinterf == TopAbs_FORWARD)
            {
                orien = TopAbs_FORWARD;
            }
            else
            {
                orien = TopAbs_REVERSED;
            }
        }
        else
        {
            if (orinterf == TopAbs_FORWARD)
            {
                orien = TopAbs_REVERSED;
            }
            else
            {
                orien = TopAbs_FORWARD;
            }
        }
    }

    internal static void OrientationIcplusNonVive(ChFiDS_Stripe? CDicplus, int jficplus, int icplusic, int sensicplus, ref TopAbs_Orientation orien)
    {
        TopAbs_Orientation orinterf = default;
        int jfp = 3 - jficplus;
        Calcul_Orientation(CDicplus, jfp, icplusic, ref orinterf);
        if (sensicplus == 1)
        {
            if (orinterf == TopAbs_FORWARD)
            {
                orien = TopAbs_FORWARD;
            }
            else
            {
                orien = TopAbs_REVERSED;
            }
        }
        else
        {
            if (orinterf == TopAbs_FORWARD)
            {
                orien = TopAbs_REVERSED;
            }
            else
            {
                orien = TopAbs_FORWARD;
            }
        }
    }

    internal static void OrientationAreteViveConsecutive(in TopoDS_Shape Fviveicicplus, in TopoDS_Shape Eviveic, in TopoDS_Vertex V1, ref TopAbs_Orientation orien)
    { // orinterf is orientation of edge ic corresponding to face Fviveicicplus taken FORWARD
        TopAbs_Orientation orinterf = TopAbs_FORWARD;
        TopoDS_Face F = TopoDS.Face(Fviveicicplus);
        TopoDS_Edge E = TopoDS.Edge(Eviveic);
        TopExp_Explorer ex = new();
        for (ex.Init(F.Oriented(TopAbs_FORWARD), TopAbs_EDGE); ex.More(); ex.Next())
        {
            if (E.IsSame(ex.Current()))
            {
                orinterf = ex.Current().Orientation();
                break;
            }
        }
        // if V1 is vertex REVERSED of edge ic the curve
        // has the same orientation as ic
        TopoDS_Vertex vl = new();
        vl = TopExp.LastVertex(E);
        if (vl.IsSame(V1))
        {
            if (orinterf == TopAbs_FORWARD)
            {
                orien = TopAbs_FORWARD;
            }
            else
            {
                orien = TopAbs_REVERSED;
            }
        }
        else
        {
            if (orinterf == TopAbs_FORWARD)
            {
                orien = TopAbs_REVERSED;
            }
            else
            {
                orien = TopAbs_FORWARD;
            }
        }
    }

    internal static void PerformTwoCornerSameExt(TopOpeBRepDS_DataStructure DStr, ChFiDS_Stripe? stripe1, int index1, int sens1, ChFiDS_Stripe? stripe2, int index2, int sens2, ref bool trouve)
    {
        TopOpeBRepDS_CurvePointInterference? Interfp1 = null, Interfp2 = null;
        TopOpeBRepDS_SurfaceCurveInterference? Interfc = null;
        TopAbs_Orientation orpcurve = default, trafil1 = default, orsurf1 = default, orsurf2 = default;
        bool isfirst = false;
        int indic1 = 0, indic2 = 0, indpoint1 = 0, indpoint2 = 0, ind = 0, indcurve = 0;
        double tol = 0.0;
        gp_Pnt P1 = new(), P2 = new(), P3 = new(), P4 = new();
        gp_Pnt2d p2d = new();
        Geom_Curve? cint = null;
        Geom2d_Curve? C2dint1 = null, C2dint2 = null;
        isfirst = sens1 == 1;
        ref ChFiDS_CommonPoint Com11 = ref stripe1.SetOfSurfData().Value(index1).ChangeVertex(isfirst, 1);
        ref ChFiDS_CommonPoint Com12 = ref stripe1.SetOfSurfData().Value(index1).ChangeVertex(isfirst, 2);
        isfirst = sens2 == 1;
        ref ChFiDS_CommonPoint Com21 = ref stripe2.SetOfSurfData().Value(index2).ChangeVertex(isfirst, 1);
        indic1 = stripe1.SetOfSurfData().Value(index1).Surf();
        indic2 = stripe2.SetOfSurfData().Value(index2).Surf();
        ChFiDS_SurfData? Fd1 = stripe1.SetOfSurfData().Value(index1);
        ChFiDS_SurfData? Fd2 = stripe2.SetOfSurfData().Value(index2);

        NCollection_Array1<double> Pardeb = new(1, 4), Parfin = new(1, 4);
        ChFiDS_FaceInterference Fi11 = Fd1.InterferenceOnS1();
        ChFiDS_FaceInterference Fi12 = Fd1.InterferenceOnS2();
        ChFiDS_FaceInterference Fi21 = Fd2.InterferenceOnS1();
        ChFiDS_FaceInterference Fi22 = Fd2.InterferenceOnS2();
        gp_Pnt2d pfi11 = new(), pfi12 = new(), pfi21 = new(), pfi22 = new();
        isfirst = sens1 == 1;
        pfi11 = Fi11.PCurveOnSurf().Value(Fi11.Parameter(isfirst));
        pfi12 = Fi12.PCurveOnSurf().Value(Fi12.Parameter(isfirst));
        isfirst = sens2 == 1;
        if (Com11.Point().Distance(Com21.Point()) < 1.0e-4)
        {
            pfi21 = Fi21.PCurveOnSurf().Value(Fi21.Parameter(isfirst));
            pfi22 = Fi22.PCurveOnSurf().Value(Fi22.Parameter(isfirst));
        }
        else
        {
            pfi22 = Fi21.PCurveOnSurf().Value(Fi21.Parameter(isfirst));
            pfi21 = Fi22.PCurveOnSurf().Value(Fi22.Parameter(isfirst));
        }

        Pardeb[1] = pfi11.X();
        Pardeb[2] = pfi11.Y();
        Pardeb[3] = pfi21.X();
        Pardeb[4] = pfi21.Y();
        Parfin[1] = pfi12.X();
        Parfin[2] = pfi12.Y();
        Parfin[3] = pfi22.X();
        Parfin[4] = pfi22.Y();

        GeomAdaptor_Surface? HS1 = ChFi3d_BoundSurf(DStr, Fd1, 1, 2);
        GeomAdaptor_Surface? HS2 = ChFi3d_BoundSurf(DStr, Fd2, 1, 2);
        trouve = false;
        if (ChFi3d_ComputeCurves(HS1, HS2, Pardeb, Parfin, ref cint, ref C2dint1, ref C2dint2, 1.0e-4, 1.0e-5, out tol))
        {
            cint.D0(cint.FirstParameter(), out P1);
            cint.D0(cint.LastParameter(), out P2);
            trouve = ((Com11.Point().Distance(P1) < 1.0e-4 || Com11.Point().Distance(P2) < 1.0e-4)
                                && (Com12.Point().Distance(P1) < 1.0e-4 || Com12.Point().Distance(P2) < 1.0e-4));
        }

        if (trouve)
        {
            isfirst = sens1 == 1;
            stripe1.InDS(isfirst);
            indpoint1 = ChFi3d_IndexPointInDS(Com11, DStr);
            indpoint2 = ChFi3d_IndexPointInDS(Com12, DStr);
            stripe1.SetIndexPoint(indpoint1, isfirst, 1);
            stripe1.SetIndexPoint(indpoint2, isfirst, 2);
            isfirst = sens2 == 1;
            stripe2.InDS(isfirst);
            if (Com11.Point().Distance(Com21.Point()) < 1.0e-4)
            {
                stripe2.SetIndexPoint(indpoint1, isfirst, 1);
                stripe2.SetIndexPoint(indpoint2, isfirst, 2);
            }
            else
            {
                stripe2.SetIndexPoint(indpoint2, isfirst, 1);
                stripe2.SetIndexPoint(indpoint1, isfirst, 2);
            }

            orsurf1 = Fd1.Orientation();
            trafil1 = DStr.Shape(Fd1.IndexOfS1()).Orientation();
            trafil1 = TopAbs.Compose(trafil1, Fd1.Orientation());
            trafil1 = TopAbs.Compose(TopAbs.Reverse(Fi11.Transition()), trafil1);
            orsurf2 = Fd2.Orientation();
            TopOpeBRepDS_Curve tcurv3d = new(cint, tol);
            indcurve = DStr.AddCurve(tcurv3d);
            cint.D0(cint.FirstParameter(), out P1);
            cint.D0(cint.LastParameter(), out P2);
            Fi11.PCurveOnFace().D0(Fi11.LastParameter(), out p2d);
            Geom_Surface? Stemp = 
                BRep_Tool.Surface(TopoDS.Face(DStr.Shape(Fd1.IndexOfS1())));
            Stemp.D0(p2d.X(), p2d.Y(), out P4);
            Fi11.PCurveOnFace().D0(Fi11.FirstParameter(), out p2d);
            Stemp.D0(p2d.X(), p2d.Y(), out P3);
            if (P1.Distance(P4) < 1.0e-4 || P2.Distance(P3) < 1.0e-4)
            {
                orpcurve = trafil1;
            }
            else
            {
                orpcurve = TopAbs.Reverse(trafil1);
            }
            if (Com11.Point().Distance(P1) > 1.0e-4)
            {
                ind = indpoint1;
                indpoint1 = indpoint2;
                indpoint2 = ind;
            }
            Interfp1 = ChFi3d_FilPointInDS(TopAbs_FORWARD, indcurve, indpoint1, cint.FirstParameter());
            Interfp2 = ChFi3d_FilPointInDS(TopAbs_REVERSED, indcurve, indpoint2, cint.LastParameter());
            DStr.ChangeCurveInterferences(indcurve).Append(Interfp1);
            DStr.ChangeCurveInterferences(indcurve).Append(Interfp2);
            Interfc = ChFi3d_FilCurveInDS(indcurve, indic1, C2dint1, orpcurve);
            DStr.ChangeSurfaceInterferences(indic1).Append(Interfc);
            if (orsurf1 == orsurf2)
            {
                orpcurve = TopAbs.Reverse(orpcurve);
            }
            Interfc = ChFi3d_FilCurveInDS(indcurve, indic2, C2dint2, orpcurve);
            DStr.ChangeSurfaceInterferences(indic2).Append(Interfc);
        }
    }

    internal static void CpOnEdge(ChFiDS_Stripe? stripe, int num, bool isfirst, in TopoDS_Edge Eadj1, in TopoDS_Edge Eadj2, ref bool compoint)
    {
        ChFiDS_CommonPoint cp1 = new(), cp2 = new();
        compoint = false;
        cp1 = stripe.SetOfSurfData().Value(num).ChangeVertex(isfirst, 1);
        cp2 = stripe.SetOfSurfData().Value(num).ChangeVertex(isfirst, 2);
        if (cp1.IsOnArc())
        {
            if (cp1.Arc().IsSame(Eadj1) || cp1.Arc().IsSame(Eadj2))
            {
                compoint = true;
            }
        }
        if (cp2.IsOnArc())
        {
            if (cp2.Arc().IsSame(Eadj1) || cp2.Arc().IsSame(Eadj2))
            {
                compoint = true;
            }
        }
    }

    internal static void RemoveSurfData(ChFiDS_StripeMap myVDataMap, ChFiDS_Map myEFMap, in TopoDS_Edge edgecouture, in TopoDS_Face facecouture, in TopoDS_Vertex V1)
    {
        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        bool isfirst = false;
        TopoDS_Edge Ecur = new(), Eadj1 = new(), Eadj2 = new();
        TopoDS_Face Fg = new(), Fd = new(), F1 = new(), F2 = new();
        TopoDS_Vertex Vbid = new();
        int nbsurf = 0, nbedge = 0, sense = 0, num = 0;
        for (It.Initialize(myVDataMap[V1]); It.More(); It.Next())
        {
            nbsurf = It.Value().SetOfSurfData().Length();
            nbedge = It.Value().Spine().NbEdges();
            if (nbsurf != 1)
            {
                num = ChFi3d_IndexOfSurfData(V1, It.Value(), out sense);
                if (sense == 1)
                {
                    Ecur = It.Value().Spine().Edges(1);
                }
                else
                {
                    Ecur = It.Value().Spine().Edges(nbedge);
                }
                ChFi3d_edge_common_faces(myEFMap[Ecur], out F1, out F2);
                if (F1.IsSame(facecouture))
                {
                    Eadj1 = edgecouture;
                }
                else
                {
                    ChFi3d_cherche_element(V1, Ecur, F1, ref Eadj1, ref Vbid);
                }
                ChFi3d_edge_common_faces(myEFMap[Eadj1], out Fg, out Fd);
                if (F2.IsSame(facecouture))
                {
                    Eadj2 = edgecouture;
                }
                else
                {
                    ChFi3d_cherche_element(V1, Ecur, F2, ref Eadj2, ref Vbid);
                }
                ChFi3d_edge_common_faces(myEFMap[Eadj2], out Fg, out Fd);
                bool compoint = false;
                isfirst = (sense == 1);
                int ind = 0;
                if (sense == 1)
                {
                    ind = 0;
                    // among surfdatas find the greatest indice ind so that
                    // surfdata could have one of commonpoint on Eadj1 and Eadj2
                    // remove surfdata from 1 to ind-1
                    for (int i = 1; i <= nbsurf; i++)
                    {
                        CpOnEdge(It.Value(), i, isfirst, Eadj1, Eadj2, ref compoint);
                        if (compoint)
                        {
                            ind = i;
                        }
                    }
                    if (ind >= 2)
                    {
                        RemoveSD(It.ChangeValue(), 1, ind - 1);
                    }
                }
                else
                {
                    ind = num;
                    // among surfdatas find the smallest indice ind so that
                    // surfdata could have one of commonpoint on Eadj1 and Eadj2
                    // remove surfdata from ind+1 to num
                    for (int i = num; i >= 1; i--)
                    {
                        CpOnEdge(It.Value(), i, isfirst, Eadj1, Eadj2, ref compoint);
                        if (compoint)
                        {
                            ind = i;
                        }
                    }
                    if (ind < num)
                    {
                        RemoveSD(It.ChangeValue(), ind + 1, num);
                    }
                }
            }
        }
    }

    internal static void ParametrePlate(int n3d, GeomPlate_BuildPlateSurface PSurf, Geom_Surface? Surf, in gp_Pnt point, double apperror, ref gp_Pnt2d uv)
    {
        int ip = 0;
        gp_Pnt P1 = new();
        double par = 0.0;
        bool trouve = false;
        for (ip = 1; ip <= n3d && !trouve; ip++)
        {
            par = PSurf.Curves2d().Value(ip).FirstParameter();
            PSurf.Curves2d().Value(ip).D0(par, out uv);
            Surf.D0(uv.X(), uv.Y(), out P1);
            trouve = P1.IsEqual(point, apperror);
            if (!trouve)
            {
                par = PSurf.Curves2d().Value(ip).LastParameter();
                PSurf.Curves2d().Value(ip).D0(par, out uv);
                Surf.D0(uv.X(), uv.Y(), out P1);
                trouve = P1.IsEqual(point, apperror);
            }
        }
    }

    internal static void SummarizeNormal(in TopoDS_Vertex V1, in TopoDS_Face Fcur, in TopoDS_Edge Ecur, ref gp_Vec SumFaceNormalAtV1)
    {
        gp_Pnt2d uv1 = new(), uv2 = new();
        BRep_Tool.UVPoints(Ecur, Fcur, out uv1, out uv2);
        if (!V1.IsSame(TopExp.FirstVertex(Ecur)))
        {
            uv1 = uv2;
        }

        gp_Pnt P = new();
        gp_Vec d1U = new(), d1V = new();
        BRep_Tool.Surface(Fcur).D1(uv1.X(), uv1.Y(), out P, out d1U, out d1V);
        gp_Vec N = d1U.Crossed(d1V);
        if (Fcur.Orientation() == TopAbs_REVERSED)
        {
            N.Reverse();
        }

        if (N.SquareMagnitude() <= Precision.PConfusion())
        {
            return;
        }

        SumFaceNormalAtV1 += N.Normalized();
        SumFaceNormalAtV1.Normalize();
    }

    internal enum ChFi3d_SurfType
    {
        ChFiSURFACE,
        FACE1,
        FACE2
    } // for call SurfIndex(...)

    internal static int SurfIndex(NCollection_Array1<ChFiDS_Stripe> StripeArray1, int StripeIndex, int SurfDataIndex, ChFi3d_SurfType SurfType)
    {
        ChFiDS_SurfData? aSurfData = 
            StripeArray1[StripeIndex].SetOfSurfData().Value(SurfDataIndex);
        switch (SurfType)
        {
            case ChFiSURFACE:
                return aSurfData.Surf();
            case FACE1:
                return aSurfData.IndexOfS1();
            case FACE2:
                return aSurfData.IndexOfS2();
            default:
                return -1;
        }
    }

    internal static TopAbs_Orientation PlateOrientation(Geom_Surface? thePlateSurf, NCollection_HArray1<Geom2d_Curve>? thePCArr, in gp_Vec theRefDir)
    {
        gp_Vec du = new(), dv = new();
        gp_Pnt pp1 = new(), pp2 = new(), pp3 = new();
        gp_Pnt2d uv = new();
        double fpar = 0.0, lpar = 0.0;
        double SumScal1 = 0, SumScal2 = 0;

        int i = 0, nb = thePCArr.Upper();
        Geom2d_Curve? aPC = thePCArr.Value(nb);
        fpar = aPC.FirstParameter();
        lpar = aPC.LastParameter();
        aPC.D0((fpar + lpar) / 2.0, out uv);
        thePlateSurf.D0(uv.X(), uv.Y(), out pp1);
        aPC.D0(lpar, out uv);
        thePlateSurf.D0(uv.X(), uv.Y(), out pp2);

        for (i = 1; i <= nb; i++)
        {
            aPC = thePCArr.Value(i);
            fpar = aPC.FirstParameter();
            lpar = aPC.LastParameter();
            aPC.D0(fpar, out uv);
            thePlateSurf.D1(uv.X(), uv.Y(), out pp2, out du, out dv);
            gp_Vec n1 = du ^ dv;
            n1.Normalize();

            aPC.D0((fpar + lpar) / 2.0, out uv);
            thePlateSurf.D0(uv.X(), uv.Y(), out pp3);

            gp_Vec vv1 = new(pp2, pp1), vv2 = new(pp2, pp3);

            gp_Vec n2 = vv2 ^ vv1;
            n2.Normalize();

            SumScal1 += n1 * n2;
            SumScal2 += n2 * theRefDir;

            pp1 = pp3;
        }
        if (SumScal2 * SumScal1 > 0)
        {
            return TopAbs_FORWARD;
        }
        else
        {
            return TopAbs_REVERSED;
        }
    }


}
