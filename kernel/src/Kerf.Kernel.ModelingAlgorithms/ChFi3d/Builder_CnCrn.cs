// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_CnCrn.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Fairing;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Plate;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.Builder_CnCrn.ChFi3d_SurfType;
using static Kerf.Engine.Fillets.Analytic.ComputeDataFcts;
using static Kerf.Engine.Fillets.Builder_CnCrn;
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
internal partial class FilletBuilderBase
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

    internal void PerformMoreThreeCorner(int jndex, int nconges)
    {
    //    ========================================
    //             Initialisations
    //     ========================================
        DsDataStructure dStr = _ds.ChangeDS();
        Vertex v1 = _vDataMap.FindKey(jndex);
        int nedge = 0;
        bool bordlibre = false;
        Edge edgelibre1 = new(), edgelibre2 = new();
        //  NCollection_List<TopoDS_Shape>::Iterator ItE;
        nedge = ChFi3d_NbNotDegeneratedEdges(v1, _veMap);
        ChFi3d_ChercheBordsLibres(_veMap, v1, out bordlibre, ref edgelibre1, ref edgelibre2);
        bool droit = false;
        if (bordlibre)
        {
            nedge = (nedge - 2) / 2 + 2;
            double angedg = Math.Abs(ChFi3d_AngleEdge(v1, edgelibre1, edgelibre2));
            droit = Math.Abs(angedg - M_PI) < 0.01;
        }
        else
        {
            nedge = nedge / 2;
        }
        int size = nedge * 2;
        NCollection_Array1<Stripe> cd = new(0, size);
        NCollection_Array1<int> jf = new(0, size);
        NCollection_Array1<int> Index = new(0, size);
        NCollection_Array1<int> Indice = new(0, size);
        NCollection_Array1<int> sens = new(0, size);
        NCollection_Array1<int> indcurve3d = new(0, size);
        NCollection_Array2<int> numfa = new(0, size, 0, size);
        NCollection_Array1<int> order = new(0, size);
        NCollection_Array2<int> i = new(0, size, 0, size);
        NCollection_Array2<int> indpoint = new(0, size, 0, 1);
        NCollection_Array1<bool> oksea = new(0, size);
        NCollection_Array1<bool> sharp = new(0, size);
        NCollection_Array1<bool> regul = new(0, size);
        NCollection_Array2<double> p = new(0, size, 0, size);
        NCollection_Array1<double> errapp = new(0, size);
        NCollection_Array1<Shape> evive = new(0, size);
        NCollection_Array2<Shape> fvive = new(0, size, 0, size);
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
        NCollection_List<Stripe>.Iterator it = new();
        Stripe? cdbid = null, cnext = null; // OCCT's unused cd2 left out
        Face face = new();
        int jfp = 0, ii;
        int ic = 0, icplus = 0, icmoins = 0, icplus2 = 0, sense = 0, index = 0, indice = 0, isurf1 = 0, isurf2 = 0;
        int n3d = 0, iVtx = 0, nb;
        bool sameside = false, trouve = false, isfirst = false;
        double pardeb = 0.0, parfin = 0.0, xdir = 0.0, ydir = 0.0;
        double tolapp = 1.0e-4, maxapp = 0.0, maxapp1 = 0.0, avedev;
        DsCurvePointInterference? interfp1 = null, interfp2 = null;
        DsSurfaceCurveInterference? interfc = null;
        Curve3D? curv3d = null;
        Regul regular = new();
        NCollection_Sequence<Shape> fproj = new();
        int num = 0;
        Edge Ecur = new();
        NCollection_List<Shape>.Iterator itF = new();
        Face f1 = new(), f2 = new();
        Vector3D sumFaceNormalAtV1 = new(0, 0, 0); // is used to define Plate orientation

        // it is determined if there is a sewing edge
        bool couture = false;
        Face facecouture = new();
        Edge edgecouture = new();
        for (itF.Initialize(_vfMap[v1]); itF.More() && !couture; itF.Next())
        {
            Face fcur = ShapeCasts.Face(itF.Value());
            ChFi3d_CoutureOnVertex(fcur, v1, out couture, ref edgecouture);
            if (couture)
            {
                facecouture = fcur;
            }
        }

        // unused surfdata are removed
        RemoveSurfData(_vDataMap, _efMap, edgecouture, facecouture, v1);

        // parse edges and faces
        trouve = false;
        Edge enext = new();
        Vertex vv = new();
        Face Fcur = new(), fnext = new();
        for (it.Initialize(_vDataMap[jndex]); it.More() && !trouve; it.Next())
        {
            cnext = it.Value();
            cd.SetValue(0, cnext);
            Index.SetValue(0, ChFi3d_IndexOfSurfData(v1, cnext, out sense));
            sens.SetValue(0, sense);
            numfa.SetValue(0, 1, SurfIndex(cd, 0, Index.Value(0), FACE2));
            numfa.SetValue(1, 0, numfa.Value(0, 1));
            Fcur = ShapeCasts.Face(dStr.Shape(numfa.Value(0, 1)));
            fvive.SetValue(0, 1, Fcur);
            fvive.SetValue(1, 0, Fcur);
            jf.SetValue(0, 2);
            if (sens.Value(0) == 1)
            {
                Ecur = cd.Value(0).Spine().Edges(1);
            }
            else
            {
                Ecur = cd.Value(0).Spine().Edges(cd.Value(0).Spine().EdgeCount);
            }
            evive.SetValue(0, Ecur);
            ChFi3d_cherche_edge(v1, evive, Fcur, ref enext, ref vv);
            trouve = !enext.IsNull;
        }
        // find sum of all face normals at V1
        SummarizeNormal(v1, Fcur, Ecur, ref sumFaceNormalAtV1);

        int nbcouture = 0;
        for (ii = 1; ii < nedge; ii++)
        {
            if (Fcur.IsSame(facecouture) && nbcouture == 0)
            {
                enext = edgecouture;
                nbcouture++;
            }
            else
            {
                ChFi3d_cherche_edge(v1, evive, Fcur, ref enext, ref vv);
            }
            if (enext.IsNull)
            {
                throw new ConstructionErrorException("PerformMoreThreeCorner: pb in the parsing of edges and faces");
            }
            if (enext.IsSame(edgelibre1) || enext.IsSame(edgelibre2))
            {
                cd.SetValue(ii, cdbid);
                Index.SetValue(ii, 0);
                sens.SetValue(ii, -1);
                Vertex vref = new();
                vref = Exploration.FirstVertex(enext);
                if (vref.IsSame(v1))
                {
                    sens.SetValue(ii, 1);
                }
                sharp.SetValue(ii, true);
                evive.SetValue(ii, enext);
                jf.SetValue(ii, 0);
                Indices(nedge, ii, ref icplus, ref icmoins);
                fvive.SetValue(ii, icplus, Fcur);
                fvive.SetValue(icplus, ii, Fcur);
                numfa.SetValue(ii, icplus, dStr.AddShape(Fcur));
                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                ii++;
                if (enext.IsSame(edgelibre1))
                {
                    Ecur = edgelibre2;
                }
                else
                {
                    Ecur = edgelibre1;
                }
                ChFi3d_edge_common_faces(_efMap[Ecur], out Fcur, out Fcur);
                Indices(nedge, ii, ref icplus, ref icmoins);
                fvive.SetValue(ii, icplus, Fcur);
                fvive.SetValue(icplus, ii, Fcur);
                numfa.SetValue(ii, icplus, dStr.AddShape(Fcur));
                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                cd.SetValue(ii, cdbid);
                Index.SetValue(ii, 0);
                sens.SetValue(ii, -1);
                vref = Exploration.FirstVertex(Ecur);
                if (vref.IsSame(v1))
                {
                    sens.SetValue(ii, 1);
                }
                sharp.SetValue(ii, true);
                evive.SetValue(ii, Ecur);
                jf.SetValue(ii, 0);
            }
            else
            {
                // it is found if Enext is in the map of stripes
                Edge ee = new();
                /*bool */ trouve = false;
                for (it.Initialize(_vDataMap[jndex]); it.More() && !trouve; it.Next())
                {
                    index = ChFi3d_IndexOfSurfData(v1, it.Value(), out sense);
                    if (sense == 1)
                    {
                        ee = it.Value().Spine().Edges(1);
                    }
                    else
                    {
                        ee = it.Value().Spine().Edges(it.Value().Spine().EdgeCount);
                    }
                    if (enext.IsSame(ee))
                    {
                        cnext = it.Value();
                        trouve = true;
                    }
                }
                if (trouve)
                {
                    cd.SetValue(ii, cnext);
                    Index.SetValue(ii, index);
                    sens.SetValue(ii, sense);
                    sharp.SetValue(ii, false);
                    evive.SetValue(ii, enext);
                }
                else
                {
                    //      edge ii is alive
                    cd.SetValue(ii, cdbid);
                    Index.SetValue(ii, 0);
                    sens.SetValue(ii, -1);
                    Vertex vref = new();
                    vref = Exploration.FirstVertex(enext);
                    if (vref.IsSame(v1))
                    {
                        sens.SetValue(ii, 1);
                    }
                    sharp.SetValue(ii, true);
                    evive.SetValue(ii, enext);
                    jf.SetValue(ii, 0);
                }
                // Face Fnext!=Fcur containing Enext
                fnext = Fcur;
                ChFi3d_cherche_face1(_efMap[enext], Fcur, ref fnext);
                Indices(nedge, ii, ref icplus, ref icmoins);
                fvive.SetValue(ii, icplus, fnext);
                fvive.SetValue(icplus, ii, fnext);
                numfa.SetValue(ii, icplus, dStr.AddShape(fnext));
                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                int numface1 = 0, numface2 = 0;
                if (trouve)
                {
                    // it is checked if numfa corresponds to IndexOfS1 or IndexOfS2
                    // jf is updated is consequently updated
                    // if it is not the case among the previous faces are found
                    // those which correspond to  IndexOfs1 IndexOfS2  and
                    // numfa and Fvive are reupdated (cts16288)
                    numface2 = SurfIndex(cd, ii, Index.Value(ii), FACE2);
                    if (numface2 == numfa.Value(ii, icplus))
                    {
                        jf.SetValue(ii, 2);
                    }
                    else
                    {
                        numface1 = SurfIndex(cd, ii, Index.Value(ii), FACE1);
                        if (numface1 == numfa.Value(ii, icplus))
                        {
                            jf.SetValue(ii, 1);
                        }
                        else
                        {
                            if (numface1 == numfa.Value(icmoins, ii))
                            {
                                jf.SetValue(ii, 2);
                                fvive.SetValue(ii, icplus, ShapeCasts.Face(dStr.Shape(numface2)));
                                fvive.SetValue(icplus, ii, ShapeCasts.Face(dStr.Shape(numface2)));
                                numfa.SetValue(ii, icplus, dStr.AddShape(ShapeCasts.Face(dStr.Shape(numface2))));
                                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                            }
                            if (numface2 == numfa.Value(icmoins, ii))
                            {
                                jf.SetValue(ii, 1);
                                fvive.SetValue(ii, icplus, ShapeCasts.Face(dStr.Shape(numface1)));
                                fvive.SetValue(icplus, ii, ShapeCasts.Face(dStr.Shape(numface1)));
                                numfa.SetValue(ii, icplus, dStr.AddShape(ShapeCasts.Face(dStr.Shape(numface1))));
                                numfa.SetValue(icplus, ii, numfa.Value(ii, icplus));
                            }
                        }
                    }
                }
                Ecur = enext;
                Fcur = fnext;
                // find sum of all face normales at V1
                SummarizeNormal(v1, Fcur, Ecur, ref sumFaceNormalAtV1);
            }
        }
        // mise a jour du tableau regul
        for (ic = 0; ic < nedge; ic++)
        {
            if (sharp.Value(ic))
            {
                Ecur = ShapeCasts.Edge(evive.Value(ic));
                if (!Ecur.IsSame(edgecouture))
                {
                    ChFi3d_edge_common_faces(_efMap[Ecur], out f1, out f2);
                    //  Modified by Sergey KHROMOV - Fri Dec 21 18:11:02 2001 Begin
                    // 	regul.SetValue(ic,BRep_Tool::Continuity(TopoDS::Edge(Evive.Value(ic)),F1,F2)
                    // 		     !=GeomAbs_C0);
                    regul.SetValue(ic, global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(ShapeCasts.Edge(evive.Value(ic)), f1, f2));
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
                Edge ereg = ShapeCasts.Edge(evive.Value(ic));
                for (ind = 0; ind < nedge && !trouve; ind++)
                {
                    if (ind != ic)
                    {
                        Edge ecur = ShapeCasts.Edge(evive.Value(ind));
                        double ang = Math.Abs(ChFi3d_AngleEdge(v1, ecur, ereg));
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
            Edge e1 = new(), e2 = new();
            for (ic = 0; ic < nedge && !trouve; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (!sharp.Value(ic) && !sharp.Value(icplus))
                {
                    e1 = ShapeCasts.Edge(evive.Value(ic));
                    e2 = ShapeCasts.Edge(evive.Value(icplus));
                    deuxconges = (Math.Abs(ChFi3d_AngleEdge(v1, e1, e2)) < 0.01);
                    trouve = deuxconges;
                }
            }
        }

        // variable deuxconges is used in the special case when there are
        // two fillets and if two other living edges are tangent (cts60072)
        if (nconges == 2 && nedge == 4)
        {
            Edge e1 = new(), e2 = new();
            for (ic = 0; ic < nedge && !deuxconges; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (sharp.Value(ic) && sharp.Value(icplus))
                {
                    e1 = ShapeCasts.Edge(evive.Value(ic));
                    e2 = ShapeCasts.Edge(evive.Value(icplus));
                    if (!e1.IsSame(edgelibre1) && !e1.IsSame(edgelibre2) && !e2.IsSame(edgelibre1)
                            && !e2.IsSame(edgelibre2))
                    {
                        double ang = Math.Abs(ChFi3d_AngleEdge(v1, e1, e2));
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
                State stat = default;
                if (sens[ic] == 1)
                {
                    stat = cd.Value(ic).Spine().FirstStatus();
                }
                else
                {
                    stat = cd.Value(ic).Spine().LastStatus();
                }

                if (stat == State.OnSame)
                {
                    isOnSame = true;
                }
                else if (stat == State.OnDiff)
                {
                    isOnDiff = true;
                }
            }
            isOnSameDiff = isOnSame && isOnDiff;
        }
        if (isOnSameDiff)
        {
            PerformOneCorner(jndex, true);
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
                    Edge arc = ShapeCasts.Edge(evive.Value(ic));
                    CommonPoint cp1 = new(), cp2 = new();
                    double angedg = M_PI;
                    Vertex vcom = new();
                    if (!sharp.Value(icplus))
                    {
                        isfirst = (sens.Value(icplus) == 1);
                        jfp = 3 - jf.Value(icplus);
                        cp1 = cd.Value(icplus)
                                        .SetOfSurfData()
                                        .Value(Index.Value(icplus))
                                        .ChangeVertex(isfirst, jfp);
                        if (cp1.IsOnArc)
                        {
                            ChFi3d_cherche_vertex(arc, cp1.Arc(), ref vcom, out trouve);
                            if (trouve)
                            {
                                angedg = Math.Abs(ChFi3d_AngleEdge(vcom, arc, cp1.Arc()));
                            }
                            if (!cp1.Arc().IsSame(arc) && Math.Abs(angedg - M_PI) < 0.01)
                            {
                                evive.SetValue(ic, cp1.Arc());
                                ChFi3d_edge_common_faces(_efMap[cp1.Arc()], out f1, out f2);
                                if (!fvive.Value(ic, icplus).IsSame(f1) && !fvive.Value(ic, icplus).IsSame(f2))
                                {
                                    if (fvive.Value(ic, icmoins).IsSame(f2))
                                    {
                                        fvive.SetValue(ic, icplus, f1);
                                        fvive.SetValue(icplus, ic, f1);
                                        numfa.SetValue(ic, icplus, dStr.AddShape(f1));
                                        numfa.SetValue(icplus, ic, dStr.AddShape(f1));
                                    }
                                    else
                                    {
                                        fvive.SetValue(ic, icplus, f2);
                                        fvive.SetValue(icplus, ic, f2);
                                        numfa.SetValue(ic, icplus, dStr.AddShape(f2));
                                        numfa.SetValue(icplus, ic, dStr.AddShape(f2));
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
                        cp2 = cd.Value(icmoins)
                                        .SetOfSurfData()
                                        .Value(Index.Value(icmoins))
                                        .ChangeVertex(isfirst, jf.Value(icmoins));
                        if (cp2.IsOnArc)
                        {
                            angedg = M_PI;
                            ChFi3d_cherche_vertex(arc, cp2.Arc(), ref vcom, out trouve);
                            if (trouve)
                            {
                                angedg = Math.Abs(ChFi3d_AngleEdge(vcom, arc, cp2.Arc()));
                            }
                            if (!cp2.Arc().IsSame(arc) && Math.Abs(angedg - M_PI) < 0.01)
                            {
                                evive.SetValue(ic, cp2.Arc());
                                ChFi3d_edge_common_faces(_efMap[cp2.Arc()], out f1, out f2);
                                if (!fvive.Value(ic, icmoins).IsSame(f1) && !fvive.Value(ic, icmoins).IsSame(f2))
                                {
                                    if (fvive.Value(ic, icplus).IsSame(f2))
                                    {
                                        fvive.SetValue(ic, icmoins, f1);
                                        numfa.SetValue(ic, icmoins, dStr.AddShape(f1));
                                        fvive.SetValue(icmoins, ic, f1);
                                        numfa.SetValue(icmoins, ic, dStr.AddShape(f1));
                                    }
                                    else
                                    {
                                        fvive.SetValue(ic, icmoins, f2);
                                        numfa.SetValue(ic, icmoins, dStr.AddShape(f2));
                                        fvive.SetValue(icmoins, ic, f2);
                                        numfa.SetValue(icmoins, ic, dStr.AddShape(f2));
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
            Edge ecom = new();
            ecom = ShapeCasts.Edge(evive.Value(ic));
            if (ecom.IsSame(edgelibre1) || ecom.IsSame(edgelibre2))
            {
                libre.SetValue(ic, true);
                trouve = true;
            }
        }

        // determine the minimum recoil distance that can't be exceeded
        bool distmini = false;
        Point3D som = Tool.Pnt(v1), pic;
        Point2D p2 = new();
        Edge edgemin = new();
        Vertex v = new(), v2 = new();
        double dst = 0.0, distmin = 0.0;
        distmin = 1.0e30;
        for (ic = 0; ic < nedge; ic++)
        {
            if (sharp.Value(ic))
            {
                edgemin = ShapeCasts.Edge(evive.Value(ic));
            }
            else
            {
                if (sens.Value(ic) == 1)
                {
                    edgemin = cd.Value(ic).Spine().Edges(1);
                }
                else
                {
                    edgemin = cd.Value(ic).Spine().Edges(cd.Value(ic).Spine().EdgeCount);
                }
            }
            v = Exploration.FirstVertex(edgemin);
            v2 = Exploration.LastVertex(edgemin);
            dst = (Tool.Pnt(v)).DistanceTo(Tool.Pnt(v2)) / 1.5;
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
                Stripe? strip = null;
                double angedg = 0.0;
                int iface = 0;
                // if two edges are tangent the intersection is not attempted (cts60046)
                angedg = Math.Abs(ChFi3d_AngleEdge(v1, ShapeCasts.Edge(evive.Value(ic)), ShapeCasts.Edge(evive.Value(icplus))));
                if (Math.Abs(angedg - M_PI) > 0.01)
                {
                    ok = ChFi3d_SearchFD(dStr, cd.Value(ic), cd.Value(icplus), sens.Value(ic), sens.Value(icplus), ref i1, ref i2,
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
                        strip = cd.Value(ic);
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
                                    iface = SurfIndex(cd, ic, nb, FACE1);
                                }
                                else
                                {
                                    iface = SurfIndex(cd, ic, nb, FACE2);
                                }
                                fproj.Append(ShapeCasts.Face(_ds.Shape(iface)));
                            }
                        }
                        if (i1 > Index.Value(ic))
                        {
                            for (nb = Index.Value(ic); nb <= i1; nb++)
                            {
                                if ((3 - jf1) == 1)
                                {
                                    iface = SurfIndex(cd, ic, nb, FACE1);
                                }
                                else
                                {
                                    iface = SurfIndex(cd, ic, nb, FACE2);
                                }
                                fproj.Append(ShapeCasts.Face(_ds.Shape(iface)));
                            }
                        }
                        strip = cd.Value(ic);
                        RemoveSD(strip, ideb, ifin);
                        num = ChFi3d_IndexOfSurfData(v1, cd.Value(ic), out sense);
                        Index.SetValue(ic, num);
                        i1 = num;
                    }
                    if (i2 != Index.Value(icplus))
                    {
                        int ideb = 0, ifin = 0;
                        strip = cd.Value(icplus);
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
                                    iface = SurfIndex(cd, icplus, nb, FACE1);
                                }
                                else
                                {
                                    iface = SurfIndex(cd, icplus, nb, FACE2);
                                }
                                fproj.Append(ShapeCasts.Face(_ds.Shape(iface)));
                            }
                        }
                        if (i2 > Index.Value(icplus))
                        {
                            for (nb = i2; nb >= Index.Value(icplus); nb--)
                            {
                                if ((3 - jfp) == 1)
                                {
                                    iface = SurfIndex(cd, icplus, nb, FACE1);
                                }
                                else
                                {
                                    iface = SurfIndex(cd, icplus, nb, FACE2);
                                }
                                fproj.Append(ShapeCasts.Face(_ds.Shape(iface)));
                            }
                        }
                        RemoveSD(strip, ideb, ifin);
                        num = ChFi3d_IndexOfSurfData(v1, cd.Value(icplus), out sense);
                        Index.SetValue(icplus, num);
                        i2 = num;
                    }
                    Calcul_P2dOnSurf(cd.Value(ic), jf1, i1, pa1, ref p2);
                    indice = SurfIndex(cd, ic, i1, ChFiSURFACE);
                    dStr.Surface(indice).Surface().D0(p2.X, p2.Y, out pic);
                    if (pic.DistanceTo(som) > distmin)
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
                            para = Tool.Parameter(v1, ShapeCasts.Edge(evive.Value(ic)));
                            p.SetValue(ic, icplus, para);
                            i.SetValue(ic, icplus, 1);
                        }
                    }
                    else
                    {
                        isfirst = (sens.Value(ic) == 1);
                        i.SetValue(ic, icplus, ChFi3d_IndexOfSurfData(v1, cd.Value(ic), out sense));
                        if (oksea.Value(icmoins))
                        {
                            para = p.Value(ic, icmoins);
                            p.SetValue(ic, icplus, para);
                        }
                        else
                        {
                            Calcul_Param(cd.Value(ic), jf.Value(ic), i.Value(ic, icplus), isfirst, ref para);
                            p.SetValue(ic, icplus, para);
                        }
                    }
                    if (sharp.Value(icplus))
                    {
                        if (!samedge.Value(icplus))
                        {
                            para = Tool.Parameter(v1, ShapeCasts.Edge(evive.Value(icplus)));
                            p.SetValue(icplus, ic, para);
                            i.SetValue(icplus, ic, 1);
                        }
                    }
                    else
                    {
                        isfirst = (sens.Value(icplus) == 1);
                        i.SetValue(icplus, ic, ChFi3d_IndexOfSurfData(v1, cd.Value(icplus), out sense));
                        if (oksea.Value(icplus))
                        {
                            para = p.Value(icplus, icplus2);
                            p.SetValue(icplus, ic, para);
                        }
                        else
                        {
                            jfp = 3 - jf.Value(icplus);
                            Calcul_Param(cd.Value(icplus), jfp, i.Value(icplus, ic), isfirst, ref para);
                            p.SetValue(icplus, ic, para);
                        }
                    }
                }
            }

            //  calculate max distance to the top at each point
            NCollection_Array1<double> dist1 = new(0, size);
            NCollection_Array1<double> dist2 = new(0, size);
            double distance = 0.0;
            Point3D sommet_ = Tool.Pnt(v1); // OCCT's sommet, which C# cannot redeclare in this scope
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
                        Calcul_P2dOnSurf(cd.Value(ic), jfp, i.Value(ic, icmoins), p.Value(ic, icmoins), ref p2);
                        indice = SurfIndex(cd, ic, i.Value(ic, icmoins), ChFiSURFACE);
                        dStr.Surface(indice).Surface().D0(p2.X, p2.Y, out pic);
                        dist1.SetValue(ic, sommet_.DistanceTo(pic));
                        if (dist1.Value(ic) > distance)
                        {
                            distance = dist1.Value(ic);
                        }

                        Calcul_P2dOnSurf(cd.Value(ic), jf.Value(ic), i.Value(ic, icplus), p.Value(ic, icplus), ref p2);
                        indice = SurfIndex(cd, ic, i.Value(ic, icplus), ChFiSURFACE);
                        dStr.Surface(indice).Surface().D0(p2.X, p2.Y, out pic);
                        dist2.SetValue(ic, sommet_.DistanceTo(pic));
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
                        EdgeCurveAdaptor c = new(ShapeCasts.Edge(evive.Value(ic)));
                        // to pass from 3D distance to a parametric distance
                        if (!tangentregul[ic])
                        {
                            ec = distance * 100 * c.Resolution(0.01);
                        }
                        else
                        {
                            ec = 0.0;
                        }
                        if (Exploration.FirstVertex(ShapeCasts.Edge(evive.Value(ic))).IsSame(v1))
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
        Point3D pe = new(), sommet = Tool.Pnt(v1);
        if (!deuxconges)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                if (sharp.Value(ic))
                {
                    Indices(nedge, ic, ref icplus, ref icmoins);
                    EdgeCurveAdaptor c = new(ShapeCasts.Edge(evive.Value(ic)));
                    pe = c.Value(p.Value(ic, icplus));
                    double d1 = 0.0, d2 = 0.0, dS = pe.DistanceTo(sommet);
                    CommonPoint cp1 = new(), cp2 = new();
                    if (!sharp.Value(icplus))
                    {
                        isfirst = (sens.Value(icplus) == 1);
                        jfp = 3 - jf.Value(icplus);
                        cp1 = cd.Value(icplus)
                                        .SetOfSurfData()
                                        .Value(i.Value(icplus, ic))
                                        .ChangeVertex(isfirst, jfp);
                        d1 = cp1.Point().DistanceTo(sommet);
                    }
                    if (!sharp.Value(icmoins))
                    {
                        isfirst = (sens.Value(icmoins) == 1);
                        cp2 = cd.Value(icmoins)
                                        .SetOfSurfData()
                                        .Value(i.Value(icmoins, ic))
                                        .ChangeVertex(isfirst, jf.Value(icmoins));
                        d2 = cp2.Point().DistanceTo(sommet);
                    }
                    bool samecompoint = false;
                    if (!sharp.Value(icmoins) && !sharp.Value(icplus))
                    {
                        samecompoint = cp1.Point().DistanceTo(cp2.Point()) < tolapp;
                    }
                    if ((dS < d1 || dS < d2) && !samecompoint)
                    {
                        // step back till Common Points
                        // without leaving the Edge ??
                        if (d2 < d1 && cp1.IsOnArc)
                        {
                            // cp1 is chosen
                            p.SetValue(ic, icmoins, cp1.ParameterOnArc());
                            p.SetValue(ic, icplus, p.Value(ic, icmoins));
                            isfirst = (sens.Value(icplus) == 1);
                            jfp = 3 - jf.Value(icplus);
                            Calcul_Param(cd.Value(icplus), jfp, i.Value(icplus, ic), isfirst, ref para);
                            p.SetValue(icplus, ic, para);
                            if (cp1.Tolerance() > tolcp && cp1.Tolerance() < 1)
                            {
                                tolcp = cp1.Tolerance();
                            }
                        }
                        else if (cp2.IsOnArc)
                        {
                            // cp2 is chosen
                            p.SetValue(ic, icmoins, cp2.ParameterOnArc());
                            p.SetValue(ic, icplus, p.Value(ic, icmoins));
                            isfirst = (sens.Value(icmoins) == 1);
                            Calcul_Param(cd.Value(icmoins), jf.Value(icmoins), i.Value(icmoins, ic), isfirst, ref para);
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
                            if ((cp1.Point().DistanceTo(pe) < cp1.Tolerance() || samecompoint || nconges == 1)
                                    && cp1.IsOnArc)
                            {
                                // it is very close to cp1
                                p.SetValue(ic, icmoins, cp1.ParameterOnArc());
                                ponctuel.SetValue(ic, true);
                                p.SetValue(ic, icplus, p.Value(ic, icmoins));
                                isfirst = (sens.Value(icplus) == 1);
                                jfp = 3 - jf.Value(icplus);
                                Calcul_Param(cd.Value(icplus), jfp, i.Value(icplus, ic), isfirst, ref para);
                                p.SetValue(icplus, ic, para);
                                if (cp1.Tolerance() > tolcp && cp1.Tolerance() < 1)
                                {
                                    tolcp = cp1.Tolerance();
                                }
                            }
                        }
                        if (!sharp.Value(icmoins))
                        {
                            if ((cp2.Point().DistanceTo(pe) < cp2.Tolerance() || samecompoint || nconges == 1)
                                    && cp2.IsOnArc)
                            {
                                // it is very close to cp2
                                ponctuel.SetValue(icmoins, true);
                                p.SetValue(ic, icmoins, cp2.ParameterOnArc());
                                p.SetValue(ic, icplus, p.Value(ic, icmoins));
                                isfirst = (sens.Value(icmoins) == 1);
                                Calcul_Param(cd.Value(icmoins), jf.Value(icmoins), i.Value(icmoins, ic), isfirst, ref para);
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
            if (ShapeCasts.Edge(evive.Value(ic)).IsSame(edgelibre1)
                    || ShapeCasts.Edge(evive.Value(ic)).IsSame(edgelibre2))
            {
                int indic = 0;
                CommonPoint CP1 = new();
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
                    CP1 = cd.Value(indic).SetOfSurfData().Value(Index.Value(indic)).ChangeVertex(isfirst, 1);
                    /*bool*/ trouve = false;
                    if (CP1.IsOnArc)
                    {
                        if (CP1.Arc().IsSame(ShapeCasts.Edge(evive.Value(ic))))
                        {
                            p.SetValue(ic, icmoins, CP1.ParameterOnArc());
                            p.SetValue(ic, icplus, CP1.ParameterOnArc());
                            trouve = true;
                        }
                    }
                    if (!trouve)
                    {
                        CP1 =
                            cd.Value(indic).SetOfSurfData().Value(Index.Value(indic)).ChangeVertex(isfirst, 2);
                        if (CP1.IsOnArc)
                        {
                            if (CP1.Arc().IsSame(ShapeCasts.Edge(evive.Value(ic))))
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
        NCollection_Array1<Curve2D> proj2d1 = new(0, size);
        NCollection_Array1<Curve2D> proj2d2 = new(0, size);
        NCollection_Array1<Curve3D> cproj1 = new(0, size);
        NCollection_Array1<Curve3D> cproj2 = new(0, size);
        if (!deuxconges)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                int ilin = 0;
                NCollection_Sequence<Curve3D> cr = new();
                NCollection_Sequence<Curve2D> pr = new();
                NCollection_Sequence<Shape> lface = new();
                NCollection_Sequence<Shape> ledge = new();
                lface.Clear();
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
                            ilin = SurfIndex(cd, icmoins, i.Value(icmoins, ic), FACE1);
                        }
                        else
                        {
                            ilin = SurfIndex(cd, icmoins, i.Value(icmoins, ic), FACE2);
                        }
                        lface.Append(ShapeCasts.Face(dStr.Shape(ilin)));
                    }
                    else
                    {
                        lface.Append(fvive[ic, icmoins]);
                    }
                    if (indfin > icmoins)
                    {
                        nbface = indfin - icmoins;
                    }
                    else
                    {
                        nbface = nedge - (icmoins - indfin);
                    }
                    NCollection_Sequence<Shape> epj = new();
                    NCollection_Sequence<double> seqpr = new();
                    ii = ic;
                    for (int nf = 1; nf <= nbface - 1; nf++)
                    {
                        int iimoins = 0, iiplus = 0;
                        Indices(nedge, ii, ref iiplus, ref iimoins);
                        ledge.Append(ShapeCasts.Edge(evive.Value(ii)));
                        seqpr.Append(p.Value(ii, iiplus));
                        if (nf != nbface - 1)
                        {
                            lface.Append(fvive[ii, iiplus]);
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
                            ilin = SurfIndex(cd, indfin, i.Value(indfin, indfinmoins), FACE1);
                        }
                        else
                        {
                            ilin = SurfIndex(cd, indfin, i.Value(indfin, indfinmoins), FACE2);
                        }
                        lface.Append(ShapeCasts.Face(dStr.Shape(ilin)));
                    }
                    else
                    {
                        lface.Append(fvive[indfin, indfinmoins]);
                    }
                    CurveHermite(dStr, cd.Value(icmoins), jf.Value(icmoins), i.Value(icmoins, ic), p.Value(icmoins, ic),
                                              sens.Value(icmoins), sharp.Value(icmoins), ShapeCasts.Edge(evive.Value(icmoins)), cd.Value(indfin),
                                              jf.Value(indfin), i.Value(indfin, indfinmoins), p.Value(indfin, indfinmoins), sens.Value(indfin),
                                              sharp.Value(indfin), ShapeCasts.Edge(evive.Value(indfin)), nbface, ledge, lface, pr, cr, epj, seqpr,
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

        NCollection_Sequence<Shape> Ecom = new();
        NCollection_Sequence<Shape> eproj = new();
        NCollection_Sequence<double> parcom = new();
        if (!deuxconges)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                int iface1 = 0, iface2 = 0;
                Face face1 = new(), face2 = new();
                Edge edge = new();
                NCollection_Sequence<Curve3D> cr = new();
                NCollection_Sequence<Curve2D> pr = new();
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (!oksea.Value(ic))
                {
                    iface1 = numfa.Value(ic, icplus);
                    iface2 = numfa.Value(icplus, ic);
                    if (!sharp.Value(ic))
                    {
                        if (jf.Value(ic) == 1)
                        {
                            iface1 = SurfIndex(cd, ic, i.Value(ic, icplus), FACE1);
                        }
                        else
                        {
                            iface1 = SurfIndex(cd, ic, i.Value(ic, icplus), FACE2);
                        }
                    }
                    face1 = ShapeCasts.Face(_ds.Shape(iface1));

                    if (!sharp.Value(icplus))
                    {
                        if (jf.Value(icplus) == 1)
                        {
                            iface2 = SurfIndex(cd, icplus, i.Value(icplus, ic), FACE2);
                        }
                        else
                        {
                            iface2 = SurfIndex(cd, icplus, i.Value(icplus, ic), FACE1);
                        }
                    }
                    face2 = ShapeCasts.Face(_ds.Shape(iface2));
                    if (!face1.IsSame(face2))
                    {
                        if (fproj.Length() == 0)
                        {
                            fproj.Append(face1);
                            fproj.Append(face2);
                        }
                        moresurf.SetValue(ic, true);
                        nbface = fproj.Length();
                        if (!ShapeCasts.Face(fproj.Value(nbface)).IsSame(face2))
                        {
                            fproj.Remove(nbface);
                            fproj.Append(face2);
                        }
                        if (!ShapeCasts.Face(fproj.Value(1)).IsSame(face1))
                        {
                            fproj.Remove(1);
                            fproj.Prepend(face1);
                        }
                        for (nb = 1; nb <= nbface - 1; nb++)
                        {
                            cherche_edge1(ShapeCasts.Face(fproj.Value(nb)), ShapeCasts.Face(fproj.Value(nb + 1)), ref edge);
                            Ecom.Append(edge);
                            para = Tool.Parameter(Exploration.FirstVertex(edge), edge);
                            parcom.Append(para);
                        }
                        CurveHermite(dStr, cd.Value(ic), jf.Value(ic), i.Value(ic, icplus), p.Value(ic, icplus), sens.Value(ic),
                                                  sharp.Value(ic), ShapeCasts.Edge(evive.Value(ic)), cd.Value(icplus), jf.Value(icplus),
                                                  i.Value(icplus, ic), p.Value(icplus, ic), sens.Value(icplus), sharp.Value(icplus),
                                                  ShapeCasts.Edge(evive.Value(icplus)), nbface, Ecom, fproj, pr, cr, eproj, parcom, ref error);
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
            Point3D p1 = new(), P2 = new(), p3 = new(), p4 = new();
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
            Calcul_P2dOnSurf(cd.Value(ic1), jfp, i.Value(ic1, icmoins), p.Value(ic1, icmoins), ref p2);
            indice = SurfIndex(cd, ic1, i.Value(ic1, icmoins), ChFiSURFACE);
            dStr.Surface(indice).Surface().D0(p2.X, p2.Y, out p1);

            Calcul_P2dOnSurf(cd.Value(ic1), jf.Value(ic1), i.Value(ic1, icplus), p.Value(ic1, icplus), ref p2);
            indice = SurfIndex(cd, ic1, i.Value(ic1, icplus), ChFiSURFACE);
            dStr.Surface(indice).Surface().D0(p2.X, p2.Y, out P2);

            jfp = 3 - jf.Value(ic2);
            Indices(nedge, ic2, ref icplus, ref icmoins);
            Calcul_P2dOnSurf(cd.Value(ic2), jfp, i.Value(ic2, icmoins), p.Value(ic2, icmoins), ref p2);
            indice = SurfIndex(cd, ic2, i.Value(ic2, icmoins), ChFiSURFACE);
            dStr.Surface(indice).Surface().D0(p2.X, p2.Y, out p3);

            Calcul_P2dOnSurf(cd.Value(ic2), jf.Value(ic2), i.Value(ic2, icplus), p.Value(ic2, icplus), ref p2);
            indice = SurfIndex(cd, ic2, i.Value(ic2, icplus), ChFiSURFACE);
            dStr.Surface(indice).Surface().D0(p2.X, p2.Y, out p4);
            intersection = (p1.DistanceTo(p4) <= 1.0e-7 || p1.DistanceTo(p3) <= 1.0e-7)
                                          && (P2.DistanceTo(p4) <= 1.0e-7 || P2.DistanceTo(p3) <= 1.0e-7);
            if (intersection)
            {
                PerformTwoCornerSameExt(dStr, cd.Value(ic1), Index.Value(ic1), sens.Value(ic1), cd.Value(ic2), Index.Value(ic2),
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
        BuildPlateSurface pSurf = new(degree, nbcurvpnt, nbiter, _tol2d, _tolapp3d, _angular);
        // calculation of curves on surface for each stripe
        for (ic = 0; ic < nedge; ic++)
        {
            Point2D p2d1 = new(), p2d2 = new();
            if (!sharp.Value(ic))
            {
                n3d++;
                Indices(nedge, ic, ref icplus, ref icmoins);
                jfp = 3 - jf.Value(ic);
                Calcul_P2dOnSurf(cd.Value(ic), jfp, i.Value(ic, icmoins), p.Value(ic, icmoins), ref p2d1);
                Calcul_P2dOnSurf(cd.Value(ic), jf.Value(ic), i.Value(ic, icplus), p.Value(ic, icplus), ref p2d2);
                //      if (i[ic][icplus]!=  i[ic][icmoins]) std::cout<<"probleme surface"<<std::endl;
                indice = SurfIndex(cd, ic, i.Value(ic, icplus), ChFiSURFACE);
                SurfaceAdaptor? asurf = 
                    new SurfaceAdaptor(dStr.Surface(indice).Surface());
                // calculation of curve 2d
                xdir = p2d2.X - p2d1.X;
                ydir = p2d2.Y - p2d1.Y;
                double l0 = Math.Sqrt(xdir * xdir + ydir * ydir);
                Direction2D dir = new(xdir, ydir);
                LineCurve2D? l = new LineCurve2D(p2d1, dir);
                Curve2D? pcurve = new TrimmedCurve2D(l, 0, l0);
                CurveAdaptor2D? acurv = new CurveAdaptor2D(pcurve);
                CurveOnSurface curvOnS = new(acurv, asurf);
                CurveOnSurface? hCons = (CurveOnSurface)curvOnS.ShallowCopy(); // OCCT copy-constructs a handle
                // Order.SetValue(ic,1);
                order.SetValue(ic, constr);
                CurveConstraint? cont = 
                    new CurveConstraint(hCons, order.Value(ic), nbcurvpnt, _tolapp3d, _angular, 0.1);
                pSurf.Add(cont);

                // calculate indexes of points and of the curve for the DS
                isfirst = (sens.Value(ic) == 1);
                global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(tolapp, curvOnS, curvOnS.FirstParameter(), curvOnS.LastParameter(), ref curv3d, out maxapp,
                                                            out avedev);
                DsCurve tcurv3d = new(curv3d, maxapp);
                indcurve3d.SetValue(n3d, dStr.AddCurve(tcurv3d));
                Point3D point1 = new(), point2 = new();
                point1 = curvOnS.Value(curvOnS.FirstParameter());
                point2 = curvOnS.Value(curvOnS.LastParameter());

                DsPoint tpoint1 = new(point1, maxapp);
                DsPoint tpoint2 = new(point2, maxapp);
                errapp.SetValue(ic, maxapp);
                if (ic == 0)
                {
                    // it is necessary to create two points
                    indpoint.SetValue(ic, 0, dStr.AddPoint(tpoint1));
                    indpoint.SetValue(ic, 1, dStr.AddPoint(tpoint2));
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
                            ref DsPoint tpt = ref dStr.ChangePoint(indpoint.Value(ii, 1));
                            if (point1.DistanceTo(tpt.Point()) < 1.0e-4)
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
                        indpoint.SetValue(ic, 0, dStr.AddPoint(tpoint1));
                    }

                    trouve = false;
                    for (ii = 0; ii < ic && (!trouve); ii++)
                    {
                        if (!sharp.Value(ii))
                        {
                            ref DsPoint tpt = ref dStr.ChangePoint(indpoint.Value(ii, 0));
                            if (point2.DistanceTo(tpt.Point()) < 1.0e-4)
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
                        indpoint.SetValue(ic, 1, dStr.AddPoint(tpoint2));
                    }
                }

                //   update of the stripe
                isurf1 = 3 - jf.Value(ic);
                isurf2 = jf.Value(ic);
                if (isurf1 == 2)
                {
                    cd.Value(ic).SetOrientation(Reversed, isfirst);
                }
                cd.Value(ic).SetCurve(indcurve3d.Value(n3d), isfirst);
                cd.Value(ic).SetIndexPoint(indpoint.Value(ic, 0), isfirst, isurf1);
                cd.Value(ic).SetIndexPoint(indpoint.Value(ic, 1), isfirst, isurf2);
                cd.Value(ic).SetParameters(isfirst, pcurve.FirstParameter, pcurve.LastParameter);
                CommonPoint cp1 = new();
                CommonPoint cp2 = new();
                cp1.SetPoint(point1);
                cp2.SetPoint(point2);
                cd.Value(ic).SetOfSurfData().Value(i.Value(ic, icmoins)).ChangeVertex(isfirst, isurf1) =
                    cp1;
                cd.Value(ic).SetOfSurfData().Value(i.Value(ic, icmoins)).ChangeVertex(isfirst, isurf2) =
                    cp2;
                cd.Value(ic)
                    .SetOfSurfData()
                    .Value(i.Value(ic, icmoins))
                    .ChangeInterference(isurf1)
                    .SetParameter(p.Value(ic, icmoins), isfirst);
                cd.Value(ic)
                    .SetOfSurfData()
                    .Value(i.Value(ic, icmoins))
                    .ChangeInterference(isurf2)
                    .SetParameter(p.Value(ic, icplus), isfirst);
                cd.Value(ic).ChangePCurve(isfirst) = pcurve;
            }
        }

        // calculate the indices of points for living edges
        for (ic = 0; ic < nedge; ic++)
        {
            if (sharp.Value(ic))
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                EdgeCurveAdaptor c = new(ShapeCasts.Edge(evive.Value(ic)));
                /*gp_Pnt*/ pe = c.Value(p.Value(ic, icplus));
                DsPoint tpe = new(pe, Tool.Tolerance(ShapeCasts.Edge(evive.Value(ic))));
                CommonPoint cp = new();
                if (deuxconges)
                {
                    iVtx = dStr.AddShape(v1);
                    indpoint.SetValue(ic, 0, iVtx);
                    indpoint.SetValue(ic, 1, iVtx);
                }
                if (!sharp.Value(icplus))
                {
                    isfirst = (sens.Value(icplus) == 1);
                    jfp = 3 - jf.Value(icplus);
                    cp =
                        cd.Value(icplus).SetOfSurfData().Value(i.Value(icplus, ic)).ChangeVertex(isfirst, jfp);
                    if (cp.Point().DistanceTo(pe) <= Max(1.0e-4, tolcp))
                    {
                        // edge was limited by the 1st CommonPoint of CD[icplus]
                        indpoint.SetValue(ic, 0, indpoint.Value(icplus, 0));
                        indpoint.SetValue(ic, 1, indpoint.Value(icplus, 0));
                    }
                }
                if (!sharp.Value(icmoins))
                {
                    isfirst = (sens.Value(icmoins) == 1);
                    cp = cd.Value(icmoins)
                                  .SetOfSurfData()
                                  .Value(i.Value(icmoins, ic))
                                  .ChangeVertex(isfirst, jf.Value(icmoins));
                    if (cp.Point().DistanceTo(pe) <= Max(1.0e-4, tolcp))
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
                    indpoint.SetValue(ic, 0, dStr.AddPoint(tpe));
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
                    Point2D p2d1 = new(), p2d2 = new();
                    Curve2D? curv2d1 = null, curv2d2 = null;
                    Curve2D? pcurve = null;
                    Curve3D? curveint = null;
                    SurfaceAdaptor? asurf = null;
                    double u1bid = 0.0, u2bid = 0.0;

                    // return the 1st curve 2d
                    // and the 1st connection point
                    if (sharp.Value(ic))
                    {
                        curv2d1 = Tool.CurveOnSurface(ShapeCasts.Edge(evive.Value(ic)), ShapeCasts.Face(fvive.Value(ic, icplus)),
                                                                                                out u1bid, out u2bid);
                    }
                    else
                    {
                        Calcul_C2dOnFace(cd.Value(ic), jf.Value(ic), i.Value(ic, icplus), ref curv2d1);
                    }

                    if (curv2d1 is null)
                    {
                        throw new ConstructionErrorException("Failed to get p-curve of edge");
                    }
                    p2d1 = curv2d1.Value(p.Value(ic, icplus));

                    // recuperation de la deuxieme courbe 2d
                    // et du deuxieme point de raccordement
                    if (sharp.Value(icplus))
                    {
                        curv2d2 = Tool.CurveOnSurface(ShapeCasts.Edge(evive.Value(icplus)), ShapeCasts.Face(fvive.Value(ic, icplus)),
                                                                                                out u1bid, out u2bid);
                    }
                    else
                    {
                        jfp = 3 - jf.Value(icplus);
                        Calcul_C2dOnFace(cd.Value(icplus), jfp, i.Value(icplus, ic), ref curv2d2);
                    }
                    if (curv2d2 is null)
                    {
                        throw new ConstructionErrorException("Failed to get p-curve of edge");
                    }
                    p2d2 = curv2d2.Value(p.Value(icplus, ic));

                    asurf = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(fvive.Value(ic, icplus))));
                    double tolu = 0.0, tolv = 0.0, ratio = 0.0;
                    tolu = asurf.UResolution(1.0e-3);
                    tolv = asurf.VResolution(1.0e-3);
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
                        bool pi1 = false, pi2 = false;
                        double xx = 0.0;
                        pi1 = 0 <= p2d1.X && p2d1.X <= M_PI;
                        pi2 = 0 <= p2d2.X && p2d2.X <= M_PI;

                        if (evive.Value(ic).IsSame(edgecouture))
                        {
                            xx = p2d1.X;
                            if (pi2 && !pi1)
                            {
                                xx = xx - 2 * M_PI;
                            }
                            if (!pi2 && pi1)
                            {
                                xx = xx + 2 * M_PI;
                            }
                            p2d1 = p2d1.WithX(xx);
                        }
                        if (evive.Value(icplus).IsSame(edgecouture))
                        {
                            xx = p2d2.X;
                            if (pi2 && !pi1)
                            {
                                xx = xx + 2 * M_PI;
                            }
                            if (!pi2 && pi1)
                            {
                                xx = xx - 2 * M_PI;
                            }
                            p2d2 = p2d2.WithX(xx);
                        }
                    }
                    xdir = p2d2.X - p2d1.X;
                    ydir = p2d2.Y - p2d1.Y;

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
                                inverseic = Exploration.FirstVertex(ShapeCasts.Edge(evive.Value(ic))).IsSame(v1);
                            }
                            else
                            {
                                inverseic = sens.Value(ic) == 1;
                            }
                            if (sharp.Value(icplus))
                            {
                                inverseicplus = Exploration.FirstVertex(ShapeCasts.Edge(evive.Value(icplus))).IsSame(v1);
                            }
                            else
                            {
                                inverseicplus = sens.Value(icplus) == 1;
                            }
                            if (ShapeCasts.Edge(evive.Value(ic)).IsSame(edgelibre1)
                                    || ShapeCasts.Edge(evive.Value(ic)).IsSame(edgelibre2))
                            {
                                contraint1 = false;
                            }
                            if (ShapeCasts.Edge(evive.Value(icplus)).IsSame(edgelibre1)
                                    || ShapeCasts.Edge(evive.Value(icplus)).IsSame(edgelibre2))
                            {
                                contraint2 = false;
                            }
                            CalculBatten(asurf, ShapeCasts.Face(fvive[ic, icplus]), xdir, ydir, p2d1, p2d2, contraint1, contraint2,
                                                      curv2d1, curv2d2, p.Value(ic, icplus), p.Value(icplus, ic), inverseic, inverseicplus,
                                                      ref pcurve);
                        }

                        // construction of borders for Plate
                        CurveAdaptor2D? acurv = new CurveAdaptor2D(pcurve);
                        CurveOnSurface curvOnS = new(acurv, asurf);
                        CurveOnSurface? hCons = (CurveOnSurface)curvOnS.ShallowCopy(); // OCCT copy-constructs a handle

                        // constraints G1 are set if edges ic and icplus are not both alive

                        order.SetValue(n3d, 0);
                        if (!sharp.Value(ic) && !sharp.Value(icplus))
                        {
                            order.SetValue(n3d, 1);
                        }
                        if (!contraint1 && !sharp.Value(icplus))
                        {
                            order.SetValue(n3d, 1);
                        }
                        if (!contraint2 && !sharp.Value(ic))
                        {
                            order.SetValue(n3d, 1);
                        }
                        if (tangentregul[ic] || tangentregul[icplus])
                        {
                            order.SetValue(n3d, 1);
                        }
                        if (isG1.Value(ic))
                        {
                            order.SetValue(n3d, 1);
                        }
                        CurveConstraint? cont = 
                            new CurveConstraint(hCons, order.Value(n3d), 10, _tolapp3d, _angular, 0.1);
                        pSurf.Add(cont);

                        // calculation of curve 3d if it is not a projection
                        if (curveint is null)
                        {
                            global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(tolapp, curvOnS, curvOnS.FirstParameter(), curvOnS.LastParameter(), ref curv3d,
                                                                        out maxapp1, out avedev);
                            pardeb = curvOnS.FirstParameter();
                            parfin = curvOnS.LastParameter();
                            curveint = new TrimmedCurve3D(curv3d, pardeb, parfin);
                        }

                        // storage in the DS
                        DsCurve tcurv3d = new(curveint, maxapp1);
                        indcurve3d.SetValue(n3d, dStr.AddCurve(tcurv3d));
                        pardeb = curveint.FirstParameter;
                        parfin = curveint.LastParameter;
                        if (sharp.Value(icplus) && indpoint.Value(icplus, 0) == 0)
                        {
                            // it is necessary to initialize indpoint[icplus][0] and indpoint[icplus][1]
                            Point3D point2 = new();
                            point2 = curveint.Value(parfin);
                            DsPoint tpoint2 = new(point2, maxapp);
                            indpoint.SetValue(icplus, 0, dStr.AddPoint(tpoint2));
                            indpoint.SetValue(icplus, 1, indpoint.Value(icplus, 0));
                        }
                        bool isVt1 = false;
                        bool isVt2 = false;
                        if (deuxconges)
                        {
                            isVt1 = sharp.Value(ic);
                            isVt2 = sharp.Value(icplus);
                        }
                        interfp1 = ChFi3d_FilPointInDS(Forward, indcurve3d.Value(n3d), indpoint.Value(ic, 1), pardeb, isVt1);
                        interfp2 = ChFi3d_FilPointInDS(Reversed, indcurve3d.Value(n3d), indpoint[icplus, 0], parfin, isVt2);
                        dStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(interfp1);
                        dStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(interfp2);
                        if (!isVt1)
                        {
                            ref DsPoint tpt1 = ref dStr.ChangePoint(indpoint[ic, 1]);
                            tpt1.Tolerance(tpt1.Tolerance() + maxapp1);
                        }
                        if (!isVt2)
                        {
                            ref DsPoint tpt2 = ref dStr.ChangePoint(indpoint[icplus, 0]);
                            tpt2.Tolerance(tpt2.Tolerance() + maxapp1);
                        }

                        // calculate orientation of the curve
                        Orientation orinterf = default;
                        if (!sharp.Value(ic))
                        {
                            OrientationIcNonVive(cd.Value(ic), jf.Value(ic), i.Value(ic, icplus), sens.Value(ic), ref orinterf);
                        }
                        else if (!sharp.Value(icplus))
                        {
                            OrientationIcplusNonVive(cd.Value(icplus), jf.Value(icplus), i.Value(icplus, ic), sens.Value(icplus),
                                                                              ref orinterf);
                        }
                        else
                        {
                            OrientationAreteViveConsecutive(fvive.Value(ic, icplus), evive.Value(ic), v1, ref orinterf);
                        }
                        interfc =
                            ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), numfa.Value(ic, icplus), pcurve, orinterf);
                        dStr.ChangeShapeInterferences(numfa.Value(ic, icplus)).Append(interfc);
                    }
                } // end of processing by edge
            } // end of the loop on edges
        } // end of processing for intermediary curves

        //  storage in the DS of curves projected on several faces
        for (ic = 0; ic < nedge; ic++)
        {
            if (moresurf.Value(ic))
            {
                Vertex vf = new(), vl = new();
                Point3D pf = new(), pl = new(), p1 = new(), P2 = new(), pcom = new();
                ind = 0; // must be initialized because of possible use, see L2249
                double up1 = 0.0, up2 = 0.0;
                Orientation orvt = default;
                Orientation oredge = Forward;
                int indpoint1 = 0, indpoint2 = 0;
                Indices(nedge, ic, ref icplus, ref icmoins);
                Curve2D? proj = null, proj2d = null;
                Curve3D? projc = null, cproj = null;
                ref DsPoint tpt1 = ref dStr.ChangePoint(indpoint[ic, 1]);
                ref DsPoint tpt2 = ref dStr.ChangePoint(indpoint[icplus, 0]);
                tpt1.Tolerance(tpt1.Tolerance() + error);
                tpt2.Tolerance(tpt1.Tolerance() + error); // OCCT: tpt1's tolerance given to tpt2
                for (nb = 1; nb <= nbface; nb++)
                {
                    orvt = Reversed;
                    vf = Exploration.FirstVertex(ShapeCasts.Edge(Ecom.Value(nb)));
                    vl = Exploration.LastVertex(ShapeCasts.Edge(Ecom.Value(nb)));
                    pf = Tool.Pnt(vf);
                    pl = Tool.Pnt(vl);
                    para = parcom.Value(nb);
                    pcom = Tool.Curve(ShapeCasts.Edge(Ecom.Value(nb)), out up1, out up2).Value(para);
                    if (pf.DistanceTo(Tool.Pnt(v1)) < pl.DistanceTo(Tool.Pnt(v1)))
                    {
                        orvt = Forward;
                    }
                    if (!eproj.Value(nb).IsNull)
                    {
                        n3d++;
                        proj = Tool.CurveOnSurface(ShapeCasts.Edge(eproj.Value(nb)), ShapeCasts.Face(fproj.Value(nb)), out up1,
                                                                                          out up2);
                        if (proj is null)
                        {
                            throw new ConstructionErrorException("Failed to get p-curve of edge");
                        }
                        proj2d = new TrimmedCurve2D(proj, up1, up2);
                        projc = Tool.Curve(ShapeCasts.Edge(eproj.Value(nb)), out up1, out up2);
                        if (projc is null)
                        {
                            throw new ConstructionErrorException("Failed to get 3D curve of edge");
                        }
                        cproj = new TrimmedCurve3D(projc, up1, up2);
                        pardeb = cproj.FirstParameter;
                        parfin = cproj.LastParameter;
                        p1 = cproj.Value(pardeb);
                        P2 = cproj.Value(parfin);
                        if (p1.DistanceTo(tpt1.Point()) < 1.0e-3)
                        {
                            indpoint1 = indpoint[ic, 1];
                        }
                        else
                        {
                            indpoint1 = ind;
                        }
                        if (P2.DistanceTo(tpt2.Point()) < 1.0e-3)
                        {
                            indpoint2 = indpoint[icplus, 0];
                        }
                        else
                        {
                            DsPoint tpoint2 = new(P2, error);
                            indpoint2 = dStr.AddPoint(tpoint2);
                            ind = indpoint2;
                        }
                        SurfaceAdaptor? asurf = null;
                        asurf = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(fproj.Value(nb))));
                        CurveAdaptor2D? acurv = new CurveAdaptor2D(proj2d);
                        CurveOnSurface curvOnS = new(acurv, asurf);
                        CurveOnSurface? hCons = (CurveOnSurface)curvOnS.ShallowCopy(); // OCCT copy-constructs a handle
                        order.SetValue(n3d, 1);
                        CurveConstraint? cont = 
                            new CurveConstraint(hCons, order.Value(n3d), 10, _tolapp3d, _angular, 0.1);
                        pSurf.Add(cont);
                        DsCurve tcurv3d = new(cproj, error);
                        indcurve3d.SetValue(n3d, dStr.AddCurve(tcurv3d));
                        interfp1 = ChFi3d_FilPointInDS(Forward, indcurve3d.Value(n3d), indpoint1, pardeb);
                        interfp2 = ChFi3d_FilPointInDS(Reversed, indcurve3d.Value(n3d), indpoint2, parfin);
                        dStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(interfp1);
                        dStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(interfp2);
                        num = dStr.AddShape(fproj.Value(nb));
                        Explorer ex = new();
                        for (ex.Init(fproj.Value(nb).Oriented(Forward), ShapeType.Edge); ex.More(); ex.Next())
                        {
                            if (Ecom.Value(nb).IsSame(ex.Current()))
                            {
                                oredge = ex.Current().Orientation;
                                break;
                            }
                        }

                        // calculation of the orientation
                        Orientation orinterf = default;
                        if (p1.DistanceTo(pcom) > 1.0e-4)
                        {
                            if (orvt == Forward)
                            {
                                orinterf = oredge;
                            }
                            else
                            {
                                orinterf = global::Kerf.Engine.Topology.TopAbs.Reverse(oredge);
                            }
                        }
                        else
                        {
                            if (orvt == Forward)
                            {
                                orinterf = global::Kerf.Engine.Topology.TopAbs.Reverse(oredge);
                            }
                            else
                            {
                                orinterf = oredge;
                            }
                        }
                        interfc = ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), num, proj2d, orinterf);
                        dStr.ChangeShapeInterferences(num).Append(interfc);
                    }
                    indice = ind;
                    if (nb != nbface)
                    {
                        if (eproj.Value(nb).IsNull)
                        {
                            indice = indpoint[ic, 1];
                        }
                        if (eproj.Value(nb + 1).IsNull)
                        {
                            indice = indpoint[icplus, 0];
                        }
                        Indice.SetValue(n3d, indice);
                        int iarc1 = dStr.AddShape(ShapeCasts.Edge(Ecom.Value(nb)));
                        interfp1 = ChFi3d_FilPointInDS(orvt, iarc1, indice, parcom.Value(nb));
                        dStr.ChangeShapeInterferences(iarc1).Append(interfp1);
                    }
                }
            }
        }

        // case when two free borders are tangent
        if (droit)
        {
            for (ic = 0; ic < nedge; ic++)
            {
                Curve3D? curve = null, ctrim = null; // OCCT's unused rcurve and rcurve2d left out
                Curve2D? curve2d = null, ctrim2d = null;
                double ufirst = 0.0, ulast = 0.0;
                Indices(nedge, ic, ref icplus, ref icmoins);
                int indpoint1 = 0, indpoint2 = 0;
                bool isvt1 = false, isvt2 = false;
                Edge ecur = ShapeCasts.Edge(evive.Value(ic));
                if (ecur.IsSame(edgelibre1) || ecur.IsSame(edgelibre2))
                {
                    n3d++;
                    curve2d = Tool.CurveOnSurface(ShapeCasts.Edge(evive.Value(ic)), ShapeCasts.Face(fvive.Value(ic, icplus)),
                                                                                            out ufirst, out ulast);
                    curve = Tool.Curve(ShapeCasts.Edge(evive.Value(ic)), out ufirst, out ulast);
                    if (Exploration.FirstVertex((ShapeCasts.Edge(evive.Value(ic)))).IsSame(v1))
                    {
                        ctrim = new TrimmedCurve3D(curve, ufirst, p.Value(ic, icmoins));
                        ctrim2d = new TrimmedCurve2D(curve2d, ufirst, p.Value(ic, icmoins));
                        indpoint1 = dStr.AddShape(v1);
                        isvt1 = true;
                        indpoint2 = indpoint[ic, 1];
                    }
                    else
                    {
                        ctrim = new TrimmedCurve3D(curve, p.Value(ic, icmoins), ulast);
                        ctrim2d = new TrimmedCurve2D(curve2d, p.Value(ic, icmoins), ulast);
                        indpoint2 = dStr.AddShape(v1);
                        isvt2 = true;
                        indpoint1 = indpoint[ic, 1];
                    }
                    if (libre.Value(ic))
                    {
                        if (Exploration.FirstVertex((ShapeCasts.Edge(evive.Value(ic)))).IsSame(v1))
                        {
                            ctrim.Reverse();
                            ctrim2d.Reverse();
                            indpoint2 = dStr.AddShape(v1);
                            isvt2 = true;
                            isvt1 = false;
                            indpoint1 = indpoint[ic, 1];
                        }
                    }
                    else
                    {
                        if (Exploration.LastVertex((ShapeCasts.Edge(evive.Value(ic)))).IsSame(v1))
                        {
                            ctrim.Reverse();
                            ctrim2d.Reverse();
                            indpoint1 = dStr.AddShape(v1);
                            isvt1 = true;
                            isvt2 = false;
                            indpoint2 = indpoint[ic, 1];
                        }
                    }
                    ufirst = ctrim.FirstParameter;
                    ulast = ctrim.LastParameter;
                    SurfaceAdaptor? asurf = null;
                    asurf = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(fvive.Value(ic, icplus))));
                    CurveAdaptor2D? acurv = new CurveAdaptor2D(ctrim2d);
                    CurveOnSurface curvOnS = new(acurv, asurf);
                    CurveOnSurface? hCons = (CurveOnSurface)curvOnS.ShallowCopy(); // OCCT copy-constructs a handle
                    order.SetValue(n3d, 0);
                    CurveConstraint? cont = 
                        new CurveConstraint(hCons, order.Value(n3d), 10, _tolapp3d, _angular, 0.1);
                    pSurf.Add(cont);
                    DsCurve tcurv3d = new(ctrim, 1.0e-4);
                    indcurve3d.SetValue(n3d, dStr.AddCurve(tcurv3d));
                    interfp1 =
                        ChFi3d_FilPointInDS(Forward, indcurve3d.Value(n3d), indpoint1, ufirst, isvt1);
                    interfp2 =
                        ChFi3d_FilPointInDS(Reversed, indcurve3d.Value(n3d), indpoint2, ulast, isvt2);
                    dStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(interfp1);
                    dStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(interfp2);
                }
            }
        }


        pSurf.Perform();


        // call of approx

        if (pSurf.IsDone)
        {
            int nbcarreau = 9;
            int degmax = 8;
            double seuil = 0.0;
            PlateSurface? gpPlate = pSurf.Surface();

            NCollection_Sequence<Xy> s2d = new();
            NCollection_Sequence<Xyz> s3d = new();
            s2d.Clear();
            s3d.Clear();
            pSurf.Disc2dContour(4, s2d);
            pSurf.Disc3dContour(4, 0, s3d);
            seuil = Max(tolapp, 10 * pSurf.G0Error());
            PlateG0Criterion critere = new(s2d, s3d, seuil);
            MakeApprox mapp = new(gpPlate, critere, tolapp, nbcarreau, degmax);
            Surface? surf = mapp.Surface();
            double coef = 1.1, apperror;
            apperror = mapp.CriterionError() * coef;


            //  Storage of the surface plate and corresponding curves in the DS

            Orientation orplate = default, orsurfdata = default, orpcurve = default, orien = default;
            //     gp_Vec n1,n2,du,dv,du1,dv1;
            //     gp_Pnt pp,pp1;
            //     double tpar;
            //     gp_Pnt2d uv;
            //     double scal;

            DsSurface tsurf = new(surf, mapp.ApproxError());
            int isurf = dStr.AddSurface(tsurf);
            // lbo : historique QDF.
            if (!_eviMap.ContainsKey(v1))
            {
                NCollection_List<int> li = new();
                _eviMap[v1] = li;
            }
            _eviMap.GetValue(v1).Append(isurf);

            int solInd = cd.Value(0).SolidIndex();
            NCollection_List<DsInterference> solidInterfs = 
                dStr.ChangeShapeInterferences(solInd);

            // in case when one rereads at top, it is necessary that
            // alive edges that arrive at the top should be removed from the DS.
            // For this they are stored in the DS with their inverted orientation
            int nbedge = 0;
            Explorer ex = new();
            if (deuxconges)
            {
                for (ic = 0; ic < nedge; ic++)
                {
                    if (!sharp.Value(ic))
                    {
                        nbedge = cd.Value(ic).Spine().EdgeCount;
                        Edge arcspine = new();
                        if (sens.Value(ic) == 1)
                        {
                            arcspine = cd.Value(ic).Spine().Edges(1);
                        }
                        else
                        {
                            arcspine = cd.Value(ic).Spine().Edges(nbedge);
                        }
                        int iArcspine = dStr.AddShape(arcspine);
                        Orientation oVtx = Forward;
                        for (ex.Init(arcspine.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
                        {
                            if (v1.IsSame(ex.Current()))
                            {
                                oVtx = ex.Current().Orientation;
                                break;
                            }
                        }
                        oVtx = global::Kerf.Engine.Topology.TopAbs.Reverse(oVtx);
                        double parVtx = Tool.Parameter(v1, arcspine);
                        DsCurvePointInterference? interfv = 
                            ChFi3d_FilVertexInDS(oVtx, iArcspine, iVtx, parVtx);
                        dStr.ChangeShapeInterferences(iArcspine).Append(interfv);
                    }
                }
            }

            // calculate orientation of Plate orplate corresponding to surfdata
            // calculation corresponding to the first stripe
            Indices(nedge, 0, ref icplus, ref icmoins);
            isfirst = (sens.Value(0) == 1);
            SurfData? fd = 
                cd.Value(0).SetOfSurfData().Value(i.Value(0, icmoins));
            indice = fd.Surf();
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
            orsurfdata = fd.Orientation();
            //     if (scal>0) orplate=orsurfdata;
            //     else  orplate=TopAbs::Reverse(orsurfdata);
            orplate = PlateOrientation(surf, pSurf.Curves2d(), sumFaceNormalAtV1);

            //  creation of solidinterderence for Plate
            DsSolidSurfaceInterference? ssi = 
                new DsSolidSurfaceInterference(new DsTransition(orplate), DsKind.Solid, solInd,
                                                                                                    DsKind.Surface, isurf);
            solidInterfs.Append(ssi);

            // calculate orientation orien of pcurves of Plate
            // the curves from ic to icplus the pcurves of Plate
            // all have the same orientation
            int ishape1 = 0, ishape2 = 0;
            Orientation trafil1 = Forward, trafil2 = Forward;
            ishape1 = fd.IndexOfS1();
            ishape2 = fd.IndexOfS2();
            FaceInterference fi1 = fd.InterferenceOnS1();
            FaceInterference fi2 = fd.InterferenceOnS2();
            if (ishape1 != 0)
            {
                if (ishape1 > 0)
                {
                    trafil1 = dStr.Shape(ishape1).Orientation;
                }
                trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(trafil1, fd.Orientation());
                trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(fi1.Transition()), trafil1);
                trafil2 = global::Kerf.Engine.Topology.TopAbs.Reverse(trafil1);
            }
            else
            {
                if (ishape2 > 0)
                {
                    trafil2 = dStr.Shape(ishape2).Orientation;
                }
                trafil2 = global::Kerf.Engine.Topology.TopAbs.Compose(trafil2, fd.Orientation());
                trafil2 = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(fi2.Transition()), trafil2);
                trafil1 = global::Kerf.Engine.Topology.TopAbs.Reverse(trafil2);
            }
            if (isfirst)
            {
                orpcurve = global::Kerf.Engine.Topology.TopAbs.Reverse(trafil1);
                orpcurve = global::Kerf.Engine.Topology.TopAbs.Compose(orpcurve, cd.Value(0).FirstPCurveOrientation());
            }
            else
            {
                orpcurve = trafil1;
                orpcurve = global::Kerf.Engine.Topology.TopAbs.Compose(orpcurve, cd.Value(0).LastPCurveOrientation());
            }
            if (orsurfdata == orplate)
            {
                orien = global::Kerf.Engine.Topology.TopAbs.Reverse(orpcurve);
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
                        Point2D uv1 = new(), uv2 = new();
                        Curve3D? c3d = null;
                        Curve2D? c2d = null; // OCCT's unused curv2d left out
                        Point3D ptic = new(), pticplus = new();
                        EdgeCurveAdaptor bCurv1 = new(ShapeCasts.Edge(evive.Value(ic)));
                        EdgeCurveAdaptor bCurv2 = new(ShapeCasts.Edge(evive.Value(icplus)));
                        double par1 = p.Value(ic, icplus);
                        double par2 = p.Value(icplus, ic);
                        bCurv1.D0(par1, out ptic);
                        bCurv2.D0(par2, out pticplus);
                        ParametrePlate(n3d, pSurf, surf, ptic, apperror, ref uv1);
                        ParametrePlate(n3d, pSurf, surf, pticplus, apperror, ref uv2);
                        double to3d = 1.0e-3, to2d = 1.0e-6, tolreached;
                        CommonPoint CP1 = new(), CP2 = new();
                        CP1.SetArc(1.0e-3, ShapeCasts.Edge(evive.Value(ic)), par1, Forward);
                        CP1.SetPoint(ptic);
                        CP2.SetArc(1.0e-3, ShapeCasts.Edge(evive.Value(icplus)), par2, Forward);
                        CP2.SetPoint(pticplus);
                        double param1 = 0.0, param2 = 0.0;
                        ChFi3d_ComputeArete(CP1, uv1, CP2, uv2, surf, ref c3d, ref c2d, ref param1, ref param2, to3d, to2d,
                                                                out tolreached, 0);
                        DsCurve tcurv3d = new(c3d, tolreached);
                        int ind1 = 0, ind2 = 0;
                        ind1 = indpoint[ic, 0];
                        ind2 = indpoint[icplus, 0];
                        int indcurv = dStr.AddCurve(tcurv3d);
                        interfp1 = ChFi3d_FilPointInDS(Forward, indcurv, ind1, param1);
                        interfp2 = ChFi3d_FilPointInDS(Reversed, indcurv, ind2, param2);
                        dStr.ChangeCurveInterferences(indcurv).Append(interfp1);
                        dStr.ChangeCurveInterferences(indcurv).Append(interfp2);
                        interfc = ChFi3d_FilCurveInDS(indcurv, isurf, c2d, orien);
                        dStr.ChangeSurfaceInterferences(isurf).Append(interfc);
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

                    apperror = mapp.CriterionError() * coef;
                    pardeb = cd.Value(ic).PCurve(isfirst).FirstParameter;
                    parfin = cd.Value(ic).PCurve(isfirst).LastParameter;

                    interfp1 =
                        ChFi3d_FilPointInDS(Forward, indcurve3d.Value(n3d), indpoint.Value(ic, 0), pardeb);
                    interfp2 = ChFi3d_FilPointInDS(Reversed, indcurve3d.Value(n3d), indpoint.Value(ic, 1), parfin);
                    dStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(interfp1);
                    dStr.ChangeCurveInterferences(indcurve3d.Value(n3d)).Append(interfp2);
                    ref DsCurve tcourb = ref dStr.ChangeCurve(indcurve3d.Value(n3d));

                    tcourb.Tolerance(errapp.Value(ic) + apperror);
                    ref DsPoint tpt1 = ref dStr.ChangePoint(indpoint[ic, 0]);
                    ref DsPoint tpt2 = ref dStr.ChangePoint(indpoint[ic, 1]);
                    tpt1.Tolerance(tpt1.Tolerance() + apperror);
                    tpt2.Tolerance(tpt2.Tolerance() + apperror);

                    // calculate surfaceinterference
                    interfc =
                        ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), isurf, pSurf.Curves2d().Value(n3d), orien);
                    dStr.ChangeSurfaceInterferences(isurf).Append(interfc);
                    regular.SetCurve(indcurve3d.Value(n3d));
                    regular.SetS1(isurf, false);
                    indice = cd.Value(ic).SetOfSurfData().Value(i.Value(ic, icmoins)).Surf();
                    regular.SetS2(indice, false);
                    _regul.Append(regular);
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
                        Orientation ori = default;
                        Point3D pf = new(), pl = new(), sommet1 = new();
                        Vertex vd = Exploration.FirstVertex(ShapeCasts.Edge(evive.Value(ic)));
                        Vertex vf = Exploration.LastVertex(ShapeCasts.Edge(evive.Value(ic)));
                        pf = Tool.Pnt(vd);
                        pl = Tool.Pnt(vf);
                        sommet1 = Tool.Pnt(v1);
                        if (pf.DistanceTo(sommet1) < pl.DistanceTo(sommet1))
                        {
                            ori = Forward;
                        }
                        else
                        {
                            ori = Reversed;
                        }
                        int iarc1 = dStr.AddShape(ShapeCasts.Edge(evive.Value(ic)));
                        interfp1 = ChFi3d_FilPointInDS(ori, iarc1, indpoint[ic, 1], p.Value(ic, icplus));
                        dStr.ChangeShapeInterferences(ShapeCasts.Edge(evive.Value(ic))).Append(interfp1);
                    }

                    if (!ponctuel.Value(ic) && !libre.Value(ic))
                    {
                        // actual connection
                        if (!moresurf.Value(ic))
                        {
                            n3d++;
                            ref DsCurve tcourb1 = ref dStr.ChangeCurve(indcurve3d.Value(n3d));
                            tcourb1.Tolerance(tcourb1.Tolerance() + apperror);
                            if (!deuxconges)
                            {
                                ref DsPoint tpt11 = ref dStr.ChangePoint(indpoint[ic, 1]);
                                ref DsPoint tpt21 = ref dStr.ChangePoint(indpoint[icplus, 0]);
                                tpt11.Tolerance(tpt11.Tolerance() + apperror);
                                tpt21.Tolerance(tpt21.Tolerance() + apperror);
                            }
                            interfc = ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), isurf, pSurf.Curves2d().Value(n3d), orien);
                            dStr.ChangeSurfaceInterferences(isurf).Append(interfc);
                            if (order.Value(n3d) == 1)
                            {
                                regular.SetCurve(indcurve3d.Value(n3d));
                                regular.SetS1(isurf, false);
                                regular.SetS2(numfa.Value(ic, icplus));
                                _regul.Append(regular);
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
                        if (!eproj.Value(nb).IsNull)
                        {
                            n3d++;
                            ref DsCurve tcourb1 = ref dStr.ChangeCurve(indcurve3d.Value(n3d));
                            tcourb1.Tolerance(tcourb1.Tolerance() + apperror);
                            if (Indice.Value(n3d) != 0)
                            {
                                ref DsPoint tpt11 = ref dStr.ChangePoint(Indice.Value(n3d));
                                tpt11.Tolerance(tpt11.Tolerance() + apperror);
                            }
                            interfc = ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), isurf, pSurf.Curves2d().Value(n3d), orien);
                            dStr.ChangeSurfaceInterferences(isurf).Append(interfc);
                            if (order.Value(n3d) == 1)
                            {
                                regular.SetCurve(indcurve3d.Value(n3d));
                                regular.SetS1(isurf, false);
                                regular.SetS2(dStr.AddShape(ShapeCasts.Face(fproj.Value(nb))));
                                _regul.Append(regular);
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
                    Edge ecom = new();
                    ecom = ShapeCasts.Edge(evive.Value(ic));
                    if (ecom.IsSame(edgelibre1) || ecom.IsSame(edgelibre2))
                    {
                        n3d++;
                        ref DsCurve tcourb1 = ref dStr.ChangeCurve(indcurve3d.Value(n3d));
                        tcourb1.Tolerance(tcourb1.Tolerance() + apperror);
                        interfc =
                            ChFi3d_FilCurveInDS(indcurve3d.Value(n3d), isurf, pSurf.Curves2d().Value(n3d), orien);
                        dStr.ChangeSurfaceInterferences(isurf).Append(interfc);
                    }
                }
            }
        }
        else
        { // there is only one partial result
            _done = false;
            _hasresult = true;
            for (ic = 0; ic < nedge; ic++)
            {
                Indices(nedge, ic, ref icplus, ref icmoins);
                if (!oksea.Value(ic))
                {
                    if (sharp.Value(ic) && !deuxconges)
                    {
                        // limitation of the alive edge
                        Orientation ori = default;
                        Point3D pf = new(), pl = new(), sommet1 = new();
                        Vertex vd = Exploration.FirstVertex(ShapeCasts.Edge(evive.Value(ic)));
                        Vertex vf = Exploration.LastVertex(ShapeCasts.Edge(evive.Value(ic)));
                        pf = Tool.Pnt(vd);
                        pl = Tool.Pnt(vf);
                        sommet1 = Tool.Pnt(v1);
                        if (pf.DistanceTo(sommet1) < pl.DistanceTo(sommet1))
                        {
                            ori = Forward;
                        }
                        else
                        {
                            ori = Reversed;
                        }
                        int iarc1 = dStr.AddShape(ShapeCasts.Edge(evive.Value(ic)));
                        interfp1 = ChFi3d_FilPointInDS(ori, iarc1, indpoint[ic, 1], p.Value(ic, icplus));
                        dStr.ChangeShapeInterferences(ShapeCasts.Edge(evive.Value(ic))).Append(interfp1);
                    }
                }
            }
        }
    }

}

internal static class Builder_CnCrn
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

    internal static void Calcul_Param(Stripe? stripe, int jfposit, int indice, bool isfirst, ref double param)
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

    internal static void Calcul_P2dOnSurf(Stripe? stripe, int jfposit, int indice, double param, ref Point2D p2)
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

    internal static void Calcul_C2dOnFace(Stripe? stripe, int jfposit, int indice, ref Curve2D? c2d)
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

    internal static void Calcul_Orientation(Stripe? stripe, int jfposit, int indice, ref Orientation orient)
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

    internal static void cherche_edge1(in Face f1, in Face f2, ref Edge edge)
    {
        int i = 0, j = 0;
        Edge ecur1 = new(), ecur2 = new();
        bool trouve = false;
        IndexedSet<Shape> mapE1 = new(ShapeHasher.Instance),
                                             mapE2 = new(ShapeHasher.Instance);

        Exploration.MapShapes(f1, global::Kerf.Topology.ShapeType.Edge, mapE1);
        Exploration.MapShapes(f2, global::Kerf.Topology.ShapeType.Edge, mapE2);
        for (i = 1; i <= mapE1.Count && !trouve; i++)
        {
            Shape localShape = mapE1[i - 1];
            ecur1 = ShapeCasts.Edge(localShape);
            //  Ecur1=TopoDS::Edge(TopoDS_Shape (MapE1[i]));
            for (j = 1; j <= mapE2.Count && !trouve; j++)
            {
                localShape = mapE2[j - 1];
                ecur2 = ShapeCasts.Edge(localShape);
                //              Ecur2=TopoDS::Edge(TopoDS_Shape (MapE2[j]));
                if (ecur2.IsSame(ecur1))
                {
                    edge = ecur1;
                    trouve = true;
                }
            }
        }
        if (edge.IsNull)
        {
            throw new ConstructionErrorException("Failed to find edge");
        }
    }

    internal static void CurveHermite(DsDataStructure dStr, Stripe? cDicmoins, int jficmoins, int icmoins, double picmoins, int sensicmoins, bool sharpicmoins, in Edge eviveicmoins, Stripe? cDicplus, int jficplus, int icplus, double picplus, int sensicplus, bool sharpicplus, in Edge eviveicplus, int nbface, NCollection_Sequence<Shape> ecom, NCollection_Sequence<Shape> face, NCollection_Sequence<Curve2D> proj2d, NCollection_Sequence<Curve3D> cproj, NCollection_Sequence<Shape> eproj, NCollection_Sequence<double> param, ref double error)
    {
        Point3D p01 = new(), p02 = new();
        Vector3D d11 = new(), d12 = new();
        int ii = 0, jj = 0;
        double up1 = 0.0, up2 = 0.0;
        int ilin = 0, jfp = 0;
        Curve3D? c1 = null, c2 = null;
        if (sharpicmoins)
        {
            c1 = Tool.Curve(eviveicmoins, out up1, out up2);
        }
        else
        {
            if (jficmoins == 1)
            {
                ilin = cDicmoins.SetOfSurfData().Value(icmoins).InterferenceOnS1().LineIndex();
            }
            else
            {
                ilin = cDicmoins.SetOfSurfData().Value(icmoins).InterferenceOnS2().LineIndex();
            }
            c1 = dStr.Curve(ilin).Curve();
        }
        if (sharpicplus)
        {
            c2 = Tool.Curve(eviveicplus, out up1, out up2);
        }
        else
        {
            jfp = 3 - jficplus;
            if (jfp == 1)
            {
                ilin = cDicplus.SetOfSurfData().Value(icplus).InterferenceOnS1().LineIndex();
            }
            else
            {
                ilin = cDicplus.SetOfSurfData().Value(icplus).InterferenceOnS2().LineIndex();
            }
            c2 = dStr.Curve(ilin).Curve();
        }
        if (c1 is null)
        {
            throw new ConstructionErrorException("Failed to get 3D curve of edge");
        }
        if (c2 is null)
        {
            throw new ConstructionErrorException("Failed to get 3D curve of edge");
        }
        c1.D1(picmoins, out p01, out d11);
        c2.D1(picplus, out p02, out d12);
        int size = 4;
        Matrix matCoefs = new(1, size, 1, size);
        NCollection_Array1<Xyz> cont = new(1, size);
        global::Kerf.Engine.Numerics.Polynomials.PLib.HermiteCoefficients(0, 1, 1, 1, matCoefs);
        double l1 = p01.DistanceTo(p02);
        double lambda = ((double)1) / Max(d11.Magnitude / l1, 1.0e-6);
        cont[1] = p01.XYZ();
        if (sensicmoins == 1)
        {
            cont[2] = d11.XYZ() * (-lambda);
        }
        else
        {
            cont[2] = d11.XYZ() * (lambda);
        }
        lambda = ((double)1) / Max(d12.Magnitude / l1, 1.0e-6);
        cont[3] = p02.XYZ();
        if (sensicplus == 1)
        {
            cont[4] = d12.XYZ() * (lambda);
        }
        else
        {
            cont[4] = d12.XYZ() * (-lambda);
        }
        NCollection_Array1<Point3D> extrapPoles = new(1, size);
        NCollection_Array1<Point3D> extraCoeffs = new(1, size);
        Point3D p0 = new(0, 0, 0);
        extraCoeffs.Init(p0);
        for (ii = 1; ii <= size; ii++)
        {
            for (jj = 1; jj <= size; jj++)
            {
                extraCoeffs[jj] = new Point3D(extraCoeffs[jj].XYZ() + (matCoefs[ii, jj] * cont[ii]));
            }
        }
        global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(extraCoeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), extrapPoles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
        BezierCurve3D? bezier = new BezierCurve3D(extrapPoles);
        LibraryMakeEdge bedge = new(bezier);
        Edge edg = bedge.Edge();
        Face f = new();
        error = 1.0e-30;
        int nb = 0;
        for (nb = 1; nb <= nbface; nb++)
        {
            f = ShapeCasts.Face(face.Value(nb));
            IndexedSet<Shape> mapE1 = new(ShapeHasher.Instance);
            Edge e1 = new();
            Curve2D? proj1 = null;
            Curve3D? proj1c = null; // OCCT's unused proj2c left out
            NormalProjection ortProj = new();
            ortProj.Init(f);
            ortProj.Add(edg);
            ortProj.SetParams(1.0e-4, 1.0e-4, C1, 14, 16);
            ortProj.Build();
            if (ortProj.IsDone)
            {
                Exploration.MapShapes(ortProj.Projection(), ShapeType.Edge, mapE1);
                if (mapE1.Count != 0)
                {
                    if (mapE1.Count != 1)
                    {
                        LibraryMakeFace bface = new(Tool.Surface(f), global::Kerf.Engine.Foundation.Precision.Confusion());
                        f = bface.Face();
                        ortProj.Init(f);
                        ortProj.Build();
                        mapE1.Clear();
                        if (ortProj.IsDone)
                        {
                            Exploration.MapShapes(ortProj.Projection(), ShapeType.Edge, mapE1);
                        }
                    }
                    if (mapE1.Count != 0)
                    {
                        bool trouve = false;
                        for (int ind = 1; ind <= mapE1.Count && !trouve; ind++)
                        {
                            Shape localShape = mapE1[ind - 1];
                            e1 = ShapeCasts.Edge(localShape);
                            //           E1=TopoDS::Edge( TopoDS_Shape (MapE1[ind]));
                            if (!Tool.Degenerated(e1))
                            {
                                trouve = true;
                            }
                        }
                        eproj.Append(e1);
                        proj1 = Tool.CurveOnSurface(e1, f, out up1, out up2);
                        if (proj1 is null)
                        {
                            throw new ConstructionErrorException("Failed to get p-curve of edge");
                        }
                        proj2d.Append(new TrimmedCurve2D(proj1, up1, up2));
                        proj1c = Tool.Curve(e1, out up1, out up2);
                        if (proj1c is null)
                        {
                            throw new ConstructionErrorException("Failed to get 3D curve of edge");
                        }
                        cproj.Append(new TrimmedCurve3D(proj1c, up1, up2));
                        if (error > Tool.Tolerance(e1))
                        {
                            error = Tool.Tolerance(e1);
                        }
                    }
                    else
                    {
                        eproj.Append(e1);
                        proj2d.Append(proj1);
                        cproj.Append(proj1c);
                    }
                }
                else
                {
                    eproj.Append(e1);
                    proj2d.Append(proj1);
                    cproj.Append(proj1c);
                }
            }
        }
        for (nb = 1; nb <= nbface - 1; nb++)
        {
            EdgeCurveAdaptor c = new(ShapeCasts.Edge(ecom.Value(nb)));
            c.D0(param.Value(nb), out p02);
            CurveAdaptor l = new(bezier);
            ExtCC ext = new(c, l);
            if (ext.IsDone)
            {
                if (!ext.IsParallel && ext.ExtCount != 0)
                {
                    POnCurv pOnC = new(), pOnL = new();
                    ext.Points(1, out pOnC, out pOnL);
                    if (pOnC.Value().DistanceTo(pOnL.Value()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        param.ChangeValue(nb) = pOnC.Parameter();
                    }
                    else
                    {
                        if (cproj.Value(nb) is not null)
                        {
                            cproj.Value(nb).D0(cproj.Value(nb).LastParameter, out p01);
                        }
                        else if (cproj.Value(nb + 1) is not null)
                        {
                            cproj.Value(nb + 1).D0(cproj.Value(nb + 1).FirstParameter, out p01);
                        }
                    }
                }
            }
            if (!ext.IsDone || ext.ExtCount == 0)
            {
                if (cproj.Value(nb) is not null)
                {
                    cproj.Value(nb).D0(cproj.Value(nb).LastParameter, out p01);
                }
                else if (cproj.Value(nb + 1) is not null)
                {
                    cproj.Value(nb + 1).D0(cproj.Value(nb + 1).FirstParameter, out p01);
                }
                if (p01.DistanceTo(p02) > 1.0e-4)
                {
                    CurvePointExtrema ext1 = new(p01, c);
                    if (ext1.IsDone)
                    {
                        if (ext1.ExtCount != 0)
                        {
                            POnCurv pOnC = ext1.Point(1);
                            param.ChangeValue(nb) = pOnC.Parameter();
                        }
                    }
                }
            }
        }
    }

    internal static void CalculDroite(in Point2D p2d1, double xdir, double ydir, ref Curve2D? pcurve)
    {
        Direction2D dir1 = new(xdir, ydir);
        LineCurve2D? l = new LineCurve2D(p2d1, dir1);
        double l0 = Math.Sqrt(xdir * xdir + ydir * ydir);
        pcurve = new TrimmedCurve2D(l, 0, l0);
    }

    internal static void CalculBatten(SurfaceAdaptor? aSurf, in Face face, double xdir, double ydir, in Point2D p2d1, in Point2D p2d2, bool contraint1, bool contraint2, Curve2D? curv2d1, Curve2D? curv2d2, double picicplus, double picplusic, bool inverseic, bool inverseicplus, ref Curve2D? pcurve)
    {
        bool isplane = false;
        bool anglebig = false;
        isplane = aSurf.GetType() == Plane;
        Direction2D dir1 = new(xdir, ydir);
        CLProps2d cl1 = new(curv2d1, picicplus, 1, 1.0e-4);
        CLProps2d cl2 = new(curv2d2, picplusic, 1, 1.0e-4);
        Direction2D dir3 = new(), dir4 = new();
        cl1.Tangent(out dir3);
        cl2.Tangent(out dir4);
        if (inverseic)
        {
            dir3 = dir3.Reversed();
        }
        if (inverseicplus)
        {
            dir4 = dir4.Reversed();
        }
        double h = p2d2.DistanceTo(p2d1) / 20;
        Batten bat = new(p2d1, p2d2, h);
        bat.SetFreeSliding(true);
        double ang1 = 0.0, ang2 = 0.0;
        ang1 = dir1.AngleTo(dir3);
        if (dir1.AngleTo(dir4) > 0)
        {
            ang2 = M_PI - dir1.AngleTo(dir4);
        }
        else
        {
            ang2 = -M_PI - dir1.AngleTo(dir4);
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
                bat.SetAngle1(ang1);
            }
            else
            {
                bat.SetConstraintOrder1(0);
            }
            if (contraint2)
            {
                bat.SetAngle2(ang2);
            }
            else
            {
                bat.SetConstraintOrder2(0);
            }
            AnalysisCode iana = default;
            bool ok = false;
            ok = bat.Compute(out iana, 25, 1.0e-2);
            if (ok)
            {
                pcurve = bat.Curve();
                double umin = 0.0, vmin = 0.0, umax = 0.0, vmax = 0.0;
                BrepTools.UVBounds(face, out umin, out umax, out vmin, out vmax);
                Box2d bf = new(), bc = new();
                CurveAdaptor2D acur = new(pcurve);
                Add2dCurve.Add(acur, 0, ref bc);
                bf.Update(umin, vmin, umax, vmax);
                double uminc = 0.0, vminc = 0.0, umaxc = 0.0, vmaxc = 0.0;
                bc.Get(out uminc, out vminc, out umaxc, out vmaxc);
                if (uminc < umin - 1.0e-7)
                {
                    ok = false;
                }
                if (umaxc > umax + 1.0e-7)
                {
                    ok = false;
                }
                if (vminc < vmin - 1.0e-7)
                {
                    ok = false;
                }
                if (vmaxc > vmax + 1.0e-7)
                {
                    ok = false;
                }
            }
            if (!ok)
            {
                CalculDroite(p2d1, xdir, ydir, ref pcurve);
            }
        }
    }

    internal static void OrientationIcNonVive(Stripe? cDic, int jfic, int icicplus, int sensic, ref Orientation orien)
    {
        Orientation orinterf = default;
        Calcul_Orientation(cDic, jfic, icicplus, ref orinterf);
        if (sensic != 1)
        {
            if (orinterf == Forward)
            {
                orien = Forward;
            }
            else
            {
                orien = Reversed;
            }
        }
        else
        {
            if (orinterf == Forward)
            {
                orien = Reversed;
            }
            else
            {
                orien = Forward;
            }
        }
    }

    internal static void OrientationIcplusNonVive(Stripe? cDicplus, int jficplus, int icplusic, int sensicplus, ref Orientation orien)
    {
        Orientation orinterf = default;
        int jfp = 3 - jficplus;
        Calcul_Orientation(cDicplus, jfp, icplusic, ref orinterf);
        if (sensicplus == 1)
        {
            if (orinterf == Forward)
            {
                orien = Forward;
            }
            else
            {
                orien = Reversed;
            }
        }
        else
        {
            if (orinterf == Forward)
            {
                orien = Reversed;
            }
            else
            {
                orien = Forward;
            }
        }
    }

    internal static void OrientationAreteViveConsecutive(in Shape fviveicicplus, in Shape eviveic, in Vertex v1, ref Orientation orien)
    { // orinterf is orientation of edge ic corresponding to face Fviveicicplus taken FORWARD
        Orientation orinterf = Forward;
        Face f = ShapeCasts.Face(fviveicicplus);
        Edge e = ShapeCasts.Edge(eviveic);
        Explorer ex = new();
        for (ex.Init(f.Oriented(Forward), ShapeType.Edge); ex.More(); ex.Next())
        {
            if (e.IsSame(ex.Current()))
            {
                orinterf = ex.Current().Orientation;
                break;
            }
        }
        // if V1 is vertex REVERSED of edge ic the curve
        // has the same orientation as ic
        Vertex vl = new();
        vl = Exploration.LastVertex(e);
        if (vl.IsSame(v1))
        {
            if (orinterf == Forward)
            {
                orien = Forward;
            }
            else
            {
                orien = Reversed;
            }
        }
        else
        {
            if (orinterf == Forward)
            {
                orien = Reversed;
            }
            else
            {
                orien = Forward;
            }
        }
    }

    internal static void PerformTwoCornerSameExt(DsDataStructure dStr, Stripe? stripe1, int index1, int sens1, Stripe? stripe2, int index2, int sens2, ref bool trouve)
    {
        DsCurvePointInterference? interfp1 = null, interfp2 = null;
        DsSurfaceCurveInterference? interfc = null;
        Orientation orpcurve = default, trafil1 = default, orsurf1 = default, orsurf2 = default;
        bool isfirst = false;
        int indic1 = 0, indic2 = 0, indpoint1 = 0, indpoint2 = 0, ind = 0, indcurve = 0;
        double tol = 0.0;
        Point3D p1 = new(), p2 = new(), p3 = new(), p4 = new();
        Point2D p2d = new();
        Curve3D? cint = null;
        Curve2D? c2dint1 = null, c2dint2 = null;
        isfirst = sens1 == 1;
        ref CommonPoint com11 = ref stripe1.SetOfSurfData().Value(index1).ChangeVertex(isfirst, 1);
        ref CommonPoint com12 = ref stripe1.SetOfSurfData().Value(index1).ChangeVertex(isfirst, 2);
        isfirst = sens2 == 1;
        ref CommonPoint com21 = ref stripe2.SetOfSurfData().Value(index2).ChangeVertex(isfirst, 1);
        indic1 = stripe1.SetOfSurfData().Value(index1).Surf();
        indic2 = stripe2.SetOfSurfData().Value(index2).Surf();
        SurfData? fd1 = stripe1.SetOfSurfData().Value(index1);
        SurfData? fd2 = stripe2.SetOfSurfData().Value(index2);

        NCollection_Array1<double> pardeb = new(1, 4), parfin = new(1, 4);
        FaceInterference fi11 = fd1.InterferenceOnS1();
        FaceInterference fi12 = fd1.InterferenceOnS2();
        FaceInterference fi21 = fd2.InterferenceOnS1();
        FaceInterference fi22 = fd2.InterferenceOnS2();
        Point2D pfi11 = new(), pfi12 = new(), pfi21 = new(), pfi22 = new();
        isfirst = sens1 == 1;
        pfi11 = fi11.PCurveOnSurf().Value(fi11.Parameter(isfirst));
        pfi12 = fi12.PCurveOnSurf().Value(fi12.Parameter(isfirst));
        isfirst = sens2 == 1;
        if (com11.Point().DistanceTo(com21.Point()) < 1.0e-4)
        {
            pfi21 = fi21.PCurveOnSurf().Value(fi21.Parameter(isfirst));
            pfi22 = fi22.PCurveOnSurf().Value(fi22.Parameter(isfirst));
        }
        else
        {
            pfi22 = fi21.PCurveOnSurf().Value(fi21.Parameter(isfirst));
            pfi21 = fi22.PCurveOnSurf().Value(fi22.Parameter(isfirst));
        }

        pardeb[1] = pfi11.X;
        pardeb[2] = pfi11.Y;
        pardeb[3] = pfi21.X;
        pardeb[4] = pfi21.Y;
        parfin[1] = pfi12.X;
        parfin[2] = pfi12.Y;
        parfin[3] = pfi22.X;
        parfin[4] = pfi22.Y;

        SurfaceAdaptor? hs1 = ChFi3d_BoundSurf(dStr, fd1, 1, 2);
        SurfaceAdaptor? hs2 = ChFi3d_BoundSurf(dStr, fd2, 1, 2);
        trouve = false;
        if (ChFi3d_ComputeCurves(hs1, hs2, pardeb, parfin, ref cint, ref c2dint1, ref c2dint2, 1.0e-4, 1.0e-5, out tol))
        {
            cint.D0(cint.FirstParameter, out p1);
            cint.D0(cint.LastParameter, out p2);
            trouve = ((com11.Point().DistanceTo(p1) < 1.0e-4 || com11.Point().DistanceTo(p2) < 1.0e-4)
                                && (com12.Point().DistanceTo(p1) < 1.0e-4 || com12.Point().DistanceTo(p2) < 1.0e-4));
        }

        if (trouve)
        {
            isfirst = sens1 == 1;
            stripe1.InDS(isfirst);
            indpoint1 = ChFi3d_IndexPointInDS(com11, dStr);
            indpoint2 = ChFi3d_IndexPointInDS(com12, dStr);
            stripe1.SetIndexPoint(indpoint1, isfirst, 1);
            stripe1.SetIndexPoint(indpoint2, isfirst, 2);
            isfirst = sens2 == 1;
            stripe2.InDS(isfirst);
            if (com11.Point().DistanceTo(com21.Point()) < 1.0e-4)
            {
                stripe2.SetIndexPoint(indpoint1, isfirst, 1);
                stripe2.SetIndexPoint(indpoint2, isfirst, 2);
            }
            else
            {
                stripe2.SetIndexPoint(indpoint2, isfirst, 1);
                stripe2.SetIndexPoint(indpoint1, isfirst, 2);
            }

            orsurf1 = fd1.Orientation();
            trafil1 = dStr.Shape(fd1.IndexOfS1()).Orientation;
            trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(trafil1, fd1.Orientation());
            trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(fi11.Transition()), trafil1);
            orsurf2 = fd2.Orientation();
            DsCurve tcurv3d = new(cint, tol);
            indcurve = dStr.AddCurve(tcurv3d);
            cint.D0(cint.FirstParameter, out p1);
            cint.D0(cint.LastParameter, out p2);
            fi11.PCurveOnFace().D0(fi11.LastParameter(), out p2d);
            Surface? stemp = 
                Tool.Surface(ShapeCasts.Face(dStr.Shape(fd1.IndexOfS1())));
            stemp.D0(p2d.X, p2d.Y, out p4);
            fi11.PCurveOnFace().D0(fi11.FirstParameter(), out p2d);
            stemp.D0(p2d.X, p2d.Y, out p3);
            if (p1.DistanceTo(p4) < 1.0e-4 || p2.DistanceTo(p3) < 1.0e-4)
            {
                orpcurve = trafil1;
            }
            else
            {
                orpcurve = global::Kerf.Engine.Topology.TopAbs.Reverse(trafil1);
            }
            if (com11.Point().DistanceTo(p1) > 1.0e-4)
            {
                ind = indpoint1;
                indpoint1 = indpoint2;
                indpoint2 = ind;
            }
            interfp1 = ChFi3d_FilPointInDS(Forward, indcurve, indpoint1, cint.FirstParameter);
            interfp2 = ChFi3d_FilPointInDS(Reversed, indcurve, indpoint2, cint.LastParameter);
            dStr.ChangeCurveInterferences(indcurve).Append(interfp1);
            dStr.ChangeCurveInterferences(indcurve).Append(interfp2);
            interfc = ChFi3d_FilCurveInDS(indcurve, indic1, c2dint1, orpcurve);
            dStr.ChangeSurfaceInterferences(indic1).Append(interfc);
            if (orsurf1 == orsurf2)
            {
                orpcurve = global::Kerf.Engine.Topology.TopAbs.Reverse(orpcurve);
            }
            interfc = ChFi3d_FilCurveInDS(indcurve, indic2, c2dint2, orpcurve);
            dStr.ChangeSurfaceInterferences(indic2).Append(interfc);
        }
    }

    internal static void CpOnEdge(Stripe? stripe, int num, bool isfirst, in Edge eadj1, in Edge eadj2, ref bool compoint)
    {
        CommonPoint cp1 = new(), cp2 = new();
        compoint = false;
        cp1 = stripe.SetOfSurfData().Value(num).ChangeVertex(isfirst, 1);
        cp2 = stripe.SetOfSurfData().Value(num).ChangeVertex(isfirst, 2);
        if (cp1.IsOnArc)
        {
            if (cp1.Arc().IsSame(eadj1) || cp1.Arc().IsSame(eadj2))
            {
                compoint = true;
            }
        }
        if (cp2.IsOnArc)
        {
            if (cp2.Arc().IsSame(eadj1) || cp2.Arc().IsSame(eadj2))
            {
                compoint = true;
            }
        }
    }

    internal static void RemoveSurfData(StripeMap myVDataMap, Map myEFMap, in Edge edgecouture, in Face facecouture, in Vertex v1)
    {
        NCollection_List<Stripe>.Iterator it = new();
        bool isfirst = false;
        Edge ecur = new(), eadj1 = new(), eadj2 = new();
        Face fg = new(), fd = new(), f1 = new(), f2 = new();
        Vertex vbid = new();
        int nbsurf = 0, nbedge = 0, sense = 0, num = 0;
        for (it.Initialize(myVDataMap[v1]); it.More(); it.Next())
        {
            nbsurf = it.Value().SetOfSurfData().Length();
            nbedge = it.Value().Spine().EdgeCount;
            if (nbsurf != 1)
            {
                num = ChFi3d_IndexOfSurfData(v1, it.Value(), out sense);
                if (sense == 1)
                {
                    ecur = it.Value().Spine().Edges(1);
                }
                else
                {
                    ecur = it.Value().Spine().Edges(nbedge);
                }
                ChFi3d_edge_common_faces(myEFMap[ecur], out f1, out f2);
                if (f1.IsSame(facecouture))
                {
                    eadj1 = edgecouture;
                }
                else
                {
                    ChFi3d_cherche_element(v1, ecur, f1, ref eadj1, ref vbid);
                }
                ChFi3d_edge_common_faces(myEFMap[eadj1], out fg, out fd);
                if (f2.IsSame(facecouture))
                {
                    eadj2 = edgecouture;
                }
                else
                {
                    ChFi3d_cherche_element(v1, ecur, f2, ref eadj2, ref vbid);
                }
                ChFi3d_edge_common_faces(myEFMap[eadj2], out fg, out fd);
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
                        CpOnEdge(it.Value(), i, isfirst, eadj1, eadj2, ref compoint);
                        if (compoint)
                        {
                            ind = i;
                        }
                    }
                    if (ind >= 2)
                    {
                        RemoveSD(it.ChangeValue(), 1, ind - 1);
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
                        CpOnEdge(it.Value(), i, isfirst, eadj1, eadj2, ref compoint);
                        if (compoint)
                        {
                            ind = i;
                        }
                    }
                    if (ind < num)
                    {
                        RemoveSD(it.ChangeValue(), ind + 1, num);
                    }
                }
            }
        }
    }

    internal static void ParametrePlate(int n3d, BuildPlateSurface pSurf, Surface? surf, in Point3D point, double apperror, ref Point2D uv)
    {
        int ip = 0;
        Point3D p1 = new();
        double par = 0.0;
        bool trouve = false;
        for (ip = 1; ip <= n3d && !trouve; ip++)
        {
            par = pSurf.Curves2d().Value(ip).FirstParameter;
            pSurf.Curves2d().Value(ip).D0(par, out uv);
            surf.D0(uv.X, uv.Y, out p1);
            trouve = p1.IsEqual(point, apperror);
            if (!trouve)
            {
                par = pSurf.Curves2d().Value(ip).LastParameter;
                pSurf.Curves2d().Value(ip).D0(par, out uv);
                surf.D0(uv.X, uv.Y, out p1);
                trouve = p1.IsEqual(point, apperror);
            }
        }
    }

    internal static void SummarizeNormal(in Vertex v1, in Face fcur, in Edge ecur, ref Vector3D sumFaceNormalAtV1)
    {
        Point2D uv1 = new(), uv2 = new();
        Tool.UVPoints(ecur, fcur, out uv1, out uv2);
        if (!v1.IsSame(Exploration.FirstVertex(ecur)))
        {
            uv1 = uv2;
        }

        Point3D p = new();
        Vector3D d1U = new(), d1V = new();
        Tool.Surface(fcur).D1(uv1.X, uv1.Y, out p, out d1U, out d1V);
        Vector3D n = d1U.Crossed(d1V);
        if (fcur.Orientation == Reversed)
        {
            n = n.Reversed();
        }

        if (n.SquareMagnitude <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return;
        }

        sumFaceNormalAtV1 += n.Normalized();
        sumFaceNormalAtV1 = sumFaceNormalAtV1.Normalized();
    }

    internal enum ChFi3d_SurfType
    {
        ChFiSURFACE,
        FACE1,
        FACE2
    } // for call SurfIndex(...)

    internal static int SurfIndex(NCollection_Array1<Stripe> stripeArray1, int stripeIndex, int surfDataIndex, ChFi3d_SurfType surfType)
    {
        SurfData? surfData = 
            stripeArray1[stripeIndex].SetOfSurfData().Value(surfDataIndex);
        switch (surfType)
        {
            case ChFiSURFACE:
                return surfData.Surf();
            case FACE1:
                return surfData.IndexOfS1();
            case FACE2:
                return surfData.IndexOfS2();
            default:
                return -1;
        }
    }

    internal static Orientation PlateOrientation(Surface? plateSurf, NCollection_HArray1<Curve2D>? pcArr, in Vector3D refDir)
    {
        Vector3D du = new(), dv = new();
        Point3D pp1 = new(), pp2 = new(), pp3 = new();
        Point2D uv = new();
        double fpar = 0.0, lpar = 0.0;
        double sumScal1 = 0, sumScal2 = 0;

        int i = 0, nb = pcArr.Upper();
        Curve2D? pc = pcArr.Value(nb);
        fpar = pc.FirstParameter;
        lpar = pc.LastParameter;
        pc.D0((fpar + lpar) / 2.0, out uv);
        plateSurf.D0(uv.X, uv.Y, out pp1);
        pc.D0(lpar, out uv);
        plateSurf.D0(uv.X, uv.Y, out pp2);

        for (i = 1; i <= nb; i++)
        {
            pc = pcArr.Value(i);
            fpar = pc.FirstParameter;
            lpar = pc.LastParameter;
            pc.D0(fpar, out uv);
            plateSurf.D1(uv.X, uv.Y, out pp2, out du, out dv);
            Vector3D n1 = du ^ dv;
            n1 = n1.Normalized();

            pc.D0((fpar + lpar) / 2.0, out uv);
            plateSurf.D0(uv.X, uv.Y, out pp3);

            Vector3D vv1 = new(pp2, pp1), vv2 = new(pp2, pp3);

            Vector3D n2 = vv2 ^ vv1;
            n2 = n2.Normalized();

            sumScal1 += n1 * n2;
            sumScal2 += n2 * refDir;

            pp1 = pp3;
        }
        if (sumScal2 * sumScal1 > 0)
        {
            return Forward;
        }
        else
        {
            return Reversed;
        }
    }


}
