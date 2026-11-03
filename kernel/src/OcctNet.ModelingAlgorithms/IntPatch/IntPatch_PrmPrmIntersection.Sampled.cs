// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// IntPatch_PrmPrmIntersection.cxx: the Perform on the domains' samplings (IntPolyh_Intersection or the voxel search),
// PointDepart and its file-local class IntPatch_InfoPD (see IntPatch_PrmPrmIntersection.cs).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

public partial class IntPatch_PrmPrmIntersection
{
    //=================================================================================================

    /// <summary>
    /// The intersection of <paramref name="Surf1"/> and <paramref name="Surf2"/> from the samplings of their domains
    /// (<c>SamplePnts</c> with <paramref name="Deflection"/>): with at most 2,500 samples on each surface, walking
    /// lines started from the section lines and tangent zones of IntPolyh_Intersection, otherwise from the voxel search
    /// <see cref="PointDepart"/>. With <paramref name="ClearFlag"/> false, the lines of an earlier Perform are kept (the
    /// voxel search keeps them in any case, as in OCCT).
    /// </summary>
    public void Perform(Adaptor3d_Surface Surf1, Adaptor3d_TopolTool D1, Adaptor3d_Surface Surf2,
                        Adaptor3d_TopolTool D2, double TolTangency, double Epsilon, double Deflection,
                        double Increment, bool ClearFlag = true)
    {
        int Limit = 2500;
        int NbU1 = 10, NbV1 = 10, NbU2 = 10, NbV2 = 10;
        //
        D1.SamplePnts(Deflection, NbU1, NbV1);
        D2.SamplePnts(Deflection, NbU2, NbV2);
        //
        NbU1 = D1.NbSamplesU();
        NbV1 = D1.NbSamplesV();
        NbU2 = D2.NbSamplesU();
        NbV2 = D2.NbSamplesV();

        NCollection_Array1<double> anUpars1 = new(1, NbU1), aVpars1 = new(1, NbV1);
        NCollection_Array1<double> anUpars2 = new(1, NbU2), aVpars2 = new(1, NbV2);
        //
        D1.UParameters(anUpars1);
        D1.VParameters(aVpars1);
        D2.UParameters(anUpars2);
        D2.VParameters(aVpars2);

        double[] Periods = new double[4];
        Periods[0] = (Surf1.IsUPeriodic()) ? Surf1.UPeriod() : 0.0;
        Periods[1] = (Surf1.IsVPeriodic()) ? Surf1.VPeriod() : 0.0;
        Periods[2] = (Surf2.IsUPeriodic()) ? Surf2.UPeriod() : 0.0;
        Periods[3] = (Surf2.IsVPeriodic()) ? Surf2.VPeriod() : 0.0;

        //---------------------------------------------
        if ((NbU1 * NbV1 <= Limit && NbV2 * NbU2 <= Limit))
        {
            empt = true;
            if (ClearFlag)
            {
                SLin.Clear();
            }
            //
            IntPolyh_Intersection pInterference;

            if (D1.IsUniformSampling() || D2.IsUniformSampling())
            {
                pInterference = new IntPolyh_Intersection(Surf1, NbU1, NbV1, Surf2, NbU2, NbV2);
            }
            else
            {
                pInterference = new IntPolyh_Intersection(Surf1, anUpars1, aVpars1, Surf2, anUpars2, aVpars2);
            }

            // OCCT tests the pointer it has just allocated against null here; `new` never gives null.
            //
            IntPolyh_Intersection Interference = pInterference;
            //
            done = Interference.IsDone();
            if (!done)
            {
                return;
            }

            int nbLigSec = Interference.NbSectionLines();
            int nbTanZon = Interference.NbTangentZones();
            double SeuildPointLigne = 15.0 * Increment * Increment;

            int NbLigCalculee = 0, ver;
            double incidence, dminiPointLigne;
            bool HasStartPoint = false, RejectLine = false;
            IntSurf_PntOn2S StartPOn2S = new();
            Point3D Point3dDebut, Point3dFin;

            NCollection_Array1<double> StartParams = new(1, 4);
            IntWalk_PWalking PW = new(Surf1, Surf2, TolTangency, Epsilon, Deflection, Increment);

            if (nbLigSec >= 1)
            {
                int[] TabL = new int[nbLigSec + 1];
                for (int ls = 1; ls <= nbLigSec; ++ls)
                {
                    TabL[ls] = ls;
                }
                //----------------------------------------1.1
                {
                    bool triok;
                    int nb_A, nb_B, tyu;
                    do
                    {
                        triok = true;
                        for (int b = 2; b <= nbLigSec; ++b)
                        {
                            nb_B = Interference.NbPointsInLine(TabL[b]);
                            nb_A = Interference.NbPointsInLine(TabL[b - 1]);

                            if (nb_B > nb_A)
                            {
                                tyu = TabL[b];
                                TabL[b] = TabL[b - 1];
                                TabL[b - 1] = tyu;
                                triok = false;
                            }
                        }
                    } while (!triok);
                }

                //----------------------------------------
                // 1.2 For the line "ls" get 2D-bounds U,V for surfaces 1,2
                //
                for (int ls = 1; ls <= nbLigSec; ++ls)
                {
                    int nbp = Interference.NbPointsInLine(TabL[ls]);
                    if (nbp == 0)
                    {
                        continue;
                    }
                    //
                    int[] TabPtDep = new int[nbp + 1];
                    for (int ilig = 1; ilig <= nbp; ++ilig)
                    {
                        TabPtDep[ilig] = 0;
                    }
                    //
                    double UminLig1, VminLig1, UmaxLig1, VmaxLig1;
                    double UminLig2, VminLig2, UmaxLig2, VmaxLig2;
                    double _x, _y, _z;
                    //
                    Interference.GetLinePoint(TabL[ls], 1, out _x, out _y, out _z, out UminLig1, out VminLig1,
                                              out UminLig2, out VminLig2, out incidence);

                    UmaxLig1 = UminLig1;
                    VmaxLig1 = VminLig1;
                    UmaxLig2 = UminLig2;
                    VmaxLig2 = VminLig2;
                    //
                    for (int ilig = 2; ilig <= nbp; ilig++)
                    {
                        Interference.GetLinePoint(TabL[ls], ilig, out _x, out _y, out _z, out double U1, out double V1,
                                                  out double U2, out double V2, out incidence);
                        //
                        if (U1 > UmaxLig1)
                        {
                            UmaxLig1 = U1;
                        }
                        if (V1 > VmaxLig1)
                        {
                            VmaxLig1 = V1;
                        }
                        if (U2 > UmaxLig2)
                        {
                            UmaxLig2 = U2;
                        }
                        if (V2 > VmaxLig2)
                        {
                            VmaxLig2 = V2;
                        }
                        //
                        if (U1 < UminLig1)
                        {
                            UminLig1 = U1;
                        }
                        if (V1 < VminLig1)
                        {
                            VminLig1 = V1;
                        }
                        if (U2 < UminLig2)
                        {
                            UminLig2 = U2;
                        }
                        if (V2 < VminLig2)
                        {
                            VminLig2 = V2;
                        }
                    } // for( ilig = 2; ilig <= nbp; ilig++ ) {
                    //
                    //----------------------------------------
                    // 1.3
                    int nbps2 = (nbp > 3) ? (nbp / 2) : 1;
                    int NombreDePointsDeDepartDuCheminement = 0;
                    bool lignetrouvee = false;
                    const int NbDePointsDeDepartDuChmLimit = 5;
                    //
                    do
                    {
                        NombreDePointsDeDepartDuCheminement++;
                        switch (NombreDePointsDeDepartDuCheminement)
                        {
                            case 1:
                                nbps2 = (nbp > 1) ? nbp / 2 : 1;
                                if (nbp < 3)
                                {
                                    NombreDePointsDeDepartDuCheminement = NbDePointsDeDepartDuChmLimit;
                                }

                                break;
                            case 2:
                                nbps2 = 1;
                                break;
                            case 3:
                                nbps2 = nbp - 1;
                                break;

                            case 4:
                                nbps2 = 3 * nbp / 4;
                                break;

                            case 5:
                                nbps2 = nbp / 4;
                                break;
                            default:
                                nbps2 = NombreDePointsDeDepartDuCheminement - 3;
                                NombreDePointsDeDepartDuCheminement++;
                                break;
                        }

                        //
                        if (TabPtDep[nbps2] == 0)
                        {
                            TabPtDep[nbps2] = 1;
                            Interference.GetLinePoint(TabL[ls], nbps2, out _x, out _y, out _z, out double U1,
                                                      out double V1, out double U2, out double V2, out incidence);

                            StartParams[1] = U1;
                            StartParams[2] = V1;
                            StartParams[3] = U2;
                            StartParams[4] = V2;

                            HasStartPoint = PW.PerformFirstPoint(StartParams, ref StartPOn2S);
                            dminiPointLigne = SeuildPointLigne + SeuildPointLigne;
                            if (HasStartPoint)
                            {
                                NbLigCalculee = SLin.Length();
                                int l;
                                for (l = 1; (l <= NbLigCalculee) && (dminiPointLigne >= SeuildPointLigne); l++)
                                {
                                    IntPatch_WLine testwline = (IntPatch_WLine)SLin.Value(l);

                                    if (IsPointOnLine(StartPOn2S, testwline, Deflection))
                                    {
                                        dminiPointLigne = 0.0;
                                    }
                                } // for( l ...

                                if (dminiPointLigne > SeuildPointLigne)
                                {
                                    PW.Perform(StartParams, UminLig1, VminLig1, UminLig2, VminLig2, UmaxLig1, VmaxLig1,
                                               UmaxLig2, VmaxLig2);

                                    //
                                    bool bPWIsDone;
                                    double aD11, aD12, aD21, aD22, aDx;
                                    //
                                    bPWIsDone = PW.IsDone();

                                    if (bPWIsDone)
                                    {
                                        bool hasBeenAdded = false;
                                        if (PW.NbPoints() > 2)
                                        {
                                            // Try to extend the intersection line to the boundary,
                                            // if it is possibly
                                            PW.PutToBoundary(Surf1, Surf2);
                                            //
                                            if (PW.NbPoints() < 3)
                                            {
                                                continue;
                                            }

                                            const int aMinNbPoints = 40;
                                            if (PW.NbPoints() < aMinNbPoints)
                                            {
                                                hasBeenAdded = PW.SeekAdditionalPoints(Surf1, Surf2, aMinNbPoints);
                                            }

                                            int iPWNbPoints = PW.NbPoints(), aNbPointsVer = 0;
                                            RejectLine = false;
                                            Point3dDebut = PW.Value(1).Value();
                                            Point3dFin = PW.Value(iPWNbPoints).Value();
                                            for (ver = 1; (!RejectLine) && (ver <= NbLigCalculee); ++ver)
                                            {
                                                IntPatch_WLine verwline = (IntPatch_WLine)SLin.Value(ver);
                                                //
                                                aNbPointsVer = verwline.NbPnts();
                                                if (aNbPointsVer < 3)
                                                {
                                                    continue;
                                                }
                                                //
                                                IntSurf_PntOn2S verPointDebut = verwline.Point(1);
                                                IntSurf_PntOn2S verPointFin = verwline.Point(verwline.NbPnts());
                                                // xf
                                                Point3D aP21 = verPointDebut.Value();
                                                Point3D aP22 = verPointFin.Value();
                                                //
                                                aD11 = Point3dDebut.Distance(aP21);
                                                aD12 = Point3dDebut.Distance(aP22);
                                                aD21 = Point3dFin.Distance(aP21);
                                                aD22 = Point3dFin.Distance(aP22);
                                                //
                                                if ((aD11 <= TolTangency && aD22 <= TolTangency)
                                                    || (aD12 <= TolTangency && aD21 <= TolTangency))
                                                {
                                                    int m, mx;
                                                    //
                                                    mx = aNbPointsVer / 2;
                                                    if ((aNbPointsVer % 2) != 0)
                                                    {
                                                        ++mx;
                                                    }
                                                    //
                                                    Point3D aPx = verwline.Point(mx).Value();
                                                    for (m = 1; m < iPWNbPoints; ++m)
                                                    {
                                                        Point3D aP1 = PW.Value(m).Value();
                                                        Point3D aP2 = PW.Value(m + 1).Value();
                                                        Vector3D aVec12 = new(aP1, aP2);
                                                        if (aVec12.SquareMagnitude() < 1.0e-20)
                                                        {
                                                            continue;
                                                        }

                                                        //
                                                        Direction3D aDir12 = new(aVec12);
                                                        Line3D aLin12 = new(aP1, aDir12);
                                                        aDx = aLin12.Distance(aPx);

                                                        // modified by NIZNHY-PKV Tue May 10 11:08:07 2011f
                                                        if (aDx <= 2.0 * Epsilon)
                                                        {
                                                            // if (aDx<=TolTangency) {
                                                            // modified by NIZNHY-PKV Tue May 10 11:08:13 2011t

                                                            RejectLine = true;
                                                            ver--;
                                                            break;
                                                        }
                                                    } // for(m=1; m<iPWNbPoints; ++m){
                                                } // if((aD11<=TolTangency && aD22<=TolTangency) ||...
                                            } // for( ver = 1 ; (!RejetLigne) && (ver<= NbLigCalculee) ; ver++) {
                                            //

                                            if (RejectLine)
                                            {
                                                DublicateOfLinesProcessing(PW, ver, SLin, ref RejectLine);
                                            }

                                            if (!RejectLine)
                                            {
                                                IntSurf_TypeTrans trans1, trans2;
                                                Vector3D norm1, norm2;
                                                Vector3D tgline = new(PW.TangentAtLine(out int indextg));
                                                PW.Line().Value(indextg).ParametersOnS1(out double locu, out double locv);
                                                Surf1.D1(locu, locv, out Point3D ptbid, out Vector3D d1u, out Vector3D d1v);
                                                norm1 = d1u.Crossed(d1v);
                                                PW.Line().Value(indextg).ParametersOnS2(out locu, out locv);
                                                Surf2.D1(locu, locv, out ptbid, out d1u, out d1v);
                                                norm2 = d1u.Crossed(d1v);
                                                if (tgline.DotCross(norm2, norm1) >= 0.0)
                                                {
                                                    trans1 = IntSurf_Out;
                                                    trans2 = IntSurf_In;
                                                }
                                                else
                                                {
                                                    trans1 = IntSurf_In;
                                                    trans2 = IntSurf_Out;
                                                }

                                                double TolTang = TolTangency;
                                                IntPatch_WLine wline = new(PW.Line(), false, trans1, trans2);
                                                wline.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLPrmPrm);
                                                wline.EnablePurging(!hasBeenAdded);
                                                // the method PutVertexOnLine can reduce the number of points in <wline>
                                                IntPatch_RstInt.PutVertexOnLine(wline, Surf1, D1, Surf2, true, TolTang);
                                                if (wline.NbPnts() < 2)
                                                {
                                                    continue;
                                                }
                                                IntPatch_RstInt.PutVertexOnLine(wline, Surf2, D2, Surf1, false, TolTang);
                                                if (wline.NbPnts() < 2)
                                                {
                                                    continue;
                                                }
                                                if (wline.NbVertex() == 0)
                                                {
                                                    AddEndVertices(wline, PW, Point3dDebut, Point3dFin, TolTang);
                                                }

                                                lignetrouvee = true;

                                                SeveralWlinesProcessing(Surf1, Surf2, SLin, Periods, trans1, trans2,
                                                                        TolTang, Max(PW.MaxStep(0), PW.MaxStep(1)),
                                                                        Max(PW.MaxStep(2), PW.MaxStep(3)), ref wline);

                                                AddWLine(SLin, wline, Deflection);
                                                empt = false;
                                            } // !RejetLigne
                                        } // PW points > 2
                                    } // done is True
                                } // dminiPointLigne > SeuildPointLigne
                            } // HasStartPoint
                        } // if TabPtDep[nbps2] == 0
                    } while (nbp > 5
                             && (NombreDePointsDeDepartDuCheminement < NbDePointsDeDepartDuChmLimit || !lignetrouvee)
                             && (NombreDePointsDeDepartDuCheminement - 3 < nbp || !(!lignetrouvee)));
                } // for( ls ...
            } // if nbLigSec >= 1
            //
            AdjustOnPeriodic(Surf1, Surf2, SLin);
            //

            //--------------------------------------------------------------------
            //-- Calcul des parametres approches a partir des Zones De Tangence --
            //--------------------------------------------------------------------
            {
                double UminLig1, VminLig1, UmaxLig1, VmaxLig1;
                double UminLig2, VminLig2, UmaxLig2, VmaxLig2;

                UminLig1 = VminLig1 = UminLig2 = VminLig2 = RealLast();
                UmaxLig1 = VmaxLig1 = UmaxLig2 = VmaxLig2 = -UminLig1;

                // NbPointsInTangentZone always == 1 (eap)

                int z;
                for (z = 1; z <= nbTanZon; z++)
                {
                    int pz = 1;
                    Interference.GetTangentZonePoint(z, pz, out _, out _, out _, out double U1, out double V1,
                                                     out double U2, out double V2);

                    if (U1 > UmaxLig1)
                    {
                        UmaxLig1 = U1;
                    }
                    if (V1 > VmaxLig1)
                    {
                        VmaxLig1 = V1;
                    }
                    if (U2 > UmaxLig2)
                    {
                        UmaxLig2 = U2;
                    }
                    if (V2 > VmaxLig2)
                    {
                        VmaxLig2 = V2;
                    }

                    if (U1 < UminLig1)
                    {
                        UminLig1 = U1;
                    }
                    if (V1 < VminLig1)
                    {
                        VminLig1 = V1;
                    }
                    if (U2 < UminLig2)
                    {
                        UminLig2 = U2;
                    }
                    if (V2 < VminLig2)
                    {
                        VminLig2 = V2;
                    }
                }

                for (z = 1; z <= nbTanZon; z++)
                {
                    int pz = 1;
                    Interference.GetTangentZonePoint(z, pz, out _, out _, out _, out double U1, out double V1,
                                                     out double U2, out double V2);

                    StartParams[1] = U1;
                    StartParams[2] = V1;
                    StartParams[3] = U2;
                    StartParams[4] = V2;

                    //-----------------------------------------------------------------------
                    //-- Calcul du premier point de cheminement a partir du point approche --
                    //-----------------------------------------------------------------------
                    HasStartPoint = PW.PerformFirstPoint(StartParams, ref StartPOn2S);
                    if (HasStartPoint)
                    {
                        //-------------------------------------------------
                        //-- Un point a ete trouve                       --
                        //-- On verifie qu il n appartient pas           --
                        //--  a une ligne de cheminement deja calculee.  --
                        //-------------------------------------------------
                        NbLigCalculee = SLin.Length();
                        dminiPointLigne = SeuildPointLigne + SeuildPointLigne;

                        for (int l = 1; (l <= NbLigCalculee) && (dminiPointLigne >= SeuildPointLigne); l++)
                        {
                            IntPatch_WLine testwline = (IntPatch_WLine)SLin.Value(l);

                            if (IsPointOnLine(StartPOn2S, testwline, Deflection))
                            {
                                dminiPointLigne = 0.0;
                            }
                        }

                        //-- Fin d exploration des lignes
                        if (dminiPointLigne > SeuildPointLigne)
                        {
                            //---------------------------------------------------
                            //-- Le point de depart du nouveau cheminement     --
                            //-- n est present dans aucune ligne deja calculee.--
                            //---------------------------------------------------
                            PW.Perform(StartParams, UminLig1, VminLig1, UminLig2, VminLig2, UmaxLig1, VmaxLig1,
                                       UmaxLig2, VmaxLig2);

                            if (PW.IsDone())
                            {
                                bool hasBeenAdded = false;
                                if (PW.NbPoints() > 2)
                                {
                                    const int aMinNbPoints = 40;
                                    if (PW.NbPoints() < aMinNbPoints)
                                    {
                                        hasBeenAdded = PW.SeekAdditionalPoints(Surf1, Surf2, aMinNbPoints);
                                    }

                                    //-----------------------------------------------
                                    //-- Verification a posteriori :
                                    //-- On teste si le point de depart et de fin de
                                    //-- la ligne de cheminement est present dans une
                                    //-- autre ligne .
                                    //-----------------------------------------------
                                    RejectLine = false;
                                    Point3dDebut = PW.Value(1).Value();
                                    IntSurf_PntOn2S PointFin = PW.Value(PW.NbPoints());
                                    Point3dFin = PointFin.Value();

                                    for (ver = 1; (!RejectLine) && (ver <= NbLigCalculee); ver++)
                                    {
                                        IntPatch_WLine verwline = (IntPatch_WLine)SLin.Value(ver);

                                        // Check end point if it is on existing line.
                                        // Start point is checked before.
                                        if (IsPointOnLine(PointFin, verwline, Deflection))
                                        {
                                            RejectLine = true;
                                            break;
                                        }

                                        IntSurf_PntOn2S verPointDebut = verwline.Point(1);
                                        IntSurf_PntOn2S verPointFin = verwline.Point(verwline.NbPnts());
                                        if (Point3dDebut.Distance(verPointDebut.Value()) < TolTangency)
                                        {
                                            RejectLine = true;
                                        }
                                        else
                                        {
                                            if (Point3dFin.Distance(verPointFin.Value()) < TolTangency)
                                            {
                                                RejectLine = true;
                                            }
                                        }
                                    }

                                    if (!RejectLine)
                                    {
                                        IntSurf_TypeTrans trans1, trans2;
                                        Vector3D norm1, norm2;
                                        Vector3D tgline = new(PW.TangentAtLine(out int indextg));
                                        PW.Line().Value(indextg).ParametersOnS1(out double locu, out double locv);
                                        Surf1.D1(locu, locv, out Point3D ptbid, out Vector3D d1u, out Vector3D d1v);
                                        norm1 = d1u.Crossed(d1v);
                                        PW.Line().Value(indextg).ParametersOnS2(out locu, out locv);
                                        Surf2.D1(locu, locv, out ptbid, out d1u, out d1v);
                                        norm2 = d1u.Crossed(d1v);
                                        if (tgline.DotCross(norm2, norm1) > 0.0)
                                        {
                                            trans1 = IntSurf_Out;
                                            trans2 = IntSurf_In;
                                        }
                                        else
                                        {
                                            trans1 = IntSurf_In;
                                            trans2 = IntSurf_Out;
                                        }

                                        double TolTang = TolTangency;
                                        IntPatch_WLine wline = new(PW.Line(), false, trans1, trans2);
                                        wline.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLPrmPrm);
                                        wline.EnablePurging(!hasBeenAdded);
                                        // the method PutVertexOnLine can reduce the number of points in <wline>
                                        IntPatch_RstInt.PutVertexOnLine(wline, Surf1, D1, Surf2, true, TolTang);
                                        if (wline.NbPnts() < 2)
                                        {
                                            continue;
                                        }
                                        IntPatch_RstInt.PutVertexOnLine(wline, Surf2, D2, Surf1, false, TolTang);
                                        if (wline.NbPnts() < 2)
                                        {
                                            continue;
                                        }

                                        //---------------
                                        if (wline.NbVertex() == 0)
                                        {
                                            AddEndVertices(wline, PW, Point3dDebut, Point3dFin, TolTang);
                                        }

                                        //---------------
                                        AddWLine(SLin, wline, Deflection);
                                        empt = false;
                                    }
                                    else
                                    {
                                        //-- std::cout<<" ----- REJET DE LIGNE (POINT DE DEPART) ----- "<<std::endl;
                                    }
                                    //------------------------------------------------------------
                                }
                            } //--  le cheminement a reussi (done a True)
                        } //--  le point approche ne renvoie pas sur une ligne existante
                    } //-- Si HasStartPoint
                } //-- Boucle sur Les Tangent Zones
            }

            return;
        } // if((NbU1*NbV1<=Limit && NbV2*NbU2<=Limit)) {

        PerformOnVoxels(Surf1, D1, Surf2, D2, TolTangency, Epsilon, Deflection, Increment, NbU1, NbV1, NbU2, NbV2);
    }

    // The rest of OCCT's Perform when a surface has more than 2,500 samples: walking lines started from the voxel
    // search's points. A separate method only because C# can't redeclare the locals of the block above.
    private void PerformOnVoxels(Adaptor3d_Surface Surf1, Adaptor3d_TopolTool D1, Adaptor3d_Surface Surf2,
                                 Adaptor3d_TopolTool D2, double TolTangency, double Epsilon, double Deflection,
                                 double Increment, int NbU1, int NbV1, int NbU2, int NbV2)
    {
        IntSurf_LineOn2S LOn2S = new();
        PointDepart(LOn2S, Surf1, NbU1, NbV1, Surf2, NbU2, NbV2);
        empt = true;
        done = true;
        // OCCT: SLin isn't cleared here, whatever ClearFlag says (occt-issues.md, 5D)

        int NbLigCalculee = 0;
        double U1, U2, V1, V2;

        NCollection_Array1<double> StartParams = new(1, 4);
        int MaxOscill = NbU1;
        if (MaxOscill < NbU2)
        {
            MaxOscill = NbU2;
        }
        if (MaxOscill < NbV1)
        {
            MaxOscill = NbV1;
        }
        if (MaxOscill < NbV2)
        {
            MaxOscill = NbV2;
        }
        _ = MaxOscill; // read only by OCCT's commented-out step reduction

        double nIncrement = Increment;
        // if(MaxOscill>10)
        // nIncrement/=0.5*MaxOscill;

        IntWalk_PWalking PW = new(Surf1, Surf2, TolTangency, Epsilon, Deflection, nIncrement);
        double SeuildPointLigne = 15.0 * Increment * Increment; //-- 10 est insuffisant
        double dminiPointLigne;
        bool HasStartPoint, RejetLigne;
        IntSurf_PntOn2S StartPOn2S = new();
        int ver;
        Point3D Point3dDebut, Point3dFin;

        //------------------------------------------------------------
        //-- Calcul des parametres approches a partir des Zones De Tangence --
        //--------------------------------------------------------------------
        int nbTanZon = LOn2S.NbPoints();
        for (int z = 1; z <= nbTanZon; z++)
        {
            IntSurf_PntOn2S POn2S = LOn2S.Value(z);
            POn2S.Parameters(out U1, out V1, out U2, out V2);
            StartParams[1] = U1;
            StartParams[2] = V1;
            StartParams[3] = U2;
            StartParams[4] = V2;

            //-----------------------------------------------------------------------
            //-- Calcul du premier point de cheminement a partir du point approche --
            //-----------------------------------------------------------------------
            HasStartPoint = PW.PerformFirstPoint(StartParams, ref StartPOn2S);
            if (HasStartPoint)
            {
                //-------------------------------------------------
                //-- Un point a ete trouve                       --
                //-- On verifie qu il n appartient pas           --
                //--  a une ligne de cheminement deja calculee.  --
                //-------------------------------------------------
                NbLigCalculee = SLin.Length();
                dminiPointLigne = SeuildPointLigne + SeuildPointLigne;

                for (int l = 1; (l <= NbLigCalculee) && (dminiPointLigne >= SeuildPointLigne); l++)
                {
                    IntPatch_WLine testwline = (IntPatch_WLine)SLin.Value(l);

                    if (IsPointOnLine(StartPOn2S, testwline, Deflection))
                    {
                        dminiPointLigne = 0.0;
                    }
                }

                //-- Fin d exploration des lignes
                if (dminiPointLigne > SeuildPointLigne)
                {
                    //---------------------------------------------------
                    //-- Le point de depart du nouveau cheminement     --
                    //-- n est present dans aucune ligne deja calculee.--
                    //---------------------------------------------------
                    PW.Perform(StartParams);
                    if (PW.IsDone())
                    {
                        if (PW.NbPoints() > 2)
                        {
                            //-----------------------------------------------
                            //-- Verification a posteriori :
                            //-- On teste si le point de depart et de fin de
                            //-- la ligne de cheminement est present dans une
                            //-- autre ligne .
                            //-----------------------------------------------
                            RejetLigne = false;
                            Point3dDebut = PW.Value(1).Value();
                            IntSurf_PntOn2S PointFin = PW.Value(PW.NbPoints());
                            Point3dFin = PointFin.Value();

                            for (ver = 1; ver <= NbLigCalculee; ver++)
                            {
                                IntPatch_WLine verwline = (IntPatch_WLine)SLin.Value(ver);

                                // Check end point if it is on existing line.
                                // Start point is checked before.
                                if (IsPointOnLine(PointFin, verwline, Deflection))
                                {
                                    RejetLigne = true;
                                    break;
                                }

                                IntSurf_PntOn2S verPointDebut = verwline.Point(1);
                                IntSurf_PntOn2S verPointFin = verwline.Point(verwline.NbPnts());
                                if ((Point3dDebut.Distance(verPointDebut.Value()) < TolTangency)
                                    || (Point3dFin.Distance(verPointFin.Value()) < TolTangency))
                                {
                                    RejetLigne = true;
                                    break;
                                }
                            }

                            if (RejetLigne)
                            {
                                DublicateOfLinesProcessing(PW, ver, SLin, ref RejetLigne);
                            }

                            if (!RejetLigne)
                            {
                                IntSurf_TypeTrans trans1, trans2;
                                Vector3D norm1, norm2;
                                Vector3D tgline = new(PW.TangentAtLine(out int indextg));
                                PW.Line().Value(indextg).ParametersOnS1(out double locu, out double locv);
                                Surf1.D1(locu, locv, out Point3D ptbid, out Vector3D d1u, out Vector3D d1v);
                                norm1 = d1u.Crossed(d1v);
                                PW.Line().Value(indextg).ParametersOnS2(out locu, out locv);
                                Surf2.D1(locu, locv, out ptbid, out d1u, out d1v);
                                norm2 = d1u.Crossed(d1v);
                                if (tgline.DotCross(norm2, norm1) > 0.0)
                                {
                                    trans1 = IntSurf_Out;
                                    trans2 = IntSurf_In;
                                }
                                else
                                {
                                    trans1 = IntSurf_In;
                                    trans2 = IntSurf_Out;
                                }

                                double TolTang = TolTangency;
                                IntPatch_WLine wline = new(PW.Line(), false, trans1, trans2);
                                wline.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLPrmPrm);

                                // the method PutVertexOnLine can reduce the number of points in <wline>
                                IntPatch_RstInt.PutVertexOnLine(wline, Surf1, D1, Surf2, true, TolTang);
                                if (wline.NbPnts() < 2)
                                {
                                    continue;
                                }
                                IntPatch_RstInt.PutVertexOnLine(wline, Surf2, D2, Surf1, false, TolTang);
                                if (wline.NbPnts() < 2)
                                {
                                    continue;
                                }

                                //---------------
                                if (wline.NbVertex() == 0)
                                {
                                    AddEndVertices(wline, PW, Point3dDebut, Point3dFin, TolTang);
                                }

                                //---------------
                                AddWLine(SLin, wline, Deflection);
                                empt = false;
                            }
                            else
                            {
                                //-- std::cout<<" ----- REJET DE LIGNE (POINT DE DEPART) ----- "<<std::endl;
                            }
                            //------------------------------------------------------------
                        }
                    } //--  le cheminement a reussi (done a True)
                } //--  le point approche ne renvoie pas sur une ligne existante
            } //-- Si HasStartPoint
        } //-- Boucle sur Les Tangent Zones
    }

    // modified by NIZNHY-PKV Wed May 25 09:39:07 2011f
    //=================================================================================================

    // OCCT's file-local IntPatch_InfoPD: the samples of both surfaces on a grid of at most 200 x 200, their voxels and
    // their position codes against the other surface's box.
    private sealed class IntPatch_InfoPD
    {
        private readonly int myNBI;
        private readonly byte[] myP1DS2;
        private readonly byte[] myP2DS1;
        private readonly int[] myIP1;
        private readonly int[] myIP2;
        private readonly Point3D[] myP1;
        private readonly Point3D[] myP2;

        //----------------------------------------C-tor
        public IntPatch_InfoPD(int aNBI)
        {
            int aNBI2, i, j;
            myNBI = aNBI;
            //
            aNBI2 = aNBI * aNBI;
            myP1DS2 = new byte[aNBI2];
            myP2DS1 = new byte[aNBI2];
            myIP1 = new int[aNBI2];
            myIP2 = new int[aNBI2];
            myP1 = new Point3D[aNBI2];
            myP2 = new Point3D[aNBI2];
            //
            for (i = 0; i < myNBI; ++i)
            {
                for (j = 0; j < myNBI; ++j)
                {
                    xP1DS2(i, j) = 0;
                    xP2DS1(i, j) = 0;
                    xIP1(i, j) = 0;
                    xIP2(i, j) = 0;
                    xP1(i, j).SetCoord(0.0, 0.0, 0.0);
                    xP2(i, j).SetCoord(0.0, 0.0, 0.0);
                }
            }
        }

        //---------------------------------------- Index
        public int Index(int i, int j) => i * myNBI + j;

        //---------------------------------------- NBI
        public int NBI() => myNBI;

        //----------------------------------------xP1DS2
        public ref byte xP1DS2(int i, int j) => ref myP1DS2[Index(i, j)];

        //----------------------------------------xP2DS1
        public ref byte xP2DS1(int i, int j) => ref myP2DS1[Index(i, j)];

        //----------------------------------------xIP1
        public ref int xIP1(int i, int j) => ref myIP1[Index(i, j)];

        //----------------------------------------xIP2
        public ref int xIP2(int i, int j) => ref myIP2[Index(i, j)];

        //----------------------------------------xP1
        public ref Point3D xP1(int i, int j) => ref myP1[Index(i, j)];

        //----------------------------------------xP1
        public ref Point3D xP2(int i, int j) => ref myP2[Index(i, j)];
    }

    // modified by NIZNHY-PKV Tue May 24 11:38:55 2011t
    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="LineOn2S"/> with start points for the walking: the surfaces are sampled (15 times their
    /// sampling, at most 200 by 200), the samples' triangles are marked in two 128³ voxel grids over the common box,
    /// and each voxel marked in both gives a point, with the parameters of samples in it (or the mean of the three
    /// nearest samples on each surface).
    /// </summary>
    public void PointDepart(IntSurf_LineOn2S LineOn2S, Adaptor3d_Surface S1, int SU_1, int SV_1,
                            Adaptor3d_Surface S2, int SU_2, int SV_2)
    {
        int i, j, xNBI;
        // modified by NIZNHY-PKV Tue May 24 11:37:38 2011f
        xNBI = 200;
        IntPatch_InfoPD aIPD = new(xNBI);
        // modified by NIZNHY-PKV Wed May 25 06:47:12 2011t
        int iC15, SU1, SV1, SU2, SV2;
        double U0, U1, V0, V1, U, V;
        double resu0, resv0;
        double du1, du2, dv1, dv2, dmaxOn1, dmaxOn2;
        double x0, y0, z0, x1, y1, z1, d;
        Box Box1 = new(), Box2 = new();
        //
        iC15 = 15;
        SU1 = iC15 * SU_1;
        SV1 = iC15 * SV_1;
        SU2 = iC15 * SU_2;
        SV2 = iC15 * SV_2;
        //
        if (xNBI < SU1)
        {
            SU1 = xNBI;
        }
        if (xNBI < SV1)
        {
            SV1 = xNBI;
        }
        if (xNBI < SU2)
        {
            SU2 = xNBI;
        }
        if (xNBI < SV2)
        {
            SV2 = xNBI;
        }
        //
        U0 = S1.FirstUParameter();
        U1 = S1.LastUParameter();
        V0 = S1.FirstVParameter();
        V1 = S1.LastVParameter();
        //
        resu0 = U0;
        resv0 = V0;
        //
        dmaxOn1 = 0.0;
        dmaxOn2 = 0.0;
        //-----
        du1 = (U1 - U0) / (SU1 - 1);
        dv1 = (V1 - V0) / (SV1 - 1);
        for (U = U0, i = 0; i < SU1; i++, U += du1)
        {
            for (V = V0, j = 0; j < SV1; V += dv1, j++)
            {
                aIPD.xP1(i, j) = S1.Value(U, V);
                Box1.Add(aIPD.xP1(i, j));
                if (i > 0 && j > 0)
                {
                    aIPD.xP1(i, j).Coord(out x0, out y0, out z0);
                    aIPD.xP1(i - 1, j - 1).Coord(out x1, out y1, out z1);
                    //
                    d = Math.Abs(x1 - x0) + Math.Abs(y1 - y0) + Math.Abs(z1 - z0);
                    if (d > dmaxOn1)
                    {
                        dmaxOn1 = d;
                    }
                }
            }
        }
        Box1.Enlarge(1.0e-8);
        //
        U0 = S2.FirstUParameter();
        U1 = S2.LastUParameter();
        V0 = S2.FirstVParameter();
        V1 = S2.LastVParameter();
        //
        du2 = (U1 - U0) / (SU2 - 1);
        dv2 = (V1 - V0) / (SV2 - 1);
        for (U = U0, i = 0; i < SU2; i++, U += du2)
        {
            for (V = V0, j = 0; j < SV2; V += dv2, j++)
            {
                aIPD.xP2(i, j) = S2.Value(U, V);
                Box2.Add(aIPD.xP2(i, j));
                if (i > 0 && j > 0)
                {
                    aIPD.xP2(i, j).Coord(out x0, out y0, out z0);
                    aIPD.xP2(i - 1, j - 1).Coord(out x1, out y1, out z1);
                    d = Math.Abs(x1 - x0) + Math.Abs(y1 - y0) + Math.Abs(z1 - z0);
                    if (d > dmaxOn2)
                    {
                        dmaxOn2 = d;
                    }
                }
            }
        }
        Box2.Enlarge(1.0e-8);
        //--------
        //
        if (Box1.IsOut(Box2))
        {
            return;
        }
        //
        int aNbPG;
        double x10, y10, z10, x11, y11, z11;
        double x20, y20, z20, x21, y21, z21;
        double dx, dy, dz, dmax;
        double dx2, dy2, dz2;
        //
        Box1.Get(out x10, out y10, out z10, out x11, out y11, out z11);
        Box2.Get(out x20, out y20, out z20, out x21, out y21, out z21);
        //
        x0 = (x10 > x20) ? x10 : x20;
        y0 = (y10 > y20) ? y10 : y20;
        z0 = (z10 > z20) ? z10 : z20;
        //
        x1 = (x11 < x21) ? x11 : x21;
        y1 = (y11 < y21) ? y11 : y21;
        z1 = (z11 < z21) ? z11 : z21;
        //
        if (dmaxOn2 > dmaxOn1)
        {
            dmaxOn1 = dmaxOn2;
        }
        //
        dmaxOn1 += dmaxOn1;
        x0 -= dmaxOn1;
        y0 -= dmaxOn1;
        z0 -= dmaxOn1;
        x1 += dmaxOn1;
        y1 += dmaxOn1;
        z1 += dmaxOn1;
        //
        x10 -= dmaxOn1;
        y10 -= dmaxOn1;
        z10 -= dmaxOn1;
        x11 += dmaxOn1;
        y11 += dmaxOn1;
        z11 += dmaxOn1;

        x20 -= dmaxOn1;
        y20 -= dmaxOn1;
        z20 -= dmaxOn1;
        x21 += dmaxOn1;
        y21 += dmaxOn1;
        z21 += dmaxOn1;

        aNbPG = NbPointsGrille();
        dx = (x1 - x0) / aNbPG;
        dy = (y1 - y0) / aNbPG;
        dz = (z1 - z0) / aNbPG;
        //
        dmax = dx;
        if (dy > dmax)
        {
            dmax = dy;
        }
        if (dz > dmax)
        {
            dmax = dz;
        }
        //
        if (dx < dmax * 0.01)
        {
            dx = dmax * 0.01;
        }
        if (dy < dmax * 0.01)
        {
            dy = dmax * 0.01;
        }
        if (dz < dmax * 0.01)
        {
            dz = dmax * 0.01;
        }
        //
        dx2 = dx * 0.5;
        dy2 = dy * 0.5;
        dz2 = dz * 0.5;
        //
        IntPatch_PrmPrmIntersection_T3Bits M1 = new(_BASE);
        IntPatch_PrmPrmIntersection_T3Bits M2 = new(_BASE);
        //
        for (i = 0; i < SU1; i++)
        {
            for (j = 0; j < SV1; j++)
            {
                aIPD.xIP1(i, j) = -1;
                Point3D P = aIPD.xP1(i, j);
                aIPD.xP1DS2(i, j) = (byte)CodeReject(x20, y20, z20, x21, y21, z21, P.X(), P.Y(), P.Z());
                int ix = CppTruncate((P.X() - x0 + dx2) / dx);
                if (DansGrille(ix) != 0)
                {
                    int iy = CppTruncate((P.Y() - y0 + dy2) / dy);
                    if (DansGrille(iy) != 0)
                    {
                        int iz = CppTruncate((P.Z() - z0 + dz2) / dz);
                        if (DansGrille(iz) != 0)
                        {
                            aIPD.xIP1(i, j) = GrilleInteger(ix, iy, iz);
                        }
                    }
                }
            }
        }
        //-- std::cout<<" Grille  du 1 fini "<<std::endl;
        for (i = 0; i < SU2; i++)
        {
            for (j = 0; j < SV2; j++)
            {
                aIPD.xIP2(i, j) = -1;
                Point3D P = aIPD.xP2(i, j);
                aIPD.xP2DS1(i, j) = (byte)CodeReject(x10, y10, z10, x11, y11, z11, P.X(), P.Y(), P.Z());
                int ix = CppTruncate((P.X() - x0 + dx2) / dx);
                if (DansGrille(ix) != 0)
                {
                    int iy = CppTruncate((P.Y() - y0 + dy2) / dy);
                    if (DansGrille(iy) != 0)
                    {
                        int iz = CppTruncate((P.Z() - z0 + dz2) / dz);
                        if (DansGrille(iz) != 0)
                        {
                            aIPD.xIP2(i, j) = GrilleInteger(ix, iy, iz);
                        }
                    }
                }
            }
        }
        //
        for (i = 0; i < SU1 - 1; i += 1)
        {
            for (j = 0; j < SV1 - 1; j += 1)
            {
                if (!(((aIPD.xP1DS2(i, j) & aIPD.xP1DS2(i + 1, j)) != 0)
                      || ((aIPD.xP1DS2(i, j) & aIPD.xP1DS2(i + 1, j + 1)) != 0)))
                {
                    Remplit(aIPD.xIP1(i, j), aIPD.xIP1(i + 1, j), aIPD.xIP1(i + 1, j + 1), M1);
                }
                if (!(((aIPD.xP1DS2(i, j) & aIPD.xP1DS2(i, j + 1)) != 0)
                      || ((aIPD.xP1DS2(i, j) & aIPD.xP1DS2(i + 1, j + 1)) != 0)))
                {
                    Remplit(aIPD.xIP1(i, j), aIPD.xIP1(i, j + 1), aIPD.xIP1(i + 1, j + 1), M1);
                }
            }
        }
        //
        for (i = 0; i < SU2 - 1; i += 1)
        {
            for (j = 0; j < SV2 - 1; j += 1)
            {
                if (!(((aIPD.xP2DS1(i, j) & aIPD.xP2DS1(i + 1, j)) != 0)
                      || ((aIPD.xP2DS1(i, j) & aIPD.xP2DS1(i + 1, j + 1)) != 0)))
                {
                    Remplit(aIPD.xIP2(i, j), aIPD.xIP2(i + 1, j), aIPD.xIP2(i + 1, j + 1), M2);
                }
                if (!(((aIPD.xP2DS1(i, j) & aIPD.xP2DS1(i, j + 1)) != 0)
                      || ((aIPD.xP2DS1(i, j) & aIPD.xP2DS1(i + 1, j + 1)) != 0)))
                {
                    Remplit(aIPD.xIP2(i, j), aIPD.xIP2(i, j + 1), aIPD.xIP2(i + 1, j + 1), M2);
                }
            }
        }
        //
        M1.ResetAnd();
        M2.ResetAnd();
        //
        int newind = 0;
        int ok = 0;
        int indicepointtraite = 0;
        int k, nu, nv;
        // The else branch's local arrays (u3 and v3 are zeroed where OCCT declares them)
        Span<double> Dist3 = stackalloc double[3];
        Span<double> u3 = stackalloc double[3];
        Span<double> v3 = stackalloc double[3];
        //
        do
        {
            indicepointtraite--;
            ok = M1.And(M2, ref newind);
            if (ok != 0)
            {
                IntegerGrille(newind, out i, out j, out k);
                int nb = 0;
                int LIM = 3;
                if (DansGrille(i - 1) != 0 && DansGrille(j - 1) != 0 && DansGrille(k - 1) != 0
                    && DansGrille(i + 1) != 0 && DansGrille(j + 1) != 0 && DansGrille(k + 1) != 0)
                {
                    int si, sj, sk;
                    for (si = -1; si <= 1 && nb < LIM; si++)
                    {
                        for (sj = -1; sj <= 1 && nb < LIM; sj++)
                        {
                            for (sk = -1; sk <= 1 && nb < LIM; sk++)
                            {
                                int lu = GrilleInteger(i + si, j + sj, k + sk);
                                if (M1.Val(lu) != 0 && M2.Val(lu) != 0)
                                {
                                    nb++;
                                }
                            }
                        }
                    }
                    if (nb >= LIM)
                    {
                        for (si = -1; si <= 1; si++)
                        {
                            for (sj = -1; sj <= 1; sj++)
                            {
                                for (sk = -1; sk <= 1; sk++)
                                {
                                    if (si != 0 || sj != 0 || sk != 0)
                                    {
                                        int lu = GrilleInteger(i + si, j + sj, k + sk);
                                        M1.Raz(lu);
                                    }
                                }
                            }
                        }
                    }
                }
                //
                Point3D P = new(dx * i + x0, dy * j + y0, dz * k + z0);
                //
                int nu1 = -1, nu2 = -1;
                int nv1 = 0, nv2 = 0;
                for (nu = 0; nu1 < 0 && nu < SU1; nu++)
                {
                    for (nv = 0; nu1 < 0 && nv < SV1; nv++)
                    {
                        if (aIPD.xIP1(nu, nv) == (int)newind)
                        {
                            aIPD.xIP1(nu, nv) = indicepointtraite;
                            nu1 = nu;
                            nv1 = nv;
                        }
                    }
                }
                if (nu1 >= 0)
                {
                    for (nu = 0; nu2 < 0 && nu < SU2; nu++)
                    {
                        for (nv = 0; nu2 < 0 && nv < SV2; nv++)
                        {
                            if (aIPD.xIP2(nu, nv) == (int)newind)
                            {
                                aIPD.xIP2(nu, nv) = indicepointtraite;
                                nu2 = nu;
                                nv2 = nv;
                            }
                        }
                    }
                }
                if (nu1 >= 0 && nu2 >= 0)
                {
                    IntSurf_PntOn2S POn2S = new();
                    POn2S.SetValue(P,
                                   S1.FirstUParameter() + nu1 * du1,
                                   S1.FirstVParameter() + nv1 * dv1,
                                   S2.FirstUParameter() + nu2 * du2,
                                   S2.FirstVParameter() + nv2 * dv2);
                    LineOn2S.Add(POn2S);
                }
                else
                {
                    //-- aucun point du triangle n a ete trouve assez proche
                    //-- on recherche les 3 points les plus proches de P
                    //-- dans chacun des tableaux
                    u3.Clear();
                    v3.Clear();
                    Dist3[0] = Dist3[1] = Dist3[2] = RealLast();
                    for (U = resu0, i = 0; i < SU1; i++, U += du1)
                    {
                        for (V = resv0, j = 0; j < SV1; V += dv1, j++)
                        {
                            //-- On place les 3 meilleures valeurs dans Dist1,Dist2,Dist3
                            double t = aIPD.xP1(i, j).SquareDistance(P);
                            SortThreeNearest(Dist3, u3, v3);
                            //-- la plus grande valeur est dans Dist3[0]
                            if (t < Dist3[0])
                            {
                                Dist3[0] = t;
                                u3[0] = U;
                                v3[0] = V;
                            }
                        }
                    }
                    //
                    double U1_3 = (u3[0] + u3[1] + u3[2]) / 3.0;
                    double V1_3 = (v3[0] + v3[1] + v3[2]) / 3.0;

                    Dist3[0] = Dist3[1] = Dist3[2] = RealLast();
                    for (U = U0, i = 0; i < SU2; i++, U += du2)
                    {
                        for (V = V0, j = 0; j < SV2; V += dv2, j++)
                        {
                            //-- On place les 3 meilleures valeurs dans Dist1,Dist2,Dist3
                            double t = aIPD.xP2(i, j).SquareDistance(P);
                            SortThreeNearest(Dist3, u3, v3);
                            //-- la plus grande valeur est dans Dist3[0]
                            if (t < Dist3[0])
                            {
                                Dist3[0] = t;
                                u3[0] = U;
                                v3[0] = V;
                            }
                        }
                    }
                    //
                    double U2_3 = (u3[0] + u3[1] + u3[2]) / 3.0;
                    double V2_3 = (v3[0] + v3[1] + v3[2]) / 3.0;
                    //
                    IntSurf_PntOn2S POn2S = new();
                    POn2S.SetValue(P, U1_3, V1_3, U2_3, V2_3);
                    LineOn2S.Add(POn2S);
                }
            }
        } while (ok != 0);
    }

    // The three swaps OCCT repeats verbatim in both nearest-sample searches of PointDepart: puts the largest of the
    // three distances (and its parameters) in the first place.
    private static void SortThreeNearest(Span<double> Dist3, Span<double> u3, Span<double> v3)
    {
        //-- On remplace la plus grande valeur ds Dist[.] par la val courante
        if (Dist3[0] < Dist3[1])
        {
            double z;
            z = Dist3[0];
            Dist3[0] = Dist3[1];
            Dist3[1] = z;
            z = u3[0];
            u3[0] = u3[1];
            u3[1] = z;
            z = v3[0];
            v3[0] = v3[1];
            v3[1] = z;
        }
        if (Dist3[1] < Dist3[2])
        {
            double z;
            z = Dist3[1];
            Dist3[1] = Dist3[2];
            Dist3[2] = z;
            z = u3[1];
            u3[1] = u3[2];
            u3[2] = z;
            z = v3[1];
            v3[1] = v3[2];
            v3[2] = z;
        }
        if (Dist3[0] < Dist3[1])
        {
            double z;
            z = Dist3[0];
            Dist3[0] = Dist3[1];
            Dist3[1] = z;
            z = u3[0];
            u3[0] = u3[1];
            u3[1] = z;
            z = v3[0];
            v3[0] = v3[1];
            v3[1] = z;
        }
    }

    // C++'s conversion of a double to int as MSVC's x64 code does it (cvttsd2si): truncation toward zero, and
    // 0x80000000 for NaN and values out of range, where .NET's conversion saturates (and gives 0 for NaN). Every
    // out-of-range value is then outside the grid, as in OCCT.
    private static int CppTruncate(double theValue) =>
        (theValue > -2147483649.0 && theValue < 2147483648.0) ? (int)theValue : int.MinValue;
}
