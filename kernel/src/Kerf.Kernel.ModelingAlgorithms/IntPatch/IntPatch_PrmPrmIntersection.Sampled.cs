// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// IntPatch_PrmPrmIntersection.cxx: the Perform on the domains' samplings (IntPolyh_Intersection or the voxel search),
// PointDepart and its file-local class IntPatch_InfoPD (see IntPatch_PrmPrmIntersection.cs).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Polyhedra;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

public partial class PrmPrmIntersection
{
    //=================================================================================================

    /// <summary>
    /// The intersection of <paramref name="surf1"/> and <paramref name="surf2"/> from the samplings of their domains
    /// (<c>SamplePnts</c> with <paramref name="deflection"/>): with at most 2,500 samples on each surface, walking
    /// lines started from the section lines and tangent zones of IntPolyh_Intersection, otherwise from the voxel search
    /// <see cref="PointDepart"/>. With <paramref name="clearFlag"/> false, the lines of an earlier Perform are kept (the
    /// voxel search keeps them in any case, as in OCCT).
    /// </summary>
    public void Perform(SurfaceAdaptorBase surf1, TopolTool d1, SurfaceAdaptorBase surf2,
                        TopolTool d2, double tolTangency, double epsilon, double deflection,
                        double increment, bool clearFlag = true)
    {
        int limit = 2500;
        int nbU1 = 10, nbV1 = 10, nbU2 = 10, nbV2 = 10;
        //
        d1.SamplePnts(deflection, nbU1, nbV1);
        d2.SamplePnts(deflection, nbU2, nbV2);
        //
        nbU1 = d1.NbSamplesU();
        nbV1 = d1.NbSamplesV();
        nbU2 = d2.NbSamplesU();
        nbV2 = d2.NbSamplesV();

        NCollection_Array1<double> upars1 = new(1, nbU1), vpars1 = new(1, nbV1);
        NCollection_Array1<double> upars2 = new(1, nbU2), vpars2 = new(1, nbV2);
        //
        d1.UParameters(upars1);
        d1.VParameters(vpars1);
        d2.UParameters(upars2);
        d2.VParameters(vpars2);

        double[] periods = new double[4];
        periods[0] = (surf1.IsUPeriodic) ? surf1.UPeriod() : 0.0;
        periods[1] = (surf1.IsVPeriodic) ? surf1.VPeriod() : 0.0;
        periods[2] = (surf2.IsUPeriodic) ? surf2.UPeriod() : 0.0;
        periods[3] = (surf2.IsVPeriodic) ? surf2.VPeriod() : 0.0;

        //---------------------------------------------
        if ((nbU1 * nbV1 <= limit && nbV2 * nbU2 <= limit))
        {
            _empt = true;
            if (clearFlag)
            {
                _sLin.Clear();
            }
            //
            PolyhedronIntersection pInterference;

            if (d1.IsUniformSampling() || d2.IsUniformSampling())
            {
                pInterference = new PolyhedronIntersection(surf1, nbU1, nbV1, surf2, nbU2, nbV2);
            }
            else
            {
                pInterference = new PolyhedronIntersection(surf1, upars1, vpars1, surf2, upars2, vpars2);
            }

            // OCCT tests the pointer it has just allocated against null here; `new` never gives null.
            //
            PolyhedronIntersection interference = pInterference;
            //
            _done = interference.IsDone;
            if (!_done)
            {
                return;
            }

            int nbLigSec = interference.SectionLineCount;
            int nbTanZon = interference.TangentZoneCount;
            double seuildPointLigne = 15.0 * increment * increment;

            int nbLigCalculee = 0, ver;
            double incidence, dminiPointLigne;
            bool hasStartPoint = false, rejectLine = false;
            PntOn2S startPOn2S = new();
            Point3D point3dDebut, point3dFin;

            NCollection_Array1<double> startParams = new(1, 4);
            PWalking pw = new(surf1, surf2, tolTangency, epsilon, deflection, increment);

            if (nbLigSec >= 1)
            {
                int[] tabL = new int[nbLigSec + 1];
                for (int ls = 1; ls <= nbLigSec; ++ls)
                {
                    tabL[ls] = ls;
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
                            nb_B = interference.NbPointsInLine(tabL[b]);
                            nb_A = interference.NbPointsInLine(tabL[b - 1]);

                            if (nb_B > nb_A)
                            {
                                tyu = tabL[b];
                                tabL[b] = tabL[b - 1];
                                tabL[b - 1] = tyu;
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
                    int nbp = interference.NbPointsInLine(tabL[ls]);
                    if (nbp == 0)
                    {
                        continue;
                    }
                    //
                    int[] tabPtDep = new int[nbp + 1];
                    for (int ilig = 1; ilig <= nbp; ++ilig)
                    {
                        tabPtDep[ilig] = 0;
                    }
                    //
                    double uminLig1, vminLig1, umaxLig1, vmaxLig1;
                    double uminLig2, vminLig2, umaxLig2, vmaxLig2;
                    double _x, _y, _z;
                    //
                    interference.GetLinePoint(tabL[ls], 1, out _x, out _y, out _z, out uminLig1, out vminLig1,
                                              out uminLig2, out vminLig2, out incidence);

                    umaxLig1 = uminLig1;
                    vmaxLig1 = vminLig1;
                    umaxLig2 = uminLig2;
                    vmaxLig2 = vminLig2;
                    //
                    for (int ilig = 2; ilig <= nbp; ilig++)
                    {
                        interference.GetLinePoint(tabL[ls], ilig, out _x, out _y, out _z, out double u1, out double v1,
                                                  out double u2, out double v2, out incidence);
                        //
                        if (u1 > umaxLig1)
                        {
                            umaxLig1 = u1;
                        }
                        if (v1 > vmaxLig1)
                        {
                            vmaxLig1 = v1;
                        }
                        if (u2 > umaxLig2)
                        {
                            umaxLig2 = u2;
                        }
                        if (v2 > vmaxLig2)
                        {
                            vmaxLig2 = v2;
                        }
                        //
                        if (u1 < uminLig1)
                        {
                            uminLig1 = u1;
                        }
                        if (v1 < vminLig1)
                        {
                            vminLig1 = v1;
                        }
                        if (u2 < uminLig2)
                        {
                            uminLig2 = u2;
                        }
                        if (v2 < vminLig2)
                        {
                            vminLig2 = v2;
                        }
                    } // for( ilig = 2; ilig <= nbp; ilig++ ) {
                    //
                    //----------------------------------------
                    // 1.3
                    int nbps2 = (nbp > 3) ? (nbp / 2) : 1;
                    int nombreDePointsDeDepartDuCheminement = 0;
                    bool lignetrouvee = false;
                    const int nbDePointsDeDepartDuChmLimit = 5;
                    //
                    do
                    {
                        nombreDePointsDeDepartDuCheminement++;
                        switch (nombreDePointsDeDepartDuCheminement)
                        {
                            case 1:
                                nbps2 = (nbp > 1) ? nbp / 2 : 1;
                                if (nbp < 3)
                                {
                                    nombreDePointsDeDepartDuCheminement = nbDePointsDeDepartDuChmLimit;
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
                                nbps2 = nombreDePointsDeDepartDuCheminement - 3;
                                nombreDePointsDeDepartDuCheminement++;
                                break;
                        }

                        //
                        if (tabPtDep[nbps2] == 0)
                        {
                            tabPtDep[nbps2] = 1;
                            interference.GetLinePoint(tabL[ls], nbps2, out _x, out _y, out _z, out double u1,
                                                      out double v1, out double u2, out double v2, out incidence);

                            startParams[1] = u1;
                            startParams[2] = v1;
                            startParams[3] = u2;
                            startParams[4] = v2;

                            hasStartPoint = pw.PerformFirstPoint(startParams, ref startPOn2S);
                            dminiPointLigne = seuildPointLigne + seuildPointLigne;
                            if (hasStartPoint)
                            {
                                nbLigCalculee = _sLin.Length();
                                int l;
                                for (l = 1; (l <= nbLigCalculee) && (dminiPointLigne >= seuildPointLigne); l++)
                                {
                                    WLine testwline = (WLine)_sLin.Value(l);

                                    if (IsPointOnLine(startPOn2S, testwline, deflection))
                                    {
                                        dminiPointLigne = 0.0;
                                    }
                                } // for( l ...

                                if (dminiPointLigne > seuildPointLigne)
                                {
                                    pw.Perform(startParams, uminLig1, vminLig1, uminLig2, vminLig2, umaxLig1, vmaxLig1,
                                               umaxLig2, vmaxLig2);

                                    //
                                    bool bPWIsDone;
                                    double d11, d12, d21, d22, dx;
                                    //
                                    bPWIsDone = pw.IsDone;

                                    if (bPWIsDone)
                                    {
                                        bool hasBeenAdded = false;
                                        if (pw.PointCount > 2)
                                        {
                                            // Try to extend the intersection line to the boundary,
                                            // if it is possibly
                                            pw.PutToBoundary(surf1, surf2);
                                            //
                                            if (pw.PointCount < 3)
                                            {
                                                continue;
                                            }

                                            const int minNbPoints = 40;
                                            if (pw.PointCount < minNbPoints)
                                            {
                                                hasBeenAdded = pw.SeekAdditionalPoints(surf1, surf2, minNbPoints);
                                            }

                                            int iPWNbPoints = pw.PointCount, nbPointsVer = 0;
                                            rejectLine = false;
                                            point3dDebut = pw.Value(1).Value();
                                            point3dFin = pw.Value(iPWNbPoints).Value();
                                            for (ver = 1; (!rejectLine) && (ver <= nbLigCalculee); ++ver)
                                            {
                                                WLine verwline = (WLine)_sLin.Value(ver);
                                                //
                                                nbPointsVer = verwline.NbPnts();
                                                if (nbPointsVer < 3)
                                                {
                                                    continue;
                                                }
                                                //
                                                PntOn2S verPointDebut = verwline.Point(1);
                                                PntOn2S verPointFin = verwline.Point(verwline.NbPnts());
                                                // xf
                                                Point3D p21 = verPointDebut.Value();
                                                Point3D p22 = verPointFin.Value();
                                                //
                                                d11 = point3dDebut.DistanceTo(p21);
                                                d12 = point3dDebut.DistanceTo(p22);
                                                d21 = point3dFin.DistanceTo(p21);
                                                d22 = point3dFin.DistanceTo(p22);
                                                //
                                                if ((d11 <= tolTangency && d22 <= tolTangency)
                                                    || (d12 <= tolTangency && d21 <= tolTangency))
                                                {
                                                    int m, mx;
                                                    //
                                                    mx = nbPointsVer / 2;
                                                    if ((nbPointsVer % 2) != 0)
                                                    {
                                                        ++mx;
                                                    }
                                                    //
                                                    Point3D px = verwline.Point(mx).Value();
                                                    for (m = 1; m < iPWNbPoints; ++m)
                                                    {
                                                        Point3D p1 = pw.Value(m).Value();
                                                        Point3D p2 = pw.Value(m + 1).Value();
                                                        Vector3D vec12 = new(p1, p2);
                                                        if (vec12.SquareMagnitude < 1.0e-20)
                                                        {
                                                            continue;
                                                        }

                                                        //
                                                        Direction3D dir12 = new(vec12);
                                                        Line3D lin12 = new(p1, dir12);
                                                        dx = lin12.DistanceTo(px);

                                                        // modified by NIZNHY-PKV Tue May 10 11:08:07 2011f
                                                        if (dx <= 2.0 * epsilon)
                                                        {
                                                            // if (aDx<=TolTangency) {
                                                            // modified by NIZNHY-PKV Tue May 10 11:08:13 2011t

                                                            rejectLine = true;
                                                            ver--;
                                                            break;
                                                        }
                                                    } // for(m=1; m<iPWNbPoints; ++m){
                                                } // if((aD11<=TolTangency && aD22<=TolTangency) ||...
                                            } // for( ver = 1 ; (!RejetLigne) && (ver<= NbLigCalculee) ; ver++) {
                                            //

                                            if (rejectLine)
                                            {
                                                DublicateOfLinesProcessing(pw, ver, _sLin, ref rejectLine);
                                            }

                                            if (!rejectLine)
                                            {
                                                WalkingTypeTrans trans1, trans2;
                                                Vector3D norm1, norm2;
                                                Vector3D tgline = new(pw.TangentAtLine(out int indextg));
                                                pw.Line().Value(indextg).ParametersOnS1(out double locu, out double locv);
                                                surf1.D1(locu, locv, out Point3D ptbid, out Vector3D d1u, out Vector3D d1v);
                                                norm1 = d1u.Crossed(d1v);
                                                pw.Line().Value(indextg).ParametersOnS2(out locu, out locv);
                                                surf2.D1(locu, locv, out ptbid, out d1u, out d1v);
                                                norm2 = d1u.Crossed(d1v);
                                                if (tgline.DotCross(norm2, norm1) >= 0.0)
                                                {
                                                    trans1 = WalkingTypeTrans.Out;
                                                    trans2 = WalkingTypeTrans.In;
                                                }
                                                else
                                                {
                                                    trans1 = WalkingTypeTrans.In;
                                                    trans2 = WalkingTypeTrans.Out;
                                                }

                                                double tolTang = tolTangency;
                                                WLine wline = new(pw.Line(), false, trans1, trans2);
                                                wline.SetCreatingWayInfo(WLine.IntPatch_WLType.WLPrmPrm);
                                                wline.EnablePurging(!hasBeenAdded);
                                                // the method PutVertexOnLine can reduce the number of points in <wline>
                                                RstInt.PutVertexOnLine(wline, surf1, d1, surf2, true, tolTang);
                                                if (wline.NbPnts() < 2)
                                                {
                                                    continue;
                                                }
                                                RstInt.PutVertexOnLine(wline, surf2, d2, surf1, false, tolTang);
                                                if (wline.NbPnts() < 2)
                                                {
                                                    continue;
                                                }
                                                if (wline.VertexCount == 0)
                                                {
                                                    AddEndVertices(wline, pw, point3dDebut, point3dFin, tolTang);
                                                }

                                                lignetrouvee = true;

                                                SeveralWlinesProcessing(surf1, surf2, _sLin, periods, trans1, trans2,
                                                                        tolTang, Max(pw.MaxStep(0), pw.MaxStep(1)),
                                                                        Max(pw.MaxStep(2), pw.MaxStep(3)), ref wline);

                                                AddWLine(_sLin, wline, deflection);
                                                _empt = false;
                                            } // !RejetLigne
                                        } // PW points > 2
                                    } // done is True
                                } // dminiPointLigne > SeuildPointLigne
                            } // HasStartPoint
                        } // if TabPtDep[nbps2] == 0
                    } while (nbp > 5
                             && (nombreDePointsDeDepartDuCheminement < nbDePointsDeDepartDuChmLimit || !lignetrouvee)
                             && (nombreDePointsDeDepartDuCheminement - 3 < nbp || !(!lignetrouvee)));
                } // for( ls ...
            } // if nbLigSec >= 1
            //
            AdjustOnPeriodic(surf1, surf2, _sLin);
            //

            //--------------------------------------------------------------------
            //-- Calcul des parametres approches a partir des Zones De Tangence --
            //--------------------------------------------------------------------
            {
                double uminLig1, vminLig1, umaxLig1, vmaxLig1;
                double uminLig2, vminLig2, umaxLig2, vmaxLig2;

                uminLig1 = vminLig1 = uminLig2 = vminLig2 = RealLast();
                umaxLig1 = vmaxLig1 = umaxLig2 = vmaxLig2 = -uminLig1;

                // NbPointsInTangentZone always == 1 (eap)

                int z;
                for (z = 1; z <= nbTanZon; z++)
                {
                    int pz = 1;
                    interference.GetTangentZonePoint(z, pz, out _, out _, out _, out double u1, out double v1,
                                                     out double u2, out double v2);

                    if (u1 > umaxLig1)
                    {
                        umaxLig1 = u1;
                    }
                    if (v1 > vmaxLig1)
                    {
                        vmaxLig1 = v1;
                    }
                    if (u2 > umaxLig2)
                    {
                        umaxLig2 = u2;
                    }
                    if (v2 > vmaxLig2)
                    {
                        vmaxLig2 = v2;
                    }

                    if (u1 < uminLig1)
                    {
                        uminLig1 = u1;
                    }
                    if (v1 < vminLig1)
                    {
                        vminLig1 = v1;
                    }
                    if (u2 < uminLig2)
                    {
                        uminLig2 = u2;
                    }
                    if (v2 < vminLig2)
                    {
                        vminLig2 = v2;
                    }
                }

                for (z = 1; z <= nbTanZon; z++)
                {
                    int pz = 1;
                    interference.GetTangentZonePoint(z, pz, out _, out _, out _, out double u1, out double v1,
                                                     out double u2, out double v2);

                    startParams[1] = u1;
                    startParams[2] = v1;
                    startParams[3] = u2;
                    startParams[4] = v2;

                    //-----------------------------------------------------------------------
                    //-- Calcul du premier point de cheminement a partir du point approche --
                    //-----------------------------------------------------------------------
                    hasStartPoint = pw.PerformFirstPoint(startParams, ref startPOn2S);
                    if (hasStartPoint)
                    {
                        //-------------------------------------------------
                        //-- Un point a ete trouve                       --
                        //-- On verifie qu il n appartient pas           --
                        //--  a une ligne de cheminement deja calculee.  --
                        //-------------------------------------------------
                        nbLigCalculee = _sLin.Length();
                        dminiPointLigne = seuildPointLigne + seuildPointLigne;

                        for (int l = 1; (l <= nbLigCalculee) && (dminiPointLigne >= seuildPointLigne); l++)
                        {
                            WLine testwline = (WLine)_sLin.Value(l);

                            if (IsPointOnLine(startPOn2S, testwline, deflection))
                            {
                                dminiPointLigne = 0.0;
                            }
                        }

                        //-- Fin d exploration des lignes
                        if (dminiPointLigne > seuildPointLigne)
                        {
                            //---------------------------------------------------
                            //-- Le point de depart du nouveau cheminement     --
                            //-- n est present dans aucune ligne deja calculee.--
                            //---------------------------------------------------
                            pw.Perform(startParams, uminLig1, vminLig1, uminLig2, vminLig2, umaxLig1, vmaxLig1,
                                       umaxLig2, vmaxLig2);

                            if (pw.IsDone)
                            {
                                bool hasBeenAdded = false;
                                if (pw.PointCount > 2)
                                {
                                    const int minNbPoints = 40;
                                    if (pw.PointCount < minNbPoints)
                                    {
                                        hasBeenAdded = pw.SeekAdditionalPoints(surf1, surf2, minNbPoints);
                                    }

                                    //-----------------------------------------------
                                    //-- Verification a posteriori :
                                    //-- On teste si le point de depart et de fin de
                                    //-- la ligne de cheminement est present dans une
                                    //-- autre ligne .
                                    //-----------------------------------------------
                                    rejectLine = false;
                                    point3dDebut = pw.Value(1).Value();
                                    PntOn2S pointFin = pw.Value(pw.PointCount);
                                    point3dFin = pointFin.Value();

                                    for (ver = 1; (!rejectLine) && (ver <= nbLigCalculee); ver++)
                                    {
                                        WLine verwline = (WLine)_sLin.Value(ver);

                                        // Check end point if it is on existing line.
                                        // Start point is checked before.
                                        if (IsPointOnLine(pointFin, verwline, deflection))
                                        {
                                            rejectLine = true;
                                            break;
                                        }

                                        PntOn2S verPointDebut = verwline.Point(1);
                                        PntOn2S verPointFin = verwline.Point(verwline.NbPnts());
                                        if (point3dDebut.DistanceTo(verPointDebut.Value()) < tolTangency)
                                        {
                                            rejectLine = true;
                                        }
                                        else
                                        {
                                            if (point3dFin.DistanceTo(verPointFin.Value()) < tolTangency)
                                            {
                                                rejectLine = true;
                                            }
                                        }
                                    }

                                    if (!rejectLine)
                                    {
                                        WalkingTypeTrans trans1, trans2;
                                        Vector3D norm1, norm2;
                                        Vector3D tgline = new(pw.TangentAtLine(out int indextg));
                                        pw.Line().Value(indextg).ParametersOnS1(out double locu, out double locv);
                                        surf1.D1(locu, locv, out Point3D ptbid, out Vector3D d1u, out Vector3D d1v);
                                        norm1 = d1u.Crossed(d1v);
                                        pw.Line().Value(indextg).ParametersOnS2(out locu, out locv);
                                        surf2.D1(locu, locv, out ptbid, out d1u, out d1v);
                                        norm2 = d1u.Crossed(d1v);
                                        if (tgline.DotCross(norm2, norm1) > 0.0)
                                        {
                                            trans1 = WalkingTypeTrans.Out;
                                            trans2 = WalkingTypeTrans.In;
                                        }
                                        else
                                        {
                                            trans1 = WalkingTypeTrans.In;
                                            trans2 = WalkingTypeTrans.Out;
                                        }

                                        double tolTang = tolTangency;
                                        WLine wline = new(pw.Line(), false, trans1, trans2);
                                        wline.SetCreatingWayInfo(WLine.IntPatch_WLType.WLPrmPrm);
                                        wline.EnablePurging(!hasBeenAdded);
                                        // the method PutVertexOnLine can reduce the number of points in <wline>
                                        RstInt.PutVertexOnLine(wline, surf1, d1, surf2, true, tolTang);
                                        if (wline.NbPnts() < 2)
                                        {
                                            continue;
                                        }
                                        RstInt.PutVertexOnLine(wline, surf2, d2, surf1, false, tolTang);
                                        if (wline.NbPnts() < 2)
                                        {
                                            continue;
                                        }

                                        //---------------
                                        if (wline.VertexCount == 0)
                                        {
                                            AddEndVertices(wline, pw, point3dDebut, point3dFin, tolTang);
                                        }

                                        //---------------
                                        AddWLine(_sLin, wline, deflection);
                                        _empt = false;
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

        PerformOnVoxels(surf1, d1, surf2, d2, tolTangency, epsilon, deflection, increment, nbU1, nbV1, nbU2, nbV2);
    }

    // The rest of OCCT's Perform when a surface has more than 2,500 samples: walking lines started from the voxel
    // search's points. A separate method only because C# can't redeclare the locals of the block above.
    private void PerformOnVoxels(SurfaceAdaptorBase surf1, TopolTool d1, SurfaceAdaptorBase surf2,
                                 TopolTool d2, double tolTangency, double epsilon, double deflection,
                                 double increment, int nbU1, int nbV1, int nbU2, int nbV2)
    {
        LineOn2S lOn2S = new();
        PointDepart(lOn2S, surf1, nbU1, nbV1, surf2, nbU2, nbV2);
        _empt = true;
        _done = true;
        // OCCT: SLin isn't cleared here, whatever ClearFlag says (occt-issues.md, 5D)

        int nbLigCalculee = 0;
        double u1, u2, v1, v2;

        NCollection_Array1<double> startParams = new(1, 4);
        int maxOscill = nbU1;
        if (maxOscill < nbU2)
        {
            maxOscill = nbU2;
        }
        if (maxOscill < nbV1)
        {
            maxOscill = nbV1;
        }
        if (maxOscill < nbV2)
        {
            maxOscill = nbV2;
        }
        _ = maxOscill; // read only by OCCT's commented-out step reduction

        double nIncrement = increment;
        // if(MaxOscill>10)
        // nIncrement/=0.5*MaxOscill;

        PWalking pw = new(surf1, surf2, tolTangency, epsilon, deflection, nIncrement);
        double seuildPointLigne = 15.0 * increment * increment; //-- 10 est insuffisant
        double dminiPointLigne;
        bool hasStartPoint, rejetLigne;
        PntOn2S startPOn2S = new();
        int ver;
        Point3D point3dDebut, point3dFin;

        //------------------------------------------------------------
        //-- Calcul des parametres approches a partir des Zones De Tangence --
        //--------------------------------------------------------------------
        int nbTanZon = lOn2S.PointCount;
        for (int z = 1; z <= nbTanZon; z++)
        {
            PntOn2S pOn2S = lOn2S.Value(z);
            pOn2S.Parameters(out u1, out v1, out u2, out v2);
            startParams[1] = u1;
            startParams[2] = v1;
            startParams[3] = u2;
            startParams[4] = v2;

            //-----------------------------------------------------------------------
            //-- Calcul du premier point de cheminement a partir du point approche --
            //-----------------------------------------------------------------------
            hasStartPoint = pw.PerformFirstPoint(startParams, ref startPOn2S);
            if (hasStartPoint)
            {
                //-------------------------------------------------
                //-- Un point a ete trouve                       --
                //-- On verifie qu il n appartient pas           --
                //--  a une ligne de cheminement deja calculee.  --
                //-------------------------------------------------
                nbLigCalculee = _sLin.Length();
                dminiPointLigne = seuildPointLigne + seuildPointLigne;

                for (int l = 1; (l <= nbLigCalculee) && (dminiPointLigne >= seuildPointLigne); l++)
                {
                    WLine testwline = (WLine)_sLin.Value(l);

                    if (IsPointOnLine(startPOn2S, testwline, deflection))
                    {
                        dminiPointLigne = 0.0;
                    }
                }

                //-- Fin d exploration des lignes
                if (dminiPointLigne > seuildPointLigne)
                {
                    //---------------------------------------------------
                    //-- Le point de depart du nouveau cheminement     --
                    //-- n est present dans aucune ligne deja calculee.--
                    //---------------------------------------------------
                    pw.Perform(startParams);
                    if (pw.IsDone)
                    {
                        if (pw.PointCount > 2)
                        {
                            //-----------------------------------------------
                            //-- Verification a posteriori :
                            //-- On teste si le point de depart et de fin de
                            //-- la ligne de cheminement est present dans une
                            //-- autre ligne .
                            //-----------------------------------------------
                            rejetLigne = false;
                            point3dDebut = pw.Value(1).Value();
                            PntOn2S pointFin = pw.Value(pw.PointCount);
                            point3dFin = pointFin.Value();

                            for (ver = 1; ver <= nbLigCalculee; ver++)
                            {
                                WLine verwline = (WLine)_sLin.Value(ver);

                                // Check end point if it is on existing line.
                                // Start point is checked before.
                                if (IsPointOnLine(pointFin, verwline, deflection))
                                {
                                    rejetLigne = true;
                                    break;
                                }

                                PntOn2S verPointDebut = verwline.Point(1);
                                PntOn2S verPointFin = verwline.Point(verwline.NbPnts());
                                if ((point3dDebut.DistanceTo(verPointDebut.Value()) < tolTangency)
                                    || (point3dFin.DistanceTo(verPointFin.Value()) < tolTangency))
                                {
                                    rejetLigne = true;
                                    break;
                                }
                            }

                            if (rejetLigne)
                            {
                                DublicateOfLinesProcessing(pw, ver, _sLin, ref rejetLigne);
                            }

                            if (!rejetLigne)
                            {
                                WalkingTypeTrans trans1, trans2;
                                Vector3D norm1, norm2;
                                Vector3D tgline = new(pw.TangentAtLine(out int indextg));
                                pw.Line().Value(indextg).ParametersOnS1(out double locu, out double locv);
                                surf1.D1(locu, locv, out Point3D ptbid, out Vector3D d1u, out Vector3D d1v);
                                norm1 = d1u.Crossed(d1v);
                                pw.Line().Value(indextg).ParametersOnS2(out locu, out locv);
                                surf2.D1(locu, locv, out ptbid, out d1u, out d1v);
                                norm2 = d1u.Crossed(d1v);
                                if (tgline.DotCross(norm2, norm1) > 0.0)
                                {
                                    trans1 = WalkingTypeTrans.Out;
                                    trans2 = WalkingTypeTrans.In;
                                }
                                else
                                {
                                    trans1 = WalkingTypeTrans.In;
                                    trans2 = WalkingTypeTrans.Out;
                                }

                                double tolTang = tolTangency;
                                WLine wline = new(pw.Line(), false, trans1, trans2);
                                wline.SetCreatingWayInfo(WLine.IntPatch_WLType.WLPrmPrm);

                                // the method PutVertexOnLine can reduce the number of points in <wline>
                                RstInt.PutVertexOnLine(wline, surf1, d1, surf2, true, tolTang);
                                if (wline.NbPnts() < 2)
                                {
                                    continue;
                                }
                                RstInt.PutVertexOnLine(wline, surf2, d2, surf1, false, tolTang);
                                if (wline.NbPnts() < 2)
                                {
                                    continue;
                                }

                                //---------------
                                if (wline.VertexCount == 0)
                                {
                                    AddEndVertices(wline, pw, point3dDebut, point3dFin, tolTang);
                                }

                                //---------------
                                AddWLine(_sLin, wline, deflection);
                                _empt = false;
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
        private readonly int _nbi;
        private readonly byte[] _p1DS2;
        private readonly byte[] _p2DS1;
        private readonly int[] _ip1;
        private readonly int[] _ip2;
        private readonly Point3D[] _p1;
        private readonly Point3D[] _p2;

        //----------------------------------------C-tor
        public IntPatch_InfoPD(int aNBI)
        {
            int nbi2, i, j;
            _nbi = aNBI;
            //
            nbi2 = aNBI * aNBI;
            _p1DS2 = new byte[nbi2];
            _p2DS1 = new byte[nbi2];
            _ip1 = new int[nbi2];
            _ip2 = new int[nbi2];
            _p1 = new Point3D[nbi2];
            _p2 = new Point3D[nbi2];
            //
            for (i = 0; i < _nbi; ++i)
            {
                for (j = 0; j < _nbi; ++j)
                {
                    XP1DS2(i, j) = 0;
                    XP2DS1(i, j) = 0;
                    XIP1(i, j) = 0;
                    XIP2(i, j) = 0;
                    XP1(i, j) = XP1(i, j).WithCoord(0.0, 0.0, 0.0);
                    XP2(i, j) = XP2(i, j).WithCoord(0.0, 0.0, 0.0);
                }
            }
        }

        //---------------------------------------- Index
        public int Index(int i, int j) => i * _nbi + j;

        //---------------------------------------- NBI
        public int NBI() => _nbi;

        //----------------------------------------xP1DS2
        public ref byte XP1DS2(int i, int j) => ref _p1DS2[Index(i, j)];

        //----------------------------------------xP2DS1
        public ref byte XP2DS1(int i, int j) => ref _p2DS1[Index(i, j)];

        //----------------------------------------xIP1
        public ref int XIP1(int i, int j) => ref _ip1[Index(i, j)];

        //----------------------------------------xIP2
        public ref int XIP2(int i, int j) => ref _ip2[Index(i, j)];

        //----------------------------------------xP1
        public ref Point3D XP1(int i, int j) => ref _p1[Index(i, j)];

        //----------------------------------------xP1
        public ref Point3D XP2(int i, int j) => ref _p2[Index(i, j)];
    }

    // modified by NIZNHY-PKV Tue May 24 11:38:55 2011t
    //=================================================================================================

    /// <summary>
    /// Fills <paramref name="lineOn2S"/> with start points for the walking: the surfaces are sampled (15 times their
    /// sampling, at most 200 by 200), the samples' triangles are marked in two 128³ voxel grids over the common box,
    /// and each voxel marked in both gives a point, with the parameters of samples in it (or the mean of the three
    /// nearest samples on each surface).
    /// </summary>
    public void PointDepart(LineOn2S lineOn2S, SurfaceAdaptorBase s1, int SU_1, int SV_1,
                            SurfaceAdaptorBase s2, int SU_2, int SV_2)
    {
        int i, j, xNBI;
        // modified by NIZNHY-PKV Tue May 24 11:37:38 2011f
        xNBI = 200;
        IntPatch_InfoPD ipd = new(xNBI);
        // modified by NIZNHY-PKV Wed May 25 06:47:12 2011t
        int iC15, su1, sv1, su2, sv2;
        double u0, u1, v0, v1, u, v;
        double resu0, resv0;
        double du1, du2, dv1, dv2, dmaxOn1, dmaxOn2;
        double x0, y0, z0, x1, y1, z1, d;
        Box box1 = new(), box2 = new();
        //
        iC15 = 15;
        su1 = iC15 * SU_1;
        sv1 = iC15 * SV_1;
        su2 = iC15 * SU_2;
        sv2 = iC15 * SV_2;
        //
        if (xNBI < su1)
        {
            su1 = xNBI;
        }
        if (xNBI < sv1)
        {
            sv1 = xNBI;
        }
        if (xNBI < su2)
        {
            su2 = xNBI;
        }
        if (xNBI < sv2)
        {
            sv2 = xNBI;
        }
        //
        u0 = s1.FirstUParameter();
        u1 = s1.LastUParameter();
        v0 = s1.FirstVParameter();
        v1 = s1.LastVParameter();
        //
        resu0 = u0;
        resv0 = v0;
        //
        dmaxOn1 = 0.0;
        dmaxOn2 = 0.0;
        //-----
        du1 = (u1 - u0) / (su1 - 1);
        dv1 = (v1 - v0) / (sv1 - 1);
        for (u = u0, i = 0; i < su1; i++, u += du1)
        {
            for (v = v0, j = 0; j < sv1; v += dv1, j++)
            {
                ipd.XP1(i, j) = s1.Value(u, v);
                box1.Add(ipd.XP1(i, j));
                if (i > 0 && j > 0)
                {
                    ipd.XP1(i, j).Coord(out x0, out y0, out z0);
                    ipd.XP1(i - 1, j - 1).Coord(out x1, out y1, out z1);
                    //
                    d = Math.Abs(x1 - x0) + Math.Abs(y1 - y0) + Math.Abs(z1 - z0);
                    if (d > dmaxOn1)
                    {
                        dmaxOn1 = d;
                    }
                }
            }
        }
        box1.Enlarge(1.0e-8);
        //
        u0 = s2.FirstUParameter();
        u1 = s2.LastUParameter();
        v0 = s2.FirstVParameter();
        v1 = s2.LastVParameter();
        //
        du2 = (u1 - u0) / (su2 - 1);
        dv2 = (v1 - v0) / (sv2 - 1);
        for (u = u0, i = 0; i < su2; i++, u += du2)
        {
            for (v = v0, j = 0; j < sv2; v += dv2, j++)
            {
                ipd.XP2(i, j) = s2.Value(u, v);
                box2.Add(ipd.XP2(i, j));
                if (i > 0 && j > 0)
                {
                    ipd.XP2(i, j).Coord(out x0, out y0, out z0);
                    ipd.XP2(i - 1, j - 1).Coord(out x1, out y1, out z1);
                    d = Math.Abs(x1 - x0) + Math.Abs(y1 - y0) + Math.Abs(z1 - z0);
                    if (d > dmaxOn2)
                    {
                        dmaxOn2 = d;
                    }
                }
            }
        }
        box2.Enlarge(1.0e-8);
        //--------
        //
        if (box1.IsOut(box2))
        {
            return;
        }
        //
        int nbPG;
        double x10, y10, z10, x11, y11, z11;
        double x20, y20, z20, x21, y21, z21;
        double dx, dy, dz, dmax;
        double dx2, dy2, dz2;
        //
        box1.Get(out x10, out y10, out z10, out x11, out y11, out z11);
        box2.Get(out x20, out y20, out z20, out x21, out y21, out z21);
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

        nbPG = PointsGrilleCount;
        dx = (x1 - x0) / nbPG;
        dy = (y1 - y0) / nbPG;
        dz = (z1 - z0) / nbPG;
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
        PrmPrmIntersectionT3Bits m1 = new(_BASE);
        PrmPrmIntersectionT3Bits m2 = new(_BASE);
        //
        for (i = 0; i < su1; i++)
        {
            for (j = 0; j < sv1; j++)
            {
                ipd.XIP1(i, j) = -1;
                Point3D p = ipd.XP1(i, j);
                ipd.XP1DS2(i, j) = (byte)CodeReject(x20, y20, z20, x21, y21, z21, p.X, p.Y, p.Z);
                int ix = CppTruncate((p.X - x0 + dx2) / dx);
                if (DansGrille(ix) != 0)
                {
                    int iy = CppTruncate((p.Y - y0 + dy2) / dy);
                    if (DansGrille(iy) != 0)
                    {
                        int iz = CppTruncate((p.Z - z0 + dz2) / dz);
                        if (DansGrille(iz) != 0)
                        {
                            ipd.XIP1(i, j) = GrilleInteger(ix, iy, iz);
                        }
                    }
                }
            }
        }
        //-- std::cout<<" Grille  du 1 fini "<<std::endl;
        for (i = 0; i < su2; i++)
        {
            for (j = 0; j < sv2; j++)
            {
                ipd.XIP2(i, j) = -1;
                Point3D p = ipd.XP2(i, j);
                ipd.XP2DS1(i, j) = (byte)CodeReject(x10, y10, z10, x11, y11, z11, p.X, p.Y, p.Z);
                int ix = CppTruncate((p.X - x0 + dx2) / dx);
                if (DansGrille(ix) != 0)
                {
                    int iy = CppTruncate((p.Y - y0 + dy2) / dy);
                    if (DansGrille(iy) != 0)
                    {
                        int iz = CppTruncate((p.Z - z0 + dz2) / dz);
                        if (DansGrille(iz) != 0)
                        {
                            ipd.XIP2(i, j) = GrilleInteger(ix, iy, iz);
                        }
                    }
                }
            }
        }
        //
        for (i = 0; i < su1 - 1; i += 1)
        {
            for (j = 0; j < sv1 - 1; j += 1)
            {
                if (!(((ipd.XP1DS2(i, j) & ipd.XP1DS2(i + 1, j)) != 0)
                      || ((ipd.XP1DS2(i, j) & ipd.XP1DS2(i + 1, j + 1)) != 0)))
                {
                    Remplit(ipd.XIP1(i, j), ipd.XIP1(i + 1, j), ipd.XIP1(i + 1, j + 1), m1);
                }
                if (!(((ipd.XP1DS2(i, j) & ipd.XP1DS2(i, j + 1)) != 0)
                      || ((ipd.XP1DS2(i, j) & ipd.XP1DS2(i + 1, j + 1)) != 0)))
                {
                    Remplit(ipd.XIP1(i, j), ipd.XIP1(i, j + 1), ipd.XIP1(i + 1, j + 1), m1);
                }
            }
        }
        //
        for (i = 0; i < su2 - 1; i += 1)
        {
            for (j = 0; j < sv2 - 1; j += 1)
            {
                if (!(((ipd.XP2DS1(i, j) & ipd.XP2DS1(i + 1, j)) != 0)
                      || ((ipd.XP2DS1(i, j) & ipd.XP2DS1(i + 1, j + 1)) != 0)))
                {
                    Remplit(ipd.XIP2(i, j), ipd.XIP2(i + 1, j), ipd.XIP2(i + 1, j + 1), m2);
                }
                if (!(((ipd.XP2DS1(i, j) & ipd.XP2DS1(i, j + 1)) != 0)
                      || ((ipd.XP2DS1(i, j) & ipd.XP2DS1(i + 1, j + 1)) != 0)))
                {
                    Remplit(ipd.XIP2(i, j), ipd.XIP2(i, j + 1), ipd.XIP2(i + 1, j + 1), m2);
                }
            }
        }
        //
        m1.ResetAnd();
        m2.ResetAnd();
        //
        int newind = 0;
        int ok = 0;
        int indicepointtraite = 0;
        int k, nu, nv;
        // The else branch's local arrays (u3 and v3 are zeroed where OCCT declares them)
        Span<double> dist3 = stackalloc double[3];
        Span<double> u3 = stackalloc double[3];
        Span<double> v3 = stackalloc double[3];
        //
        do
        {
            indicepointtraite--;
            ok = m1.And(m2, ref newind);
            if (ok != 0)
            {
                IntegerGrille(newind, out i, out j, out k);
                int nb = 0;
                int lim = 3;
                if (DansGrille(i - 1) != 0 && DansGrille(j - 1) != 0 && DansGrille(k - 1) != 0
                    && DansGrille(i + 1) != 0 && DansGrille(j + 1) != 0 && DansGrille(k + 1) != 0)
                {
                    int si, sj, sk;
                    for (si = -1; si <= 1 && nb < lim; si++)
                    {
                        for (sj = -1; sj <= 1 && nb < lim; sj++)
                        {
                            for (sk = -1; sk <= 1 && nb < lim; sk++)
                            {
                                int lu = GrilleInteger(i + si, j + sj, k + sk);
                                if (m1.Val(lu) != 0 && m2.Val(lu) != 0)
                                {
                                    nb++;
                                }
                            }
                        }
                    }
                    if (nb >= lim)
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
                                        m1.Raz(lu);
                                    }
                                }
                            }
                        }
                    }
                }
                //
                Point3D p = new(dx * i + x0, dy * j + y0, dz * k + z0);
                //
                int nu1 = -1, nu2 = -1;
                int nv1 = 0, nv2 = 0;
                for (nu = 0; nu1 < 0 && nu < su1; nu++)
                {
                    for (nv = 0; nu1 < 0 && nv < sv1; nv++)
                    {
                        if (ipd.XIP1(nu, nv) == (int)newind)
                        {
                            ipd.XIP1(nu, nv) = indicepointtraite;
                            nu1 = nu;
                            nv1 = nv;
                        }
                    }
                }
                if (nu1 >= 0)
                {
                    for (nu = 0; nu2 < 0 && nu < su2; nu++)
                    {
                        for (nv = 0; nu2 < 0 && nv < sv2; nv++)
                        {
                            if (ipd.XIP2(nu, nv) == (int)newind)
                            {
                                ipd.XIP2(nu, nv) = indicepointtraite;
                                nu2 = nu;
                                nv2 = nv;
                            }
                        }
                    }
                }
                if (nu1 >= 0 && nu2 >= 0)
                {
                    PntOn2S pOn2S = new();
                    pOn2S.SetValue(p,
                                   s1.FirstUParameter() + nu1 * du1,
                                   s1.FirstVParameter() + nv1 * dv1,
                                   s2.FirstUParameter() + nu2 * du2,
                                   s2.FirstVParameter() + nv2 * dv2);
                    lineOn2S.Add(pOn2S);
                }
                else
                {
                    //-- aucun point du triangle n a ete trouve assez proche
                    //-- on recherche les 3 points les plus proches de P
                    //-- dans chacun des tableaux
                    u3.Clear();
                    v3.Clear();
                    dist3[0] = dist3[1] = dist3[2] = RealLast();
                    for (u = resu0, i = 0; i < su1; i++, u += du1)
                    {
                        for (v = resv0, j = 0; j < sv1; v += dv1, j++)
                        {
                            //-- On place les 3 meilleures valeurs dans Dist1,Dist2,Dist3
                            double t = ipd.XP1(i, j).SquareDistanceTo(p);
                            SortThreeNearest(dist3, u3, v3);
                            //-- la plus grande valeur est dans Dist3[0]
                            if (t < dist3[0])
                            {
                                dist3[0] = t;
                                u3[0] = u;
                                v3[0] = v;
                            }
                        }
                    }
                    //
                    double U1_3 = (u3[0] + u3[1] + u3[2]) / 3.0;
                    double V1_3 = (v3[0] + v3[1] + v3[2]) / 3.0;

                    dist3[0] = dist3[1] = dist3[2] = RealLast();
                    for (u = u0, i = 0; i < su2; i++, u += du2)
                    {
                        for (v = v0, j = 0; j < sv2; v += dv2, j++)
                        {
                            //-- On place les 3 meilleures valeurs dans Dist1,Dist2,Dist3
                            double t = ipd.XP2(i, j).SquareDistanceTo(p);
                            SortThreeNearest(dist3, u3, v3);
                            //-- la plus grande valeur est dans Dist3[0]
                            if (t < dist3[0])
                            {
                                dist3[0] = t;
                                u3[0] = u;
                                v3[0] = v;
                            }
                        }
                    }
                    //
                    double U2_3 = (u3[0] + u3[1] + u3[2]) / 3.0;
                    double V2_3 = (v3[0] + v3[1] + v3[2]) / 3.0;
                    //
                    PntOn2S pOn2S = new();
                    pOn2S.SetValue(p, U1_3, V1_3, U2_3, V2_3);
                    lineOn2S.Add(pOn2S);
                }
            }
        } while (ok != 0);
    }

    // The three swaps OCCT repeats verbatim in both nearest-sample searches of PointDepart: puts the largest of the
    // three distances (and its parameters) in the first place.
    private static void SortThreeNearest(Span<double> dist3, Span<double> u3, Span<double> v3)
    {
        //-- On remplace la plus grande valeur ds Dist[.] par la val courante
        if (dist3[0] < dist3[1])
        {
            double z;
            z = dist3[0];
            dist3[0] = dist3[1];
            dist3[1] = z;
            z = u3[0];
            u3[0] = u3[1];
            u3[1] = z;
            z = v3[0];
            v3[0] = v3[1];
            v3[1] = z;
        }
        if (dist3[1] < dist3[2])
        {
            double z;
            z = dist3[1];
            dist3[1] = dist3[2];
            dist3[2] = z;
            z = u3[1];
            u3[1] = u3[2];
            u3[2] = z;
            z = v3[1];
            v3[1] = v3[2];
            v3[2] = z;
        }
        if (dist3[0] < dist3[1])
        {
            double z;
            z = dist3[0];
            dist3[0] = dist3[1];
            dist3[1] = z;
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
    private static int CppTruncate(double value) =>
        (value > -2147483649.0 && value < 2147483648.0) ? (int)value : int.MinValue;
}
