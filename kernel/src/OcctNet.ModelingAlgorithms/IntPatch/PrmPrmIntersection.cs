// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// The intersection of two bi-parametric surfaces: start points from an interference of the surfaces' polyhedra
/// (IntPatch_InterferencePolyhedron), of their triangulations (IntPolyh_Intersection) or of a voxel grid
/// (<see cref="PointDepart"/>), or given, then walking lines between them (IntWalk_PWalking), with their vertices on
/// the surfaces' restrictions (IntPatch_RstInt).
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <c>Perform</c> fills it, and its lines are the caller's to edit (decision 609). The
/// file-static functions of OCCT's .cxx are private static methods (IntPatch_PrmPrmIntersection.Tools.cs), and the
/// voxel search with its file-local class IntPatch_InfoPD is in IntPatch_PrmPrmIntersection.PointDepart.cs. Each
/// line keeps the IntSurf_LineOn2S of the walking that made it, as OCCT's: the walking makes a new one at each
/// <c>Perform</c>. OCCT's <c>Perform</c> with the ListOfPnts appends to the caller's list the points it adds on
/// closed surfaces, as the port does. The intersection is serial and the same every run.
/// </remarks>
public partial class PrmPrmIntersection
{
    private const int _DECAL = 7;
    private const int _DECAL2 = 14;
    private const int _BASE = 128;
    private const int _BASEM1 = 127;

    private bool _done;
    private bool _empt;
    private readonly NCollection_Sequence<PatchLine> _sLin = new();

    /// <summary>An empty intersection.</summary>
    public PrmPrmIntersection()
    {
        _done = false;
        _empt = true;
    }

    //=================================================================================================

    /// <summary>The self-intersection of <paramref name="surf1"/>, on a polyhedron of its domain's sampling.</summary>
    public void Perform(SurfaceAdaptorBase surf1, TopolTool d1, double tolTangency, double epsilon,
                        double deflection, double increment)
    {
        Polyhedron poly1 = new(surf1, d1.NbSamplesU(), d1.NbSamplesV());
        Perform(surf1, poly1, d1, tolTangency, epsilon, deflection, increment);
    }

    //=================================================================================================

    /// <summary>
    /// The intersection of <paramref name="surf1"/>, approximated by <paramref name="poly1"/>, and
    /// <paramref name="surf2"/>, whose polyhedron is computed.
    /// </summary>
    public void Perform(SurfaceAdaptorBase surf1, Polyhedron poly1, TopolTool d1,
                        SurfaceAdaptorBase surf2, TopolTool d2, double tolTangency, double epsilon,
                        double deflection, double increment)
    {
        Polyhedron poly2 = new(surf2);
        Perform(surf1, poly1, d1, surf2, poly2, d2, tolTangency, epsilon, deflection, increment);
    }

    //=================================================================================================

    /// <summary>
    /// The intersection of <paramref name="surf1"/>, whose polyhedron is computed, and <paramref name="surf2"/>,
    /// approximated by <paramref name="poly2"/>.
    /// </summary>
    public void Perform(SurfaceAdaptorBase surf1, TopolTool d1, SurfaceAdaptorBase surf2,
                        Polyhedron poly2, TopolTool d2, double tolTangency, double epsilon,
                        double deflection, double increment)
    {
        Polyhedron poly1 = new(surf1);
        Perform(surf1, poly1, d1, surf2, poly2, d2, tolTangency, epsilon, deflection, increment);
    }

    //=================================================================================================

    /// <summary>
    /// The intersection of <paramref name="surf1"/> and <paramref name="surf2"/>, approximated by the polyhedra
    /// <paramref name="poly1"/> and <paramref name="poly2"/>: walking lines started from the section lines and
    /// tangent zones of the polyhedra's interference.
    /// </summary>
    public void Perform(SurfaceAdaptorBase surf1, Polyhedron poly1, TopolTool d1,
                        SurfaceAdaptorBase surf2, Polyhedron poly2, TopolTool d2,
                        double tolTangency, double epsilon, double deflection, double increment)
    {
        InterferencePolyhedron interference = new(poly1, poly2);
        _empt = true;
        _done = true;
        _sLin.Clear();

        int nbLigSec = interference.SectionLineCount;
        int nbTanZon = interference.TangentZoneCount;

        int nbLigCalculee = 0;

        double u1 = 0.0, u2 = 0.0, v1 = 0.0, v2 = 0.0;
        double pu1, pu2, pv1, pv2;

        NCollection_Array1<double> startParams = new(1, 4);

        PWalking pw = new(surf1, surf2, tolTangency, epsilon, deflection, increment);

        double seuildPointLigne = 15.0 * increment * increment; //-- 10 est insuffisant
        double incidence;
        double dminiPointLigne;

        bool hasStartPoint, rejetLigne;

        PntOn2S startPOn2S = new();

        int ver;

        Point3D point3dDebut, point3dFin;

        if (nbLigSec >= 1)
        {
            int[] tabL = new int[nbLigSec + 1];
            int ls;
            for (ls = 1; ls <= nbLigSec; ls++)
            {
                tabL[ls] = ls;
            }

            bool triok;
            do
            {
                triok = true;
                for (int b = 2; b <= nbLigSec; b++)
                {
                    int nb_B = interference.LineValue(tabL[b]).NumberOfPoints();
                    int nb_A = interference.LineValue(tabL[b - 1]).NumberOfPoints();
                    if (nb_B > nb_A)
                    {
                        int tyu = tabL[b];
                        tabL[b] = tabL[b - 1];
                        tabL[b - 1] = tyu;
                        triok = false;
                    }
                }
            } while (!triok);

            for (ls = 1; ls <= nbLigSec; ls++)
            {
                SectionLine lineSec = interference.LineValue(tabL[ls]);
                int nbp = lineSec.NumberOfPoints();

                int[] tabPtDep = new int[nbp + 1];
                int ilig;
                for (ilig = 1; ilig <= nbp; ilig++)
                {
                    tabPtDep[ilig] = 0;
                }

                double uminLig1 = 0.0, vminLig1 = 0.0, umaxLig1, vmaxLig1;
                double uminLig2 = 0.0, vminLig2 = 0.0, umaxLig2, vmaxLig2;

                SectionPointToParameters(lineSec.GetPoint(1), poly1, poly2, ref uminLig1, ref vminLig1, ref uminLig2,
                                         ref vminLig2);

                umaxLig1 = uminLig1;
                vmaxLig1 = vminLig1;
                umaxLig2 = uminLig2;
                vmaxLig2 = vminLig2;

                for (ilig = 2; ilig <= nbp; ilig++)
                {
                    SectionPointToParameters(lineSec.GetPoint(ilig), poly1, poly2, ref u1, ref v1, ref u2, ref v2);

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

                int nbps2 = (nbp > 3) ? (nbp / 2) : 1;
                int nombreDePointsDeDepartDuCheminement = 0;
                int indicePointdeDepart1 = 0, indicePointdeDepart2 = 0;
                bool lignetrouvee = false;

                do
                {
                    nombreDePointsDeDepartDuCheminement++;
                    if (nombreDePointsDeDepartDuCheminement == 1)
                    {
                        incidence = 3.0;
                        int nbp1_4 = nbp / 4;
                        int nbp3_4 = nbp - nbp1_4;

                        int nsp;
                        for (nsp = nbp / 2; nsp < nbp3_4; nsp++)
                        {
                            double currentIncidence = lineSec.GetPoint(nsp).Incidence();
                            if (currentIncidence < incidence)
                            {
                                nbps2 = nsp;
                                incidence = 0.9 * currentIncidence;
                            }
                        }

                        for (nsp = nbp / 2; nsp > nbp1_4; nsp--)
                        {
                            double currentIncidence = lineSec.GetPoint(nsp).Incidence();
                            if (currentIncidence < incidence)
                            {
                                nbps2 = nsp;
                                incidence = 0.9 * currentIncidence;
                            }
                        }

                        if (nbp < 3)
                        {
                            nombreDePointsDeDepartDuCheminement = 3;
                        }

                        indicePointdeDepart1 = nbps2;
                    }
                    else if (nombreDePointsDeDepartDuCheminement == 2)
                    {
                        if (indicePointdeDepart1 == 1)
                        {
                            nbps2 = nbp / 2;
                            indicePointdeDepart2 = nbps2;
                        }
                        else
                        {
                            nbps2 = 1;
                            indicePointdeDepart2 = 1;
                        }
                    }
                    else if (nombreDePointsDeDepartDuCheminement == 3)
                    {
                        if (indicePointdeDepart1 == nbp)
                        {
                            nbps2 = (indicePointdeDepart1 + indicePointdeDepart2) / 2;
                        }
                        else
                        {
                            nbps2 = nbp;
                        }
                    }
                    else
                    {
                        nbps2 = nombreDePointsDeDepartDuCheminement - 3;
                        nombreDePointsDeDepartDuCheminement++;
                    }

                    if (tabPtDep[nbps2] == 0)
                    {
                        tabPtDep[nbps2] = 1;
                        SectionPointToParameters(lineSec.GetPoint(nbps2), poly1, poly2, ref u1, ref v1, ref u2, ref v2);

                        startParams[1] = u1;
                        startParams[2] = v1;
                        startParams[3] = u2;
                        startParams[4] = v2;

                        hasStartPoint = pw.PerformFirstPoint(startParams, ref startPOn2S);
                        dminiPointLigne = seuildPointLigne + seuildPointLigne;

                        if (hasStartPoint)
                        {
                            startPOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
                            nbLigCalculee = _sLin.Length();
                            int l;
                            for (l = 1; (l <= nbLigCalculee) && (dminiPointLigne >= seuildPointLigne); l++)
                            {
                                WLine testwline = (WLine)_sLin.Value(l);

                                if (IsPointOnLine(startPOn2S, testwline, deflection))
                                {
                                    dminiPointLigne = 0.0;
                                }
                            } // for ( l ...

                            if (dminiPointLigne > seuildPointLigne)
                            {
                                pw.Perform(startParams, uminLig1, vminLig1, uminLig2, vminLig2, umaxLig1, vmaxLig1,
                                           umaxLig2, vmaxLig2);
                                if (pw.IsDone)
                                {
                                    if (pw.PointCount > 2)
                                    {
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
                                            if ((point3dDebut.Distance(verPointDebut.Value()) <= tolTangency)
                                                && (point3dFin.Distance(verPointFin.Value()) <= tolTangency))
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
                                            // Calculation transition
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

                                            if (wline.VertexCount == 0)
                                            {
                                                AddEndVertices(wline, pw, point3dDebut, point3dFin, tolTang);
                                            }

                                            lignetrouvee = true;
                                            AddWLine(_sLin, wline, deflection);
                                            _empt = false;
                                        } // !RejetLigne
                                    } // PW.NbPoints()>2
                                } // done is True
                            } // dminiPointLigne > SeuildPointLigne
                        } // HasStartPoint
                    } // TabPtDep[nbps2]==0
                } while (nbp > 5
                         && (((nombreDePointsDeDepartDuCheminement < 3) || !lignetrouvee)
                             && ((nombreDePointsDeDepartDuCheminement - 3 < nbp) || lignetrouvee)));
            } // for( ls ...
        } // nbLigSec >= 1

        {
            double uminLig1, vminLig1, umaxLig1, vmaxLig1;
            double uminLig2, vminLig2, umaxLig2, vmaxLig2;

            uminLig1 = vminLig1 = uminLig2 = vminLig2 = RealLast();
            umaxLig1 = vmaxLig1 = umaxLig2 = vmaxLig2 = -uminLig1;

            int z;
            for (z = 1; z <= nbTanZon; z++)
            {
                TangentZone tangentZone = interference.ZoneValue(z);
                for (int pz = 1; pz <= tangentZone.NumberOfPoints(); pz++)
                {
                    SectionPointToParameters(tangentZone.GetPoint(pz), poly1, poly2, ref u1, ref v1, ref u2, ref v2);

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
            }

            for (z = 1; z <= nbTanZon; z++)
            {
                TangentZone tangentZone = interference.ZoneValue(z);
                int pz;
                for (pz = 1; pz <= tangentZone.NumberOfPoints(); pz++)
                {
                    SectionPointToParameters(tangentZone.GetPoint(pz), poly1, poly2, ref u1, ref v1, ref u2, ref v2);

                    startParams[1] = u1;
                    startParams[2] = v1;
                    startParams[3] = u2;
                    startParams[4] = v2;

                    hasStartPoint = pw.PerformFirstPoint(startParams, ref startPOn2S);
                    if (hasStartPoint)
                    {
                        startPOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
                        nbLigCalculee = _sLin.Length();
                        dminiPointLigne = seuildPointLigne + seuildPointLigne;
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
                            if (pw.IsDone)
                            {
                                if (pw.PointCount > 2)
                                {
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
                                        if ((point3dDebut.Distance(verPointDebut.Value()) < tolTangency)
                                            || (point3dFin.Distance(verPointFin.Value()) < tolTangency))
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

                                        if (wline.VertexCount == 0)
                                        {
                                            AddEndVertices(wline, pw, point3dDebut, point3dFin, tolTang);
                                        }

                                        AddWLine(_sLin, wline, deflection);
                                        _empt = false;
                                    } // if !RejetLigne
                                } // PW.NbPoints()>2
                            } // done is True
                        } // dminiPointLigne > SeuildPointLigne
                    } // HasStartPoint
                } // for( pz ...
            } // for( z ...
        }
    }

    //=================================================================================================

    /// <summary>
    /// The self-intersection of <paramref name="surf1"/>, approximated by <paramref name="poly1"/>: walking lines
    /// started from the polyhedron's self-interference, keeping the parts away from the diagonal (U1, V1) = (U2, V2).
    /// </summary>
    public void Perform(SurfaceAdaptorBase surf1, Polyhedron poly1, TopolTool d1, double tolTangency,
                        double epsilon, double deflection, double increment)
    {
        InterferencePolyhedron interference = new(poly1);
        _empt = true;
        _done = true;
        _sLin.Clear();

        int nbLigSec = interference.SectionLineCount;
        int nbTanZon = interference.TangentZoneCount;

        int nbPntOn2SOnLine;
        int nbLigCalculee = 0;

        double u1 = 0.0, u2 = 0.0, v1 = 0.0, v2 = 0.0;
        double pu1, pu2, pv1, pv2;

        NCollection_Array1<double> startParams = new(1, 4);
        PWalking pw = new(surf1, surf1, tolTangency, epsilon, deflection, increment);

        double seuildPointLigne = 15.0 * increment * increment; //-- 10 est insuffisant
        double incidence;
        double dminiPointLigne;

        bool hasStartPoint, rejetLigne;

        PntOn2S startPOn2S = new();

        int ver;

        Point3D point3dDebut, point3dFin;

        if (nbLigSec >= 1)
        {
            int ls;
            for (ls = 1; ls <= nbLigSec; ls++)
            {
                SectionLine lineSec = interference.LineValue(ls);
                int nbp = lineSec.NumberOfPoints();
                int nbps2 = (nbp > 3) ? (nbp / 2) : 1;
                int nombreDePointsDeDepartDuCheminement = 0;
                int indicePointdeDepart1 = 0, indicePointdeDepart2 = 0;
                do
                {
                    nombreDePointsDeDepartDuCheminement++;
                    if (nombreDePointsDeDepartDuCheminement == 1)
                    {
                        incidence = 0.0;
                        int nsp1;
                        for (nsp1 = nbp / 2; nsp1 >= 1; nsp1--)
                        {
                            SectionPointToParameters(lineSec.GetPoint(nsp1), poly1, poly1, ref u1, ref v1, ref u2,
                                                     ref v2);
                            double currentIncidence = Math.Abs(u1 - u2) + Math.Abs(v1 - v2);
                            if (currentIncidence > incidence)
                            {
                                nbps2 = nsp1;
                                incidence = currentIncidence;
                            }
                        }
                        for (nsp1 = nbp / 2; nsp1 <= nbp; nsp1++)
                        {
                            SectionPointToParameters(lineSec.GetPoint(nsp1), poly1, poly1, ref u1, ref v1, ref u2,
                                                     ref v2);
                            double currentIncidence = Math.Abs(u1 - u2) + Math.Abs(v1 - v2);
                            if (currentIncidence > incidence)
                            {
                                nbps2 = nsp1;
                                incidence = currentIncidence;
                            }
                        }

                        if (nbp < 3)
                        {
                            nombreDePointsDeDepartDuCheminement = 3;
                        }

                        indicePointdeDepart1 = nbps2;
                    }
                    else if (nombreDePointsDeDepartDuCheminement == 2)
                    {
                        if (indicePointdeDepart1 == 1)
                        {
                            nbps2 = nbp / 2;
                            indicePointdeDepart2 = nbps2;
                        }
                        else
                        {
                            nbps2 = 1;
                            indicePointdeDepart2 = 1;
                        }
                    }
                    else
                    {
                        if (indicePointdeDepart1 == nbp)
                        {
                            nbps2 = (indicePointdeDepart1 + indicePointdeDepart2) / 2;
                        }
                        else
                        {
                            nbps2 = nbp;
                        }
                    }

                    SectionPointToParameters(lineSec.GetPoint(nbps2), poly1, poly1, ref u1, ref v1, ref u2, ref v2);

                    startParams[1] = u1;
                    startParams[2] = v1;
                    startParams[3] = u2;
                    startParams[4] = v2;

                    hasStartPoint = pw.PerformFirstPoint(startParams, ref startPOn2S);
                    dminiPointLigne = seuildPointLigne + seuildPointLigne;
                    if (hasStartPoint)
                    {
                        startPOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
                        if (Math.Abs(pu1 - pu2) > 1e-7 || Math.Abs(pv1 - pv2) > 1e-7)
                        {
                            nbLigCalculee = _sLin.Length();
                            int l;
                            for (l = 1; (l <= nbLigCalculee) && (dminiPointLigne >= seuildPointLigne); l++)
                            {
                                WLine testwline = (WLine)_sLin.Value(l);
                                if ((!testwline.IsOutSurf1Box(new Point2D(pu1, pv1)))
                                    && (!testwline.IsOutSurf2Box(new Point2D(pu2, pv2)))
                                    && (!testwline.IsOutBox(startPOn2S.Value())))
                                {
                                    nbPntOn2SOnLine = testwline.NbPnts();
                                    int ll;
                                    for (ll = 1; (ll < nbPntOn2SOnLine) && (dminiPointLigne >= seuildPointLigne); ll++)
                                    {
                                        double t;
                                        testwline.Point(ll).Parameters(out double au1, out double av1, out _, out _);
                                        testwline.Point(ll + 1).Parameters(out double bu1, out double bv1, out _,
                                                                           out _);
                                        if (au1 > bu1)
                                        {
                                            t = au1;
                                            au1 = bu1;
                                            bu1 = t;
                                        }
                                        if (av1 > bv1)
                                        {
                                            t = av1;
                                            av1 = bv1;
                                            bv1 = t;
                                        }
                                        au1 -= 1.0e-7;
                                        av1 -= 1.0e-7;
                                        bu1 += 1.0e-7;
                                        bv1 += 1.0e-7;

                                        if ((pu1 >= au1) && (pu1 <= bu1) && (pv1 >= av1) && (pv1 <= bv1))
                                        {
                                            dminiPointLigne = 0.0;
                                        }
                                        else
                                        {
                                            if ((pu2 >= au1) && (pu2 <= bu1) && (pv2 >= av1) && (pv2 <= bv1))
                                            {
                                                dminiPointLigne = 0.0;
                                            }
                                        }
                                    } // for( ll ...
                                } // if ...
                            } // for( l ...

                            if (dminiPointLigne > seuildPointLigne)
                            {
                                pw.Perform(startParams);
                                if (pw.IsDone)
                                {
                                    if (pw.PointCount > 2)
                                    {
                                        rejetLigne = false;
                                        point3dDebut = pw.Value(1).Value();
                                        point3dFin = pw.Value(pw.PointCount).Value();
                                        for (ver = 1; ver <= nbLigCalculee; ver++)
                                        {
                                            WLine verwline = (WLine)_sLin.Value(ver);
                                            PntOn2S verPointDebut = verwline.Point(1);
                                            PntOn2S verPointFin = verwline.Point(verwline.NbPnts());
                                            if ((point3dDebut.Distance(verPointDebut.Value()) < tolTangency)
                                                || (point3dFin.Distance(verPointFin.Value()) < tolTangency))
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
                                            surf1.D1(locu, locv, out ptbid, out d1u, out d1v);
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

                                            if (KeepOffDiagonalPart(pw, surf1, d1, trans1, trans2, tolTangency,
                                                                    point3dDebut, point3dFin))
                                            {
                                                _empt = false;
                                            }
                                        } // !RejetLigne
                                    } // PW.NbPoints()>2
                                } // done is True
                            } // dminiPointLigne > SeuildPointLigne
                        } // Abs || Abs
                    } // HasStartPoint
                } while (nbp > 5 && nombreDePointsDeDepartDuCheminement < 3);
            } // for( ls ...
        } // nbLigSec>=1

        int z;
        for (z = 1; z <= nbTanZon; z++)
        {
            TangentZone tangentZone = interference.ZoneValue(z);
            for (int pz = 1; pz <= tangentZone.NumberOfPoints(); pz++)
            {
                SectionPointToParameters(tangentZone.GetPoint(pz), poly1, poly1, ref u1, ref v1, ref u2, ref v2);

                startParams[1] = u1;
                startParams[2] = v1;
                startParams[3] = u2;
                startParams[4] = v2;

                hasStartPoint = pw.PerformFirstPoint(startParams, ref startPOn2S);
                if (hasStartPoint)
                {
                    startPOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
                    if (Math.Abs(pu1 - pu2) > 1e-7 || Math.Abs(pv1 - pv2) > 1e-7)
                    {
                        nbLigCalculee = _sLin.Length();
                        dminiPointLigne = seuildPointLigne + seuildPointLigne;
                        int l;
                        for (l = 1; (l <= nbLigCalculee) && (dminiPointLigne >= seuildPointLigne); l++)
                        {
                            WLine testwline = (WLine)_sLin.Value(l);
                            if ((!testwline.IsOutSurf1Box(new Point2D(pu1, pv1)))
                                && (!testwline.IsOutSurf2Box(new Point2D(pu2, pv2)))
                                && (!testwline.IsOutBox(startPOn2S.Value())))
                            {
                                nbPntOn2SOnLine = testwline.NbPnts();
                                int ll;
                                for (ll = 1; (ll < nbPntOn2SOnLine) && (dminiPointLigne >= seuildPointLigne); ll++)
                                {
                                    double t;
                                    testwline.Point(ll).Parameters(out double au1, out double av1, out _, out _);
                                    testwline.Point(ll + 1).Parameters(out double bu1, out double bv1, out _, out _);
                                    if (au1 > bu1)
                                    {
                                        t = au1;
                                        au1 = bu1;
                                        bu1 = t;
                                    }
                                    if (av1 > bv1)
                                    {
                                        t = av1;
                                        av1 = bv1;
                                        bv1 = t;
                                    }
                                    au1 -= 1.0e-7;
                                    av1 -= 1.0e-7;
                                    bu1 += 1.0e-7;
                                    bv1 += 1.0e-7;
                                    if ((pu1 >= au1) && (pu1 <= bu1) && (pv1 >= av1) && (pv1 <= bv1))
                                    {
                                        dminiPointLigne = 0.0;
                                    }
                                    else
                                    {
                                        if ((pu2 >= au1) && (pu2 <= bu1) && (pv2 >= av1) && (pv2 <= bv1))
                                        {
                                            dminiPointLigne = 0.0;
                                        }
                                    }
                                } // for( ll ...
                            } // if ...
                        } // for( l ...

                        if (dminiPointLigne > seuildPointLigne)
                        {
                            pw.Perform(startParams);
                            if (pw.IsDone)
                            {
                                if (pw.PointCount > 2)
                                {
                                    rejetLigne = false;
                                    point3dDebut = pw.Value(1).Value();
                                    point3dFin = pw.Value(pw.PointCount).Value();
                                    for (ver = 1; ver <= nbLigCalculee; ver++)
                                    {
                                        WLine verwline = (WLine)_sLin.Value(ver);
                                        PntOn2S verPointDebut = verwline.Point(1);
                                        PntOn2S verPointFin = verwline.Point(verwline.NbPnts());
                                        if ((point3dDebut.Distance(verPointDebut.Value()) < tolTangency)
                                            || (point3dFin.Distance(verPointFin.Value()) < tolTangency))
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
                                        surf1.D1(locu, locv, out ptbid, out d1u, out d1v);
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

                                        if (KeepOffDiagonalPart(pw, surf1, d1, trans1, trans2, tolTangency,
                                                                point3dDebut, point3dFin))
                                        {
                                            _empt = false;
                                        }
                                    } // !RejetLigne
                                } // PW.NbPoints()>2
                            } // done a True
                        } // dminiPointLigne > SeuildPointLigne
                    } // Abs || Abs
                } // HasStartPoint
            } // for ( pz ...
        } // for( z ...
    }

    // The block both self-intersection loops of OCCT repeat verbatim after the transitions: the part of the walking
    // line between its first and last points off the diagonal (one more point at each end), with its end vertex,
    // its vertices on the restrictions, appended to SLin. Returns true where OCCT sets empt to false; OCCT's
    // `continue` after PutVertexOnLine (a line of fewer than two points) is the `return false`, which reaches the end
    // of the loop body as the `continue` does.
    private bool KeepOffDiagonalPart(PWalking pw, SurfaceAdaptorBase surf1, TopolTool d1,
                                     WalkingTypeTrans trans1, WalkingTypeTrans trans2, double tolTangency,
                                     Point3D point3dDebut, Point3D point3dFin)
    {
        int nbpw, imin, imax, i;
        nbpw = pw.Line().PointCount;
        double u1, v1, u2, v2;
        i = 0;
        do
        {
            i++;
            imin = i;
            PntOn2S pi = pw.Line().Value(i);
            pi.Parameters(out u1, out v1, out u2, out v2);
        } while ((i < nbpw) && (Math.Abs(u1 - u2) <= 1e-6 && Math.Abs(v1 - v2) <= 1e-6));

        if (imin > 2)
        {
            imin--;
        }

        i = nbpw + 1;
        do
        {
            i--;
            imax = i;
            PntOn2S pi = pw.Line().Value(i);
            pi.Parameters(out u1, out v1, out u2, out v2);
        } while ((i > 2) && (Math.Abs(u1 - u2) <= 1e-6 && Math.Abs(v1 - v2) <= 1e-6));

        if (imax < nbpw)
        {
            imax++;
        }

        if (imin < imax)
        {
            LineOn2S pwLine = new();
            for (i = imin; i <= imax; i++)
            {
                pwLine.Add(pw.Line().Value(i));
            }

            double tolTang = tolTangency;
            WLine wline = new(pwLine, false, trans1, trans2);
            wline.SetCreatingWayInfo(WLine.IntPatch_WLType.WLPrmPrm);

            PntOn2S pOn2SDeb = wline.Point(1);
            PntOn2S pOn2SFin = wline.Point(wline.NbPnts());
            if ((pOn2SDeb.Value()).Distance(pOn2SFin.Value()) <= tolTangency)
            {
                pOn2SDeb.Parameters(out double u1t, out double v1t, out double u2t, out double v2t);
                PatchPoint vtx = new();
                vtx.SetValue(pOn2SDeb.Value(), tolTang, false);
                vtx.SetParameters(u2t, v2t, u1t, v1t);
                vtx.SetParameter(wline.NbPnts());
                wline.SetPoint(wline.NbPnts(), vtx);
            }
            // the method PutVertexOnLine can reduce the number of points in <wline>
            RstInt.PutVertexOnLine(wline, surf1, d1, surf1, true, tolTang);
            if (wline.NbPnts() < 2)
            {
                return false;
            }
            if (wline.VertexCount == 0)
            {
                AddEndVertices(wline, pw, point3dDebut, point3dFin, tolTang);
            }
            _sLin.Append(wline);
            return true;
        } // imin<imax
        return false;
    }

    // The block every Perform of OCCT repeats verbatim when a line has no vertex: vertices at the walking's first
    // point and at its point of the line's last index, at the given 3D points.
    private static void AddEndVertices(WLine wline, PWalking pw, in Point3D point3dDebut,
                                       in Point3D point3dFin, double tolTang)
    {
        PatchPoint vtx = new();
        PntOn2S pOn2S = pw.Line().Value(1);
        pOn2S.Parameters(out double pu1, out double pv1, out double pu2, out double pv2);
        vtx.SetValue(point3dDebut, tolTang, false);
        vtx.SetParameters(pu1, pv1, pu2, pv2);
        vtx.SetParameter(1);
        wline.AddVertex(vtx);

        pOn2S = pw.Line().Value(wline.NbPnts());
        pOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
        vtx.SetValue(point3dFin, tolTang, false);
        vtx.SetParameters(pu1, pv1, pu2, pv2);
        vtx.SetParameter(wline.NbPnts());
        wline.AddVertex(vtx);
    }

    //=================================================================================================

    /// <summary>
    /// A new line through the points <paramref name="low"/> to <paramref name="high"/> of the line
    /// <paramref name="numLine"/>, with about <paramref name="nbPntsToInsert"/> walking points inserted at constant
    /// arc length (measured on the first surface's parameters) where the existing points aren't too close.
    /// </summary>
    /// <exception cref="OutOfRangeException">The line index is out of range or <paramref name="low"/> ≥ <paramref name="high"/>.</exception>
    /// <exception cref="NotDoneException">The intersection is not done.</exception>
    public PatchLine NewLine(SurfaceAdaptorBase surf1, SurfaceAdaptorBase surf2, int numLine, int low, int high,
                                 int nbPntsToInsert)
    {
        int nbPnts = nbPntsToInsert + high - low;
        if (numLine > LineCount || numLine < 1 || low >= high)
        {
            throw new OutOfRangeException(" IntPatch_PrmPrmIntersection NewLine ");
        }
        //------------------------------------------------------------------
        //--  Indice     :   Low       Low+1     I    I+1         High    --
        //--                                                              --
        //--  Abs.Curv.  :  S(Low)              S(I)  S(I+1)      S(High) --
        //--                                                              --
        //--                On echantillonne a abcisse curviligne         --
        //--                constante.                                    --
        //--                L abcisse est calculee sur les params U1,V1   --
        //------------------------------------------------------------------
        NCollection_Array1<double> U1 = new(low, high);
        NCollection_Array1<double> V1 = new(low, high);
        NCollection_Array1<double> U2 = new(low, high);
        NCollection_Array1<double> V2 = new(low, high);
        NCollection_Array1<double> ac = new(low, high);

        double s, ds;
        WLine theLine = (Line(numLine) as WLine)!;
        PntOn2S point = theLine.Point(low);
        point.Parameters(out double u1, out double v1, out double u2, out double v2);
        U1[low] = u1;
        V1[low] = v1;
        U2[low] = u2;
        V2[low] = v2;
        ac[low] = 0.0;

        PWalking pw = new(surf1, surf2, 0.000001, 0.000001, 0.001, 0.001);

        int i;
        for (i = low + 1; i <= high; i++)
        {
            PntOn2S pointi = theLine.Point(i);
            pointi.Parameters(out u1, out v1, out u2, out v2);
            U1[i] = u1;
            V1[i] = v1;
            U2[i] = u2;
            V2[i] = v2;

            double du1 = u1 - U1[i - 1];
            double dv1 = v1 - V1[i - 1];

            ac[i] = ac[i - 1] + Math.Sqrt((du1 * du1) + (dv1 * dv1));
        }

        LineOn2S resultPntOn2SLine = new();

        PntOn2S startPOn2S = new();
        NCollection_Array1<double> startParams = new(1, 4);

        resultPntOn2SLine.Add(theLine.Point(low));

        ds = ac[high] / (nbPnts - 1);
        int indice = low;

        double dsmin = ds * 0.3;
        double smax = ac[high];

        for (i = 2, s = ds; (i < nbPnts) && (s < smax); i++, s += ds)
        {
            while (ac[indice + 1] <= s)
            {
                resultPntOn2SLine.Add(theLine.Point(indice));
                indice++;
            }
            double a = s - ac[indice];
            double b = ac[indice + 1] - s;
            double nab = 1.0 / (a + b);
            //----------------------------------------------------------
            //-- Verification :  Si Dist au prochain  point < dsmin   --
            //--                 Si Dist au precedent point < dsmin   --
            //--                                                      --
            //----------------------------------------------------------
            // OCCT compares 1 / (a + b), an inverse length, with the step ds (occt-issues.md, 5D)
            if ((nab > ds) && (a > dsmin) && (b > dsmin))
            {
                startParams[1] = (U1[indice] * b + U1[indice + 1] * a) * nab;
                startParams[2] = (V1[indice] * b + V1[indice + 1] * a) * nab;
                startParams[3] = (U2[indice] * b + U2[indice + 1] * a) * nab;
                startParams[4] = (V2[indice] * b + V2[indice + 1] * a) * nab;

                bool hasStartPoint = pw.PerformFirstPoint(startParams, ref startPOn2S);
                if (hasStartPoint)
                {
                    resultPntOn2SLine.Add(startPOn2S);
                }
            }
            else
            {
                s += dsmin;
            }
        }

        resultPntOn2SLine.Add(theLine.Point(high));

        WLine rwl = new(resultPntOn2SLine, false);
        rwl.SetCreatingWayInfo(WLine.IntPatch_WLType.WLPrmPrm);

        return (rwl);
    }

    //=================================================================================================

    /// <summary>Marks in <paramref name="map"/> the voxels of the segment between two voxels, by bisection.</summary>
    public void RemplitLin(int x1, int y1, int z1, int x2, int y2, int z2, PrmPrmIntersectionT3Bits map)
    {
        int xg, yg, zg;
        xg = x1 - x2;
        if (xg < 0)
        {
            xg = -xg;
        }
        yg = y1 - y2;
        if (yg < 0)
        {
            yg = -yg;
        }
        zg = z1 - z2;
        if (zg < 0)
        {
            zg = -zg;
        }
        if (DansGrille(x1) != 0 && DansGrille(y1) != 0 && DansGrille(z1) != 0)
        {
            int t = GrilleInteger(x1, y1, z1);
            map.Add(t);
        }
        if (xg <= 1 && yg <= 1 && zg <= 1)
        {
            return;
        }
        xg = (x1 + x2) >> 1;
        yg = (y1 + y2) >> 1;
        zg = (z1 + z2) >> 1;
        RemplitLin(x1, y1, z1, xg, yg, zg, map);
        RemplitLin(x2, y2, z2, xg, yg, zg, map);
    }

    //=================================================================================================

    /// <summary>Marks in <paramref name="map"/> the voxels of the triangle of three voxels, by subdivision.</summary>
    public void RemplitTri(int x1, int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3,
                           PrmPrmIntersectionT3Bits map)
    {
        if (x1 == x2 && x1 == x3 && y1 == y2 && y1 == y3 && z1 == z2 && z1 == z3)
        {
            if (DansGrille(x1) != 0 && DansGrille(y1) != 0 && DansGrille(z1) != 0)
            {
                int t = GrilleInteger(x1, y1, z1);
                map.Add(t);
            }
            return;
        }
        else
        {
            int xg = (x1 + x2 + x3) / 3;
            int yg = (y1 + y2 + y3) / 3;
            int zg = (z1 + z2 + z3) / 3;
            if (xg == x1 && yg == y1 && zg == z1)
            {
                RemplitLin(x1, y1, z1, x2, y2, z2, map);
                RemplitLin(x1, y1, z1, x3, y3, z3, map);
                return;
            }
            if (xg == x2 && yg == y2 && zg == z2)
            {
                RemplitLin(x2, y2, z2, x1, y1, z1, map);
                RemplitLin(x2, y2, z2, x3, y3, z3, map);
                return;
            }
            if (xg == x3 && yg == y3 && zg == z3)
            {
                RemplitLin(x3, y3, z3, x2, y2, z2, map);
                RemplitLin(x3, y3, z3, x1, y1, z1, map);
                return;
            }
            if (DansGrille(xg) != 0 && DansGrille(yg) != 0 && DansGrille(zg) != 0)
            {
                int t = GrilleInteger(xg, yg, zg);
                map.Add(t);
            }
            if (xg != x3 || yg != y3 || zg != z3)
            {
                RemplitTri(x1, y1, z1, x2, y2, z2, xg, yg, zg, map);
            }
            if (xg != x1 || yg != y1 || zg != z1)
            {
                RemplitTri(xg, yg, zg, x2, y2, z2, x3, y3, z3, map);
            }
            if (xg != x2 || yg != y2 || zg != z2)
            {
                RemplitTri(x1, y1, z1, xg, yg, zg, x3, y3, z3, map);
            }
        }
    }

    //=================================================================================================

    /// <summary>Marks in <paramref name="map"/> the voxels <paramref name="a"/>, <paramref name="b"/>, <paramref name="c"/> (-1: none) and the triangle between them.</summary>
    public void Remplit(int a, int b, int c, PrmPrmIntersectionT3Bits map)
    {
        int iax, iay, iaz, ibx, iby, ibz, icx, icy, icz;
        if (a != -1)
        {
            map.Add(a);
        }
        if (b != -1)
        {
            map.Add(b);
        }
        if (c != -1)
        {
            map.Add(c);
        }

        if (a != -1 && b != -1 && c != -1)
        {
            IntegerGrille(a, out iax, out iay, out iaz);
            IntegerGrille(b, out ibx, out iby, out ibz);
            IntegerGrille(c, out icx, out icy, out icz);
            RemplitTri(iax, iay, iaz, ibx, iby, ibz, icx, icy, icz, map);
        }
    }

    //=================================================================================================

    /// <summary>
    /// The intersection of the surfaces walked from the given points (with, on closed surfaces, their images on the
    /// other side of the seams, which are appended to <paramref name="lOfPnts"/>). Lines already computed by an
    /// earlier Perform are kept.
    /// </summary>
    public void Perform(SurfaceAdaptorBase surf1, TopolTool d1, SurfaceAdaptorBase surf2,
                        TopolTool d2, double tolTangency, double epsilon, double deflection,
                        double increment, NCollection_List<PntOn2S> lOfPnts)
    {
        if (lOfPnts.IsEmpty())
        {
            _done = true;
            return;
        }

        _empt = true;
        _sLin.Clear();

        double uminLig1, vminLig1, umaxLig1, vmaxLig1;
        double uminLig2, vminLig2, umaxLig2, vmaxLig2;
        double u1, u2, v1, v2;

        uminLig1 = surf1.FirstUParameter();
        vminLig1 = surf1.FirstVParameter();
        umaxLig1 = surf1.LastUParameter();
        vmaxLig1 = surf1.LastVParameter();
        uminLig2 = surf2.FirstUParameter();
        vminLig2 = surf2.FirstVParameter();
        umaxLig2 = surf2.LastUParameter();
        vmaxLig2 = surf2.LastVParameter();

        double[] periods = new double[4];
        periods[0] = (surf1.IsUPeriodic) ? surf1.UPeriod() : 0.0;
        periods[1] = (surf1.IsVPeriodic) ? surf1.VPeriod() : 0.0;
        periods[2] = (surf2.IsUPeriodic) ? surf2.UPeriod() : 0.0;
        periods[3] = (surf2.IsVPeriodic) ? surf2.VPeriod() : 0.0;

        NCollection_List<PntOn2S>.Iterator iterLOP1 = new(lOfPnts);
        if (surf1.IsUClosed || surf1.IsVClosed || surf2.IsUClosed || surf2.IsVClosed)
        {
            double tolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();
            NCollection_List<PntOn2S> additionalPnts = new();
            double newU1, newV1, newU2, newV2;
            for (; iterLOP1.More(); iterLOP1.Next())
            {
                PntOn2S Pnt = iterLOP1.Value();
                Pnt.Parameters(out u1, out v1, out u2, out v2);
                PntOn2S NewPnt = new();
                if (surf1.IsUClosed)
                {
                    if (Math.Abs(u1 - surf1.FirstUParameter()) <= tolPar)
                    {
                        newU1 = surf1.LastUParameter();
                        NewPnt.SetValue(newU1, v1, u2, v2);
                        additionalPnts.Append(NewPnt);
                    }
                    else if (Math.Abs(u1 - surf1.LastUParameter()) <= tolPar)
                    {
                        newU1 = surf1.FirstUParameter();
                        NewPnt.SetValue(newU1, v1, u2, v2);
                        additionalPnts.Append(NewPnt);
                    }
                }
                if (surf1.IsVClosed)
                {
                    if (Math.Abs(v1 - surf1.FirstVParameter()) <= tolPar)
                    {
                        newV1 = surf1.LastVParameter();
                        NewPnt.SetValue(u1, newV1, u2, v2);
                        additionalPnts.Append(NewPnt);
                    }
                    else if (Math.Abs(v1 - surf1.LastVParameter()) <= tolPar)
                    {
                        newV1 = surf1.FirstVParameter();
                        NewPnt.SetValue(u1, newV1, u2, v2);
                        additionalPnts.Append(NewPnt);
                    }
                }
                if (surf2.IsUClosed)
                {
                    if (Math.Abs(u2 - surf2.FirstUParameter()) <= tolPar)
                    {
                        newU2 = surf2.LastUParameter();
                        NewPnt.SetValue(u1, v1, newU2, v2);
                        additionalPnts.Append(NewPnt);
                    }
                    else if (Math.Abs(u2 - surf2.LastUParameter()) <= tolPar)
                    {
                        newU2 = surf2.FirstUParameter();
                        NewPnt.SetValue(u1, v1, newU2, v2);
                        additionalPnts.Append(NewPnt);
                    }
                }
                if (surf2.IsVClosed)
                {
                    if (Math.Abs(v2 - surf2.FirstVParameter()) <= tolPar)
                    {
                        newV2 = surf2.LastVParameter();
                        NewPnt.SetValue(u1, v1, u2, newV2);
                        additionalPnts.Append(NewPnt);
                    }
                    else if (Math.Abs(v2 - surf2.LastVParameter()) <= tolPar)
                    {
                        newV2 = surf2.FirstVParameter();
                        NewPnt.SetValue(u1, v1, u2, newV2);
                        additionalPnts.Append(NewPnt);
                    }
                }
            }
            // Cut repeated points
            for (iterLOP1.Initialize(lOfPnts); iterLOP1.More(); iterLOP1.Next())
            {
                PntOn2S aPnt = iterLOP1.Value();
                aPnt.Parameters(out u1, out v1, out u2, out v2);
                NCollection_List<PntOn2S>.Iterator iter2 = new(additionalPnts);
                while (iter2.More())
                {
                    PntOn2S newPnt = iter2.Value();
                    newPnt.Parameters(out newU1, out newV1, out newU2, out newV2);
                    if (Math.Abs(u1 - newU1) <= tolPar && Math.Abs(v1 - newV1) <= tolPar
                        && Math.Abs(u2 - newU2) <= tolPar && Math.Abs(v2 - newV2) <= tolPar)
                    {
                        additionalPnts.Remove(iter2);
                    }
                    else
                    {
                        iter2.Next();
                    }
                }
            }

            lOfPnts.Append(additionalPnts);
        }

        for (iterLOP1.Initialize(lOfPnts); iterLOP1.More(); iterLOP1.Next())
        {
            PntOn2S pnt = iterLOP1.Value();
            pnt.Parameters(out u1, out v1, out u2, out v2);
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

        double seuildPointLigne = 15.0 * increment * increment;

        int nbLigCalculee = 0, ver;
        double dminiPointLigne;
        bool hasStartPoint, rejetLigne;
        PntOn2S startPOn2S = new();
        Point3D point3dDebut, point3dFin;

        NCollection_Array1<double> startParams = new(1, 4);
        PWalking pw = new(surf1, surf2, tolTangency, epsilon, deflection, increment);

        NCollection_List<PntOn2S>.Iterator iterLOP2 = new(lOfPnts);
        for (; iterLOP2.More(); iterLOP2.Next())
        {
            PntOn2S cPnt = iterLOP2.Value();
            cPnt.Parameters(out u1, out v1, out u2, out v2);

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
                    pw.Perform(startParams, uminLig1, vminLig1, uminLig2, vminLig2, umaxLig1, vmaxLig1, umaxLig2,
                               vmaxLig2);
                    if (pw.IsDone)
                    {
                        if (pw.PointCount > 2)
                        {
                            // Try to extend the intersection line to boundary, if it is possible.
                            pw.PutToBoundary(surf1, surf2);

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
                                if ((point3dDebut.Distance(verPointDebut.Value()) <= tolTangency)
                                    && (point3dFin.Distance(verPointFin.Value()) <= tolTangency))
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

                                SeveralWlinesProcessing(surf1, surf2, _sLin, periods, trans1, trans2, tolTang,
                                                        Max(pw.MaxStep(0), pw.MaxStep(1)),
                                                        Max(pw.MaxStep(2), pw.MaxStep(3)), ref wline);

                                AddWLine(_sLin, wline, deflection);
                                _empt = false;
                            } // !RejetLigne
                        } // PW points > 2
                    } // done is True
                } // dminiPointLigne > SeuildPointLigne
            } // HasStartPoint
        } // for( IterLOP ...
        _done = true;
    }

    //=================================================================================================

    /// <summary>
    /// The intersection line walked from the given start parameters on both surfaces (none if no start point is
    /// found near them).
    /// </summary>
    public void Perform(SurfaceAdaptorBase surf1, TopolTool d1, SurfaceAdaptorBase surf2,
                        TopolTool d2, double u1Depart, double v1Depart, double u2Depart, double v2Depart,
                        double tolTangency, double epsilon, double deflection, double increment)
    {
        //-- Traitement des Lignes de sections
        _empt = true;
        _done = true;
        _sLin.Clear();

        //------------------------------------------------------------

        NCollection_Array1<double> startParams = new(1, 4);

        PWalking pw = new(surf1, surf2, tolTangency, epsilon, deflection, increment); // nIncrement);

        bool hasStartPoint; //,RejetLigne;

        PntOn2S startPOn2S = new();

        Point3D point3dDebut, point3dFin;

        //------------------------------------------------------------

        startParams[1] = u1Depart;
        startParams[2] = v1Depart;
        startParams[3] = u2Depart;
        startParams[4] = v2Depart;

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

            pw.Perform(startParams);
            if (pw.IsDone)
            {
                if (pw.PointCount > 2)
                {
                    point3dDebut = pw.Value(1).Value();
                    point3dFin = pw.Value(pw.PointCount).Value();

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
                        return;
                    }
                    RstInt.PutVertexOnLine(wline, surf2, d2, surf1, false, tolTang);
                    if (wline.NbPnts() < 2)
                    {
                        return;
                    }

                    //---------------
                    if (wline.VertexCount == 0)
                    {
                        AddEndVertices(wline, pw, point3dDebut, point3dFin, tolTang);
                    }

                    //---------------
                    _sLin.Append(wline);
                    _empt = false;
                } // if(PW.NbPoints()>2)
            } // if(PW.IsDone())
        } // if(HasStartPoint)
    }

    //=================================================================================================

    /// <summary>True if the intersection was computed.</summary>
    public bool IsDone => _done;

    /// <summary>True if there is no intersection line.</summary>
    /// <exception cref="NotDoneException">The intersection is not done.</exception>
    public bool IsEmpty
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException(" IntPatch_PrmPrmIntersection ");
            }
            return (_empt);
        }
    }

    /// <summary>The number of intersection lines.</summary>
    /// <exception cref="NotDoneException">The intersection is not done.</exception>
    public int LineCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException(" IntPatch_PrmPrmIntersection ");
            }
            return (_sLin.Length());
        }
    }

    /// <summary>The line <paramref name="n"/> (the line itself, decision 609).</summary>
    /// <exception cref="NotDoneException">The intersection is not done.</exception>
    public PatchLine Line(int n)
    {
        if (!_done)
        {
            throw new NotDoneException(" IntPatch_PrmPrmIntersection ");
        }
        return (_sLin.Value(n));
    }

    /// <summary>The voxel of coordinates (<paramref name="ix"/>, <paramref name="iy"/>, <paramref name="iz"/>), 7 bits each.</summary>
    public int GrilleInteger(int ix, int iy, int iz)
    {
        int tz = iz << _DECAL2;
        int ty = iy << _DECAL;
        int t = ix;
        t |= ty;
        t |= tz;
        return (t);
    }

    /// <summary>The coordinates of the voxel <paramref name="tt"/>.</summary>
    public void IntegerGrille(int tt, out int ix, out int iy, out int iz)
    {
        int t = tt;
        ix = t & _BASEM1;
        t >>= _DECAL;
        iy = t & _BASEM1;
        t >>= _DECAL;
        iz = t;
    }

    /// <summary>1 if the coordinate <paramref name="t"/> is in the grid [0, 128), else 0.</summary>
    public int DansGrille(int t)
    {
        if (t >= 0)
        {
            if (t < _BASE)
            {
                return (1);
            }
        }
        return (0);
    }

    /// <summary>The number of voxels along each axis (128).</summary>
    public int PointsGrilleCount => (_BASE);

    /// <summary>
    /// The code of the position of (<paramref name="x"/>, <paramref name="y"/>, <paramref name="z"/>) against the box
    /// (<paramref name="x0"/>, ...) to (<paramref name="x1"/>, ...): one bit for each side it is beyond.
    /// </summary>
    public int CodeReject(double x0, double y0, double z0, double x1, double y1, double z1, double x, double y,
                          double z)
    {
        int code = 0;
        if (x < x0)
        {
            code = 1;
        }
        if (y < y0)
        {
            code |= 2;
        }
        if (z < z0)
        {
            code |= 4;
        }
        if (x > x1)
        {
            code |= 8;
        }
        if (y > y1)
        {
            code |= 16;
        }
        if (z > z1)
        {
            code |= 32;
        }
        return (code);
    }
}
