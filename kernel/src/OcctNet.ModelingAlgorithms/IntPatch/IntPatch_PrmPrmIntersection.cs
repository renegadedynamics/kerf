// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_PrmPrmIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

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
public partial class IntPatch_PrmPrmIntersection
{
    private const int _DECAL = 7;
    private const int _DECAL2 = 14;
    private const int _BASE = 128;
    private const int _BASEM1 = 127;

    private bool done;
    private bool empt;
    private readonly NCollection_Sequence<IntPatch_Line> SLin = new();

    /// <summary>An empty intersection.</summary>
    public IntPatch_PrmPrmIntersection()
    {
        done = false;
        empt = true;
    }

    //=================================================================================================

    /// <summary>The self-intersection of <paramref name="Surf1"/>, on a polyhedron of its domain's sampling.</summary>
    public void Perform(Adaptor3d_Surface Surf1, Adaptor3d_TopolTool D1, double TolTangency, double Epsilon,
                        double Deflection, double Increment)
    {
        IntPatch_Polyhedron Poly1 = new(Surf1, D1.NbSamplesU(), D1.NbSamplesV());
        Perform(Surf1, Poly1, D1, TolTangency, Epsilon, Deflection, Increment);
    }

    //=================================================================================================

    /// <summary>
    /// The intersection of <paramref name="Surf1"/>, approximated by <paramref name="Poly1"/>, and
    /// <paramref name="Surf2"/>, whose polyhedron is computed.
    /// </summary>
    public void Perform(Adaptor3d_Surface Surf1, IntPatch_Polyhedron Poly1, Adaptor3d_TopolTool D1,
                        Adaptor3d_Surface Surf2, Adaptor3d_TopolTool D2, double TolTangency, double Epsilon,
                        double Deflection, double Increment)
    {
        IntPatch_Polyhedron Poly2 = new(Surf2);
        Perform(Surf1, Poly1, D1, Surf2, Poly2, D2, TolTangency, Epsilon, Deflection, Increment);
    }

    //=================================================================================================

    /// <summary>
    /// The intersection of <paramref name="Surf1"/>, whose polyhedron is computed, and <paramref name="Surf2"/>,
    /// approximated by <paramref name="Poly2"/>.
    /// </summary>
    public void Perform(Adaptor3d_Surface Surf1, Adaptor3d_TopolTool D1, Adaptor3d_Surface Surf2,
                        IntPatch_Polyhedron Poly2, Adaptor3d_TopolTool D2, double TolTangency, double Epsilon,
                        double Deflection, double Increment)
    {
        IntPatch_Polyhedron Poly1 = new(Surf1);
        Perform(Surf1, Poly1, D1, Surf2, Poly2, D2, TolTangency, Epsilon, Deflection, Increment);
    }

    //=================================================================================================

    /// <summary>
    /// The intersection of <paramref name="Surf1"/> and <paramref name="Surf2"/>, approximated by the polyhedra
    /// <paramref name="Poly1"/> and <paramref name="Poly2"/>: walking lines started from the section lines and
    /// tangent zones of the polyhedra's interference.
    /// </summary>
    public void Perform(Adaptor3d_Surface Surf1, IntPatch_Polyhedron Poly1, Adaptor3d_TopolTool D1,
                        Adaptor3d_Surface Surf2, IntPatch_Polyhedron Poly2, Adaptor3d_TopolTool D2,
                        double TolTangency, double Epsilon, double Deflection, double Increment)
    {
        IntPatch_InterferencePolyhedron Interference = new(Poly1, Poly2);
        empt = true;
        done = true;
        SLin.Clear();

        int nbLigSec = Interference.NbSectionLines();
        int nbTanZon = Interference.NbTangentZones();

        int NbLigCalculee = 0;

        double U1 = 0.0, U2 = 0.0, V1 = 0.0, V2 = 0.0;
        double pu1, pu2, pv1, pv2;

        NCollection_Array1<double> StartParams = new(1, 4);

        IntWalk_PWalking PW = new(Surf1, Surf2, TolTangency, Epsilon, Deflection, Increment);

        double SeuildPointLigne = 15.0 * Increment * Increment; //-- 10 est insuffisant
        double incidence;
        double dminiPointLigne;

        bool HasStartPoint, RejetLigne;

        IntSurf_PntOn2S StartPOn2S = new();

        int ver;

        Point3D Point3dDebut, Point3dFin;

        if (nbLigSec >= 1)
        {
            int[] TabL = new int[nbLigSec + 1];
            int ls;
            for (ls = 1; ls <= nbLigSec; ls++)
            {
                TabL[ls] = ls;
            }

            bool triok;
            do
            {
                triok = true;
                for (int b = 2; b <= nbLigSec; b++)
                {
                    int nb_B = Interference.LineValue(TabL[b]).NumberOfPoints();
                    int nb_A = Interference.LineValue(TabL[b - 1]).NumberOfPoints();
                    if (nb_B > nb_A)
                    {
                        int tyu = TabL[b];
                        TabL[b] = TabL[b - 1];
                        TabL[b - 1] = tyu;
                        triok = false;
                    }
                }
            } while (!triok);

            for (ls = 1; ls <= nbLigSec; ls++)
            {
                Intf_SectionLine LineSec = Interference.LineValue(TabL[ls]);
                int nbp = LineSec.NumberOfPoints();

                int[] TabPtDep = new int[nbp + 1];
                int ilig;
                for (ilig = 1; ilig <= nbp; ilig++)
                {
                    TabPtDep[ilig] = 0;
                }

                double UminLig1 = 0.0, VminLig1 = 0.0, UmaxLig1, VmaxLig1;
                double UminLig2 = 0.0, VminLig2 = 0.0, UmaxLig2, VmaxLig2;

                SectionPointToParameters(LineSec.GetPoint(1), Poly1, Poly2, ref UminLig1, ref VminLig1, ref UminLig2,
                                         ref VminLig2);

                UmaxLig1 = UminLig1;
                VmaxLig1 = VminLig1;
                UmaxLig2 = UminLig2;
                VmaxLig2 = VminLig2;

                for (ilig = 2; ilig <= nbp; ilig++)
                {
                    SectionPointToParameters(LineSec.GetPoint(ilig), Poly1, Poly2, ref U1, ref V1, ref U2, ref V2);

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

                int nbps2 = (nbp > 3) ? (nbp / 2) : 1;
                int NombreDePointsDeDepartDuCheminement = 0;
                int IndicePointdeDepart1 = 0, IndicePointdeDepart2 = 0;
                bool lignetrouvee = false;

                do
                {
                    NombreDePointsDeDepartDuCheminement++;
                    if (NombreDePointsDeDepartDuCheminement == 1)
                    {
                        incidence = 3.0;
                        int nbp1_4 = nbp / 4;
                        int nbp3_4 = nbp - nbp1_4;

                        int nsp;
                        for (nsp = nbp / 2; nsp < nbp3_4; nsp++)
                        {
                            double CurrentIncidence = LineSec.GetPoint(nsp).Incidence();
                            if (CurrentIncidence < incidence)
                            {
                                nbps2 = nsp;
                                incidence = 0.9 * CurrentIncidence;
                            }
                        }

                        for (nsp = nbp / 2; nsp > nbp1_4; nsp--)
                        {
                            double CurrentIncidence = LineSec.GetPoint(nsp).Incidence();
                            if (CurrentIncidence < incidence)
                            {
                                nbps2 = nsp;
                                incidence = 0.9 * CurrentIncidence;
                            }
                        }

                        if (nbp < 3)
                        {
                            NombreDePointsDeDepartDuCheminement = 3;
                        }

                        IndicePointdeDepart1 = nbps2;
                    }
                    else if (NombreDePointsDeDepartDuCheminement == 2)
                    {
                        if (IndicePointdeDepart1 == 1)
                        {
                            nbps2 = nbp / 2;
                            IndicePointdeDepart2 = nbps2;
                        }
                        else
                        {
                            nbps2 = 1;
                            IndicePointdeDepart2 = 1;
                        }
                    }
                    else if (NombreDePointsDeDepartDuCheminement == 3)
                    {
                        if (IndicePointdeDepart1 == nbp)
                        {
                            nbps2 = (IndicePointdeDepart1 + IndicePointdeDepart2) / 2;
                        }
                        else
                        {
                            nbps2 = nbp;
                        }
                    }
                    else
                    {
                        nbps2 = NombreDePointsDeDepartDuCheminement - 3;
                        NombreDePointsDeDepartDuCheminement++;
                    }

                    if (TabPtDep[nbps2] == 0)
                    {
                        TabPtDep[nbps2] = 1;
                        SectionPointToParameters(LineSec.GetPoint(nbps2), Poly1, Poly2, ref U1, ref V1, ref U2, ref V2);

                        StartParams[1] = U1;
                        StartParams[2] = V1;
                        StartParams[3] = U2;
                        StartParams[4] = V2;

                        HasStartPoint = PW.PerformFirstPoint(StartParams, ref StartPOn2S);
                        dminiPointLigne = SeuildPointLigne + SeuildPointLigne;

                        if (HasStartPoint)
                        {
                            StartPOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
                            NbLigCalculee = SLin.Length();
                            int l;
                            for (l = 1; (l <= NbLigCalculee) && (dminiPointLigne >= SeuildPointLigne); l++)
                            {
                                IntPatch_WLine testwline = (IntPatch_WLine)SLin.Value(l);

                                if (IsPointOnLine(StartPOn2S, testwline, Deflection))
                                {
                                    dminiPointLigne = 0.0;
                                }
                            } // for ( l ...

                            if (dminiPointLigne > SeuildPointLigne)
                            {
                                PW.Perform(StartParams, UminLig1, VminLig1, UminLig2, VminLig2, UmaxLig1, VmaxLig1,
                                           UmaxLig2, VmaxLig2);
                                if (PW.IsDone())
                                {
                                    if (PW.NbPoints() > 2)
                                    {
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
                                            if ((Point3dDebut.Distance(verPointDebut.Value()) <= TolTangency)
                                                && (Point3dFin.Distance(verPointFin.Value()) <= TolTangency))
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
                                            // Calculation transition
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

                                            if (wline.NbVertex() == 0)
                                            {
                                                AddEndVertices(wline, PW, Point3dDebut, Point3dFin, TolTang);
                                            }

                                            lignetrouvee = true;
                                            AddWLine(SLin, wline, Deflection);
                                            empt = false;
                                        } // !RejetLigne
                                    } // PW.NbPoints()>2
                                } // done is True
                            } // dminiPointLigne > SeuildPointLigne
                        } // HasStartPoint
                    } // TabPtDep[nbps2]==0
                } while (nbp > 5
                         && (((NombreDePointsDeDepartDuCheminement < 3) || !lignetrouvee)
                             && ((NombreDePointsDeDepartDuCheminement - 3 < nbp) || lignetrouvee)));
            } // for( ls ...
        } // nbLigSec >= 1

        {
            double UminLig1, VminLig1, UmaxLig1, VmaxLig1;
            double UminLig2, VminLig2, UmaxLig2, VmaxLig2;

            UminLig1 = VminLig1 = UminLig2 = VminLig2 = RealLast();
            UmaxLig1 = VmaxLig1 = UmaxLig2 = VmaxLig2 = -UminLig1;

            int z;
            for (z = 1; z <= nbTanZon; z++)
            {
                Intf_TangentZone TangentZone = Interference.ZoneValue(z);
                for (int pz = 1; pz <= TangentZone.NumberOfPoints(); pz++)
                {
                    SectionPointToParameters(TangentZone.GetPoint(pz), Poly1, Poly2, ref U1, ref V1, ref U2, ref V2);

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
            }

            for (z = 1; z <= nbTanZon; z++)
            {
                Intf_TangentZone TangentZone = Interference.ZoneValue(z);
                int pz;
                for (pz = 1; pz <= TangentZone.NumberOfPoints(); pz++)
                {
                    SectionPointToParameters(TangentZone.GetPoint(pz), Poly1, Poly2, ref U1, ref V1, ref U2, ref V2);

                    StartParams[1] = U1;
                    StartParams[2] = V1;
                    StartParams[3] = U2;
                    StartParams[4] = V2;

                    HasStartPoint = PW.PerformFirstPoint(StartParams, ref StartPOn2S);
                    if (HasStartPoint)
                    {
                        StartPOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
                        NbLigCalculee = SLin.Length();
                        dminiPointLigne = SeuildPointLigne + SeuildPointLigne;
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
                            if (PW.IsDone())
                            {
                                if (PW.NbPoints() > 2)
                                {
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

                                        if (wline.NbVertex() == 0)
                                        {
                                            AddEndVertices(wline, PW, Point3dDebut, Point3dFin, TolTang);
                                        }

                                        AddWLine(SLin, wline, Deflection);
                                        empt = false;
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
    /// The self-intersection of <paramref name="Surf1"/>, approximated by <paramref name="Poly1"/>: walking lines
    /// started from the polyhedron's self-interference, keeping the parts away from the diagonal (U1, V1) = (U2, V2).
    /// </summary>
    public void Perform(Adaptor3d_Surface Surf1, IntPatch_Polyhedron Poly1, Adaptor3d_TopolTool D1, double TolTangency,
                        double Epsilon, double Deflection, double Increment)
    {
        IntPatch_InterferencePolyhedron Interference = new(Poly1);
        empt = true;
        done = true;
        SLin.Clear();

        int nbLigSec = Interference.NbSectionLines();
        int nbTanZon = Interference.NbTangentZones();

        int NbPntOn2SOnLine;
        int NbLigCalculee = 0;

        double U1 = 0.0, U2 = 0.0, V1 = 0.0, V2 = 0.0;
        double pu1, pu2, pv1, pv2;

        NCollection_Array1<double> StartParams = new(1, 4);
        IntWalk_PWalking PW = new(Surf1, Surf1, TolTangency, Epsilon, Deflection, Increment);

        double SeuildPointLigne = 15.0 * Increment * Increment; //-- 10 est insuffisant
        double incidence;
        double dminiPointLigne;

        bool HasStartPoint, RejetLigne;

        IntSurf_PntOn2S StartPOn2S = new();

        int ver;

        Point3D Point3dDebut, Point3dFin;

        if (nbLigSec >= 1)
        {
            int ls;
            for (ls = 1; ls <= nbLigSec; ls++)
            {
                Intf_SectionLine LineSec = Interference.LineValue(ls);
                int nbp = LineSec.NumberOfPoints();
                int nbps2 = (nbp > 3) ? (nbp / 2) : 1;
                int NombreDePointsDeDepartDuCheminement = 0;
                int IndicePointdeDepart1 = 0, IndicePointdeDepart2 = 0;
                do
                {
                    NombreDePointsDeDepartDuCheminement++;
                    if (NombreDePointsDeDepartDuCheminement == 1)
                    {
                        incidence = 0.0;
                        int nsp1;
                        for (nsp1 = nbp / 2; nsp1 >= 1; nsp1--)
                        {
                            SectionPointToParameters(LineSec.GetPoint(nsp1), Poly1, Poly1, ref U1, ref V1, ref U2,
                                                     ref V2);
                            double CurrentIncidence = Math.Abs(U1 - U2) + Math.Abs(V1 - V2);
                            if (CurrentIncidence > incidence)
                            {
                                nbps2 = nsp1;
                                incidence = CurrentIncidence;
                            }
                        }
                        for (nsp1 = nbp / 2; nsp1 <= nbp; nsp1++)
                        {
                            SectionPointToParameters(LineSec.GetPoint(nsp1), Poly1, Poly1, ref U1, ref V1, ref U2,
                                                     ref V2);
                            double CurrentIncidence = Math.Abs(U1 - U2) + Math.Abs(V1 - V2);
                            if (CurrentIncidence > incidence)
                            {
                                nbps2 = nsp1;
                                incidence = CurrentIncidence;
                            }
                        }

                        if (nbp < 3)
                        {
                            NombreDePointsDeDepartDuCheminement = 3;
                        }

                        IndicePointdeDepart1 = nbps2;
                    }
                    else if (NombreDePointsDeDepartDuCheminement == 2)
                    {
                        if (IndicePointdeDepart1 == 1)
                        {
                            nbps2 = nbp / 2;
                            IndicePointdeDepart2 = nbps2;
                        }
                        else
                        {
                            nbps2 = 1;
                            IndicePointdeDepart2 = 1;
                        }
                    }
                    else
                    {
                        if (IndicePointdeDepart1 == nbp)
                        {
                            nbps2 = (IndicePointdeDepart1 + IndicePointdeDepart2) / 2;
                        }
                        else
                        {
                            nbps2 = nbp;
                        }
                    }

                    SectionPointToParameters(LineSec.GetPoint(nbps2), Poly1, Poly1, ref U1, ref V1, ref U2, ref V2);

                    StartParams[1] = U1;
                    StartParams[2] = V1;
                    StartParams[3] = U2;
                    StartParams[4] = V2;

                    HasStartPoint = PW.PerformFirstPoint(StartParams, ref StartPOn2S);
                    dminiPointLigne = SeuildPointLigne + SeuildPointLigne;
                    if (HasStartPoint)
                    {
                        StartPOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
                        if (Math.Abs(pu1 - pu2) > 1e-7 || Math.Abs(pv1 - pv2) > 1e-7)
                        {
                            NbLigCalculee = SLin.Length();
                            int l;
                            for (l = 1; (l <= NbLigCalculee) && (dminiPointLigne >= SeuildPointLigne); l++)
                            {
                                IntPatch_WLine testwline = (IntPatch_WLine)SLin.Value(l);
                                if ((!testwline.IsOutSurf1Box(new Point2D(pu1, pv1)))
                                    && (!testwline.IsOutSurf2Box(new Point2D(pu2, pv2)))
                                    && (!testwline.IsOutBox(StartPOn2S.Value())))
                                {
                                    NbPntOn2SOnLine = testwline.NbPnts();
                                    int ll;
                                    for (ll = 1; (ll < NbPntOn2SOnLine) && (dminiPointLigne >= SeuildPointLigne); ll++)
                                    {
                                        double t;
                                        testwline.Point(ll).Parameters(out double Au1, out double Av1, out _, out _);
                                        testwline.Point(ll + 1).Parameters(out double Bu1, out double Bv1, out _,
                                                                           out _);
                                        if (Au1 > Bu1)
                                        {
                                            t = Au1;
                                            Au1 = Bu1;
                                            Bu1 = t;
                                        }
                                        if (Av1 > Bv1)
                                        {
                                            t = Av1;
                                            Av1 = Bv1;
                                            Bv1 = t;
                                        }
                                        Au1 -= 1.0e-7;
                                        Av1 -= 1.0e-7;
                                        Bu1 += 1.0e-7;
                                        Bv1 += 1.0e-7;

                                        if ((pu1 >= Au1) && (pu1 <= Bu1) && (pv1 >= Av1) && (pv1 <= Bv1))
                                        {
                                            dminiPointLigne = 0.0;
                                        }
                                        else
                                        {
                                            if ((pu2 >= Au1) && (pu2 <= Bu1) && (pv2 >= Av1) && (pv2 <= Bv1))
                                            {
                                                dminiPointLigne = 0.0;
                                            }
                                        }
                                    } // for( ll ...
                                } // if ...
                            } // for( l ...

                            if (dminiPointLigne > SeuildPointLigne)
                            {
                                PW.Perform(StartParams);
                                if (PW.IsDone())
                                {
                                    if (PW.NbPoints() > 2)
                                    {
                                        RejetLigne = false;
                                        Point3dDebut = PW.Value(1).Value();
                                        Point3dFin = PW.Value(PW.NbPoints()).Value();
                                        for (ver = 1; ver <= NbLigCalculee; ver++)
                                        {
                                            IntPatch_WLine verwline = (IntPatch_WLine)SLin.Value(ver);
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
                                            Surf1.D1(locu, locv, out ptbid, out d1u, out d1v);
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

                                            if (KeepOffDiagonalPart(PW, Surf1, D1, trans1, trans2, TolTangency,
                                                                    Point3dDebut, Point3dFin))
                                            {
                                                empt = false;
                                            }
                                        } // !RejetLigne
                                    } // PW.NbPoints()>2
                                } // done is True
                            } // dminiPointLigne > SeuildPointLigne
                        } // Abs || Abs
                    } // HasStartPoint
                } while (nbp > 5 && NombreDePointsDeDepartDuCheminement < 3);
            } // for( ls ...
        } // nbLigSec>=1

        int z;
        for (z = 1; z <= nbTanZon; z++)
        {
            Intf_TangentZone TangentZone = Interference.ZoneValue(z);
            for (int pz = 1; pz <= TangentZone.NumberOfPoints(); pz++)
            {
                SectionPointToParameters(TangentZone.GetPoint(pz), Poly1, Poly1, ref U1, ref V1, ref U2, ref V2);

                StartParams[1] = U1;
                StartParams[2] = V1;
                StartParams[3] = U2;
                StartParams[4] = V2;

                HasStartPoint = PW.PerformFirstPoint(StartParams, ref StartPOn2S);
                if (HasStartPoint)
                {
                    StartPOn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
                    if (Math.Abs(pu1 - pu2) > 1e-7 || Math.Abs(pv1 - pv2) > 1e-7)
                    {
                        NbLigCalculee = SLin.Length();
                        dminiPointLigne = SeuildPointLigne + SeuildPointLigne;
                        int l;
                        for (l = 1; (l <= NbLigCalculee) && (dminiPointLigne >= SeuildPointLigne); l++)
                        {
                            IntPatch_WLine testwline = (IntPatch_WLine)SLin.Value(l);
                            if ((!testwline.IsOutSurf1Box(new Point2D(pu1, pv1)))
                                && (!testwline.IsOutSurf2Box(new Point2D(pu2, pv2)))
                                && (!testwline.IsOutBox(StartPOn2S.Value())))
                            {
                                NbPntOn2SOnLine = testwline.NbPnts();
                                int ll;
                                for (ll = 1; (ll < NbPntOn2SOnLine) && (dminiPointLigne >= SeuildPointLigne); ll++)
                                {
                                    double t;
                                    testwline.Point(ll).Parameters(out double Au1, out double Av1, out _, out _);
                                    testwline.Point(ll + 1).Parameters(out double Bu1, out double Bv1, out _, out _);
                                    if (Au1 > Bu1)
                                    {
                                        t = Au1;
                                        Au1 = Bu1;
                                        Bu1 = t;
                                    }
                                    if (Av1 > Bv1)
                                    {
                                        t = Av1;
                                        Av1 = Bv1;
                                        Bv1 = t;
                                    }
                                    Au1 -= 1.0e-7;
                                    Av1 -= 1.0e-7;
                                    Bu1 += 1.0e-7;
                                    Bv1 += 1.0e-7;
                                    if ((pu1 >= Au1) && (pu1 <= Bu1) && (pv1 >= Av1) && (pv1 <= Bv1))
                                    {
                                        dminiPointLigne = 0.0;
                                    }
                                    else
                                    {
                                        if ((pu2 >= Au1) && (pu2 <= Bu1) && (pv2 >= Av1) && (pv2 <= Bv1))
                                        {
                                            dminiPointLigne = 0.0;
                                        }
                                    }
                                } // for( ll ...
                            } // if ...
                        } // for( l ...

                        if (dminiPointLigne > SeuildPointLigne)
                        {
                            PW.Perform(StartParams);
                            if (PW.IsDone())
                            {
                                if (PW.NbPoints() > 2)
                                {
                                    RejetLigne = false;
                                    Point3dDebut = PW.Value(1).Value();
                                    Point3dFin = PW.Value(PW.NbPoints()).Value();
                                    for (ver = 1; ver <= NbLigCalculee; ver++)
                                    {
                                        IntPatch_WLine verwline = (IntPatch_WLine)SLin.Value(ver);
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
                                        Surf1.D1(locu, locv, out ptbid, out d1u, out d1v);
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

                                        if (KeepOffDiagonalPart(PW, Surf1, D1, trans1, trans2, TolTangency,
                                                                Point3dDebut, Point3dFin))
                                        {
                                            empt = false;
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
    private bool KeepOffDiagonalPart(IntWalk_PWalking PW, Adaptor3d_Surface Surf1, Adaptor3d_TopolTool D1,
                                     IntSurf_TypeTrans trans1, IntSurf_TypeTrans trans2, double TolTangency,
                                     Point3D Point3dDebut, Point3D Point3dFin)
    {
        int nbpw, imin, imax, i;
        nbpw = PW.Line().NbPoints();
        double u1, v1, u2, v2;
        i = 0;
        do
        {
            i++;
            imin = i;
            IntSurf_PntOn2S Pi = PW.Line().Value(i);
            Pi.Parameters(out u1, out v1, out u2, out v2);
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
            IntSurf_PntOn2S Pi = PW.Line().Value(i);
            Pi.Parameters(out u1, out v1, out u2, out v2);
        } while ((i > 2) && (Math.Abs(u1 - u2) <= 1e-6 && Math.Abs(v1 - v2) <= 1e-6));

        if (imax < nbpw)
        {
            imax++;
        }

        if (imin < imax)
        {
            IntSurf_LineOn2S PWLine = new();
            for (i = imin; i <= imax; i++)
            {
                PWLine.Add(PW.Line().Value(i));
            }

            double TolTang = TolTangency;
            IntPatch_WLine wline = new(PWLine, false, trans1, trans2);
            wline.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLPrmPrm);

            IntSurf_PntOn2S POn2SDeb = wline.Point(1);
            IntSurf_PntOn2S POn2SFin = wline.Point(wline.NbPnts());
            if ((POn2SDeb.Value()).Distance(POn2SFin.Value()) <= TolTangency)
            {
                POn2SDeb.Parameters(out double u1t, out double v1t, out double u2t, out double v2t);
                IntPatch_Point vtx = new();
                vtx.SetValue(POn2SDeb.Value(), TolTang, false);
                vtx.SetParameters(u2t, v2t, u1t, v1t);
                vtx.SetParameter(wline.NbPnts());
                wline.SetPoint(wline.NbPnts(), vtx);
            }
            // the method PutVertexOnLine can reduce the number of points in <wline>
            IntPatch_RstInt.PutVertexOnLine(wline, Surf1, D1, Surf1, true, TolTang);
            if (wline.NbPnts() < 2)
            {
                return false;
            }
            if (wline.NbVertex() == 0)
            {
                AddEndVertices(wline, PW, Point3dDebut, Point3dFin, TolTang);
            }
            SLin.Append(wline);
            return true;
        } // imin<imax
        return false;
    }

    // The block every Perform of OCCT repeats verbatim when a line has no vertex: vertices at the walking's first
    // point and at its point of the line's last index, at the given 3D points.
    private static void AddEndVertices(IntPatch_WLine wline, IntWalk_PWalking PW, in Point3D Point3dDebut,
                                       in Point3D Point3dFin, double TolTang)
    {
        IntPatch_Point vtx = new();
        IntSurf_PntOn2S POn2S = PW.Line().Value(1);
        POn2S.Parameters(out double pu1, out double pv1, out double pu2, out double pv2);
        vtx.SetValue(Point3dDebut, TolTang, false);
        vtx.SetParameters(pu1, pv1, pu2, pv2);
        vtx.SetParameter(1);
        wline.AddVertex(vtx);

        POn2S = PW.Line().Value(wline.NbPnts());
        POn2S.Parameters(out pu1, out pv1, out pu2, out pv2);
        vtx.SetValue(Point3dFin, TolTang, false);
        vtx.SetParameters(pu1, pv1, pu2, pv2);
        vtx.SetParameter(wline.NbPnts());
        wline.AddVertex(vtx);
    }

    //=================================================================================================

    /// <summary>
    /// A new line through the points <paramref name="Low"/> to <paramref name="High"/> of the line
    /// <paramref name="NumLine"/>, with about <paramref name="NbPntsToInsert"/> walking points inserted at constant
    /// arc length (measured on the first surface's parameters) where the existing points aren't too close.
    /// </summary>
    /// <exception cref="OutOfRangeException">The line index is out of range or <paramref name="Low"/> ≥ <paramref name="High"/>.</exception>
    /// <exception cref="NotDoneException">The intersection is not done.</exception>
    public IntPatch_Line NewLine(Adaptor3d_Surface Surf1, Adaptor3d_Surface Surf2, int NumLine, int Low, int High,
                                 int NbPntsToInsert)
    {
        int NbPnts = NbPntsToInsert + High - Low;
        if (NumLine > NbLines() || NumLine < 1 || Low >= High)
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
        NCollection_Array1<double> U1 = new(Low, High);
        NCollection_Array1<double> V1 = new(Low, High);
        NCollection_Array1<double> U2 = new(Low, High);
        NCollection_Array1<double> V2 = new(Low, High);
        NCollection_Array1<double> AC = new(Low, High);

        double s, ds;
        IntPatch_WLine TheLine = (Line(NumLine) as IntPatch_WLine)!;
        IntSurf_PntOn2S Point = TheLine.Point(Low);
        Point.Parameters(out double u1, out double v1, out double u2, out double v2);
        U1[Low] = u1;
        V1[Low] = v1;
        U2[Low] = u2;
        V2[Low] = v2;
        AC[Low] = 0.0;

        IntWalk_PWalking PW = new(Surf1, Surf2, 0.000001, 0.000001, 0.001, 0.001);

        int i;
        for (i = Low + 1; i <= High; i++)
        {
            IntSurf_PntOn2S Pointi = TheLine.Point(i);
            Pointi.Parameters(out u1, out v1, out u2, out v2);
            U1[i] = u1;
            V1[i] = v1;
            U2[i] = u2;
            V2[i] = v2;

            double du1 = u1 - U1[i - 1];
            double dv1 = v1 - V1[i - 1];

            AC[i] = AC[i - 1] + Math.Sqrt((du1 * du1) + (dv1 * dv1));
        }

        IntSurf_LineOn2S ResultPntOn2SLine = new();

        IntSurf_PntOn2S StartPOn2S = new();
        NCollection_Array1<double> StartParams = new(1, 4);

        ResultPntOn2SLine.Add(TheLine.Point(Low));

        ds = AC[High] / (NbPnts - 1);
        int Indice = Low;

        double dsmin = ds * 0.3;
        double smax = AC[High];

        for (i = 2, s = ds; (i < NbPnts) && (s < smax); i++, s += ds)
        {
            while (AC[Indice + 1] <= s)
            {
                ResultPntOn2SLine.Add(TheLine.Point(Indice));
                Indice++;
            }
            double a = s - AC[Indice];
            double b = AC[Indice + 1] - s;
            double nab = 1.0 / (a + b);
            //----------------------------------------------------------
            //-- Verification :  Si Dist au prochain  point < dsmin   --
            //--                 Si Dist au precedent point < dsmin   --
            //--                                                      --
            //----------------------------------------------------------
            // OCCT compares 1 / (a + b), an inverse length, with the step ds (occt-issues.md, 5D)
            if ((nab > ds) && (a > dsmin) && (b > dsmin))
            {
                StartParams[1] = (U1[Indice] * b + U1[Indice + 1] * a) * nab;
                StartParams[2] = (V1[Indice] * b + V1[Indice + 1] * a) * nab;
                StartParams[3] = (U2[Indice] * b + U2[Indice + 1] * a) * nab;
                StartParams[4] = (V2[Indice] * b + V2[Indice + 1] * a) * nab;

                bool HasStartPoint = PW.PerformFirstPoint(StartParams, ref StartPOn2S);
                if (HasStartPoint)
                {
                    ResultPntOn2SLine.Add(StartPOn2S);
                }
            }
            else
            {
                s += dsmin;
            }
        }

        ResultPntOn2SLine.Add(TheLine.Point(High));

        IntPatch_WLine aRWL = new(ResultPntOn2SLine, false);
        aRWL.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLPrmPrm);

        return (aRWL);
    }

    //=================================================================================================

    /// <summary>Marks in <paramref name="Map"/> the voxels of the segment between two voxels, by bisection.</summary>
    public void RemplitLin(int x1, int y1, int z1, int x2, int y2, int z2, IntPatch_PrmPrmIntersection_T3Bits Map)
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
            Map.Add(t);
        }
        if (xg <= 1 && yg <= 1 && zg <= 1)
        {
            return;
        }
        xg = (x1 + x2) >> 1;
        yg = (y1 + y2) >> 1;
        zg = (z1 + z2) >> 1;
        RemplitLin(x1, y1, z1, xg, yg, zg, Map);
        RemplitLin(x2, y2, z2, xg, yg, zg, Map);
    }

    //=================================================================================================

    /// <summary>Marks in <paramref name="Map"/> the voxels of the triangle of three voxels, by subdivision.</summary>
    public void RemplitTri(int x1, int y1, int z1, int x2, int y2, int z2, int x3, int y3, int z3,
                           IntPatch_PrmPrmIntersection_T3Bits Map)
    {
        if (x1 == x2 && x1 == x3 && y1 == y2 && y1 == y3 && z1 == z2 && z1 == z3)
        {
            if (DansGrille(x1) != 0 && DansGrille(y1) != 0 && DansGrille(z1) != 0)
            {
                int t = GrilleInteger(x1, y1, z1);
                Map.Add(t);
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
                RemplitLin(x1, y1, z1, x2, y2, z2, Map);
                RemplitLin(x1, y1, z1, x3, y3, z3, Map);
                return;
            }
            if (xg == x2 && yg == y2 && zg == z2)
            {
                RemplitLin(x2, y2, z2, x1, y1, z1, Map);
                RemplitLin(x2, y2, z2, x3, y3, z3, Map);
                return;
            }
            if (xg == x3 && yg == y3 && zg == z3)
            {
                RemplitLin(x3, y3, z3, x2, y2, z2, Map);
                RemplitLin(x3, y3, z3, x1, y1, z1, Map);
                return;
            }
            if (DansGrille(xg) != 0 && DansGrille(yg) != 0 && DansGrille(zg) != 0)
            {
                int t = GrilleInteger(xg, yg, zg);
                Map.Add(t);
            }
            if (xg != x3 || yg != y3 || zg != z3)
            {
                RemplitTri(x1, y1, z1, x2, y2, z2, xg, yg, zg, Map);
            }
            if (xg != x1 || yg != y1 || zg != z1)
            {
                RemplitTri(xg, yg, zg, x2, y2, z2, x3, y3, z3, Map);
            }
            if (xg != x2 || yg != y2 || zg != z2)
            {
                RemplitTri(x1, y1, z1, xg, yg, zg, x3, y3, z3, Map);
            }
        }
    }

    //=================================================================================================

    /// <summary>Marks in <paramref name="Map"/> the voxels <paramref name="a"/>, <paramref name="b"/>, <paramref name="c"/> (-1: none) and the triangle between them.</summary>
    public void Remplit(int a, int b, int c, IntPatch_PrmPrmIntersection_T3Bits Map)
    {
        int iax, iay, iaz, ibx, iby, ibz, icx, icy, icz;
        if (a != -1)
        {
            Map.Add(a);
        }
        if (b != -1)
        {
            Map.Add(b);
        }
        if (c != -1)
        {
            Map.Add(c);
        }

        if (a != -1 && b != -1 && c != -1)
        {
            IntegerGrille(a, out iax, out iay, out iaz);
            IntegerGrille(b, out ibx, out iby, out ibz);
            IntegerGrille(c, out icx, out icy, out icz);
            RemplitTri(iax, iay, iaz, ibx, iby, ibz, icx, icy, icz, Map);
        }
    }

    //=================================================================================================

    /// <summary>
    /// The intersection of the surfaces walked from the given points (with, on closed surfaces, their images on the
    /// other side of the seams, which are appended to <paramref name="LOfPnts"/>). Lines already computed by an
    /// earlier Perform are kept.
    /// </summary>
    public void Perform(Adaptor3d_Surface Surf1, Adaptor3d_TopolTool D1, Adaptor3d_Surface Surf2,
                        Adaptor3d_TopolTool D2, double TolTangency, double Epsilon, double Deflection,
                        double Increment, NCollection_List<IntSurf_PntOn2S> LOfPnts)
    {
        if (LOfPnts.IsEmpty())
        {
            done = true;
            return;
        }

        empt = true;
        SLin.Clear();

        double UminLig1, VminLig1, UmaxLig1, VmaxLig1;
        double UminLig2, VminLig2, UmaxLig2, VmaxLig2;
        double U1, U2, V1, V2;

        UminLig1 = Surf1.FirstUParameter();
        VminLig1 = Surf1.FirstVParameter();
        UmaxLig1 = Surf1.LastUParameter();
        VmaxLig1 = Surf1.LastVParameter();
        UminLig2 = Surf2.FirstUParameter();
        VminLig2 = Surf2.FirstVParameter();
        UmaxLig2 = Surf2.LastUParameter();
        VmaxLig2 = Surf2.LastVParameter();

        double[] Periods = new double[4];
        Periods[0] = (Surf1.IsUPeriodic()) ? Surf1.UPeriod() : 0.0;
        Periods[1] = (Surf1.IsVPeriodic()) ? Surf1.VPeriod() : 0.0;
        Periods[2] = (Surf2.IsUPeriodic()) ? Surf2.UPeriod() : 0.0;
        Periods[3] = (Surf2.IsVPeriodic()) ? Surf2.VPeriod() : 0.0;

        NCollection_List<IntSurf_PntOn2S>.Iterator IterLOP1 = new(LOfPnts);
        if (Surf1.IsUClosed() || Surf1.IsVClosed() || Surf2.IsUClosed() || Surf2.IsVClosed())
        {
            double TolPar = global::Kerf.Engine.Foundation.Precision.PConfusion();
            NCollection_List<IntSurf_PntOn2S> AdditionalPnts = new();
            double NewU1, NewV1, NewU2, NewV2;
            for (; IterLOP1.More(); IterLOP1.Next())
            {
                IntSurf_PntOn2S Pnt = IterLOP1.Value();
                Pnt.Parameters(out U1, out V1, out U2, out V2);
                IntSurf_PntOn2S NewPnt = new();
                if (Surf1.IsUClosed())
                {
                    if (Math.Abs(U1 - Surf1.FirstUParameter()) <= TolPar)
                    {
                        NewU1 = Surf1.LastUParameter();
                        NewPnt.SetValue(NewU1, V1, U2, V2);
                        AdditionalPnts.Append(NewPnt);
                    }
                    else if (Math.Abs(U1 - Surf1.LastUParameter()) <= TolPar)
                    {
                        NewU1 = Surf1.FirstUParameter();
                        NewPnt.SetValue(NewU1, V1, U2, V2);
                        AdditionalPnts.Append(NewPnt);
                    }
                }
                if (Surf1.IsVClosed())
                {
                    if (Math.Abs(V1 - Surf1.FirstVParameter()) <= TolPar)
                    {
                        NewV1 = Surf1.LastVParameter();
                        NewPnt.SetValue(U1, NewV1, U2, V2);
                        AdditionalPnts.Append(NewPnt);
                    }
                    else if (Math.Abs(V1 - Surf1.LastVParameter()) <= TolPar)
                    {
                        NewV1 = Surf1.FirstVParameter();
                        NewPnt.SetValue(U1, NewV1, U2, V2);
                        AdditionalPnts.Append(NewPnt);
                    }
                }
                if (Surf2.IsUClosed())
                {
                    if (Math.Abs(U2 - Surf2.FirstUParameter()) <= TolPar)
                    {
                        NewU2 = Surf2.LastUParameter();
                        NewPnt.SetValue(U1, V1, NewU2, V2);
                        AdditionalPnts.Append(NewPnt);
                    }
                    else if (Math.Abs(U2 - Surf2.LastUParameter()) <= TolPar)
                    {
                        NewU2 = Surf2.FirstUParameter();
                        NewPnt.SetValue(U1, V1, NewU2, V2);
                        AdditionalPnts.Append(NewPnt);
                    }
                }
                if (Surf2.IsVClosed())
                {
                    if (Math.Abs(V2 - Surf2.FirstVParameter()) <= TolPar)
                    {
                        NewV2 = Surf2.LastVParameter();
                        NewPnt.SetValue(U1, V1, U2, NewV2);
                        AdditionalPnts.Append(NewPnt);
                    }
                    else if (Math.Abs(V2 - Surf2.LastVParameter()) <= TolPar)
                    {
                        NewV2 = Surf2.FirstVParameter();
                        NewPnt.SetValue(U1, V1, U2, NewV2);
                        AdditionalPnts.Append(NewPnt);
                    }
                }
            }
            // Cut repeated points
            for (IterLOP1.Initialize(LOfPnts); IterLOP1.More(); IterLOP1.Next())
            {
                IntSurf_PntOn2S aPnt = IterLOP1.Value();
                aPnt.Parameters(out U1, out V1, out U2, out V2);
                NCollection_List<IntSurf_PntOn2S>.Iterator iter2 = new(AdditionalPnts);
                while (iter2.More())
                {
                    IntSurf_PntOn2S aNewPnt = iter2.Value();
                    aNewPnt.Parameters(out NewU1, out NewV1, out NewU2, out NewV2);
                    if (Math.Abs(U1 - NewU1) <= TolPar && Math.Abs(V1 - NewV1) <= TolPar
                        && Math.Abs(U2 - NewU2) <= TolPar && Math.Abs(V2 - NewV2) <= TolPar)
                    {
                        AdditionalPnts.Remove(iter2);
                    }
                    else
                    {
                        iter2.Next();
                    }
                }
            }

            LOfPnts.Append(AdditionalPnts);
        }

        for (IterLOP1.Initialize(LOfPnts); IterLOP1.More(); IterLOP1.Next())
        {
            IntSurf_PntOn2S Pnt = IterLOP1.Value();
            Pnt.Parameters(out U1, out V1, out U2, out V2);
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

        double SeuildPointLigne = 15.0 * Increment * Increment;

        int NbLigCalculee = 0, ver;
        double dminiPointLigne;
        bool HasStartPoint, RejetLigne;
        IntSurf_PntOn2S StartPOn2S = new();
        Point3D Point3dDebut, Point3dFin;

        NCollection_Array1<double> StartParams = new(1, 4);
        IntWalk_PWalking PW = new(Surf1, Surf2, TolTangency, Epsilon, Deflection, Increment);

        NCollection_List<IntSurf_PntOn2S>.Iterator IterLOP2 = new(LOfPnts);
        for (; IterLOP2.More(); IterLOP2.Next())
        {
            IntSurf_PntOn2S cPnt = IterLOP2.Value();
            cPnt.Parameters(out U1, out V1, out U2, out V2);

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
                    PW.Perform(StartParams, UminLig1, VminLig1, UminLig2, VminLig2, UmaxLig1, VmaxLig1, UmaxLig2,
                               VmaxLig2);
                    if (PW.IsDone())
                    {
                        if (PW.NbPoints() > 2)
                        {
                            // Try to extend the intersection line to boundary, if it is possible.
                            PW.PutToBoundary(Surf1, Surf2);

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
                                if ((Point3dDebut.Distance(verPointDebut.Value()) <= TolTangency)
                                    && (Point3dFin.Distance(verPointFin.Value()) <= TolTangency))
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

                                SeveralWlinesProcessing(Surf1, Surf2, SLin, Periods, trans1, trans2, TolTang,
                                                        Max(PW.MaxStep(0), PW.MaxStep(1)),
                                                        Max(PW.MaxStep(2), PW.MaxStep(3)), ref wline);

                                AddWLine(SLin, wline, Deflection);
                                empt = false;
                            } // !RejetLigne
                        } // PW points > 2
                    } // done is True
                } // dminiPointLigne > SeuildPointLigne
            } // HasStartPoint
        } // for( IterLOP ...
        done = true;
    }

    //=================================================================================================

    /// <summary>
    /// The intersection line walked from the given start parameters on both surfaces (none if no start point is
    /// found near them).
    /// </summary>
    public void Perform(Adaptor3d_Surface Surf1, Adaptor3d_TopolTool D1, Adaptor3d_Surface Surf2,
                        Adaptor3d_TopolTool D2, double U1Depart, double V1Depart, double U2Depart, double V2Depart,
                        double TolTangency, double Epsilon, double Deflection, double Increment)
    {
        //-- Traitement des Lignes de sections
        empt = true;
        done = true;
        SLin.Clear();

        //------------------------------------------------------------

        NCollection_Array1<double> StartParams = new(1, 4);

        IntWalk_PWalking PW = new(Surf1, Surf2, TolTangency, Epsilon, Deflection, Increment); // nIncrement);

        bool HasStartPoint; //,RejetLigne;

        IntSurf_PntOn2S StartPOn2S = new();

        Point3D Point3dDebut, Point3dFin;

        //------------------------------------------------------------

        StartParams[1] = U1Depart;
        StartParams[2] = V1Depart;
        StartParams[3] = U2Depart;
        StartParams[4] = V2Depart;

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

            PW.Perform(StartParams);
            if (PW.IsDone())
            {
                if (PW.NbPoints() > 2)
                {
                    Point3dDebut = PW.Value(1).Value();
                    Point3dFin = PW.Value(PW.NbPoints()).Value();

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
                        return;
                    }
                    IntPatch_RstInt.PutVertexOnLine(wline, Surf2, D2, Surf1, false, TolTang);
                    if (wline.NbPnts() < 2)
                    {
                        return;
                    }

                    //---------------
                    if (wline.NbVertex() == 0)
                    {
                        AddEndVertices(wline, PW, Point3dDebut, Point3dFin, TolTang);
                    }

                    //---------------
                    SLin.Append(wline);
                    empt = false;
                } // if(PW.NbPoints()>2)
            } // if(PW.IsDone())
        } // if(HasStartPoint)
    }

    //=================================================================================================

    /// <summary>True if the intersection was computed.</summary>
    public bool IsDone() => done;

    /// <summary>True if there is no intersection line.</summary>
    /// <exception cref="NotDoneException">The intersection is not done.</exception>
    public bool IsEmpty()
    {
        if (!done)
        {
            throw new NotDoneException(" IntPatch_PrmPrmIntersection ");
        }
        return (empt);
    }

    /// <summary>The number of intersection lines.</summary>
    /// <exception cref="NotDoneException">The intersection is not done.</exception>
    public int NbLines()
    {
        if (!done)
        {
            throw new NotDoneException(" IntPatch_PrmPrmIntersection ");
        }
        return (SLin.Length());
    }

    /// <summary>The line <paramref name="n"/> (the line itself, decision 609).</summary>
    /// <exception cref="NotDoneException">The intersection is not done.</exception>
    public IntPatch_Line Line(int n)
    {
        if (!done)
        {
            throw new NotDoneException(" IntPatch_PrmPrmIntersection ");
        }
        return (SLin.Value(n));
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
    public int NbPointsGrille() => (_BASE);

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
