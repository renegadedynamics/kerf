// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_SpKP.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_Builder_SpKP;
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
public partial class ChFi3d_Builder
{


    //=======================================================================
    // function : CompTra
    // purpose  : Calculate the Transition from start point.
    //=======================================================================

    //=======================================================================
    // function : CompCommonpoint
    // purpose  : Fill the commonpoint in case of a vertex.
    //=======================================================================

    //=======================================================================
    // function : CpInterf
    // purpose  : Construct new SurfData sharing faces, surface and curves.
    //=======================================================================

    //=======================================================================
    // function : CpSD
    // purpose  : Construct new SurfData sharing faces, surface and curves.
    //=======================================================================

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    internal void Trunc(ChFiDS_SurfData? SD, ChFiDS_Spine? Spine, SurfaceAdaptorBase? S1, SurfaceAdaptorBase? S2, int iedge, bool isfirst, int cntlFiOnS)
    {
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        // Return points and tangents on edge and spine.
        double wtg = SD.InterferenceOnS1().Parameter(isfirst);
        bool bid = false;
        double wsp = ParamOnSpine(DStr, wtg, SD, Spine, iedge, false, false, tolesp, ref bid);
        Point3D ped = new(), psp = new();
        Vector3D ded = new(), dsp = new();
        Vertex bout1 = new(), bout2 = new(), boutemp = new();

        EdgeCurveAdaptor bc = Spine.CurrentElementarySpine(iedge);
        // Modif against Vertex isolated on spine
        Edge support = bc.Edge();
        Exploration.Vertices(support, out bout1, out bout2);
        if (support.Orientation() == Reversed)
        {
            boutemp = bout2;
            bout2 = bout1;
            bout1 = boutemp;
        }
        if (!isfirst)
        {
            bout1 = bout2;
        }
        // finmodif
        double edf = bc.FirstParameter(), edl = bc.LastParameter();
        double edglen = edl - edf;
        if (Spine.Edges(iedge).Orientation() == Forward)
        {
            bc.D1(wtg + edf, out ped, out ded);
        }
        else
        {
            bc.D1(-wtg + edl, out ped, out ded);
            ded.Reverse();
        }
        Spine.D1(wsp, out psp, out dsp);
        Point3D p1 = new(), p2 = new();
        Surface? surf = DStr.Surface(SD.Surf()).Surface();
        Point2D pp1 = new(), pp2 = new();
        pp1 = SD.InterferenceOnS1().PCurveOnSurf().Value(wtg);
        pp2 = SD.InterferenceOnS2().PCurveOnSurf().Value(wtg);
        p1 = surf.Value(pp1.X(), pp1.Y());
        p2 = surf.Value(pp2.X(), pp2.Y());
        bool tron = false;
        double Ang = dsp.Angle(ded);
        double dis1 = psp.Distance(ped);
        double dis2 = p1.Distance(p2);
        if (Ang > M_PI / 18.0)
        {
            tron = true;
        }
        if (dis1 >= 0.1 * dis2)
        {
            tron = true;
        }
        int ivois = 0;
        if (!tron && YaUnVoisin(Spine, iedge, ref ivois, isfirst))
        {
            FaceSurfaceAdaptor? BS1 = (S1 as FaceSurfaceAdaptor);
            FaceSurfaceAdaptor? BS2 = (S2 as FaceSurfaceAdaptor);
            if (BS1 is not null && BS2 is not null)
            {
                Face FBID = new();
                Face F1 = BS1.Face();
                Face F2 = BS2.Face();
                ChFiDS_CommonPoint cp1 = SD.Vertex(isfirst, 1);
                ChFiDS_CommonPoint cp2 = SD.Vertex(isfirst, 2);
                if (!((cp1.IsOnArc() && SearchFace(Spine, cp1, F1, ref FBID))
                            || (cp2.IsOnArc() && SearchFace(Spine, cp2, F2, ref FBID))))
                {
                    tron = ChFi3d_KParticular(Spine, ivois, BS1, BS2);
                }
            }
        }
        // modification of lvt against isolated vertex
        if (!tron && YaUnVoisin(Spine, iedge, ref ivois, isfirst))
        {
            NCollection_List<Shape>.Iterator It = new();
            int nbed = -2;
            for (It.Initialize(myVEMap[bout1]); It.More(); It.Next())
            {
                nbed++;
            }
            if (nbed < 3)
            {
                tron = true;
            }
        }
        // finmodif

        if (tron)
        {
            double par = 0.0, x, y, dPar = 0;
            if (!isfirst)
            {
                par = edglen;
            }
            if (cntlFiOnS != 0)
            {
                // detect the case where FaceInterference ends before the place we are
                // going to truncate SD. Then we cut so that FaceInterference length to
                // be at least zero, not negative (eap, occ354)
                double fiPar = SD.Interference(cntlFiOnS).Parameter(!isfirst);
                bool isTheCase = isfirst ? (par > fiPar) : (par < fiPar);
                if (isTheCase)
                {
                    dPar = par - fiPar;
                    par = fiPar;
                }
            }
            for (int i = 1; i <= 2; i++)
            {
                SD.ChangeInterference(i).SetParameter(par, isfirst);
                Curve2D? pc = SD.Interference(i).PCurveOnSurf();
                pc.Value(par).Coord(out x, out y);
                SD.ChangeVertex(isfirst, i).Reset();
                SD.ChangeVertex(isfirst, i).SetPoint(surf.Value(x, y));
                if (isfirst)
                {
                    SD.FirstSpineParam(Spine.FirstParameter(iedge) - dPar);
                }
                else
                {
                    SD.LastSpineParam(Spine.LastParameter(iedge) - dPar);
                }
            }
        }
    }

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=======================================================================
    // function : SplitKPart
    // purpose  : Reconstruct SurfData depending on restrictions of faces.
    //=======================================================================

    public bool SplitKPart(ChFiDS_SurfData? Data, NCollection_Sequence<ChFiDS_SurfData> SetData, ChFiDS_Spine? Spine, int Iedge, SurfaceAdaptorBase? S1, TopolTool? I1, SurfaceAdaptorBase? S2, TopolTool? I2, ref bool intf, ref bool intl)
    {
        // The hatching of each faces is started by tangency lines.

        double pitol = global::Kerf.Engine.Foundation.Precision.PIntersection();

        NCollection_DataMap<int, Curve2DAdaptorBase> M1 = new(), M2 = new();
        int iH1 = 0, iH2 = 0;
        int Nb1 = 1, Nb2 = 1;

        // Cutting of tangency lines (hatching).
        Geom2dHatch_Intersector Inter = new(pitol, pitol);
        Geom2dHatch_Hatcher H1 = new(Inter, tol2d, tolapp3d), H2 = new(Inter, tol2d, tolapp3d);
        int ie = 0;
        Curve2D? C1 = Data.InterferenceOnS1().PCurveOnFace();
        CurveAdaptor2D ll1 = new();
        if (C1 is not null)
        {
            ll1.Load(C1);
            for (I1.Init(); I1.More(); I1.Next())
            {
                EdgeCurve2DAdaptor? Bc = (I1.Value() as EdgeCurve2DAdaptor);
                CurveAdaptor2D? Gc = (I1.Value() as CurveAdaptor2D);
                if (Bc is null)
                {
                    ie = H1.AddElement(Gc, Forward);
                }
                else
                {
                    ie = H1.AddElement(Bc, Bc.Edge().Orientation());
                }
                M1.Bind(ie, I1.Value());
            }
            iH1 = H1.Trim(ll1);
            H1.ComputeDomains(iH1);
            if (!H1.IsDone(iH1))
            {
                return false;
            }
            Nb1 = H1.NbDomains(iH1);
            if (Nb1 == 0)
            {
                return false;
            }
        }

        Curve2D? C2 = Data.InterferenceOnS2().PCurveOnFace();
        CurveAdaptor2D ll2 = new();
        if (C2 is not null)
        {
            ll2.Load(C2);
            for (I2.Init(); I2.More(); I2.Next())
            {
                EdgeCurve2DAdaptor? Bc = (I2.Value() as EdgeCurve2DAdaptor);
                CurveAdaptor2D? Gc = (I2.Value() as CurveAdaptor2D);
                if (Bc is null)
                {
                    ie = H2.AddElement(Gc, Forward);
                }
                else
                {
                    ie = H2.AddElement(Bc, Bc.Edge().Orientation());
                }
                M2.Bind(ie, I2.Value());
            }
            iH2 = H2.Trim(ll2);
            H2.ComputeDomains(iH2);
            if (!H2.IsDone(iH2))
            {
                return false;
            }
            Nb2 = H2.NbDomains(iH2);
            if (Nb2 == 0)
            {
                return false;
            }
        }

        // Return start and end vertexes of the Spine
        Vertex bout1 = new(), bout2 = new(), boutemp = new();
        EdgeCurveAdaptor bc = Spine.CurrentElementarySpine(Iedge);
        Edge support = bc.Edge();
        Exploration.Vertices(support, out bout1, out bout2);
        if (support.Orientation() == Reversed)
        {
            boutemp = bout2;
            bout2 = bout1;
            bout1 = boutemp;
        }

        // Return faces.
        Face F1 = new(), F2 = new();
        FaceSurfaceAdaptor? bhs = (S1 as FaceSurfaceAdaptor);
        if (bhs is not null)
        {
            F1 = bhs.Face();
        }
        bhs = (S2 as FaceSurfaceAdaptor);
        if (bhs is not null)
        {
            F2 = bhs.Face();
        }
        Face FBID = new();

        // Restriction of SurfDatas by cut lines.
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        ChFiDS_SurfData? CD = Data;
        CD.ChangeIndexOfS1(DStr.AddShape(F1));
        CD.ChangeIndexOfS2(DStr.AddShape(F2));

        double f1 = 0.0, l1 = 0.0, f2 = 0.0, l2 = 0.0;
        NCollection_Array1<int> Ind1 = new(1, Nb1), Ind2 = new(1, Nb2);
        double wref = 0.0;

        int onS = 1;// switcher of access to surfs of SurfData, eap occ293
        int cntlFiOnS = 0;// FaceInterference to control length in OnSame
                                              // situation, eap occ354

        if (C1 is null && C2 is null)
        {
            return false;
        }
        else if (C1 is null || (Nb1 == 1 && !H1.Domain(iH1, 1).HasFirstPoint()))
        {
            // It is checked if the point 2d of the degenerated edge is in the face.
            if (C1 is null)
            {
                Point2D p2d1 = CD.Get2dPoints(false, 1);
                Containment situ = I1.Classify(p2d1, 1.0e-8, false);
                if (situ == Out)
                {
                    return false;
                }
            }

            // Parsing of domains by increasing parameters,
            if (!Tri(H2, iH2, Ind2, wref, 0.0, pitol, ref Nb2))
            {
                return false;
            }
            // Filling of SurfData
            for (int i = 1; i <= Nb2; i++)
            {
                HatchGen_Domain Dom2 = H2.Domain(iH2, Ind2[i]);
                FillSD(DStr, CD, M2, Dom2, Dom2.FirstPoint().Parameter(), true, 2, pitol, bout1);
                FillSD(DStr, CD, M2, Dom2, Dom2.SecondPoint().Parameter(), false, 2, pitol, bout2);
                SetData.Append(CD);
                CD = CpSD(DStr, CD);
            }
            if (intf)
            {
                ref ChFiDS_SurfData? sd = ref SetData.ChangeValue(1);
                ref ChFiDS_CommonPoint CP2 = ref sd.ChangeVertexFirstOnS2();
                if (CP2.IsOnArc() && Spine.FirstStatus() == ChFiDS_OnSame)
                {
                    intf = !SearchFace(Spine, CP2, F2, ref FBID);
                }
                else
                {
                    intf = false;
                }
            }
            if (intl)
            {
                ref ChFiDS_SurfData? sd = ref SetData.ChangeValue(SetData.Length());
                ref ChFiDS_CommonPoint CP2 = ref sd.ChangeVertexLastOnS2();
                if (CP2.IsOnArc() && Spine.LastStatus() == ChFiDS_OnSame)
                {
                    intl = !SearchFace(Spine, CP2, F2, ref FBID);
                }
                else
                {
                    intl = false;
                }
            }
        }
        else if (C2 is null || (Nb2 == 1 && !H2.Domain(iH2, 1).HasFirstPoint()))
        {
            // It is checked if the point 2d of the degenerated is in the face.
            if (C2 is null)
            {
                Point2D p2d2 = CD.Get2dPoints(false, 2);
                Containment situ = I2.Classify(p2d2, 1.0e-8, false);
                if (situ == Out)
                {
                    return false;
                }
            }

            // Parsing of domains by increasing parameters,
            if (!Tri(H1, iH1, Ind1, wref, 0.0, pitol, ref Nb1))
            {
                return false;
            }
            // Filling of SurfData
            for (int i = 1; i <= Nb1; i++)
            {
                HatchGen_Domain Dom1 = H1.Domain(iH1, Ind1[i]);
                FillSD(DStr, CD, M1, Dom1, Dom1.FirstPoint().Parameter(), true, 1, pitol, bout1);
                FillSD(DStr, CD, M1, Dom1, Dom1.SecondPoint().Parameter(), false, 1, pitol, bout2);
                SetData.Append(CD);
                CD = CpSD(DStr, CD);
            }
            if (intf)
            {
                ref ChFiDS_SurfData? sd = ref SetData.ChangeValue(1);
                ref ChFiDS_CommonPoint CP1 = ref sd.ChangeVertexFirstOnS1();
                if (CP1.IsOnArc() && Spine.FirstStatus() == ChFiDS_OnSame)
                {
                    intf = !SearchFace(Spine, CP1, F1, ref FBID);
                }
                else
                {
                    intf = false;
                }
            }
            if (intl)
            {
                ref ChFiDS_SurfData? sd = ref SetData.ChangeValue(SetData.Length());
                ref ChFiDS_CommonPoint CP1 = ref sd.ChangeVertexLastOnS1();
                if (CP1.IsOnArc() && Spine.LastStatus() == ChFiDS_OnSame)
                {
                    intl = !SearchFace(Spine, CP1, F1, ref FBID);
                }
                else
                {
                    intl = false;
                }
            }
        }
        else
        {

            // Parsing of domains by increasing parameters,
            // if there is a 2d circle on a plane, one goes on 2D line of opposite face.
            double period1 = 0.0, period2 = 0.0;
            if (ll1.IsPeriodic)
            {
                if (!Tri(H2, iH2, Ind2, wref, 0.0, pitol, ref Nb2))
                {
                    return false;
                }
                period1 = ll1.Period();
                if (!Tri(H1, iH1, Ind1, wref, period1, pitol, ref Nb1))
                {
                    return false;
                }
            }
            else
            {
                if (!Tri(H1, iH1, Ind1, wref, 0.0, pitol, ref Nb1))
                {
                    return false;
                }
                if (ll2.IsPeriodic)
                {
                    period2 = ll2.Period();
                }
                if (!Tri(H2, iH2, Ind2, wref, period2, pitol, ref Nb2))
                {
                    return false;
                }
            }

            // Filling of SurfData
            NCollection_Sequence<int> ion1 = new(), ion2 = new();
            for (int i = 1; i <= Nb1; i++)
            {
                HatchGen_Domain Dom1 = H1.Domain(iH1, Ind1[i]);
                int nbcoup1 = 1;
                bool acheval1 = AdjustParam(Dom1, ref f1, ref l1, wref, period1, pitol);
                if (acheval1)
                {
                    nbcoup1 = 2;
                }
                for (int icoup1 = 1; icoup1 <= nbcoup1; icoup1++)
                {
                    for (int j = 1; j <= Nb2; j++)
                    {
                        HatchGen_Domain Dom2 = H2.Domain(iH2, j);
                        int nbcoup2 = 1;
                        bool acheval2 = AdjustParam(Dom2, ref f2, ref l2, wref, period2, pitol);
                        if (acheval2)
                        {
                            nbcoup2 = 2;
                        }
                        for (int icoup2 = 1; icoup2 <= nbcoup2; icoup2++)
                        {
                            if (f2 <= l1 && f1 <= l2)
                            {
                                if (f1 >= f2 - tol2d)
                                {
                                    FillSD(DStr, CD, M1, Dom1, f1, true, 1, pitol, bout1);
                                }
                                if (f2 >= f1 - tol2d)
                                {
                                    FillSD(DStr, CD, M2, Dom2, f2, true, 2, pitol, bout1);
                                }
                                if (l1 >= l2 - tol2d)
                                {
                                    FillSD(DStr, CD, M2, Dom2, l2, false, 2, pitol, bout2);
                                }
                                if (l2 >= l1 - tol2d)
                                {
                                    FillSD(DStr, CD, M1, Dom1, l1, false, 1, pitol, bout2);
                                }
                                SetData.Append(CD);
                                CD = CpSD(DStr, CD);
                                ion1.Append(i);
                                ion2.Append(j);
                            }
                            f2 += period2;
                            l2 += period2;
                        }
                    }
                    f1 += period1;
                    l1 += period1;
                }
            }

            // Processing of extensions.
            // Do not truncate, otherwise, problems of intersection for PerformCorner
            // -----------------------------------------------------------------
            // After call of SplitKPart in PerformSetOfKPart, spines have been
            // extended to the extremities by methods Extent to permit
            // intersections. Extensions of  SurfData are preserved.

            if (intf)
            {
                // We are at the beginning of the spine
                //-------------------------
                int ifirst = 0;
                double dist = RealLast(), ptg, dsp;
                EdgeCurveAdaptor ed = Spine.CurrentElementarySpine(Iedge);
                for (int i1 = 1; i1 <= SetData.Length(); i1++)
                {
                    ref ChFiDS_SurfData? CD1 = ref SetData.ChangeValue(i1);
                    ref ChFiDS_CommonPoint CP1l = ref CD1.ChangeVertexFirstOnS1();
                    ref ChFiDS_CommonPoint CP2l = ref CD1.ChangeVertexFirstOnS2();
                    if (CP1l.IsOnArc() && !SearchFace(Spine, CP1l, F1, ref FBID))
                    {
                        ptg = CD1.InterferenceOnS1().FirstParameter();
                        dsp = ComputeAbscissa(ed, ptg);
                        if (Math.Abs(dsp) < dist)
                        {
                            ifirst = i1;
                            dist = Math.Abs(dsp);
                        }
                    }
                    else if (CP2l.IsOnArc() && !SearchFace(Spine, CP2l, F2, ref FBID))
                    {
                        ptg = CD1.InterferenceOnS2().FirstParameter();
                        dsp = ComputeAbscissa(ed, ptg);
                        if (Math.Abs(dsp) < dist)
                        {
                            ifirst = i1;
                            dist = Math.Abs(dsp);
                        }
                    }
                }
                if (ifirst > 1)
                {
                    SetData.Remove(1, ifirst - 1);
                    ion1.Remove(1, ifirst - 1);
                    ion2.Remove(1, ifirst - 1);
                }
                if (SetData.IsEmpty())
                {
                    return false;
                }
                ref ChFiDS_SurfData? CD2 = ref SetData.ChangeValue(1);
                ref ChFiDS_CommonPoint CP1 = ref CD2.ChangeVertexFirstOnS1();
                ref ChFiDS_CommonPoint CP2 = ref CD2.ChangeVertexFirstOnS2();
                ChFiDS_CommonPoint sov = new();

                if (CP1.IsOnArc() && CP2.IsOnArc())
                {
                    intf = !SearchFace(Spine, CP1, F1, ref FBID) && !SearchFace(Spine, CP2, F2, ref FBID);
                }
                else if (CP1.IsOnArc())
                {
                    sov = CP2;
                    if (!SearchFace(Spine, CP1, F1, ref FBID))
                    {
                        FillSD(DStr, CD2, M2, H2.Domain(iH2, Ind2[ion2.First()]),
                                      H2.Domain(iH2, Ind2[ion2.First()]).FirstPoint().Parameter(), true, 2, pitol, bout1);
                        if (!CP2.IsOnArc() || (CP2.IsOnArc() && SearchFace(Spine, CP2, F2, ref FBID)))
                        {
                            CP2 = sov;
                            if (Spine.FirstStatus() != ChFiDS_OnSame)
                            {
                                CD2.ChangeInterference(2).SetParameter(CD2.Interference(1).Parameter(true), true);
                                intf = false;
                            }
                        }
                    }
                    else
                    {
                        intf = false;
                    }
                }
                else if (CP2.IsOnArc())
                {
                    sov = CP1;
                    if (!SearchFace(Spine, CP2, F2, ref FBID))
                    {
                        FillSD(DStr, CD2, M1, H1.Domain(iH1, Ind1[ion1.First()]),
                                      H1.Domain(iH1, Ind1[ion1.First()]).FirstPoint().Parameter(), true, 1, pitol, bout1);
                        if (!CP1.IsOnArc() || (CP1.IsOnArc() && SearchFace(Spine, CP1, F1, ref FBID)))
                        {
                            CP1 = sov;
                            if (Spine.FirstStatus() != ChFiDS_OnSame)
                            {
                                CD2.ChangeInterference(1).SetParameter(CD2.Interference(2).Parameter(true), true);
                                intf = false;
                            }
                        }
                    }
                    else
                    {
                        intf = false;
                    }
                }
                // select <onS> switcher so that to get on spine params from
                // Interference with a face where both edges at corner are OnSame
                // eap occ293
                if (intf && Spine.FirstStatus() == ChFiDS_OnSame)
                {
                    Edge[] threeE = new Edge[3];
                    ChFi3d_cherche_element(bout1, support, F1, ref threeE[0], ref boutemp);
                    ChFi3d_cherche_element(bout1, support, F2, ref threeE[1], ref boutemp);
                    threeE[2] = support;
                    if (ChFi3d_EdgeState(threeE, myEFMap) == ChFiDS_OnSame)
                    {
                        onS = 1;
                    }
                    else
                    {
                        onS = 2;
                    }
                    cntlFiOnS = 3 - onS;
                }
            }
            if (intl)
            {
                // we are at the end of the spine
                //-----------------------
                int ilast = 0;
                double dist = RealLast(), ptg, dsp;
                double f = Spine.FirstParameter(Iedge);
                double l = Spine.LastParameter(Iedge);
                EdgeCurveAdaptor ed = Spine.CurrentElementarySpine(Iedge);
                for (int i2 = 1; i2 <= SetData.Length(); i2++)
                {
                    ref ChFiDS_SurfData? CD3 = ref SetData.ChangeValue(i2);
                    ref ChFiDS_CommonPoint CP1l = ref CD3.ChangeVertexLastOnS1();
                    ref ChFiDS_CommonPoint CP2l = ref CD3.ChangeVertexLastOnS2();
                    if (CP1l.IsOnArc() && !SearchFace(Spine, CP1l, F1, ref FBID))
                    {
                        ptg = CD3.InterferenceOnS1().LastParameter();
                        dsp = -ComputeAbscissa(ed, ptg) - f + l;
                        if (Math.Abs(dsp) < dist)
                        {
                            ilast = i2;
                            dist = Math.Abs(dsp);
                        }
                    }
                    else if (CP2l.IsOnArc() && !SearchFace(Spine, CP2l, F2, ref FBID))
                    {
                        ptg = CD3.InterferenceOnS2().LastParameter();
                        dsp = -ComputeAbscissa(ed, ptg) - f + l;
                        if (Math.Abs(dsp) < dist)
                        {
                            ilast = i2;
                            dist = Math.Abs(dsp);
                        }
                    }
                }
                int lll = SetData.Length();
                if (ilast < lll)
                {
                    SetData.Remove(ilast + 1, lll);
                    ion1.Remove(ilast + 1, lll);
                    ion2.Remove(ilast + 1, lll);
                }
                if (SetData.IsEmpty())
                {
                    return false;
                }
                ref ChFiDS_SurfData? CD4 = ref SetData.ChangeValue(SetData.Length());
                ref ChFiDS_CommonPoint CP1 = ref CD4.ChangeVertexLastOnS1();
                ref ChFiDS_CommonPoint CP2 = ref CD4.ChangeVertexLastOnS2();
                ChFiDS_CommonPoint sov = new();
                if (CP1.IsOnArc() && CP2.IsOnArc())
                {
                    intl = !SearchFace(Spine, CP1, F1, ref FBID) && !SearchFace(Spine, CP2, F2, ref FBID);
                }
                else if (CP1.IsOnArc())
                {
                    sov = CP2;
                    if (!SearchFace(Spine, CP1, F1, ref FBID))
                    {
                        FillSD(DStr, CD4, M2, H2.Domain(iH2, Ind2[ion2.Last()]),
                                      H2.Domain(iH2, Ind2[ion2.Last()]).SecondPoint().Parameter(), false, 2, pitol, bout2);
                        if (!CP2.IsOnArc() || (CP2.IsOnArc() && SearchFace(Spine, CP2, F2, ref FBID)))
                        {
                            CP2 = sov;
                            if (Spine.LastStatus() != ChFiDS_OnSame)
                            {
                                CD4.ChangeInterference(2).SetParameter(CD4.Interference(1).Parameter(false), false);
                                intl = false;
                            }
                        }
                    }
                    else
                    {
                        intl = false;
                    }
                }
                else if (CP2.IsOnArc())
                {
                    sov = CP1;
                    if (!SearchFace(Spine, CP2, F2, ref FBID))
                    {
                        FillSD(DStr, CD4, M1, H1.Domain(iH1, Ind1[ion1.Last()]),
                                      H1.Domain(iH1, Ind1[ion1.Last()]).SecondPoint().Parameter(), false, 1, pitol, bout2);
                        if (!CP1.IsOnArc() || (CP1.IsOnArc() && SearchFace(Spine, CP1, F1, ref FBID)))
                        {
                            CP1 = sov;
                            if (Spine.LastStatus() != ChFiDS_OnSame)
                            {
                                CD4.ChangeInterference(1).SetParameter(CD4.Interference(2).Parameter(false), false);
                                intl = false;
                            }
                        }
                    }
                    else
                    {
                        intl = false;
                    }
                }

                // select <onS> switcher so that to get on spine params from
                // Interference with a face where both edges at corner are OnSame
                // eap occ293
                if (intl && Spine.LastStatus() == ChFiDS_OnSame)
                {
                    Edge[] threeE = new Edge[3];
                    ChFi3d_cherche_element(bout2, support, F1, ref threeE[0], ref boutemp);
                    ChFi3d_cherche_element(bout2, support, F2, ref threeE[1], ref boutemp);
                    threeE[2] = support;
                    if (ChFi3d_EdgeState(threeE, myEFMap) == ChFiDS_OnSame)
                    {
                        onS = 1;
                    }
                    else
                    {
                        onS = 2;
                    }
                    cntlFiOnS = 3 - onS;
                }
            }
        }

        if (!intf)
        {
            // SurfData are entirely suspended before the beginning of the edge.
            bool okdoc = SetData.IsEmpty();
            int i = 1;
            while (!okdoc)
            {
                ref ChFiDS_SurfData? CD5 = ref SetData.ChangeValue(i);
                double ltg = CD5.Interference(onS).LastParameter();
                double Nl = ComputeAbscissa(Spine.CurrentElementarySpine(Iedge), ltg);
                if (Nl < -tolesp)
                {
                    SetData.Remove(i);
                }
                else
                {
                    i++;
                }
                okdoc = (SetData.IsEmpty() || i > SetData.Length());
            }
        }
        if (!intl)
        {
            // SurfData are entirely suspended after the end of the edge.
            bool okdoc = SetData.IsEmpty();
            int i = 1;
            while (!okdoc)
            {
                ref ChFiDS_SurfData? CD6 = ref SetData.ChangeValue(i);
                double ftg = CD6.Interference(onS).FirstParameter();
                double f = Spine.FirstParameter(Iedge);
                double l = Spine.LastParameter(Iedge);
                double Nl = ComputeAbscissa(Spine.CurrentElementarySpine(Iedge), ftg);
                if (Nl > (l - f + tolesp))
                {
                    SetData.Remove(i);
                }
                else
                {
                    i++;
                }
                okdoc = (SetData.IsEmpty() || i > SetData.Length());
            }
        }
        // Add parameters of the spine on SurfDatas.
        //  for (int i = 1; i <= SetData.Length(); i++) {
        int iSD = 0;
        for (iSD = 1; iSD <= SetData.Length(); iSD++)
        {
            bool pokdeb = false, pokfin = false;
            ref ChFiDS_SurfData? CD7 = ref SetData.ChangeValue(iSD);
            double ftg = CD7.Interference(onS).FirstParameter();
            double ltg = CD7.Interference(onS).LastParameter();
            double fsp = ParamOnSpine(DStr, ftg, CD7, Spine, Iedge, intf, intl, tolesp, ref pokdeb);
            if (!pokdeb)
            {
                fsp = ResetProl(DStr, CD7, Spine, Iedge, true);
            }
            double lsp = ParamOnSpine(DStr, ltg, CD7, Spine, Iedge, intf, intl, tolesp, ref pokfin);
            if (!pokfin)
            {
                lsp = ResetProl(DStr, CD7, Spine, Iedge, false);
            }
            if (Spine.IsPeriodic() && Iedge == Spine.NbEdges() && lsp < fsp)
            {
                lsp += Spine.Period();
            }
            else if (Spine.IsPeriodic() && Iedge == 1 && lsp < fsp)
            {
                fsp -= Spine.Period();
            }
            CD7.FirstSpineParam(fsp);
            CD7.LastSpineParam(lsp);
        }

        if (intf && !SetData.IsEmpty())
        {
            // extension of the spine
            Spine.SetFirstParameter(SetData.First().FirstSpineParam());
        }
        else
        {
            // Trnncation at the beginning.
            for (iSD = 1; iSD <= SetData.Length(); iSD++)
            {
                ref ChFiDS_SurfData? CD8 = ref SetData.ChangeValue(iSD);
                double fsp = CD8.FirstSpineParam();
                double lsp = CD8.LastSpineParam();
                if (lsp > Spine.FirstParameter(Iedge))
                {
                    if (fsp > Spine.FirstParameter(Iedge))
                    {
                        break;
                    }
                    else
                    {
                        Trunc(CD8, Spine, S1, S2, Iedge, true, cntlFiOnS);
                        break;
                    }
                }
            }
            if (iSD > 1)
            {
                SetData.Remove(1, iSD - 1);
            }
        }

        if (intl && !SetData.IsEmpty())
        {
            // extension of the spine
            Spine.SetLastParameter(SetData.Last().LastSpineParam());
        }
        else
        {
            // Truncation at the end.
            for (iSD = SetData.Length(); iSD >= 1; iSD--)
            {
                ref ChFiDS_SurfData? CD9 = ref SetData.ChangeValue(iSD);
                double fsp = CD9.FirstSpineParam();
                double lsp = CD9.LastSpineParam();
                if (fsp < Spine.LastParameter(Iedge))
                {
                    if (lsp < Spine.LastParameter(Iedge))
                    {
                        break;
                    }
                    else
                    {
                        Trunc(CD9, Spine, S1, S2, Iedge, false, cntlFiOnS);
                        break;
                    }
                }
            }
            if (iSD < SetData.Length())
            {
                SetData.Remove(iSD + 1, SetData.Length());
            }
        }
        return true;
    }

}

internal static class ChFi3d_Builder_SpKP
{
    internal static Orientation CompTra(Orientation O1, Orientation O2, bool isfirst)
    {
        if (isfirst)
        {
            return global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(O1, O2));
        }
        else
        {
            return global::Kerf.Engine.Topology.TopAbs.Compose(O1, O2);
        }
    }

    internal static void CompCommonPoint(ref ChFiDS_CommonPoint FilPoint, in Edge arc, HatchGen_PointOnElement PE, Orientation Or)
    {
        Orientation pos = PE.Position();
        Vertex V = new();
        if (pos == Forward)
        {
            V = Exploration.FirstVertex(arc);
        }
        else
        {
            V = Exploration.LastVertex(arc);
        }
        FilPoint.SetVertex(V);
        FilPoint.SetArc(global::Kerf.Engine.Foundation.Precision.PIntersection(), arc, PE.Parameter(), global::Kerf.Engine.Topology.TopAbs.Compose(arc.Orientation(), Or));
    }

    internal static ChFiDS_FaceInterference CpInterf(TopOpeBRepDS_DataStructure DStr, in ChFiDS_FaceInterference FI)
    {
        ChFiDS_FaceInterference newF = FI;
        TopOpeBRepDS_Curve toc = DStr.Curve(FI.LineIndex());
        Curve3D? newC = null;
        if (toc.Curve() is not null)
        {
            newC = (toc.Curve().Copy() as Curve3D);
        }
        newF.SetLineIndex(DStr.AddCurve(new TopOpeBRepDS_Curve(newC, toc.Tolerance())));

        if (FI.PCurveOnFace() is not null)
        {
            newF.ChangePCurveOnFace() = (FI.PCurveOnFace().Copy() as Curve2D);
        }
        if (FI.PCurveOnSurf() is not null)
        {
            newF.ChangePCurveOnSurf() = (FI.PCurveOnSurf().Copy() as Curve2D);
        }
        return newF;
    }

    internal static ChFiDS_SurfData? CpSD(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData? Data)
    {
        ChFiDS_SurfData? newData = new ChFiDS_SurfData();
        TopOpeBRepDS_Surface tos = DStr.Surface(Data.Surf());
        Surface? newS = (tos.Surface().Copy() as Surface);
        double tol = tos.Tolerance();
        newData.ChangeSurf(DStr.AddSurface(new TopOpeBRepDS_Surface(newS, tol)));
        newData.ChangeIndexOfS1(Data.IndexOfS1());
        newData.ChangeIndexOfS2(Data.IndexOfS2());
        newData.ChangeOrientation() = Data.Orientation();
        newData.ChangeInterferenceOnS1() = CpInterf(DStr, Data.InterferenceOnS1());
        newData.ChangeInterferenceOnS2() = CpInterf(DStr, Data.InterferenceOnS2());
        return newData;
    }

    internal static bool AdjustParam(in HatchGen_Domain Dom, ref double f, ref double l, double wref, double period, double pitol)
    {
        if (Dom.HasFirstPoint())
        {
            f = Dom.FirstPoint().Parameter();
        }
        else
        {
            f = 0.0;
        }
        if (Dom.HasSecondPoint())
        {
            l = Dom.SecondPoint().Parameter();
        }
        else
        {
            l = period;
        }
        if (period == 0.0)
        {
            return false;
        }

        f = global::Kerf.Engine.Geometry.ElCLib.InPeriod(f, wref - pitol, wref + period - pitol);
        l = global::Kerf.Engine.Geometry.ElCLib.InPeriod(l, wref + pitol, wref + period + pitol);
        if (l < f)
        {
            f -= period;
            return true;
        }
        return false;
    }

    internal static double ComputeAbscissa(EdgeCurveAdaptor C, double U)
    {
        switch (C.GetType())
        {
            case Line:
                return U;
            case Circle:
                return C.Circle().Radius() * U;
            default:
                return 0;
        }
    }

    internal static double ParamOnSpine(TopOpeBRepDS_DataStructure DStr, double ptg, ChFiDS_SurfData? CD, ChFiDS_Spine? Spine, int iedge, bool intf, bool intl, double tol, ref bool pok)
    {
        double Nl = 0.0;
        double f = Spine.FirstParameter(iedge);
        double l = Spine.LastParameter(iedge);

        Nl = ComputeAbscissa(Spine.CurrentElementarySpine(iedge), ptg) + f;
        if ((Nl >= (f - tol) || intf) && (Nl <= (l + tol) || intl))
        {
            pok = true;
            return Nl;
        }
        else
        {
            // construction of the plane containing the section of CD with parameter ptg.
            Point3D PP = new();
            Vector3D VV = new();
            Curve3D? c3d = null;
            if (CD.InterferenceOnS1().LineIndex() != 0)
            {
                c3d = DStr.Curve(CD.InterferenceOnS1().LineIndex()).Curve();
            }
            if (c3d is null)
            {
                c3d = DStr.Curve(CD.InterferenceOnS2().LineIndex()).Curve();
            }
            c3d.D1(ptg, out PP, out VV);

            Plane3D nlp = new(PP, new Direction3D(VV));
            PlaneSurface? pln = new PlaneSurface(nlp);
            SurfaceAdaptor? plan = new SurfaceAdaptor(pln);

            // intersection plane spine.
            bool found = false;
            bool fini = false;
            int sens = 1;
            if (Nl <= f)
            {
                sens = -1;
            }
            int ii = iedge + sens;
            if (Spine.IsPeriodic())
            {
                if (ii <= 0)
                {
                    ii += Spine.NbEdges();
                }
                if (ii > Spine.NbEdges())
                {
                    ii -= Spine.NbEdges();
                }
            }
            else if (ii < 1 || ii > Spine.NbEdges())
            {
                pok = true;
                return Nl;
            }
            EdgeCurveAdaptor? HE = new EdgeCurveAdaptor();
            EdgeCurveAdaptor CE = HE; // a reference to *HE in OCCT

            while (!found && !fini)
            {
                Orientation O = Spine.Edges(ii).Orientation();
                bool First = ((O == Forward && sens == 1) || (O == Reversed && sens == -1));
                CE.Initialize(Spine.Edges(ii));
                double tolc = CE.Resolution(tol);
                found = ChFi3d_InterPlaneEdge(plan, HE, ref Nl, First, tolc);
                Point3D point = CE.Value(Nl);
                if (found)
                {
                    Nl = Spine.Absc(Nl, ii);
                }
                point = Spine.Value(Nl);

                ii += sens;
                if (Spine.IsPeriodic())
                {
                    if (ii <= 0)
                    {
                        ii += Spine.NbEdges();
                    }
                    if (ii > Spine.NbEdges())
                    {
                        ii -= Spine.NbEdges();
                    }
                    fini = (ii == iedge);
                }
                else
                {
                    fini = (ii < 1 || ii > Spine.NbEdges());
                }
            }
            pok = found;
            return Nl;
        }
    }

    internal static bool YaUnVoisin(ChFiDS_Spine? Spine, int iedge, ref int ivois, bool isfirst)
    {
        int nbed = Spine.NbEdges();
        if (nbed == 1)
        {
            return false;
        }
        bool periodic = Spine.IsPeriodic();
        if (isfirst)
        {
            ivois = iedge - 1;
        }
        else
        {
            ivois = iedge + 1;
        }
        if (periodic)
        {
            if (ivois == 0)
            {
                ivois = nbed;
            }
            if (ivois == nbed + 1)
            {
                ivois = 1;
            }
        }
        return (ivois > 0 && ivois <= nbed);
    }

    internal static double ResetProl(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData? CD, ChFiDS_Spine? Spine, int iedge, bool isfirst)
    {
        EdgeCurveAdaptor bc = Spine.CurrentElementarySpine(iedge);
        double edglen = bc.LastParameter() - bc.FirstParameter();
        Surface? surf = DStr.Surface(CD.Surf()).Surface();
        double par = 0.0, x, y;
        if (!isfirst)
        {
            par = edglen;
        }
        double sppar = 0.0;
        for (int i = 1; i <= 2; i++)
        {
            CD.ChangeInterference(i).SetParameter(par, isfirst);
            Curve2D? pc = CD.Interference(i).PCurveOnSurf();
            pc.Value(par).Coord(out x, out y);
            CD.ChangeVertex(isfirst, i).Reset();
            CD.ChangeVertex(isfirst, i).SetPoint(surf.Value(x, y));
            if (isfirst)
            {
                sppar = Spine.FirstParameter(iedge);
                CD.FirstSpineParam(sppar);
            }
            else
            {
                sppar = Spine.LastParameter(iedge);
                CD.LastSpineParam(sppar);
            }
        }
        return sppar;
    }

    internal static bool Tri(Geom2dHatch_Hatcher H, int iH, NCollection_Array1<int> Ind, double wref, double period, double pitol, ref int Nbdom)
    {
        //  for (int i = 1; i <= Nbdom; i++) { Ind[i] = i; }
        int i = 0;
        for (i = 1; i <= Nbdom; i++)
        {
            Ind[i] = i;
        }
        double f1 = 0.0, f2 = 0.0, l = 0.0;
        int tmp = 0;
        bool Invert = true;

        while (Invert)
        {
            Invert = false;
            for (i = 1; i < Nbdom; i++)
            {
                AdjustParam(H.Domain(iH, Ind[i]), ref f1, ref l, wref, period, pitol);
                AdjustParam(H.Domain(iH, Ind[i + 1]), ref f2, ref l, wref, period, pitol);
                if (f2 < f1)
                {
                    tmp = Ind[i];
                    Ind[i] = Ind[i + 1];
                    Ind[i + 1] = tmp;
                    Invert = true;
                }
            }
        }

        int iSansFirst = 0, iSansLast = 0;

        if (Nbdom != 1)
        {
            for (i = 1; i <= Nbdom; i++)
            {
                if (!H.Domain(iH, Ind[i]).HasFirstPoint())
                {
                    iSansFirst = i;
                }
                if (!H.Domain(iH, Ind[i]).HasSecondPoint())
                {
                    iSansLast = i;
                }
            }
        }
        if (iSansFirst != 0)
        {
            if (iSansLast == 0)
            {
                return false;
            }
            // OCCT casts the constness of the hatcher's domain and point away and edits them in place
            ref HatchGen_Domain Dom = ref H.ChangeDomain(iH, Ind[iSansFirst]);
            HatchGen_PointOnHatching PH = H.Domain(iH, Ind[iSansLast]).FirstPoint();
            double NewPar = H.HatchingCurve(iH).FirstParameter() - period
                            + H.Domain(iH, Ind[iSansLast]).FirstPoint().Parameter();
            PH.SetParameter(NewPar);
            Dom.SetFirstPoint(PH);

            for (int k = iSansLast; k < Nbdom; k++)
            {
                Ind[k] = Ind[k + 1];
            }
            Nbdom--;
        }
        return true;
    }

    internal static void FillSD(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData? CD, NCollection_DataMap<int, Curve2DAdaptorBase> M, in HatchGen_Domain Dom, double ponH, bool isFirst, int ons, double pitol, in Vertex bout)
    {
        int opp = 3 - ons;
        ref ChFiDS_CommonPoint Pons = ref CD.ChangeVertex(isFirst, ons);
        ref ChFiDS_CommonPoint Popp = ref CD.ChangeVertex(isFirst, opp);

        // OCCT points at a copy of the domain's point (a local that leaves its scope)
        HatchGen_PointOnHatching? pPH = null;
        if (isFirst && Dom.HasFirstPoint())
        {
            HatchGen_PointOnHatching PHtemp = new(Dom.FirstPoint());
            pPH = PHtemp;
        }
        else if (!isFirst && Dom.HasSecondPoint())
        {
            HatchGen_PointOnHatching PHtemp = new(Dom.SecondPoint());
            pPH = PHtemp;
        }
        double x = 0.0, y = 0.0;
        Surface? Surf = DStr.Surface(CD.Surf()).Surface();
        if (pPH == null)
        {
            CD.ChangeInterference(ons).SetParameter(ponH, isFirst);
            Curve2D? pcons = CD.Interference(ons).PCurveOnSurf();
            pcons.Value(ponH).Coord(out x, out y);
            CD.ChangeVertex(isFirst, ons).SetPoint(Surf.Value(x, y));
        }
        else
        {
            // Modification to find already existing vertexes
            int LeType = 1;
            int NbInt = pPH.NbPoints();
            if (NbInt > 1)
            {
                bool trouve = true;
                int IE1 = 0;
                Vertex V1 = new(), V2 = new();
                bool suite = true;
                for (; trouve;)
                {
                    HatchGen_PointOnElement PEtemp = pPH.Point(LeType);
                    IE1 = PEtemp.Index();
                    EdgeCurve2DAdaptor? HE1 = (M[IE1] as EdgeCurve2DAdaptor);
                    if (HE1 is not null)
                    {
                        Edge Etemp = HE1.Edge();
                        Exploration.Vertices(Etemp, out V1, out V2);
                    }
                    else
                    {
                        suite = false;
                    }
                    if (((V1.IsSame(bout)) || (V2.IsSame(bout))) && suite)
                    {
                        trouve = true;
                        break;
                    }
                    else
                    {
                        suite = true;
                        trouve = false;
                        LeType++;
                        if (LeType > NbInt)
                        {
                            trouve = true;
                            LeType = 1;
                        }
                    }
                }
            }
            HatchGen_PointOnElement PE = pPH.Point(LeType);
            int IE = PE.Index();
            EdgeCurve2DAdaptor? HE = (M[IE] as EdgeCurve2DAdaptor);
            if (HE is null)
            {
                return;
            }
            Edge E = HE.Edge();

            if (PE.Position() != Internal)
            {
                Orientation O = CD.Interference(ons).Transition();
                if (isFirst)
                {
                    O = global::Kerf.Engine.Topology.TopAbs.Reverse(O);
                }
                CompCommonPoint(ref Pons, E, PE, O);
            }
            else
            {
                Pons.SetArc(pitol, E, PE.Parameter(), CompTra(CD.Interference(ons).Transition(), E.Orientation(), isFirst));
            }
            Curve2D? pcadj = CD.Interference(ons).PCurveOnSurf();
            pcadj.Value(ponH).Coord(out x, out y);
            CD.ChangeInterference(ons).SetParameter(ponH, isFirst);
            CD.ChangeVertex(isFirst, ons).SetPoint(Surf.Value(x, y));
        }
        if (!Popp.IsOnArc())
        {
            CD.ChangeInterference(opp).SetParameter(ponH, isFirst);
            Curve2D? pcopp = CD.Interference(opp).PCurveOnSurf();
            pcopp.Value(ponH).Coord(out x, out y);
            CD.ChangeVertex(isFirst, opp).SetPoint(Surf.Value(x, y));
        }
    }


}
