// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_SpKP.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Classification.Hatching;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.Builder_SpKP;
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
public partial class FilletBuilderBase
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

    internal void Trunc(SurfData? sd, Spine? spine, SurfaceAdaptorBase? s1, SurfaceAdaptorBase? s2, int iedge, bool isfirst, int cntlFiOnS)
    {
        DsDataStructure dStr = _ds.ChangeDS();
        // Return points and tangents on edge and spine.
        double wtg = sd.InterferenceOnS1().Parameter(isfirst);
        bool bid = false;
        double wsp = ParamOnSpine(dStr, wtg, sd, spine, iedge, false, false, _tolesp, ref bid);
        Point3D ped = new(), psp = new();
        Vector3D ded = new(), dsp = new();
        Vertex bout1 = new(), bout2 = new(), boutemp = new();

        EdgeCurveAdaptor bc = spine.CurrentElementarySpine(iedge);
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
        if (spine.Edges(iedge).Orientation() == Forward)
        {
            bc.D1(wtg + edf, out ped, out ded);
        }
        else
        {
            bc.D1(-wtg + edl, out ped, out ded);
            ded.Reverse();
        }
        spine.D1(wsp, out psp, out dsp);
        Point3D p1 = new(), p2 = new();
        Surface? surf = dStr.Surface(sd.Surf()).Surface();
        Point2D pp1 = new(), pp2 = new();
        pp1 = sd.InterferenceOnS1().PCurveOnSurf().Value(wtg);
        pp2 = sd.InterferenceOnS2().PCurveOnSurf().Value(wtg);
        p1 = surf.Value(pp1.X(), pp1.Y());
        p2 = surf.Value(pp2.X(), pp2.Y());
        bool tron = false;
        double ang = dsp.Angle(ded);
        double dis1 = psp.Distance(ped);
        double dis2 = p1.Distance(p2);
        if (ang > M_PI / 18.0)
        {
            tron = true;
        }
        if (dis1 >= 0.1 * dis2)
        {
            tron = true;
        }
        int ivois = 0;
        if (!tron && YaUnVoisin(spine, iedge, ref ivois, isfirst))
        {
            FaceSurfaceAdaptor? bs1 = (s1 as FaceSurfaceAdaptor);
            FaceSurfaceAdaptor? bs2 = (s2 as FaceSurfaceAdaptor);
            if (bs1 is not null && bs2 is not null)
            {
                Face fbid = new();
                Face f1 = bs1.Face();
                Face f2 = bs2.Face();
                CommonPoint cp1 = sd.Vertex(isfirst, 1);
                CommonPoint cp2 = sd.Vertex(isfirst, 2);
                if (!((cp1.IsOnArc && SearchFace(spine, cp1, f1, ref fbid))
                            || (cp2.IsOnArc && SearchFace(spine, cp2, f2, ref fbid))))
                {
                    tron = ChFi3d_KParticular(spine, ivois, bs1, bs2);
                }
            }
        }
        // modification of lvt against isolated vertex
        if (!tron && YaUnVoisin(spine, iedge, ref ivois, isfirst))
        {
            NCollection_List<Shape>.Iterator it = new();
            int nbed = -2;
            for (it.Initialize(_veMap[bout1]); it.More(); it.Next())
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
                double fiPar = sd.Interference(cntlFiOnS).Parameter(!isfirst);
                bool isTheCase = isfirst ? (par > fiPar) : (par < fiPar);
                if (isTheCase)
                {
                    dPar = par - fiPar;
                    par = fiPar;
                }
            }
            for (int i = 1; i <= 2; i++)
            {
                sd.ChangeInterference(i).SetParameter(par, isfirst);
                Curve2D? pc = sd.Interference(i).PCurveOnSurf();
                pc.Value(par).Coord(out x, out y);
                sd.ChangeVertex(isfirst, i).Reset();
                sd.ChangeVertex(isfirst, i).SetPoint(surf.Value(x, y));
                if (isfirst)
                {
                    sd.FirstSpineParam(spine.FirstParameter(iedge) - dPar);
                }
                else
                {
                    sd.LastSpineParam(spine.LastParameter(iedge) - dPar);
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

    public bool SplitKPart(SurfData? data, NCollection_Sequence<SurfData> setData, Spine? spine, int iedge, SurfaceAdaptorBase? s1, TopolTool? I1, SurfaceAdaptorBase? s2, TopolTool? I2, ref bool intf, ref bool intl)
    {
        // The hatching of each faces is started by tangency lines.

        double pitol = global::Kerf.Engine.Foundation.Precision.PIntersection();

        NCollection_DataMap<int, Curve2DAdaptorBase> m1 = new(), m2 = new();
        int iH1 = 0, iH2 = 0;
        int nb1 = 1, nb2 = 1;

        // Cutting of tangency lines (hatching).
        Intersector2D inter = new(pitol, pitol);
        Hatcher2D h1 = new(inter, _tol2d, _tolapp3d), h2 = new(inter, _tol2d, _tolapp3d);
        int ie = 0;
        Curve2D? c1 = data.InterferenceOnS1().PCurveOnFace();
        CurveAdaptor2D ll1 = new();
        if (c1 is not null)
        {
            ll1.Load(c1);
            for (I1.Init(); I1.More(); I1.Next())
            {
                EdgeCurve2DAdaptor? Bc = (I1.Value() as EdgeCurve2DAdaptor);
                CurveAdaptor2D? gc = (I1.Value() as CurveAdaptor2D);
                if (Bc is null)
                {
                    ie = h1.AddElement(gc, Forward);
                }
                else
                {
                    ie = h1.AddElement(Bc, Bc.Edge().Orientation());
                }
                m1.Bind(ie, I1.Value());
            }
            iH1 = h1.Trim(ll1);
            h1.ComputeDomains(iH1);
            if (!h1.IsDone(iH1))
            {
                return false;
            }
            nb1 = h1.NbDomains(iH1);
            if (nb1 == 0)
            {
                return false;
            }
        }

        Curve2D? c2 = data.InterferenceOnS2().PCurveOnFace();
        CurveAdaptor2D ll2 = new();
        if (c2 is not null)
        {
            ll2.Load(c2);
            for (I2.Init(); I2.More(); I2.Next())
            {
                EdgeCurve2DAdaptor? Bc = (I2.Value() as EdgeCurve2DAdaptor);
                CurveAdaptor2D? gc = (I2.Value() as CurveAdaptor2D);
                if (Bc is null)
                {
                    ie = h2.AddElement(gc, Forward);
                }
                else
                {
                    ie = h2.AddElement(Bc, Bc.Edge().Orientation());
                }
                m2.Bind(ie, I2.Value());
            }
            iH2 = h2.Trim(ll2);
            h2.ComputeDomains(iH2);
            if (!h2.IsDone(iH2))
            {
                return false;
            }
            nb2 = h2.NbDomains(iH2);
            if (nb2 == 0)
            {
                return false;
            }
        }

        // Return start and end vertexes of the Spine
        Vertex bout1 = new(), bout2 = new(), boutemp = new();
        EdgeCurveAdaptor bc = spine.CurrentElementarySpine(iedge);
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
        FaceSurfaceAdaptor? bhs = (s1 as FaceSurfaceAdaptor);
        if (bhs is not null)
        {
            F1 = bhs.Face();
        }
        bhs = (s2 as FaceSurfaceAdaptor);
        if (bhs is not null)
        {
            F2 = bhs.Face();
        }
        Face fbid = new();

        // Restriction of SurfDatas by cut lines.
        DsDataStructure dStr = _ds.ChangeDS();
        SurfData? cd = data;
        cd.ChangeIndexOfS1(dStr.AddShape(F1));
        cd.ChangeIndexOfS2(dStr.AddShape(F2));

        double f1 = 0.0, l1 = 0.0, f2 = 0.0, l2 = 0.0;
        NCollection_Array1<int> ind1 = new(1, nb1), ind2 = new(1, nb2);
        double wref = 0.0;

        int onS = 1;// switcher of access to surfs of SurfData, eap occ293
        int cntlFiOnS = 0;// FaceInterference to control length in OnSame
                                              // situation, eap occ354

        if (c1 is null && c2 is null)
        {
            return false;
        }
        else if (c1 is null || (nb1 == 1 && !h1.Domain(iH1, 1).HasFirstPoint))
        {
            // It is checked if the point 2d of the degenerated edge is in the face.
            if (c1 is null)
            {
                Point2D p2d1 = cd.Get2dPoints(false, 1);
                Containment situ = I1.Classify(p2d1, 1.0e-8, false);
                if (situ == Out)
                {
                    return false;
                }
            }

            // Parsing of domains by increasing parameters,
            if (!Tri(h2, iH2, ind2, wref, 0.0, pitol, ref nb2))
            {
                return false;
            }
            // Filling of SurfData
            for (int i = 1; i <= nb2; i++)
            {
                HatchingDomain dom2 = h2.Domain(iH2, ind2[i]);
                FillSD(dStr, cd, m2, dom2, dom2.FirstPoint().Parameter(), true, 2, pitol, bout1);
                FillSD(dStr, cd, m2, dom2, dom2.SecondPoint().Parameter(), false, 2, pitol, bout2);
                setData.Append(cd);
                cd = CpSD(dStr, cd);
            }
            if (intf)
            {
                ref SurfData? sd = ref setData.ChangeValue(1);
                ref CommonPoint cp2 = ref sd.ChangeVertexFirstOnS2();
                if (cp2.IsOnArc && spine.FirstStatus() == State.OnSame)
                {
                    intf = !SearchFace(spine, cp2, F2, ref fbid);
                }
                else
                {
                    intf = false;
                }
            }
            if (intl)
            {
                ref SurfData? sd = ref setData.ChangeValue(setData.Length());
                ref CommonPoint cp2 = ref sd.ChangeVertexLastOnS2();
                if (cp2.IsOnArc && spine.LastStatus() == State.OnSame)
                {
                    intl = !SearchFace(spine, cp2, F2, ref fbid);
                }
                else
                {
                    intl = false;
                }
            }
        }
        else if (c2 is null || (nb2 == 1 && !h2.Domain(iH2, 1).HasFirstPoint))
        {
            // It is checked if the point 2d of the degenerated is in the face.
            if (c2 is null)
            {
                Point2D p2d2 = cd.Get2dPoints(false, 2);
                Containment situ = I2.Classify(p2d2, 1.0e-8, false);
                if (situ == Out)
                {
                    return false;
                }
            }

            // Parsing of domains by increasing parameters,
            if (!Tri(h1, iH1, ind1, wref, 0.0, pitol, ref nb1))
            {
                return false;
            }
            // Filling of SurfData
            for (int i = 1; i <= nb1; i++)
            {
                HatchingDomain dom1 = h1.Domain(iH1, ind1[i]);
                FillSD(dStr, cd, m1, dom1, dom1.FirstPoint().Parameter(), true, 1, pitol, bout1);
                FillSD(dStr, cd, m1, dom1, dom1.SecondPoint().Parameter(), false, 1, pitol, bout2);
                setData.Append(cd);
                cd = CpSD(dStr, cd);
            }
            if (intf)
            {
                ref SurfData? sd = ref setData.ChangeValue(1);
                ref CommonPoint cp1 = ref sd.ChangeVertexFirstOnS1();
                if (cp1.IsOnArc && spine.FirstStatus() == State.OnSame)
                {
                    intf = !SearchFace(spine, cp1, F1, ref fbid);
                }
                else
                {
                    intf = false;
                }
            }
            if (intl)
            {
                ref SurfData? sd = ref setData.ChangeValue(setData.Length());
                ref CommonPoint cp1 = ref sd.ChangeVertexLastOnS1();
                if (cp1.IsOnArc && spine.LastStatus() == State.OnSame)
                {
                    intl = !SearchFace(spine, cp1, F1, ref fbid);
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
                if (!Tri(h2, iH2, ind2, wref, 0.0, pitol, ref nb2))
                {
                    return false;
                }
                period1 = ll1.Period();
                if (!Tri(h1, iH1, ind1, wref, period1, pitol, ref nb1))
                {
                    return false;
                }
            }
            else
            {
                if (!Tri(h1, iH1, ind1, wref, 0.0, pitol, ref nb1))
                {
                    return false;
                }
                if (ll2.IsPeriodic)
                {
                    period2 = ll2.Period();
                }
                if (!Tri(h2, iH2, ind2, wref, period2, pitol, ref nb2))
                {
                    return false;
                }
            }

            // Filling of SurfData
            NCollection_Sequence<int> ion1 = new(), ion2 = new();
            for (int i = 1; i <= nb1; i++)
            {
                HatchingDomain dom1 = h1.Domain(iH1, ind1[i]);
                int nbcoup1 = 1;
                bool acheval1 = AdjustParam(dom1, ref f1, ref l1, wref, period1, pitol);
                if (acheval1)
                {
                    nbcoup1 = 2;
                }
                for (int icoup1 = 1; icoup1 <= nbcoup1; icoup1++)
                {
                    for (int j = 1; j <= nb2; j++)
                    {
                        HatchingDomain dom2 = h2.Domain(iH2, j);
                        int nbcoup2 = 1;
                        bool acheval2 = AdjustParam(dom2, ref f2, ref l2, wref, period2, pitol);
                        if (acheval2)
                        {
                            nbcoup2 = 2;
                        }
                        for (int icoup2 = 1; icoup2 <= nbcoup2; icoup2++)
                        {
                            if (f2 <= l1 && f1 <= l2)
                            {
                                if (f1 >= f2 - _tol2d)
                                {
                                    FillSD(dStr, cd, m1, dom1, f1, true, 1, pitol, bout1);
                                }
                                if (f2 >= f1 - _tol2d)
                                {
                                    FillSD(dStr, cd, m2, dom2, f2, true, 2, pitol, bout1);
                                }
                                if (l1 >= l2 - _tol2d)
                                {
                                    FillSD(dStr, cd, m2, dom2, l2, false, 2, pitol, bout2);
                                }
                                if (l2 >= l1 - _tol2d)
                                {
                                    FillSD(dStr, cd, m1, dom1, l1, false, 1, pitol, bout2);
                                }
                                setData.Append(cd);
                                cd = CpSD(dStr, cd);
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
                EdgeCurveAdaptor ed = spine.CurrentElementarySpine(iedge);
                for (int i1 = 1; i1 <= setData.Length(); i1++)
                {
                    ref SurfData? cd1 = ref setData.ChangeValue(i1);
                    ref CommonPoint cp1l = ref cd1.ChangeVertexFirstOnS1();
                    ref CommonPoint cp2l = ref cd1.ChangeVertexFirstOnS2();
                    if (cp1l.IsOnArc && !SearchFace(spine, cp1l, F1, ref fbid))
                    {
                        ptg = cd1.InterferenceOnS1().FirstParameter();
                        dsp = ComputeAbscissa(ed, ptg);
                        if (Math.Abs(dsp) < dist)
                        {
                            ifirst = i1;
                            dist = Math.Abs(dsp);
                        }
                    }
                    else if (cp2l.IsOnArc && !SearchFace(spine, cp2l, F2, ref fbid))
                    {
                        ptg = cd1.InterferenceOnS2().FirstParameter();
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
                    setData.Remove(1, ifirst - 1);
                    ion1.Remove(1, ifirst - 1);
                    ion2.Remove(1, ifirst - 1);
                }
                if (setData.IsEmpty())
                {
                    return false;
                }
                ref SurfData? cd2 = ref setData.ChangeValue(1);
                ref CommonPoint cp1 = ref cd2.ChangeVertexFirstOnS1();
                ref CommonPoint cp2 = ref cd2.ChangeVertexFirstOnS2();
                CommonPoint sov = new();

                if (cp1.IsOnArc && cp2.IsOnArc)
                {
                    intf = !SearchFace(spine, cp1, F1, ref fbid) && !SearchFace(spine, cp2, F2, ref fbid);
                }
                else if (cp1.IsOnArc)
                {
                    sov = cp2;
                    if (!SearchFace(spine, cp1, F1, ref fbid))
                    {
                        FillSD(dStr, cd2, m2, h2.Domain(iH2, ind2[ion2.First()]),
                                      h2.Domain(iH2, ind2[ion2.First()]).FirstPoint().Parameter(), true, 2, pitol, bout1);
                        if (!cp2.IsOnArc || (cp2.IsOnArc && SearchFace(spine, cp2, F2, ref fbid)))
                        {
                            cp2 = sov;
                            if (spine.FirstStatus() != State.OnSame)
                            {
                                cd2.ChangeInterference(2).SetParameter(cd2.Interference(1).Parameter(true), true);
                                intf = false;
                            }
                        }
                    }
                    else
                    {
                        intf = false;
                    }
                }
                else if (cp2.IsOnArc)
                {
                    sov = cp1;
                    if (!SearchFace(spine, cp2, F2, ref fbid))
                    {
                        FillSD(dStr, cd2, m1, h1.Domain(iH1, ind1[ion1.First()]),
                                      h1.Domain(iH1, ind1[ion1.First()]).FirstPoint().Parameter(), true, 1, pitol, bout1);
                        if (!cp1.IsOnArc || (cp1.IsOnArc && SearchFace(spine, cp1, F1, ref fbid)))
                        {
                            cp1 = sov;
                            if (spine.FirstStatus() != State.OnSame)
                            {
                                cd2.ChangeInterference(1).SetParameter(cd2.Interference(2).Parameter(true), true);
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
                if (intf && spine.FirstStatus() == State.OnSame)
                {
                    Edge[] threeE = new Edge[3];
                    ChFi3d_cherche_element(bout1, support, F1, ref threeE[0], ref boutemp);
                    ChFi3d_cherche_element(bout1, support, F2, ref threeE[1], ref boutemp);
                    threeE[2] = support;
                    if (ChFi3d_EdgeState(threeE, _efMap) == State.OnSame)
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
                double f = spine.FirstParameter(iedge);
                double l = spine.LastParameter(iedge);
                EdgeCurveAdaptor ed = spine.CurrentElementarySpine(iedge);
                for (int i2 = 1; i2 <= setData.Length(); i2++)
                {
                    ref SurfData? cd3 = ref setData.ChangeValue(i2);
                    ref CommonPoint cp1l = ref cd3.ChangeVertexLastOnS1();
                    ref CommonPoint cp2l = ref cd3.ChangeVertexLastOnS2();
                    if (cp1l.IsOnArc && !SearchFace(spine, cp1l, F1, ref fbid))
                    {
                        ptg = cd3.InterferenceOnS1().LastParameter();
                        dsp = -ComputeAbscissa(ed, ptg) - f + l;
                        if (Math.Abs(dsp) < dist)
                        {
                            ilast = i2;
                            dist = Math.Abs(dsp);
                        }
                    }
                    else if (cp2l.IsOnArc && !SearchFace(spine, cp2l, F2, ref fbid))
                    {
                        ptg = cd3.InterferenceOnS2().LastParameter();
                        dsp = -ComputeAbscissa(ed, ptg) - f + l;
                        if (Math.Abs(dsp) < dist)
                        {
                            ilast = i2;
                            dist = Math.Abs(dsp);
                        }
                    }
                }
                int lll = setData.Length();
                if (ilast < lll)
                {
                    setData.Remove(ilast + 1, lll);
                    ion1.Remove(ilast + 1, lll);
                    ion2.Remove(ilast + 1, lll);
                }
                if (setData.IsEmpty())
                {
                    return false;
                }
                ref SurfData? cd4 = ref setData.ChangeValue(setData.Length());
                ref CommonPoint cp1 = ref cd4.ChangeVertexLastOnS1();
                ref CommonPoint cp2 = ref cd4.ChangeVertexLastOnS2();
                CommonPoint sov = new();
                if (cp1.IsOnArc && cp2.IsOnArc)
                {
                    intl = !SearchFace(spine, cp1, F1, ref fbid) && !SearchFace(spine, cp2, F2, ref fbid);
                }
                else if (cp1.IsOnArc)
                {
                    sov = cp2;
                    if (!SearchFace(spine, cp1, F1, ref fbid))
                    {
                        FillSD(dStr, cd4, m2, h2.Domain(iH2, ind2[ion2.Last()]),
                                      h2.Domain(iH2, ind2[ion2.Last()]).SecondPoint().Parameter(), false, 2, pitol, bout2);
                        if (!cp2.IsOnArc || (cp2.IsOnArc && SearchFace(spine, cp2, F2, ref fbid)))
                        {
                            cp2 = sov;
                            if (spine.LastStatus() != State.OnSame)
                            {
                                cd4.ChangeInterference(2).SetParameter(cd4.Interference(1).Parameter(false), false);
                                intl = false;
                            }
                        }
                    }
                    else
                    {
                        intl = false;
                    }
                }
                else if (cp2.IsOnArc)
                {
                    sov = cp1;
                    if (!SearchFace(spine, cp2, F2, ref fbid))
                    {
                        FillSD(dStr, cd4, m1, h1.Domain(iH1, ind1[ion1.Last()]),
                                      h1.Domain(iH1, ind1[ion1.Last()]).SecondPoint().Parameter(), false, 1, pitol, bout2);
                        if (!cp1.IsOnArc || (cp1.IsOnArc && SearchFace(spine, cp1, F1, ref fbid)))
                        {
                            cp1 = sov;
                            if (spine.LastStatus() != State.OnSame)
                            {
                                cd4.ChangeInterference(1).SetParameter(cd4.Interference(2).Parameter(false), false);
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
                if (intl && spine.LastStatus() == State.OnSame)
                {
                    Edge[] threeE = new Edge[3];
                    ChFi3d_cherche_element(bout2, support, F1, ref threeE[0], ref boutemp);
                    ChFi3d_cherche_element(bout2, support, F2, ref threeE[1], ref boutemp);
                    threeE[2] = support;
                    if (ChFi3d_EdgeState(threeE, _efMap) == State.OnSame)
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
            bool okdoc = setData.IsEmpty();
            int i = 1;
            while (!okdoc)
            {
                ref SurfData? cd5 = ref setData.ChangeValue(i);
                double ltg = cd5.Interference(onS).LastParameter();
                double nl = ComputeAbscissa(spine.CurrentElementarySpine(iedge), ltg);
                if (nl < -_tolesp)
                {
                    setData.Remove(i);
                }
                else
                {
                    i++;
                }
                okdoc = (setData.IsEmpty() || i > setData.Length());
            }
        }
        if (!intl)
        {
            // SurfData are entirely suspended after the end of the edge.
            bool okdoc = setData.IsEmpty();
            int i = 1;
            while (!okdoc)
            {
                ref SurfData? cd6 = ref setData.ChangeValue(i);
                double ftg = cd6.Interference(onS).FirstParameter();
                double f = spine.FirstParameter(iedge);
                double l = spine.LastParameter(iedge);
                double nl = ComputeAbscissa(spine.CurrentElementarySpine(iedge), ftg);
                if (nl > (l - f + _tolesp))
                {
                    setData.Remove(i);
                }
                else
                {
                    i++;
                }
                okdoc = (setData.IsEmpty() || i > setData.Length());
            }
        }
        // Add parameters of the spine on SurfDatas.
        //  for (int i = 1; i <= SetData.Length(); i++) {
        int iSD = 0;
        for (iSD = 1; iSD <= setData.Length(); iSD++)
        {
            bool pokdeb = false, pokfin = false;
            ref SurfData? cd7 = ref setData.ChangeValue(iSD);
            double ftg = cd7.Interference(onS).FirstParameter();
            double ltg = cd7.Interference(onS).LastParameter();
            double fsp = ParamOnSpine(dStr, ftg, cd7, spine, iedge, intf, intl, _tolesp, ref pokdeb);
            if (!pokdeb)
            {
                fsp = ResetProl(dStr, cd7, spine, iedge, true);
            }
            double lsp = ParamOnSpine(dStr, ltg, cd7, spine, iedge, intf, intl, _tolesp, ref pokfin);
            if (!pokfin)
            {
                lsp = ResetProl(dStr, cd7, spine, iedge, false);
            }
            if (spine.IsPeriodic && iedge == spine.EdgeCount && lsp < fsp)
            {
                lsp += spine.Period();
            }
            else if (spine.IsPeriodic && iedge == 1 && lsp < fsp)
            {
                fsp -= spine.Period();
            }
            cd7.FirstSpineParam(fsp);
            cd7.LastSpineParam(lsp);
        }

        if (intf && !setData.IsEmpty())
        {
            // extension of the spine
            spine.SetFirstParameter(setData.First().FirstSpineParam());
        }
        else
        {
            // Trnncation at the beginning.
            for (iSD = 1; iSD <= setData.Length(); iSD++)
            {
                ref SurfData? cd8 = ref setData.ChangeValue(iSD);
                double fsp = cd8.FirstSpineParam();
                double lsp = cd8.LastSpineParam();
                if (lsp > spine.FirstParameter(iedge))
                {
                    if (fsp > spine.FirstParameter(iedge))
                    {
                        break;
                    }
                    else
                    {
                        Trunc(cd8, spine, s1, s2, iedge, true, cntlFiOnS);
                        break;
                    }
                }
            }
            if (iSD > 1)
            {
                setData.Remove(1, iSD - 1);
            }
        }

        if (intl && !setData.IsEmpty())
        {
            // extension of the spine
            spine.SetLastParameter(setData.Last().LastSpineParam());
        }
        else
        {
            // Truncation at the end.
            for (iSD = setData.Length(); iSD >= 1; iSD--)
            {
                ref SurfData? cd9 = ref setData.ChangeValue(iSD);
                double fsp = cd9.FirstSpineParam();
                double lsp = cd9.LastSpineParam();
                if (fsp < spine.LastParameter(iedge))
                {
                    if (lsp < spine.LastParameter(iedge))
                    {
                        break;
                    }
                    else
                    {
                        Trunc(cd9, spine, s1, s2, iedge, false, cntlFiOnS);
                        break;
                    }
                }
            }
            if (iSD < setData.Length())
            {
                setData.Remove(iSD + 1, setData.Length());
            }
        }
        return true;
    }

}

internal static class Builder_SpKP
{
    internal static Orientation CompTra(Orientation o1, Orientation o2, bool isfirst)
    {
        if (isfirst)
        {
            return global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(o1, o2));
        }
        else
        {
            return global::Kerf.Engine.Topology.TopAbs.Compose(o1, o2);
        }
    }

    internal static void CompCommonPoint(ref CommonPoint filPoint, in Edge arc, PointOnElement pe, Orientation or)
    {
        Orientation pos = pe.Position();
        Vertex v = new();
        if (pos == Forward)
        {
            v = Exploration.FirstVertex(arc);
        }
        else
        {
            v = Exploration.LastVertex(arc);
        }
        filPoint.SetVertex(v);
        filPoint.SetArc(global::Kerf.Engine.Foundation.Precision.PIntersection(), arc, pe.Parameter(), global::Kerf.Engine.Topology.TopAbs.Compose(arc.Orientation(), or));
    }

    internal static FaceInterference CpInterf(DsDataStructure dStr, in FaceInterference fi)
    {
        FaceInterference newF = fi;
        DsCurve toc = dStr.Curve(fi.LineIndex());
        Curve3D? newC = null;
        if (toc.Curve() is not null)
        {
            newC = (toc.Curve().Copy() as Curve3D);
        }
        newF.SetLineIndex(dStr.AddCurve(new DsCurve(newC, toc.Tolerance())));

        if (fi.PCurveOnFace() is not null)
        {
            newF.ChangePCurveOnFace() = (fi.PCurveOnFace().Copy() as Curve2D);
        }
        if (fi.PCurveOnSurf() is not null)
        {
            newF.ChangePCurveOnSurf() = (fi.PCurveOnSurf().Copy() as Curve2D);
        }
        return newF;
    }

    internal static SurfData? CpSD(DsDataStructure dStr, SurfData? data)
    {
        SurfData? newData = new SurfData();
        DsSurface tos = dStr.Surface(data.Surf());
        Surface? newS = (tos.Surface().Copy() as Surface);
        double tol = tos.Tolerance();
        newData.ChangeSurf(dStr.AddSurface(new DsSurface(newS, tol)));
        newData.ChangeIndexOfS1(data.IndexOfS1());
        newData.ChangeIndexOfS2(data.IndexOfS2());
        newData.ChangeOrientation() = data.Orientation();
        newData.ChangeInterferenceOnS1() = CpInterf(dStr, data.InterferenceOnS1());
        newData.ChangeInterferenceOnS2() = CpInterf(dStr, data.InterferenceOnS2());
        return newData;
    }

    internal static bool AdjustParam(in HatchingDomain dom, ref double f, ref double l, double wref, double period, double pitol)
    {
        if (dom.HasFirstPoint)
        {
            f = dom.FirstPoint().Parameter();
        }
        else
        {
            f = 0.0;
        }
        if (dom.HasSecondPoint)
        {
            l = dom.SecondPoint().Parameter();
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

    internal static double ComputeAbscissa(EdgeCurveAdaptor c, double u)
    {
        switch (c.GetType())
        {
            case Line:
                return u;
            case Circle:
                return c.Circle().Radius() * u;
            default:
                return 0;
        }
    }

    internal static double ParamOnSpine(DsDataStructure dStr, double ptg, SurfData? cd, Spine? spine, int iedge, bool intf, bool intl, double tol, ref bool pok)
    {
        double nl = 0.0;
        double f = spine.FirstParameter(iedge);
        double l = spine.LastParameter(iedge);

        nl = ComputeAbscissa(spine.CurrentElementarySpine(iedge), ptg) + f;
        if ((nl >= (f - tol) || intf) && (nl <= (l + tol) || intl))
        {
            pok = true;
            return nl;
        }
        else
        {
            // construction of the plane containing the section of CD with parameter ptg.
            Point3D pp = new();
            Vector3D vv = new();
            Curve3D? c3d = null;
            if (cd.InterferenceOnS1().LineIndex() != 0)
            {
                c3d = dStr.Curve(cd.InterferenceOnS1().LineIndex()).Curve();
            }
            if (c3d is null)
            {
                c3d = dStr.Curve(cd.InterferenceOnS2().LineIndex()).Curve();
            }
            c3d.D1(ptg, out pp, out vv);

            Plane3D nlp = new(pp, new Direction3D(vv));
            PlaneSurface? pln = new PlaneSurface(nlp);
            SurfaceAdaptor? plan = new SurfaceAdaptor(pln);

            // intersection plane spine.
            bool found = false;
            bool fini = false;
            int sens = 1;
            if (nl <= f)
            {
                sens = -1;
            }
            int ii = iedge + sens;
            if (spine.IsPeriodic)
            {
                if (ii <= 0)
                {
                    ii += spine.EdgeCount;
                }
                if (ii > spine.EdgeCount)
                {
                    ii -= spine.EdgeCount;
                }
            }
            else if (ii < 1 || ii > spine.EdgeCount)
            {
                pok = true;
                return nl;
            }
            EdgeCurveAdaptor? he = new EdgeCurveAdaptor();
            EdgeCurveAdaptor ce = he; // a reference to *HE in OCCT

            while (!found && !fini)
            {
                Orientation o = spine.Edges(ii).Orientation();
                bool first = ((o == Forward && sens == 1) || (o == Reversed && sens == -1));
                ce.Initialize(spine.Edges(ii));
                double tolc = ce.Resolution(tol);
                found = ChFi3d_InterPlaneEdge(plan, he, ref nl, first, tolc);
                Point3D point = ce.Value(nl);
                if (found)
                {
                    nl = spine.Absc(nl, ii);
                }
                point = spine.Value(nl);

                ii += sens;
                if (spine.IsPeriodic)
                {
                    if (ii <= 0)
                    {
                        ii += spine.EdgeCount;
                    }
                    if (ii > spine.EdgeCount)
                    {
                        ii -= spine.EdgeCount;
                    }
                    fini = (ii == iedge);
                }
                else
                {
                    fini = (ii < 1 || ii > spine.EdgeCount);
                }
            }
            pok = found;
            return nl;
        }
    }

    internal static bool YaUnVoisin(Spine? spine, int iedge, ref int ivois, bool isfirst)
    {
        int nbed = spine.EdgeCount;
        if (nbed == 1)
        {
            return false;
        }
        bool periodic = spine.IsPeriodic;
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

    internal static double ResetProl(DsDataStructure dStr, SurfData? cd, Spine? spine, int iedge, bool isfirst)
    {
        EdgeCurveAdaptor bc = spine.CurrentElementarySpine(iedge);
        double edglen = bc.LastParameter() - bc.FirstParameter();
        Surface? surf = dStr.Surface(cd.Surf()).Surface();
        double par = 0.0, x, y;
        if (!isfirst)
        {
            par = edglen;
        }
        double sppar = 0.0;
        for (int i = 1; i <= 2; i++)
        {
            cd.ChangeInterference(i).SetParameter(par, isfirst);
            Curve2D? pc = cd.Interference(i).PCurveOnSurf();
            pc.Value(par).Coord(out x, out y);
            cd.ChangeVertex(isfirst, i).Reset();
            cd.ChangeVertex(isfirst, i).SetPoint(surf.Value(x, y));
            if (isfirst)
            {
                sppar = spine.FirstParameter(iedge);
                cd.FirstSpineParam(sppar);
            }
            else
            {
                sppar = spine.LastParameter(iedge);
                cd.LastSpineParam(sppar);
            }
        }
        return sppar;
    }

    internal static bool Tri(Hatcher2D h, int iH, NCollection_Array1<int> ind, double wref, double period, double pitol, ref int nbdom)
    {
        //  for (int i = 1; i <= Nbdom; i++) { Ind[i] = i; }
        int i = 0;
        for (i = 1; i <= nbdom; i++)
        {
            ind[i] = i;
        }
        double f1 = 0.0, f2 = 0.0, l = 0.0;
        int tmp = 0;
        bool invert = true;

        while (invert)
        {
            invert = false;
            for (i = 1; i < nbdom; i++)
            {
                AdjustParam(h.Domain(iH, ind[i]), ref f1, ref l, wref, period, pitol);
                AdjustParam(h.Domain(iH, ind[i + 1]), ref f2, ref l, wref, period, pitol);
                if (f2 < f1)
                {
                    tmp = ind[i];
                    ind[i] = ind[i + 1];
                    ind[i + 1] = tmp;
                    invert = true;
                }
            }
        }

        int iSansFirst = 0, iSansLast = 0;

        if (nbdom != 1)
        {
            for (i = 1; i <= nbdom; i++)
            {
                if (!h.Domain(iH, ind[i]).HasFirstPoint)
                {
                    iSansFirst = i;
                }
                if (!h.Domain(iH, ind[i]).HasSecondPoint)
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
            ref HatchingDomain dom = ref h.ChangeDomain(iH, ind[iSansFirst]);
            PointOnHatching ph = h.Domain(iH, ind[iSansLast]).FirstPoint();
            double newPar = h.HatchingCurve(iH).FirstParameter() - period
                            + h.Domain(iH, ind[iSansLast]).FirstPoint().Parameter();
            ph.SetParameter(newPar);
            dom.SetFirstPoint(ph);

            for (int k = iSansLast; k < nbdom; k++)
            {
                ind[k] = ind[k + 1];
            }
            nbdom--;
        }
        return true;
    }

    internal static void FillSD(DsDataStructure dStr, SurfData? cd, NCollection_DataMap<int, Curve2DAdaptorBase> m, in HatchingDomain dom, double ponH, bool isFirst, int ons, double pitol, in Vertex bout)
    {
        int opp = 3 - ons;
        ref CommonPoint pons = ref cd.ChangeVertex(isFirst, ons);
        ref CommonPoint popp = ref cd.ChangeVertex(isFirst, opp);

        // OCCT points at a copy of the domain's point (a local that leaves its scope)
        PointOnHatching? pPH = null;
        if (isFirst && dom.HasFirstPoint)
        {
            PointOnHatching pHtemp = new(dom.FirstPoint());
            pPH = pHtemp;
        }
        else if (!isFirst && dom.HasSecondPoint)
        {
            PointOnHatching pHtemp = new(dom.SecondPoint());
            pPH = pHtemp;
        }
        double x = 0.0, y = 0.0;
        Surface? surf = dStr.Surface(cd.Surf()).Surface();
        if (pPH == null)
        {
            cd.ChangeInterference(ons).SetParameter(ponH, isFirst);
            Curve2D? pcons = cd.Interference(ons).PCurveOnSurf();
            pcons.Value(ponH).Coord(out x, out y);
            cd.ChangeVertex(isFirst, ons).SetPoint(surf.Value(x, y));
        }
        else
        {
            // Modification to find already existing vertexes
            int leType = 1;
            int nbInt = pPH.PointCount;
            if (nbInt > 1)
            {
                bool trouve = true;
                int ie1 = 0;
                Vertex v1 = new(), v2 = new();
                bool suite = true;
                for (; trouve;)
                {
                    PointOnElement pEtemp = pPH.Point(leType);
                    ie1 = pEtemp.Index();
                    EdgeCurve2DAdaptor? he1 = (m[ie1] as EdgeCurve2DAdaptor);
                    if (he1 is not null)
                    {
                        Edge etemp = he1.Edge();
                        Exploration.Vertices(etemp, out v1, out v2);
                    }
                    else
                    {
                        suite = false;
                    }
                    if (((v1.IsSame(bout)) || (v2.IsSame(bout))) && suite)
                    {
                        trouve = true;
                        break;
                    }
                    else
                    {
                        suite = true;
                        trouve = false;
                        leType++;
                        if (leType > nbInt)
                        {
                            trouve = true;
                            leType = 1;
                        }
                    }
                }
            }
            PointOnElement pe = pPH.Point(leType);
            int ie = pe.Index();
            EdgeCurve2DAdaptor? he = (m[ie] as EdgeCurve2DAdaptor);
            if (he is null)
            {
                return;
            }
            Edge e = he.Edge();

            if (pe.Position() != Internal)
            {
                Orientation o = cd.Interference(ons).Transition();
                if (isFirst)
                {
                    o = global::Kerf.Engine.Topology.TopAbs.Reverse(o);
                }
                CompCommonPoint(ref pons, e, pe, o);
            }
            else
            {
                pons.SetArc(pitol, e, pe.Parameter(), CompTra(cd.Interference(ons).Transition(), e.Orientation(), isFirst));
            }
            Curve2D? pcadj = cd.Interference(ons).PCurveOnSurf();
            pcadj.Value(ponH).Coord(out x, out y);
            cd.ChangeInterference(ons).SetParameter(ponH, isFirst);
            cd.ChangeVertex(isFirst, ons).SetPoint(surf.Value(x, y));
        }
        if (!popp.IsOnArc)
        {
            cd.ChangeInterference(opp).SetParameter(ponH, isFirst);
            Curve2D? pcopp = cd.Interference(opp).PCurveOnSurf();
            pcopp.Value(ponH).Coord(out x, out y);
            cd.ChangeVertex(isFirst, opp).SetPoint(surf.Value(x, y));
        }
    }


}
