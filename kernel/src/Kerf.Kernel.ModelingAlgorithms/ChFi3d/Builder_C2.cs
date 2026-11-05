// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_C2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.Builder_C2;
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


    //=======================================================================
    // function : PerformTwoCornerbyInter
    // purpose  : Performs PerformTwoCorner by intersection.
    //           In case of Biseau for all cases the
    //           path is used; 3D curve and 2 pcurves are approximated.
    //=======================================================================

    public bool PerformTwoCornerbyInter(int index)
    {
        _done = false;
        Vertex vtx = _vDataMap.FindKey(index);
        DsDataStructure dStr = _ds.ChangeDS();

        // Information on fillets is extracted
        //------------------------------------------------------

        // the first
        //----------
        NCollection_List<Stripe>.Iterator it = new();
        it.Initialize(_vDataMap[index]);
        Stripe? corner1 = it.ChangeValue();
        int sens1 = 0;
        int iFd1 = ChFi3d_IndexOfSurfData(vtx, corner1, out sens1);
        NCollection_Sequence<SurfData> seqFil1 = 
            corner1.ChangeSetOfSurfData();
        SurfData? fd1 = seqFil1.ChangeValue(iFd1);

        // the second
        //----------
        it.Next();
        Stripe? corner2 = it.ChangeValue();
        int sens2 = 0;
        int iFd2 = 0;
        if (corner2 == corner1)
        {
            sens2 = -1;
            iFd2 = corner2.SetOfSurfData().Length();
        }
        else
        {
            iFd2 = ChFi3d_IndexOfSurfData(vtx, corner2, out sens2);
        }
        NCollection_Sequence<SurfData> seqFil2 = 
            corner2.ChangeSetOfSurfData();
        SurfData? fd2 = seqFil2.ChangeValue(iFd2);

        // The concavities are analysed in case of different concavities,
        // preview an evolutionary connection of type ThreeCorner of R to 0.
        // Otherwise the opposite face
        // and the eventual intersection of 2 pcurves on this face are found.

        bool isfirst1 = (sens1 == 1);
        bool isfirst2 = (sens2 == 1);
        bool okinterCC = false, okvisavis = false, sameSide = false;
        int iFaCo1 = 0, iFaCo2 = 0;
        double uIntPC1 = 0.0, uIntPC2 = 0.0;
        Face faCo = new();
        okinterCC = ChFi3d_IsInFront(dStr, corner1, corner2, iFd1, iFd2, sens1, sens2, ref uIntPC1, ref uIntPC2, ref faCo,
                                                                  ref sameSide, ref iFaCo1, ref iFaCo2, ref okvisavis, vtx, true);
        if (!okvisavis)
        {
            _done = false;
            return _done;
        }
        if (!okinterCC)
        {
            // The intersection of pcurves is calculated without restricting them by
            // common points.
            okinterCC = ChFi3d_IsInFront(dStr, corner1, corner2, iFd1, iFd2, sens1, sens2, ref uIntPC1, ref uIntPC2, ref faCo,
                                                                      ref sameSide, ref iFaCo1, ref iFaCo2, ref okvisavis, vtx, true, true);
        }

        if (!okvisavis)
        {
            _done = false;
            return _done;
        }
        if (!okinterCC)
        {
            _done = false;
            return _done;
        }
        int iFaArc1 = 3 - iFaCo1, iFaArc2 = 3 - iFaCo2;

        // It is checked if the fillets have a commonpoint on a common arc.
        // This edge is the pivot of the bevel or of the kneecap.

        ref CommonPoint cp1 = ref fd1.ChangeVertex(isfirst1, iFaArc1);
        ref CommonPoint cp2 = ref fd2.ChangeVertex(isfirst2, iFaArc2);

        if (!cp1.IsOnArc || !cp2.IsOnArc)
        {
            _done = false;
            return _done;
        }
        if (!cp1.Arc().IsSame(cp2.Arc()))
        {
            // look like OnSame + OnDiff case (eap, Arp 9 2002, occ266)
            _done = true;
            PerformMoreThreeCorner(index, 2);
            return _done;
        }

        Edge pivot = new();
        pivot = cp1.Arc();
        double parCP1 = cp1.ParameterOnArc();
        double parCP2 = cp2.ParameterOnArc();
        EdgeCurveAdaptor? hpivot = new EdgeCurveAdaptor(pivot);
        if (!pivot.IsSame(cp2.Arc()))
        {
            Curve3D? csau = null;
            double ubid = 0.0, vbid = 0.0;
            csau = Tool.Curve(pivot, out ubid, out vbid);
            BoundedCurve3D? c1 = (csau as BoundedCurve3D);
            if (c1 is not null)
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref c1, cp2.Point(), 1, false);
                CurveAdaptor cad = new();
                cad.Load(c1);
                CurvePointExtrema ext = new(cp2.Point(), cad, 1.0e-4);
                parCP2 = ext.Point(1).Parameter();
            }
        }
        Point3D psp1 = hpivot.Value(parCP1);
        Point3D psp2 = hpivot.Value(parCP2);
        double sameparam = (psp1.DistanceTo(psp2) < 10.0 * _tolapp3d) ? 1.0 : 0.0; // OCCT: a bool in a double

        Face ff1 = ShapeCasts.Face(dStr.Shape(fd1.Index(iFaArc1)));
        Face ff2 = ShapeCasts.Face(dStr.Shape(fd2.Index(iFaArc2)));
        NCollection_List<Shape>.Iterator kt = new();
        bool ok1 = false, ok2 = false;
        for (kt.Initialize(_efMap[pivot]); kt.More(); kt.Next())
        {
            Face f = ShapeCasts.Face(kt.Value());
            if (!ok1 && ff1.IsSame(f))
            {
                ok1 = true;
            }
            if (!ok2 && ff2.IsSame(f))
            {
                ok2 = true;
            }
        }
        if (!ok1 || !ok2)
        {
            _done = false;
            return _done;
        }

        SurfaceAdaptor? hs1 = null, hs2 = null;
        hs1 = ChFi3d_BoundSurf(dStr, fd1, iFaCo1, iFaArc1);
        hs2 = ChFi3d_BoundSurf(dStr, fd2, iFaCo2, iFaArc2);

        NCollection_Array1<double> pardeb = new(1, 4), parfin = new(1, 4);

        Curve2D? pGc1 = null, pGc2 = null;
        Curve3D? gc = null;

        if (sameparam != 0.0)
        {
            // Side common face, calculation of Pardeb.
            ChFi3d_ComputesIntPC(fd1.Interference(iFaCo1), fd2.Interference(iFaCo2), hs1, hs2, ref uIntPC1, ref uIntPC2);
            Point2D uv = new();
            uv = fd1.Interference(iFaCo1).PCurveOnSurf().Value(uIntPC1);
            pardeb[1] = uv.X;
            pardeb[2] = uv.Y;
            uv = fd2.Interference(iFaCo2).PCurveOnSurf().Value(uIntPC2);
            pardeb[3] = uv.X;
            pardeb[4] = uv.Y;
            Point3D pFaCo = hs1.Value(pardeb[1], pardeb[2]);

            // Side arc, calculation of Parfin.
            double uIntArc1 = fd1.Interference(iFaArc1).Parameter(isfirst1);
            double uIntArc2 = fd2.Interference(iFaArc2).Parameter(isfirst2);

            ChFi3d_ComputesIntPC(fd1.Interference(iFaArc1), fd2.Interference(iFaArc2), hs1, hs2, ref uIntArc1, ref uIntArc2);
            uv = fd1.Interference(iFaArc1).PCurveOnSurf().Value(uIntArc1);
            parfin[1] = uv.X;
            parfin[2] = uv.Y;
            uv = fd2.Interference(iFaArc2).PCurveOnSurf().Value(uIntArc2);
            parfin[3] = uv.X;
            parfin[4] = uv.Y;

            if (fd1.Surf() == fd2.Surf())
            {
                Reduce(uIntPC1, uIntPC2, hs1, hs2);
            }

            double tolreached = _tolapp3d;
            if (iFaCo1 == 1
                    && !ChFi3d_ComputeCurves(hs1, hs2, pardeb, parfin, ref gc, ref pGc1, ref pGc2, _tolapp3d, _tol2d,
                                                                      out tolreached))
            {
                _done = false;
                return _done;
            }
            else if (iFaCo1 == 2
                              && !ChFi3d_ComputeCurves(hs1, hs2, parfin, pardeb, ref gc, ref pGc1, ref pGc2, _tolapp3d, _tol2d,
                                                                                out tolreached))
            {
                _done = false;
                return _done;
            }
            // CornerData are updated with results of the intersection.
            double wFirst = gc.FirstParameter;
            double wLast = gc.LastParameter;
            int ipoin1 = 0;
            int ipoin2 = 0;
            ref CommonPoint cpco1 = ref fd1.ChangeVertex(isfirst1, iFaCo1);
            ref CommonPoint cpco2 = ref fd2.ChangeVertex(isfirst2, iFaCo2);
            double tolpco = Max(cpco1.Tolerance(), cpco2.Tolerance());
            ref CommonPoint cparc1 = ref fd1.ChangeVertex(isfirst1, iFaArc1);
            ref CommonPoint cparc2 = ref fd2.ChangeVertex(isfirst2, iFaArc2);
            double tolparc = Max(cparc1.Tolerance(), cparc2.Tolerance());
            int iCurv = dStr.AddCurve(new DsCurve(gc, tolreached));
            // Corner1
            corner1.SetParameters(isfirst1, wFirst, wLast);
            corner1.SetCurve(iCurv, isfirst1);
            corner1.ChangePCurve(isfirst1) = pGc1;
            cpco1.Reset();
            cpco1.SetPoint(pFaCo);
            cpco1.SetTolerance(Max(tolreached, tolpco));
            fd1.ChangeInterference(iFaCo1).SetParameter(uIntPC1, isfirst1);
            tolparc = Max(tolparc, tolreached);
            cparc1.SetTolerance(Max(tolparc, tolreached));
            ipoin1 = ChFi3d_IndexPointInDS(fd1.Vertex(isfirst1, 1), dStr);
            corner1.SetIndexPoint(ipoin1, isfirst1, 1);
            ipoin2 = ChFi3d_IndexPointInDS(fd1.Vertex(isfirst1, 2), dStr);
            corner1.SetIndexPoint(ipoin2, isfirst1, 2);
            // Corner2
            corner2.SetParameters(isfirst2, wFirst, wLast);
            corner2.SetCurve(iCurv, isfirst2);
            corner2.ChangePCurve(isfirst2) = pGc2;
            fd2.ChangeInterference(iFaCo2).SetParameter(uIntPC2, isfirst2);
            fd2.ChangeVertex(isfirst2, iFaCo2) = fd1.Vertex(isfirst1, iFaCo1);
            fd2.ChangeVertex(isfirst2, iFaArc2) = fd1.Vertex(isfirst1, iFaArc1);
            if (iFaCo1 != iFaCo2)
            {
                corner2.SetOrientation(Reversed, isfirst2);
            }
            corner2.SetIndexPoint(corner1.IndexPoint(isfirst1, iFaCo1), isfirst2, iFaCo2);
            corner2.SetIndexPoint(corner1.IndexPoint(isfirst1, iFaArc1), isfirst2, iFaArc2);
            // The tolerances of points are updated.
            Box bco = new(), barc = new();
            if (iFaCo1 == 1)
            {
                ChFi3d_EnlargeBox(dStr, corner1, fd1, ref bco, ref barc, isfirst1);
            }
            else
            {
                ChFi3d_EnlargeBox(dStr, corner1, fd1, ref barc, ref bco, isfirst1);
            }
            if (iFaCo2 == 1)
            {
                ChFi3d_EnlargeBox(dStr, corner2, fd2, ref bco, ref barc, isfirst2);
            }
            else
            {
                ChFi3d_EnlargeBox(dStr, corner2, fd2, ref barc, ref bco, isfirst2);
            }
            CommonPoint cparc = fd1.Vertex(isfirst1, iFaArc1);
            ChFi3d_EnlargeBox(cparc.Arc(), _efMap[cparc.Arc()], cparc.ParameterOnArc(), ref barc);
            ChFi3d_SetPointTolerance(dStr, barc, corner1.IndexPoint(isfirst1, iFaArc1));
            ChFi3d_SetPointTolerance(dStr, bco, corner1.IndexPoint(isfirst1, iFaCo1));
        }
        else
        {
            // It is necessary to identify the border surface,
            // find the end point of the intersection Surf/Surf
            // by the intersection of the tangency line of the small
            // on the opposing face with the surface of the big,
            // and finally intersect the big with the face at end
            // between this point and the point on arc.
            bool parcrois = false;
            Explorer expl = new();
            for (expl.Init(pivot.Oriented(Forward), ShapeType.Vertex); expl.More(); expl.Next())
            {
                if (expl.Current().IsSame(vtx))
                {
                    parcrois = (expl.Current().Orientation == Forward);
                    break;
                }
            }
            Stripe? bigCD = null, smaCD = null;
            SurfData? bigFD = null, smaFD = null;
            SurfaceAdaptor? bigHS = null, smaHS = null;
            int iFaCoBig = 0, iFaCoSma = 0, iFaArcBig = 0, iFaArcSma = 0;
            bool isfirstBig = false, isfirstSma = false;
            double uIntPCBig = 0.0, uIntPCSma = 0.0;

            if ((parcrois && parCP2 > parCP1) || (!parcrois && parCP2 < parCP1))
            {
                uIntPCBig = uIntPC2;
                uIntPCSma = uIntPC1;
                bigHS = hs2;
                smaHS = hs1;
                bigCD = corner2;
                smaCD = corner1;
                bigFD = fd2;
                smaFD = fd1;
                iFaCoBig = iFaCo2;
                iFaCoSma = iFaCo1;
                iFaArcBig = iFaArc2;
                iFaArcSma = iFaArc1;
                isfirstBig = isfirst2;
                isfirstSma = isfirst1;
            }
            else
            {
                uIntPCBig = uIntPC1;
                uIntPCSma = uIntPC2;
                bigHS = hs1;
                smaHS = hs2;
                bigCD = corner1;
                smaCD = corner2;
                bigFD = fd1;
                smaFD = fd2;
                iFaCoBig = iFaCo1;
                iFaCoSma = iFaCo2;
                iFaArcBig = iFaArc1;
                iFaArcSma = iFaArc2;
                isfirstBig = isfirst1;
                isfirstSma = isfirst2;
            }

            // Intersection of the big with the small :
            //------------------------------------

            // Pardeb (parameters of point PFaCo)
            // the intersection is checked
            ChFi3d_ComputesIntPC(smaFD.Interference(iFaCoSma), bigFD.Interference(iFaCoBig), smaHS, bigHS, ref uIntPCSma,
                                                      ref uIntPCBig);
            Point2D uVi = new();
            uVi = bigFD.Interference(iFaCoBig).PCurveOnSurf().Value(uIntPCBig);
            pardeb[3] = uVi.X;
            pardeb[4] = uVi.Y;
            uVi = smaFD.Interference(iFaCoSma).PCurveOnSurf().Value(uIntPCSma);
            pardeb[1] = uVi.X;
            pardeb[2] = uVi.Y;
            Point3D pFaCo = smaHS.Value(uVi.X, uVi.Y);

            // Parfin (parameters of point PMil)
            // OCCT holds a reference: it sees the SetParameter below
            ref FaceInterference fiArcSma = ref smaFD.ChangeInterference(iFaArcSma);
            Curve3D? ctg = dStr.Curve(fiArcSma.LineIndex()).Curve();
            CurveAdaptor? hctg = new CurveAdaptor();
            CurveAdaptor bid = hctg;
            double temp = 0.0, wi = 0.0;

            if (isfirstSma)
            {
                wi = temp = fiArcSma.FirstParameter();
                if (uIntPCSma < temp)
                {
                    temp = uIntPCSma;
                }
                bid.Load(ctg, temp, fiArcSma.LastParameter());
            }
            else
            {
                wi = temp = fiArcSma.LastParameter();
                if (uIntPCSma > temp)
                {
                    temp = uIntPCSma;
                }
                bid.Load(ctg, fiArcSma.FirstParameter(), temp);
            }
            if (smaFD.Surf() == bigFD.Surf())
            {
                Reduce(uIntPCSma, uIntPCBig, smaHS, bigHS);
                Reduce(uIntPCSma, uIntPCBig, hctg);
            }
            if (!ChFi3d_IntCS(bigHS, hctg, ref uVi, ref wi))
            {
                _done = false;
                return _done;
            }
            parfin[3] = uVi.X;
            parfin[4] = uVi.Y;
            uVi = fiArcSma.PCurveOnSurf().Value(wi);
            parfin[1] = uVi.X;
            parfin[2] = uVi.Y;
            Point3D pMil = smaHS.Value(parfin[1], parfin[2]);

            double tolreached = 0.0;
            if (!ChFi3d_ComputeCurves(smaHS, bigHS, pardeb, parfin, ref gc, ref pGc1, ref pGc2, _tolapp3d, _tol2d,
                                                                out tolreached))
            {
                _done = false;
                return _done;
            }
            // SmaCD is updated, for it this is all.
            double wFirst = gc.FirstParameter;
            double wLast = gc.LastParameter;
            int ipointCo = 0, ipointMil = 0, ipointArc = 0;
            ref CommonPoint psmaco = ref smaFD.ChangeVertex(isfirstSma, iFaCoSma);
            ref CommonPoint pbigco = ref bigFD.ChangeVertex(isfirstBig, iFaCoBig);
            double tolpco = Max(psmaco.Tolerance(), pbigco.Tolerance());
            ref CommonPoint psmamil = ref smaFD.ChangeVertex(isfirstSma, iFaArcSma);
            double tolpmil = psmamil.Tolerance();
            int iCurv = dStr.AddCurve(new DsCurve(gc, tolreached));

            smaCD.SetParameters(isfirstSma, wFirst, wLast);
            smaCD.SetCurve(iCurv, isfirstSma);
            smaCD.ChangePCurve(isfirstSma) = pGc1;
            psmaco.Reset();
            psmaco.SetPoint(pFaCo);
            psmaco.SetTolerance(Max(tolpco, tolreached));
            smaFD.ChangeInterference(iFaCoSma).SetParameter(uIntPCSma, isfirstSma);
            psmamil.Reset();
            psmamil.SetPoint(pMil);
            psmamil.SetTolerance(Max(tolpmil, tolreached));
            smaFD.ChangeInterference(iFaArcSma).SetParameter(wi, isfirstSma);
            ipointCo = ChFi3d_IndexPointInDS(psmaco, dStr);
            smaCD.SetIndexPoint(ipointCo, isfirstSma, iFaCoSma);
            ipointMil = ChFi3d_IndexPointInDS(psmamil, dStr);
            smaCD.SetIndexPoint(ipointMil, isfirstSma, iFaArcSma);
            if (iFaCoSma == 2)
            {
                smaCD.SetOrientation(Reversed, isfirstSma);
            }

            // For BigCD the first results are met in the DS.
            bigCD.SetIndexPoint(ipointCo, isfirstBig, iFaCoBig);
            bigFD.ChangeVertex(isfirstBig, iFaCoBig) = psmaco;
            bigFD.ChangeInterference(iFaCoBig).SetParameter(uIntPCBig, isfirstBig);

            NCollection_List<DsInterference> li = 
                dStr.ChangeCurveInterferences(iCurv);
            DsCurvePointInterference? interfp = null;
            interfp = ChFi3d_FilPointInDS(Forward, iCurv, ipointCo, wFirst);
            li.Append(interfp);
            interfp = ChFi3d_FilPointInDS(Reversed, iCurv, ipointMil, wLast);
            li.Append(interfp);

            // the transition of curves of intersection on the Big
            Orientation tra = bigFD.InterferenceOnS1().Transition();
            Orientation ofac = dStr.Shape(bigFD.IndexOfS1()).Orientation;
            Orientation ofil = bigFD.Orientation();
            Orientation tracurv = global::Kerf.Engine.Topology.TopAbs.Compose(ofac, ofil);
            tracurv = global::Kerf.Engine.Topology.TopAbs.Compose(tracurv, tra);
            if (!isfirstBig)
            {
                tracurv = global::Kerf.Engine.Topology.TopAbs.Reverse(tracurv);
            }
            if (iFaCoBig != 1)
            {
                tracurv = global::Kerf.Engine.Topology.TopAbs.Reverse(tracurv);
            }

            DsSurfaceCurveInterference? interfc = null;
            int iSurf = bigFD.Surf();
            interfc = ChFi3d_FilCurveInDS(iCurv, iSurf, pGc2, tracurv);
            dStr.ChangeSurfaceInterferences(iSurf).Append(interfc);

            // The tolerances of points are updated (beginning).
            Box bco = new(), bmil = new(), barc = new();
            if (iFaCoSma == 1)
            {
                ChFi3d_EnlargeBox(dStr, smaCD, smaFD, ref bco, ref bmil, isfirstSma);
            }
            else
            {
                ChFi3d_EnlargeBox(dStr, smaCD, smaFD, ref bmil, ref bco, isfirstSma);
            }
            ChFi3d_EnlargeBox(bigHS, pGc2, wFirst, wLast, ref bco, ref bmil);

            // Intersection of the big with the face at end :
            // -------------------------------------------

            // Pardeb (parameters of PMil)
            // The intersection curve surface is tried again, now with representation
            // pcurve on face of the curve to be sure.
            Face f = ShapeCasts.Face(dStr.Shape(smaFD.Index(iFaArcSma)));
            FaceSurfaceAdaptor? hf = new FaceSurfaceAdaptor(f);
            double fsma = fiArcSma.FirstParameter();
            double lsma = fiArcSma.LastParameter();
            double deltSma = 0.05 * (lsma - fsma);
            Curve2D? pcpc = smaFD.Interference(iFaArcSma).PCurveOnFace();
            fsma = Max(pcpc.FirstParameter, wi - deltSma);
            lsma = Min(pcpc.LastParameter, wi + deltSma);
            if (lsma < fsma)
            {
                _done = false;
                return _done;
            }
            CurveAdaptor2D? c2df = 
                new CurveAdaptor2D(smaFD.Interference(iFaArcSma).PCurveOnFace(), fsma, lsma);
            CurveOnSurface consf = new(c2df, hf);
            CurveOnSurface? hconsf = consf; // OCCT copies consf into a handle; consf is not used again

            if (!ChFi3d_IntCS(bigHS, hconsf, ref uVi, ref wi))
            {
                _done = false;
                return _done;
            }
            pardeb[3] = uVi.X;
            pardeb[4] = uVi.Y;
            uVi = smaFD.Interference(iFaArcSma).PCurveOnFace().Value(wi);
            pardeb[1] = uVi.X;
            pardeb[2] = uVi.Y;
            Point2D ppff1 = uVi;

            // Parfin (parameters of the point cpend)
            double ptg = bigFD.Interference(iFaArcBig).Parameter(isfirstBig);
            uVi = bigFD.Interference(iFaArcBig).PCurveOnSurf().Value(ptg);
            parfin[3] = uVi.X;
            parfin[4] = uVi.Y;
            ref CommonPoint cpend = ref bigFD.ChangeVertex(isfirstBig, iFaArcBig);
            Edge etest = cpend.Arc();
            if (Tool.IsClosed(etest, f))
            {
                etest.Reverse();
            }
            EdgeCurve2DAdaptor arc = new(etest, f);
            uVi = arc.Value(cpend.ParameterOnArc());
            parfin[1] = uVi.X;
            parfin[2] = uVi.Y;
            Point2D ppff2 = uVi;

            // Intersection.
            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(ppff1, ppff2, out uu1, out uu2, out vv1, out vv2);
            // for the case when two chamfers are on two edges OnSame,
            // it is necessary to extend the surface carrying F, or at least
            // not to limit it.
            ChFi3d_BoundFac(hf, uu1, uu2, vv1, vv2, true);

            if (!ChFi3d_ComputeCurves(hf, bigHS, pardeb, parfin, ref gc, ref pGc1, ref pGc2, _tolapp3d, _tol2d, out tolreached))
            {
                _done = false;
                return _done;
            }

            // End of update of the BigCD and the DS.
            wFirst = gc.FirstParameter;
            wLast = gc.LastParameter;
            iCurv = dStr.AddCurve(new DsCurve(gc, tolreached));
            cpend.SetTolerance(Max(cpend.Tolerance(), tolreached));
            ipointArc = ChFi3d_IndexPointInDS(cpend, dStr);
            bigCD.SetIndexPoint(ipointArc, isfirstBig, iFaArcBig);

            NCollection_List<DsInterference> li7 = 
                dStr.ChangeCurveInterferences(iCurv);
            interfp = ChFi3d_FilPointInDS(Forward, iCurv, ipointMil, wFirst);
            li7.Append(interfp);
            interfp = ChFi3d_FilPointInDS(Reversed, iCurv, ipointArc, wLast);
            li7.Append(interfp);
            interfc = ChFi3d_FilCurveInDS(iCurv, iSurf, pGc2, tracurv);
            dStr.ChangeSurfaceInterferences(iSurf).Append(interfc);
            bigCD.InDS(isfirstBig);

            // Finally the information on faces is placed in the DS.
            int iShape = dStr.AddShape(f);
            if (smaFD.Surf() == bigFD.Surf())
            {
                tracurv = global::Kerf.Engine.Topology.TopAbs.Compose(etest.Orientation, cpend.TransitionOnArc());
            }
            else
            {
                Explorer exp = new();
                for (exp.Init(f.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
                {
                    if (exp.Current().IsSame(etest))
                    {
                        tracurv = global::Kerf.Engine.Topology.TopAbs.Compose(exp.Current().Orientation, cpend.TransitionOnArc());
                        break;
                    }
                }
            }
            interfc = ChFi3d_FilCurveInDS(iCurv, iShape, pGc1, tracurv);
            dStr.ChangeShapeInterferences(iShape).Append(interfc);

            // The tolerances of points are updated (end).
            Stripe? bidst = null;
            if (iFaCoBig == 1)
            {
                ChFi3d_EnlargeBox(dStr, bidst, bigFD, ref bco, ref barc, isfirstBig);
            }
            else
            {
                ChFi3d_EnlargeBox(dStr, bidst, bigFD, ref barc, ref bco, isfirstBig);
            }
            ChFi3d_EnlargeBox(bigHS, pGc2, wFirst, wLast, ref bmil, ref barc);

            ChFi3d_EnlargeBox(hf, pGc1, wFirst, wLast, ref bmil, ref barc);

            ChFi3d_EnlargeBox(gc, wFirst, wLast, ref bmil, ref barc);
            CommonPoint cparc = bigFD.Vertex(isfirstBig, iFaArcBig);
            ChFi3d_EnlargeBox(cparc.Arc(), _efMap[cparc.Arc()], cparc.ParameterOnArc(), ref barc);

            ChFi3d_SetPointTolerance(dStr, bco, smaCD.IndexPoint(isfirstSma, iFaCoSma));
            ChFi3d_SetPointTolerance(dStr, bmil, smaCD.IndexPoint(isfirstSma, iFaArcSma));
            ChFi3d_SetPointTolerance(dStr, barc, bigCD.IndexPoint(isfirstBig, iFaArcBig));
        }
        _done = true;

        return _done;
    }

    //=======================================================================
    // function : UpdateTolesp
    // purpose  : Update tolesp field to fit it to range of parameter of spine curve
    //=======================================================================

    private void UpdateTolesp()
    {
        NCollection_List<Stripe>.Iterator itel = new();
        // tolesp = Precision::Infinite();
        for (itel.Initialize(_listStripe); itel.More(); itel.Next())
        {
            Stripe? curStripe = itel.ChangeValue();
            Spine? spine = curStripe.ChangeSpine();
            double current_stripe_tolesp = spine.GetTolesp();
            if (_tolesp > current_stripe_tolesp)
            {
                _tolesp = current_stripe_tolesp;
            }
        }
    }

}

internal static class Builder_C2
{
    internal static void Reduce(double p1, double p2, SurfaceAdaptor? hs1, SurfaceAdaptor? hs2)
    {
        SurfaceAdaptor s1 = hs1;
        SurfaceAdaptor s2 = hs2;
        Surface? surf = s1.Surface();
        double ud = 0.0, uf = 0.0, vd = 0.0, vf = 0.0;
        surf.ParameterBounds(out ud, out uf, out vd, out vf);
        double milmoins = 0.51 * vd + 0.49 * vf, milplus = 0.49 * vd + 0.51 * vf;
        if (p1 < p2)
        {
            s1.Load(surf, ud, uf, vd, milmoins);
            s2.Load(surf, ud, uf, milplus, vf);
        }
        else
        {
            s1.Load(surf, ud, uf, milplus, vf);
            s2.Load(surf, ud, uf, vd, milmoins);
        }
    }

    internal static void Reduce(double p1, double p2, CurveAdaptor? hc)
    {
        CurveAdaptor c = hc;
        double f = c.FirstParameter();
        double l = c.LastParameter();
        double milmoins = 0.51 * f + 0.49 * l, milplus = 0.49 * f + 0.51 * l;
        if (p1 < p2)
        {
            c.Load(c.Curve(), f, milmoins);
        }
        else
        {
            c.Load(c.Curve(), milplus, l);
        }
    }


}
