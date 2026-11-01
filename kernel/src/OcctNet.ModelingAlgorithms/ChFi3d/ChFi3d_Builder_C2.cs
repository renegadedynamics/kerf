// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_C2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_Builder_C2;
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


    //=======================================================================
    // function : PerformTwoCornerbyInter
    // purpose  : Performs PerformTwoCorner by intersection.
    //           In case of Biseau for all cases the
    //           path is used; 3D curve and 2 pcurves are approximated.
    //=======================================================================

    public bool PerformTwoCornerbyInter(int Index)
    {
        done = false;
        TopoDS_Vertex Vtx = myVDataMap.FindKey(Index);
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();

        // Information on fillets is extracted
        //------------------------------------------------------

        // the first
        //----------
        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        It.Initialize(myVDataMap[Index]);
        ChFiDS_Stripe? Corner1 = It.ChangeValue();
        int Sens1 = 0;
        int IFd1 = ChFi3d_IndexOfSurfData(Vtx, Corner1, out Sens1);
        NCollection_Sequence<ChFiDS_SurfData> SeqFil1 = 
            Corner1.ChangeSetOfSurfData();
        ChFiDS_SurfData? Fd1 = SeqFil1.ChangeValue(IFd1);

        // the second
        //----------
        It.Next();
        ChFiDS_Stripe? Corner2 = It.ChangeValue();
        int Sens2 = 0;
        int IFd2 = 0;
        if (Corner2 == Corner1)
        {
            Sens2 = -1;
            IFd2 = Corner2.SetOfSurfData().Length();
        }
        else
        {
            IFd2 = ChFi3d_IndexOfSurfData(Vtx, Corner2, out Sens2);
        }
        NCollection_Sequence<ChFiDS_SurfData> SeqFil2 = 
            Corner2.ChangeSetOfSurfData();
        ChFiDS_SurfData? Fd2 = SeqFil2.ChangeValue(IFd2);

        // The concavities are analysed in case of different concavities,
        // preview an evolutionary connection of type ThreeCorner of R to 0.
        // Otherwise the opposite face
        // and the eventual intersection of 2 pcurves on this face are found.

        bool isfirst1 = (Sens1 == 1);
        bool isfirst2 = (Sens2 == 1);
        bool OkinterCC = false, Okvisavis = false, SameSide = false;
        int IFaCo1 = 0, IFaCo2 = 0;
        double UIntPC1 = 0.0, UIntPC2 = 0.0;
        TopoDS_Face FaCo = new();
        OkinterCC = ChFi3d_IsInFront(DStr, Corner1, Corner2, IFd1, IFd2, Sens1, Sens2, ref UIntPC1, ref UIntPC2, ref FaCo,
                                                                  ref SameSide, ref IFaCo1, ref IFaCo2, ref Okvisavis, Vtx, true);
        if (!Okvisavis)
        {
            done = false;
            return done;
        }
        if (!OkinterCC)
        {
            // The intersection of pcurves is calculated without restricting them by
            // common points.
            OkinterCC = ChFi3d_IsInFront(DStr, Corner1, Corner2, IFd1, IFd2, Sens1, Sens2, ref UIntPC1, ref UIntPC2, ref FaCo,
                                                                      ref SameSide, ref IFaCo1, ref IFaCo2, ref Okvisavis, Vtx, true, true);
        }

        if (!Okvisavis)
        {
            done = false;
            return done;
        }
        if (!OkinterCC)
        {
            done = false;
            return done;
        }
        int IFaArc1 = 3 - IFaCo1, IFaArc2 = 3 - IFaCo2;

        // It is checked if the fillets have a commonpoint on a common arc.
        // This edge is the pivot of the bevel or of the kneecap.

        ref ChFiDS_CommonPoint CP1 = ref Fd1.ChangeVertex(isfirst1, IFaArc1);
        ref ChFiDS_CommonPoint CP2 = ref Fd2.ChangeVertex(isfirst2, IFaArc2);

        if (!CP1.IsOnArc() || !CP2.IsOnArc())
        {
            done = false;
            return done;
        }
        if (!CP1.Arc().IsSame(CP2.Arc()))
        {
            // look like OnSame + OnDiff case (eap, Arp 9 2002, occ266)
            done = true;
            PerformMoreThreeCorner(Index, 2);
            return done;
        }

        TopoDS_Edge pivot = new();
        pivot = CP1.Arc();
        double parCP1 = CP1.ParameterOnArc();
        double parCP2 = CP2.ParameterOnArc();
        BRepAdaptor_Curve? Hpivot = new BRepAdaptor_Curve(pivot);
        if (!pivot.IsSame(CP2.Arc()))
        {
            Geom_Curve? csau = null;
            double ubid = 0.0, vbid = 0.0;
            csau = BRep_Tool.Curve(pivot, out ubid, out vbid);
            Geom_BoundedCurve? C1 = (csau as Geom_BoundedCurve);
            if (C1 is not null)
            {
                GeomLib.ExtendCurveToPoint(ref C1, CP2.Point(), 1, false);
                GeomAdaptor_Curve cad = new();
                cad.Load(C1);
                Extrema_ExtPC ext = new(CP2.Point(), cad, 1.0e-4);
                parCP2 = ext.Point(1).Parameter();
            }
        }
        gp_Pnt psp1 = Hpivot.Value(parCP1);
        gp_Pnt psp2 = Hpivot.Value(parCP2);
        double sameparam = (psp1.Distance(psp2) < 10.0 * tolapp3d) ? 1.0 : 0.0; // OCCT: a bool in a double

        TopoDS_Face FF1 = TopoDS.Face(DStr.Shape(Fd1.Index(IFaArc1)));
        TopoDS_Face FF2 = TopoDS.Face(DStr.Shape(Fd2.Index(IFaArc2)));
        NCollection_List<TopoDS_Shape>.Iterator Kt = new();
        bool ok1 = false, ok2 = false;
        for (Kt.Initialize(myEFMap[pivot]); Kt.More(); Kt.Next())
        {
            TopoDS_Face F = TopoDS.Face(Kt.Value());
            if (!ok1 && FF1.IsSame(F))
            {
                ok1 = true;
            }
            if (!ok2 && FF2.IsSame(F))
            {
                ok2 = true;
            }
        }
        if (!ok1 || !ok2)
        {
            done = false;
            return done;
        }

        GeomAdaptor_Surface? HS1 = null, HS2 = null;
        HS1 = ChFi3d_BoundSurf(DStr, Fd1, IFaCo1, IFaArc1);
        HS2 = ChFi3d_BoundSurf(DStr, Fd2, IFaCo2, IFaArc2);

        NCollection_Array1<double> Pardeb = new(1, 4), Parfin = new(1, 4);

        Geom2d_Curve? PGc1 = null, PGc2 = null;
        Geom_Curve? Gc = null;

        if (sameparam != 0.0)
        {
            // Side common face, calculation of Pardeb.
            ChFi3d_ComputesIntPC(Fd1.Interference(IFaCo1), Fd2.Interference(IFaCo2), HS1, HS2, ref UIntPC1, ref UIntPC2);
            gp_Pnt2d UV = new();
            UV = Fd1.Interference(IFaCo1).PCurveOnSurf().Value(UIntPC1);
            Pardeb[1] = UV.X();
            Pardeb[2] = UV.Y();
            UV = Fd2.Interference(IFaCo2).PCurveOnSurf().Value(UIntPC2);
            Pardeb[3] = UV.X();
            Pardeb[4] = UV.Y();
            gp_Pnt PFaCo = HS1.Value(Pardeb[1], Pardeb[2]);

            // Side arc, calculation of Parfin.
            double UIntArc1 = Fd1.Interference(IFaArc1).Parameter(isfirst1);
            double UIntArc2 = Fd2.Interference(IFaArc2).Parameter(isfirst2);

            ChFi3d_ComputesIntPC(Fd1.Interference(IFaArc1), Fd2.Interference(IFaArc2), HS1, HS2, ref UIntArc1, ref UIntArc2);
            UV = Fd1.Interference(IFaArc1).PCurveOnSurf().Value(UIntArc1);
            Parfin[1] = UV.X();
            Parfin[2] = UV.Y();
            UV = Fd2.Interference(IFaArc2).PCurveOnSurf().Value(UIntArc2);
            Parfin[3] = UV.X();
            Parfin[4] = UV.Y();

            if (Fd1.Surf() == Fd2.Surf())
            {
                Reduce(UIntPC1, UIntPC2, HS1, HS2);
            }

            double tolreached = tolapp3d;
            if (IFaCo1 == 1
                    && !ChFi3d_ComputeCurves(HS1, HS2, Pardeb, Parfin, ref Gc, ref PGc1, ref PGc2, tolapp3d, tol2d,
                                                                      out tolreached))
            {
                done = false;
                return done;
            }
            else if (IFaCo1 == 2
                              && !ChFi3d_ComputeCurves(HS1, HS2, Parfin, Pardeb, ref Gc, ref PGc1, ref PGc2, tolapp3d, tol2d,
                                                                                out tolreached))
            {
                done = false;
                return done;
            }
            // CornerData are updated with results of the intersection.
            double WFirst = Gc.FirstParameter();
            double WLast = Gc.LastParameter();
            int Ipoin1 = 0;
            int Ipoin2 = 0;
            ref ChFiDS_CommonPoint cpco1 = ref Fd1.ChangeVertex(isfirst1, IFaCo1);
            ref ChFiDS_CommonPoint cpco2 = ref Fd2.ChangeVertex(isfirst2, IFaCo2);
            double tolpco = Max(cpco1.Tolerance(), cpco2.Tolerance());
            ref ChFiDS_CommonPoint cparc1 = ref Fd1.ChangeVertex(isfirst1, IFaArc1);
            ref ChFiDS_CommonPoint cparc2 = ref Fd2.ChangeVertex(isfirst2, IFaArc2);
            double tolparc = Max(cparc1.Tolerance(), cparc2.Tolerance());
            int ICurv = DStr.AddCurve(new TopOpeBRepDS_Curve(Gc, tolreached));
            // Corner1
            Corner1.SetParameters(isfirst1, WFirst, WLast);
            Corner1.SetCurve(ICurv, isfirst1);
            Corner1.ChangePCurve(isfirst1) = PGc1;
            cpco1.Reset();
            cpco1.SetPoint(PFaCo);
            cpco1.SetTolerance(Max(tolreached, tolpco));
            Fd1.ChangeInterference(IFaCo1).SetParameter(UIntPC1, isfirst1);
            tolparc = Max(tolparc, tolreached);
            cparc1.SetTolerance(Max(tolparc, tolreached));
            Ipoin1 = ChFi3d_IndexPointInDS(Fd1.Vertex(isfirst1, 1), DStr);
            Corner1.SetIndexPoint(Ipoin1, isfirst1, 1);
            Ipoin2 = ChFi3d_IndexPointInDS(Fd1.Vertex(isfirst1, 2), DStr);
            Corner1.SetIndexPoint(Ipoin2, isfirst1, 2);
            // Corner2
            Corner2.SetParameters(isfirst2, WFirst, WLast);
            Corner2.SetCurve(ICurv, isfirst2);
            Corner2.ChangePCurve(isfirst2) = PGc2;
            Fd2.ChangeInterference(IFaCo2).SetParameter(UIntPC2, isfirst2);
            Fd2.ChangeVertex(isfirst2, IFaCo2) = Fd1.Vertex(isfirst1, IFaCo1);
            Fd2.ChangeVertex(isfirst2, IFaArc2) = Fd1.Vertex(isfirst1, IFaArc1);
            if (IFaCo1 != IFaCo2)
            {
                Corner2.SetOrientation(TopAbs_REVERSED, isfirst2);
            }
            Corner2.SetIndexPoint(Corner1.IndexPoint(isfirst1, IFaCo1), isfirst2, IFaCo2);
            Corner2.SetIndexPoint(Corner1.IndexPoint(isfirst1, IFaArc1), isfirst2, IFaArc2);
            // The tolerances of points are updated.
            Bnd_Box bco = new(), barc = new();
            if (IFaCo1 == 1)
            {
                ChFi3d_EnlargeBox(DStr, Corner1, Fd1, ref bco, ref barc, isfirst1);
            }
            else
            {
                ChFi3d_EnlargeBox(DStr, Corner1, Fd1, ref barc, ref bco, isfirst1);
            }
            if (IFaCo2 == 1)
            {
                ChFi3d_EnlargeBox(DStr, Corner2, Fd2, ref bco, ref barc, isfirst2);
            }
            else
            {
                ChFi3d_EnlargeBox(DStr, Corner2, Fd2, ref barc, ref bco, isfirst2);
            }
            ChFiDS_CommonPoint cparc = Fd1.Vertex(isfirst1, IFaArc1);
            ChFi3d_EnlargeBox(cparc.Arc(), myEFMap[cparc.Arc()], cparc.ParameterOnArc(), ref barc);
            ChFi3d_SetPointTolerance(DStr, barc, Corner1.IndexPoint(isfirst1, IFaArc1));
            ChFi3d_SetPointTolerance(DStr, bco, Corner1.IndexPoint(isfirst1, IFaCo1));
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
            TopExp_Explorer Expl = new();
            for (Expl.Init(pivot.Oriented(TopAbs_FORWARD), TopAbs_VERTEX); Expl.More(); Expl.Next())
            {
                if (Expl.Current().IsSame(Vtx))
                {
                    parcrois = (Expl.Current().Orientation() == TopAbs_FORWARD);
                    break;
                }
            }
            ChFiDS_Stripe? BigCD = null, SmaCD = null;
            ChFiDS_SurfData? BigFD = null, SmaFD = null;
            GeomAdaptor_Surface? BigHS = null, SmaHS = null;
            int IFaCoBig = 0, IFaCoSma = 0, IFaArcBig = 0, IFaArcSma = 0;
            bool isfirstBig = false, isfirstSma = false;
            double UIntPCBig = 0.0, UIntPCSma = 0.0;

            if ((parcrois && parCP2 > parCP1) || (!parcrois && parCP2 < parCP1))
            {
                UIntPCBig = UIntPC2;
                UIntPCSma = UIntPC1;
                BigHS = HS2;
                SmaHS = HS1;
                BigCD = Corner2;
                SmaCD = Corner1;
                BigFD = Fd2;
                SmaFD = Fd1;
                IFaCoBig = IFaCo2;
                IFaCoSma = IFaCo1;
                IFaArcBig = IFaArc2;
                IFaArcSma = IFaArc1;
                isfirstBig = isfirst2;
                isfirstSma = isfirst1;
            }
            else
            {
                UIntPCBig = UIntPC1;
                UIntPCSma = UIntPC2;
                BigHS = HS1;
                SmaHS = HS2;
                BigCD = Corner1;
                SmaCD = Corner2;
                BigFD = Fd1;
                SmaFD = Fd2;
                IFaCoBig = IFaCo1;
                IFaCoSma = IFaCo2;
                IFaArcBig = IFaArc1;
                IFaArcSma = IFaArc2;
                isfirstBig = isfirst1;
                isfirstSma = isfirst2;
            }

            // Intersection of the big with the small :
            //------------------------------------

            // Pardeb (parameters of point PFaCo)
            // the intersection is checked
            ChFi3d_ComputesIntPC(SmaFD.Interference(IFaCoSma), BigFD.Interference(IFaCoBig), SmaHS, BigHS, ref UIntPCSma,
                                                      ref UIntPCBig);
            gp_Pnt2d UVi = new();
            UVi = BigFD.Interference(IFaCoBig).PCurveOnSurf().Value(UIntPCBig);
            Pardeb[3] = UVi.X();
            Pardeb[4] = UVi.Y();
            UVi = SmaFD.Interference(IFaCoSma).PCurveOnSurf().Value(UIntPCSma);
            Pardeb[1] = UVi.X();
            Pardeb[2] = UVi.Y();
            gp_Pnt PFaCo = SmaHS.Value(UVi.X(), UVi.Y());

            // Parfin (parameters of point PMil)
            // OCCT holds a reference: it sees the SetParameter below
            ref ChFiDS_FaceInterference FiArcSma = ref SmaFD.ChangeInterference(IFaArcSma);
            Geom_Curve? ctg = DStr.Curve(FiArcSma.LineIndex()).Curve();
            GeomAdaptor_Curve? Hctg = new GeomAdaptor_Curve();
            GeomAdaptor_Curve bid = Hctg;
            double temp = 0.0, wi = 0.0;

            if (isfirstSma)
            {
                wi = temp = FiArcSma.FirstParameter();
                if (UIntPCSma < temp)
                {
                    temp = UIntPCSma;
                }
                bid.Load(ctg, temp, FiArcSma.LastParameter());
            }
            else
            {
                wi = temp = FiArcSma.LastParameter();
                if (UIntPCSma > temp)
                {
                    temp = UIntPCSma;
                }
                bid.Load(ctg, FiArcSma.FirstParameter(), temp);
            }
            if (SmaFD.Surf() == BigFD.Surf())
            {
                Reduce(UIntPCSma, UIntPCBig, SmaHS, BigHS);
                Reduce(UIntPCSma, UIntPCBig, Hctg);
            }
            if (!ChFi3d_IntCS(BigHS, Hctg, ref UVi, ref wi))
            {
                done = false;
                return done;
            }
            Parfin[3] = UVi.X();
            Parfin[4] = UVi.Y();
            UVi = FiArcSma.PCurveOnSurf().Value(wi);
            Parfin[1] = UVi.X();
            Parfin[2] = UVi.Y();
            gp_Pnt PMil = SmaHS.Value(Parfin[1], Parfin[2]);

            double tolreached = 0.0;
            if (!ChFi3d_ComputeCurves(SmaHS, BigHS, Pardeb, Parfin, ref Gc, ref PGc1, ref PGc2, tolapp3d, tol2d,
                                                                out tolreached))
            {
                done = false;
                return done;
            }
            // SmaCD is updated, for it this is all.
            double WFirst = Gc.FirstParameter();
            double WLast = Gc.LastParameter();
            int IpointCo = 0, IpointMil = 0, IpointArc = 0;
            ref ChFiDS_CommonPoint psmaco = ref SmaFD.ChangeVertex(isfirstSma, IFaCoSma);
            ref ChFiDS_CommonPoint pbigco = ref BigFD.ChangeVertex(isfirstBig, IFaCoBig);
            double tolpco = Max(psmaco.Tolerance(), pbigco.Tolerance());
            ref ChFiDS_CommonPoint psmamil = ref SmaFD.ChangeVertex(isfirstSma, IFaArcSma);
            double tolpmil = psmamil.Tolerance();
            int ICurv = DStr.AddCurve(new TopOpeBRepDS_Curve(Gc, tolreached));

            SmaCD.SetParameters(isfirstSma, WFirst, WLast);
            SmaCD.SetCurve(ICurv, isfirstSma);
            SmaCD.ChangePCurve(isfirstSma) = PGc1;
            psmaco.Reset();
            psmaco.SetPoint(PFaCo);
            psmaco.SetTolerance(Max(tolpco, tolreached));
            SmaFD.ChangeInterference(IFaCoSma).SetParameter(UIntPCSma, isfirstSma);
            psmamil.Reset();
            psmamil.SetPoint(PMil);
            psmamil.SetTolerance(Max(tolpmil, tolreached));
            SmaFD.ChangeInterference(IFaArcSma).SetParameter(wi, isfirstSma);
            IpointCo = ChFi3d_IndexPointInDS(psmaco, DStr);
            SmaCD.SetIndexPoint(IpointCo, isfirstSma, IFaCoSma);
            IpointMil = ChFi3d_IndexPointInDS(psmamil, DStr);
            SmaCD.SetIndexPoint(IpointMil, isfirstSma, IFaArcSma);
            if (IFaCoSma == 2)
            {
                SmaCD.SetOrientation(TopAbs_REVERSED, isfirstSma);
            }

            // For BigCD the first results are met in the DS.
            BigCD.SetIndexPoint(IpointCo, isfirstBig, IFaCoBig);
            BigFD.ChangeVertex(isfirstBig, IFaCoBig) = psmaco;
            BigFD.ChangeInterference(IFaCoBig).SetParameter(UIntPCBig, isfirstBig);

            NCollection_List<TopOpeBRepDS_Interference> Li = 
                DStr.ChangeCurveInterferences(ICurv);
            TopOpeBRepDS_CurvePointInterference? Interfp = null;
            Interfp = ChFi3d_FilPointInDS(TopAbs_FORWARD, ICurv, IpointCo, WFirst);
            Li.Append(Interfp);
            Interfp = ChFi3d_FilPointInDS(TopAbs_REVERSED, ICurv, IpointMil, WLast);
            Li.Append(Interfp);

            // the transition of curves of intersection on the Big
            TopAbs_Orientation tra = BigFD.InterferenceOnS1().Transition();
            TopAbs_Orientation ofac = DStr.Shape(BigFD.IndexOfS1()).Orientation();
            TopAbs_Orientation ofil = BigFD.Orientation();
            TopAbs_Orientation tracurv = TopAbs.Compose(ofac, ofil);
            tracurv = TopAbs.Compose(tracurv, tra);
            if (!isfirstBig)
            {
                tracurv = TopAbs.Reverse(tracurv);
            }
            if (IFaCoBig != 1)
            {
                tracurv = TopAbs.Reverse(tracurv);
            }

            TopOpeBRepDS_SurfaceCurveInterference? Interfc = null;
            int ISurf = BigFD.Surf();
            Interfc = ChFi3d_FilCurveInDS(ICurv, ISurf, PGc2, tracurv);
            DStr.ChangeSurfaceInterferences(ISurf).Append(Interfc);

            // The tolerances of points are updated (beginning).
            Bnd_Box bco = new(), bmil = new(), barc = new();
            if (IFaCoSma == 1)
            {
                ChFi3d_EnlargeBox(DStr, SmaCD, SmaFD, ref bco, ref bmil, isfirstSma);
            }
            else
            {
                ChFi3d_EnlargeBox(DStr, SmaCD, SmaFD, ref bmil, ref bco, isfirstSma);
            }
            ChFi3d_EnlargeBox(BigHS, PGc2, WFirst, WLast, ref bco, ref bmil);

            // Intersection of the big with the face at end :
            // -------------------------------------------

            // Pardeb (parameters of PMil)
            // The intersection curve surface is tried again, now with representation
            // pcurve on face of the curve to be sure.
            TopoDS_Face F = TopoDS.Face(DStr.Shape(SmaFD.Index(IFaArcSma)));
            BRepAdaptor_Surface? HF = new BRepAdaptor_Surface(F);
            double fsma = FiArcSma.FirstParameter();
            double lsma = FiArcSma.LastParameter();
            double deltSma = 0.05 * (lsma - fsma);
            Geom2d_Curve? pcpc = SmaFD.Interference(IFaArcSma).PCurveOnFace();
            fsma = Max(pcpc.FirstParameter(), wi - deltSma);
            lsma = Min(pcpc.LastParameter(), wi + deltSma);
            if (lsma < fsma)
            {
                done = false;
                return done;
            }
            Geom2dAdaptor_Curve? c2df = 
                new Geom2dAdaptor_Curve(SmaFD.Interference(IFaArcSma).PCurveOnFace(), fsma, lsma);
            Adaptor3d_CurveOnSurface consf = new(c2df, HF);
            Adaptor3d_CurveOnSurface? Hconsf = consf; // OCCT copies consf into a handle; consf is not used again

            if (!ChFi3d_IntCS(BigHS, Hconsf, ref UVi, ref wi))
            {
                done = false;
                return done;
            }
            Pardeb[3] = UVi.X();
            Pardeb[4] = UVi.Y();
            UVi = SmaFD.Interference(IFaArcSma).PCurveOnFace().Value(wi);
            Pardeb[1] = UVi.X();
            Pardeb[2] = UVi.Y();
            gp_Pnt2d ppff1 = UVi;

            // Parfin (parameters of the point cpend)
            double ptg = BigFD.Interference(IFaArcBig).Parameter(isfirstBig);
            UVi = BigFD.Interference(IFaArcBig).PCurveOnSurf().Value(ptg);
            Parfin[3] = UVi.X();
            Parfin[4] = UVi.Y();
            ref ChFiDS_CommonPoint cpend = ref BigFD.ChangeVertex(isfirstBig, IFaArcBig);
            TopoDS_Edge etest = cpend.Arc();
            if (BRep_Tool.IsClosed(etest, F))
            {
                etest.Reverse();
            }
            BRepAdaptor_Curve2d arc = new(etest, F);
            UVi = arc.Value(cpend.ParameterOnArc());
            Parfin[1] = UVi.X();
            Parfin[2] = UVi.Y();
            gp_Pnt2d ppff2 = UVi;

            // Intersection.
            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(ppff1, ppff2, out uu1, out uu2, out vv1, out vv2);
            // for the case when two chamfers are on two edges OnSame,
            // it is necessary to extend the surface carrying F, or at least
            // not to limit it.
            ChFi3d_BoundFac(HF, uu1, uu2, vv1, vv2, true);

            if (!ChFi3d_ComputeCurves(HF, BigHS, Pardeb, Parfin, ref Gc, ref PGc1, ref PGc2, tolapp3d, tol2d, out tolreached))
            {
                done = false;
                return done;
            }

            // End of update of the BigCD and the DS.
            WFirst = Gc.FirstParameter();
            WLast = Gc.LastParameter();
            ICurv = DStr.AddCurve(new TopOpeBRepDS_Curve(Gc, tolreached));
            cpend.SetTolerance(Max(cpend.Tolerance(), tolreached));
            IpointArc = ChFi3d_IndexPointInDS(cpend, DStr);
            BigCD.SetIndexPoint(IpointArc, isfirstBig, IFaArcBig);

            NCollection_List<TopOpeBRepDS_Interference> Li7 = 
                DStr.ChangeCurveInterferences(ICurv);
            Interfp = ChFi3d_FilPointInDS(TopAbs_FORWARD, ICurv, IpointMil, WFirst);
            Li7.Append(Interfp);
            Interfp = ChFi3d_FilPointInDS(TopAbs_REVERSED, ICurv, IpointArc, WLast);
            Li7.Append(Interfp);
            Interfc = ChFi3d_FilCurveInDS(ICurv, ISurf, PGc2, tracurv);
            DStr.ChangeSurfaceInterferences(ISurf).Append(Interfc);
            BigCD.InDS(isfirstBig);

            // Finally the information on faces is placed in the DS.
            int IShape = DStr.AddShape(F);
            if (SmaFD.Surf() == BigFD.Surf())
            {
                tracurv = TopAbs.Compose(etest.Orientation(), cpend.TransitionOnArc());
            }
            else
            {
                TopExp_Explorer Exp = new();
                for (Exp.Init(F.Oriented(TopAbs_FORWARD), TopAbs_EDGE); Exp.More(); Exp.Next())
                {
                    if (Exp.Current().IsSame(etest))
                    {
                        tracurv = TopAbs.Compose(Exp.Current().Orientation(), cpend.TransitionOnArc());
                        break;
                    }
                }
            }
            Interfc = ChFi3d_FilCurveInDS(ICurv, IShape, PGc1, tracurv);
            DStr.ChangeShapeInterferences(IShape).Append(Interfc);

            // The tolerances of points are updated (end).
            ChFiDS_Stripe? bidst = null;
            if (IFaCoBig == 1)
            {
                ChFi3d_EnlargeBox(DStr, bidst, BigFD, ref bco, ref barc, isfirstBig);
            }
            else
            {
                ChFi3d_EnlargeBox(DStr, bidst, BigFD, ref barc, ref bco, isfirstBig);
            }
            ChFi3d_EnlargeBox(BigHS, PGc2, WFirst, WLast, ref bmil, ref barc);

            ChFi3d_EnlargeBox(HF, PGc1, WFirst, WLast, ref bmil, ref barc);

            ChFi3d_EnlargeBox(Gc, WFirst, WLast, ref bmil, ref barc);
            ChFiDS_CommonPoint cparc = BigFD.Vertex(isfirstBig, IFaArcBig);
            ChFi3d_EnlargeBox(cparc.Arc(), myEFMap[cparc.Arc()], cparc.ParameterOnArc(), ref barc);

            ChFi3d_SetPointTolerance(DStr, bco, SmaCD.IndexPoint(isfirstSma, IFaCoSma));
            ChFi3d_SetPointTolerance(DStr, bmil, SmaCD.IndexPoint(isfirstSma, IFaArcSma));
            ChFi3d_SetPointTolerance(DStr, barc, BigCD.IndexPoint(isfirstBig, IFaArcBig));
        }
        done = true;

        return done;
    }

    //=======================================================================
    // function : UpdateTolesp
    // purpose  : Update tolesp field to fit it to range of parameter of spine curve
    //=======================================================================

    private void UpdateTolesp()
    {
        NCollection_List<ChFiDS_Stripe>.Iterator itel = new();
        // tolesp = Precision::Infinite();
        for (itel.Initialize(myListStripe); itel.More(); itel.Next())
        {
            ChFiDS_Stripe? curStripe = itel.ChangeValue();
            ChFiDS_Spine? Spine = curStripe.ChangeSpine();
            double current_stripe_tolesp = Spine.GetTolesp();
            if (tolesp > current_stripe_tolesp)
            {
                tolesp = current_stripe_tolesp;
            }
        }
    }

}

internal static class ChFi3d_Builder_C2
{
    internal static void Reduce(double p1, double p2, GeomAdaptor_Surface? hs1, GeomAdaptor_Surface? hs2)
    {
        GeomAdaptor_Surface s1 = hs1;
        GeomAdaptor_Surface s2 = hs2;
        Geom_Surface? surf = s1.Surface();
        double ud = 0.0, uf = 0.0, vd = 0.0, vf = 0.0;
        surf.Bounds(out ud, out uf, out vd, out vf);
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

    internal static void Reduce(double p1, double p2, GeomAdaptor_Curve? hc)
    {
        GeomAdaptor_Curve c = hc;
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
