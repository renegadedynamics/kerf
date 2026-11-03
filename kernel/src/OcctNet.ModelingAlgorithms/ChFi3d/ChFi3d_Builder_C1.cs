// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_C1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ChFi3d_Builder_0;
using static Occt.ChFiKPart_ComputeData_Fcts;
using static Kerf.Engine.Extrema.ExtFlag;
using static Occt.ChFi3d_Builder_C1;
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

    //  Modified by skv - Mon Jun  7 18:38:57 2004 OCC5898
    //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222



    //=======================================================================
    // function : Update
    // purpose  : Calculate the intersection of the face at the end of
    //           the tangency line to update CommonPoint and its
    //           parameter in FaceInterference.
    //=======================================================================

    //=======================================================================
    // function : Update
    // purpose  : Intersect surface <fb> and 3d curve <ct>
    //           Update <isfirst> parameter of FaceInterference <fi> and point of
    //           CommonPoint <cp>. Return new intersection parameters in <wop>
    //           and <p2dbout>
    //=======================================================================

    //=======================================================================
    // function : IntersUpdateOnSame
    // purpose  : Intersect  curve <c3dFI> of ChFi-<Fop> interference with extended
    //           surface <HBs> of <Fprol> . Return intersection parameters in
    //           <FprolUV>, <c3dU> and updating <FIop> and <CPop>
    //           <HGs> is a surface of ChFi
    //           <Fop> is a face having 2 edges at corner with OnSame state
    //           <Fprol> is a face non-adjacent to spine edge
    //           <Vtx> is a corner vertex
    //=======================================================================

    //=======================================================================
    // function : Update
    // purpose  : Calculate the extrema curveonsurf/curveonsurf to prefer
    //           the values concerning the trace on surf and the pcurve on the
    //           face at end.
    //=======================================================================

    //=================================================================================================

    //=======================================================================
    // function : ComputeCurve2d
    // purpose  : calculate the 2d of the curve Ct on face Face
    //=======================================================================

    //=================================================================================================

    //=======================================================================
    // function : ChFi3d_SelectStripe
    // purpose  : find stripe with ChFiDS_OnSame state if <thePrepareOnSame> is True
    //=======================================================================

    //=======================================================================
    // function : PerformOneCorner
    // purpose  : Calculate a corner with three edges and a fillet.
    //           3 separate case: (22/07/94 only 1st is implemented)
    //
    //           - same concavity on three edges, intersection with the
    //             face at end,
    //           - concavity of 2 outgoing edges is opposite to the one of the fillet,
    //             if the face at end is ready for that, the same in  case 1 on extended face,
    //             otherwise a small cap is done with GeomFill,
    //           - only one outgoing edge has concavity opposed to the edge of the
    //             fillet and the third edge, the top of the corner is reread
    //             in the empty of the fillet and closed, either by extending the face
    //             at end if it is plane and orthogonal to the
    //             guiding edge, or by a cap of type GeomFill.
    //
    //           <thePrepareOnSame> means that only needed thing is redefinition
    //           of intersection pameter of OnSame-Stripe with <Arcprol>
    //           (eap, Arp 9 2002, occ266)
    //=======================================================================

    private void PerformOneCorner(int Index, bool thePrepareOnSame = false)
    {
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();

        // the top,
        Vertex Vtx = myVDataMap.FindKey(Index);
        // The fillet is returned,
        NCollection_List<ChFiDS_Stripe>.Iterator StrIt = new();
        StrIt.Initialize(myVDataMap[Index]);
        if (!ChFi3d_SelectStripe(StrIt, Vtx, thePrepareOnSame))
        {
            return;
        }
        ChFiDS_Stripe? stripe = StrIt.Value();
        ChFiDS_Spine? spine = stripe.Spine();
        NCollection_Sequence<ChFiDS_SurfData> SeqFil = 
            stripe.ChangeSetOfSurfData();
        // SurfData and its CommonPoints,
        int sens = 0;

        // Choose proper SurfData
        int num = ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
        bool isfirst = (sens == 1);
        if (isfirst)
        {
            for (; num < SeqFil.Length()
                          && ((SeqFil.Value(num).IndexOfS1() == 0) || (SeqFil.Value(num).IndexOfS2() == 0));)
            {
                SeqFil.Remove(num); // The surplus is removed
            }
        }
        else
        {
            for (; num > 1
                          && ((SeqFil.Value(num).IndexOfS1() == 0) || (SeqFil.Value(num).IndexOfS2() == 0));)
            {
                SeqFil.Remove(num); // The surplus is removed
                num--;
            }
        }

        ChFiDS_SurfData? Fd = SeqFil.ChangeValue(num);
        ref ChFiDS_CommonPoint CV1 = ref Fd.ChangeVertex(isfirst, 1);
        ref ChFiDS_CommonPoint CV2 = ref Fd.ChangeVertex(isfirst, 2);
        // To evaluate the new points.
        Box box1 = new(), box2 = new();

        // The cases of cap and intersection are processed separately.
        // ----------------------------------------------------------
        ChFiDS_State stat = default;
        if (isfirst)
        {
            stat = spine.FirstStatus();
        }
        else
        {
            stat = spine.LastStatus();
        }
        bool onsame = (stat == ChFiDS_OnSame);
        Face Fv = new(), Fad = new(), Fop = new();
        Edge Arcpiv = new(), Arcprol = new(), Arcspine = new();
        if (isfirst)
        {
            Arcspine = spine.Edges(1);
        }
        else
        {
            Arcspine = spine.Edges(spine.NbEdges());
        }
        Orientation OArcprolv = Forward, OArcprolop = Forward;
        int ICurve = 0;
        FaceSurfaceAdaptor? HBs = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? HBad = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? HBop = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor Bs = HBs;
        FaceSurfaceAdaptor Bad = HBad;
        FaceSurfaceAdaptor Bop = HBop;
        Curve3D? Cc = null;
        Curve2D? Pc = null, Ps = null;
        double Ubid = 0.0, Vbid = 0.0;//,mu,Mu,mv,Mv;
        double Udeb = 0.0, Ufin = 0.0;
        //  gp_Pnt2d UVf1,UVl1,UVf2,UVl2;
        //  double Du,Dv,Step;
        bool inters = true;
        int IFadArc = 1, IFopArc = 2;
        Fop = ShapeCasts.Face(DStr.Shape(Fd.Index(IFopArc)));
        Explorer ex = new();


        if (onsame)
        {
            if (!CV1.IsOnArc() && !CV2.IsOnArc())
            {
                throw new ConstructionErrorException("Corner OnSame : no point on arc");
            }
            else if (CV1.IsOnArc() && CV2.IsOnArc())
            {
                bool sur1 = false, sur2 = false;
                for (ex.Init(CV1.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (Vtx.IsSame(ex.Current()))
                    {
                        sur1 = true;
                        break;
                    }
                }
                for (ex.Init(CV2.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (Vtx.IsSame(ex.Current()))
                    {
                        sur2 = true;
                        break;
                    }
                }
                if (sur1 && sur2)
                {
                    Edge[] E = [new(), new(), new()];
                    E[0] = CV1.Arc();
                    E[1] = CV2.Arc();
                    E[2] = Arcspine;
                    if (ChFi3d_EdgeState(E, myEFMap) != ChFiDS_OnDiff)
                    {
                        IFadArc = 2;
                    }
                }
                else if (sur2)
                {
                    IFadArc = 2;
                }
            }
            else if (CV2.IsOnArc())
            {
                IFadArc = 2;
            }
            IFopArc = 3 - IFadArc;

            Arcpiv = Fd.Vertex(isfirst, IFadArc).Arc();
            Fad = ShapeCasts.Face(DStr.Shape(Fd.Index(IFadArc)));
            Fop = ShapeCasts.Face(DStr.Shape(Fd.Index(IFopArc)));
            NCollection_List<Shape>.Iterator It = new();
            // The face at end is returned without check of its unicity.
            for (It.Initialize(myEFMap[Arcpiv]); It.More(); It.Next())
            {
                if (!Fad.IsSame(It.Value()))
                {
                    Fv = ShapeCasts.Face(It.Value());
                    break;
                }
            }

            // Does the face at bout contain the Vertex ?
            bool isinface = false;
            for (ex.Init(Fv, ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(Vtx))
                {
                    isinface = true;
                    break;
                }
            }
            if (!isinface && Fd.Vertex(isfirst, 3 - IFadArc).IsOnArc())
            {
                IFadArc = 3 - IFadArc;
                IFopArc = 3 - IFopArc;
                Arcpiv = Fd.Vertex(isfirst, IFadArc).Arc();
                Fad = ShapeCasts.Face(DStr.Shape(Fd.Index(IFadArc)));
                Fop = ShapeCasts.Face(DStr.Shape(Fd.Index(IFopArc)));
                // NCollection_List<TopoDS_Shape>::Iterator It;
                // The face at end is returned without check of its unicity.
                for (It.Initialize(myEFMap[Arcpiv]); It.More(); It.Next())
                {
                    if (!Fad.IsSame(It.Value()))
                    {
                        Fv = ShapeCasts.Face(It.Value());
                        break;
                    }
                }
            }

            if (Fv.IsNull)
            {
                throw new NotDoneException("OneCorner : face at end not found");
            }

            Fv.Orientation(Forward);
            Fad.Orientation(Forward);
            Fop.Orientation(Forward);

            // The edge that will be extended is returned.
            for (It.Initialize(myVEMap[Vtx]); It.More() && Arcprol.IsNull; It.Next())
            {
                if (!Arcpiv.IsSame(It.Value()))
                {
                    for (ex.Init(Fv, ShapeType.Edge); ex.More(); ex.Next())
                    {
                        if (It.Value().IsSame(ex.Current()))
                        {
                            Arcprol = ShapeCasts.Edge(It.Value());
                            OArcprolv = ex.Current().Orientation();
                            break;
                        }
                    }
                }
            }
            if (Arcprol.IsNull) /*throw new StdFail_NotDone("OneCorner : edge a prolonger non trouve");*/
            {
                PerformIntersectionAtEnd(Index);
                return;
            }
            for (ex.Init(Fop, ShapeType.Edge); ex.More(); ex.Next())
            {
                if (Arcprol.IsSame(ex.Current()))
                {
                    OArcprolop = ex.Current().Orientation();
                    break;
                }
            }
            Face FFv = new();
            double tol = 0.0;
            int prol = 0;
            BrepBuilder BRE = new();
            Surface? Sface = null;
            Sface = Tool.Surface(Fv);
            ChFi3d_ExtendSurface(ref Sface, ref prol);
            tol = Tool.Tolerance(Fv);
            BRE.MakeFace(ref FFv, Sface, tol);
            if (prol != 0)

            {
                Bs.Initialize(FFv, false);
                DStr.SetNewSurface(Fv, Sface);
            }
            else
            {
                Bs.Initialize(Fv, false);
            }
            Bad.Initialize(Fad);
            Bop.Initialize(Fop);
        }
        // in case of OnSame it is necessary to modify the CommonPoint
        // in the empty and its parameter in the FaceInterference.
        // They are both returned in non const references. Attention the modifications are done behind
        // de CV1,CV2,Fi1,Fi2.
        ref ChFiDS_CommonPoint CPopArc = ref Fd.ChangeVertex(isfirst, IFopArc);
        ref ChFiDS_FaceInterference FiopArc = ref Fd.ChangeInterference(IFopArc);
        ref ChFiDS_CommonPoint CPadArc = ref Fd.ChangeVertex(isfirst, IFadArc);
        ref ChFiDS_FaceInterference FiadArc = ref Fd.ChangeInterference(IFadArc);
        // the parameter of the vertex in the air is initialiced with the value of
        // its opposite (point on arc).
        double wop = Fd.ChangeInterference(IFadArc).Parameter(isfirst);
        Curve3D? c3df = null;
        SurfaceAdaptor? HGs = 
            new SurfaceAdaptor(DStr.Surface(Fd.Surf()).Surface());
        Point2D p2dbout = new();

        if (onsame)
        {

            ChFiDS_CommonPoint saveCPopArc = CPopArc;
            c3df = DStr.Curve(FiopArc.LineIndex()).Curve();

            inters = IntersUpdateOnSame(HGs, HBs, c3df, Fop, Fv, Arcprol, Vtx, isfirst,
                                                                    10 * tolapp3d, // in
                                                                    ref FiopArc, ref CPopArc, ref p2dbout,
                                                                    ref wop); // out

            EdgeCurve2DAdaptor? pced = new EdgeCurve2DAdaptor();
            pced.Initialize(CPadArc.Arc(), Fv);
            // in the case of degenerated Fi, parameter difference can be even negative (eap, occ293)
            if ((FiadArc.LastParameter() - FiadArc.FirstParameter()) > 10 * tolesp)
            {
                Update(HBs, pced, HGs, ref FiadArc, ref CPadArc, isfirst);
            }

            if (thePrepareOnSame)
            {
                // saveCPopArc.SetParameter(wop);
                saveCPopArc.SetPoint(CPopArc.Point());
                CPopArc = saveCPopArc;
                return;
            }
        }
        else
        {
            inters = FindFace(Vtx, CV1, CV2, ref Fv, Fop);
            if (!inters)
            {
                PerformIntersectionAtEnd(Index);
                return;
            }
            Bs.Initialize(Fv);
            EdgeCurve2DAdaptor? pced = new EdgeCurve2DAdaptor();
            pced.Initialize(CV1.Arc(), Fv);
            Update(HBs, pced, HGs, ref Fd.ChangeInterferenceOnS1(), ref CV1, isfirst);
            pced.Initialize(CV2.Arc(), Fv);
            Update(HBs, pced, HGs, ref Fd.ChangeInterferenceOnS2(), ref CV2, isfirst);
        }


        Edge edgecouture = new();
        bool couture = false, intcouture = false;
        double tolreached = tolapp3d;
        double par1 = 0.0, par2 = 0.0;
        int indpt = 0, Icurv1 = 0, Icurv2 = 0;
        TrimmedCurve3D? curv1 = null, curv2 = null;
        Curve2D? c2d1 = null, c2d2 = null;

        int Isurf = Fd.Surf();

        if (inters)
        {
            HGs = ChFi3d_BoundSurf(DStr, Fd, 1, 2);
            ChFiDS_FaceInterference Fi1 = Fd.InterferenceOnS1();
            ChFiDS_FaceInterference Fi2 = Fd.InterferenceOnS2();
            NCollection_Array1<double> Pardeb = new(1, 4), Parfin = new(1, 4);
            Point2D pfil1 = new(), pfac1 = new(), pfil2 = new(), pfac2 = new();
            Curve2D? Hc1 = null, Hc2 = null;
            if (onsame && IFopArc == 1)
            {
                pfac1 = p2dbout;
            }
            else
            {
                Hc1 = Tool.CurveOnSurface(CV1.Arc(), Fv, out Ubid, out Ubid);
                if (Hc1 is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                pfac1 = Hc1.Value(CV1.ParameterOnArc());
            }
            if (onsame && IFopArc == 2)
            {
                pfac2 = p2dbout;
            }
            else
            {
                Hc2 = Tool.CurveOnSurface(CV2.Arc(), Fv, out Ubid, out Ubid);
                if (Hc2 is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                pfac2 = Hc2.Value(CV2.ParameterOnArc());
            }
            if (Fi1.LineIndex() != 0)
            {
                pfil1 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(isfirst));
            }
            else
            {
                pfil1 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(!isfirst));
            }
            if (Fi2.LineIndex() != 0)
            {
                pfil2 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(isfirst));
            }
            else
            {
                pfil2 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(!isfirst));
            }
            if (onsame)
            {
                ChFi3d_Recale(Bs, ref pfac1, ref pfac2, (IFadArc == 1));
            }

            Pardeb[1] = pfil1.X();
            Pardeb[2] = pfil1.Y();
            Pardeb[3] = pfac1.X();
            Pardeb[4] = pfac1.Y();
            Parfin[1] = pfil2.X();
            Parfin[2] = pfil2.Y();
            Parfin[3] = pfac2.X();
            Parfin[4] = pfac2.Y();

            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pfac1, pfac2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(Bs, uu1, uu2, vv1, vv2);

            if (!ChFi3d_ComputeCurves(HGs, HBs, Pardeb, Parfin, ref Cc, ref Ps, ref Pc, tolapp3d, tol2d, out tolreached))
            {
                throw new FailureException("OneCorner : echec calcul intersection");
            }

            Udeb = Cc.FirstParameter();
            Ufin = Cc.LastParameter();

            //  determine if the curve has an intersection with edge of sewing

            ChFi3d_Couture(Fv, out couture, ref edgecouture);

            if (couture && !Tool.Degenerated(edgecouture))
            {

                // double Ubid,Vbid;
                Curve3D? C = Tool.Curve(edgecouture, out Ubid, out Vbid);
                TrimmedCurve3D? Ctrim = new TrimmedCurve3D(C, Ubid, Vbid);
                CurveAdaptor cur1 = new(Ctrim.BasisCurve());
                CurveAdaptor cur2 = new(Cc);
                ExtCC extCC = new(cur1, cur2);
                if (extCC.IsDone && extCC.ExtCount != 0)
                {
                    int imin = 0;
                    double distmin2 = RealLast();
                    for (int i = 1; i <= extCC.ExtCount; i++)
                    {
                        if (extCC.SquareDistance(i) < distmin2)
                        {
                            distmin2 = extCC.SquareDistance(i);
                            imin = i;
                        }
                    }
                    if (distmin2 <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        POnCurv ponc1 = new(), ponc2 = new();
                        extCC.Points(imin, out ponc1, out ponc2);
                        par1 = ponc1.Parameter();
                        par2 = ponc2.Parameter();
                        double Tol = 1.0e-4;
                        if (Math.Abs(par2 - Udeb) > Tol && Math.Abs(Ufin - par2) > Tol)
                        {
                            Point3D P1 = ponc1.Value();
                            TopOpeBRepDS_Point tpoint = new(P1, Tol);
                            indpt = DStr.AddPoint(tpoint);
                            intcouture = true;
                            curv1 = new TrimmedCurve3D(Cc, Udeb, par2);
                            curv2 = new TrimmedCurve3D(Cc, par2, Ufin);
                            TopOpeBRepDS_Curve tcurv1 = new(curv1, tolreached);
                            TopOpeBRepDS_Curve tcurv2 = new(curv2, tolreached);
                            Icurv1 = DStr.AddCurve(tcurv1);
                            Icurv2 = DStr.AddCurve(tcurv2);
                        }
                    }
                }
            }
        }
        else
        { // (!inters)
            throw new MethodNotImplementedException("OneCorner : bouchon non ecrit");
        }
        int IShape = DStr.AddShape(Fv);
        Orientation Et = Forward;
        if (IFadArc == 1)
        {
            Explorer Exp = new();
            for (Exp.Init(Fv.Oriented(Forward), ShapeType.Edge); Exp.More(); Exp.Next())
            {
                if (Exp.Current().IsSame(CV1.Arc()))
                {
                    Et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(Exp.Current().Orientation(), CV1.TransitionOnArc()));
                    break;
                }
            }
        }
        else
        {
            Explorer Exp = new();
            for (Exp.Init(Fv.Oriented(Forward), ShapeType.Edge); Exp.More(); Exp.Next())
            {
                if (Exp.Current().IsSame(CV2.Arc()))
                {
                    Et = global::Kerf.Engine.Topology.TopAbs.Compose(Exp.Current().Orientation(), CV2.TransitionOnArc());
                    break;
                }
            }

            //
        }


        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(CV1, DStr), isfirst, 1);
        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(CV2, DStr), isfirst, 2);

        if (!intcouture)
        {
            // there is no intersection with the sewing edge
            // the curve Cc is stored in the stripe
            // the storage in the DS is not done by FILDS.

            TopOpeBRepDS_Curve Tc = new(Cc, tolreached);
            ICurve = DStr.AddCurve(Tc);
            TopOpeBRepDS_SurfaceCurveInterference? Interfc = 
                ChFi3d_FilCurveInDS(ICurve, IShape, Pc, Et);

            // 31/01/02 akm vvv : (OCC119) Prevent the builder from creating
            //                    intersecting fillets - they are bad.
            Geom2dInt_GInter anIntersector = new();
            CurveAdaptor2D aCorkPCurve = new(Pc, Udeb, Ufin);

            // Take all the interferences with faces from all the stripes
            // and look if their pcurves intersect our cork pcurve.
            // Unfortunately, by this moment they do not exist in DStr.
            NCollection_List<ChFiDS_Stripe>.Iterator aStrIt = new(myListStripe);
            for (; aStrIt.More(); aStrIt.Next())
            {
                ChFiDS_Stripe? aCheckStripe = aStrIt.Value();
                NCollection_HSequence<ChFiDS_SurfData>? aSeqData = 
                    aCheckStripe.SetOfSurfData();
                // Loop on parts of the stripe
                int iPart = 0;
                for (iPart = 1; iPart <= aSeqData.Length(); iPart++)
                {
                    ChFiDS_SurfData? aData = aSeqData.Value(iPart);
                    CurveAdaptor2D anOtherPCurve = new();
                    if (IShape == aData.IndexOfS1())
                    {
                        Curve2D? aPCurve = aData.InterferenceOnS1().PCurveOnFace();
                        if (aPCurve is null)
                        {
                            continue;
                        }

                        anOtherPCurve.Load(aPCurve, aData.InterferenceOnS1().FirstParameter(),
                                                              aData.InterferenceOnS1().LastParameter());
                    }
                    else if (IShape == aData.IndexOfS2())
                    {
                        Curve2D? aPCurve = aData.InterferenceOnS2().PCurveOnFace();
                        if (aPCurve is null)
                        {
                            continue;
                        }

                        anOtherPCurve.Load(aPCurve, aData.InterferenceOnS2().FirstParameter(),
                                                              aData.InterferenceOnS2().LastParameter());
                    }
                    else
                    {
                        // Normal case - no common surface
                        continue;
                    }
                    if (IsEqual(anOtherPCurve.LastParameter(), anOtherPCurve.FirstParameter()))
                    {
                        // Degenerates
                        continue;
                    }
                    anIntersector.Perform(aCorkPCurve, anOtherPCurve, tol2d, global::Kerf.Engine.Foundation.Precision.PConfusion());
                    if (anIntersector.NbSegments() > 0 || anIntersector.NbPoints() > 0)
                    {
                        throw new NotDoneException("OneCorner : fillets have too big radiuses");
                    }
                }
            }
            NCollection_List<TopOpeBRepDS_Interference>.Iterator anIter = new(DStr.ChangeShapeInterferences(IShape));
            for (; anIter.More(); anIter.Next())
            {
                TopOpeBRepDS_SurfaceCurveInterference? anOtherIntrf = 
                    (anIter.Value() as TopOpeBRepDS_SurfaceCurveInterference);
                // We need only interferences between cork face and curves
                // of intersection with another fillet surfaces
                if (anOtherIntrf is null)
                {
                    continue;
                }
                // Look if there is an intersection between pcurves
                TrimmedCurve3D? anOtherCur = 
                    (DStr.Curve(anOtherIntrf.Geometry()).Curve() as TrimmedCurve3D);
                if (anOtherCur is null)
                {
                    continue;
                }
                // OCCT's anOtherPCurve, renamed: C# cannot redeclare it in this scope
                CurveAdaptor2D anOtherPCurve2 = new(anOtherIntrf.PCurve(), anOtherCur.FirstParameter(),
                                                        anOtherCur.LastParameter());
                anIntersector.Perform(aCorkPCurve, anOtherPCurve2, tol2d, global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (anIntersector.NbSegments() > 0 || anIntersector.NbPoints() > 0)
                {
                    throw new NotDoneException("OneCorner : fillets have too big radiuses");
                }
            }
            // 31/01/02 akm ^^^
            DStr.ChangeShapeInterferences(IShape).Append(Interfc);
            //// modified by jgv, 26.03.02 for OCC32 ////
            ChFiDS_CommonPoint[] CV = new ChFiDS_CommonPoint[2];
            CV[0] = CV1;
            CV[1] = CV2;
            for (int i = 0; i < 2; i++)
            {
                if (CV[i].IsOnArc() && ChFi3d_IsPseudoSeam(CV[i].Arc(), Fv))
                {
                    Point2D pfac1 = new(), PcF = new(), PcL = new();
                    Vector2D DerPc = new(), DerHc = new();
                    double first = 0.0, last = 0.0, prm1 = 0.0, prm2 = 0.0;
                    bool onfirst = false, FirstToPar = false;
                    Curve2D? Hc = Tool.CurveOnSurface(CV[i].Arc(), Fv, out first, out last);
                    if (Hc is null)
                    {
                        throw new ConstructionErrorException("Failed to get p-curve of edge");
                    }
                    pfac1 = Hc.Value(CV[i].ParameterOnArc());
                    PcF = Pc.Value(Udeb);
                    PcL = Pc.Value(Ufin);
                    onfirst = pfac1.Distance(PcF) < pfac1.Distance(PcL);
                    if (onfirst)
                    {
                        Pc.D1(Udeb, out PcF, out DerPc);
                    }
                    else
                    {
                        Pc.D1(Ufin, out PcL, out DerPc);
                        DerPc.Reverse();
                    }
                    Hc.D1(CV[i].ParameterOnArc(), out pfac1, out DerHc);

                    if (DerHc.Dot(DerPc) > 0.0)
                    {
                        prm1 = CV[i].ParameterOnArc();
                        prm2 = last;
                        FirstToPar = false;
                    }
                    else
                    {
                        prm1 = first;
                        prm2 = CV[i].ParameterOnArc();
                        FirstToPar = true;
                    }
                    Curve3D? Ct = Tool.Curve(CV[i].Arc(), out first, out last);
                    Ct = new TrimmedCurve3D(Ct, prm1, prm2);
                    double toled = Tool.Tolerance(CV[i].Arc());
                    TopOpeBRepDS_Curve tcurv = new(Ct, toled);
                    TopOpeBRepDS_CurvePointInterference? Interfp1 = null, Interfp2 = null;
                    int indcurv = 0;
                    indcurv = DStr.AddCurve(tcurv);
                    int indpoint = (isfirst) ? stripe.IndexFirstPointOnS1() : stripe.IndexLastPointOnS1();
                    int indvertex = DStr.AddShape(Vtx);
                    if (FirstToPar)
                    {
                        Interfp1 = ChFi3d_FilPointInDS(Forward, indcurv, indvertex, prm1, true);
                        Interfp2 = ChFi3d_FilPointInDS(Reversed, indcurv, indpoint, prm2, false);
                    }
                    else
                    {
                        Interfp1 = ChFi3d_FilPointInDS(Forward, indcurv, indpoint, prm1, false);
                        Interfp2 = ChFi3d_FilPointInDS(Reversed, indcurv, indvertex, prm2, true);
                    }
                    DStr.ChangeCurveInterferences(indcurv).Append(Interfp1);
                    DStr.ChangeCurveInterferences(indcurv).Append(Interfp2);
                    int indface = DStr.AddShape(Fv);
                    Interfc = ChFi3d_FilCurveInDS(indcurv, indface, Hc, CV[i].Arc().Orientation());
                    DStr.ChangeShapeInterferences(indface).Append(Interfc);
                    Edge aLocalEdge = CV[i].Arc();
                    aLocalEdge.Reverse();
                    Curve2D? HcR = Tool.CurveOnSurface(aLocalEdge, Fv, out first, out last);
                    if (HcR is null)
                    {
                        throw new ConstructionErrorException("Failed to get p-curve of edge");
                    }
                    Interfc = ChFi3d_FilCurveInDS(indcurv, indface, HcR, aLocalEdge.Orientation());
                    DStr.ChangeShapeInterferences(indface).Append(Interfc);
                    // modify degenerated edge
                    bool DegenExist = false;
                    Edge Edeg = new();
                    Explorer Explo = new(Fv, ShapeType.Edge);
                    for (; Explo.More(); Explo.Next())
                    {
                        Edge Ecur = ShapeCasts.Edge(Explo.Current());
                        if (Tool.Degenerated(Ecur))
                        {
                            Vertex Vf = new(), Vl = new();
                            Exploration.Vertices(Ecur, out Vf, out Vl);
                            if (Vf.IsSame(Vtx) || Vl.IsSame(Vtx))
                            {
                                DegenExist = true;
                                Edeg = Ecur;
                                break;
                            }
                        }
                    }
                    if (DegenExist)
                    {
                        double fd = 0.0, ld = 0.0;
                        Curve2D? Cd = Tool.CurveOnSurface(Edeg, Fv, out fd, out ld);
                        if (Cd is null)
                        {
                            throw new ConstructionErrorException("Failed to get p-curve of edge");
                        }
                        TrimmedCurve2D? tCd = (Cd as TrimmedCurve2D);
                        if (tCd is not null)
                        {
                            Cd = tCd.BasisCurve();
                        }
                        Point2D P2d = (FirstToPar) ? Hc.Value(first) : Hc.Value(last);
                        Geom2dAPI_ProjectPointOnCurve Projector = new(P2d, Cd);
                        double par = Projector.LowerDistanceParameter();
                        int Ideg = DStr.AddShape(Edeg);
                        // clang-format off
                        Orientation ori = (par < fd)? Forward : Reversed;//if par<fd => par>ld
                        // clang-format on
                        Interfp1 = ChFi3d_FilPointInDS(ori, Ideg, indvertex, par, true);
                        DStr.ChangeShapeInterferences(Ideg).Append(Interfp1);
                    }
                }
            }
            /////////////////////////////////////////////
            stripe.ChangePCurve(isfirst) = Ps;
            stripe.SetCurve(ICurve, isfirst);
            stripe.SetParameters(isfirst, Udeb, Ufin);
        }
        else
        {
            // curves curv1 are curv2 stored in the DS
            // these curves will not be reconstructed by FILDS as
            // one places stripe->InDS(isfirst);

            // interferences of curv1 and curv2 on Fv
            ComputeCurve2d(curv1, ref Fv, ref c2d1);
            TopOpeBRepDS_SurfaceCurveInterference? InterFv = null;
            InterFv = ChFi3d_FilCurveInDS(Icurv1, IShape, c2d1, Et);
            DStr.ChangeShapeInterferences(IShape).Append(InterFv);
            ComputeCurve2d(curv2, ref Fv, ref c2d2);
            InterFv = ChFi3d_FilCurveInDS(Icurv2, IShape, c2d2, Et);
            DStr.ChangeShapeInterferences(IShape).Append(InterFv);
            // interferences of curv1 and curv2 on Isurf
            if (Fd.Orientation() == Fv.Orientation())
            {
                Et = global::Kerf.Engine.Topology.TopAbs.Reverse(Et);
            }
            c2d1 = new TrimmedCurve2D(Ps, Udeb, par2);
            InterFv = ChFi3d_FilCurveInDS(Icurv1, Isurf, c2d1, Et);
            DStr.ChangeSurfaceInterferences(Isurf).Append(InterFv);
            c2d2 = new TrimmedCurve2D(Ps, par2, Ufin);
            InterFv = ChFi3d_FilCurveInDS(Icurv2, Isurf, c2d2, Et);
            DStr.ChangeSurfaceInterferences(Isurf).Append(InterFv);

            // limitation of the sewing edge
            int Iarc = DStr.AddShape(edgecouture);
            TopOpeBRepDS_CurvePointInterference? Interfedge = null;
            Orientation ori = default;
            Vertex Vdeb = new(), Vfin = new();
            Vdeb = Exploration.FirstVertex(edgecouture);
            Vfin = Exploration.LastVertex(edgecouture);
            double pard = 0.0, parf = 0.0;
            pard = Tool.Parameter(Vdeb, edgecouture);
            parf = Tool.Parameter(Vfin, edgecouture);
            if (Math.Abs(par1 - pard) < Math.Abs(parf - par1))
            {
                ori = Forward;
            }
            else
            {
                ori = Reversed;
            }
            Interfedge = ChFi3d_FilPointInDS(ori, Iarc, indpt, par1);
            DStr.ChangeShapeInterferences(Iarc).Append(Interfedge);

            // creation of CurveInterferences from Icurv1 and Icurv2
            stripe.InDS(isfirst);
            int ind1 = stripe.IndexPoint(isfirst, 1);
            int ind2 = stripe.IndexPoint(isfirst, 2);
            TopOpeBRepDS_CurvePointInterference? interfprol = 
                ChFi3d_FilPointInDS(Forward, Icurv1, ind1, Udeb);
            DStr.ChangeCurveInterferences(Icurv1).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Reversed, Icurv1, indpt, par2);
            DStr.ChangeCurveInterferences(Icurv1).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Forward, Icurv2, indpt, par2);
            DStr.ChangeCurveInterferences(Icurv2).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Reversed, Icurv2, ind2, Ufin);
            DStr.ChangeCurveInterferences(Icurv2).Append(interfprol);
        }

        ChFi3d_EnlargeBox(HBs, Pc, Udeb, Ufin, ref box1, ref box2);

        if (onsame && inters)
        {
    // VARIANT 1:
    // A small missing end of curve is added for the extension
    // of the face at end and the limitation of the opposing face.

    //   VARIANT 2 : extend Arcprol, not create new small edge
    //   To do: modify for intcouture
    // (OCCT defines VARIANT1: only its branches are ported)

            // First of all the points are cut with the edge of the spine.
            int IArcspine = DStr.AddShape(Arcspine);
            int IVtx = DStr.AddShape(Vtx);
            Orientation OVtx = Forward;
            for (ex.Init(Arcspine.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (Vtx.IsSame(ex.Current()))
                {
                    OVtx = ex.Current().Orientation();
                    break;
                }
            }
            OVtx = global::Kerf.Engine.Topology.TopAbs.Reverse(OVtx);
            double parVtx = Tool.Parameter(Vtx, Arcspine);
            TopOpeBRepDS_CurvePointInterference? interfv = 
                ChFi3d_FilVertexInDS(OVtx, IArcspine, IVtx, parVtx);
            DStr.ChangeShapeInterferences(IArcspine).Append(interfv);

            // Now the missing curves are constructed.
            Vertex V2 = new();
            for (ex.Init(Arcprol.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (Vtx.IsSame(ex.Current()))
                {
                    OVtx = ex.Current().Orientation();
                }
                else
                {
                    V2 = ShapeCasts.Vertex(ex.Current());
                }
            }

            Curve2D? Hc = null;
            parVtx = Tool.Parameter(Vtx, Arcprol); // VARIANT1
            ChFiDS_FaceInterference Fiop = Fd.Interference(IFopArc);
            Point2D pop1 = new(), pop2 = new(), pv1 = new(), pv2 = new();
            Hc = Tool.CurveOnSurface(Arcprol, Fop, out Ubid, out Ubid);
            if (Hc is null)
            {
                throw new ConstructionErrorException("Failed to get p-curve of edge");
            }
            pop1 = Hc.Value(parVtx);
            pop2 = Fiop.PCurveOnFace().Value(Fiop.Parameter(isfirst));
            Hc = Tool.CurveOnSurface(Arcprol, Fv, out Ubid, out Ubid);
            if (Hc is null)
            {
                throw new ConstructionErrorException("Failed to get p-curve of edge");
            }
            pv1 = Hc.Value(parVtx);
            pv2 = p2dbout;
            ChFi3d_Recale(Bs, ref pv1, ref pv2, true);
            NCollection_Array1<double> Pardeb = new(1, 4), Parfin = new(1, 4);
            Pardeb[1] = pop1.X();
            Pardeb[2] = pop1.Y();
            Pardeb[3] = pv1.X();
            Pardeb[4] = pv1.Y();
            Parfin[1] = pop2.X();
            Parfin[2] = pop2.Y();
            Parfin[3] = pv2.X();
            Parfin[4] = pv2.Y();
            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pv1, pv2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(Bs, uu1, uu2, vv1, vv2);
            ChFi3d_Boite(pop1, pop2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(Bop, uu1, uu2, vv1, vv2);

            Curve3D? zob3d = null;
            Curve2D? zob2dop = null, zob2dv = null;
            // double tolreached;
            if (!ChFi3d_ComputeCurves(HBop, HBs, Pardeb, Parfin, ref zob3d, ref zob2dop, ref zob2dv, tolapp3d, tol2d,
                                                                out tolreached))
            {
                throw new FailureException("OneCorner : echec calcul intersection");
            }

            Udeb = zob3d.FirstParameter();
            Ufin = zob3d.LastParameter();
            TopOpeBRepDS_Curve Zob = new(zob3d, tolreached);
            int IZob = DStr.AddCurve(Zob);

            // it is determined if Fop has an edge of sewing
            // it is determined if the curve has an intersection with the edge of sewing

            // TopoDS_Edge edgecouture;
            // bool couture;
            ChFi3d_Couture(Fop, out couture, ref edgecouture);

            if (couture && !Tool.Degenerated(edgecouture))
            {
                BRepLib_MakeEdge Bedge = new(zob3d);
                Edge edg = Bedge.Edge();
                BRepExtrema_ExtCC extCC = new(edgecouture, edg);
                if (extCC.IsDone() && extCC.NbExt() != 0)
                {
                    for (int i = 1; i <= extCC.NbExt() && !intcouture; i++)
                    {
                        if (extCC.SquareDistance(i) <= 1.0e-8)
                        {
                            par1 = extCC.ParameterOnE1(i);
                            par2 = extCC.ParameterOnE2(i);
                            Point3D P1 = extCC.PointOnE1(i);
                            TopOpeBRepDS_Point tpoint = new(P1, 1.0e-4);
                            indpt = DStr.AddPoint(tpoint);
                            intcouture = true;
                            curv1 = new TrimmedCurve3D(zob3d, Udeb, par2);
                            curv2 = new TrimmedCurve3D(zob3d, par2, Ufin);
                            TopOpeBRepDS_Curve tcurv1 = new(curv1, tolreached);
                            TopOpeBRepDS_Curve tcurv2 = new(curv2, tolreached);
                            Icurv1 = DStr.AddCurve(tcurv1);
                            Icurv2 = DStr.AddCurve(tcurv2);
                        }
                    }
                }
            }
            if (intcouture)
            {

                // interference of curv1 and curv2 on Ishape
                Et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(OVtx, OArcprolv));
                ComputeCurve2d(curv1, ref Fop, ref c2d1);
                TopOpeBRepDS_SurfaceCurveInterference? InterFv = 
                    ChFi3d_FilCurveInDS(Icurv1, IShape, /*zob2dv*/ c2d1, Et);
                DStr.ChangeShapeInterferences(IShape).Append(InterFv);
                ComputeCurve2d(curv2, ref Fop, ref c2d2);
                InterFv = ChFi3d_FilCurveInDS(Icurv2, IShape, /*zob2dv*/ c2d2, Et);
                DStr.ChangeShapeInterferences(IShape).Append(InterFv);

                // limitation of the sewing edge
                int Iarc = DStr.AddShape(edgecouture);
                TopOpeBRepDS_CurvePointInterference? Interfedge = null;
                Orientation ori = default;
                Vertex Vdeb = new(), Vfin = new();
                Vdeb = Exploration.FirstVertex(edgecouture);
                Vfin = Exploration.LastVertex(edgecouture);
                double pard = 0.0, parf = 0.0;
                pard = Tool.Parameter(Vdeb, edgecouture);
                parf = Tool.Parameter(Vfin, edgecouture);
                if (Math.Abs(par1 - pard) < Math.Abs(parf - par1))
                {
                    ori = Reversed;
                }
                else
                {
                    ori = Forward;
                }
                Interfedge = ChFi3d_FilPointInDS(ori, Iarc, indpt, par1);
                DStr.ChangeShapeInterferences(Iarc).Append(Interfedge);

                //  interference of curv1 and curv2 on Iop
                int Iop = DStr.AddShape(Fop);
                Et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(OVtx, OArcprolop));
                TopOpeBRepDS_SurfaceCurveInterference? Interfop = null;
                ComputeCurve2d(curv1, ref Fop, ref c2d1);
                Interfop = ChFi3d_FilCurveInDS(Icurv1, Iop, c2d1, Et);
                DStr.ChangeShapeInterferences(Iop).Append(Interfop);
                ComputeCurve2d(curv2, ref Fop, ref c2d2);
                Interfop = ChFi3d_FilCurveInDS(Icurv2, Iop, c2d2, Et);
                DStr.ChangeShapeInterferences(Iop).Append(Interfop);
                TopOpeBRepDS_CurvePointInterference? interfprol = 
                    ChFi3d_FilVertexInDS(Forward, Icurv1, IVtx, Udeb);
                DStr.ChangeCurveInterferences(Icurv1).Append(interfprol);
                interfprol = ChFi3d_FilPointInDS(Reversed, Icurv1, indpt, par2);
                DStr.ChangeCurveInterferences(Icurv1).Append(interfprol);
                int icc = stripe.IndexPoint(isfirst, IFopArc);
                interfprol = ChFi3d_FilPointInDS(Forward, Icurv2, indpt, par2);
                DStr.ChangeCurveInterferences(Icurv2).Append(interfprol);
                interfprol = ChFi3d_FilPointInDS(Reversed, Icurv2, icc, Ufin);
                DStr.ChangeCurveInterferences(Icurv2).Append(interfprol);
            }
            else
            {
                Et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(OVtx, OArcprolv));
                TopOpeBRepDS_SurfaceCurveInterference? InterFv = 
                    ChFi3d_FilCurveInDS(IZob, IShape, zob2dv, Et);
                DStr.ChangeShapeInterferences(IShape).Append(InterFv);
                Et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(OVtx, OArcprolop));
                int Iop = DStr.AddShape(Fop);
                TopOpeBRepDS_SurfaceCurveInterference? Interfop = 
                    ChFi3d_FilCurveInDS(IZob, Iop, zob2dop, Et);
                DStr.ChangeShapeInterferences(Iop).Append(Interfop);
                TopOpeBRepDS_CurvePointInterference? interfprol = null;
                interfprol = ChFi3d_FilVertexInDS(Forward, IZob, IVtx, Udeb); // VARIANT1
                DStr.ChangeCurveInterferences(IZob).Append(interfprol);
                int icc = stripe.IndexPoint(isfirst, IFopArc);
                interfprol = ChFi3d_FilPointInDS(Reversed, IZob, icc, Ufin);
                DStr.ChangeCurveInterferences(IZob).Append(interfprol);
                { // VARIANT1
                    if (IFopArc == 1)
                    {
                        box1.Add(zob3d.Value(Ufin));
                    }
                    else
                    {
                        box2.Add(zob3d.Value(Ufin));
                    }
                }
            }
        }
        ChFi3d_EnlargeBox(DStr, stripe, Fd, ref box1, ref box2, isfirst);
        if (CV1.IsOnArc())
        {
            ChFi3d_EnlargeBox(CV1.Arc(), myEFMap[CV1.Arc()], CV1.ParameterOnArc(), ref box1);
        }
        if (CV2.IsOnArc())
        {
            ChFi3d_EnlargeBox(CV2.Arc(), myEFMap[CV2.Arc()], CV2.ParameterOnArc(), ref box2);
        }
        if (!CV1.IsVertex())
        {
            ChFi3d_SetPointTolerance(DStr, box1, stripe.IndexPoint(isfirst, 1));
        }
        if (!CV2.IsVertex())
        {
            ChFi3d_SetPointTolerance(DStr, box2, stripe.IndexPoint(isfirst, 2));
        }

    }

    //=======================================================================
    // function : cherche_face
    // purpose  : find face F belonging to the map, different from faces
    //           F1  F2 F3 and containing edge E
    //=======================================================================

    //=======================================================================
    // function : cherche_edge1
    // purpose  : find common edge between faces F1 and F2
    //=======================================================================

    //=======================================================================
    // function : containV
    // purpose  : return true if vertex V belongs to F1
    //=======================================================================

    //=======================================================================
    // function : containE
    // purpose  : return true if edge E belongs to F1
    //=======================================================================

    //=======================================================================
    // function : IsShrink
    // purpose  : check if U (if <isU>==True) or V of points of <PC> is within
    //           <tol> from <Param>, check points between <Pf> and <Pl>
    //=======================================================================

    //=================================================================================================

    private void PerformIntersectionAtEnd(int Index)
    {

        // intersection at end of fillet with at least two faces
        // process the following cases:
        // - top has n (n>3) adjacent edges
        // - top has 3 edges and fillet on one of edges touches
        //   more than one face


        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        int nn = 15;
        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        It.Initialize(myVDataMap[Index]);
        ChFiDS_Stripe? stripe = It.Value();
        ChFiDS_Spine? spine = stripe.Spine();
        NCollection_Sequence<ChFiDS_SurfData> SeqFil = 
            stripe.ChangeSetOfSurfData();
        Vertex Vtx = myVDataMap.FindKey(Index);
        int sens = 0, num, num1;
        bool couture = false, isfirst;
        // int sense;
        Edge edgelibre1 = new(), edgelibre2 = new(), EdgeSpine = new();
        bool bordlibre = false;
        // determine the number of faces and edges
        NCollection_Array1<Shape> tabedg = new(0, nn);
        Face F1 = new(), F2 = new();
        int nface = ChFi3d_nbface(myVFMap[Vtx]);
        NCollection_List<Shape>.Iterator ItF = new();
        int nbarete = 0;
        nbarete = ChFi3d_NbNotDegeneratedEdges(Vtx, myVEMap);
        ChFi3d_ChercheBordsLibres(myVEMap, Vtx, out bordlibre, ref edgelibre1, ref edgelibre2);
        if (bordlibre)
        {
            nbarete = (nbarete - 2) / 2 + 2;
        }
        else
        {
            nbarete = nbarete / 2;
        }
        // it is determined if there is an edge of sewing and it face

        Face facecouture = new();
        Edge edgecouture = new();

        bool trouve = false;
        for (ItF.Initialize(myVFMap[Vtx]); ItF.More() && !couture; ItF.Next())
        {
            Face fcur = ShapeCasts.Face(ItF.Value());
            ChFi3d_CoutureOnVertex(fcur, Vtx, out couture, ref edgecouture);
            if (couture)
            {
                facecouture = fcur;
            }
        }
        // it is determined if one of edges adjacent to the fillet is regular
        bool reg1 = false, reg2 = false;
        Edge Ecur = new(), Eadj1 = new(), Eadj2 = new();
        Face Fga = new(), Fdr = new();
        Vertex Vbid1 = new();
        int nbsurf = 0, nbedge = 0;
        reg1 = false;
        reg2 = false;
        nbsurf = SeqFil.Length();
        nbedge = spine.NbEdges();
        num = ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
        isfirst = (sens == 1);
        ChFiDS_State state = default;
        if (isfirst)
        {
            EdgeSpine = spine.Edges(1);
            num1 = num + 1;
            state = spine.FirstStatus();
        }
        else
        {
            EdgeSpine = spine.Edges(nbedge);
            num1 = num - 1;
            state = spine.LastStatus();
        }
        if (nbsurf != nbedge && nbsurf != 1)
        {
            ChFi3d_edge_common_faces(myEFMap[EdgeSpine], out F1, out F2);
            if (F1.IsSame(facecouture))
            {
                Eadj1 = edgecouture;
            }
            else
            {
                ChFi3d_cherche_element(Vtx, EdgeSpine, F1, ref Eadj1, ref Vbid1);
            }
            ChFi3d_edge_common_faces(myEFMap[Eadj1], out Fga, out Fdr);
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:57:32 2001 Begin
            //  reg1=BRep_Tool::Continuity(Eadj1,Fga,Fdr)!=GeomAbs_C0;
            reg1 = ChFi3d.IsTangentFaces(Eadj1, Fga, Fdr);
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:57:33 2001 End
            if (F2.IsSame(facecouture))
            {
                Eadj2 = edgecouture;
            }
            else
            {
                ChFi3d_cherche_element(Vtx, EdgeSpine, F2, ref Eadj2, ref Vbid1);
            }
            ChFi3d_edge_common_faces(myEFMap[Eadj2], out Fga, out Fdr);
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:58:22 2001 Begin
            //  reg2=BRep_Tool::Continuity(Eadj2,Fga,Fdr)!=GeomAbs_C0;
            reg2 = ChFi3d.IsTangentFaces(Eadj2, Fga, Fdr);
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:58:24 2001 End

            // two faces common to the edge are found
            if (reg1 || reg2)
            {
                bool compoint1 = false;
                bool compoint2 = false;
                ChFiDS_CommonPoint cp1 = new(), cp2 = new();
                cp1 = SeqFil[num1].ChangeVertex(isfirst, 1);
                cp2 = SeqFil[num1].ChangeVertex(isfirst, 2);
                if (cp1.IsOnArc())
                {
                    if (cp1.Arc().IsSame(Eadj1) || cp1.Arc().IsSame(Eadj2))
                    {
                        compoint1 = true;
                    }
                }
                if (cp2.IsOnArc())
                {
                    if (cp2.Arc().IsSame(Eadj1) || cp2.Arc().IsSame(Eadj2))
                    {
                        compoint2 = true;
                    }
                }
                if (compoint1 && compoint2)
                {
                    SeqFil.Remove(num);
                    num = ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
                    if (isfirst)
                    {
                        num1 = num + 1;
                    }
                    else
                    {
                        num1 = num - 1;
                    }
                    reg1 = false;
                    reg2 = false;
                }
            }
        }
        // there is only one face at end if FindFace is true and if the face
        // is not the face with sewing edge
        Face face = new();
        ChFiDS_SurfData? Fd = SeqFil.ChangeValue(num);
        ref ChFiDS_CommonPoint CV1 = ref Fd.ChangeVertex(isfirst, 1);
        ref ChFiDS_CommonPoint CV2 = ref Fd.ChangeVertex(isfirst, 2);
        bool onecorner = false;
        if (FindFace(Vtx, CV1, CV2, ref face))
        {
            if (!couture)
            {
                onecorner = true;
            }
            else if (!face.IsSame(facecouture))
            {
                onecorner = true;
            }
        }
        if (onecorner)
        {
            if (MoreSurfdata(Index))
            {
                PerformMoreSurfdata(Index);
                return;
            }
        }
        if (!onecorner && (reg1 || reg2) && !couture && state != ChFiDS_OnSame)
        {
            PerformMoreThreeCorner(Index, 1);
            return;
        }
        SurfaceAdaptor? HGs = ChFi3d_BoundSurf(DStr, Fd, 1, 2);
        ChFiDS_FaceInterference Fi1 = Fd.InterferenceOnS1();
        ChFiDS_FaceInterference Fi2 = Fd.InterferenceOnS2();
        SurfaceAdaptor Gs = HGs;
        FaceSurfaceAdaptor? HBs = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor Bs = HBs;
        Curve3D? Cc = null;
        Curve2D? Pc = null, Ps = null;
        double Ubid = 0.0, Vbid = 0.0;
        Orientation orsurfdata = default;
        orsurfdata = Fd.Orientation();
        int IsurfPrev = 0, Isurf = Fd.Surf();
        ChFiDS_SurfData? SDprev = null;
        if (num1 > 0 && num1 <= SeqFil.Length())
        {
            SDprev = SeqFil[num1];
            IsurfPrev = SDprev.Surf();
        }
        // calculate the orientation of curves at end

        double tolpt = 1.0e-4;
        double tolreached = 0.0;
        Orientation orcourbe = default, orface = default, orien = default;

        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(CV1, DStr), isfirst, 1);
        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(CV2, DStr), isfirst, 2);

        //  gp_Pnt p3d;
        //  gp_Pnt2d p2d;
        double dist = 0.0;
        int Ishape1 = Fd.IndexOfS1();
        Orientation trafil1 = Forward;
        if (Ishape1 != 0)
        {
            if (Ishape1 > 0)
            {
                trafil1 = DStr.Shape(Ishape1).Orientation();
            }
            trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(trafil1, Fd.Orientation());

            trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(Fi1.Transition()), trafil1);
        }
        // eap, Apr 22 2002, occ 293
        //   Fi1.PCurveOnFace()->D0(Fi1.LastParameter(),p2d);
        //   const occ::handle<Geom_Surface> Stemp =
        //     BRep_Tool::Surface(TopoDS::Face(DStr.Shape(Ishape1)));
        //   Stemp ->D0(p2d.X(),p2d.Y(),p3d);
        //   dist=p3d.Distance(CV1.Point());
        //   if (dist<tolpt) orcourbe=trafil1;
        //   else            orcourbe=TopAbs::Reverse(trafil1);
        if (!isfirst)
        {
            orcourbe = trafil1;
        }
        else
        {
            orcourbe = global::Kerf.Engine.Topology.TopAbs.Reverse(trafil1);
        }

        // eap, Apr 22 2002, occ 293
        // variables to show OnSame situation
        bool isOnSame1 = false, isOnSame2 = false;
        // In OnSame situation, the case of degenerated FaceInterference curve
        // is probable when a corner cuts the ChFi3d earlier built on OnSame edge.
        // In such a case, chamfer face can partially shrink to a line and we need
        // to cut off that shrinked part
        // If <isOnSame1>, FaceInterference with F2 can be degenerated
        bool checkShrink = false, isShrink = false, isUShrink = false;
        isShrink = isUShrink = isOnSame1 = isOnSame2 = false;
        double checkShrParam = 0.0, prevSDParam = 0.0;
        Point2D midP2d = new();
        int midIpoint = 0;

        // find Fi1,Fi2 lengths used to extend ChFi surface
        // and by the way define necessity to check shrink
        Point2D P2d1 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(isfirst));
        Point2D P2d2 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(!isfirst));
        Point3D aP1 = new(), aP2 = new();
        HGs.D0(P2d1.X(), P2d1.Y(), out aP1);
        HGs.D0(P2d2.X(), P2d2.Y(), out aP2);
        double Fi1Length = aP1.Distance(aP2);
        //  double eps = Precision::Confusion();
        checkShrink = (Fi1Length <= global::Kerf.Engine.Foundation.Precision.Confusion());

        Point2D P2d3 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(isfirst));
        Point2D P2d4 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(!isfirst));
        HGs.D0(P2d3.X(), P2d3.Y(), out aP1);
        HGs.D0(P2d4.X(), P2d4.Y(), out aP2);
        double Fi2Length = aP1.Distance(aP2);
        checkShrink = checkShrink || (Fi2Length <= global::Kerf.Engine.Foundation.Precision.Confusion());

        if (checkShrink)
        {
            if (Math.Abs(P2d2.Y() - P2d4.Y()) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                isUShrink = false;
                checkShrParam = P2d2.Y();
            }
            else if (Math.Abs(P2d2.X() - P2d4.X()) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                isUShrink = true;
                checkShrParam = P2d2.X();
            }
            else
            {
                checkShrink = false;
            }
        }

        /***********************************************************************/
        //  find faces intersecting with the fillet and edges limiting intersections
        //  nbface is the nb of faces intersected, Face[i] contains the faces
        //  to intersect (i=0.. nbface-1). Edge[i] contains edges limiting
        //  the intersections (i=0 ..nbface)
        /**********************************************************************/

        int nb = 1, nbface;
        Edge E1 = new(), E2 = new(), E = new(), Ei = new(), edgesau = new();
        Edge[] Edge = new Edge[nn];
        Face[] Face = new Face[nn];
        for (int k = 0; k < nn; k++)
        {
            Edge[k] = new();
            Face[k] = new();
        }
        Face facesau = new();
        bool oneintersection1 = false;
        bool oneintersection2 = false;
        Face F = new(), F3 = new();
        Vertex V1 = new(), V2 = new(), V = new(), Vfin = new();
        bool findonf1 = false, findonf2 = false;
        NCollection_List<Shape>.Iterator It3 = new();
        F1 = ShapeCasts.Face(DStr.Shape(Fd.IndexOfS1()));
        F2 = ShapeCasts.Face(DStr.Shape(Fd.IndexOfS2()));
        F3 = F1;
        if (couture || bordlibre)
        {
            nface = nface + 1;
        }
        if (nface == 3)
        {
            nbface = 2;
        }
        else
        {
            nbface = nface - 2;
        }
        if (!CV1.IsOnArc() || !CV2.IsOnArc())
        {
            PerformMoreThreeCorner(Index, 1);
            return;
        }

        Edge[0] = CV1.Arc();
        Edge[nbface] = CV2.Arc();
        tabedg.SetValue(0, Edge[0]);
        tabedg.SetValue(nbface, Edge[nbface]);
        // processing of a fillet arriving on a vertex
        // edge contained in CV.Arc is not inevitably good
        // the edge concerned by the intersection is found

        double dist1 = 0.0, dist2 = 0.0;
        if (CV1.IsVertex())
        {
            trouve = false;
            /*TopoDS_Vertex */ V = CV1.Vertex();
            for (It3.Initialize(myVEMap[V]); It3.More() && !trouve; It3.Next())
            {
                E = ShapeCasts.Edge(It3.Value());
                if (!E.IsSame(Edge[0]) && (containE(F1, E)))
                {
                    trouve = true;
                }
            }
            Vertex Vt = new(), V3 = new(), V4 = new();
            V1 = Exploration.FirstVertex(Edge[0]);
            V2 = Exploration.LastVertex(Edge[0]);
            if (V.IsSame(V1))
            {
                Vt = V2;
            }
            else
            {
                Vt = V1;
            }
            dist1 = (Tool.Pnt(Vt)).Distance(Tool.Pnt(Vtx));
            V3 = Exploration.FirstVertex(E);
            V4 = Exploration.LastVertex(E);
            if (V.IsSame(V3))
            {
                Vt = V4;
            }
            else
            {
                Vt = V3;
            }
            dist2 = (Tool.Pnt(Vt)).Distance(Tool.Pnt(Vtx));
            if (dist2 < dist1)
            {
                Edge[0] = E;
                Orientation ori = default;
                if (V2.IsSame(V3) || V1.IsSame(V4))
                {
                    ori = CV1.TransitionOnArc();
                }
                else
                {
                    ori = global::Kerf.Engine.Topology.TopAbs.Reverse(CV1.TransitionOnArc());
                }
                double par = Tool.Parameter(V, Edge[0]);
                double tol = CV1.Tolerance();
                CV1.SetArc(tol, Edge[0], par, ori);
            }
        }

        if (CV2.IsVertex())
        {
            trouve = false;
            /*TopoDS_Vertex*/ V = CV2.Vertex();
            for (It3.Initialize(myVEMap[V]); It3.More() && !trouve; It3.Next())
            {
                E = ShapeCasts.Edge(It3.Value());
                if (!E.IsSame(Edge[2]) && (containE(F2, E)))
                {
                    trouve = true;
                }
            }
            Vertex Vt = new(), V3 = new(), V4 = new();
            V1 = Exploration.FirstVertex(Edge[2]);
            V2 = Exploration.LastVertex(Edge[2]);
            if (V.IsSame(V1))
            {
                Vt = V2;
            }
            else
            {
                Vt = V1;
            }
            dist1 = (Tool.Pnt(Vt)).Distance(Tool.Pnt(Vtx));
            V3 = Exploration.FirstVertex(E);
            V4 = Exploration.LastVertex(E);
            if (V.IsSame(V3))
            {
                Vt = V4;
            }
            else
            {
                Vt = V3;
            }
            dist2 = (Tool.Pnt(Vt)).Distance(Tool.Pnt(Vtx));
            if (dist2 < dist1)
            {
                Edge[2] = E;
                Orientation ori = default;
                if (V2.IsSame(V3) || V1.IsSame(V4))
                {
                    ori = CV2.TransitionOnArc();
                }
                else
                {
                    ori = global::Kerf.Engine.Topology.TopAbs.Reverse(CV2.TransitionOnArc());
                }
                double par = Tool.Parameter(V, Edge[2]);
                double tol = CV2.Tolerance();
                CV2.SetArc(tol, Edge[2], par, ori);
            }
        }
        if (!onecorner)
        {
            // If there is a regular edge, the faces adjacent to it
            // are not in Fd->IndexOfS1 or Fd->IndexOfS2

            //     TopoDS_Face Find1 ,Find2;
            //     if (isfirst)
            //       edge=stripe->Spine()->Edges(1);
            //     else  edge=stripe->Spine()->Edges(stripe->Spine()->NbEdges());
            //     It3.Initialize(myEFMap[edge]);
            //     Find1=TopoDS::Face(It3.Value());
            //     trouve=false;
            //     for (It3.Initialize(myEFMap[edge]);It3.More()&&!trouve;It3.Next()) {
            //       F=TopoDS::Face (It3.Value());
            //       if (!F.IsSame(Find1)) {
            // 	Find2=F;trouve=true;
            //       }
            //     }

            // if nface =3 there is a top with 3 edges and a fillet
            // and their common points are on different faces
            // otherwise there is a case when a top has more than 3 edges

            if (nface == 3)
            {
                if (CV1.IsVertex())
                {
                    findonf1 = true;
                }
                if (CV2.IsVertex())
                {
                    findonf2 = true;
                }
                if (!findonf1)
                {
                    NCollection_IndexedMap<Shape> MapV = new(ShapeHasher.Instance);
                    Exploration.MapShapes(Edge[0], ShapeType.Vertex, MapV);
                    if (MapV.Extent() == 2)
                    {
                        if (!MapV[1].IsSame(Vtx) && !MapV[2].IsSame(Vtx))
                        {
                            findonf1 = true;
                        }
                    }
                }
                if (!findonf2)
                {
                    NCollection_IndexedMap<Shape> MapV = new(ShapeHasher.Instance);
                    Exploration.MapShapes(Edge[2], ShapeType.Vertex, MapV);
                    if (MapV.Extent() == 2)
                    {
                        if (!MapV[1].IsSame(Vtx) && !MapV[2].IsSame(Vtx))
                        {
                            findonf2 = true;
                        }
                    }
                }

                // detect and process OnSame situatuation
                if (state == ChFiDS_OnSame)
                {
                    Edge[] threeE = [new(), new(), new()];

                    ChFi3d_cherche_element(Vtx, EdgeSpine, F1, ref threeE[0], ref V2);
                    ChFi3d_cherche_element(Vtx, EdgeSpine, F2, ref threeE[1], ref V2);
                    threeE[2] = EdgeSpine;
                    if (ChFi3d_EdgeState(threeE, myEFMap) == ChFiDS_OnSame)
                    {
                        isOnSame1 = true;
                        nb = 1;
                        Edge[0] = threeE[0];
                        ChFi3d_cherche_face1(myEFMap[Edge[0]], F1, ref Face[0]);
                        if (findonf2)
                        {
                            findonf1 = true; // not to look for Face[0] again
                        }
                        else
                        {
                            Edge[1] = CV2.Arc();
                        }
                    }
                    else
                    {
                        isOnSame2 = true;
                    }
                }

                // findonf1 findonf2 show if F1 and/or F2 are adjacent
                // to many faces at end
                // the faces at end and intersected edges are found

                if (findonf1 && !isOnSame1)
                {
                    if (CV1.TransitionOnArc() == Forward)
                    {
                        V1 = Exploration.FirstVertex(CV1.Arc());
                    }
                    else
                    {
                        V1 = Exploration.LastVertex(CV1.Arc());
                    }
                    ChFi3d_cherche_face1(myEFMap[CV1.Arc()], F1, ref Face[0]);
                    nb = 1;
                    Ei = Edge[0];
                    while (!V1.IsSame(Vtx))
                    {
                        ChFi3d_cherche_element(V1, Ei, F1, ref E, ref V2);
                        V1 = V2;
                        Ei = E;
                        ChFi3d_cherche_face1(myEFMap[E], F1, ref Face[nb]);
                        cherche_edge1(Face[nb - 1], Face[nb], ref Edge[nb]);
                        nb++;
                        if (nb >= nn)
                        {
                            throw new FailureException("IntersectionAtEnd : the max number of faces reached");
                        }
                    }
                    if (!findonf2)
                    {
                        Edge[nb] = CV2.Arc();
                    }
                }
                if (findonf2 && !isOnSame2)
                {
                    if (!findonf1)
                    {
                        nb = 1;
                    }
                    V1 = Vtx;
                    if (CV2.TransitionOnArc() == Forward)
                    {
                        Vfin = Exploration.LastVertex(CV2.Arc());
                    }
                    else
                    {
                        Vfin = Exploration.FirstVertex(CV2.Arc());
                    }
                    if (!findonf1)
                    {
                        ChFi3d_cherche_face1(myEFMap[CV1.Arc()], F1, ref Face[nb - 1]);
                    }
                    ChFi3d_cherche_element(V1, EdgeSpine, F2, ref E, ref V2);
                    Ei = E;
                    V1 = V2;
                    while (!V1.IsSame(Vfin))
                    {
                        ChFi3d_cherche_element(V1, Ei, F2, ref E, ref V2);
                        Ei = E;
                        V1 = V2;
                        ChFi3d_cherche_face1(myEFMap[E], F2, ref Face[nb]);
                        cherche_edge1(Face[nb - 1], Face[nb], ref Edge[nb]);
                        nb++;
                        if (nb >= nn)
                        {
                            throw new FailureException("IntersectionAtEnd : the max number of faces reached");
                        }
                    }
                    Edge[nb] = CV2.Arc();
                }
                if (isOnSame2)
                {
                    cherche_edge1(Face[nb - 1], F2, ref Edge[nb]);
                    Face[nb] = F2;
                }

                nbface = nb;
            }

            else
            {

                //  this is the case when a top has more than three edges
                //  the faces and edges concerned are found
                bool /*trouve,*/ possible1, possible2;
                trouve = possible1 = possible2 = false;
                Explorer ex = new();
                nb = 0;
                for (ex.Init(CV1.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (Vtx.IsSame(ex.Current()))
                    {
                        possible1 = true;
                    }
                }
                for (ex.Init(CV2.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (Vtx.IsSame(ex.Current()))
                    {
                        possible2 = true;
                    }
                }
                if ((possible1 && possible2) || (!possible1 && !possible2) || (nbarete > 4))
                {
                    while (!trouve)
                    {
                        nb++;
                        if (nb >= nn)
                        {
                            throw new FailureException("IntersectionAtEnd : the max number of faces reached");
                        }
                        if (nb != 1)
                        {
                            F3 = Face[nb - 2];
                        }
                        Face[nb - 1] = F3;
                        if (CV1.Arc().IsSame(edgelibre1))
                        {
                            cherche_face(myVFMap[Vtx], edgelibre2, F1, F2, F3, ref Face[nb - 1]);
                        }
                        else if (CV1.Arc().IsSame(edgelibre2))
                        {
                            cherche_face(myVFMap[Vtx], edgelibre1, F1, F2, F3, ref Face[nb - 1]);
                        }
                        else
                        {
                            cherche_face(myVFMap[Vtx], Edge[nb - 1], F1, F2, F3, ref Face[nb - 1]);
                        }
                        ChFi3d_cherche_edge(Vtx, tabedg, Face[nb - 1], ref Edge[nb], ref V);
                        tabedg.SetValue(nb, Edge[nb]);
                        if (Edge[nb].IsSame(CV2.Arc()))
                        {
                            trouve = true;
                        }
                    }
                    nbface = nb;
                }
                else
                {
                    IntersectMoreCorner(Index);
                    return;
                }
                if (nbarete == 4)
                {
                    // if two consecutive edges are G1 there is only one face of intersection
                    double ang1 = 0.0;
                    Vertex Vcom = new();
                    trouve = false;
                    ChFi3d_cherche_vertex(Edge[0], Edge[1], ref Vcom, out trouve);
                    if (Vcom.IsSame(Vtx))
                    {
                        ang1 = ChFi3d_AngleEdge(Vtx, Edge[0], Edge[1]);
                    }
                    if (Math.Abs(ang1 - M_PI) < 0.01)
                    {
                        oneintersection1 = true;
                        facesau = Face[0];
                        edgesau = Edge[1];
                        Face[0] = Face[1];
                        Edge[1] = Edge[2];
                        nbface = 1;
                    }

                    if (!oneintersection1)
                    {
                        trouve = false;
                        ChFi3d_cherche_vertex(Edge[1], Edge[2], ref Vcom, out trouve);
                        if (Vcom.IsSame(Vtx))
                        {
                            ang1 = ChFi3d_AngleEdge(Vtx, Edge[1], Edge[2]);
                        }
                        if (Math.Abs(ang1 - M_PI) < 0.01)
                        {
                            oneintersection2 = true;
                            facesau = Face[1];
                            edgesau = Edge[1];
                            Edge[1] = Edge[2];
                            nbface = 1;
                        }
                    }
                }
                else if (nbarete == 5)
                {
                    // pro15368
                    //  Modified by Sergey KHROMOV - Fri Dec 21 18:07:43 2001 End
                    bool isTangent0 = ChFi3d.IsTangentFaces(Edge[0], F1, Face[0]);
                    bool isTangent1 = ChFi3d.IsTangentFaces(Edge[1], Face[0], Face[1]);
                    bool isTangent2 = ChFi3d.IsTangentFaces(Edge[2], Face[1], Face[2]);
                    if ((isTangent0 || isTangent2) && isTangent1)
                    {
                        //         GeomAbs_Shape cont0,cont1,cont2;
                        //         cont0=BRep_Tool::Continuity(Edge[0],F1,Face[0]);
                        //         cont1=BRep_Tool::Continuity(Edge[1],Face[0],Face[1]);
                        //         cont2=BRep_Tool::Continuity(Edge[2],Face[1],Face[2]);
                        //         if ((cont0!=GeomAbs_C0 || cont2!=GeomAbs_C0) && cont1!=GeomAbs_C0) {
                        //  Modified by Sergey KHROMOV - Fri Dec 21 18:07:49 2001 Begin
                        facesau = Face[0];
                        edgesau = Edge[0];
                        nbface = 1;
                        Edge[1] = Edge[3];
                        Face[0] = Face[2];
                        oneintersection1 = true;
                    }
                }
            }
        }
        else
        {
            nbface = 1;
            Face[0] = face;
            Edge[1] = Edge[2];
        }

        NCollection_Array1<double> Pardeb = new(1, 4), Parfin = new(1, 4);
        Point2D pfil1 = new(), pfac1 = new(), pfil2 = new(), pfac2 = new(), pint = new(), pfildeb = new();
        Curve2D? Hc1 = null, Hc2 = null;
        IntCurveSurface_HInter inters = new();
        int[] proledge = new int[nn], prolface = new int[nn + 1]; // last prolface[nn] is for Fd
        int[] shrink = new int[nn];
        Face[] faceprol = new Face[nn];
        for (int k = 0; k < nn; k++)
        {
            faceprol[k] = new();
        }
        int[] indcurve = new int[nn];
        int indpoint2 = 0, indpoint1 = 0;
        TopOpeBRepDS_CurvePointInterference? Interfp1 = null, Interfp2 = null;
        TopOpeBRepDS_CurvePointInterference?[] Interfedge = new TopOpeBRepDS_CurvePointInterference?[nn];
        TopOpeBRepDS_SurfaceCurveInterference? Interfc = null;
        TopOpeBRepDS_SurfaceCurveInterference?[] InterfPC = new TopOpeBRepDS_SurfaceCurveInterference?[nn],
                                                 InterfPS = new TopOpeBRepDS_SurfaceCurveInterference?[nn];
        double u2 = 0.0, v2 = 0.0, p1 = 0.0, p2 = 0.0, paredge1 = 0.0;
        double paredge2 = 0.0, tolex = 1.0e-4;
        bool extend = false;
        Surface? Sfacemoins1 = null, Sface = null;
        /***************************************************************************/
        // calculate intersection of the fillet and each face
        // and storage in the DS
        /***************************************************************************/
        for (nb = 1; nb <= nbface; nb++)
        {
            prolface[nb - 1] = 0;
            proledge[nb - 1] = 0;
            shrink[nb - 1] = 0;
        }
        proledge[nbface] = 0;
        prolface[nn] = 0;
        if (oneintersection1 || oneintersection2)
        {
            faceprol[1] = facesau;
        }
        if (!isOnSame1 && !isOnSame2)
        {
            checkShrink = false;
        }
        // in OnSame situation we need intersect Fd with Edge[0] or Edge[nbface] as well
        if (isOnSame1)
        {
            nb = 0;
        }
        else
        {
            nb = 1;
        }
        bool intersOnSameFailed = false;

        for (; nb <= nbface; nb++)
        {
            extend = false;
            E2 = Edge[nb];
            if (nb == 0)
            {
                F = F1;
            }
            else
            {
                F = Face[nb - 1];
                if (prolface[nb - 1] == 0)
                {
                    faceprol[nb - 1] = F;
                }
            }

            if (F.IsNull)
            {
                throw new NullObjectException("IntersectionAtEnd : Trying to intersect with null face");
            }

            Sfacemoins1 = Tool.Surface(F);
            Curve3D? cint = null;
            Curve2D? C2dint1 = null, C2dint2 = null, cface = null, cfacemoins1 = null;

            ///////////////////////////////////////////////////////
            // determine intersections of edges and the fillet
            // to find limitations of intersections face - fillet
            ///////////////////////////////////////////////////////

            if (nb == 1)
            {
                Hc1 = Tool.CurveOnSurface(Edge[0], Face[0], out Ubid, out Ubid);
                if (isOnSame1)
                {
                    // update interference param on Fi1 and point of CV1
                    if (prolface[0] != 0)
                    {
                        Bs.Initialize(faceprol[0], false);
                    }
                    else
                    {
                        Bs.Initialize(Face[0], false);
                    }
                    Curve3D? c3df = DStr.Curve(Fi1.LineIndex()).Curve();
                    double Ufi = Fi2.Parameter(isfirst);
                    ref ChFiDS_FaceInterference Fi = ref Fd.ChangeInterferenceOnS1();
                    if (!IntersUpdateOnSame(HGs, HBs, c3df, F1, Face[0], Edge[0], Vtx, isfirst,
                                                                    10 * tolapp3d, // in
                                                                    ref Fi, ref CV1, ref pfac1, ref Ufi))
                    { // out
                        throw new FailureException("IntersectionAtEnd: pb intersection Face - Fi");
                    }
                    Fi1 = Fi;
                    if (intersOnSameFailed)
                    { // probable at fillet building
                        // look for paredge2
                        Geom2dAPI_ProjectPointOnCurve proj = new();
                        if (C2dint2 is null)
                        {
                            proj.Init(pfac1, Hc1);
                        }
                        else
                        {
                            proj.Init(pfac1, C2dint2);
                        }
                        paredge2 = proj.LowerDistanceParameter();
                    }
                    // update stripe point
                    TopOpeBRepDS_Point tpoint = new(CV1.Point(), tolapp3d);
                    indpoint1 = DStr.AddPoint(tpoint);
                    stripe.SetIndexPoint(indpoint1, isfirst, 1);
                    // reset arc of CV1
                    Vertex vert1 = new(), vert2 = new();
                    Exploration.Vertices(Edge[0], out vert1, out vert2);
                    Orientation arcOri = Vtx.IsSame(vert1) ? Forward : Reversed;
                    CV1.SetArc(tolapp3d, Edge[0], paredge2, arcOri);
                }
                else
                {
                    if (Hc1 is null)
                    {
                        // curve 2d not found. Sfacemoins1 is extended and projection is done there
                        // CV1.Point ()
                        ChFi3d_ExtendSurface(ref Sfacemoins1, ref prolface[0]);
                        if (prolface[0] != 0)
                        {
                            extend = true;
                            BrepBuilder BRE = new();
                            double tol = Tool.Tolerance(F);
                            BRE.MakeFace(ref faceprol[0], Sfacemoins1, F.Location(), tol);
                            if (!isOnSame1)
                            {
                                SurfaceAdaptor Asurf = new();
                                Asurf.Load(Sfacemoins1);
                                ExtPS ext = new(CV1.Point(), Asurf, tol, tol, ExtFlag_MIN);
                                double uc1 = 0.0, vc1 = 0.0;
                                if (ext.IsDone)
                                {
                                    ext.Point(1).Parameter(out uc1, out vc1);
                                    pfac1.SetX(uc1);
                                    pfac1.SetY(vc1);
                                }
                            }
                        }
                    }
                    else
                    {
                        pfac1 = Hc1.Value(CV1.ParameterOnArc());
                    }
                }
                paredge1 = CV1.ParameterOnArc();
                if (Fi1.LineIndex() != 0)
                {
                    pfil1 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(isfirst));
                }
                else
                {
                    pfil1 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(!isfirst));
                }
                pfildeb = pfil1;
            }
            else
            {
                pfil1 = pfil2;
                paredge1 = paredge2;
                pfac1 = pint;
            }

            if (nb != nbface || isOnSame2)
            {
                int nbp = 0;

                Curve3D? C = null;
                C = Tool.Curve(E2, out Ubid, out Vbid);
                TrimmedCurve3D? Ctrim = new TrimmedCurve3D(C, Ubid, Vbid);
                double Utrim = 0.0, Vtrim = 0.0;
                Utrim = Ctrim.BasisCurve().FirstParameter();
                Vtrim = Ctrim.BasisCurve().LastParameter();
                if (Ctrim.IsPeriodic)
                {
                    if (Ubid > Ctrim.Period())
                    {
                        Ubid = (Utrim + Vtrim) / 2;
                        Vbid = Vtrim;
                    }
                    else
                    {
                        Ubid = Utrim;
                        Vbid = (Utrim + Vtrim) / 2;
                    }
                }
                else
                {
                    Ubid = Utrim;
                    Vbid = Vtrim;
                }
                CurveAdaptor? HC = new CurveAdaptor(C, Ubid, Vbid);
                CurveAdaptor Cad = HC;
                inters.Perform(HC, HGs);
                if (prolface[nn] == 0 && (!inters.IsDone() || (inters.NbPoints() == 0)))
                {
                    // extend surface of conge
                    BoundedSurface? S1 = 
                        (DStr.Surface(Fd.Surf()).Surface() as BoundedSurface);
                    if (S1 is not null)
                    {
                        double length = 0.5 * Max(Fi1Length, Fi2Length);
                        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref S1, length, 1, false, !isfirst);
                        prolface[nn] = 1;
                        if (stripe.IsInDS(!isfirst) == 0)
                        {
                            Gs.Load(S1);
                            inters.Perform(HC, HGs);
                            if (inters.IsDone() && inters.NbPoints() != 0)
                            {
                                Fd.ChangeSurf(DStr.AddSurface(new TopOpeBRepDS_Surface(S1, DStr.ChangeSurface(Isurf).Tolerance())));
                                // update history
                                if (myEVIMap.IsBound(EdgeSpine))
                                {
                                    NCollection_List<int>.Iterator itl = new(myEVIMap.ChangeFind(EdgeSpine));
                                    for (; itl.More(); itl.Next())
                                    {
                                        if (itl.Value() == Isurf)
                                        {
                                            myEVIMap.ChangeFind(EdgeSpine).Remove(itl);
                                            break;
                                        }
                                    }
                                    myEVIMap.ChangeFind(EdgeSpine).Append(Fd.Surf());
                                }
                                else
                                {
                                    NCollection_List<int> IndexList = new();
                                    IndexList.Append(Fd.Surf());
                                    myEVIMap.Bind(EdgeSpine, IndexList);
                                }
                                ////////////////
                                Isurf = Fd.Surf();
                            }
                        }
                    }
                }
                if (!inters.IsDone() || (inters.NbPoints() == 0))
                {
                    BSplineCurve3D? cd = (C as BSplineCurve3D);
                    BezierCurve3D? cd1 = (C as BezierCurve3D);
                    if (cd is not null || cd1 is not null)
                    {
                        BrepBuilder BRE = new();
                        Sface = Tool.Surface(Face[nb]);
                        ChFi3d_ExtendSurface(ref Sface, ref prolface[nb]);
                        double tol = Tool.Tolerance(F);
                        BRE.MakeFace(ref faceprol[nb], Sface, Face[nb].Location(), tol);
                        if (nb != 0 && prolface[nb - 1] == 0)
                        {
                            ChFi3d_ExtendSurface(ref Sfacemoins1, ref prolface[nb - 1]);
                            if (prolface[nb - 1] != 0)
                            {
                                tol = Tool.Tolerance(F);
                                BRE.MakeFace(ref faceprol[nb - 1], Sfacemoins1, F.Location(), tol);
                            }
                        }
                        else
                        {
                            int prol = 0;
                            ChFi3d_ExtendSurface(ref Sfacemoins1, ref prol);
                        }
                        GeomInt_IntSS InterSS = new(Sfacemoins1, Sface, 1.0e-7, true, true, true);
                        if (InterSS.IsDone())
                        {
                            trouve = false;
                            for (int i = 1; i <= InterSS.NbLines() && !trouve; i++)
                            {
                                extend = true;
                                cint = InterSS.Line(i);
                                C2dint1 = InterSS.LineOnS1(i);
                                C2dint2 = InterSS.LineOnS2(i);
                                Cad.Load(cint);
                                inters.Perform(HC, HGs);
                                trouve = inters.IsDone() && inters.NbPoints() != 0;
                                // eap occ293, eval tolex on finally trimmed curves
                                //               occ::handle<GeomAdaptor_Surface> H1=new
                                //               GeomAdaptor_Surface(Sfacemoins1); occ::handle<GeomAdaptor_Surface>
                                //               H2=new GeomAdaptor_Surface(Sface);
                                //              tolex=ChFi3d_EvalTolReached(H1,C2dint1,H2,C2dint2,cint);
                                tolex = InterSS.TolReached3d();
                            }
                        }
                    }
                }
                if (inters.IsDone())
                {
                    nbp = inters.NbPoints();
                    if (nbp == 0)
                    {
                        if (nb == 0 || nb == nbface)
                        {
                            intersOnSameFailed = true;
                        }
                        else
                        {
                            PerformMoreThreeCorner(Index, 1);
                            return;
                        }
                    }
                    else
                    {
                        Point3D P = Tool.Pnt(Vtx);
                        double distmin = P.Distance(inters.Point(1).Pnt());
                        nbp = 1;
                        for (int i = 2; i <= inters.NbPoints(); i++)
                        {
                            dist = P.Distance(inters.Point(i).Pnt());
                            if (dist < distmin)
                            {
                                distmin = dist;
                                nbp = i;
                            }
                        }
                        Point2D pt2d = new(inters.Point(nbp).U(), inters.Point(nbp).V());
                        pfil2 = pt2d;
                        paredge2 = inters.Point(nbp).W();
                        if (!extend)
                        {
                            cfacemoins1 = Tool.CurveOnSurface(E2, F, out u2, out v2);
                            if (cfacemoins1 is null)
                            {
                                throw new ConstructionErrorException("Failed to get p-curve of edge");
                            }
                            cface = Tool.CurveOnSurface(E2, Face[nb], out u2, out v2);
                            if (cface is null)
                            {
                                throw new ConstructionErrorException("Failed to get p-curve of edge");
                            }
                            cfacemoins1.D0(paredge2, out pfac2);
                            cface.D0(paredge2, out pint);
                        }
                        else if (C2dint1 is null || C2dint2 is null)
                        {
                            throw new ConstructionErrorException("Failed to get p-curve of edge");
                        }
                        else
                        {
                            C2dint1.D0(paredge2, out pfac2);
                            C2dint2.D0(paredge2, out pint);

                        }
                    }
                }
                else
                {
                    throw new FailureException("IntersectionAtEnd: pb intersection Face cb");
                }
            }
            else
            {
                Hc2 = Tool.CurveOnSurface(E2, Face[nbface - 1], out Ubid, out Ubid);
                if (Hc2 is null)
                {
                    // curve 2d is not found,  Sfacemoins1 is extended CV2.Point() is projected there

                    ChFi3d_ExtendSurface(ref Sfacemoins1, ref prolface[0]);
                    if (prolface[0] != 0)
                    {
                        BrepBuilder BRE = new();
                        extend = true;
                        double tol = Tool.Tolerance(F);
                        BRE.MakeFace(ref faceprol[nb - 1], Sfacemoins1, F.Location(), tol);
                        SurfaceAdaptor Asurf = new();
                        Asurf.Load(Sfacemoins1);
                        ExtPS ext = new(CV2.Point(), Asurf, tol, tol, ExtFlag_MIN);
                        double uc2 = 0.0, vc2 = 0.0;
                        if (ext.IsDone)
                        {
                            ext.Point(1).Parameter(out uc2, out vc2);
                            pfac2.SetX(uc2);
                            pfac2.SetY(vc2);
                        }
                    }
                }
                else
                {
                    pfac2 = Hc2.Value(CV2.ParameterOnArc());
                }
                paredge2 = CV2.ParameterOnArc();
                if (Fi2.LineIndex() != 0)
                {
                    pfil2 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(isfirst));
                }
                else
                {
                    pfil2 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(!isfirst));
                }
            }
            if (nb == 0)
            {
                continue; // found paredge1 on Edge[0] in OnSame situation on F1
            }

            if (nb == nbface && isOnSame2)
            {
                // update interference param on Fi2 and point of CV2
                if (prolface[nb - 1] != 0)
                {
                    Bs.Initialize(faceprol[nb - 1]);
                }
                else
                {
                    Bs.Initialize(Face[nb - 1]);
                }
                Curve3D? c3df = DStr.Curve(Fi2.LineIndex()).Curve();
                double Ufi = Fi1.Parameter(isfirst);
                ref ChFiDS_FaceInterference Fi = ref Fd.ChangeInterferenceOnS2();
                if (!IntersUpdateOnSame(HGs, HBs, c3df, F2, F, Edge[nb], Vtx, isfirst,
                                                                10 * tolapp3d, // in
                                                                ref Fi, ref CV2, ref pfac2, ref Ufi))
                { // out
                    throw new FailureException("IntersectionAtEnd: pb intersection Face - Fi");
                }
                Fi2 = Fi;
                if (intersOnSameFailed)
                { // probable at fillet building
                    // look for paredge2
                    Geom2dAPI_ProjectPointOnCurve proj = new();
                    if (extend)
                    {
                        proj.Init(pfac2, C2dint2);
                    }
                    else
                    {
                        proj.Init(pfac2, Tool.CurveOnSurface(E2, Face[nbface - 1], out Ubid, out Ubid));
                    }
                    paredge2 = proj.LowerDistanceParameter();
                }
                // update stripe point
                TopOpeBRepDS_Point tpoint = new(CV2.Point(), tolapp3d);
                indpoint2 = DStr.AddPoint(tpoint);
                stripe.SetIndexPoint(indpoint2, isfirst, 2);
                // reset arc of CV2
                Vertex vert1 = new(), vert2 = new();
                Exploration.Vertices(Edge[nbface], out vert1, out vert2);
                Orientation arcOri = Vtx.IsSame(vert1) ? Forward : Reversed;
                CV2.SetArc(tolapp3d, Edge[nbface], paredge2, arcOri);
            }

            if (prolface[nb - 1] != 0)
            {
                Bs.Initialize(faceprol[nb - 1]);
            }
            else
            {
                Bs.Initialize(Face[nb - 1]);
            }

            // offset of parameters if they are not in the same period

            // commented by eap 30 May 2002 occ354
            // the following code may cause trimming a wrong part of periodic surface

            //     double  deb,xx1,xx2;
            //     bool  moins2pi,moins2pi1,moins2pi2;
            //     if (DStr.Surface(Fd->Surf()).Surface()->IsUPeriodic()) {
            //       deb=pfildeb.X();
            //       xx1=pfil1.X();
            //       xx2=pfil2.X();
            //       moins2pi=std::abs(deb)< std::abs(std::abs(deb)-2*M_PI);
            //       moins2pi1=std::abs(xx1)< std::abs(std::abs(xx1)-2*M_PI);
            //       moins2pi2=std::abs(xx2)< std::abs(std::abs(xx2)-2*M_PI);
            //       if (moins2pi1!=moins2pi2) {
            //         if  (moins2pi) {
            //           if (!moins2pi1) xx1=xx1-2*M_PI;
            //           if (!moins2pi2) xx2=xx2-2*M_PI;
            //         }
            //         else {
            //           if (moins2pi1) xx1=xx1+2*M_PI;
            //           if (moins2pi2) xx2=xx2+2*M_PI;
            //         }
            //       }
            //       pfil1.SetX(xx1);
            //       pfil2.SetX(xx2);
            //     }
            //     if (couture || Sfacemoins1->IsUPeriodic()) {

            //       double ufmin,ufmax,vfmin,vfmax;
            //       BRepTools::UVBounds(Face[nb-1],out ufmin,out ufmax,out vfmin,out vfmax);
            //       deb=ufmin;
            //       xx1=pfac1.X();
            //       xx2=pfac2.X();
            //       moins2pi=std::abs(deb)< std::abs(std::abs(deb)-2*M_PI);
            //       moins2pi1=std::abs(xx1)< std::abs(std::abs(xx1)-2*M_PI);
            //       moins2pi2=std::abs(xx2)< std::abs(std::abs(xx2)-2*M_PI);
            //       if (moins2pi1!=moins2pi2) {
            //         if  (moins2pi) {
            //           if (!moins2pi1) xx1=xx1-2*M_PI;
            //           if (!moins2pi2) xx2=xx2-2*M_PI;
            //         }
            //         else {
            //           if (moins2pi1) xx1=xx1+2*M_PI;
            //           if (moins2pi2) xx2=xx2+2*M_PI;
            //         }
            //       }
            //       pfac1.SetX(xx1);
            //       pfac2.SetX(xx2);
            //     }

            Pardeb[1] = pfil1.X();
            Pardeb[2] = pfil1.Y();
            Pardeb[3] = pfac1.X();
            Pardeb[4] = pfac1.Y();
            Parfin[1] = pfil2.X();
            Parfin[2] = pfil2.Y();
            Parfin[3] = pfac2.X();
            Parfin[4] = pfac2.Y();

            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pfac1, pfac2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(Bs, uu1, uu2, vv1, vv2);

            //////////////////////////////////////////////////////////////////////
            // calculate intersections face - fillet
            //////////////////////////////////////////////////////////////////////

            if (!ChFi3d_ComputeCurves(HGs, HBs, Pardeb, Parfin, ref Cc, ref Ps, ref Pc, tolapp3d, tol2d, out tolreached,
                                                                nbface == 1))
            {
                PerformMoreThreeCorner(Index, 1);
                return;
            }
            // storage of information in the data structure

            // evaluate tolerances
            p1 = Cc.FirstParameter();
            p2 = Cc.LastParameter();
            double to1 = 0.0, to2 = 0.0;
            Point2D p2d1 = new(), p2d2 = new();
            Point3D P1 = new(), P2 = new(), P3 = new(), P4 = new(), P5 = new(), P6 = new(), P7 = new(), P8 = new();
            HGs.D0(Pardeb[1], Pardeb[2], out P1);
            HGs.D0(Parfin[1], Parfin[2], out P2);
            HBs.D0(Pardeb[3], Pardeb[4], out P3);
            HBs.D0(Parfin[3], Parfin[4], out P4);
            Pc.D0(p1, out p2d1);
            Pc.D0(p2, out p2d2);
            HBs.D0(p2d1.X(), p2d1.Y(), out P7);
            HBs.D0(p2d2.X(), p2d2.Y(), out P8);
            Ps.D0(p1, out p2d1);
            Ps.D0(p2, out p2d2);
            HGs.D0(p2d1.X(), p2d1.Y(), out P5);
            HGs.D0(p2d2.X(), p2d2.Y(), out P6);
            to1 = Max(P1.Distance(P5) + P3.Distance(P7), tolreached);
            to2 = Max(P2.Distance(P6) + P4.Distance(P8), tolreached);

            //////////////////////////////////////////////////////////////////////
            // storage in the DS of the intersection curve
            //////////////////////////////////////////////////////////////////////

            bool Isvtx1 = false;
            bool Isvtx2 = false;
            int indice = 0;

            if (nb == 1)
            {
                indpoint1 = stripe.IndexPoint(isfirst, 1);
                if (!CV1.IsVertex())
                {
                    ref TopOpeBRepDS_Point tpt = ref DStr.ChangePoint(indpoint1);
                    tpt.Tolerance(Max(tpt.Tolerance(), to1));
                }
                else
                {
                    Isvtx1 = true;
                }
            }
            if (nb == nbface)
            {
                indpoint2 = stripe.IndexPoint(isfirst, 2);
                if (!CV2.IsVertex())
                {
                    ref TopOpeBRepDS_Point tpt = ref DStr.ChangePoint(indpoint2);
                    tpt.Tolerance(Max(tpt.Tolerance(), to2));
                }
                else
                {
                    Isvtx2 = true;
                }
            }
            else
            {
                Point3D point = Cc.Value(Cc.LastParameter());
                TopOpeBRepDS_Point tpoint = new(point, to2);
                indpoint2 = DStr.AddPoint(tpoint);
            }

            if (nb != 1)
            {
                ref TopOpeBRepDS_Point tpt = ref DStr.ChangePoint(indpoint1);
                tpt.Tolerance(Max(tpt.Tolerance(), to1));
            }
            TopOpeBRepDS_Curve tcurv3d = new(Cc, tolreached);
            indcurve[nb - 1] = DStr.AddCurve(tcurv3d);

            Interfp1 = ChFi3d_FilPointInDS(Forward, indcurve[nb - 1], indpoint1, Cc.FirstParameter(), Isvtx1);

            Interfp2 = ChFi3d_FilPointInDS(Reversed, indcurve[nb - 1], indpoint2, Cc.LastParameter(), Isvtx2);

            DStr.ChangeCurveInterferences(indcurve[nb - 1]).Append(Interfp1);
            DStr.ChangeCurveInterferences(indcurve[nb - 1]).Append(Interfp2);

            //////////////////////////////////////////////////////////////////////
            // storage for the face
            //////////////////////////////////////////////////////////////////////

            Orientation ori = Forward;
            orface = Face[nb - 1].Orientation();
            if (orface == orsurfdata)
            {
                orien = global::Kerf.Engine.Topology.TopAbs.Reverse(orcourbe);
            }
            else
            {
                orien = orcourbe;
            }
            // limitation of edges of faces
            if (nb == 1)
            {
                int Iarc1 = DStr.AddShape(Edge[0]);
                Interfedge[0] =
                    ChFi3d_FilPointInDS(CV1.TransitionOnArc(), Iarc1, indpoint1, paredge1, Isvtx1);
                // DStr.ChangeShapeInterferences(Edge[0]).Append(Interfp1);
            }
            if (nb == nbface)
            {
                int Iarc2 = DStr.AddShape(Edge[nb]);
                Interfedge[nb] =
                    ChFi3d_FilPointInDS(CV2.TransitionOnArc(), Iarc2, indpoint2, paredge2, Isvtx2);
                // DStr.ChangeShapeInterferences(Edge[nb]).Append(Interfp2);
            }

            if (nb != nbface || oneintersection1 || oneintersection2)
            {
                if (nface == 3)
                {
                    V1 = Exploration.FirstVertex(Edge[nb]);
                    V2 = Exploration.LastVertex(Edge[nb]);
                    if (containV(F1, V1) || containV(F2, V1))
                    {
                        ori = Forward;
                    }
                    else if (containV(F1, V2) || containV(F2, V2))
                    {
                        ori = Reversed;
                    }
                    else
                    {
                        throw new FailureException("IntersectionAtEnd : pb orientation");
                    }

                    if (containV(F1, V1) && containV(F1, V2))
                    {
                        dist1 = (Tool.Pnt(V1)).Distance(Tool.Pnt(Vtx));
                        dist2 = (Tool.Pnt(V2)).Distance(Tool.Pnt(Vtx));
                        if (dist1 < dist2)
                        {
                            ori = Forward;
                        }
                        else
                        {
                            ori = Reversed;
                        }
                    }
                    if (containV(F2, V1) && containV(F2, V2))
                    {
                        dist1 = (Tool.Pnt(V1)).Distance(Tool.Pnt(Vtx));
                        dist2 = (Tool.Pnt(V2)).Distance(Tool.Pnt(Vtx));
                        if (dist1 < dist2)
                        {
                            ori = Forward;
                        }
                        else
                        {
                            ori = Reversed;
                        }
                    }
                }
                else
                {
                    if (Exploration.FirstVertex(Edge[nb]).IsSame(Vtx))
                    {
                        ori = Forward;
                    }
                    else
                    {
                        ori = Reversed;
                    }
                }
                if (!extend && !(oneintersection1 || oneintersection2))
                {
                    int Iarc2 = DStr.AddShape(Edge[nb]);
                    Interfedge[nb] = ChFi3d_FilPointInDS(ori, Iarc2, indpoint2, paredge2);
                    //  DStr.ChangeShapeInterferences(Edge[nb]).Append(Interfp2);
                }
                else
                {
                    if (!(oneintersection1 || oneintersection2))
                    {
                        proledge[nb] = 1; // OCCT: true in an int
                    }
                    int indp1 = 0, indp2 = 0, ind = 0;
                    Point3D pext = new();
                    double ubid = 0.0, vbid = 0.0;
                    pext = Tool.Pnt(Vtx);
                    CurveAdaptor cad = new();
                    Curve3D? csau = null;
                    if (!(oneintersection1 || oneintersection2))
                    {
                        cad.Load(cint);
                        csau = cint;
                    }
                    else
                    {
                        csau = Tool.Curve(edgesau, out ubid, out vbid);
                        BoundedCurve3D? C1 = (csau as BoundedCurve3D);
                        if (oneintersection1 && extend)
                        {
                            if (C1 is not null)
                            {
                                Point3D Pl = new();
                                Pl = C1.Value(C1.LastParameter());
                                // bool sens;
                                sens = Pl.Distance(pext) < tolpt ? 1 : 0; // OCCT: a bool in the int sens
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref C1, CV1.Point(), 1, sens != 0);
                                csau = C1;
                            }
                        }
                        else if (oneintersection2 && extend)
                        {
                            if (C1 is not null)
                            {
                                Point3D Pl = new();
                                Pl = C1.Value(C1.LastParameter());
                                // bool sens;
                                sens = Pl.Distance(pext) < tolpt ? 1 : 0; // OCCT: a bool in the int sens
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref C1, CV2.Point(), 1, sens != 0);
                                csau = C1;
                            }
                        }
                        cad.Load(csau);
                    }
                    CurvePointExtrema ext = new(pext, cad, tolpt);
                    double par1 = 0.0, par2 = 0.0, par = 0.0, ParVtx = 0.0;
                    bool vtx1 = false;
                    bool vtx2 = false;
                    par1 = ext.Point(1).Parameter();
                    ParVtx = par1;
                    if (oneintersection1 || oneintersection2)
                    {
                        if (oneintersection2)
                        {
                            pext = CV2.Point();
                            ind = indpoint2;
                        }
                        else
                        {
                            pext = CV1.Point();
                            ind = indpoint1;
                        }
                        CurvePointExtrema ext2 = new(pext, cad, tolpt);
                        par2 = ext2.Point(1).Parameter();
                    }
                    else
                    {
                        par2 = paredge2;
                        ind = indpoint2;
                    }
                    if (par1 > par2)
                    {
                        indp1 = ind;
                        indp2 = DStr.AddShape(Vtx);
                        vtx2 = true;
                        par = par1;
                        par1 = par2;
                        par2 = par;
                    }
                    else
                    {
                        indp1 = DStr.AddShape(Vtx);
                        indp2 = ind;
                        vtx1 = true;
                    }
                    Curve3D? Ct = new TrimmedCurve3D(csau, par1, par2);
                    Orientation orient = default;
                    Cc.D0(Cc.FirstParameter(), out P1);
                    Cc.D0(Cc.LastParameter(), out P2);
                    Ct.D0(Ct.FirstParameter(), out P3);
                    Ct.D0(Ct.LastParameter(), out P4);
                    if (P2.Distance(P3) < tolpt || P1.Distance(P4) < tolpt)
                    {
                        orient = orien;
                    }
                    else
                    {
                        orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orien);
                    }
                    if (oneintersection1 || oneintersection2)
                    {
                        indice = DStr.AddShape(Face[0]);
                        if (extend)
                        {
                            DStr.SetNewSurface(Face[0], Sfacemoins1);
                            ComputeCurve2d(Ct, ref faceprol[0], ref C2dint1);
                        }
                        else
                        {
                            Edge aLocalEdge = edgesau;
                            if (edgesau.Orientation() != orient)
                            {
                                aLocalEdge.Reverse();
                            }
                            C2dint1 = Tool.CurveOnSurface(aLocalEdge, Face[0], out ubid, out vbid);
                        }
                    }
                    else
                    {
                        indice = DStr.AddShape(Face[nb - 1]);
                        DStr.SetNewSurface(Face[nb - 1], Sfacemoins1);
                    }
                    //// for periodic 3d curves ////
                    if (cad.IsPeriodic && C2dint1 is not null)
                    {
                        Point2D P2d = Tool.Parameters(Vtx, Face[0]);
                        Geom2dAPI_ProjectPointOnCurve Projector = new(P2d, C2dint1);
                        par = Projector.LowerDistanceParameter();
                        double shift = par - ParVtx;
                        if (Math.Abs(shift) > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            par1 += shift;
                            par2 += shift;
                        }
                    }
                    ////////////////////////////////

                    Ct = new TrimmedCurve3D(csau, par1, par2);
                    if (oneintersection1 || oneintersection2)
                    {
                        tolex = 10 * Tool.Tolerance(edgesau);
                    }
                    if (extend)
                    {
                        SurfaceAdaptor? H1 = null, H2 = null;
                        H1 = new SurfaceAdaptor(Sfacemoins1);
                        if (Sface is null)
                        {
                            tolex = Max(tolex, ChFi3d_EvalTolReached(H1, C2dint1, H1, C2dint1, Ct));
                        }
                        else
                        {
                            H2 = new SurfaceAdaptor(Sface);
                            tolex = Max(tolex, ChFi3d_EvalTolReached(H1, C2dint1, H2, C2dint2, Ct));
                        }
                    }
                    TopOpeBRepDS_Curve tcurv = new(Ct, tolex);
                    int indcurv = 0;
                    indcurv = DStr.AddCurve(tcurv);
                    Interfp1 = ChFi3d_FilPointInDS(Forward, indcurv, indp1, par1, vtx1);
                    Interfp2 = ChFi3d_FilPointInDS(Reversed, indcurv, indp2, par2, vtx2);
                    DStr.ChangeCurveInterferences(indcurv).Append(Interfp1);
                    DStr.ChangeCurveInterferences(indcurv).Append(Interfp2);

                    Interfc = ChFi3d_FilCurveInDS(indcurv, indice, C2dint1, orient);
                    DStr.ChangeShapeInterferences(indice).Append(Interfc);
                    if (oneintersection1 || oneintersection2)
                    {
                        indice = DStr.AddShape(facesau);
                        if (facesau.Orientation() == Face[0].Orientation())
                        {
                            orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                        }
                        if (extend)
                        {
                            ComputeCurve2d(Ct, ref faceprol[1], ref C2dint2);
                        }
                        else
                        {
                            Edge aLocalEdge = edgesau;
                            if (edgesau.Orientation() != orient)
                            {
                                aLocalEdge.Reverse();
                            }
                            C2dint2 = Tool.CurveOnSurface(aLocalEdge, facesau, out ubid, out vbid);
                            // Reverse for case of edgesau on closed surface (Face[0] is equal to facesau)
                        }
                    }
                    else
                    {
                        indice = DStr.AddShape(Face[nb]);
                        DStr.SetNewSurface(Face[nb], Sface);
                        if (Face[nb].Orientation() == Face[nb - 1].Orientation())
                        {
                            orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                        }
                    }
                    if (!bordlibre)
                    {
                        Interfc = ChFi3d_FilCurveInDS(indcurv, indice, C2dint2, orient);
                        DStr.ChangeShapeInterferences(indice).Append(Interfc);
                    }
                }
            }

            if (checkShrink
                    && IsShrink(new CurveAdaptor2D(Ps), p1, p2, checkShrParam, isUShrink, global::Kerf.Engine.Foundation.Precision.Parametric(tolreached)))
            {
                shrink[nb - 1] = 1;
                // store section face-chamf curve for previous SurfData
                // Suppose Fd and SDprev are parametrized similarly
                if (!isShrink)
                { // first time
                    ChFiDS_FaceInterference Fi = SDprev.InterferenceOnS1();
                    Point2D UV = Fi.PCurveOnSurf().Value(Fi.Parameter(isfirst));
                    prevSDParam = isUShrink ? UV.X() : UV.Y();
                }
                Point2D UV1 = p2d1, UV2 = p2d2;
                UV1.SetCoord(isUShrink ? 1 : 2, prevSDParam);
                UV2.SetCoord(isUShrink ? 1 : 2, prevSDParam);
                double aTolreached = 0.0;
                ChFi3d_ComputePCurv(Cc, UV1, UV2, out Ps, DStr.Surface(SDprev.Surf()).Surface(), p1, p2, tolapp3d,
                                                        out aTolreached);
                ref TopOpeBRepDS_Curve TCurv = ref DStr.ChangeCurve(indcurve[nb - 1]);
                TCurv.Tolerance(Max(TCurv.Tolerance(), aTolreached));

                InterfPS[nb - 1] = ChFi3d_FilCurveInDS(indcurve[nb - 1], IsurfPrev, Ps, orcourbe);
                DStr.ChangeSurfaceInterferences(IsurfPrev).Append(InterfPS[nb - 1]);

                if (isOnSame2)
                {
                    midP2d = p2d2;
                    midIpoint = indpoint2;
                }
                else if (!isShrink)
                {
                    midP2d = p2d1;
                    midIpoint = indpoint1;
                }
                isShrink = true;
            } // end if shrink

            indice = DStr.AddShape(Face[nb - 1]);
            InterfPC[nb - 1] = ChFi3d_FilCurveInDS(indcurve[nb - 1], indice, Pc, orien);
            if (shrink[nb - 1] == 0)
            {
                InterfPS[nb - 1] = ChFi3d_FilCurveInDS(indcurve[nb - 1], Isurf, Ps, orcourbe);
            }
            indpoint1 = indpoint2;

        } // end loop on faces being intersected with ChFi

        if (isOnSame1)
        {
            CV1.Reset();
        }
        if (isOnSame2)
        {
            CV2.Reset();
        }

        for (nb = 1; nb <= nbface; nb++)
        {
            int indice = DStr.AddShape(Face[nb - 1]);
            DStr.ChangeShapeInterferences(indice).Append(InterfPC[nb - 1]);
            if (shrink[nb - 1] == 0)
            {
                DStr.ChangeSurfaceInterferences(Isurf).Append(InterfPS[nb - 1]);
            }
            if (proledge[nb - 1] == 0)
            {
                DStr.ChangeShapeInterferences(Edge[nb - 1]).Append(Interfedge[nb - 1]);
            }
        }
        DStr.ChangeShapeInterferences(Edge[nbface]).Append(Interfedge[nbface]);

        if (!isShrink)
        {
            stripe.InDS(isfirst);
        }
        else
        {
            // compute curves for !<isfirst> end of <Fd> and <isfirst> end of previous <SurfData>

            // for Fd
            // Bnd_Box box;
            Point2D UV = new(), UV1 = midP2d, UV2 = midP2d;
            if (isOnSame1)
            {
                UV = UV2 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(!isfirst));
            }
            else
            {
                UV = UV1 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(!isfirst));
            }
            double aTolreached = 0.0;
            Curve3D? C3d = null;
            Surface? aSurf = DStr.Surface(Fd.Surf()).Surface();
            // box.Add(aSurf->Value(UV.X(), UV.Y()));

            ChFi3d_ComputeArete(CV1, UV1, CV2, UV2,
                                                    aSurf, ref // in
                                                    C3d, ref Ps, ref p1, ref p2, tolapp3d, tol2d, out aTolreached,
                                                    0); // out except tolers

            indpoint1 = indpoint2 = midIpoint;
            Point3D point = new();
            if (isOnSame1)
            {
                point = C3d.Value(p2);
                TopOpeBRepDS_Point tpoint = new(point, aTolreached);
                indpoint2 = DStr.AddPoint(tpoint);
                UV = Ps.Value(p2);
            }
            else
            {
                point = C3d.Value(p1);
                TopOpeBRepDS_Point tpoint = new(point, aTolreached);
                indpoint1 = DStr.AddPoint(tpoint);
                UV = Ps.Value(p1);
            }
            // box.Add(point);
            // box.Add(aSurf->Value(UV.X(), UV.Y()));

            TopOpeBRepDS_Curve Crv = new TopOpeBRepDS_Curve(C3d, aTolreached);
            int Icurv = DStr.AddCurve(Crv);
            Interfp1 = ChFi3d_FilPointInDS(Forward, Icurv, indpoint1, p1, false);
            Interfp2 = ChFi3d_FilPointInDS(Reversed, Icurv, indpoint2, p2, false);
            Interfc = ChFi3d_FilCurveInDS(Icurv, Isurf, Ps, orcourbe);
            DStr.ChangeCurveInterferences(Icurv).Append(Interfp1);
            DStr.ChangeCurveInterferences(Icurv).Append(Interfp2);
            DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc);

            // for SDprev
            aSurf = DStr.Surface(SDprev.Surf()).Surface();
            UV1.SetCoord(isUShrink ? 1 : 2, prevSDParam);
            UV2.SetCoord(isUShrink ? 1 : 2, prevSDParam);

            ChFi3d_ComputePCurv(C3d, UV1, UV2, out Pc, aSurf, p1, p2, tolapp3d, out aTolreached);

            Crv.Tolerance(Max(Crv.Tolerance(), aTolreached)); // OCCT: changes the local copy, not the curve in the DS

            Interfc = ChFi3d_FilCurveInDS(Icurv, IsurfPrev, Pc, global::Kerf.Engine.Topology.TopAbs.Reverse(orcourbe));
            DStr.ChangeSurfaceInterferences(IsurfPrev).Append(Interfc);

            // UV = isOnSame1 ? UV2 : UV1;
            // box.Add(aSurf->Value(UV.X(), UV.Y()));
            // UV = Ps->Value(isOnSame1 ? p2 : p1);
            // box.Add(aSurf->Value(UV.X(), UV.Y()));
            // ChFi3d_SetPointTolerance(DStr,box, isOnSame1 ? indpoint2 : indpoint1);

            // to process properly this case in ChFi3d_FilDS()
            stripe.InDS(isfirst, 2);
            Fd.ChangeInterference(isOnSame1 ? 2 : 1).SetLineIndex(0);
            ref ChFiDS_CommonPoint CPprev1 = ref SDprev.ChangeVertex(isfirst, isOnSame1 ? 2 : 1);
            ref ChFiDS_CommonPoint CPlast1 = ref Fd.ChangeVertex(isfirst, isOnSame1 ? 2 : 1);
            ref ChFiDS_CommonPoint CPlast2 = ref Fd.ChangeVertex(!isfirst, isOnSame1 ? 2 : 1);
            if (CPprev1.IsOnArc())
            {
                CPlast1 = CPprev1;
                CPprev1.Reset();
                CPprev1.SetPoint(CPlast1.Point());
                CPlast2.Reset();
                CPlast2.SetPoint(CPlast1.Point());
            }

            // in shrink case, self intersection is possible at <midIpoint>,
            // eval its tolerance intersecting Ps and Pcurve at end.
            // Find end curves closest to shrinked part
            for (nb = 0; nb < nbface; nb++)
            {
                if (isOnSame1 ? shrink[nb + 1] != 0 : shrink[nb] == 0)
                {
                    break;
                }
            }
            Curve3D? Cend = DStr.Curve(indcurve[nb]).Curve();
            Curve2D? PCend = InterfPS[nb].PCurve();
            // point near which self intersection may occur
            ref TopOpeBRepDS_Point Pds = ref DStr.ChangePoint(midIpoint);
            Point3D Pvert = Pds.Point();
            double tol = Pds.Tolerance();

            CurveAdaptor2D PC1 = new(Ps), PC2 = new(PCend);
            Geom2dInt_GInter Intersector = new(PC1, PC2, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (!Intersector.IsDone())
            {
                return;
            }
            for (nb = 1; nb <= Intersector.NbPoints(); nb++)
            {
                IntRes2d_IntersectionPoint ip = Intersector.Point(nb);
                Point3D Pint = C3d.Value(ip.ParamOnFirst());
                tol = Max(tol, Pvert.Distance(Pint));
                Pint = Cend.Value(ip.ParamOnSecond());
                tol = Max(tol, Pvert.Distance(Pint));
            }
            for (nb = 1; nb <= Intersector.NbSegments(); nb++)
            {
                IntRes2d_IntersectionSegment @is = Intersector.Segment(nb);
                if (@is.HasFirstPoint())
                {
                    IntRes2d_IntersectionPoint ip = @is.FirstPoint();
                    Point3D Pint = C3d.Value(ip.ParamOnFirst());
                    tol = Max(tol, Pvert.Distance(Pint));
                    Pint = Cend.Value(ip.ParamOnSecond());
                    tol = Max(tol, Pvert.Distance(Pint));
                }
                if (@is.HasLastPoint())
                {
                    IntRes2d_IntersectionPoint ip = @is.LastPoint();
                    Point3D Pint = C3d.Value(ip.ParamOnFirst());
                    tol = Max(tol, Pvert.Distance(Pint));
                    Pint = Cend.Value(ip.ParamOnSecond());
                    tol = Max(tol, Pvert.Distance(Pint));
                }
            }
            Pds.Tolerance(tol);
        }
    }

    //  Modified by Sergey KHROMOV - Thu Apr 11 12:23:40 2002 Begin

    //=======================================================================
    // function : PerformMoreSurfdata
    // purpose  :  determine intersections at end on several surfdata
    //=======================================================================
    private void PerformMoreSurfdata(int Index)
    {
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        NCollection_List<ChFiDS_Stripe> aLOfStripe = myVDataMap[Index];
        ChFiDS_Stripe? aStripe = null;
        ChFiDS_Spine? aSpine = null;
        double aTol3d = 1.0e-4;

        if (aLOfStripe.IsEmpty())
        {
            return;
        }

        aStripe = aLOfStripe.First();
        aSpine = aStripe.Spine();

        NCollection_Sequence<ChFiDS_SurfData> aSeqSurfData = 
            aStripe.ChangeSetOfSurfData();
        Vertex aVtx = myVDataMap.FindKey(Index);
        int aSens = 0;
        int anInd = ChFi3d_IndexOfSurfData(aVtx, aStripe, out aSens);
        bool isFirst = (aSens == 1);
        int anIndPrev = 0;
        ChFiDS_SurfData? aSurfData = null;
        ChFiDS_CommonPoint aCP1 = new();
        ChFiDS_CommonPoint aCP2 = new();

        aSurfData = aSeqSurfData.Value(anInd);

        aCP1 = aSurfData.Vertex(isFirst, 1);
        aCP2 = aSurfData.Vertex(isFirst, 2);

        Surface? aSurfPrev = null;
        Surface? aSurf = null;
        Face aFace = new();
        Face aNeighborFace = new();

        FindFace(aVtx, aCP1, aCP2, ref aFace);
        aSurfPrev = Tool.Surface(aFace);

        if (aSens == 1)
        {
            anIndPrev = anInd + 1;
        }
        else
        {
            anIndPrev = anInd - 1;
        }

        Edge anArc1 = new();
        Edge anArc2 = new();
        NCollection_List<Shape>.Iterator anIter = new(myVEMap[aVtx]);
        bool isFound = false;

        for (; anIter.More() && !isFound; anIter.Next())
        {
            anArc1 = ShapeCasts.Edge(anIter.Value());

            if (containE(aFace, anArc1))
            {
                isFound = true;
            }
        }

        isFound = false;
        anIter.Initialize(myVEMap[aVtx]);

        for (; anIter.More() && !isFound; anIter.Next())
        {
            anArc2 = ShapeCasts.Edge(anIter.Value());

            if (containE(aFace, anArc2) && !anArc2.IsSame(anArc1))
            {
                isFound = true;
            }
        }

        // determination of common points aCP1onArc, aCP2onArc and aCP2NotonArc
        // aCP1onArc    is the point on arc of index anInd
        // aCP2onArc    is the point on arc of index anIndPrev
        // aCP2NotonArc is the point of index anIndPrev which is not on arc.

        bool is1stCP1OnArc = false;
        bool is2ndCP1OnArc = false;
        ChFiDS_CommonPoint aCP1onArc = new();
        ChFiDS_CommonPoint aCP2onArc = new();
        ChFiDS_CommonPoint aCP2NotonArc = new();

        aSurfData = aSeqSurfData.Value(anIndPrev);
        aCP1 = aSurfData.Vertex(isFirst, 1);
        aCP2 = aSurfData.Vertex(isFirst, 2);

        if (aCP1.IsOnArc() && (aCP1.Arc().IsSame(anArc1) || aCP1.Arc().IsSame(anArc2)))
        {
            aCP2onArc = aCP1;
            aCP2NotonArc = aCP2;
            is2ndCP1OnArc = true;
        }
        else if (aCP2.IsOnArc() && (aCP2.Arc().IsSame(anArc1) || aCP2.Arc().IsSame(anArc2)))
        {
            aCP2onArc = aCP2;
            aCP2NotonArc = aCP1;
            is2ndCP1OnArc = false;
        }
        else
        {
            return;
        }

        aSurfData = aSeqSurfData.Value(anInd);
        aCP1 = aSurfData.Vertex(isFirst, 1);
        aCP2 = aSurfData.Vertex(isFirst, 2);

        if (aCP1.Point().Distance(aCP2onArc.Point()) <= aTol3d)
        {
            aCP1onArc = aCP2;
            is1stCP1OnArc = false;
        }
        else
        {
            aCP1onArc = aCP1;
            is1stCP1OnArc = true;
        }

        if (!aCP1onArc.IsOnArc())
        {
            return;
        }

        // determination of neighbor surface
        int indSurface = 0;
        if (is1stCP1OnArc)
        {
            indSurface = myListStripe.First().SetOfSurfData().Value(anInd).IndexOfS1();
        }
        else
        {
            indSurface = myListStripe.First().SetOfSurfData().Value(anInd).IndexOfS2();
        }

        aNeighborFace = ShapeCasts.Face(myDS.Shape(indSurface));

        // calculation of intersections
        Curve3D? aCracc = null;
        Curve2D? aPCurv1 = null;
        double aParf = 0.0;
        double aParl = 0.0;
        double aTolReached = 0.0;

        aSurfData = aSeqSurfData.Value(anInd);

        if (isFirst)
        {
            ChFi3d_ComputeArete(aSurfData.VertexLastOnS1(), aSurfData.InterferenceOnS1().PCurveOnSurf().Value(
                                                        aSurfData.InterferenceOnS1().LastParameter()), aSurfData.VertexLastOnS2(),
                                                    aSurfData.InterferenceOnS2().PCurveOnSurf().Value(
                                                        aSurfData.InterferenceOnS2().LastParameter()), DStr.Surface(aSurfData.Surf()).Surface(),
                                                    ref aCracc, ref aPCurv1, ref aParf, ref aParl, aTol3d, tol2d, out aTolReached, 0);
        }
        else
        {
            ChFi3d_ComputeArete(aSurfData.VertexFirstOnS1(), aSurfData.InterferenceOnS1().PCurveOnSurf().Value(
                                                        aSurfData.InterferenceOnS1().FirstParameter()), aSurfData.VertexFirstOnS2(),
                                                    aSurfData.InterferenceOnS2().PCurveOnSurf().Value(
                                                        aSurfData.InterferenceOnS2().FirstParameter()), DStr.Surface(aSurfData.Surf()).Surface(),
                                                    ref aCracc, ref aPCurv1, ref aParf, ref aParl, aTol3d, tol2d, out aTolReached, 0);
        }

        // calculation of the index of the line on anInd.
        // aPClineOnSurf is the pcurve on anInd.
        // aPClineOnFace is the pcurve on face.
        ChFiDS_FaceInterference aFI = new();

        if (is1stCP1OnArc)
        {
            aFI = aSurfData.InterferenceOnS1();
        }
        else
        {
            aFI = aSurfData.InterferenceOnS2();
        }

        Curve3D? aCline = null;
        Curve2D? aPClineOnSurf = null;
        Curve2D? aPClineOnFace = null;
        int indLine = 0;

        indLine = aFI.LineIndex();
        aCline = DStr.Curve(aFI.LineIndex()).Curve();
        aPClineOnSurf = aFI.PCurveOnSurf();
        aPClineOnFace = aFI.PCurveOnFace();

        // intersection between the SurfData number anInd and the Face aFace.
        // Obtaining of curves aCint1, aPCint11 and aPCint12.
        aSurf = DStr.Surface(aSurfData.Surf()).Surface();

        GeomInt_IntSS anInterSS = new(aSurfPrev, aSurf, 1.0e-7, true, true, true);
        Curve3D? aCint1 = null;
        Curve2D? aPCint11 = null;
        Curve2D? aPCint12 = null;
        SurfaceAdaptor? H1 = new SurfaceAdaptor(aSurfPrev);
        SurfaceAdaptor? H2 = new SurfaceAdaptor(aSurf);
        double aTolex1 = 0.0;
        int i = 0;
        Point3D aPext1 = new();
        Point3D aPext2 = new();
        Point3D aPext = new();
        bool isPextFound = false;

        if (!anInterSS.IsDone())
        {
            return;
        }

        isFound = false;

        for (i = 1; i <= anInterSS.NbLines() && !isFound; i++)
        {
            aCint1 = anInterSS.Line(i);
            aPCint11 = anInterSS.LineOnS1(i);
            aPCint12 = anInterSS.LineOnS2(i);
            aTolex1 = ChFi3d_EvalTolReached(H1, aPCint11, H2, aPCint12, aCint1);

            aCint1.D0(aCint1.FirstParameter(), out aPext1);
            aCint1.D0(aCint1.LastParameter(), out aPext2);

            //  Modified by skv - Mon Jun  7 18:38:57 2004 OCC5898 Begin
            //     if (aPext1.Distance(aCP1onArc.Point()) <= aTol3d ||
            // 	aPext2.Distance(aCP1onArc.Point()))
            if (aPext1.Distance(aCP1onArc.Point()) <= aTol3d
                    || aPext2.Distance(aCP1onArc.Point()) <= aTol3d)
            {
                //  Modified by skv - Mon Jun  7 18:38:58 2004 OCC5898 End
                isFound = true;
            }
        }

        if (!isFound)
        {
            return;
        }

        if (aPext1.Distance(aCP2onArc.Point()) > aTol3d && aPext1.Distance(aCP1onArc.Point()) > aTol3d)
        {
            aPext = aPext1;
            isPextFound = true;
        }
        else if (aPext2.Distance(aCP2onArc.Point()) > aTol3d
                          && aPext2.Distance(aCP1onArc.Point()) > aTol3d)
        {
            aPext = aPext2;
            isPextFound = true;
        }
        else
        {
            isPextFound = false;
        }

        bool isDoSecondSection = false;
        double aPar = 0.0;

        if (isPextFound)
        {
            CurveAdaptor aCad = new(aCracc);
            CurvePointExtrema anExt = new(aPext, aCad, aTol3d);

            if (!anExt.IsDone)
            {
                return;
            }

            isFound = false;
            for (i = 1; i <= anExt.ExtCount && !isFound; i++)
            {
                if (anExt.IsMin(i))
                {
                    Point3D aProjPnt = anExt.Point(i).Value();

                    if (aPext.Distance(aProjPnt) <= aTol3d)
                    {
                        aPar = anExt.Point(i).Parameter();
                        isDoSecondSection = true;
                    }
                }
            }
        }

        Curve3D? aTrCracc = null;
        Orientation anOrSD1 = default;
        Orientation anOrSD2 = default;
        int indShape = 0;

        anOrSD1 = aSurfData.Orientation();
        aSurfData = aSeqSurfData.Value(anIndPrev);
        anOrSD2 = aSurfData.Orientation();
        aSurf = DStr.Surface(aSurfData.Surf()).Surface();

        // The following variables will be used if isDoSecondSection is true
        Curve3D? aCint2 = null;
        Curve2D? aPCint21 = null;
        Curve2D? aPCint22 = null;
        double aTolex2 = 0.0;

        if (isDoSecondSection)
        {
            double aPar1 = 0.0;

            aCracc.D0(aCracc.FirstParameter(), out aPext1);

            if (aPext1.Distance(aCP2NotonArc.Point()) <= aTol3d)
            {
                aPar1 = aCracc.FirstParameter();
            }
            else
            {
                aPar1 = aCracc.LastParameter();
            }

            if (aPar1 < aPar)
            {
                aTrCracc = new TrimmedCurve3D(aCracc, aPar1, aPar);
            }
            else
            {
                aTrCracc = new TrimmedCurve3D(aCracc, aPar, aPar1);
            }

            // Second section
            GeomInt_IntSS anInterSS2 = new(aSurfPrev, aSurf, 1.0e-7, true, true, true);

            if (!anInterSS2.IsDone())
            {
                return;
            }

            H1 = new SurfaceAdaptor(aSurfPrev);
            H2 = new SurfaceAdaptor(aSurf);

            isFound = false;

            for (i = 1; i <= anInterSS2.NbLines() && !isFound; i++)
            {
                aCint2 = anInterSS2.Line(i);
                aPCint21 = anInterSS2.LineOnS1(i);
                aPCint22 = anInterSS2.LineOnS2(i);
                aTolex2 = ChFi3d_EvalTolReached(H1, aPCint21, H2, aPCint22, aCint2);

                aCint2.D0(aCint2.FirstParameter(), out aPext1);
                aCint2.D0(aCint2.LastParameter(), out aPext2);

                if (aPext1.Distance(aCP2onArc.Point()) <= aTol3d
                        || aPext2.Distance(aCP2onArc.Point()) <= aTol3d)
                {
                    isFound = true;
                }
            }

            if (!isFound)
            {
                return;
            }
        }
        else
        {
            aTrCracc = new TrimmedCurve3D(aCracc, aCracc.FirstParameter(), aCracc.LastParameter());
        }

        // Storage of the data structure

        // calculation of the orientation of line of surfdata number
        // anIndPrev which contains aCP2onArc

        Curve2D? aPCraccS = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(aTrCracc, aSurf);

        if (is2ndCP1OnArc)
        {
            aFI = aSurfData.InterferenceOnS1();
            indShape = aSurfData.IndexOfS1();
        }
        else
        {
            aFI = aSurfData.InterferenceOnS2();
            indShape = aSurfData.IndexOfS2();
        }

        if (indShape <= 0)
        {
            return;
        }

        Orientation aCurOrient = default;

        aCurOrient = DStr.Shape(indShape).Orientation();
        aCurOrient = global::Kerf.Engine.Topology.TopAbs.Compose(aCurOrient, aSurfData.Orientation());
        aCurOrient = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(aFI.Transition()), aCurOrient);

        // Filling the data structure
        aSurfData = aSeqSurfData.Value(anInd);

        TopOpeBRepDS_Point aPtCP1 = new(aCP1onArc.Point(), aCP1onArc.Tolerance());
        int indCP1onArc = DStr.AddPoint(aPtCP1);
        int indSurf1 = aSurfData.Surf();
        int indArc1 = DStr.AddShape(aCP1onArc.Arc());
        int indSol = aStripe.SolidIndex();

        TopOpeBRepDS_CurvePointInterference? anInterfp1 = null;
        TopOpeBRepDS_CurvePointInterference? anInterfp2 = null;

        anInterfp1 = ChFi3d_FilPointInDS(aCP1onArc.TransitionOnArc(), indArc1, indCP1onArc, aCP1onArc.ParameterOnArc());
        DStr.ChangeShapeInterferences(aCP1onArc.Arc()).Append(anInterfp1);

        NCollection_List<TopOpeBRepDS_Interference> SolidInterfs = 
            DStr.ChangeShapeInterferences(indSol);
        TopOpeBRepDS_SolidSurfaceInterference? SSI = 
            new TopOpeBRepDS_SolidSurfaceInterference(new TopOpeBRepDS_Transition(anOrSD1), TopOpeBRepDS_SOLID, indSol,
                                                                                                TopOpeBRepDS_SURFACE, indSurf1);
        SolidInterfs.Append(SSI);

        // deletion of Surface Data.
        aSeqSurfData.Remove(anInd);

        if (!isFirst)
        {
            anInd--;
        }

        aSurfData = aSeqSurfData.Value(anInd);

        // definition of indices of common points in Data Structure

        int indCP2onArc = 0;
        int indCP2NotonArc = 0;

        if (is2ndCP1OnArc)
        {
            aStripe.SetIndexPoint(ChFi3d_IndexPointInDS(aCP2onArc, DStr), isFirst, 1);
            aStripe.SetIndexPoint(ChFi3d_IndexPointInDS(aCP2NotonArc, DStr), isFirst, 2);

            if (isFirst)
            {
                indCP2onArc = aStripe.IndexFirstPointOnS1();
                indCP2NotonArc = aStripe.IndexFirstPointOnS2();
            }
            else
            {
                indCP2onArc = aStripe.IndexLastPointOnS1();
                indCP2NotonArc = aStripe.IndexLastPointOnS2();
            }
        }
        else
        {
            aStripe.SetIndexPoint(ChFi3d_IndexPointInDS(aCP2onArc, DStr), isFirst, 2);
            aStripe.SetIndexPoint(ChFi3d_IndexPointInDS(aCP2NotonArc, DStr), isFirst, 1);

            if (isFirst)
            {
                indCP2onArc = aStripe.IndexFirstPointOnS2();
                indCP2NotonArc = aStripe.IndexFirstPointOnS1();
            }
            else
            {
                indCP2onArc = aStripe.IndexLastPointOnS2();
                indCP2NotonArc = aStripe.IndexLastPointOnS1();
            }
        }

        int indPoint1 = 0;
        int indPoint2 = 0;
        Point3D aPoint1 = new();
        Point3D aPoint2 = new();

        if (is2ndCP1OnArc)
        {
            aFI = aSurfData.InterferenceOnS1();
            indShape = aSurfData.IndexOfS1();
        }
        else
        {
            aFI = aSurfData.InterferenceOnS2();
            indShape = aSurfData.IndexOfS2();
        }

        Point2D aP2d = new();
        TopOpeBRepDS_SurfaceCurveInterference? anInterfc = null;
        Orientation anOrSurf = aCurOrient;
        Orientation anOrFace = aFace.Orientation();
        int indaFace = DStr.AddShape(aFace);
        int indPoint = indCP2onArc;
        int indCurve = 0;

        aFI.PCurveOnFace().D0(aFI.LastParameter(), out aP2d);
        Surface? Stemp2 = Tool.Surface(ShapeCasts.Face(DStr.Shape(indShape)));
        Stemp2.D0(aP2d.X(), aP2d.Y(), out aPoint2);
        aFI.PCurveOnFace().D0(aFI.FirstParameter(), out aP2d);
        Stemp2.D0(aP2d.X(), aP2d.Y(), out aPoint1);

        if (isDoSecondSection)
        {
            TopOpeBRepDS_Point tpoint = new(aPext, aTolex2);
            TopOpeBRepDS_Curve tcint2 = new(aCint2, aTolex2);

            indPoint = DStr.AddPoint(tpoint);
            indCurve = DStr.AddCurve(tcint2);

            aCint2.D0(aCint2.FirstParameter(), out aPext1);
            aCint2.D0(aCint2.LastParameter(), out aPext2);

            if (aPext1.Distance(aPext) <= aTol3d)
            {
                indPoint1 = indPoint;
                indPoint2 = indCP2onArc;
            }
            else
            {
                indPoint1 = indCP2onArc;
                indPoint2 = indPoint;
            }

            // define the orientation of aCint2
            if (aPext1.Distance(aPoint2) > aTol3d && aPext2.Distance(aPoint1) > aTol3d)
            {
                anOrSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(anOrSurf);
            }

            // ---------------------------------------------------------------
            // storage of aCint2
            anInterfp1 = ChFi3d_FilPointInDS(Forward, indCurve, indPoint1, aCint2.FirstParameter());
            anInterfp2 = ChFi3d_FilPointInDS(Reversed, indCurve, indPoint2, aCint2.LastParameter());
            DStr.ChangeCurveInterferences(indCurve).Append(anInterfp1);
            DStr.ChangeCurveInterferences(indCurve).Append(anInterfp2);

            // interference of aCint2 on the SurfData number anIndPrev
            anInterfc = ChFi3d_FilCurveInDS(indCurve, aSurfData.Surf(), aPCint22, anOrSurf);

            DStr.ChangeSurfaceInterferences(aSurfData.Surf()).Append(anInterfc);
            // interference of aCint2 on aFace

            if (anOrFace == anOrSD2)
            {
                anOrFace = global::Kerf.Engine.Topology.TopAbs.Reverse(anOrSurf);
            }
            else
            {
                anOrFace = anOrSurf;
            }

            anInterfc = ChFi3d_FilCurveInDS(indCurve, indaFace, aPCint21, anOrFace);
            DStr.ChangeShapeInterferences(indaFace).Append(anInterfc);
        }

        aTrCracc.D0(aTrCracc.FirstParameter(), out aPext1);
        aTrCracc.D0(aTrCracc.LastParameter(), out aPext2);
        if (aPext1.Distance(aCP2NotonArc.Point()) <= aTol3d)
        {
            indPoint1 = indCP2NotonArc;
            indPoint2 = indPoint;
        }
        else
        {
            indPoint1 = indPoint;
            indPoint2 = indCP2NotonArc;
        }

        // Define the orientation of aTrCracc
        bool isToReverse = false;
        Point3D aP1 = new();
        Point3D aP2 = new();
        Point3D aP3 = new();
        Point3D aP4 = new();

        if (isDoSecondSection)
        {
            aTrCracc.D0(aTrCracc.FirstParameter(), out aP1);
            aTrCracc.D0(aTrCracc.LastParameter(), out aP2);
            aCint2.D0(aCint2.FirstParameter(), out aP3);
            aCint2.D0(aCint2.LastParameter(), out aP4);
            isToReverse = (aP1.Distance(aP4) > aTol3d && aP2.Distance(aP3) > aTol3d);
        }
        else
        {
            isToReverse = (aPext1.Distance(aPoint2) > aTol3d && aPext2.Distance(aPoint1) > aTol3d);
        }

        if (isToReverse)
        {
            anOrSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(anOrSurf);
        }

        // ---------------------------------------------------------------
        // storage of aTrCracc
        TopOpeBRepDS_Curve tct2 = new(aTrCracc, aTolReached);

        indCurve = DStr.AddCurve(tct2);
        anInterfp1 = ChFi3d_FilPointInDS(Forward, indCurve, indPoint1, aTrCracc.FirstParameter());
        anInterfp2 = ChFi3d_FilPointInDS(Reversed, indCurve, indPoint2, aTrCracc.LastParameter());
        DStr.ChangeCurveInterferences(indCurve).Append(anInterfp1);
        DStr.ChangeCurveInterferences(indCurve).Append(anInterfp2);

        // interference of aTrCracc on the SurfData number anIndPrev

        anInterfc = ChFi3d_FilCurveInDS(indCurve, aSurfData.Surf(), aPCraccS, anOrSurf);
        DStr.ChangeSurfaceInterferences(aSurfData.Surf()).Append(anInterfc);
        aStripe.InDS(isFirst);

        // interference of aTrCracc on the SurfData number anInd
        if (anOrSD1 == anOrSD2)
        {
            anOrSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(anOrSurf);
        }

        anInterfc = ChFi3d_FilCurveInDS(indCurve, indSurf1, aPCurv1, anOrSurf);
        DStr.ChangeSurfaceInterferences(indSurf1).Append(anInterfc);

        // ---------------------------------------------------------------
        // storage of aCint1

        aCint1.D0(aCint1.FirstParameter(), out aPext1);
        if (aPext1.Distance(aCP1onArc.Point()) <= aTol3d)
        {
            indPoint1 = indCP1onArc;
            indPoint2 = indPoint;
        }
        else
        {
            indPoint1 = indPoint;
            indPoint2 = indCP1onArc;
        }

        //  definition of the orientation of aCint1

        aCint1.D0(aCint1.FirstParameter(), out aP1);
        aCint1.D0(aCint1.LastParameter(), out aP2);
        aTrCracc.D0(aTrCracc.FirstParameter(), out aP3);
        aTrCracc.D0(aTrCracc.LastParameter(), out aP4);

        if (aP1.Distance(aP4) > aTol3d && aP2.Distance(aP3) > aTol3d)
        {
            anOrSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(anOrSurf);
        }

        TopOpeBRepDS_Curve aTCint1 = new(aCint1, aTolex1);
        indCurve = DStr.AddCurve(aTCint1);
        anInterfp1 = ChFi3d_FilPointInDS(Forward, indCurve, indPoint1, aCint1.FirstParameter());
        anInterfp2 = ChFi3d_FilPointInDS(Reversed, indCurve, indPoint2, aCint1.LastParameter());
        DStr.ChangeCurveInterferences(indCurve).Append(anInterfp1);
        DStr.ChangeCurveInterferences(indCurve).Append(anInterfp2);

        // interference of aCint1 on the SurfData number anInd

        anInterfc = ChFi3d_FilCurveInDS(indCurve, indSurf1, aPCint12, anOrSurf);
        DStr.ChangeSurfaceInterferences(indSurf1).Append(anInterfc);

        // interference of aCint1 on aFace

        anOrFace = aFace.Orientation();

        if (anOrFace == anOrSD1)
        {
            anOrFace = global::Kerf.Engine.Topology.TopAbs.Reverse(anOrSurf);
        }
        else
        {
            anOrFace = anOrSurf;
        }

        anInterfc = ChFi3d_FilCurveInDS(indCurve, indaFace, aPCint11, anOrFace);
        DStr.ChangeShapeInterferences(indaFace).Append(anInterfc);
        // ---------------------------------------------------------------
        // storage of aCline passing through aCP1onArc and aCP2NotonArc

        Curve3D? aTrCline = 
            new TrimmedCurve3D(aCline, aCline.FirstParameter(), aCline.LastParameter());
        double aTolerance = DStr.Curve(indLine).Tolerance();
        TopOpeBRepDS_Curve aTct3 = new(aTrCline, aTolerance);

        indCurve = DStr.AddCurve(aTct3);

        aTrCline.D0(aTrCline.FirstParameter(), out aPext1);

        if (aPext1.Distance(aCP1onArc.Point()) < aTol3d)
        {
            indPoint1 = indCP1onArc;
            indPoint2 = indCP2NotonArc;
        }
        else
        {
            indPoint1 = indCP2NotonArc;
            indPoint2 = indCP1onArc;
        }
        //  definition of the orientation of aTrCline

        aTrCline.D0(aTrCline.FirstParameter(), out aP1);
        aTrCline.D0(aTrCline.LastParameter(), out aP2);
        aCint1.D0(aCint1.FirstParameter(), out aP3);
        aCint1.D0(aCint1.LastParameter(), out aP4);

        if (aP1.Distance(aP4) > aTol3d && aP2.Distance(aP3) > aTol3d)
        {
            anOrSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(anOrSurf);
        }

        anInterfp1 = ChFi3d_FilPointInDS(Forward, indCurve, indPoint1, aTrCline.FirstParameter());
        anInterfp2 = ChFi3d_FilPointInDS(Reversed, indCurve, indPoint2, aTrCline.LastParameter());
        DStr.ChangeCurveInterferences(indCurve).Append(anInterfp1);
        DStr.ChangeCurveInterferences(indCurve).Append(anInterfp2);

        // interference of aTrCline on the SurfData number anInd

        anInterfc = ChFi3d_FilCurveInDS(indCurve, indSurf1, aPClineOnSurf, anOrSurf);
        DStr.ChangeSurfaceInterferences(indSurf1).Append(anInterfc);

        // interference de ctlin par rapport a Fvoisin
        indShape = DStr.AddShape(aNeighborFace);
        anOrFace = aNeighborFace.Orientation();

        if (anOrFace == anOrSD1)
        {
            anOrFace = global::Kerf.Engine.Topology.TopAbs.Reverse(anOrSurf);
        }
        else
        {
            anOrFace = anOrSurf;
        }

        anInterfc = ChFi3d_FilCurveInDS(indCurve, indShape, aPClineOnFace, anOrFace);
        DStr.ChangeShapeInterferences(indShape).Append(anInterfc);
    }

    //  Modified by Sergey KHROMOV - Thu Apr 11 12:23:40 2002 End

    //==============================================================
    // function : FindFace
    // purpose  : attention it works only if there is only one common face
    //           between P1,P2,V
    //===========================================================

    private bool FindFace(in Vertex V, in ChFiDS_CommonPoint P1, in ChFiDS_CommonPoint P2, ref Face Fv)
    {
        Face Favoid = new();
        return FindFace(V, P1, P2, ref Fv, Favoid);
    }

    private bool FindFace(in Vertex V, in ChFiDS_CommonPoint P1, in ChFiDS_CommonPoint P2, ref Face Fv, in Face Favoid)
    {
        if (P1.IsVertex() || P2.IsVertex())
        {
        }
        if (!(P1.IsOnArc() && P2.IsOnArc()))
        {
            return false;
        }
        NCollection_List<Shape>.Iterator It = new(), Jt = new();
        bool Found = false;
        for (It.Initialize(myEFMap[P1.Arc()]); It.More() && !Found; It.Next())
        {
            Fv = ShapeCasts.Face(It.Value());
            if (!Fv.IsSame(Favoid))
            {
                for (Jt.Initialize(myEFMap[P2.Arc()]); Jt.More() && !Found; Jt.Next())
                {
                    if (ShapeCasts.Face(Jt.Value()).IsSame(Fv))
                    {
                        Found = true;
                    }
                }
            }
        }
        return Found;
    }

    //=======================================================================
    // function : MoreSurfdata
    // purpose  : detects if the intersection at end concerns several Surfdata
    //=======================================================================
    private bool MoreSurfdata(int Index)
    {
        // intersection at end is created on several surfdata if :
        // - the number of surfdata concerning the vertex is more than 1.
        // - and if the last but one surfdata has one of commonpoints on one of
        // two arcs, which constitute the intersections of the face at end and of the fillet

        NCollection_List<ChFiDS_Stripe>.Iterator It = new();
        It.Initialize(myVDataMap[Index]);
        ChFiDS_Stripe? stripe = It.ChangeValue();
        NCollection_Sequence<ChFiDS_SurfData> SeqFil = 
            stripe.ChangeSetOfSurfData();
        Vertex Vtx = myVDataMap.FindKey(Index);
        int sens = 0;
        int num = ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
        bool isfirst = (sens == 1);
        ChFiDS_SurfData? Fd = SeqFil.ChangeValue(num);
        ref ChFiDS_CommonPoint CV1 = ref Fd.ChangeVertex(isfirst, 1);
        ref ChFiDS_CommonPoint CV2 = ref Fd.ChangeVertex(isfirst, 2);

        int num1 = 0, num2 = 0, nbsurf = 0;
        Face Fv = new();
        bool inters = false, oksurf = false;
        nbsurf = stripe.SetOfSurfData().Length();
        // Fv is the face at end
        inters = FindFace(Vtx, CV1, CV2, ref Fv);
        if (sens == 1)
        {
            num1 = 1;
            num2 = num1 + 1;
        }
        else
        {
            num1 = nbsurf;
            num2 = num1 - 1;
        }

        oksurf = false;

        if (nbsurf != 1 && inters)
        {

            // determination of arc1 and arc2 intersection of the fillet and the face at end

            Edge arc1 = new(), arc2 = new();
            NCollection_List<Shape>.Iterator ItE = new();
            bool trouve = false;
            for (ItE.Initialize(myVEMap[Vtx]); ItE.More() && !trouve; ItE.Next())
            {
                arc1 = ShapeCasts.Edge(ItE.Value());
                if (containE(Fv, arc1))
                {
                    trouve = true;
                }
            }
            trouve = false;
            for (ItE.Initialize(myVEMap[Vtx]); ItE.More() && !trouve; ItE.Next())
            {
                arc2 = ShapeCasts.Edge(ItE.Value());
                if (containE(Fv, arc2) && !arc2.IsSame(arc1))
                {
                    trouve = true;
                }
            }

            ChFiDS_SurfData? Fd1 = SeqFil.ChangeValue(num2);
            ref ChFiDS_CommonPoint CV3 = ref Fd1.ChangeVertex(isfirst, 1);
            ref ChFiDS_CommonPoint CV4 = ref Fd1.ChangeVertex(isfirst, 2);

            if (CV3.IsOnArc())
            {
                if (CV3.Arc().IsSame(arc1))
                {
                    if (CV1.Point().Distance(CV3.Point()) < 1.0e-4)
                    {
                        oksurf = true;
                    }
                }
                else if (CV3.Arc().IsSame(arc2))
                {
                    if (CV2.Point().Distance(CV3.Point()) < 1.0e-4)
                    {
                        oksurf = true;
                    }
                }
            }

            if (CV4.IsOnArc())
            {
                if (CV1.Point().Distance(CV4.Point()) < 1.0e-4)
                {
                    oksurf = true;
                }
                else if (CV4.Arc().IsSame(arc2))
                {
                    if (CV2.Point().Distance(CV4.Point()) < 1.0e-4)
                    {
                        oksurf = true;
                    }
                }
            }
        }
        return oksurf;
    }

    // Case of fillets on top with 4 edges, one of them is on the same geometry as the edgeof the fillet

    private void IntersectMoreCorner(int Index)
    {
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();

        // The fillet is returned,
        NCollection_List<ChFiDS_Stripe>.Iterator StrIt = new();
        StrIt.Initialize(myVDataMap[Index]);
        ChFiDS_Stripe? stripe = StrIt.Value();
        ChFiDS_Spine? spine = stripe.Spine();
        NCollection_Sequence<ChFiDS_SurfData> SeqFil = 
            stripe.ChangeSetOfSurfData();
        // the top,
        Vertex Vtx = myVDataMap.FindKey(Index);
        // the SurfData concerned and its CommonPoints,
        int sens = 0;

        // Choose the proper SurfData
        int num = ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
        bool isfirst = (sens == 1);
        if (isfirst)
        {
            for (; num < SeqFil.Length()
                          && ((SeqFil.Value(num).IndexOfS1() == 0) || (SeqFil.Value(num).IndexOfS2() == 0));)
            {
                SeqFil.Remove(num); // The surplus is removed
            }
        }
        else
        {
            for (; num > 1
                          && ((SeqFil.Value(num).IndexOfS1() == 0) || (SeqFil.Value(num).IndexOfS2() == 0));)
            {
                SeqFil.Remove(num); // The surplus is removed
                num--;
            }
        }

        ChFiDS_SurfData? Fd = SeqFil.ChangeValue(num);
        ref ChFiDS_CommonPoint CV1 = ref Fd.ChangeVertex(isfirst, 1);
        ref ChFiDS_CommonPoint CV2 = ref Fd.ChangeVertex(isfirst, 2);
        // To evaluate the cloud of new points.
        Box box1 = new(), box2 = new();

        // The cases of cap are processed separately from intersection.
        // ----------------------------------------------------------

        Face Fv = new(), Fad = new(), Fop = new(), Fopbis = new();
        Edge Arcpiv = new(), Arcprol = new(), Arcspine = new(), Arcprolbis = new();
        if (isfirst)
        {
            Arcspine = spine.Edges(1);
        }
        else
        {
            Arcspine = spine.Edges(spine.NbEdges());
        }
        Orientation OArcprolbis = Forward;
        Orientation OArcprolv = Forward, OArcprolop = Forward;
        int ICurve = 0;
        FaceSurfaceAdaptor? HBs = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? HBad = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? HBop = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor Bs = HBs;
        FaceSurfaceAdaptor Bad = HBad;
        FaceSurfaceAdaptor Bop = HBop;
        Curve3D? Cc = null;
        Curve2D? Pc = null, Ps = null;
        double Ubid = 0.0, Vbid = 0.0;//,mu,Mu,mv,Mv;
        double Udeb = 0.0, Ufin = 0.0;
        // gp_Pnt2d UVf1,UVl1,UVf2,UVl2;
        // double Du,Dv,Step;
        bool inters = true;
        int IFadArc = 1, IFopArc = 2;
        Fop = ShapeCasts.Face(DStr.Shape(Fd.Index(IFopArc)));
        Explorer ex = new();

        {
            if (!CV1.IsOnArc() && !CV2.IsOnArc())
            {
                throw new FailureException("Corner intersmore : no point on arc");
            }
            else if (CV1.IsOnArc() && CV2.IsOnArc())
            {
                bool sur2 = false;
                for (ex.Init(CV1.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (Vtx.IsSame(ex.Current()))
                    {
                        break;
                    }
                }
                for (ex.Init(CV2.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (Vtx.IsSame(ex.Current()))
                    {
                        sur2 = true;
                        break;
                    }
                }
                if (sur2)
                {
                    IFadArc = 2;
                }
            }
            else if (CV2.IsOnArc())
            {
                IFadArc = 2;
            }
            IFopArc = 3 - IFadArc;

            Arcpiv = Fd.Vertex(isfirst, IFadArc).Arc();
            Fad = ShapeCasts.Face(DStr.Shape(Fd.Index(IFadArc)));
            Fop = ShapeCasts.Face(DStr.Shape(Fd.Index(IFopArc)));
            NCollection_List<Shape>.Iterator It = new();
            // The face at end is returned without control of its unicity.
            for (It.Initialize(myEFMap[Arcpiv]); It.More(); It.Next())
            {
                if (!Fad.IsSame(It.Value()))
                {
                    Fv = ShapeCasts.Face(It.Value());
                    break;
                }
            }

            // does the face at end contain the Vertex ?
            bool isinface = false;
            for (ex.Init(Fv, ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(Vtx))
                {
                    isinface = true;
                    break;
                }
            }
            if (!isinface)
            {
                IFadArc = 3 - IFadArc;
                IFopArc = 3 - IFopArc;
                Arcpiv = Fd.Vertex(isfirst, IFadArc).Arc();
                Fad = ShapeCasts.Face(DStr.Shape(Fd.Index(IFadArc)));
                Fop = ShapeCasts.Face(DStr.Shape(Fd.Index(IFopArc)));
                // NCollection_List<TopoDS_Shape>::Iterator It;
                // The face at end is returned without control of its unicity.
                for (It.Initialize(myEFMap[Arcpiv]); It.More(); It.Next())
                {
                    if (!Fad.IsSame(It.Value()))
                    {
                        Fv = ShapeCasts.Face(It.Value());
                        break;
                    }
                }
            }

            if (Fv.IsNull)
            {
                throw new NotDoneException("OneCorner : face at end is not found");
            }

            Fv.Orientation(Forward);
            Fad.Orientation(Forward);

            // In the same way the edge to be extended is returned.
            for (It.Initialize(myVEMap[Vtx]); It.More() && Arcprol.IsNull; It.Next())
            {
                if (!Arcpiv.IsSame(It.Value()))
                {
                    for (ex.Init(Fv, ShapeType.Edge); ex.More(); ex.Next())
                    {
                        if (It.Value().IsSame(ex.Current()))
                        {
                            Arcprol = ShapeCasts.Edge(It.Value());
                            OArcprolv = ex.Current().Orientation();
                            break;
                        }
                    }
                }
            }

            // Guard: Arcprol may be null if the loop above found no matching edge.
            if (Arcprol.IsNull)
            {
                throw new NotDoneException("IntersectMoreCorner: edge to be extended is not found");
            }

            // Fopbis is the face containing the trace of fillet CP.Arc() which of does not contain Vtx.
            // Normally Fobis is either the same as Fop (cylinder), or Fobis is G1 with Fop.
            Fopbis.Orientation(Forward);

            // Fop calls the 4th face non-used for the vertex
            cherche_face(myVFMap[Vtx], Arcprol, Fad, Fv, Fv, ref Fopbis);
            Fop.Orientation(Forward);
            for (ex.Init(Fopbis, ShapeType.Edge); ex.More(); ex.Next())
            {
                if (Arcprol.IsSame(ex.Current()))
                {
                    OArcprolop = ex.Current().Orientation();
                    break;
                }
            }
            Face FFv = new();
            double tol = 0.0;
            int prol = 0;
            BrepBuilder BRE = new();
            Surface? Sface = null;
            Sface = Tool.Surface(Fv);
            ChFi3d_ExtendSurface(ref Sface, ref prol);
            tol = Tool.Tolerance(Fv);
            BRE.MakeFace(ref FFv, Sface, tol);
            if (prol != 0)

            {
                Bs.Initialize(FFv, false);
                DStr.SetNewSurface(Fv, Sface);
            }
            else
            {
                Bs.Initialize(Fv, false);
            }
            Bad.Initialize(Fad);
            Bop.Initialize(Fop);
        }
        // it is necessary to modify the CommonPoint
        // in the space and its parameter in FaceInterference.
        // So both of them are returned in references
        // non const. Attention the modifications are done behind
        // CV1,CV2,Fi1,Fi2.
        ref ChFiDS_CommonPoint CPopArc = ref Fd.ChangeVertex(isfirst, IFopArc);
        ref ChFiDS_FaceInterference FiopArc = ref Fd.ChangeInterference(IFopArc);
        ref ChFiDS_CommonPoint CPadArc = ref Fd.ChangeVertex(isfirst, IFadArc);
        ref ChFiDS_FaceInterference FiadArc = ref Fd.ChangeInterference(IFadArc);
        // the parameter of the vertex is initialized with the value
        // of its opposing vertex (point on arc).
        double wop = Fd.ChangeInterference(IFadArc).Parameter(isfirst);
        Curve3D? c3df = null;
        SurfaceAdaptor? HGs = 
            new SurfaceAdaptor(DStr.Surface(Fd.Surf()).Surface());
        Point2D p2dbout = new();
        {

            // add here more or less restrictive criteria to
            // decide if the intersection with face is done at the
            // extended end or if there will be a cap on sharp end.
            c3df = DStr.Curve(FiopArc.LineIndex()).Curve();
            double uf = FiopArc.FirstParameter();
            double ul = FiopArc.LastParameter();
            CurveAdaptor? Hc3df = null;
            if (c3df.IsPeriodic)
            {
                Hc3df = new CurveAdaptor(c3df);
            }
            else
            {
                Hc3df = new CurveAdaptor(c3df, uf, ul);
            }
            inters = Update(HBs, Hc3df, ref FiopArc, ref CPopArc, ref p2dbout, isfirst, ref wop);
            //  Modified by Sergey KHROMOV - Fri Dec 21 18:08:27 2001 Begin
            //  if(!inters && BRep_Tool::Continuity(Arcprol,Fv,Fop) != GeomAbs_C0){
            if (!inters && ChFi3d.IsTangentFaces(Arcprol, Fv, Fop))
            {
                //  Modified by Sergey KHROMOV - Fri Dec 21 18:08:29 2001 End
                // Arcprol is an edge of tangency, ultimate adjustment by an extrema curve/curve is attempted.
                double ff = 0.0, ll = 0.0;
                Curve2D? gpcprol = Tool.CurveOnSurface(Arcprol, Fv, out ff, out ll);
                if (gpcprol is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                CurveAdaptor2D? pcprol = new CurveAdaptor2D(gpcprol);
                double partemp = Tool.Parameter(Vtx, Arcprol);
                inters =
                    Update(HBs, pcprol, HGs, ref FiopArc, ref CPopArc, ref p2dbout, isfirst, ref partemp, ref wop, 10 * tolapp3d);
            }
            EdgeCurve2DAdaptor? pced = new EdgeCurve2DAdaptor();
            pced.Initialize(CPadArc.Arc(), Fv);
            Update(HBs, pced, HGs, ref FiadArc, ref CPadArc, isfirst);
        }

        Edge edgecouture = new();
        bool couture = false, intcouture = false;
        double tolreached = tolapp3d;
        double par1 = 0.0, par2 = 0.0;
        int indpt = 0, Icurv1 = 0, Icurv2 = 0;
        TrimmedCurve3D? curv1 = null, curv2 = null;
        Curve2D? c2d1 = null, c2d2 = null;

        int Isurf = Fd.Surf();

        if (inters)
        {
            HGs = ChFi3d_BoundSurf(DStr, Fd, 1, 2);
            ChFiDS_FaceInterference Fi1 = Fd.InterferenceOnS1();
            ChFiDS_FaceInterference Fi2 = Fd.InterferenceOnS2();
            NCollection_Array1<double> Pardeb = new(1, 4), Parfin = new(1, 4);
            Point2D pfil1 = new(), pfac1 = new(), pfil2 = new(), pfac2 = new();
            Curve2D? Hc1 = null, Hc2 = null;
            if (IFopArc == 1)
            {
                pfac1 = p2dbout;
            }
            else
            {
                Hc1 = Tool.CurveOnSurface(CV1.Arc(), Fv, out Ubid, out Ubid);
                if (Hc1 is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                pfac1 = Hc1.Value(CV1.ParameterOnArc());
            }
            if (IFopArc == 2)
            {
                pfac2 = p2dbout;
            }
            else
            {
                Hc2 = Tool.CurveOnSurface(CV2.Arc(), Fv, out Ubid, out Ubid);
                if (Hc2 is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                pfac2 = Hc2.Value(CV2.ParameterOnArc());
            }
            if (Fi1.LineIndex() != 0)
            {
                pfil1 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(isfirst));
            }
            else
            {
                pfil1 = Fi1.PCurveOnSurf().Value(Fi1.Parameter(!isfirst));
            }
            if (Fi2.LineIndex() != 0)
            {
                pfil2 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(isfirst));
            }
            else
            {
                pfil2 = Fi2.PCurveOnSurf().Value(Fi2.Parameter(!isfirst));
            }
            ChFi3d_Recale(Bs, ref pfac1, ref pfac2, (IFadArc == 1));
            Pardeb[1] = pfil1.X();
            Pardeb[2] = pfil1.Y();
            Pardeb[3] = pfac1.X();
            Pardeb[4] = pfac1.Y();
            Parfin[1] = pfil2.X();
            Parfin[2] = pfil2.Y();
            Parfin[3] = pfac2.X();
            Parfin[4] = pfac2.Y();
            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pfac1, pfac2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(Bs, uu1, uu2, vv1, vv2);

            if (!ChFi3d_ComputeCurves(HGs, HBs, Pardeb, Parfin, ref Cc, ref Ps, ref Pc, tolapp3d, tol2d, out tolreached))
            {
                throw new FailureException("OneCorner : failed calculation intersection");
            }

            Udeb = Cc.FirstParameter();
            Ufin = Cc.LastParameter();

            // check if the curve has an intersection with sewing edge

            ChFi3d_Couture(Fv, out couture, ref edgecouture);

            if (couture && !Tool.Degenerated(edgecouture))
            {

                // double Ubid,Vbid;
                Curve3D? C = Tool.Curve(edgecouture, out Ubid, out Vbid);
                TrimmedCurve3D? Ctrim = new TrimmedCurve3D(C, Ubid, Vbid);
                CurveAdaptor cur1 = new(Ctrim.BasisCurve());
                CurveAdaptor cur2 = new(Cc);
                ExtCC extCC = new(cur1, cur2);
                if (extCC.IsDone && extCC.ExtCount != 0)
                {
                    int imin = 0;
                    double dist2min = RealLast();
                    for (int i = 1; i <= extCC.ExtCount; i++)
                    {
                        if (extCC.SquareDistance(i) < dist2min)
                        {
                            dist2min = extCC.SquareDistance(i);
                            imin = i;
                        }
                    }
                    if (dist2min <= global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        POnCurv ponc1 = new(), ponc2 = new();
                        extCC.Points(imin, out ponc1, out ponc2);
                        par1 = ponc1.Parameter();
                        par2 = ponc2.Parameter();
                        double Tol = 1.0e-4;
                        if (Math.Abs(par2 - Udeb) > Tol && Math.Abs(Ufin - par2) > Tol)
                        {
                            Point3D P1 = ponc1.Value();
                            TopOpeBRepDS_Point tpoint = new(P1, Tol);
                            indpt = DStr.AddPoint(tpoint);
                            intcouture = true;
                            curv1 = new TrimmedCurve3D(Cc, Udeb, par2);
                            curv2 = new TrimmedCurve3D(Cc, par2, Ufin);
                            TopOpeBRepDS_Curve tcurv1 = new(curv1, tolreached);
                            TopOpeBRepDS_Curve tcurv2 = new(curv2, tolreached);
                            Icurv1 = DStr.AddCurve(tcurv1);
                            Icurv2 = DStr.AddCurve(tcurv2);
                        }
                    }
                }
            }
        }

        else
        {
            throw new MethodNotImplementedException("OneCorner : cap not written");
        }
        int IShape = DStr.AddShape(Fv);
        Orientation Et = Forward;
        if (IFadArc == 1)
        {
            Explorer Exp = new();
            for (Exp.Init(Fv.Oriented(Forward), ShapeType.Edge); Exp.More(); Exp.Next())
            {
                if (Exp.Current().IsSame(CV1.Arc()))
                {
                    Et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(Exp.Current().Orientation(), CV1.TransitionOnArc()));
                    break;
                }
            }
        }
        else
        {
            Explorer Exp = new();
            for (Exp.Init(Fv.Oriented(Forward), ShapeType.Edge); Exp.More(); Exp.Next())
            {
                if (Exp.Current().IsSame(CV2.Arc()))
                {
                    Et = global::Kerf.Engine.Topology.TopAbs.Compose(Exp.Current().Orientation(), CV2.TransitionOnArc());
                    break;
                }
            }

            //
        }


        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(CV1, DStr), isfirst, 1);
        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(CV2, DStr), isfirst, 2);

        if (!intcouture)
        {
            // there is no intersection with edge of sewing
            // curve Cc is stored in the stripe
            // the storage in the DS is done by FILDS.

            TopOpeBRepDS_Curve Tc = new(Cc, tolreached);
            ICurve = DStr.AddCurve(Tc);
            TopOpeBRepDS_SurfaceCurveInterference? Interfc = 
                ChFi3d_FilCurveInDS(ICurve, IShape, Pc, Et);
            DStr.ChangeShapeInterferences(IShape).Append(Interfc);
            stripe.ChangePCurve(isfirst) = Ps;
            stripe.SetCurve(ICurve, isfirst);
            stripe.SetParameters(isfirst, Udeb, Ufin);
        }
        else
        {
            // curves curv1 and curv2 are stored in the DS
            // these curves are not reconstructed by FILDS as
            // stripe->InDS(isfirst) is placed;

            // interferences of curv1 and curv2 on Fv
            ComputeCurve2d(curv1, ref Fv, ref c2d1);
            TopOpeBRepDS_SurfaceCurveInterference? InterFv = null;
            InterFv = ChFi3d_FilCurveInDS(Icurv1, IShape, c2d1, Et);
            DStr.ChangeShapeInterferences(IShape).Append(InterFv);
            ComputeCurve2d(curv2, ref Fv, ref c2d2);
            InterFv = ChFi3d_FilCurveInDS(Icurv2, IShape, c2d2, Et);
            DStr.ChangeShapeInterferences(IShape).Append(InterFv);
            // interferences of curv1 and curv2 on Isurf
            if (Fd.Orientation() == Fv.Orientation())
            {
                Et = global::Kerf.Engine.Topology.TopAbs.Reverse(Et);
            }
            c2d1 = new TrimmedCurve2D(Ps, Udeb, par2);
            InterFv = ChFi3d_FilCurveInDS(Icurv1, Isurf, c2d1, Et);
            DStr.ChangeSurfaceInterferences(Isurf).Append(InterFv);
            c2d2 = new TrimmedCurve2D(Ps, par2, Ufin);
            InterFv = ChFi3d_FilCurveInDS(Icurv2, Isurf, c2d2, Et);
            DStr.ChangeSurfaceInterferences(Isurf).Append(InterFv);

            // limitation of the sewing edge
            int Iarc = DStr.AddShape(edgecouture);
            TopOpeBRepDS_CurvePointInterference? Interfedge = null;
            Orientation ori = default;
            Vertex Vdeb = new(), Vfin = new();
            Vdeb = Exploration.FirstVertex(edgecouture);
            Vfin = Exploration.LastVertex(edgecouture);
            double pard = 0.0, parf = 0.0;
            pard = Tool.Parameter(Vdeb, edgecouture);
            parf = Tool.Parameter(Vfin, edgecouture);
            if (Math.Abs(par1 - pard) < Math.Abs(parf - par1))
            {
                ori = Forward;
            }
            else
            {
                ori = Reversed;
            }
            Interfedge = ChFi3d_FilPointInDS(ori, Iarc, indpt, par1);
            DStr.ChangeShapeInterferences(Iarc).Append(Interfedge);

            // creation of CurveInterferences from Icurv1 and Icurv2
            stripe.InDS(isfirst);
            int ind1 = stripe.IndexPoint(isfirst, 1);
            int ind2 = stripe.IndexPoint(isfirst, 2);
            TopOpeBRepDS_CurvePointInterference? interfprol = 
                ChFi3d_FilPointInDS(Forward, Icurv1, ind1, Udeb);
            DStr.ChangeCurveInterferences(Icurv1).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Reversed, Icurv1, indpt, par2);
            DStr.ChangeCurveInterferences(Icurv1).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Forward, Icurv2, indpt, par2);
            DStr.ChangeCurveInterferences(Icurv2).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Reversed, Icurv2, ind2, Ufin);
            DStr.ChangeCurveInterferences(Icurv2).Append(interfprol);
        }

        ChFi3d_EnlargeBox(HBs, Pc, Udeb, Ufin, ref box1, ref box2);

        if (inters)
        {
            //

            // The small end of curve missing for the extension
            // of the face at end and the limitation of the opposing face is added.

            // Above all the points cut the points with the edge of the spine.
            int IArcspine = DStr.AddShape(Arcspine);
            int IVtx = DStr.AddShape(Vtx);
            Orientation OVtx2 = Forward;
            Orientation OVtx = Forward;
            for (ex.Init(Arcspine.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (Vtx.IsSame(ex.Current()))
                {
                    OVtx = ex.Current().Orientation();
                    break;
                }
            }
            OVtx = global::Kerf.Engine.Topology.TopAbs.Reverse(OVtx);
            double parVtx = Tool.Parameter(Vtx, Arcspine);
            TopOpeBRepDS_CurvePointInterference? interfv = 
                ChFi3d_FilVertexInDS(OVtx, IArcspine, IVtx, parVtx);
            DStr.ChangeShapeInterferences(IArcspine).Append(interfv);

            // Modif of lvt to find the suite of Arcprol in the other face
            {
                NCollection_List<Shape>.Iterator It = new();
                for (It.Initialize(myVEMap[Vtx]); It.More(); It.Next())
                {
                    if (!(Arcprol.IsSame(It.Value()) || Arcspine.IsSame(It.Value())
                                || Arcpiv.IsSame(It.Value())))
                    {
                        Arcprolbis = ShapeCasts.Edge(It.Value());
                        break;
                    }
                }
            }
            // end of modif

            // Guard: Arcprolbis may be null if myVEMap[Vtx] contained only
            // Arcprol, Arcspine, and Arcpiv (e.g. when topology maps carry stale
            // data from shared TShape pointers after prior operations).
            if (Arcprolbis.IsNull)
            {
                throw new ConstructionErrorException("IntersectMoreCorner: continuation edge (Arcprolbis) is not found");
            }

            // Now the missing curves are constructed.
            for (ex.Init(Arcprolbis.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (Vtx.IsSame(ex.Current()))
                {
                    OVtx2 = ex.Current().Orientation();
                    break;
                }
            }
            for (ex.Init(Arcprol.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (Vtx.IsSame(ex.Current()))
                {
                    OVtx = ex.Current().Orientation();
                    break;
                }
            }
            // it is checked if Fop has a sewing edge

            //     TopoDS_Edge edgecouture;
            //     bool couture;
            ChFi3d_Couture(Fop, out couture, ref edgecouture);
            Curve2D? Hc = null;
            //    parVtx = BRep_Tool::Parameter(Vtx,Arcprol);
            ChFiDS_FaceInterference Fiop = Fd.Interference(IFopArc);
            Point2D pop1 = new(), pop2 = new(), pv1 = new(), pv2 = new();
            // deb modif
            parVtx = Tool.Parameter(Vtx, Arcprolbis);
            //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222 Begin
            //    if(Fop.IsSame(Fopbis)) OArcprolbis = OArcprolop;
            //    else OArcprolbis = Arcprolbis.Orientation();
            if (Fop.IsSame(Fopbis))
            {
                OArcprolbis = OArcprolop;
            }
            else
            {
                for (ex.Init(Fop, ShapeType.Edge); ex.More(); ex.Next())
                {
                    if (Arcprolbis.IsSame(ex.Current()))
                    {
                        OArcprolbis = ex.Current().Orientation();
                        break;
                    }
                }
            }
            //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222 End
            // fin modif
            Hc = Tool.CurveOnSurface(Arcprolbis, Fop, out Ubid, out Ubid);
            if (Hc is null)
            {
                throw new ConstructionErrorException("Failed to get p-curve of edge");
            }
            pop1 = Hc.Value(parVtx);
            pop2 = Fiop.PCurveOnFace().Value(Fiop.Parameter(isfirst));
            Hc = Tool.CurveOnSurface(Arcprol, Fv, out Ubid, out Ubid);
            if (Hc is null)
            {
                throw new ConstructionErrorException("Failed to get p-curve of edge");
            }
            // modif
            parVtx = Tool.Parameter(Vtx, Arcprol);
            // fin modif
            pv1 = Hc.Value(parVtx);
            pv2 = p2dbout;
            ChFi3d_Recale(Bs, ref pv1, ref pv2, true);
            NCollection_Array1<double> Pardeb = new(1, 4), Parfin = new(1, 4);
            Pardeb[1] = pop1.X();
            Pardeb[2] = pop1.Y();
            Pardeb[3] = pv1.X();
            Pardeb[4] = pv1.Y();
            Parfin[1] = pop2.X();
            Parfin[2] = pop2.Y();
            Parfin[3] = pv2.X();
            Parfin[4] = pv2.Y();
            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pv1, pv2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(Bs, uu1, uu2, vv1, vv2);
            ChFi3d_Boite(pop1, pop2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(Bop, uu1, uu2, vv1, vv2);

            Curve3D? zob3d = null;
            Curve2D? zob2dop = null, zob2dv = null;
            //    double tolreached;
            if (!ChFi3d_ComputeCurves(HBop, HBs, Pardeb, Parfin, ref zob3d, ref zob2dop, ref zob2dv, tolapp3d, tol2d,
                                                                out tolreached))
            {
                throw new FailureException("OneCorner : echec calcul intersection");
            }

            Udeb = zob3d.FirstParameter();
            Ufin = zob3d.LastParameter();
            TopOpeBRepDS_Curve Zob = new(zob3d, tolreached);
            int IZob = DStr.AddCurve(Zob);

            // it is not determined if the curve has an intersection with the sewing edge

            {
                Et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(OVtx, OArcprolv));
                int Iop = DStr.AddShape(Fop);
                TopOpeBRepDS_SurfaceCurveInterference? InterFv = 
                    ChFi3d_FilCurveInDS(IZob, IShape, zob2dv, Et);
                DStr.ChangeShapeInterferences(IShape).Append(InterFv);
                // OVtx = TopAbs::Reverse(OVtx);
                //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222 Begin
                //      Et = TopAbs::Reverse(TopAbs::Compose(OVtx,OArcprolbis));
                Et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(OVtx2, OArcprolbis));
                //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222 End
                // OVtx = TopAbs::Reverse(OVtx);
                //      Et = TopAbs::Reverse(Et);
                TopOpeBRepDS_SurfaceCurveInterference? Interfop = 
                    ChFi3d_FilCurveInDS(IZob, Iop, zob2dop, Et);
                DStr.ChangeShapeInterferences(Iop).Append(Interfop);
                TopOpeBRepDS_CurvePointInterference? interfprol = 
                    ChFi3d_FilVertexInDS(Forward, IZob, IVtx, Udeb);
                DStr.ChangeCurveInterferences(IZob).Append(interfprol);
                int icc = stripe.IndexPoint(isfirst, IFopArc);
                interfprol = ChFi3d_FilPointInDS(Reversed, IZob, icc, Ufin);
                DStr.ChangeCurveInterferences(IZob).Append(interfprol);
            }
        }
        ChFi3d_EnlargeBox(DStr, stripe, Fd, ref box1, ref box2, isfirst);
        if (CV1.IsOnArc())
        {
            ChFi3d_EnlargeBox(CV1.Arc(), myEFMap[CV1.Arc()], CV1.ParameterOnArc(), ref box1);
        }
        if (CV2.IsOnArc())
        {
            ChFi3d_EnlargeBox(CV2.Arc(), myEFMap[CV2.Arc()], CV2.ParameterOnArc(), ref box2);
        }
        if (!CV1.IsVertex())
        {
            ChFi3d_SetPointTolerance(DStr, box1, stripe.IndexPoint(isfirst, 1));
        }
        if (!CV2.IsVertex())
        {
            ChFi3d_SetPointTolerance(DStr, box2, stripe.IndexPoint(isfirst, 2));
        }

    }

}

internal static class ChFi3d_Builder_C1
{
    internal static double recadre(double p, double @ref, bool isfirst, double first, double last)
    {
        double pp = p;
        if (isfirst)
        {
            pp -= (last - first);
        }
        else
        {
            pp += (last - first);
        }
        if (Math.Abs(pp - @ref) < Math.Abs(p - @ref))
        {
            return pp;
        }
        return p;
    }

    internal static bool Update(SurfaceAdaptorBase? fb, Curve2DAdaptorBase? pcfb, SurfaceAdaptorBase? surf, ref ChFiDS_FaceInterference fi, ref ChFiDS_CommonPoint cp, ref Point2D p2dbout, bool isfirst, ref double pared, ref double wop, double tol)
    {
        CurveOnSurface c1 = new(pcfb, fb);
        Curve2D? pc = fi.PCurveOnSurf();
        CurveAdaptor2D? hpc = new CurveAdaptor2D(pc);
        CurveOnSurface c2 = new(hpc, surf);
        LocateExtCC ext = new(c1, c2, pared, wop);
        if (ext.IsDone)
        {
            double dist2 = ext.SquareDistance();
            if (dist2 < tol * tol)
            {
                POnCurv ponc1 = new(), ponc2 = new();
                ext.Point(out ponc1, out ponc2);
                double parfb = ponc1.Parameter();
                p2dbout = pcfb.Value(parfb);
                pared = ponc1.Parameter();
                wop = ponc2.Parameter();
                fi.SetParameter(wop, isfirst);
                cp.Reset();
                cp.SetPoint(ponc1.Value());
                return true;
            }
        }
        return false;
    }

    internal static bool Update(SurfaceAdaptorBase? fb, CurveAdaptorBase? ct, ref ChFiDS_FaceInterference fi, ref ChFiDS_CommonPoint cp, ref Point2D p2dbout, bool isfirst, ref double wop)
    {
        IntCurveSurface_HInter Intersection = new();
        // check if in KPart the limits of the tangency line
        // are already in place at this stage.
        // Modif lvt : the periodic cases are reframed, espercially if nothing was found.
        double w = 0.0, uf = ct.FirstParameter(), ul = ct.LastParameter();

        double wbis = 0.0;

        bool isperiodic = ct.IsPeriodic, recadrebis = false;
        Intersection.Perform(ct, fb);
        if (Intersection.IsDone())
        {
            int nbp = Intersection.NbPoints(), i, isol = 0, isolbis = 0;
            double dist = global::Kerf.Engine.Foundation.Precision.Infinite();
            double distbis = global::Kerf.Engine.Foundation.Precision.Infinite();
            for (i = 1; i <= nbp; i++)
            {
                w = Intersection.Point(i).W();
                if (isperiodic)
                {
                    w = recadre(w, wop, isfirst, uf, ul);
                }
                if (uf <= w && ul >= w && Math.Abs(w - wop) < dist)
                {
                    isol = i;
                    dist = Math.Abs(w - wop);
                }
            }
            if (isperiodic)
            {
                for (i = 1; i <= nbp; i++)
                {
                    w = Intersection.Point(i).W();
                    if (uf <= w && ul >= w && Math.Abs(w - wop) < distbis
                            && (Math.Abs(w - ul) <= 0.01 || Math.Abs(w - uf) <= 0.01))
                    {
                        isolbis = i;
                        wbis = recadre(w, wop, isfirst, uf, ul);
                        distbis = Math.Abs(wbis - wop);
                        recadrebis = true;
                    }
                }
            }
            if (isol == 0 && isolbis == 0)
            {
                return false;
            }
            if (!recadrebis)
            {
                IntCurveSurface_IntersectionPoint pint = Intersection.Point(isol);
                p2dbout.SetCoord(pint.U(), pint.V());
                w = pint.W();
                if (isperiodic)
                {
                    w = global::Kerf.Engine.Geometry.ElCLib.InPeriod(w, uf, ul);
                }
            }
            else
            {
                if (dist > distbis)
                {
                    IntCurveSurface_IntersectionPoint pint = Intersection.Point(isolbis);
                    p2dbout.SetCoord(pint.U(), pint.V());
                    w = wbis;
                }
                else
                {
                    IntCurveSurface_IntersectionPoint pint = Intersection.Point(isol);
                    p2dbout.SetCoord(pint.U(), pint.V());
                    w = pint.W();
                    w = global::Kerf.Engine.Geometry.ElCLib.InPeriod(w, uf, ul);
                }
            }
            fi.SetParameter(w, isfirst);
            cp.Reset();
            cp.SetPoint(ct.Value(w));
            wop = w;
            return true;
        }
        return false;
    }

    internal static bool IntersUpdateOnSame(SurfaceAdaptor? HGs, FaceSurfaceAdaptor? HBs, Curve3D? c3dFI, in Face Fop, in Face Fprol, in Edge Eprol, in Vertex Vtx, bool isFirst, double Tol, ref ChFiDS_FaceInterference FIop, ref ChFiDS_CommonPoint CPop, ref Point2D FprolUV, ref double c3dU)
    {
        // add more or less restrictive criterions to
        // decide if the intersection is done with the face at
        // extended end or if the end is sharp.
        double uf = FIop.FirstParameter();
        double ul = FIop.LastParameter();
        CurveAdaptor? Hc3df = null;
        if (c3dFI.IsPeriodic)
        {
            Hc3df = new CurveAdaptor(c3dFI);
        }
        else
        {
            Hc3df = new CurveAdaptor(c3dFI, uf, ul);
        }

        if (Update(HBs, Hc3df, ref FIop, ref CPop, ref FprolUV, isFirst, ref c3dU))
        {
            return true;
        }

        if (!ChFi3d.IsTangentFaces(Eprol, Fprol, Fop))
        {
            return false;
        }

        Curve2D? gpcprol = Tool.CurveOnSurface(Eprol, Fprol, out uf, out ul);
        if (gpcprol is null)
        {
            throw new ConstructionErrorException("Failed to get p-curve of edge");
        }
        CurveAdaptor2D? pcprol = new CurveAdaptor2D(gpcprol);
        double partemp = Tool.Parameter(Vtx, Eprol);

        return Update(HBs, pcprol, HGs, ref FIop, ref CPop, ref FprolUV, isFirst, ref partemp, ref c3dU, Tol);
    }

    internal static bool Update(SurfaceAdaptorBase? face, Curve2DAdaptorBase? edonface, SurfaceAdaptorBase? surf, ref ChFiDS_FaceInterference fi, ref ChFiDS_CommonPoint cp, bool isfirst)
    {
        if (!cp.IsOnArc())
        {
            return false;
        }
        CurveOnSurface c1 = new(edonface, face);
        double pared = cp.ParameterOnArc();
        double parltg = fi.Parameter(isfirst);
        Curve2D? pc = fi.PCurveOnSurf();
        double f = fi.FirstParameter();
        double l = fi.LastParameter();
        double delta = 0.1 * (l - f);
        f = Max(f - delta, pc.FirstParameter());
        l = Min(l + delta, pc.LastParameter());
        CurveAdaptor2D? hpc = new CurveAdaptor2D(pc, f, l);
        CurveOnSurface c2 = new(hpc, surf);

        LocateExtCC ext = new(c1, c2, pared, parltg);
        if (ext.IsDone)
        {
            POnCurv ponc1 = new(), ponc2 = new();
            ext.Point(out ponc1, out ponc2);
            pared = ponc1.Parameter();
            parltg = ponc2.Parameter();
            if ((parltg > f) && (parltg < l))
            {
                ////modified by jgv, 10.05.2012 for the bug 23139, 25657////
                Curve2D? PConF = fi.PCurveOnFace();
                if (PConF is not null)
                {
                    TrimmedCurve2D? aTrCurve = (PConF as TrimmedCurve2D);
                    if (aTrCurve is not null)
                    {
                        PConF = aTrCurve.BasisCurve();
                    }
                    if (!PConF.IsPeriodic)
                    {
                        if (isfirst)
                        {
                            double fpar = PConF.FirstParameter();
                            if (parltg < fpar)
                            {
                                parltg = fpar;
                            }
                        }
                        else
                        {
                            double lpar = PConF.LastParameter();
                            if (parltg > lpar)
                            {
                                parltg = lpar;
                            }
                        }
                    }
                }
                /////////////////////////////////////////////////////
                fi.SetParameter(parltg, isfirst);
                cp.SetArc(cp.Tolerance(), cp.Arc(), pared, cp.TransitionOnArc());
                return true;
            }
        }
        return false;
    }

    internal static void ChFi3d_ExtendSurface(ref Surface? S, ref int prol)
    {
        if (prol != 0)
        {
            return;
        }

        prol = ((S is global::Kerf.Geometry.BSplineSurface) ? 1 : (S is global::Kerf.Geometry.BezierSurface) ? 2 : 0);
        if (prol == 0)
        {
            return;
        }

        double length = 0.0, umin = 0.0, umax = 0.0, vmin = 0.0, vmax = 0.0;
        Point3D P1 = new(), P2 = new();
        S.Bounds(out umin, out umax, out vmin, out vmax);
        S.D0(umin, vmin, out P1);
        S.D0(umax, vmax, out P2);
        length = P1.Distance(P2);

        BoundedSurface? aBS = (S as BoundedSurface);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aBS, length, 1, false, true);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aBS, length, 1, true, true);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aBS, length, 1, false, false);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aBS, length, 1, true, false);
        S = aBS;
    }

    internal static void ComputeCurve2d(Curve3D? Ct, ref Face Face, ref Curve2D? C2d)
    {
        Edge E1 = new();
        NCollection_IndexedMap<Shape> MapE1 = new(ShapeHasher.Instance);
        BRepLib_MakeEdge Bedge = new(Ct);
        Edge edg = Bedge.Edge();
        BRepAlgo_NormalProjection OrtProj = new();
        OrtProj.Init(Face);
        OrtProj.Add(edg);
        OrtProj.SetParams(1.0e-6, 1.0e-6, C1, 14, 16);
        OrtProj.SetLimit(false);
        OrtProj.Compute3d(false);
        OrtProj.Build();
        double up1 = 0.0, up2 = 0.0;
        if (OrtProj.IsDone())
        {
            Exploration.MapShapes(OrtProj.Projection(), ShapeType.Edge, MapE1);
            if (MapE1.Extent() != 0)
            {
                Shape aLocalShape = MapE1[1];
                E1 = ShapeCasts.Edge(aLocalShape);
                //      E1=TopoDS::Edge( TopoDS_Shape (MapE1[1]));
                C2d = Tool.CurveOnSurface(E1, Face, out up1, out up2);
            }
        }
    }

    internal static void ChFi3d_Recale(FaceSurfaceAdaptor Bs, ref Point2D p1, ref Point2D p2, bool refon1)
    {
        Surface? surf = Bs.GeomSurfaceOriginal();
        TrimmedSurface? ts = 
            (surf as TrimmedSurface);
        if (ts is not null)
        {
            surf = ts.BasisSurface();
        }
        if (surf.IsUPeriodic)
        {
            double u1 = p1.X(), u2 = p2.X();
            double uper = surf.UPeriod();
            if (Math.Abs(u2 - u1) > 0.5 * uper)
            {
                if (u2 < u1 && refon1)
                {
                    u2 += uper;
                }
                else if (u2 < u1 && !refon1)
                {
                    u1 -= uper;
                }
                else if (u1 < u2 && refon1)
                {
                    u2 -= uper;
                }
                else if (u1 < u2 && !refon1)
                {
                    u1 += uper;
                }
            }
            p1.SetX(u1);
            p2.SetX(u2);
        }
        if (surf.IsVPeriodic)
        {
            double v1 = p1.Y(), v2 = p2.Y();
            double vper = surf.VPeriod();
            if (Math.Abs(v2 - v1) > 0.5 * vper)
            {
                if (v2 < v1 && refon1)
                {
                    v2 += vper;
                }
                else if (v2 < v1 && !refon1)
                {
                    v1 -= vper;
                }
                else if (v1 < v2 && refon1)
                {
                    v2 -= vper;
                }
                else if (v1 < v2 && !refon1)
                {
                    v1 += vper;
                }
            }
            p1.SetY(v1);
            p2.SetY(v2);
        }
    }

    internal static bool ChFi3d_SelectStripe(NCollection_List<ChFiDS_Stripe>.Iterator It, in Vertex Vtx, bool thePrepareOnSame)
    {
        if (!thePrepareOnSame)
        {
            return true;
        }

        for (; It.More(); It.Next())
        {
            int sens = 0;
            ChFiDS_Stripe? stripe = It.Value();
            ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
            ChFiDS_State stat = default;
            if (sens == 1)
            {
                stat = stripe.Spine().FirstStatus();
            }
            else
            {
                stat = stripe.Spine().LastStatus();
            }
            if (stat == ChFiDS_OnSame)
            {
                return true;
            }
        }

        return false;
    }

    internal static void cherche_face(NCollection_List<Shape> map, in Edge E, in Face F1, in Face F2, in Face F3, ref Face F)
    {
        Face Fcur = new();
        bool trouve = false;
        NCollection_List<Shape>.Iterator It = new();
        int ie = 0;
        for (It.Initialize(map); It.More() && !trouve; It.Next())
        {
            Fcur = ShapeCasts.Face(It.Value());
            if (!Fcur.IsSame(F1) && !Fcur.IsSame(F2) && !Fcur.IsSame(F3))
            {
                NCollection_IndexedMap<Shape> MapE = new(ShapeHasher.Instance);
                Exploration.MapShapes(Fcur, ShapeType.Edge, MapE);
                for (ie = 1; ie <= MapE.Extent() && !trouve; ie++)
                {
                    Shape aLocalShape = MapE[ie];
                    if (E.IsSame(ShapeCasts.Edge(aLocalShape)))
                    //            if (E.IsSame(TopoDS::Edge(TopoDS_Shape (MapE[ie]))))
                    {
                        F = Fcur;
                        trouve = true;
                    }
                }
            }
        }
        if (F.IsNull)
        {
            throw new ConstructionErrorException("Failed to find face.");
        }
    }

    internal static void cherche_edge1(in Face F1, in Face F2, ref Edge Edge)
    {
        int i = 0, j = 0;
        Edge Ecur1 = new(), Ecur2 = new();
        bool trouve = false;
        NCollection_IndexedMap<Shape> MapE1 = new(ShapeHasher.Instance), MapE2 = new(ShapeHasher.Instance);
        Exploration.MapShapes(F1, global::Kerf.Topology.ShapeType.Edge, MapE1);
        Exploration.MapShapes(F2, global::Kerf.Topology.ShapeType.Edge, MapE2);
        for (i = 1; i <= MapE1.Extent() && !trouve; i++)
        {
            Shape aLocalShape = MapE1[i];
            Ecur1 = ShapeCasts.Edge(aLocalShape);
            //	Ecur1=TopoDS::Edge(TopoDS_Shape (MapE1[i]));
            for (j = 1; j <= MapE2.Extent() && !trouve; j++)
            {
                aLocalShape = MapE2[j];
                Ecur2 = ShapeCasts.Edge(aLocalShape);
                //	      Ecur2=TopoDS::Edge(TopoDS_Shape (MapE2[j]));
                if (Ecur2.IsSame(Ecur1))
                {
                    Edge = Ecur1;
                    trouve = true;
                }
            }
        }
        if (Edge.IsNull)
        {
            throw new ConstructionErrorException("Failed to find edge.");
        }
    }

    internal static bool containV(in Face F1, in Vertex V)
    {
        int i = 0;
        Vertex Vcur = new();
        bool trouve = false;
        bool contain = false;
        NCollection_IndexedMap<Shape> MapV = new(ShapeHasher.Instance);
        Exploration.MapShapes(F1, ShapeType.Vertex, MapV);
        for (i = 1; i <= MapV.Extent() && !trouve; i++)
        {
            Shape aLocalShape = MapV[i];
            Vcur = ShapeCasts.Vertex(aLocalShape);
            //	Vcur=TopoDS::Vertex(TopoDS_Shape (MapV[i]));
            if (Vcur.IsSame(V))
            {
                contain = true;
                trouve = true;
            }
        }
        return contain;
    }

    internal static bool containE(in Face F1, in Edge E)
    {
        int i = 0;
        Edge Ecur = new();
        bool trouve = false;
        bool contain = false;
        NCollection_IndexedMap<Shape> MapE = new(ShapeHasher.Instance);
        Exploration.MapShapes(F1, ShapeType.Edge, MapE);
        for (i = 1; i <= MapE.Extent() && !trouve; i++)
        {
            Shape aLocalShape = MapE[i];
            Ecur = ShapeCasts.Edge(aLocalShape);
            //	Ecur=TopoDS::Edge(TopoDS_Shape (MapE[i]));
            if (Ecur.IsSame(E))
            {
                contain = true;
                trouve = true;
            }
        }
        return contain;
    }

    internal static bool IsShrink(CurveAdaptor2D PC, double Pf, double Pl, double Param, bool isU, double tol)
    {
        switch (PC.GetType())
        {
            case Line: {
                Point2D P1 = PC.Value(Pf);
                Point2D P2 = PC.Value(Pl);
                return Math.Abs(P1.Coord(isU ? 1 : 2) - Param) <= tol
                              && Math.Abs(P2.Coord(isU ? 1 : 2) - Param) <= tol;
            }
            case BezierCurve:
            case BSplineCurve: {
                FunctionSample aSample = new(Pf, Pl, 10);
                int i = 0;
                for (i = 1; i <= aSample.PointCount; i++)
                {
                    Point2D P = PC.Value(aSample.GetParameter(i));
                    if (Math.Abs(P.Coord(isU ? 1 : 2) - Param) > tol)
                    {
                        return false;
                    }
                }
                return true;
            }
            default:
                break;
        }
        return false;

    }


}
