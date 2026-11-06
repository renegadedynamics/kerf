// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_C1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The corners (unit 7N2). OCCT_DEBUG blocks (traces, chronometers) are left out; the file's static functions are in
// the static class named after it.

#nullable disable warnings

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Extrema.ExtFlag;
using static Kerf.Engine.Fillets.Builder_C1;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets;
internal partial class FilletBuilderBase
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

    private void PerformOneCorner(int index, bool prepareOnSame = false)
    {
        DsDataStructure dStr = _ds.ChangeDS();

        // the top,
        Vertex vtx = _vDataMap.FindKey(index);
        // The fillet is returned,
        NCollection_List<Stripe>.Iterator StrIt = new();
        StrIt.Initialize(_vDataMap[index]);
        if (!ChFi3d_SelectStripe(StrIt, vtx, prepareOnSame))
        {
            return;
        }
        Stripe? stripe = StrIt.Value();
        Spine? spine = stripe.Spine();
        NCollection_Sequence<SurfData> seqFil = 
            stripe.ChangeSetOfSurfData();
        // SurfData and its CommonPoints,
        int sens = 0;

        // Choose proper SurfData
        int num = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
        bool isfirst = (sens == 1);
        if (isfirst)
        {
            for (; num < seqFil.Length()
                          && ((seqFil.Value(num).IndexOfS1() == 0) || (seqFil.Value(num).IndexOfS2() == 0));)
            {
                seqFil.Remove(num); // The surplus is removed
            }
        }
        else
        {
            for (; num > 1
                          && ((seqFil.Value(num).IndexOfS1() == 0) || (seqFil.Value(num).IndexOfS2() == 0));)
            {
                seqFil.Remove(num); // The surplus is removed
                num--;
            }
        }

        SurfData? Fd = seqFil.ChangeValue(num);
        ref CommonPoint cv1 = ref Fd.ChangeVertex(isfirst, 1);
        ref CommonPoint cv2 = ref Fd.ChangeVertex(isfirst, 2);
        // To evaluate the new points.
        Box box1 = new(), box2 = new();

        // The cases of cap and intersection are processed separately.
        // ----------------------------------------------------------
        State stat = default;
        if (isfirst)
        {
            stat = spine.FirstStatus();
        }
        else
        {
            stat = spine.LastStatus();
        }
        bool onsame = (stat == State.OnSame);
        Face fv = new(), fad = new(), fop = new();
        Edge arcpiv = new(), arcprol = new(), arcspine = new();
        if (isfirst)
        {
            arcspine = spine.Edges(1);
        }
        else
        {
            arcspine = spine.Edges(spine.EdgeCount);
        }
        Orientation oArcprolv = Forward, oArcprolop = Forward;
        int iCurve = 0;
        FaceSurfaceAdaptor? hBs = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hBad = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hBop = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor bs = hBs;
        FaceSurfaceAdaptor bad = hBad;
        FaceSurfaceAdaptor bop = hBop;
        Curve3D? cc = null;
        Curve2D? pc = null, ps = null;
        double ubid = 0.0, vbid = 0.0;//,mu,Mu,mv,Mv;
        double udeb = 0.0, ufin = 0.0;
        //  gp_Pnt2d UVf1,UVl1,UVf2,UVl2;
        //  double Du,Dv,Step;
        bool inters = true;
        int iFadArc = 1, iFopArc = 2;
        fop = ShapeCasts.Face(dStr.Shape(Fd.Index(iFopArc)));
        Explorer ex = new();


        if (onsame)
        {
            if (!cv1.IsOnArc && !cv2.IsOnArc)
            {
                throw new ConstructionErrorException("Corner OnSame : no point on arc");
            }
            else if (cv1.IsOnArc && cv2.IsOnArc)
            {
                bool sur1 = false, sur2 = false;
                for (ex.Init(cv1.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (vtx.IsSame(ex.Current()))
                    {
                        sur1 = true;
                        break;
                    }
                }
                for (ex.Init(cv2.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (vtx.IsSame(ex.Current()))
                    {
                        sur2 = true;
                        break;
                    }
                }
                if (sur1 && sur2)
                {
                    Edge[] e = [new(), new(), new()];
                    e[0] = cv1.Arc();
                    e[1] = cv2.Arc();
                    e[2] = arcspine;
                    if (ChFi3d_EdgeState(e, _efMap) != State.OnDiff)
                    {
                        iFadArc = 2;
                    }
                }
                else if (sur2)
                {
                    iFadArc = 2;
                }
            }
            else if (cv2.IsOnArc)
            {
                iFadArc = 2;
            }
            iFopArc = 3 - iFadArc;

            arcpiv = Fd.Vertex(isfirst, iFadArc).Arc();
            fad = ShapeCasts.Face(dStr.Shape(Fd.Index(iFadArc)));
            fop = ShapeCasts.Face(dStr.Shape(Fd.Index(iFopArc)));
            NCollection_List<Shape>.Iterator it = new();
            // The face at end is returned without check of its unicity.
            for (it.Initialize(_efMap[arcpiv]); it.More(); it.Next())
            {
                if (!fad.IsSame(it.Value()))
                {
                    fv = ShapeCasts.Face(it.Value());
                    break;
                }
            }

            // Does the face at bout contain the Vertex ?
            bool isinface = false;
            for (ex.Init(fv, ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(vtx))
                {
                    isinface = true;
                    break;
                }
            }
            if (!isinface && Fd.Vertex(isfirst, 3 - iFadArc).IsOnArc)
            {
                iFadArc = 3 - iFadArc;
                iFopArc = 3 - iFopArc;
                arcpiv = Fd.Vertex(isfirst, iFadArc).Arc();
                fad = ShapeCasts.Face(dStr.Shape(Fd.Index(iFadArc)));
                fop = ShapeCasts.Face(dStr.Shape(Fd.Index(iFopArc)));
                // NCollection_List<TopoDS_Shape>::Iterator It;
                // The face at end is returned without check of its unicity.
                for (it.Initialize(_efMap[arcpiv]); it.More(); it.Next())
                {
                    if (!fad.IsSame(it.Value()))
                    {
                        fv = ShapeCasts.Face(it.Value());
                        break;
                    }
                }
            }

            if (fv.IsNull)
            {
                throw new NotDoneException("OneCorner : face at end not found");
            }

            fv.Orient(Forward);
            fad.Orient(Forward);
            fop.Orient(Forward);

            // The edge that will be extended is returned.
            for (it.Initialize(_veMap[vtx]); it.More() && arcprol.IsNull; it.Next())
            {
                if (!arcpiv.IsSame(it.Value()))
                {
                    for (ex.Init(fv, ShapeType.Edge); ex.More(); ex.Next())
                    {
                        if (it.Value().IsSame(ex.Current()))
                        {
                            arcprol = ShapeCasts.Edge(it.Value());
                            oArcprolv = ex.Current().Orientation;
                            break;
                        }
                    }
                }
            }
            if (arcprol.IsNull) /*throw new StdFail_NotDone("OneCorner : edge a prolonger non trouve");*/
            {
                PerformIntersectionAtEnd(index);
                return;
            }
            for (ex.Init(fop, ShapeType.Edge); ex.More(); ex.Next())
            {
                if (arcprol.IsSame(ex.Current()))
                {
                    oArcprolop = ex.Current().Orientation;
                    break;
                }
            }
            Face fFv = new();
            double tol = 0.0;
            int prol = 0;
            BrepBuilder bre = new();
            Surface? sface = null;
            sface = Tool.Surface(fv);
            ChFi3d_ExtendSurface(ref sface, ref prol);
            tol = Tool.Tolerance(fv);
            bre.MakeFace(ref fFv, sface, tol);
            if (prol != 0)

            {
                bs.Initialize(fFv, false);
                dStr.SetNewSurface(fv, sface);
            }
            else
            {
                bs.Initialize(fv, false);
            }
            bad.Initialize(fad);
            bop.Initialize(fop);
        }
        // in case of OnSame it is necessary to modify the CommonPoint
        // in the empty and its parameter in the FaceInterference.
        // They are both returned in non const references. Attention the modifications are done behind
        // de CV1,CV2,Fi1,Fi2.
        ref CommonPoint cPopArc = ref Fd.ChangeVertex(isfirst, iFopArc);
        ref FaceInterference fiopArc = ref Fd.ChangeInterference(iFopArc);
        ref CommonPoint cPadArc = ref Fd.ChangeVertex(isfirst, iFadArc);
        ref FaceInterference fiadArc = ref Fd.ChangeInterference(iFadArc);
        // the parameter of the vertex in the air is initialiced with the value of
        // its opposite (point on arc).
        double wop = Fd.ChangeInterference(iFadArc).Parameter(isfirst);
        Curve3D? c3df = null;
        SurfaceAdaptor? hGs = 
            new SurfaceAdaptor(dStr.Surface(Fd.Surf()).Surface());
        Point2D p2dbout = new();

        if (onsame)
        {

            CommonPoint saveCPopArc = cPopArc;
            c3df = dStr.Curve(fiopArc.LineIndex()).Curve();

            inters = IntersUpdateOnSame(hGs, hBs, c3df, fop, fv, arcprol, vtx, isfirst,
                                                                    10 * _tolapp3d, // in
                                                                    ref fiopArc, ref cPopArc, ref p2dbout,
                                                                    ref wop); // out

            EdgeCurve2DAdaptor? pced = new EdgeCurve2DAdaptor();
            pced.Initialize(cPadArc.Arc(), fv);
            // in the case of degenerated Fi, parameter difference can be even negative (eap, occ293)
            if ((fiadArc.LastParameter() - fiadArc.FirstParameter()) > 10 * _tolesp)
            {
                Update(hBs, pced, hGs, ref fiadArc, ref cPadArc, isfirst);
            }

            if (prepareOnSame)
            {
                // saveCPopArc.SetParameter(wop);
                saveCPopArc.SetPoint(cPopArc.Point());
                cPopArc = saveCPopArc;
                return;
            }
        }
        else
        {
            inters = FindFace(cv1, cv2, ref fv, fop);
            if (!inters)
            {
                PerformIntersectionAtEnd(index);
                return;
            }
            bs.Initialize(fv);
            EdgeCurve2DAdaptor? pced = new EdgeCurve2DAdaptor();
            pced.Initialize(cv1.Arc(), fv);
            Update(hBs, pced, hGs, ref Fd.ChangeInterferenceOnS1(), ref cv1, isfirst);
            pced.Initialize(cv2.Arc(), fv);
            Update(hBs, pced, hGs, ref Fd.ChangeInterferenceOnS2(), ref cv2, isfirst);
        }


        Edge edgecouture = new();
        bool couture = false, intcouture = false;
        double tolreached = _tolapp3d;
        double par1 = 0.0, par2 = 0.0;
        int indpt = 0, icurv1 = 0, icurv2 = 0;
        TrimmedCurve3D? curv1 = null, curv2 = null;
        Curve2D? c2d1 = null, c2d2 = null;

        int isurf = Fd.Surf();

        if (inters)
        {
            hGs = ChFi3d_BoundSurf(dStr, Fd, 1, 2);
            FaceInterference fi1 = Fd.InterferenceOnS1();
            FaceInterference fi2 = Fd.InterferenceOnS2();
            NCollection_Array1<double> pardeb = new(1, 4), parfin = new(1, 4);
            Point2D pfil1 = new(), pfac1 = new(), pfil2 = new(), pfac2 = new();
            Curve2D? hc1 = null, hc2 = null;
            if (onsame && iFopArc == 1)
            {
                pfac1 = p2dbout;
            }
            else
            {
                hc1 = Tool.CurveOnSurface(cv1.Arc(), fv, out ubid, out ubid);
                if (hc1 is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                pfac1 = hc1.Value(cv1.ParameterOnArc());
            }
            if (onsame && iFopArc == 2)
            {
                pfac2 = p2dbout;
            }
            else
            {
                hc2 = Tool.CurveOnSurface(cv2.Arc(), fv, out ubid, out ubid);
                if (hc2 is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                pfac2 = hc2.Value(cv2.ParameterOnArc());
            }
            if (fi1.LineIndex() != 0)
            {
                pfil1 = fi1.PCurveOnSurf().Value(fi1.Parameter(isfirst));
            }
            else
            {
                pfil1 = fi1.PCurveOnSurf().Value(fi1.Parameter(!isfirst));
            }
            if (fi2.LineIndex() != 0)
            {
                pfil2 = fi2.PCurveOnSurf().Value(fi2.Parameter(isfirst));
            }
            else
            {
                pfil2 = fi2.PCurveOnSurf().Value(fi2.Parameter(!isfirst));
            }
            if (onsame)
            {
                ChFi3d_Recale(bs, ref pfac1, ref pfac2, (iFadArc == 1));
            }

            pardeb[1] = pfil1.X;
            pardeb[2] = pfil1.Y;
            pardeb[3] = pfac1.X;
            pardeb[4] = pfac1.Y;
            parfin[1] = pfil2.X;
            parfin[2] = pfil2.Y;
            parfin[3] = pfac2.X;
            parfin[4] = pfac2.Y;

            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pfac1, pfac2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(bs, uu1, uu2, vv1, vv2);

            if (!ChFi3d_ComputeCurves(hGs, hBs, pardeb, parfin, ref cc, ref ps, ref pc, _tolapp3d, _tol2d, out tolreached))
            {
                throw new FailureException("OneCorner : echec calcul intersection");
            }

            udeb = cc.FirstParameter;
            ufin = cc.LastParameter;

            //  determine if the curve has an intersection with edge of sewing

            ChFi3d_Couture(fv, out couture, ref edgecouture);

            if (couture && !Tool.Degenerated(edgecouture))
            {

                // double Ubid,Vbid;
                Curve3D? c = Tool.Curve(edgecouture, out ubid, out vbid);
                TrimmedCurve3D? ctrim = new TrimmedCurve3D(c, ubid, vbid);
                CurveAdaptor cur1 = new(ctrim.BasisCurve);
                CurveAdaptor cur2 = new(cc);
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
                        if (Math.Abs(par2 - udeb) > Tol && Math.Abs(ufin - par2) > Tol)
                        {
                            Point3D p1 = ponc1.Value();
                            DsPoint tpoint = new(p1, Tol);
                            indpt = dStr.AddPoint(tpoint);
                            intcouture = true;
                            curv1 = new TrimmedCurve3D(cc, udeb, par2);
                            curv2 = new TrimmedCurve3D(cc, par2, ufin);
                            DsCurve tcurv1 = new(curv1, tolreached);
                            DsCurve tcurv2 = new(curv2, tolreached);
                            icurv1 = dStr.AddCurve(tcurv1);
                            icurv2 = dStr.AddCurve(tcurv2);
                        }
                    }
                }
            }
        }
        else
        { // (!inters)
            throw new MethodNotImplementedException("OneCorner : bouchon non ecrit");
        }
        int iShape = dStr.AddShape(fv);
        Orientation et = Forward;
        if (iFadArc == 1)
        {
            Explorer exp = new();
            for (exp.Init(fv.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(cv1.Arc()))
                {
                    et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(exp.Current().Orientation, cv1.TransitionOnArc()));
                    break;
                }
            }
        }
        else
        {
            Explorer exp = new();
            for (exp.Init(fv.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(cv2.Arc()))
                {
                    et = global::Kerf.Engine.Topology.TopAbs.Compose(exp.Current().Orientation, cv2.TransitionOnArc());
                    break;
                }
            }

            //
        }


        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cv1, dStr), isfirst, 1);
        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cv2, dStr), isfirst, 2);

        if (!intcouture)
        {
            // there is no intersection with the sewing edge
            // the curve Cc is stored in the stripe
            // the storage in the DS is not done by FILDS.

            DsCurve tc = new(cc, tolreached);
            iCurve = dStr.AddCurve(tc);
            DsSurfaceCurveInterference? interfc = 
                ChFi3d_FilCurveInDS(iCurve, iShape, pc, et);

            // 31/01/02 akm vvv : (OCC119) Prevent the builder from creating
            //                    intersecting fillets - they are bad.
            GInter intersector = new();
            CurveAdaptor2D corkPCurve = new(pc, udeb, ufin);

            // Take all the interferences with faces from all the stripes
            // and look if their pcurves intersect our cork pcurve.
            // Unfortunately, by this moment they do not exist in DStr.
            NCollection_List<Stripe>.Iterator strIt = new(_listStripe);
            for (; strIt.More(); strIt.Next())
            {
                Stripe? checkStripe = strIt.Value();
                NCollection_HSequence<SurfData>? seqData = 
                    checkStripe.SetOfSurfData();
                // Loop on parts of the stripe
                int iPart = 0;
                for (iPart = 1; iPart <= seqData.Length(); iPart++)
                {
                    SurfData? data = seqData.Value(iPart);
                    CurveAdaptor2D otherPCurve = new();
                    if (iShape == data.IndexOfS1())
                    {
                        Curve2D? pCurve = data.InterferenceOnS1().PCurveOnFace();
                        if (pCurve is null)
                        {
                            continue;
                        }

                        otherPCurve.Load(pCurve, data.InterferenceOnS1().FirstParameter(),
                                                              data.InterferenceOnS1().LastParameter());
                    }
                    else if (iShape == data.IndexOfS2())
                    {
                        Curve2D? pCurve = data.InterferenceOnS2().PCurveOnFace();
                        if (pCurve is null)
                        {
                            continue;
                        }

                        otherPCurve.Load(pCurve, data.InterferenceOnS2().FirstParameter(),
                                                              data.InterferenceOnS2().LastParameter());
                    }
                    else
                    {
                        // Normal case - no common surface
                        continue;
                    }
                    if (IsEqual(otherPCurve.LastParameter(), otherPCurve.FirstParameter()))
                    {
                        // Degenerates
                        continue;
                    }
                    intersector.Perform(corkPCurve, otherPCurve, _tol2d, global::Kerf.Engine.Foundation.Precision.PConfusion());
                    if (intersector.SegmentCount > 0 || intersector.PointCount > 0)
                    {
                        throw new NotDoneException("OneCorner : fillets have too big radiuses");
                    }
                }
            }
            NCollection_List<DsInterference>.Iterator iter = new(dStr.ChangeShapeInterferences(iShape));
            for (; iter.More(); iter.Next())
            {
                DsSurfaceCurveInterference? otherIntrf = 
                    (iter.Value() as DsSurfaceCurveInterference);
                // We need only interferences between cork face and curves
                // of intersection with another fillet surfaces
                if (otherIntrf is null)
                {
                    continue;
                }
                // Look if there is an intersection between pcurves
                TrimmedCurve3D? otherCur = 
                    (dStr.Curve(otherIntrf.Geometry()).Curve() as TrimmedCurve3D);
                if (otherCur is null)
                {
                    continue;
                }
                // OCCT's anOtherPCurve, renamed: C# cannot redeclare it in this scope
                CurveAdaptor2D otherPCurve2 = new(otherIntrf.PCurve(), otherCur.FirstParameter,
                                                        otherCur.LastParameter);
                intersector.Perform(corkPCurve, otherPCurve2, _tol2d, global::Kerf.Engine.Foundation.Precision.PConfusion());
                if (intersector.SegmentCount > 0 || intersector.PointCount > 0)
                {
                    throw new NotDoneException("OneCorner : fillets have too big radiuses");
                }
            }
            // 31/01/02 akm ^^^
            dStr.ChangeShapeInterferences(iShape).Append(interfc);
            //// modified by jgv, 26.03.02 for OCC32 ////
            CommonPoint[] cv = new CommonPoint[2];
            cv[0] = cv1;
            cv[1] = cv2;
            for (int i = 0; i < 2; i++)
            {
                if (cv[i].IsOnArc && ChFi3d_IsPseudoSeam(cv[i].Arc(), fv))
                {
                    Point2D pfac1 = new(), pcF = new(), pcL = new();
                    Vector2D derPc = new(), derHc = new();
                    double first = 0.0, last = 0.0, prm1 = 0.0, prm2 = 0.0;
                    bool onfirst = false, firstToPar = false;
                    Curve2D? hc = Tool.CurveOnSurface(cv[i].Arc(), fv, out first, out last);
                    if (hc is null)
                    {
                        throw new ConstructionErrorException("Failed to get p-curve of edge");
                    }
                    pfac1 = hc.Value(cv[i].ParameterOnArc());
                    pcF = pc.Value(udeb);
                    pcL = pc.Value(ufin);
                    onfirst = pfac1.DistanceTo(pcF) < pfac1.DistanceTo(pcL);
                    if (onfirst)
                    {
                        pc.D1(udeb, out pcF, out derPc);
                    }
                    else
                    {
                        pc.D1(ufin, out pcL, out derPc);
                        derPc = derPc.Reversed();
                    }
                    hc.D1(cv[i].ParameterOnArc(), out pfac1, out derHc);

                    if (derHc.Dot(derPc) > 0.0)
                    {
                        prm1 = cv[i].ParameterOnArc();
                        prm2 = last;
                        firstToPar = false;
                    }
                    else
                    {
                        prm1 = first;
                        prm2 = cv[i].ParameterOnArc();
                        firstToPar = true;
                    }
                    Curve3D? ct = Tool.Curve(cv[i].Arc(), out first, out last);
                    ct = new TrimmedCurve3D(ct, prm1, prm2);
                    double toled = Tool.Tolerance(cv[i].Arc());
                    DsCurve tcurv = new(ct, toled);
                    DsCurvePointInterference? interfp1 = null, interfp2 = null;
                    int indcurv = 0;
                    indcurv = dStr.AddCurve(tcurv);
                    int indpoint = (isfirst) ? stripe.IndexFirstPointOnS1() : stripe.IndexLastPointOnS1();
                    int indvertex = dStr.AddShape(vtx);
                    if (firstToPar)
                    {
                        interfp1 = ChFi3d_FilPointInDS(Forward, indcurv, indvertex, prm1, true);
                        interfp2 = ChFi3d_FilPointInDS(Reversed, indcurv, indpoint, prm2, false);
                    }
                    else
                    {
                        interfp1 = ChFi3d_FilPointInDS(Forward, indcurv, indpoint, prm1, false);
                        interfp2 = ChFi3d_FilPointInDS(Reversed, indcurv, indvertex, prm2, true);
                    }
                    dStr.ChangeCurveInterferences(indcurv).Append(interfp1);
                    dStr.ChangeCurveInterferences(indcurv).Append(interfp2);
                    int indface = dStr.AddShape(fv);
                    interfc = ChFi3d_FilCurveInDS(indcurv, indface, hc, cv[i].Arc().Orientation);
                    dStr.ChangeShapeInterferences(indface).Append(interfc);
                    Edge localEdge = cv[i].Arc();
                    localEdge.Reverse();
                    Curve2D? hcR = Tool.CurveOnSurface(localEdge, fv, out first, out last);
                    if (hcR is null)
                    {
                        throw new ConstructionErrorException("Failed to get p-curve of edge");
                    }
                    interfc = ChFi3d_FilCurveInDS(indcurv, indface, hcR, localEdge.Orientation);
                    dStr.ChangeShapeInterferences(indface).Append(interfc);
                    // modify degenerated edge
                    bool degenExist = false;
                    Edge edeg = new();
                    Explorer explo = new(fv, ShapeType.Edge);
                    for (; explo.More(); explo.Next())
                    {
                        Edge ecur = ShapeCasts.Edge(explo.Current());
                        if (Tool.Degenerated(ecur))
                        {
                            Vertex vf = new(), vl = new();
                            Exploration.Vertices(ecur, out vf, out vl);
                            if (vf.IsSame(vtx) || vl.IsSame(vtx))
                            {
                                degenExist = true;
                                edeg = ecur;
                                break;
                            }
                        }
                    }
                    if (degenExist)
                    {
                        double fd = 0.0, ld = 0.0;
                        Curve2D? cd = Tool.CurveOnSurface(edeg, fv, out fd, out ld);
                        if (cd is null)
                        {
                            throw new ConstructionErrorException("Failed to get p-curve of edge");
                        }
                        TrimmedCurve2D? tCd = (cd as TrimmedCurve2D);
                        if (tCd is not null)
                        {
                            cd = tCd.BasisCurve;
                        }
                        Point2D p2d = (firstToPar) ? hc.Value(first) : hc.Value(last);
                        ProjectPointOnCurve2D projector = new(p2d, cd);
                        double par = projector.LowerDistanceParameter();
                        int ideg = dStr.AddShape(edeg);
                        // clang-format off
                        Orientation ori = (par < fd)? Forward : Reversed;//if par<fd => par>ld
                        // clang-format on
                        interfp1 = ChFi3d_FilPointInDS(ori, ideg, indvertex, par, true);
                        dStr.ChangeShapeInterferences(ideg).Append(interfp1);
                    }
                }
            }
            /////////////////////////////////////////////
            stripe.ChangePCurve(isfirst) = ps;
            stripe.SetCurve(iCurve, isfirst);
            stripe.SetParameters(isfirst, udeb, ufin);
        }
        else
        {
            // curves curv1 are curv2 stored in the DS
            // these curves will not be reconstructed by FILDS as
            // one places stripe->InDS(isfirst);

            // interferences of curv1 and curv2 on Fv
            ComputeCurve2d(curv1, ref fv, ref c2d1);
            DsSurfaceCurveInterference? interFv = null;
            interFv = ChFi3d_FilCurveInDS(icurv1, iShape, c2d1, et);
            dStr.ChangeShapeInterferences(iShape).Append(interFv);
            ComputeCurve2d(curv2, ref fv, ref c2d2);
            interFv = ChFi3d_FilCurveInDS(icurv2, iShape, c2d2, et);
            dStr.ChangeShapeInterferences(iShape).Append(interFv);
            // interferences of curv1 and curv2 on Isurf
            if (Fd.Orientation() == fv.Orientation)
            {
                et = global::Kerf.Engine.Topology.TopAbs.Reverse(et);
            }
            c2d1 = new TrimmedCurve2D(ps, udeb, par2);
            interFv = ChFi3d_FilCurveInDS(icurv1, isurf, c2d1, et);
            dStr.ChangeSurfaceInterferences(isurf).Append(interFv);
            c2d2 = new TrimmedCurve2D(ps, par2, ufin);
            interFv = ChFi3d_FilCurveInDS(icurv2, isurf, c2d2, et);
            dStr.ChangeSurfaceInterferences(isurf).Append(interFv);

            // limitation of the sewing edge
            int iarc = dStr.AddShape(edgecouture);
            DsCurvePointInterference? interfedge = null;
            Orientation ori = default;
            Vertex vdeb = new(), vfin = new();
            vdeb = Exploration.FirstVertex(edgecouture);
            vfin = Exploration.LastVertex(edgecouture);
            double pard = 0.0, parf = 0.0;
            pard = Tool.Parameter(vdeb, edgecouture);
            parf = Tool.Parameter(vfin, edgecouture);
            if (Math.Abs(par1 - pard) < Math.Abs(parf - par1))
            {
                ori = Forward;
            }
            else
            {
                ori = Reversed;
            }
            interfedge = ChFi3d_FilPointInDS(ori, iarc, indpt, par1);
            dStr.ChangeShapeInterferences(iarc).Append(interfedge);

            // creation of CurveInterferences from Icurv1 and Icurv2
            stripe.InDS(isfirst);
            int ind1 = stripe.IndexPoint(isfirst, 1);
            int ind2 = stripe.IndexPoint(isfirst, 2);
            DsCurvePointInterference? interfprol = 
                ChFi3d_FilPointInDS(Forward, icurv1, ind1, udeb);
            dStr.ChangeCurveInterferences(icurv1).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Reversed, icurv1, indpt, par2);
            dStr.ChangeCurveInterferences(icurv1).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Forward, icurv2, indpt, par2);
            dStr.ChangeCurveInterferences(icurv2).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Reversed, icurv2, ind2, ufin);
            dStr.ChangeCurveInterferences(icurv2).Append(interfprol);
        }

        ChFi3d_EnlargeBox(hBs, pc, udeb, ufin, ref box1, ref box2);

        if (onsame && inters)
        {
    // VARIANT 1:
    // A small missing end of curve is added for the extension
    // of the face at end and the limitation of the opposing face.

    //   VARIANT 2 : extend Arcprol, not create new small edge
    //   To do: modify for intcouture
    // (OCCT defines VARIANT1: only its branches are ported)

            // First of all the points are cut with the edge of the spine.
            int iArcspine = dStr.AddShape(arcspine);
            int iVtx = dStr.AddShape(vtx);
            Orientation oVtx = Forward;
            for (ex.Init(arcspine.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (vtx.IsSame(ex.Current()))
                {
                    oVtx = ex.Current().Orientation;
                    break;
                }
            }
            oVtx = global::Kerf.Engine.Topology.TopAbs.Reverse(oVtx);
            double parVtx = Tool.Parameter(vtx, arcspine);
            DsCurvePointInterference? interfv = 
                ChFi3d_FilVertexInDS(oVtx, iArcspine, iVtx, parVtx);
            dStr.ChangeShapeInterferences(iArcspine).Append(interfv);

            // Now the missing curves are constructed.
            Vertex v2 = new();
            for (ex.Init(arcprol.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (vtx.IsSame(ex.Current()))
                {
                    oVtx = ex.Current().Orientation;
                }
                else
                {
                    v2 = ShapeCasts.Vertex(ex.Current());
                }
            }

            Curve2D? hc = null;
            parVtx = Tool.Parameter(vtx, arcprol); // VARIANT1
            FaceInterference fiop = Fd.Interference(iFopArc);
            Point2D pop1 = new(), pop2 = new(), pv1 = new(), pv2 = new();
            hc = Tool.CurveOnSurface(arcprol, fop, out ubid, out ubid);
            if (hc is null)
            {
                throw new ConstructionErrorException("Failed to get p-curve of edge");
            }
            pop1 = hc.Value(parVtx);
            pop2 = fiop.PCurveOnFace().Value(fiop.Parameter(isfirst));
            hc = Tool.CurveOnSurface(arcprol, fv, out ubid, out ubid);
            if (hc is null)
            {
                throw new ConstructionErrorException("Failed to get p-curve of edge");
            }
            pv1 = hc.Value(parVtx);
            pv2 = p2dbout;
            ChFi3d_Recale(bs, ref pv1, ref pv2, true);
            NCollection_Array1<double> pardeb = new(1, 4), parfin = new(1, 4);
            pardeb[1] = pop1.X;
            pardeb[2] = pop1.Y;
            pardeb[3] = pv1.X;
            pardeb[4] = pv1.Y;
            parfin[1] = pop2.X;
            parfin[2] = pop2.Y;
            parfin[3] = pv2.X;
            parfin[4] = pv2.Y;
            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pv1, pv2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(bs, uu1, uu2, vv1, vv2);
            ChFi3d_Boite(pop1, pop2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(bop, uu1, uu2, vv1, vv2);

            Curve3D? zob3d = null;
            Curve2D? zob2dop = null, zob2dv = null;
            // double tolreached;
            if (!ChFi3d_ComputeCurves(hBop, hBs, pardeb, parfin, ref zob3d, ref zob2dop, ref zob2dv, _tolapp3d, _tol2d,
                                                                out tolreached))
            {
                throw new FailureException("OneCorner : echec calcul intersection");
            }

            udeb = zob3d.FirstParameter;
            ufin = zob3d.LastParameter;
            DsCurve zob = new(zob3d, tolreached);
            int iZob = dStr.AddCurve(zob);

            // it is determined if Fop has an edge of sewing
            // it is determined if the curve has an intersection with the edge of sewing

            // TopoDS_Edge edgecouture;
            // bool couture;
            ChFi3d_Couture(fop, out couture, ref edgecouture);

            if (couture && !Tool.Degenerated(edgecouture))
            {
                LibraryMakeEdge bedge = new(zob3d);
                Edge edg = bedge.Edge();
                DistanceExtCC extCC = new(edgecouture, edg);
                if (extCC.IsDone && extCC.ExtCount != 0)
                {
                    for (int i = 1; i <= extCC.ExtCount && !intcouture; i++)
                    {
                        if (extCC.SquareDistance(i) <= 1.0e-8)
                        {
                            par1 = extCC.ParameterOnE1(i);
                            par2 = extCC.ParameterOnE2(i);
                            Point3D p1 = extCC.PointOnE1(i);
                            DsPoint tpoint = new(p1, 1.0e-4);
                            indpt = dStr.AddPoint(tpoint);
                            intcouture = true;
                            curv1 = new TrimmedCurve3D(zob3d, udeb, par2);
                            curv2 = new TrimmedCurve3D(zob3d, par2, ufin);
                            DsCurve tcurv1 = new(curv1, tolreached);
                            DsCurve tcurv2 = new(curv2, tolreached);
                            icurv1 = dStr.AddCurve(tcurv1);
                            icurv2 = dStr.AddCurve(tcurv2);
                        }
                    }
                }
            }
            if (intcouture)
            {

                // interference of curv1 and curv2 on Ishape
                et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(oVtx, oArcprolv));
                ComputeCurve2d(curv1, ref fop, ref c2d1);
                DsSurfaceCurveInterference? interFv = 
                    ChFi3d_FilCurveInDS(icurv1, iShape, /*zob2dv*/ c2d1, et);
                dStr.ChangeShapeInterferences(iShape).Append(interFv);
                ComputeCurve2d(curv2, ref fop, ref c2d2);
                interFv = ChFi3d_FilCurveInDS(icurv2, iShape, /*zob2dv*/ c2d2, et);
                dStr.ChangeShapeInterferences(iShape).Append(interFv);

                // limitation of the sewing edge
                int iarc = dStr.AddShape(edgecouture);
                DsCurvePointInterference? interfedge = null;
                Orientation ori = default;
                Vertex vdeb = new(), vfin = new();
                vdeb = Exploration.FirstVertex(edgecouture);
                vfin = Exploration.LastVertex(edgecouture);
                double pard = 0.0, parf = 0.0;
                pard = Tool.Parameter(vdeb, edgecouture);
                parf = Tool.Parameter(vfin, edgecouture);
                if (Math.Abs(par1 - pard) < Math.Abs(parf - par1))
                {
                    ori = Reversed;
                }
                else
                {
                    ori = Forward;
                }
                interfedge = ChFi3d_FilPointInDS(ori, iarc, indpt, par1);
                dStr.ChangeShapeInterferences(iarc).Append(interfedge);

                //  interference of curv1 and curv2 on Iop
                int iop = dStr.AddShape(fop);
                et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(oVtx, oArcprolop));
                DsSurfaceCurveInterference? interfop = null;
                ComputeCurve2d(curv1, ref fop, ref c2d1);
                interfop = ChFi3d_FilCurveInDS(icurv1, iop, c2d1, et);
                dStr.ChangeShapeInterferences(iop).Append(interfop);
                ComputeCurve2d(curv2, ref fop, ref c2d2);
                interfop = ChFi3d_FilCurveInDS(icurv2, iop, c2d2, et);
                dStr.ChangeShapeInterferences(iop).Append(interfop);
                DsCurvePointInterference? interfprol = 
                    ChFi3d_FilVertexInDS(Forward, icurv1, iVtx, udeb);
                dStr.ChangeCurveInterferences(icurv1).Append(interfprol);
                interfprol = ChFi3d_FilPointInDS(Reversed, icurv1, indpt, par2);
                dStr.ChangeCurveInterferences(icurv1).Append(interfprol);
                int icc = stripe.IndexPoint(isfirst, iFopArc);
                interfprol = ChFi3d_FilPointInDS(Forward, icurv2, indpt, par2);
                dStr.ChangeCurveInterferences(icurv2).Append(interfprol);
                interfprol = ChFi3d_FilPointInDS(Reversed, icurv2, icc, ufin);
                dStr.ChangeCurveInterferences(icurv2).Append(interfprol);
            }
            else
            {
                et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(oVtx, oArcprolv));
                DsSurfaceCurveInterference? interFv = 
                    ChFi3d_FilCurveInDS(iZob, iShape, zob2dv, et);
                dStr.ChangeShapeInterferences(iShape).Append(interFv);
                et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(oVtx, oArcprolop));
                int iop = dStr.AddShape(fop);
                DsSurfaceCurveInterference? interfop = 
                    ChFi3d_FilCurveInDS(iZob, iop, zob2dop, et);
                dStr.ChangeShapeInterferences(iop).Append(interfop);
                DsCurvePointInterference? interfprol = null;
                interfprol = ChFi3d_FilVertexInDS(Forward, iZob, iVtx, udeb); // VARIANT1
                dStr.ChangeCurveInterferences(iZob).Append(interfprol);
                int icc = stripe.IndexPoint(isfirst, iFopArc);
                interfprol = ChFi3d_FilPointInDS(Reversed, iZob, icc, ufin);
                dStr.ChangeCurveInterferences(iZob).Append(interfprol);
                { // VARIANT1
                    if (iFopArc == 1)
                    {
                        box1.Add(zob3d.Value(ufin));
                    }
                    else
                    {
                        box2.Add(zob3d.Value(ufin));
                    }
                }
            }
        }
        ChFi3d_EnlargeBox(dStr, stripe, Fd, ref box1, ref box2, isfirst);
        if (cv1.IsOnArc)
        {
            ChFi3d_EnlargeBox(cv1.Arc(), _efMap[cv1.Arc()], cv1.ParameterOnArc(), ref box1);
        }
        if (cv2.IsOnArc)
        {
            ChFi3d_EnlargeBox(cv2.Arc(), _efMap[cv2.Arc()], cv2.ParameterOnArc(), ref box2);
        }
        if (!cv1.IsVertex)
        {
            ChFi3d_SetPointTolerance(dStr, box1, stripe.IndexPoint(isfirst, 1));
        }
        if (!cv2.IsVertex)
        {
            ChFi3d_SetPointTolerance(dStr, box2, stripe.IndexPoint(isfirst, 2));
        }

    }

    //=======================================================================
    // function : ChercheFace
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

    private void PerformIntersectionAtEnd(int index)
    {

        // intersection at end of fillet with at least two faces
        // process the following cases:
        // - top has n (n>3) adjacent edges
        // - top has 3 edges and fillet on one of edges touches
        //   more than one face


        DsDataStructure dStr = _ds.ChangeDS();
        int nn = 15;
        NCollection_List<Stripe>.Iterator it = new();
        it.Initialize(_vDataMap[index]);
        Stripe? stripe = it.Value();
        Spine? spine = stripe.Spine();
        NCollection_Sequence<SurfData> seqFil = 
            stripe.ChangeSetOfSurfData();
        Vertex vtx = _vDataMap.FindKey(index);
        int sens = 0, num, num1;
        bool couture = false, isfirst;
        // int sense;
        Edge edgelibre1 = new(), edgelibre2 = new(), edgeSpine = new();
        bool bordlibre = false;
        // determine the number of faces and edges
        NCollection_Array1<Shape> tabedg = new(0, nn);
        Face f1 = new(), f2 = new();
        int nface = ChFi3d_nbface(_vfMap[vtx]);
        NCollection_List<Shape>.Iterator itF = new();
        int nbarete = 0;
        nbarete = ChFi3d_NbNotDegeneratedEdges(vtx, _veMap);
        ChFi3d_ChercheBordsLibres(_veMap, vtx, out bordlibre, ref edgelibre1, ref edgelibre2);
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
        for (itF.Initialize(_vfMap[vtx]); itF.More() && !couture; itF.Next())
        {
            Face fcur = ShapeCasts.Face(itF.Value());
            ChFi3d_CoutureOnVertex(fcur, vtx, out couture, ref edgecouture);
            if (couture)
            {
                facecouture = fcur;
            }
        }
        // it is determined if one of edges adjacent to the fillet is regular
        bool reg1 = false, reg2 = false;
        Edge ecur = new(), eadj1 = new(), eadj2 = new();
        Face fga = new(), fdr = new();
        Vertex vbid1 = new();
        int nbsurf = 0, nbedge = 0;
        reg1 = false;
        reg2 = false;
        nbsurf = seqFil.Length();
        nbedge = spine.EdgeCount;
        num = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
        isfirst = (sens == 1);
        State state = default;
        if (isfirst)
        {
            edgeSpine = spine.Edges(1);
            num1 = num + 1;
            state = spine.FirstStatus();
        }
        else
        {
            edgeSpine = spine.Edges(nbedge);
            num1 = num - 1;
            state = spine.LastStatus();
        }
        if (nbsurf != nbedge && nbsurf != 1)
        {
            ChFi3d_edge_common_faces(_efMap[edgeSpine], out f1, out f2);
            if (f1.IsSame(facecouture))
            {
                eadj1 = edgecouture;
            }
            else
            {
                ChFi3d_cherche_element(vtx, edgeSpine, f1, ref eadj1, ref vbid1);
            }
            ChFi3d_edge_common_faces(_efMap[eadj1], out fga, out fdr);
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:57:32 2001 Begin
            //  reg1=BRep_Tool::Continuity(Eadj1,Fga,Fdr)!=GeomAbs_C0;
            reg1 = global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(eadj1, fga, fdr);
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:57:33 2001 End
            if (f2.IsSame(facecouture))
            {
                eadj2 = edgecouture;
            }
            else
            {
                ChFi3d_cherche_element(vtx, edgeSpine, f2, ref eadj2, ref vbid1);
            }
            ChFi3d_edge_common_faces(_efMap[eadj2], out fga, out fdr);
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:58:22 2001 Begin
            //  reg2=BRep_Tool::Continuity(Eadj2,Fga,Fdr)!=GeomAbs_C0;
            reg2 = global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(eadj2, fga, fdr);
            //  Modified by Sergey KHROMOV - Fri Dec 21 17:58:24 2001 End

            // two faces common to the edge are found
            if (reg1 || reg2)
            {
                bool compoint1 = false;
                bool compoint2 = false;
                CommonPoint cp1 = new(), cp2 = new();
                cp1 = seqFil[num1].ChangeVertex(isfirst, 1);
                cp2 = seqFil[num1].ChangeVertex(isfirst, 2);
                if (cp1.IsOnArc)
                {
                    if (cp1.Arc().IsSame(eadj1) || cp1.Arc().IsSame(eadj2))
                    {
                        compoint1 = true;
                    }
                }
                if (cp2.IsOnArc)
                {
                    if (cp2.Arc().IsSame(eadj1) || cp2.Arc().IsSame(eadj2))
                    {
                        compoint2 = true;
                    }
                }
                if (compoint1 && compoint2)
                {
                    seqFil.Remove(num);
                    num = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
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
        SurfData? fd = seqFil.ChangeValue(num);
        ref CommonPoint cv1 = ref fd.ChangeVertex(isfirst, 1);
        ref CommonPoint cv2 = ref fd.ChangeVertex(isfirst, 2);
        bool onecorner = false;
        if (FindFace(vtx, cv1, cv2, ref face))
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
            if (MoreSurfdata(index))
            {
                PerformMoreSurfdata(index);
                return;
            }
        }
        if (!onecorner && (reg1 || reg2) && !couture && state != State.OnSame)
        {
            PerformMoreThreeCorner(index, 1);
            return;
        }
        SurfaceAdaptor? hGs = ChFi3d_BoundSurf(dStr, fd, 1, 2);
        FaceInterference fi1 = fd.InterferenceOnS1();
        FaceInterference fi2 = fd.InterferenceOnS2();
        SurfaceAdaptor gs = hGs;
        FaceSurfaceAdaptor? hBs = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor bs = hBs;
        Curve3D? cc = null;
        Curve2D? pc = null, ps = null;
        double Ubid = 0.0, Vbid = 0.0;
        Orientation orsurfdata = default;
        orsurfdata = fd.Orientation();
        int isurfPrev = 0, isurf = fd.Surf();
        SurfData? sDprev = null;
        if (num1 > 0 && num1 <= seqFil.Length())
        {
            sDprev = seqFil[num1];
            isurfPrev = sDprev.Surf();
        }
        // calculate the orientation of curves at end

        double tolpt = 1.0e-4;
        double tolreached = 0.0;
        Orientation orcourbe = default, orface = default, orien = default;

        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cv1, dStr), isfirst, 1);
        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cv2, dStr), isfirst, 2);

        //  gp_Pnt p3d;
        //  gp_Pnt2d p2d;
        double dist = 0.0;
        int ishape1 = fd.IndexOfS1();
        Orientation trafil1 = Forward;
        if (ishape1 != 0)
        {
            if (ishape1 > 0)
            {
                trafil1 = dStr.Shape(ishape1).Orientation;
            }
            trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(trafil1, fd.Orientation());

            trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(fi1.Transition()), trafil1);
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
        Point2D P2d1 = fi1.PCurveOnSurf().Value(fi1.Parameter(isfirst));
        Point2D P2d2 = fi1.PCurveOnSurf().Value(fi1.Parameter(!isfirst));
        Point3D aP1 = new(), aP2 = new();
        hGs.D0(P2d1.X, P2d1.Y, out aP1);
        hGs.D0(P2d2.X, P2d2.Y, out aP2);
        double fi1Length = aP1.DistanceTo(aP2);
        //  double eps = Precision::Confusion();
        checkShrink = (fi1Length <= global::Kerf.Engine.Foundation.Precision.Confusion());

        Point2D p2d3 = fi2.PCurveOnSurf().Value(fi2.Parameter(isfirst));
        Point2D p2d4 = fi2.PCurveOnSurf().Value(fi2.Parameter(!isfirst));
        hGs.D0(p2d3.X, p2d3.Y, out aP1);
        hGs.D0(p2d4.X, p2d4.Y, out aP2);
        double fi2Length = aP1.DistanceTo(aP2);
        checkShrink = checkShrink || (fi2Length <= global::Kerf.Engine.Foundation.Precision.Confusion());

        if (checkShrink)
        {
            if (Math.Abs(P2d2.Y - p2d4.Y) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                isUShrink = false;
                checkShrParam = P2d2.Y;
            }
            else if (Math.Abs(P2d2.X - p2d4.X) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                isUShrink = true;
                checkShrParam = P2d2.X;
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
        Edge e1 = new(), e2 = new(), e = new(), ei = new(), edgesau = new();
        Edge[] edge = new Edge[nn];
        Face[] Face = new Face[nn];
        for (int k = 0; k < nn; k++)
        {
            edge[k] = new();
            Face[k] = new();
        }
        Face facesau = new();
        bool oneintersection1 = false;
        bool oneintersection2 = false;
        Face f = new(), f3 = new();
        Vertex v1 = new(), V2 = new(), v = new(), vfin = new();
        bool findonf1 = false, findonf2 = false;
        NCollection_List<Shape>.Iterator it3 = new();
        f1 = ShapeCasts.Face(dStr.Shape(fd.IndexOfS1()));
        f2 = ShapeCasts.Face(dStr.Shape(fd.IndexOfS2()));
        f3 = f1;
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
        if (!cv1.IsOnArc || !cv2.IsOnArc)
        {
            PerformMoreThreeCorner(index, 1);
            return;
        }

        edge[0] = cv1.Arc();
        edge[nbface] = cv2.Arc();
        tabedg.SetValue(0, edge[0]);
        tabedg.SetValue(nbface, edge[nbface]);
        // processing of a fillet arriving on a vertex
        // edge contained in CV.Arc is not inevitably good
        // the edge concerned by the intersection is found

        double dist1 = 0.0, dist2 = 0.0;
        if (cv1.IsVertex)
        {
            trouve = false;
            /*TopoDS_Vertex */ v = cv1.Vertex();
            for (it3.Initialize(_veMap[v]); it3.More() && !trouve; it3.Next())
            {
                e = ShapeCasts.Edge(it3.Value());
                if (!e.IsSame(edge[0]) && (ContainE(f1, e)))
                {
                    trouve = true;
                }
            }
            Vertex vt = new(), v3 = new(), v4 = new();
            v1 = Exploration.FirstVertex(edge[0]);
            V2 = Exploration.LastVertex(edge[0]);
            if (v.IsSame(v1))
            {
                vt = V2;
            }
            else
            {
                vt = v1;
            }
            dist1 = (Tool.Pnt(vt)).DistanceTo(Tool.Pnt(vtx));
            v3 = Exploration.FirstVertex(e);
            v4 = Exploration.LastVertex(e);
            if (v.IsSame(v3))
            {
                vt = v4;
            }
            else
            {
                vt = v3;
            }
            dist2 = (Tool.Pnt(vt)).DistanceTo(Tool.Pnt(vtx));
            if (dist2 < dist1)
            {
                edge[0] = e;
                Orientation ori = default;
                if (V2.IsSame(v3) || v1.IsSame(v4))
                {
                    ori = cv1.TransitionOnArc();
                }
                else
                {
                    ori = global::Kerf.Engine.Topology.TopAbs.Reverse(cv1.TransitionOnArc());
                }
                double par = Tool.Parameter(v, edge[0]);
                double tol = cv1.Tolerance();
                cv1.SetArc(tol, edge[0], par, ori);
            }
        }

        if (cv2.IsVertex)
        {
            trouve = false;
            /*TopoDS_Vertex*/ v = cv2.Vertex();
            for (it3.Initialize(_veMap[v]); it3.More() && !trouve; it3.Next())
            {
                e = ShapeCasts.Edge(it3.Value());
                if (!e.IsSame(edge[2]) && (ContainE(f2, e)))
                {
                    trouve = true;
                }
            }
            Vertex vt = new(), v3 = new(), v4 = new();
            v1 = Exploration.FirstVertex(edge[2]);
            V2 = Exploration.LastVertex(edge[2]);
            if (v.IsSame(v1))
            {
                vt = V2;
            }
            else
            {
                vt = v1;
            }
            dist1 = (Tool.Pnt(vt)).DistanceTo(Tool.Pnt(vtx));
            v3 = Exploration.FirstVertex(e);
            v4 = Exploration.LastVertex(e);
            if (v.IsSame(v3))
            {
                vt = v4;
            }
            else
            {
                vt = v3;
            }
            dist2 = (Tool.Pnt(vt)).DistanceTo(Tool.Pnt(vtx));
            if (dist2 < dist1)
            {
                edge[2] = e;
                Orientation ori = default;
                if (V2.IsSame(v3) || v1.IsSame(v4))
                {
                    ori = cv2.TransitionOnArc();
                }
                else
                {
                    ori = global::Kerf.Engine.Topology.TopAbs.Reverse(cv2.TransitionOnArc());
                }
                double par = Tool.Parameter(v, edge[2]);
                double tol = cv2.Tolerance();
                cv2.SetArc(tol, edge[2], par, ori);
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
                if (cv1.IsVertex)
                {
                    findonf1 = true;
                }
                if (cv2.IsVertex)
                {
                    findonf2 = true;
                }
                if (!findonf1)
                {
                    IndexedSet<Shape> mapV = new(ShapeHasher.Instance);
                    Exploration.MapShapes(edge[0], ShapeType.Vertex, mapV);
                    if (mapV.Count == 2)
                    {
                        if (!mapV[0].IsSame(vtx) && !mapV[1].IsSame(vtx))
                        {
                            findonf1 = true;
                        }
                    }
                }
                if (!findonf2)
                {
                    IndexedSet<Shape> mapV = new(ShapeHasher.Instance);
                    Exploration.MapShapes(edge[2], ShapeType.Vertex, mapV);
                    if (mapV.Count == 2)
                    {
                        if (!mapV[0].IsSame(vtx) && !mapV[1].IsSame(vtx))
                        {
                            findonf2 = true;
                        }
                    }
                }

                // detect and process OnSame situatuation
                if (state == State.OnSame)
                {
                    Edge[] threeE = [new(), new(), new()];

                    ChFi3d_cherche_element(vtx, edgeSpine, f1, ref threeE[0], ref V2);
                    ChFi3d_cherche_element(vtx, edgeSpine, f2, ref threeE[1], ref V2);
                    threeE[2] = edgeSpine;
                    if (ChFi3d_EdgeState(threeE, _efMap) == State.OnSame)
                    {
                        isOnSame1 = true;
                        nb = 1;
                        edge[0] = threeE[0];
                        ChFi3d_cherche_face1(_efMap[edge[0]], f1, ref Face[0]);
                        if (findonf2)
                        {
                            findonf1 = true; // not to look for Face[0] again
                        }
                        else
                        {
                            edge[1] = cv2.Arc();
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
                    if (cv1.TransitionOnArc() == Forward)
                    {
                        v1 = Exploration.FirstVertex(cv1.Arc());
                    }
                    else
                    {
                        v1 = Exploration.LastVertex(cv1.Arc());
                    }
                    ChFi3d_cherche_face1(_efMap[cv1.Arc()], f1, ref Face[0]);
                    nb = 1;
                    ei = edge[0];
                    while (!v1.IsSame(vtx))
                    {
                        ChFi3d_cherche_element(v1, ei, f1, ref e, ref V2);
                        v1 = V2;
                        ei = e;
                        ChFi3d_cherche_face1(_efMap[e], f1, ref Face[nb]);
                        cherche_edge1(Face[nb - 1], Face[nb], ref edge[nb]);
                        nb++;
                        if (nb >= nn)
                        {
                            throw new FailureException("IntersectionAtEnd : the max number of faces reached");
                        }
                    }
                    if (!findonf2)
                    {
                        edge[nb] = cv2.Arc();
                    }
                }
                if (findonf2 && !isOnSame2)
                {
                    if (!findonf1)
                    {
                        nb = 1;
                    }
                    v1 = vtx;
                    if (cv2.TransitionOnArc() == Forward)
                    {
                        vfin = Exploration.LastVertex(cv2.Arc());
                    }
                    else
                    {
                        vfin = Exploration.FirstVertex(cv2.Arc());
                    }
                    if (!findonf1)
                    {
                        ChFi3d_cherche_face1(_efMap[cv1.Arc()], f1, ref Face[nb - 1]);
                    }
                    ChFi3d_cherche_element(v1, edgeSpine, f2, ref e, ref V2);
                    ei = e;
                    v1 = V2;
                    while (!v1.IsSame(vfin))
                    {
                        ChFi3d_cherche_element(v1, ei, f2, ref e, ref V2);
                        ei = e;
                        v1 = V2;
                        ChFi3d_cherche_face1(_efMap[e], f2, ref Face[nb]);
                        cherche_edge1(Face[nb - 1], Face[nb], ref edge[nb]);
                        nb++;
                        if (nb >= nn)
                        {
                            throw new FailureException("IntersectionAtEnd : the max number of faces reached");
                        }
                    }
                    edge[nb] = cv2.Arc();
                }
                if (isOnSame2)
                {
                    cherche_edge1(Face[nb - 1], f2, ref edge[nb]);
                    Face[nb] = f2;
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
                for (ex.Init(cv1.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (vtx.IsSame(ex.Current()))
                    {
                        possible1 = true;
                    }
                }
                for (ex.Init(cv2.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (vtx.IsSame(ex.Current()))
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
                            f3 = Face[nb - 2];
                        }
                        Face[nb - 1] = f3;
                        if (cv1.Arc().IsSame(edgelibre1))
                        {
                            ChercheFace(_vfMap[vtx], edgelibre2, f1, f2, f3, ref Face[nb - 1]);
                        }
                        else if (cv1.Arc().IsSame(edgelibre2))
                        {
                            ChercheFace(_vfMap[vtx], edgelibre1, f1, f2, f3, ref Face[nb - 1]);
                        }
                        else
                        {
                            ChercheFace(_vfMap[vtx], edge[nb - 1], f1, f2, f3, ref Face[nb - 1]);
                        }
                        ChFi3d_cherche_edge(vtx, tabedg, Face[nb - 1], ref edge[nb], ref v);
                        tabedg.SetValue(nb, edge[nb]);
                        if (edge[nb].IsSame(cv2.Arc()))
                        {
                            trouve = true;
                        }
                    }
                    nbface = nb;
                }
                else
                {
                    IntersectMoreCorner(index);
                    return;
                }
                if (nbarete == 4)
                {
                    // if two consecutive edges are G1 there is only one face of intersection
                    double ang1 = 0.0;
                    Vertex vcom = new();
                    trouve = false;
                    ChFi3d_cherche_vertex(edge[0], edge[1], ref vcom, out trouve);
                    if (vcom.IsSame(vtx))
                    {
                        ang1 = ChFi3d_AngleEdge(vtx, edge[0], edge[1]);
                    }
                    if (Math.Abs(ang1 - M_PI) < 0.01)
                    {
                        oneintersection1 = true;
                        facesau = Face[0];
                        edgesau = edge[1];
                        Face[0] = Face[1];
                        edge[1] = edge[2];
                        nbface = 1;
                    }

                    if (!oneintersection1)
                    {
                        trouve = false;
                        ChFi3d_cherche_vertex(edge[1], edge[2], ref vcom, out trouve);
                        if (vcom.IsSame(vtx))
                        {
                            ang1 = ChFi3d_AngleEdge(vtx, edge[1], edge[2]);
                        }
                        if (Math.Abs(ang1 - M_PI) < 0.01)
                        {
                            oneintersection2 = true;
                            facesau = Face[1];
                            edgesau = edge[1];
                            edge[1] = edge[2];
                            nbface = 1;
                        }
                    }
                }
                else if (nbarete == 5)
                {
                    // pro15368
                    //  Modified by Sergey KHROMOV - Fri Dec 21 18:07:43 2001 End
                    bool isTangent0 = global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(edge[0], f1, Face[0]);
                    bool isTangent1 = global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(edge[1], Face[0], Face[1]);
                    bool isTangent2 = global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(edge[2], Face[1], Face[2]);
                    if ((isTangent0 || isTangent2) && isTangent1)
                    {
                        //         GeomAbs_Shape cont0,cont1,cont2;
                        //         cont0=BRep_Tool::Continuity(Edge[0],F1,Face[0]);
                        //         cont1=BRep_Tool::Continuity(Edge[1],Face[0],Face[1]);
                        //         cont2=BRep_Tool::Continuity(Edge[2],Face[1],Face[2]);
                        //         if ((cont0!=GeomAbs_C0 || cont2!=GeomAbs_C0) && cont1!=GeomAbs_C0) {
                        //  Modified by Sergey KHROMOV - Fri Dec 21 18:07:49 2001 Begin
                        facesau = Face[0];
                        edgesau = edge[0];
                        nbface = 1;
                        edge[1] = edge[3];
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
            edge[1] = edge[2];
        }

        NCollection_Array1<double> pardeb = new(1, 4), parfin = new(1, 4);
        Point2D pfil1 = new(), pfac1 = new(), pfil2 = new(), pfac2 = new(), pint = new(), pfildeb = new();
        Curve2D? hc1 = null, hc2 = null;
        HInter inters = new();
        int[] proledge = new int[nn], prolface = new int[nn + 1]; // last prolface[nn] is for Fd
        int[] shrink = new int[nn];
        Face[] faceprol = new Face[nn];
        for (int k = 0; k < nn; k++)
        {
            faceprol[k] = new();
        }
        int[] indcurve = new int[nn];
        int indpoint2 = 0, indpoint1 = 0;
        DsCurvePointInterference? interfp1 = null, interfp2 = null;
        DsCurvePointInterference?[] interfedge = new DsCurvePointInterference?[nn];
        DsSurfaceCurveInterference? interfc = null;
        DsSurfaceCurveInterference?[] interfPC = new DsSurfaceCurveInterference?[nn],
                                                 interfPS = new DsSurfaceCurveInterference?[nn];
        double u2 = 0.0, v2 = 0.0, p1 = 0.0, p2 = 0.0, paredge1 = 0.0;
        double paredge2 = 0.0, tolex = 1.0e-4;
        bool extend = false;
        Surface? sfacemoins1 = null, sface = null;
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
            e2 = edge[nb];
            if (nb == 0)
            {
                f = f1;
            }
            else
            {
                f = Face[nb - 1];
                if (prolface[nb - 1] == 0)
                {
                    faceprol[nb - 1] = f;
                }
            }

            if (f.IsNull)
            {
                throw new NullObjectException("IntersectionAtEnd : Trying to intersect with null face");
            }

            sfacemoins1 = Tool.Surface(f);
            Curve3D? cint = null;
            Curve2D? c2dint1 = null, c2dint2 = null, cface = null, cfacemoins1 = null;

            ///////////////////////////////////////////////////////
            // determine intersections of edges and the fillet
            // to find limitations of intersections face - fillet
            ///////////////////////////////////////////////////////

            if (nb == 1)
            {
                hc1 = Tool.CurveOnSurface(edge[0], Face[0], out Ubid, out Ubid);
                if (isOnSame1)
                {
                    // update interference param on Fi1 and point of CV1
                    if (prolface[0] != 0)
                    {
                        bs.Initialize(faceprol[0], false);
                    }
                    else
                    {
                        bs.Initialize(Face[0], false);
                    }
                    Curve3D? c3df = dStr.Curve(fi1.LineIndex()).Curve();
                    double ufi = fi2.Parameter(isfirst);
                    ref FaceInterference fi = ref fd.ChangeInterferenceOnS1();
                    if (!IntersUpdateOnSame(hGs, hBs, c3df, f1, Face[0], edge[0], vtx, isfirst,
                                                                    10 * _tolapp3d, // in
                                                                    ref fi, ref cv1, ref pfac1, ref ufi))
                    { // out
                        throw new FailureException("IntersectionAtEnd: pb intersection Face - Fi");
                    }
                    fi1 = fi;
                    if (intersOnSameFailed)
                    { // probable at fillet building
                        // look for paredge2
                        ProjectPointOnCurve2D proj = new();
                        if (c2dint2 is null)
                        {
                            proj.Init(pfac1, hc1);
                        }
                        else
                        {
                            proj.Init(pfac1, c2dint2);
                        }
                        paredge2 = proj.LowerDistanceParameter();
                    }
                    // update stripe point
                    DsPoint tpoint = new(cv1.Point(), _tolapp3d);
                    indpoint1 = dStr.AddPoint(tpoint);
                    stripe.SetIndexPoint(indpoint1, isfirst, 1);
                    // reset arc of CV1
                    Vertex vert1 = new(), vert2 = new();
                    Exploration.Vertices(edge[0], out vert1, out vert2);
                    Orientation arcOri = vtx.IsSame(vert1) ? Forward : Reversed;
                    cv1.SetArc(_tolapp3d, edge[0], paredge2, arcOri);
                }
                else
                {
                    if (hc1 is null)
                    {
                        // curve 2d not found. Sfacemoins1 is extended and projection is done there
                        // CV1.Point ()
                        ChFi3d_ExtendSurface(ref sfacemoins1, ref prolface[0]);
                        if (prolface[0] != 0)
                        {
                            extend = true;
                            BrepBuilder bre = new();
                            double tol = Tool.Tolerance(f);
                            bre.MakeFace(ref faceprol[0], sfacemoins1, f.Placement, tol);
                            if (!isOnSame1)
                            {
                                SurfaceAdaptor asurf = new();
                                asurf.Load(sfacemoins1);
                                ExtPS ext = new(cv1.Point(), asurf, tol, tol, ExtFlag_MIN);
                                double uc1 = 0.0, vc1 = 0.0;
                                if (ext.IsDone)
                                {
                                    ext.Point(1).Parameter(out uc1, out vc1);
                                    pfac1 = pfac1.WithX(uc1);
                                    pfac1 = pfac1.WithY(vc1);
                                }
                            }
                        }
                    }
                    else
                    {
                        pfac1 = hc1.Value(cv1.ParameterOnArc());
                    }
                }
                paredge1 = cv1.ParameterOnArc();
                if (fi1.LineIndex() != 0)
                {
                    pfil1 = fi1.PCurveOnSurf().Value(fi1.Parameter(isfirst));
                }
                else
                {
                    pfil1 = fi1.PCurveOnSurf().Value(fi1.Parameter(!isfirst));
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

                Curve3D? c = null;
                c = Tool.Curve(e2, out Ubid, out Vbid);
                TrimmedCurve3D? ctrim = new TrimmedCurve3D(c, Ubid, Vbid);
                double utrim = 0.0, vtrim = 0.0;
                utrim = ctrim.BasisCurve.FirstParameter;
                vtrim = ctrim.BasisCurve.LastParameter;
                if (ctrim.IsPeriodic)
                {
                    if (Ubid > ctrim.Period)
                    {
                        Ubid = (utrim + vtrim) / 2;
                        Vbid = vtrim;
                    }
                    else
                    {
                        Ubid = utrim;
                        Vbid = (utrim + vtrim) / 2;
                    }
                }
                else
                {
                    Ubid = utrim;
                    Vbid = vtrim;
                }
                CurveAdaptor? hc = new CurveAdaptor(c, Ubid, Vbid);
                CurveAdaptor Cad = hc;
                inters.Perform(hc, hGs);
                if (prolface[nn] == 0 && (!inters.IsDone || (inters.PointCount == 0)))
                {
                    // extend surface of conge
                    BoundedSurface? s1 = 
                        (dStr.Surface(fd.Surf()).Surface() as BoundedSurface);
                    if (s1 is not null)
                    {
                        double length = 0.5 * Max(fi1Length, fi2Length);
                        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref s1, length, 1, false, !isfirst);
                        prolface[nn] = 1;
                        if (stripe.IsInDS(!isfirst) == 0)
                        {
                            gs.Load(s1);
                            inters.Perform(hc, hGs);
                            if (inters.IsDone && inters.PointCount != 0)
                            {
                                fd.ChangeSurf(dStr.AddSurface(new DsSurface(s1, dStr.ChangeSurface(isurf).Tolerance())));
                                // update history
                                if (_eviMap.ContainsKey(edgeSpine))
                                {
                                    NCollection_List<int>.Iterator itl = new(_eviMap.GetValue(edgeSpine));
                                    for (; itl.More(); itl.Next())
                                    {
                                        if (itl.Value() == isurf)
                                        {
                                            _eviMap.GetValue(edgeSpine).Remove(itl);
                                            break;
                                        }
                                    }
                                    _eviMap.GetValue(edgeSpine).Append(fd.Surf());
                                }
                                else
                                {
                                    NCollection_List<int> indexList = new();
                                    indexList.Append(fd.Surf());
                                    _eviMap[edgeSpine] = indexList;
                                }
                                ////////////////
                                isurf = fd.Surf();
                            }
                        }
                    }
                }
                if (!inters.IsDone || (inters.PointCount == 0))
                {
                    BSplineCurve3D? cd = (c as BSplineCurve3D);
                    BezierCurve3D? cd1 = (c as BezierCurve3D);
                    if (cd is not null || cd1 is not null)
                    {
                        BrepBuilder bre = new();
                        sface = Tool.Surface(Face[nb]);
                        ChFi3d_ExtendSurface(ref sface, ref prolface[nb]);
                        double tol = Tool.Tolerance(f);
                        bre.MakeFace(ref faceprol[nb], sface, Face[nb].Placement, tol);
                        if (nb != 0 && prolface[nb - 1] == 0)
                        {
                            ChFi3d_ExtendSurface(ref sfacemoins1, ref prolface[nb - 1]);
                            if (prolface[nb - 1] != 0)
                            {
                                tol = Tool.Tolerance(f);
                                bre.MakeFace(ref faceprol[nb - 1], sfacemoins1, f.Placement, tol);
                            }
                        }
                        else
                        {
                            int prol = 0;
                            ChFi3d_ExtendSurface(ref sfacemoins1, ref prol);
                        }
                        IntSS interSS = new(sfacemoins1, sface, 1.0e-7, true, true, true);
                        if (interSS.IsDone)
                        {
                            trouve = false;
                            for (int i = 1; i <= interSS.LineCount && !trouve; i++)
                            {
                                extend = true;
                                cint = interSS.Line(i);
                                c2dint1 = interSS.LineOnS1(i);
                                c2dint2 = interSS.LineOnS2(i);
                                Cad.Load(cint);
                                inters.Perform(hc, hGs);
                                trouve = inters.IsDone && inters.PointCount != 0;
                                // eap occ293, eval tolex on finally trimmed curves
                                //               occ::handle<GeomAdaptor_Surface> H1=new
                                //               GeomAdaptor_Surface(Sfacemoins1); occ::handle<GeomAdaptor_Surface>
                                //               H2=new GeomAdaptor_Surface(Sface);
                                //              tolex=ChFi3d_EvalTolReached(H1,C2dint1,H2,C2dint2,cint);
                                tolex = interSS.TolReached3d();
                            }
                        }
                    }
                }
                if (inters.IsDone)
                {
                    nbp = inters.PointCount;
                    if (nbp == 0)
                    {
                        if (nb == 0 || nb == nbface)
                        {
                            intersOnSameFailed = true;
                        }
                        else
                        {
                            PerformMoreThreeCorner(index, 1);
                            return;
                        }
                    }
                    else
                    {
                        Point3D p = Tool.Pnt(vtx);
                        double distmin = p.DistanceTo(inters.Point(1).Pnt());
                        nbp = 1;
                        for (int i = 2; i <= inters.PointCount; i++)
                        {
                            dist = p.DistanceTo(inters.Point(i).Pnt());
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
                            cfacemoins1 = Tool.CurveOnSurface(e2, f, out u2, out v2);
                            if (cfacemoins1 is null)
                            {
                                throw new ConstructionErrorException("Failed to get p-curve of edge");
                            }
                            cface = Tool.CurveOnSurface(e2, Face[nb], out u2, out v2);
                            if (cface is null)
                            {
                                throw new ConstructionErrorException("Failed to get p-curve of edge");
                            }
                            cfacemoins1.D0(paredge2, out pfac2);
                            cface.D0(paredge2, out pint);
                        }
                        else if (c2dint1 is null || c2dint2 is null)
                        {
                            throw new ConstructionErrorException("Failed to get p-curve of edge");
                        }
                        else
                        {
                            c2dint1.D0(paredge2, out pfac2);
                            c2dint2.D0(paredge2, out pint);

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
                hc2 = Tool.CurveOnSurface(e2, Face[nbface - 1], out Ubid, out Ubid);
                if (hc2 is null)
                {
                    // curve 2d is not found,  Sfacemoins1 is extended CV2.Point() is projected there

                    ChFi3d_ExtendSurface(ref sfacemoins1, ref prolface[0]);
                    if (prolface[0] != 0)
                    {
                        BrepBuilder bre = new();
                        extend = true;
                        double tol = Tool.Tolerance(f);
                        bre.MakeFace(ref faceprol[nb - 1], sfacemoins1, f.Placement, tol);
                        SurfaceAdaptor asurf = new();
                        asurf.Load(sfacemoins1);
                        ExtPS ext = new(cv2.Point(), asurf, tol, tol, ExtFlag_MIN);
                        double uc2 = 0.0, vc2 = 0.0;
                        if (ext.IsDone)
                        {
                            ext.Point(1).Parameter(out uc2, out vc2);
                            pfac2 = pfac2.WithX(uc2);
                            pfac2 = pfac2.WithY(vc2);
                        }
                    }
                }
                else
                {
                    pfac2 = hc2.Value(cv2.ParameterOnArc());
                }
                paredge2 = cv2.ParameterOnArc();
                if (fi2.LineIndex() != 0)
                {
                    pfil2 = fi2.PCurveOnSurf().Value(fi2.Parameter(isfirst));
                }
                else
                {
                    pfil2 = fi2.PCurveOnSurf().Value(fi2.Parameter(!isfirst));
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
                    bs.Initialize(faceprol[nb - 1]);
                }
                else
                {
                    bs.Initialize(Face[nb - 1]);
                }
                Curve3D? c3df = dStr.Curve(fi2.LineIndex()).Curve();
                double ufi = fi1.Parameter(isfirst);
                ref FaceInterference fi = ref fd.ChangeInterferenceOnS2();
                if (!IntersUpdateOnSame(hGs, hBs, c3df, f2, f, edge[nb], vtx, isfirst,
                                                                10 * _tolapp3d, // in
                                                                ref fi, ref cv2, ref pfac2, ref ufi))
                { // out
                    throw new FailureException("IntersectionAtEnd: pb intersection Face - Fi");
                }
                fi2 = fi;
                if (intersOnSameFailed)
                { // probable at fillet building
                    // look for paredge2
                    ProjectPointOnCurve2D proj = new();
                    if (extend)
                    {
                        proj.Init(pfac2, c2dint2);
                    }
                    else
                    {
                        proj.Init(pfac2, Tool.CurveOnSurface(e2, Face[nbface - 1], out Ubid, out Ubid));
                    }
                    paredge2 = proj.LowerDistanceParameter();
                }
                // update stripe point
                DsPoint tpoint = new(cv2.Point(), _tolapp3d);
                indpoint2 = dStr.AddPoint(tpoint);
                stripe.SetIndexPoint(indpoint2, isfirst, 2);
                // reset arc of CV2
                Vertex vert1 = new(), vert2 = new();
                Exploration.Vertices(edge[nbface], out vert1, out vert2);
                Orientation arcOri = vtx.IsSame(vert1) ? Forward : Reversed;
                cv2.SetArc(_tolapp3d, edge[nbface], paredge2, arcOri);
            }

            if (prolface[nb - 1] != 0)
            {
                bs.Initialize(faceprol[nb - 1]);
            }
            else
            {
                bs.Initialize(Face[nb - 1]);
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

            pardeb[1] = pfil1.X;
            pardeb[2] = pfil1.Y;
            pardeb[3] = pfac1.X;
            pardeb[4] = pfac1.Y;
            parfin[1] = pfil2.X;
            parfin[2] = pfil2.Y;
            parfin[3] = pfac2.X;
            parfin[4] = pfac2.Y;

            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pfac1, pfac2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(bs, uu1, uu2, vv1, vv2);

            //////////////////////////////////////////////////////////////////////
            // calculate intersections face - fillet
            //////////////////////////////////////////////////////////////////////

            if (!ChFi3d_ComputeCurves(hGs, hBs, pardeb, parfin, ref cc, ref ps, ref pc, _tolapp3d, _tol2d, out tolreached,
                                                                nbface == 1))
            {
                PerformMoreThreeCorner(index, 1);
                return;
            }
            // storage of information in the data structure

            // evaluate tolerances
            p1 = cc.FirstParameter;
            p2 = cc.LastParameter;
            double to1 = 0.0, to2 = 0.0;
            Point2D p2d1 = new(), p2d2 = new();
            Point3D P1 = new(), P2 = new(), p3 = new(), p4 = new(), p5 = new(), p6 = new(), p7 = new(), p8 = new();
            hGs.D0(pardeb[1], pardeb[2], out P1);
            hGs.D0(parfin[1], parfin[2], out P2);
            hBs.D0(pardeb[3], pardeb[4], out p3);
            hBs.D0(parfin[3], parfin[4], out p4);
            pc.D0(p1, out p2d1);
            pc.D0(p2, out p2d2);
            hBs.D0(p2d1.X, p2d1.Y, out p7);
            hBs.D0(p2d2.X, p2d2.Y, out p8);
            ps.D0(p1, out p2d1);
            ps.D0(p2, out p2d2);
            hGs.D0(p2d1.X, p2d1.Y, out p5);
            hGs.D0(p2d2.X, p2d2.Y, out p6);
            to1 = Max(P1.DistanceTo(p5) + p3.DistanceTo(p7), tolreached);
            to2 = Max(P2.DistanceTo(p6) + p4.DistanceTo(p8), tolreached);

            //////////////////////////////////////////////////////////////////////
            // storage in the DS of the intersection curve
            //////////////////////////////////////////////////////////////////////

            bool isvtx1 = false;
            bool isvtx2 = false;
            int indice = 0;

            if (nb == 1)
            {
                indpoint1 = stripe.IndexPoint(isfirst, 1);
                if (!cv1.IsVertex)
                {
                    ref DsPoint tpt = ref dStr.ChangePoint(indpoint1);
                    tpt.Tolerance(Max(tpt.Tolerance(), to1));
                }
                else
                {
                    isvtx1 = true;
                }
            }
            if (nb == nbface)
            {
                indpoint2 = stripe.IndexPoint(isfirst, 2);
                if (!cv2.IsVertex)
                {
                    ref DsPoint tpt = ref dStr.ChangePoint(indpoint2);
                    tpt.Tolerance(Max(tpt.Tolerance(), to2));
                }
                else
                {
                    isvtx2 = true;
                }
            }
            else
            {
                Point3D point = cc.Value(cc.LastParameter);
                DsPoint tpoint = new(point, to2);
                indpoint2 = dStr.AddPoint(tpoint);
            }

            if (nb != 1)
            {
                ref DsPoint tpt = ref dStr.ChangePoint(indpoint1);
                tpt.Tolerance(Max(tpt.Tolerance(), to1));
            }
            DsCurve tcurv3d = new(cc, tolreached);
            indcurve[nb - 1] = dStr.AddCurve(tcurv3d);

            interfp1 = ChFi3d_FilPointInDS(Forward, indcurve[nb - 1], indpoint1, cc.FirstParameter, isvtx1);

            interfp2 = ChFi3d_FilPointInDS(Reversed, indcurve[nb - 1], indpoint2, cc.LastParameter, isvtx2);

            dStr.ChangeCurveInterferences(indcurve[nb - 1]).Append(interfp1);
            dStr.ChangeCurveInterferences(indcurve[nb - 1]).Append(interfp2);

            //////////////////////////////////////////////////////////////////////
            // storage for the face
            //////////////////////////////////////////////////////////////////////

            Orientation ori = Forward;
            orface = Face[nb - 1].Orientation;
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
                int iarc1 = dStr.AddShape(edge[0]);
                interfedge[0] =
                    ChFi3d_FilPointInDS(cv1.TransitionOnArc(), iarc1, indpoint1, paredge1, isvtx1);
                // DStr.ChangeShapeInterferences(Edge[0]).Append(Interfp1);
            }
            if (nb == nbface)
            {
                int iarc2 = dStr.AddShape(edge[nb]);
                interfedge[nb] =
                    ChFi3d_FilPointInDS(cv2.TransitionOnArc(), iarc2, indpoint2, paredge2, isvtx2);
                // DStr.ChangeShapeInterferences(Edge[nb]).Append(Interfp2);
            }

            if (nb != nbface || oneintersection1 || oneintersection2)
            {
                if (nface == 3)
                {
                    v1 = Exploration.FirstVertex(edge[nb]);
                    V2 = Exploration.LastVertex(edge[nb]);
                    if (ContainV(f1, v1) || ContainV(f2, v1))
                    {
                        ori = Forward;
                    }
                    else if (ContainV(f1, V2) || ContainV(f2, V2))
                    {
                        ori = Reversed;
                    }
                    else
                    {
                        throw new FailureException("IntersectionAtEnd : pb orientation");
                    }

                    if (ContainV(f1, v1) && ContainV(f1, V2))
                    {
                        dist1 = (Tool.Pnt(v1)).DistanceTo(Tool.Pnt(vtx));
                        dist2 = (Tool.Pnt(V2)).DistanceTo(Tool.Pnt(vtx));
                        if (dist1 < dist2)
                        {
                            ori = Forward;
                        }
                        else
                        {
                            ori = Reversed;
                        }
                    }
                    if (ContainV(f2, v1) && ContainV(f2, V2))
                    {
                        dist1 = (Tool.Pnt(v1)).DistanceTo(Tool.Pnt(vtx));
                        dist2 = (Tool.Pnt(V2)).DistanceTo(Tool.Pnt(vtx));
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
                    if (Exploration.FirstVertex(edge[nb]).IsSame(vtx))
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
                    int iarc2 = dStr.AddShape(edge[nb]);
                    interfedge[nb] = ChFi3d_FilPointInDS(ori, iarc2, indpoint2, paredge2);
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
                    pext = Tool.Pnt(vtx);
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
                        BoundedCurve3D? c1 = (csau as BoundedCurve3D);
                        if (oneintersection1 && extend)
                        {
                            if (c1 is not null)
                            {
                                Point3D pl = new();
                                pl = c1.Value(c1.LastParameter);
                                // bool sens;
                                sens = pl.DistanceTo(pext) < tolpt ? 1 : 0; // OCCT: a bool in the int sens
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref c1, cv1.Point(), 1, sens != 0);
                                csau = c1;
                            }
                        }
                        else if (oneintersection2 && extend)
                        {
                            if (c1 is not null)
                            {
                                Point3D pl = new();
                                pl = c1.Value(c1.LastParameter);
                                // bool sens;
                                sens = pl.DistanceTo(pext) < tolpt ? 1 : 0; // OCCT: a bool in the int sens
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref c1, cv2.Point(), 1, sens != 0);
                                csau = c1;
                            }
                        }
                        cad.Load(csau);
                    }
                    CurvePointExtrema ext = new(pext, cad, tolpt);
                    double par1 = 0.0, par2 = 0.0, par = 0.0, parVtx = 0.0;
                    bool vtx1 = false;
                    bool vtx2 = false;
                    par1 = ext.Point(1).Parameter();
                    parVtx = par1;
                    if (oneintersection1 || oneintersection2)
                    {
                        if (oneintersection2)
                        {
                            pext = cv2.Point();
                            ind = indpoint2;
                        }
                        else
                        {
                            pext = cv1.Point();
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
                        indp2 = dStr.AddShape(vtx);
                        vtx2 = true;
                        par = par1;
                        par1 = par2;
                        par2 = par;
                    }
                    else
                    {
                        indp1 = dStr.AddShape(vtx);
                        indp2 = ind;
                        vtx1 = true;
                    }
                    Curve3D? ct = new TrimmedCurve3D(csau, par1, par2);
                    Orientation orient = default;
                    cc.D0(cc.FirstParameter, out P1);
                    cc.D0(cc.LastParameter, out P2);
                    ct.D0(ct.FirstParameter, out p3);
                    ct.D0(ct.LastParameter, out p4);
                    if (P2.DistanceTo(p3) < tolpt || P1.DistanceTo(p4) < tolpt)
                    {
                        orient = orien;
                    }
                    else
                    {
                        orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orien);
                    }
                    if (oneintersection1 || oneintersection2)
                    {
                        indice = dStr.AddShape(Face[0]);
                        if (extend)
                        {
                            dStr.SetNewSurface(Face[0], sfacemoins1);
                            ComputeCurve2d(ct, ref faceprol[0], ref c2dint1);
                        }
                        else
                        {
                            Edge localEdge = edgesau;
                            if (edgesau.Orientation != orient)
                            {
                                localEdge.Reverse();
                            }
                            c2dint1 = Tool.CurveOnSurface(localEdge, Face[0], out ubid, out vbid);
                        }
                    }
                    else
                    {
                        indice = dStr.AddShape(Face[nb - 1]);
                        dStr.SetNewSurface(Face[nb - 1], sfacemoins1);
                    }
                    //// for periodic 3d curves ////
                    if (cad.IsPeriodic && c2dint1 is not null)
                    {
                        Point2D p2d = Tool.Parameters(vtx, Face[0]);
                        ProjectPointOnCurve2D projector = new(p2d, c2dint1);
                        par = projector.LowerDistanceParameter();
                        double shift = par - parVtx;
                        if (Math.Abs(shift) > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            par1 += shift;
                            par2 += shift;
                        }
                    }
                    ////////////////////////////////

                    ct = new TrimmedCurve3D(csau, par1, par2);
                    if (oneintersection1 || oneintersection2)
                    {
                        tolex = 10 * Tool.Tolerance(edgesau);
                    }
                    if (extend)
                    {
                        SurfaceAdaptor? h1 = null, h2 = null;
                        h1 = new SurfaceAdaptor(sfacemoins1);
                        if (sface is null)
                        {
                            tolex = Max(tolex, ChFi3d_EvalTolReached(h1, c2dint1, h1, c2dint1, ct));
                        }
                        else
                        {
                            h2 = new SurfaceAdaptor(sface);
                            tolex = Max(tolex, ChFi3d_EvalTolReached(h1, c2dint1, h2, c2dint2, ct));
                        }
                    }
                    DsCurve tcurv = new(ct, tolex);
                    int indcurv = 0;
                    indcurv = dStr.AddCurve(tcurv);
                    interfp1 = ChFi3d_FilPointInDS(Forward, indcurv, indp1, par1, vtx1);
                    interfp2 = ChFi3d_FilPointInDS(Reversed, indcurv, indp2, par2, vtx2);
                    dStr.ChangeCurveInterferences(indcurv).Append(interfp1);
                    dStr.ChangeCurveInterferences(indcurv).Append(interfp2);

                    interfc = ChFi3d_FilCurveInDS(indcurv, indice, c2dint1, orient);
                    dStr.ChangeShapeInterferences(indice).Append(interfc);
                    if (oneintersection1 || oneintersection2)
                    {
                        indice = dStr.AddShape(facesau);
                        if (facesau.Orientation == Face[0].Orientation)
                        {
                            orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                        }
                        if (extend)
                        {
                            ComputeCurve2d(ct, ref faceprol[1], ref c2dint2);
                        }
                        else
                        {
                            Edge localEdge = edgesau;
                            if (edgesau.Orientation != orient)
                            {
                                localEdge.Reverse();
                            }
                            c2dint2 = Tool.CurveOnSurface(localEdge, facesau, out ubid, out vbid);
                            // Reverse for case of edgesau on closed surface (Face[0] is equal to facesau)
                        }
                    }
                    else
                    {
                        indice = dStr.AddShape(Face[nb]);
                        dStr.SetNewSurface(Face[nb], sface);
                        if (Face[nb].Orientation == Face[nb - 1].Orientation)
                        {
                            orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                        }
                    }
                    if (!bordlibre)
                    {
                        interfc = ChFi3d_FilCurveInDS(indcurv, indice, c2dint2, orient);
                        dStr.ChangeShapeInterferences(indice).Append(interfc);
                    }
                }
            }

            if (checkShrink
                    && IsShrink(new CurveAdaptor2D(ps), p1, p2, checkShrParam, isUShrink, global::Kerf.Engine.Foundation.Precision.Parametric(tolreached)))
            {
                shrink[nb - 1] = 1;
                // store section face-chamf curve for previous SurfData
                // Suppose Fd and SDprev are parametrized similarly
                if (!isShrink)
                { // first time
                    FaceInterference fi = sDprev.InterferenceOnS1();
                    Point2D uv = fi.PCurveOnSurf().Value(fi.Parameter(isfirst));
                    prevSDParam = isUShrink ? uv.X : uv.Y;
                }
                Point2D uv1 = p2d1, uv2 = p2d2;
                uv1 = uv1.WithCoord(isUShrink ? 1 : 2, prevSDParam);
                uv2 = uv2.WithCoord(isUShrink ? 1 : 2, prevSDParam);
                double aTolreached = 0.0;
                ChFi3d_ComputePCurv(cc, uv1, uv2, out ps, dStr.Surface(sDprev.Surf()).Surface(), p1, p2, _tolapp3d,
                                                        out aTolreached);
                ref DsCurve tCurv = ref dStr.ChangeCurve(indcurve[nb - 1]);
                tCurv.Tolerance(Max(tCurv.Tolerance(), aTolreached));

                interfPS[nb - 1] = ChFi3d_FilCurveInDS(indcurve[nb - 1], isurfPrev, ps, orcourbe);
                dStr.ChangeSurfaceInterferences(isurfPrev).Append(interfPS[nb - 1]);

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

            indice = dStr.AddShape(Face[nb - 1]);
            interfPC[nb - 1] = ChFi3d_FilCurveInDS(indcurve[nb - 1], indice, pc, orien);
            if (shrink[nb - 1] == 0)
            {
                interfPS[nb - 1] = ChFi3d_FilCurveInDS(indcurve[nb - 1], isurf, ps, orcourbe);
            }
            indpoint1 = indpoint2;

        } // end loop on faces being intersected with ChFi

        if (isOnSame1)
        {
            cv1.Reset();
        }
        if (isOnSame2)
        {
            cv2.Reset();
        }

        for (nb = 1; nb <= nbface; nb++)
        {
            int indice = dStr.AddShape(Face[nb - 1]);
            dStr.ChangeShapeInterferences(indice).Append(interfPC[nb - 1]);
            if (shrink[nb - 1] == 0)
            {
                dStr.ChangeSurfaceInterferences(isurf).Append(interfPS[nb - 1]);
            }
            if (proledge[nb - 1] == 0)
            {
                dStr.ChangeShapeInterferences(edge[nb - 1]).Append(interfedge[nb - 1]);
            }
        }
        dStr.ChangeShapeInterferences(edge[nbface]).Append(interfedge[nbface]);

        if (!isShrink)
        {
            stripe.InDS(isfirst);
        }
        else
        {
            // compute curves for !<isfirst> end of <Fd> and <isfirst> end of previous <SurfData>

            // for Fd
            // Bnd_Box box;
            Point2D uv = new(), uv1 = midP2d, uv2 = midP2d;
            if (isOnSame1)
            {
                uv = uv2 = fi1.PCurveOnSurf().Value(fi1.Parameter(!isfirst));
            }
            else
            {
                uv = uv1 = fi2.PCurveOnSurf().Value(fi2.Parameter(!isfirst));
            }
            double aTolreached = 0.0;
            Curve3D? c3d = null;
            Surface? surf = dStr.Surface(fd.Surf()).Surface();
            // box.Add(aSurf->Value(UV.X(), UV.Y()));

            ChFi3d_ComputeArete(cv1, uv1, cv2, uv2,
                                                    surf, ref // in
                                                    c3d, ref ps, ref p1, ref p2, _tolapp3d, _tol2d, out aTolreached,
                                                    0); // out except tolers

            indpoint1 = indpoint2 = midIpoint;
            Point3D point = new();
            if (isOnSame1)
            {
                point = c3d.Value(p2);
                DsPoint tpoint = new(point, aTolreached);
                indpoint2 = dStr.AddPoint(tpoint);
                uv = ps.Value(p2);
            }
            else
            {
                point = c3d.Value(p1);
                DsPoint tpoint = new(point, aTolreached);
                indpoint1 = dStr.AddPoint(tpoint);
                uv = ps.Value(p1);
            }
            // box.Add(point);
            // box.Add(aSurf->Value(UV.X(), UV.Y()));

            DsCurve crv = new DsCurve(c3d, aTolreached);
            int icurv = dStr.AddCurve(crv);
            interfp1 = ChFi3d_FilPointInDS(Forward, icurv, indpoint1, p1, false);
            interfp2 = ChFi3d_FilPointInDS(Reversed, icurv, indpoint2, p2, false);
            interfc = ChFi3d_FilCurveInDS(icurv, isurf, ps, orcourbe);
            dStr.ChangeCurveInterferences(icurv).Append(interfp1);
            dStr.ChangeCurveInterferences(icurv).Append(interfp2);
            dStr.ChangeSurfaceInterferences(isurf).Append(interfc);

            // for SDprev
            surf = dStr.Surface(sDprev.Surf()).Surface();
            uv1 = uv1.WithCoord(isUShrink ? 1 : 2, prevSDParam);
            uv2 = uv2.WithCoord(isUShrink ? 1 : 2, prevSDParam);

            ChFi3d_ComputePCurv(c3d, uv1, uv2, out pc, surf, p1, p2, _tolapp3d, out aTolreached);

            crv.Tolerance(Max(crv.Tolerance(), aTolreached)); // OCCT: changes the local copy, not the curve in the DS

            interfc = ChFi3d_FilCurveInDS(icurv, isurfPrev, pc, global::Kerf.Engine.Topology.TopAbs.Reverse(orcourbe));
            dStr.ChangeSurfaceInterferences(isurfPrev).Append(interfc);

            // UV = isOnSame1 ? UV2 : UV1;
            // box.Add(aSurf->Value(UV.X(), UV.Y()));
            // UV = Ps->Value(isOnSame1 ? p2 : p1);
            // box.Add(aSurf->Value(UV.X(), UV.Y()));
            // ChFi3d_SetPointTolerance(DStr,box, isOnSame1 ? indpoint2 : indpoint1);

            // to process properly this case in ChFi3d_FilDS()
            stripe.InDS(isfirst, 2);
            fd.ChangeInterference(isOnSame1 ? 2 : 1).SetLineIndex(0);
            ref CommonPoint cPprev1 = ref sDprev.ChangeVertex(isfirst, isOnSame1 ? 2 : 1);
            ref CommonPoint cPlast1 = ref fd.ChangeVertex(isfirst, isOnSame1 ? 2 : 1);
            ref CommonPoint cPlast2 = ref fd.ChangeVertex(!isfirst, isOnSame1 ? 2 : 1);
            if (cPprev1.IsOnArc)
            {
                cPlast1 = cPprev1;
                cPprev1.Reset();
                cPprev1.SetPoint(cPlast1.Point());
                cPlast2.Reset();
                cPlast2.SetPoint(cPlast1.Point());
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
            Curve3D? cend = dStr.Curve(indcurve[nb]).Curve();
            Curve2D? pCend = interfPS[nb].PCurve();
            // point near which self intersection may occur
            ref DsPoint pds = ref dStr.ChangePoint(midIpoint);
            Point3D pvert = pds.Point();
            double tol = pds.Tolerance();

            CurveAdaptor2D pc1 = new(ps), pc2 = new(pCend);
            GInter intersector = new(pc1, pc2, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (!intersector.IsDone)
            {
                return;
            }
            for (nb = 1; nb <= intersector.PointCount; nb++)
            {
                IntersectionPoint2D ip = intersector.Point(nb);
                Point3D Pint = c3d.Value(ip.ParamOnFirst());
                tol = Max(tol, pvert.DistanceTo(Pint));
                Pint = cend.Value(ip.ParamOnSecond());
                tol = Max(tol, pvert.DistanceTo(Pint));
            }
            for (nb = 1; nb <= intersector.SegmentCount; nb++)
            {
                IntersectionSegment2D @is = intersector.Segment(nb);
                if (@is.HasFirstPoint)
                {
                    IntersectionPoint2D ip = @is.FirstPoint();
                    Point3D Pint = c3d.Value(ip.ParamOnFirst());
                    tol = Max(tol, pvert.DistanceTo(Pint));
                    Pint = cend.Value(ip.ParamOnSecond());
                    tol = Max(tol, pvert.DistanceTo(Pint));
                }
                if (@is.HasLastPoint)
                {
                    IntersectionPoint2D ip = @is.LastPoint();
                    Point3D Pint = c3d.Value(ip.ParamOnFirst());
                    tol = Max(tol, pvert.DistanceTo(Pint));
                    Pint = cend.Value(ip.ParamOnSecond());
                    tol = Max(tol, pvert.DistanceTo(Pint));
                }
            }
            pds.Tolerance(tol);
        }
    }

    //  Modified by Sergey KHROMOV - Thu Apr 11 12:23:40 2002 Begin

    //=======================================================================
    // function : PerformMoreSurfdata
    // purpose  :  determine intersections at end on several surfdata
    //=======================================================================
    private void PerformMoreSurfdata(int index)
    {
        DsDataStructure dStr = _ds.ChangeDS();
        NCollection_List<Stripe> lOfStripe = _vDataMap[index];
        Stripe? stripe = null;
        Spine? spine = null;
        double tol3d = 1.0e-4;

        if (lOfStripe.IsEmpty())
        {
            return;
        }

        stripe = lOfStripe.First();
        spine = stripe.Spine();

        NCollection_Sequence<SurfData> seqSurfData = 
            stripe.ChangeSetOfSurfData();
        Vertex vtx = _vDataMap.FindKey(index);
        int sens = 0;
        int ind = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
        bool isFirst = (sens == 1);
        int indPrev = 0;
        SurfData? surfData = null;
        CommonPoint cp1 = new();
        CommonPoint cp2 = new();

        surfData = seqSurfData.Value(ind);

        cp1 = surfData.Vertex(isFirst, 1);
        cp2 = surfData.Vertex(isFirst, 2);

        Surface? surfPrev = null;
        Surface? surf = null;
        Face face = new();
        Face neighborFace = new();

        FindFace(vtx, cp1, cp2, ref face);
        surfPrev = Tool.Surface(face);

        if (sens == 1)
        {
            indPrev = ind + 1;
        }
        else
        {
            indPrev = ind - 1;
        }

        Edge arc1 = new();
        Edge arc2 = new();
        NCollection_List<Shape>.Iterator iter = new(_veMap[vtx]);
        bool isFound = false;

        for (; iter.More() && !isFound; iter.Next())
        {
            arc1 = ShapeCasts.Edge(iter.Value());

            if (ContainE(face, arc1))
            {
                isFound = true;
            }
        }

        isFound = false;
        iter.Initialize(_veMap[vtx]);

        for (; iter.More() && !isFound; iter.Next())
        {
            arc2 = ShapeCasts.Edge(iter.Value());

            if (ContainE(face, arc2) && !arc2.IsSame(arc1))
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
        CommonPoint cp1onArc = new();
        CommonPoint cp2onArc = new();
        CommonPoint cp2NotonArc = new();

        surfData = seqSurfData.Value(indPrev);
        cp1 = surfData.Vertex(isFirst, 1);
        cp2 = surfData.Vertex(isFirst, 2);

        if (cp1.IsOnArc && (cp1.Arc().IsSame(arc1) || cp1.Arc().IsSame(arc2)))
        {
            cp2onArc = cp1;
            cp2NotonArc = cp2;
            is2ndCP1OnArc = true;
        }
        else if (cp2.IsOnArc && (cp2.Arc().IsSame(arc1) || cp2.Arc().IsSame(arc2)))
        {
            cp2onArc = cp2;
            cp2NotonArc = cp1;
            is2ndCP1OnArc = false;
        }
        else
        {
            return;
        }

        surfData = seqSurfData.Value(ind);
        cp1 = surfData.Vertex(isFirst, 1);
        cp2 = surfData.Vertex(isFirst, 2);

        if (cp1.Point().DistanceTo(cp2onArc.Point()) <= tol3d)
        {
            cp1onArc = cp2;
            is1stCP1OnArc = false;
        }
        else
        {
            cp1onArc = cp1;
            is1stCP1OnArc = true;
        }

        if (!cp1onArc.IsOnArc)
        {
            return;
        }

        // determination of neighbor surface
        int indSurface = 0;
        if (is1stCP1OnArc)
        {
            indSurface = _listStripe.First().SetOfSurfData().Value(ind).IndexOfS1();
        }
        else
        {
            indSurface = _listStripe.First().SetOfSurfData().Value(ind).IndexOfS2();
        }

        neighborFace = ShapeCasts.Face(_ds.Shape(indSurface));

        // calculation of intersections
        Curve3D? cracc = null;
        Curve2D? pCurv1 = null;
        double parf = 0.0;
        double parl = 0.0;
        double tolReached = 0.0;

        surfData = seqSurfData.Value(ind);

        if (isFirst)
        {
            ChFi3d_ComputeArete(surfData.VertexLastOnS1(), surfData.InterferenceOnS1().PCurveOnSurf().Value(
                                                        surfData.InterferenceOnS1().LastParameter()), surfData.VertexLastOnS2(),
                                                    surfData.InterferenceOnS2().PCurveOnSurf().Value(
                                                        surfData.InterferenceOnS2().LastParameter()), dStr.Surface(surfData.Surf()).Surface(),
                                                    ref cracc, ref pCurv1, ref parf, ref parl, tol3d, _tol2d, out tolReached, 0);
        }
        else
        {
            ChFi3d_ComputeArete(surfData.VertexFirstOnS1(), surfData.InterferenceOnS1().PCurveOnSurf().Value(
                                                        surfData.InterferenceOnS1().FirstParameter()), surfData.VertexFirstOnS2(),
                                                    surfData.InterferenceOnS2().PCurveOnSurf().Value(
                                                        surfData.InterferenceOnS2().FirstParameter()), dStr.Surface(surfData.Surf()).Surface(),
                                                    ref cracc, ref pCurv1, ref parf, ref parl, tol3d, _tol2d, out tolReached, 0);
        }

        // calculation of the index of the line on anInd.
        // aPClineOnSurf is the pcurve on anInd.
        // aPClineOnFace is the pcurve on face.
        FaceInterference fi = new();

        if (is1stCP1OnArc)
        {
            fi = surfData.InterferenceOnS1();
        }
        else
        {
            fi = surfData.InterferenceOnS2();
        }

        Curve3D? cline = null;
        Curve2D? pClineOnSurf = null;
        Curve2D? pClineOnFace = null;
        int indLine = 0;

        indLine = fi.LineIndex();
        cline = dStr.Curve(fi.LineIndex()).Curve();
        pClineOnSurf = fi.PCurveOnSurf();
        pClineOnFace = fi.PCurveOnFace();

        // intersection between the SurfData number anInd and the Face aFace.
        // Obtaining of curves aCint1, aPCint11 and aPCint12.
        surf = dStr.Surface(surfData.Surf()).Surface();

        IntSS interSS = new(surfPrev, surf, 1.0e-7, true, true, true);
        Curve3D? cint1 = null;
        Curve2D? pCint11 = null;
        Curve2D? pCint12 = null;
        SurfaceAdaptor? h1 = new SurfaceAdaptor(surfPrev);
        SurfaceAdaptor? h2 = new SurfaceAdaptor(surf);
        double tolex1 = 0.0;
        int i = 0;
        Point3D pext1 = new();
        Point3D pext2 = new();
        Point3D pext = new();
        bool isPextFound = false;

        if (!interSS.IsDone)
        {
            return;
        }

        isFound = false;

        for (i = 1; i <= interSS.LineCount && !isFound; i++)
        {
            cint1 = interSS.Line(i);
            pCint11 = interSS.LineOnS1(i);
            pCint12 = interSS.LineOnS2(i);
            tolex1 = ChFi3d_EvalTolReached(h1, pCint11, h2, pCint12, cint1);

            cint1.D0(cint1.FirstParameter, out pext1);
            cint1.D0(cint1.LastParameter, out pext2);

            //  Modified by skv - Mon Jun  7 18:38:57 2004 OCC5898 Begin
            //     if (aPext1.Distance(aCP1onArc.Point()) <= aTol3d ||
            // 	aPext2.Distance(aCP1onArc.Point()))
            if (pext1.DistanceTo(cp1onArc.Point()) <= tol3d
                    || pext2.DistanceTo(cp1onArc.Point()) <= tol3d)
            {
                //  Modified by skv - Mon Jun  7 18:38:58 2004 OCC5898 End
                isFound = true;
            }
        }

        if (!isFound)
        {
            return;
        }

        if (pext1.DistanceTo(cp2onArc.Point()) > tol3d && pext1.DistanceTo(cp1onArc.Point()) > tol3d)
        {
            pext = pext1;
            isPextFound = true;
        }
        else if (pext2.DistanceTo(cp2onArc.Point()) > tol3d
                          && pext2.DistanceTo(cp1onArc.Point()) > tol3d)
        {
            pext = pext2;
            isPextFound = true;
        }
        else
        {
            isPextFound = false;
        }

        bool isDoSecondSection = false;
        double par = 0.0;

        if (isPextFound)
        {
            CurveAdaptor cad = new(cracc);
            CurvePointExtrema ext = new(pext, cad, tol3d);

            if (!ext.IsDone)
            {
                return;
            }

            isFound = false;
            for (i = 1; i <= ext.ExtCount && !isFound; i++)
            {
                if (ext.IsMin(i))
                {
                    Point3D projPnt = ext.Point(i).Value();

                    if (pext.DistanceTo(projPnt) <= tol3d)
                    {
                        par = ext.Point(i).Parameter();
                        isDoSecondSection = true;
                    }
                }
            }
        }

        Curve3D? trCracc = null;
        Orientation orSD1 = default;
        Orientation orSD2 = default;
        int indShape = 0;

        orSD1 = surfData.Orientation();
        surfData = seqSurfData.Value(indPrev);
        orSD2 = surfData.Orientation();
        surf = dStr.Surface(surfData.Surf()).Surface();

        // The following variables will be used if isDoSecondSection is true
        Curve3D? cint2 = null;
        Curve2D? pCint21 = null;
        Curve2D? pCint22 = null;
        double tolex2 = 0.0;

        if (isDoSecondSection)
        {
            double par1 = 0.0;

            cracc.D0(cracc.FirstParameter, out pext1);

            if (pext1.DistanceTo(cp2NotonArc.Point()) <= tol3d)
            {
                par1 = cracc.FirstParameter;
            }
            else
            {
                par1 = cracc.LastParameter;
            }

            if (par1 < par)
            {
                trCracc = new TrimmedCurve3D(cracc, par1, par);
            }
            else
            {
                trCracc = new TrimmedCurve3D(cracc, par, par1);
            }

            // Second section
            IntSS interSS2 = new(surfPrev, surf, 1.0e-7, true, true, true);

            if (!interSS2.IsDone)
            {
                return;
            }

            h1 = new SurfaceAdaptor(surfPrev);
            h2 = new SurfaceAdaptor(surf);

            isFound = false;

            for (i = 1; i <= interSS2.LineCount && !isFound; i++)
            {
                cint2 = interSS2.Line(i);
                pCint21 = interSS2.LineOnS1(i);
                pCint22 = interSS2.LineOnS2(i);
                tolex2 = ChFi3d_EvalTolReached(h1, pCint21, h2, pCint22, cint2);

                cint2.D0(cint2.FirstParameter, out pext1);
                cint2.D0(cint2.LastParameter, out pext2);

                if (pext1.DistanceTo(cp2onArc.Point()) <= tol3d
                        || pext2.DistanceTo(cp2onArc.Point()) <= tol3d)
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
            trCracc = new TrimmedCurve3D(cracc, cracc.FirstParameter, cracc.LastParameter);
        }

        // Storage of the data structure

        // calculation of the orientation of line of surfdata number
        // anIndPrev which contains aCP2onArc

        Curve2D? pCraccS = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(trCracc, surf);

        if (is2ndCP1OnArc)
        {
            fi = surfData.InterferenceOnS1();
            indShape = surfData.IndexOfS1();
        }
        else
        {
            fi = surfData.InterferenceOnS2();
            indShape = surfData.IndexOfS2();
        }

        if (indShape <= 0)
        {
            return;
        }

        Orientation curOrient = default;

        curOrient = dStr.Shape(indShape).Orientation;
        curOrient = global::Kerf.Engine.Topology.TopAbs.Compose(curOrient, surfData.Orientation());
        curOrient = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(fi.Transition()), curOrient);

        // Filling the data structure
        surfData = seqSurfData.Value(ind);

        DsPoint ptCP1 = new(cp1onArc.Point(), cp1onArc.Tolerance());
        int indCP1onArc = dStr.AddPoint(ptCP1);
        int indSurf1 = surfData.Surf();
        int indArc1 = dStr.AddShape(cp1onArc.Arc());
        int indSol = stripe.SolidIndex();

        DsCurvePointInterference? interfp1 = null;
        DsCurvePointInterference? interfp2 = null;

        interfp1 = ChFi3d_FilPointInDS(cp1onArc.TransitionOnArc(), indArc1, indCP1onArc, cp1onArc.ParameterOnArc());
        dStr.ChangeShapeInterferences(cp1onArc.Arc()).Append(interfp1);

        NCollection_List<DsInterference> solidInterfs = 
            dStr.ChangeShapeInterferences(indSol);
        DsSolidSurfaceInterference? ssi = 
            new DsSolidSurfaceInterference(new DsTransition(orSD1), DsKind.Solid, indSol,
                                                                                                DsKind.Surface, indSurf1);
        solidInterfs.Append(ssi);

        // deletion of Surface Data.
        seqSurfData.Remove(ind);

        if (!isFirst)
        {
            ind--;
        }

        surfData = seqSurfData.Value(ind);

        // definition of indices of common points in Data Structure

        int indCP2onArc = 0;
        int indCP2NotonArc = 0;

        if (is2ndCP1OnArc)
        {
            stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cp2onArc, dStr), isFirst, 1);
            stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cp2NotonArc, dStr), isFirst, 2);

            if (isFirst)
            {
                indCP2onArc = stripe.IndexFirstPointOnS1();
                indCP2NotonArc = stripe.IndexFirstPointOnS2();
            }
            else
            {
                indCP2onArc = stripe.IndexLastPointOnS1();
                indCP2NotonArc = stripe.IndexLastPointOnS2();
            }
        }
        else
        {
            stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cp2onArc, dStr), isFirst, 2);
            stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cp2NotonArc, dStr), isFirst, 1);

            if (isFirst)
            {
                indCP2onArc = stripe.IndexFirstPointOnS2();
                indCP2NotonArc = stripe.IndexFirstPointOnS1();
            }
            else
            {
                indCP2onArc = stripe.IndexLastPointOnS2();
                indCP2NotonArc = stripe.IndexLastPointOnS1();
            }
        }

        int indPoint1 = 0;
        int indPoint2 = 0;
        Point3D point1 = new();
        Point3D point2 = new();

        if (is2ndCP1OnArc)
        {
            fi = surfData.InterferenceOnS1();
            indShape = surfData.IndexOfS1();
        }
        else
        {
            fi = surfData.InterferenceOnS2();
            indShape = surfData.IndexOfS2();
        }

        Point2D p2d = new();
        DsSurfaceCurveInterference? interfc = null;
        Orientation orSurf = curOrient;
        Orientation orFace = face.Orientation;
        int indaFace = dStr.AddShape(face);
        int indPoint = indCP2onArc;
        int indCurve = 0;

        fi.PCurveOnFace().D0(fi.LastParameter(), out p2d);
        Surface? stemp2 = Tool.Surface(ShapeCasts.Face(dStr.Shape(indShape)));
        stemp2.D0(p2d.X, p2d.Y, out point2);
        fi.PCurveOnFace().D0(fi.FirstParameter(), out p2d);
        stemp2.D0(p2d.X, p2d.Y, out point1);

        if (isDoSecondSection)
        {
            DsPoint tpoint = new(pext, tolex2);
            DsCurve tcint2 = new(cint2, tolex2);

            indPoint = dStr.AddPoint(tpoint);
            indCurve = dStr.AddCurve(tcint2);

            cint2.D0(cint2.FirstParameter, out pext1);
            cint2.D0(cint2.LastParameter, out pext2);

            if (pext1.DistanceTo(pext) <= tol3d)
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
            if (pext1.DistanceTo(point2) > tol3d && pext2.DistanceTo(point1) > tol3d)
            {
                orSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(orSurf);
            }

            // ---------------------------------------------------------------
            // storage of aCint2
            interfp1 = ChFi3d_FilPointInDS(Forward, indCurve, indPoint1, cint2.FirstParameter);
            interfp2 = ChFi3d_FilPointInDS(Reversed, indCurve, indPoint2, cint2.LastParameter);
            dStr.ChangeCurveInterferences(indCurve).Append(interfp1);
            dStr.ChangeCurveInterferences(indCurve).Append(interfp2);

            // interference of aCint2 on the SurfData number anIndPrev
            interfc = ChFi3d_FilCurveInDS(indCurve, surfData.Surf(), pCint22, orSurf);

            dStr.ChangeSurfaceInterferences(surfData.Surf()).Append(interfc);
            // interference of aCint2 on aFace

            if (orFace == orSD2)
            {
                orFace = global::Kerf.Engine.Topology.TopAbs.Reverse(orSurf);
            }
            else
            {
                orFace = orSurf;
            }

            interfc = ChFi3d_FilCurveInDS(indCurve, indaFace, pCint21, orFace);
            dStr.ChangeShapeInterferences(indaFace).Append(interfc);
        }

        trCracc.D0(trCracc.FirstParameter, out pext1);
        trCracc.D0(trCracc.LastParameter, out pext2);
        if (pext1.DistanceTo(cp2NotonArc.Point()) <= tol3d)
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
        Point3D p1 = new();
        Point3D p2 = new();
        Point3D p3 = new();
        Point3D p4 = new();

        if (isDoSecondSection)
        {
            trCracc.D0(trCracc.FirstParameter, out p1);
            trCracc.D0(trCracc.LastParameter, out p2);
            cint2.D0(cint2.FirstParameter, out p3);
            cint2.D0(cint2.LastParameter, out p4);
            isToReverse = (p1.DistanceTo(p4) > tol3d && p2.DistanceTo(p3) > tol3d);
        }
        else
        {
            isToReverse = (pext1.DistanceTo(point2) > tol3d && pext2.DistanceTo(point1) > tol3d);
        }

        if (isToReverse)
        {
            orSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(orSurf);
        }

        // ---------------------------------------------------------------
        // storage of aTrCracc
        DsCurve tct2 = new(trCracc, tolReached);

        indCurve = dStr.AddCurve(tct2);
        interfp1 = ChFi3d_FilPointInDS(Forward, indCurve, indPoint1, trCracc.FirstParameter);
        interfp2 = ChFi3d_FilPointInDS(Reversed, indCurve, indPoint2, trCracc.LastParameter);
        dStr.ChangeCurveInterferences(indCurve).Append(interfp1);
        dStr.ChangeCurveInterferences(indCurve).Append(interfp2);

        // interference of aTrCracc on the SurfData number anIndPrev

        interfc = ChFi3d_FilCurveInDS(indCurve, surfData.Surf(), pCraccS, orSurf);
        dStr.ChangeSurfaceInterferences(surfData.Surf()).Append(interfc);
        stripe.InDS(isFirst);

        // interference of aTrCracc on the SurfData number anInd
        if (orSD1 == orSD2)
        {
            orSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(orSurf);
        }

        interfc = ChFi3d_FilCurveInDS(indCurve, indSurf1, pCurv1, orSurf);
        dStr.ChangeSurfaceInterferences(indSurf1).Append(interfc);

        // ---------------------------------------------------------------
        // storage of aCint1

        cint1.D0(cint1.FirstParameter, out pext1);
        if (pext1.DistanceTo(cp1onArc.Point()) <= tol3d)
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

        cint1.D0(cint1.FirstParameter, out p1);
        cint1.D0(cint1.LastParameter, out p2);
        trCracc.D0(trCracc.FirstParameter, out p3);
        trCracc.D0(trCracc.LastParameter, out p4);

        if (p1.DistanceTo(p4) > tol3d && p2.DistanceTo(p3) > tol3d)
        {
            orSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(orSurf);
        }

        DsCurve tCint1 = new(cint1, tolex1);
        indCurve = dStr.AddCurve(tCint1);
        interfp1 = ChFi3d_FilPointInDS(Forward, indCurve, indPoint1, cint1.FirstParameter);
        interfp2 = ChFi3d_FilPointInDS(Reversed, indCurve, indPoint2, cint1.LastParameter);
        dStr.ChangeCurveInterferences(indCurve).Append(interfp1);
        dStr.ChangeCurveInterferences(indCurve).Append(interfp2);

        // interference of aCint1 on the SurfData number anInd

        interfc = ChFi3d_FilCurveInDS(indCurve, indSurf1, pCint12, orSurf);
        dStr.ChangeSurfaceInterferences(indSurf1).Append(interfc);

        // interference of aCint1 on aFace

        orFace = face.Orientation;

        if (orFace == orSD1)
        {
            orFace = global::Kerf.Engine.Topology.TopAbs.Reverse(orSurf);
        }
        else
        {
            orFace = orSurf;
        }

        interfc = ChFi3d_FilCurveInDS(indCurve, indaFace, pCint11, orFace);
        dStr.ChangeShapeInterferences(indaFace).Append(interfc);
        // ---------------------------------------------------------------
        // storage of aCline passing through aCP1onArc and aCP2NotonArc

        Curve3D? trCline = 
            new TrimmedCurve3D(cline, cline.FirstParameter, cline.LastParameter);
        double tolerance = dStr.Curve(indLine).Tolerance();
        DsCurve tct3 = new(trCline, tolerance);

        indCurve = dStr.AddCurve(tct3);

        trCline.D0(trCline.FirstParameter, out pext1);

        if (pext1.DistanceTo(cp1onArc.Point()) < tol3d)
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

        trCline.D0(trCline.FirstParameter, out p1);
        trCline.D0(trCline.LastParameter, out p2);
        cint1.D0(cint1.FirstParameter, out p3);
        cint1.D0(cint1.LastParameter, out p4);

        if (p1.DistanceTo(p4) > tol3d && p2.DistanceTo(p3) > tol3d)
        {
            orSurf = global::Kerf.Engine.Topology.TopAbs.Reverse(orSurf);
        }

        interfp1 = ChFi3d_FilPointInDS(Forward, indCurve, indPoint1, trCline.FirstParameter);
        interfp2 = ChFi3d_FilPointInDS(Reversed, indCurve, indPoint2, trCline.LastParameter);
        dStr.ChangeCurveInterferences(indCurve).Append(interfp1);
        dStr.ChangeCurveInterferences(indCurve).Append(interfp2);

        // interference of aTrCline on the SurfData number anInd

        interfc = ChFi3d_FilCurveInDS(indCurve, indSurf1, pClineOnSurf, orSurf);
        dStr.ChangeSurfaceInterferences(indSurf1).Append(interfc);

        // interference de ctlin par rapport a Fvoisin
        indShape = dStr.AddShape(neighborFace);
        orFace = neighborFace.Orientation;

        if (orFace == orSD1)
        {
            orFace = global::Kerf.Engine.Topology.TopAbs.Reverse(orSurf);
        }
        else
        {
            orFace = orSurf;
        }

        interfc = ChFi3d_FilCurveInDS(indCurve, indShape, pClineOnFace, orFace);
        dStr.ChangeShapeInterferences(indShape).Append(interfc);
    }

    //  Modified by Sergey KHROMOV - Thu Apr 11 12:23:40 2002 End

    //==============================================================
    // function : FindFace
    // purpose  : attention it works only if there is only one common face
    //           between P1,P2,V
    //===========================================================

    private bool FindFace(in Vertex v, in CommonPoint p1, in CommonPoint p2, ref Face fv)
    {
        Face favoid = new();
        return FindFace(p1, p2, ref fv, favoid);
    }

    private bool FindFace(in CommonPoint p1, in CommonPoint p2, ref Face fv, in Face favoid)
    {
        if (p1.IsVertex || p2.IsVertex)
        {
        }
        if (!(p1.IsOnArc && p2.IsOnArc))
        {
            return false;
        }
        NCollection_List<Shape>.Iterator it = new(), jt = new();
        bool found = false;
        for (it.Initialize(_efMap[p1.Arc()]); it.More() && !found; it.Next())
        {
            fv = ShapeCasts.Face(it.Value());
            if (!fv.IsSame(favoid))
            {
                for (jt.Initialize(_efMap[p2.Arc()]); jt.More() && !found; jt.Next())
                {
                    if (ShapeCasts.Face(jt.Value()).IsSame(fv))
                    {
                        found = true;
                    }
                }
            }
        }
        return found;
    }

    //=======================================================================
    // function : MoreSurfdata
    // purpose  : detects if the intersection at end concerns several Surfdata
    //=======================================================================
    private bool MoreSurfdata(int index)
    {
        // intersection at end is created on several surfdata if :
        // - the number of surfdata concerning the vertex is more than 1.
        // - and if the last but one surfdata has one of commonpoints on one of
        // two arcs, which constitute the intersections of the face at end and of the fillet

        NCollection_List<Stripe>.Iterator it = new();
        it.Initialize(_vDataMap[index]);
        Stripe? stripe = it.ChangeValue();
        NCollection_Sequence<SurfData> seqFil = 
            stripe.ChangeSetOfSurfData();
        Vertex vtx = _vDataMap.FindKey(index);
        int sens = 0;
        int num = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
        bool isfirst = (sens == 1);
        SurfData? fd = seqFil.ChangeValue(num);
        ref CommonPoint cv1 = ref fd.ChangeVertex(isfirst, 1);
        ref CommonPoint cv2 = ref fd.ChangeVertex(isfirst, 2);

        int num1 = 0, num2 = 0, nbsurf = 0;
        Face fv = new();
        bool inters = false, oksurf = false;
        nbsurf = stripe.SetOfSurfData().Length();
        // Fv is the face at end
        inters = FindFace(vtx, cv1, cv2, ref fv);
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
            NCollection_List<Shape>.Iterator itE = new();
            bool trouve = false;
            for (itE.Initialize(_veMap[vtx]); itE.More() && !trouve; itE.Next())
            {
                arc1 = ShapeCasts.Edge(itE.Value());
                if (ContainE(fv, arc1))
                {
                    trouve = true;
                }
            }
            trouve = false;
            for (itE.Initialize(_veMap[vtx]); itE.More() && !trouve; itE.Next())
            {
                arc2 = ShapeCasts.Edge(itE.Value());
                if (ContainE(fv, arc2) && !arc2.IsSame(arc1))
                {
                    trouve = true;
                }
            }

            SurfData? fd1 = seqFil.ChangeValue(num2);
            ref CommonPoint cv3 = ref fd1.ChangeVertex(isfirst, 1);
            ref CommonPoint cv4 = ref fd1.ChangeVertex(isfirst, 2);

            if (cv3.IsOnArc)
            {
                if (cv3.Arc().IsSame(arc1))
                {
                    if (cv1.Point().DistanceTo(cv3.Point()) < 1.0e-4)
                    {
                        oksurf = true;
                    }
                }
                else if (cv3.Arc().IsSame(arc2))
                {
                    if (cv2.Point().DistanceTo(cv3.Point()) < 1.0e-4)
                    {
                        oksurf = true;
                    }
                }
            }

            if (cv4.IsOnArc)
            {
                if (cv1.Point().DistanceTo(cv4.Point()) < 1.0e-4)
                {
                    oksurf = true;
                }
                else if (cv4.Arc().IsSame(arc2))
                {
                    if (cv2.Point().DistanceTo(cv4.Point()) < 1.0e-4)
                    {
                        oksurf = true;
                    }
                }
            }
        }
        return oksurf;
    }

    // Case of fillets on top with 4 edges, one of them is on the same geometry as the edgeof the fillet

    private void IntersectMoreCorner(int index)
    {
        DsDataStructure dStr = _ds.ChangeDS();

        // The fillet is returned,
        NCollection_List<Stripe>.Iterator strIt = new();
        strIt.Initialize(_vDataMap[index]);
        Stripe? stripe = strIt.Value();
        Spine? spine = stripe.Spine();
        NCollection_Sequence<SurfData> seqFil = 
            stripe.ChangeSetOfSurfData();
        // the top,
        Vertex vtx = _vDataMap.FindKey(index);
        // the SurfData concerned and its CommonPoints,
        int sens = 0;

        // Choose the proper SurfData
        int num = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
        bool isfirst = (sens == 1);
        if (isfirst)
        {
            for (; num < seqFil.Length()
                          && ((seqFil.Value(num).IndexOfS1() == 0) || (seqFil.Value(num).IndexOfS2() == 0));)
            {
                seqFil.Remove(num); // The surplus is removed
            }
        }
        else
        {
            for (; num > 1
                          && ((seqFil.Value(num).IndexOfS1() == 0) || (seqFil.Value(num).IndexOfS2() == 0));)
            {
                seqFil.Remove(num); // The surplus is removed
                num--;
            }
        }

        SurfData? fd = seqFil.ChangeValue(num);
        ref CommonPoint cv1 = ref fd.ChangeVertex(isfirst, 1);
        ref CommonPoint cv2 = ref fd.ChangeVertex(isfirst, 2);
        // To evaluate the cloud of new points.
        Box box1 = new(), box2 = new();

        // The cases of cap are processed separately from intersection.
        // ----------------------------------------------------------

        Face fv = new(), fad = new(), fop = new(), fopbis = new();
        Edge arcpiv = new(), arcprol = new(), arcspine = new(), arcprolbis = new();
        if (isfirst)
        {
            arcspine = spine.Edges(1);
        }
        else
        {
            arcspine = spine.Edges(spine.EdgeCount);
        }
        Orientation oArcprolbis = Forward;
        Orientation oArcprolv = Forward, oArcprolop = Forward;
        int iCurve = 0;
        FaceSurfaceAdaptor? hBs = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hBad = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hBop = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor bs = hBs;
        FaceSurfaceAdaptor bad = hBad;
        FaceSurfaceAdaptor bop = hBop;
        Curve3D? cc = null;
        Curve2D? pc = null, ps = null;
        double ubid = 0.0, vbid = 0.0;//,mu,Mu,mv,Mv;
        double udeb = 0.0, ufin = 0.0;
        // gp_Pnt2d UVf1,UVl1,UVf2,UVl2;
        // double Du,Dv,Step;
        bool inters = true;
        int iFadArc = 1, iFopArc = 2;
        fop = ShapeCasts.Face(dStr.Shape(fd.Index(iFopArc)));
        Explorer ex = new();

        {
            if (!cv1.IsOnArc && !cv2.IsOnArc)
            {
                throw new FailureException("Corner intersmore : no point on arc");
            }
            else if (cv1.IsOnArc && cv2.IsOnArc)
            {
                bool sur2 = false;
                for (ex.Init(cv1.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (vtx.IsSame(ex.Current()))
                    {
                        break;
                    }
                }
                for (ex.Init(cv2.Arc(), ShapeType.Vertex); ex.More(); ex.Next())
                {
                    if (vtx.IsSame(ex.Current()))
                    {
                        sur2 = true;
                        break;
                    }
                }
                if (sur2)
                {
                    iFadArc = 2;
                }
            }
            else if (cv2.IsOnArc)
            {
                iFadArc = 2;
            }
            iFopArc = 3 - iFadArc;

            arcpiv = fd.Vertex(isfirst, iFadArc).Arc();
            fad = ShapeCasts.Face(dStr.Shape(fd.Index(iFadArc)));
            fop = ShapeCasts.Face(dStr.Shape(fd.Index(iFopArc)));
            NCollection_List<Shape>.Iterator it = new();
            // The face at end is returned without control of its unicity.
            for (it.Initialize(_efMap[arcpiv]); it.More(); it.Next())
            {
                if (!fad.IsSame(it.Value()))
                {
                    fv = ShapeCasts.Face(it.Value());
                    break;
                }
            }

            // does the face at end contain the Vertex ?
            bool isinface = false;
            for (ex.Init(fv, ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (ex.Current().IsSame(vtx))
                {
                    isinface = true;
                    break;
                }
            }
            if (!isinface)
            {
                iFadArc = 3 - iFadArc;
                iFopArc = 3 - iFopArc;
                arcpiv = fd.Vertex(isfirst, iFadArc).Arc();
                fad = ShapeCasts.Face(dStr.Shape(fd.Index(iFadArc)));
                fop = ShapeCasts.Face(dStr.Shape(fd.Index(iFopArc)));
                // NCollection_List<TopoDS_Shape>::Iterator It;
                // The face at end is returned without control of its unicity.
                for (it.Initialize(_efMap[arcpiv]); it.More(); it.Next())
                {
                    if (!fad.IsSame(it.Value()))
                    {
                        fv = ShapeCasts.Face(it.Value());
                        break;
                    }
                }
            }

            if (fv.IsNull)
            {
                throw new NotDoneException("OneCorner : face at end is not found");
            }

            fv.Orient(Forward);
            fad.Orient(Forward);

            // In the same way the edge to be extended is returned.
            for (it.Initialize(_veMap[vtx]); it.More() && arcprol.IsNull; it.Next())
            {
                if (!arcpiv.IsSame(it.Value()))
                {
                    for (ex.Init(fv, ShapeType.Edge); ex.More(); ex.Next())
                    {
                        if (it.Value().IsSame(ex.Current()))
                        {
                            arcprol = ShapeCasts.Edge(it.Value());
                            oArcprolv = ex.Current().Orientation;
                            break;
                        }
                    }
                }
            }

            // Guard: Arcprol may be null if the loop above found no matching edge.
            if (arcprol.IsNull)
            {
                throw new NotDoneException("IntersectMoreCorner: edge to be extended is not found");
            }

            // Fopbis is the face containing the trace of fillet CP.Arc() which of does not contain Vtx.
            // Normally Fobis is either the same as Fop (cylinder), or Fobis is G1 with Fop.
            fopbis.Orient(Forward);

            // Fop calls the 4th face non-used for the vertex
            ChercheFace(_vfMap[vtx], arcprol, fad, fv, fv, ref fopbis);
            fop.Orient(Forward);
            for (ex.Init(fopbis, ShapeType.Edge); ex.More(); ex.Next())
            {
                if (arcprol.IsSame(ex.Current()))
                {
                    oArcprolop = ex.Current().Orientation;
                    break;
                }
            }
            Face fFv = new();
            double tol = 0.0;
            int prol = 0;
            BrepBuilder bre = new();
            Surface? sface = null;
            sface = Tool.Surface(fv);
            ChFi3d_ExtendSurface(ref sface, ref prol);
            tol = Tool.Tolerance(fv);
            bre.MakeFace(ref fFv, sface, tol);
            if (prol != 0)

            {
                bs.Initialize(fFv, false);
                dStr.SetNewSurface(fv, sface);
            }
            else
            {
                bs.Initialize(fv, false);
            }
            bad.Initialize(fad);
            bop.Initialize(fop);
        }
        // it is necessary to modify the CommonPoint
        // in the space and its parameter in FaceInterference.
        // So both of them are returned in references
        // non const. Attention the modifications are done behind
        // CV1,CV2,Fi1,Fi2.
        ref CommonPoint cPopArc = ref fd.ChangeVertex(isfirst, iFopArc);
        ref FaceInterference fiopArc = ref fd.ChangeInterference(iFopArc);
        ref CommonPoint cPadArc = ref fd.ChangeVertex(isfirst, iFadArc);
        ref FaceInterference fiadArc = ref fd.ChangeInterference(iFadArc);
        // the parameter of the vertex is initialized with the value
        // of its opposing vertex (point on arc).
        double wop = fd.ChangeInterference(iFadArc).Parameter(isfirst);
        Curve3D? c3df = null;
        SurfaceAdaptor? hGs = 
            new SurfaceAdaptor(dStr.Surface(fd.Surf()).Surface());
        Point2D p2dbout = new();
        {

            // add here more or less restrictive criteria to
            // decide if the intersection with face is done at the
            // extended end or if there will be a cap on sharp end.
            c3df = dStr.Curve(fiopArc.LineIndex()).Curve();
            double uf = fiopArc.FirstParameter();
            double ul = fiopArc.LastParameter();
            CurveAdaptor? hc3df = null;
            if (c3df.IsPeriodic)
            {
                hc3df = new CurveAdaptor(c3df);
            }
            else
            {
                hc3df = new CurveAdaptor(c3df, uf, ul);
            }
            inters = Update(hBs, hc3df, ref fiopArc, ref cPopArc, ref p2dbout, isfirst, ref wop);
            //  Modified by Sergey KHROMOV - Fri Dec 21 18:08:27 2001 Begin
            //  if(!inters && BRep_Tool::Continuity(Arcprol,Fv,Fop) != GeomAbs_C0){
            if (!inters && global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(arcprol, fv, fop))
            {
                //  Modified by Sergey KHROMOV - Fri Dec 21 18:08:29 2001 End
                // Arcprol is an edge of tangency, ultimate adjustment by an extrema curve/curve is attempted.
                double ff = 0.0, ll = 0.0;
                Curve2D? gpcprol = Tool.CurveOnSurface(arcprol, fv, out ff, out ll);
                if (gpcprol is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                CurveAdaptor2D? pcprol = new CurveAdaptor2D(gpcprol);
                double partemp = Tool.Parameter(vtx, arcprol);
                inters =
                    Update(hBs, pcprol, hGs, ref fiopArc, ref cPopArc, ref p2dbout, isfirst, ref partemp, ref wop, 10 * _tolapp3d);
            }
            EdgeCurve2DAdaptor? pced = new EdgeCurve2DAdaptor();
            pced.Initialize(cPadArc.Arc(), fv);
            Update(hBs, pced, hGs, ref fiadArc, ref cPadArc, isfirst);
        }

        Edge edgecouture = new();
        bool couture = false, intcouture = false;
        double tolreached = _tolapp3d;
        double par1 = 0.0, par2 = 0.0;
        int indpt = 0, icurv1 = 0, icurv2 = 0;
        TrimmedCurve3D? curv1 = null, curv2 = null;
        Curve2D? c2d1 = null, c2d2 = null;

        int isurf = fd.Surf();

        if (inters)
        {
            hGs = ChFi3d_BoundSurf(dStr, fd, 1, 2);
            FaceInterference fi1 = fd.InterferenceOnS1();
            FaceInterference fi2 = fd.InterferenceOnS2();
            NCollection_Array1<double> pardeb = new(1, 4), parfin = new(1, 4);
            Point2D pfil1 = new(), pfac1 = new(), pfil2 = new(), pfac2 = new();
            Curve2D? hc1 = null, hc2 = null;
            if (iFopArc == 1)
            {
                pfac1 = p2dbout;
            }
            else
            {
                hc1 = Tool.CurveOnSurface(cv1.Arc(), fv, out ubid, out ubid);
                if (hc1 is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                pfac1 = hc1.Value(cv1.ParameterOnArc());
            }
            if (iFopArc == 2)
            {
                pfac2 = p2dbout;
            }
            else
            {
                hc2 = Tool.CurveOnSurface(cv2.Arc(), fv, out ubid, out ubid);
                if (hc2 is null)
                {
                    throw new ConstructionErrorException("Failed to get p-curve of edge");
                }
                pfac2 = hc2.Value(cv2.ParameterOnArc());
            }
            if (fi1.LineIndex() != 0)
            {
                pfil1 = fi1.PCurveOnSurf().Value(fi1.Parameter(isfirst));
            }
            else
            {
                pfil1 = fi1.PCurveOnSurf().Value(fi1.Parameter(!isfirst));
            }
            if (fi2.LineIndex() != 0)
            {
                pfil2 = fi2.PCurveOnSurf().Value(fi2.Parameter(isfirst));
            }
            else
            {
                pfil2 = fi2.PCurveOnSurf().Value(fi2.Parameter(!isfirst));
            }
            ChFi3d_Recale(bs, ref pfac1, ref pfac2, (iFadArc == 1));
            pardeb[1] = pfil1.X;
            pardeb[2] = pfil1.Y;
            pardeb[3] = pfac1.X;
            pardeb[4] = pfac1.Y;
            parfin[1] = pfil2.X;
            parfin[2] = pfil2.Y;
            parfin[3] = pfac2.X;
            parfin[4] = pfac2.Y;
            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pfac1, pfac2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(bs, uu1, uu2, vv1, vv2);

            if (!ChFi3d_ComputeCurves(hGs, hBs, pardeb, parfin, ref cc, ref ps, ref pc, _tolapp3d, _tol2d, out tolreached))
            {
                throw new FailureException("OneCorner : failed calculation intersection");
            }

            udeb = cc.FirstParameter;
            ufin = cc.LastParameter;

            // check if the curve has an intersection with sewing edge

            ChFi3d_Couture(fv, out couture, ref edgecouture);

            if (couture && !Tool.Degenerated(edgecouture))
            {

                // double Ubid,Vbid;
                Curve3D? c = Tool.Curve(edgecouture, out ubid, out vbid);
                TrimmedCurve3D? ctrim = new TrimmedCurve3D(c, ubid, vbid);
                CurveAdaptor cur1 = new(ctrim.BasisCurve);
                CurveAdaptor cur2 = new(cc);
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
                        if (Math.Abs(par2 - udeb) > Tol && Math.Abs(ufin - par2) > Tol)
                        {
                            Point3D p1 = ponc1.Value();
                            DsPoint tpoint = new(p1, Tol);
                            indpt = dStr.AddPoint(tpoint);
                            intcouture = true;
                            curv1 = new TrimmedCurve3D(cc, udeb, par2);
                            curv2 = new TrimmedCurve3D(cc, par2, ufin);
                            DsCurve tcurv1 = new(curv1, tolreached);
                            DsCurve tcurv2 = new(curv2, tolreached);
                            icurv1 = dStr.AddCurve(tcurv1);
                            icurv2 = dStr.AddCurve(tcurv2);
                        }
                    }
                }
            }
        }

        else
        {
            throw new MethodNotImplementedException("OneCorner : cap not written");
        }
        int iShape = dStr.AddShape(fv);
        Orientation et = Forward;
        if (iFadArc == 1)
        {
            Explorer exp = new();
            for (exp.Init(fv.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(cv1.Arc()))
                {
                    et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(exp.Current().Orientation, cv1.TransitionOnArc()));
                    break;
                }
            }
        }
        else
        {
            Explorer exp = new();
            for (exp.Init(fv.Oriented(Forward), ShapeType.Edge); exp.More(); exp.Next())
            {
                if (exp.Current().IsSame(cv2.Arc()))
                {
                    et = global::Kerf.Engine.Topology.TopAbs.Compose(exp.Current().Orientation, cv2.TransitionOnArc());
                    break;
                }
            }

            //
        }


        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cv1, dStr), isfirst, 1);
        stripe.SetIndexPoint(ChFi3d_IndexPointInDS(cv2, dStr), isfirst, 2);

        if (!intcouture)
        {
            // there is no intersection with edge of sewing
            // curve Cc is stored in the stripe
            // the storage in the DS is done by FILDS.

            DsCurve tc = new(cc, tolreached);
            iCurve = dStr.AddCurve(tc);
            DsSurfaceCurveInterference? interfc = 
                ChFi3d_FilCurveInDS(iCurve, iShape, pc, et);
            dStr.ChangeShapeInterferences(iShape).Append(interfc);
            stripe.ChangePCurve(isfirst) = ps;
            stripe.SetCurve(iCurve, isfirst);
            stripe.SetParameters(isfirst, udeb, ufin);
        }
        else
        {
            // curves curv1 and curv2 are stored in the DS
            // these curves are not reconstructed by FILDS as
            // stripe->InDS(isfirst) is placed;

            // interferences of curv1 and curv2 on Fv
            ComputeCurve2d(curv1, ref fv, ref c2d1);
            DsSurfaceCurveInterference? interFv = null;
            interFv = ChFi3d_FilCurveInDS(icurv1, iShape, c2d1, et);
            dStr.ChangeShapeInterferences(iShape).Append(interFv);
            ComputeCurve2d(curv2, ref fv, ref c2d2);
            interFv = ChFi3d_FilCurveInDS(icurv2, iShape, c2d2, et);
            dStr.ChangeShapeInterferences(iShape).Append(interFv);
            // interferences of curv1 and curv2 on Isurf
            if (fd.Orientation() == fv.Orientation)
            {
                et = global::Kerf.Engine.Topology.TopAbs.Reverse(et);
            }
            c2d1 = new TrimmedCurve2D(ps, udeb, par2);
            interFv = ChFi3d_FilCurveInDS(icurv1, isurf, c2d1, et);
            dStr.ChangeSurfaceInterferences(isurf).Append(interFv);
            c2d2 = new TrimmedCurve2D(ps, par2, ufin);
            interFv = ChFi3d_FilCurveInDS(icurv2, isurf, c2d2, et);
            dStr.ChangeSurfaceInterferences(isurf).Append(interFv);

            // limitation of the sewing edge
            int iarc = dStr.AddShape(edgecouture);
            DsCurvePointInterference? interfedge = null;
            Orientation ori = default;
            Vertex vdeb = new(), vfin = new();
            vdeb = Exploration.FirstVertex(edgecouture);
            vfin = Exploration.LastVertex(edgecouture);
            double pard = 0.0, parf = 0.0;
            pard = Tool.Parameter(vdeb, edgecouture);
            parf = Tool.Parameter(vfin, edgecouture);
            if (Math.Abs(par1 - pard) < Math.Abs(parf - par1))
            {
                ori = Forward;
            }
            else
            {
                ori = Reversed;
            }
            interfedge = ChFi3d_FilPointInDS(ori, iarc, indpt, par1);
            dStr.ChangeShapeInterferences(iarc).Append(interfedge);

            // creation of CurveInterferences from Icurv1 and Icurv2
            stripe.InDS(isfirst);
            int ind1 = stripe.IndexPoint(isfirst, 1);
            int ind2 = stripe.IndexPoint(isfirst, 2);
            DsCurvePointInterference? interfprol = 
                ChFi3d_FilPointInDS(Forward, icurv1, ind1, udeb);
            dStr.ChangeCurveInterferences(icurv1).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Reversed, icurv1, indpt, par2);
            dStr.ChangeCurveInterferences(icurv1).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Forward, icurv2, indpt, par2);
            dStr.ChangeCurveInterferences(icurv2).Append(interfprol);
            interfprol = ChFi3d_FilPointInDS(Reversed, icurv2, ind2, ufin);
            dStr.ChangeCurveInterferences(icurv2).Append(interfprol);
        }

        ChFi3d_EnlargeBox(hBs, pc, udeb, ufin, ref box1, ref box2);

        if (inters)
        {
            //

            // The small end of curve missing for the extension
            // of the face at end and the limitation of the opposing face is added.

            // Above all the points cut the points with the edge of the spine.
            int iArcspine = dStr.AddShape(arcspine);
            int iVtx = dStr.AddShape(vtx);
            Orientation oVtx2 = Forward;
            Orientation oVtx = Forward;
            for (ex.Init(arcspine.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (vtx.IsSame(ex.Current()))
                {
                    oVtx = ex.Current().Orientation;
                    break;
                }
            }
            oVtx = global::Kerf.Engine.Topology.TopAbs.Reverse(oVtx);
            double parVtx = Tool.Parameter(vtx, arcspine);
            DsCurvePointInterference? interfv = 
                ChFi3d_FilVertexInDS(oVtx, iArcspine, iVtx, parVtx);
            dStr.ChangeShapeInterferences(iArcspine).Append(interfv);

            // Modif of lvt to find the suite of Arcprol in the other face
            {
                NCollection_List<Shape>.Iterator it = new();
                for (it.Initialize(_veMap[vtx]); it.More(); it.Next())
                {
                    if (!(arcprol.IsSame(it.Value()) || arcspine.IsSame(it.Value())
                                || arcpiv.IsSame(it.Value())))
                    {
                        arcprolbis = ShapeCasts.Edge(it.Value());
                        break;
                    }
                }
            }
            // end of modif

            // Guard: Arcprolbis may be null if myVEMap[Vtx] contained only
            // Arcprol, Arcspine, and Arcpiv (e.g. when topology maps carry stale
            // data from shared TShape pointers after prior operations).
            if (arcprolbis.IsNull)
            {
                throw new ConstructionErrorException("IntersectMoreCorner: continuation edge (Arcprolbis) is not found");
            }

            // Now the missing curves are constructed.
            for (ex.Init(arcprolbis.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (vtx.IsSame(ex.Current()))
                {
                    oVtx2 = ex.Current().Orientation;
                    break;
                }
            }
            for (ex.Init(arcprol.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
            {
                if (vtx.IsSame(ex.Current()))
                {
                    oVtx = ex.Current().Orientation;
                    break;
                }
            }
            // it is checked if Fop has a sewing edge

            //     TopoDS_Edge edgecouture;
            //     bool couture;
            ChFi3d_Couture(fop, out couture, ref edgecouture);
            Curve2D? hc = null;
            //    parVtx = BRep_Tool::Parameter(Vtx,Arcprol);
            FaceInterference fiop = fd.Interference(iFopArc);
            Point2D pop1 = new(), pop2 = new(), pv1 = new(), pv2 = new();
            // deb modif
            parVtx = Tool.Parameter(vtx, arcprolbis);
            //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222 Begin
            //    if(Fop.IsSame(Fopbis)) OArcprolbis = OArcprolop;
            //    else OArcprolbis = Arcprolbis.Orientation();
            if (fop.IsSame(fopbis))
            {
                oArcprolbis = oArcprolop;
            }
            else
            {
                for (ex.Init(fop, ShapeType.Edge); ex.More(); ex.Next())
                {
                    if (arcprolbis.IsSame(ex.Current()))
                    {
                        oArcprolbis = ex.Current().Orientation;
                        break;
                    }
                }
            }
            //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222 End
            // fin modif
            hc = Tool.CurveOnSurface(arcprolbis, fop, out ubid, out ubid);
            if (hc is null)
            {
                throw new ConstructionErrorException("Failed to get p-curve of edge");
            }
            pop1 = hc.Value(parVtx);
            pop2 = fiop.PCurveOnFace().Value(fiop.Parameter(isfirst));
            hc = Tool.CurveOnSurface(arcprol, fv, out ubid, out ubid);
            if (hc is null)
            {
                throw new ConstructionErrorException("Failed to get p-curve of edge");
            }
            // modif
            parVtx = Tool.Parameter(vtx, arcprol);
            // fin modif
            pv1 = hc.Value(parVtx);
            pv2 = p2dbout;
            ChFi3d_Recale(bs, ref pv1, ref pv2, true);
            NCollection_Array1<double> pardeb = new(1, 4), parfin = new(1, 4);
            pardeb[1] = pop1.X;
            pardeb[2] = pop1.Y;
            pardeb[3] = pv1.X;
            pardeb[4] = pv1.Y;
            parfin[1] = pop2.X;
            parfin[2] = pop2.Y;
            parfin[3] = pv2.X;
            parfin[4] = pv2.Y;
            double uu1 = 0.0, uu2 = 0.0, vv1 = 0.0, vv2 = 0.0;
            ChFi3d_Boite(pv1, pv2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(bs, uu1, uu2, vv1, vv2);
            ChFi3d_Boite(pop1, pop2, out uu1, out uu2, out vv1, out vv2);
            ChFi3d_BoundFac(bop, uu1, uu2, vv1, vv2);

            Curve3D? zob3d = null;
            Curve2D? zob2dop = null, zob2dv = null;
            //    double tolreached;
            if (!ChFi3d_ComputeCurves(hBop, hBs, pardeb, parfin, ref zob3d, ref zob2dop, ref zob2dv, _tolapp3d, _tol2d,
                                                                out tolreached))
            {
                throw new FailureException("OneCorner : echec calcul intersection");
            }

            udeb = zob3d.FirstParameter;
            ufin = zob3d.LastParameter;
            DsCurve zob = new(zob3d, tolreached);
            int iZob = dStr.AddCurve(zob);

            // it is not determined if the curve has an intersection with the sewing edge

            {
                et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(oVtx, oArcprolv));
                int iop = dStr.AddShape(fop);
                DsSurfaceCurveInterference? interFv = 
                    ChFi3d_FilCurveInDS(iZob, iShape, zob2dv, et);
                dStr.ChangeShapeInterferences(iShape).Append(interFv);
                // OVtx = TopAbs::Reverse(OVtx);
                //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222 Begin
                //      Et = TopAbs::Reverse(TopAbs::Compose(OVtx,OArcprolbis));
                et = global::Kerf.Engine.Topology.TopAbs.Reverse(global::Kerf.Engine.Topology.TopAbs.Compose(oVtx2, oArcprolbis));
                //  Modified by skv - Thu Aug 21 11:55:58 2008 OCC20222 End
                // OVtx = TopAbs::Reverse(OVtx);
                //      Et = TopAbs::Reverse(Et);
                DsSurfaceCurveInterference? interfop = 
                    ChFi3d_FilCurveInDS(iZob, iop, zob2dop, et);
                dStr.ChangeShapeInterferences(iop).Append(interfop);
                DsCurvePointInterference? interfprol = 
                    ChFi3d_FilVertexInDS(Forward, iZob, iVtx, udeb);
                dStr.ChangeCurveInterferences(iZob).Append(interfprol);
                int icc = stripe.IndexPoint(isfirst, iFopArc);
                interfprol = ChFi3d_FilPointInDS(Reversed, iZob, icc, ufin);
                dStr.ChangeCurveInterferences(iZob).Append(interfprol);
            }
        }
        ChFi3d_EnlargeBox(dStr, stripe, fd, ref box1, ref box2, isfirst);
        if (cv1.IsOnArc)
        {
            ChFi3d_EnlargeBox(cv1.Arc(), _efMap[cv1.Arc()], cv1.ParameterOnArc(), ref box1);
        }
        if (cv2.IsOnArc)
        {
            ChFi3d_EnlargeBox(cv2.Arc(), _efMap[cv2.Arc()], cv2.ParameterOnArc(), ref box2);
        }
        if (!cv1.IsVertex)
        {
            ChFi3d_SetPointTolerance(dStr, box1, stripe.IndexPoint(isfirst, 1));
        }
        if (!cv2.IsVertex)
        {
            ChFi3d_SetPointTolerance(dStr, box2, stripe.IndexPoint(isfirst, 2));
        }

    }

}

internal static class Builder_C1
{
    internal static double Recadre(double p, double @ref, bool isfirst, double first, double last)
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

    internal static bool Update(SurfaceAdaptorBase? fb, Curve2DAdaptorBase? pcfb, SurfaceAdaptorBase? surf, ref FaceInterference fi, ref CommonPoint cp, ref Point2D p2dbout, bool isfirst, ref double pared, ref double wop, double tol)
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

    internal static bool Update(SurfaceAdaptorBase? fb, CurveAdaptorBase? ct, ref FaceInterference fi, ref CommonPoint cp, ref Point2D p2dbout, bool isfirst, ref double wop)
    {
        HInter intersection = new();
        // check if in KPart the limits of the tangency line
        // are already in place at this stage.
        // Modif lvt : the periodic cases are reframed, espercially if nothing was found.
        double w = 0.0, uf = ct.FirstParameter(), ul = ct.LastParameter();

        double wbis = 0.0;

        bool isperiodic = ct.IsPeriodic, recadrebis = false;
        intersection.Perform(ct, fb);
        if (intersection.IsDone)
        {
            int nbp = intersection.PointCount, i, isol = 0, isolbis = 0;
            double dist = global::Kerf.Engine.Foundation.Precision.Infinite();
            double distbis = global::Kerf.Engine.Foundation.Precision.Infinite();
            for (i = 1; i <= nbp; i++)
            {
                w = intersection.Point(i).W();
                if (isperiodic)
                {
                    w = Recadre(w, wop, isfirst, uf, ul);
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
                    w = intersection.Point(i).W();
                    if (uf <= w && ul >= w && Math.Abs(w - wop) < distbis
                            && (Math.Abs(w - ul) <= 0.01 || Math.Abs(w - uf) <= 0.01))
                    {
                        isolbis = i;
                        wbis = Recadre(w, wop, isfirst, uf, ul);
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
                CurveSurfaceIntersectionPoint pint = intersection.Point(isol);
                p2dbout = p2dbout.WithCoord(pint.U(), pint.V());
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
                    CurveSurfaceIntersectionPoint pint = intersection.Point(isolbis);
                    p2dbout = p2dbout.WithCoord(pint.U(), pint.V());
                    w = wbis;
                }
                else
                {
                    CurveSurfaceIntersectionPoint pint = intersection.Point(isol);
                    p2dbout = p2dbout.WithCoord(pint.U(), pint.V());
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

    internal static bool IntersUpdateOnSame(SurfaceAdaptor? hGs, FaceSurfaceAdaptor? hBs, Curve3D? c3dFI, in Face fop, in Face fprol, in Edge eprol, in Vertex vtx, bool isFirst, double tol, ref FaceInterference fIop, ref CommonPoint cPop, ref Point2D fprolUV, ref double c3dU)
    {
        // add more or less restrictive criterions to
        // decide if the intersection is done with the face at
        // extended end or if the end is sharp.
        double uf = fIop.FirstParameter();
        double ul = fIop.LastParameter();
        CurveAdaptor? hc3df = null;
        if (c3dFI.IsPeriodic)
        {
            hc3df = new CurveAdaptor(c3dFI);
        }
        else
        {
            hc3df = new CurveAdaptor(c3dFI, uf, ul);
        }

        if (Update(hBs, hc3df, ref fIop, ref cPop, ref fprolUV, isFirst, ref c3dU))
        {
            return true;
        }

        if (!global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(eprol, fprol, fop))
        {
            return false;
        }

        Curve2D? gpcprol = Tool.CurveOnSurface(eprol, fprol, out uf, out ul);
        if (gpcprol is null)
        {
            throw new ConstructionErrorException("Failed to get p-curve of edge");
        }
        CurveAdaptor2D? pcprol = new CurveAdaptor2D(gpcprol);
        double partemp = Tool.Parameter(vtx, eprol);

        return Update(hBs, pcprol, hGs, ref fIop, ref cPop, ref fprolUV, isFirst, ref partemp, ref c3dU, tol);
    }

    internal static bool Update(SurfaceAdaptorBase? face, Curve2DAdaptorBase? edonface, SurfaceAdaptorBase? surf, ref FaceInterference fi, ref CommonPoint cp, bool isfirst)
    {
        if (!cp.IsOnArc)
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
        f = Max(f - delta, pc.FirstParameter);
        l = Min(l + delta, pc.LastParameter);
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
                Curve2D? pConF = fi.PCurveOnFace();
                if (pConF is not null)
                {
                    TrimmedCurve2D? trCurve = (pConF as TrimmedCurve2D);
                    if (trCurve is not null)
                    {
                        pConF = trCurve.BasisCurve;
                    }
                    if (!pConF.IsPeriodic)
                    {
                        if (isfirst)
                        {
                            double fpar = pConF.FirstParameter;
                            if (parltg < fpar)
                            {
                                parltg = fpar;
                            }
                        }
                        else
                        {
                            double lpar = pConF.LastParameter;
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

    internal static void ChFi3d_ExtendSurface(ref Surface? s, ref int prol)
    {
        if (prol != 0)
        {
            return;
        }

        prol = ((s is global::Kerf.Geometry.BSplineSurface) ? 1 : (s is global::Kerf.Geometry.BezierSurface) ? 2 : 0);
        if (prol == 0)
        {
            return;
        }

        double length = 0.0, umin = 0.0, umax = 0.0, vmin = 0.0, vmax = 0.0;
        Point3D p1 = new(), p2 = new();
        s.ParameterBounds(out umin, out umax, out vmin, out vmax);
        s.D0(umin, vmin, out p1);
        s.D0(umax, vmax, out p2);
        length = p1.DistanceTo(p2);

        BoundedSurface? bs = (s as BoundedSurface);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref bs, length, 1, false, true);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref bs, length, 1, true, true);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref bs, length, 1, false, false);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref bs, length, 1, true, false);
        s = bs;
    }

    internal static void ComputeCurve2d(Curve3D? ct, ref Face face, ref Curve2D? c2d)
    {
        Edge e1 = new();
        IndexedSet<Shape> mapE1 = new(ShapeHasher.Instance);
        LibraryMakeEdge bedge = new(ct);
        Edge edg = bedge.Edge();
        NormalProjection ortProj = new();
        ortProj.Init(face);
        ortProj.Add(edg);
        ortProj.SetParams(1.0e-6, 1.0e-6, C1, 14, 16);
        ortProj.SetLimit(false);
        ortProj.Compute3d(false);
        ortProj.Build();
        double up1 = 0.0, up2 = 0.0;
        if (ortProj.IsDone)
        {
            Exploration.MapShapes(ortProj.Projection(), ShapeType.Edge, mapE1);
            if (mapE1.Count != 0)
            {
                Shape localShape = mapE1[0];
                e1 = ShapeCasts.Edge(localShape);
                //      E1=TopoDS::Edge( TopoDS_Shape (MapE1[1]));
                c2d = Tool.CurveOnSurface(e1, face, out up1, out up2);
            }
        }
    }

    internal static void ChFi3d_Recale(FaceSurfaceAdaptor bs, ref Point2D p1, ref Point2D p2, bool refon1)
    {
        Surface? surf = bs.GeomSurfaceOriginal();
        TrimmedSurface? ts = 
            (surf as TrimmedSurface);
        if (ts is not null)
        {
            surf = ts.BasisSurface;
        }
        if (surf.IsUPeriodic)
        {
            double u1 = p1.X, u2 = p2.X;
            double uper = surf.UPeriod;
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
            p1 = p1.WithX(u1);
            p2 = p2.WithX(u2);
        }
        if (surf.IsVPeriodic)
        {
            double v1 = p1.Y, v2 = p2.Y;
            double vper = surf.VPeriod;
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
            p1 = p1.WithY(v1);
            p2 = p2.WithY(v2);
        }
    }

    internal static bool ChFi3d_SelectStripe(NCollection_List<Stripe>.Iterator it, in Vertex vtx, bool prepareOnSame)
    {
        if (!prepareOnSame)
        {
            return true;
        }

        for (; it.More(); it.Next())
        {
            int sens = 0;
            Stripe? stripe = it.Value();
            ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
            State stat = default;
            if (sens == 1)
            {
                stat = stripe.Spine().FirstStatus();
            }
            else
            {
                stat = stripe.Spine().LastStatus();
            }
            if (stat == State.OnSame)
            {
                return true;
            }
        }

        return false;
    }

    internal static void ChercheFace(NCollection_List<Shape> map, in Edge e, in Face f1, in Face f2, in Face f3, ref Face f)
    {
        Face fcur = new();
        bool trouve = false;
        NCollection_List<Shape>.Iterator it = new();
        int ie = 0;
        for (it.Initialize(map); it.More() && !trouve; it.Next())
        {
            fcur = ShapeCasts.Face(it.Value());
            if (!fcur.IsSame(f1) && !fcur.IsSame(f2) && !fcur.IsSame(f3))
            {
                IndexedSet<Shape> mapE = new(ShapeHasher.Instance);
                Exploration.MapShapes(fcur, ShapeType.Edge, mapE);
                for (ie = 1; ie <= mapE.Count && !trouve; ie++)
                {
                    Shape localShape = mapE[ie - 1];
                    if (e.IsSame(ShapeCasts.Edge(localShape)))
                    //            if (E.IsSame(TopoDS::Edge(TopoDS_Shape (MapE[ie]))))
                    {
                        f = fcur;
                        trouve = true;
                    }
                }
            }
        }
        if (f.IsNull)
        {
            throw new ConstructionErrorException("Failed to find face.");
        }
    }

    internal static void cherche_edge1(in Face f1, in Face f2, ref Edge edge)
    {
        int i = 0, j = 0;
        Edge ecur1 = new(), ecur2 = new();
        bool trouve = false;
        IndexedSet<Shape> mapE1 = new(ShapeHasher.Instance), mapE2 = new(ShapeHasher.Instance);
        Exploration.MapShapes(f1, global::Kerf.Topology.ShapeType.Edge, mapE1);
        Exploration.MapShapes(f2, global::Kerf.Topology.ShapeType.Edge, mapE2);
        for (i = 1; i <= mapE1.Count && !trouve; i++)
        {
            Shape localShape = mapE1[i - 1];
            ecur1 = ShapeCasts.Edge(localShape);
            //	Ecur1=TopoDS::Edge(TopoDS_Shape (MapE1[i]));
            for (j = 1; j <= mapE2.Count && !trouve; j++)
            {
                localShape = mapE2[j - 1];
                ecur2 = ShapeCasts.Edge(localShape);
                //	      Ecur2=TopoDS::Edge(TopoDS_Shape (MapE2[j]));
                if (ecur2.IsSame(ecur1))
                {
                    edge = ecur1;
                    trouve = true;
                }
            }
        }
        if (edge.IsNull)
        {
            throw new ConstructionErrorException("Failed to find edge.");
        }
    }

    internal static bool ContainV(in Face f1, in Vertex v)
    {
        int i = 0;
        Vertex vcur = new();
        bool trouve = false;
        bool contain = false;
        IndexedSet<Shape> mapV = new(ShapeHasher.Instance);
        Exploration.MapShapes(f1, ShapeType.Vertex, mapV);
        for (i = 1; i <= mapV.Count && !trouve; i++)
        {
            Shape localShape = mapV[i - 1];
            vcur = ShapeCasts.Vertex(localShape);
            //	Vcur=TopoDS::Vertex(TopoDS_Shape (MapV[i]));
            if (vcur.IsSame(v))
            {
                contain = true;
                trouve = true;
            }
        }
        return contain;
    }

    internal static bool ContainE(in Face f1, in Edge e)
    {
        int i = 0;
        Edge ecur = new();
        bool trouve = false;
        bool contain = false;
        IndexedSet<Shape> mapE = new(ShapeHasher.Instance);
        Exploration.MapShapes(f1, ShapeType.Edge, mapE);
        for (i = 1; i <= mapE.Count && !trouve; i++)
        {
            Shape localShape = mapE[i - 1];
            ecur = ShapeCasts.Edge(localShape);
            //	Ecur=TopoDS::Edge(TopoDS_Shape (MapE[i]));
            if (ecur.IsSame(e))
            {
                contain = true;
                trouve = true;
            }
        }
        return contain;
    }

    internal static bool IsShrink(CurveAdaptor2D pc, double pf, double pl, double param, bool isU, double tol)
    {
        switch (pc.GetType())
        {
            case Line: {
                Point2D p1 = pc.Value(pf);
                Point2D p2 = pc.Value(pl);
                return Math.Abs(p1.Coord(isU ? 1 : 2) - param) <= tol
                              && Math.Abs(p2.Coord(isU ? 1 : 2) - param) <= tol;
            }
            case BezierCurve:
            case BSplineCurve: {
                FunctionSample sample = new(pf, pl, 10);
                int i = 0;
                for (i = 1; i <= sample.PointCount; i++)
                {
                    Point2D p = pc.Value(sample.GetParameter(i));
                    if (Math.Abs(p.Coord(isU ? 1 : 2) - param) > tol)
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
