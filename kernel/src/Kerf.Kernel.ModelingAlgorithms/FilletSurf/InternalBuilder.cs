// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_InternalBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_InternalBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.BlendFunctions.SectionShape;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Fillets.FilletShape;
using static Kerf.Engine.Fillets.Surfaces.SurfaceStatusType;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Surfaces;

/// <summary>
/// The fillet builder of <see cref="SurfaceBuilder"/>: one contour of constant radius, whose surfaces, curves and
/// sections are computed (the stripe of <c>PerformSetOfKGen</c>) without the reconstruction of a shape.
/// </summary>
/// <remarks>
/// It never runs <see cref="FilletBuilderBase.Compute"/>, so the reconstruction (decision 992) is created and never used,
/// as OCCT's HBuilder (plan 1.3). It works on the caller's shape: <see cref="Perform"/> and <see cref="Simulate"/> write
/// the spine's 3D curves in the builder's working overlay (unit 10H5), and the support faces handed back are the
/// caller's.
/// </remarks>
internal sealed class InternalBuilder : FilBuilder
{
    private static bool Isinlist(in Shape e, NCollection_List<Shape> l)
    {
        NCollection_List<Shape>.Iterator it = new();
        for (it.Initialize(l); it.More(); it.Next())
        {
            if (e.IsSame(it.Value()))
            {
                return true;
            }
        }
        return false;
    }

    private static bool IntPlanEdge(EdgeCurveAdaptor ed, in Plane3D p, ref double w, double tol3d)
    {
        bool done = false;
        double f = ed.FirstParameter();
        double l = ed.LastParameter();
        Point3D or = p.Location;
        PlaneSurface pln = new(p);
        SurfaceAdaptor plan = new(pln);

        HInter intersection = new();
        int nbp, iip;
        CurveSurfaceIntersectionPoint ip;
        double dist = RealLast();

        intersection.Perform(ed, plan);

        if (intersection.IsDone)
        {
            nbp = intersection.PointCount;
            for (iip = 1; iip <= nbp; iip++)
            {
                ip = intersection.Point(iip);
                Point3D pint = ip.Pnt();
                double d = pint.DistanceTo(or);
                if (d < dist)
                {
                    done = true;
                    w = ip.W();
                    dist = d;
                }
            }
        }
        Point3D pdeb = ed.Value(f);
        Point3D pfin = ed.Value(l);
        double u, v;
        // check if the extremities are not solution
        global::Kerf.Engine.Geometry.ElSLib.Parameters(p, pdeb, out u, out v);
        Point3D projdeb = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, p);
        double dprojdeb = pdeb.DistanceTo(projdeb);
        if (dprojdeb < tol3d)
        {
            double d = pdeb.DistanceTo(or);
            if (d < dist)
            {
                done = true;
                w = f;
                dist = d;
            }
        }
        global::Kerf.Engine.Geometry.ElSLib.Parameters(p, pfin, out u, out v);
        Point3D projfin = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, p);
        double dprojfin = pfin.DistanceTo(projfin);
        if (dprojfin < tol3d)
        {
            double d = pfin.DistanceTo(or);
            if (d < dist)
            {
                done = true;
                w = l;
                dist = d;
            }
        }
        return done;
    }

    private static bool ComputeEdgeParameter(Spine spine, int ind, double pelsp, ref double ped, double tol3d)
    {
        ElSpine guide = spine.ElSpine(ind);
        guide.D1(pelsp, out Point3D p, out Vector3D v);
        Plane3D pln = new(p, new Direction3D(v));
        EdgeCurveAdaptor ed = (EdgeCurveAdaptor)spine.CurrentElementarySpine(ind).ShallowCopy(); // OCCT: a new adaptor copying it
        return IntPlanEdge(ed, pln, ref ped, tol3d);
    }

    //=================================================================================================

    internal InternalBuilder(in Shape s, global::Kerf.Fillets.FilletShape fShape = global::Kerf.Fillets.FilletShape.Polynomial,
                                        double ta = 1.0e-2, double tapp3d = 1.0e-4, double tapp2d = 1.0e-5)
        : base(s, fShape, ta)
    {
        SetParams(ta, tapp3d, tapp2d, tapp3d, tapp2d, 1.0e-3);
        SetContinuity(C2, ta);
    }

    //=======================================================================
    // function : Add
    // purpose  : creation of spine on a set of edges
    //
    //  0 : no problem
    //  1 : empty list
    //  2 : non g1 edges
    //  3 : non G1 adjacent faces
    //  4 : edge is not on the shape
    //  5 : edge is not alive
    //=======================================================================

    internal int Add(NCollection_List<Shape> e, double r)
    {
        if (e.IsEmpty())
        {
            return 1;
        }
        NCollection_List<Shape> E = new();
        foreach (Shape anE in e)
        {
            E.Append(anE);
        }
        NCollection_List<Shape>.Iterator it = new();
        // OCCT: the inner loop re-initializes It, the outer loop's iterator, on the edge's faces; the outer loop then
        // goes on over that list (or ends), so only the first edge is checked.
        for (it.Initialize(E); it.More(); it.Next())
        {
            Edge cured = ShapeCasts.Edge(it.Value());
            if (cured.IsNull)
            {
                return 4;
            }
            if (!_efMap.Contains(cured))
            {
                return 4;
            }
            // check if the edge is a fracture edge
            Face ff1 = new(), ff2 = new();
            for (it.Initialize(_efMap[cured]); it.More(); it.Next())
            {
                if (ff1.IsNull)
                {
                    ff1 = ShapeCasts.Face(it.Value());
                }
                else
                {
                    ff2 = ShapeCasts.Face(it.Value());
                    if (!ff2.IsSame(ff1))
                    {
                        break;
                    }
                }
            }
            if (ff1.IsNull || ff2.IsNull)
            {
                return 5;
            }
            if (ff1.IsSame(ff2))
            {
                return 5;
            }
            if (Tool.Continuity(cured, ff1, ff2) != C0)
            {
                return 5;
            }
        }
        Edge ed = ShapeCasts.Edge(E.First());
        ed.Orient(Forward);
        base.Add(r, ed);
        Stripe st = _listStripe.First();
        ref Spine sp = ref st.ChangeSpine();
        bool periodic = sp.IsPeriodic;

        // It is checked if edges of list E are in the contour,
        // the edges that arenot in the list are removed from the contour,
        // it is checked that the remainder is monoblock.

        for (it.Initialize(E); it.More(); it.Next())
        {
            Edge cured = ShapeCasts.Edge(it.Value());
            if (Contains(cured) == 0)
            {
                return 2;
            }
        }

        FilSpine newsp = new();
        bool debut = false;
        int premierquinyestpas = 0;
        int yatrou = 0;
        for (int i = 1; i <= sp.EdgeCount; i++)
        {
            Edge cured = sp.Edges(i);
            if (Isinlist(cured, E))
            {
                debut = true;
                if (premierquinyestpas != 0)
                {
                    yatrou = 1;
                    break;
                }
                newsp.SetEdges(cured);
            }
            else if (debut && premierquinyestpas == 0)
            {
                premierquinyestpas = i;
            }
        }
        if (!periodic && yatrou != 0)
        {
            return 2;
        }
        if (periodic && yatrou != 0)
        {
            bool vraitrou = false, localDebut = false;
            for (int i = sp.EdgeCount; i > yatrou; i--)
            {
                Edge cured = sp.Edges(i);
                if (Isinlist(cured, E))
                {
                    if (vraitrou)
                    {
                        return 2;
                    }
                    newsp.PutInFirst(cured);
                }
                else if (localDebut)
                {
                    vraitrou = true;
                }
                localDebut = true;
            }
        }

        if (newsp.EdgeCount != sp.EdgeCount)
        {
            newsp.Load();
            newsp.SetRadius(r);
            sp = newsp;
        }

        // ElSpine is immediately constructed
        ElSpine hels = new();
        sp.D1(sp.FirstParameter(), out Point3D pFirst, out Vector3D tFirst);
        sp.D1(sp.LastParameter(), out Point3D pLast, out Vector3D tLast);
        hels.FirstParameter(sp.FirstParameter());
        hels.SetFirstPointAndTgt(pFirst, tFirst);
        hels.LastParameter(sp.LastParameter());
        hels.SetLastPointAndTgt(pLast, tLast);
        ChFi3d_PerformElSpine(hels, sp, _conti, _tolesp);
        sp.AppendElSpine(hels);
        sp.SplitDone(true);
        return 0;
    }

    //=================================================================================================

    internal void Perform()
    {
        using WorkingOverlay.Scope scope = EnterOverlay(); // Kerf: the elementary spines' 3D curves (unit 10H5)
        // PerformSetOfSurfOnElSpine is enough.

        Stripe stripe = _listStripe.First();
        ref NCollection_HSequence<SurfData> hData = ref stripe.ChangeSetOfSurfData();
        hData = new NCollection_HSequence<SurfData>();
        Spine spine = stripe.ChangeSpine();
        Orientation refOr1 = Forward, refOr2 = Forward;
        int refChoix = 0;
        StripeOrientations(spine, ref refOr1, ref refOr2, ref refChoix);
        stripe.OrientationOnFace1(refOr1);
        stripe.OrientationOnFace2(refOr2);
        stripe.Choix(refChoix);
        PerformSetOfKGen(stripe, false);
    }

    //=================================================================================================

    internal override bool PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine guide,
                                       Spine spine, int choix, FaceSurfaceAdaptor s1, TopolTool i1,
                                       FaceSurfaceAdaptor s2, TopolTool i2, double maxStep, double fleche,
                                       double tolGuide, ref double first, ref double last, bool inside, bool appro,
                                       bool forward, bool recOnS1, bool recOnS2, math_Vector soldep, ref int intf,
                                       ref int intl)
    {
        SurfData data = seqData[1];
        FilSpine fsp = (spine as FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        Line lin = null;
        Orientation or = s1.Face().Orientation;
        if (!fsp.IsConstant())
        {
            throw new ConstructionErrorException("PerformSurf : no variable radiuses");
        }
        // bool maybesingular; //pour scinder les Surfdata singulieres

        ElSpine emptyGuide = null;

        ConstRad func = new(s1, s2, guide);
        ConstRadInv fInv = new(s1, s2, guide);
        func.Set(fsp.Radius(), choix);
        fInv.Set(fsp.Radius(), choix);
        switch (GetFilletShape())
        {
            case global::Kerf.Fillets.FilletShape.Rational:
                func.Set(SectionShape.Rational);
                break;
            case global::Kerf.Fillets.FilletShape.QuasiAngular:
                func.Set(SectionShape.QuasiAngular);
                break;
            case global::Kerf.Fillets.FilletShape.Polynomial:
                func.Set(SectionShape.Polynomial);
                break;
        }
        double pFirst = first;
        _done = SimulData(data, guide, emptyGuide, ref lin, s1, i1, s2, i2, func, fInv, pFirst, maxStep, fleche, tolGuide,
                         ref first, ref last, inside, appro, forward, soldep, 20, recOnS1, recOnS2);
        if (!_done)
        {
            return false;
        }
        if (lin.StartPointOnFirst().PointOnRstCount != 0)
        {
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref data.ChangeVertexFirstOnS1(),
                                  _tolapp3d);
        }
        if (lin.EndPointOnFirst().PointOnRstCount != 0)
        {
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref data.ChangeVertexLastOnS1(),
                                  _tolapp3d);
        }
        if (lin.StartPointOnSecond().PointOnRstCount != 0)
        {
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref data.ChangeVertexFirstOnS2(),
                                  _tolapp3d);
        }
        if (lin.EndPointOnSecond().PointOnRstCount != 0)
        {
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref data.ChangeVertexLastOnS2(),
                                  _tolapp3d);
        }
        _done = CompleteData(data, func, lin, s1, s2, or, false, false, false, false);
        if (!_done)
        {
            throw new FailureException("PerformSurf : Failed approximation!");
        }
        //  maybesingular = (Func.GetMinimalDistance()<=100*tolapp3d);
        bool ok = false;
        if (!forward)
        {
            intf = 0;
            CommonPoint cpf1 = data.VertexFirstOnS1();
            if (cpf1.IsOnArc)
            {
                Face f1 = s1.Face();
                Face bid = new();
                intf = !SearchFace(spine, cpf1, f1, ref bid) ? 1 : 0;
                ok = intf != 0;
            }
            CommonPoint cpf2 = data.VertexFirstOnS2();
            if (cpf2.IsOnArc && !ok)
            {
                Face f2 = s2.Face();
                Face bid = new();
                intf = !SearchFace(spine, cpf2, f2, ref bid) ? 1 : 0;
            }
        }
        intl = 0;
        ok = false;
        CommonPoint cpl1 = data.VertexLastOnS1();
        if (cpl1.IsOnArc)
        {
            Face f1 = s1.Face();
            Face bid = new();
            intl = !SearchFace(spine, cpl1, f1, ref bid) ? 1 : 0;
            ok = intl != 0;
        }
        CommonPoint cpl2 = data.VertexLastOnS2();
        if (cpl2.IsOnArc && !ok)
        {
            Face f2 = s2.Face();
            Face bid = new();
            intl = !SearchFace(spine, cpl2, f2, ref bid) ? 1 : 0;
        }
        data.FirstSpineParam(first);
        data.LastSpineParam(last);

        //  if (maybesingular) SplitSurf(SeqData, lin);
        //  Necessite de trimer resultats : A faire
        return true;
    }

    internal override void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine hGuide,
                                       Spine spine, int choix, FaceSurfaceAdaptor hs1, TopolTool i1,
                                       EdgeCurve2DAdaptor pc1, FaceSurfaceAdaptor hSref1, EdgeCurve2DAdaptor pCref1,
                                       ref bool decroch1, FaceSurfaceAdaptor hs2, TopolTool i2,
                                       Orientation or2, double maxStep, double fleche, double tolGuide,
                                       ref double first, ref double last, bool inside, bool appro, bool forward,
                                       bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        throw new DomainErrorException("BlendFunc_CSConstRad::Section : Not implemented");
    }

    internal override void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine hGuide,
                                       Spine spine, int choix, FaceSurfaceAdaptor hs1, TopolTool i1,
                                       Orientation or1, FaceSurfaceAdaptor hs2, TopolTool i2,
                                       EdgeCurve2DAdaptor pc2, FaceSurfaceAdaptor hSref2, EdgeCurve2DAdaptor pCref2,
                                       ref bool decroch2, double maxStep, double fleche, double tolGuide,
                                       ref double first, ref double last, bool inside, bool appro, bool forward,
                                       bool recP, bool recS, bool recRst, math_Vector soldep)
    {
        throw new DomainErrorException("BlendFunc_CSConstRad::Section : Not implemented");
    }

    internal override void PerformSurf(NCollection_Sequence<SurfData> seqData, ElSpine hGuide,
                                       Spine spine, int choix, FaceSurfaceAdaptor hs1, TopolTool i1,
                                       EdgeCurve2DAdaptor pc1, FaceSurfaceAdaptor hSref1, EdgeCurve2DAdaptor pCref1,
                                       ref bool decroch1, Orientation or1, FaceSurfaceAdaptor hs2,
                                       TopolTool i2, EdgeCurve2DAdaptor pc2, FaceSurfaceAdaptor hSref2,
                                       EdgeCurve2DAdaptor pCref2, ref bool decroch2, Orientation or2,
                                       double maxStep, double fleche, double tolGuide, ref double first,
                                       ref double last, bool inside, bool appro, bool forward, bool recP1,
                                       bool recRst1, bool recP2, bool recRst2, math_Vector soldep)
    {
        throw new DomainErrorException("BlendFunc_CSConstRad::Section : Not implemented");
    }

    internal bool Done()
    {
        return _done;
    }

    //=======================================================================
    // function : NbSurface
    // purpose  :  gives the number of NUBS surfaces  of the Fillet
    //=======================================================================

    internal int SurfaceCount
    {
        get
        {
            return _listStripe.First().SetOfSurfData().Length();
        }
    }

    //=======================================================================
    // function : SurfaceFillet
    // purpose  : gives the NUBS surface of index Index
    //=======================================================================

    internal Surface SurfaceFillet(int index)
    {
        int isurf = _listStripe.First().SetOfSurfData().Value(index).Surf();

        return _ds.Surface(isurf).Surface();
    }

    //=======================================================================
    // function : TolApp3d
    // purpose  :  gives the 3d tolerance reached during approximation
    //=======================================================================

    internal double TolApp3d(int index)
    {
        int isurf = _listStripe.First().SetOfSurfData().Value(index).Surf();

        return _ds.Surface(isurf).Tolerance();
    }

    //=======================================================================
    // function : SupportFace1
    // purpose  : gives the first support  face relative to SurfaceFillet(Index)
    //=======================================================================

    internal Face SupportFace1(int index)
    {
        int isurf = _listStripe.First().SetOfSurfData().Value(index).IndexOfS1();

        return ShapeCasts.Face(_ds.Shape(isurf));
    }

    //=======================================================================
    // function : SupportFace2
    // purpose  : gives the second support face relative to SurfaceFillet(Index)
    //=======================================================================

    internal Face SupportFace2(int index)
    {
        int isurf = _listStripe.First().SetOfSurfData().Value(index).IndexOfS2();

        return ShapeCasts.Face(_ds.Shape(isurf));
    }

    //===============================================================================
    // function : CurveOnSup1
    // purpose  :  gives  the 3d curve  of SurfaceFillet(Index)  on SupportFace1(Index)
    //===============================================================================

    internal Curve3D CurveOnFace1(int index)
    {
        int icurv = _listStripe.First().SetOfSurfData().Value(index).InterferenceOnS1().LineIndex();
        return _ds.Curve(icurv).Curve();
    }

    //=======================================================================
    // function : CurveOnSup2
    // purpose  : gives the 3d  curve of  SurfaceFillet(Index) on SupportFace2(Index
    //=======================================================================

    internal Curve3D CurveOnFace2(int index)
    {
        int icurv = _listStripe.First().SetOfSurfData().Value(index).InterferenceOnS2().LineIndex();
        return _ds.Curve(icurv).Curve();
    }

    //=======================================================================
    // function : PCurveOnFace1
    // purpose  : gives the  PCurve associated to CurvOnSup1(Index)  on the support face
    //=======================================================================

    internal Curve2D PCurveOnFace1(int index)
    {
        return _listStripe.First().SetOfSurfData().Value(index).InterferenceOnS1().PCurveOnFace();
    }

    //=======================================================================
    // function : PCurveOnFillet1
    // purpose  : gives the PCurve associated to CurveOnFace1(Index) on the Fillet
    //=======================================================================

    internal Curve2D PCurve1OnFillet(int index)
    {
        return _listStripe.First().SetOfSurfData().Value(index).InterferenceOnS1().PCurveOnSurf();
    }

    //=======================================================================
    // function : PCurveOnFace2
    // purpose  : gives the  PCurve associated to CurvOnSup2(Index)  on the support face
    //=======================================================================

    internal Curve2D PCurveOnFace2(int index)
    {
        return _listStripe.First().SetOfSurfData().Value(index).InterferenceOnS2().PCurveOnFace();
    }

    //=======================================================================
    // function : PCurveOnFillet2
    // purpose  : gives the PCurve associated to CurveOnFace2(Index) on the Fillet
    //=======================================================================

    internal Curve2D PCurve2OnFillet(int index)
    {
        return _listStripe.First().SetOfSurfData().Value(index).InterferenceOnS2().PCurveOnSurf();
    }

    //=======================================================================
    // function : FirstParameter
    // purpose  : gives the parameter of the fillet  on the first edge
    //=======================================================================

    internal double FirstParameter()
    {
        Stripe st = _listStripe.First();
        Spine sp = st.Spine();
        SurfData sd = st.SetOfSurfData().Value(1);
        double p = sd.FirstSpineParam();
        int ind = 1;
        if (sp.IsPeriodic)
        {
            ind = sp.Index(p);
        }
        double ep = 0.0;
        if (ComputeEdgeParameter(sp, ind, p, ref ep, _tolapp3d))
        {
            return ep;
        }
        return 0.0;
    }

    //=======================================================================
    // function : LastParameter
    // purpose  :  gives the parameter of the fillet  on the last edge
    //=======================================================================

    internal double LastParameter()
    {
        Stripe st = _listStripe.First();
        Spine sp = st.Spine();
        SurfData sd = st.SetOfSurfData().Value(SurfaceCount);
        double p = sd.LastSpineParam();
        int ind = sp.EdgeCount;
        if (sp.IsPeriodic)
        {
            ind = sp.Index(p);
        }
        double ep = 0.0;
        if (ComputeEdgeParameter(sp, ind, p, ref ep, _tolapp3d))
        {
            return ep;
        }
        return 0.0;
    }

    //=======================================================================
    // function : StatusStartSection
    // purpose  :  returns:
    //       twoExtremityonEdge: each extremity of  start section of the Fillet is
    //                        on the edge of  the corresponding support face.
    //       OneExtremityOnEdge:  only one  of  the extremities of  start  section  of the  Fillet
    //                           is on the  edge of the corresponding support face.
    //       NoExtremityOnEdge:  any extremity of  start  section  of the fillet is  on
    //                           the edge  of   the  corresponding support face.
    //=======================================================================

    internal SurfaceStatusType StartSectionStatus()
    {
        bool isonedge1 = _listStripe.First().SetOfSurfData().Value(1).VertexFirstOnS1().IsOnArc;
        bool isonedge2 = _listStripe.First().SetOfSurfData().Value(1).VertexFirstOnS2().IsOnArc;

        if (isonedge1 && isonedge2)
        {
            return SurfaceStatusType.TwoExtremityOnEdge;
        }
        else if ((!isonedge1) && (!isonedge2))
        {
            return SurfaceStatusType.NoExtremityOnEdge;
        }
        else
        {
            return SurfaceStatusType.OneExtremityOnEdge;
        }
    }

    //=======================================================================
    // function : StatusEndSection()
    //=======================================================================

    internal SurfaceStatusType EndSectionStatus()
    {
        bool isonedge1 = _listStripe.First().SetOfSurfData().Value(SurfaceCount).VertexLastOnS1().IsOnArc;
        bool isonedge2 = _listStripe.First().SetOfSurfData().Value(SurfaceCount).VertexLastOnS2().IsOnArc;

        if (isonedge1 && isonedge2)
        {
            return SurfaceStatusType.TwoExtremityOnEdge;
        }
        else if ((!isonedge1) && (!isonedge2))
        {
            return SurfaceStatusType.NoExtremityOnEdge;
        }
        else
        {
            return SurfaceStatusType.OneExtremityOnEdge;
        }
    }

    //=======================================================================
    // function : Simulate
    // purpose  : computes only the sections used in the computation of the fillet
    //=======================================================================

    internal void Simulate()
    {
        using WorkingOverlay.Scope scope = EnterOverlay(); // Kerf: the elementary spines' 3D curves (unit 10H5)
        // ChFi3d_FilBuilder::Simulate(1);
        Stripe stripe = _listStripe.First();
        ref NCollection_HSequence<SurfData> hData = ref stripe.ChangeSetOfSurfData();
        hData = new NCollection_HSequence<SurfData>();
        Spine spine = stripe.ChangeSpine();
        Orientation refOr1 = Forward, refOr2 = Forward;
        int refChoix = 0;
        StripeOrientations(spine, ref refOr1, ref refOr2, ref refChoix);
        stripe.OrientationOnFace1(refOr1);
        stripe.OrientationOnFace2(refOr2);
        stripe.Choix(refChoix);
        PerformSetOfKGen(stripe, true);
    }

    //=======================================================================
    // function : NbSection
    // purpose  :  gives the number of sections relative to SurfaceFillet(IndexSurf)
    //=======================================================================

    internal int NbSection(int indexSurf)
    {
        return Sect(1, indexSurf).Length();
    }

    //=======================================================================
    // function : Section
    // purpose  : gives the   arc of circle corresponding    to section number
    //           IndexSec  of  SurfaceFillet(IndexSurf)  (The   basis curve  of the
    //           trimmed curve is a Geom_Circle)
    //=======================================================================

    internal TrimmedCurve3D Section(int indexSurf, int indexSec)
    {
        Sect(1, indexSurf).Value(indexSec).Get(out Circle3D c, out double deb, out double fin);
        CircleCurve3D gc = new(c);
        return new TrimmedCurve3D(gc, deb, fin);
    }
}
