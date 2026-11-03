// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_InternalBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_InternalBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BlendFunc_SectionShape;
using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_FilletShape;
using static Occt.FilletSurf_StatusType;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// The fillet builder of <see cref="FilletSurf_Builder"/>: one contour of constant radius, whose surfaces, curves and
/// sections are computed (the stripe of <c>PerformSetOfKGen</c>) without the reconstruction of a shape.
/// </summary>
/// <remarks>
/// It never runs <see cref="ChFi3d_Builder.Compute"/>, so the reconstruction (decision 992) is created and never used,
/// as OCCT's HBuilder (plan 1.3). The caller's edges are mapped onto the builder's working copy, and the support faces
/// handed back are the caller's (decision 1119).
/// </remarks>
internal sealed class FilletSurf_InternalBuilder : ChFi3d_FilBuilder
{
    private static bool isinlist(in Shape E, NCollection_List<Shape> L)
    {
        NCollection_List<Shape>.Iterator It = new();
        for (It.Initialize(L); It.More(); It.Next())
        {
            if (E.IsSame(It.Value()))
            {
                return true;
            }
        }
        return false;
    }

    private static bool IntPlanEdge(EdgeCurveAdaptor Ed, in Plane3D P, ref double w, double tol3d)
    {
        bool done = false;
        double f = Ed.FirstParameter();
        double l = Ed.LastParameter();
        Point3D Or = P.Location();
        PlaneSurface Pln = new(P);
        SurfaceAdaptor Plan = new(Pln);

        IntCurveSurface_HInter Intersection = new();
        int nbp, iip;
        IntCurveSurface_IntersectionPoint IP;
        double dist = RealLast();

        Intersection.Perform(Ed, Plan);

        if (Intersection.IsDone())
        {
            nbp = Intersection.NbPoints();
            for (iip = 1; iip <= nbp; iip++)
            {
                IP = Intersection.Point(iip);
                Point3D pint = IP.Pnt();
                double d = pint.Distance(Or);
                if (d < dist)
                {
                    done = true;
                    w = IP.W();
                    dist = d;
                }
            }
        }
        Point3D pdeb = Ed.Value(f);
        Point3D pfin = Ed.Value(l);
        double u, v;
        // check if the extremities are not solution
        global::Kerf.Engine.Geometry.ElSLib.Parameters(P, pdeb, out u, out v);
        Point3D projdeb = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, P);
        double dprojdeb = pdeb.Distance(projdeb);
        if (dprojdeb < tol3d)
        {
            double d = pdeb.Distance(Or);
            if (d < dist)
            {
                done = true;
                w = f;
                dist = d;
            }
        }
        global::Kerf.Engine.Geometry.ElSLib.Parameters(P, pfin, out u, out v);
        Point3D projfin = global::Kerf.Engine.Geometry.ElSLib.Value(u, v, P);
        double dprojfin = pfin.Distance(projfin);
        if (dprojfin < tol3d)
        {
            double d = pfin.Distance(Or);
            if (d < dist)
            {
                done = true;
                w = l;
                dist = d;
            }
        }
        return done;
    }

    private static bool ComputeEdgeParameter(ChFiDS_Spine Spine, int ind, double pelsp, ref double ped, double tol3d)
    {
        ChFiDS_ElSpine Guide = Spine.ElSpine(ind);
        Guide.D1(pelsp, out Point3D P, out Vector3D V);
        Plane3D pln = new(P, new Direction3D(V));
        EdgeCurveAdaptor ed = (EdgeCurveAdaptor)Spine.CurrentElementarySpine(ind).ShallowCopy(); // OCCT: a new adaptor copying it
        return IntPlanEdge(ed, pln, ref ped, tol3d);
    }

    //=================================================================================================

    internal FilletSurf_InternalBuilder(in Shape S, ChFi3d_FilletShape FShape = ChFi3d_Polynomial,
                                        double Ta = 1.0e-2, double Tapp3d = 1.0e-4, double Tapp2d = 1.0e-5)
        : base(S, FShape, Ta)
    {
        SetParams(Ta, Tapp3d, Tapp2d, Tapp3d, Tapp2d, 1.0e-3);
        SetContinuity(C2, Ta);
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

    internal int Add(NCollection_List<Shape> theE, double R)
    {
        if (theE.IsEmpty())
        {
            return 1;
        }
        // OCCT.NET: the caller's edges on the working copy (decision 1119)
        NCollection_List<Shape> E = new();
        foreach (Shape anE in theE)
        {
            E.Append(ToWork(anE));
        }
        NCollection_List<Shape>.Iterator It = new();
        // OCCT: the inner loop re-initializes It, the outer loop's iterator, on the edge's faces; the outer loop then
        // goes on over that list (or ends), so only the first edge is checked.
        for (It.Initialize(E); It.More(); It.Next())
        {
            Edge cured = ShapeCasts.Edge(It.Value());
            if (cured.IsNull)
            {
                return 4;
            }
            if (!myEFMap.Contains(cured))
            {
                return 4;
            }
            // check if the edge is a fracture edge
            Face ff1 = new(), ff2 = new();
            for (It.Initialize(myEFMap[cured]); It.More(); It.Next())
            {
                if (ff1.IsNull)
                {
                    ff1 = ShapeCasts.Face(It.Value());
                }
                else
                {
                    ff2 = ShapeCasts.Face(It.Value());
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
        ed.Orientation(Forward);
        base.Add(R, ed);
        ChFiDS_Stripe st = myListStripe.First();
        ref ChFiDS_Spine sp = ref st.ChangeSpine();
        bool periodic = sp.IsPeriodic();

        // It is checked if edges of list E are in the contour,
        // the edges that arenot in the list are removed from the contour,
        // it is checked that the remainder is monoblock.

        for (It.Initialize(E); It.More(); It.Next())
        {
            Edge cured = ShapeCasts.Edge(It.Value());
            if (Contains(cured) == 0)
            {
                return 2;
            }
        }

        ChFiDS_FilSpine newsp = new();
        bool debut = false;
        int premierquinyestpas = 0;
        int yatrou = 0;
        for (int i = 1; i <= sp.NbEdges(); i++)
        {
            Edge cured = sp.Edges(i);
            if (isinlist(cured, E))
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
            bool vraitrou = false, aLocalDebut = false;
            for (int i = sp.NbEdges(); i > yatrou; i--)
            {
                Edge cured = sp.Edges(i);
                if (isinlist(cured, E))
                {
                    if (vraitrou)
                    {
                        return 2;
                    }
                    newsp.PutInFirst(cured);
                }
                else if (aLocalDebut)
                {
                    vraitrou = true;
                }
                aLocalDebut = true;
            }
        }

        if (newsp.NbEdges() != sp.NbEdges())
        {
            newsp.Load();
            newsp.SetRadius(R);
            sp = newsp;
        }

        // ElSpine is immediately constructed
        ChFiDS_ElSpine hels = new();
        sp.D1(sp.FirstParameter(), out Point3D PFirst, out Vector3D TFirst);
        sp.D1(sp.LastParameter(), out Point3D PLast, out Vector3D TLast);
        hels.FirstParameter(sp.FirstParameter());
        hels.SetFirstPointAndTgt(PFirst, TFirst);
        hels.LastParameter(sp.LastParameter());
        hels.SetLastPointAndTgt(PLast, TLast);
        ChFi3d_PerformElSpine(hels, sp, myConti, tolesp);
        sp.AppendElSpine(hels);
        sp.SplitDone(true);
        return 0;
    }

    //=================================================================================================

    internal void Perform()
    {
        // PerformSetOfSurfOnElSpine is enough.

        ChFiDS_Stripe Stripe = myListStripe.First();
        ref NCollection_HSequence<ChFiDS_SurfData> HData = ref Stripe.ChangeSetOfSurfData();
        HData = new NCollection_HSequence<ChFiDS_SurfData>();
        ChFiDS_Spine Spine = Stripe.ChangeSpine();
        Orientation RefOr1 = Forward, RefOr2 = Forward;
        int RefChoix = 0;
        StripeOrientations(Spine, ref RefOr1, ref RefOr2, ref RefChoix);
        Stripe.OrientationOnFace1(RefOr1);
        Stripe.OrientationOnFace2(RefOr2);
        Stripe.Choix(RefChoix);
        PerformSetOfKGen(Stripe, false);
    }

    //=================================================================================================

    internal override bool PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine Guide,
                                       ChFiDS_Spine Spine, int Choix, FaceSurfaceAdaptor S1, TopolTool I1,
                                       FaceSurfaceAdaptor S2, TopolTool I2, double MaxStep, double Fleche,
                                       double TolGuide, ref double First, ref double Last, bool Inside, bool Appro,
                                       bool Forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int Intf,
                                       ref int Intl)
    {
        ChFiDS_SurfData Data = SeqData[1];
        ChFiDS_FilSpine fsp = (Spine as ChFiDS_FilSpine);
        if (fsp is null)
        {
            throw new ConstructionErrorException("PerformSurf : this is not the spine of a fillet");
        }
        BRepBlend_Line lin = null;
        Orientation Or = S1.Face().Orientation();
        if (!fsp.IsConstant())
        {
            throw new ConstructionErrorException("PerformSurf : no variable radiuses");
        }
        // bool maybesingular; //pour scinder les Surfdata singulieres

        ChFiDS_ElSpine EmptyGuide = null;

        BlendFunc_ConstRad Func = new(S1, S2, Guide);
        BlendFunc_ConstRadInv FInv = new(S1, S2, Guide);
        Func.Set(fsp.Radius(), Choix);
        FInv.Set(fsp.Radius(), Choix);
        switch (GetFilletShape())
        {
            case ChFi3d_Rational:
                Func.Set(BlendFunc_Rational);
                break;
            case ChFi3d_QuasiAngular:
                Func.Set(BlendFunc_QuasiAngular);
                break;
            case ChFi3d_Polynomial:
                Func.Set(BlendFunc_Polynomial);
                break;
        }
        double PFirst = First;
        done = SimulData(Data, Guide, EmptyGuide, ref lin, S1, I1, S2, I2, Func, FInv, PFirst, MaxStep, Fleche, TolGuide,
                         ref First, ref Last, Inside, Appro, Forward, Soldep, 20, RecOnS1, RecOnS2);
        if (!done)
        {
            return false;
        }
        if (lin.StartPointOnFirst().NbPointOnRst() != 0)
        {
            ChFi3d_FilCommonPoint(lin.StartPointOnFirst(), lin.TransitionOnS1(), true, ref Data.ChangeVertexFirstOnS1(),
                                  tolapp3d);
        }
        if (lin.EndPointOnFirst().NbPointOnRst() != 0)
        {
            ChFi3d_FilCommonPoint(lin.EndPointOnFirst(), lin.TransitionOnS1(), false, ref Data.ChangeVertexLastOnS1(),
                                  tolapp3d);
        }
        if (lin.StartPointOnSecond().NbPointOnRst() != 0)
        {
            ChFi3d_FilCommonPoint(lin.StartPointOnSecond(), lin.TransitionOnS2(), true, ref Data.ChangeVertexFirstOnS2(),
                                  tolapp3d);
        }
        if (lin.EndPointOnSecond().NbPointOnRst() != 0)
        {
            ChFi3d_FilCommonPoint(lin.EndPointOnSecond(), lin.TransitionOnS2(), false, ref Data.ChangeVertexLastOnS2(),
                                  tolapp3d);
        }
        done = CompleteData(Data, Func, lin, S1, S2, Or, false, false, false, false);
        if (!done)
        {
            throw new FailureException("PerformSurf : Failed approximation!");
        }
        //  maybesingular = (Func.GetMinimalDistance()<=100*tolapp3d);
        bool ok = false;
        if (!Forward)
        {
            Intf = 0;
            ChFiDS_CommonPoint cpf1 = Data.VertexFirstOnS1();
            if (cpf1.IsOnArc())
            {
                Face F1 = S1.Face();
                Face bid = new();
                Intf = !SearchFace(Spine, cpf1, F1, ref bid) ? 1 : 0;
                ok = Intf != 0;
            }
            ChFiDS_CommonPoint cpf2 = Data.VertexFirstOnS2();
            if (cpf2.IsOnArc() && !ok)
            {
                Face F2 = S2.Face();
                Face bid = new();
                Intf = !SearchFace(Spine, cpf2, F2, ref bid) ? 1 : 0;
            }
        }
        Intl = 0;
        ok = false;
        ChFiDS_CommonPoint cpl1 = Data.VertexLastOnS1();
        if (cpl1.IsOnArc())
        {
            Face F1 = S1.Face();
            Face bid = new();
            Intl = !SearchFace(Spine, cpl1, F1, ref bid) ? 1 : 0;
            ok = Intl != 0;
        }
        ChFiDS_CommonPoint cpl2 = Data.VertexLastOnS2();
        if (cpl2.IsOnArc() && !ok)
        {
            Face F2 = S2.Face();
            Face bid = new();
            Intl = !SearchFace(Spine, cpl2, F2, ref bid) ? 1 : 0;
        }
        Data.FirstSpineParam(First);
        Data.LastSpineParam(Last);

        //  if (maybesingular) SplitSurf(SeqData, lin);
        //  Necessite de trimer resultats : A faire
        return true;
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine HGuide,
                                       ChFiDS_Spine Spine, int Choix, FaceSurfaceAdaptor HS1, TopolTool I1,
                                       EdgeCurve2DAdaptor PC1, FaceSurfaceAdaptor HSref1, EdgeCurve2DAdaptor PCref1,
                                       ref bool Decroch1, FaceSurfaceAdaptor HS2, TopolTool I2,
                                       Orientation Or2, double MaxStep, double Fleche, double TolGuide,
                                       ref double First, ref double Last, bool Inside, bool Appro, bool Forward,
                                       bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new DomainErrorException("BlendFunc_CSConstRad::Section : Not implemented");
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine HGuide,
                                       ChFiDS_Spine Spine, int Choix, FaceSurfaceAdaptor HS1, TopolTool I1,
                                       Orientation Or1, FaceSurfaceAdaptor HS2, TopolTool I2,
                                       EdgeCurve2DAdaptor PC2, FaceSurfaceAdaptor HSref2, EdgeCurve2DAdaptor PCref2,
                                       ref bool Decroch2, double MaxStep, double Fleche, double TolGuide,
                                       ref double First, ref double Last, bool Inside, bool Appro, bool Forward,
                                       bool RecP, bool RecS, bool RecRst, math_Vector Soldep)
    {
        throw new DomainErrorException("BlendFunc_CSConstRad::Section : Not implemented");
    }

    internal override void PerformSurf(NCollection_Sequence<ChFiDS_SurfData> SeqData, ChFiDS_ElSpine HGuide,
                                       ChFiDS_Spine Spine, int Choix, FaceSurfaceAdaptor HS1, TopolTool I1,
                                       EdgeCurve2DAdaptor PC1, FaceSurfaceAdaptor HSref1, EdgeCurve2DAdaptor PCref1,
                                       ref bool Decroch1, Orientation Or1, FaceSurfaceAdaptor HS2,
                                       TopolTool I2, EdgeCurve2DAdaptor PC2, FaceSurfaceAdaptor HSref2,
                                       EdgeCurve2DAdaptor PCref2, ref bool Decroch2, Orientation Or2,
                                       double MaxStep, double Fleche, double TolGuide, ref double First,
                                       ref double Last, bool Inside, bool Appro, bool Forward, bool RecP1,
                                       bool RecRst1, bool RecP2, bool RecRst2, math_Vector Soldep)
    {
        throw new DomainErrorException("BlendFunc_CSConstRad::Section : Not implemented");
    }

    internal bool Done()
    {
        return done;
    }

    //=======================================================================
    // function : NbSurface
    // purpose  :  gives the number of NUBS surfaces  of the Fillet
    //=======================================================================

    internal int NbSurface()
    {
        return myListStripe.First().SetOfSurfData().Length();
    }

    //=======================================================================
    // function : SurfaceFillet
    // purpose  : gives the NUBS surface of index Index
    //=======================================================================

    internal Surface SurfaceFillet(int Index)
    {
        int isurf = myListStripe.First().SetOfSurfData().Value(Index).Surf();

        return myDS.Surface(isurf).Surface();
    }

    //=======================================================================
    // function : TolApp3d
    // purpose  :  gives the 3d tolerance reached during approximation
    //=======================================================================

    internal double TolApp3d(int Index)
    {
        int isurf = myListStripe.First().SetOfSurfData().Value(Index).Surf();

        return myDS.Surface(isurf).Tolerance();
    }

    //=======================================================================
    // function : SupportFace1
    // purpose  : gives the first support  face relative to SurfaceFillet(Index)
    //=======================================================================

    internal Face SupportFace1(int Index)
    {
        int isurf = myListStripe.First().SetOfSurfData().Value(Index).IndexOfS1();

        return ShapeCasts.Face(FromWork(myDS.Shape(isurf))); // the caller's (OCCT.NET)
    }

    //=======================================================================
    // function : SupportFace2
    // purpose  : gives the second support face relative to SurfaceFillet(Index)
    //=======================================================================

    internal Face SupportFace2(int Index)
    {
        int isurf = myListStripe.First().SetOfSurfData().Value(Index).IndexOfS2();

        return ShapeCasts.Face(FromWork(myDS.Shape(isurf))); // the caller's (OCCT.NET)
    }

    //===============================================================================
    // function : CurveOnSup1
    // purpose  :  gives  the 3d curve  of SurfaceFillet(Index)  on SupportFace1(Index)
    //===============================================================================

    internal Curve3D CurveOnFace1(int Index)
    {
        int icurv = myListStripe.First().SetOfSurfData().Value(Index).InterferenceOnS1().LineIndex();
        return myDS.Curve(icurv).Curve();
    }

    //=======================================================================
    // function : CurveOnSup2
    // purpose  : gives the 3d  curve of  SurfaceFillet(Index) on SupportFace2(Index
    //=======================================================================

    internal Curve3D CurveOnFace2(int Index)
    {
        int icurv = myListStripe.First().SetOfSurfData().Value(Index).InterferenceOnS2().LineIndex();
        return myDS.Curve(icurv).Curve();
    }

    //=======================================================================
    // function : PCurveOnFace1
    // purpose  : gives the  PCurve associated to CurvOnSup1(Index)  on the support face
    //=======================================================================

    internal Curve2D PCurveOnFace1(int Index)
    {
        return myListStripe.First().SetOfSurfData().Value(Index).InterferenceOnS1().PCurveOnFace();
    }

    //=======================================================================
    // function : PCurveOnFillet1
    // purpose  : gives the PCurve associated to CurveOnFace1(Index) on the Fillet
    //=======================================================================

    internal Curve2D PCurve1OnFillet(int Index)
    {
        return myListStripe.First().SetOfSurfData().Value(Index).InterferenceOnS1().PCurveOnSurf();
    }

    //=======================================================================
    // function : PCurveOnFace2
    // purpose  : gives the  PCurve associated to CurvOnSup2(Index)  on the support face
    //=======================================================================

    internal Curve2D PCurveOnFace2(int Index)
    {
        return myListStripe.First().SetOfSurfData().Value(Index).InterferenceOnS2().PCurveOnFace();
    }

    //=======================================================================
    // function : PCurveOnFillet2
    // purpose  : gives the PCurve associated to CurveOnFace2(Index) on the Fillet
    //=======================================================================

    internal Curve2D PCurve2OnFillet(int Index)
    {
        return myListStripe.First().SetOfSurfData().Value(Index).InterferenceOnS2().PCurveOnSurf();
    }

    //=======================================================================
    // function : FirstParameter
    // purpose  : gives the parameter of the fillet  on the first edge
    //=======================================================================

    internal double FirstParameter()
    {
        ChFiDS_Stripe st = myListStripe.First();
        ChFiDS_Spine sp = st.Spine();
        ChFiDS_SurfData sd = st.SetOfSurfData().Value(1);
        double p = sd.FirstSpineParam();
        int ind = 1;
        if (sp.IsPeriodic())
        {
            ind = sp.Index(p);
        }
        double ep = 0.0;
        if (ComputeEdgeParameter(sp, ind, p, ref ep, tolapp3d))
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
        ChFiDS_Stripe st = myListStripe.First();
        ChFiDS_Spine sp = st.Spine();
        ChFiDS_SurfData sd = st.SetOfSurfData().Value(NbSurface());
        double p = sd.LastSpineParam();
        int ind = sp.NbEdges();
        if (sp.IsPeriodic())
        {
            ind = sp.Index(p);
        }
        double ep = 0.0;
        if (ComputeEdgeParameter(sp, ind, p, ref ep, tolapp3d))
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

    internal FilletSurf_StatusType StartSectionStatus()
    {
        bool isonedge1 = myListStripe.First().SetOfSurfData().Value(1).VertexFirstOnS1().IsOnArc();
        bool isonedge2 = myListStripe.First().SetOfSurfData().Value(1).VertexFirstOnS2().IsOnArc();

        if (isonedge1 && isonedge2)
        {
            return FilletSurf_TwoExtremityOnEdge;
        }
        else if ((!isonedge1) && (!isonedge2))
        {
            return FilletSurf_NoExtremityOnEdge;
        }
        else
        {
            return FilletSurf_OneExtremityOnEdge;
        }
    }

    //=======================================================================
    // function : StatusEndSection()
    //=======================================================================

    internal FilletSurf_StatusType EndSectionStatus()
    {
        bool isonedge1 = myListStripe.First().SetOfSurfData().Value(NbSurface()).VertexLastOnS1().IsOnArc();
        bool isonedge2 = myListStripe.First().SetOfSurfData().Value(NbSurface()).VertexLastOnS2().IsOnArc();

        if (isonedge1 && isonedge2)
        {
            return FilletSurf_TwoExtremityOnEdge;
        }
        else if ((!isonedge1) && (!isonedge2))
        {
            return FilletSurf_NoExtremityOnEdge;
        }
        else
        {
            return FilletSurf_OneExtremityOnEdge;
        }
    }

    //=======================================================================
    // function : Simulate
    // purpose  : computes only the sections used in the computation of the fillet
    //=======================================================================

    internal void Simulate()
    {
        // ChFi3d_FilBuilder::Simulate(1);
        ChFiDS_Stripe Stripe = myListStripe.First();
        ref NCollection_HSequence<ChFiDS_SurfData> HData = ref Stripe.ChangeSetOfSurfData();
        HData = new NCollection_HSequence<ChFiDS_SurfData>();
        ChFiDS_Spine Spine = Stripe.ChangeSpine();
        Orientation RefOr1 = Forward, RefOr2 = Forward;
        int RefChoix = 0;
        StripeOrientations(Spine, ref RefOr1, ref RefOr2, ref RefChoix);
        Stripe.OrientationOnFace1(RefOr1);
        Stripe.OrientationOnFace2(RefOr2);
        Stripe.Choix(RefChoix);
        PerformSetOfKGen(Stripe, true);
    }

    //=======================================================================
    // function : NbSection
    // purpose  :  gives the number of sections relative to SurfaceFillet(IndexSurf)
    //=======================================================================

    internal int NbSection(int IndexSurf)
    {
        return Sect(1, IndexSurf).Length();
    }

    //=======================================================================
    // function : Section
    // purpose  : gives the   arc of circle corresponding    to section number
    //           IndexSec  of  SurfaceFillet(IndexSurf)  (The   basis curve  of the
    //           trimmed curve is a Geom_Circle)
    //=======================================================================

    internal TrimmedCurve3D Section(int IndexSurf, int IndexSec)
    {
        Sect(1, IndexSurf).Value(IndexSec).Get(out Circle3D c, out double deb, out double fin);
        CircleCurve3D Gc = new(c);
        return new TrimmedCurve3D(Gc, deb, fin);
    }
}
