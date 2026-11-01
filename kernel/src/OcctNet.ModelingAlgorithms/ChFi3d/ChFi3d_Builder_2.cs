// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.ChFi3d_Builder_0;
using static Occt.ChFi3d_Builder_2;
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


    // Simple wrapper over BRep_Tool::CurveOnSurface to get the 2d curve of an edge on a face
    // without caring about the first and last parameters of the curve.
    //===================================================================
    //   Definition by a plane
    //
    // It is considered that P1 and P2 are points associated to commonpoints compoint1 and
    // compoint2, while E1 and E2 are edges containing P1 and P2.
    // The plane containing three directions D12 T1 T2  ou  D12 represente la direction formee
    // par les points P1 et P2, T1 la tangente de E1 en P1 et T2 la tangente de
    // E2 en P2 is found (if exists).
    // Then fillet HConge is intersected by this plane
    // to find associated curve 3d C3d and the curve 2d.
    //
    //====================================================================
    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=================================================================================================

    //=======================================================================
    // function : IsInput
    // purpose  : Checks if a vector belongs to a Face
    //=======================================================================

    //=================================================================================================

    //=======================================================================
    // function : SearchFaceOnV
    // purpose  : Finds the output face(s) of the path by a vertex
    //           The following criteria should be followed
    //         -1 : The face shares regular edges with FRef
    //              (too hard condition that should be reconsidered)
    //         -2 : The vector starting in CommonPoint "belongs" to the face
    //========================================================================
    //=======================================================================
    // function : ChangeTransition
    // purpose  : Changes the transition of the second common Point, when the surface
    //           does not cross the arc
    //           As it is supposed that the support Faces are the same, it is enough
    //           to examine the cas of cutting edges.
    //========================================================================
    //=======================================================================
    // function : CallPerformSurf
    // purpose  : Encapsulates call to PerformSurf/SimulSurf
    //========================================================================

    internal void CallPerformSurf(ChFiDS_Stripe? Stripe, bool Simul, NCollection_Sequence<ChFiDS_SurfData> SeqSD, ChFiDS_SurfData? SD, ChFiDS_ElSpine? HGuide, ChFiDS_Spine? Spine, BRepAdaptor_Surface? HS1, BRepAdaptor_Surface? HS3, in gp_Pnt2d pp1, in gp_Pnt2d pp3, Adaptor3d_TopolTool? It1, BRepAdaptor_Surface? HS2, BRepAdaptor_Surface? HS4, in gp_Pnt2d pp2, in gp_Pnt2d pp4, Adaptor3d_TopolTool? It2, double MaxStep, double Fleche, double _unused1, ref double First, ref double Last, bool Inside, bool _unused2, bool forward, bool RecOnS1, bool RecOnS2, math_Vector Soldep, ref int intf, ref int intl, ref BRepAdaptor_Surface? Surf1, ref BRepAdaptor_Surface? Surf2)
    {
        BRepAdaptor_Surface? HSon1 = null, HSon2 = null;
        HSon1 = HS1;
        HSon2 = HS2;
        // Definition of the domain of path It1, It2
        It1.Initialize(HSon1);
        It2.Initialize(HSon2);

        TopAbs_Orientation Or1 = HS1.Face().Orientation();
        TopAbs_Orientation Or2 = HS2.Face().Orientation();
        int Choix = ChFi3d.NextSide(ref Or1, ref Or2, Stripe.OrientationOnFace1(), Stripe.OrientationOnFace2(),
                                                                  Stripe.Choix());
        Soldep[1] = pp1.X();
        Soldep[2] = pp1.Y();
        Soldep[3] = pp2.X();
        Soldep[4] = pp2.Y();

        double thef = First, thel = Last;
        bool isdone = false;

        if (Simul)
        {
            isdone = SimulSurf(SD, HGuide, Spine, Choix, HS1, It1, HS2, It2, tolesp, ref First, ref Last, Inside, Inside,
                                                  forward, RecOnS1, RecOnS2, Soldep, ref intf, ref intl);
        }
        else
        {
            isdone = PerformSurf(SeqSD, HGuide, Spine, Choix, HS1, It1, HS2, It2, MaxStep, Fleche, tolesp, ref First,
                                                      ref Last, Inside, Inside,
                                                      forward, RecOnS1, RecOnS2, Soldep, ref intf, ref intl);
        }

        // Case of error
        if (!isdone)
        {
            First = thef;
            Last = thel;
            bool reprise = false;
            if (HS3 is not null)
            {
                HSon1 = HS3;
                It1.Initialize(HS3);
                Or1 = HS3.Face().Orientation();
                Soldep[1] = pp3.X();
                Soldep[2] = pp3.Y();
                reprise = true;
            }
            else if (HS4 is not null)
            {
                HSon2 = HS4;
                It2.Initialize(HS4);
                Or2 = HS4.Face().Orientation();
                Soldep[3] = pp4.X();
                Soldep[4] = pp4.Y();
                reprise = true;
            }

            if (reprise)
            {
                Choix = ChFi3d.NextSide(ref Or1, ref Or2, Stripe.OrientationOnFace1(), Stripe.OrientationOnFace2(),
                                                                  Stripe.Choix());
                if (Simul)
                {
                    isdone = SimulSurf(SD, HGuide, Spine, Choix, HSon1, It1, HSon2, It2, tolesp, ref First, ref Last, Inside,
                                                          Inside,
                                                          forward, RecOnS1, RecOnS2, Soldep, ref intf, ref intl);
                }
                else
                {
                    isdone = PerformSurf(SeqSD, HGuide, Spine, Choix, HSon1, It1, HSon2, It2, MaxStep, Fleche, tolesp, ref First,
                                                              ref Last, Inside, Inside,
                                                              forward, RecOnS1, RecOnS2, Soldep, ref intf, ref intl);
                }
            }
        }
        Surf1 = HSon1;
        Surf2 = HSon2;
    }

    //=======================================================================
    // function : StripeOrientation
    // purpose  : Calculates the reference orientation determining the
    //           concave face for construction of the fillet.
    //=======================================================================

    internal bool StripeOrientations(ChFiDS_Spine? Spine, ref TopAbs_Orientation Or1, ref TopAbs_Orientation Or2, ref int ChoixConge)
    {
        // NCollection_List<TopoDS_Shape>::Iterator It;
        BRepAdaptor_Surface Sb1 = new(), Sb2 = new();
        TopAbs_Orientation Of1 = default, Of2 = default;
        TopoDS_Face ff1 = new(), ff2 = new();
        TopoDS_Edge anEdge = Spine.Edges(1);
        TopoDS_Face FirstFace = TopoDS.Face(myEdgeFirstFace[anEdge]);
        ChFi3d_conexfaces(anEdge, out ff1, out ff2, myEFMap);
        if (ff2.IsSame(FirstFace))
        {
            TopoDS_Face TmpFace = ff1;
            ff1 = ff2;
            ff2 = TmpFace;
        }
        Of1 = ff1.Orientation();
        ff1.Orientation(TopAbs_FORWARD);
        Sb1.Initialize(ff1);
        Of2 = ff2.Orientation();
        ff2.Orientation(TopAbs_FORWARD);
        Sb2.Initialize(ff2);

        ChoixConge = ChFi3d.ConcaveSide(Sb1, Sb2, Spine.Edges(1), out Or1, out Or2);
        Or1 = TopAbs.Compose(Or1, Of1);
        Or2 = TopAbs.Compose(Or2, Of2);
        return true;
    }

    //=================================================================================================

    private void ConexFaces(ChFiDS_Spine? Spine, int IEdge, ref BRepAdaptor_Surface? HS1, ref BRepAdaptor_Surface? HS2)
    {
        if (HS1 is null)
        {
            HS1 = new BRepAdaptor_Surface();
        }
        if (HS2 is null)
        {
            HS2 = new BRepAdaptor_Surface();
        }
        BRepAdaptor_Surface Sb1 = HS1; // references to *HS1, *HS2 in OCCT
        BRepAdaptor_Surface Sb2 = HS2;

        TopoDS_Face ff1 = new(), ff2 = new();
        TopoDS_Edge anEdge = Spine.Edges(IEdge);
        ChFi3d_conexfaces(Spine.Edges(IEdge), out ff1, out ff2, myEFMap);

        TopoDS_Face FirstFace = TopoDS.Face(myEdgeFirstFace[anEdge]);
        if (ff2.IsSame(FirstFace))
        {
            TopoDS_Face TmpFace = ff1;
            ff1 = ff2;
            ff2 = TmpFace;
        }

        Sb1.Initialize(ff1);
        Sb2.Initialize(ff2);
    }

    //=======================================================================
    // function : StartSol
    // purpose  : Calculates a starting solution :
    //           - one starts by parsing about ten points on the spine,
    //           - in case of fail one finds the solution on neighbor faces;
    //             section plane of edges of the adjacent face
    //             and identification of the face by connection to that edge.
    //=======================================================================

    private void StartSol(ChFiDS_Stripe? Stripe, ChFiDS_ElSpine? HGuide, ref BRepAdaptor_Surface? HS1, ref BRepAdaptor_Surface? HS2, BRepTopAdaptor_TopolTool? I1, BRepTopAdaptor_TopolTool? I2, ref gp_Pnt2d P1, ref gp_Pnt2d P2, ref double First)
    {
        ref ChFiDS_Spine? Spine = ref Stripe.ChangeSpine();
        ChFiDS_ElSpine els = HGuide; // a reference in OCCT
        int nbed = Spine.NbEdges();
        int nbessaimax = 3 * nbed;
        if (nbessaimax < 10)
        {
            nbessaimax = 10;
        }
        double unsurnbessaimax = 1.0 / nbessaimax;
        double wf = 0.9981 * Spine.FirstParameter(1) + 0.0019 * Spine.LastParameter(1);
        double wl = 0.9973 * Spine.LastParameter(nbed) + 0.0027 * Spine.FirstParameter(nbed);

        double TolE = 1.0e-7;
        BRepAdaptor_Surface AS = new();

        int nbessai = 0;
        int iedge = 0;
        int RC = Stripe.Choix();
        gp_Vec2d derive = new();
        gp_Pnt2d P2d = new();
        TopoDS_Edge cured = new();
        TopoDS_Face f1 = new(), f2 = new();
        TopAbs_Orientation Or1 = default, Or2 = default;
        int Choix = 0;
        math_Vector SolDep = new(1, 4);
        Geom2d_Curve? PC = null;
        Extrema_ExtPC PExt = new();
        PExt.Initialize(els, Spine.FirstParameter(1), Spine.LastParameter(nbed), Precision.Confusion());
        TopAbs_State Pos1 = default, Pos2 = default;
        for (nbessai = 0; nbessai <= nbessaimax; nbessai++)
        {
            double t = nbessai * unsurnbessaimax;
            double w = wf * (1.0 - t) + wl * t;
            int ie = Spine.Index(w);
            if (iedge != ie)
            {
                iedge = ie;
                cured = Spine.Edges(iedge);
                TolE = BRep_Tool.Tolerance(cured);
                ConexFaces(Spine, iedge, ref HS1, ref HS2);
                f1 = HS1.Face();
                f2 = HS2.Face();
                Or1 = f1.Orientation();
                Or2 = f2.Orientation();
                Choix =
                    ChFi3d.NextSide(ref Or1, ref Or2, Stripe.OrientationOnFace1(), Stripe.OrientationOnFace2(), RC);
            }

            double woned = 0.0, ResU = 0.0, ResV = 0.0;
            Spine.Parameter(iedge, w, out woned, true);
            cured.Orientation(TopAbs_FORWARD);
            TopoDS_Face f1forward = f1, f2forward = f2;
            f1forward.Orientation(TopAbs_FORWARD);
            f2forward.Orientation(TopAbs_FORWARD);
            PC = getCurveOnSurface(cured, f1forward);
            I1.Initialize(HS1);
            PC.D1(woned, out P1, out derive);
            // There are points on the border, and internal points are found
            if (derive.Magnitude() > Precision.PConfusion())
            {
                derive.Normalize();
                derive.Rotate(M_PI / 2);
                AS.Initialize(f1);
                ResU = AS.UResolution(TolE);
                ResV = AS.VResolution(TolE);
                derive *= 2 * (Math.Abs(derive.X()) * ResU + Math.Abs(derive.Y()) * ResV);
                P2d = P1.Translated(derive);
                if (I1.Classify(P2d, Min(ResU, ResV), false) == TopAbs_IN)
                {
                    P1 = P2d;
                }
                else
                {
                    P2d = P1.Translated(-derive);
                    if (I1.Classify(P2d, Min(ResU, ResV), false) == TopAbs_IN)
                    {
                        P1 = P2d;
                    }
                }
            }
            if (f1.IsSame(f2))
            {
                cured.Orientation(TopAbs_REVERSED);
            }
            PC = getCurveOnSurface(cured, f2forward);
            P2 = PC.Value(woned);
            Adaptor3d_Surface? HSon2 = HS2;// to avoid ambiguity
            I2.Initialize(HSon2);

            SolDep[1] = P1.X();
            SolDep[2] = P1.Y();
            SolDep[3] = P2.X();
            SolDep[4] = P2.Y();
            BRepAdaptor_Curve Ced = Spine.CurrentElementarySpine(iedge);
            gp_Pnt pnt = Ced.Value(woned);

            if (Projection(PExt, pnt, els, ref w, tolapp3d)
                    && PerformFirstSection(Spine, HGuide, Choix, HS1, HS2, I1, I2, w, SolDep, ref Pos1, ref Pos2))
            {
                P1.SetCoord(SolDep[1], SolDep[2]);
                P2.SetCoord(SolDep[3], SolDep[4]);
                First = w;
                return;
            }
        }
        // No solution was found for the faces adjacent to the trajectory.
        // Now one tries the neighbor faces.
        iedge = 0;
        for (nbessai = 0; nbessai <= nbessaimax; nbessai++)
        {
            double t = nbessai * unsurnbessaimax;
            double w = wf * (1.0 - t) + wl * t;
            iedge = Spine.Index(w);
            cured = Spine.Edges(iedge);
            ConexFaces(Spine, iedge, ref HS1, ref HS2);
            f1 = HS1.Face();
            f2 = HS2.Face();
            Or1 = f1.Orientation();
            Or2 = f2.Orientation();
            Choix =
                ChFi3d.NextSide(ref Or1, ref Or2, Stripe.OrientationOnFace1(), Stripe.OrientationOnFace2(), RC);
            double woned = 0.0;
            Spine.Parameter(iedge, w, out woned, true);
            TopoDS_Face f1forward = f1, f2forward = f2;
            f1forward.Orientation(TopAbs_FORWARD);
            f2forward.Orientation(TopAbs_FORWARD);
            PC = getCurveOnSurface(cured, f1forward);
            P1 = PC.Value(woned);
            PC = getCurveOnSurface(cured, f2forward);
            P2 = PC.Value(woned);
            Adaptor3d_Surface? HSon1 = HS1;// to avoid ambiguity
            Adaptor3d_Surface? HSon2 = HS2;// to avoid ambiguity
            I1.Initialize(HSon1);
            I2.Initialize(HSon2);
            SolDep[1] = P1.X();
            SolDep[2] = P1.Y();
            SolDep[3] = P2.X();
            SolDep[4] = P2.Y();
            BRepAdaptor_Curve Ced = Spine.CurrentElementarySpine(iedge);
            gp_Pnt pnt = Ced.Value(woned);
            //    Extrema_LocateExtPC ext(pnt,els,w,1.e-8);
            //    if(ext.IsDone()){
            //      w = ext.Point().Parameter();
            if (Projection(PExt, pnt, els, ref w, tolapp3d))
            {
                PerformFirstSection(Spine, HGuide, Choix, HS1, HS2, I1, I2, w, SolDep, ref Pos1, ref Pos2);
                gp_Pnt P = new();
                gp_Vec V = new();
                HGuide.D1(w, out P, out V);
                Geom_Plane? pl = new Geom_Plane(P, new gp_Dir(V));
                GeomAdaptor_Surface? plane = new GeomAdaptor_Surface(pl);

                bool bonvoisin = true, found = false;
                int NbChangement = 0;
                for (NbChangement = 1; bonvoisin && (!found) && (NbChangement < 5); NbChangement++)
                {
                    if (Pos1 != TopAbs_IN)
                    {
                        bonvoisin = BonVoisin(P, HS1, ref f1, plane, cured, ref SolDep[1], ref SolDep[2], myEFMap, tolapp3d);
                    }
                    if (Pos2 != TopAbs_IN && bonvoisin)
                    {
                        bonvoisin = BonVoisin(P, HS2, ref f2, plane, cured, ref SolDep[3], ref SolDep[4], myEFMap, tolapp3d);
                    }
                    if (bonvoisin)
                    {
                        f1 = HS1.Face();
                        f2 = HS2.Face();
                        Or1 = f1.Orientation();
                        Or2 = f2.Orientation();
                        Choix = ChFi3d.NextSide(ref Or1, ref Or2,
                                                                          Stripe.OrientationOnFace1(), Stripe.OrientationOnFace2(), RC);
                        Adaptor3d_Surface? HSon1new = HS1;// to avoid ambiguity
                        Adaptor3d_Surface? HSon2new = HS2;// to avoid ambiguity
                        I1.Initialize(HSon1new);
                        I2.Initialize(HSon2new);
                        if (PerformFirstSection(Spine, HGuide, Choix, HS1, HS2, I1, I2, w, SolDep, ref Pos1, ref Pos2))
                        {
                            P1.SetCoord(SolDep[1], SolDep[2]);
                            P2.SetCoord(SolDep[3], SolDep[4]);
                            First = w;
                            found = true;
                        }
                    }
                }
                if (found)
                {
                    return;
                }
            }
        }
        Spine.SetErrorStatus(ChFiDS_StartsolFailure);
        throw new Standard_Failure("StartSol echec");
    }

    //=================================================================================================

    //=======================================================================
    // function : StartSol
    // purpose  : If the commonpoint is not OnArc the input face
    //           is returned and 2D point is updated,
    //           if it is OnArc
    //              if it is detached  the input face
    //              is returned and 2D point is updated,
    //              otherwise
    //                 either there is a neighbor tangent face and it is returned
    //                         with recalculated 2D point
    //                 or if there is no face
    //                         if the reference arc is Vref (extremity of the spine)
    //                            this is the end and the input face is returned
    //                         otherwise this is an obstacle and HC is updated.
    //=======================================================================

    private bool StartSol(ChFiDS_Spine Spine,
                          BRepAdaptor_Surface HS,            // New face
                          ref gp_Pnt2d pons,                 // " Localization
                          ref BRepAdaptor_Curve2d HC,        // Representation of the obstacle
                          ref double W, ChFiDS_SurfData SD, bool isfirst, int ons,
                          BRepAdaptor_Surface HSref,         // The other representation
                          BRepAdaptor_Curve2d HCref,         // of the obstacle
                          ref bool RecP, ref bool RecS, ref bool RecRst, ref bool c1obstacle,
                          ref BRepAdaptor_Surface HSBis,     // Face of support
                          ref gp_Pnt2d PBis,                 // and its point
                          bool decroch, in TopoDS_Vertex Vref)
    {
        RecP = false;
        RecS = false;
        RecRst = false;
        c1obstacle = false;
        TopoDS_Face Fv = new();
        Geom2d_Curve? aPCurve = null;

        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        TopoDS_Face Fref = HSref is not null ? HSref.Face() : new TopoDS_Face();
        TopoDS_Face F = TopoDS.Face(DStr.Shape(SD.Index(ons)));
        if (F.IsNull())
        {
            return false;
        }

        ChFiDS_CommonPoint aCommonPoint = SD.Vertex(isfirst, ons);
        HSBis = null;

        if (aCommonPoint.IsOnArc())
        {
            int notons = ons == 1 ? 2 : 1;
            ChFiDS_CommonPoint CPbis = SD.Vertex(isfirst, notons);
            if (CPbis.IsOnArc())
            { // It is checked if it is not the extension zone
                // In case aCommonPoint is not at the end of surfdata and it is not necessary to take it into
                // account except for separate cases (ie pointus) ... ts and tns were earlier
                // aCommonPoint.Parameter() and CPbis.Parameter, but sometimes they had no values.
                double ts = SD.Interference(ons).Parameter(isfirst);
                double tns = SD.Interference(notons).Parameter(isfirst);
                // Arbitrary test (to precise)
                bool isExtend = isfirst ? (ts - tns > 100 * tolesp) : (tns - ts > 100 * tolesp);
                if (isExtend && !aCommonPoint.Point().IsEqual(CPbis.Point(), 0))
                {
                    //  the state is preserved and False is returned (extension by the expected plane).
                    aPCurve = SD.Interference(ons).PCurveOnFace();
                    if (aPCurve is null)
                    {
                        return false;
                    }
                    HS.Initialize(F);
                    // The 2nd point is given by its trace on the support surface
                    RecS = false;
                    pons = aPCurve.Value(tns);
                    return false;
                }
            }
        }

        if (aCommonPoint.IsVertex() && HC is not null && !decroch)
        {
            // The edge is changed, the parameter is updated and
            // eventually the support face and(or) the reference face.
            TopoDS_Vertex VCP = aCommonPoint.Vertex();
            TopoDS_Edge EHC = HC.Edge();
            if (VCP.IsNull() || EHC.IsNull() || Fref.IsNull() || HSref is null || HCref is null)
            {
                return false;
            }
            // One starts by searching in Fref another edge referencing VCP.
            TopoDS_Edge newedge = new();
            TopoDS_Edge edgereg = new();
            TopoDS_Face facereg = new();
            TopoDS_Face bidface = Fref;
            bidface.Orientation(TopAbs_FORWARD);
            for (TopExp_Explorer ex1 = new(bidface, TopAbs_EDGE); ex1.More(); ex1.Next())
            {
                TopoDS_Edge cured = TopoDS.Edge(ex1.Current());
                bool found = false;
                if (!cured.IsSame(EHC))
                {
                    for (TopExp_Explorer ex2 = new(cured, TopAbs_VERTEX); ex2.More() && !found; ex2.Next())
                    {
                        if (ex2.Current().IsSame(VCP))
                        {
                            if (IsG1(myEFMap, cured, Fref, ref Fv))
                            {
                                edgereg = cured;
                                facereg = Fv;
                            }
                            else
                            {
                                found = true;
                            }
                        }
                    }
                }
                if (found)
                {
                    newedge = cured;
                    break;
                }
            }
            if (newedge.IsNull())
            {
                // It is checked if EHC is not a closed edge.
                TopoDS_Vertex V1 = new(), V2 = new();
                TopExp.Vertices(EHC, out V1, out V2);
                if (V1.IsSame(V2))
                {
                    newedge = EHC;
                    double w1 = BRep_Tool.Parameter(V1, EHC);
                    double w2 = BRep_Tool.Parameter(V2, EHC);
                    ChFiDS_FaceInterference fi = SD.Interference(ons);
                    Geom2d_Curve? pcf = fi.PCurveOnFace();
                    double ww = fi.Parameter(isfirst);
                    gp_Pnt2d pww = pcf is null ? SD.Get2dPoints(isfirst, ons) : pcf.Value(ww);
                    gp_Pnt2d p1 = HC.Value(w1);
                    gp_Pnt2d p2 = HC.Value(w2);

                    if (p1.Distance(pww) > p2.Distance(pww))
                    {
                        W = w1;
                        pons = p1;
                    }
                    else
                    {
                        W = w2;
                        pons = p2;
                    }
                    RecP = true;
                    c1obstacle = true;
                    return true;
                }
                else if (!edgereg.IsNull())
                {
                    // the reference edge and face are changed.
                    Fref = facereg;
                    HSref.Initialize(Fref);
                    for (TopExp_Explorer ex1 = new(facereg, TopAbs_EDGE); ex1.More() && newedge.IsNull(); ex1.Next())
                    {
                        TopoDS_Edge cured = TopoDS.Edge(ex1.Current());
                        if (!cured.IsSame(edgereg))
                        {
                            for (TopExp_Explorer ex2 = new(cured, TopAbs_VERTEX); ex2.More(); ex2.Next())
                            {
                                if (ex2.Current().IsSame(VCP))
                                {
                                    if (!IsG1(myEFMap, cured, Fref, ref Fv))
                                    {
                                        newedge = cured;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            // it is necessary to find the new support face of the fillet :
            // connected to FRef along the newedge.
            if (newedge.IsNull())
            {
                throw new Standard_Failure("StartSol : chain is not possible, new obstacle not found");
            }
            if (IsG1(myEFMap, newedge, Fref, ref Fv))
            {
                throw new Standard_Failure("StartSol : chain is not possible, config non processed");
            }
            else if (Fv.IsNull())
            {
                throw new Standard_Failure("StartSol : chain is not possible, new obstacle not found");
            }
            else
            {
                Geom2d_Curve? aRefPCurve = getCurveOnSurface(newedge, Fref);
                if (aRefPCurve is null)
                {
                    return false;
                }

                TopoDS_Face newface = Fv;
                newface.Orientation(TopAbs_FORWARD);
                for (TopExp_Explorer ex = new(newface, TopAbs_EDGE); ex.More(); ex.Next())
                {
                    if (ex.Current().IsSame(newedge))
                    {
                        newedge = TopoDS.Edge(ex.Current());
                        break;
                    }
                }
                Geom2d_Curve? aNewPCurve = getCurveOnSurface(newedge, Fv);
                if (aNewPCurve is null)
                {
                    return false;
                }
                HS.Initialize(Fv);
                W = BRep_Tool.Parameter(VCP, newedge);
                HCref.Initialize(newedge, Fref);
                HC.Initialize(newedge, Fv);
                pons = HC.Value(W);
            }
            RecP = true;
            c1obstacle = true;
            return true;
        } // End of Case Vertex && Obstacle
        else if (aCommonPoint.IsOnArc() && HC is not null && !decroch)
        {
            // Nothing is changed, the parameter is only updated.
            W = aCommonPoint.ParameterOnArc();
            c1obstacle = true;
            return true;
        }

        HC = null;

        if (aCommonPoint.IsOnArc())
        {
            TopoDS_Edge anArcEdge = aCommonPoint.Arc();
            if (anArcEdge.IsNull())
            {
                return false;
            }

            // Lambda to avoid code duplication.
            // Attempts to set HS, W, aPCurve and pons to default return values.
            // (A static local function: C# lambdas cannot capture the ref parameters W and pons.)
            static bool prepareDefaultReturn(in TopoDS_Edge anArcEdge, in TopoDS_Face F, BRepAdaptor_Surface HS,
                                             in ChFiDS_CommonPoint aCommonPoint, ref Geom2d_Curve aPCurve,
                                             ref double W, ref gp_Pnt2d pons)
            {
                aPCurve = getCurveOnSurface(anArcEdge, F);
                if (aPCurve is null)
                {
                    return false;
                }
                HS.Initialize(F);
                W = aCommonPoint.ParameterOnArc();
                pons = aPCurve.Value(W);
                return true;
            }

            if (decroch)
            {
                if (Fref.IsNull())
                {
                    prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                    return false;
                }
                aPCurve = getCurveOnSurface(anArcEdge, Fref);
                if (aPCurve is null)
                {
                    prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                    return false;
                }
                HS.Initialize(Fref);
                W = aCommonPoint.ParameterOnArc();
                pons = aPCurve.Value(W);
                RecS = true;
                return true;
            }

            if (SearchFace(Spine, aCommonPoint, F, ref Fv))
            {
                if (Fv.IsNull())
                {
                    prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                    return false;
                }
                HS.Initialize(Fv);
                RecS = true;
                if (aCommonPoint.IsVertex())
                {
                    // One goes directly by the Vertex
                    // And it is checked that there are no other candidates
                    TopoDS_Face aux = new();
                    int Nb = SearchFaceOnV(aCommonPoint, F, myVEMap, myEFMap, ref Fv, ref aux);

                    pons = BRep_Tool.Parameters(aCommonPoint.Vertex(), Fv);
                    HS.Initialize(Fv);
                    if (Nb >= 2)
                    {
                        HSBis = new BRepAdaptor_Surface(aux);
                        PBis = BRep_Tool.Parameters(aCommonPoint.Vertex(), aux);
                    }
                    return true;
                }
                // otherwise one passes by the arc...
                if (!Fv.IsSame(F))
                {
                    Fv.Orientation(TopAbs_FORWARD);
                    TopoDS_Edge newedge = new();
                    for (TopExp_Explorer ex = new(Fv, TopAbs_EDGE); ex.More(); ex.Next())
                    {
                        if (ex.Current().IsSame(anArcEdge))
                        {
                            newedge = TopoDS.Edge(ex.Current());
                            break;
                        }
                    }
                    //  In cas of Tangent output, the current face becomes the support face
                    if (isTangentToArc(aCommonPoint, 0.1))
                    {
                        aPCurve = getCurveOnSurface(aCommonPoint.Arc(), F);
                        if (aPCurve is null)
                        {
                            prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                            return false;
                        }
                        HSBis = new BRepAdaptor_Surface(F);
                        PBis = aPCurve.Value(aCommonPoint.ParameterOnArc());
                    }

                    if (newedge.IsNull())
                    {
                        prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                        return false;
                    }
                    aPCurve = getCurveOnSurface(newedge, Fv);
                }
                else
                {
                    TopoDS_Edge newedge = anArcEdge;
                    newedge.Reverse();
                    Fv.Orientation(TopAbs_FORWARD);
                    aPCurve = getCurveOnSurface(newedge, Fv);
                }
                if (aPCurve is null)
                {
                    prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                    return false;
                }
                pons = aPCurve.Value(aCommonPoint.ParameterOnArc());
                return true;
            }
            else if (!Fv.IsNull())
            {
                c1obstacle = true;
                if (!Vref.IsNull())
                {
                    for (TopExp_Explorer ex = new(anArcEdge, TopAbs_VERTEX); ex.More(); ex.Next())
                    {
                        if (ex.Current().IsSame(Vref))
                        {
                            c1obstacle = false;
                            break;
                        }
                    }
                }
                if (c1obstacle)
                {
                    if (HSref is null || HCref is null)
                    {
                        prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                        return false;
                    }
                    HS.Initialize(Fv);
                    HSref.Initialize(F);
                    W = aCommonPoint.ParameterOnArc();
                    HC = new BRepAdaptor_Curve2d();
                    TopoDS_Edge newedge = new();
                    TopoDS_Face newface = Fv;
                    newface.Orientation(TopAbs_FORWARD);
                    for (TopExp_Explorer ex = new(newface, TopAbs_EDGE); ex.More(); ex.Next())
                    {
                        if (ex.Current().IsSame(anArcEdge))
                        {
                            newedge = TopoDS.Edge(ex.Current());
                            break;
                        }
                    }

                    if (!newedge.IsNull())
                    {
                        HC.Initialize(newedge, Fv);
                        pons = HC.Value(W);
                        HCref.Initialize(anArcEdge, F);
                        if (aCommonPoint.IsVertex())
                        {
                            RecP = true;
                        }
                        else
                        {
                            RecRst = true;
                        }
                        return true;
                    }
                    else
                    {
                        prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                        return false;
                    }
                }
                else
                {
                    prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                    return false;
                }
            }
            else
            {
                // there is no neighbor face, the state is preserved and False is returned.
                prepareDefaultReturn(anArcEdge, F, HS, aCommonPoint, ref aPCurve, ref W, ref pons);
                return false;
            }
        }
        else
        {
            HS.Initialize(F);
            ChFiDS_FaceInterference FI = SD.Interference(ons);
            if (FI.PCurveOnFace() is null)
            {
                pons = SD.Get2dPoints(isfirst, ons);
            }
            else
            {
                pons = FI.PCurveOnFace().Value(FI.Parameter(isfirst));
            }
        }
        return true;
    }

    //=================================================================================================

    internal bool SearchFace(ChFiDS_Spine? Spine, in ChFiDS_CommonPoint Pc, in TopoDS_Face FRef, ref TopoDS_Face FVoi)
    {
        bool Trouve = false;
        if (!Pc.IsOnArc())
        {
            return false;
        }
        FVoi.Nullify();
        TopoDS_Edge E = new();
        if (Pc.IsVertex())
        {
            // attention it is necessary to analyze all faces that turn around of the vertex
            if (Pc.HasVector())
            { // General processing
                TopoDS_Face Fbis = new();
                int nb_faces = 0;
                nb_faces = SearchFaceOnV(Pc, FRef, myVEMap, myEFMap, ref FVoi, ref Fbis);
                return (nb_faces > 0);
            }
            else
            { // Processing using the spine
                bool FindFace = false;
                gp_Pnt Point = new();
                gp_Vec VecSpine = new();
                Spine.D1(Pc.Parameter(), out Point, out VecSpine);

                // It is checked if one leaves from the current face.
                FindFace = IsInput(VecSpine, Pc.Vertex(), FRef);
                if (FindFace)
                {
                    VecSpine.Reverse();
                    FindFace = IsInput(VecSpine, Pc.Vertex(), FRef);
                }
                // If one does not leave, it is ended
                if (FindFace)
                {
                    FVoi = FRef;
                    return true;
                }

                // Otherwise one finds the next among shared Faces
                // by a common edge G1
                NCollection_List<TopoDS_Shape>.Iterator ItE = new(), ItF = new();
                for (ItE.Initialize(myVEMap[Pc.Vertex()]); ItE.More() && (!FindFace); ItE.Next())
                {
                    E = TopoDS.Edge(ItE.Value());
                    Trouve = false;
                    for (ItF.Initialize(myEFMap[E]); //, Trouve=false;           15.11.99 SVV
                              ItF.More() && (!Trouve);
                              ItF.Next())
                    {
                        if (TopoDS.Face(ItF.Value()).IsSame(FRef))
                        {
                            Trouve = true;
                        }
                    }
                    if (Trouve)
                    {
                        FindFace = IsG1(myEFMap, E, FRef, ref FVoi);
                    }
                    if (FindFace)
                    {
                        FindFace = false;
                        if (Spine is null)
                        {
                            // La Spine peut etre nulle (ThreeCorner)
                            return false;
                        }

                        // It is checked if the selected face actually possesses edges of the spine
                        // containing the vertex on its front
                        // This processing should go only if the Vertex belongs to the spine
                        // This is a single case, for other vertexes it is required to do other things
                        Trouve = false;
                        for (int IE = 1; //, Trouve=false;                   15.11.99  SVV
                                  (IE <= Spine.NbEdges()) && (!Trouve);
                                  IE++)
                        {
                            E = Spine.Edges(IE);
                            if ((TopExp.FirstVertex(E).IsSame(Pc.Vertex()))
                                    || (TopExp.LastVertex(E).IsSame(Pc.Vertex())))
                            {
                                for (ItF.Initialize(myEFMap[E]), Trouve = false; ItF.More() && (!Trouve); ItF.Next())
                                {
                                    if (TopoDS.Face(ItF.Value()).IsSame(FVoi))
                                    {
                                        Trouve = true;
                                    }
                                }
                            }
                        }
                        FindFace = Trouve;
                    }
                }
            }
        }
        else
        {
            return IsG1(myEFMap, Pc.Arc(), FRef, ref FVoi);
        }
        return false;
    }

    //=======================================================================
    // function : ChFi3d_SingularExtremity
    // purpose  : load the vertex in the DS and calculate the pcurve
    //           for an extremity in case of singular freeboundary
    //           or periodic and singular at the cut.
    //=======================================================================
    //=======================================================================
    // function : ChFi3d_MakeExtremities
    // purpose  : calculate Curves3d and pcurves of extremities in
    //           periodic and freeboundary cases.
    //=======================================================================
    //=================================================================================================

    //=======================================================================
    // function : InsertAfter
    // purpose  : insert Item after ref in Seq. If ref is null, the item is
    //           inserted at the beginning.
    //=======================================================================

    //=================================================================================================

    //=======================================================================
    // function : InsertBefore
    // purpose  : Insert item before ref in Seq. If ref is null, the item is
    //           inserted in the queue.
    //=======================================================================

    //=================================================================================================

    private void PerformSetOfSurfOnElSpine(ChFiDS_ElSpine? HGuide, ChFiDS_Stripe? Stripe, BRepTopAdaptor_TopolTool? It1, BRepTopAdaptor_TopolTool? It2, bool Simul = false)
    {
        ChFiDS_ElSpine Guide = HGuide; // a reference in OCCT

        ChFiDS_ElSpine? OffsetHGuide = null;
        ref ChFiDS_Spine? Spine = ref Stripe.ChangeSpine();
        if (Spine.Mode() == ChFiDS_ChamfMode.ChFiDS_ConstThroatWithPenetrationChamfer)
        {
            NCollection_List<ChFiDS_ElSpine> ll = Spine.ChangeElSpines();
            NCollection_List<ChFiDS_ElSpine> ll_offset = Spine.ChangeOffsetElSpines();
            NCollection_List<ChFiDS_ElSpine>.Iterator ILES = new(ll), ILES_offset = new(ll_offset);
            for (; ILES.More(); ILES.Next(), ILES_offset.Next())
            {
                ChFiDS_ElSpine? aHElSpine = ILES.Value();
                if (aHElSpine == HGuide)
                {
                    OffsetHGuide = ILES_offset.Value();
                }
            }
        }

        double wf = Guide.FirstParameter();
        double wl = Guide.LastParameter();
        double locfleche = (wl - wf) * fleche;
        double wfsav = wf, wlsav = wl;
        if (!Guide.IsPeriodic())
        {
            // Now the ElSpine is artificially extended to help rsnld.
            double prab = 0.01;
            Guide.FirstParameter(wf - prab * (wl - wf));
            Guide.LastParameter(wl + prab * (wl - wf));
            if (OffsetHGuide is not null)
            {
                OffsetHGuide.FirstParameter(wf - prab * (wl - wf));
                OffsetHGuide.LastParameter(wl + prab * (wl - wf));
            }
        }
        // occ::handle<ChFiDS_Spine>&  Spine = Stripe->ChangeSpine();
        int ii = 0, nbed = Spine.NbEdges();
        double lastedlastp = Spine.LastParameter(nbed);

        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();

        ChFiDS_SurfData @ref = Guide.Previous(); // "ref" in OCCT
        ChFiDS_SurfData? refbis = null, SD = null;
        ChFiDS_SurfData? raf = Guide.Next();
        RemoveSD(Stripe, @ref, raf);

        BRepAdaptor_Surface? HS1 = new BRepAdaptor_Surface();
        BRepAdaptor_Surface? HS2 = new BRepAdaptor_Surface();
        BRepAdaptor_Surface? HS3 = null, HS4 = null;
        BRepAdaptor_Surface? HSref1 = new BRepAdaptor_Surface();
        BRepAdaptor_Surface? HSref2 = new BRepAdaptor_Surface();
        BRepAdaptor_Curve2d? HC1 = null, HC2 = null;
        BRepAdaptor_Curve2d? HCref1 = new BRepAdaptor_Curve2d();
        BRepAdaptor_Curve2d? HCref2 = new BRepAdaptor_Curve2d();
        bool decroch1 = false, decroch2 = false;
        bool RecP1 = false, RecS1 = false, RecRst1 = false, obstacleon1 = false;
        bool RecP2 = false, RecS2 = false, RecRst2 = false, obstacleon2 = false;
        gp_Pnt2d pp1 = new(), pp2 = new(), pp3 = new(), pp4 = new();
        double w1 = 0.0, w2 = 0.0;
        math_Vector Soldep = new(1, 4);
        math_Vector SoldepCS = new(1, 3);
        math_Vector SoldepCC = new(1, 2);

        // Restore a neighboring KPart.
        // If no neighbor calculation start point.
        bool forward = true;
        bool Inside = false;
        double First = wf;
        double Last = wl;
        bool Ok1 = true, Ok2 = true;
        // Restore the next KPart if it exists
        TopoDS_Vertex Vref = new();
        if (@ref is null && raf is null)
        {
            // sinon solution approchee.
            Inside = true;

            StartSol(Stripe, HGuide, ref HS1, ref HS2, It1, It2, ref pp1, ref pp2, ref First);

            Last = wf;
            if (Guide.IsPeriodic())
            {
                Last = First - Guide.Period();
                Guide.SaveFirstParameter();
                Guide.FirstParameter(Last);
                Guide.SaveLastParameter();
                Guide.LastParameter(First * 1.1); // Extension to help rsnld.
                if (OffsetHGuide is not null)
                {
                    OffsetHGuide.SaveFirstParameter();
                    OffsetHGuide.FirstParameter(Last);
                    OffsetHGuide.SaveLastParameter();
                    OffsetHGuide.LastParameter(First * 1.1); // Extension to help rsnld.
                }
            }
        }
        else
        {
            if (!Spine.IsPeriodic() && (wl - lastedlastp > -tolesp))
            {
                Vref = Spine.LastVertex();
            }
            if (@ref is null)
            {
                if (!Spine.IsPeriodic() && (wf < tolesp))
                {
                    Vref = Spine.FirstVertex();
                }
                @ref = raf;
                forward = false;
                First = wl;
                Last = Guide.FirstParameter();
            }

            Ok1 = StartSol(Spine, HS1, ref pp1, ref HC1, ref w1, @ref, !forward, 1, HSref1, HCref1, ref RecP1, ref RecS1,
                               ref RecRst1, ref obstacleon1, ref HS3, ref pp3, decroch1, Vref);
            Ok2 = StartSol(Spine, HS2, ref pp2, ref HC2, ref w2, @ref, !forward, 2, HSref2, HCref2, ref RecP2, ref RecS2,
                               ref RecRst2, ref obstacleon2, ref HS4, ref pp4, decroch2, Vref);
            HC1 = null;
            HC2 = null;

            if (Ok1 && Ok2)
            {
                if (forward)
                {
                    Guide.FirstParameter(wf);
                    if (OffsetHGuide is not null)
                    {
                        OffsetHGuide.FirstParameter(wf);
                    }
                }
                else
                {
                    Guide.LastParameter(wl);
                    if (OffsetHGuide is not null)
                    {
                        OffsetHGuide.LastParameter(wl);
                    }
                }
            }
        }
        bool fini = false;
        bool complete = Inside;
        if (!Guide.IsPeriodic())
        {
            int indf = Spine.Index(wf);
            int indl = Spine.Index(wl, false);
            if (Spine.IsPeriodic() && (indl < indf))
            {
                indl += nbed;
            }
            nbed = indl - indf + 1;
        }
        // No Max at the touch : 20 points by edge at average without
        // counting the extensions.

        double bidf = wf, bidl = wl;
        if (!Spine.IsPeriodic())
        {
            bidf = Max(0.0, wf);
            bidl = Min(wl, Spine.LastParameter(Spine.NbEdges()));
            // PMN 20/07/98 : Attention in case if there is only extension
            if ((bidl - bidf) < 0.01 * Spine.LastParameter(Spine.NbEdges()))
            {
                bidf = wf;
                bidl = wl;
            }
        }
        double MaxStep = (bidl - bidf) * 0.05 / nbed;
        double Firstsov = 0.0;
        int intf = 0, intl = 0;
        while (!fini)
        {
            // are these the ends (no extension on periodic).
            Ok1 = true;
            Ok2 = true;
            if (!Spine.IsPeriodic())
            {
                if (wf < tolesp && (complete == Inside))
                {
                    if (Spine.FirstStatus() == ChFiDS_OnSame)
                    {
                        intf = 2;
                    }
                    else
                    {
                        intf = 1;
                    }
                }
                if (Spine.IsTangencyExtremity(true))
                {
                    intf = 4;
                    Guide.FirstParameter(wfsav);
                    if (OffsetHGuide is not null)
                    {
                        OffsetHGuide.FirstParameter(wfsav);
                    }
                }
                if (wl - lastedlastp > -tolesp)
                {
                    if (Spine.LastStatus() == ChFiDS_OnSame)
                    {
                        intl = 2;
                    }
                    else
                    {
                        intl = 1;
                    }
                }
                if (Spine.IsTangencyExtremity(false))
                {
                    intl = 4;
                    Guide.LastParameter(wlsav);
                    if (OffsetHGuide is not null)
                    {
                        OffsetHGuide.LastParameter(wlsav);
                    }
                }
            }
            if (intf != 0 && !forward)
            {
                Vref = Spine.FirstVertex();
            }
            if (intl != 0 && forward)
            {
                Vref = Spine.LastVertex();
            }
            if (@ref is not null)
            {
                Ok1 = StartSol(Spine, HS1, ref pp1, ref HC1, ref w1, @ref, !forward, 1, HSref1, HCref1, ref RecP1, ref RecS1,
                               ref RecRst1, ref obstacleon1, ref HS3, ref pp3, decroch1, Vref);
                Ok2 = StartSol(Spine, HS2, ref pp2, ref HC2, ref w2, @ref, !forward, 2, HSref2, HCref2, ref RecP2, ref RecS2,
                               ref RecRst2, ref obstacleon2, ref HS4, ref pp4, decroch2, Vref);
            }

            // No more connected faces. Construction of the tangent plane to continue the path
            // till the output on the other face.
            if ((!Ok1 && HC1 is null) || (!Ok2 && HC2 is null))
            {
                if ((intf != 0 && !forward) || (intl != 0 && forward))
                {
                    if (!Ok1)
                    {
                        ChFi3d_BuildPlane(DStr, HS1, ref pp1, @ref, !forward, 1);
                    }
                    if (!Ok2)
                    {
                        ChFi3d_BuildPlane(DStr, HS2, ref pp2, @ref, !forward, 2);
                    }
                    if (intf != 0)
                    {
                        intf = 5;
                    }
                    else if (intl != 0)
                    {
                        intl = 5;
                    }
                    if (forward)
                    {
                        Guide.FirstParameter(wf);
                        if (OffsetHGuide is not null)
                        {
                            OffsetHGuide.FirstParameter(wf);
                        }
                    }
                    else
                    {
                        Guide.LastParameter(wl);
                        if (OffsetHGuide is not null)
                        {
                            OffsetHGuide.LastParameter(wl);
                        }
                    }
                }
                else
                {
                    throw new Standard_Failure("PerformSetOfSurfOnElSpine : Chaining is impossible.");
                }
            }

            // Definition of the domain of patch It1, It2
            Adaptor3d_Surface? HSon1 = HS1;// to avoid ambiguity
            Adaptor3d_Surface? HSon2 = HS2;// to avoid ambiguity
            It1.Initialize(HSon1);
            It2.Initialize(HSon2);

            // Calculate one (several if singularity) SurfaData
            SD = new ChFiDS_SurfData();
            NCollection_Sequence<ChFiDS_SurfData> SeqSD = new();
            SeqSD.Append(SD);

            if (obstacleon1 && obstacleon2)
            {
                TopAbs_Orientation Or1 = HSref1.Face().Orientation();
                TopAbs_Orientation Or2 = HSref2.Face().Orientation();
                int Choix = ChFi3d.NextSide(ref Or1, ref Or2, Stripe.OrientationOnFace1(), Stripe.OrientationOnFace2(),
                                                                          Stripe.Choix());

                // Calculate the criterion of Choice edge / edge
                if (Choix % 2 == 0)
                {
                    Choix = 4;
                }
                else
                {
                    Choix = 1;
                }

                SoldepCC[1] = w1;
                SoldepCC[2] = w2;
                if (Simul)
                {
                    SimulSurf(SD, HGuide, Spine, Choix, HS1, It1, HC1, HSref1, HCref1, ref decroch1, Or1, HS2, It2, HC2, HSref2,
                                        HCref2, ref decroch2, Or2, locfleche, tolesp, ref First, ref Last, Inside, Inside,
                                        forward, RecP1, RecRst1, RecP2, RecRst2, SoldepCC);
                }
                else
                {
                    PerformSurf(SeqSD, HGuide, Spine, Choix, HS1, It1, HC1, HSref1, HCref1, ref decroch1, Or1, HS2, It2, HC2,
                                            HSref2, HCref2, ref decroch2, Or2, MaxStep, locfleche, tolesp, ref First, ref Last, Inside,
                                            Inside,
                                            forward, RecP1, RecRst1, RecP2, RecRst2, SoldepCC);
                }
                SD.ChangeIndexOfS1(DStr.AddShape(HS1.Face()));
                SD.ChangeIndexOfS2(DStr.AddShape(HS2.Face()));
            }
            else if (obstacleon1)
            {
                TopAbs_Orientation Or1 = HSref1.Face().Orientation();
                TopAbs_Orientation Or2 = HS2.Face().Orientation();
                int Choix = ChFi3d.NextSide(ref Or1, ref Or2, Stripe.OrientationOnFace1(), Stripe.OrientationOnFace2(),
                                                                          -Stripe.Choix());
                if (Choix % 2 == 1)
                {
                    Choix++;
                }
                else
                {
                    Choix--;
                }
                SoldepCS[3] = w1;
                SoldepCS[1] = pp2.X();
                SoldepCS[2] = pp2.Y();
                if (Simul)
                {
                    SimulSurf(SD, HGuide, Spine, Choix, HS1, It1, HC1, HSref1, HCref1, ref decroch1, HS2, It2, Or2, locfleche, tolesp,
                                        ref First, ref Last, Inside, Inside,
                                        forward, RecP1, RecS2, RecRst1, SoldepCS);
                }
                else
                {
                    PerformSurf(SeqSD, HGuide, Spine, Choix, HS1, It1, HC1, HSref1, HCref1, ref decroch1, HS2, It2, Or2, MaxStep,
                                            locfleche, tolesp, ref First, ref Last, Inside, Inside,
                                            forward, RecP1, RecS2, RecRst1, SoldepCS);
                }
                SD.ChangeIndexOfS1(DStr.AddShape(HS1.Face()));
                SD.ChangeIndexOfS2(DStr.AddShape(HS2.Face()));
                decroch2 = false;
            }
            else if (obstacleon2)
            {
                TopAbs_Orientation Or1 = HS1.Face().Orientation();
                TopAbs_Orientation Or2 = HSref2.Face().Orientation();
                int Choix = ChFi3d.NextSide(ref Or1, ref Or2, Stripe.OrientationOnFace1(), Stripe.OrientationOnFace2(),
                                                                          Stripe.Choix());
                SoldepCS[3] = w2;
                SoldepCS[1] = pp1.X();
                SoldepCS[2] = pp1.Y();
                if (Simul)
                {
                    SimulSurf(SD, HGuide, Spine, Choix, HS1, It1, Or1, HS2, It2, HC2, HSref2, HCref2, ref decroch2, locfleche, tolesp,
                                        ref First, ref Last, Inside, Inside,
                                        forward, RecP2, RecS1, RecRst2, SoldepCS);
                }
                else
                {
                    PerformSurf(SeqSD, HGuide, Spine, Choix, HS1, It1, Or1, HS2, It2, HC2, HSref2, HCref2, ref decroch2, MaxStep,
                                            locfleche, tolesp, ref First, ref Last, Inside, Inside,
                                            forward, RecP2, RecS1, RecRst2, SoldepCS);
                }
                SD.ChangeIndexOfS1(DStr.AddShape(HS1.Face()));
                SD.ChangeIndexOfS2(DStr.AddShape(HS2.Face()));
                decroch1 = false;
            }
            else
            {
                Adaptor3d_TopolTool? aTT1 = It1;// to avoid ambiguity
                Adaptor3d_TopolTool? aTT2 = It2;// to avoid ambiguity
                CallPerformSurf(Stripe, Simul, SeqSD, SD, HGuide, Spine, HS1, HS3, pp1, pp3, aTT1, HS2, HS4, pp2, pp4, aTT2,
                                                MaxStep, locfleche, tolesp, ref First, ref Last, Inside, Inside,
                                                forward, RecS1, RecS2, Soldep, ref intf, ref intl, ref HS1, ref HS2);
                decroch1 = decroch2 = false;
            }

            if (!done)
            { // Case of fail
                if ((!Ok1 && !obstacleon1) || (!Ok2 && !obstacleon2))
                {
                    // Fail in a part of extension is not serious
                    // Here one stops.
                    done = true;
                    Inside = false;
                    if (forward)
                    {
                        intl = 1;
                    }
                    else
                    {
                        intf = 1;
                    }
                }
                else
                { // Otherwise invalidation of the stripe.
                    Spine.SetErrorStatus(ChFiDS_WalkingFailure);
                    throw new Standard_Failure("CallPerformSurf : Path failed!");
                }
            }

            else
            {
                refbis = @ref;
                if (forward)
                {
                    for (ii = 1; ii <= SeqSD.Length(); ii++)
                    {
                        SD = SeqSD[ii];
                        SD.ChangeIndexOfS1(DStr.AddShape(HS1.Face()));
                        if (obstacleon1)
                        {
                            SD.SetIndexOfC1(DStr.AddShape(HC1.Edge()));
                        }
                        SD.ChangeIndexOfS2(DStr.AddShape(HS2.Face()));
                        if (obstacleon2)
                        {
                            SD.SetIndexOfC2(DStr.AddShape(HC2.Edge()));
                        }
                        InsertAfter(Stripe, refbis, SD);
                        refbis = SD;
                    }
                }
                else
                {
                    for (ii = SeqSD.Length(); ii >= 1; ii--)
                    {
                        SD = SeqSD[ii];
                        SD.ChangeIndexOfS1(DStr.AddShape(HS1.Face()));
                        if (obstacleon1)
                        {
                            SD.SetIndexOfC1(DStr.AddShape(HC1.Edge()));
                        }
                        SD.ChangeIndexOfS2(DStr.AddShape(HS2.Face()));
                        if (obstacleon2)
                        {
                            SD.SetIndexOfC2(DStr.AddShape(HC2.Edge()));
                        }
                        InsertBefore(Stripe, refbis, SD);
                        refbis = SD;
                    }
                }

                if (!Ok1 && !obstacleon1)
                {
                    // clean infos on the plane of extension.
                    ChFi3d_Purge(Stripe, SD, @ref.Vertex(!forward, 1), !forward, 1, ref intf, ref intl);
                }

                if (!Ok2 && !obstacleon2)
                {
                    // clean infos on the plane of extension.
                    ChFi3d_Purge(Stripe, SD, @ref.Vertex(!forward, 2), !forward, 2, ref intf, ref intl);
                }

                // The end. The reference is changed.
                @ref = refbis;
            }

            if (Inside)
            { // There are starting solutions for the next.
                Inside = false;
                Firstsov = First;
                if (Guide.IsPeriodic())
                {
                    complete = false;
                    wf = Guide.FirstParameter();
                    wl = Guide.LastParameter();
                }
            }
            if (forward)
            {
                fini =
                    ((wl - Last) <= 10.0 * tolesp || (intl != 0 && !(obstacleon1 || obstacleon2))); // General case

                if (!fini && Guide.IsPeriodic() && ((wl - Last) < Guide.Period() * 1.0e-3))
                {
                    // It is tested if reframing of extremes is done at the same edge
                    // Loop Condition
                    ChFiDS_SurfData? thefirst = null, thelast = null;
                    thefirst = Stripe.SetOfSurfData().First();
                    thelast = Stripe.SetOfSurfData().Last();

                    if (thefirst.VertexFirstOnS1().IsOnArc() && thelast.VertexLastOnS1().IsOnArc())
                    {
                        fini = thefirst.VertexFirstOnS1().Arc().IsSame(thelast.VertexLastOnS1().Arc());
                    }
                    if (!fini && thefirst.VertexFirstOnS2().IsOnArc() && thelast.VertexLastOnS2().IsOnArc())
                    {
                        fini = thefirst.VertexFirstOnS2().Arc().IsSame(thelast.VertexLastOnS2().Arc());
                    }

                    if (fini)
                    {
                        return; // It is ended!
                    }
                }

                if (fini && complete)
                {
                    // restart in the opposite direction.
                    @ref = Stripe.SetOfSurfData().First();
                    forward = false;
                    fini = false;
                    First = Firstsov;
                }
                else
                {
                    First = Last;
                    Last = wl;
                }
            }
            if (!forward)
            {
                fini = ((First - wf) <= 10.0 * tolesp || (intf != 0 && !(obstacleon1 || obstacleon2)));
                complete = false;
                Last = wf;
            }
        }
        // The initial state is restored
        if (!Guide.IsPeriodic())
        {
            Guide.FirstParameter(wfsav);
            Guide.LastParameter(wlsav);
            if (OffsetHGuide is not null)
            {
                OffsetHGuide.FirstParameter(wfsav);
                OffsetHGuide.LastParameter(wlsav);
            }
        }
    }

    //=================================================================================================

    internal void PerformSetOfKPart(ChFiDS_Stripe? Stripe, bool Simul = false)
    {
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
        ref ChFiDS_Spine? Spine = ref Stripe.ChangeSpine();
        BRepAdaptor_Surface? HS1 = null, HS2 = null;
        TopAbs_Orientation Or1 = default, Or2 = default, RefOr1 = default, RefOr2 = default;
        int RefChoix = 0;

        // initialization of the stripe.
        Stripe.Reset();
        ref NCollection_HSequence<ChFiDS_SurfData>? HData = ref Stripe.ChangeSetOfSurfData();
        HData = new NCollection_HSequence<ChFiDS_SurfData>();
        NCollection_Sequence<ChFiDS_SurfData> SeqSurf = HData; // ChangeSequence(): the HSequence is the sequence

        StripeOrientations(Spine, ref RefOr1, ref RefOr2, ref RefChoix);
        Stripe.OrientationOnFace1(RefOr1);
        Stripe.OrientationOnFace2(RefOr2);
        Stripe.Choix(RefChoix);

        BRepTopAdaptor_TopolTool? It1 = new BRepTopAdaptor_TopolTool();
        BRepTopAdaptor_TopolTool? It2 = new BRepTopAdaptor_TopolTool();

        double WFirst = 0.0, WLast = 0.0;
        gp_Vec TFirst = new(), TLast = new(), TEndPeriodic = new();
        gp_Pnt PFirst = new(), PLast = new(), PEndPeriodic = new();
        bool intf = false, intl = false;

        // OCCT copies two default ElSpines into the handles: new default ones here
        ChFiDS_ElSpine? CurrentHE = new ChFiDS_ElSpine();
        ChFiDS_ElSpine? CurrentOffsetHE = new ChFiDS_ElSpine();
        Spine.D1(Spine.FirstParameter(), out PFirst, out TFirst);
        CurrentHE.FirstParameter(Spine.FirstParameter());
        CurrentHE.SetFirstPointAndTgt(PFirst, TFirst);
        CurrentOffsetHE.FirstParameter(Spine.FirstParameter());
        CurrentOffsetHE.SetFirstPointAndTgt(PFirst, TFirst);

        bool YaKPart = false;
        int iedgelastkpart = 0;

        double WStartPeriodic = 0.0;
        double WEndPeriodic = Spine.LastParameter(Spine.NbEdges());
        Spine.D1(WEndPeriodic, out PEndPeriodic, out TEndPeriodic);

        // Construction of particular cases.

        for (int iedge = 1; iedge <= Spine.NbEdges(); iedge++)
        {

            ConexFaces(Spine, iedge, ref HS1, ref HS2);

            if (ChFi3d_KParticular(Spine, iedge, HS1, HS2))
            {
                intf = ((iedge == 1) && !Spine.IsPeriodic());
                intl = ((iedge == Spine.NbEdges()) && !Spine.IsPeriodic());
                Or1 = HS1.Face().Orientation();
                Or2 = HS2.Face().Orientation();
                ChFi3d.NextSide(ref Or1, ref Or2, RefOr1, RefOr2, RefChoix);
                Adaptor3d_Surface? HSon1 = HS1;// to avoid ambiguity
                Adaptor3d_Surface? HSon2 = HS2;// to avoid ambiguity
                It1.Initialize(HSon1);
                It2.Initialize(HSon2);

                ChFiDS_SurfData? SD = new ChFiDS_SurfData();
                NCollection_Sequence<ChFiDS_SurfData> LSD = new();

                if (!ChFiKPart_ComputeData.Compute(DStr, SD, HS1, HS2, Or1, Or2, Spine, iedge))
                {
                }
                else if (!SplitKPart(SD, LSD, Spine, iedge, HS1, It1, HS2, It2, ref intf, ref intl))
                {
                    LSD.Clear();
                }
                else
                {
                    iedgelastkpart = iedge;
                }
                if (Spine.IsPeriodic())
                { // debug provisory for SD that arrive in disorder.
                    int nbsd = LSD.Length();
                    double period = Spine.Period();
                    double wfp = WStartPeriodic, wlp = WEndPeriodic;
                    //  modified by NIZHNY-EAP Thu Nov 25 12:57:53 1999 ___BEGIN___
                    if (!YaKPart && nbsd > 0)
                    {
                        //	if(!YaKPart){
                        //  modified by NIZHNY-EAP Thu Nov 25 12:57:57 1999 ___END___
                        ChFiDS_SurfData? firstSD = LSD.ChangeValue(1);
                        double wwf = firstSD.FirstSpineParam();
                        double wwl = firstSD.LastSpineParam();
                        wwf = ChFi3d_InPeriod(wwf, wfp, wlp, tolesp);
                        wwl = ChFi3d_InPeriod(wwl, wfp, wlp, tolesp);
                        if (wwl <= wwf + tolesp)
                        {
                            wwl += period;
                        }
                        wfp = wwf;
                        wlp = wfp + period;
                    }
                    for (int j = 1; j < nbsd; j++)
                    {
                        ChFiDS_SurfData? jSD = LSD.Value(j);
                        for (int k = j + 1; k <= nbsd; k++)
                        {
                            ChFiDS_SurfData? kSD = LSD.Value(k);
                            double jwf = jSD.FirstSpineParam();
                            jwf = ChFi3d_InPeriod(jwf, wfp, wlp, tolesp);
                            double kwf = kSD.FirstSpineParam();
                            kwf = ChFi3d_InPeriod(kwf, wfp, wlp, tolesp);
                            if (kwf < jwf)
                            {
                                LSD.SetValue(j, kSD);
                                LSD.SetValue(k, jSD);
                            }
                        }
                    }
                }
                NCollection_List<int> li = new();
                for (int j = 1; j <= LSD.Length(); j++)
                {
                    ref ChFiDS_SurfData? curSD = ref LSD.ChangeValue(j);
                    if (Simul)
                    {
                        SimulKPart(curSD);
                    }
                    SeqSurf.Append(curSD);
                    if (!Simul)
                    {
                        li.Append(curSD.Surf());
                    }
                    WFirst = LSD.Value(j).FirstSpineParam();
                    WLast = LSD.Value(j).LastSpineParam();
                    if (Spine.IsPeriodic())
                    {
                        WFirst = ChFi3d_InPeriod(WFirst, WStartPeriodic, WEndPeriodic, tolesp);
                        WLast = ChFi3d_InPeriod(WLast, WStartPeriodic, WEndPeriodic, tolesp);
                        if (WLast <= WFirst + tolesp)
                        {
                            WLast += Spine.Period();
                        }
                    }
                    TgtKP(LSD.Value(j), Spine, iedge, true, ref PFirst, ref TFirst);
                    TgtKP(LSD.Value(j), Spine, iedge, false, ref PLast, ref TLast);

                    // Determine the sections to approximate
                    if (!YaKPart)
                    {
                        if (Spine.IsPeriodic())
                        {
                            WStartPeriodic = WFirst;
                            WEndPeriodic = WStartPeriodic + Spine.Period();
                            WLast = ElCLib.InPeriod(WLast, WStartPeriodic, WEndPeriodic);
                            if (WLast <= WFirst + tolesp)
                            {
                                WLast += Spine.Period();
                            }
                            PEndPeriodic = PFirst;
                            TEndPeriodic = TFirst;
                            Spine.SetFirstParameter(WStartPeriodic);
                            Spine.SetLastParameter(WEndPeriodic);
                        }
                        else if (!intf || (iedge > 1))
                        {
                            // start section -> first KPart
                            // update of extension.
                            Spine.SetFirstTgt(Min(0.0, WFirst));
                            CurrentHE.LastParameter(WFirst);
                            CurrentHE.SetLastPointAndTgt(PFirst, TFirst);
                            Spine.AppendElSpine(CurrentHE);
                            CurrentHE.ChangeNext() = LSD.Value(j);
                            CurrentHE = new ChFiDS_ElSpine();

                            CurrentOffsetHE.LastParameter(WFirst);
                            CurrentOffsetHE.SetLastPointAndTgt(PFirst, TFirst);
                            Spine.AppendOffsetElSpine(CurrentOffsetHE);
                            CurrentOffsetHE.ChangeNext() = LSD.Value(j);
                            CurrentOffsetHE = new ChFiDS_ElSpine();
                        }
                        CurrentHE.FirstParameter(WLast);
                        CurrentHE.SetFirstPointAndTgt(PLast, TLast);
                        CurrentHE.ChangePrevious() = LSD.Value(j);
                        CurrentOffsetHE.FirstParameter(WLast);
                        CurrentOffsetHE.SetFirstPointAndTgt(PLast, TLast);
                        CurrentOffsetHE.ChangePrevious() = LSD.Value(j);
                        YaKPart = true;
                    }
                    else
                    {
                        if (WFirst - CurrentHE.FirstParameter() > tolesp)
                        {
                            // section between two KPart
                            CurrentHE.LastParameter(WFirst);
                            CurrentHE.SetLastPointAndTgt(PFirst, TFirst);
                            Spine.AppendElSpine(CurrentHE);
                            CurrentHE.ChangeNext() = LSD.Value(j);
                            CurrentHE = new ChFiDS_ElSpine();

                            CurrentOffsetHE.LastParameter(WFirst);
                            CurrentOffsetHE.SetLastPointAndTgt(PFirst, TFirst);
                            Spine.AppendOffsetElSpine(CurrentOffsetHE);
                            CurrentOffsetHE.ChangeNext() = LSD.Value(j);
                            CurrentOffsetHE = new ChFiDS_ElSpine();
                        }
                        CurrentHE.FirstParameter(WLast);
                        CurrentHE.SetFirstPointAndTgt(PLast, TLast);
                        CurrentHE.ChangePrevious() = LSD.Value(j);
                        CurrentOffsetHE.FirstParameter(WLast);
                        CurrentOffsetHE.SetFirstPointAndTgt(PLast, TLast);
                        CurrentOffsetHE.ChangePrevious() = LSD.Value(j);
                    }
                }
                if (!li.IsEmpty())
                {
                    myEVIMap.Bind(Spine.Edges(iedge), li);
                }
            }
        }

        if (!intl || (iedgelastkpart < Spine.NbEdges()))
        {
            // section last KPart(or start of the spine) -> End of the spine.
            // update of the extension.

            if (Spine.IsPeriodic())
            {
                if (WEndPeriodic - WLast > tolesp)
                {
                    CurrentHE.LastParameter(WEndPeriodic);
                    CurrentHE.SetLastPointAndTgt(PEndPeriodic, TEndPeriodic);
                    if (!YaKPart)
                    {
                        CurrentHE.SetPeriodic(true);
                    }
                    Spine.AppendElSpine(CurrentHE);

                    CurrentOffsetHE.LastParameter(WEndPeriodic);
                    CurrentOffsetHE.SetLastPointAndTgt(PEndPeriodic, TEndPeriodic);
                    if (!YaKPart)
                    {
                        CurrentOffsetHE.SetPeriodic(true);
                    }
                    Spine.AppendOffsetElSpine(CurrentOffsetHE);
                }
            }
            else
            {
                Spine.D1(Spine.LastParameter(), out PLast, out TLast);
                Spine.SetLastTgt(Max(Spine.LastParameter(Spine.NbEdges()), WLast));
                if (Spine.LastParameter() - WLast > tolesp)
                {
                    CurrentHE.LastParameter(Spine.LastParameter());
                    CurrentHE.SetLastPointAndTgt(PLast, TLast);
                    Spine.AppendElSpine(CurrentHE);

                    CurrentOffsetHE.LastParameter(Spine.LastParameter());
                    CurrentOffsetHE.SetLastPointAndTgt(PLast, TLast);
                    Spine.AppendOffsetElSpine(CurrentOffsetHE);
                }
            }
        }

        NCollection_List<ChFiDS_ElSpine> ll = Spine.ChangeElSpines();
        NCollection_List<ChFiDS_ElSpine>.Iterator ILES = new(ll);
        for (; ILES.More(); ILES.Next())
        {
            ChFi3d_PerformElSpine(ILES.ChangeValue(), Spine, myConti, tolesp);
        }
        if (Spine.Mode() == ChFiDS_ChamfMode.ChFiDS_ConstThroatWithPenetrationChamfer)
        {
            NCollection_List<ChFiDS_ElSpine> offsetll = Spine.ChangeOffsetElSpines();
            for (ILES.Initialize(offsetll); ILES.More(); ILES.Next())
            {
                ChFi3d_PerformElSpine(ILES.ChangeValue(), Spine, myConti, tolesp, true);
            }
        }
        Spine.SplitDone(true);
    }

    //=================================================================================================

    internal void PerformSetOfKGen(ChFiDS_Stripe? Stripe, bool Simul = false)
    {
        BRepTopAdaptor_TopolTool? It1 = new BRepTopAdaptor_TopolTool();
        BRepTopAdaptor_TopolTool? It2 = new BRepTopAdaptor_TopolTool();
        ref ChFiDS_Spine? Spine = ref Stripe.ChangeSpine();
        NCollection_List<ChFiDS_ElSpine> ll = Spine.ChangeElSpines();
        NCollection_List<ChFiDS_ElSpine>.Iterator ILES = new(ll);
        for (; ILES.More(); ILES.Next())
        {
            PerformSetOfSurfOnElSpine(ILES.Value(), Stripe, It1, It2, Simul);
        }
        if (!Simul)
        {
            TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();
            ref NCollection_HSequence<ChFiDS_SurfData>? HData = ref Stripe.ChangeSetOfSurfData();
            NCollection_Sequence<ChFiDS_SurfData> SeqSurf = HData; // ChangeSequence()
            int len = SeqSurf.Length();
            int last = len, i;
            bool periodic = Spine.IsPeriodic();
            if (periodic)
            {
                last++;
            }
            // It is attempted to reprocess the squares that bore.
            for (i = 1; i <= len; i++)
            {
                ref ChFiDS_SurfData? cursd = ref SeqSurf.ChangeValue(i);
                bool tw1 = cursd.TwistOnS1();
                bool tw2 = cursd.TwistOnS2();
                ChFiDS_SurfData? prevsd = null, nextsd = null;
                int iprev = i - 1;
                if (iprev == 0)
                {
                    if (periodic)
                    {
                        iprev = len;
                    }
                }
                int inext = i + 1;
                if (inext > len)
                {
                    if (periodic)
                    {
                        inext = 1;
                    }
                    else
                    {
                        inext = 0;
                    }
                }

                // For the moment only the surfaces where the twist is
                // detected at the path are corrected, it is necessary to control
                // more subtly the ugly traces (size, curvature, inflexion... )
                if (!tw1 && !tw2)
                {
                    continue;
                }

                // It is decided (fairly at random) if the extended surface is ready for the filling.
                ref ChFiDS_FaceInterference intf1 = ref cursd.ChangeInterferenceOnS1();
                ref ChFiDS_FaceInterference intf2 = ref cursd.ChangeInterferenceOnS2();
                int cursurf1 = cursd.IndexOfS1();
                int cursurf2 = cursd.IndexOfS2();
                ref ChFiDS_CommonPoint cpd1 = ref cursd.ChangeVertexFirstOnS1();
                ref ChFiDS_CommonPoint cpd2 = ref cursd.ChangeVertexFirstOnS2();
                ref ChFiDS_CommonPoint cpf1 = ref cursd.ChangeVertexLastOnS1();
                ref ChFiDS_CommonPoint cpf2 = ref cursd.ChangeVertexLastOnS2();
                gp_Pnt pd1 = cpd1.Point();
                gp_Pnt pd2 = cpd2.Point();
                gp_Pnt pf1 = cpf1.Point();
                gp_Pnt pf2 = cpf2.Point();
                double ddeb = pd1.Distance(pd2);
                double dfin = pf1.Distance(pf2);
                double don1 = pd1.Distance(pf1);
                double don2 = pd2.Distance(pf2);
                bool possibleon1 = (don1 < 2 * (ddeb + dfin));
                bool possibleon2 = (don2 < 2 * (ddeb + dfin));
                if ((tw1 && !possibleon1) || (tw2 && !possibleon2))
                {
                    Spine.SetErrorStatus(ChFiDS_TwistedSurface);
                    throw new Standard_Failure("adjustment by reprocessing the non-written points");
                }

                // It is checked if there are presentable neighbors
                bool yaprevon1 = false, yaprevon2 = false;
                bool samesurfon1 = false, samesurfon2 = false;
                if (iprev != 0)
                {
                    prevsd = SeqSurf.ChangeValue(iprev);
                    yaprevon1 = !prevsd.TwistOnS1();
                    samesurfon1 = (prevsd.IndexOfS1() == cursurf1);
                    yaprevon2 = !prevsd.TwistOnS2();
                    samesurfon2 = (prevsd.IndexOfS2() == cursurf2);
                }
                bool yanexton1 = false, yanexton2 = false;
                if (inext != 0)
                {
                    nextsd = SeqSurf.ChangeValue(inext);
                    yanexton1 = !nextsd.TwistOnS1();
                    if (samesurfon1)
                    {
                        samesurfon1 = (nextsd.IndexOfS1() == cursurf1);
                    }
                    yanexton2 = !nextsd.TwistOnS2();
                    if (samesurfon2)
                    {
                        samesurfon2 = (nextsd.IndexOfS2() == cursurf2);
                    }
                }
                // A contour of filling is constructed
                Geom2d_Curve? PC1 = intf1.PCurveOnFace();
                Geom2d_Curve? PC2 = intf2.PCurveOnFace();
                BRepAdaptor_Surface? S1 = new BRepAdaptor_Surface();
                TopoDS_Face F1 = TopoDS.Face(DStr.Shape(cursurf1));
                S1.Initialize(F1);
                BRepAdaptor_Surface? S2 = new BRepAdaptor_Surface();
                TopoDS_Face F2 = TopoDS.Face(DStr.Shape(cursurf2));
                S2.Initialize(F2);
                GeomFill_Boundary? Bdeb = null, Bfin = null, Bon1 = null, Bon2 = null;
                bool pointuon1 = false, pointuon2 = false;
                if (tw1)
                {
                    if (!yaprevon1 || !yanexton1)
                    {
                        Spine.SetErrorStatus(ChFiDS_TwistedSurface);
                        throw new Standard_Failure("adjustment by reprocessing the non-written points: no neighbor");
                    }
                    ref ChFiDS_FaceInterference previntf1_ = ref prevsd.ChangeInterferenceOnS1();
                    ref ChFiDS_FaceInterference nextintf1_ = ref nextsd.ChangeInterferenceOnS1();
                    double prevpar1_ = previntf1_.LastParameter();
                    double nextpar1_ = nextintf1_.FirstParameter();
                    if (samesurfon1)
                    {
                        // It is checked if it is possible to intersect traces of neighbors
                        // to create a sharp end.
                        Geom2d_Curve? pcprev1 = previntf1_.PCurveOnFace();
                        Geom2d_Curve? pcnext1 = nextintf1_.PCurveOnFace();
                        double nprevpar1 = 0.0, nnextpar1 = 0.0;
                        gp_Pnt2d p2d = new();
                        //  Modified by Sergey KHROMOV - Wed Feb  5 12:03:17 2003 Begin
                        // 	  if(ChFi3d_IntTraces(prevsd,prevpar1_,ref nprevpar1,1,1,
                        // 			      nextsd,nextpar1_,ref nnextpar1,1,-1,p2d)){
                        if (ChFi3d_IntTraces(prevsd, prevpar1_, ref nprevpar1, 1, 1, nextsd, nextpar1_, ref nnextpar1, 1, -1, p2d,
                                                                  false, true))
                        {
                            //  Modified by Sergey KHROMOV - Wed Feb  5 12:03:17 2003 End
                            previntf1_.SetLastParameter(nprevpar1);
                            nextintf1_.SetFirstParameter(nnextpar1);
                            pointuon1 = true;
                            PC1 = null;
                        }
                        else
                        {
                            gp_Pnt2d pdeb1 = new(), pfin1 = new();
                            gp_Vec2d vdeb1 = new(), vfin1 = new();
                            pcprev1.D1(prevpar1_, out pdeb1, out vdeb1);
                            pcnext1.D1(nextpar1_, out pfin1, out vfin1);
                            Bon1 = ChFi3d_mkbound(S1, out PC1, -1, pdeb1, vdeb1, 1, pfin1, vfin1, tolapp3d, 2.0e-4);
                        }
                    }
                    else
                    {
                        // here the base is on 3D tangents of neighbors.
                        Geom_Curve? c3dprev1 = DStr.Curve(previntf1_.LineIndex()).Curve();
                        Geom_Curve? c3dnext1 = DStr.Curve(nextintf1_.LineIndex()).Curve();
                        gp_Pnt Pdeb1 = new(), Pfin1 = new();
                        gp_Vec Vdeb1 = new(), Vfin1 = new();
                        c3dprev1.D1(prevpar1_, out Pdeb1, out Vdeb1);
                        c3dnext1.D1(nextpar1_, out Pfin1, out Vfin1);
                        gp_Pnt2d pdeb1 = new(), pfin1 = new();
                        double pardeb1 = intf1.FirstParameter();
                        double parfin1 = intf1.LastParameter();
                        pdeb1 = PC1.Value(pardeb1);
                        pfin1 = PC1.Value(parfin1);
                        Bon1 = ChFi3d_mkbound(S1, out PC1, -1, pdeb1, ref Vdeb1, 1, pfin1, ref Vfin1, tolapp3d, 2.0e-4);
                    }
                }
                else
                {
                    Bon1 = ChFi3d_mkbound(S1, PC1, tolapp3d, 2.0e-4);
                }
                if (tw2)
                {
                    if (!yaprevon2 || !yanexton2)
                    {
                        throw new Standard_Failure("adjustment by reprocessing the non-written points: no neighbor");
                    }
                    ref ChFiDS_FaceInterference previntf2_ = ref prevsd.ChangeInterferenceOnS2();
                    ref ChFiDS_FaceInterference nextintf2_ = ref nextsd.ChangeInterferenceOnS2();
                    double prevpar2_ = previntf2_.LastParameter();
                    double nextpar2_ = nextintf2_.FirstParameter();
                    if (samesurfon2)
                    {
                        // It is checked if it is possible to intersect traces of neighbors
                        // to create a sharp end.
                        Geom2d_Curve? pcprev2 = previntf2_.PCurveOnFace();
                        Geom2d_Curve? pcnext2 = nextintf2_.PCurveOnFace();
                        double nprevpar2 = 0.0, nnextpar2 = 0.0;
                        gp_Pnt2d p2d = new();
                        //  Modified by Sergey KHROMOV - Wed Feb  5 12:03:17 2003 Begin
                        // 	  if(ChFi3d_IntTraces(prevsd,prevpar2_,ref nprevpar2,2,1,
                        // 			      nextsd,nextpar2_,ref nnextpar2,2,-1,p2d)){
                        if (ChFi3d_IntTraces(prevsd, prevpar2_, ref nprevpar2, 2, 1, nextsd, nextpar2_, ref nnextpar2, 2, -1, p2d,
                                                                  false, true))
                        {
                            //  Modified by Sergey KHROMOV - Wed Feb  5 12:03:17 2003 End
                            previntf2_.SetLastParameter(nprevpar2);
                            nextintf2_.SetFirstParameter(nnextpar2);
                            pointuon2 = true;
                            PC2 = null;
                        }
                        else
                        {
                            gp_Pnt2d pdeb2 = new(), pfin2 = new();
                            gp_Vec2d vdeb2 = new(), vfin2 = new();
                            pcprev2.D1(prevpar2_, out pdeb2, out vdeb2);
                            pcnext2.D1(nextpar2_, out pfin2, out vfin2);
                            Bon2 = ChFi3d_mkbound(S2, out PC2, -1, pdeb2, vdeb2, 1, pfin2, vfin2, tolapp3d, 2.0e-4);
                        }
                    }
                    else
                    {
                        // here the base is on 3D tangents of neighbors.
                        Geom_Curve? c3dprev2 = DStr.Curve(previntf2_.LineIndex()).Curve();
                        Geom_Curve? c3dnext2 = DStr.Curve(nextintf2_.LineIndex()).Curve();
                        gp_Pnt Pdeb2 = new(), Pfin2 = new();
                        gp_Vec Vdeb2 = new(), Vfin2 = new();
                        c3dprev2.D1(prevpar2_, out Pdeb2, out Vdeb2);
                        c3dnext2.D1(nextpar2_, out Pfin2, out Vfin2);
                        gp_Pnt2d pdeb2 = new(), pfin2 = new();
                        double pardeb2 = intf2.FirstParameter();
                        double parfin2 = intf2.LastParameter();
                        pdeb2 = PC2.Value(pardeb2);
                        pfin2 = PC2.Value(parfin2);
                        Bon2 = ChFi3d_mkbound(S2, out PC2, -1, pdeb2, ref Vdeb2, 1, pfin2, ref Vfin2, tolapp3d, 2.0e-4);
                    }
                }
                else
                {
                    Bon2 = ChFi3d_mkbound(S2, PC2, tolapp3d, 2.0e-4);
                }
                // The parameters of neighbor traces are updated, so
                // straight lines uv are pulled.
                Geom_Surface? sprev = DStr.Surface(prevsd.Surf()).Surface();
                Geom_Surface? snext = DStr.Surface(nextsd.Surf()).Surface();
                ref ChFiDS_FaceInterference previntf1 = ref prevsd.ChangeInterferenceOnS1();
                ref ChFiDS_FaceInterference nextintf1 = ref nextsd.ChangeInterferenceOnS1();
                ref ChFiDS_FaceInterference previntf2 = ref prevsd.ChangeInterferenceOnS2();
                ref ChFiDS_FaceInterference nextintf2 = ref nextsd.ChangeInterferenceOnS2();
                Geom2d_Curve? pcsprev1 = previntf1.PCurveOnSurf();
                Geom2d_Curve? pcsnext1 = nextintf1.PCurveOnSurf();
                double prevpar1 = previntf1.LastParameter();
                double nextpar1 = nextintf1.FirstParameter();
                Geom2d_Curve? pcsprev2 = previntf2.PCurveOnSurf();
                Geom2d_Curve? pcsnext2 = nextintf2.PCurveOnSurf();
                double prevpar2 = previntf2.LastParameter();
                double nextpar2 = nextintf2.FirstParameter();
                gp_Pnt2d pdebs1 = pcsprev1.Value(prevpar1);
                gp_Pnt2d pdebs2 = pcsprev2.Value(prevpar2);
                gp_Pnt2d pfins1 = pcsnext1.Value(nextpar1);
                gp_Pnt2d pfins2 = pcsnext2.Value(nextpar2);
                Bdeb = ChFi3d_mkbound(sprev, pdebs1, pdebs2, tolapp3d, 2.0e-4);
                Bfin = ChFi3d_mkbound(snext, pfins1, pfins2, tolapp3d, 2.0e-4);

                GeomFill_ConstrainedFilling fil = new(11, 20);
                if (pointuon1)
                {
                    fil.Init(Bon2, Bfin, Bdeb, true);
                }
                else if (pointuon2)
                {
                    fil.Init(Bon1, Bfin, Bdeb, true);
                }
                else
                {
                    fil.Init(Bon1, Bfin, Bon2, Bdeb, true);
                }

                ChFi3d_ReparamPcurv(0.0, 1.0, ref PC1);
                ChFi3d_ReparamPcurv(0.0, 1.0, ref PC2);
                Geom_Surface? newsurf = fil.InternalSurface(); // unfrozen: VReverse below (decision 1122)
                if (pointuon1)
                {
                    newsurf.VReverse(); // we return to direction 1 from  2;
                    done = CompleteData(cursd, newsurf, S1, PC1, S2, PC2, F2.Orientation(), false, false, false, false, false);
                    cursd.ChangeIndexOfS1(0);
                }
                else
                {
                    done = CompleteData(cursd, newsurf, S1, PC1, S2, PC2, F1.Orientation(), true, false, false, false, false);
                    if (pointuon2)
                    {
                        cursd.ChangeIndexOfS2(0);
                    }
                }
                if (tw1)
                {
                    prevsd.ChangeVertexLastOnS1().SetPoint(cpd1.Point());
                    nextsd.ChangeVertexFirstOnS1().SetPoint(cpf1.Point());
                }
                if (tw2)
                {
                    prevsd.ChangeVertexLastOnS2().SetPoint(cpd2.Point());
                    nextsd.ChangeVertexFirstOnS2().SetPoint(cpf2.Point());
                }
            }
            // The tolerance of points is updated.
            for (i = 1; i < last; i++)
            {
                int j = i % len + 1;
                int curs1 = 0, curs2 = 0;
                int nexts1 = 0, nexts2 = 0;
                ref ChFiDS_SurfData? cursd = ref SeqSurf.ChangeValue(i);
                ref ChFiDS_SurfData? nextsd = ref SeqSurf.ChangeValue(j);
                ref ChFiDS_CommonPoint curp1 = ref cursd.ChangeVertexLastOnS1();
                ref ChFiDS_CommonPoint nextp1 = ref nextsd.ChangeVertexFirstOnS1();
                if (cursd.IsOnCurve1())
                {
                    curs1 = cursd.IndexOfC1();
                }
                else
                {
                    curs1 = cursd.IndexOfS1();
                }
                if (cursd.IsOnCurve2())
                {
                    curs2 = cursd.IndexOfC2();
                }
                else
                {
                    curs2 = cursd.IndexOfS2();
                }
                double tol1 = Max(curp1.Tolerance(), nextp1.Tolerance());
                ref ChFiDS_CommonPoint curp2 = ref cursd.ChangeVertexLastOnS2();
                ref ChFiDS_CommonPoint nextp2 = ref nextsd.ChangeVertexFirstOnS2();
                double tol2 = Max(curp2.Tolerance(), nextp2.Tolerance());
                if (nextsd.IsOnCurve1())
                {
                    nexts1 = nextsd.IndexOfC1();
                }
                else
                {
                    nexts1 = nextsd.IndexOfS1();
                }
                if (nextsd.IsOnCurve2())
                {
                    nexts2 = nextsd.IndexOfC2();
                }
                else
                {
                    nexts2 = nextsd.IndexOfS2();
                }

                if (!curp1.IsOnArc() && nextp1.IsOnArc())
                {
                    curp1 = nextp1;
                    if ((curs1 == nexts1) && !nextsd.IsOnCurve1())
                    {
                        // Case when it is not possible to pass along the border without leaving
                        ChangeTransition(nextp1, ref curp1, nexts1, myDS);
                    }
                }
                else if (curp1.IsOnArc() && !nextp1.IsOnArc())
                {
                    nextp1 = curp1;
                    if ((curs1 == nexts1) && !cursd.IsOnCurve1())
                    {
                        ChangeTransition(curp1, ref nextp1, curs1, myDS);
                    }
                }

                if (!curp2.IsOnArc() && nextp2.IsOnArc())
                {
                    curp2 = nextp2;
                    if ((curs2 == nexts2) && !nextsd.IsOnCurve2())
                    {
                        ChangeTransition(nextp2, ref curp2, curs2, myDS);
                    }
                }
                else if (curp2.IsOnArc() && !nextp2.IsOnArc())
                {
                    nextp2 = curp2;
                    if ((curs2 == nexts2) && !cursd.IsOnCurve2())
                    {
                        ChangeTransition(curp2, ref nextp2, curs2, myDS);
                    }
                }

                curp1.SetTolerance(tol1);
                nextp1.SetTolerance(tol1);
                curp2.SetTolerance(tol2);
                nextp2.SetTolerance(tol2);

                Bnd_Box b1 = new(), b2 = new();
                if (curp1.IsOnArc())
                {
                    ChFi3d_EnlargeBox(curp1.Arc(), myEFMap[curp1.Arc()], curp1.ParameterOnArc(), ref b1);
                }
                if (curp2.IsOnArc())
                {
                    ChFi3d_EnlargeBox(curp2.Arc(), myEFMap[curp2.Arc()], curp2.ParameterOnArc(), ref b2);
                }
                ChFiDS_Stripe? bidst = null;
                ChFi3d_EnlargeBox(DStr, bidst, cursd, ref b1, ref b2, false);
                ChFi3d_EnlargeBox(DStr, bidst, nextsd, ref b1, ref b2, true);
                tol1 = ChFi3d_BoxDiag(b1);
                tol2 = ChFi3d_BoxDiag(b2);
                curp1.SetTolerance(tol1);
                nextp1.SetTolerance(tol1);
                curp2.SetTolerance(tol2);
                nextp2.SetTolerance(tol2);
            }
            // The connections edge/new faces are updated.
            for (ILES.Initialize(ll); ILES.More(); ILES.Next())
            {
                ChFiDS_ElSpine? curhels = ILES.Value();
                double WF = curhels.FirstParameter();
                double WL = curhels.LastParameter();
                int IF = 0, IL = 0;
                double nwf = WF, nwl = WL;
                double period = 0.0;
                int nbed = Spine.NbEdges();
                if (periodic)
                {
                    period = Spine.Period();
                    nwf = ElCLib.InPeriod(WF, -tolesp, period - tolesp);
                    IF = Spine.Index(nwf, true);
                    nwl = ElCLib.InPeriod(WL, tolesp, period + tolesp);
                    IL = Spine.Index(nwl, false);
                    if (nwl < nwf + tolesp)
                    {
                        IL += nbed;
                    }
                }
                else
                {
                    IF = Spine.Index(WF, true);
                    IL = Spine.Index(WL, false);
                }
                if (IF == IL)
                {
                    // fast processing
                    int IFloc = IF;
                    if (periodic)
                    {
                        IFloc = (IF - 1) % nbed + 1;
                    }
                    TopoDS_Edge Ej = Spine.Edges(IFloc);
                    for (i = 1; i <= len; i++)
                    {
                        ref ChFiDS_SurfData? cursd = ref SeqSurf.ChangeValue(i);
                        double fp = cursd.FirstSpineParam();
                        double lp = cursd.LastSpineParam();
                        if (lp < WF + tolesp || fp > WL - tolesp)
                        {
                            continue;
                        }
                        if (!myEVIMap.IsBound(Ej))
                        {
                            NCollection_List<int> li = new();
                            myEVIMap.Bind(Ej, li);
                        }
                        myEVIMap.ChangeFind(Ej).Append(cursd.Surf());
                    }
                }
                else if (IF < IL)
                {
                    NCollection_Array1<double> wv = new(IF, IL - 1);
                    for (i = IF; i < IL; i++)
                    {
                        int iloc = i;
                        if (periodic)
                        {
                            iloc = (i - 1) % nbed + 1;
                        }
                        double wi = Spine.LastParameter(iloc);
                        if (periodic)
                        {
                            wi = ElCLib.InPeriod(wi, WF, WF + period);
                        }
                        gp_Pnt pv = Spine.Value(wi);
                        Extrema_LocateExtPC ext = new(pv, curhels, wi, 1.0e-8);
                        wv[i] = wi;
                        if (ext.IsDone())
                        {
                            wv[i] = ext.Point().Parameter();
                        }
                    }
                    for (i = 1; i <= len; i++)
                    {
                        ref ChFiDS_SurfData? cursd = ref SeqSurf.ChangeValue(i);
                        double fp = cursd.FirstSpineParam();
                        double lp = cursd.LastSpineParam();
                        int j = 0;
                        int jf = 0, jl = 0;
                        if (lp < WF + tolesp || fp > WL - tolesp)
                        {
                            continue;
                        }
                        for (j = IF; j < IL; j++)
                        {
                            jf = j;
                            if (fp < wv[j] - tolesp)
                            {
                                break;
                            }
                        }
                        for (j = IF; j < IL; j++)
                        {
                            jl = j;
                            if (lp < wv[j] + tolesp)
                            {
                                break;
                            }
                        }
                        for (j = jf; j <= jl; j++)
                        {
                            int jloc = j;
                            if (periodic)
                            {
                                jloc = (j - 1) % nbed + 1;
                            }
                            TopoDS_Edge Ej = Spine.Edges(jloc);
                            if (!myEVIMap.IsBound(Ej))
                            {
                                NCollection_List<int> li = new();
                                myEVIMap.Bind(Ej, li);
                            }
                            myEVIMap.ChangeFind(Ej).Append(cursd.Surf());
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    internal void PerformSetOfSurf(ChFiDS_Stripe? Stripe, bool Simul = false)
    {
        TopOpeBRepDS_DataStructure DStr = myDS.ChangeDS();

        ChFiDS_Spine? sp = Stripe.Spine();
        int SI = ChFi3d_SolidIndex(sp, DStr, myESoMap, myEShMap);
        Stripe.SetSolidIndex(SI);
        if (!sp.SplitDone())
        {
            PerformSetOfKPart(Stripe, Simul);
        }

        PerformSetOfKGen(Stripe, Simul);

        if (!Simul)
        {
            ChFi3d_MakeExtremities(Stripe, DStr, myEFMap, tolapp3d, tol2d);
        }
    }

}

internal static class ChFi3d_Builder_2
{
    internal static Geom2d_Curve? getCurveOnSurface(in TopoDS_Edge theEdge, in TopoDS_Face theFace)
    {
        if (theEdge.IsNull() || theFace.IsNull())
        {
            return null;
        }

        double aFirstParam = 0.0;
        double aLastParam = 0.0;
        return BRep_Tool.CurveOnSurface(theEdge, theFace, out aFirstParam, out aLastParam);
    }

    internal static void ChFi3d_CoupeParPlan(in ChFiDS_CommonPoint compoint1, in ChFiDS_CommonPoint compoint2, GeomAdaptor_Surface? HConge, in gp_Pnt2d UV1, in gp_Pnt2d UV2, double tol3d, double tol2d, ref Geom_Curve? C3d, ref Geom2d_Curve? pcurve, ref double tolreached, ref double Pardeb, ref double Parfin, ref bool plane)
    {
        plane = true;
        if (compoint1.IsOnArc() && compoint2.IsOnArc())
        {
            gp_Pnt P1 = new(), P2 = new();
            BRepAdaptor_Curve BCurv1 = new(compoint1.Arc());
            BRepAdaptor_Curve BCurv2 = new(compoint2.Arc());
            double parE1 = 0.0, parE2 = 0.0;
            parE1 = compoint1.ParameterOnArc();
            parE2 = compoint2.ParameterOnArc();
            gp_Vec t1 = new(), t2 = new();
            BCurv1.D1(parE1, out P1, out t1);
            BCurv2.D1(parE2, out P2, out t2);
            gp_Dir tgt1 = new(t1);
            gp_Dir tgt2 = new(t2);
            gp_Vec v12 = new(P2.X() - P1.X(), P2.Y() - P1.Y(), P2.Z() - P1.Z());
            gp_Dir d12 = new(v12);
            gp_Dir nor = tgt1.Crossed(d12);
            Geom_Plane? Plan = new Geom_Plane(P1, nor);
            double scal = 0.0;
            scal = Math.Abs(nor.Dot(tgt2));
            if (scal < 0.01)
            {
                GeomAdaptor_Surface? HPlan = new GeomAdaptor_Surface(Plan);
                Geom2d_Curve? C2dint2 = null;
                NCollection_Array1<double> Pdeb = new(1, 4), Pfin = new(1, 4);
                GeomAdaptor_Surface AS = new(Plan);
                Extrema_ExtPS anExtPS = new(P1, AS, AS.FirstUParameter(), AS.LastUParameter(), AS.FirstVParameter(),
                                            AS.LastVParameter(), 1.0e-3, 1.0e-3, Extrema_ExtFlag.Extrema_ExtFlag_MIN);
                double u1 = 0.0, v1 = 0.0;
                anExtPS.Point(1).Parameter(out u1, out v1);
                Pdeb[1] = UV1.X();
                Pdeb[2] = UV1.Y();
                Pdeb[3] = u1;
                Pdeb[4] = v1;
                anExtPS.Perform(P2);
                anExtPS.Point(1).Parameter(out u1, out v1);
                Pfin[1] = UV2.X();
                Pfin[2] = UV2.Y();
                Pfin[3] = u1;
                Pfin[4] = v1;
                if (ChFi3d_ComputeCurves(HConge, HPlan, Pdeb, Pfin, ref C3d, ref pcurve, ref C2dint2, tol3d, tol2d,
                                                                  out tolreached))
                {
                    Pardeb = C3d.FirstParameter();
                    Parfin = C3d.LastParameter();
                }
                else
                {
                    plane = false;
                }
            }
            else
            {
                plane = false;
            }
        }
        else
        {
            plane = false;
        }
    }

    internal static bool isTangentToArc(in ChFiDS_CommonPoint aCommonPoint, double anAngularTolerance)
    {
        if (!aCommonPoint.HasVector())
        {
            return false;
        }

        double aDummyParam1 = 0.0, aDummyParam2 = 0.0;// Not used.
        Geom_Curve? anArcCurve = 
            BRep_Tool.Curve(aCommonPoint.Arc(), out aDummyParam1, out aDummyParam2);

        gp_Pnt aDummyPoint = new(); // Not used.
        gp_Vec anArcTangent = new(); // Tangent to the arc at the common point.
        anArcCurve.D1(aCommonPoint.ParameterOnArc(), out aDummyPoint, out anArcTangent);

        gp_Vec aCommonPointVector = aCommonPoint.Vector();
        return aCommonPointVector.IsParallel(anArcTangent, anAngularTolerance);
    }

    internal static bool BonVoisin(in gp_Pnt Point, BRepAdaptor_Surface? HS, ref TopoDS_Face F, GeomAdaptor_Surface? plane, in TopoDS_Edge cured, ref double XDep, ref double YDep, ChFiDS_Map EFMap, double tol3d)
    {
        bool bonvoisin = true;
        double winter = 0.0;
        gp_Pnt papp = HS.Value(XDep, YDep);
        double dist = RealLast();
        BRepAdaptor_Curve? hc = new BRepAdaptor_Curve();
        Geom2d_Curve? PC = null;
        bool found = false;

        TopExp_Explorer Ex = new();
        for (Ex.Init(F, TopAbs_EDGE); Ex.More(); Ex.Next())
        {
            TopoDS_Edge ecur = TopoDS.Edge(Ex.Current());
            if (!ecur.IsSame(cured))
            {
                hc.Initialize(ecur);
                double tolc = hc.Resolution(tol3d);
                if (ChFi3d_InterPlaneEdge(plane, hc, ref winter, true, tolc))
                {
                    gp_Pnt np = hc.Value(winter);
                    double ndist = np.SquareDistance(papp);
                    if (ndist < dist)
                    {
                        NCollection_List<TopoDS_Shape>.Iterator It = new();
                        TopoDS_Face ff = new();
                        bool isclosed = BRep_Tool.IsClosed(ecur, F);
                        bool isreallyclosed = BRepTools.IsReallyClosed(ecur, F);
                        for (It.Initialize(EFMap[ecur]); It.More(); It.Next())
                        {
                            ff = TopoDS.Face(It.Value());
                            bool issame = ff.IsSame(F);
                            //  Modified by Sergey KHROMOV - Fri Dec 21 17:12:48 2001 Begin
                            // 	    bool istg =
                            // 	      BRep_Tool::Continuity(ecur,ff,F) != GeomAbs_C0;
                            bool istg = ChFi3d.IsTangentFaces(ecur, ff, F);
                            //  Modified by Sergey KHROMOV - Fri Dec 21 17:12:51 2001 End
                            if ((!issame || (issame && isreallyclosed)) && istg)
                            {
                                found = true;
                                TopoDS_Edge newe = ecur;
                                newe.Orientation(TopAbs_FORWARD);
                                dist = ndist;
                                HS.Initialize(ff);
                                if (isclosed && !isreallyclosed)
                                {
                                    TopoDS_Face fff = ff;
                                    fff.Orientation(TopAbs_FORWARD);
                                    TopExp_Explorer Ex2 = new();
                                    for (Ex2.Init(fff, TopAbs_EDGE); Ex2.More(); Ex2.Next())
                                    {
                                        if (newe.IsSame(Ex2.Current()))
                                        {
                                            newe = TopoDS.Edge(Ex2.Current());
                                            PC = getCurveOnSurface(newe, fff);
                                            break;
                                        }
                                    }
                                }
                                else
                                {
                                    PC = getCurveOnSurface(newe, ff);
                                }
                                PC.Value(winter).Coord(out XDep, out YDep);
                                if (issame)
                                {
                                    gp_Pnt spt = new();
                                    gp_Vec sdu = new(), sdv = new(), nors = new();
                                    HS.D1(XDep, YDep, out spt, out sdu, out sdv);
                                    nors = sdu.Crossed(sdv);
                                    gp_Pnt cpt = new();
                                    gp_Vec cd = new();
                                    hc.D1(winter, out cpt, out cd);
                                    gp_Vec vref = new(Point, cpt);
                                    TopoDS_Face fff = ff;
                                    fff.Orientation(TopAbs_FORWARD);
                                    if (vref.Dot(nors.Crossed(cd)) < 0.0)
                                    {
                                        newe.Orientation(TopAbs_REVERSED);
                                    }
                                    PC = getCurveOnSurface(newe, fff);
                                    PC.Value(winter).Coord(out XDep, out YDep);
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
        if (!found)
        {
            bonvoisin = false;
        }
        return bonvoisin;
    }

    internal static bool Projection(Extrema_ExtPC PExt, in gp_Pnt P, Adaptor3d_Curve C, ref double W, double Tol)
    {
        double Dist2 = 0.0, daux2 = 0.0;
        Dist2 = C.Value(W).SquareDistance(P);

        // It is checked if it is not already a solution
        if (Dist2 < Tol * Tol)
        {
            return true;
        }

        bool Ok = false;

        // On essai une resolution initialise
        Extrema_LocateExtPC ext = new(P, C, W, Tol / 10);
        if (ext.IsDone())
        {
            daux2 = C.Value(ext.Point().Parameter()).SquareDistance(P);
            if (daux2 < Dist2)
            {
                W = ext.Point().Parameter();
                Dist2 = daux2;
                Ok = true;
                if (Dist2 < Tol * Tol)
                {
                    return true;
                }
            }
        }

        // Global resolution
        PExt.Perform(P);
        if (PExt.IsDone())
        {
            for (int ii = 1; ii <= PExt.NbExt(); ii++)
            {
                if (PExt.SquareDistance(ii) < Dist2)
                {
                    Dist2 = PExt.SquareDistance(ii);
                    W = PExt.Point(ii).Parameter();
                    Ok = true;
                }
            }
        }
        return Ok;
    }

    internal static void TgtKP(ChFiDS_SurfData? CD, ChFiDS_Spine? Spine, int iedge, bool isfirst, ref gp_Pnt ped, ref gp_Vec ded)
    {
        double wtg = CD.InterferenceOnS1().Parameter(isfirst);
        BRepAdaptor_Curve bc = Spine.CurrentElementarySpine(iedge);
        if (Spine.Edges(iedge).Orientation() == TopAbs_FORWARD)
        {
            bc.D1(wtg + bc.FirstParameter(), out ped, out ded);
        }
        else
        {
            bc.D1(-wtg + bc.LastParameter(), out ped, out ded);
            ded.Reverse();
        }
        ded.Normalize();
    }

    internal static bool IsInput(in gp_Vec Vec, in TopoDS_Vertex Ve, in TopoDS_Face Fa)
    {
        TopExp_Explorer FaceExp = new(Fa, TopAbs_WIRE);
        BRepTools_WireExplorer WireExp = new();
        int Trouve = 0;
        TopoDS_Wire W = new();
        TopoDS_Edge E = new();
        TopoDS_Vertex Vf = new(), Vl = new();
        gp_Vec[] Vec3d = new gp_Vec[2];
        gp_Pnt Point = new();

        // Find edges and compute 3D vectors
        for (; (FaceExp.More() && (Trouve < 2)); FaceExp.Next())
        {
            W = TopoDS.Wire(FaceExp.Current());
            for (Trouve = 0, WireExp.Init(W); WireExp.More() && (Trouve < 2); WireExp.Next())
            {
                E = TopoDS.Edge(WireExp.Current());
                TopExp.Vertices(E, out Vf, out Vl);
                if (Vf.IsSame(Ve))
                {
                    BRepAdaptor_Curve Cb = new(E);
                    Cb.D1(BRep_Tool.Parameter(Ve, E), out Point, out Vec3d[Trouve]);
                    Trouve++;
                }
                else if (Vl.IsSame(Ve))
                {
                    BRepAdaptor_Curve Cb = new(E);
                    Cb.D1(BRep_Tool.Parameter(Ve, E), out Point, out Vec3d[Trouve]);
                    Vec3d[Trouve].Reverse();
                    Trouve++;
                }
            }
        }
        if (Trouve < 2)
        {
            return false;
        }
        // Calculate the normal and the angles in the associated vector plane
        gp_Vec Normal = new();
        Normal = Vec3d[0] ^ Vec3d[1];
        if (Normal.SquareMagnitude() < Precision.Confusion())
        { // Colinear case
            return (Vec.IsParallel(Vec3d[0], Precision.Confusion()));
        }

        double amin = 0.0, amax = 0.0;
        amax = Vec3d[1].AngleWithRef(Vec3d[0], Normal);
        if (amax < 0)
        {
            amin = amax;
            amax = 0;
        }
        else
        {
            amin = 0;
        }

        // Projection of the vector
        gp_Ax3 Axe = new(Point, new gp_Dir(Normal), new gp_Dir(Vec3d[0]));
        gp_Trsf Transf = new();
        Transf.SetTransformation(Axe);
        gp_XYZ coord = Vec.XYZ();
        Transf.Transforms(ref coord);
        coord.SetZ(0);
        Transf.Invert();
        Transf.Transforms(ref coord);
        gp_Vec theProj = new(coord);

        // and finally...
        double Angle = theProj.AngleWithRef(Vec3d[0], Normal);
        return ((Angle >= amin) && (Angle <= amax));
    }

    internal static bool IsG1(ChFiDS_Map TheMap, in TopoDS_Edge E, in TopoDS_Face FRef, ref TopoDS_Face FVoi)
    {
        NCollection_List<TopoDS_Shape>.Iterator It = new();
        // Find a neighbor of E different from FRef (general case).
        for (It.Initialize(TheMap[E]); It.More(); It.Next())
        {
            if (!TopoDS.Face(It.Value()).IsSame(FRef))
            {
                FVoi = TopoDS.Face(It.Value());
                //  Modified by Sergey KHROMOV - Fri Dec 21 17:09:32 2001 Begin
                //    if (BRep_Tool::Continuity(E,FRef,FVoi) != GeomAbs_C0) {
                if (ChFi3d.IsTangentFaces(E, FRef, FVoi))
                {
                    //  Modified by Sergey KHROMOV - Fri Dec 21 17:09:33 2001 End
                    return true;
                }
            }
        }
        // If is was not found it is checked if E is a cutting edge,
        // in which case FVoi = FRef is returned (less frequent case).
        TopExp_Explorer Ex = new();
        bool orset = false;
        TopAbs_Orientation orient = TopAbs_FORWARD;
        TopoDS_Edge ed = new();
        for (Ex.Init(FRef, TopAbs_EDGE); Ex.More(); Ex.Next())
        {
            ed = TopoDS.Edge(Ex.Current());
            if (ed.IsSame(E))
            {
                if (!orset)
                {
                    orient = ed.Orientation();
                    orset = true;
                }
                else if (ed.Orientation() == TopAbs.Reverse(orient))
                {
                    FVoi = FRef;
                    //  Modified by Sergey KHROMOV - Fri Dec 21 17:15:12 2001 Begin
                    //  if (BRep_Tool::Continuity(E,FRef,FRef) >= GeomAbs_G1)
                    return ChFi3d.IsTangentFaces(E, FRef, FRef);
                }
            }
        }
        return false;
    }

    internal static int SearchFaceOnV(in ChFiDS_CommonPoint Pc, in TopoDS_Face FRef, ChFiDS_Map VEMap, ChFiDS_Map EFMap, ref TopoDS_Face F1, ref TopoDS_Face F2)
    {
        // it is checked that it leaves the current face.
        bool FindFace = IsInput(Pc.Vector(), Pc.Vertex(), FRef);
        if (FindFace)
        {
            FindFace = IsInput(Pc.Vector().Reversed(), Pc.Vertex(), FRef);
        }
        // If it does not leave, it is finished
        if (FindFace)
        {
            F1 = FRef;
            return 1;
        }
        int Num = 0;
        bool Trouve = false;
        NCollection_List<TopoDS_Shape>.Iterator ItE = new(), ItF = new();
        TopoDS_Edge E = new();
        TopoDS_Face FVoi = new();

        for (ItE.Initialize(VEMap[Pc.Vertex()]); ItE.More() && (Num < 2); ItE.Next())
        {
            E = TopoDS.Edge(ItE.Value());
            for (ItF.Initialize(EFMap[E]), Trouve = false; ItF.More() && (!Trouve); ItF.Next())
            {
                if (TopoDS.Face(ItF.Value()).IsSame(FRef))
                {
                    Trouve = true;
                }
            }
            if (Trouve)
            {
                Trouve = IsG1(EFMap, E, FRef, ref FVoi);
            }
            if (Trouve)
            {
                Trouve = IsInput(Pc.Vector(), Pc.Vertex(), FVoi);
            }
            if (Trouve)
            {
                if (Num == 0)
                {
                    F1 = FVoi;
                }
                else
                {
                    F2 = FVoi;
                }
                Num++;
            }
        }
        return Num;
    }

    internal static void ChangeTransition(in ChFiDS_CommonPoint Precedant, ref ChFiDS_CommonPoint Courant, int FaceIndex, TopOpeBRepDS_HDataStructure? DS)
    {
        bool tochange = true;
        TopoDS_Face F = TopoDS.Face(DS.Shape(FaceIndex));
        TopoDS_Edge Arc = Precedant.Arc();
        Geom2d_Curve? PCurve1 = null, PCurve2 = null;
        PCurve1 = getCurveOnSurface(Arc, F);
        TopoDS_Shape aLocalShape = Arc.Reversed();
        PCurve2 = getCurveOnSurface(TopoDS.Edge(aLocalShape), F);
        if (PCurve1 != PCurve2)
        {
            // This is a cutting edge, it is necessary to make a small Geometric test
            gp_Vec tgarc = new();
            gp_Pnt P = new();
            BRepAdaptor_Curve AC = new(Arc);
            AC.D1(Precedant.ParameterOnArc(), out P, out tgarc);
            tochange = tgarc.IsParallel(Precedant.Vector(), Precision.Confusion());
        }

        if (tochange)
        {
            Courant.SetArc(Precision.Confusion(), Arc, Precedant.ParameterOnArc(),
                                          TopAbs.Reverse(Precedant.TransitionOnArc()));
        }
    }

    internal static void ChFi3d_BuildPlane(TopOpeBRepDS_DataStructure DStr, BRepAdaptor_Surface? HS, ref gp_Pnt2d pons, ChFiDS_SurfData? SD, bool isfirst, int ons)
    {
        TopoDS_Face F = TopoDS.Face(DStr.Shape(SD.Index(ons)));
        if (F.IsNull())
        {
            return;
        }

        if (SD.Vertex(isfirst, ons).IsOnArc())
        {
            double u = 0.0, v = 0.0;
            Geom2d_Curve? Hc = 
                BRep_Tool.CurveOnSurface(SD.Vertex(isfirst, ons).Arc(), F, out u, out v);
            Hc.Value(SD.Vertex(isfirst, ons).ParameterOnArc()).Coord(out u, out v);
            BRepLProp_SLProps theProp = new(HS, u, v, 1, 1.0e-12);
            if (theProp.IsNormalDefined())
            {
                Geom_Plane? Pln = new Geom_Plane(theProp.Value(), theProp.Normal());
                TopoDS_Face NewF = new BRepLib_MakeFace(Pln, Precision.Confusion()).Face();
                NewF.Orientation(F.Orientation());
                pons.SetCoord(0.0, 0.0);
                HS.Initialize(NewF);
                return; // everything is good !
            }
        }
        throw new Standard_Failure("ChFi3d_BuildPlane : echec .");
    }

    internal static void ChFi3d_SingularExtremity(ChFiDS_Stripe? stripe, TopOpeBRepDS_DataStructure DStr, in TopoDS_Vertex Vtx, double tol3d, double tol2d)
    {
        ChFiDS_SurfData? Fd = null;
        double tolreached = 0.0;
        double Pardeb = 0.0, Parfin = 0.0;
        gp_Pnt2d VOnS1 = new(), VOnS2 = new();
        Geom_Curve? C3d = null;
        Geom2d_Curve? PCurv = null;
        TopOpeBRepDS_Curve Crv = new();
        // SurfData and its CommonPoints,
        int Ivtx = 0, Icurv = 0;
        bool isfirst = false;

        if (stripe.Spine().IsPeriodic())
        {
            isfirst = true;
            Fd = stripe.SetOfSurfData().First();
        }
        else
        {
            int sens = 0;
            int num = ChFi3d_IndexOfSurfData(Vtx, stripe, out sens);
            Fd = stripe.SetOfSurfData().Value(num);
            isfirst = (sens == 1);
        }

        ChFiDS_CommonPoint CV1 = Fd.Vertex(isfirst, 1);
        ChFiDS_CommonPoint CV2 = Fd.Vertex(isfirst, 2);
        // Is it always degenerated ?
        if (CV1.Point().IsEqual(CV2.Point(), 0))
        {
            Ivtx = ChFi3d_IndexPointInDS(CV1, DStr);
            if (isfirst)
            {
                VOnS1 = Fd.InterferenceOnS1().PCurveOnSurf().Value(Fd.InterferenceOnS1().FirstParameter());
                VOnS2 = Fd.InterferenceOnS2().PCurveOnSurf().Value(Fd.InterferenceOnS2().FirstParameter());
            }
            else
            {
                VOnS1 = Fd.InterferenceOnS1().PCurveOnSurf().Value(Fd.InterferenceOnS1().LastParameter());
                VOnS2 = Fd.InterferenceOnS2().PCurveOnSurf().Value(Fd.InterferenceOnS2().LastParameter());
            }

            ChFi3d_ComputeArete(CV1, VOnS1, CV2, VOnS2, DStr.Surface(Fd.Surf()).Surface(), ref C3d, ref PCurv, ref Pardeb,
                                                    ref Parfin, tol3d, tol2d, out tolreached, 0);
            Crv = new TopOpeBRepDS_Curve(C3d, tolreached);
            Icurv = DStr.AddCurve(Crv);

            stripe.SetCurve(Icurv, isfirst);
            stripe.SetParameters(isfirst, Pardeb, Parfin);
            stripe.ChangePCurve(isfirst) = PCurv;
            stripe.SetIndexPoint(Ivtx, isfirst, 1);
            stripe.SetIndexPoint(Ivtx, isfirst, 2);

            if (stripe.Spine().IsPeriodic())
            {
                // periodic case : The operation is renewed
                // the curve 3d is not shared.
                // 2 degenerated edges coinciding in 3d
                isfirst = false;
                Fd = stripe.SetOfSurfData().Last();
                VOnS1 = Fd.InterferenceOnS1().PCurveOnSurf().Value(Fd.InterferenceOnS1().LastParameter());
                VOnS2 = Fd.InterferenceOnS2().PCurveOnSurf().Value(Fd.InterferenceOnS2().LastParameter());

                ChFi3d_ComputeArete(CV1, VOnS1, CV2, VOnS2, DStr.Surface(Fd.Surf()).Surface(), ref C3d, ref PCurv, ref Pardeb,
                                                        ref Parfin, tol3d, tol2d, out tolreached, 0);
                Crv = new TopOpeBRepDS_Curve(C3d, tolreached);
                Icurv = DStr.AddCurve(Crv);

                stripe.SetCurve(Icurv, isfirst);
                stripe.SetParameters(isfirst, Pardeb, Parfin);
                stripe.ChangePCurve(isfirst) = PCurv;
                stripe.SetIndexPoint(Ivtx, isfirst, 1);
                stripe.SetIndexPoint(Ivtx, isfirst, 2);
            }
        }
    }

    internal static bool IsFree(in TopoDS_Shape E, ChFiDS_Map EFMap)
    {
        if (!EFMap.Contains(E))
        {
            return false;
        }
        NCollection_List<TopoDS_Shape>.Iterator It = new();
        TopoDS_Shape Fref = new();
        for (It.Initialize(EFMap[E]); It.More(); It.Next())
        {
            if (Fref.IsNull())
            {
                Fref = It.Value();
            }
            else if (!Fref.IsSame(It.Value()))
            {
                return false;
            }
        }
        return true;
    }

    internal static void ChFi3d_MakeExtremities(ChFiDS_Stripe? Stripe, TopOpeBRepDS_DataStructure DStr, ChFiDS_Map EFMap, double tol3d, double tol2d)
    {
        ref ChFiDS_Spine? sp = ref Stripe.ChangeSpine();
        double Pardeb = 0.0, Parfin = 0.0;
        Geom_Curve? C3d = null;
        double tolreached = 0.0;
        if (sp.IsPeriodic())
        {
            Bnd_Box b1 = new(), b2 = new();
            ChFiDS_SurfData? SDF = Stripe.SetOfSurfData().First();
            ChFiDS_CommonPoint CV1 = SDF.VertexFirstOnS1();
            ChFiDS_CommonPoint CV2 = SDF.VertexFirstOnS2();
            if (!CV1.Point().IsEqual(CV2.Point(), 0))
            {
                ChFi3d_ComputeArete(CV1, SDF.InterferenceOnS1().PCurveOnSurf().Value(SDF.InterferenceOnS1().FirstParameter()),
                    CV2, SDF.InterferenceOnS2().PCurveOnSurf().Value(SDF.InterferenceOnS2().FirstParameter()),
                    DStr.Surface(SDF.Surf()).Surface(), ref C3d, ref Stripe.ChangeFirstPCurve(), ref Pardeb, ref Parfin, tol3d,
                    tol2d, out tolreached, 0);
                TopOpeBRepDS_Curve Crv = new(C3d, tolreached);
                Stripe.ChangeFirstCurve(DStr.AddCurve(Crv));
                Stripe.ChangeFirstParameters(Pardeb, Parfin);
                Stripe.ChangeIndexFirstPointOnS1(ChFi3d_IndexPointInDS(SDF.VertexFirstOnS1(), DStr));
                Stripe.ChangeIndexFirstPointOnS2(ChFi3d_IndexPointInDS(SDF.VertexFirstOnS2(), DStr));
                int ICurv = Stripe.FirstCurve();
                Stripe.ChangeLastParameters(Pardeb, Parfin);
                Stripe.ChangeLastCurve(ICurv);
                Stripe.ChangeIndexLastPointOnS1(Stripe.IndexFirstPointOnS1());
                Stripe.ChangeIndexLastPointOnS2(Stripe.IndexFirstPointOnS2());

                ChFiDS_SurfData? SDL = Stripe.SetOfSurfData().Last();

                ChFi3d_ComputePCurv(C3d, SDL.InterferenceOnS1().PCurveOnSurf().Value(SDL.InterferenceOnS1().LastParameter()),
                    SDL.InterferenceOnS2().PCurveOnSurf().Value(SDL.InterferenceOnS2().LastParameter()),
                    out Stripe.ChangeLastPCurve(), DStr.Surface(SDL.Surf()).Surface(), Pardeb, Parfin, tol3d, out tolreached);
                double oldtol = DStr.ChangeCurve(ICurv).Tolerance();
                DStr.ChangeCurve(ICurv).Tolerance(Max(oldtol, tolreached));
                if (CV1.IsOnArc())
                {
                    ChFi3d_EnlargeBox(CV1.Arc(), EFMap[CV1.Arc()], CV1.ParameterOnArc(), ref b1);
                }

                if (CV2.IsOnArc())
                {
                    ChFi3d_EnlargeBox(CV2.Arc(), EFMap[CV2.Arc()], CV2.ParameterOnArc(), ref b2);
                }
                ChFi3d_EnlargeBox(DStr, Stripe, SDF, ref b1, ref b2, true);
                ChFi3d_EnlargeBox(DStr, Stripe, SDL, ref b1, ref b2, false);
                if (!CV1.IsVertex())
                {
                    ChFi3d_SetPointTolerance(DStr, b1, Stripe.IndexFirstPointOnS1());
                }
                if (!CV2.IsVertex())
                {
                    ChFi3d_SetPointTolerance(DStr, b2, Stripe.IndexFirstPointOnS2());
                }
            }
            else
            {
                // Case of the single extremity
                if (CV1.IsVertex())
                {
                    ChFi3d_SingularExtremity(Stripe, DStr, CV1.Vertex(), tol3d, tol2d);
                }
                // #ifdef CHFI3D_DEB: a trace, left out
            }
            return;
        }

        ChFiDS_SurfData? SDdeb = Stripe.SetOfSurfData().First();

        ChFiDS_CommonPoint cpdeb1 = SDdeb.VertexFirstOnS1();
        ChFiDS_CommonPoint cpdeb2 = SDdeb.VertexFirstOnS2();
        bool freedeb = sp.FirstStatus() == ChFiDS_FreeBoundary;
        if (!freedeb && cpdeb1.IsOnArc() && cpdeb2.IsOnArc())
        {
            freedeb = (IsFree(cpdeb1.Arc(), EFMap) && IsFree(cpdeb2.Arc(), EFMap));
        }
        if (freedeb)
        {
            sp.SetFirstStatus(ChFiDS_FreeBoundary);
            Bnd_Box b1 = new(), b2 = new();
            if (!cpdeb1.Point().IsEqual(cpdeb2.Point(), 0))
            {
                bool plane = false;
                gp_Pnt2d UV1 = new(), UV2 = new();
                UV1 =
                    SDdeb.InterferenceOnS1().PCurveOnSurf().Value(SDdeb.InterferenceOnS1().FirstParameter());
                UV2 =
                    SDdeb.InterferenceOnS2().PCurveOnSurf().Value(SDdeb.InterferenceOnS2().FirstParameter());
                // The intersection of the fillet by a plane is attempted

                GeomAdaptor_Surface? HConge = ChFi3d_BoundSurf(DStr, SDdeb, 1, 2);
                ChFi3d_CoupeParPlan(cpdeb1, cpdeb2, HConge, UV1, UV2, tol3d, tol2d, ref C3d, ref Stripe.ChangeFirstPCurve(), ref tolreached,
                                                        ref Pardeb, ref Parfin, ref plane);
                if (!plane)
                {
                    ChFi3d_ComputeArete(cpdeb1, SDdeb.InterferenceOnS1().PCurveOnSurf().Value(
                                                                SDdeb.InterferenceOnS1().FirstParameter()), cpdeb2,
                                                            SDdeb.InterferenceOnS2().PCurveOnSurf().Value(SDdeb.InterferenceOnS2().FirstParameter()),
                                                            DStr.Surface(SDdeb.Surf()).Surface(), ref C3d, ref Stripe.ChangeFirstPCurve(), ref Pardeb,
                                                            ref Parfin, tol3d, tol2d, out tolreached, 0);
                }
                TopOpeBRepDS_Curve Crv = new(C3d, tolreached);
                Stripe.ChangeFirstCurve(DStr.AddCurve(Crv));
                Stripe.ChangeFirstParameters(Pardeb, Parfin);
                Stripe.ChangeIndexFirstPointOnS1(ChFi3d_IndexPointInDS(SDdeb.VertexFirstOnS1(), DStr));
                Stripe.ChangeIndexFirstPointOnS2(ChFi3d_IndexPointInDS(SDdeb.VertexFirstOnS2(), DStr));
                if (cpdeb1.IsOnArc())
                {
                    ChFi3d_EnlargeBox(cpdeb1.Arc(), EFMap[cpdeb1.Arc()], cpdeb1.ParameterOnArc(), ref b1);
                }
                if (cpdeb2.IsOnArc())
                {
                    ChFi3d_EnlargeBox(cpdeb2.Arc(), EFMap[cpdeb2.Arc()], cpdeb2.ParameterOnArc(), ref b2);
                }
                ChFi3d_EnlargeBox(DStr, Stripe, SDdeb, ref b1, ref b2, true);
                if (!cpdeb1.IsVertex())
                {
                    ChFi3d_SetPointTolerance(DStr, b1, Stripe.IndexFirstPointOnS1());
                }
                if (!cpdeb2.IsVertex())
                {
                    ChFi3d_SetPointTolerance(DStr, b2, Stripe.IndexFirstPointOnS2());
                }
            }
            else
            { // Case of a singular extremity
                if (cpdeb1.IsVertex())
                {
                    ChFi3d_SingularExtremity(Stripe, DStr, cpdeb1.Vertex(), tol3d, tol2d);
                }
                // #ifdef CHFI3D_DEB: a trace, left out
            }
        }
        ChFiDS_SurfData? SDfin = Stripe.SetOfSurfData().Last();
        ChFiDS_CommonPoint cpfin1 = SDfin.VertexLastOnS1();
        ChFiDS_CommonPoint cpfin2 = SDfin.VertexLastOnS2();
        bool freefin = sp.LastStatus() == ChFiDS_FreeBoundary;
        if (!freefin && cpfin1.IsOnArc() && cpfin2.IsOnArc())
        {
            freefin = (IsFree(cpfin1.Arc(), EFMap) && IsFree(cpfin2.Arc(), EFMap));
        }
        if (freefin)
        {
            sp.SetLastStatus(ChFiDS_FreeBoundary);
            Bnd_Box b1 = new(), b2 = new();
            if (!cpfin1.Point().IsEqual(cpfin2.Point(), 0))
            {
                bool plane = false;
                gp_Pnt2d UV1 = new(), UV2 = new();
                UV1 =
                    SDfin.InterferenceOnS1().PCurveOnSurf().Value(SDfin.InterferenceOnS1().LastParameter());
                UV2 =
                    SDfin.InterferenceOnS2().PCurveOnSurf().Value(SDfin.InterferenceOnS2().LastParameter());
                // Intersection of the fillet by a plane is attempted

                GeomAdaptor_Surface? HConge = ChFi3d_BoundSurf(DStr, SDfin, 1, 2);
                ChFi3d_CoupeParPlan(cpfin1, cpfin2, HConge, UV1, UV2, tol3d, tol2d, ref C3d, ref Stripe.ChangeLastPCurve(), ref tolreached,
                                                        ref Pardeb, ref Parfin, ref plane);
                if (!plane)
                {
                    ChFi3d_ComputeArete(cpfin1, SDfin.InterferenceOnS1().PCurveOnSurf().Value(
                                                                SDfin.InterferenceOnS1().LastParameter()), cpfin2,
                                                            SDfin.InterferenceOnS2().PCurveOnSurf().Value(SDfin.InterferenceOnS2().LastParameter()),
                                                            DStr.Surface(SDfin.Surf()).Surface(), ref C3d, ref Stripe.ChangeLastPCurve(), ref Pardeb,
                                                            ref Parfin, tol3d, tol2d, out tolreached, 0);
                }
                TopOpeBRepDS_Curve Crv = new(C3d, tolreached);
                Stripe.ChangeLastCurve(DStr.AddCurve(Crv));
                Stripe.ChangeLastParameters(Pardeb, Parfin);
                Stripe.ChangeIndexLastPointOnS1(ChFi3d_IndexPointInDS(SDfin.VertexLastOnS1(), DStr));
                Stripe.ChangeIndexLastPointOnS2(ChFi3d_IndexPointInDS(SDfin.VertexLastOnS2(), DStr));
                if (cpfin1.IsOnArc())
                {
                    ChFi3d_EnlargeBox(cpfin1.Arc(), EFMap[cpfin1.Arc()], cpfin1.ParameterOnArc(), ref b1);
                }
                if (cpfin2.IsOnArc())
                {
                    ChFi3d_EnlargeBox(cpfin2.Arc(), EFMap[cpfin2.Arc()], cpfin2.ParameterOnArc(), ref b2);
                }
                ChFi3d_EnlargeBox(DStr, Stripe, SDfin, ref b1, ref b2, false);
                if (!cpfin1.IsVertex())
                {
                    ChFi3d_SetPointTolerance(DStr, b1, Stripe.IndexLastPointOnS1());
                }
                if (!cpfin2.IsVertex())
                {
                    ChFi3d_SetPointTolerance(DStr, b2, Stripe.IndexLastPointOnS2());
                }
            }
            else
            { // Case of the single extremity
                if (cpfin1.IsVertex())
                {
                    ChFi3d_SingularExtremity(Stripe, DStr, cpfin1.Vertex(), tol3d, tol2d);
                }
                // #ifdef CHFI3D_DEB: a trace, left out
            }
        }
    }

    internal static void ChFi3d_Purge(ChFiDS_Stripe? Stripe, ChFiDS_SurfData? SD, in ChFiDS_CommonPoint VRef, bool isfirst, int ons, ref int intf, ref int intl)
    {
        if (isfirst)
        {
            intf = 1;
        }
        else
        {
            intl = 1; // End.
        }
        int opp = 3 - ons;
        if (!SD.Vertex(isfirst, opp).IsOnArc() || SD.TwistOnS1() || SD.TwistOnS2())
        {
            NCollection_Sequence<ChFiDS_SurfData> Seq = 
                Stripe.ChangeSetOfSurfData();
            if (isfirst)
            {
                Seq.Remove(1);
            }
            else
            {
                Seq.Remove(Seq.Length());
            }
            return;
        }
        if (ons == 1)
        {
            SD.ChangeIndexOfS1(0);
        }
        else
        {
            SD.ChangeIndexOfS2(0);
        }

        SD.ChangeVertex(!isfirst, ons) = VRef;
        SD.ChangeVertex(isfirst, ons) = VRef;

        ref ChFiDS_FaceInterference fi = ref SD.ChangeInterference(ons);
        if (isfirst)
        {
            fi.SetFirstParameter(fi.LastParameter());
        }
        else
        {
            fi.SetLastParameter(fi.FirstParameter());
        }
        fi.SetLineIndex(0);
    }

    internal static void InsertAfter(ChFiDS_Stripe? Stripe, ChFiDS_SurfData? Ref, ChFiDS_SurfData? Item)
    {
        if (Ref == Item)
        {
            throw new Standard_Failure("InsertAfter : twice the same surfdata.");
        }

        NCollection_Sequence<ChFiDS_SurfData> Seq = 
            Stripe.ChangeSetOfSurfData();

        if (Seq.IsEmpty() || Ref is null)
        {
            Seq.Prepend(Item);
        }
        for (int i = 1; i <= Seq.Length(); i++)
        {
            if (Seq.Value(i) == Ref)
            {
                Seq.InsertAfter(i, Item);
                break;
            }
        }
    }

    internal static void RemoveSD(ChFiDS_Stripe? Stripe, ChFiDS_SurfData? Prev, ChFiDS_SurfData? Next)
    {
        NCollection_Sequence<ChFiDS_SurfData> Seq = 
            Stripe.ChangeSetOfSurfData();
        if (Seq.IsEmpty())
        {
            return;
        }
        int iprev = 0, inext = 0;
        for (int i = 1; i <= Seq.Length(); i++)
        {
            if (Seq.Value(i) == Prev)
            {
                iprev = i + 1;
            }
            if (Seq.Value(i) == Next)
            {
                inext = i - 1;
                break;
            }
        }
        if (Prev is null)
        {
            iprev = 1;
        }
        if (Next is null)
        {
            inext = Seq.Length();
        }
        if (iprev <= inext)
        {
            Seq.Remove(iprev, inext);
        }
    }

    internal static void InsertBefore(ChFiDS_Stripe? Stripe, ChFiDS_SurfData? Ref, ChFiDS_SurfData? Item)
    {
        if (Ref == Item)
        {
            throw new Standard_Failure("InsertBefore : twice the same surfdata.");
        }

        NCollection_Sequence<ChFiDS_SurfData> Seq = 
            Stripe.ChangeSetOfSurfData();

        if (Seq.IsEmpty() || Ref is null)
        {
            Seq.Append(Item);
        }
        for (int i = 1; i <= Seq.Length(); i++)
        {
            if (Seq.Value(i) == Ref)
            {
                Seq.InsertBefore(i, Item);
                break;
            }
        }
    }

    internal static double ChFi3d_BoxDiag(in Bnd_Box box)
    {
        double a = 0.0, b = 0.0, c = 0.0, d = 0.0, e = 0.0, f = 0.0;
        box.Get(out a, out b, out c, out d, out e, out f);
        d -= a;
        e -= b;
        f -= c;
        d *= d;
        e *= e;
        f *= f;
        double diag = Math.Sqrt(d + e + f);
        return diag;
    }


}
