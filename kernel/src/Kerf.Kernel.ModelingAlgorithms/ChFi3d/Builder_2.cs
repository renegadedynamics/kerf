// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Properties;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Engine.Fillets.Builder_2;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Fillets;
internal partial class FilletBuilderBase
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

    internal void CallPerformSurf(Stripe? stripe, bool simul, NCollection_Sequence<SurfData> seqSD, SurfData? sd, ElSpine? hGuide, Spine? spine, FaceSurfaceAdaptor? hs1, FaceSurfaceAdaptor? hs3, in Point2D pp1, in Point2D pp3, TopolTool? it1, FaceSurfaceAdaptor? hs2, FaceSurfaceAdaptor? hs4, in Point2D pp2, in Point2D pp4, TopolTool? it2, double maxStep, double fleche, double _unused1, ref double first, ref double last, bool inside, bool _unused2, bool forward, bool recOnS1, bool recOnS2, math_Vector soldep, ref int intf, ref int intl, ref FaceSurfaceAdaptor? surf1, ref FaceSurfaceAdaptor? surf2)
    {
        FaceSurfaceAdaptor? hSon1 = null, hSon2 = null;
        hSon1 = hs1;
        hSon2 = hs2;
        // Definition of the domain of path It1, It2
        it1.Initialize(hSon1);
        it2.Initialize(hSon2);

        Orientation or1 = hs1.Face().Orientation;
        Orientation or2 = hs2.Face().Orientation;
        int choix = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2, stripe.OrientationOnFace1(), stripe.OrientationOnFace2(),
                                                                  stripe.Choix());
        soldep[1] = pp1.X;
        soldep[2] = pp1.Y;
        soldep[3] = pp2.X;
        soldep[4] = pp2.Y;

        double thef = first, thel = last;
        bool isdone = false;

        if (simul)
        {
            isdone = SimulSurf(sd, hGuide, spine, choix, hs1, it1, hs2, it2, _tolesp, ref first, ref last, inside, inside,
                                                  forward, recOnS1, recOnS2, soldep, ref intf, ref intl);
        }
        else
        {
            isdone = PerformSurf(seqSD, hGuide, spine, choix, hs1, it1, hs2, it2, maxStep, fleche, _tolesp, ref first,
                                                      ref last, inside, inside,
                                                      forward, recOnS1, recOnS2, soldep, ref intf, ref intl);
        }

        // Case of error
        if (!isdone)
        {
            first = thef;
            last = thel;
            bool reprise = false;
            if (hs3 is not null)
            {
                hSon1 = hs3;
                it1.Initialize(hs3);
                or1 = hs3.Face().Orientation;
                soldep[1] = pp3.X;
                soldep[2] = pp3.Y;
                reprise = true;
            }
            else if (hs4 is not null)
            {
                hSon2 = hs4;
                it2.Initialize(hs4);
                or2 = hs4.Face().Orientation;
                soldep[3] = pp4.X;
                soldep[4] = pp4.Y;
                reprise = true;
            }

            if (reprise)
            {
                choix = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2, stripe.OrientationOnFace1(), stripe.OrientationOnFace2(),
                                                                  stripe.Choix());
                if (simul)
                {
                    isdone = SimulSurf(sd, hGuide, spine, choix, hSon1, it1, hSon2, it2, _tolesp, ref first, ref last, inside,
                                                          inside,
                                                          forward, recOnS1, recOnS2, soldep, ref intf, ref intl);
                }
                else
                {
                    isdone = PerformSurf(seqSD, hGuide, spine, choix, hSon1, it1, hSon2, it2, maxStep, fleche, _tolesp, ref first,
                                                              ref last, inside, inside,
                                                              forward, recOnS1, recOnS2, soldep, ref intf, ref intl);
                }
            }
        }
        surf1 = hSon1;
        surf2 = hSon2;
    }

    //=======================================================================
    // function : StripeOrientation
    // purpose  : Calculates the reference orientation determining the
    //           concave face for construction of the fillet.
    //=======================================================================

    internal bool StripeOrientations(Spine? spine, ref Orientation or1, ref Orientation or2, ref int choixConge)
    {
        // NCollection_List<TopoDS_Shape>::Iterator It;
        FaceSurfaceAdaptor sb1 = new(), sb2 = new();
        Orientation of1 = default, of2 = default;
        Face ff1 = new(), ff2 = new();
        Edge edge = spine.Edges(1);
        Face firstFace = ShapeCasts.Face(_edgeFirstFace.GetValue(edge));
        ChFi3d_conexfaces(edge, out ff1, out ff2, _efMap);
        if (ff2.IsSame(firstFace))
        {
            Face tmpFace = ff1;
            ff1 = ff2;
            ff2 = tmpFace;
        }
        of1 = ff1.Orientation;
        ff1.Orient(Forward);
        sb1.Initialize(ff1);
        of2 = ff2.Orientation;
        ff2.Orient(Forward);
        sb2.Initialize(ff2);

        choixConge = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(sb1, sb2, spine.Edges(1), out or1, out or2);
        or1 = global::Kerf.Engine.Topology.TopAbs.Compose(or1, of1);
        or2 = global::Kerf.Engine.Topology.TopAbs.Compose(or2, of2);
        return true;
    }

    //=================================================================================================

    private void ConexFaces(Spine? spine, int iEdge, ref FaceSurfaceAdaptor? hs1, ref FaceSurfaceAdaptor? hs2)
    {
        if (hs1 is null)
        {
            hs1 = new FaceSurfaceAdaptor();
        }
        if (hs2 is null)
        {
            hs2 = new FaceSurfaceAdaptor();
        }
        FaceSurfaceAdaptor sb1 = hs1; // references to *HS1, *HS2 in OCCT
        FaceSurfaceAdaptor sb2 = hs2;

        Face ff1 = new(), ff2 = new();
        Edge edge = spine.Edges(iEdge);
        ChFi3d_conexfaces(spine.Edges(iEdge), out ff1, out ff2, _efMap);

        Face firstFace = ShapeCasts.Face(_edgeFirstFace.GetValue(edge));
        if (ff2.IsSame(firstFace))
        {
            Face tmpFace = ff1;
            ff1 = ff2;
            ff2 = tmpFace;
        }

        sb1.Initialize(ff1);
        sb2.Initialize(ff2);
    }

    //=======================================================================
    // function : StartSol
    // purpose  : Calculates a starting solution :
    //           - one starts by parsing about ten points on the spine,
    //           - in case of fail one finds the solution on neighbor faces;
    //             section plane of edges of the adjacent face
    //             and identification of the face by connection to that edge.
    //=======================================================================

    private void StartSol(Stripe? stripe, ElSpine? hGuide, ref FaceSurfaceAdaptor? hs1, ref FaceSurfaceAdaptor? hs2, BrepTopolTool? i1, BrepTopolTool? i2, ref Point2D p1, ref Point2D p2, ref double first)
    {
        ref Spine? spine = ref stripe.ChangeSpine();
        ElSpine els = hGuide; // a reference in OCCT
        int nbed = spine.EdgeCount;
        int nbessaimax = 3 * nbed;
        if (nbessaimax < 10)
        {
            nbessaimax = 10;
        }
        double unsurnbessaimax = 1.0 / nbessaimax;
        double wf = 0.9981 * spine.FirstParameter(1) + 0.0019 * spine.LastParameter(1);
        double wl = 0.9973 * spine.LastParameter(nbed) + 0.0027 * spine.FirstParameter(nbed);

        double tolE = 1.0e-7;
        FaceSurfaceAdaptor AS = new();

        int nbessai = 0;
        int iedge = 0;
        int rc = stripe.Choix();
        Vector2D derive = new();
        Point2D p2d = new();
        Edge cured = new();
        Face f1 = new(), f2 = new();
        Orientation or1 = default, or2 = default;
        int choix = 0;
        math_Vector solDep = new(1, 4);
        Curve2D? pc = null;
        CurvePointExtrema pExt = new();
        pExt.Initialize(els, spine.FirstParameter(1), spine.LastParameter(nbed), global::Kerf.Engine.Foundation.Precision.Confusion());
        Containment pos1 = default, pos2 = default;
        for (nbessai = 0; nbessai <= nbessaimax; nbessai++)
        {
            double t = nbessai * unsurnbessaimax;
            double w = wf * (1.0 - t) + wl * t;
            int ie = spine.Index(w);
            if (iedge != ie)
            {
                iedge = ie;
                cured = spine.Edges(iedge);
                tolE = Tool.Tolerance(cured);
                ConexFaces(spine, iedge, ref hs1, ref hs2);
                f1 = hs1.Face();
                f2 = hs2.Face();
                or1 = f1.Orientation;
                or2 = f2.Orientation;
                choix =
                    global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2, stripe.OrientationOnFace1(), stripe.OrientationOnFace2(), rc);
            }

            double woned = 0.0, resU = 0.0, resV = 0.0;
            spine.Parameter(iedge, w, out woned, true);
            cured.Orient(Forward);
            Face f1forward = f1, f2forward = f2;
            f1forward.Orient(Forward);
            f2forward.Orient(Forward);
            pc = GetCurveOnSurface(cured, f1forward);
            i1.Initialize(hs1);
            pc.D1(woned, out p1, out derive);
            // There are points on the border, and internal points are found
            if (derive.Magnitude > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                derive = derive.Normalized();
                derive = derive.Rotated(M_PI / 2);
                AS.Initialize(f1);
                resU = AS.UResolution(tolE);
                resV = AS.VResolution(tolE);
                derive *= 2 * (Math.Abs(derive.X) * resU + Math.Abs(derive.Y) * resV);
                p2d = p1.Translated(derive);
                if (i1.Classify(p2d, Min(resU, resV), false) == In)
                {
                    p1 = p2d;
                }
                else
                {
                    p2d = p1.Translated(-derive);
                    if (i1.Classify(p2d, Min(resU, resV), false) == In)
                    {
                        p1 = p2d;
                    }
                }
            }
            if (f1.IsSame(f2))
            {
                cured.Orient(Reversed);
            }
            pc = GetCurveOnSurface(cured, f2forward);
            p2 = pc.Value(woned);
            SurfaceAdaptorBase? hSon2 = hs2;// to avoid ambiguity
            i2.Initialize(hSon2);

            solDep[1] = p1.X;
            solDep[2] = p1.Y;
            solDep[3] = p2.X;
            solDep[4] = p2.Y;
            EdgeCurveAdaptor ced = spine.CurrentElementarySpine(iedge);
            Point3D pnt = ced.Value(woned);

            if (Projection(pExt, pnt, els, ref w, _tolapp3d)
                    && PerformFirstSection(spine, hGuide, choix, hs1, hs2, i1, i2, w, solDep, ref pos1, ref pos2))
            {
                p1 = p1.WithCoord(solDep[1], solDep[2]);
                p2 = p2.WithCoord(solDep[3], solDep[4]);
                first = w;
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
            iedge = spine.Index(w);
            cured = spine.Edges(iedge);
            ConexFaces(spine, iedge, ref hs1, ref hs2);
            f1 = hs1.Face();
            f2 = hs2.Face();
            or1 = f1.Orientation;
            or2 = f2.Orientation;
            choix =
                global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2, stripe.OrientationOnFace1(), stripe.OrientationOnFace2(), rc);
            double woned = 0.0;
            spine.Parameter(iedge, w, out woned, true);
            Face f1forward = f1, f2forward = f2;
            f1forward.Orient(Forward);
            f2forward.Orient(Forward);
            pc = GetCurveOnSurface(cured, f1forward);
            p1 = pc.Value(woned);
            pc = GetCurveOnSurface(cured, f2forward);
            p2 = pc.Value(woned);
            SurfaceAdaptorBase? hSon1 = hs1;// to avoid ambiguity
            SurfaceAdaptorBase? hSon2 = hs2;// to avoid ambiguity
            i1.Initialize(hSon1);
            i2.Initialize(hSon2);
            solDep[1] = p1.X;
            solDep[2] = p1.Y;
            solDep[3] = p2.X;
            solDep[4] = p2.Y;
            EdgeCurveAdaptor ced = spine.CurrentElementarySpine(iedge);
            Point3D pnt = ced.Value(woned);
            //    Extrema_LocateExtPC ext(pnt,els,w,1.e-8);
            //    if(ext.IsDone()){
            //      w = ext.Point().Parameter();
            if (Projection(pExt, pnt, els, ref w, _tolapp3d))
            {
                PerformFirstSection(spine, hGuide, choix, hs1, hs2, i1, i2, w, solDep, ref pos1, ref pos2);
                Point3D p = new();
                Vector3D v = new();
                hGuide.D1(w, out p, out v);
                PlaneSurface? pl = new PlaneSurface(p, new Direction3D(v));
                SurfaceAdaptor? plane = new SurfaceAdaptor(pl);

                bool bonvoisin = true, found = false;
                int nbChangement = 0;
                for (nbChangement = 1; bonvoisin && (!found) && (nbChangement < 5); nbChangement++)
                {
                    if (pos1 != In)
                    {
                        bonvoisin = BonVoisin(p, hs1, ref f1, plane, cured, ref solDep[1], ref solDep[2], _efMap, _tolapp3d);
                    }
                    if (pos2 != In && bonvoisin)
                    {
                        bonvoisin = BonVoisin(p, hs2, ref f2, plane, cured, ref solDep[3], ref solDep[4], _efMap, _tolapp3d);
                    }
                    if (bonvoisin)
                    {
                        f1 = hs1.Face();
                        f2 = hs2.Face();
                        or1 = f1.Orientation;
                        or2 = f2.Orientation;
                        choix = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2,
                                                                          stripe.OrientationOnFace1(), stripe.OrientationOnFace2(), rc);
                        SurfaceAdaptorBase? hSon1new = hs1;// to avoid ambiguity
                        SurfaceAdaptorBase? hSon2new = hs2;// to avoid ambiguity
                        i1.Initialize(hSon1new);
                        i2.Initialize(hSon2new);
                        if (PerformFirstSection(spine, hGuide, choix, hs1, hs2, i1, i2, w, solDep, ref pos1, ref pos2))
                        {
                            p1 = p1.WithCoord(solDep[1], solDep[2]);
                            p2 = p2.WithCoord(solDep[3], solDep[4]);
                            first = w;
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
        spine.SetErrorStatus(ErrorStatus.StartsolFailure);
        throw new FailureException("StartSol echec");
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

    private bool StartSol(Spine spine,
                          FaceSurfaceAdaptor hs,            // New face
                          ref Point2D pons,                 // " Localization
                          ref EdgeCurve2DAdaptor hc,        // Representation of the obstacle
                          ref double w, SurfData sd, bool isfirst, int ons,
                          FaceSurfaceAdaptor hSref,         // The other representation
                          EdgeCurve2DAdaptor hCref,         // of the obstacle
                          ref bool recP, ref bool recS, ref bool recRst, ref bool c1obstacle,
                          ref FaceSurfaceAdaptor hsBis,     // Face of support
                          ref Point2D pBis,                 // and its point
                          bool decroch, in Vertex vref)
    {
        recP = false;
        recS = false;
        recRst = false;
        c1obstacle = false;
        Face fv = new();
        Curve2D? pCurve = null;

        DsDataStructure dStr = _ds.ChangeDS();
        Face fref = hSref is not null ? hSref.Face() : new Face();
        Face f = ShapeCasts.Face(dStr.Shape(sd.Index(ons)));
        if (f.IsNull)
        {
            return false;
        }

        CommonPoint commonPoint = sd.Vertex(isfirst, ons);
        hsBis = null;

        if (commonPoint.IsOnArc)
        {
            int notons = ons == 1 ? 2 : 1;
            CommonPoint cPbis = sd.Vertex(isfirst, notons);
            if (cPbis.IsOnArc)
            { // It is checked if it is not the extension zone
                // In case aCommonPoint is not at the end of surfdata and it is not necessary to take it into
                // account except for separate cases (ie pointus) ... ts and tns were earlier
                // aCommonPoint.Parameter() and CPbis.Parameter, but sometimes they had no values.
                double ts = sd.Interference(ons).Parameter(isfirst);
                double tns = sd.Interference(notons).Parameter(isfirst);
                // Arbitrary test (to precise)
                bool isExtend = isfirst ? (ts - tns > 100 * _tolesp) : (tns - ts > 100 * _tolesp);
                if (isExtend && !commonPoint.Point().IsEqual(cPbis.Point(), 0))
                {
                    //  the state is preserved and False is returned (extension by the expected plane).
                    pCurve = sd.Interference(ons).PCurveOnFace();
                    if (pCurve is null)
                    {
                        return false;
                    }
                    hs.Initialize(f);
                    // The 2nd point is given by its trace on the support surface
                    recS = false;
                    pons = pCurve.Value(tns);
                    return false;
                }
            }
        }

        if (commonPoint.IsVertex && hc is not null && !decroch)
        {
            // The edge is changed, the parameter is updated and
            // eventually the support face and(or) the reference face.
            Vertex vcp = commonPoint.Vertex();
            Edge ehc = hc.Edge();
            if (vcp.IsNull || ehc.IsNull || fref.IsNull || hSref is null || hCref is null)
            {
                return false;
            }
            // One starts by searching in Fref another edge referencing VCP.
            Edge newedge = new();
            Edge edgereg = new();
            Face facereg = new();
            Face bidface = fref;
            bidface.Orient(Forward);
            for (Explorer ex1 = new(bidface, ShapeType.Edge); ex1.More(); ex1.Next())
            {
                Edge cured = ShapeCasts.Edge(ex1.Current());
                bool found = false;
                if (!cured.IsSame(ehc))
                {
                    for (Explorer ex2 = new(cured, ShapeType.Vertex); ex2.More() && !found; ex2.Next())
                    {
                        if (ex2.Current().IsSame(vcp))
                        {
                            if (IsG1(_efMap, cured, fref, ref fv))
                            {
                                edgereg = cured;
                                facereg = fv;
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
            if (newedge.IsNull)
            {
                // It is checked if EHC is not a closed edge.
                Vertex v1 = new(), v2 = new();
                Exploration.Vertices(ehc, out v1, out v2);
                if (v1.IsSame(v2))
                {
                    newedge = ehc;
                    double w1 = Tool.Parameter(v1, ehc);
                    double w2 = Tool.Parameter(v2, ehc);
                    FaceInterference fi = sd.Interference(ons);
                    Curve2D? pcf = fi.PCurveOnFace();
                    double ww = fi.Parameter(isfirst);
                    Point2D pww = pcf is null ? sd.Get2dPoints(isfirst, ons) : pcf.Value(ww);
                    Point2D p1 = hc.Value(w1);
                    Point2D p2 = hc.Value(w2);

                    if (p1.DistanceTo(pww) > p2.DistanceTo(pww))
                    {
                        w = w1;
                        pons = p1;
                    }
                    else
                    {
                        w = w2;
                        pons = p2;
                    }
                    recP = true;
                    c1obstacle = true;
                    return true;
                }
                else if (!edgereg.IsNull)
                {
                    // the reference edge and face are changed.
                    fref = facereg;
                    hSref.Initialize(fref);
                    for (Explorer ex1 = new(facereg, ShapeType.Edge); ex1.More() && newedge.IsNull; ex1.Next())
                    {
                        Edge cured = ShapeCasts.Edge(ex1.Current());
                        if (!cured.IsSame(edgereg))
                        {
                            for (Explorer ex2 = new(cured, ShapeType.Vertex); ex2.More(); ex2.Next())
                            {
                                if (ex2.Current().IsSame(vcp))
                                {
                                    if (!IsG1(_efMap, cured, fref, ref fv))
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
            if (newedge.IsNull)
            {
                throw new FailureException("StartSol : chain is not possible, new obstacle not found");
            }
            if (IsG1(_efMap, newedge, fref, ref fv))
            {
                throw new FailureException("StartSol : chain is not possible, config non processed");
            }
            else if (fv.IsNull)
            {
                throw new FailureException("StartSol : chain is not possible, new obstacle not found");
            }
            else
            {
                Curve2D? refPCurve = GetCurveOnSurface(newedge, fref);
                if (refPCurve is null)
                {
                    return false;
                }

                Face newface = fv;
                newface.Orient(Forward);
                for (Explorer ex = new(newface, ShapeType.Edge); ex.More(); ex.Next())
                {
                    if (ex.Current().IsSame(newedge))
                    {
                        newedge = ShapeCasts.Edge(ex.Current());
                        break;
                    }
                }
                Curve2D? newPCurve = GetCurveOnSurface(newedge, fv);
                if (newPCurve is null)
                {
                    return false;
                }
                hs.Initialize(fv);
                w = Tool.Parameter(vcp, newedge);
                hCref.Initialize(newedge, fref);
                hc.Initialize(newedge, fv);
                pons = hc.Value(w);
            }
            recP = true;
            c1obstacle = true;
            return true;
        } // End of Case Vertex && Obstacle
        else if (commonPoint.IsOnArc && hc is not null && !decroch)
        {
            // Nothing is changed, the parameter is only updated.
            w = commonPoint.ParameterOnArc();
            c1obstacle = true;
            return true;
        }

        hc = null;

        if (commonPoint.IsOnArc)
        {
            Edge arcEdge = commonPoint.Arc();
            if (arcEdge.IsNull)
            {
                return false;
            }

            // Lambda to avoid code duplication.
            // Attempts to set HS, W, aPCurve and pons to default return values.
            // (A static local function: C# lambdas cannot capture the ref parameters W and pons.)
            static bool prepareDefaultReturn(in Edge anArcEdge, in Face f, FaceSurfaceAdaptor hs,
                                             in CommonPoint aCommonPoint, ref Curve2D aPCurve,
                                             ref double w, ref Point2D pons)
            {
                aPCurve = GetCurveOnSurface(anArcEdge, f);
                if (aPCurve is null)
                {
                    return false;
                }
                hs.Initialize(f);
                w = aCommonPoint.ParameterOnArc();
                pons = aPCurve.Value(w);
                return true;
            }

            if (decroch)
            {
                if (fref.IsNull)
                {
                    prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                    return false;
                }
                pCurve = GetCurveOnSurface(arcEdge, fref);
                if (pCurve is null)
                {
                    prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                    return false;
                }
                hs.Initialize(fref);
                w = commonPoint.ParameterOnArc();
                pons = pCurve.Value(w);
                recS = true;
                return true;
            }

            if (SearchFace(spine, commonPoint, f, ref fv))
            {
                if (fv.IsNull)
                {
                    prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                    return false;
                }
                hs.Initialize(fv);
                recS = true;
                if (commonPoint.IsVertex)
                {
                    // One goes directly by the Vertex
                    // And it is checked that there are no other candidates
                    Face aux = new();
                    int nb = SearchFaceOnV(commonPoint, f, _veMap, _efMap, ref fv, ref aux);

                    pons = Tool.Parameters(commonPoint.Vertex(), fv);
                    hs.Initialize(fv);
                    if (nb >= 2)
                    {
                        hsBis = new FaceSurfaceAdaptor(aux);
                        pBis = Tool.Parameters(commonPoint.Vertex(), aux);
                    }
                    return true;
                }
                // otherwise one passes by the arc...
                if (!fv.IsSame(f))
                {
                    fv.Orient(Forward);
                    Edge newedge = new();
                    for (Explorer ex = new(fv, ShapeType.Edge); ex.More(); ex.Next())
                    {
                        if (ex.Current().IsSame(arcEdge))
                        {
                            newedge = ShapeCasts.Edge(ex.Current());
                            break;
                        }
                    }
                    //  In cas of Tangent output, the current face becomes the support face
                    if (IsTangentToArc(commonPoint, 0.1))
                    {
                        pCurve = GetCurveOnSurface(commonPoint.Arc(), f);
                        if (pCurve is null)
                        {
                            prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                            return false;
                        }
                        hsBis = new FaceSurfaceAdaptor(f);
                        pBis = pCurve.Value(commonPoint.ParameterOnArc());
                    }

                    if (newedge.IsNull)
                    {
                        prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                        return false;
                    }
                    pCurve = GetCurveOnSurface(newedge, fv);
                }
                else
                {
                    Edge newedge = arcEdge;
                    newedge.Reverse();
                    fv.Orient(Forward);
                    pCurve = GetCurveOnSurface(newedge, fv);
                }
                if (pCurve is null)
                {
                    prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                    return false;
                }
                pons = pCurve.Value(commonPoint.ParameterOnArc());
                return true;
            }
            else if (!fv.IsNull)
            {
                c1obstacle = true;
                if (!vref.IsNull)
                {
                    for (Explorer ex = new(arcEdge, ShapeType.Vertex); ex.More(); ex.Next())
                    {
                        if (ex.Current().IsSame(vref))
                        {
                            c1obstacle = false;
                            break;
                        }
                    }
                }
                if (c1obstacle)
                {
                    if (hSref is null || hCref is null)
                    {
                        prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                        return false;
                    }
                    hs.Initialize(fv);
                    hSref.Initialize(f);
                    w = commonPoint.ParameterOnArc();
                    hc = new EdgeCurve2DAdaptor();
                    Edge newedge = new();
                    Face newface = fv;
                    newface.Orient(Forward);
                    for (Explorer ex = new(newface, ShapeType.Edge); ex.More(); ex.Next())
                    {
                        if (ex.Current().IsSame(arcEdge))
                        {
                            newedge = ShapeCasts.Edge(ex.Current());
                            break;
                        }
                    }

                    if (!newedge.IsNull)
                    {
                        hc.Initialize(newedge, fv);
                        pons = hc.Value(w);
                        hCref.Initialize(arcEdge, f);
                        if (commonPoint.IsVertex)
                        {
                            recP = true;
                        }
                        else
                        {
                            recRst = true;
                        }
                        return true;
                    }
                    else
                    {
                        prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                        return false;
                    }
                }
                else
                {
                    prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                    return false;
                }
            }
            else
            {
                // there is no neighbor face, the state is preserved and False is returned.
                prepareDefaultReturn(arcEdge, f, hs, commonPoint, ref pCurve, ref w, ref pons);
                return false;
            }
        }
        else
        {
            hs.Initialize(f);
            FaceInterference FI = sd.Interference(ons);
            if (FI.PCurveOnFace() is null)
            {
                pons = sd.Get2dPoints(isfirst, ons);
            }
            else
            {
                pons = FI.PCurveOnFace().Value(FI.Parameter(isfirst));
            }
        }
        return true;
    }

    //=================================================================================================

    internal bool SearchFace(Spine? spine, in CommonPoint pc, in Face fRef, ref Face fVoi)
    {
        bool trouve = false;
        if (!pc.IsOnArc)
        {
            return false;
        }
        fVoi.Nullify();
        Edge e = new();
        if (pc.IsVertex)
        {
            // attention it is necessary to analyze all faces that turn around of the vertex
            if (pc.HasVector)
            { // General processing
                Face fbis = new();
                int nb_faces = 0;
                nb_faces = SearchFaceOnV(pc, fRef, _veMap, _efMap, ref fVoi, ref fbis);
                return (nb_faces > 0);
            }
            else
            { // Processing using the spine
                bool findFace = false;
                Point3D point = new();
                Vector3D vecSpine = new();
                spine.D1(pc.Parameter(), out point, out vecSpine);

                // It is checked if one leaves from the current face.
                findFace = IsInput(vecSpine, pc.Vertex(), fRef);
                if (findFace)
                {
                    vecSpine = vecSpine.Reversed();
                    findFace = IsInput(vecSpine, pc.Vertex(), fRef);
                }
                // If one does not leave, it is ended
                if (findFace)
                {
                    fVoi = fRef;
                    return true;
                }

                // Otherwise one finds the next among shared Faces
                // by a common edge G1
                NCollection_List<Shape>.Iterator itE = new(), itF = new();
                for (itE.Initialize(_veMap[pc.Vertex()]); itE.More() && (!findFace); itE.Next())
                {
                    e = ShapeCasts.Edge(itE.Value());
                    trouve = false;
                    for (itF.Initialize(_efMap[e]); //, Trouve=false;           15.11.99 SVV
                              itF.More() && (!trouve);
                              itF.Next())
                    {
                        if (ShapeCasts.Face(itF.Value()).IsSame(fRef))
                        {
                            trouve = true;
                        }
                    }
                    if (trouve)
                    {
                        findFace = IsG1(_efMap, e, fRef, ref fVoi);
                    }
                    if (findFace)
                    {
                        findFace = false;
                        if (spine is null)
                        {
                            // La Spine peut etre nulle (ThreeCorner)
                            return false;
                        }

                        // It is checked if the selected face actually possesses edges of the spine
                        // containing the vertex on its front
                        // This processing should go only if the Vertex belongs to the spine
                        // This is a single case, for other vertexes it is required to do other things
                        trouve = false;
                        for (int ie = 1; //, Trouve=false;                   15.11.99  SVV
                                  (ie <= spine.EdgeCount) && (!trouve);
                                  ie++)
                        {
                            e = spine.Edges(ie);
                            if ((Exploration.FirstVertex(e).IsSame(pc.Vertex()))
                                    || (Exploration.LastVertex(e).IsSame(pc.Vertex())))
                            {
                                for (itF.Initialize(_efMap[e]), trouve = false; itF.More() && (!trouve); itF.Next())
                                {
                                    if (ShapeCasts.Face(itF.Value()).IsSame(fVoi))
                                    {
                                        trouve = true;
                                    }
                                }
                            }
                        }
                        findFace = trouve;
                    }
                }
            }
        }
        else
        {
            return IsG1(_efMap, pc.Arc(), fRef, ref fVoi);
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

    private void PerformSetOfSurfOnElSpine(ElSpine? hGuide, Stripe? stripe, BrepTopolTool? it1, BrepTopolTool? it2, bool simul = false)
    {
        ElSpine guide = hGuide; // a reference in OCCT

        ElSpine? offsetHGuide = null;
        ref Spine? spine = ref stripe.ChangeSpine();
        if (spine.Mode() == ChamfMode.ConstThroatWithPenetrationChamfer)
        {
            NCollection_List<ElSpine> ll = spine.ChangeElSpines();
            NCollection_List<ElSpine> ll_offset = spine.ChangeOffsetElSpines();
            NCollection_List<ElSpine>.Iterator iles = new(ll), ILES_offset = new(ll_offset);
            for (; iles.More(); iles.Next(), ILES_offset.Next())
            {
                ElSpine? hElSpine = iles.Value();
                if (hElSpine == hGuide)
                {
                    offsetHGuide = ILES_offset.Value();
                }
            }
        }

        double wf = guide.FirstParameter();
        double wl = guide.LastParameter();
        double locfleche = (wl - wf) * _fleche;
        double wfsav = wf, wlsav = wl;
        if (!guide.IsPeriodic)
        {
            // Now the ElSpine is artificially extended to help rsnld.
            double prab = 0.01;
            guide.FirstParameter(wf - prab * (wl - wf));
            guide.LastParameter(wl + prab * (wl - wf));
            if (offsetHGuide is not null)
            {
                offsetHGuide.FirstParameter(wf - prab * (wl - wf));
                offsetHGuide.LastParameter(wl + prab * (wl - wf));
            }
        }
        // occ::handle<ChFiDS_Spine>&  Spine = Stripe->ChangeSpine();
        int ii = 0, nbed = spine.EdgeCount;
        double lastedlastp = spine.LastParameter(nbed);

        DsDataStructure dStr = _ds.ChangeDS();

        SurfData @ref = guide.Previous(); // "ref" in OCCT
        SurfData? refbis = null, sd = null;
        SurfData? raf = guide.Next();
        RemoveSD(stripe, @ref, raf);

        FaceSurfaceAdaptor? hs1 = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hs2 = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hs3 = null, hs4 = null;
        FaceSurfaceAdaptor? hSref1 = new FaceSurfaceAdaptor();
        FaceSurfaceAdaptor? hSref2 = new FaceSurfaceAdaptor();
        EdgeCurve2DAdaptor? hc1 = null, hc2 = null;
        EdgeCurve2DAdaptor? hCref1 = new EdgeCurve2DAdaptor();
        EdgeCurve2DAdaptor? hCref2 = new EdgeCurve2DAdaptor();
        bool decroch1 = false, decroch2 = false;
        bool recP1 = false, recS1 = false, recRst1 = false, obstacleon1 = false;
        bool recP2 = false, recS2 = false, recRst2 = false, obstacleon2 = false;
        Point2D pp1 = new(), pp2 = new(), pp3 = new(), pp4 = new();
        double w1 = 0.0, w2 = 0.0;
        math_Vector soldep = new(1, 4);
        math_Vector soldepCS = new(1, 3);
        math_Vector soldepCC = new(1, 2);

        // Restore a neighboring KPart.
        // If no neighbor calculation start point.
        bool forward = true;
        bool inside = false;
        double first = wf;
        double last = wl;
        bool ok1 = true, ok2 = true;
        // Restore the next KPart if it exists
        Vertex vref = new();
        if (@ref is null && raf is null)
        {
            // sinon solution approchee.
            inside = true;

            StartSol(stripe, hGuide, ref hs1, ref hs2, it1, it2, ref pp1, ref pp2, ref first);

            last = wf;
            if (guide.IsPeriodic)
            {
                last = first - guide.Period();
                guide.SaveFirstParameter();
                guide.FirstParameter(last);
                guide.SaveLastParameter();
                guide.LastParameter(first * 1.1); // Extension to help rsnld.
                if (offsetHGuide is not null)
                {
                    offsetHGuide.SaveFirstParameter();
                    offsetHGuide.FirstParameter(last);
                    offsetHGuide.SaveLastParameter();
                    offsetHGuide.LastParameter(first * 1.1); // Extension to help rsnld.
                }
            }
        }
        else
        {
            if (!spine.IsPeriodic && (wl - lastedlastp > -_tolesp))
            {
                vref = spine.LastVertex();
            }
            if (@ref is null)
            {
                if (!spine.IsPeriodic && (wf < _tolesp))
                {
                    vref = spine.FirstVertex();
                }
                @ref = raf;
                forward = false;
                first = wl;
                last = guide.FirstParameter();
            }

            ok1 = StartSol(spine, hs1, ref pp1, ref hc1, ref w1, @ref, !forward, 1, hSref1, hCref1, ref recP1, ref recS1,
                               ref recRst1, ref obstacleon1, ref hs3, ref pp3, decroch1, vref);
            ok2 = StartSol(spine, hs2, ref pp2, ref hc2, ref w2, @ref, !forward, 2, hSref2, hCref2, ref recP2, ref recS2,
                               ref recRst2, ref obstacleon2, ref hs4, ref pp4, decroch2, vref);
            hc1 = null;
            hc2 = null;

            if (ok1 && ok2)
            {
                if (forward)
                {
                    guide.FirstParameter(wf);
                    if (offsetHGuide is not null)
                    {
                        offsetHGuide.FirstParameter(wf);
                    }
                }
                else
                {
                    guide.LastParameter(wl);
                    if (offsetHGuide is not null)
                    {
                        offsetHGuide.LastParameter(wl);
                    }
                }
            }
        }
        bool fini = false;
        bool complete = inside;
        if (!guide.IsPeriodic)
        {
            int indf = spine.Index(wf);
            int indl = spine.Index(wl, false);
            if (spine.IsPeriodic && (indl < indf))
            {
                indl += nbed;
            }
            nbed = indl - indf + 1;
        }
        // No Max at the touch : 20 points by edge at average without
        // counting the extensions.

        double bidf = wf, bidl = wl;
        if (!spine.IsPeriodic)
        {
            bidf = Max(0.0, wf);
            bidl = Min(wl, spine.LastParameter(spine.EdgeCount));
            // PMN 20/07/98 : Attention in case if there is only extension
            if ((bidl - bidf) < 0.01 * spine.LastParameter(spine.EdgeCount))
            {
                bidf = wf;
                bidl = wl;
            }
        }
        double maxStep = (bidl - bidf) * 0.05 / nbed;
        double firstsov = 0.0;
        int intf = 0, intl = 0;
        while (!fini)
        {
            // are these the ends (no extension on periodic).
            ok1 = true;
            ok2 = true;
            if (!spine.IsPeriodic)
            {
                if (wf < _tolesp && (complete == inside))
                {
                    if (spine.FirstStatus() == State.OnSame)
                    {
                        intf = 2;
                    }
                    else
                    {
                        intf = 1;
                    }
                }
                if (spine.IsTangencyExtremity(true))
                {
                    intf = 4;
                    guide.FirstParameter(wfsav);
                    if (offsetHGuide is not null)
                    {
                        offsetHGuide.FirstParameter(wfsav);
                    }
                }
                if (wl - lastedlastp > -_tolesp)
                {
                    if (spine.LastStatus() == State.OnSame)
                    {
                        intl = 2;
                    }
                    else
                    {
                        intl = 1;
                    }
                }
                if (spine.IsTangencyExtremity(false))
                {
                    intl = 4;
                    guide.LastParameter(wlsav);
                    if (offsetHGuide is not null)
                    {
                        offsetHGuide.LastParameter(wlsav);
                    }
                }
            }
            if (intf != 0 && !forward)
            {
                vref = spine.FirstVertex();
            }
            if (intl != 0 && forward)
            {
                vref = spine.LastVertex();
            }
            if (@ref is not null)
            {
                ok1 = StartSol(spine, hs1, ref pp1, ref hc1, ref w1, @ref, !forward, 1, hSref1, hCref1, ref recP1, ref recS1,
                               ref recRst1, ref obstacleon1, ref hs3, ref pp3, decroch1, vref);
                ok2 = StartSol(spine, hs2, ref pp2, ref hc2, ref w2, @ref, !forward, 2, hSref2, hCref2, ref recP2, ref recS2,
                               ref recRst2, ref obstacleon2, ref hs4, ref pp4, decroch2, vref);
            }

            // No more connected faces. Construction of the tangent plane to continue the path
            // till the output on the other face.
            if ((!ok1 && hc1 is null) || (!ok2 && hc2 is null))
            {
                if ((intf != 0 && !forward) || (intl != 0 && forward))
                {
                    if (!ok1)
                    {
                        ChFi3d_BuildPlane(dStr, hs1, ref pp1, @ref, !forward, 1);
                    }
                    if (!ok2)
                    {
                        ChFi3d_BuildPlane(dStr, hs2, ref pp2, @ref, !forward, 2);
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
                        guide.FirstParameter(wf);
                        if (offsetHGuide is not null)
                        {
                            offsetHGuide.FirstParameter(wf);
                        }
                    }
                    else
                    {
                        guide.LastParameter(wl);
                        if (offsetHGuide is not null)
                        {
                            offsetHGuide.LastParameter(wl);
                        }
                    }
                }
                else
                {
                    throw new FailureException("PerformSetOfSurfOnElSpine : Chaining is impossible.");
                }
            }

            // Definition of the domain of patch It1, It2
            SurfaceAdaptorBase? hSon1 = hs1;// to avoid ambiguity
            SurfaceAdaptorBase? hSon2 = hs2;// to avoid ambiguity
            it1.Initialize(hSon1);
            it2.Initialize(hSon2);

            // Calculate one (several if singularity) SurfaData
            sd = new SurfData();
            NCollection_Sequence<SurfData> seqSD = new();
            seqSD.Append(sd);

            if (obstacleon1 && obstacleon2)
            {
                Orientation or1 = hSref1.Face().Orientation;
                Orientation or2 = hSref2.Face().Orientation;
                int choix = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2, stripe.OrientationOnFace1(), stripe.OrientationOnFace2(),
                                                                          stripe.Choix());

                // Calculate the criterion of Choice edge / edge
                if (choix % 2 == 0)
                {
                    choix = 4;
                }
                else
                {
                    choix = 1;
                }

                soldepCC[1] = w1;
                soldepCC[2] = w2;
                if (simul)
                {
                    SimulSurf(sd, hGuide, spine, choix, hs1, it1, hc1, hSref1, hCref1, ref decroch1, or1, hs2, it2, hc2, hSref2,
                                        hCref2, ref decroch2, or2, locfleche, _tolesp, ref first, ref last, inside, inside,
                                        forward, recP1, recRst1, recP2, recRst2, soldepCC);
                }
                else
                {
                    PerformSurf(seqSD, hGuide, spine, choix, hs1, it1, hc1, hSref1, hCref1, ref decroch1, or1, hs2, it2, hc2,
                                            hSref2, hCref2, ref decroch2, or2, maxStep, locfleche, _tolesp, ref first, ref last, inside,
                                            inside,
                                            forward, recP1, recRst1, recP2, recRst2, soldepCC);
                }
                sd.ChangeIndexOfS1(dStr.AddShape(hs1.Face()));
                sd.ChangeIndexOfS2(dStr.AddShape(hs2.Face()));
            }
            else if (obstacleon1)
            {
                Orientation or1 = hSref1.Face().Orientation;
                Orientation or2 = hs2.Face().Orientation;
                int choix = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2, stripe.OrientationOnFace1(), stripe.OrientationOnFace2(),
                                                                          -stripe.Choix());
                if (choix % 2 == 1)
                {
                    choix++;
                }
                else
                {
                    choix--;
                }
                soldepCS[3] = w1;
                soldepCS[1] = pp2.X;
                soldepCS[2] = pp2.Y;
                if (simul)
                {
                    SimulSurf(sd, hGuide, spine, choix, hs1, it1, hc1, hSref1, hCref1, ref decroch1, hs2, it2, or2, locfleche, _tolesp,
                                        ref first, ref last, inside, inside,
                                        forward, recP1, recS2, recRst1, soldepCS);
                }
                else
                {
                    PerformSurf(seqSD, hGuide, spine, choix, hs1, it1, hc1, hSref1, hCref1, ref decroch1, hs2, it2, or2, maxStep,
                                            locfleche, _tolesp, ref first, ref last, inside, inside,
                                            forward, recP1, recS2, recRst1, soldepCS);
                }
                sd.ChangeIndexOfS1(dStr.AddShape(hs1.Face()));
                sd.ChangeIndexOfS2(dStr.AddShape(hs2.Face()));
                decroch2 = false;
            }
            else if (obstacleon2)
            {
                Orientation or1 = hs1.Face().Orientation;
                Orientation or2 = hSref2.Face().Orientation;
                int choix = global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2, stripe.OrientationOnFace1(), stripe.OrientationOnFace2(),
                                                                          stripe.Choix());
                soldepCS[3] = w2;
                soldepCS[1] = pp1.X;
                soldepCS[2] = pp1.Y;
                if (simul)
                {
                    SimulSurf(sd, hGuide, spine, choix, hs1, it1, or1, hs2, it2, hc2, hSref2, hCref2, ref decroch2, locfleche, _tolesp,
                                        ref first, ref last, inside, inside,
                                        forward, recP2, recS1, recRst2, soldepCS);
                }
                else
                {
                    PerformSurf(seqSD, hGuide, spine, choix, hs1, it1, or1, hs2, it2, hc2, hSref2, hCref2, ref decroch2, maxStep,
                                            locfleche, _tolesp, ref first, ref last, inside, inside,
                                            forward, recP2, recS1, recRst2, soldepCS);
                }
                sd.ChangeIndexOfS1(dStr.AddShape(hs1.Face()));
                sd.ChangeIndexOfS2(dStr.AddShape(hs2.Face()));
                decroch1 = false;
            }
            else
            {
                TopolTool? tt1 = it1;// to avoid ambiguity
                TopolTool? tt2 = it2;// to avoid ambiguity
                CallPerformSurf(stripe, simul, seqSD, sd, hGuide, spine, hs1, hs3, pp1, pp3, tt1, hs2, hs4, pp2, pp4, tt2,
                                                maxStep, locfleche, _tolesp, ref first, ref last, inside, inside,
                                                forward, recS1, recS2, soldep, ref intf, ref intl, ref hs1, ref hs2);
                decroch1 = decroch2 = false;
            }

            if (!_done)
            { // Case of fail
                if ((!ok1 && !obstacleon1) || (!ok2 && !obstacleon2))
                {
                    // Fail in a part of extension is not serious
                    // Here one stops.
                    _done = true;
                    inside = false;
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
                    spine.SetErrorStatus(ErrorStatus.WalkingFailure);
                    throw new FailureException("CallPerformSurf : Path failed!");
                }
            }

            else
            {
                refbis = @ref;
                if (forward)
                {
                    for (ii = 1; ii <= seqSD.Length(); ii++)
                    {
                        sd = seqSD[ii];
                        sd.ChangeIndexOfS1(dStr.AddShape(hs1.Face()));
                        if (obstacleon1)
                        {
                            sd.SetIndexOfC1(dStr.AddShape(hc1.Edge()));
                        }
                        sd.ChangeIndexOfS2(dStr.AddShape(hs2.Face()));
                        if (obstacleon2)
                        {
                            sd.SetIndexOfC2(dStr.AddShape(hc2.Edge()));
                        }
                        InsertAfter(stripe, refbis, sd);
                        refbis = sd;
                    }
                }
                else
                {
                    for (ii = seqSD.Length(); ii >= 1; ii--)
                    {
                        sd = seqSD[ii];
                        sd.ChangeIndexOfS1(dStr.AddShape(hs1.Face()));
                        if (obstacleon1)
                        {
                            sd.SetIndexOfC1(dStr.AddShape(hc1.Edge()));
                        }
                        sd.ChangeIndexOfS2(dStr.AddShape(hs2.Face()));
                        if (obstacleon2)
                        {
                            sd.SetIndexOfC2(dStr.AddShape(hc2.Edge()));
                        }
                        InsertBefore(stripe, refbis, sd);
                        refbis = sd;
                    }
                }

                if (!ok1 && !obstacleon1)
                {
                    // clean infos on the plane of extension.
                    ChFi3d_Purge(stripe, sd, @ref.Vertex(!forward, 1), !forward, 1, ref intf, ref intl);
                }

                if (!ok2 && !obstacleon2)
                {
                    // clean infos on the plane of extension.
                    ChFi3d_Purge(stripe, sd, @ref.Vertex(!forward, 2), !forward, 2, ref intf, ref intl);
                }

                // The end. The reference is changed.
                @ref = refbis;
            }

            if (inside)
            { // There are starting solutions for the next.
                inside = false;
                firstsov = first;
                if (guide.IsPeriodic)
                {
                    complete = false;
                    wf = guide.FirstParameter();
                    wl = guide.LastParameter();
                }
            }
            if (forward)
            {
                fini =
                    ((wl - last) <= 10.0 * _tolesp || (intl != 0 && !(obstacleon1 || obstacleon2))); // General case

                if (!fini && guide.IsPeriodic && ((wl - last) < guide.Period() * 1.0e-3))
                {
                    // It is tested if reframing of extremes is done at the same edge
                    // Loop Condition
                    SurfData? thefirst = null, thelast = null;
                    thefirst = stripe.SetOfSurfData().First();
                    thelast = stripe.SetOfSurfData().Last();

                    if (thefirst.VertexFirstOnS1().IsOnArc && thelast.VertexLastOnS1().IsOnArc)
                    {
                        fini = thefirst.VertexFirstOnS1().Arc().IsSame(thelast.VertexLastOnS1().Arc());
                    }
                    if (!fini && thefirst.VertexFirstOnS2().IsOnArc && thelast.VertexLastOnS2().IsOnArc)
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
                    @ref = stripe.SetOfSurfData().First();
                    forward = false;
                    fini = false;
                    first = firstsov;
                }
                else
                {
                    first = last;
                    last = wl;
                }
            }
            if (!forward)
            {
                fini = ((first - wf) <= 10.0 * _tolesp || (intf != 0 && !(obstacleon1 || obstacleon2)));
                complete = false;
                last = wf;
            }
        }
        // The initial state is restored
        if (!guide.IsPeriodic)
        {
            guide.FirstParameter(wfsav);
            guide.LastParameter(wlsav);
            if (offsetHGuide is not null)
            {
                offsetHGuide.FirstParameter(wfsav);
                offsetHGuide.LastParameter(wlsav);
            }
        }
    }

    //=================================================================================================

    internal void PerformSetOfKPart(Stripe? stripe, bool simul = false)
    {
        DsDataStructure dStr = _ds.ChangeDS();
        ref Spine? spine = ref stripe.ChangeSpine();
        FaceSurfaceAdaptor? hs1 = null, hs2 = null;
        Orientation or1 = default, or2 = default, refOr1 = default, refOr2 = default;
        int refChoix = 0;

        // initialization of the stripe.
        stripe.Reset();
        ref NCollection_HSequence<SurfData>? hData = ref stripe.ChangeSetOfSurfData();
        hData = new NCollection_HSequence<SurfData>();
        NCollection_Sequence<SurfData> seqSurf = hData; // ChangeSequence(): the HSequence is the sequence

        StripeOrientations(spine, ref refOr1, ref refOr2, ref refChoix);
        stripe.OrientationOnFace1(refOr1);
        stripe.OrientationOnFace2(refOr2);
        stripe.Choix(refChoix);

        BrepTopolTool? it1 = new BrepTopolTool();
        BrepTopolTool? it2 = new BrepTopolTool();

        double wFirst = 0.0, wLast = 0.0;
        Vector3D tFirst = new(), tLast = new(), tEndPeriodic = new();
        Point3D pFirst = new(), pLast = new(), pEndPeriodic = new();
        bool intf = false, intl = false;

        // OCCT copies two default ElSpines into the handles: new default ones here
        ElSpine? currentHE = new ElSpine();
        ElSpine? currentOffsetHE = new ElSpine();
        spine.D1(spine.FirstParameter(), out pFirst, out tFirst);
        currentHE.FirstParameter(spine.FirstParameter());
        currentHE.SetFirstPointAndTgt(pFirst, tFirst);
        currentOffsetHE.FirstParameter(spine.FirstParameter());
        currentOffsetHE.SetFirstPointAndTgt(pFirst, tFirst);

        bool yaKPart = false;
        int iedgelastkpart = 0;

        double wStartPeriodic = 0.0;
        double wEndPeriodic = spine.LastParameter(spine.EdgeCount);
        spine.D1(wEndPeriodic, out pEndPeriodic, out tEndPeriodic);

        // Construction of particular cases.

        for (int iedge = 1; iedge <= spine.EdgeCount; iedge++)
        {

            ConexFaces(spine, iedge, ref hs1, ref hs2);

            if (ChFi3d_KParticular(spine, iedge, hs1, hs2))
            {
                intf = ((iedge == 1) && !spine.IsPeriodic);
                intl = ((iedge == spine.EdgeCount) && !spine.IsPeriodic);
                or1 = hs1.Face().Orientation;
                or2 = hs2.Face().Orientation;
                global::Kerf.Engine.Fillets.ChFi3d.NextSide(ref or1, ref or2, refOr1, refOr2, refChoix);
                SurfaceAdaptorBase? hSon1 = hs1;// to avoid ambiguity
                SurfaceAdaptorBase? hSon2 = hs2;// to avoid ambiguity
                it1.Initialize(hSon1);
                it2.Initialize(hSon2);

                SurfData? sd = new SurfData();
                NCollection_Sequence<SurfData> lsd = new();

                if (!global::Kerf.Engine.Fillets.Analytic.ComputeData.Compute(dStr, sd, hs1, hs2, or1, or2, spine, iedge))
                {
                }
                else if (!SplitKPart(sd, lsd, spine, iedge, hs1, it1, hs2, it2, ref intf, ref intl))
                {
                    lsd.Clear();
                }
                else
                {
                    iedgelastkpart = iedge;
                }
                if (spine.IsPeriodic)
                { // debug provisory for SD that arrive in disorder.
                    int nbsd = lsd.Length();
                    double period = spine.Period();
                    double wfp = wStartPeriodic, wlp = wEndPeriodic;
                    //  modified by NIZHNY-EAP Thu Nov 25 12:57:53 1999 ___BEGIN___
                    if (!yaKPart && nbsd > 0)
                    {
                        //	if(!YaKPart){
                        //  modified by NIZHNY-EAP Thu Nov 25 12:57:57 1999 ___END___
                        SurfData? firstSD = lsd.ChangeValue(1);
                        double wwf = firstSD.FirstSpineParam();
                        double wwl = firstSD.LastSpineParam();
                        wwf = ChFi3d_InPeriod(wwf, wfp, wlp, _tolesp);
                        wwl = ChFi3d_InPeriod(wwl, wfp, wlp, _tolesp);
                        if (wwl <= wwf + _tolesp)
                        {
                            wwl += period;
                        }
                        wfp = wwf;
                        wlp = wfp + period;
                    }
                    for (int j = 1; j < nbsd; j++)
                    {
                        SurfData? jSD = lsd.Value(j);
                        for (int k = j + 1; k <= nbsd; k++)
                        {
                            SurfData? kSD = lsd.Value(k);
                            double jwf = jSD.FirstSpineParam();
                            jwf = ChFi3d_InPeriod(jwf, wfp, wlp, _tolesp);
                            double kwf = kSD.FirstSpineParam();
                            kwf = ChFi3d_InPeriod(kwf, wfp, wlp, _tolesp);
                            if (kwf < jwf)
                            {
                                lsd.SetValue(j, kSD);
                                lsd.SetValue(k, jSD);
                            }
                        }
                    }
                }
                NCollection_List<int> li = new();
                for (int j = 1; j <= lsd.Length(); j++)
                {
                    ref SurfData? curSD = ref lsd.ChangeValue(j);
                    if (simul)
                    {
                        SimulKPart(curSD);
                    }
                    seqSurf.Append(curSD);
                    if (!simul)
                    {
                        li.Append(curSD.Surf());
                    }
                    wFirst = lsd.Value(j).FirstSpineParam();
                    wLast = lsd.Value(j).LastSpineParam();
                    if (spine.IsPeriodic)
                    {
                        wFirst = ChFi3d_InPeriod(wFirst, wStartPeriodic, wEndPeriodic, _tolesp);
                        wLast = ChFi3d_InPeriod(wLast, wStartPeriodic, wEndPeriodic, _tolesp);
                        if (wLast <= wFirst + _tolesp)
                        {
                            wLast += spine.Period();
                        }
                    }
                    TgtKP(lsd.Value(j), spine, iedge, true, ref pFirst, ref tFirst);
                    TgtKP(lsd.Value(j), spine, iedge, false, ref pLast, ref tLast);

                    // Determine the sections to approximate
                    if (!yaKPart)
                    {
                        if (spine.IsPeriodic)
                        {
                            wStartPeriodic = wFirst;
                            wEndPeriodic = wStartPeriodic + spine.Period();
                            wLast = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wLast, wStartPeriodic, wEndPeriodic);
                            if (wLast <= wFirst + _tolesp)
                            {
                                wLast += spine.Period();
                            }
                            pEndPeriodic = pFirst;
                            tEndPeriodic = tFirst;
                            spine.SetFirstParameter(wStartPeriodic);
                            spine.SetLastParameter(wEndPeriodic);
                        }
                        else if (!intf || (iedge > 1))
                        {
                            // start section -> first KPart
                            // update of extension.
                            spine.SetFirstTgt(Min(0.0, wFirst));
                            currentHE.LastParameter(wFirst);
                            currentHE.SetLastPointAndTgt(pFirst, tFirst);
                            spine.AppendElSpine(currentHE);
                            currentHE.ChangeNext() = lsd.Value(j);
                            currentHE = new ElSpine();

                            currentOffsetHE.LastParameter(wFirst);
                            currentOffsetHE.SetLastPointAndTgt(pFirst, tFirst);
                            spine.AppendOffsetElSpine(currentOffsetHE);
                            currentOffsetHE.ChangeNext() = lsd.Value(j);
                            currentOffsetHE = new ElSpine();
                        }
                        currentHE.FirstParameter(wLast);
                        currentHE.SetFirstPointAndTgt(pLast, tLast);
                        currentHE.ChangePrevious() = lsd.Value(j);
                        currentOffsetHE.FirstParameter(wLast);
                        currentOffsetHE.SetFirstPointAndTgt(pLast, tLast);
                        currentOffsetHE.ChangePrevious() = lsd.Value(j);
                        yaKPart = true;
                    }
                    else
                    {
                        if (wFirst - currentHE.FirstParameter() > _tolesp)
                        {
                            // section between two KPart
                            currentHE.LastParameter(wFirst);
                            currentHE.SetLastPointAndTgt(pFirst, tFirst);
                            spine.AppendElSpine(currentHE);
                            currentHE.ChangeNext() = lsd.Value(j);
                            currentHE = new ElSpine();

                            currentOffsetHE.LastParameter(wFirst);
                            currentOffsetHE.SetLastPointAndTgt(pFirst, tFirst);
                            spine.AppendOffsetElSpine(currentOffsetHE);
                            currentOffsetHE.ChangeNext() = lsd.Value(j);
                            currentOffsetHE = new ElSpine();
                        }
                        currentHE.FirstParameter(wLast);
                        currentHE.SetFirstPointAndTgt(pLast, tLast);
                        currentHE.ChangePrevious() = lsd.Value(j);
                        currentOffsetHE.FirstParameter(wLast);
                        currentOffsetHE.SetFirstPointAndTgt(pLast, tLast);
                        currentOffsetHE.ChangePrevious() = lsd.Value(j);
                    }
                }
                if (!li.IsEmpty())
                {
                    _eviMap[spine.Edges(iedge)] = li;
                }
            }
        }

        if (!intl || (iedgelastkpart < spine.EdgeCount))
        {
            // section last KPart(or start of the spine) -> End of the spine.
            // update of the extension.

            if (spine.IsPeriodic)
            {
                if (wEndPeriodic - wLast > _tolesp)
                {
                    currentHE.LastParameter(wEndPeriodic);
                    currentHE.SetLastPointAndTgt(pEndPeriodic, tEndPeriodic);
                    if (!yaKPart)
                    {
                        currentHE.SetPeriodic(true);
                    }
                    spine.AppendElSpine(currentHE);

                    currentOffsetHE.LastParameter(wEndPeriodic);
                    currentOffsetHE.SetLastPointAndTgt(pEndPeriodic, tEndPeriodic);
                    if (!yaKPart)
                    {
                        currentOffsetHE.SetPeriodic(true);
                    }
                    spine.AppendOffsetElSpine(currentOffsetHE);
                }
            }
            else
            {
                spine.D1(spine.LastParameter(), out pLast, out tLast);
                spine.SetLastTgt(Max(spine.LastParameter(spine.EdgeCount), wLast));
                if (spine.LastParameter() - wLast > _tolesp)
                {
                    currentHE.LastParameter(spine.LastParameter());
                    currentHE.SetLastPointAndTgt(pLast, tLast);
                    spine.AppendElSpine(currentHE);

                    currentOffsetHE.LastParameter(spine.LastParameter());
                    currentOffsetHE.SetLastPointAndTgt(pLast, tLast);
                    spine.AppendOffsetElSpine(currentOffsetHE);
                }
            }
        }

        NCollection_List<ElSpine> ll = spine.ChangeElSpines();
        NCollection_List<ElSpine>.Iterator iles = new(ll);
        for (; iles.More(); iles.Next())
        {
            _guidesSmoothed += ChFi3d_PerformElSpine(iles.ChangeValue(), spine, _conti, _tolesp, guideTolerance: _guideTolerance);
        }
        if (spine.Mode() == ChamfMode.ConstThroatWithPenetrationChamfer)
        {
            NCollection_List<ElSpine> offsetll = spine.ChangeOffsetElSpines();
            for (iles.Initialize(offsetll); iles.More(); iles.Next())
            {
                _guidesSmoothed += ChFi3d_PerformElSpine(iles.ChangeValue(), spine, _conti, _tolesp, true, _guideTolerance);
            }
        }
        spine.SplitDone(true);
    }

    //=================================================================================================

    internal void PerformSetOfKGen(Stripe? stripe, bool simul = false)
    {
        BrepTopolTool? it1 = new BrepTopolTool();
        BrepTopolTool? it2 = new BrepTopolTool();
        ref Spine? spine = ref stripe.ChangeSpine();
        NCollection_List<ElSpine> ll = spine.ChangeElSpines();
        NCollection_List<ElSpine>.Iterator iles = new(ll);
        for (; iles.More(); iles.Next())
        {
            PerformSetOfSurfOnElSpine(iles.Value(), stripe, it1, it2, simul);
        }
        if (!simul)
        {
            DsDataStructure dStr = _ds.ChangeDS();
            ref NCollection_HSequence<SurfData>? hData = ref stripe.ChangeSetOfSurfData();
            NCollection_Sequence<SurfData> seqSurf = hData; // ChangeSequence()
            int len = seqSurf.Length();
            int last = len, i;
            bool periodic = spine.IsPeriodic;
            if (periodic)
            {
                last++;
            }
            // It is attempted to reprocess the squares that bore.
            for (i = 1; i <= len; i++)
            {
                ref SurfData? cursd = ref seqSurf.ChangeValue(i);
                bool tw1 = cursd.TwistOnS1();
                bool tw2 = cursd.TwistOnS2();
                SurfData? prevsd = null, nextsd = null;
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
                ref FaceInterference intf1 = ref cursd.ChangeInterferenceOnS1();
                ref FaceInterference intf2 = ref cursd.ChangeInterferenceOnS2();
                int cursurf1 = cursd.IndexOfS1();
                int cursurf2 = cursd.IndexOfS2();
                ref CommonPoint cpd1 = ref cursd.ChangeVertexFirstOnS1();
                ref CommonPoint cpd2 = ref cursd.ChangeVertexFirstOnS2();
                ref CommonPoint cpf1 = ref cursd.ChangeVertexLastOnS1();
                ref CommonPoint cpf2 = ref cursd.ChangeVertexLastOnS2();
                Point3D pd1 = cpd1.Point();
                Point3D pd2 = cpd2.Point();
                Point3D pf1 = cpf1.Point();
                Point3D pf2 = cpf2.Point();
                double ddeb = pd1.DistanceTo(pd2);
                double dfin = pf1.DistanceTo(pf2);
                double don1 = pd1.DistanceTo(pf1);
                double don2 = pd2.DistanceTo(pf2);
                bool possibleon1 = (don1 < 2 * (ddeb + dfin));
                bool possibleon2 = (don2 < 2 * (ddeb + dfin));
                if ((tw1 && !possibleon1) || (tw2 && !possibleon2))
                {
                    spine.SetErrorStatus(ErrorStatus.TwistedSurface);
                    throw new FailureException("adjustment by reprocessing the non-written points");
                }

                // It is checked if there are presentable neighbors
                bool yaprevon1 = false, yaprevon2 = false;
                bool samesurfon1 = false, samesurfon2 = false;
                if (iprev != 0)
                {
                    prevsd = seqSurf.ChangeValue(iprev);
                    yaprevon1 = !prevsd.TwistOnS1();
                    samesurfon1 = (prevsd.IndexOfS1() == cursurf1);
                    yaprevon2 = !prevsd.TwistOnS2();
                    samesurfon2 = (prevsd.IndexOfS2() == cursurf2);
                }
                bool yanexton1 = false, yanexton2 = false;
                if (inext != 0)
                {
                    nextsd = seqSurf.ChangeValue(inext);
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
                Curve2D? pc1 = intf1.PCurveOnFace();
                Curve2D? pc2 = intf2.PCurveOnFace();
                FaceSurfaceAdaptor? s1 = new FaceSurfaceAdaptor();
                Face f1 = ShapeCasts.Face(dStr.Shape(cursurf1));
                s1.Initialize(f1);
                FaceSurfaceAdaptor? s2 = new FaceSurfaceAdaptor();
                Face f2 = ShapeCasts.Face(dStr.Shape(cursurf2));
                s2.Initialize(f2);
                Boundary? bdeb = null, bfin = null, bon1 = null, bon2 = null;
                bool pointuon1 = false, pointuon2 = false;
                if (tw1)
                {
                    if (!yaprevon1 || !yanexton1)
                    {
                        spine.SetErrorStatus(ErrorStatus.TwistedSurface);
                        throw new FailureException("adjustment by reprocessing the non-written points: no neighbor");
                    }
                    ref FaceInterference previntf1_ = ref prevsd.ChangeInterferenceOnS1();
                    ref FaceInterference nextintf1_ = ref nextsd.ChangeInterferenceOnS1();
                    double prevpar1_ = previntf1_.LastParameter();
                    double nextpar1_ = nextintf1_.FirstParameter();
                    if (samesurfon1)
                    {
                        // It is checked if it is possible to intersect traces of neighbors
                        // to create a sharp end.
                        Curve2D? pcprev1 = previntf1_.PCurveOnFace();
                        Curve2D? pcnext1 = nextintf1_.PCurveOnFace();
                        double nprevpar1 = 0.0, nnextpar1 = 0.0;
                        Point2D p2d = new();
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
                            pc1 = null;
                        }
                        else
                        {
                            Point2D pdeb1 = new(), pfin1 = new();
                            Vector2D vdeb1 = new(), vfin1 = new();
                            pcprev1.D1(prevpar1_, out pdeb1, out vdeb1);
                            pcnext1.D1(nextpar1_, out pfin1, out vfin1);
                            bon1 = ChFi3d_mkbound(s1, out pc1, -1, pdeb1, vdeb1, 1, pfin1, vfin1, _tolapp3d, 2.0e-4);
                        }
                    }
                    else
                    {
                        // here the base is on 3D tangents of neighbors.
                        Curve3D? c3dprev1 = dStr.Curve(previntf1_.LineIndex()).Curve();
                        Curve3D? c3dnext1 = dStr.Curve(nextintf1_.LineIndex()).Curve();
                        Point3D Pdeb1 = new(), Pfin1 = new();
                        Vector3D Vdeb1 = new(), Vfin1 = new();
                        c3dprev1.D1(prevpar1_, out Pdeb1, out Vdeb1);
                        c3dnext1.D1(nextpar1_, out Pfin1, out Vfin1);
                        Point2D pdeb1 = new(), pfin1 = new();
                        double pardeb1 = intf1.FirstParameter();
                        double parfin1 = intf1.LastParameter();
                        pdeb1 = pc1.Value(pardeb1);
                        pfin1 = pc1.Value(parfin1);
                        bon1 = ChFi3d_mkbound(s1, out pc1, -1, pdeb1, ref Vdeb1, 1, pfin1, ref Vfin1, _tolapp3d, 2.0e-4);
                    }
                }
                else
                {
                    bon1 = ChFi3d_mkbound(s1, pc1, _tolapp3d, 2.0e-4);
                }
                if (tw2)
                {
                    if (!yaprevon2 || !yanexton2)
                    {
                        throw new FailureException("adjustment by reprocessing the non-written points: no neighbor");
                    }
                    ref FaceInterference previntf2_ = ref prevsd.ChangeInterferenceOnS2();
                    ref FaceInterference nextintf2_ = ref nextsd.ChangeInterferenceOnS2();
                    double prevpar2_ = previntf2_.LastParameter();
                    double nextpar2_ = nextintf2_.FirstParameter();
                    if (samesurfon2)
                    {
                        // It is checked if it is possible to intersect traces of neighbors
                        // to create a sharp end.
                        Curve2D? pcprev2 = previntf2_.PCurveOnFace();
                        Curve2D? pcnext2 = nextintf2_.PCurveOnFace();
                        double nprevpar2 = 0.0, nnextpar2 = 0.0;
                        Point2D p2d = new();
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
                            pc2 = null;
                        }
                        else
                        {
                            Point2D pdeb2 = new(), pfin2 = new();
                            Vector2D vdeb2 = new(), vfin2 = new();
                            pcprev2.D1(prevpar2_, out pdeb2, out vdeb2);
                            pcnext2.D1(nextpar2_, out pfin2, out vfin2);
                            bon2 = ChFi3d_mkbound(s2, out pc2, -1, pdeb2, vdeb2, 1, pfin2, vfin2, _tolapp3d, 2.0e-4);
                        }
                    }
                    else
                    {
                        // here the base is on 3D tangents of neighbors.
                        Curve3D? c3dprev2 = dStr.Curve(previntf2_.LineIndex()).Curve();
                        Curve3D? c3dnext2 = dStr.Curve(nextintf2_.LineIndex()).Curve();
                        Point3D Pdeb2 = new(), Pfin2 = new();
                        Vector3D Vdeb2 = new(), Vfin2 = new();
                        c3dprev2.D1(prevpar2_, out Pdeb2, out Vdeb2);
                        c3dnext2.D1(nextpar2_, out Pfin2, out Vfin2);
                        Point2D pdeb2 = new(), pfin2 = new();
                        double pardeb2 = intf2.FirstParameter();
                        double parfin2 = intf2.LastParameter();
                        pdeb2 = pc2.Value(pardeb2);
                        pfin2 = pc2.Value(parfin2);
                        bon2 = ChFi3d_mkbound(s2, out pc2, -1, pdeb2, ref Vdeb2, 1, pfin2, ref Vfin2, _tolapp3d, 2.0e-4);
                    }
                }
                else
                {
                    bon2 = ChFi3d_mkbound(s2, pc2, _tolapp3d, 2.0e-4);
                }
                // The parameters of neighbor traces are updated, so
                // straight lines uv are pulled.
                Surface? sprev = dStr.Surface(prevsd.Surf()).Surface();
                Surface? snext = dStr.Surface(nextsd.Surf()).Surface();
                ref FaceInterference previntf1 = ref prevsd.ChangeInterferenceOnS1();
                ref FaceInterference nextintf1 = ref nextsd.ChangeInterferenceOnS1();
                ref FaceInterference previntf2 = ref prevsd.ChangeInterferenceOnS2();
                ref FaceInterference nextintf2 = ref nextsd.ChangeInterferenceOnS2();
                Curve2D? pcsprev1 = previntf1.PCurveOnSurf();
                Curve2D? pcsnext1 = nextintf1.PCurveOnSurf();
                double prevpar1 = previntf1.LastParameter();
                double nextpar1 = nextintf1.FirstParameter();
                Curve2D? pcsprev2 = previntf2.PCurveOnSurf();
                Curve2D? pcsnext2 = nextintf2.PCurveOnSurf();
                double prevpar2 = previntf2.LastParameter();
                double nextpar2 = nextintf2.FirstParameter();
                Point2D pdebs1 = pcsprev1.Value(prevpar1);
                Point2D pdebs2 = pcsprev2.Value(prevpar2);
                Point2D pfins1 = pcsnext1.Value(nextpar1);
                Point2D pfins2 = pcsnext2.Value(nextpar2);
                bdeb = ChFi3d_mkbound(sprev, pdebs1, pdebs2, _tolapp3d, 2.0e-4);
                bfin = ChFi3d_mkbound(snext, pfins1, pfins2, _tolapp3d, 2.0e-4);

                ConstrainedFilling fil = new(11, 20);
                if (pointuon1)
                {
                    fil.Init(bon2, bfin, bdeb, true);
                }
                else if (pointuon2)
                {
                    fil.Init(bon1, bfin, bdeb, true);
                }
                else
                {
                    fil.Init(bon1, bfin, bon2, bdeb, true);
                }

                ChFi3d_ReparamPcurv(0.0, 1.0, ref pc1);
                ChFi3d_ReparamPcurv(0.0, 1.0, ref pc2);
                Surface? newsurf = fil.InternalSurface(); // unfrozen: VReverse below (decision 1122)
                if (pointuon1)
                {
                    newsurf.VReverse(); // we return to direction 1 from  2;
                    _done = CompleteData(cursd, newsurf, s1, pc1, s2, pc2, f2.Orientation, false, false, false, false, false);
                    cursd.ChangeIndexOfS1(0);
                }
                else
                {
                    _done = CompleteData(cursd, newsurf, s1, pc1, s2, pc2, f1.Orientation, true, false, false, false, false);
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
                ref SurfData? cursd = ref seqSurf.ChangeValue(i);
                ref SurfData? nextsd = ref seqSurf.ChangeValue(j);
                ref CommonPoint curp1 = ref cursd.ChangeVertexLastOnS1();
                ref CommonPoint nextp1 = ref nextsd.ChangeVertexFirstOnS1();
                if (cursd.IsOnCurve1)
                {
                    curs1 = cursd.IndexOfC1();
                }
                else
                {
                    curs1 = cursd.IndexOfS1();
                }
                if (cursd.IsOnCurve2)
                {
                    curs2 = cursd.IndexOfC2();
                }
                else
                {
                    curs2 = cursd.IndexOfS2();
                }
                double tol1 = Max(curp1.Tolerance(), nextp1.Tolerance());
                ref CommonPoint curp2 = ref cursd.ChangeVertexLastOnS2();
                ref CommonPoint nextp2 = ref nextsd.ChangeVertexFirstOnS2();
                double tol2 = Max(curp2.Tolerance(), nextp2.Tolerance());
                if (nextsd.IsOnCurve1)
                {
                    nexts1 = nextsd.IndexOfC1();
                }
                else
                {
                    nexts1 = nextsd.IndexOfS1();
                }
                if (nextsd.IsOnCurve2)
                {
                    nexts2 = nextsd.IndexOfC2();
                }
                else
                {
                    nexts2 = nextsd.IndexOfS2();
                }

                if (!curp1.IsOnArc && nextp1.IsOnArc)
                {
                    curp1 = nextp1;
                    if ((curs1 == nexts1) && !nextsd.IsOnCurve1)
                    {
                        // Case when it is not possible to pass along the border without leaving
                        ChangeTransition(nextp1, ref curp1, nexts1, _ds);
                    }
                }
                else if (curp1.IsOnArc && !nextp1.IsOnArc)
                {
                    nextp1 = curp1;
                    if ((curs1 == nexts1) && !cursd.IsOnCurve1)
                    {
                        ChangeTransition(curp1, ref nextp1, curs1, _ds);
                    }
                }

                if (!curp2.IsOnArc && nextp2.IsOnArc)
                {
                    curp2 = nextp2;
                    if ((curs2 == nexts2) && !nextsd.IsOnCurve2)
                    {
                        ChangeTransition(nextp2, ref curp2, curs2, _ds);
                    }
                }
                else if (curp2.IsOnArc && !nextp2.IsOnArc)
                {
                    nextp2 = curp2;
                    if ((curs2 == nexts2) && !cursd.IsOnCurve2)
                    {
                        ChangeTransition(curp2, ref nextp2, curs2, _ds);
                    }
                }

                curp1.SetTolerance(tol1);
                nextp1.SetTolerance(tol1);
                curp2.SetTolerance(tol2);
                nextp2.SetTolerance(tol2);

                Box b1 = new(), b2 = new();
                if (curp1.IsOnArc)
                {
                    ChFi3d_EnlargeBox(curp1.Arc(), _efMap[curp1.Arc()], curp1.ParameterOnArc(), ref b1);
                }
                if (curp2.IsOnArc)
                {
                    ChFi3d_EnlargeBox(curp2.Arc(), _efMap[curp2.Arc()], curp2.ParameterOnArc(), ref b2);
                }
                Stripe? bidst = null;
                ChFi3d_EnlargeBox(dStr, bidst, cursd, ref b1, ref b2, false);
                ChFi3d_EnlargeBox(dStr, bidst, nextsd, ref b1, ref b2, true);
                tol1 = ChFi3d_BoxDiag(b1);
                tol2 = ChFi3d_BoxDiag(b2);
                curp1.SetTolerance(tol1);
                nextp1.SetTolerance(tol1);
                curp2.SetTolerance(tol2);
                nextp2.SetTolerance(tol2);
            }
            // The connections edge/new faces are updated.
            for (iles.Initialize(ll); iles.More(); iles.Next())
            {
                ElSpine? curhels = iles.Value();
                double wf = curhels.FirstParameter();
                double wl = curhels.LastParameter();
                int IF = 0, il = 0;
                double nwf = wf, nwl = wl;
                double period = 0.0;
                int nbed = spine.EdgeCount;
                if (periodic)
                {
                    period = spine.Period();
                    nwf = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wf, -_tolesp, period - _tolesp);
                    IF = spine.Index(nwf, true);
                    nwl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wl, _tolesp, period + _tolesp);
                    il = spine.Index(nwl, false);
                    if (nwl < nwf + _tolesp)
                    {
                        il += nbed;
                    }
                }
                else
                {
                    IF = spine.Index(wf, true);
                    il = spine.Index(wl, false);
                }
                if (IF == il)
                {
                    // fast processing
                    int iFloc = IF;
                    if (periodic)
                    {
                        iFloc = (IF - 1) % nbed + 1;
                    }
                    Edge ej = spine.Edges(iFloc);
                    for (i = 1; i <= len; i++)
                    {
                        ref SurfData? cursd = ref seqSurf.ChangeValue(i);
                        double fp = cursd.FirstSpineParam();
                        double lp = cursd.LastSpineParam();
                        if (lp < wf + _tolesp || fp > wl - _tolesp)
                        {
                            continue;
                        }
                        if (!_eviMap.ContainsKey(ej))
                        {
                            NCollection_List<int> li = new();
                            _eviMap[ej] = li;
                        }
                        _eviMap.GetValue(ej).Append(cursd.Surf());
                    }
                }
                else if (IF < il)
                {
                    NCollection_Array1<double> wv = new(IF, il - 1);
                    for (i = IF; i < il; i++)
                    {
                        int iloc = i;
                        if (periodic)
                        {
                            iloc = (i - 1) % nbed + 1;
                        }
                        double wi = spine.LastParameter(iloc);
                        if (periodic)
                        {
                            wi = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wi, wf, wf + period);
                        }
                        Point3D pv = spine.Value(wi);
                        LocateExtPC ext = new(pv, curhels, wi, 1.0e-8);
                        wv[i] = wi;
                        if (ext.IsDone)
                        {
                            wv[i] = ext.Point().Parameter();
                        }
                    }
                    for (i = 1; i <= len; i++)
                    {
                        ref SurfData? cursd = ref seqSurf.ChangeValue(i);
                        double fp = cursd.FirstSpineParam();
                        double lp = cursd.LastSpineParam();
                        int j = 0;
                        int jf = 0, jl = 0;
                        if (lp < wf + _tolesp || fp > wl - _tolesp)
                        {
                            continue;
                        }
                        for (j = IF; j < il; j++)
                        {
                            jf = j;
                            if (fp < wv[j] - _tolesp)
                            {
                                break;
                            }
                        }
                        for (j = IF; j < il; j++)
                        {
                            jl = j;
                            if (lp < wv[j] + _tolesp)
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
                            Edge ej = spine.Edges(jloc);
                            if (!_eviMap.ContainsKey(ej))
                            {
                                NCollection_List<int> li = new();
                                _eviMap[ej] = li;
                            }
                            _eviMap.GetValue(ej).Append(cursd.Surf());
                        }
                    }
                }
            }
        }
    }

    //=================================================================================================

    internal void PerformSetOfSurf(Stripe? stripe, bool simul = false)
    {
        DsDataStructure dStr = _ds.ChangeDS();

        Spine? sp = stripe.Spine();
        int si = ChFi3d_SolidIndex(sp, dStr, _eSoMap, _eShMap);
        stripe.SetSolidIndex(si);
        if (!sp.SplitDone())
        {
            PerformSetOfKPart(stripe, simul);
        }

        PerformSetOfKGen(stripe, simul);

        if (!simul)
        {
            ChFi3d_MakeExtremities(stripe, dStr, _efMap, _tolapp3d, _tol2d);
        }
    }

}

internal static class Builder_2
{
    internal static Curve2D? GetCurveOnSurface(in Edge edge, in Face face)
    {
        if (edge.IsNull || face.IsNull)
        {
            return null;
        }

        double firstParam = 0.0;
        double lastParam = 0.0;
        return Tool.CurveOnSurface(edge, face, out firstParam, out lastParam);
    }

    internal static void ChFi3d_CoupeParPlan(in CommonPoint compoint1, in CommonPoint compoint2, SurfaceAdaptor? hConge, in Point2D uv1, in Point2D uv2, double tol3d, double tol2d, ref Curve3D? c3d, ref Curve2D? pcurve, ref double tolreached, ref double pardeb, ref double parfin, ref bool plane)
    {
        plane = true;
        if (compoint1.IsOnArc && compoint2.IsOnArc)
        {
            Point3D p1 = new(), p2 = new();
            EdgeCurveAdaptor bCurv1 = new(compoint1.Arc());
            EdgeCurveAdaptor bCurv2 = new(compoint2.Arc());
            double parE1 = 0.0, parE2 = 0.0;
            parE1 = compoint1.ParameterOnArc();
            parE2 = compoint2.ParameterOnArc();
            Vector3D t1 = new(), t2 = new();
            bCurv1.D1(parE1, out p1, out t1);
            bCurv2.D1(parE2, out p2, out t2);
            Direction3D tgt1 = new(t1);
            Direction3D tgt2 = new(t2);
            Vector3D v12 = new(p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z);
            Direction3D d12 = new(v12);
            Direction3D nor = tgt1.Crossed(d12);
            PlaneSurface? plan = new PlaneSurface(p1, nor);
            double scal = 0.0;
            scal = Math.Abs(nor.Dot(tgt2));
            if (scal < 0.01)
            {
                SurfaceAdaptor? hPlan = new SurfaceAdaptor(plan);
                Curve2D? c2dint2 = null;
                NCollection_Array1<double> pdeb = new(1, 4), pfin = new(1, 4);
                SurfaceAdaptor AS = new(plan);
                ExtPS extPS = new(p1, AS, AS.FirstUParameter(), AS.LastUParameter(), AS.FirstVParameter(),
                                            AS.LastVParameter(), 1.0e-3, 1.0e-3, ExtFlag.ExtFlag_MIN);
                double u1 = 0.0, v1 = 0.0;
                extPS.Point(1).Parameter(out u1, out v1);
                pdeb[1] = uv1.X;
                pdeb[2] = uv1.Y;
                pdeb[3] = u1;
                pdeb[4] = v1;
                extPS.Perform(p2);
                extPS.Point(1).Parameter(out u1, out v1);
                pfin[1] = uv2.X;
                pfin[2] = uv2.Y;
                pfin[3] = u1;
                pfin[4] = v1;
                if (ChFi3d_ComputeCurves(hConge, hPlan, pdeb, pfin, ref c3d, ref pcurve, ref c2dint2, tol3d, tol2d,
                                                                  out tolreached))
                {
                    pardeb = c3d.FirstParameter;
                    parfin = c3d.LastParameter;
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

    internal static bool IsTangentToArc(in CommonPoint aCommonPoint, double anAngularTolerance)
    {
        if (!aCommonPoint.HasVector)
        {
            return false;
        }

        double dummyParam1 = 0.0, dummyParam2 = 0.0;// Not used.
        Curve3D? arcCurve = 
            Tool.Curve(aCommonPoint.Arc(), out dummyParam1, out dummyParam2);

        Point3D dummyPoint = new(); // Not used.
        Vector3D arcTangent = new(); // Tangent to the arc at the common point.
        arcCurve.D1(aCommonPoint.ParameterOnArc(), out dummyPoint, out arcTangent);

        Vector3D commonPointVector = aCommonPoint.Vector();
        return commonPointVector.IsParallel(arcTangent, anAngularTolerance);
    }

    internal static bool BonVoisin(in Point3D point, FaceSurfaceAdaptor? hs, ref Face f, SurfaceAdaptor? plane, in Edge cured, ref double xDep, ref double yDep, Map efMap, double tol3d)
    {
        bool bonvoisin = true;
        double winter = 0.0;
        Point3D papp = hs.Value(xDep, yDep);
        double dist = RealLast();
        EdgeCurveAdaptor? hc = new EdgeCurveAdaptor();
        Curve2D? pc = null;
        bool found = false;

        Explorer ex = new();
        for (ex.Init(f, ShapeType.Edge); ex.More(); ex.Next())
        {
            Edge ecur = ShapeCasts.Edge(ex.Current());
            if (!ecur.IsSame(cured))
            {
                hc.Initialize(ecur);
                double tolc = hc.Resolution(tol3d);
                if (ChFi3d_InterPlaneEdge(plane, hc, ref winter, true, tolc))
                {
                    Point3D np = hc.Value(winter);
                    double ndist = np.SquareDistanceTo(papp);
                    if (ndist < dist)
                    {
                        NCollection_List<Shape>.Iterator it = new();
                        Face ff = new();
                        bool isclosed = Tool.IsClosed(ecur, f);
                        bool isreallyclosed = BrepTools.IsReallyClosed(ecur, f);
                        for (it.Initialize(efMap[ecur]); it.More(); it.Next())
                        {
                            ff = ShapeCasts.Face(it.Value());
                            bool issame = ff.IsSame(f);
                            //  Modified by Sergey KHROMOV - Fri Dec 21 17:12:48 2001 Begin
                            // 	    bool istg =
                            // 	      BRep_Tool::Continuity(ecur,ff,F) != GeomAbs_C0;
                            bool istg = global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(ecur, ff, f);
                            //  Modified by Sergey KHROMOV - Fri Dec 21 17:12:51 2001 End
                            if ((!issame || (issame && isreallyclosed)) && istg)
                            {
                                found = true;
                                Edge newe = ecur;
                                newe.Orient(Forward);
                                dist = ndist;
                                hs.Initialize(ff);
                                if (isclosed && !isreallyclosed)
                                {
                                    Face fff = ff;
                                    fff.Orient(Forward);
                                    Explorer ex2 = new();
                                    for (ex2.Init(fff, ShapeType.Edge); ex2.More(); ex2.Next())
                                    {
                                        if (newe.IsSame(ex2.Current()))
                                        {
                                            newe = ShapeCasts.Edge(ex2.Current());
                                            pc = GetCurveOnSurface(newe, fff);
                                            break;
                                        }
                                    }
                                }
                                else
                                {
                                    pc = GetCurveOnSurface(newe, ff);
                                }
                                pc.Value(winter).Coord(out xDep, out yDep);
                                if (issame)
                                {
                                    Point3D spt = new();
                                    Vector3D sdu = new(), sdv = new(), nors = new();
                                    hs.D1(xDep, yDep, out spt, out sdu, out sdv);
                                    nors = sdu.Crossed(sdv);
                                    Point3D cpt = new();
                                    Vector3D cd = new();
                                    hc.D1(winter, out cpt, out cd);
                                    Vector3D vref = new(point, cpt);
                                    Face fff = ff;
                                    fff.Orient(Forward);
                                    if (vref.Dot(nors.Crossed(cd)) < 0.0)
                                    {
                                        newe.Orient(Reversed);
                                    }
                                    pc = GetCurveOnSurface(newe, fff);
                                    pc.Value(winter).Coord(out xDep, out yDep);
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

    internal static bool Projection(CurvePointExtrema pExt, in Point3D p, CurveAdaptorBase c, ref double w, double tol)
    {
        double dist2 = 0.0, daux2 = 0.0;
        dist2 = c.Value(w).SquareDistanceTo(p);

        // It is checked if it is not already a solution
        if (dist2 < tol * tol)
        {
            return true;
        }

        bool ok = false;

        // On essai une resolution initialise
        LocateExtPC ext = new(p, c, w, tol / 10);
        if (ext.IsDone)
        {
            daux2 = c.Value(ext.Point().Parameter()).SquareDistanceTo(p);
            if (daux2 < dist2)
            {
                w = ext.Point().Parameter();
                dist2 = daux2;
                ok = true;
                if (dist2 < tol * tol)
                {
                    return true;
                }
            }
        }

        // Global resolution
        pExt.Perform(p);
        if (pExt.IsDone)
        {
            for (int ii = 1; ii <= pExt.ExtCount; ii++)
            {
                if (pExt.SquareDistance(ii) < dist2)
                {
                    dist2 = pExt.SquareDistance(ii);
                    w = pExt.Point(ii).Parameter();
                    ok = true;
                }
            }
        }
        return ok;
    }

    internal static void TgtKP(SurfData? cd, Spine? spine, int iedge, bool isfirst, ref Point3D ped, ref Vector3D ded)
    {
        double wtg = cd.InterferenceOnS1().Parameter(isfirst);
        EdgeCurveAdaptor bc = spine.CurrentElementarySpine(iedge);
        if (spine.Edges(iedge).Orientation == Forward)
        {
            bc.D1(wtg + bc.FirstParameter(), out ped, out ded);
        }
        else
        {
            bc.D1(-wtg + bc.LastParameter(), out ped, out ded);
            ded = ded.Reversed();
        }
        ded = ded.Normalized();
    }

    internal static bool IsInput(in Vector3D vec, in Vertex ve, in Face fa)
    {
        Explorer faceExp = new(fa, ShapeType.Wire);
        WireExplorer wireExp = new();
        int trouve = 0;
        Wire w = new();
        Edge e = new();
        Vertex vf = new(), vl = new();
        Vector3D[] vec3d = new Vector3D[2];
        Point3D point = new();

        // Find edges and compute 3D vectors
        for (; (faceExp.More() && (trouve < 2)); faceExp.Next())
        {
            w = ShapeCasts.Wire(faceExp.Current());
            for (trouve = 0, wireExp.Init(w); wireExp.More() && (trouve < 2); wireExp.Next())
            {
                e = ShapeCasts.Edge(wireExp.Current());
                Exploration.Vertices(e, out vf, out vl);
                if (vf.IsSame(ve))
                {
                    EdgeCurveAdaptor cb = new(e);
                    cb.D1(Tool.Parameter(ve, e), out point, out vec3d[trouve]);
                    trouve++;
                }
                else if (vl.IsSame(ve))
                {
                    EdgeCurveAdaptor cb = new(e);
                    cb.D1(Tool.Parameter(ve, e), out point, out vec3d[trouve]);
                    vec3d[trouve] = vec3d[trouve].Reversed();
                    trouve++;
                }
            }
        }
        if (trouve < 2)
        {
            return false;
        }
        // Calculate the normal and the angles in the associated vector plane
        Vector3D normal = new();
        normal = vec3d[0] ^ vec3d[1];
        if (normal.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
        { // Colinear case
            return (vec.IsParallel(vec3d[0], global::Kerf.Engine.Foundation.Precision.Confusion()));
        }

        double amin = 0.0, amax = 0.0;
        amax = vec3d[1].AngleWithRef(vec3d[0], normal);
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
        CoordinateSystem axe = new(point, new Direction3D(normal), new Direction3D(vec3d[0]));
        Transform3D transf = new Transform3D().WithTransformation(axe);
        Xyz coord = vec.XYZ();
        transf.Transforms(ref coord);
        coord.SetZ(0);
        transf = transf.Inverted();
        transf.Transforms(ref coord);
        Vector3D theProj = new(coord);

        // and finally...
        double angle = theProj.AngleWithRef(vec3d[0], normal);
        return ((angle >= amin) && (angle <= amax));
    }

    internal static bool IsG1(Map theMap, in Edge e, in Face fRef, ref Face fVoi)
    {
        NCollection_List<Shape>.Iterator it = new();
        // Find a neighbor of E different from FRef (general case).
        for (it.Initialize(theMap[e]); it.More(); it.Next())
        {
            if (!ShapeCasts.Face(it.Value()).IsSame(fRef))
            {
                fVoi = ShapeCasts.Face(it.Value());
                //  Modified by Sergey KHROMOV - Fri Dec 21 17:09:32 2001 Begin
                //    if (BRep_Tool::Continuity(E,FRef,FVoi) != GeomAbs_C0) {
                if (global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(e, fRef, fVoi))
                {
                    //  Modified by Sergey KHROMOV - Fri Dec 21 17:09:33 2001 End
                    return true;
                }
            }
        }
        // If is was not found it is checked if E is a cutting edge,
        // in which case FVoi = FRef is returned (less frequent case).
        Explorer ex = new();
        bool orset = false;
        Orientation orient = Forward;
        Edge ed = new();
        for (ex.Init(fRef, ShapeType.Edge); ex.More(); ex.Next())
        {
            ed = ShapeCasts.Edge(ex.Current());
            if (ed.IsSame(e))
            {
                if (!orset)
                {
                    orient = ed.Orientation;
                    orset = true;
                }
                else if (ed.Orientation == global::Kerf.Engine.Topology.TopAbs.Reverse(orient))
                {
                    fVoi = fRef;
                    //  Modified by Sergey KHROMOV - Fri Dec 21 17:15:12 2001 Begin
                    //  if (BRep_Tool::Continuity(E,FRef,FRef) >= GeomAbs_G1)
                    return global::Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(e, fRef, fRef);
                }
            }
        }
        return false;
    }

    internal static int SearchFaceOnV(in CommonPoint pc, in Face fRef, Map veMap, Map efMap, ref Face f1, ref Face f2)
    {
        // it is checked that it leaves the current face.
        bool findFace = IsInput(pc.Vector(), pc.Vertex(), fRef);
        if (findFace)
        {
            findFace = IsInput(pc.Vector().Reversed(), pc.Vertex(), fRef);
        }
        // If it does not leave, it is finished
        if (findFace)
        {
            f1 = fRef;
            return 1;
        }
        int num = 0;
        bool trouve = false;
        NCollection_List<Shape>.Iterator itE = new(), itF = new();
        Edge e = new();
        Face fVoi = new();

        for (itE.Initialize(veMap[pc.Vertex()]); itE.More() && (num < 2); itE.Next())
        {
            e = ShapeCasts.Edge(itE.Value());
            for (itF.Initialize(efMap[e]), trouve = false; itF.More() && (!trouve); itF.Next())
            {
                if (ShapeCasts.Face(itF.Value()).IsSame(fRef))
                {
                    trouve = true;
                }
            }
            if (trouve)
            {
                trouve = IsG1(efMap, e, fRef, ref fVoi);
            }
            if (trouve)
            {
                trouve = IsInput(pc.Vector(), pc.Vertex(), fVoi);
            }
            if (trouve)
            {
                if (num == 0)
                {
                    f1 = fVoi;
                }
                else
                {
                    f2 = fVoi;
                }
                num++;
            }
        }
        return num;
    }

    internal static void ChangeTransition(in CommonPoint precedant, ref CommonPoint courant, int faceIndex, DsHDataStructure? ds)
    {
        bool tochange = true;
        Face f = ShapeCasts.Face(ds.Shape(faceIndex));
        Edge arc = precedant.Arc();
        Curve2D? pCurve1 = null, pCurve2 = null;
        pCurve1 = GetCurveOnSurface(arc, f);
        Shape localShape = arc.Reversed();
        pCurve2 = GetCurveOnSurface(ShapeCasts.Edge(localShape), f);
        if (pCurve1 != pCurve2)
        {
            // This is a cutting edge, it is necessary to make a small Geometric test
            Vector3D tgarc = new();
            Point3D p = new();
            EdgeCurveAdaptor ac = new(arc);
            ac.D1(precedant.ParameterOnArc(), out p, out tgarc);
            tochange = tgarc.IsParallel(precedant.Vector(), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        if (tochange)
        {
            courant.SetArc(global::Kerf.Engine.Foundation.Precision.Confusion(), arc, precedant.ParameterOnArc(),
                                          global::Kerf.Engine.Topology.TopAbs.Reverse(precedant.TransitionOnArc()));
        }
    }

    internal static void ChFi3d_BuildPlane(DsDataStructure dStr, FaceSurfaceAdaptor? hs, ref Point2D pons, SurfData? sd, bool isfirst, int ons)
    {
        Face f = ShapeCasts.Face(dStr.Shape(sd.Index(ons)));
        if (f.IsNull)
        {
            return;
        }

        if (sd.Vertex(isfirst, ons).IsOnArc)
        {
            double u = 0.0, v = 0.0;
            Curve2D? hc = 
                Tool.CurveOnSurface(sd.Vertex(isfirst, ons).Arc(), f, out u, out v);
            hc.Value(sd.Vertex(isfirst, ons).ParameterOnArc()).Coord(out u, out v);
            SLProps theProp = new(hs, u, v, 1, 1.0e-12);
            if (theProp.IsNormalDefined)
            {
                PlaneSurface? pln = new PlaneSurface(theProp.Value(), theProp.Normal());
                Face newF = new LibraryMakeFace(pln, global::Kerf.Engine.Foundation.Precision.Confusion()).Face();
                newF.Orient(f.Orientation);
                pons = pons.WithCoord(0.0, 0.0);
                hs.Initialize(newF);
                return; // everything is good !
            }
        }
        throw new FailureException("ChFi3d_BuildPlane : echec .");
    }

    internal static void ChFi3d_SingularExtremity(Stripe? stripe, DsDataStructure dStr, in Vertex vtx, double tol3d, double tol2d)
    {
        SurfData? fd = null;
        double tolreached = 0.0;
        double pardeb = 0.0, parfin = 0.0;
        Point2D vOnS1 = new(), vOnS2 = new();
        Curve3D? c3d = null;
        Curve2D? pCurv = null;
        DsCurve crv = new();
        // SurfData and its CommonPoints,
        int ivtx = 0, icurv = 0;
        bool isfirst = false;

        if (stripe.Spine().IsPeriodic)
        {
            isfirst = true;
            fd = stripe.SetOfSurfData().First();
        }
        else
        {
            int sens = 0;
            int num = ChFi3d_IndexOfSurfData(vtx, stripe, out sens);
            fd = stripe.SetOfSurfData().Value(num);
            isfirst = (sens == 1);
        }

        CommonPoint cv1 = fd.Vertex(isfirst, 1);
        CommonPoint cv2 = fd.Vertex(isfirst, 2);
        // Is it always degenerated ?
        if (cv1.Point().IsEqual(cv2.Point(), 0))
        {
            ivtx = ChFi3d_IndexPointInDS(cv1, dStr);
            if (isfirst)
            {
                vOnS1 = fd.InterferenceOnS1().PCurveOnSurf().Value(fd.InterferenceOnS1().FirstParameter());
                vOnS2 = fd.InterferenceOnS2().PCurveOnSurf().Value(fd.InterferenceOnS2().FirstParameter());
            }
            else
            {
                vOnS1 = fd.InterferenceOnS1().PCurveOnSurf().Value(fd.InterferenceOnS1().LastParameter());
                vOnS2 = fd.InterferenceOnS2().PCurveOnSurf().Value(fd.InterferenceOnS2().LastParameter());
            }

            ChFi3d_ComputeArete(cv1, vOnS1, cv2, vOnS2, dStr.Surface(fd.Surf()).Surface(), ref c3d, ref pCurv, ref pardeb,
                                                    ref parfin, tol3d, tol2d, out tolreached, 0);
            crv = new DsCurve(c3d, tolreached);
            icurv = dStr.AddCurve(crv);

            stripe.SetCurve(icurv, isfirst);
            stripe.SetParameters(isfirst, pardeb, parfin);
            stripe.ChangePCurve(isfirst) = pCurv;
            stripe.SetIndexPoint(ivtx, isfirst, 1);
            stripe.SetIndexPoint(ivtx, isfirst, 2);

            if (stripe.Spine().IsPeriodic)
            {
                // periodic case : The operation is renewed
                // the curve 3d is not shared.
                // 2 degenerated edges coinciding in 3d
                isfirst = false;
                fd = stripe.SetOfSurfData().Last();
                vOnS1 = fd.InterferenceOnS1().PCurveOnSurf().Value(fd.InterferenceOnS1().LastParameter());
                vOnS2 = fd.InterferenceOnS2().PCurveOnSurf().Value(fd.InterferenceOnS2().LastParameter());

                ChFi3d_ComputeArete(cv1, vOnS1, cv2, vOnS2, dStr.Surface(fd.Surf()).Surface(), ref c3d, ref pCurv, ref pardeb,
                                                        ref parfin, tol3d, tol2d, out tolreached, 0);
                crv = new DsCurve(c3d, tolreached);
                icurv = dStr.AddCurve(crv);

                stripe.SetCurve(icurv, isfirst);
                stripe.SetParameters(isfirst, pardeb, parfin);
                stripe.ChangePCurve(isfirst) = pCurv;
                stripe.SetIndexPoint(ivtx, isfirst, 1);
                stripe.SetIndexPoint(ivtx, isfirst, 2);
            }
        }
    }

    internal static bool IsFree(in Shape e, Map efMap)
    {
        if (!efMap.Contains(e))
        {
            return false;
        }
        NCollection_List<Shape>.Iterator it = new();
        Shape fref = new();
        for (it.Initialize(efMap[e]); it.More(); it.Next())
        {
            if (fref.IsNull)
            {
                fref = it.Value();
            }
            else if (!fref.IsSame(it.Value()))
            {
                return false;
            }
        }
        return true;
    }

    internal static void ChFi3d_MakeExtremities(Stripe? stripe, DsDataStructure dStr, Map efMap, double tol3d, double tol2d)
    {
        ref Spine? sp = ref stripe.ChangeSpine();
        double pardeb = 0.0, parfin = 0.0;
        Curve3D? c3d = null;
        double tolreached = 0.0;
        if (sp.IsPeriodic)
        {
            Box b1 = new(), b2 = new();
            SurfData? sdf = stripe.SetOfSurfData().First();
            CommonPoint cv1 = sdf.VertexFirstOnS1();
            CommonPoint cv2 = sdf.VertexFirstOnS2();
            if (!cv1.Point().IsEqual(cv2.Point(), 0))
            {
                ChFi3d_ComputeArete(cv1, sdf.InterferenceOnS1().PCurveOnSurf().Value(sdf.InterferenceOnS1().FirstParameter()),
                    cv2, sdf.InterferenceOnS2().PCurveOnSurf().Value(sdf.InterferenceOnS2().FirstParameter()),
                    dStr.Surface(sdf.Surf()).Surface(), ref c3d, ref stripe.ChangeFirstPCurve(), ref pardeb, ref parfin, tol3d,
                    tol2d, out tolreached, 0);
                DsCurve crv = new(c3d, tolreached);
                stripe.ChangeFirstCurve(dStr.AddCurve(crv));
                stripe.ChangeFirstParameters(pardeb, parfin);
                stripe.ChangeIndexFirstPointOnS1(ChFi3d_IndexPointInDS(sdf.VertexFirstOnS1(), dStr));
                stripe.ChangeIndexFirstPointOnS2(ChFi3d_IndexPointInDS(sdf.VertexFirstOnS2(), dStr));
                int iCurv = stripe.FirstCurve();
                stripe.ChangeLastParameters(pardeb, parfin);
                stripe.ChangeLastCurve(iCurv);
                stripe.ChangeIndexLastPointOnS1(stripe.IndexFirstPointOnS1());
                stripe.ChangeIndexLastPointOnS2(stripe.IndexFirstPointOnS2());

                SurfData? sdl = stripe.SetOfSurfData().Last();

                ChFi3d_ComputePCurv(c3d, sdl.InterferenceOnS1().PCurveOnSurf().Value(sdl.InterferenceOnS1().LastParameter()),
                    sdl.InterferenceOnS2().PCurveOnSurf().Value(sdl.InterferenceOnS2().LastParameter()),
                    out stripe.ChangeLastPCurve(), dStr.Surface(sdl.Surf()).Surface(), pardeb, parfin, tol3d, out tolreached);
                double oldtol = dStr.ChangeCurve(iCurv).Tolerance();
                dStr.ChangeCurve(iCurv).Tolerance(Max(oldtol, tolreached));
                if (cv1.IsOnArc)
                {
                    ChFi3d_EnlargeBox(cv1.Arc(), efMap[cv1.Arc()], cv1.ParameterOnArc(), ref b1);
                }

                if (cv2.IsOnArc)
                {
                    ChFi3d_EnlargeBox(cv2.Arc(), efMap[cv2.Arc()], cv2.ParameterOnArc(), ref b2);
                }
                ChFi3d_EnlargeBox(dStr, stripe, sdf, ref b1, ref b2, true);
                ChFi3d_EnlargeBox(dStr, stripe, sdl, ref b1, ref b2, false);
                if (!cv1.IsVertex)
                {
                    ChFi3d_SetPointTolerance(dStr, b1, stripe.IndexFirstPointOnS1());
                }
                if (!cv2.IsVertex)
                {
                    ChFi3d_SetPointTolerance(dStr, b2, stripe.IndexFirstPointOnS2());
                }
            }
            else
            {
                // Case of the single extremity
                if (cv1.IsVertex)
                {
                    ChFi3d_SingularExtremity(stripe, dStr, cv1.Vertex(), tol3d, tol2d);
                }
                // #ifdef CHFI3D_DEB: a trace, left out
            }
            return;
        }

        SurfData? sDdeb = stripe.SetOfSurfData().First();

        CommonPoint cpdeb1 = sDdeb.VertexFirstOnS1();
        CommonPoint cpdeb2 = sDdeb.VertexFirstOnS2();
        bool freedeb = sp.FirstStatus() == State.FreeBoundary;
        if (!freedeb && cpdeb1.IsOnArc && cpdeb2.IsOnArc)
        {
            freedeb = (IsFree(cpdeb1.Arc(), efMap) && IsFree(cpdeb2.Arc(), efMap));
        }
        if (freedeb)
        {
            sp.SetFirstStatus(State.FreeBoundary);
            Box b1 = new(), b2 = new();
            if (!cpdeb1.Point().IsEqual(cpdeb2.Point(), 0))
            {
                bool plane = false;
                Point2D uv1 = new(), uv2 = new();
                uv1 =
                    sDdeb.InterferenceOnS1().PCurveOnSurf().Value(sDdeb.InterferenceOnS1().FirstParameter());
                uv2 =
                    sDdeb.InterferenceOnS2().PCurveOnSurf().Value(sDdeb.InterferenceOnS2().FirstParameter());
                // The intersection of the fillet by a plane is attempted

                SurfaceAdaptor? hConge = ChFi3d_BoundSurf(dStr, sDdeb, 1, 2);
                ChFi3d_CoupeParPlan(cpdeb1, cpdeb2, hConge, uv1, uv2, tol3d, tol2d, ref c3d, ref stripe.ChangeFirstPCurve(), ref tolreached,
                                                        ref pardeb, ref parfin, ref plane);
                if (!plane)
                {
                    ChFi3d_ComputeArete(cpdeb1, sDdeb.InterferenceOnS1().PCurveOnSurf().Value(
                                                                sDdeb.InterferenceOnS1().FirstParameter()), cpdeb2,
                                                            sDdeb.InterferenceOnS2().PCurveOnSurf().Value(sDdeb.InterferenceOnS2().FirstParameter()),
                                                            dStr.Surface(sDdeb.Surf()).Surface(), ref c3d, ref stripe.ChangeFirstPCurve(), ref pardeb,
                                                            ref parfin, tol3d, tol2d, out tolreached, 0);
                }
                DsCurve crv = new(c3d, tolreached);
                stripe.ChangeFirstCurve(dStr.AddCurve(crv));
                stripe.ChangeFirstParameters(pardeb, parfin);
                stripe.ChangeIndexFirstPointOnS1(ChFi3d_IndexPointInDS(sDdeb.VertexFirstOnS1(), dStr));
                stripe.ChangeIndexFirstPointOnS2(ChFi3d_IndexPointInDS(sDdeb.VertexFirstOnS2(), dStr));
                if (cpdeb1.IsOnArc)
                {
                    ChFi3d_EnlargeBox(cpdeb1.Arc(), efMap[cpdeb1.Arc()], cpdeb1.ParameterOnArc(), ref b1);
                }
                if (cpdeb2.IsOnArc)
                {
                    ChFi3d_EnlargeBox(cpdeb2.Arc(), efMap[cpdeb2.Arc()], cpdeb2.ParameterOnArc(), ref b2);
                }
                ChFi3d_EnlargeBox(dStr, stripe, sDdeb, ref b1, ref b2, true);
                if (!cpdeb1.IsVertex)
                {
                    ChFi3d_SetPointTolerance(dStr, b1, stripe.IndexFirstPointOnS1());
                }
                if (!cpdeb2.IsVertex)
                {
                    ChFi3d_SetPointTolerance(dStr, b2, stripe.IndexFirstPointOnS2());
                }
            }
            else
            { // Case of a singular extremity
                if (cpdeb1.IsVertex)
                {
                    ChFi3d_SingularExtremity(stripe, dStr, cpdeb1.Vertex(), tol3d, tol2d);
                }
                // #ifdef CHFI3D_DEB: a trace, left out
            }
        }
        SurfData? sDfin = stripe.SetOfSurfData().Last();
        CommonPoint cpfin1 = sDfin.VertexLastOnS1();
        CommonPoint cpfin2 = sDfin.VertexLastOnS2();
        bool freefin = sp.LastStatus() == State.FreeBoundary;
        if (!freefin && cpfin1.IsOnArc && cpfin2.IsOnArc)
        {
            freefin = (IsFree(cpfin1.Arc(), efMap) && IsFree(cpfin2.Arc(), efMap));
        }
        if (freefin)
        {
            sp.SetLastStatus(State.FreeBoundary);
            Box b1 = new(), b2 = new();
            if (!cpfin1.Point().IsEqual(cpfin2.Point(), 0))
            {
                bool plane = false;
                Point2D uv1 = new(), uv2 = new();
                uv1 =
                    sDfin.InterferenceOnS1().PCurveOnSurf().Value(sDfin.InterferenceOnS1().LastParameter());
                uv2 =
                    sDfin.InterferenceOnS2().PCurveOnSurf().Value(sDfin.InterferenceOnS2().LastParameter());
                // Intersection of the fillet by a plane is attempted

                SurfaceAdaptor? hConge = ChFi3d_BoundSurf(dStr, sDfin, 1, 2);
                ChFi3d_CoupeParPlan(cpfin1, cpfin2, hConge, uv1, uv2, tol3d, tol2d, ref c3d, ref stripe.ChangeLastPCurve(), ref tolreached,
                                                        ref pardeb, ref parfin, ref plane);
                if (!plane)
                {
                    ChFi3d_ComputeArete(cpfin1, sDfin.InterferenceOnS1().PCurveOnSurf().Value(
                                                                sDfin.InterferenceOnS1().LastParameter()), cpfin2,
                                                            sDfin.InterferenceOnS2().PCurveOnSurf().Value(sDfin.InterferenceOnS2().LastParameter()),
                                                            dStr.Surface(sDfin.Surf()).Surface(), ref c3d, ref stripe.ChangeLastPCurve(), ref pardeb,
                                                            ref parfin, tol3d, tol2d, out tolreached, 0);
                }
                DsCurve crv = new(c3d, tolreached);
                stripe.ChangeLastCurve(dStr.AddCurve(crv));
                stripe.ChangeLastParameters(pardeb, parfin);
                stripe.ChangeIndexLastPointOnS1(ChFi3d_IndexPointInDS(sDfin.VertexLastOnS1(), dStr));
                stripe.ChangeIndexLastPointOnS2(ChFi3d_IndexPointInDS(sDfin.VertexLastOnS2(), dStr));
                if (cpfin1.IsOnArc)
                {
                    ChFi3d_EnlargeBox(cpfin1.Arc(), efMap[cpfin1.Arc()], cpfin1.ParameterOnArc(), ref b1);
                }
                if (cpfin2.IsOnArc)
                {
                    ChFi3d_EnlargeBox(cpfin2.Arc(), efMap[cpfin2.Arc()], cpfin2.ParameterOnArc(), ref b2);
                }
                ChFi3d_EnlargeBox(dStr, stripe, sDfin, ref b1, ref b2, false);
                if (!cpfin1.IsVertex)
                {
                    ChFi3d_SetPointTolerance(dStr, b1, stripe.IndexLastPointOnS1());
                }
                if (!cpfin2.IsVertex)
                {
                    ChFi3d_SetPointTolerance(dStr, b2, stripe.IndexLastPointOnS2());
                }
            }
            else
            { // Case of the single extremity
                if (cpfin1.IsVertex)
                {
                    ChFi3d_SingularExtremity(stripe, dStr, cpfin1.Vertex(), tol3d, tol2d);
                }
                // #ifdef CHFI3D_DEB: a trace, left out
            }
        }
    }

    internal static void ChFi3d_Purge(Stripe? stripe, SurfData? sd, in CommonPoint vRef, bool isfirst, int ons, ref int intf, ref int intl)
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
        if (!sd.Vertex(isfirst, opp).IsOnArc || sd.TwistOnS1() || sd.TwistOnS2())
        {
            NCollection_Sequence<SurfData> seq = 
                stripe.ChangeSetOfSurfData();
            if (isfirst)
            {
                seq.Remove(1);
            }
            else
            {
                seq.Remove(seq.Length());
            }
            return;
        }
        if (ons == 1)
        {
            sd.ChangeIndexOfS1(0);
        }
        else
        {
            sd.ChangeIndexOfS2(0);
        }

        sd.ChangeVertex(!isfirst, ons) = vRef;
        sd.ChangeVertex(isfirst, ons) = vRef;

        ref FaceInterference fi = ref sd.ChangeInterference(ons);
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

    internal static void InsertAfter(Stripe? stripe, SurfData? Ref, SurfData? item)
    {
        if (Ref == item)
        {
            throw new FailureException("InsertAfter : twice the same surfdata.");
        }

        NCollection_Sequence<SurfData> seq = 
            stripe.ChangeSetOfSurfData();

        if (seq.IsEmpty() || Ref is null)
        {
            seq.Prepend(item);
        }
        for (int i = 1; i <= seq.Length(); i++)
        {
            if (seq.Value(i) == Ref)
            {
                seq.InsertAfter(i, item);
                break;
            }
        }
    }

    internal static void RemoveSD(Stripe? stripe, SurfData? prev, SurfData? next)
    {
        NCollection_Sequence<SurfData> seq = 
            stripe.ChangeSetOfSurfData();
        if (seq.IsEmpty())
        {
            return;
        }
        int iprev = 0, inext = 0;
        for (int i = 1; i <= seq.Length(); i++)
        {
            if (seq.Value(i) == prev)
            {
                iprev = i + 1;
            }
            if (seq.Value(i) == next)
            {
                inext = i - 1;
                break;
            }
        }
        if (prev is null)
        {
            iprev = 1;
        }
        if (next is null)
        {
            inext = seq.Length();
        }
        if (iprev <= inext)
        {
            seq.Remove(iprev, inext);
        }
    }

    internal static void InsertBefore(Stripe? stripe, SurfData? Ref, SurfData? item)
    {
        if (Ref == item)
        {
            throw new FailureException("InsertBefore : twice the same surfdata.");
        }

        NCollection_Sequence<SurfData> seq = 
            stripe.ChangeSetOfSurfData();

        if (seq.IsEmpty() || Ref is null)
        {
            seq.Append(item);
        }
        for (int i = 1; i <= seq.Length(); i++)
        {
            if (seq.Value(i) == Ref)
            {
                seq.InsertBefore(i, item);
                break;
            }
        }
    }

    internal static double ChFi3d_BoxDiag(in Box box)
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
