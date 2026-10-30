// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.cxx (from line 2130; merged into ShapeFix_Wire.cs)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public partial class ShapeFix_Wire
{
    /// <summary>
    /// Fixes the degenerated edge <paramref name="num"/> (or before it): inserts a degenerated edge where the wire
    /// passes a singularity of the surface, or replaces the edge by one; removes an edge encoded degenerated with no
    /// pcurve and no singularity.
    /// </summary>
    public bool FixDegenerated(int num)
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        // analysis
        gp_Pnt2d p2d1 = new gp_Pnt2d(), p2d2 = new gp_Pnt2d();
        myAnalyzer.CheckDegenerated(num, ref p2d1, ref p2d2);
        if (myAnalyzer.LastCheckStatus(ShapeExtend_FAIL1))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        //: abv 29.08.01: torHalf2.sat: if edge was encoded as degenerated but
        //  has no pcurve and no singularity is found at that point, remove it
        if (myAnalyzer.LastCheckStatus(ShapeExtend_FAIL2))
        {
            WireData()!.Remove(num);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            return true;
        }
        if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        // action: create degenerated edge and insert it (or replace)

        gp_Vec2d    vect2d = new gp_Vec2d(p2d1, p2d2);
        gp_Dir2d    dir2d  = new gp_Dir2d(vect2d);
        Geom2d_Line line2d = new Geom2d_Line(p2d1, dir2d);

        TopoDS_Edge  degEdge = new TopoDS_Edge();
        BRep_Builder B       = new BRep_Builder();
        B.MakeEdge(ref degEdge);
        B.Degenerated(degEdge, true);
        B.UpdateEdge(degEdge, line2d, Face(), Occt.Precision.Confusion());
        B.Range(degEdge, Face(), 0.0, vect2d.Magnitude());

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());

        bool lack = myAnalyzer.LastCheckStatus(ShapeExtend_DONE1);
        int  n3   = (lack ? n2 : (n2 < sbwd.NbEdges() ? n2 + 1 : 1));

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex      V1  = sae.LastVertex(sbwd.Edge(n1));
        TopoDS_Vertex      V2  = sae.FirstVertex(sbwd.Edge(n3));

        V1.Orientation(TopAbs_FORWARD);
        V2.Orientation(TopAbs_REVERSED);
        B.Add(degEdge, V1);
        B.Add(degEdge, V2);
        degEdge.Orientation(TopAbs_FORWARD);

        if (lack)
        {
            sbwd.Add(degEdge, n2);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        else
        {
            sbwd.Set(degEdge, n2);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }

        //  commented to avoid extra messages
        //  SendWarning ( degEdge, Message_Msg ( "FixWire.FixDegenerated.MSG0" ) );// Degenerated edge(s)
        //  detected

        return true;
    }

    //=======================================================================
    // function : FixSelfIntersectingEdge
    // purpose  : Tests edge for self-intersection and updates tolerance of vertex
    //           if intersection is found
    //           Returns True if tolerance was increased
    //=======================================================================

    // Create edge on basis of E with new pcurve and call FixSP
    // Return resulting tolerance and modified pcurve
    private static bool TryNewPCurve(in TopoDS_Edge E, in TopoDS_Face face, ref Geom2d_Curve? c2d, ref double first,
                                     ref double last, ref double tol)
    {
        Geom_Curve? crv = BRep_Tool.Curve(E, out double f, out double l);
        if (crv is null)
        {
            return false;
        }

        // make temp edge and compute tolerance
        BRepBuilderAPI_MakeEdge mkedge = new BRepBuilderAPI_MakeEdge(crv, f, l);

        ShapeBuild_Edge SBE = new ShapeBuild_Edge();     // skl 17.07.2001
        SBE.SetRange3d(mkedge.InternalEdge(), f, l); // skl 17.07.2001

        if (!mkedge.IsDone())
        {
            return false;
        }

        TopoDS_Edge  edge = mkedge.InternalEdge();
        BRep_Builder B    = new BRep_Builder();
        B.UpdateEdge(edge, c2d, face, 0.0);
        B.Range(edge, face, first, last);
        B.SameRange(edge, false);
        // no call to BRepLib:  B.SameParameter ( edge, false );

        ShapeFix_Edge sfe = new ShapeFix_Edge();
        sfe.FixSameParameter(edge, face);
        c2d = BRep_Tool.CurveOnSurface(edge, face, out first, out last);
        tol = BRep_Tool.Tolerance(edge);
        return true;
    }

    //: k3 abv 24 Dec 98: BUC50070 #26682 and #30087:
    // Try to cut out the loop on the pcurve and make new pcurve by concatenating
    // the parts.
    // If result is not SameParameter with prec, does nothing and returns False
    // Warning: resulting pcurve will be C0

    private static int howMuchPCurves(in TopoDS_Edge E)
    {
        int count = 0;
        // OCCT.NET: frozen: the list is only read (OCCT walks ChangeCurves())
        var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)E.TShape()!).Curves());

        while (itcr.More())
        {
            BRep_CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface())
            {
                count++;
            }
            itcr.Next();
        }

        return count;
    }

    private static bool RemoveLoop(in TopoDS_Edge E, in TopoDS_Face face, in IntRes2d_IntersectionPoint IP, double tolfact,
                                   double prec, bool RemoveLoop3d)
    {
        bool loopRemoved3d;
        if (BRep_Tool.IsClosed(E, face))
        {
            return false;
        }

        Geom_Curve? crv = BRep_Tool.Curve(E, out double f, out double l);

        double t1 = IP.ParamOnFirst();
        double t2 = IP.ParamOnSecond();
        if (t1 > t2)
        {
            double t = t1;
            t1       = t2;
            t2       = t;
        }

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        if (!sae.PCurve(E, face, out Geom2d_Curve? c2d, out double a, out double b, false))
        {
            return false;
        }

        GeomAdaptor_Curve GAC = new GeomAdaptor_Curve(crv!, f, l);
        double            dt  = tolfact * GAC.Resolution(prec);
        t1 -= dt; // 1e-3;//::Precision::PConfusion();
        t2 += dt; // 1e-3;//::Precision::PConfusion();

        if (t1 <= a || t2 >= b)
        { // should not be so, but to be sure ..
            return false;
        }

        // direct construction causes error on osf system.
        gp_Pnt p   = new gp_Pnt(0, 0, 0);
        gp_Dir d   = new gp_Dir(gp_Dir.D.Z);
        gp_Ax3 ax  = new gp_Ax3(p, d);
        gp_Pln Pln = new gp_Pln(ax);

        // PTV OCC884
        Geom_Plane? aPlaneSurf = BRep_Tool.Surface(face) as Geom_Plane;
        Geom_Curve? pcurve3d   = crv;
        if (aPlaneSurf is not null)
        {
            Pln = aPlaneSurf.Pln();
        }
        else
        {
            pcurve3d = GeomAPI.To3d(c2d!, Pln);
        }

        // first segment
        // occ::handle<Geom_Curve> pcurve3d = GeomAPI::To3d ( c2d, Pln );
        Geom_TrimmedCurve                   trim    = new Geom_TrimmedCurve(pcurve3d!, a, t1);
        GeomConvert_CompCurveToBSplineCurve connect = new GeomConvert_CompCurveToBSplineCurve(trim);

        // null-length segment patch instead of loop
        NCollection_Array1<gp_Pnt> Poles = new NCollection_Array1<gp_Pnt>(1, 2);
        NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int>    Mults = new NCollection_Array1<int>(1, 2);

        Poles.SetValue(1, trim.Value(t1));
        Knots.SetValue(1, t1);
        Mults.SetValue(1, 2);

        trim = new Geom_TrimmedCurve(pcurve3d!, t2, b);
        Poles.SetValue(2, trim.Value(t2));
        Knots.SetValue(2, t2);
        Mults.SetValue(2, 2);

        Geom_BSplineCurve patch = new Geom_BSplineCurve(Poles, Knots, Mults, 1);
        if (!connect.Add(patch, Occt.Precision.PConfusion(), true, false))
        {
            return false;
        }

        // last segment
        if (!connect.Add(trim, Occt.Precision.PConfusion(), true, false))
        {
            return false;
        }
        // PTV OCC884
        // keep created 3d curve
        Geom_Curve? aNew3dCrv = connect.InternalBSplineCurve();

        Geom2d_Curve? bs = GeomAPI.To2d(aNew3dCrv!, Pln);
        if (bs is null)
        {
            return false;
        }

        // make temp edge and compute tolerance
        BRep_Builder B = new BRep_Builder();

        if (!RemoveLoop3d)
        { // old variant (not remove loop 3d)
            double newtol = 0;
            // OCC901
            int nbC2d = howMuchPCurves(E);
            if (nbC2d <= 1 && aPlaneSurf is not null)
            {
                B.UpdateEdge(E, bs, face, 0);
            }
            else if (!TryNewPCurve(E, face, ref bs, ref a, ref b, ref newtol))
            {
                return false;
            }

            double tol = BRep_Tool.Tolerance(E);
            if (newtol > Max(prec, tol))
            {
                return false;
            }
            //: s2  bs = BRep_Tool::CurveOnSurface ( edge, face, a, b );
            if (Math.Abs(a - f) > Occt.Precision.PConfusion() || // smth strange, cancel
                Math.Abs(b - l) > Occt.Precision.PConfusion())
            {
                return false;
            }
            // PTV OCC884
            if (aPlaneSurf is not null)
            {
                B.UpdateEdge(E, aNew3dCrv, Max(newtol, tol));
                // OCC901
                if (!TryNewPCurve(E, face, ref bs, ref a, ref b, ref newtol))
                {
                    return false;
                }
            }
            B.UpdateEdge(E, bs, face, newtol);
            B.UpdateVertex(sae.FirstVertex(E), newtol);
            B.UpdateVertex(sae.LastVertex(E), newtol);
            return true;
        }

        //: q1
        Geom_Surface?       S  = BRep_Tool.Surface(face, out TopLoc_Location L);
        Geom2dAdaptor_Curve AC = new Geom2dAdaptor_Curve(c2d!);
        GeomAdaptor_Surface AS = new GeomAdaptor_Surface(S!);

        Adaptor3d_CurveOnSurface ACS = new Adaptor3d_CurveOnSurface(AC, AS);
        gp_Pnt                   P1  = ACS.Value(t1);
        gp_Pnt                   P2  = ACS.Value(t2);
        gp_Pnt pcurPnt = new gp_Pnt((P1.X() + P2.X()) / 2, (P1.Y() + P2.Y()) / 2, (P1.Z() + P2.Z()) / 2);

        ShapeAnalysis_TransferParametersProj SFTP  = new ShapeAnalysis_TransferParametersProj(E, face);
        NCollection_HSequence<double>        Seq2d = new NCollection_HSequence<double>();
        Seq2d.Append(t1);
        Seq2d.Append(t2);
        Seq2d.Append((t1 + t2) / 2);
        NCollection_HSequence<double> Seq3d = SFTP.Perform(Seq2d, false);

        double dist1 = pcurPnt.Distance(crv!.Value(Seq3d.Value(1))); // correcting Seq3d already project
        double dist2 = pcurPnt.Distance(crv.Value(Seq3d.Value(2)));
        double dist3 = pcurPnt.Distance(crv.Value(Seq3d.Value(3)));
        double ftrim, ltrim;
        loopRemoved3d = dist3 <= Max(dist1, dist2);

        Geom_Curve? bs1 = null;

        if (!loopRemoved3d)
        {
            // create new 3d curve
            ftrim = Seq3d.Value(1);
            ltrim = Seq3d.Value(2);
            ftrim -= dt;
            ltrim += dt;
            Geom_TrimmedCurve                   trim1    = new Geom_TrimmedCurve(crv, f, ftrim);
            GeomConvert_CompCurveToBSplineCurve connect1 = new GeomConvert_CompCurveToBSplineCurve(trim1);
            NCollection_Array1<gp_Pnt>          Poles1   = new NCollection_Array1<gp_Pnt>(1, 2);
            NCollection_Array1<double>          Knots1   = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int>             Mults1   = new NCollection_Array1<int>(1, 2);

            Poles1.SetValue(1, trim1.Value(ftrim));
            Knots1.SetValue(1, ftrim);
            Mults1.SetValue(1, 2);

            trim1 = new Geom_TrimmedCurve(crv, ltrim, l);
            Poles1.SetValue(2, trim1.Value(ltrim));
            Knots1.SetValue(2, ltrim);
            Mults1.SetValue(2, 2);

            // create b-spline curve
            Geom_BSplineCurve patch1 = new Geom_BSplineCurve(Poles1, Knots1, Mults1, 1);

            if (!connect1.Add(patch1, Occt.Precision.PConfusion(), true, false))
            {
                return false;
            }

            // last segment
            if (!connect1.Add(trim1, Occt.Precision.PConfusion(), true, false))
            {
                return false;
            }

            bs1 = connect1.InternalBSplineCurve();

            if (bs1 is null)
            {
                return false;
            }
        }
        //  double oldtol = BRep_Tool::Tolerance ( E );

        if (!loopRemoved3d)
        {
            B.UpdateEdge(E, bs1, L, 0.0);
        }
        B.UpdateEdge(E, bs, face, 0.0);
        B.Range(E, face, f, l);
        B.SameRange(E, false);

        ShapeFix_Edge sfe = new ShapeFix_Edge();
        sfe.FixSameParameter(E);

        return true;
    }

    private static bool RemoveLoop(in TopoDS_Edge E, in TopoDS_Face face, in IntRes2d_IntersectionPoint IP2d,
                                   ref TopoDS_Edge E1, ref TopoDS_Edge E2)
    {
        if (BRep_Tool.IsClosed(E, face))
        {
            return false;
        }

        Geom_Curve? crv = BRep_Tool.Curve(E, out double f, out double l);

        double t1 = IP2d.ParamOnFirst();
        double t2 = IP2d.ParamOnSecond();

        if (t1 > t2)
        {
            double t = t1;
            t1       = t2;
            t2       = t;
        }

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();

        // define vertexes Vfirst , Vlast, Vmid
        TopoDS_Vertex Vfirst, Vlast, Vmid = new TopoDS_Vertex();
        // initialize Vfirst and Vlast
        Vfirst = sae.FirstVertex(E);
        Vlast  = sae.LastVertex(E);

        // find a 2d curve and parameters from edge E
        if (!sae.PCurve(E, face, out Geom2d_Curve? c2d, out double a, out double b, false))
        {
            return false;
        }

        // first segment for 2d curve
        Geom2d_TrimmedCurve? trim1 = null;
        if ((t1 - a) > Occt.Precision.PConfusion())
        {
            trim1 = new Geom2d_TrimmedCurve(c2d!, a, t1);
        }
        // second segment for 2d curve
        Geom2d_TrimmedCurve? trim2 = new Geom2d_TrimmedCurve(c2d!, t2, b);

        //  if ( trim1.IsNull() || trim2.IsNull()  )
        if (trim2 is null)
        {
            return false;
        }

        Geom_Surface?       S  = BRep_Tool.Surface(face, out TopLoc_Location L);
        Geom2dAdaptor_Curve AC = new Geom2dAdaptor_Curve(c2d!);
        GeomAdaptor_Surface AS = new GeomAdaptor_Surface(S!);

        Adaptor3d_CurveOnSurface ACS = new Adaptor3d_CurveOnSurface(AC, AS);
        gp_Pnt                   P1  = ACS.Value(t1);
        gp_Pnt                   P2  = ACS.Value(t2);
        gp_Pnt pcurPnt = new gp_Pnt((P1.X() + P2.X()) / 2, (P1.Y() + P2.Y()) / 2, (P1.Z() + P2.Z()) / 2);

        // transfer parameters from pcurve to 3d curve
        ShapeAnalysis_TransferParametersProj SFTP  = new ShapeAnalysis_TransferParametersProj(E, face);
        NCollection_HSequence<double>        Seq2d = new NCollection_HSequence<double>();
        Seq2d.Append(t1);
        Seq2d.Append(t2);
        Seq2d.Append((t1 + t2) / 2);
        NCollection_HSequence<double> Seq3d = SFTP.Perform(Seq2d, false);

        double dist1 = pcurPnt.Distance(crv!.Value(Seq3d.Value(1))); // correcting Seq3d already project
        double dist2 = pcurPnt.Distance(crv.Value(Seq3d.Value(2)));
        double dist3 = pcurPnt.Distance(crv.Value(Seq3d.Value(3)));
        double ftrim, ltrim;
        if (dist3 > Max(dist1, dist2))
        { // is loop in 3d
            ftrim = Seq3d.Value(1);
            ltrim = Seq3d.Value(2);
        }
        else
        { // not loop in 3d
            ftrim = Seq3d.Value(3);
            ltrim = Seq3d.Value(3);
        }

        //  ftrim = Seq3d->Value(1);
        //  ltrim = Seq3d->Value(2);

        // trim for 3d curve 'crv' with parameters from 'f' to 'l'
        Geom_TrimmedCurve? trim3 = null;
        if (trim1 is not null)
        {
            trim3 = new Geom_TrimmedCurve(crv, f, ftrim);
        }
        // second segment for 3d curve
        Geom_TrimmedCurve? trim4 = new Geom_TrimmedCurve(crv, ltrim, l);

        //  if ( trim3.IsNull() || trim4.IsNull()  )
        if (trim4 is null)
        {
            return false;
        }

        // create a point for middle vertex
        gp_Pnt pnt1 = crv.Value(ftrim);
        gp_Pnt pnt2 = crv.Value(ltrim);
        gp_Pnt Pmid = new gp_Pnt((pnt1.X() + pnt2.X()) / 2, (pnt1.Y() + pnt2.Y()) / 2, (pnt1.Z() + pnt2.Z()) / 2);

        BRep_Builder B = new BRep_Builder();

        // create new copies for E1 and E2
        if (trim1 is not null)
        {
            E1 = TopoDS.Edge(E.EmptyCopied());
        }
        E2 = TopoDS.Edge(E.EmptyCopied());

        // initialize middle vertex Vmid
        if (trim1 is null)
        {
            B.MakeVertex(ref Vmid, pnt2, 0.0);
        }
        else
        {
            B.MakeVertex(ref Vmid, Pmid, 0.0);
        }

        ShapeBuild_Edge sbe = new ShapeBuild_Edge();

        // replace verteces for new edges E1 and E2
        if (E.Orientation() == TopAbs_FORWARD)
        {
            if (!E1.IsNull())
            {
                E1 = sbe.InternalCopyReplaceVertices(E1, Vfirst, Vmid);
            }
            E2 = sbe.InternalCopyReplaceVertices(E2, Vmid, Vlast);
        }
        else
        {
            if (!E1.IsNull())
            {
                E1 = sbe.InternalCopyReplaceVertices(E1, Vmid, Vlast);
            }
            E2 = sbe.InternalCopyReplaceVertices(E2, Vfirst, Vmid);
        }

        // Update edges by 2d and 3d curves
        ShapeFix_Edge mySfe = new ShapeFix_Edge();
        if (!E1.IsNull())
        {
            B.UpdateEdge(E1, trim1, face, 0.0);
            B.UpdateEdge(E1, trim3, 0.0);
            B.Range(E1, f, ftrim);
            B.SameRange(E1, false);
            //    B.SameParameter(E1,false);
            mySfe.FixSameParameter(E1);
            mySfe.FixVertexTolerance(E1);
        }
        B.UpdateEdge(E2, trim2, face, 0.0);
        B.UpdateEdge(E2, trim4, 0.0);
        B.Range(E2, ltrim, l);
        B.SameRange(E2, false);
        //  B.SameParameter(E2, false);
        mySfe.FixSameParameter(E2);
        mySfe.FixVertexTolerance(E2);

        return true;
    }

    private bool FixSelfIntersectingEdge(int num)
    {
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        // analysis
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d       = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<gp_Pnt>                     points3d       = new NCollection_Sequence<gp_Pnt>();
        ShapeAnalysis_Wire?                              theAdvAnalyzer = myAnalyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckSelfIntersectingEdge(num, points2d, points3d);
        if (theAdvAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        // action: increase tolerance of vertex

        TopoDS_Edge E = WireData()!.Edge(num > 0 ? num : NbEdges());

        ShapeAnalysis_Edge sae  = new ShapeAnalysis_Edge();
        TopoDS_Vertex      V1   = sae.FirstVertex(E);
        TopoDS_Vertex      V2   = sae.LastVertex(E);
        double             tol1 = BRep_Tool.Tolerance(V1);
        double             tol2 = BRep_Tool.Tolerance(V2);
        gp_Pnt             pnt1 = BRep_Tool.Pnt(V1);
        gp_Pnt             pnt2 = BRep_Tool.Pnt(V2);

        // cycle is to verify fix in case of RemoveLoop
        double        tolfact = 0.1; // factor for shifting by parameter in RemoveLoop
        double        f2d = 0.0, l2d = 0.0;
        Geom2d_Curve? c2d    = null;
        double        newtol = 0.0; // = Precision();

        if (myRemoveLoopMode < 1)
        {
            for (int iter = 0; iter < 30; iter++)
            {
                bool   loopRemoved = false;
                double prevFirst = 0, prevLast = 0;
                for (int i = 1; i <= points2d.Length(); i++)
                {
                    gp_Pnt pint   = points3d.Value(i);
                    double dist21 = pnt1.SquareDistance(pint);
                    double dist22 = pnt2.SquareDistance(pint);
                    if (dist21 < tol1 * tol1 || dist22 < tol2 * tol2)
                    {
                        continue;
                    }
                    newtol = 1.001 * Math.Sqrt(Min(dist21, dist22)); //: f8

                    //: k3 abv 24 Dec 98: BUC50070 #26682 and #30087: try to remove loop
                    if (myGeomMode)
                    {
                        if (c2d is null)
                        {
                            sae.PCurve(E, Face(), out c2d, out f2d, out l2d, false);
                        }
                        double firstpar = points2d.Value(i).ParamOnFirst();
                        double lastpar  = points2d.Value(i).ParamOnSecond();
                        if (firstpar > prevFirst && lastpar < prevLast)
                        {
                            continue;
                        }
                        if (RemoveLoop(E,
                                       Face(),
                                       points2d.Value(i),
                                       tolfact,
                                       Min(MaxTolerance(), Max(newtol, Precision())),
                                       myRemoveLoopMode == 0))
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                            loopRemoved = true;
                            prevFirst   = firstpar;
                            prevLast    = lastpar;
                            continue; // repeat of fix on that edge required (to be done by caller)
                        }
                    }
                    if (newtol < MaxTolerance())
                    {
                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                        BRep_Builder B = new BRep_Builder();
                        if (dist21 < dist22)
                        {
                            tol1 = newtol;
                            B.UpdateVertex(V1, tol1);
                        }
                        else
                        {
                            tol2 = newtol;
                            B.UpdateVertex(V2, tol2);
                        }
                    }
                    else
                    {
                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                    }
                }

                // after RemoveLoop, check that self-intersection disappeared
                if (loopRemoved)
                {
                    NCollection_Sequence<IntRes2d_IntersectionPoint> pnts2d = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
                    NCollection_Sequence<gp_Pnt>                     pnts3d = new NCollection_Sequence<gp_Pnt>();
                    theAdvAnalyzer.CheckSelfIntersectingEdge(num, pnts2d, pnts3d);
                    if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
                    {
                        break;
                    }
                    // points3d.Append(pnts3d);
                    // points2d.Append(pnts2d);
                    points3d = pnts3d;
                    points2d = pnts2d;
                    BRep_Builder B = new BRep_Builder();
                    B.UpdateEdge(E, c2d, Face(), 0.0);
                    B.Range(E, Face(), f2d, l2d);
                    // newtol+=Precision();
                }
                else
                {
                    break;
                }
            }
        }

        //===============================================
        // RemoveLoopMode = 1 , insert vertex
        //===============================================
        if (myRemoveLoopMode == 1)
        {
            // after fixing will be nb+1 edges
            bool loopRemoved;
            // create a sequence of resulting edges
            NCollection_HSequence<TopoDS_Shape> TTSS = new NCollection_HSequence<TopoDS_Shape>();
            TopoDS_Edge                         E2   = new TopoDS_Edge();

            loopRemoved = false;
            //: k3 abv 24 Dec 98: BUC50070 #26682 and #30087: try to remove loop
            if (myGeomMode)
            {
                if (c2d is null)
                {
                    sae.PCurve(E, Face(), out c2d, out f2d, out l2d, false);
                }
                TopoDS_Edge E1 = new TopoDS_Edge();
                if (RemoveLoop(E, Face(), points2d.Value(1), ref E1, ref E2))
                {
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                    loopRemoved = true;
                    if (!E1.IsNull())
                    {
                        TTSS.Append(E1);
                        newtol = Max(BRep_Tool.Tolerance(E1), BRep_Tool.Tolerance(E2));
                    }
                    else
                    {
                        newtol = BRep_Tool.Tolerance(E2);
                    }
                }
            }

            TTSS.Append(E2);

            if (newtol > MaxTolerance())
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }

            ShapeExtend_WireData sewd = new ShapeExtend_WireData();
            for (int i = 1; i <= TTSS.Length(); i++)
            {
                sewd.Add(TopoDS.Edge(TTSS.Value(i)));
            }
            if (Context() is not null)
            {
                Context()!.Replace(E, sewd.InternalWire());
                UpdateWire();
            }
            else
            {
                WireData()!.Remove(num > 0 ? num : NbEdges());
                WireData()!.Add(sewd.InternalWire(), num > 0 ? num : NbEdges());
            }
            if (loopRemoved)
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE8);
            }
        }

        if (LastFixStatus(ShapeExtend_DONE) && !myShape.IsNull())
        {
            SendWarning(E, new Message_Msg("FixAdvWire.FixIntersection.MSG5")); // Edge was self-intersecting, corrected
        }

        return LastFixStatus(ShapeExtend_DONE);
    }

    private static double ComputeLocalDeviation(in TopoDS_Edge edge, in gp_Pnt pint, in gp_Pnt pnt, double f, double l,
                                                in TopoDS_Face face)
    {
        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        if (!sae.Curve3d(edge, out Geom_Curve? c3d, out double a, out double b, false))
        {
            return RealLast();
        }

        gp_Lin line = new gp_Lin(pint, new gp_Dir(new gp_Vec(pint, pnt)));

        if (sae.PCurve(edge, face, out Geom2d_Curve? Crv, out double fp, out double lp, false))
        {
            if (Crv is Geom2d_TrimmedCurve tc)
            {
                if (tc.BasisCurve() is Geom2d_Line)
                {
                    f = a + (f - fp) * (b - a) / (lp - fp);
                    l = a + (l - fp) * (b - a) / (lp - fp);
                }
            }
        }

        const int NSEG = 10;
        double    step = (l - f) / NSEG;
        double    dev  = 0.0;
        for (int i = 1; i < NSEG; i++)
        {
            gp_Pnt p = c3d!.Value(f + i * step);
            double d = line.Distance(p);
            if (dev < d)
            {
                dev = d;
            }
        }
        return dev;
    }

    private bool FixIntersectingEdges(int num)
    {
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady() || NbEdges() < 2)
        {
            return false;
        }

        // analysis
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d       = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<gp_Pnt>                     points3d       = new NCollection_Sequence<gp_Pnt>();
        NCollection_Sequence<double>                     errors         = new NCollection_Sequence<double>();
        ShapeAnalysis_Wire?                              theAdvAnalyzer = myAnalyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckIntersectingEdges(num, points2d, points3d, errors);
        if (theAdvAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        // rln 03/02/98: CSR#BUC50004 entity 56 (to avoid later inserting lacking edge)
        //: l0  bool isLacking = myAnalyzer->CheckLacking ( num );

        // action: increase tolerance of vertex

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());
        TopoDS_Edge          E1   = sbwd.Edge(n1);
        TopoDS_Edge          E2   = sbwd.Edge(n2);
        if (Context() is not null)
        {
            E1 = TopoDS.Edge(Context()!.InternalApply(sbwd.Edge(n1)));
            E2 = TopoDS.Edge(Context()!.InternalApply(sbwd.Edge(n2)));
        }

        bool isForward1 = (E1.Orientation() == TopAbs_FORWARD);
        bool isForward2 = (E2.Orientation() == TopAbs_FORWARD);
        BRep_Tool.Range(E1, Face(), out double a1, out double b1);
        BRep_Tool.Range(E2, Face(), out double a2, out double b2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex      Vp  = sae.FirstVertex(E1);
        TopoDS_Vertex      V1  = sae.LastVertex(E1);
        TopoDS_Vertex      V2  = sae.FirstVertex(E2);
        TopoDS_Vertex      Vn  = sae.LastVertex(E2);

        double tol = BRep_Tool.Tolerance(V1);
        gp_Pnt pnt = BRep_Tool.Pnt(V1);

        double prevRange1 = RealLast(), prevRange2 = RealLast();
        bool   cutEdge1 = false, cutEdge2 = false;
        bool   IsCutLine     = false;
        bool   isChangedEdge = false;

        BRep_Builder B = new BRep_Builder();

        int nb = points3d.Length();
        for (int i = 1; i <= nb; i++)
        {
            IntRes2d_IntersectionPoint IP     = points2d.Value(i);
            double                     param1 = (num == 1 ? IP.ParamOnSecond() : IP.ParamOnFirst());
            double                     param2 = (num == 1 ? IP.ParamOnFirst() : IP.ParamOnSecond());

            double newRange1 = Math.Abs((isForward1 ? a1 : b1) - param1);
            double newRange2 = Math.Abs((isForward2 ? b2 : a2) - param2);
            if (newRange1 > prevRange1 && newRange2 > prevRange2)
            {
                continue;
            }

            gp_Pnt pint   = points3d.Value(i);
            double rad    = errors.Value(i);
            double newtol = 1.0001 * (pnt.Distance(pint) + rad);

            // GeomAdaptor_Surface& Ads = myAnalyzer->Surface()->Adaptor3d()->ChangeSurface();

            //: r8 abv 12 Apr 99: try increasing tolerance of edge

            bool locMayEdit = myTopoMode;
            // Always try to modify the tolerance firstly as a better solution
            if (/*! myTopoMode &&*/ newtol > tol)
            {
                double te1 =
                    rad + ComputeLocalDeviation(E1, pint, pnt, param1, (isForward1 ? b1 : a1), Face());
                double te2 =
                    rad + ComputeLocalDeviation(E2, pint, pnt, (isForward2 ? a2 : b2), param2, Face());
                double maxte = Max(te1, te2);
                if (maxte < MaxTolerance() && maxte < newtol)
                {
                    if (BRep_Tool.Tolerance(E1) < te1 || BRep_Tool.Tolerance(E2) < te2)
                    {
                        // Make copy of edges.
                        if (Context() is not null)
                        {
                            isChangedEdge = true; // To avoid double copying of vertexes.

                            // Intersection point of two base edges.
                            ShapeBuild_Edge aSBE = new ShapeBuild_Edge();
                            TopoDS_Vertex   VV1  = Context()!.CopyVertex(V1);

                            TopoDS_Vertex VVp = Vp;
                            TopoDS_Vertex VVn = Vn;
                            if (Vp.IsSame(Vn))
                            {
                                // Should modify only one vertex.
                                VVp = Context()!.CopyVertex(Vp);
                                VVn = VVp;
                            }
                            else
                            {
                                VVp = Context()!.CopyVertex(Vp);
                                VVn = Context()!.CopyVertex(Vn);
                            }

                            TopoDS_Edge EE1 = aSBE.InternalCopyReplaceVertices(E1, VVp, VV1);
                            TopoDS_Edge EE2 = aSBE.InternalCopyReplaceVertices(E2, VV1, VVn);

                            Context()!.Replace(E1, EE1);
                            Context()!.Replace(E2, EE2);

                            UpdateWire();
                            E1 = sbwd.Edge(n1);
                            E2 = sbwd.Edge(n2);
                            Vp = sae.FirstVertex(E1);
                            V1 = sae.LastVertex(E1);
                            V2 = sae.FirstVertex(E2);
                            Vn = sae.LastVertex(E2);
                        }

                        B.UpdateEdge(E1, 1.000001 * te1);
                        B.UpdateVertex(sae.FirstVertex(E1), 1.000001 * te1);
                        B.UpdateVertex(sae.LastVertex(E1), 1.000001 * te1);
                        B.UpdateEdge(E2, 1.000001 * te2);
                        B.UpdateVertex(sae.FirstVertex(E2), 1.000001 * te2);
                        B.UpdateVertex(sae.LastVertex(E2), 1.000001 * te2);

                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE6);
                        locMayEdit = false;
                    }
                    newtol = 1.000001 * maxte;
                }
            }

            if (locMayEdit || newtol <= MaxTolerance())
            {
                prevRange1 = newRange1;
                prevRange2 = newRange2;
                if (locMayEdit)
                {
                    newtol = 1.0001 * (pnt.Distance(pint) + rad);
                    //: j6 abv 7 Dec 98: ProSTEP TR10 r0601_id.stp #57676 & #58586: do not cut edges because of
                    //: influence on adjacent faces
                    ShapeFix_SplitTool aTool = new ShapeFix_SplitTool();

                    if (!aTool.CutEdge(E1, (isForward1 ? a1 : b1), param1, Face(), ref IsCutLine))
                    {
                        if (V1.IsSame(Vp))
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                        }
                        else
                        {
                            locMayEdit = false;
                        }
                    }
                    else
                    {
                        cutEdge1 = true; //: h4
                    }

                    if (!aTool.CutEdge(E2, (isForward2 ? b2 : a2), param2, Face(), ref IsCutLine))
                    {
                        if (V2.IsSame(Vn))
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                        }
                        else
                        {
                            locMayEdit = false;
                        }
                    }
                    else
                    {
                        cutEdge2 = true; //: h4
                    }
                }

                if (locMayEdit && newRange1 <= prevRange1 && newRange2 <= prevRange2 && // rln 09/01/98
                    BRep_Tool.SameParameter(E1) && BRep_Tool.SameParameter(E2))
                {
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                    pnt = pint;
                    if (tol <= rad)
                    {
                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                        tol = 1.001 * rad;
                    }
                }
                else
                {
                    if (IsCutLine)
                    {
                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                        pnt = pint;
                        if (tol <= rad)
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                            tol = 1.001 * rad;
                        }
                    }
                    else
                    { // else increase tolerance
                        if (tol < newtol)
                        { // rln 07.04.99 CCI60005-brep.igs
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                            tol = newtol;
                        }
                    }
                }
            }
            else
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }
        }

        if (!LastFixStatus(ShapeExtend_DONE))
        {
            return false;
        }

        if (isChangedEdge)
        {
            B.UpdateVertex(V1, pnt, tol);
            B.UpdateVertex(V2, pnt, tol);
        }
        else
        {
            if (Context() is not null)
            {
                if (V1.IsSame(V2))
                {
                    Context()!.CopyVertex(V1, pnt, tol);
                }
                else
                {
                    Context()!.CopyVertex(V1, pnt, tol);
                    Context()!.CopyVertex(V2, pnt, tol);
                }
            }
            else
            {
                B.UpdateVertex(V1, pnt, tol);
                B.UpdateVertex(V2, pnt, tol);
            }
        }

        //: h4: make edges SP (after all cuts: t4mug.stp #3730+#6460)
        if (cutEdge1)
        {
            if (Context() is not null)
            {
                E1 = TopoDS.Edge(Context()!.InternalApply(E1));
            }
            myFixEdge.FixSameParameter(E1);
        }
        if (cutEdge2 && !IsCutLine)
        {
            if (Context() is not null)
            {
                E2 = TopoDS.Edge(Context()!.InternalApply(E2));
            }
            myFixEdge.FixSameParameter(E2);
        }
        if (cutEdge1 || cutEdge2)
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE7);
        }
        if (!myShape.IsNull())
        {
            SendWarning(new Message_Msg("FixAdvWire.FixIntersection.MSG10")); // Edges were intersecting, corrected
        }
        return true;
    }

    // pdn 17.03.99 fixing non adjacent intersection by increasing tolerance of vertex

    private bool FixIntersectingEdges(int num1, int num2)
    {
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d       = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<gp_Pnt>                     points3d       = new NCollection_Sequence<gp_Pnt>();
        NCollection_Sequence<double>                     errors         = new NCollection_Sequence<double>();
        ShapeAnalysis_Wire?                              theAdvAnalyzer = myAnalyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckIntersectingEdges(num1, num2, points2d, points3d, errors);
        if (theAdvAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }
        NCollection_Array1<gp_Pnt>       vertexPoints = new NCollection_Array1<gp_Pnt>(1, 4);
        NCollection_Array1<double>       vertexTolers = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<double>       newTolers    = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<TopoDS_Shape> vertices     = new NCollection_Array1<TopoDS_Shape>(1, 4);
        newTolers.Init(0);

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num1 > 0 ? num1 : sbwd.NbEdges());
        int                  n1   = (num2 > 1 ? num2 : sbwd.NbEdges());
        if (n1 == n2)
        {
            return false;
        }

        TopoDS_Edge edge1 = sbwd.Edge(n1);
        TopoDS_Edge edge2 = sbwd.Edge(n2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        vertices[1] = sae.FirstVertex(edge1);
        vertices[2] = sae.LastVertex(edge1);
        vertices[3] = sae.FirstVertex(edge2);
        vertices[4] = sae.LastVertex(edge2);

        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= 4; i++)
        {
            vertexPoints[i] = BRep_Tool.Pnt(TopoDS.Vertex(vertices[i]));
            vertexTolers[i] = BRep_Tool.Tolerance(TopoDS.Vertex(vertices[i]));
        }

        double aNewTolEdge1 = 0.0, aNewTolEdge2 = 0.0;
        int    nb = points3d.Length();
        for (i = 1; i <= nb; i++)
        {
            gp_Pnt pint = points3d.Value(i);

            // searching for the nearest vertexies to the intersection point
            double aVtx1Param = 0.0, aVtx2Param = 0.0;
            int    aVC1, aVC2;
            double aMinDist          = RealLast();
            gp_Pnt aNearestVertex    = new gp_Pnt();
            double aNecessaryVtxTole = 0.0;
            for (aVC1 = 1; aVC1 <= 2; aVC1++)
            {
                for (aVC2 = 3; aVC2 <= 4; aVC2++)
                {

                    double aVtxIPDist  = pint.Distance(vertexPoints[aVC1]);
                    double aVtxVtxDist = vertexPoints[aVC1].Distance(vertexPoints[aVC2]);
                    if (aMinDist > aVtxIPDist && aVtxIPDist > aVtxVtxDist)
                    {
                        aNecessaryVtxTole = aVtxVtxDist;
                        aNearestVertex    = vertexPoints[aVC1];
                        aMinDist          = aVtxIPDist;
                        aVtx1Param        = BRep_Tool.Parameter(TopoDS.Vertex(vertices[aVC1]), edge1);
                        aVtx2Param        = BRep_Tool.Parameter(TopoDS.Vertex(vertices[aVC2]), edge2);
                    }
                }
            }

            // calculation of necessary tolerances of edges
            IntRes2d_IntersectionPoint IP     = points2d.Value(i);
            double                     param1 = IP.ParamOnFirst();
            double                     param2 = IP.ParamOnSecond();
            Geom_Curve?                aCurve1, aCurve2;
            aCurve1 = BRep_Tool.Curve(edge1, out TopLoc_Location L1, out _, out _);
            aCurve2 = BRep_Tool.Curve(edge2, out TopLoc_Location L2, out _, out _);

            // if aMinDist lower than resolution than the intersection point lyes inside the vertex
            if (aMinDist < gp.Resolution())
            {
                continue;
            }

            double aMaxEdgeTol1 = 0.0, aMaxEdgeTol2 = 0.0;
            if (aMinDist < RealLast() && aCurve1 is not null && aCurve2 is not null)
            {
                gp_Lin aLig = new gp_Lin(aNearestVertex, new gp_Dir(new gp_Vec(aNearestVertex, pint)));
                int    aPointsC;
                double du1   = 0.05 * (param1 - aVtx1Param);
                double du2   = 0.05 * (param2 - aVtx2Param);
                double tole1 = BRep_Tool.Tolerance(edge1);
                double tole2 = BRep_Tool.Tolerance(edge2);
                for (aPointsC = 2; aPointsC < 19; aPointsC++)
                {
                    double u  = aVtx1Param + aPointsC * du1;
                    gp_Pnt P1 = aCurve1.Value(u);
                    P1.Transform(L1.Transformation());
                    double d1 = aLig.Distance(P1) * 2.0000001;
                    if (d1 > tole1 && d1 > aMaxEdgeTol1)
                    {
                        aMaxEdgeTol1 = d1;
                    }

                    u         = aVtx2Param + aPointsC * du2;
                    gp_Pnt P2 = aCurve2.Value(u);
                    P2.Transform(L2.Transformation());
                    double d2 = aLig.Distance(P2) * 2.0000001;
                    if (d2 > tole2 && d2 > aMaxEdgeTol2)
                    {
                        aMaxEdgeTol2 = d2;
                    }
                }
                if (aMaxEdgeTol1 == 0.0 && aMaxEdgeTol2 == 0.0)
                {
                    continue;
                }
                // if the vertexies are far than tolerances so
                // we do not need to increase edge tolerance
                if (aNecessaryVtxTole > Max(aMaxEdgeTol1, tole1)
                    || aNecessaryVtxTole > Max(aMaxEdgeTol2, tole2))
                {
                    aMaxEdgeTol1 = 0.0;
                    aMaxEdgeTol2 = 0.0;
                }
            }

            double rad    = errors.Value(i);
            double finTol = RealLast();
            int    rank   = 1;
            for (int j = 1; j <= 4; j++)
            {
                double newtol = 1.0001 * (pint.Distance(vertexPoints[j]) + rad);
                if (newtol < finTol)
                {
                    rank   = j;
                    finTol = newtol;
                }
            }
            if (finTol <= MaxTolerance())
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                if (newTolers[rank] < finTol)
                {
                    if (Max(aMaxEdgeTol1, aMaxEdgeTol2) < finTol && (aMaxEdgeTol1 > 0 || aMaxEdgeTol2 > 0))
                    {
                        aNewTolEdge1 = Max(aNewTolEdge1, aMaxEdgeTol1);
                        aNewTolEdge2 = Max(aNewTolEdge2, aMaxEdgeTol2);
                    }
                    else
                    {
                        newTolers[rank] = finTol;
                    }
                }
            }
            else
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }
        }

        BRep_Builder B = new BRep_Builder();
        // update of tolerances of edges
        if (aNewTolEdge1 > 0)
        {
            for (i = 1; i <= 2; i++)
            {
                if (aNewTolEdge1 > Max(vertexTolers[i], newTolers[i]))
                {
                    newTolers[i] = aNewTolEdge1;
                }
            }
            B.UpdateEdge(edge1, aNewTolEdge1);
        }
        if (aNewTolEdge2 > 0)
        {
            for (i = 3; i <= 4; i++)
            {
                if (aNewTolEdge2 > Max(vertexTolers[i], newTolers[i]))
                {
                    newTolers[i] = aNewTolEdge2;
                }
            }
            B.UpdateEdge(edge2, aNewTolEdge2);
        }

        // update of tolerances of vertexies
        for (i = 1; i <= 4; i++)
        {
            if (newTolers[i] > 0)
            {
                B.UpdateVertex(TopoDS.Vertex(vertices[i]), newTolers[i]);
            }
        }

        if (!myShape.IsNull())
        {
            SendWarning(new Message_Msg("FixAdvWire.FixIntersection.MSG10")); // Edges were intersecting, corrected
        }
        return true;
    }

    //=======================================================================
    // function : FixLacking
    // purpose  : Test if two adjucent edges are disconnected in 2d (while connected
    //           in 3d), and in that case either increase tolerance of the vertex or
    //           add a new edge (straight in 2d space), in order to close wire in 2d.
    //           Returns True if edge was added or tolerance was increased.
    // NOTE     : Is to be run after FixDegenerated
    // Algorithm: 1. Compute the 2d gap between edges and calculate a tolerance
    //              which should have vertex in order to comprise the gap
    //              (using GeomAdaptor_Surface); computed value is inctol
    //           2. If inctol < tol of vertex, return False (everything is OK)
    //           3. If inctol < Precision, just increase tolerance of vertex to inctol
    //           4. Else (if both edges are not degenerated) try to add new edge
    //              with straight pcurve (in order to close the gap):
    //              a) if flag MayEdit is False
    //                 1. if inctol < MaxTolerance, increase tolerance of vertex to inctol
    //                 2. else try to add degenerated edge (check that middle point of
    //                    that pcurveis inside the vertex)
    //              b) if MayEdit is True
    //                 1. try to replace big vertex with two new small vertices
    //                    connected by new edge. This is made if there is a 3d space
    //                    between ends of adjacent edges.
    //                 2. if inctol < MaxTolerance, increase tolerance of vertex to inctol
    //                 3. else add either degenerated or closed edge (if middle point
    //                    of a pcurve of a new edge is inside the vertex, then
    //                    degenerated edge is added, else new edge is closed).
    //           5. If new edge cannot be added, but inctol < MaxTolerance,
    //              when increase tolerance of vertex to a value of inctol
    // Short list of some internal variables:
    // tol    - tolerance of vertex
    // tol2d  - tolerance in parametric space of the surface corresponding to 2*tol
    // dist2d - distance between ends of pcurves of edges (2d)
    // inctol - tolerance required for vertex to close 2d gap (=tol*dist2d/tol2d)
    // tol1, tol2 - tolerances of edges, tol0 = tol1 + tol2
    // p3d1, p3d2 - ends of 3d curves of edges
    //=======================================================================
    //: h2 abv 28 May 98: merged modifications by abv 22 Apr 98, gka 27 May 98
    // and pdn 25 May 98 concerning lacking closed or degenerated edges
    // Example files: r0501_pe #107813, UKI60107-6 250, UKI60107-3 1577.

    //: s2 abv 21 Apr 99: add functionality for bending pcurve
    private static bool TryBendingPCurve(in TopoDS_Edge E, in TopoDS_Face face, in gp_Pnt2d p2d, bool end,
                                         ref Geom2d_Curve? c2d, ref double first, ref double last, ref double tol)
    {
        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        if (!sae.PCurve(E, face, out c2d, out first, out last, false))
        {
            return false;
        }

        {
            try
            {
                Geom2d_BSplineCurve? bs;
                if (c2d is Geom2d_BSplineCurve)
                {
                    bs = c2d.Copy() as Geom2d_BSplineCurve;
                }
                else // if ( c2d->IsKind(STANDARD_TYPE(Geom2d_Line)) )
                {
                    Geom2d_TrimmedCurve trim = new Geom2d_TrimmedCurve(c2d!, first, last);
                    bs                       = Geom2dConvert.CurveToBSplineCurve(trim);
                }
                if (bs is null)
                {
                    return false;
                }
                // OCCT.NET: frozen: the poles are moved and the curve segmented in place; a copy above in OCCT too,
                // but guard against a converter that hands back a shared curve
                if (bs.IsFrozen())
                {
                    bs = (Geom2d_BSplineCurve)bs.Copy();
                }

                double par = (end ? last : first);
                if (Math.Abs(bs.FirstParameter() - par) < Occt.Precision.PConfusion()
                    && bs.Multiplicity(1) > bs.Degree())
                {
                    bs.SetPole(1, p2d);
                }
                else if (Math.Abs(bs.LastParameter() - par) < Occt.Precision.PConfusion()
                         && bs.Multiplicity(bs.NbKnots()) > bs.Degree())
                {
                    bs.SetPole(bs.NbPoles(), p2d);
                }
                else
                {
                    bs.Segment(first, last);
                    if (Math.Abs(bs.FirstParameter() - par) < Occt.Precision.PConfusion()
                        && bs.Multiplicity(1) > bs.Degree())
                    {
                        bs.SetPole(1, p2d);
                    }
                    else if (Math.Abs(bs.LastParameter() - par) < Occt.Precision.PConfusion()
                             && bs.Multiplicity(bs.NbKnots()) > bs.Degree())
                    {
                        bs.SetPole(bs.NbPoles(), p2d);
                    }
                    else
                    {
                        return false;
                    }
                }
                c2d = bs;

                if (!TryNewPCurve(E, face, ref c2d, ref first, ref last, ref tol))
                {
                    return false;
                }
            }
            catch (Standard_Failure)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Fixes a lacking edge between the edge <paramref name="num"/> and the one before it (connected in 3d, not in
    /// 2d): increases the vertex tolerance, bends the pcurves, or adds an edge (straight in 2d).
    /// </summary>
    public bool FixLacking(int num, bool force = false)
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        //=============
        // First phase: analysis whether the problem (gap) exists
        gp_Pnt2d p2d1 = new gp_Pnt2d(), p2d2 = new gp_Pnt2d();
        myAnalyzer.CheckLacking(num, (force ? Precision() : 0.0), ref p2d1, ref p2d2);
        if (myAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        //=============
        // Second phase: collection of data necessary for further analysis

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());
        TopoDS_Edge          E1   = sbwd.Edge(n1);
        TopoDS_Edge          E2   = sbwd.Edge(n2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex      V1  = sae.LastVertex(E1);
        TopoDS_Vertex      V2  = sae.FirstVertex(E2);
        double             tol = Max(BRep_Tool.Tolerance(V1), BRep_Tool.Tolerance(V2));

        double Prec   = Precision();
        double dist2d = myAnalyzer.MaxDistance2d();
        double inctol = myAnalyzer.MaxDistance3d();

        TopoDS_Face            face = myAnalyzer.Face();
        ShapeAnalysis_Surface? surf = myAnalyzer.Surface();

        gp_Pnt p3d1 = new gp_Pnt(), p3d2 = new gp_Pnt();
        double tol1 = Occt.Precision.Confusion(), tol2 = Occt.Precision.Confusion(); // SK

        //=============
        //: s2 abv 21 Apr 99: Speculation: try bending pcurves
        double        bendtol1 = 0.0, bendtol2 = 0.0;
        Geom2d_Curve? bendc1 = null, bendc2 = null;
        double        bendf1 = 0.0, bendl1 = 0.0, bendf2 = 0.0, bendl2 = 0.0;
        if (myGeomMode && !BRep_Tool.IsClosed(E1, face) && !BRep_Tool.IsClosed(E2, face))
        {
            gp_Pnt2d p2d = new gp_Pnt2d(0.5 * (p2d1.XY() + p2d2.XY()));
            bool     ok1 = TryBendingPCurve(E1,
                                        face,
                                        p2d,
                                        E1.Orientation() == TopAbs_FORWARD,
                                        ref bendc1,
                                        ref bendf1,
                                        ref bendl1,
                                        ref bendtol1);
            bool     ok2 = TryBendingPCurve(E2,
                                        face,
                                        p2d,
                                        E2.Orientation() == TopAbs_REVERSED,
                                        ref bendc2,
                                        ref bendf2,
                                        ref bendl2,
                                        ref bendtol2);
            if (ok1 && !ok2)
            {
                bendtol2 = BRep_Tool.Tolerance(E2);
                ok1      = TryBendingPCurve(E1,
                                       face,
                                       p2d2,
                                       E1.Orientation() == TopAbs_FORWARD,
                                       ref bendc1,
                                       ref bendf1,
                                       ref bendl1,
                                       ref bendtol1);
            }
            else if (!ok1 && ok2)
            {
                bendtol1 = BRep_Tool.Tolerance(E1);
                ok2      = TryBendingPCurve(E2,
                                       face,
                                       p2d1,
                                       E2.Orientation() == TopAbs_FORWARD,
                                       ref bendc2,
                                       ref bendf2,
                                       ref bendl2,
                                       ref bendtol2);
            }
            if (!ok1 && !ok2)
            {
                bendc1 = null;
            }
        }

        //=============
        // Third phase: analyse how to fix the problem

        // selector of solutions
        bool doIncrease  = false; // increase tolerance
        bool doAddLong   = false; // add long 3d edge in replacement of a vertex
        bool doAddClosed = false; // add closed 3d edge
        bool doAddDegen  = false; // add degenerated edge
        bool doBend      = false; //: s2 bend pcurves

        // if bending is OK with existing tolerances of edges, take it
        if (bendc1 is not null && bendc2 is not null
            && ((bendtol1 < BRep_Tool.Tolerance(E1) && bendtol2 < BRep_Tool.Tolerance(E2))
                || (inctol < Prec && bendtol1 < inctol && bendtol2 < inctol)))
        {
            doBend = true;

            // is it OK just to increase tolerance (to a value less than preci)?
        }
        else if (inctol < Prec)
        {
            doIncrease = true;

            // If increase is not OK or force, try to find other solutions (adding edge)
        }
        else if (!BRep_Tool.Degenerated(E2) && !BRep_Tool.Degenerated(E1))
        {

            // analyze the 3d space btw edges: is it enough to add long 3d edge?
            if (myTopoMode)
            {
                if (!sae.Curve3d(E1, out Geom_Curve? c3d, out double a, out double b, true))
                { // cannot work
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                    return false;
                }
                p3d1             = c3d!.Value(b);
                double dist2d3d1 = p3d1.Distance(surf!.Value(p2d1));
                if (!sae.Curve3d(E2, out c3d, out a, out b, true))
                { // cannot work
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                    return false;
                }
                p3d2             = c3d!.Value(a);
                double dist2d3d2 = p3d2.Distance(surf.Value(p2d2));

                tol1 = Max(BRep_Tool.Tolerance(E1), dist2d3d1);
                tol2 = Max(BRep_Tool.Tolerance(E2), dist2d3d2);
                //: c5  double tol0 = Max ( tol1 + tol2, thepreci );
                double tol0    = tol1 + tol2; //: c5 abv 26 Feb 98: CTS17806 #44418
                double dist3d2 = p3d1.SquareDistance(p3d2);

                // is it OK to add a long 3d edge?
                if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE2)
                    && //: 81 abv 20 Jan 98: don`t add back-going edges (zigzags)
                    dist3d2 > 1.25 * tol0 * tol0
                    && (force || dist3d2 > Prec * Prec || inctol > MaxTolerance()))
                {
                    doAddLong = true;
                }
            }

            //: h6 abv 25 Jun 98: BUC40132 6361: try to increase tol up to MaxTol if not add
            if (!doAddLong && inctol < MaxTolerance()
                && !myAnalyzer.Surface()!.IsDegenerated(p2d1, p2d2, 2.0 * tol, 10.0))
            { //: p7
                if (bendc1 is not null && bendc2 is not null && bendtol1 < inctol && bendtol2 < inctol)
                {
                    doBend = true;
                }
                else
                {
                    doIncrease = true;
                }
            }
            else

            // else try to add either degenerated or closed edge
            if (!doAddLong)
            {
                gp_Pnt pV = new gp_Pnt(0.5 * (BRep_Tool.Pnt(V1).XYZ() + BRep_Tool.Pnt(V2).XYZ()));
                gp_Pnt pm = myAnalyzer.Surface()!.Value(new gp_Pnt2d(0.5 * (p2d1.XY() + p2d2.XY())));

                double dist = pV.Distance(pm);
                if (dist <= tol)
                {
                    doAddDegen = true;
                }
                else if (myTopoMode)
                {
                    doAddClosed = true;
                }
                else if (dist <= MaxTolerance())
                { //: r7 abv 12 Apr 99: t3d_opt.stp #14245 after S4136
                    doAddDegen = true;
                    doIncrease = true;
                    inctol     = dist;
                }
            }
        }

        else if (!BRep_Tool.Degenerated(E2) && BRep_Tool.Degenerated(E1))
        {
            // create new degenerated edge and replace E1 to new edge
        }
        else if (BRep_Tool.Degenerated(E2) && !BRep_Tool.Degenerated(E1))
        {
            // create new degenerated edge and replace E2 to new edge
        }

        //=============
        // Third phase - do the fixes
        BRep_Builder B = new BRep_Builder();

        // add edge
        if (doAddLong || doAddDegen || doAddClosed)
        {

            // construct new vertices
            TopoDS_Vertex newV1, newV2;
            if (doAddLong)
            {
                newV1 = new BRepBuilderAPI_MakeVertex(p3d1).InternalVertex();
                newV1.Reverse();
                newV2 = new BRepBuilderAPI_MakeVertex(p3d2).InternalVertex();
                B.UpdateVertex(newV1, 1.001 * tol1);
                B.UpdateVertex(newV2, 1.001 * tol2);
            }
            else
            {
                newV1 = V1;
                newV2 = V2;
            }

            // prepare new edge
            TopoDS_Edge edge = new TopoDS_Edge();
            B.MakeEdge(ref edge);
            if (doAddDegen)
            {
                B.Degenerated(edge, true); // sln: do it before adding curve
            }
            gp_Vec2d    v12       = new gp_Vec2d(p2d1, p2d2);
            Geom2d_Line theLine2d = new Geom2d_Line(p2d1, new gp_Dir2d(v12));
            B.UpdateEdge(edge, theLine2d, face, Occt.Precision.Confusion());
            B.Range(edge, face, 0, dist2d);
            B.Add(edge, newV1.Oriented(TopAbs_FORWARD));
            B.Add(edge, newV2.Oriented(TopAbs_REVERSED));
            ShapeBuild_Edge sbe = new ShapeBuild_Edge();
            if (!doAddDegen && !sbe.BuildCurve3d(edge))
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
                return false;
            }

            // if long edge is added, replace vertices of adjacent edges
            if (doAddLong)
            {

                // replace 1st edge (n1==n2 - special case: wire consists of one edge)
                TopoDS_Edge edge1 = sbe.InternalCopyReplaceVertices(E1, (n1 == n2 ? newV2 : new TopoDS_Vertex()), newV1);
                sbwd.Set(edge1, n1);
                if (Context() is not null)
                {
                    Context()!.Replace(E1, edge1);
                    // actually, this will occur only in context of single face
                    // hence, recording to ReShape is rather for tracking modifications
                    // than for keeping sharing
                    Context()!.Replace(V1, newV1.Oriented(V1.Orientation()));
                    if (!V1.IsSame(V2))
                    {
                        Context()!.Replace(V2, newV2.Oriented(V2.Orientation()));
                    }
                }
                // replace 2nd edge
                if (n1 != n2)
                {
                    TopoDS_Edge edge2 = sbe.InternalCopyReplaceVertices(E2, newV2, new TopoDS_Vertex());
                    sbwd.Set(edge2, n2);
                    if (Context() is not null)
                    {
                        Context()!.Replace(E2, edge2);
                    }
                }
                if (Context() is not null)
                {
                    UpdateWire();
                }
            }

            // insert new edge
            if (doAddDegen)
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            }
            else if (!doAddLong)
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
            }
            sbwd.Add(edge, n2);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }

        // else try to increase tol up to MaxTol
        else if (inctol > tol && inctol < MaxTolerance())
        {
            if (bendc1 is not null && bendc2 is not null && bendtol1 < inctol && bendtol2 < inctol)
            {
                doBend = true;
            }
            else
            {
                doIncrease = true;
            }
        }

        // bend pcurves
        if (doBend)
        { //: s2 abv 21 Apr 99
            B.UpdateEdge(E1, bendc1, face, bendtol1);
            B.Range(E1, face, bendf1, bendl1);
            B.UpdateEdge(E2, bendc2, face, bendtol2);
            B.Range(E2, face, bendf2, bendl2);
            B.UpdateVertex(sae.FirstVertex(E1), bendtol1);
            B.UpdateVertex(sae.LastVertex(E1), bendtol1);
            B.UpdateVertex(sae.FirstVertex(E2), bendtol2);
            B.UpdateVertex(sae.LastVertex(E2), bendtol2);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
            //: s3 abv 22 Apr 99: PRO7187 #11534: self-intersection not detected unitil curve is bent (!)
            FixSelfIntersectingEdge(n1);
            FixSelfIntersectingEdge(n2);
            FixIntersectingEdges(n2); // skl 24.04.2003 for OCC58
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
        }

        // increase vertex tolerance
        if (doIncrease)
        {
            B.UpdateVertex(V1, 1.001 * inctol);
            B.UpdateVertex(V2, 1.001 * inctol);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        if (LastFixStatus(ShapeExtend_DONE))
        {
            return true;
        }

        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        return false;
    }

    /// <summary>Fixes notched edges (an edge going back along its neighbour): splits it and removes the notch.</summary>
    public bool FixNotchedEdges()
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        ShapeAnalysis_Wire theAdvAnalyzer = myAnalyzer;
        TopoDS_Face        face           = Face();
        if (Context() is not null)
        {
            UpdateWire();
        }
        ShapeExtend_WireData sewd = WireData()!;

        for (int i = 1; i <= NbEdges() && NbEdges() > 2; i++)
        {
            double param    = 0;
            int    toRemove = 0;
            if (theAdvAnalyzer.CheckNotchedEdges(i, ref toRemove, ref param, MinTolerance()))
            {
                int                n2            = (i > 0) ? i : NbEdges();
                int                n1            = (n2 > 1) ? n2 - 1 : NbEdges();
                bool               isRemoveFirst = (n1 == toRemove);
                int                toSplit       = (n2 == toRemove ? n1 : n2);
                TopoDS_Edge        splitE        = sewd.Edge(toSplit);
                ShapeAnalysis_Edge sae           = new ShapeAnalysis_Edge();
                sae.PCurve(splitE, face, out Geom2d_Curve? c2d, out double a, out double b, true);
                ShapeBuild_Edge    sbe    = new ShapeBuild_Edge();
                TopAbs_Orientation orient = splitE.Orientation();

                // check whether the whole edges should be removed - this is the case
                // when split point coincides with the end of the edge;
                // for closed edges split point may fall at the other end (see issue #0029780)
                if (Math.Abs(param - (isRemoveFirst ? b : a)) <= Occt.Precision.PConfusion()
                    || (sae.IsClosed3d(splitE)
                        && Math.Abs(param - (isRemoveFirst ? a : b)) <= Occt.Precision.PConfusion()))
                {
                    FixDummySeam(n1);
                    // The seam edge is removed from the list. So, need to step back to avoid missing of edge
                    // processing
                    i--;
                }
                else // perform splitting of the edge and adding to wire
                {
                    // pdn check if it is necessary
                    if (Math.Abs((isRemoveFirst ? a : b) - param) < Occt.Precision.PConfusion())
                    {
                        continue;
                    }

                    ShapeAnalysis_TransferParametersProj transferParameters = new ShapeAnalysis_TransferParametersProj();
                    transferParameters.SetMaxTolerance(MaxTolerance());
                    transferParameters.Init(splitE, face);
                    double first, last;
                    if (a < b)
                    {
                        first = a;
                        last  = b;
                    }
                    else
                    {
                        first = b;
                        last  = a;
                    }
                    TopoDS_Vertex Vnew = new TopoDS_Vertex();
                    BRep_Builder  B    = new BRep_Builder();
                    B.MakeVertex(ref Vnew,
                                 Analyzer().Surface()!.Value(c2d!.Value(param)),
                                 Occt.Precision.Confusion());
                    TopoDS_Edge wE = splitE;
                    wE.Orientation(TopAbs_FORWARD);
                    TopoDS_Shape aTmpShape = Vnew.Oriented(TopAbs_REVERSED); // for porting
                    TopoDS_Edge newE1 =
                        sbe.InternalCopyReplaceVertices(wE, sae.FirstVertex(wE), TopoDS.Vertex(aTmpShape));
                    sbe.CopyPCurves(newE1, wE);
                    transferParameters.TransferRange(newE1, first, param, true);
                    B.SameRange(newE1, false);
                    B.SameParameter(newE1, false);
                    aTmpShape = Vnew.Oriented(TopAbs_FORWARD);
                    TopoDS_Edge newE2 =
                        sbe.InternalCopyReplaceVertices(wE, TopoDS.Vertex(aTmpShape), sae.LastVertex(wE));
                    sbe.CopyPCurves(newE2, wE);
                    transferParameters.TransferRange(newE2, param, last, true);
                    B.SameRange(newE2, false);
                    B.SameParameter(newE2, false);

                    if (Context() is not null)
                    {
                        TopoDS_Wire wire = new TopoDS_Wire();
                        B.MakeWire(ref wire);
                        B.Add(wire, newE1);
                        B.Add(wire, newE2);
                        Context()!.Replace(wE, wire);
                    }

                    newE1.Orientation(orient);
                    newE2.Orientation(orient);
                    if (orient == TopAbs_REVERSED)
                    {
                        TopoDS_Edge tmp = newE2;
                        newE2           = newE1;
                        newE1           = tmp;
                    }

                    bool isRemoveLast = ((n1 == NbEdges()) && (n2 == 1));
                    sewd.Set(newE1, toSplit);
                    sewd.Add(newE2, (toSplit == NbEdges() ? 0 : toSplit + 1));

                    FixDummySeam(isRemoveLast ? NbEdges() : toRemove);
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                }

                i--;
                if (Context() is not null)
                { // skl 07.03.2002 for OCC180
                    UpdateWire();
                }
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }
        }
        myStatusNotches = myLastFixStatus;
        return LastFixStatus(ShapeExtend_DONE);
    }

    private static void CopyReversePcurves(in TopoDS_Edge toedge, in TopoDS_Edge fromedge, bool reverse)
    {
        TopLoc_Location fromLoc = fromedge.Location();
        TopLoc_Location toLoc   = toedge.Location();
        // OCCT.NET: frozen: the source edge's list is only read (OCCT walks ChangeCurves())
        for (var fromitcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)fromedge.TShape()!).Curves());
             fromitcr.More();
             fromitcr.Next())
        {
            if (fromitcr.Value() is not BRep_GCurve fromGC)
            {
                continue;
            }
            if (fromGC.IsCurveOnSurface())
            {
                Geom_Surface?                              surface = fromGC.Surface();
                TopLoc_Location                            L       = fromGC.Location();
                bool                                       found   = false;
                NCollection_List<BRep_CurveRepresentation> tolist  = ((BRep_TEdge)toedge.TShape()!).ChangeCurves();
                BRep_GCurve?                               toGC    = null;
                for (var toitcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(tolist);
                     toitcr.More() && !found;
                     toitcr.Next())
                {
                    toGC = toitcr.Value() as BRep_GCurve;
                    if (toGC is null || !toGC.IsCurveOnSurface() || surface != toGC.Surface()
                        || L != toGC.Location())
                    {
                        continue;
                    }
                    found = true;
                    break;
                }
                if (!found)
                {
                    double fp = fromGC.First();
                    double lp = fromGC.Last();
                    toGC      = (BRep_GCurve)fromGC.Copy();
                    tolist.Append(toGC);
                    Geom2d_Curve pcurve = (Geom2d_Curve)fromGC.PCurve()!.Copy();
                    if (reverse)
                    {
                        fp = pcurve.ReversedParameter(fp);
                        lp = pcurve.ReversedParameter(lp);
                        pcurve.Reverse();
                        double tmp = fp;
                        fp         = lp;
                        lp         = tmp;
                    }
                    // bug OCC209 invalid location of pcurve in the edge after copying
                    TopLoc_Location newLoc = (fromLoc * L).Predivided(toLoc);
                    toGC.SetRange(fp, lp);
                    toGC.PCurve(pcurve);
                    toGC.Location(newLoc);
                    if (fromGC.IsCurveOnClosedSurface())
                    {
                        Geom2d_Curve pcurve2 = fromGC.PCurve2()!;
                        toGC.PCurve2((Geom2d_Curve)pcurve2.Copy());
                    }
                }
            }
        }
    }

    //  Note:    HasNewPCurves, which FixDummySeam's commented-out code would call, is commented out in OCCT too.

    private void FixDummySeam(int num)
    {
        ShapeAnalysis_Edge   sae  = new ShapeAnalysis_Edge();
        ShapeBuild_Edge      sbe  = new ShapeBuild_Edge();
        ShapeBuild_Vertex    sbv  = new ShapeBuild_Vertex();
        int                  num1 = (num == NbEdges()) ? 1 : num + 1;
        ShapeExtend_WireData sewd = WireData()!;
        TopoDS_Edge          E1 = sewd.Edge(num), E2 = sewd.Edge(num1);
        TopoDS_Vertex        V1 = sae.FirstVertex(E1), V2 = sae.LastVertex(E2);
        TopoDS_Vertex        Vm = sbv.InternalCombineVertex(V1, V2, 1.0001);

        // pnd defining if new pcurves exists
        // pdn Temporary not removed
        //  bool toRemove = !(HasNewPCurves(E1,E2)||HasNewPCurves(E2,E1));
        bool toRemove = false;

        // creating new edge with pcurves and new vertex
        TopoDS_Vertex Vs = sae.FirstVertex(E2);
        if (Vs.IsSame(V1) || Vs.IsSame(V2))
        {
            Vs = Vm;
        }
        TopoDS_Edge newEdge = sbe.InternalCopyReplaceVertices(E2, Vs, Vm);
        CopyReversePcurves(newEdge, E1, E1.Orientation() == E2.Orientation());
        BRep_Builder B = new BRep_Builder();
        B.SameRange(newEdge, false);
        B.SameParameter(newEdge, false);

        if (Context() is not null)
        {
            if (toRemove)
            {
                Context()!.Remove(E2);
                Context()!.Remove(E1);
            }
            else
            {
                Context()!.Replace(E2, newEdge);
                Context()!.Replace(E1, newEdge.Reversed());
            }
            Context()!.Replace(V1, Vm.Oriented(V1.Orientation()));
            Context()!.Replace(V2, Vm.Oriented(V2.Orientation()));
        }

        int         next  = (num1 == NbEdges()) ? 1 : num1 + 1;
        int         prev  = (num > 1) ? num - 1 : NbEdges();
        TopoDS_Edge prevE = sewd.Edge(prev), nextE = sewd.Edge(next);

        TopoDS_Edge tmpE1 = sbe.InternalCopyReplaceVertices(prevE, new TopoDS_Vertex(), Vm);
        sewd.Set(tmpE1, prev);
        if (Context() is not null)
        {
            Context()!.Replace(prevE, tmpE1);
        }

        tmpE1 = sbe.InternalCopyReplaceVertices(nextE, Vm, new TopoDS_Vertex());
        sewd.Set(tmpE1, next);
        if (Context() is not null)
        {
            Context()!.Replace(nextE, tmpE1);
        }

        // removing edges from wire
        int n1, n2;
        if (num < num1)
        {
            n1 = num;
            n2 = num1;
        }
        else
        {
            n1 = num1;
            n2 = num;
        }
        sewd.Remove(n2);
        sewd.Remove(n1);
    }

    /// <summary>Updates the edges of the wire data with the context's replacements.</summary>
    protected void UpdateWire()
    {
        ShapeExtend_WireData sbwd = WireData()!;
        for (int i = 1; i <= sbwd.NbEdges(); i++)
        {
            TopoDS_Edge  E = sbwd.Edge(i);
            TopoDS_Shape S = Context()!.InternalApply(E);
            if (S == E)
            {
                continue;
            }
            for (TopExp_Explorer exp = new TopExp_Explorer(S, TopAbs_EDGE); exp.More(); exp.Next())
            {
                sbwd.Add(exp.Current(), i++);
            }
            sbwd.Remove(i--);
        }
    }

    /// <summary>Fixes tails (thin spikes between adjacent edges), with the maximal angle and width set.</summary>
    public bool FixTails()
    {
        enterWork(); // OCCT.NET: frozen
        if (myMaxTailWidth < 0 || !IsReady())
        {
            return false;
        }

        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (Context() is not null)
        {
            UpdateWire();
        }
        ShapeExtend_WireData aSEWD   = WireData()!;
        int                  aECount = NbEdges();
        int[]                aENs    = [aECount, 1];
        bool                 aCheckAngle = true;
        while (aECount >= 2 && aENs[1] <= aECount)
        {
            TopoDS_Edge[] aEs     = [aSEWD.Edge(aENs[0]), aSEWD.Edge(aENs[1])];
            var           aEParts = new TopoDS_Edge[2, 2];
            for (int k = 0; k < 2; ++k)
            {
                aEParts[k, 0] = new TopoDS_Edge();
                aEParts[k, 1] = new TopoDS_Edge();
            }
            // OCCT.NET: frozen: the internal form leaves the cut parts unfrozen, since they go into the working wire
            if (!myAnalyzer.InternalCheckTail(aEs[0],
                                              aEs[1],
                                              aCheckAngle ? myMaxTailAngleSine : -1,
                                              myMaxTailWidth,
                                              MaxTolerance(),
                                              ref aEParts[0, 0],
                                              ref aEParts[0, 1],
                                              ref aEParts[1, 0],
                                              ref aEParts[1, 1]))
            {
                aENs[0]     = aENs[1]++;
                aCheckAngle = true;
                continue;
            }

            // Provide not less than 1 edge in the result wire.
            int[] aSplitCounts = [aEParts[0, 1].IsNull() ? 0 : 1, aEParts[1, 1].IsNull() ? 0 : 1];
            int   aRemoveCount = (aEParts[0, 0].IsNull() ? 0 : 1) + (aEParts[1, 0].IsNull() ? 0 : 1);
            if (aECount + aSplitCounts[0] + aSplitCounts[1] < 1 + aRemoveCount)
            {
                aENs[0]     = aENs[1]++;
                aCheckAngle = true;
                continue;
            }

            // Split the edges.
            for (int aEI = 0; aEI < 2; ++aEI)
            {
                if (aSplitCounts[aEI] == 0)
                {
                    continue;
                }

                // Replace the edge by the wire of its parts in the shape.
                TopoDS_Edge aE = aEs[aEI];
                if (Context() is not null)
                {
                    TopoDS_Wire aEWire = new TopoDS_Wire();
                    new BRep_Builder().MakeWire(ref aEWire);
                    new BRep_Builder().Add(aEWire, aEParts[aEI, 0]);
                    new BRep_Builder().Add(aEWire, aEParts[aEI, 1]);
                    TopoDS_Edge aFE = TopoDS.Edge(aE.Oriented(TopAbs_FORWARD));
                    Context()!.Replace(aFE, aEWire);
                }

                // Replace the edge by its parts in the edge wire.
                TopAbs_Orientation aOrient = aE.Orientation();
                aEParts[aEI, 0].Orientation(aOrient);
                aEParts[aEI, 1].Orientation(aOrient);
                int aFirstPI = (aOrient != TopAbs_REVERSED) ? 0 : 1;
                int aAdd     = (aEI == 0 || aENs[1] < aENs[0]) ? 0 : aSplitCounts[0];
                aSEWD.Set(aEParts[aEI, aFirstPI], aENs[aEI] + aAdd);
                aSEWD.Add(aEParts[aEI, 1 - aFirstPI], aENs[aEI] + 1 + aAdd);
            }

            // Remove the tail.
            if (aRemoveCount == 2)
            {
                aCheckAngle = true;
                FixDummySeam(aENs[0] + aSplitCounts[0] + ((aENs[0] < aENs[1]) ? 0 : aSplitCounts[1]));
                if (Context() is not null)
                {
                    UpdateWire();
                }
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE);

                if (aSplitCounts[0] + aSplitCounts[1] == 2)
                {
                    aENs[0] = aENs[1]++;
                    continue;
                }

                if (aSplitCounts[0] == aSplitCounts[1])
                {
                    aECount -= 2;
                    if (aENs[1] >= 3)
                    {
                        --aENs[0];
                        --aENs[1];
                    }
                    else
                    {
                        aENs[0] = aECount;
                        aENs[1] = 1;
                    }
                    aCheckAngle = false;
                }
                else
                {
                    --aECount;
                    if (aSplitCounts[0] != 0)
                    {
                        aENs[0] = (aENs[0] <= aECount) ? aENs[0] : aECount;
                    }
                    else
                    {
                        if (aENs[1] >= 3)
                        {
                            --aENs[0];
                            --aENs[1];
                        }
                        else
                        {
                            aENs[0] = aECount;
                            aENs[1] = 1;
                        }
                    }
                }
            }
            else
            {
                aCheckAngle = false;
                --aECount;
                int aRI = aEParts[0, 0].IsNull() ? 1 : 0;
                if (aSplitCounts[aRI] != 0)
                {
                    if (aRI == 0)
                    {
                        if (aENs[1] >= 3)
                        {
                            --aENs[0];
                            --aENs[1];
                        }
                        else
                        {
                            aENs[0] = aECount;
                            aENs[1] = 1;
                        }
                    }
                    else
                    {
                        aENs[0] = (aENs[1] > 1) ? aENs[0] : aECount;
                    }
                }
                aSEWD.Remove(aENs[aRI] + ((aRI != 0 || aSplitCounts[0] == 0) ? 0 : 1));
                if (Context() is not null)
                {
                    Context()!.Remove(aEs[aRI].Oriented(TopAbs_FORWARD));
                    UpdateWire();
                }
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE);
            }
        }
        // OCCT: the tails' status goes into myStatusNotches; StatusFixTails is never set
        myStatusNotches = myLastFixStatus;
        return ShapeExtend.DecodeStatus(myLastFixStatus, ShapeExtend_DONE);
    }
}
