// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// szv 19.08.99: new methods for fixing gaps between edges (3d curves and pcurves)

using static Occt.GeomAbs_Shape;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public partial class ShapeFix_Wire
{
    /// <summary>Fixes the gaps between the 3d curves of adjacent edges (see <see cref="FixGap3d"/>).</summary>
    public bool FixGaps3d()
    {
        enterWork(); // OCCT.NET: frozen
        myStatusGaps3d = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        // if ( !IsReady() ) return false;

        int i, start = (myClosedMode ? 1 : 2);
        if (myFixGapsByRanges)
        {
            for (i = start; i <= NbEdges(); i++)
            {
                FixGap3d(i);
                myStatusGaps3d |= myLastFixStatus;
            }
        }
        for (i = start; i <= NbEdges(); i++)
        {
            FixGap3d(i, true);
            myStatusGaps3d |= myLastFixStatus;
        }

        return StatusGaps3d(ShapeExtend_DONE);
    }

    /// <summary>Fixes the gaps between the pcurves of adjacent edges (see <see cref="FixGap2d"/>).</summary>
    public bool FixGaps2d()
    {
        enterWork(); // OCCT.NET: frozen
        myStatusGaps2d = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        //  if ( !IsReady() ) return false;

        int i, start = (myClosedMode ? 1 : 2);
        if (myFixGapsByRanges)
        {
            for (i = start; i <= NbEdges(); i++)
            {
                FixGap2d(i);
                myStatusGaps2d |= myLastFixStatus;
            }
        }
        for (i = start; i <= NbEdges(); i++)
        {
            FixGap2d(i, true);
            myStatusGaps2d |= myLastFixStatus;
        }

        return StatusGaps2d(ShapeExtend_DONE);
    }

    private static double AdjustOnPeriodic3d(Geom_Curve c, bool takefirst, double first, double last, double param)
    {
        // 15.11.2002 PTV OCC966
        if (ShapeAnalysis_Curve.IsPeriodic(c))
        {
            double T     = c.Period();
            double shift = -Math.Truncate(first / T) * T;
            if (first < 0.0)
            {
                shift += T;
            }
            double sfirst = first + shift, slast = last + shift;
            if (takefirst && (param > slast) && (param > sfirst))
            {
                return param - T - shift;
            }
            if (!takefirst && (param < slast) && (param < sfirst))
            {
                return param + T - shift;
            }
        }
        return param;
    }

    /// <summary>
    /// Fixes the gap between the 3d curves of the edge <paramref name="num"/> and the previous one: by moving the
    /// curves' ends to their intersection or closest points (changing the ranges), or with
    /// <paramref name="convert"/>, by converting the curves to B-splines and moving their end poles to the vertex.
    /// </summary>
    public bool FixGap3d(int num, bool convert = false)
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        //  if ( !IsReady() ) return false;

        //=============
        // First phase: analysis whether the problem (gap) exists
        //=============

        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }

        double preci = Precision();

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());
        // smh#8
        TopoDS_Shape tmp1 = Context()!.InternalApply(sbwd.Edge(n1)), tmp2 = Context()!.InternalApply(sbwd.Edge(n2));
        TopoDS_Edge  E1 = TopoDS.Edge(tmp1), E2 = TopoDS.Edge(tmp2);
        //  TopoDS_Face face = myAnalyzer->Face(); // comment by enk

        // Retrieve curves on edges
        double             cfirst1, clast1, cfirst2, clast2;
        Geom_Curve?        C1, C2;
        ShapeAnalysis_Edge SAE = new ShapeAnalysis_Edge();
        if (!SAE.Curve3d(E1, out C1, out cfirst1, out clast1) || !SAE.Curve3d(E2, out C2, out cfirst2, out clast2))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        // Check gap in 3d space
        gp_Pnt cpnt1 = C1!.Value(clast1), cpnt2 = C2!.Value(cfirst2);
        double gap = cpnt1.Distance(cpnt2);
        if (!convert && gap <= preci)
        {
            return false;
        }

        //=============
        // Second phase: collecting data necessary for further analysis
        //=============

        bool reversed1 = (E1.Orientation() == TopAbs_REVERSED),
             reversed2 = (E2.Orientation() == TopAbs_REVERSED);

        TopoDS_Vertex V1 = SAE.LastVertex(E1), V2 = SAE.FirstVertex(E2);
        gp_Pnt        vpnt = (V1.IsSame(V2))
                               ? BRep_Tool.Pnt(V1)
                               : new gp_Pnt((BRep_Tool.Pnt(V1).XYZ() + BRep_Tool.Pnt(V2).XYZ()) * 0.5);

        double first1, last1, first2, last2;
        if (reversed1)
        {
            first1 = clast1;
            last1  = cfirst1;
        }
        else
        {
            first1 = cfirst1;
            last1  = clast1;
        }
        if (reversed2)
        {
            first2 = clast2;
            last2  = cfirst2;
        }
        else
        {
            first2 = cfirst2;
            last2  = clast2;
        }

        Geom_Curve c1 = C1, c2 = C2;

        // Extract basic curves from trimmed and offset
        bool   basic    = false;
        bool   trimmed1 = false, offset1 = false;
        gp_XYZ offval1  = new gp_XYZ(0.0, 0.0, 0.0);
        while (!basic)
        {
            if (c1 is Geom_TrimmedCurve aTC1)
            {
                c1       = aTC1.BasisCurve();
                trimmed1 = true;
            }
            else if (c1 is Geom_OffsetCurve oc)
            {
                c1 = oc.BasisCurve();
                offval1 += oc.Offset() * oc.Direction().XYZ();
                offset1 = true;
            }
            else
            {
                basic = true;
            }
        }
        basic = false;
        bool   trimmed2 = false, offset2 = false;
        gp_XYZ offval2  = new gp_XYZ(0.0, 0.0, 0.0);
        while (!basic)
        {
            if (c2 is Geom_TrimmedCurve aTC2)
            {
                c2       = aTC2.BasisCurve();
                trimmed2 = true;
            }
            else if (c2 is Geom_OffsetCurve oc)
            {
                c2 = oc.BasisCurve();
                offval2 += oc.Offset() * oc.Direction().XYZ();
                offset2 = true;
            }
            else
            {
                basic = true;
            }
        }
        // Restore offset curves
        if (offset1)
        {
            c1 = new Geom_OffsetCurve(c1, offval1.Modulus(), new gp_Dir(offval1));
        }
        if (offset2)
        {
            c2 = new Geom_OffsetCurve(c2, offval2.Modulus(), new gp_Dir(offval2));
        }

        bool done1 = false, done2 = false;

        if (convert)
        {
            // Check that gap satisfies the precision - in this case no conversion produced
            if (cpnt1.Distance(vpnt) < preci && cpnt2.Distance(vpnt) < preci)
            {
                return false;
            }

            Geom_BSplineCurve? bsp1 = null, bsp2 = null;
            Geom_Curve         c;
            double             first, last;

            // iterate on curves
            int nbcurv = (n1 == n2 ? 1 : 2);
            for (int j = 1; j <= nbcurv; j++)
            {
                // bool trim = false;  // skl
                if (j == 1)
                {
                    if (cpnt1.Distance(vpnt) < preci)
                    {
                        if (n1 == n2)
                        {
                            if (cpnt2.Distance(vpnt) < preci)
                            {
                                continue;
                            }
                        }
                        else
                        {
                            continue;
                        }
                    }
                    c     = c1;
                    first = first1;
                    last  = last1; /*trim = trimmed1;*/ // skl
                }
                else
                {
                    if (cpnt2.Distance(vpnt) < preci)
                    {
                        continue;
                    }
                    c     = c2;
                    first = first2;
                    last  = last2; /*trim = trimmed2;*/ // skl
                }

                Geom_BSplineCurve? bsp = null;

                // Convert curve to bspline
                // OCCT.NET: frozen: every B-spline here is the caller's (an unfrozen copy, a split, a conversion or the
                // approximation's own curve, read through InternalCurve), so the pole edits below need no copy.
                if (c is Geom_BSplineCurve)
                {
                    bsp = (Geom_BSplineCurve)c.Copy();
                    // take segment if trim and range differ
                    double fbsp = bsp.FirstParameter(), lbsp = bsp.LastParameter();
                    bool   segment = false;
                    if (first > fbsp)
                    {
                        fbsp    = first;
                        segment = true;
                    }
                    if (last < lbsp)
                    {
                        lbsp    = last;
                        segment = true;
                    }
                    if (segment)
                    {
                        bsp = GeomConvert.SplitBSplineCurve(bsp, fbsp, lbsp, Occt.Precision.Confusion());
                    }
                }
                else if (c is Geom_Conic)
                {
                    var Conv = new Approx_Curve3d(new GeomAdaptor_Curve(c, first, last),
                                                  myAnalyzer.Precision(),
                                                  GeomAbs_C1,
                                                  9,
                                                  1000);
                    if (Conv.IsDone() || Conv.HasResult())
                    {
                        bsp = Conv.InternalCurve(); // OCCT.NET: frozen: Curve() freezes; the poles are edited below
                    }
                }
                else
                {
                    // Restore trim for pcurve
                    Geom_Curve tc;
                    try
                    {
                        // 15.11.2002 PTV OCC966
                        if (!ShapeAnalysis_Curve.IsPeriodic(c))
                        {
                            tc = new Geom_TrimmedCurve(c,
                                                       Max(first, c.FirstParameter()),
                                                       Min(last, c.LastParameter()));
                        }
                        else
                        {
                            tc = new Geom_TrimmedCurve(c, first, last);
                        }
                        bsp = GeomConvert.CurveToBSplineCurve(tc);
                    }
                    catch (Standard_Failure)
                    {
                        // OCCT_DEBUG warning left out
                    }
                }

                if (j == 1)
                {
                    bsp1 = bsp;
                }
                else
                {
                    bsp2 = bsp;
                }
            }

            // Take curves ends if could not convert
            if (bsp1 is null)
            {
                vpnt = cpnt1;
            }
            else if (bsp2 is null)
            {
                vpnt = cpnt2;
            }

            if (bsp1 is not null)
            {
                if (bsp1.Degree() == 1)
                {
                    bsp1.IncreaseDegree(2); // gka
                }
                if (n1 == n2)
                {
                    bsp1.SetPole(1, vpnt);
                    bsp1.SetPole(bsp1.NbPoles(), vpnt);
                }
                else
                {
                    if (reversed1)
                    {
                        bsp1.SetPole(1, vpnt);
                    }
                    else
                    {
                        bsp1.SetPole(bsp1.NbPoles(), vpnt);
                    }
                }
                first1 = bsp1.FirstParameter();
                last1  = bsp1.LastParameter();
                c1     = bsp1;
                done1  = true;
            }
            if (bsp2 is not null)
            {
                if (bsp2.Degree() == 1)
                {
                    bsp2.IncreaseDegree(2); // gka
                }
                if (reversed2)
                {
                    bsp2.SetPole(bsp2.NbPoles(), vpnt);
                }
                else
                {
                    bsp2.SetPole(1, vpnt);
                }
                first2 = bsp2.FirstParameter();
                last2  = bsp2.LastParameter();
                c2     = bsp2;
                done2  = true;
            }
        }
        else
        {
            if (n1 == n2)
            {
                if (c1 is Geom_Circle || c1 is Geom_Ellipse)
                {
                    double diff = M_PI - Math.Abs(clast1 - cfirst2) * 0.5;
                    first1 -= diff;
                    last1 += diff;
                    done1 = true;
                }
            }
            else
            {
                // Determine domains for extremal points locating
                double domfirst1 = first1, domlast1 = last1;
                if (c1 is Geom_BSplineCurve || c1 is Geom_BezierCurve)
                {
                    domfirst1 = c1.FirstParameter();
                    domlast1  = c1.LastParameter();
                }
                else if (c1 is Geom_Line || c1 is Geom_Parabola || c1 is Geom_Hyperbola)
                {
                    double diff = domlast1 - domfirst1;
                    if (reversed1)
                    {
                        domfirst1 -= 10.0 * diff;
                    }
                    else
                    {
                        domlast1 += 10.0 * diff;
                    }
                }
                else if (c1 is Geom_Circle || c1 is Geom_Ellipse)
                {
                    domfirst1 = 0.0;
                    domlast1  = 2 * M_PI;
                }
                double domfirst2 = first2, domlast2 = last2;
                if (c2 is Geom_BSplineCurve || c2 is Geom_BezierCurve)
                {
                    domfirst2 = c2.FirstParameter();
                    domlast2  = c2.LastParameter();
                }
                else if (c2 is Geom_Line || c2 is Geom_Parabola || c2 is Geom_Hyperbola)
                {
                    double diff = domlast2 - domfirst2;
                    if (reversed2)
                    {
                        domlast2 += 10.0 * diff;
                    }
                    else
                    {
                        domfirst2 -= 10.0 * diff;
                    }
                }
                else if (c2 is Geom_Circle || c2 is Geom_Ellipse)
                {
                    domfirst2 = 0.0;
                    domlast2  = 2 * M_PI;
                }

                double ipar1 = clast1, ipar2 = cfirst2;

                // Try to find projections of vertex point
                var    Proj = new GeomAPI_ProjectPointOnCurve();
                double u1   = ipar1, u2 = ipar2;
                Proj.Init(vpnt, c1, domfirst1, domlast1);
                if (Proj.NbPoints() != 0)
                {
                    int    index = 1;
                    double dist, mindist = -1.0;
                    for (int i = 1; i <= Proj.NbPoints(); i++)
                    {
                        dist = vpnt.Distance(Proj.Point(i));
                        if (mindist > dist || mindist < 0.0)
                        {
                            index   = i;
                            mindist = dist;
                        }
                        u1 = Proj.Parameter(index);
                    }
                }
                Proj.Init(vpnt, c2, domfirst2, domlast2);
                if (Proj.NbPoints() != 0)
                {
                    int    index = 1;
                    double dist, mindist = -1.0;
                    for (int i = 1; i <= Proj.NbPoints(); i++)
                    {
                        dist = vpnt.Distance(Proj.Point(i));
                        if (mindist > dist || mindist < 0.0)
                        {
                            index   = i;
                            mindist = dist;
                        }
                        u2 = Proj.Parameter(index);
                    }
                }
                // Adjust parameters on periodic curves
                u1 = AdjustOnPeriodic3d(c1, reversed1, first1, last1, u1);
                u2 = AdjustOnPeriodic3d(c2, !reversed2, first2, last2, u2);
                // Check points to satisfy distance criterium
                gp_Pnt p1 = c1.Value(u1), p2 = c2.Value(u2);
                if (p1.Distance(p2) <= gap && Math.Abs(cfirst1 - u1) > Occt.Precision.PConfusion()
                    && Math.Abs(clast2 - u2) > Occt.Precision.PConfusion()
                    && (((u1 > first1) && (u1 < last1)) || ((u2 > first2) && (u2 < last2))
                        || (cpnt1.Distance(p1) <= gap) || (cpnt2.Distance(p2) <= gap)))
                {
                    ipar1 = u1;
                    ipar2 = u2;
                    done1 = done2 = true;
                }

                // Try to find closest points if nothing yet found
                if (!done1)
                {
                    // Recompute domains
                    if (reversed1)
                    {
                        domfirst1 = ipar1;
                        domlast1  = last1;
                    }
                    else
                    {
                        domfirst1 = first1;
                        domlast1  = ipar1;
                    }
                    if (reversed2)
                    {
                        domfirst2 = first2;
                        domlast2  = ipar2;
                    }
                    else
                    {
                        domfirst2 = ipar2;
                        domlast2  = last2;
                    }

                    var Extr = new GeomAPI_ExtremaCurveCurve(c1, c2, domfirst1, domlast1, domfirst2, domlast2);
                    if (Extr.NbExtrema() != 0)
                    {
                        try
                        {
                            // First find all intersections
                            gp_Pnt pp1, pp2;
                            int    index1 = 0, index2 = 0;
                            double uu1, uu2, pardist, pardist1 = -1.0, pardist2 = -1.0;
                            for (int i = 1; i <= Extr.NbExtrema(); i++)
                            {
                                Extr.Parameters(i, out uu1, out uu2);
                                // Adjust parameters on periodic curves
                                uu1 = AdjustOnPeriodic3d(c1, reversed1, first1, last1, uu1);
                                uu2 = AdjustOnPeriodic3d(c2, !reversed2, first2, last2, uu2);
                                pp1 = c1.Value(uu1);
                                pp2 = c2.Value(uu2);
                                if (pp1.Distance(pp2) < Occt.Precision.Confusion())
                                {
                                    // assume intersection
                                    pardist = Math.Abs(cfirst1 - uu1);
                                    if (pardist1 > pardist || pardist1 < 0.0)
                                    {
                                        index1   = i;
                                        pardist1 = pardist;
                                    }
                                    pardist = Math.Abs(clast2 - uu2);
                                    if (pardist2 > pardist || pardist2 < 0.0)
                                    {
                                        index2   = i;
                                        pardist2 = pardist;
                                    }
                                }
                            }
                            if (index1 != 0 && index2 != 0)
                            {
                                if (index1 != index2)
                                {
                                    // take intersection closer to vertex point
                                    Extr.Parameters(index1, out uu1, out uu2);
                                    pp1 = new gp_Pnt((c1.Value(uu1).XYZ() + c2.Value(uu2).XYZ()) * 0.5);
                                    Extr.Parameters(index2, out uu1, out uu2);
                                    pp2 = new gp_Pnt((c1.Value(uu1).XYZ() + c2.Value(uu2).XYZ()) * 0.5);
                                    if (pp2.Distance(vpnt) < pp1.Distance(vpnt))
                                    {
                                        index1 = index2;
                                    }
                                }
                                Extr.Parameters(index1, out uu1, out uu2);
                            }
                            else
                            {
                                Extr.LowerDistanceParameters(out uu1, out uu2);
                            }
                            // Adjust parameters on periodic curves
                            uu1 = AdjustOnPeriodic3d(c1, reversed1, first1, last1, uu1);
                            uu2 = AdjustOnPeriodic3d(c2, !reversed2, first2, last2, uu2);
                            // Check points to satisfy distance criterium
                            pp1 = c1.Value(uu1);
                            pp2 = c2.Value(uu2);
                            if (pp1.Distance(pp2) <= gap && Math.Abs(cfirst1 - uu1) > Occt.Precision.PConfusion()
                                && Math.Abs(clast2 - uu2) > Occt.Precision.PConfusion()
                                && (((uu1 > first1) && (uu1 < last1)) || ((uu2 > first2) && (uu2 < last2))
                                    || (cpnt1.Distance(pp1) <= gap) || (cpnt2.Distance(pp2) <= gap)))
                            {
                                ipar1 = uu1;
                                ipar2 = uu2;
                                done1 = done2 = true;
                            }
                        }
                        catch (Standard_Failure)
                        {
                        }
                    }
                }

                try
                {
                    if (done1)
                    {
                        if (ipar1 == clast1)
                        {
                            done1 = false;
                        }
                        else
                        {
                            // Set up new bounds for curve
                            if (reversed1)
                            {
                                first1 = ipar1;
                            }
                            else
                            {
                                last1 = ipar1;
                            }
                            // Set new trim for old curve
                            if (trimmed1)
                            {
                                // double ff1 = c1->FirstParameter();
                                // double ll1 = c1->LastParameter();
                                c1 = new Geom_TrimmedCurve(c1, first1, last1);
                            }
                        }
                    }
                    if (done2)
                    {
                        if (ipar2 == cfirst2)
                        {
                            done2 = false;
                        }
                        else
                        {
                            // Set up new bounds for curve
                            if (reversed2)
                            {
                                last2 = ipar2;
                            }
                            else
                            {
                                first2 = ipar2;
                            }
                            // Set new trim for old curve
                            if (trimmed2)
                            {
                                // double ff2 = c2->FirstParameter();
                                // double ll2 = c2->LastParameter();
                                c2 = new Geom_TrimmedCurve(c2, first2, last2);
                            }
                        }
                    }
                }
                catch (Standard_Failure)
                {
                    // OCCT_DEBUG warning left out
                }
            }
        }

        if (done1 || done2)
        {
            var B    = new BRep_Builder();
            var SBE  = new ShapeBuild_Edge();
            var SFST = new ShapeFix_ShapeTolerance();

            // Update vertices
            TopoDS_Vertex nullV = new TopoDS_Vertex(), newV1;
            // smh#8
            TopoDS_Shape  emptyCopiedV2 = V2.EmptyCopied();
            TopoDS_Vertex newV2         = TopoDS.Vertex(emptyCopiedV2);
            SFST.SetTolerance(newV2, Occt.Precision.Confusion());
            Context()!.Replace(V2, newV2);
            if (V1.IsSame(V2))
            // smh#8
            {
                TopoDS_Shape tmpV2 = newV2.Oriented(TopAbs_REVERSED);
                newV1              = TopoDS.Vertex(tmpV2);
            }
            else
            {
                // smh#8
                TopoDS_Shape emptyCopied = V1.EmptyCopied();
                newV1                    = TopoDS.Vertex(emptyCopied);
                SFST.SetTolerance(newV1, Occt.Precision.Confusion());
                Context()!.Replace(V1, newV1);
            }

            if (done1)
            {
                // Update first edge
                TopoDS_Edge newE1 = SBE.InternalCopyReplaceVertices(E1, nullV, newV1);
                // smh#8
                TopoDS_Shape tmpE1 = newE1.Oriented(TopAbs_FORWARD);
                B.UpdateEdge(TopoDS.Edge(tmpE1), c1, 0.0);
                SBE.SetRange3d(TopoDS.Edge(tmpE1), first1, last1);
                SFST.SetTolerance(newE1, Occt.Precision.Confusion(), TopAbs_EDGE);
                B.SameRange(newE1, false);
                //      B.SameParameter(newE1,false);

                // To keep NM vertices belonging initial edges
                for (var aItv = new TopoDS_Iterator(E1, false); aItv.More(); aItv.Next())
                {
                    if (aItv.Value().Orientation() == TopAbs_INTERNAL
                        || aItv.Value().Orientation() == TopAbs_EXTERNAL)
                    {
                        TopoDS_Vertex aOldV = TopoDS.Vertex(aItv.Value());
                        TopoDS_Vertex anewV =
                            ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aOldV, newE1, E1);
                        B.Add(newE1, anewV);
                        Context()!.Replace(aOldV, anewV);
                    }
                }

                Context()!.Replace(E1, newE1);
                sbwd.Set(newE1, n1);
            }

            if (done2)
            {
                // Update second edge
                TopoDS_Edge newE2 = SBE.InternalCopyReplaceVertices(E2, newV2, nullV);
                // smh#8
                TopoDS_Shape tmpE2 = newE2.Oriented(TopAbs_FORWARD);
                B.UpdateEdge(TopoDS.Edge(tmpE2), c2, 0.0);
                SBE.SetRange3d(TopoDS.Edge(tmpE2), first2, last2);
                SFST.SetTolerance(newE2, Occt.Precision.Confusion(), TopAbs_EDGE);
                B.SameRange(newE2, false);
                //      B.SameParameter(newE2,false);

                // To keep NM vertices belonging initial edges
                for (var aItv = new TopoDS_Iterator(E2, false); aItv.More(); aItv.Next())
                {
                    if (aItv.Value().Orientation() == TopAbs_INTERNAL
                        || aItv.Value().Orientation() == TopAbs_EXTERNAL)
                    {
                        TopoDS_Vertex aOldV = TopoDS.Vertex(aItv.Value());
                        TopoDS_Vertex anewV =
                            ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aOldV, newE2, E2);
                        B.Add(newE2, anewV);
                        Context()!.Replace(aOldV, anewV);
                    }
                }
                Context()!.Replace(E2, newE2);
                sbwd.Set(newE2, n2);
            }

            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        else if (convert)
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }

        return (done1 || done2);
    }

    private static double AdjustOnPeriodic2d(Geom2d_Curve pc, bool takefirst, double first, double last, double param)
    {
        // 15.11.2002 PTV OCC966
        if (ShapeAnalysis_Curve.IsPeriodic(pc))
        {
            double T     = pc.Period();
            double shift = -Math.Truncate(first / T) * T;
            if (first < 0.0)
            {
                shift += T;
            }
            double sfirst = first + shift, slast = last + shift;
            if (takefirst && (param > slast) && (param > sfirst))
            {
                return param - T - shift;
            }
            if (!takefirst && (param < slast) && (param < sfirst))
            {
                return param + T - shift;
            }
        }
        return param;
    }

    /// <summary>
    /// Fixes the gap between the pcurves (on the face) of the edge <paramref name="num"/> and the previous one: by
    /// moving the pcurves' ends to their intersection, closest or projected points (changing the ranges), or with
    /// <paramref name="convert"/>, by converting the pcurves to B-splines and moving their end poles to the middle
    /// point.
    /// </summary>
    public bool FixGap2d(int num, bool convert = false)
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        //=============
        // First phase: analysis whether the problem (gap) exists
        //=============

        if (Context() is null)
        {
            SetContext(new ShapeBuild_ReShape());
        }

        double preci = Occt.Precision.PConfusion();
        // double preci = Precision();
        // GeomAdaptor_Surface& SA = Analyzer().Surface()->Adaptor()->ChangeSurface();
        // preci = std::max(SA.UResolution(preci), SA.VResolution(preci));

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());
        // smh#8
        TopoDS_Shape tmp1 = Context()!.InternalApply(sbwd.Edge(n1)), tmp2 = Context()!.InternalApply(sbwd.Edge(n2));
        TopoDS_Edge  E1 = TopoDS.Edge(tmp1), E2 = TopoDS.Edge(tmp2);
        TopoDS_Face  face = myAnalyzer.Face();

        // Retrieve pcurves on edges
        double             cfirst1, clast1, cfirst2, clast2;
        Geom2d_Curve?      PC1, PC2;
        ShapeAnalysis_Edge SAE = new ShapeAnalysis_Edge();
        if (!SAE.PCurve(E1, face, out PC1, out cfirst1, out clast1) || !SAE.PCurve(E2, face, out PC2, out cfirst2, out clast2)
            || sbwd.IsSeam(n1) || sbwd.IsSeam(n2))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        // Check gap in 2d space
        gp_Pnt2d cpnt1 = PC1!.Value(clast1), cpnt2 = PC2!.Value(cfirst2);
        double   gap = cpnt1.Distance(cpnt2);
        if (gap <= preci)
        {
            return false;
        }

        //=============
        // Second phase: collecting data necessary for further analysis
        //=============

        bool reversed1 = (E1.Orientation() == TopAbs_REVERSED),
             reversed2 = (E2.Orientation() == TopAbs_REVERSED);

        double first1, last1, first2, last2;
        if (reversed1)
        {
            first1 = clast1;
            last1  = cfirst1;
        }
        else
        {
            first1 = cfirst1;
            last1  = clast1;
        }
        if (reversed2)
        {
            first2 = clast2;
            last2  = cfirst2;
        }
        else
        {
            first2 = cfirst2;
            last2  = clast2;
        }

        Geom2d_Curve pc1 = PC1, pc2 = PC2;

        // Extract basic curves from trimmed and offset
        bool   basic    = false;
        bool   trimmed1 = false, offset1 = false;
        double offval1  = 0.0;
        while (!basic)
        {
            if (pc1 is Geom2d_TrimmedCurve aTC1)
            {
                pc1      = aTC1.BasisCurve();
                trimmed1 = true;
            }
            else if (pc1 is Geom2d_OffsetCurve oc)
            {
                pc1 = oc.BasisCurve();
                offval1 += oc.Offset();
                offset1 = true;
            }
            else
            {
                basic = true;
            }
        }
        basic = false;
        bool   trimmed2 = false, offset2 = false;
        double offval2  = 0.0;
        while (!basic)
        {
            if (pc2 is Geom2d_TrimmedCurve aTC2)
            {
                pc2      = aTC2.BasisCurve();
                trimmed2 = true;
            }
            else if (pc2 is Geom2d_OffsetCurve oc)
            {
                pc2 = oc.BasisCurve();
                offval2 += oc.Offset();
                offset2 = true;
            }
            else
            {
                basic = true;
            }
        }
        // Restore offset curves
        if (offset1)
        {
            pc1 = new Geom2d_OffsetCurve(pc1, offval1);
        }
        if (offset2)
        {
            pc2 = new Geom2d_OffsetCurve(pc2, offval2);
        }

        bool done1 = false, done2 = false;

        // Determine same edge case
        if (convert)
        {
            Geom2d_BSplineCurve? bsp1 = null, bsp2 = null;
            Geom2d_Curve         pc;
            double               first, last;

            // iterate on pcurves
            int nbcurv = (n1 == n2 ? 1 : 2);
            for (int j = 1; j <= nbcurv; j++)
            {
                // int trim = false; // skl
                Geom2d_BSplineCurve? bsp = null;

                if (j == 1)
                {
                    pc    = pc1;
                    first = first1;
                    last  = last1; /*trim = trimmed1;*/
                } // skl
                else
                {
                    pc    = pc2;
                    first = first2;
                    last  = last2; /*trim = trimmed2;*/
                } // skl

                // Convert pcurve to bspline
                // OCCT.NET: frozen: every B-spline here is the caller's (an unfrozen copy, a split, a conversion or the
                // approximation's own curve, read through InternalCurve), so the pole edits below need no copy.
                if (pc is Geom2d_BSplineCurve)
                {
                    bsp = (Geom2d_BSplineCurve)pc.Copy();
                    // take segment if trim and range differ
                    double fbsp = bsp.FirstParameter(), lbsp = bsp.LastParameter();
                    bool   segment = false;
                    if (first > fbsp)
                    {
                        fbsp    = first;
                        segment = true;
                    }
                    if (last < lbsp)
                    {
                        lbsp    = last;
                        segment = true;
                    }
                    if (segment)
                    {
                        bsp = Geom2dConvert.SplitBSplineCurve(bsp, fbsp, lbsp, Occt.Precision.PConfusion());
                    }
                }
                else if (pc is Geom2d_Conic)
                {
                    GeomAdaptor_Surface AS   = myAnalyzer.Surface()!.Adaptor3d()!;
                    double              tolu = AS.UResolution(myAnalyzer.Precision()),
                                        tolv = AS.VResolution(myAnalyzer.Precision());
                    var Conv = new Approx_Curve2d(new Geom2dAdaptor_Curve(pc, first, last),
                                                  first,
                                                  last,
                                                  tolu,
                                                  tolv,
                                                  GeomAbs_C1,
                                                  9,
                                                  1000);
                    if (Conv.IsDone() || Conv.HasResult())
                    {
                        bsp = Conv.InternalCurve(); // OCCT.NET: frozen: Curve() freezes; the poles are edited below
                    }
                }
                else
                {
                    // Restore trim for pcurve
                    try
                    {
                        Geom2d_Curve c;
                        // 15.11.2002 PTV OCC966
                        if (!ShapeAnalysis_Curve.IsPeriodic(pc))
                        {
                            c = new Geom2d_TrimmedCurve(pc,
                                                        Max(first, pc.FirstParameter()),
                                                        Min(last, pc.LastParameter()));
                        }
                        else
                        {
                            c = new Geom2d_TrimmedCurve(pc, first, last);
                        }
                        bsp = Geom2dConvert.CurveToBSplineCurve(c);
                    }
                    catch (Standard_Failure)
                    {
                        // OCCT_DEBUG warning left out
                    }
                }

                if (j == 1)
                {
                    bsp1 = bsp;
                }
                else
                {
                    bsp2 = bsp;
                }
            }

            // Take curves ends if could not convert
            gp_Pnt2d mpnt = new gp_Pnt2d((cpnt1.XY() + cpnt2.XY()) * 0.5);
            if (bsp1 is null)
            {
                mpnt = cpnt1;
            }
            else if (bsp2 is null)
            {
                mpnt = cpnt2;
            }

            if (bsp1 is not null)
            {
                if (bsp1.Degree() == 1)
                {
                    bsp1.IncreaseDegree(2);
                }
                if (n1 == n2)
                {
                    bsp1.SetPole(1, mpnt);
                    bsp1.SetPole(bsp1.NbPoles(), mpnt);
                }
                else
                {
                    if (reversed1)
                    {
                        bsp1.SetPole(1, mpnt);
                    }
                    else
                    {
                        bsp1.SetPole(bsp1.NbPoles(), mpnt);
                    }
                }
                first1 = bsp1.FirstParameter();
                last1  = bsp1.LastParameter();
                pc1    = bsp1;
                done1  = true;
            }
            if (bsp2 is not null)
            {
                if (bsp2.Degree() == 1)
                {
                    bsp2.IncreaseDegree(2);
                }
                if (reversed2)
                {
                    bsp2.SetPole(bsp2.NbPoles(), mpnt);
                }
                else
                {
                    bsp2.SetPole(1, mpnt);
                }
                first2 = bsp2.FirstParameter();
                last2  = bsp2.LastParameter();
                pc2    = bsp2;
                done2  = true;
            }
        }
        else
        {
            if (n1 == n2)
            {
                if (pc1 is Geom2d_Circle || pc1 is Geom2d_Ellipse)
                {
                    double diff = M_PI - Math.Abs(clast1 - cfirst2) * 0.5;
                    first1 -= diff;
                    last1 += diff;
                    done1 = true;
                }
            }
            else
            {
                // Determine domains for extremal points locating
                double domfirst1 = first1, domlast1 = last1;
                if (pc1 is Geom2d_BSplineCurve || pc1 is Geom2d_BezierCurve)
                {
                    domfirst1 = pc1.FirstParameter();
                    domlast1  = pc1.LastParameter();
                }
                else if (pc1 is Geom2d_Line || pc1 is Geom2d_Parabola || pc1 is Geom2d_Hyperbola)
                {
                    double diff = domlast1 - domfirst1;
                    if (reversed1)
                    {
                        domfirst1 -= 10.0 * diff;
                    }
                    else
                    {
                        domlast1 += 10.0 * diff;
                    }
                }
                else if (pc1 is Geom2d_Circle || pc1 is Geom2d_Ellipse)
                {
                    domfirst1 = 0.0;
                    domlast1  = 2 * M_PI;
                }
                double domfirst2 = first2, domlast2 = last2;
                if (pc2 is Geom2d_BSplineCurve || pc2 is Geom2d_BezierCurve)
                {
                    domfirst2 = pc2.FirstParameter();
                    domlast2  = pc2.LastParameter();
                }
                else if (pc2 is Geom2d_Line || pc2 is Geom2d_Parabola || pc2 is Geom2d_Hyperbola)
                {
                    double diff = domlast2 - domfirst2;
                    if (reversed2)
                    {
                        domlast2 += 10.0 * diff;
                    }
                    else
                    {
                        domfirst2 -= 10.0 * diff;
                    }
                }
                else if (pc2 is Geom2d_Circle || pc2 is Geom2d_Ellipse)
                {
                    domfirst2 = 0.0;
                    domlast2  = 2 * M_PI;
                }

                double ipar1 = clast1, ipar2 = cfirst2;

                var    Inter  = new Geom2dInt_GInter();
                double tolint = Occt.Precision.PConfusion();

                var AC1 = new Geom2dAdaptor_Curve(pc1);
                var AC2 = new Geom2dAdaptor_Curve(pc2);

                // Try to find intersection points
                var dom1 = new IntRes2d_Domain(pc1.Value(domfirst1), domfirst1, tolint, pc1.Value(domlast1), domlast1, tolint);
                var dom2 = new IntRes2d_Domain(pc2.Value(domfirst2), domfirst2, tolint, pc2.Value(domlast2), domlast2, tolint);
                Inter.Perform(AC1, dom1, AC2, dom2, tolint, tolint);
                if (Inter.IsDone())
                {
                    if (Inter.NbPoints() != 0 || Inter.NbSegments() != 0)
                    {
                        int    i, index1 = 0, index2 = 0;
                        double pardist, pardist1 = -1.0, pardist2 = -1.0;
                        // iterate on intersection points
                        IntRes2d_IntersectionPoint IP;
                        for (i = 1; i <= Inter.NbPoints(); i++)
                        {
                            IP = Inter.Point(i);
                            // Adjust parameters on periodic curves
                            double uu1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, IP.ParamOnFirst());
                            double uu2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, IP.ParamOnSecond());
                            pardist = Math.Abs(cfirst1 - uu1);
                            if (pardist1 > pardist || pardist1 < 0.0)
                            {
                                index1   = i;
                                pardist1 = pardist;
                            }
                            pardist = Math.Abs(clast2 - uu2);
                            if (pardist2 > pardist || pardist2 < 0.0)
                            {
                                index2   = i;
                                pardist2 = pardist;
                            }
                        }
                        int flag1 = 0, flag2 = 0;
                        // iterate on intersection segments
                        IntRes2d_IntersectionSegment IS;
                        for (i = 1; i <= Inter.NbSegments(); i++)
                        {
                            IS = Inter.Segment(i);
                            for (int j = 1; j <= 2; j++)
                            {
                                if ((j == 1 && IS.HasFirstPoint()) || (j == 2 && IS.HasLastPoint()))
                                {
                                    if (j == 1)
                                    {
                                        IP = IS.FirstPoint();
                                    }
                                    else
                                    {
                                        IP = IS.LastPoint();
                                    }
                                    // Adjust parameters on periodic curves
                                    double uu1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, IP.ParamOnFirst());
                                    double uu2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, IP.ParamOnSecond());
                                    pardist = Math.Abs(cfirst1 - uu1);
                                    if (pardist1 > pardist || pardist1 < 0.0)
                                    {
                                        flag1    = j;
                                        index1   = i;
                                        pardist1 = pardist;
                                    }
                                    pardist = Math.Abs(clast2 - uu2);
                                    if (pardist2 > pardist || pardist2 < 0.0)
                                    {
                                        flag2    = j;
                                        index2   = i;
                                        pardist2 = pardist;
                                    }
                                }
                            }
                        }
                        if (index1 != index2 || flag1 != flag2)
                        {
                            // take intersection closer to mean point
                            gp_Pnt2d pt1, pt2;
                            if (flag1 == 0)
                            {
                                pt1 = Inter.Point(index1).Value();
                            }
                            else
                            {
                                IS = Inter.Segment(index1);
                                if (flag1 == 1)
                                {
                                    pt1 = IS.FirstPoint().Value();
                                }
                                else
                                {
                                    pt1 = IS.LastPoint().Value();
                                }
                            }
                            if (flag2 == 0)
                            {
                                pt2 = Inter.Point(index2).Value();
                            }
                            else
                            {
                                IS = Inter.Segment(index2);
                                if (flag2 == 1)
                                {
                                    pt2 = IS.FirstPoint().Value();
                                }
                                else
                                {
                                    pt2 = IS.LastPoint().Value();
                                }
                            }
                            gp_Pnt2d mpnt = new gp_Pnt2d((cpnt1.XY() + cpnt2.XY()) * 0.5);
                            if (pt2.Distance(mpnt) < pt1.Distance(mpnt))
                            {
                                index1 = index2;
                                flag1  = flag2;
                            }
                        }
                        if (flag1 == 0)
                        {
                            IP = Inter.Point(index1);
                        }
                        else
                        {
                            IS = Inter.Segment(index1);
                            if (flag1 == 1)
                            {
                                IP = IS.FirstPoint();
                            }
                            else
                            {
                                IP = IS.LastPoint();
                            }
                        }
                        // Adjust parameters on periodic curves
                        double u1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, IP.ParamOnFirst());
                        double u2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, IP.ParamOnSecond());
                        // Check points to satisfy distance criterium
                        gp_Pnt2d p1 = pc1.Value(u1), p2 = pc2.Value(u2);
                        if (p1.Distance(p2) <= gap && Math.Abs(cfirst1 - u1) > Occt.Precision.PConfusion()
                            && Math.Abs(clast2 - u2) > Occt.Precision.PConfusion()
                            && (((u1 > first1) && (u1 < last1)) || ((u2 > first2) && (u2 < last2))
                                || (cpnt1.Distance(p1) <= gap) || (cpnt2.Distance(p2) <= gap)))
                        {
                            ipar1 = u1;
                            ipar2 = u2;
                            done1 = done2 = true;
                        }
                    }
                }

                // Try to find closest points if nothing yet found
                if (!done1)
                {
                    var Extr = new Geom2dAPI_ExtremaCurveCurve(pc1, pc2, domfirst1, domlast1, domfirst2, domlast2);
                    if (Extr.NbExtrema() != 0)
                    {
                        double u1, u2;
                        Extr.LowerDistanceParameters(out u1, out u2);
                        // Adjust parameters on periodic curves
                        u1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, u1);
                        u2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, u2);
                        // Check points to satisfy distance criterium
                        gp_Pnt2d p1 = pc1.Value(u1), p2 = pc2.Value(u2);
                        if (p1.Distance(p2) <= gap && Math.Abs(cfirst1 - u1) > Occt.Precision.PConfusion()
                            && Math.Abs(clast2 - u2) > Occt.Precision.PConfusion()
                            && (((u1 > first1) && (u1 < last1)) || ((u2 > first2) && (u2 < last2))
                                || (cpnt1.Distance(p1) <= gap) || (cpnt2.Distance(p2) <= gap)))
                        {
                            ipar1 = u1;
                            ipar2 = u2;
                            done1 = done2 = true;
                        }
                    }
                }

                // Try to find projections if nothing yet found
                if (!done1)
                {
                    var      Proj  = new Geom2dAPI_ProjectPointOnCurve();
                    gp_Pnt2d ipnt1 = cpnt1, ipnt2 = cpnt2;
                    double   u1    = ipar1, u2 = ipar2;
                    Proj.Init(cpnt2, pc1, domfirst1, domlast1);
                    if (Proj.NbPoints() != 0)
                    {
                        int    index = 1;
                        double dist, mindist = -1.0;
                        for (int i = 1; i <= Proj.NbPoints(); i++)
                        {
                            dist = cpnt2.Distance(Proj.Point(i));
                            if (mindist > dist || mindist < 0.0)
                            {
                                index   = i;
                                mindist = dist;
                            }
                            ipnt1 = Proj.Point(index);
                            u1    = Proj.Parameter(index);
                        }
                    }
                    Proj.Init(cpnt1, pc2, domfirst2, domlast2);
                    if (Proj.NbPoints() != 0)
                    {
                        int    index = 1;
                        double dist, mindist = -1.0;
                        for (int i = 1; i <= Proj.NbPoints(); i++)
                        {
                            dist = cpnt1.Distance(Proj.Point(i));
                            if (mindist > dist || mindist < 0.0)
                            {
                                index   = i;
                                mindist = dist;
                            }
                            ipnt2 = Proj.Point(index);
                            u2    = Proj.Parameter(index);
                        }
                    }
                    // Adjust parameters on periodic curves
                    u1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, u1);
                    u2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, u2);
                    // Process special case of projection
                    if ((((reversed1 && u1 > clast1) || (!reversed1 && u1 < clast1))
                         && ((reversed2 && u2 < cfirst2) || (!reversed2 && u2 > cfirst2)))
                        || (((reversed1 && u1 < clast1) || (!reversed1 && u1 > clast1))
                            && ((reversed2 && u2 > cfirst2) || (!reversed2 && u2 < cfirst2))))
                    {
                        // both projections lie inside/outside initial domains
                        // project mean point
                        gp_Pnt2d mpnt = new gp_Pnt2d((cpnt1.XY() + cpnt2.XY()) * 0.5);
                        u1 = ipar1;
                        u2 = ipar2;
                        Proj.Init(mpnt, pc1, domfirst1, domlast1);
                        if (Proj.NbPoints() != 0)
                        {
                            int    index = 1;
                            double dist, mindist = -1.0;
                            for (int i = 1; i <= Proj.NbPoints(); i++)
                            {
                                dist = mpnt.Distance(Proj.Point(i));
                                if (mindist > dist || mindist < 0.0)
                                {
                                    index   = i;
                                    mindist = dist;
                                }
                                ipnt1 = Proj.Point(index);
                                u1    = Proj.Parameter(index);
                            }
                        }
                        Proj.Init(mpnt, pc2, domfirst2, domlast2);
                        if (Proj.NbPoints() != 0)
                        {
                            int    index = 1;
                            double dist, mindist = -1.0;
                            for (int i = 1; i <= Proj.NbPoints(); i++)
                            {
                                dist = mpnt.Distance(Proj.Point(i));
                                if (mindist > dist || mindist < 0.0)
                                {
                                    index   = i;
                                    mindist = dist;
                                }
                                ipnt2 = Proj.Point(index);
                                u2    = Proj.Parameter(index);
                            }
                        }
                    }
                    else
                    {
                        if (cpnt1.Distance(ipnt2) < cpnt2.Distance(ipnt1))
                        {
                            u1 = ipar1;
                        }
                        else
                        {
                            u2 = ipar2;
                        }
                    }
                    // Adjust parameters on periodic curves
                    u1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, u1);
                    u2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, u2);
                    // Check points to satisfy distance criterium
                    gp_Pnt2d p1 = pc1.Value(u1), p2 = pc2.Value(u2);
                    if (p1.Distance(p2) <= gap && Math.Abs(cfirst1 - u1) > Occt.Precision.PConfusion()
                        && Math.Abs(clast2 - u2) > Occt.Precision.PConfusion()
                        && (((u1 > first1) && (u1 < last1)) || ((u2 > first2) && (u2 < last2))
                            || (cpnt1.Distance(p1) <= gap) || (cpnt2.Distance(p2) <= gap)))
                    {
                        ipar1 = u1;
                        ipar2 = u2;
                        done1 = done2 = true;
                    }
                }

                if (done1)
                {
                    if (ipar1 < first1 || ipar1 > last1 || ipar2 < first2 || ipar2 > last2)
                    {
                        // Check whether new points lie inside the surface bounds
                        myAnalyzer.Surface()!.Surface()!.Bounds(out double umin, out double umax, out double vmin, out double vmax);
                        if (Occt.Precision.IsInfinite(umin) || Occt.Precision.IsInfinite(umax)
                            || Occt.Precision.IsInfinite(vmin) || Occt.Precision.IsInfinite(vmax))
                        {
                            BRepTools.UVBounds(face, out double fumin, out double fumax, out double fvmin, out double fvmax);
                            if (Occt.Precision.IsInfinite(umin))
                            {
                                umin = fumin - preci;
                            }
                            if (Occt.Precision.IsInfinite(umax))
                            {
                                umax = fumax + preci;
                            }
                            if (Occt.Precision.IsInfinite(vmin))
                            {
                                vmin = fvmin - preci;
                            }
                            if (Occt.Precision.IsInfinite(vmax))
                            {
                                vmax = fvmax + preci;
                            }
                        }

                        gp_Pnt2d ipnt, P1 = new gp_Pnt2d(), P2 = new gp_Pnt2d();
                        double   u, v;
                        bool     @out;
                        // iterate on curves
                        for (int j = 1; j <= 2; j++)
                        {
                            if (j == 1)
                            {
                                if (ipar1 >= first1 && ipar1 <= last1)
                                {
                                    continue;
                                }
                                ipnt = pc1.Value(ipar1);
                            }
                            else
                            {
                                if (ipar2 >= first2 && ipar2 <= last2)
                                {
                                    continue;
                                }
                                ipnt = pc2.Value(ipar2);
                            }

                            // iterate on bounding lines
                            for (int k = 1; k <= 2; k++)
                            {
                                u = ipnt.X();
                                v = ipnt.Y();

                                @out = true;
                                if (k == 1)
                                {
                                    if (u < umin)
                                    {
                                        P1 = new gp_Pnt2d(umin, vmin);
                                        P2 = new gp_Pnt2d(umin, vmax);
                                    }
                                    else if (u > umax)
                                    {
                                        P1 = new gp_Pnt2d(umax, vmin);
                                        P2 = new gp_Pnt2d(umax, vmax);
                                    }
                                    else
                                    {
                                        @out = false;
                                    }
                                }
                                else
                                {
                                    if (v < vmin)
                                    {
                                        P1 = new gp_Pnt2d(umin, vmin);
                                        P2 = new gp_Pnt2d(umax, vmin);
                                    }
                                    else if (v > vmax)
                                    {
                                        P1 = new gp_Pnt2d(umin, vmax);
                                        P2 = new gp_Pnt2d(umax, vmax);
                                    }
                                    else
                                    {
                                        @out = false;
                                    }
                                }

                                if (@out)
                                {
                                    // Intersect pcurve with bounding line
                                    var lin  = new Geom2d_Line(P1, new gp_Dir2d(new gp_Vec2d(P1, P2)));
                                    var ACL  = new Geom2dAdaptor_Curve(lin);
                                    var dlin = new IntRes2d_Domain(P1, 0.0, tolint, P2, P1.Distance(P2), tolint);

                                    Geom2d_Curve pc;
                                    double       fpar, lpar;
                                    if (j == 1)
                                    {
                                        if (cfirst1 < ipar1)
                                        {
                                            fpar = cfirst1;
                                            lpar = ipar1;
                                        }
                                        else
                                        {
                                            fpar = ipar1;
                                            lpar = cfirst1;
                                        }
                                        pc = pc1;
                                    }
                                    else
                                    {
                                        if (clast2 < ipar2)
                                        {
                                            fpar = clast2;
                                            lpar = ipar2;
                                        }
                                        else
                                        {
                                            fpar = ipar2;
                                            lpar = clast2;
                                        }
                                        pc = pc2;
                                    }
                                    var ACC  = new Geom2dAdaptor_Curve(pc);
                                    var domc = new IntRes2d_Domain(pc.Value(fpar), fpar, tolint, pc.Value(lpar), lpar, tolint);

                                    // Intersect line with the pcurve
                                    Inter.Perform(ACL, dlin, ACC, domc, tolint, tolint);
                                    if (Inter.IsDone())
                                    {
                                        if (Inter.NbPoints() != 0 || Inter.NbSegments() != 0)
                                        {
                                            int    i, index = 1;
                                            double uu, dist, mindist = -1.0;
                                            // iterate on intersection points
                                            for (i = 1; i <= Inter.NbPoints(); i++)
                                            {
                                                // Adjust parameters on periodic curve
                                                uu = AdjustOnPeriodic2d(pc,
                                                                        (j == 1 ? reversed1 : !reversed2),
                                                                        fpar,
                                                                        lpar,
                                                                        Inter.Point(i).ParamOnSecond());
                                                dist = Math.Abs((j == 1 ? cfirst1 : clast2) - uu);
                                                if (mindist > dist || mindist < 0.0)
                                                {
                                                    index   = i;
                                                    mindist = dist;
                                                }
                                            }
                                            // iterate on intersection segments
                                            int                          flag = 0;
                                            IntRes2d_IntersectionPoint   IP;
                                            IntRes2d_IntersectionSegment IS;
                                            for (i = 1; i <= Inter.NbSegments(); i++)
                                            {
                                                IS = Inter.Segment(i);
                                                for (int jj = 1; jj <= 2; jj++)
                                                {
                                                    if ((jj == 1 && IS.HasFirstPoint()) || (jj == 2 && IS.HasLastPoint()))
                                                    {
                                                        if (jj == 1)
                                                        {
                                                            IP = IS.FirstPoint();
                                                        }
                                                        else
                                                        {
                                                            IP = IS.LastPoint();
                                                        }
                                                        // Adjust parameters on periodic curve
                                                        // OCCT: jj (the segment end) is used where j (the curve) is meant
                                                        uu = AdjustOnPeriodic2d(pc,
                                                                                (jj == 1 ? reversed1 : !reversed2),
                                                                                fpar,
                                                                                lpar,
                                                                                IP.ParamOnSecond());
                                                        dist = Math.Abs((jj == 1 ? cfirst1 : clast2) - uu);
                                                        if (mindist > dist || mindist < 0.0)
                                                        {
                                                            flag    = jj;
                                                            index   = i;
                                                            mindist = dist;
                                                        }
                                                    }
                                                }
                                            }
                                            if (flag == 0)
                                            {
                                                IP = Inter.Point(index);
                                            }
                                            else
                                            {
                                                IS = Inter.Segment(index);
                                                if (flag == 1)
                                                {
                                                    IP = IS.FirstPoint();
                                                }
                                                else
                                                {
                                                    IP = IS.LastPoint();
                                                }
                                            }
                                            // Adjust parameters on periodic curve
                                            uu = AdjustOnPeriodic2d(pc,
                                                                    (j == 1 ? reversed1 : !reversed2),
                                                                    fpar,
                                                                    lpar,
                                                                    IP.ParamOnSecond());
                                            if (j == 1 && Math.Abs(cfirst1 - uu) > Occt.Precision.PConfusion())
                                            {
                                                ipar1 = uu;
                                                ipnt  = IP.Value();
                                            }
                                            if (j == 2 && Math.Abs(clast2 - uu) > Occt.Precision.PConfusion())
                                            {
                                                ipar2 = uu;
                                                ipnt  = IP.Value();
                                            }
                                        }
                                    }
                                }
                            }

                            // Adjust if intersection lies inside old bounds
                            if (j == 1)
                            {
                                if (reversed1)
                                {
                                    if (ipar1 > first1)
                                    {
                                        ipar1 = first1;
                                    }
                                }
                                else
                                {
                                    if (ipar1 < last1)
                                    {
                                        ipar1 = last1;
                                    }
                                }
                            }
                            else
                            {
                                if (reversed2)
                                {
                                    if (ipar2 < last2)
                                    {
                                        ipar2 = last2;
                                    }
                                }
                                else
                                {
                                    if (ipar2 > first2)
                                    {
                                        ipar2 = first2;
                                    }
                                }
                            }
                        }
                    }
                }
                try
                {
                    if (done1)
                    {
                        if (ipar1 == clast1)
                        {
                            done1 = false;
                        }
                        else
                        {
                            // Set up new bounds for pcurve
                            if (reversed1)
                            {
                                first1 = ipar1;
                            }
                            else
                            {
                                last1 = ipar1;
                            }
                            // Set new trim for old pcurve
                            if (trimmed1)
                            {
                                pc1 = new Geom2d_TrimmedCurve(pc1, first1, last1);
                            }
                        }
                    }
                    if (done2)
                    {
                        if (ipar2 == cfirst2)
                        {
                            done2 = false;
                        }
                        else
                        {
                            // Set up new bounds for pcurve
                            if (reversed2)
                            {
                                last2 = ipar2;
                            }
                            else
                            {
                                first2 = ipar2;
                            }
                            // Set new trim for old pcurve
                            if (trimmed2)
                            {
                                pc2 = new Geom2d_TrimmedCurve(pc2, first2, last2);
                            }
                        }
                    }
                }
                catch (Standard_Failure)
                {
                    // OCCT_DEBUG warning left out
                }
            }
        }

        if (done1 || done2)
        {
            var B    = new BRep_Builder();
            var SBE  = new ShapeBuild_Edge();
            var SFST = new ShapeFix_ShapeTolerance();

            // Update vertices
            TopoDS_Vertex V1 = SAE.LastVertex(E1), V2 = SAE.FirstVertex(E2);
            TopoDS_Vertex nullV = new TopoDS_Vertex(), newV1;
            // smh#8
            TopoDS_Shape  emptyCopiedV2 = V2.EmptyCopied();
            TopoDS_Vertex newV2         = TopoDS.Vertex(emptyCopiedV2);
            SFST.SetTolerance(newV2, Occt.Precision.Confusion());
            Context()!.Replace(V2, newV2);
            if (V1.IsSame(V2))
            // smh#8
            {
                TopoDS_Shape tmpVertexRev = newV2.Oriented(TopAbs_REVERSED);
                newV1                     = TopoDS.Vertex(tmpVertexRev);
            }
            else
            {
                // smh#8
                TopoDS_Shape emptyCopiedV1 = V1.EmptyCopied();
                newV1                      = TopoDS.Vertex(emptyCopiedV1);
                SFST.SetTolerance(newV1, Occt.Precision.Confusion());
                Context()!.Replace(V1, newV1);
            }

            if (done1)
            {
                // Update first edge
                TopoDS_Edge newE1 = SBE.InternalCopyReplaceVertices(E1, nullV, newV1);
                // smh#8
                TopoDS_Shape tmpE1 = newE1.Oriented(TopAbs_FORWARD);
                B.UpdateEdge(TopoDS.Edge(tmpE1), pc1, face, 0.0);
                B.Range(TopoDS.Edge(tmpE1), face, first1, last1);
                SFST.SetTolerance(newE1, Occt.Precision.Confusion(), TopAbs_EDGE);
                B.SameRange(newE1, false);
                //      B.SameParameter(newE1,false);

                // To keep NM vertices belonging initial edges
                for (var aItv = new TopoDS_Iterator(E1, false); aItv.More(); aItv.Next())
                {
                    if (aItv.Value().Orientation() == TopAbs_INTERNAL
                        || aItv.Value().Orientation() == TopAbs_EXTERNAL)
                    {
                        TopoDS_Vertex aOldV = TopoDS.Vertex(aItv.Value());
                        TopoDS_Vertex anewV =
                            ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aOldV, newE1, E1);
                        B.Add(newE1, anewV);
                        Context()!.Replace(aOldV, anewV);
                    }
                }

                Context()!.Replace(E1, newE1);
                sbwd.Set(newE1, n1);
            }

            if (done2)
            {
                // Update second edge
                TopoDS_Edge newE2 = SBE.InternalCopyReplaceVertices(E2, newV2, nullV);
                // smh#8
                TopoDS_Shape tmpE2 = newE2.Oriented(TopAbs_FORWARD);
                B.UpdateEdge(TopoDS.Edge(tmpE2), pc2, face, 0.0);
                B.Range(TopoDS.Edge(tmpE2), face, first2, last2);
                SFST.SetTolerance(newE2, Occt.Precision.Confusion(), TopAbs_EDGE);
                B.SameRange(newE2, false);
                //      B.SameParameter(newE2,false);
                // To keep NM vertices belonging initial edges
                for (var aItv = new TopoDS_Iterator(E2, false); aItv.More(); aItv.Next())
                {
                    if (aItv.Value().Orientation() == TopAbs_INTERNAL
                        || aItv.Value().Orientation() == TopAbs_EXTERNAL)
                    {
                        TopoDS_Vertex aOldV = TopoDS.Vertex(aItv.Value());
                        TopoDS_Vertex anewV =
                            ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(aOldV, newE2, E2);
                        B.Add(newE2, anewV);
                        Context()!.Replace(aOldV, anewV);
                    }
                }
                Context()!.Replace(E2, newE2);
                sbwd.Set(newE2, n2);
            }

            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        else if (convert)
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }

        return (done1 || done2);
    }
}
