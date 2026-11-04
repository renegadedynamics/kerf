// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// szv 19.08.99: new methods for fixing gaps between edges (3d curves and pcurves)

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

internal partial class WireFixer
{
    /// <summary>Fixes the gaps between the 3d curves of adjacent edges (see <see cref="FixGap3d"/>).</summary>
    public bool FixGaps3d()
    {
        using WorkingOverlay.Scope work = EnterWork(); // Kerf: the working overlay (unit 10H4)
        _statusGaps3d = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        // if ( !IsReady() ) return false;

        int i, start = (_closedMode ? 1 : 2);
        if (_fixGapsByRanges)
        {
            for (i = start; i <= NbEdges(); i++)
            {
                FixGap3d(i);
                _statusGaps3d |= _lastFixStatus;
            }
        }
        for (i = start; i <= NbEdges(); i++)
        {
            FixGap3d(i, true);
            _statusGaps3d |= _lastFixStatus;
        }

        return StatusGaps3d(ExtendStatus.Done);
    }

    /// <summary>Fixes the gaps between the pcurves of adjacent edges (see <see cref="FixGap2d"/>).</summary>
    public bool FixGaps2d()
    {
        using WorkingOverlay.Scope work = EnterWork(); // Kerf: the working overlay (unit 10H4)
        _statusGaps2d = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        //  if ( !IsReady() ) return false;

        int i, start = (_closedMode ? 1 : 2);
        if (_fixGapsByRanges)
        {
            for (i = start; i <= NbEdges(); i++)
            {
                FixGap2d(i);
                _statusGaps2d |= _lastFixStatus;
            }
        }
        for (i = start; i <= NbEdges(); i++)
        {
            FixGap2d(i, true);
            _statusGaps2d |= _lastFixStatus;
        }

        return StatusGaps2d(ExtendStatus.Done);
    }

    private static double AdjustOnPeriodic3d(Curve3D c, bool takefirst, double first, double last, double param)
    {
        // 15.11.2002 PTV OCC966
        if (CurveAnalysis.IsPeriodic(c))
        {
            double t     = c.Period;
            double shift = -Math.Truncate(first / t) * t;
            if (first < 0.0)
            {
                shift += t;
            }
            double sfirst = first + shift, slast = last + shift;
            if (takefirst && (param > slast) && (param > sfirst))
            {
                return param - t - shift;
            }
            if (!takefirst && (param < slast) && (param < sfirst))
            {
                return param + t - shift;
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
        using WorkingOverlay.Scope work = EnterWork(); // Kerf: the working overlay (unit 10H4)
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        //  if ( !IsReady() ) return false;

        //=============
        // First phase: analysis whether the problem (gap) exists
        //=============

        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }

        double preci = Precision();

        ExtendWireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.EdgeCount);
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.EdgeCount);
        // smh#8
        Shape tmp1 = Context()!.InternalApply(sbwd.Edge(n1)), tmp2 = Context()!.InternalApply(sbwd.Edge(n2));
        Edge  e1 = ShapeCasts.Edge(tmp1), e2 = ShapeCasts.Edge(tmp2);
        //  TopoDS_Face face = myAnalyzer->Face(); // comment by enk

        // Retrieve curves on edges
        double             cfirst1, clast1, cfirst2, clast2;
        Curve3D?        C1, C2;
        EdgeAnalysis sae = new EdgeAnalysis();
        if (!sae.Curve3d(e1, out C1, out cfirst1, out clast1) || !sae.Curve3d(e2, out C2, out cfirst2, out clast2))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        // Check gap in 3d space
        Point3D cpnt1 = C1!.Value(clast1), cpnt2 = C2!.Value(cfirst2);
        double gap = cpnt1.DistanceTo(cpnt2);
        if (!convert && gap <= preci)
        {
            return false;
        }

        //=============
        // Second phase: collecting data necessary for further analysis
        //=============

        bool reversed1 = (e1.Orientation == Reversed),
             reversed2 = (e2.Orientation == Reversed);

        Vertex v1 = sae.LastVertex(e1), v2 = sae.FirstVertex(e2);
        Point3D        vpnt = (v1.IsSame(v2))
                               ? Tool.Pnt(v1)
                               : new Point3D((Tool.Pnt(v1).XYZ() + Tool.Pnt(v2).XYZ()) * 0.5);

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

        Curve3D c1 = C1, c2 = C2;

        // Extract basic curves from trimmed and offset
        bool   basic    = false;
        bool   trimmed1 = false, offset1 = false;
        Xyz offval1  = new Xyz(0.0, 0.0, 0.0);
        while (!basic)
        {
            if (c1 is TrimmedCurve3D tc1)
            {
                c1       = tc1.BasisCurve;
                trimmed1 = true;
            }
            else if (c1 is OffsetCurve3D oc)
            {
                c1 = oc.BasisCurve;
                offval1 += oc.Offset * oc.Direction.XYZ();
                offset1 = true;
            }
            else
            {
                basic = true;
            }
        }
        basic = false;
        bool   trimmed2 = false, offset2 = false;
        Xyz offval2  = new Xyz(0.0, 0.0, 0.0);
        while (!basic)
        {
            if (c2 is TrimmedCurve3D tc2)
            {
                c2       = tc2.BasisCurve;
                trimmed2 = true;
            }
            else if (c2 is OffsetCurve3D oc)
            {
                c2 = oc.BasisCurve;
                offval2 += oc.Offset * oc.Direction.XYZ();
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
            c1 = new OffsetCurve3D(c1, offval1.Modulus(), new Direction3D(offval1));
        }
        if (offset2)
        {
            c2 = new OffsetCurve3D(c2, offval2.Modulus(), new Direction3D(offval2));
        }

        bool done1 = false, done2 = false;

        if (convert)
        {
            // Check that gap satisfies the precision - in this case no conversion produced
            if (cpnt1.DistanceTo(vpnt) < preci && cpnt2.DistanceTo(vpnt) < preci)
            {
                return false;
            }

            BSplineCurve3D? bsp1 = null, bsp2 = null;
            Curve3D         c;
            double             first, last;

            // iterate on curves
            int nbcurv = (n1 == n2 ? 1 : 2);
            for (int j = 1; j <= nbcurv; j++)
            {
                // bool trim = false;  // skl
                if (j == 1)
                {
                    if (cpnt1.DistanceTo(vpnt) < preci)
                    {
                        if (n1 == n2)
                        {
                            if (cpnt2.DistanceTo(vpnt) < preci)
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
                    if (cpnt2.DistanceTo(vpnt) < preci)
                    {
                        continue;
                    }
                    c     = c2;
                    first = first2;
                    last  = last2; /*trim = trimmed2;*/ // skl
                }

                BSplineCurve3D? bsp = null;

                // Convert curve to bspline
                // OCCT.NET: frozen: every B-spline here is the caller's (an unfrozen copy, a split, a conversion or the
                // approximation's own curve, read through InternalCurve), so the pole edits below need no copy.
                if (c is BSplineCurve3D)
                {
                    bsp = (BSplineCurve3D)c.Copy();
                    // take segment if trim and range differ
                    double fbsp = bsp.FirstParameter, lbsp = bsp.LastParameter;
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
                        bsp = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(bsp, fbsp, lbsp, Kerf.Engine.Foundation.Precision.Confusion());
                    }
                }
                else if (c is ConicCurve3D)
                {
                    var conv = new Curve3d(new CurveAdaptor(c, first, last),
                                                  _analyzer.Precision(),
                                                  global::Kerf.Geometry.Continuity.C1,
                                                  9,
                                                  1000);
                    if (conv.IsDone || conv.HasResult)
                    {
                        bsp = conv.InternalCurve(); // OCCT.NET: frozen: Curve() freezes; the poles are edited below
                    }
                }
                else
                {
                    // Restore trim for pcurve
                    Curve3D tc;
                    try
                    {
                        // 15.11.2002 PTV OCC966
                        if (!CurveAnalysis.IsPeriodic(c))
                        {
                            tc = new TrimmedCurve3D(c,
                                                       Max(first, c.FirstParameter),
                                                       Min(last, c.LastParameter));
                        }
                        else
                        {
                            tc = new TrimmedCurve3D(c, first, last);
                        }
                        bsp = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(tc);
                    }
                    catch (FailureException)
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
                if (bsp1.Degree == 1)
                {
                    bsp1.IncreaseDegree(2); // gka
                }
                if (n1 == n2)
                {
                    bsp1.SetPole(1, vpnt);
                    bsp1.SetPole(bsp1.PoleCount, vpnt);
                }
                else
                {
                    if (reversed1)
                    {
                        bsp1.SetPole(1, vpnt);
                    }
                    else
                    {
                        bsp1.SetPole(bsp1.PoleCount, vpnt);
                    }
                }
                first1 = bsp1.FirstParameter;
                last1  = bsp1.LastParameter;
                c1     = bsp1;
                done1  = true;
            }
            if (bsp2 is not null)
            {
                if (bsp2.Degree == 1)
                {
                    bsp2.IncreaseDegree(2); // gka
                }
                if (reversed2)
                {
                    bsp2.SetPole(bsp2.PoleCount, vpnt);
                }
                else
                {
                    bsp2.SetPole(1, vpnt);
                }
                first2 = bsp2.FirstParameter;
                last2  = bsp2.LastParameter;
                c2     = bsp2;
                done2  = true;
            }
        }
        else
        {
            if (n1 == n2)
            {
                if (c1 is CircleCurve3D || c1 is EllipseCurve3D)
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
                if (c1 is BSplineCurve3D || c1 is BezierCurve3D)
                {
                    domfirst1 = c1.FirstParameter;
                    domlast1  = c1.LastParameter;
                }
                else if (c1 is LineCurve3D || c1 is ParabolaCurve3D || c1 is HyperbolaCurve3D)
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
                else if (c1 is CircleCurve3D || c1 is EllipseCurve3D)
                {
                    domfirst1 = 0.0;
                    domlast1  = 2 * M_PI;
                }
                double domfirst2 = first2, domlast2 = last2;
                if (c2 is BSplineCurve3D || c2 is BezierCurve3D)
                {
                    domfirst2 = c2.FirstParameter;
                    domlast2  = c2.LastParameter;
                }
                else if (c2 is LineCurve3D || c2 is ParabolaCurve3D || c2 is HyperbolaCurve3D)
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
                else if (c2 is CircleCurve3D || c2 is EllipseCurve3D)
                {
                    domfirst2 = 0.0;
                    domlast2  = 2 * M_PI;
                }

                double ipar1 = clast1, ipar2 = cfirst2;

                // Try to find projections of vertex point
                var    proj = new ProjectPointOnCurve();
                double u1   = ipar1, u2 = ipar2;
                proj.Init(vpnt, c1, domfirst1, domlast1);
                if (proj.NbPoints() != 0)
                {
                    int    index = 1;
                    double dist, mindist = -1.0;
                    for (int i = 1; i <= proj.NbPoints(); i++)
                    {
                        dist = vpnt.DistanceTo(proj.Point(i));
                        if (mindist > dist || mindist < 0.0)
                        {
                            index   = i;
                            mindist = dist;
                        }
                        u1 = proj.Parameter(index);
                    }
                }
                proj.Init(vpnt, c2, domfirst2, domlast2);
                if (proj.NbPoints() != 0)
                {
                    int    index = 1;
                    double dist, mindist = -1.0;
                    for (int i = 1; i <= proj.NbPoints(); i++)
                    {
                        dist = vpnt.DistanceTo(proj.Point(i));
                        if (mindist > dist || mindist < 0.0)
                        {
                            index   = i;
                            mindist = dist;
                        }
                        u2 = proj.Parameter(index);
                    }
                }
                // Adjust parameters on periodic curves
                u1 = AdjustOnPeriodic3d(c1, reversed1, first1, last1, u1);
                u2 = AdjustOnPeriodic3d(c2, !reversed2, first2, last2, u2);
                // Check points to satisfy distance criterium
                Point3D p1 = c1.Value(u1), p2 = c2.Value(u2);
                if (p1.DistanceTo(p2) <= gap && Math.Abs(cfirst1 - u1) > Kerf.Engine.Foundation.Precision.PConfusion()
                    && Math.Abs(clast2 - u2) > Kerf.Engine.Foundation.Precision.PConfusion()
                    && (((u1 > first1) && (u1 < last1)) || ((u2 > first2) && (u2 < last2))
                        || (cpnt1.DistanceTo(p1) <= gap) || (cpnt2.DistanceTo(p2) <= gap)))
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

                    var extr = new ExtremaCurveCurve(c1, c2, domfirst1, domlast1, domfirst2, domlast2);
                    if (extr.NbExtrema() != 0)
                    {
                        try
                        {
                            // First find all intersections
                            Point3D pp1, pp2;
                            int    index1 = 0, index2 = 0;
                            double uu1, uu2, pardist, pardist1 = -1.0, pardist2 = -1.0;
                            for (int i = 1; i <= extr.NbExtrema(); i++)
                            {
                                extr.Parameters(i, out uu1, out uu2);
                                // Adjust parameters on periodic curves
                                uu1 = AdjustOnPeriodic3d(c1, reversed1, first1, last1, uu1);
                                uu2 = AdjustOnPeriodic3d(c2, !reversed2, first2, last2, uu2);
                                pp1 = c1.Value(uu1);
                                pp2 = c2.Value(uu2);
                                if (pp1.DistanceTo(pp2) < Kerf.Engine.Foundation.Precision.Confusion())
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
                                    extr.Parameters(index1, out uu1, out uu2);
                                    pp1 = new Point3D((c1.Value(uu1).XYZ() + c2.Value(uu2).XYZ()) * 0.5);
                                    extr.Parameters(index2, out uu1, out uu2);
                                    pp2 = new Point3D((c1.Value(uu1).XYZ() + c2.Value(uu2).XYZ()) * 0.5);
                                    if (pp2.DistanceTo(vpnt) < pp1.DistanceTo(vpnt))
                                    {
                                        index1 = index2;
                                    }
                                }
                                extr.Parameters(index1, out uu1, out uu2);
                            }
                            else
                            {
                                extr.LowerDistanceParameters(out uu1, out uu2);
                            }
                            // Adjust parameters on periodic curves
                            uu1 = AdjustOnPeriodic3d(c1, reversed1, first1, last1, uu1);
                            uu2 = AdjustOnPeriodic3d(c2, !reversed2, first2, last2, uu2);
                            // Check points to satisfy distance criterium
                            pp1 = c1.Value(uu1);
                            pp2 = c2.Value(uu2);
                            if (pp1.DistanceTo(pp2) <= gap && Math.Abs(cfirst1 - uu1) > Kerf.Engine.Foundation.Precision.PConfusion()
                                && Math.Abs(clast2 - uu2) > Kerf.Engine.Foundation.Precision.PConfusion()
                                && (((uu1 > first1) && (uu1 < last1)) || ((uu2 > first2) && (uu2 < last2))
                                    || (cpnt1.DistanceTo(pp1) <= gap) || (cpnt2.DistanceTo(pp2) <= gap)))
                            {
                                ipar1 = uu1;
                                ipar2 = uu2;
                                done1 = done2 = true;
                            }
                        }
                        catch (FailureException)
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
                                c1 = new TrimmedCurve3D(c1, first1, last1);
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
                                c2 = new TrimmedCurve3D(c2, first2, last2);
                            }
                        }
                    }
                }
                catch (FailureException)
                {
                    // OCCT_DEBUG warning left out
                }
            }
        }

        if (done1 || done2)
        {
            var b    = new BrepBuilder();
            var sbe  = new BuildEdgeBuilder();
            var sfst = new ShapeToleranceFixer();

            // Update vertices
            Vertex nullV = new Vertex(), newV1;
            // smh#8
            Shape  emptyCopiedV2 = v2.EmptyCopied();
            Vertex newV2         = ShapeCasts.Vertex(emptyCopiedV2);
            sfst.SetTolerance(newV2, Kerf.Engine.Foundation.Precision.Confusion());
            Context()!.Replace(v2, newV2);
            if (v1.IsSame(v2))
            // smh#8
            {
                Shape tmpV2 = newV2.Oriented(Reversed);
                newV1              = ShapeCasts.Vertex(tmpV2);
            }
            else
            {
                // smh#8
                Shape emptyCopied = v1.EmptyCopied();
                newV1                    = ShapeCasts.Vertex(emptyCopied);
                sfst.SetTolerance(newV1, Kerf.Engine.Foundation.Precision.Confusion());
                Context()!.Replace(v1, newV1);
            }

            if (done1)
            {
                // Update first edge
                Edge newE1 = sbe.InternalCopyReplaceVertices(e1, nullV, newV1);
                // smh#8
                Shape tmpE1 = newE1.Oriented(Forward);
                b.UpdateEdge(ShapeCasts.Edge(tmpE1), c1, 0.0);
                sbe.SetRange3d(ShapeCasts.Edge(tmpE1), first1, last1);
                sfst.SetTolerance(newE1, Kerf.Engine.Foundation.Precision.Confusion(), ShapeType.Edge);
                b.SameRange(newE1, false);
                //      B.SameParameter(newE1,false);

                // To keep NM vertices belonging initial edges
                for (var itv = new Iterator(e1, false); itv.More(); itv.Next())
                {
                    if (itv.Value().Orientation == Internal
                        || itv.Value().Orientation == External)
                    {
                        Vertex oldV = ShapeCasts.Vertex(itv.Value());
                        Vertex anewV =
                            TransferParametersProjAnalysis.InternalCopyNMVertex(oldV, newE1, e1);
                        b.Add(newE1, anewV);
                        Context()!.Replace(oldV, anewV);
                    }
                }

                Context()!.Replace(e1, newE1);
                sbwd.Set(newE1, n1);
            }

            if (done2)
            {
                // Update second edge
                Edge newE2 = sbe.InternalCopyReplaceVertices(e2, newV2, nullV);
                // smh#8
                Shape tmpE2 = newE2.Oriented(Forward);
                b.UpdateEdge(ShapeCasts.Edge(tmpE2), c2, 0.0);
                sbe.SetRange3d(ShapeCasts.Edge(tmpE2), first2, last2);
                sfst.SetTolerance(newE2, Kerf.Engine.Foundation.Precision.Confusion(), ShapeType.Edge);
                b.SameRange(newE2, false);
                //      B.SameParameter(newE2,false);

                // To keep NM vertices belonging initial edges
                for (var itv = new Iterator(e2, false); itv.More(); itv.Next())
                {
                    if (itv.Value().Orientation == Internal
                        || itv.Value().Orientation == External)
                    {
                        Vertex oldV = ShapeCasts.Vertex(itv.Value());
                        Vertex anewV =
                            TransferParametersProjAnalysis.InternalCopyNMVertex(oldV, newE2, e2);
                        b.Add(newE2, anewV);
                        Context()!.Replace(oldV, anewV);
                    }
                }
                Context()!.Replace(e2, newE2);
                sbwd.Set(newE2, n2);
            }

            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        else if (convert)
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }

        return (done1 || done2);
    }

    private static double AdjustOnPeriodic2d(Curve2D pc, bool takefirst, double first, double last, double param)
    {
        // 15.11.2002 PTV OCC966
        if (CurveAnalysis.IsPeriodic(pc))
        {
            double t     = pc.Period;
            double shift = -Math.Truncate(first / t) * t;
            if (first < 0.0)
            {
                shift += t;
            }
            double sfirst = first + shift, slast = last + shift;
            if (takefirst && (param > slast) && (param > sfirst))
            {
                return param - t - shift;
            }
            if (!takefirst && (param < slast) && (param < sfirst))
            {
                return param + t - shift;
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
        using WorkingOverlay.Scope work = EnterWork(); // Kerf: the working overlay (unit 10H4)
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        //=============
        // First phase: analysis whether the problem (gap) exists
        //=============

        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }

        double preci = Kerf.Engine.Foundation.Precision.PConfusion();
        // double preci = Precision();
        // GeomAdaptor_Surface& SA = Analyzer().Surface()->Adaptor()->ChangeSurface();
        // preci = std::max(SA.UResolution(preci), SA.VResolution(preci));

        ExtendWireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.EdgeCount);
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.EdgeCount);
        // smh#8
        Shape tmp1 = Context()!.InternalApply(sbwd.Edge(n1)), tmp2 = Context()!.InternalApply(sbwd.Edge(n2));
        Edge  e1 = ShapeCasts.Edge(tmp1), e2 = ShapeCasts.Edge(tmp2);
        Face  face = _analyzer.Face();

        // Retrieve pcurves on edges
        double             cfirst1, clast1, cfirst2, clast2;
        Curve2D?      PC1, PC2;
        EdgeAnalysis sae = new EdgeAnalysis();
        if (!sae.PCurve(e1, face, out PC1, out cfirst1, out clast1) || !sae.PCurve(e2, face, out PC2, out cfirst2, out clast2)
            || sbwd.IsSeam(n1) || sbwd.IsSeam(n2))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        // Check gap in 2d space
        Point2D cpnt1 = PC1!.Value(clast1), cpnt2 = PC2!.Value(cfirst2);
        double   gap = cpnt1.DistanceTo(cpnt2);
        if (gap <= preci)
        {
            return false;
        }

        //=============
        // Second phase: collecting data necessary for further analysis
        //=============

        bool reversed1 = (e1.Orientation == Reversed),
             reversed2 = (e2.Orientation == Reversed);

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

        Curve2D pc1 = PC1, pc2 = PC2;

        // Extract basic curves from trimmed and offset
        bool   basic    = false;
        bool   trimmed1 = false, offset1 = false;
        double offval1  = 0.0;
        while (!basic)
        {
            if (pc1 is TrimmedCurve2D tc1)
            {
                pc1      = tc1.BasisCurve;
                trimmed1 = true;
            }
            else if (pc1 is OffsetCurve2D oc)
            {
                pc1 = oc.BasisCurve;
                offval1 += oc.Offset;
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
            if (pc2 is TrimmedCurve2D tc2)
            {
                pc2      = tc2.BasisCurve;
                trimmed2 = true;
            }
            else if (pc2 is OffsetCurve2D oc)
            {
                pc2 = oc.BasisCurve;
                offval2 += oc.Offset;
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
            pc1 = new OffsetCurve2D(pc1, offval1);
        }
        if (offset2)
        {
            pc2 = new OffsetCurve2D(pc2, offval2);
        }

        bool done1 = false, done2 = false;

        // Determine same edge case
        if (convert)
        {
            BSplineCurve2D? bsp1 = null, bsp2 = null;
            Curve2D         pc;
            double               first, last;

            // iterate on pcurves
            int nbcurv = (n1 == n2 ? 1 : 2);
            for (int j = 1; j <= nbcurv; j++)
            {
                // int trim = false; // skl
                BSplineCurve2D? bsp = null;

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
                if (pc is BSplineCurve2D)
                {
                    bsp = (BSplineCurve2D)pc.Copy();
                    // take segment if trim and range differ
                    double fbsp = bsp.FirstParameter, lbsp = bsp.LastParameter;
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
                        bsp = Geom2dConvert2D.SplitBSplineCurve(bsp, fbsp, lbsp, Kerf.Engine.Foundation.Precision.PConfusion());
                    }
                }
                else if (pc is ConicCurve2D)
                {
                    SurfaceAdaptor AS   = _analyzer.Surface()!.Adaptor3d()!;
                    double              tolu = AS.UResolution(_analyzer.Precision()),
                                        tolv = AS.VResolution(_analyzer.Precision());
                    var conv = new Curve2d(new CurveAdaptor2D(pc, first, last),
                                                  first,
                                                  last,
                                                  tolu,
                                                  tolv,
                                                  C1,
                                                  9,
                                                  1000);
                    if (conv.IsDone || conv.HasResult)
                    {
                        bsp = conv.InternalCurve(); // OCCT.NET: frozen: Curve() freezes; the poles are edited below
                    }
                }
                else
                {
                    // Restore trim for pcurve
                    try
                    {
                        Curve2D c;
                        // 15.11.2002 PTV OCC966
                        if (!CurveAnalysis.IsPeriodic(pc))
                        {
                            c = new TrimmedCurve2D(pc,
                                                        Max(first, pc.FirstParameter),
                                                        Min(last, pc.LastParameter));
                        }
                        else
                        {
                            c = new TrimmedCurve2D(pc, first, last);
                        }
                        bsp = Geom2dConvert2D.CurveToBSplineCurve(c);
                    }
                    catch (FailureException)
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
            Point2D mpnt = new Point2D((cpnt1.XY() + cpnt2.XY()) * 0.5);
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
                if (bsp1.Degree == 1)
                {
                    bsp1.IncreaseDegree(2);
                }
                if (n1 == n2)
                {
                    bsp1.SetPole(1, mpnt);
                    bsp1.SetPole(bsp1.PoleCount, mpnt);
                }
                else
                {
                    if (reversed1)
                    {
                        bsp1.SetPole(1, mpnt);
                    }
                    else
                    {
                        bsp1.SetPole(bsp1.PoleCount, mpnt);
                    }
                }
                first1 = bsp1.FirstParameter;
                last1  = bsp1.LastParameter;
                pc1    = bsp1;
                done1  = true;
            }
            if (bsp2 is not null)
            {
                if (bsp2.Degree == 1)
                {
                    bsp2.IncreaseDegree(2);
                }
                if (reversed2)
                {
                    bsp2.SetPole(bsp2.PoleCount, mpnt);
                }
                else
                {
                    bsp2.SetPole(1, mpnt);
                }
                first2 = bsp2.FirstParameter;
                last2  = bsp2.LastParameter;
                pc2    = bsp2;
                done2  = true;
            }
        }
        else
        {
            if (n1 == n2)
            {
                if (pc1 is CircleCurve2D || pc1 is EllipseCurve2D)
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
                if (pc1 is BSplineCurve2D || pc1 is BezierCurve2D)
                {
                    domfirst1 = pc1.FirstParameter;
                    domlast1  = pc1.LastParameter;
                }
                else if (pc1 is LineCurve2D || pc1 is ParabolaCurve2D || pc1 is HyperbolaCurve2D)
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
                else if (pc1 is CircleCurve2D || pc1 is EllipseCurve2D)
                {
                    domfirst1 = 0.0;
                    domlast1  = 2 * M_PI;
                }
                double domfirst2 = first2, domlast2 = last2;
                if (pc2 is BSplineCurve2D || pc2 is BezierCurve2D)
                {
                    domfirst2 = pc2.FirstParameter;
                    domlast2  = pc2.LastParameter;
                }
                else if (pc2 is LineCurve2D || pc2 is ParabolaCurve2D || pc2 is HyperbolaCurve2D)
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
                else if (pc2 is CircleCurve2D || pc2 is EllipseCurve2D)
                {
                    domfirst2 = 0.0;
                    domlast2  = 2 * M_PI;
                }

                double ipar1 = clast1, ipar2 = cfirst2;

                var    inter  = new GInter();
                double tolint = Kerf.Engine.Foundation.Precision.PConfusion();

                var ac1 = new CurveAdaptor2D(pc1);
                var ac2 = new CurveAdaptor2D(pc2);

                // Try to find intersection points
                var dom1 = new TwoDDomain(pc1.Value(domfirst1), domfirst1, tolint, pc1.Value(domlast1), domlast1, tolint);
                var dom2 = new TwoDDomain(pc2.Value(domfirst2), domfirst2, tolint, pc2.Value(domlast2), domlast2, tolint);
                inter.Perform(ac1, dom1, ac2, dom2, tolint, tolint);
                if (inter.IsDone)
                {
                    if (inter.PointCount != 0 || inter.SegmentCount != 0)
                    {
                        int    i, index1 = 0, index2 = 0;
                        double pardist, pardist1 = -1.0, pardist2 = -1.0;
                        // iterate on intersection points
                        IntersectionPoint2D ip;
                        for (i = 1; i <= inter.PointCount; i++)
                        {
                            ip = inter.Point(i);
                            // Adjust parameters on periodic curves
                            double uu1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, ip.ParamOnFirst());
                            double uu2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, ip.ParamOnSecond());
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
                        IntersectionSegment2D IS;
                        for (i = 1; i <= inter.SegmentCount; i++)
                        {
                            IS = inter.Segment(i);
                            for (int j = 1; j <= 2; j++)
                            {
                                if ((j == 1 && IS.HasFirstPoint) || (j == 2 && IS.HasLastPoint))
                                {
                                    if (j == 1)
                                    {
                                        ip = IS.FirstPoint();
                                    }
                                    else
                                    {
                                        ip = IS.LastPoint();
                                    }
                                    // Adjust parameters on periodic curves
                                    double uu1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, ip.ParamOnFirst());
                                    double uu2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, ip.ParamOnSecond());
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
                            Point2D pt1, pt2;
                            if (flag1 == 0)
                            {
                                pt1 = inter.Point(index1).Value();
                            }
                            else
                            {
                                IS = inter.Segment(index1);
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
                                pt2 = inter.Point(index2).Value();
                            }
                            else
                            {
                                IS = inter.Segment(index2);
                                if (flag2 == 1)
                                {
                                    pt2 = IS.FirstPoint().Value();
                                }
                                else
                                {
                                    pt2 = IS.LastPoint().Value();
                                }
                            }
                            Point2D mpnt = new Point2D((cpnt1.XY() + cpnt2.XY()) * 0.5);
                            if (pt2.DistanceTo(mpnt) < pt1.DistanceTo(mpnt))
                            {
                                index1 = index2;
                                flag1  = flag2;
                            }
                        }
                        if (flag1 == 0)
                        {
                            ip = inter.Point(index1);
                        }
                        else
                        {
                            IS = inter.Segment(index1);
                            if (flag1 == 1)
                            {
                                ip = IS.FirstPoint();
                            }
                            else
                            {
                                ip = IS.LastPoint();
                            }
                        }
                        // Adjust parameters on periodic curves
                        double u1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, ip.ParamOnFirst());
                        double u2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, ip.ParamOnSecond());
                        // Check points to satisfy distance criterium
                        Point2D p1 = pc1.Value(u1), p2 = pc2.Value(u2);
                        if (p1.DistanceTo(p2) <= gap && Math.Abs(cfirst1 - u1) > Kerf.Engine.Foundation.Precision.PConfusion()
                            && Math.Abs(clast2 - u2) > Kerf.Engine.Foundation.Precision.PConfusion()
                            && (((u1 > first1) && (u1 < last1)) || ((u2 > first2) && (u2 < last2))
                                || (cpnt1.DistanceTo(p1) <= gap) || (cpnt2.DistanceTo(p2) <= gap)))
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
                    var extr = new ExtremaCurveCurve2D(pc1, pc2, domfirst1, domlast1, domfirst2, domlast2);
                    if (extr.NbExtrema() != 0)
                    {
                        double u1, u2;
                        extr.LowerDistanceParameters(out u1, out u2);
                        // Adjust parameters on periodic curves
                        u1 = AdjustOnPeriodic2d(pc1, reversed1, first1, last1, u1);
                        u2 = AdjustOnPeriodic2d(pc2, !reversed2, first2, last2, u2);
                        // Check points to satisfy distance criterium
                        Point2D p1 = pc1.Value(u1), p2 = pc2.Value(u2);
                        if (p1.DistanceTo(p2) <= gap && Math.Abs(cfirst1 - u1) > Kerf.Engine.Foundation.Precision.PConfusion()
                            && Math.Abs(clast2 - u2) > Kerf.Engine.Foundation.Precision.PConfusion()
                            && (((u1 > first1) && (u1 < last1)) || ((u2 > first2) && (u2 < last2))
                                || (cpnt1.DistanceTo(p1) <= gap) || (cpnt2.DistanceTo(p2) <= gap)))
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
                    var      proj  = new ProjectPointOnCurve2D();
                    Point2D ipnt1 = cpnt1, ipnt2 = cpnt2;
                    double   u1    = ipar1, u2 = ipar2;
                    proj.Init(cpnt2, pc1, domfirst1, domlast1);
                    if (proj.NbPoints() != 0)
                    {
                        int    index = 1;
                        double dist, mindist = -1.0;
                        for (int i = 1; i <= proj.NbPoints(); i++)
                        {
                            dist = cpnt2.DistanceTo(proj.Point(i));
                            if (mindist > dist || mindist < 0.0)
                            {
                                index   = i;
                                mindist = dist;
                            }
                            ipnt1 = proj.Point(index);
                            u1    = proj.Parameter(index);
                        }
                    }
                    proj.Init(cpnt1, pc2, domfirst2, domlast2);
                    if (proj.NbPoints() != 0)
                    {
                        int    index = 1;
                        double dist, mindist = -1.0;
                        for (int i = 1; i <= proj.NbPoints(); i++)
                        {
                            dist = cpnt1.DistanceTo(proj.Point(i));
                            if (mindist > dist || mindist < 0.0)
                            {
                                index   = i;
                                mindist = dist;
                            }
                            ipnt2 = proj.Point(index);
                            u2    = proj.Parameter(index);
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
                        Point2D mpnt = new Point2D((cpnt1.XY() + cpnt2.XY()) * 0.5);
                        u1 = ipar1;
                        u2 = ipar2;
                        proj.Init(mpnt, pc1, domfirst1, domlast1);
                        if (proj.NbPoints() != 0)
                        {
                            int    index = 1;
                            double dist, mindist = -1.0;
                            for (int i = 1; i <= proj.NbPoints(); i++)
                            {
                                dist = mpnt.DistanceTo(proj.Point(i));
                                if (mindist > dist || mindist < 0.0)
                                {
                                    index   = i;
                                    mindist = dist;
                                }
                                ipnt1 = proj.Point(index);
                                u1    = proj.Parameter(index);
                            }
                        }
                        proj.Init(mpnt, pc2, domfirst2, domlast2);
                        if (proj.NbPoints() != 0)
                        {
                            int    index = 1;
                            double dist, mindist = -1.0;
                            for (int i = 1; i <= proj.NbPoints(); i++)
                            {
                                dist = mpnt.DistanceTo(proj.Point(i));
                                if (mindist > dist || mindist < 0.0)
                                {
                                    index   = i;
                                    mindist = dist;
                                }
                                ipnt2 = proj.Point(index);
                                u2    = proj.Parameter(index);
                            }
                        }
                    }
                    else
                    {
                        if (cpnt1.DistanceTo(ipnt2) < cpnt2.DistanceTo(ipnt1))
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
                    Point2D p1 = pc1.Value(u1), p2 = pc2.Value(u2);
                    if (p1.DistanceTo(p2) <= gap && Math.Abs(cfirst1 - u1) > Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(clast2 - u2) > Kerf.Engine.Foundation.Precision.PConfusion()
                        && (((u1 > first1) && (u1 < last1)) || ((u2 > first2) && (u2 < last2))
                            || (cpnt1.DistanceTo(p1) <= gap) || (cpnt2.DistanceTo(p2) <= gap)))
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
                        _analyzer.Surface()!.Surface()!.ParameterBounds(out double umin, out double umax, out double vmin, out double vmax);
                        if (Kerf.Engine.Foundation.Precision.IsInfinite(umin) || Kerf.Engine.Foundation.Precision.IsInfinite(umax)
                            || Kerf.Engine.Foundation.Precision.IsInfinite(vmin) || Kerf.Engine.Foundation.Precision.IsInfinite(vmax))
                        {
                            BrepTools.UVBounds(face, out double fumin, out double fumax, out double fvmin, out double fvmax);
                            if (Kerf.Engine.Foundation.Precision.IsInfinite(umin))
                            {
                                umin = fumin - preci;
                            }
                            if (Kerf.Engine.Foundation.Precision.IsInfinite(umax))
                            {
                                umax = fumax + preci;
                            }
                            if (Kerf.Engine.Foundation.Precision.IsInfinite(vmin))
                            {
                                vmin = fvmin - preci;
                            }
                            if (Kerf.Engine.Foundation.Precision.IsInfinite(vmax))
                            {
                                vmax = fvmax + preci;
                            }
                        }

                        Point2D ipnt, P1 = new Point2D(), P2 = new Point2D();
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
                                u = ipnt.X;
                                v = ipnt.Y;

                                @out = true;
                                if (k == 1)
                                {
                                    if (u < umin)
                                    {
                                        P1 = new Point2D(umin, vmin);
                                        P2 = new Point2D(umin, vmax);
                                    }
                                    else if (u > umax)
                                    {
                                        P1 = new Point2D(umax, vmin);
                                        P2 = new Point2D(umax, vmax);
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
                                        P1 = new Point2D(umin, vmin);
                                        P2 = new Point2D(umax, vmin);
                                    }
                                    else if (v > vmax)
                                    {
                                        P1 = new Point2D(umin, vmax);
                                        P2 = new Point2D(umax, vmax);
                                    }
                                    else
                                    {
                                        @out = false;
                                    }
                                }

                                if (@out)
                                {
                                    // Intersect pcurve with bounding line
                                    var lin  = new LineCurve2D(P1, new Direction2D(new Vector2D(P1, P2)));
                                    var acl  = new CurveAdaptor2D(lin);
                                    var dlin = new TwoDDomain(P1, 0.0, tolint, P2, P1.DistanceTo(P2), tolint);

                                    Curve2D pc;
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
                                    var acc  = new CurveAdaptor2D(pc);
                                    var domc = new TwoDDomain(pc.Value(fpar), fpar, tolint, pc.Value(lpar), lpar, tolint);

                                    // Intersect line with the pcurve
                                    inter.Perform(acl, dlin, acc, domc, tolint, tolint);
                                    if (inter.IsDone)
                                    {
                                        if (inter.PointCount != 0 || inter.SegmentCount != 0)
                                        {
                                            int    i, index = 1;
                                            double uu, dist, mindist = -1.0;
                                            // iterate on intersection points
                                            for (i = 1; i <= inter.PointCount; i++)
                                            {
                                                // Adjust parameters on periodic curve
                                                uu = AdjustOnPeriodic2d(pc,
                                                                        (j == 1 ? reversed1 : !reversed2),
                                                                        fpar,
                                                                        lpar,
                                                                        inter.Point(i).ParamOnSecond());
                                                dist = Math.Abs((j == 1 ? cfirst1 : clast2) - uu);
                                                if (mindist > dist || mindist < 0.0)
                                                {
                                                    index   = i;
                                                    mindist = dist;
                                                }
                                            }
                                            // iterate on intersection segments
                                            int                          flag = 0;
                                            IntersectionPoint2D   ip;
                                            IntersectionSegment2D IS;
                                            for (i = 1; i <= inter.SegmentCount; i++)
                                            {
                                                IS = inter.Segment(i);
                                                for (int jj = 1; jj <= 2; jj++)
                                                {
                                                    if ((jj == 1 && IS.HasFirstPoint) || (jj == 2 && IS.HasLastPoint))
                                                    {
                                                        if (jj == 1)
                                                        {
                                                            ip = IS.FirstPoint();
                                                        }
                                                        else
                                                        {
                                                            ip = IS.LastPoint();
                                                        }
                                                        // Adjust parameters on periodic curve
                                                        // OCCT: jj (the segment end) is used where j (the curve) is meant
                                                        uu = AdjustOnPeriodic2d(pc,
                                                                                (jj == 1 ? reversed1 : !reversed2),
                                                                                fpar,
                                                                                lpar,
                                                                                ip.ParamOnSecond());
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
                                                ip = inter.Point(index);
                                            }
                                            else
                                            {
                                                IS = inter.Segment(index);
                                                if (flag == 1)
                                                {
                                                    ip = IS.FirstPoint();
                                                }
                                                else
                                                {
                                                    ip = IS.LastPoint();
                                                }
                                            }
                                            // Adjust parameters on periodic curve
                                            uu = AdjustOnPeriodic2d(pc,
                                                                    (j == 1 ? reversed1 : !reversed2),
                                                                    fpar,
                                                                    lpar,
                                                                    ip.ParamOnSecond());
                                            if (j == 1 && Math.Abs(cfirst1 - uu) > Kerf.Engine.Foundation.Precision.PConfusion())
                                            {
                                                ipar1 = uu;
                                                ipnt  = ip.Value();
                                            }
                                            if (j == 2 && Math.Abs(clast2 - uu) > Kerf.Engine.Foundation.Precision.PConfusion())
                                            {
                                                ipar2 = uu;
                                                ipnt  = ip.Value();
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
                                pc1 = new TrimmedCurve2D(pc1, first1, last1);
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
                                pc2 = new TrimmedCurve2D(pc2, first2, last2);
                            }
                        }
                    }
                }
                catch (FailureException)
                {
                    // OCCT_DEBUG warning left out
                }
            }
        }

        if (done1 || done2)
        {
            var b    = new BrepBuilder();
            var sbe  = new BuildEdgeBuilder();
            var sfst = new ShapeToleranceFixer();

            // Update vertices
            Vertex v1 = sae.LastVertex(e1), v2 = sae.FirstVertex(e2);
            Vertex nullV = new Vertex(), newV1;
            // smh#8
            Shape  emptyCopiedV2 = v2.EmptyCopied();
            Vertex newV2         = ShapeCasts.Vertex(emptyCopiedV2);
            sfst.SetTolerance(newV2, Kerf.Engine.Foundation.Precision.Confusion());
            Context()!.Replace(v2, newV2);
            if (v1.IsSame(v2))
            // smh#8
            {
                Shape tmpVertexRev = newV2.Oriented(Reversed);
                newV1                     = ShapeCasts.Vertex(tmpVertexRev);
            }
            else
            {
                // smh#8
                Shape emptyCopiedV1 = v1.EmptyCopied();
                newV1                      = ShapeCasts.Vertex(emptyCopiedV1);
                sfst.SetTolerance(newV1, Kerf.Engine.Foundation.Precision.Confusion());
                Context()!.Replace(v1, newV1);
            }

            if (done1)
            {
                // Update first edge
                Edge newE1 = sbe.InternalCopyReplaceVertices(e1, nullV, newV1);
                // smh#8
                Shape tmpE1 = newE1.Oriented(Forward);
                b.UpdateEdge(ShapeCasts.Edge(tmpE1), pc1, face, 0.0);
                b.Range(ShapeCasts.Edge(tmpE1), face, first1, last1);
                sfst.SetTolerance(newE1, Kerf.Engine.Foundation.Precision.Confusion(), ShapeType.Edge);
                b.SameRange(newE1, false);
                //      B.SameParameter(newE1,false);

                // To keep NM vertices belonging initial edges
                for (var itv = new Iterator(e1, false); itv.More(); itv.Next())
                {
                    if (itv.Value().Orientation == Internal
                        || itv.Value().Orientation == External)
                    {
                        Vertex oldV = ShapeCasts.Vertex(itv.Value());
                        Vertex anewV =
                            TransferParametersProjAnalysis.InternalCopyNMVertex(oldV, newE1, e1);
                        b.Add(newE1, anewV);
                        Context()!.Replace(oldV, anewV);
                    }
                }

                Context()!.Replace(e1, newE1);
                sbwd.Set(newE1, n1);
            }

            if (done2)
            {
                // Update second edge
                Edge newE2 = sbe.InternalCopyReplaceVertices(e2, newV2, nullV);
                // smh#8
                Shape tmpE2 = newE2.Oriented(Forward);
                b.UpdateEdge(ShapeCasts.Edge(tmpE2), pc2, face, 0.0);
                b.Range(ShapeCasts.Edge(tmpE2), face, first2, last2);
                sfst.SetTolerance(newE2, Kerf.Engine.Foundation.Precision.Confusion(), ShapeType.Edge);
                b.SameRange(newE2, false);
                //      B.SameParameter(newE2,false);
                // To keep NM vertices belonging initial edges
                for (var itv = new Iterator(e2, false); itv.More(); itv.Next())
                {
                    if (itv.Value().Orientation == Internal
                        || itv.Value().Orientation == External)
                    {
                        Vertex oldV = ShapeCasts.Vertex(itv.Value());
                        Vertex anewV =
                            TransferParametersProjAnalysis.InternalCopyNMVertex(oldV, newE2, e2);
                        b.Add(newE2, anewV);
                        Context()!.Replace(oldV, anewV);
                    }
                }
                Context()!.Replace(e2, newE2);
                sbwd.Set(newE2, n2);
            }

            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        else if (convert)
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }

        return (done1 || done2);
    }
}
