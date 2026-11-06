// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Curve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Analyses curves: projection of points (with protections near the ends and fallbacks where extrema fail), range
/// validation, exact 2D bounding boxes, the forward seam of two pcurves, planarity, closure and periodicity, and
/// sample points.
/// </summary>
/// <remarks>A working object without state (OCCT's is used by value).</remarks>
internal sealed class CurveAnalysis
{
    // Projects a point onto a curve by evaluating the curve at several points within the parameter range. It finds
    // the closest point on the curve to thePoint and updates the projection distance, point and parameter. It also
    // narrows the start and end parameters to the segment around the closest point, even if no distance below
    // aProjDistance was found.
    private static void ProjectOnSegments(CurveAdaptorBase curve,
                                          in Point3D       point,
                                          int             segmentCount,
                                          ref double      aStartParam,
                                          ref double      anEndParam,
                                          ref double      aProjDistance,
                                          ref Point3D      aProjPoint,
                                          ref double      aProjParam)
    {
        // We consider <nbseg> points on [uMin,uMax]
        // Which is the closest? And what is the new interval?
        // (it cannot overflow the old one)

        if (segmentCount <= 0)
        {
            return; // No segments to project on
        }

        double paramStep     = (anEndParam - aStartParam) / segmentCount;
        double minSqDistance = aProjDistance * aProjDistance;
        bool   hasChanged    = false;
        for (int i = 0; i <= segmentCount; i++)
        {
            double currentParam      = aStartParam + (paramStep * i);
            Point3D currentPoint      = curve.Value(currentParam);
            double currentSqDistance = currentPoint.SquareDistanceTo(point);
            if (currentSqDistance < minSqDistance)
            {
                minSqDistance = currentSqDistance;
                aProjPoint     = currentPoint;
                aProjParam     = currentParam;
                hasChanged    = true;
            }
        }

        if (hasChanged)
        {
            aProjDistance = Math.Sqrt(minSqDistance);
        }

        anEndParam  = Min(anEndParam, aProjParam + paramStep);
        aStartParam = Max(aStartParam, aProjParam - paramStep);
    }

    /// <summary>
    /// Projects a point on a curve. Returns the distance between the point and its projection, which is stored in
    /// <paramref name="proj"/> with its parameter in <paramref name="param"/>. If the distance to an end of a bounded
    /// curve is within <paramref name="preci"/> (when <paramref name="adjustToEnds"/>), that end is taken.
    /// </summary>
    public double Project(Curve3D c3D, in Point3D p3D, double preci, out Point3D proj, out double param,
                          bool adjustToEnds = true)
    {
        double uMin = c3D.FirstParameter;
        double uMax = c3D.LastParameter;
        if (uMin < uMax)
        {
            return Project(c3D, p3D, preci, out proj, out param, uMin, uMax, adjustToEnds);
        }
        else
        {
            return Project(c3D, p3D, preci, out proj, out param, uMax, uMin, adjustToEnds);
        }
    }

    /// <summary>Projects a point on a curve restricted to [cf, cl] (see the overload without bounds).</summary>
    public double Project(Curve3D c3D, in Point3D p3D, double preci, out Point3D proj, out double param, double cf,
                          double cl, bool adjustToEnds = true)
    {
        double distmin;
        double uMin = (cf < cl ? cf : cl);
        double uMax = (cf < cl ? cl : cf);

        var gac = new CurveAdaptor(c3D, uMin, uMax);
        if (c3D is BoundedCurve3D)
        {
            //:j8 abv 10 Dec 98: tr10_r0501_db.stp #9423: protection against densing of points near one end
            double prec     = (adjustToEnds ? preci : global::Kerf.Engine.Foundation.Precision.Confusion());
            Point3D lowBound = gac.Value(uMin);
            Point3D higBound = gac.Value(uMax);
            distmin = lowBound.DistanceTo(p3D);
            if (distmin <= prec)
            {
                param = uMin;
                proj  = lowBound;
                return distmin;
            }

            distmin = higBound.DistanceTo(p3D);
            if (distmin <= prec)
            {
                param = uMax;
                proj  = higBound;
                return distmin;
            }
        }

        if (!c3D.IsClosed)
        {
            // modified by rln on 16/12/97 after CSR# PRO11641 entity 20767
            // the VIso was not closed (according to C3D->IsClosed()) while it "almost"
            // was (the distance between ends of the curve was a little bit more than
            // Precision::Confusion())
            // in that case value 0.1 was too much and this method returned not correct parameter
            //  modified by pdn on 01.07.98 after BUC60195 entity 1952 (std::min() added)
            double delta = Min(gac.Resolution(preci), (uMax - uMin) * 0.1);
            uMin -= delta;
            uMax += delta;
            gac.Load(c3D, uMin, uMax);
        }

        return ProjectAct(gac, p3D, preci, out proj, out param);
    }

    /// <summary>Projects a point on a curve given by an adaptor (see the Geom_Curve overload).</summary>
    public double Project(CurveAdaptorBase c3D, in Point3D p3D, double preci, out Point3D proj, out double param,
                          bool adjustToEnds = true)
    {
        double uMin = c3D.FirstParameter();
        double uMax = c3D.LastParameter();

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(uMin) && global::Kerf.Engine.Foundation.Precision.IsInfinite(uMax))
        {
            return ProjectAct(c3D, p3D, preci, out proj, out param);
        }

        double distmin_L = global::Kerf.Engine.Foundation.Precision.Infinite(), distmin_H = global::Kerf.Engine.Foundation.Precision.Infinite();
        //:j8 abv 10 Dec 98: tr10_r0501_db.stp #9423: protection against densing of points near one end
        double prec     = (adjustToEnds ? preci : global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D lowBound = c3D.Value(uMin);
        Point3D higBound = c3D.Value(uMax);
        distmin_L = lowBound.DistanceTo(p3D);
        distmin_H = higBound.DistanceTo(p3D);

        if (distmin_L <= prec)
        {
            param = uMin;
            proj  = lowBound;
            return distmin_L;
        }

        if (distmin_H <= prec)
        {
            param = uMax;
            proj  = higBound;
            return distmin_H;
        }

        double distProj = ProjectAct(c3D, p3D, preci, out proj, out param);
        if (distProj < distmin_L + global::Kerf.Engine.Foundation.Precision.Confusion() && distProj < distmin_H + global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return distProj;
        }

        if (distmin_L < distmin_H)
        {
            param = uMin;
            proj  = lowBound;
            return distmin_L;
        }

        param = uMax;
        proj  = higBound;
        return distmin_H;
    }

    /// <summary>
    /// Projects a point on a curve by Extrema_ExtPC, with fallbacks on analytic curves (ElCLib) and by sampling and
    /// Newton iterations on the others. Returns the distance to the projection, even when it isn't within the
    /// tolerance.
    /// </summary>
    /// <remarks>
    /// OCCT reads the caller's <paramref name="projPoint"/> before writing it when Extrema finds nothing, but that
    /// value can't reach the result; the port starts it at the origin.
    /// </remarks>
    public double ProjectAct(CurveAdaptorBase curve, in Point3D point, double tolerance,
                             out Point3D projPoint, out double projParam)
    {
        projPoint = new Point3D();
        bool ok = false;
        projParam = 0.0;
        try
        {
            var    curveExtrema       = new CurvePointExtrema(point, curve);
            double minExtremaDistance = RealLast();
            int    minExtremaIndex    = 0;
            if (curveExtrema.IsDone && (curveExtrema.ExtCount > 0))
            {
                for (int i = 1; i <= curveExtrema.ExtCount; i++)
                {
                    if (!curveExtrema.IsMin(i))
                    {
                        continue;
                    }

                    double currentDistance = curveExtrema.SquareDistance(i);
                    if (currentDistance < minExtremaDistance)
                    {
                        minExtremaDistance = currentDistance;
                        minExtremaIndex    = i;
                    }
                }

                if (minExtremaIndex != 0)
                {
                    projParam = (curveExtrema.Point(minExtremaIndex)).Parameter();
                    projPoint = (curveExtrema.Point(minExtremaIndex)).Value();
                    ok           = true;
                }
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            ok = false;
        }

        // szv#4:S4163:12Mar99 moved
        double uMin            = curve.FirstParameter();
        double uMax            = curve.LastParameter();
        bool   isClosedCurve = false;
        double curvePeriod    = 0.0;
        // Distance between the point and the projection point.
        double projDistance = global::Kerf.Engine.Foundation.Precision.Infinite();
        double modMin       = global::Kerf.Engine.Foundation.Precision.Infinite();

        // Remember the computed values.
        // These values will be used in case the projection is not successful.
        double computedParam = projParam;
        Point3D computedProj  = projPoint;

        // PTV 29.05.2002 remember the old solution, cause it could be better
        bool   isHaveOldSolution = false;
        double oldParam          = 0.0;
        Point3D oldProj           = new Point3D();
        if (ok)
        {
            isHaveOldSolution = true;
            oldProj           = projPoint;
            oldParam          = projParam;
            projDistance       = projPoint.DistanceTo(point);
            modMin             = projDistance;
            if (projDistance > tolerance)
            {
                ok = false;
            }

            if (curve.IsClosed)
            {
                isClosedCurve = true;
                curvePeriod    = uMax - uMin; // szv#4:S4163:12Mar99 optimized
            }
        }

        if (!ok)
        {
            // BUG NICOLAS - If the point is on the curve 0 Solutions. This works with ElCLib

            // Generally speaking, we try to ALWAYS return a result that's NOT EVEN GOOD. The caller can
            // then decide what to do.
            projParam = 0.0;

            switch (curve.GetType())
            {
                case Circle:
                {
                    Circle3D circ = curve.Circle();
                    projPoint = circ.Position.Location;
                    if (circ.Radius <= Constants.Resolution() || point.SquareDistanceTo(projPoint) <= Constants.Resolution())
                    {
                        projParam = curve.FirstParameter();
                        projPoint = new Point3D(projPoint.XYZ() + circ.XAxis.Direction.XYZ() * circ.Radius);
                    }
                    else
                    {
                        projParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(circ, point);
                        projPoint = global::Kerf.Engine.Geometry.ElCLib.Value(projParam, circ);
                    }

                    isClosedCurve = true;
                    curvePeriod    = 2.0 * M_PI;
                    break;
                }

                case Hyperbola:
                {
                    projParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(curve.Hyperbola(), point);
                    projPoint = global::Kerf.Engine.Geometry.ElCLib.Value(projParam, curve.Hyperbola());
                    break;
                }

                case Parabola:
                {
                    projParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(curve.Parabola(), point);
                    projPoint = global::Kerf.Engine.Geometry.ElCLib.Value(projParam, curve.Parabola());
                    break;
                }

                case Line:
                {
                    projParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(curve.Line(), point);
                    projPoint = global::Kerf.Engine.Geometry.ElCLib.Value(projParam, curve.Line());
                    break;
                }

                case Ellipse:
                {
                    projParam    = global::Kerf.Engine.Geometry.ElCLib.Parameter(curve.Ellipse(), point);
                    projPoint    = global::Kerf.Engine.Geometry.ElCLib.Value(projParam, curve.Ellipse());
                    isClosedCurve = true;
                    curvePeriod    = 2.0 * M_PI;
                    break;
                }

                default:
                {
                    // Bspline or something, no easy solution.
                    // Here this algorithm tries to find the closest point on the curve
                    // by evaluating the distance between the point and the curve
                    // at several points within the parameter range.
                    projDistance = global::Kerf.Engine.Foundation.Precision.Infinite();
                    ProjectOnSegments(curve, point, 25, ref uMin, ref uMax, ref projDistance, ref projPoint,
                                      ref projParam);
                    if (projDistance <= tolerance)
                    {
                        return projDistance;
                    }

                    var projector = new LocateExtPC(point, curve, projParam /*U0*/, uMin, uMax,
                                                             tolerance /*TolU*/);
                    if (projector.IsDone)
                    {
                        projParam = projector.Point().Parameter();
                        projPoint = projector.Point().Value();
                        double distNewton = point.DistanceTo(projPoint);
                        if (distNewton < modMin)
                        {
                            return distNewton;
                        }
                    }

                    // Here we are trying to find the closest point on the curve
                    // by repeatedly evaluating the distance between the point
                    // and the curve at several points within the parameter range.
                    // After each call to ProjectOnSegments, uMin and uMax
                    // are updated to the start and end parameters of the segment
                    // containing the projection point. So, the range of probing
                    // is reduced in each iteration.
                    // The particular number of segments to probe was chosen to be 40, 20, 25, and 40 again
                    // (OCCT doesn't know why).
                    foreach (int segmentCount in SegmentCounts)
                    {
                        ProjectOnSegments(curve, point, segmentCount, ref uMin, ref uMax, ref projDistance,
                                          ref projPoint, ref projParam);
                        if (projDistance <= tolerance)
                        {
                            return projDistance;
                        }
                    }

                    // Did not find a point on the curve that is closer than the tolerance.
                    // So, we return the closest point found so far.
                    if (projDistance > modMin)
                    {
                        projDistance = modMin;
                        projParam  = computedParam;
                        projPoint  = computedProj;
                    }

                    return projDistance;
                }
            }
        }

        if (isClosedCurve && (projParam < uMin || projParam > uMax))
        {
            projParam += ShapeAnalysisAnalysis.AdjustByPeriod(projParam, 0.5 * (uMin + uMax), curvePeriod);
        }

        if (isHaveOldSolution)
        {
            // PTV 29.05.2002 Compare old solution and new;
            double oldDist = oldProj.SquareDistanceTo(point);
            double newDist  = projPoint.SquareDistanceTo(point);
            if (oldDist < newDist)
            {
                projPoint = oldProj;
                projParam = oldParam;
            }
        }

        return projPoint.DistanceTo(point);
    }

    private static readonly int[] SegmentCounts = [40, 20, 25, 40];

    /// <summary>
    /// Projects a point on a curve restricted to [cf, cl] by a Newton search from the previous parameter
    /// <paramref name="paramPrev"/>; where it fails, as <c>Project</c> on the adaptor with AdjustToEnds false.
    /// </summary>
    public double NextProject(double paramPrev, Curve3D c3D, in Point3D p3D, double preci, out Point3D proj,
                              out double param, double cf, double cl, bool adjustToEnds = true)
    {
        double uMin    = (cf < cl ? cf : cl);
        double uMax    = (cf < cl ? cl : cf);
        double distmin = global::Kerf.Engine.Foundation.Precision.Infinite();
        var    gac     = new CurveAdaptor(c3D, uMin, uMax);
        if (c3D is BoundedCurve3D)
        {
            //:j8 abv 10 Dec 98: tr10_r0501_db.stp #9423: protection against densing of points near one end
            double prec     = (adjustToEnds ? preci : global::Kerf.Engine.Foundation.Precision.Confusion());
            Point3D lowBound = gac.Value(uMin);
            Point3D higBound = gac.Value(uMax);
            distmin = lowBound.DistanceTo(p3D);
            if (distmin <= prec)
            {
                param = uMin;
                proj  = lowBound;
                return distmin;
            }

            distmin = higBound.DistanceTo(p3D);
            if (distmin <= prec)
            {
                param = uMax;
                proj  = higBound;
                return distmin;
            }
        }

        if (!c3D.IsClosed)
        {
            //  modified by pdn on 01.07.98 after BUC60195 entity 1952 (std::min() added)
            double delta = Min(gac.Resolution(preci), (uMax - uMin) * 0.1);
            uMin -= delta;
            uMax += delta;
            gac.Load(c3D, uMin, uMax);
        }

        return NextProject(paramPrev, gac, p3D, preci, out proj, out param);
    }

    /// <summary>Projects a point on a curve given by an adaptor, by Newton from <paramref name="paramPrev"/>.</summary>
    public double NextProject(double paramPrev, CurveAdaptorBase c3D, in Point3D p3D, double preci, out Point3D proj,
                              out double param)
    {
        double uMin = c3D.FirstParameter();
        double uMax = c3D.LastParameter();

        var projector = new LocateExtPC(p3D, c3D, paramPrev /*U0*/, uMin, uMax, preci /*TolU*/);
        if (projector.IsDone)
        {
            param = projector.Point().Parameter();
            proj  = projector.Point().Value();
            return p3D.DistanceTo(proj);
        }

        return Project(c3D, p3D, preci, out proj, out param, false);
    }

    /// <summary>
    /// Validates the range [First, Last] of a curve: brings it into the curve's bounds, adjusts it to the period, and
    /// orders it. Returns false when the range had to be made up.
    /// </summary>
    /// <remarks>
    /// Where the range is reversed on a curve that isn't closed, OCCT reverses the curve in place
    /// (<c>Geom_Curve::Reverse</c>). The port does too, so on a frozen curve that case throws
    /// <see cref="ImmutableObjectException"/>: pass a copy.
    /// </remarks>
    public bool ValidateRange(Curve3D curve, ref double first, ref double last, double preci)
    {
        // First and/or Last may be outside the natural bounds of the curve.
        // They are then given the end values.

        double cf = curve.FirstParameter;
        double cl = curve.LastParameter;

        if (curve is BoundedCurve3D && !curve.IsClosed)
        {
            if (first < cf)
            {
                first = cf;
            }
            else if (first > cl)
            {
                first = cl;
            }

            if (last < cf)
            {
                last = cf;
            }
            else if (last > cl)
            {
                last = cl;
            }
        }

        // 15.11.2002 PTV OCC966
        if (CurveAnalysis.IsPeriodic(curve))
        {
            // OCCT: on a trimmed (or offset) periodic curve, [cf, cl] is the trimmed range, not the period
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref first, ref last); //:a7 abv 11 Feb 98: preci -> PConfusion()
        }
        else if (first < last)
        {
            // nothing to fix
        }
        else if (curve.IsClosed)
        {
            // one of the projected points is at the origin of the 3D curve's parametrisation:
            // the algorithm gave cl +- preci instead of cf or vice versa
            // DANGER: 3D precision applied to a 1D space

            // Last = cf instead of Last = cl
            if (Math.Abs(last - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
            {
                last = cl;
                // First = cl instead of First = cf
            }
            else if (Math.Abs(first - cl) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
            {
                first = cf;

                // the origin is crossed, illegal on a closed non-periodic curve:
                // the parameters are inverted anyway
            }
            else
            {
                //: S4136 abv 20 Apr 99: r0701_ug.stp #6230: add check in 3d
                if (curve.Value(first).DistanceTo(curve.Value(cf)) < preci)
                {
                    first = cf;
                }

                if (curve.Value(last).DistanceTo(curve.Value(cl)) < preci)
                {
                    last = cl;
                }

                if (first > last)
                {
                    double tmp = first;
                    first = last;
                    last  = tmp;
                }
            }
        }
        // The curve is closed within the 3D tolerance
        else if (curve is BSplineCurve3D bSpline)
        {
            if (bSpline.StartPoint.DistanceTo(bSpline.EndPoint) <= preci)
            {
                // Last = cf instead of Last = cl
                if (Math.Abs(last - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
                {
                    last = cl;
                    // First = cl instead of First = cf
                }
                else if (Math.Abs(first - cl) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
                {
                    first = cf;
                }
                else
                {
                    double tmp = first;
                    first = last;
                    last  = tmp;
                }
            }
            // abv 15.03.00 #72 bm1_pe_t4 protection of exceptions in draw
            else if (first > last)
            {
                first = curve.ReversedParameter(first);
                last  = curve.ReversedParameter(last);
                curve.Reverse();
            }

            //: j9 abv 11 Dec 98: PRO7747 #4875, after :j8:    else
            if (first == last)
            { // gka 10.07.1998 file PRO7656 entity 33334
                first = cf;
                last  = cl;
                return false;
            }
        }
        else
        {
            // abv 15.03.00 #72 bm1_pe_t4 protection of exceptions in draw
            if (first > last)
            {
                first = curve.ReversedParameter(first);
                last  = curve.ReversedParameter(last);
                curve.Reverse();
            }

            // pdn 11.01.99 #144 bm1_pe_t4 protection of exceptions in draw
            if (first == last)
            {
                first -= global::Kerf.Engine.Foundation.Precision.PConfusion();
                last  += global::Kerf.Engine.Foundation.Precision.PConfusion();
            }

            return false;
        }

        return true;
    }

    // search for extremum using Newton (OCCT returns int, with the values true and false)
    private static bool SearchForExtremum(Curve2D c2d, double first, double last, in Vector2D dir, ref double par,
                                          ref Point2D res)
    {
        double prevpar;
        int    nbOut = 0;
        for (int i = 0; i < 10; i++)
        {
            prevpar = par;

            c2d.D2(par, out res, out Vector2D d1, out Vector2D d2);
            double det = (d2 * dir);
            if (Math.Abs(det) < 1e-10)
            {
                return true;
            }

            par -= (d1 * dir) / det;
            if (Math.Abs(par - prevpar) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return true;
            }

            if (par < first)
            {
                if (nbOut++ > 2 || prevpar == first)
                {
                    return false;
                }

                par = first;
            }

            if (par > last)
            {
                if (nbOut++ > 2 || prevpar == last)
                {
                    return false;
                }

                par = last;
            }
        }

        return true;
    }

    /// <summary>
    /// Adds to the box the points of a pcurve on [First, Last]: <paramref name="nPoints"/> equidistant points, or
    /// when <paramref name="exact"/>, the ends of each interval of C2 continuity and the extrema in X and Y found on
    /// it by Newton (a work-around for BndLib's inexact bounds).
    /// </summary>
    public void FillBndBox(Curve2D c2d, double first, double last, int nPoints, bool exact, ref Box2d box)
    {
        if (!exact)
        {
            int    nseg = (nPoints < 2 ? 1 : nPoints - 1);
            double step = (last - first) / nseg;
            for (int i = 0; i <= nseg; i++)
            {
                double   par = first + i * step;
                Point2D pnt = c2d.Value(par);
                box.Add(pnt);
            }

            return;
        }

        // We should solve the task on intervals of C2 continuity.
        var ac  = new CurveAdaptor2D(c2d, first, last);
        int nbInt = ac.NbIntervals(C2);
        // If we have only 1 interval then use input NPoints parameter to get samples.
        int nbSamples = (nbInt < 2 ? nPoints - 1 : nbInt);
        var aParams   = new NCollection_Array1<double>(1, nbSamples + 1);
        if (nbSamples == nbInt)
        {
            ac.Intervals(aParams, C2);
        }
        else
        {
            double step = (last - first) / nbSamples;
            for (int i = 0; i <= nbSamples; i++)
            {
                aParams[i + 1] = first + i * step;
            }
        }

        for (int i = 1; i <= nbSamples + 1; i++)
        {
            double   par1 = aParams[i];
            Point2D aPnt  = c2d.Value(par1);
            box.Add(aPnt);
            if (i <= nbSamples)
            {
                double   par2   = aParams[i + 1];
                double   par     = (par1 + par2) * 0.5;
                Point2D pextr   = new Point2D();
                double   parextr = par;
                if (SearchForExtremum(c2d, par1, par2, new Vector2D(1, 0), ref parextr, ref pextr))
                {
                    box.Add(pextr);
                }

                parextr = par;
                if (SearchForExtremum(c2d, par1, par2, new Vector2D(0, 1), ref parextr, ref pextr))
                {
                    box.Add(pextr);
                }
            }
        }
    }

    /// <summary>
    /// Of two pcurves of a seam (lines, or bounded curves taken as the lines through their ends), returns the index
    /// (1 or 2) of the forward one, or 0 if it can't be told.
    /// </summary>
    public int SelectForwardSeam(Curve2D c1, Curve2D c2)
    {
        int theCurveIndice = 0;

        LineCurve2D? l1 = c1 as LineCurve2D;
        if (l1 is null)
        {
            // if we have BoundedCurve, create a line from C1
            if (c1 is not BoundedCurve2D bc1)
            {
                return theCurveIndice;
            }

            Point2D startBC1 = bc1.StartPoint;
            Point2D endBC1   = bc1.EndPoint;
            var      vecBC1   = new Vector2D(startBC1, endBC1);
            if (vecBC1.SquareMagnitude < Constants.Resolution())
            {
                return theCurveIndice;
            }

            l1 = new LineCurve2D(startBC1, new Direction2D(vecBC1));
        }

        LineCurve2D? l2 = c2 as LineCurve2D;
        if (l2 is null)
        {
            // if we have BoundedCurve, creates a line from C2
            if (c2 is not BoundedCurve2D bc2)
            {
                return theCurveIndice;
            }

            Point2D startBC2 = bc2.StartPoint;
            Point2D endBC2   = bc2.EndPoint;
            var      vecBC2   = new Vector2D(startBC2, endBC2);
            if (vecBC2.SquareMagnitude < Constants.Resolution())
            {
                return theCurveIndice;
            }

            l2 = new LineCurve2D(startBC2, new Direction2D(vecBC2));
        }

        bool udirPos, udirNeg, vdirPos, vdirNeg;
        udirPos = udirNeg = vdirPos = vdirNeg = false;

        Direction2D theDir  = l1.Direction;
        Point2D theLoc1 = l1.Location;
        Point2D theLoc2 = l2.Location;

        if (theDir.X > 0.0)
        {
            udirPos = true; // szv#4:S4163:12Mar99 Udir unused
        }
        else if (theDir.X < 0.0)
        {
            udirNeg = true; // szv#4:S4163:12Mar99 Udir unused
        }
        else if (theDir.Y > 0.0)
        {
            vdirPos = true; // szv#4:S4163:12Mar99 Vdir unused
        }
        else if (theDir.Y < 0.0)
        {
            vdirNeg = true; // szv#4:S4163:12Mar99 Vdir unused
        }

        if (vdirPos)
        {
            // max of Loc1.X() Loc2.X()
            if (theLoc1.X > theLoc2.X)
            {
                theCurveIndice = 1;
            }
            else
            {
                theCurveIndice = 2;
            }
        }
        else if (vdirNeg)
        {
            if (theLoc1.X > theLoc2.X)
            {
                theCurveIndice = 2;
            }
            else
            {
                theCurveIndice = 1;
            }
        }
        else if (udirPos)
        {
            // min of Loc1.X() Loc2.X()
            if (theLoc1.Y < theLoc2.Y)
            {
                theCurveIndice = 1;
            }
            else
            {
                theCurveIndice = 2;
            }
        }
        else if (udirNeg)
        {
            if (theLoc1.Y < theLoc2.Y)
            {
                theCurveIndice = 2;
            }
            else
            {
                theCurveIndice = 1;
            }
        }

        return theCurveIndice;
    }

    //%11 pdn 12.01.98
    // Static methods for IsPlanar

    private static Xyz GetAnyNormal(in Xyz orig)
    {
        var norm = new Xyz();
        if (Math.Abs(orig.Z()) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            norm.SetCoord(0, 0, 1);
        }
        else
        {
            norm.SetCoord(orig.Z(), 0, -orig.X());
            double nrm = norm.Modulus();
            if (nrm < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                norm.SetCoord(0, 0, 1);
            }
            else
            {
                norm = norm / nrm;
            }
        }

        return norm;
    }

    private static void AppendControlPoles(NCollection_Sequence<Point3D> seq, Curve3D curve)
    {
        if (curve is LineCurve3D)
        {
            seq.Append(curve.Value(0));
            seq.Append(curve.Value(1));
        }
        else if (curve is ConicCurve3D)
        {
            seq.Append(curve.Value(0));
            seq.Append(curve.Value(M_PI / 2));
            seq.Append(curve.Value(M_PI));
        }
        else if (curve is TrimmedCurve3D trimmed)
        {
            Curve3D baseCrv = trimmed.BasisCurve;
            bool       done     = false;
            if (baseCrv is BSplineCurve3D)
            {
                try
                {
                    var bslp = (BSplineCurve3D)baseCrv.Copy();
                    bslp.Segment(curve.FirstParameter, curve.LastParameter);
                    AppendControlPoles(seq, bslp);
                    done = true;
                }
                catch (Exception caught) when (Refusal.IsFailure(caught))
                {
                }
            }
            else if (baseCrv is BezierCurve3D)
            {
                try
                {
                    var bz = (BezierCurve3D)baseCrv.Copy();
                    bz.Segment(curve.FirstParameter, curve.LastParameter);
                    AppendControlPoles(seq, bz);
                    done = true;
                }
                catch (Exception caught) when (Refusal.IsFailure(caught))
                {
                }
            }

            if (!done)
            {
                seq.Append(curve.Value(curve.FirstParameter));
                seq.Append(curve.Value((curve.FirstParameter + curve.LastParameter) / 2.0));
                seq.Append(curve.Value(curve.LastParameter));
            }
        }
        else if (curve is OffsetCurve3D)
        {
            seq.Append(curve.Value(curve.FirstParameter));
            seq.Append(curve.Value((curve.FirstParameter + curve.LastParameter) / 2.0));
            seq.Append(curve.Value(curve.LastParameter));
        }
        else if (curve is BSplineCurve3D bSpline)
        {
            NCollection_Array1<Point3D> poles = bSpline.InternalPoles();
            for (int i = 1; i <= poles.Length(); i++)
            {
                seq.Append(poles[i]);
            }
        }
        else if (curve is BezierCurve3D bezier)
        {
            NCollection_Array1<Point3D> poles = bezier.InternalPoles();
            for (int i = 1; i <= poles.Length(); i++)
            {
                seq.Append(poles[i]);
            }
        }
    }

    /// <summary>
    /// Tells if the points lie in a plane within <paramref name="preci"/> (Precision::Confusion() if not positive).
    /// If <paramref name="normal"/> is zero on input, the plane's normal is computed and returned in it; otherwise
    /// the points are checked against that normal.
    /// </summary>
    public static bool IsPlanar(NCollection_Array1<Point3D> pnts, ref Xyz normal, double preci = 0)
    {
        double precision = (preci > 0.0) ? preci : global::Kerf.Engine.Foundation.Precision.Confusion();
        bool   noNorm    = (normal.SquareModulus() == 0);

        if (pnts.Length() < 3)
        {
            Xyz n1 = pnts[1].XYZ() - pnts[2].XYZ();
            if (noNorm)
            {
                normal = GetAnyNormal(n1);
                return true;
            }

            return Math.Abs(n1 * normal) < global::Kerf.Engine.Foundation.Precision.Confusion();
        }

        Xyz maxDir = new Xyz();
        if (noNorm)
        {
            // define a center point
            var center = new Xyz(0, 0, 0);
            int i       = 1;
            for (; i <= pnts.Length(); i++)
            {
                center += pnts[i].XYZ();
            }

            center /= pnts.Length();

            maxDir = pnts[1].XYZ() - center;
            normal  = (pnts[pnts.Length()].XYZ() - center) ^ maxDir;

            for (i = 1; i < pnts.Length(); i++)
            {
                Xyz tmpDir = pnts[i + 1].XYZ() - center;
                if (tmpDir.SquareModulus() > maxDir.SquareModulus())
                {
                    maxDir = tmpDir;
                }

                Xyz delta = (pnts[i].XYZ() - center) ^ (pnts[i + 1].XYZ() - center);
                if (normal * delta < 0)
                {
                    delta *= -1;
                }

                normal += delta;
            }
        }

        // check if points are linear
        double nrm = normal.Modulus();
        if (nrm < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            normal = GetAnyNormal(maxDir);
            return true;
        }

        normal = normal / nrm;

        double mind = RealLast(), maxd = -RealLast(), dev;
        for (int i = 1; i <= pnts.Length(); i++)
        {
            dev = pnts[i].XYZ() * normal;
            if (dev < mind)
            {
                mind = dev;
            }

            if (dev > maxd)
            {
                maxd = dev;
            }
        }

        return ((maxd - mind) <= precision);
    }

    /// <summary>
    /// Tells if a curve is planar within <paramref name="preci"/>: lines, conics, trimmed and offset curves of them,
    /// B-spline and Bezier curves by their poles, complex curves by their segments' control points; false for any
    /// other curve. See the overload on points for <paramref name="normal"/>.
    /// </summary>
    public static bool IsPlanar(Curve3D curve, ref Xyz normal, double preci = 0)
    {
        double precision = (preci > 0.0) ? preci : global::Kerf.Engine.Foundation.Precision.Confusion();
        bool   noNorm    = (normal.SquareModulus() == 0);

        if (curve is LineCurve3D line)
        {
            Xyz n1 = line.Position.Direction.XYZ();
            if (noNorm)
            {
                normal = GetAnyNormal(n1);
                return true;
            }

            return Math.Abs(n1 * normal) < global::Kerf.Engine.Foundation.Precision.Confusion();
        }

        if (curve is ConicCurve3D conic)
        {
            Xyz n1 = conic.Axis.Direction.XYZ();
            if (noNorm)
            {
                normal = n1;
                return true;
            }

            Xyz vecMul = n1 ^ normal;
            return vecMul.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        }

        if (curve is TrimmedCurve3D trimmed)
        {
            return IsPlanar(trimmed.BasisCurve, ref normal, precision);
        }

        if (curve is OffsetCurve3D offsetC)
        {
            return IsPlanar(offsetC.BasisCurve, ref normal, precision);
        }

        if (curve is BSplineCurve3D bSpline)
        {
            return IsPlanar(bSpline.InternalPoles(), ref normal, precision);
        }

        if (curve is BezierCurve3D bezier)
        {
            return IsPlanar(bezier.InternalPoles(), ref normal, precision);
        }

        if (curve is ComplexCurve complex)
        {
            var sequence = new NCollection_Sequence<Point3D>();
            int i;
            for (i = 1; i <= complex.CurveCount; i++)
            {
                AppendControlPoles(sequence, complex.Curve(i));
            }

            var poles = new NCollection_Array1<Point3D>(1, sequence.Length());
            for (i = 1; i <= sequence.Length(); i++)
            {
                poles[i] = sequence[i];
            }

            return IsPlanar(poles, ref normal, precision);
        }

        return false;
    }

    /// <summary>
    /// Appends to <paramref name="seq"/> sample points of a curve on [first, last] (2 on lines, 360 per period on
    /// circles, knots × degree on B-splines, poles + 3 on Bezier curves, else 100 per parameter range). Returns false
    /// if the curve's parameter range is empty.
    /// </summary>
    public static bool GetSamplePoints(Curve3D curve, double first, double last, NCollection_Sequence<Point3D> seq)
    {
        double adelta = curve.LastParameter - curve.FirstParameter;
        if (adelta == 0)
        {
            return false;
        }

        int k  = (int)Math.Ceiling((last - first) / adelta);
        int nbp = 100 * k;
        if (curve is LineCurve3D)
        {
            nbp = 2;
        }
        else if (curve is CircleCurve3D)
        {
            nbp = 360 * k;
        }
        else if (curve is BSplineCurve3D bspl)
        {
            nbp = bspl.KnotCount * bspl.Degree * k;
            if (nbp < 2.0)
            {
                nbp = 2;
            }
        }
        else if (curve is BezierCurve3D b)
        {
            nbp = 3 + b.PoleCount;
        }
        else if (curve is OffsetCurve3D oc)
        {
            return GetSamplePoints(oc.BasisCurve, first, last, seq);
        }
        else if (curve is TrimmedCurve3D tc)
        {
            return GetSamplePoints(tc.BasisCurve, first, last, seq);
        }

        var    gac  = new CurveAdaptor(curve);
        double step = (last - first) / (double)(nbp - 1);
        for (int i = 0; i < nbp - 1; ++i)
        {
            seq.Append(gac.Value(first + step * i));
        }

        seq.Append(gac.Value(last));
        return true;
    }

    /// <summary>
    /// Appends to <paramref name="seq"/> sample points of a pcurve on [first, last], distributed as
    /// BRepTopAdaptor_FClass2d does. Returns true.
    /// </summary>
    public static bool GetSamplePoints(Curve2D curve, double first, double last, NCollection_Sequence<Point2D> seq)
    {
        //: abv 05.06.02: TUBE.stp
        // Use the same distribution of points as BRepTopAdaptor_FClass2d for consistency
        var c   = new CurveAdaptor2D(curve, first, last);
        int nbs = Geom2dCurveTool.NbSamples(c);
        //-- Beware of rational B-splines of degree 3 (pieces of circles among others)
        if (nbs > 2)
        {
            nbs *= 4;
        }

        double step = (last - first) / (double)(nbs - 1);
        for (int i = 0; i < nbs - 1; ++i)
        {
            seq.Append(c.Value(first + step * i));
        }

        seq.Append(c.Value(last));
        return true;
    }

    /// <summary>
    /// Tells if a curve is closed: by <c>Geom_Curve::IsClosed</c>, else if its ends are within
    /// <paramref name="preci"/> (at least Precision::Confusion()).
    /// </summary>
    public static bool IsClosed(Curve3D curve, double preci = -1)
    {
        if (curve.IsClosed)
        {
            return true;
        }

        double prec = Max(preci, global::Kerf.Engine.Foundation.Precision.Confusion());

        double f, l;
        f = curve.FirstParameter;
        l = curve.LastParameter;

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(f) || global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
        {
            return false;
        }

        double closedVal = curve.Value(f).SquareDistanceTo(curve.Value(l));
        double preci2     = prec * prec;

        return (closedVal <= preci2);
    }

    /// <summary>Tells if a curve is periodic, asking the basis curve of offset and trimmed curves.</summary>
    public static bool IsPeriodic(Curve3D curve)
    {
        // 15.11.2002 PTV OCC966
        // remove regressions in DE tests (diva, divb, divc, toe3) in KAS:dev
        // ask IsPeriodic on BasisCurve
        Curve3D tmpCurve = curve;
        while (tmpCurve is OffsetCurve3D || tmpCurve is TrimmedCurve3D)
        {
            if (tmpCurve is OffsetCurve3D offset)
            {
                tmpCurve = offset.BasisCurve;
            }

            if (tmpCurve is TrimmedCurve3D trimmed)
            {
                tmpCurve = trimmed.BasisCurve;
            }
        }

        return tmpCurve.IsPeriodic;
    }

    /// <summary>Tells if a pcurve is periodic, asking the basis curve of offset and trimmed curves.</summary>
    public static bool IsPeriodic(Curve2D curve)
    {
        // 15.11.2002 PTV OCC966
        // ask IsPeriodic on BasisCurve
        Curve2D tmpCurve = curve;
        while (tmpCurve is OffsetCurve2D || tmpCurve is TrimmedCurve2D)
        {
            if (tmpCurve is OffsetCurve2D offset)
            {
                tmpCurve = offset.BasisCurve;
            }

            if (tmpCurve is TrimmedCurve2D trimmed)
            {
                tmpCurve = trimmed.BasisCurve;
            }
        }

        return tmpCurve.IsPeriodic;
    }
}
