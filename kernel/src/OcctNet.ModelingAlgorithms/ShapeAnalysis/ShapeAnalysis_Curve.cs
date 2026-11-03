// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Curve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Analyses curves: projection of points (with protections near the ends and fallbacks where extrema fail), range
/// validation, exact 2D bounding boxes, the forward seam of two pcurves, planarity, closure and periodicity, and
/// sample points.
/// </summary>
/// <remarks>A working object without state (OCCT's is used by value).</remarks>
public sealed class ShapeAnalysis_Curve
{
    // Projects a point onto a curve by evaluating the curve at several points within the parameter range. It finds
    // the closest point on the curve to thePoint and updates the projection distance, point and parameter. It also
    // narrows the start and end parameters to the segment around the closest point, even if no distance below
    // aProjDistance was found.
    private static void ProjectOnSegments(Adaptor3d_Curve theCurve,
                                          in Point3D       thePoint,
                                          int             theSegmentCount,
                                          ref double      aStartParam,
                                          ref double      anEndParam,
                                          ref double      aProjDistance,
                                          ref Point3D      aProjPoint,
                                          ref double      aProjParam)
    {
        // We consider <nbseg> points on [uMin,uMax]
        // Which is the closest? And what is the new interval?
        // (it cannot overflow the old one)

        if (theSegmentCount <= 0)
        {
            return; // No segments to project on
        }

        double aParamStep     = (anEndParam - aStartParam) / theSegmentCount;
        double aMinSqDistance = aProjDistance * aProjDistance;
        bool   aHasChanged    = false;
        for (int i = 0; i <= theSegmentCount; i++)
        {
            double aCurrentParam      = aStartParam + (aParamStep * i);
            Point3D aCurrentPoint      = theCurve.Value(aCurrentParam);
            double aCurrentSqDistance = aCurrentPoint.SquareDistance(thePoint);
            if (aCurrentSqDistance < aMinSqDistance)
            {
                aMinSqDistance = aCurrentSqDistance;
                aProjPoint     = aCurrentPoint;
                aProjParam     = aCurrentParam;
                aHasChanged    = true;
            }
        }

        if (aHasChanged)
        {
            aProjDistance = Math.Sqrt(aMinSqDistance);
        }

        anEndParam  = Min(anEndParam, aProjParam + aParamStep);
        aStartParam = Max(aStartParam, aProjParam - aParamStep);
    }

    /// <summary>
    /// Projects a point on a curve. Returns the distance between the point and its projection, which is stored in
    /// <paramref name="proj"/> with its parameter in <paramref name="param"/>. If the distance to an end of a bounded
    /// curve is within <paramref name="preci"/> (when <paramref name="AdjustToEnds"/>), that end is taken.
    /// </summary>
    public double Project(Geom_Curve C3D, in Point3D P3D, double preci, out Point3D proj, out double param,
                          bool AdjustToEnds = true)
    {
        double uMin = C3D.FirstParameter();
        double uMax = C3D.LastParameter();
        if (uMin < uMax)
        {
            return Project(C3D, P3D, preci, out proj, out param, uMin, uMax, AdjustToEnds);
        }
        else
        {
            return Project(C3D, P3D, preci, out proj, out param, uMax, uMin, AdjustToEnds);
        }
    }

    /// <summary>Projects a point on a curve restricted to [cf, cl] (see the overload without bounds).</summary>
    public double Project(Geom_Curve C3D, in Point3D P3D, double preci, out Point3D proj, out double param, double cf,
                          double cl, bool AdjustToEnds = true)
    {
        double distmin;
        double uMin = (cf < cl ? cf : cl);
        double uMax = (cf < cl ? cl : cf);

        var GAC = new GeomAdaptor_Curve(C3D, uMin, uMax);
        if (C3D is Geom_BoundedCurve)
        {
            //:j8 abv 10 Dec 98: tr10_r0501_db.stp #9423: protection against densing of points near one end
            double prec     = (AdjustToEnds ? preci : global::Kerf.Engine.Foundation.Precision.Confusion());
            Point3D LowBound = GAC.Value(uMin);
            Point3D HigBound = GAC.Value(uMax);
            distmin = LowBound.Distance(P3D);
            if (distmin <= prec)
            {
                param = uMin;
                proj  = LowBound;
                return distmin;
            }

            distmin = HigBound.Distance(P3D);
            if (distmin <= prec)
            {
                param = uMax;
                proj  = HigBound;
                return distmin;
            }
        }

        if (!C3D.IsClosed())
        {
            // modified by rln on 16/12/97 after CSR# PRO11641 entity 20767
            // the VIso was not closed (according to C3D->IsClosed()) while it "almost"
            // was (the distance between ends of the curve was a little bit more than
            // Precision::Confusion())
            // in that case value 0.1 was too much and this method returned not correct parameter
            //  modified by pdn on 01.07.98 after BUC60195 entity 1952 (std::min() added)
            double delta = Min(GAC.Resolution(preci), (uMax - uMin) * 0.1);
            uMin -= delta;
            uMax += delta;
            GAC.Load(C3D, uMin, uMax);
        }

        return ProjectAct(GAC, P3D, preci, out proj, out param);
    }

    /// <summary>Projects a point on a curve given by an adaptor (see the Geom_Curve overload).</summary>
    public double Project(Adaptor3d_Curve C3D, in Point3D P3D, double preci, out Point3D proj, out double param,
                          bool AdjustToEnds = true)
    {
        double uMin = C3D.FirstParameter();
        double uMax = C3D.LastParameter();

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(uMin) && global::Kerf.Engine.Foundation.Precision.IsInfinite(uMax))
        {
            return ProjectAct(C3D, P3D, preci, out proj, out param);
        }

        double distmin_L = global::Kerf.Engine.Foundation.Precision.Infinite(), distmin_H = global::Kerf.Engine.Foundation.Precision.Infinite();
        //:j8 abv 10 Dec 98: tr10_r0501_db.stp #9423: protection against densing of points near one end
        double prec     = (AdjustToEnds ? preci : global::Kerf.Engine.Foundation.Precision.Confusion());
        Point3D LowBound = C3D.Value(uMin);
        Point3D HigBound = C3D.Value(uMax);
        distmin_L = LowBound.Distance(P3D);
        distmin_H = HigBound.Distance(P3D);

        if (distmin_L <= prec)
        {
            param = uMin;
            proj  = LowBound;
            return distmin_L;
        }

        if (distmin_H <= prec)
        {
            param = uMax;
            proj  = HigBound;
            return distmin_H;
        }

        double distProj = ProjectAct(C3D, P3D, preci, out proj, out param);
        if (distProj < distmin_L + global::Kerf.Engine.Foundation.Precision.Confusion() && distProj < distmin_H + global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return distProj;
        }

        if (distmin_L < distmin_H)
        {
            param = uMin;
            proj  = LowBound;
            return distmin_L;
        }

        param = uMax;
        proj  = HigBound;
        return distmin_H;
    }

    /// <summary>
    /// Projects a point on a curve by Extrema_ExtPC, with fallbacks on analytic curves (ElCLib) and by sampling and
    /// Newton iterations on the others. Returns the distance to the projection, even when it isn't within the
    /// tolerance.
    /// </summary>
    /// <remarks>
    /// OCCT reads the caller's <paramref name="theProjPoint"/> before writing it when Extrema finds nothing, but that
    /// value can't reach the result; the port starts it at the origin.
    /// </remarks>
    public double ProjectAct(Adaptor3d_Curve theCurve, in Point3D thePoint, double theTolerance,
                             out Point3D theProjPoint, out double theProjParam)
    {
        theProjPoint = new Point3D();
        bool OK = false;
        theProjParam = 0.0;
        try
        {
            var    aCurveExtrema       = new Extrema_ExtPC(thePoint, theCurve);
            double aMinExtremaDistance = RealLast();
            int    aMinExtremaIndex    = 0;
            if (aCurveExtrema.IsDone() && (aCurveExtrema.NbExt() > 0))
            {
                for (int i = 1; i <= aCurveExtrema.NbExt(); i++)
                {
                    if (!aCurveExtrema.IsMin(i))
                    {
                        continue;
                    }

                    double aCurrentDistance = aCurveExtrema.SquareDistance(i);
                    if (aCurrentDistance < aMinExtremaDistance)
                    {
                        aMinExtremaDistance = aCurrentDistance;
                        aMinExtremaIndex    = i;
                    }
                }

                if (aMinExtremaIndex != 0)
                {
                    theProjParam = (aCurveExtrema.Point(aMinExtremaIndex)).Parameter();
                    theProjPoint = (aCurveExtrema.Point(aMinExtremaIndex)).Value();
                    OK           = true;
                }
            }
        }
        catch (FailureException)
        {
            OK = false;
        }

        // szv#4:S4163:12Mar99 moved
        double uMin            = theCurve.FirstParameter();
        double uMax            = theCurve.LastParameter();
        bool   anIsClosedCurve = false;
        double aCurvePeriod    = 0.0;
        // Distance between the point and the projection point.
        double aProjDistance = global::Kerf.Engine.Foundation.Precision.Infinite();
        double aModMin       = global::Kerf.Engine.Foundation.Precision.Infinite();

        // Remember the computed values.
        // These values will be used in case the projection is not successful.
        double aComputedParam = theProjParam;
        Point3D aComputedProj  = theProjPoint;

        // PTV 29.05.2002 remember the old solution, cause it could be better
        bool   anIsHaveOldSolution = false;
        double anOldParam          = 0.0;
        Point3D anOldProj           = new Point3D();
        if (OK)
        {
            anIsHaveOldSolution = true;
            anOldProj           = theProjPoint;
            anOldParam          = theProjParam;
            aProjDistance       = theProjPoint.Distance(thePoint);
            aModMin             = aProjDistance;
            if (aProjDistance > theTolerance)
            {
                OK = false;
            }

            if (theCurve.IsClosed())
            {
                anIsClosedCurve = true;
                aCurvePeriod    = uMax - uMin; // szv#4:S4163:12Mar99 optimized
            }
        }

        if (!OK)
        {
            // BUG NICOLAS - If the point is on the curve 0 Solutions. This works with ElCLib

            // Generally speaking, we try to ALWAYS return a result that's NOT EVEN GOOD. The caller can
            // then decide what to do.
            theProjParam = 0.0;

            switch (theCurve.GetType())
            {
                case Circle:
                {
                    Circle3D aCirc = theCurve.Circle();
                    theProjPoint = aCirc.Position().Location();
                    if (aCirc.Radius() <= Constants.Resolution() || thePoint.SquareDistance(theProjPoint) <= Constants.Resolution())
                    {
                        theProjParam = theCurve.FirstParameter();
                        theProjPoint = new Point3D(theProjPoint.XYZ() + aCirc.XAxis().Direction().XYZ() * aCirc.Radius());
                    }
                    else
                    {
                        theProjParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(aCirc, thePoint);
                        theProjPoint = global::Kerf.Engine.Geometry.ElCLib.Value(theProjParam, aCirc);
                    }

                    anIsClosedCurve = true;
                    aCurvePeriod    = 2.0 * M_PI;
                    break;
                }

                case Hyperbola:
                {
                    theProjParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCurve.Hyperbola(), thePoint);
                    theProjPoint = global::Kerf.Engine.Geometry.ElCLib.Value(theProjParam, theCurve.Hyperbola());
                    break;
                }

                case Parabola:
                {
                    theProjParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCurve.Parabola(), thePoint);
                    theProjPoint = global::Kerf.Engine.Geometry.ElCLib.Value(theProjParam, theCurve.Parabola());
                    break;
                }

                case Line:
                {
                    theProjParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCurve.Line(), thePoint);
                    theProjPoint = global::Kerf.Engine.Geometry.ElCLib.Value(theProjParam, theCurve.Line());
                    break;
                }

                case Ellipse:
                {
                    theProjParam    = global::Kerf.Engine.Geometry.ElCLib.Parameter(theCurve.Ellipse(), thePoint);
                    theProjPoint    = global::Kerf.Engine.Geometry.ElCLib.Value(theProjParam, theCurve.Ellipse());
                    anIsClosedCurve = true;
                    aCurvePeriod    = 2.0 * M_PI;
                    break;
                }

                default:
                {
                    // Bspline or something, no easy solution.
                    // Here this algorithm tries to find the closest point on the curve
                    // by evaluating the distance between the point and the curve
                    // at several points within the parameter range.
                    aProjDistance = global::Kerf.Engine.Foundation.Precision.Infinite();
                    ProjectOnSegments(theCurve, thePoint, 25, ref uMin, ref uMax, ref aProjDistance, ref theProjPoint,
                                      ref theProjParam);
                    if (aProjDistance <= theTolerance)
                    {
                        return aProjDistance;
                    }

                    var aProjector = new Extrema_LocateExtPC(thePoint, theCurve, theProjParam /*U0*/, uMin, uMax,
                                                             theTolerance /*TolU*/);
                    if (aProjector.IsDone())
                    {
                        theProjParam = aProjector.Point().Parameter();
                        theProjPoint = aProjector.Point().Value();
                        double aDistNewton = thePoint.Distance(theProjPoint);
                        if (aDistNewton < aModMin)
                        {
                            return aDistNewton;
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
                    foreach (int aSegmentCount in SegmentCounts)
                    {
                        ProjectOnSegments(theCurve, thePoint, aSegmentCount, ref uMin, ref uMax, ref aProjDistance,
                                          ref theProjPoint, ref theProjParam);
                        if (aProjDistance <= theTolerance)
                        {
                            return aProjDistance;
                        }
                    }

                    // Did not find a point on the curve that is closer than the tolerance.
                    // So, we return the closest point found so far.
                    if (aProjDistance > aModMin)
                    {
                        aProjDistance = aModMin;
                        theProjParam  = aComputedParam;
                        theProjPoint  = aComputedProj;
                    }

                    return aProjDistance;
                }
            }
        }

        if (anIsClosedCurve && (theProjParam < uMin || theProjParam > uMax))
        {
            theProjParam += ShapeAnalysis.AdjustByPeriod(theProjParam, 0.5 * (uMin + uMax), aCurvePeriod);
        }

        if (anIsHaveOldSolution)
        {
            // PTV 29.05.2002 Compare old solution and new;
            double anOldDist = anOldProj.SquareDistance(thePoint);
            double aNewDist  = theProjPoint.SquareDistance(thePoint);
            if (anOldDist < aNewDist)
            {
                theProjPoint = anOldProj;
                theProjParam = anOldParam;
            }
        }

        return theProjPoint.Distance(thePoint);
    }

    private static readonly int[] SegmentCounts = [40, 20, 25, 40];

    /// <summary>
    /// Projects a point on a curve restricted to [cf, cl] by a Newton search from the previous parameter
    /// <paramref name="paramPrev"/>; where it fails, as <c>Project</c> on the adaptor with AdjustToEnds false.
    /// </summary>
    public double NextProject(double paramPrev, Geom_Curve C3D, in Point3D P3D, double preci, out Point3D proj,
                              out double param, double cf, double cl, bool AdjustToEnds = true)
    {
        double uMin    = (cf < cl ? cf : cl);
        double uMax    = (cf < cl ? cl : cf);
        double distmin = global::Kerf.Engine.Foundation.Precision.Infinite();
        var    GAC     = new GeomAdaptor_Curve(C3D, uMin, uMax);
        if (C3D is Geom_BoundedCurve)
        {
            //:j8 abv 10 Dec 98: tr10_r0501_db.stp #9423: protection against densing of points near one end
            double prec     = (AdjustToEnds ? preci : global::Kerf.Engine.Foundation.Precision.Confusion());
            Point3D LowBound = GAC.Value(uMin);
            Point3D HigBound = GAC.Value(uMax);
            distmin = LowBound.Distance(P3D);
            if (distmin <= prec)
            {
                param = uMin;
                proj  = LowBound;
                return distmin;
            }

            distmin = HigBound.Distance(P3D);
            if (distmin <= prec)
            {
                param = uMax;
                proj  = HigBound;
                return distmin;
            }
        }

        if (!C3D.IsClosed())
        {
            //  modified by pdn on 01.07.98 after BUC60195 entity 1952 (std::min() added)
            double delta = Min(GAC.Resolution(preci), (uMax - uMin) * 0.1);
            uMin -= delta;
            uMax += delta;
            GAC.Load(C3D, uMin, uMax);
        }

        return NextProject(paramPrev, GAC, P3D, preci, out proj, out param);
    }

    /// <summary>Projects a point on a curve given by an adaptor, by Newton from <paramref name="paramPrev"/>.</summary>
    public double NextProject(double paramPrev, Adaptor3d_Curve C3D, in Point3D P3D, double preci, out Point3D proj,
                              out double param)
    {
        double uMin = C3D.FirstParameter();
        double uMax = C3D.LastParameter();

        var aProjector = new Extrema_LocateExtPC(P3D, C3D, paramPrev /*U0*/, uMin, uMax, preci /*TolU*/);
        if (aProjector.IsDone())
        {
            param = aProjector.Point().Parameter();
            proj  = aProjector.Point().Value();
            return P3D.Distance(proj);
        }

        return Project(C3D, P3D, preci, out proj, out param, false);
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
    public bool ValidateRange(Geom_Curve theCurve, ref double First, ref double Last, double preci)
    {
        // First and/or Last may be outside the natural bounds of the curve.
        // They are then given the end values.

        double cf = theCurve.FirstParameter();
        double cl = theCurve.LastParameter();

        if (theCurve is Geom_BoundedCurve && !theCurve.IsClosed())
        {
            if (First < cf)
            {
                First = cf;
            }
            else if (First > cl)
            {
                First = cl;
            }

            if (Last < cf)
            {
                Last = cf;
            }
            else if (Last > cl)
            {
                Last = cl;
            }
        }

        // 15.11.2002 PTV OCC966
        if (ShapeAnalysis_Curve.IsPeriodic(theCurve))
        {
            // OCCT: on a trimmed (or offset) periodic curve, [cf, cl] is the trimmed range, not the period
            global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(cf, cl, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref First, ref Last); //:a7 abv 11 Feb 98: preci -> PConfusion()
        }
        else if (First < Last)
        {
            // nothing to fix
        }
        else if (theCurve.IsClosed())
        {
            // one of the projected points is at the origin of the 3D curve's parametrisation:
            // the algorithm gave cl +- preci instead of cf or vice versa
            // DANGER: 3D precision applied to a 1D space

            // Last = cf instead of Last = cl
            if (Math.Abs(Last - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
            {
                Last = cl;
                // First = cl instead of First = cf
            }
            else if (Math.Abs(First - cl) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
            {
                First = cf;

                // the origin is crossed, illegal on a closed non-periodic curve:
                // the parameters are inverted anyway
            }
            else
            {
                //: S4136 abv 20 Apr 99: r0701_ug.stp #6230: add check in 3d
                if (theCurve.Value(First).Distance(theCurve.Value(cf)) < preci)
                {
                    First = cf;
                }

                if (theCurve.Value(Last).Distance(theCurve.Value(cl)) < preci)
                {
                    Last = cl;
                }

                if (First > Last)
                {
                    double tmp = First;
                    First = Last;
                    Last  = tmp;
                }
            }
        }
        // The curve is closed within the 3D tolerance
        else if (theCurve is Geom_BSplineCurve aBSpline)
        {
            if (aBSpline.StartPoint().Distance(aBSpline.EndPoint()) <= preci)
            {
                // Last = cf instead of Last = cl
                if (Math.Abs(Last - cf) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
                {
                    Last = cl;
                    // First = cl instead of First = cf
                }
                else if (Math.Abs(First - cl) < global::Kerf.Engine.Foundation.Precision.PConfusion() /*preci*/)
                {
                    First = cf;
                }
                else
                {
                    double tmp = First;
                    First = Last;
                    Last  = tmp;
                }
            }
            // abv 15.03.00 #72 bm1_pe_t4 protection of exceptions in draw
            else if (First > Last)
            {
                First = theCurve.ReversedParameter(First);
                Last  = theCurve.ReversedParameter(Last);
                theCurve.Reverse();
            }

            //: j9 abv 11 Dec 98: PRO7747 #4875, after :j8:    else
            if (First == Last)
            { // gka 10.07.1998 file PRO7656 entity 33334
                First = cf;
                Last  = cl;
                return false;
            }
        }
        else
        {
            // abv 15.03.00 #72 bm1_pe_t4 protection of exceptions in draw
            if (First > Last)
            {
                First = theCurve.ReversedParameter(First);
                Last  = theCurve.ReversedParameter(Last);
                theCurve.Reverse();
            }

            // pdn 11.01.99 #144 bm1_pe_t4 protection of exceptions in draw
            if (First == Last)
            {
                First -= global::Kerf.Engine.Foundation.Precision.PConfusion();
                Last  += global::Kerf.Engine.Foundation.Precision.PConfusion();
            }

            return false;
        }

        return true;
    }

    // search for extremum using Newton (OCCT returns int, with the values true and false)
    private static bool SearchForExtremum(Geom2d_Curve C2d, double First, double Last, in Vector2D dir, ref double par,
                                          ref Point2D res)
    {
        double prevpar;
        int    nbOut = 0;
        for (int i = 0; i < 10; i++)
        {
            prevpar = par;

            C2d.D2(par, out res, out Vector2D D1, out Vector2D D2);
            double Det = (D2 * dir);
            if (Math.Abs(Det) < 1e-10)
            {
                return true;
            }

            par -= (D1 * dir) / Det;
            if (Math.Abs(par - prevpar) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return true;
            }

            if (par < First)
            {
                if (nbOut++ > 2 || prevpar == First)
                {
                    return false;
                }

                par = First;
            }

            if (par > Last)
            {
                if (nbOut++ > 2 || prevpar == Last)
                {
                    return false;
                }

                par = Last;
            }
        }

        return true;
    }

    /// <summary>
    /// Adds to the box the points of a pcurve on [First, Last]: <paramref name="NPoints"/> equidistant points, or
    /// when <paramref name="Exact"/>, the ends of each interval of C2 continuity and the extrema in X and Y found on
    /// it by Newton (a work-around for BndLib's inexact bounds).
    /// </summary>
    public void FillBndBox(Geom2d_Curve C2d, double First, double Last, int NPoints, bool Exact, ref Box2d Box)
    {
        if (!Exact)
        {
            int    nseg = (NPoints < 2 ? 1 : NPoints - 1);
            double step = (Last - First) / nseg;
            for (int i = 0; i <= nseg; i++)
            {
                double   par = First + i * step;
                Point2D pnt = C2d.Value(par);
                Box.Add(pnt);
            }

            return;
        }

        // We should solve the task on intervals of C2 continuity.
        var anAC  = new Geom2dAdaptor_Curve(C2d, First, Last);
        int nbInt = anAC.NbIntervals(C2);
        // If we have only 1 interval then use input NPoints parameter to get samples.
        int nbSamples = (nbInt < 2 ? NPoints - 1 : nbInt);
        var aParams   = new NCollection_Array1<double>(1, nbSamples + 1);
        if (nbSamples == nbInt)
        {
            anAC.Intervals(aParams, C2);
        }
        else
        {
            double step = (Last - First) / nbSamples;
            for (int i = 0; i <= nbSamples; i++)
            {
                aParams[i + 1] = First + i * step;
            }
        }

        for (int i = 1; i <= nbSamples + 1; i++)
        {
            double   aPar1 = aParams[i];
            Point2D aPnt  = C2d.Value(aPar1);
            Box.Add(aPnt);
            if (i <= nbSamples)
            {
                double   aPar2   = aParams[i + 1];
                double   par     = (aPar1 + aPar2) * 0.5;
                Point2D pextr   = new Point2D();
                double   parextr = par;
                if (SearchForExtremum(C2d, aPar1, aPar2, new Vector2D(1, 0), ref parextr, ref pextr))
                {
                    Box.Add(pextr);
                }

                parextr = par;
                if (SearchForExtremum(C2d, aPar1, aPar2, new Vector2D(0, 1), ref parextr, ref pextr))
                {
                    Box.Add(pextr);
                }
            }
        }
    }

    /// <summary>
    /// Of two pcurves of a seam (lines, or bounded curves taken as the lines through their ends), returns the index
    /// (1 or 2) of the forward one, or 0 if it can't be told.
    /// </summary>
    public int SelectForwardSeam(Geom2d_Curve C1, Geom2d_Curve C2)
    {
        int theCurveIndice = 0;

        Geom2d_Line? L1 = C1 as Geom2d_Line;
        if (L1 is null)
        {
            // if we have BoundedCurve, create a line from C1
            if (C1 is not Geom2d_BoundedCurve BC1)
            {
                return theCurveIndice;
            }

            Point2D StartBC1 = BC1.StartPoint();
            Point2D EndBC1   = BC1.EndPoint();
            var      VecBC1   = new Vector2D(StartBC1, EndBC1);
            if (VecBC1.SquareMagnitude() < Constants.Resolution())
            {
                return theCurveIndice;
            }

            L1 = new Geom2d_Line(StartBC1, new Direction2D(VecBC1));
        }

        Geom2d_Line? L2 = C2 as Geom2d_Line;
        if (L2 is null)
        {
            // if we have BoundedCurve, creates a line from C2
            if (C2 is not Geom2d_BoundedCurve BC2)
            {
                return theCurveIndice;
            }

            Point2D StartBC2 = BC2.StartPoint();
            Point2D EndBC2   = BC2.EndPoint();
            var      VecBC2   = new Vector2D(StartBC2, EndBC2);
            if (VecBC2.SquareMagnitude() < Constants.Resolution())
            {
                return theCurveIndice;
            }

            L2 = new Geom2d_Line(StartBC2, new Direction2D(VecBC2));
        }

        bool UdirPos, UdirNeg, VdirPos, VdirNeg;
        UdirPos = UdirNeg = VdirPos = VdirNeg = false;

        Direction2D theDir  = L1.Direction();
        Point2D theLoc1 = L1.Location();
        Point2D theLoc2 = L2.Location();

        if (theDir.X() > 0.0)
        {
            UdirPos = true; // szv#4:S4163:12Mar99 Udir unused
        }
        else if (theDir.X() < 0.0)
        {
            UdirNeg = true; // szv#4:S4163:12Mar99 Udir unused
        }
        else if (theDir.Y() > 0.0)
        {
            VdirPos = true; // szv#4:S4163:12Mar99 Vdir unused
        }
        else if (theDir.Y() < 0.0)
        {
            VdirNeg = true; // szv#4:S4163:12Mar99 Vdir unused
        }

        if (VdirPos)
        {
            // max of Loc1.X() Loc2.X()
            if (theLoc1.X() > theLoc2.X())
            {
                theCurveIndice = 1;
            }
            else
            {
                theCurveIndice = 2;
            }
        }
        else if (VdirNeg)
        {
            if (theLoc1.X() > theLoc2.X())
            {
                theCurveIndice = 2;
            }
            else
            {
                theCurveIndice = 1;
            }
        }
        else if (UdirPos)
        {
            // min of Loc1.X() Loc2.X()
            if (theLoc1.Y() < theLoc2.Y())
            {
                theCurveIndice = 1;
            }
            else
            {
                theCurveIndice = 2;
            }
        }
        else if (UdirNeg)
        {
            if (theLoc1.Y() < theLoc2.Y())
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
        var Norm = new Xyz();
        if (Math.Abs(orig.Z()) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Norm.SetCoord(0, 0, 1);
        }
        else
        {
            Norm.SetCoord(orig.Z(), 0, -orig.X());
            double nrm = Norm.Modulus();
            if (nrm < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                Norm.SetCoord(0, 0, 1);
            }
            else
            {
                Norm = Norm / nrm;
            }
        }

        return Norm;
    }

    private static void AppendControlPoles(NCollection_Sequence<Point3D> seq, Geom_Curve curve)
    {
        if (curve is Geom_Line)
        {
            seq.Append(curve.Value(0));
            seq.Append(curve.Value(1));
        }
        else if (curve is Geom_Conic)
        {
            seq.Append(curve.Value(0));
            seq.Append(curve.Value(M_PI / 2));
            seq.Append(curve.Value(M_PI));
        }
        else if (curve is Geom_TrimmedCurve Trimmed)
        {
            Geom_Curve aBaseCrv = Trimmed.BasisCurve();
            bool       done     = false;
            if (aBaseCrv is Geom_BSplineCurve)
            {
                try
                {
                    var bslp = (Geom_BSplineCurve)aBaseCrv.Copy();
                    bslp.Segment(curve.FirstParameter(), curve.LastParameter());
                    AppendControlPoles(seq, bslp);
                    done = true;
                }
                catch (FailureException)
                {
                }
            }
            else if (aBaseCrv is Geom_BezierCurve)
            {
                try
                {
                    var bz = (Geom_BezierCurve)aBaseCrv.Copy();
                    bz.Segment(curve.FirstParameter(), curve.LastParameter());
                    AppendControlPoles(seq, bz);
                    done = true;
                }
                catch (FailureException)
                {
                }
            }

            if (!done)
            {
                seq.Append(curve.Value(curve.FirstParameter()));
                seq.Append(curve.Value((curve.FirstParameter() + curve.LastParameter()) / 2.0));
                seq.Append(curve.Value(curve.LastParameter()));
            }
        }
        else if (curve is Geom_OffsetCurve)
        {
            seq.Append(curve.Value(curve.FirstParameter()));
            seq.Append(curve.Value((curve.FirstParameter() + curve.LastParameter()) / 2.0));
            seq.Append(curve.Value(curve.LastParameter()));
        }
        else if (curve is Geom_BSplineCurve BSpline)
        {
            NCollection_Array1<Point3D> Poles = BSpline.InternalPoles();
            for (int i = 1; i <= Poles.Length(); i++)
            {
                seq.Append(Poles[i]);
            }
        }
        else if (curve is Geom_BezierCurve Bezier)
        {
            NCollection_Array1<Point3D> Poles = Bezier.InternalPoles();
            for (int i = 1; i <= Poles.Length(); i++)
            {
                seq.Append(Poles[i]);
            }
        }
    }

    /// <summary>
    /// Tells if the points lie in a plane within <paramref name="preci"/> (Precision::Confusion() if not positive).
    /// If <paramref name="Normal"/> is zero on input, the plane's normal is computed and returned in it; otherwise
    /// the points are checked against that normal.
    /// </summary>
    public static bool IsPlanar(NCollection_Array1<Point3D> pnts, ref Xyz Normal, double preci = 0)
    {
        double precision = (preci > 0.0) ? preci : global::Kerf.Engine.Foundation.Precision.Confusion();
        bool   noNorm    = (Normal.SquareModulus() == 0);

        if (pnts.Length() < 3)
        {
            Xyz N1 = pnts[1].XYZ() - pnts[2].XYZ();
            if (noNorm)
            {
                Normal = GetAnyNormal(N1);
                return true;
            }

            return Math.Abs(N1 * Normal) < global::Kerf.Engine.Foundation.Precision.Confusion();
        }

        Xyz aMaxDir = new Xyz();
        if (noNorm)
        {
            // define a center point
            var aCenter = new Xyz(0, 0, 0);
            int i       = 1;
            for (; i <= pnts.Length(); i++)
            {
                aCenter += pnts[i].XYZ();
            }

            aCenter /= pnts.Length();

            aMaxDir = pnts[1].XYZ() - aCenter;
            Normal  = (pnts[pnts.Length()].XYZ() - aCenter) ^ aMaxDir;

            for (i = 1; i < pnts.Length(); i++)
            {
                Xyz aTmpDir = pnts[i + 1].XYZ() - aCenter;
                if (aTmpDir.SquareModulus() > aMaxDir.SquareModulus())
                {
                    aMaxDir = aTmpDir;
                }

                Xyz aDelta = (pnts[i].XYZ() - aCenter) ^ (pnts[i + 1].XYZ() - aCenter);
                if (Normal * aDelta < 0)
                {
                    aDelta *= -1;
                }

                Normal += aDelta;
            }
        }

        // check if points are linear
        double nrm = Normal.Modulus();
        if (nrm < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Normal = GetAnyNormal(aMaxDir);
            return true;
        }

        Normal = Normal / nrm;

        double mind = RealLast(), maxd = -RealLast(), dev;
        for (int i = 1; i <= pnts.Length(); i++)
        {
            dev = pnts[i].XYZ() * Normal;
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
    /// other curve. See the overload on points for <paramref name="Normal"/>.
    /// </summary>
    public static bool IsPlanar(Geom_Curve curve, ref Xyz Normal, double preci = 0)
    {
        double precision = (preci > 0.0) ? preci : global::Kerf.Engine.Foundation.Precision.Confusion();
        bool   noNorm    = (Normal.SquareModulus() == 0);

        if (curve is Geom_Line Line)
        {
            Xyz N1 = Line.Position().Direction().XYZ();
            if (noNorm)
            {
                Normal = GetAnyNormal(N1);
                return true;
            }

            return Math.Abs(N1 * Normal) < global::Kerf.Engine.Foundation.Precision.Confusion();
        }

        if (curve is Geom_Conic Conic)
        {
            Xyz N1 = Conic.Axis().Direction().XYZ();
            if (noNorm)
            {
                Normal = N1;
                return true;
            }

            Xyz aVecMul = N1 ^ Normal;
            return aVecMul.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        }

        if (curve is Geom_TrimmedCurve Trimmed)
        {
            return IsPlanar(Trimmed.BasisCurve(), ref Normal, precision);
        }

        if (curve is Geom_OffsetCurve OffsetC)
        {
            return IsPlanar(OffsetC.BasisCurve(), ref Normal, precision);
        }

        if (curve is Geom_BSplineCurve BSpline)
        {
            return IsPlanar(BSpline.InternalPoles(), ref Normal, precision);
        }

        if (curve is Geom_BezierCurve Bezier)
        {
            return IsPlanar(Bezier.InternalPoles(), ref Normal, precision);
        }

        if (curve is ShapeExtend_ComplexCurve Complex)
        {
            var sequence = new NCollection_Sequence<Point3D>();
            int i;
            for (i = 1; i <= Complex.NbCurves(); i++)
            {
                AppendControlPoles(sequence, Complex.Curve(i));
            }

            var Poles = new NCollection_Array1<Point3D>(1, sequence.Length());
            for (i = 1; i <= sequence.Length(); i++)
            {
                Poles[i] = sequence[i];
            }

            return IsPlanar(Poles, ref Normal, precision);
        }

        return false;
    }

    /// <summary>
    /// Appends to <paramref name="seq"/> sample points of a curve on [first, last] (2 on lines, 360 per period on
    /// circles, knots × degree on B-splines, poles + 3 on Bezier curves, else 100 per parameter range). Returns false
    /// if the curve's parameter range is empty.
    /// </summary>
    public static bool GetSamplePoints(Geom_Curve curve, double first, double last, NCollection_Sequence<Point3D> seq)
    {
        double adelta = curve.LastParameter() - curve.FirstParameter();
        if (adelta == 0)
        {
            return false;
        }

        int aK  = (int)Math.Ceiling((last - first) / adelta);
        int nbp = 100 * aK;
        if (curve is Geom_Line)
        {
            nbp = 2;
        }
        else if (curve is Geom_Circle)
        {
            nbp = 360 * aK;
        }
        else if (curve is Geom_BSplineCurve aBspl)
        {
            nbp = aBspl.NbKnots() * aBspl.Degree() * aK;
            if (nbp < 2.0)
            {
                nbp = 2;
            }
        }
        else if (curve is Geom_BezierCurve aB)
        {
            nbp = 3 + aB.NbPoles();
        }
        else if (curve is Geom_OffsetCurve aOC)
        {
            return GetSamplePoints(aOC.BasisCurve(), first, last, seq);
        }
        else if (curve is Geom_TrimmedCurve aTC)
        {
            return GetSamplePoints(aTC.BasisCurve(), first, last, seq);
        }

        var    GAC  = new GeomAdaptor_Curve(curve);
        double step = (last - first) / (double)(nbp - 1);
        for (int i = 0; i < nbp - 1; ++i)
        {
            seq.Append(GAC.Value(first + step * i));
        }

        seq.Append(GAC.Value(last));
        return true;
    }

    /// <summary>
    /// Appends to <paramref name="seq"/> sample points of a pcurve on [first, last], distributed as
    /// BRepTopAdaptor_FClass2d does. Returns true.
    /// </summary>
    public static bool GetSamplePoints(Geom2d_Curve curve, double first, double last, NCollection_Sequence<Point2D> seq)
    {
        //: abv 05.06.02: TUBE.stp
        // Use the same distribution of points as BRepTopAdaptor_FClass2d for consistency
        var C   = new Geom2dAdaptor_Curve(curve, first, last);
        int nbs = Geom2dInt_Geom2dCurveTool.NbSamples(C);
        //-- Beware of rational B-splines of degree 3 (pieces of circles among others)
        if (nbs > 2)
        {
            nbs *= 4;
        }

        double step = (last - first) / (double)(nbs - 1);
        for (int i = 0; i < nbs - 1; ++i)
        {
            seq.Append(C.Value(first + step * i));
        }

        seq.Append(C.Value(last));
        return true;
    }

    /// <summary>
    /// Tells if a curve is closed: by <c>Geom_Curve::IsClosed</c>, else if its ends are within
    /// <paramref name="preci"/> (at least Precision::Confusion()).
    /// </summary>
    public static bool IsClosed(Geom_Curve theCurve, double preci = -1)
    {
        if (theCurve.IsClosed())
        {
            return true;
        }

        double prec = Max(preci, global::Kerf.Engine.Foundation.Precision.Confusion());

        double f, l;
        f = theCurve.FirstParameter();
        l = theCurve.LastParameter();

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(f) || global::Kerf.Engine.Foundation.Precision.IsInfinite(l))
        {
            return false;
        }

        double aClosedVal = theCurve.Value(f).SquareDistance(theCurve.Value(l));
        double preci2     = prec * prec;

        return (aClosedVal <= preci2);
    }

    /// <summary>Tells if a curve is periodic, asking the basis curve of offset and trimmed curves.</summary>
    public static bool IsPeriodic(Geom_Curve theCurve)
    {
        // 15.11.2002 PTV OCC966
        // remove regressions in DE tests (diva, divb, divc, toe3) in KAS:dev
        // ask IsPeriodic on BasisCurve
        Geom_Curve aTmpCurve = theCurve;
        while (aTmpCurve is Geom_OffsetCurve || aTmpCurve is Geom_TrimmedCurve)
        {
            if (aTmpCurve is Geom_OffsetCurve anOffset)
            {
                aTmpCurve = anOffset.BasisCurve();
            }

            if (aTmpCurve is Geom_TrimmedCurve aTrimmed)
            {
                aTmpCurve = aTrimmed.BasisCurve();
            }
        }

        return aTmpCurve.IsPeriodic();
    }

    /// <summary>Tells if a pcurve is periodic, asking the basis curve of offset and trimmed curves.</summary>
    public static bool IsPeriodic(Geom2d_Curve theCurve)
    {
        // 15.11.2002 PTV OCC966
        // ask IsPeriodic on BasisCurve
        Geom2d_Curve aTmpCurve = theCurve;
        while (aTmpCurve is Geom2d_OffsetCurve || aTmpCurve is Geom2d_TrimmedCurve)
        {
            if (aTmpCurve is Geom2d_OffsetCurve anOffset)
            {
                aTmpCurve = anOffset.BasisCurve();
            }

            if (aTmpCurve is Geom2d_TrimmedCurve aTrimmed)
            {
                aTmpCurve = aTrimmed.BasisCurve();
            }
        }

        return aTmpCurve.IsPeriodic();
    }
}
